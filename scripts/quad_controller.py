#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import rospy
import numpy
import math
import rospidlib.rospidlib_quad as rospidlib_quad
import tf
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import TransformStamped, Point, Twist
from std_msgs.msg import Bool, UInt8, String, Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from aapacs_pelican.msg import LLStatus

# Todo:
# - Add transition between experiment and hover set points, avoid sharp changes in position
# - Add minimum z alt support in geofence
# - Add ability to receive battery health + do something about it
# - Check yaw angle before take off
# - Perform control saturations here, not in PID controller
# - Update warnings on fence breach - don't care if we're grounded
# - Autoland neutral inputs - read from integrators
# - Updates to PID Controller
# -   Optionally saturate integral terms
# -   Support for I and D gains on yaw
# -   Ability to read state of itegrators
# -   Ability to tune PIDs through tk interface

class quad_control:

  ## ---------------------------------- Set Up ---------------------------------- ##
  def __init__(self):
    # ROS Initialisations
    self.node = rospy.init_node('quad_control', anonymous=True)                                  # Node naming
    self.rc_pub = rospy.Publisher('rcctrl', Twist)                                               # publisher for RC commands
    self.ref_pub = rospy.Publisher('ref', TransformStamped)                                      # publisher for reference position being used
    self.err_pub = rospy.Publisher('err', TransformStamped)                                      # publisher for error between position and reference
    self.status_pub = rospy.Publisher('status', String)                                          # publisher for airfraft status
    self.info_pub = rospy.Publisher('info', String)                                              # publisher for info messages
    self.vicon_sub = rospy.Subscriber('drone', TransformStamped, self.vicon_callback)            # subscriber for Vicon data
    self.local_sub = rospy.Subscriber('ref_local', TransformStamped, self.ref_local_callback)    # subscriber for reference point input, relative to quad
    self.nudge_sub = rospy.Subscriber('ref_nudge', TransformStamped, self.ref_nudge_callback)    # subscriber for reference point input, relative to quad
    self.global_sub = rospy.Subscriber('ref_global', TransformStamped, self.ref_global_callback) # subscriber for reference point input, global coordinates
    self.vel_sub = rospy.Subscriber('ref_vel', Twist, self.twist_callback)                       # subscriber for velocity input
    self.freeze_int = rospy.Subscriber('freeze_int', Bool, self.freeze_int_callback)             # subscriber for integrator freeze
    self.button_sub = rospy.Subscriber('button_in', UInt8, self.button_callback)                 # subscriber for controller buttons
    self.heartbeat_sub = rospy.Subscriber('cmd_heartbeat', UInt8, self.heartbeat_callback)       # subscriber for command link heartbeat
    self.commands_sub = rospy.Subscriber('quad_cmd', String, self.commands_callback)             # subscriber for command link heartbeat
    self.asctec_sub = rospy.Subscriber('/asctec/LL_STATUS', LLStatus, self.asctec_callback)      # subscriber for asctec pelican info

    # States used in internal logic
    self.states = ['Grounded',         # Aircraft is on the ground and not allowed to take off unless self.ready == 1
                   'Spoolup',          # Aircraft is spooling up, i.e. arming the motors
                   'Takeoff',          # Aircraft is taking off, preferably smoothly to demonstrate quad is well controlled
                   'Hover',            # Aircraft will hover, holding position - initally in 'Home' location, otherwise from position during statepoint entry
                   'Experiment',       # Aircraft will accept experimental inputs, can be reference setpoints in local/global frame, velocity inputs or raw inputs
                   'ReturnHome',       # Aircraft will fly to 'Home' location and then switch to hover
                   'Land',             # Aircraft will land directly below current position
                   'Spooldown',        # Aircraft will ramp the motors down, hopefully for a smooth touchdown (only below self.spooldown_alt)
                   'PositionFailsafe', # Loss of position information, failsafe action to attempt to bring the aircraft down safely
                   'Emergency']        # Unused emergency state

    # Experiment types
    self.experiments = ['RefGlobal',   # Experiment will follow global reference point inputs
                        'RefLocal',    # Experiment will follow local reference point intputs
                        'RefNudge',    # Experiment will follow nudge intputs, identical to local ref, but separate for control panel
                        'RefVel']      # Experiment will follow velocity inputs

    # Internal logic
    self.state = 0              # Current state, position in list above
    self.ready = 0              # Are we ready to take off
    self.experiment = 0         # Are we in experiment mode
    self.valid_position = 1     # Do we have a valid position (from vicon)
    self.last_position = 0      # Timestamp of most recent position update (from vicon)
    self.position_timeout = 0.1 # Time in seconds after which the lack of position updates causes failsafe action
    self.ctrl_link_health = 0   # External controller for commands like takeoff/land, link heath, 0=bad, 1=good
    self.last_heartbeat = 0     # Time stamp of last heartbeat message (from controller)
    self.hb_timeout = 3         # Enter failsafe if no heartbeat from controller received for 3 seconds
    self.spoolup_start = 0      # Time at which spoolup started (timer is not reset until new spool up is initiated)
    self.spoolup_duration = 1   # Time in second that it takes to spool up
    self.spooldn_start = 0      # Time at which spool down is started (timer is not reset until new spool down is initiated)
    self.spooldn_duration = 2   # Time in second that it takes to spool down
    self.takeoff_timeout = 15   # Time in seconds after which a take off is aborted
    self.takeoff_duration = 3   # Desired time taken to climb to hover altitude
    self.landing_duration = 3   # Desired time taken to descend to landing point
    self.spoolup_complete = 0   # Flag, set to 1 upon completion of spool up, cleared upon leaving spool up state
    self.takeoff_complete = 0   # Flag, set to 1 upon completion of take off, cleared upon leaving take off state
    self.landing_complete = 0   # Flag, set to 1 upon completion of landing, cleared upon leaving landing state
    self.fence_breach_t = 0     # Time at which the geofence was last breached
    self.outside_fence = 0      # Flag, set to 1 when we leave the geofenced area. 2 when we've been outside for more than maxtime
    self.fence_maxtime = 5      # Time after which we give up trying to recover from leaving the flying area and land
    self.autoland_time = 3      # Number of seconds before spooldown after open loop landing
    self.autoland_start = 0     # Time at which position failsafe was entered
    self.autoland_thrustdz = rospy.get_param("~autoland/thrustdz",-0.03) # Amount to reduce throttle by in autoland
    self.is_pelican = rospy.get_param("~is_pelican", 0) # Flag, 0 = normal, 1 = pelican. Used to control special pelican logic such as disarming
    self.asctec_info = LLStatus()

    # PID Controller
    self.pid = rospidlib_quad.RospidQuad('~pid')
    self.pid.freeze_integrator()                                           # freeze the pid integrator until flying
    self.last_thrust = 0                                                   # Record the last thrust input, used in spool down ramp
    self.neutral_roll =   rospy.get_param("~neutral/roll",0)     # 'Zero' roll input
    self.neutral_pitch =  rospy.get_param("~neutral/pitch",0.15) # 'Zero' pitch input
    self.neutral_thrust = rospy.get_param("~neutral/thrust",0.1) # Thrust required to hover
    self.neutral_yaw =    rospy.get_param("~neutral/yaw",0)      # 'Zero' yaw input
    
    # Take off and land parameters
    self.zero_height = rospy.get_param("~altitudes/zero",0.12)                # Height below which the vehicle is considered to be sitting on the ground (Vicon bodies rarely sit at z=0) , also the altitude at which we freeze the PID integrators
    self.land_target_alt = rospy.get_param("~altitudes/land_target",0.14)     # Height that controller should aim for when landing
    self.spooldown_alt = rospy.get_param("~altitudes/spooldown",0.16)         # Height below which it is safe to spool down the motors
    self.ground_effect_alt = rospy.get_param("~ground_effect/maxalt",0.3)     # Height at which ground effect causes aircraft to hover on less thrust
    self.ground_effect_scale = rospy.get_param("~ground_effect/thrscale",0.4) # Ratio at which ground effect increases throttle at zero altitude
    self.arming_input = Twist()
    self.disarming_input = Twist()
    self.u_autoland = Twist()
    self.arming_input.linear.z = rospy.get_param("~arming/thrust",-1)
    self.arming_input.angular.z = rospy.get_param("~arming/yaw",1)
    self.disarming_input.linear.z = rospy.get_param("~disarming/thrust",-1)
    self.disarming_input.angular.z = rospy.get_param("~disarming/yaw",-1)
    self.u_autoland.linear.z = rospy.get_param("~disarming/thrust",-1)
    self.u_autoland.angular.z = rospy.get_param("~disarming/yaw",-1)

    # Position Variables
    self.current_posn = TransformStamped()     # current position
    self.home_location = TransformStamped()    # location at which the aircraft shall hover after takeoff
    self.ref_point = TransformStamped()        # reference (i.e. target) position in vicon coordinate frame
    self.ref_vel = Twist()                     # reference velocity in quad coordinate frmae
    self.target_br = tf.TransformBroadcaster() # broadcast target point to RViz
    self.ref_yaw = 0                           # reference yaw position
    self.home_location.transform.translation.x = rospy.get_param('~home/x', 0.0) # default is at origin, 0.8m off ground
    self.home_location.transform.translation.y = rospy.get_param('~home/y', 0.0)
    self.home_location.transform.translation.z = rospy.get_param('~home/z', 0.8)
    init_yaw = rospy.get_param('~home/yaw', 0)
    self.home_location.transform.rotation = self.yaw_to_rotation(init_yaw)
    self.ref_point = self.copy_transform(self.home_location)
    self.ref_point.transform.translation.z = self.land_target_alt
    self.land_start_alt = self.land_target_alt # Altitude at which landing sequence intiated, updated upon entering landing state
    self.land_start_time = 0                   # Time at which landing was initiated
    self.last_vel_update = 0                   # Time at which the reference point was last updated due to a velocity command

    # Geofence: min/max permissible setpoints
    self.min_x = rospy.get_param('~fence/min_x', -3.0)
    self.min_y = rospy.get_param('~fence/min_y', -5.0)
    self.min_z = rospy.get_param('~fence/min_z', 0.0)
    self.max_x = rospy.get_param('~fence/max_x', 3.0)
    self.max_y = rospy.get_param('~fence/max_y', 5.0)
    self.max_z = rospy.get_param('~fence/max_z', 3.0)
    self.rel_threshold = rospy.get_param('~fence/rel_threshold', 2.5) # Maximum setpoint displacement from current posn
    self.max_vel = rospy.get_param('~fence/max_vel', 0.75) # Maximum velocity

    # Rate of main control loop
    self.control_rate = rospy.Rate(float(rospy.get_param('~control_rate', '50')))

    # Print out the parameters to screen for checking
    rospy.logwarn('Autoland thrust = %7.4f', self.autoland_thrustdz)
    rospy.logwarn('neutral/roll = %7.4f', self.neutral_roll)
    rospy.logwarn('neutral/pitch = %7.4f', self.neutral_pitch)
    rospy.logwarn('neutral/thrust = %7.4f', self.neutral_thrust)
    rospy.logwarn('neutral/yaw = %7.4f', self.neutral_yaw)
    rospy.logwarn('altitudes/zero = %7.4f', self.zero_height)
    rospy.logwarn('altitudes/land_target = %7.4f', self.land_target_alt)
    rospy.logwarn('altitudes/spooldown = %7.4f', self.spooldown_alt)
    rospy.logwarn('ground_effect/maxalt = %7.4f', self.ground_effect_alt)
    rospy.logwarn('ground_effect/thrscale = %7.4f', self.ground_effect_scale)
    rospy.logwarn('arming/thrust = %7.4f', self.arming_input.linear.z)
    rospy.logwarn('arming/yaw = %7.4f', self.arming_input.angular.z)
    rospy.logwarn('disarming/thrust = %7.4f', self.disarming_input.linear.z)
    rospy.logwarn('disarming/yaw = %7.4f', self.disarming_input.angular.z)
    rospy.logwarn('home/x = %7.4f', self.home_location.transform.translation.x)
    rospy.logwarn('home/y = %7.4f', self.home_location.transform.translation.y)
    rospy.logwarn('home/z = %7.4f', self.home_location.transform.translation.z)
    rospy.logwarn('home/yaw = %7.4f', init_yaw)
    rospy.logwarn('fence/min_x = %7.4f', self.min_x)
    rospy.logwarn('fence/max_x = %7.4f', self.max_x)
    rospy.logwarn('fence/min_y = %7.4f', self.min_y)
    rospy.logwarn('fence/max_y = %7.4f', self.max_y)
    rospy.logwarn('fence/min_z = %7.4f', self.min_z)
    rospy.logwarn('fence/max_z = %7.4f', self.max_z)
    rospy.logwarn('fence/rel_threshold = %7.4f', self.rel_threshold)
    rospy.logwarn('control_rate = %7.4f Hz', 1/self.control_rate.sleep_dur.to_sec())

    if self.is_pelican:
      rospy.logwarn('Configured for flying a pelican')

    # Some sanity checks on the above variables
    passchecks = 1
    if self.zero_height > self.spooldown_alt:
      rospy.logwarn('Spool down altitude is too small, or zero_height is too large')
      passchecks = 0
    if self.land_target_alt < self.zero_height:
      rospy.logwarn('Target landing altitude is below the zero altitude, proceed with caution!')
    if self.min_x >= self.max_x:
      rospy.logwarn('Operating region invalid in x-direction')
      passchecks = 0
    if self.min_y >= self.max_y:
      rospy.logwarn('Operating region invalid in y-direction')
      passchecks = 0
    if self.min_z >= self.max_z:
      rospy.logwarn('Operating region invalid in z-direction')
      passchecks = 0

    # Main while loop
    if passchecks:
      while not rospy.is_shutdown():
        self.check_state()
        if self.states[self.state] in ['Grounded','Emergency']:
          if self.is_pelican:
            self.asctec_disarm()
          else:
            self.send_disarm()
        elif self.states[self.state] == 'Spoolup':
          self.send_arm()
        elif self.states[self.state] == 'Spooldown':
          #self.calc_spooldown()
          self.spooldown_controls()
        elif self.states[self.state] in ['Takeoff','Hover','Experiment','ReturnHome','Land']:
          if self.state == self.states.index('Experiment') and self.experiment == self.experiments.index('RefVel'):
            self.velocity_calcs()
          #self.calc_controls()
          self.std_controls()
        elif self.states[self.state] == 'PositionFailsafe':
          self.update_autoland()
        else:
          rospy.logwarn('Warning, no controls being published because of a programming error!')

        # Update reference and error messages
        error = TransformStamped()
        error.transform.translation.x = self.ref_point.transform.translation.x - self.current_posn.transform.translation.x
        error.transform.translation.y = self.ref_point.transform.translation.y - self.current_posn.transform.translation.y
        error.transform.translation.z = self.ref_point.transform.translation.z - self.current_posn.transform.translation.z
        err_yaw = self.rotation_to_yaw(self.ref_point) - self.rotation_to_yaw(self.current_posn)
        error.transform.rotation = self.yaw_to_rotation(err_yaw)

        self.ref_pub.publish(self.ref_point)
        self.err_pub.publish(error)
        
        self.control_rate.sleep()

  ## ---------------------------------- Button input ---------------------------------- ##
  def button_callback(self,data):
    # Has a button been pressed
    if data.data != 0:
      rospy.loginfo('Pressed button id %i', data.data)

    # Is this a take off command
    if data.data & 1:
      self.command_takeoff()

    # Is this a landing command
    if data.data & 2:
      self.command_landing()

    # Is this an experiment command
    if data.data & 4:
      self.change_state(self.states.index('Experiment'))


  ## ---------------------------------- Commands input ---------------------------------- ##
  def commands_callback(self,data):
    # Before processing a command, allow client to clear latest message
    self.info_pub.publish('')

    if data.data == 'takeoff':
      self.command_takeoff()

    if data.data == 'land':
      self.command_landing()

    if data.data == 'hover':
      self.change_state(self.states.index('Hover'))

    if data.data == 'returnhome':
      self.change_state(self.states.index('ReturnHome'))

    if data.data == 'positionfailsafe':
      self.change_state(self.states.index('PositionFailsafe'))

    if data.data[0:4] == 'exp:':
      experiment = data.data[4:]
      expchange = 0
      if experiment == 'reflocal':
        expno = self.experiments.index('RefLocal')
        if self.experiment != expno or self.state != self.states.index('Experiment'):
          self.experiment = expno
          expchange = 1
      if experiment == 'refglobal':
        expno = self.experiments.index('RefGlobal')
        if self.experiment != expno or self.state != self.states.index('Experiment'):
          self.experiment = expno
          expchange = 1
      if experiment == 'refnudge':
        expno = self.experiments.index('RefNudge')
        if self.experiment != expno or self.state != self.states.index('Experiment'):
          self.experiment = expno
          expchange = 1
      if experiment == 'refvel':
        expno = self.experiments.index('RefVel')
        if self.experiment != expno or self.state != self.states.index('Experiment'):
          self.experiment = expno
          self.last_vel_update = rospy.get_rostime().to_sec()
          expchange = 1

      if expchange == 1:
        # If we're not experiment, try and start an experiment
        if self.state != self.states.index('Experiment'):
          self.change_state(self.states.index('Experiment'))

        # If we are experimenting, or was able to start experimenting then update the experiment type
        if self.state == self.states.index('Experiment'):
          self.experiment = expno

          # Tell the user about it
          rospy.logwarn('Experiment type changed to %s', self.experiments[self.experiment])
          self.info_pub.publish('Experiment type changed to ' + self.experiments[self.experiment])

    if data.data == 'resetintegrators':
      if self.state == self.states.index('Grounded'):
        self.pid.reset_integrators(Point())

    if data.data == 'getstate':
      self.status_pub.publish(self.states[self.state])

  ## ---------------------------------- Landing command ---------------------------------- ##
  def command_landing(self):
    if self.states[self.state] in ['Takeoff','Hover','Experiment']:
      self.change_state(self.states.index('Land'))
    elif self.states[self.state] == 'Spoolup':
      self.change_state(self.states.index('Grounded'))

  ## ---------------------------------- Take off command ---------------------------------- ##
  def command_takeoff(self):
    if self.state == self.states.index('Grounded'): # and self.ready:
      self.change_state(self.states.index('Spoolup'))

  ## ---------------------------------- Check ready for takeoff ---------------------------------- ##
  def check_ready(self):
    if abs(self.current_posn.transform.translation.x - self.home_location.transform.translation.x) < 0.1 and abs(self.current_posn.transform.translation.y - self.home_location.transform.translation.y) < 0.1 and abs(self.current_posn.transform.translation.z) < self.zero_height:
      self.ready = 1
    else:
      self.ready = 0

  ## ---------------------------------- State changing logic ---------------------------------- ##
  def change_state(self,newstate):
    # Change state may be called by internal process or button press command for example
    # This function does some checking and on state change actions
    state_changed = 0

    # Want to enter grounded state
    # Allow if we're spooling up or down
    if newstate == self.states.index('Grounded'):
      if self.states[self.state] in ['Spoolup','Spooldown','PositionFailsafe','Emergency']:
        self.state = newstate
        self.ref_point = self.copy_transform(self.home_location)
        self.ref_point.transform.translation.z = self.land_target_alt
        state_changed = 1
      else:
        rospy.logwarn('Cant become grounded from %s',self.states[self.state])

    # Want to enter spool up state
    # Allow if we're in grounded state, ready and have vicon
    elif newstate == self.states.index('Spoolup'):
      if self.state == self.states.index('Grounded'):
        self.check_ready()
        if self.valid_position and self.ready and self.ctrl_link_health == 1 and self.outside_fence == 0:
          self.state = newstate
          self.spoolup_start = rospy.get_rostime().to_sec()
          self.spoolup_complete = 0
          state_changed = 1
        else:
          if not self.valid_position:
            rospy.logwarn('Cannot take off due to lack of valid position')
            self.info_pub.publish('Cannot take off due to lack of valid position')
          elif not self.ready:
            rospy.logwarn('Aircraft must be below initial hover location to take off')
            self.info_pub.publish('Aircraft must be below initial hover location to take off')
          elif self.ctrl_link_health != 1:
            rospy.logwarn('Bad command link health, check heartbeats are being transmitted')
            self.info_pub.publish('Bad command link health, check heartbeats are being transmitted')
          elif self.outside_fence != 0:
            rospy.logwarn('Aircraft outside geofence, ignoring take off command')
            self.info_pub.publish('Aircraft outside geofence, ignoring take off command')

    # Want to enter takeoff state
    # Cannot be commanded directly, changes upon spool up completion
    elif newstate == self.states.index('Takeoff'):
      if self.state == self.states.index('Spoolup') and self.spoolup_complete == 1:
        self.state = newstate
        self.spoolup_complete = 0
        state_changed = 1

    # Want to enter hover state
    elif newstate == self.states.index('Hover'):
      if self.state == self.states.index('Takeoff') and self.takeoff_complete == 1:
        self.state = newstate
        self.takeoff_complete = 0
        state_changed = 1
      elif self.state == self.states.index('Experiment'):
        self.state = newstate
        # After leaving experiment mode, hover at current position
        self.ref_point = self.copy_transform(self.current_posn)
        # But we need to check fence breaches - need to hover slightly back inside area
        if self.outside_fence:
          pos = self.current_posn.transform.translation
          if pos.x > self.max_x:
            self.ref_point.transform.translation.x = self.max_x - 0.2
          if pos.y > self.max_y:
            self.ref_point.transform.translation.y = self.max_y - 0.2
          if pos.z > self.max_z:
            self.ref_point.transform.translation.z = self.max_z - 0.2
          if pos.x < self.min_x:
            self.ref_point.transform.translation.x = self.min_x + 0.2
          if pos.y < self.min_y:
            self.ref_point.transform.translation.y = self.min_y + 0.2
          if pos.z < self.min_z:
            self.ref_point.transform.translation.z = self.min_z + 0.2
        state_changed = 1
        # Zero experiment velocity
        self.ref_vel = Twist()
      elif self.state == self.states.index('ReturnHome'):
        self.state = newstate
        state_changed = 1

    # Want to return to original home position and hover
    elif newstate == self.states.index('ReturnHome'):
      if self.states[self.state] in ['Hover', 'Experiment']:
        self.state = newstate
        state_changed = 1
        self.ref_point = self.copy_transform(self.home_location)

    # Want to enter experiment state
    elif newstate == self.states.index('Experiment'):
      if self.state == self.states.index('Hover'):
        self.state = newstate
        state_changed = 1


    # Want to enter land state
    elif newstate == self.states.index('Land'):
      if self.states[self.state] in ['Takeoff','Hover','Experiment']:
        self.state = newstate
        self.ref_point = self.current_posn
        self.land_start_alt = self.ref_point.transform.translation.z
        self.land_start_time = rospy.get_rostime().to_sec()
        #self.ref_point.transform.translation.z = self.land_target_alt
        state_changed = 1

    # Want to enter spool down state
    elif newstate == self.states.index('Spooldown'):
      if self.state == self.states.index('Land'):
        self.landing_complete = 0
        self.spooldn_start = rospy.get_rostime().to_sec()
        self.state = newstate
        state_changed = 1

    # Want to enter emergency state - not fully realised, currently just commands landing
    elif newstate == self.states.index('Emergency'):
      self.command_landing()

    # Lose vicon, perform open loop descent
    elif newstate == self.states.index('PositionFailsafe'):
      if self.state != self.states.index('Grounded'):
        # Log time of entry to position failsafe
        self.autoland_start = rospy.get_rostime().to_sec()

        self.init_autoland()
        self.state = newstate
        state_changed = 1

    if state_changed:
      rospy.logwarn('State changed to %s', self.states[newstate])

    self.status_pub.publish(self.states[self.state])

  ## ---------------------------------- Regular state checking ---------------------------------- ##
  # Update state function to be polled, makes checks to see if we should transition to a different state
  def check_state(self):

    # Perform system health checks, e.g. vicon position, user controller heartbeats, geofencing

    # Check if the aircraft has left the geofenced area
    if self.outside_fence == 0:
      if self.check_fence() == 1:
        # Update variables flagging left area
        self.outside_fence = 1
        self.fence_breach_t = rospy.get_rostime().to_sec()
        # Change state to hover, cancel any experiments
        self.change_state(self.states.index('Hover'))
        # Let the user know
        str_dirn = ''
        pos = self.current_posn.transform.translation
        if pos.x <= self.min_x:
          str_dirn = str_dirn + ', -ve x'
        if pos.x >= self.max_x:
          str_dirn = str_dirn + ', +ve x'
        if pos.y <= self.min_y:
          str_dirn = str_dirn + ', -ve y'
        if pos.y >= self.max_y:
          str_dirn = str_dirn + ', +ve y'
        if pos.z <= self.min_z:
          str_dirn = str_dirn + ', -ve z'
        if pos.z >= self.max_z:
          str_dirn = str_dirn + ', +ve z'
        rospy.logwarn('Aircraft outside geofence, switched to hover' + str_dirn)
        self.info_pub.publish('Aircraft outside geofence, switched to hover' + str_dirn)

    # If we violate the geofence for too long, then command a landing
    if self.outside_fence == 1:
      if rospy.get_rostime().to_sec() - self.fence_breach_t > self.fence_maxtime:
        rospy.logwarn('Outside fence for too long, command a landing')
        self.command_landing()
        self.outside_fence = 2
      if self.check_fence() == 0:
        self.outside_fence = 0

    # Have we violated for too long
    if self.outside_fence == 2 and self.state == self.states.index('Grounded'):
      if self.check_fence() == 0:
        self.outside_fence = 0

    # Check for position time out
    if self.valid_position == 1:
      if rospy.get_rostime().to_sec() - self.last_position > self.position_timeout:
        self.valid_position = 0
        self.change_state(self.states.index('PositionFailsafe'))

    # Check if command link has gone down
    if self.ctrl_link_health == 1:
      if rospy.get_rostime().to_sec() - self.last_heartbeat > self.hb_timeout:
        rospy.logwarn('Command link timed out')
        self.info_pub.publish('Command link timed out')
        self.ctrl_link_health = 0
        self.command_landing()

    # Consider the state we're in. Check if still valid and update useful variables

    # Grounded state
    if self.state == self.states.index('Grounded'):
      self.check_ready()

    # Spooling up
    elif self.state == self.states.index('Spoolup'):
      self.check_ready()
      if self.ready != 1:
        self.change_state(self.states.index('Grounded'))
        rospy.logwarn('Vehicle moved too much during spool up, aborting take off')
        self.info_pub.publish('Vehicle moved too much during spool up, aborting take off')

      delta_t = rospy.get_rostime().to_sec() - self.spoolup_start
      if delta_t >= self.spoolup_duration:
        self.spoolup_complete = 1
        self.change_state(self.states.index('Takeoff'))

    # Taking off
    elif self.state == self.states.index('Takeoff'):
      if self.current_posn.transform.translation.z >= 0.9*self.home_location.transform.translation.z:
        self.takeoff_complete = 1
        self.ref_point.transform.translation.z = self.home_location.transform.translation.z
        self.change_state(self.states.index('Hover'))
      else:
        delta_t = rospy.get_rostime().to_sec() - self.spoolup_start - self.spoolup_duration
        if delta_t >= self.takeoff_timeout:
          self.change_state(self.states.index('Land'))
          rospy.logwarn('Take off procedure timed out, aborting')
          self.info_pub.publish('Take off procedure timed out, aborting')

        # Update reference point during take off as ramp in z
        z_ratio = delta_t/self.takeoff_duration
        if z_ratio < 0:
          z_ratio = 0
        if z_ratio > 1:
          z_ratio = 1
        self.ref_point.transform.translation.z = self.land_target_alt + z_ratio*(self.home_location.transform.translation.z - self.land_target_alt)

    # Hovering
    elif self.state == self.states.index('Hover'):
      pass

    # Returning home
    elif self.state == self.states.index('ReturnHome'):
      if abs(self.current_posn.transform.translation.x - self.home_location.transform.translation.x) < 0.25 and abs(self.current_posn.transform.translation.y - self.home_location.transform.translation.y) < 0.25 and abs(self.current_posn.transform.translation.z - self.home_location.transform.translation.z) < 0.25:
        self.change_state(self.states.index('Hover'))

    # Experimenting
    elif self.state == self.states.index('Experiment'):
      pass

    # Landing
    elif self.state == self.states.index('Land'):
      if self.current_posn.transform.translation.z <= self.spooldown_alt:
        self.change_state(self.states.index('Spooldown'))
      else:
        #rospy.logwarn('Current altitude = %.2f', self.current_posn.transform.translation.z)
        delta_t = rospy.get_rostime().to_sec() - self.land_start_time

        # Update reference point during landing as ramp in z
        z_ratio = delta_t/self.landing_duration
        if z_ratio < 0:
          z_ratio = 0
        if z_ratio > 1:
          z_ratio = 1
        self.ref_point.transform.translation.z = self.land_start_alt - z_ratio*(self.land_start_alt - self.land_target_alt)
        # rospy.logwarn('Z = %.2f, U = %.2f', self.current_posn.transform.translation.z, self.last_thrust)

    # Spooling down motors
    elif self.state == self.states.index('Spooldown'):
      delta_t = rospy.get_rostime().to_sec() - self.spooldn_start
      if delta_t >= self.spooldn_duration:
        self.landing_complete = 1
        self.change_state(self.states.index('Grounded'))

    # Emergency state
    elif self.state == self.states.index('Emergency'):
      pass


  ## ---------------------------------- Setpoints / inputs ---------------------------------- ##
  def ref_local_callback(self,data):
    # rospy.logwarn('Received local reference input')
    if self.states[self.state] == 'Experiment' and self.experiment == self.experiments.index('RefLocal'):
      self.local_to_global_ref(data, 0)

  def ref_nudge_callback(self,data):
    rospy.logwarn('Received local nudge input')
    if self.states[self.state] == 'Experiment' and self.experiment == self.experiments.index('RefNudge'):
      self.local_to_global_ref(data, 1)
      
  def twist_callback(self,data):
    rospy.logwarn('Received velocity reference input')
    if self.states[self.state] == 'Experiment' and self.experiment == self.experiments.index('RefVel'):
      if abs(data.linear.x) > self.max_vel or abs(data.linear.y) > self.max_vel or abs(data.linear.z) > self.max_vel:
        rospy.logwarn('New velocity input too high, ignored')
      else:
        self.ref_vel = data

  def ref_global_callback(self,data):
    # Only allow new reference inputs if we're in experiment mode
    if self.states[self.state] == 'Experiment' and self.experiment == self.experiments.index('RefGlobal'):
      # Sanity check new refpoint
      if self.check_absolute_ref(data.transform.translation):
        if self.check_relative_ref(data.transform.translation):
            self.ref_point = data
            self.ref_yaw = self.rotation_to_yaw(self.ref_point)
            rospy.loginfo('New reference received: (%f, %f, %f, %f)', self.ref_point.transform.translation.x, self.ref_point.transform.translation.y, self.ref_point.transform.translation.z, self.ref_yaw)
        else:
            rospy.logwarn('New reference outside of relative limits')
            self.info_pub.publish('New reference outside of relative limits')
      else:
        rospy.logwarn('New reference outside of absolute limits')
        self.info_pub.publish('New reference outside of relative limits')

  def heartbeat_callback(self, data):
    self.ctrl_link_health = 1
    self.last_heartbeat = rospy.get_rostime().to_sec()


  ## ---------------------------------- Setpoint processing ---------------------------------- ##
  def velocity_calcs(self):
    now_t = rospy.get_rostime().to_sec()
    delta_t = now_t - self.last_vel_update
    self.last_vel_update = now_t
    if delta_t > self.control_rate.sleep_dur.to_sec()*1.5:
      # delta_t is unusually high, don't do anything
      rospy.logwarn('High delta_t in velocity calcs %.3f > %.3f', delta_t,  self.control_rate.sleep_dur.to_sec()*1.5)
      return
    else:
      dist = Twist()
      dist.linear.x = self.ref_vel.linear.x * delta_t
      dist.linear.y = self.ref_vel.linear.y * delta_t
      dist.linear.z = self.ref_vel.linear.z * delta_t
      dist.angular.z = self.ref_vel.angular.z * delta_t
      self.local_to_global_ref(dist, 1)

  def local_to_global_ref(self, data, isadditive):
    # if len(kwargs) == 0:
    #   return
    # data = kwargs.values()[0]
    if type(data) == TransformStamped: # kwargs.keys()[0] == 'transform':
      dx = data.transform.translation.x
      dy = data.transform.translation.y
      dz = data.transform.translation.z
      dyaw = self.rotation_to_yaw(data)
    elif type(data) == Twist: #kwargs.keys()[0] == 'twist':
      dx = data.linear.x
      dy = data.linear.y
      dz = data.linear.z
      dyaw = data.angular.z
    else:
      return

    # Create a new reference point and check absolute movement before accepting
    newref = TransformStamped()
    if isadditive:
      oldpoint = self.ref_point
    else:
      oldpoint = self.current_posn

    y_yaw = self.rotation_to_yaw(self.current_posn)
    r_yaw = self.rotation_to_yaw(oldpoint)
    newref.transform.translation.x = oldpoint.transform.translation.x + dx*math.cos(y_yaw) - dy*math.sin(y_yaw)
    newref.transform.translation.y = oldpoint.transform.translation.y + dx*math.sin(y_yaw) + dy*math.cos(y_yaw)
    newref.transform.translation.z = oldpoint.transform.translation.z + dz
    newref.transform.rotation = self.yaw_to_rotation(r_yaw + dyaw)
    if self.check_absolute_ref(newref.transform.translation):
      if self.check_relative_ref(newref.transform.translation):
        self.ref_point = newref
      else:
        rospy.logwarn('New local reference outside of relative limits')
    else:
      rospy.logwarn('New local reference outside of fence')

    # y_yaw = self.rotation_to_yaw(self.current_posn)
    # r_yaw = self.rotation_to_yaw(self.ref_point)
    # self.ref_point.transform.translation.x = self.ref_point.transform.translation.x + dx*math.cos(y_yaw) - dy*math.sin(y_yaw)
    # self.ref_point.transform.translation.y = self.ref_point.transform.translation.y + dx*math.sin(y_yaw) + dy*math.cos(y_yaw)
    # self.ref_point.transform.translation.z = self.ref_point.transform.translation.z + dz
    # self.ref_point.transform.rotation = self.yaw_to_rotation(r_yaw + dyaw)
    #rospy.logwarn('dx = %.4f, dy = %.4f, dz = %.4f', dx, dy, dz)



  ## ---------------------------------- Setpoint / input sanity checks ---------------------------------- ##
  def check_absolute_ref(self,data):
    return data.x >= self.min_x and data.x <= self.max_x and data.y >= self.min_y and data.y <= self.max_y and data.z >= self.min_z and data.z <= self.max_z

  def check_relative_ref(self, data):
    if hasattr(self, 'current_posn'):
      xdiff = self.current_posn.transform.translation.x - data.x
      ydiff = self.current_posn.transform.translation.y - data.y
      zdiff = self.current_posn.transform.translation.z - data.z

      rospy.loginfo('relative reference %f', np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff))
      return np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff) < self.rel_threshold
    else:
      return True

  ## ---------------------------------- Fence breaching checks ---------------------------------- ##
  def check_fence(self):
    pos = self.current_posn.transform.translation
    return not (pos.x >= self.min_x and pos.x <= self.max_x and pos.y >= self.min_y and pos.y <= self.max_y and pos.z >= self.min_z and pos.z <= self.max_z)


  ## ---------------------------------- Position updates ---------------------------------- ##
  def vicon_callback(self,data):
    #self.current_pos = Point(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    self.current_posn = data
    #publish current target point to Rviz
    self.target_br.sendTransform((self.ref_point.transform.translation.x, self.ref_point.transform.translation.y, self.ref_point.transform.translation.z), 
		(0.0, 0.0, 0.0, 1.0), rospy.Time.now(), rospy.get_namespace() + "/control_ref_point", "world")
    # extract the time in seconds
    t = data.header.stamp.to_sec()
    self.last_position = rospy.get_rostime().to_sec()
    self.valid_position = 1

  ## ---------------------------------- Quadrotor info feedback ---------------------------------- ##
  def asctec_callback(self,data):
    self.asctec_info = data
    # rospy.logwarn('Battery = %5.2f', data.battery_voltage_1/1000.0)

  ## ---------------------------------- Quadrotor control calculations ---------------------------------- ##
  def init_autoland(self):
    # Calculate open loop controls
    Iterms = self.pid.read_integrators()
    self.u_autoland.linear.x = self.neutral_pitch + Iterms.x
    self.u_autoland.linear.y = self.neutral_roll + Iterms.y
    # self.u_autoland.linear.z = self.neutral_thrust + Iterms.z + self.autoland_thrustdz
    self.u_autoland.linear.z = self.last_thrust + self.autoland_thrustdz
    self.u_autoland.angular.z = self.neutral_yaw

    # Make a log of thrust on way down
    self.last_thrust = self.u_autoland.linear.z

    # Publish controls
    self.rc_pub.publish(self.u_autoland)

  def update_autoland(self):
    delta_t = rospy.get_rostime().to_sec() - self.autoland_start
    if delta_t < self.autoland_time:
      self.rc_pub.publish(self.u_autoland)
    elif delta_t < self.autoland_time + self.spooldn_duration:
      # Modify thrust command to simply ramp down from previous thrust setting
      c_thrust = (self.last_thrust - (rospy.get_rostime().to_sec()-(self.autoland_start + self.autoland_time))/self.spooldn_duration) * (self.last_thrust+1)
      if c_thrust > self.last_thrust:
        c_thrust = self.last_thrust
      if c_thrust < -1:
        c_thrust = -1

      self.u_autoland.linear.z = c_thrust
      self.rc_pub.publish(self.u_autoland)
    else:
      self.change_state(self.states.index('Grounded'))


  def std_controls(self):
    u = self.calc_controls()

    # send it to the bridge
    self.rc_pub.publish(u) #rc_ctrl)

    # Save the last known thrust input, in case we start spooling down
    self.last_thrust = u.linear.z

  def spooldown_controls(self):
    u = self.calc_controls()

    # Modify thrust command to simply ramp down from initial thrust setting
    c_thrust = (self.last_thrust - (rospy.get_rostime().to_sec()-self.spooldn_start)/self.spooldn_duration) * (self.last_thrust+1)
    if c_thrust > self.last_thrust:
      c_thrust = self.last_thrust
    if c_thrust < -1:
      c_thrust = -1

    u.linear.z = c_thrust

    # send it to the bridge
    self.rc_pub.publish(u)


  def calc_controls(self):
    # only enable integral action when not on the ground - to avoid wind-up
    if self.current_posn.transform.translation.z > self.zero_height and self.states[self.state] not in ['Grounded','PositionFailsafe']:
      self.pid.enable_integrator()
    else:
      self.pid.freeze_integrator()
    # Update pid controller
    u = self.pid.update(self.current_posn, self.ref_point, rospy.get_rostime().to_sec())

    # Add neutral inputs onto controls
    u.linear.x = u.linear.x + self.neutral_pitch
    u.linear.y = u.linear.y + self.neutral_roll
    u.linear.z = u.linear.z + self.neutral_thrust
    u.angular.z = u.angular.z + self.neutral_yaw

    # reduce thrust input if in ground effect
    if self.current_posn.transform.translation.z <= self.ground_effect_alt:
      c_thrust_offset = 1 - self.current_posn.transform.translation.z/self.ground_effect_alt
      if c_thrust_offset > 1:
        c_thrust_offset = 1
      if c_thrust_offset < 0:
        c_thrust_offset = 0
      c_thrust_offset = self.ground_effect_scale * c_thrust_offset
      u.linear.z = u.linear.z - c_thrust_offset

    # print data.transform.translation.x, 0.0, t, u_roll
    rospy.loginfo('(Roll pitch yaw) = (%f %f %f) Thrust = %f',u.linear.y, u.linear.x, u.angular.z, u.linear.z)

    # return controls to calling function
    return u

  def freeze_int_callback(self, data):
    if data:
      rospy.logwarn('Enabled integrator due to callback')
      self.thrust_pid.freeze_integrator()
    else:
      self.thrust_pid.enable_integrator()

  def send_arm(self):
    self.rc_pub.publish(self.arming_input)

  def send_disarm(self):
    self.rc_pub.publish(self.disarming_input)

  def asctec_disarm(self):
    # Asctec autopilots disarm with same input as the arming input
    # Return true once vehicle has been disarmed
    if self.asctec_info.flying == 1:
      self.rc_pub.publish(self.arming_input)
      return 0
    else:
      self.send_disarm()
      return 1

  ## ---------------------------------- Helper functions ---------------------------------- ##

  def yaw_to_rotation(self, yaw):
    # Converts a yaw angle in radians into quaternions as TransformStamped.translation.rotation
    quaternion = quaternion_from_euler(0,0,yaw)
    output = TransformStamped()
    output.transform.rotation.x = quaternion[0]
    output.transform.rotation.y = quaternion[1]
    output.transform.rotation.z = quaternion[2]
    output.transform.rotation.w = quaternion[3]
    return output.transform.rotation

  def rotation_to_yaw(self, data):
    # Converts a TransformStamped.translation.rotation into a yaw angle
    # Function takes the TransformStamped() directly as the input
    quaternion = (
      data.transform.rotation.x,
      data.transform.rotation.y,
      data.transform.rotation.z,
      data.transform.rotation.w)
    euler_angle = euler_from_quaternion(quaternion)
    return euler_angle[2]

  def copy_transform(self, data):
    new = TransformStamped()
    new.transform.translation.x = data.transform.translation.x
    new.transform.translation.y = data.transform.translation.y
    new.transform.translation.z = data.transform.translation.z
    new.transform.rotation.x = data.transform.rotation.x
    new.transform.rotation.y = data.transform.rotation.y
    new.transform.rotation.z = data.transform.rotation.z
    new.transform.rotation.w = data.transform.rotation.w
    return new

if __name__ == "__main__":
  qc = quad_control()
  rospy.spin()



