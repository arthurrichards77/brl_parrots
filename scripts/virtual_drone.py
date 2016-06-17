#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, atan2
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# mode - "velocity" or "trajectory"
movement_mode = 'velocity'

# parent frame name, in which the virtual drone moves
global_frame = "world"
drone_frame = "virtual_drone"

# velocity scaling
ref_vel_scale = 2.0
ref_yaw_scale = 2.0

# joint names
my_joint_names = ["move_x","move_y","move_z","turn_z"]

# indices to find joints in trajectory message
my_joint_inds = [0,1,2,3]

# joint state message
js_msg = JointState()
js_msg.position = [0.0, 0.0, 0.0, 0.0]
js_msg.velocity = [0.0, 0.0, 0.0, 0.0]
js_msg.effort = [0.0, 0.0, 0.0, 0.0]

# executing trajectory
ref_traj = JointTrajectory()
ref_traj_start_time = rospy.Duration(0.0)

def saturate(inp,limit):
  # limit quantity to [-limit,limit]
  out = inp
  if inp>limit:
    out=limit
  elif inp<-limit:
    out = -limit
  return out

def trajUpdate():
  # globals so I can revert to hold mode at end
  global movement_mode
  global ref_velocity
  # default is to stay where I am
  x = ref_transform.translation.x
  y = ref_transform.translation.y
  z = ref_transform.translation.z
  # extract Euler angles from current pose
  rpyAngles = euler_from_quaternion(ref_transform.rotation)
  # and I just need the yaw
  yawAngle = rpyAngles[2]
  # get time relative to current trajectory
  time_since_traj_sent = rospy.Time.now() - ref_traj_start_time
  if time_since_traj_sent < ref_traj.points[0].time_from_start:
    # not started yet - nothing to do
    rospy.loginfo("Waiting for trajectory to start")    
  elif time_since_traj_sent > ref_traj.points[-1].time_from_start:
    # finished - also nothing to do
    rospy.loginfo("Finished trajectory")
    ref_velocity = Twist()  
    movement_mode = 'velocity'
  else:
    # find which time segment I am in
    ii = 1
    while time_since_traj_sent > ref_traj.points[ii].time_from_start:
      ii = ii+1
    rospy.loginfo("On trajectory segment %d" % ii)
    # find coefficients for linear interpolating
    t1 = (time_since_traj_sent - ref_traj.points[ii-1].time_from_start)
    t2 = (ref_traj.points[ii].time_from_start - ref_traj.points[ii-1].time_from_start)
    coeff = t1.to_sec()/t2.to_sec()
    # assume joint states in order for now
    x = ref_traj.points[ii-1].positions[my_joint_inds[0]]*(1.0-coeff)+ref_traj.points[ii].positions[my_joint_inds[0]]*coeff
    y = ref_traj.points[ii-1].positions[my_joint_inds[1]]*(1.0-coeff)+ref_traj.points[ii].positions[my_joint_inds[1]]*coeff
    z = ref_traj.points[ii-1].positions[my_joint_inds[2]]*(1.0-coeff)+ref_traj.points[ii].positions[my_joint_inds[2]]*coeff
    # clever interpolating of yaw to go through discontinuity
    c1=cos(ref_traj.points[ii-1].positions[my_joint_inds[3]])
    c2=cos(ref_traj.points[ii].positions[my_joint_inds[3]])
    s1=sin(ref_traj.points[ii-1].positions[my_joint_inds[3]])
    s2=sin(ref_traj.points[ii].positions[my_joint_inds[3]])
    c_now = c1*(1.0-coeff) + c2*coeff
    s_now = s1*(1.0-coeff) + s2*coeff
    yawAngle = atan2(s_now,c_now)
  # send back the new joints
  return(x,y,z,yawAngle)
  

def velUpdate():
  # update for reference in velocity mode
  # extract Euler angles from current pose
  rpyAngles = euler_from_quaternion(ref_transform.rotation)
  # and I just need the yaw
  yawAngle = rpyAngles[2]
  # propagate the reference
  x = ref_transform.translation.x + delta_t*ref_vel_scale*(ref_velocity.linear.x*cos(yawAngle)-ref_velocity.linear.y*sin(yawAngle))
  y = ref_transform.translation.y + delta_t*ref_vel_scale*(ref_velocity.linear.x*sin(yawAngle)+ref_velocity.linear.y*cos(yawAngle))
  z = ref_transform.translation.z + delta_t*ref_vel_scale*ref_velocity.linear.z
  # update the rotation as well
  yawAngle = yawAngle + delta_t*ref_yaw_scale*ref_velocity.angular.z
  # return the new joint states
  return (x,y,z,yawAngle)

def refUpdate():
  # need to explicitly ref global as I'm writing to it
  global ref_transform
  global js_msg
  # update according to what mode we're in
  if movement_mode == 'velocity':
    (x,y,z,yawAngle)=velUpdate()
  elif movement_mode == 'trajectory':
    (x,y,z,yawAngle)=trajUpdate()

  # put the new states back in the transform memory
  ref_transform.translation.x = x;
  ref_transform.translation.y = y;
  ref_transform.translation.z = z;
  ref_transform.rotation = quaternion_from_euler(0.0, 0.0, yawAngle)

  # publish the reference as a transform
  # pub_ref_tf.sendTransform((ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z),
  #                         ref_transform.rotation,
  #                         rospy.Time.now(),
  #                         drone_frame, global_frame)

  # and as a set of joint states for the URDF drone
  js_msg.header.stamp = rospy.Time.now()
  js_msg.position = [ref_transform.translation.x, ref_transform.translation.y, ref_transform.translation.z, yawAngle]
  pub_joint_states.publish(js_msg)

def refVelCallback(data):
  global ref_velocity
  global movement_mode
  movement_mode = 'velocity'
  ref_velocity = data

def trajCallback(data):
  global ref_traj
  global ref_traj_start_time
  global movement_mode
  global my_joint_inds
  # first check that all required joints are present
  if all([joint in data.joint_names for joint in my_joint_names]):
    my_joint_inds = [data.joint_names.index(joint_name) for joint_name in my_joint_names]
    movement_mode = 'trajectory'
    ref_traj_start_time = rospy.Time.now()
    ref_traj = data
    rospy.loginfo("Got new trajectory")
  else:
    rospy.loginfo("Ignoring trajectory - joints missing")

def reftfCallback(data):
  global ref_velocity
  global movement_mode
  movement_mode = 'velocity'
  # set reference movement to zero
  ref_velocity = Twist()
  # and set position to whatever was received
  ref_transform.translation.x = data.transform.translation.x
  ref_transform.translation.y = data.transform.translation.y
  ref_transform.translation.z = data.transform.translation.z

rospy.init_node('virtual_drone', anonymous=True)
sub_ref_vel = rospy.Subscriber('cmd_vel', Twist, refVelCallback)
sub_ref_traj = rospy.Subscriber('/cmd_traj', JointTrajectory, trajCallback)
#sub_ref_pos = rospy.Subscriber('cmd_tf', TransformStamped, reftfCallback)
#pub_ref_tf = tf.TransformBroadcaster()
pub_joint_states = rospy.Publisher('joint_states', JointState)

# use drone name parameter as prefix to joint names
if rospy.has_param('drone_name'):
  drone_name = rospy.get_param('drone_name')
  my_joint_names=[drone_name + "_" + joint_name for joint_name in my_joint_names]
js_msg.name=my_joint_names

# and for reference position
ref_transform = Transform()
# default altitude
ref_transform.translation.z = 1.2
# default orientation
ref_transform.rotation = quaternion_from_euler(0.0, 0.0, 0.0)
# and the reference velocity
ref_velocity = Twist()

# update rate and time step
updateRateHz = 10
delta_t = 1./updateRateHz
rate = rospy.Rate(updateRateHz)

while not rospy.is_shutdown():
  refUpdate()
  rate.sleep()
