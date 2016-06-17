#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import rospy
import numpy
import rospidlib
import rospidlib_quad
import tf
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class setpoint_control:

  def __init__(self):
    self.node = rospy.init_node('setpoint_control', anonymous=True)
    # publisher for RC commands
    self.rc_pub = rospy.Publisher('rcctrl', numpy_msg(Floats))
    # subscriber for Vicon data
    self.vicon_sub = rospy.Subscriber('drone', TransformStamped, self.vicon_callback)
    # subscriber for reference point input
    self.point_sub = rospy.Subscriber('refpoint', TransformStamped, self.ref_callback)
    # subscriber for integrator freeze
    self.freeze_int = rospy.Subscriber('freeze_int', Bool, self.freeze_int_callback)

    # PID Controller
    self.pid = rospidlib_quad.RospidQuad(0.075,0.0,0.11,'~pid')
    # freeze the pid integrator until flying    
    self.pid.freeze_integrator()
    # current position as point
    self.current_pos = Point()
    self.current_posn = TransformStamped()
    # point reference input in vicon coordinate frame
    self.ref_point = TransformStamped()
    self.ref_yaw = 0
    # Rate of main control loop
    self.control_rate = rospy.Rate(float(rospy.get_param('~control_rate', '50')))
    # default is at origin, 0.8m off ground
    self.ref_point.transform.translation.x = rospy.get_param('init_x', 0.0)
    self.ref_point.transform.translation.y = rospy.get_param('init_y', 0.0)
    self.ref_point.transform.translation.z = rospy.get_param('init_z', 0.8)
    # Geofence: min/max permissible setpoints
    self.min_x = rospy.get_param('/min_x', -3.0)
    self.min_y = rospy.get_param('/min_y', -5.0)
    self.min_z = rospy.get_param('/min_z', 0.0)
    self.max_x = rospy.get_param('/max_x', 3.0)
    self.max_y = rospy.get_param('/max_y', 5.0)
    self.max_z = rospy.get_param('/max_z', 3.0)
    self.rel_threshold = rospy.get_param('/rel_threshold', 2.5)
    #broadcast target point to RViz
    self.target_br = tf.TransformBroadcaster()

    # Main while loop
    while not rospy.is_shutdown():
      self.calc_controls()
      self.control_rate.sleep()
      

  def ref_callback(self,data):
    #Sanity check new refpoint
    if self.check_absolute_ref(data):
        if self.check_relative_ref(data):
            self.ref_point = data
	    quaternion = (
	      self.ref_point.transform.rotation.x,
	      self.ref_point.transform.rotation.y,
	      self.ref_point.transform.rotation.z,
	      self.ref_point.transform.rotation.w)
	    ref_euler = euler_from_quaternion(quaternion)
	    self.ref_yaw = ref_euler[2]
            rospy.loginfo('New reference received: (%f, %f, %f, %f)', self.ref_point.transform.translation.x, self.ref_point.transform.translation.y, self.ref_point.transform.translation.z, self.ref_yaw)
        else:
            rospy.loginfo('New reference outside of relative limits')
    else:
        rospy.loginfo('New reference outside of absolute limits')

  def check_absolute_ref(self,data):
    return data.x >= self.min_x and data.x <= self.max_x and data.y >= self.min_y and data.y <= self.max_y and data.z >= self.min_z and data.z <= self.max_z

  def check_relative_ref(self, data):
    if hasattr(self, 'current_pos'):
      xdiff = self.current_pos.x - data.x
      ydiff = self.current_pos.y - data.y
      zdiff = self.current_pos.z - data.z

      rospy.loginfo('relative reference %f', np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff))
      return np.sqrt(xdiff*xdiff+ydiff*ydiff+zdiff*zdiff) < self.rel_threshold
    else:
      return True

  def vicon_callback(self,data):
    self.current_pos = Point(data.transform.translation.x, data.transform.translation.y, data.transform.translation.z)
    self.current_posn = data
    #publish current target point to Rviz
    self.target_br.sendTransform((self.ref_point.transform.translation.x, self.ref_point.transform.translation.y, self.ref_point.transform.translation.z), 
		(0.0, 0.0, 0.0, 1.0), rospy.Time.now(), rospy.get_namespace() + "/control_ref_point", "world")
    #rospy.loginfo('%s',data)
    # extract the time in seconds
    t = data.header.stamp.to_sec()
    #t = data.header.stamp

  def calc_controls(self):
    # only enable integral action when over 20cm off ground - to avoid wind-up
    if self.current_posn.transform.translation.z > 0.2:
      self.pid.enable_integrator()
    else:
      self.pid.freeze_integrator()
    # Update pid controller
    u = self.pid.update(self.current_posn, self.ref_point, rospy.get_rostime().to_sec())

    # Update each control loop
    u_roll = u.transform.translation.x
    u_pitch = u.transform.translation.y
    u_thrust = u.transform.translation.z
    u_yaw = u.transform.rotation.z
    # centre around 0.5 and limit
    c_roll = 0.5 + rospidlib.saturate(u_roll,0.25)
    c_pitch = 0.5 + rospidlib.saturate(u_pitch,0.25)
    c_yaw = 0.5 + rospidlib.saturate(u_yaw,0.5)
    # except thrust, centered around something bigger
    c_thrust = 0.6 + rospidlib.saturate(u_thrust,0.35)
    # print data.transform.translation.x, 0.0, t, u_roll
    rospy.loginfo('(Roll pitch yaw) = (%f %f %f) Thrust = %f',u_roll, u_pitch, u_yaw, u_thrust)
    # rospy.loginfo('Thrust integrator = %f', self.thrust_pid.read_integrator())
    # compile into message for RC bridge
    # channels: 1 = Roll (pos right) 2 = Pitch (pos forward) 3 = Thrust (pos up) 4 = Yaw (pos right / CW)
    rc_ctrl = numpy.array([c_roll,c_pitch,c_thrust,c_yaw,0.0,0.0,0.0,0.0], dtype=numpy.float32)
    # send it to the bridge
    self.rc_pub.publish(rc_ctrl)

  def freeze_int_callback(self, data):
    if data:
      self.thrust_pid.freeze_integrator()
    else:
      self.thrust_pid.enable_integrator()

if __name__ == "__main__":
  sp = setpoint_control()
  rospy.spin()



