#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Imu
from std_msgs.msg import String

acc_threshold = 0.8
pos_bump = 0.3

def viconCallback(data):
  # need to explicitly ref message store global as I'm writing to it
  global last_tf
  # just store in case I need it
  last_tf = data

def imuCallback(data):
  # default ref transform
  ref_tf = last_tf
  # suppress reflex under 0.3m altitude
  if last_tf.transform.translation.z > 0.3:
    # check for X collision
    if data.linear_acceleration.x < -acc_threshold:
      # reflex fowards
      ref_tf.transform.translation.x = ref_tf.transform.translation.x - pos_bump
      pub_ref_tf.publish(ref_tf)
      pub_msg.publish('BUMP! Back away')
      rospy.loginfo('Bump: back away')

rospy.init_node('collision_reflex', anonymous=True)
# subscribes to vicon and ARDrone IMU
sub_vicondata = rospy.Subscriber('drone', TransformStamped, viconCallback)
sub_ref_imu = rospy.Subscriber('ardrone/imu', Imu, imuCallback)
# publishes to "ref_tf" channel to command drone position
pub_ref_tf = rospy.Publisher('ref_tf', TransformStamped, queue_size=1)
# publisher for status message to monitor
pub_msg = rospy.Publisher('monitor/status_msg',String, queue_size=1)

# and for reference position
last_tf = TransformStamped()

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
