#!/usr/bin/env python
import roslib
roslib.load_manifest('drone_oflow2')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist

def navCallback(data):
  # default zero output
  cmd_twist = Twist()
  # if tracking
  if data.tags_count>0:
    # linear feedback on yaw
    cmd_twist.angular.z = 0.002*(500 - data.tags_xc[0])
    # limit in [-0.5,0.5]
    if cmd_twist.angular.z>0.5:
      cmd_twist.angular.z = 0.5
    elif cmd_twist.angular.z<-0.5:
      cmd_twist.angular.z = -0.5
    # linear feedback from width to forward
    cmd_twist.linear.x = 0.003*(40 - data.tags_width[0])
    # limit in [-0.5,0.5]
    if cmd_twist.linear.x>0.3:
      cmd_twist.linear.x = 0.3
    elif cmd_twist.linear.x<-0.3:
      cmd_twist.linear.x = -0.3
    print cmd_twist
  # send message
  pub_Cmd.publish(cmd_twist)

  

rospy.init_node('drone_follow', anonymous=True)
sub_Navdata = rospy.Subscriber('/brenda/ardrone/navdata', Navdata, navCallback)
pub_Cmd = rospy.Publisher('/brenda/cmd_vel', Twist)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
