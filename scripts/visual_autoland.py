#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def inpVelCallback(data):
  # record that there was one
  global currState
  # disarm trigger if armed
  if currState==1:
    currState=0
    # and pass along the message
    pub_Cmd.publish(data)
  elif currState==0:
    # just pass along the message
    pub_Cmd.publish(data)

def navCallback(data):
  global currState
  # if I'm close to the ground, land anyway
  if data.altd<200:
    pub_land.publish(Empty())
  # only matters if I'm rotating/looking
  if currState==3:
    # if tracking
    if data.tags_count>0:
      print 'Found tag!\r\n'
      # start landing
      currState=4
  if currState==4:
    if data.tags_count<1:
      # lost it - go back to rotate
      print 'Lost tag!\r\n'
      currState=3
      rot_cmd = Twist()
      rot_cmd.angular.z = 0.1
      pub_Cmd.publish(rot_cmd)
    else:
      # still tracking OK - run feedback
      cmd_twist = Twist()
      # linear feedback on yaw
      cmd_twist.angular.z = 0.0003*(500 - data.tags_xc[0])
      # limit in [-0.5,0.5]
      if cmd_twist.angular.z>0.5:
        cmd_twist.angular.z = 0.5
      elif cmd_twist.angular.z<-0.5:
        cmd_twist.angular.z = -0.5
      # linear feedback from height to downward
      cmd_twist.linear.z = -0.0005*data.altd*0
      # limit in [-0.5,0.5]
      if cmd_twist.linear.z>0.3:
        cmd_twist.linear.z = 0.3
      elif cmd_twist.linear.z<-0.3:
        cmd_twist.linear.z = -0.3
      # send message
      pub_Cmd.publish(cmd_twist)
      # linear feedback from width to forward
      cmd_twist.linear.x = -0.0002*(600 - data.tags_yc[0])
      # limit in [-0.5,0.5]
      if cmd_twist.linear.x>0.1:
        cmd_twist.linear.x = 0.1
      elif cmd_twist.linear.x<-0.1:
        cmd_twist.linear.x = -0.1
      # send message
      pub_Cmd.publish(cmd_twist)
  
# setup node and subs/pubs
rospy.init_node('drone_follow', anonymous=False)
sub_Navdata = rospy.Subscriber('ardrone/navdata', Navdata, navCallback)
sub_cmd_vel = rospy.Subscriber('control_vel', Twist, inpVelCallback)
pub_Cmd = rospy.Publisher('cmd_vel', Twist)
pub_land = rospy.Publisher('ardrone/land', Empty)

# state machine variable
currState = 0

# run at 1Hz
r = rospy.Rate(0.5)

while not rospy.is_shutdown():
  r.sleep()
  if currState==0:
    # arm trigger for input timeout
    currState=1
  elif currState==1:
    # no input since arm - takeover
    print 'Timeout!\r\n'
    currState=2
    # start reverse
    rev_cmd = Twist()
    rev_cmd.linear.x = -0.03
    pub_Cmd.publish(rev_cmd)
  elif currState==2:
    # end of reverse: start rotation and looking
    print 'Looking for tag\r\n'
    currState=3
    rot_cmd = Twist()
    rot_cmd.angular.z = 0.1
    pub_Cmd.publish(rot_cmd)
