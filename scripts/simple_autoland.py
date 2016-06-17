#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String

def resetCallback(data):
  global currState
  # if drone reset called, reset the state machine
  currState=0
  str = "Auto-land reset"
  rospy.loginfo(str)
  pub_msg.publish(str)

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
  # only used in state 3 (landing)
  if currState==3:
    # if I'm close to the ground, land anyway
    if data.altd<300:
      pub_land.publish(Empty())
      rospy.loginfo('Control timeout - landing')
      pub_msg.publish('Control timeout - land')
      # no need to do this again
      currState=4
    else:
      # still tracking OK - run feedback
      cmd_twist = Twist()
      # linear feedback on yaw
      cmd_twist.linear.z = -0.0005*data.altd
      # limit in [-0.5,0.5]
      if cmd_twist.linear.z>0.5:
        cmd_twist.linear.z = 0.5
      elif cmd_twist.linear.z<-0.5:
        cmd_twist.linear.z = -0.5
      # send message
      pub_Cmd.publish(cmd_twist)
  
# setup node and subs/pubs
rospy.init_node('drone_follow', anonymous=False)
# subscribers
sub_Navdata = rospy.Subscriber('ardrone/navdata', Navdata, navCallback)
sub_cmd_vel = rospy.Subscriber('control_vel', Twist, inpVelCallback)
sub_reset = rospy.Subscriber('ardrone/reset', Empty, resetCallback)
# publishers
pub_Cmd = rospy.Publisher('cmd_vel', Twist,queue_size=1)
pub_land = rospy.Publisher('ardrone/land', Empty,queue_size=1)
pub_msg = rospy.Publisher('monitor/status_msg', String,queue_size=1)

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
    rospy.loginfo('Control timeout - back away')
    pub_msg.publish('Control timeout - back away')
    currState=2
    # start reverse
    rev_cmd = Twist()
    rev_cmd.linear.x = -0.2
    pub_Cmd.publish(rev_cmd)
  elif currState==2:
    # end of reverse: start landing
    print 'Landing\r\n'
    rospy.loginfo('Control timeout - descend')
    pub_msg.publish('Control timeout - descend')
    currState=3

