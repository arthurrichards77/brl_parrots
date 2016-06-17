#!/usr/bin/env python
import rospy
import numpy
import math
from aapacs_drones.msg import OptflowLK
from geometry_msgs.msg import Twist


def callback(data):
  # average calcs
  ave = 0.0
  imwidth = data.width

  # draw the tracks
  for i in range(0, data.tags_count):       
    # calculate averages
    #print str(i) + " of " + str(data.tags_count)
    if data.tags_x[i] != 0.5*(imwidth):
      ave = ave + data.tags_dx[i]
  
	# normalize averages
  if data.tags_count == 0:
    ave = 0
  else:
    ave = ave/data.tags_count

  # Publish average rotational rate in Twist()
  output = Twist()
  output.angular.z = ave*0.2
  pub.publish(output)


# Node setup
rospy.init_node('optflow_to_twist', anonymous=False)
pub = rospy.Publisher("opticflow_vel", Twist, queue_size=1)
rospy.Subscriber('opticflow',OptflowLK,callback)
rospy.spin()

