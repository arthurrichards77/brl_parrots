#!/usr/bin/env python
import roslib
roslib.load_manifest('brl_parrots')
import rospidlib.rospidlib_quad as rospidlib_quad
import tf
from geometry_msgs.msg import Twist

if __name__ == "__main__":
  qc = quad_control()
  rospy.spin()



