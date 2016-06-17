#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket, sys
import struct
import time
import random
from rospy_tutorials.msg import Floats
import numpy
from rospy.numpy_msg import numpy_msg

def ref_generation():
  xbox_ctrl = rospy.Publisher('xboxctrl', numpy_msg(Floats))
  rospy.init_node('xboxbridge', anonymous=True)
  rate = rospy.Rate(100) # Run no faster than 100hz
  UDP_IP = "192.168.10.81"
  UDP_PORT = 27300

  sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
  sock.bind((UDP_IP, UDP_PORT))
  sock.setblocking(0)

  packer = struct.Struct('c c c c f f f f B B h')
  while not rospy.is_shutdown():
    try:
      data = sock.recv(48) # buffer size is 48 bytes
      if len(data) == 24:
        if data[0] == "X" and data[1] == "B" and data[2] == "O" and data[3] == "X":
          output = packer.unpack(data)
          ctrl = numpy.array([output[4],output[5],output[6],output[7],output[8]], dtype=numpy.float32)
          xbox_ctrl.publish(ctrl)
          #print output
    except:
      # print "Nothing"
      if str(sys.exc_info()[1]) != '[Errno 11] Resource temporarily unavailable':
        rospy.logwarn("Unexpected error: %s", sys.exc_info()[1])
      nothing = 0

    rate.sleep()

if __name__ == '__main__':
  try:
    ref_generation()
  except rospy.ROSInterruptException:
    pass


