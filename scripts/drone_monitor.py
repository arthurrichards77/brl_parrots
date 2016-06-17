#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ardrone_autonomy.msg import Navdata

class image_converter:

  def __init__(self):

    self.imname = rospy.resolve_name("image")
    cv2.namedWindow(self.imname, 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image",Image,self.callback)
    self.nav_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navcallback)
    self.msg_sub = rospy.Subscriber("monitor/status_msg",String,self.msgcallback)
    self.last_batt = 100
    self.status_msg = "Status"

  def navcallback(self,data):
    # store the battery percentage
    self.last_batt = data.batteryPercent

  def msgcallback(self,data):
    # store the status message
    self.status_msg = data.data

  def callback(self,data):
    try:
      cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # conversion from fuerte bridge format to new opencv
    cv_image = numpy.asarray(cv1_image)

    # show battery percentage and status message
    screen_msg="Battery = %i percent" % self.last_batt
    cv2.putText(cv_image, screen_msg, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    cv2.putText(cv_image, self.status_msg, (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)

    # show the image
    cv2.imshow(self.imname, cv_image)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
