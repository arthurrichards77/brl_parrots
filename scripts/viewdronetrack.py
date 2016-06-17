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
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("ardrone/image_raw",Image,self.callback)
    self.nav_sub = rospy.Subscriber("ardrone/navdata",Navdata,self.navcallback)

  def navcallback(self,data):
    # store the first tag detected, if any
    if data.tags_count>0:
      self.last_tag = (data.tags_xc[0], data.tags_yc[0], data.tags_width[0])
    else:
      self.last_tag = ()

  def callback(self,data):
    try:
      cv1_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    # conversion from fuerte bridge format to new opencv
    cv_image = numpy.asarray(cv1_image)

    # get img size and mark centre
    (rows,cols,channels) = cv_image.shape
    ctr_x = int(0.5*cols)
    ctr_y = int(0.5*rows)
    cv2.line(cv_image, (ctr_x-10, ctr_y), (ctr_x+10, ctr_y), (0,255,0))
    cv2.line(cv_image, (ctr_x, ctr_y-10), (ctr_x, ctr_y+10), (0,255,0))

    # circle tag location, if any seen    
    if len(self.last_tag)>0 :
      cx = int(cols*self.last_tag[0]/1000.0)
      cy = int(rows*self.last_tag[1]/1000.0)
      cv2.circle(cv_image, (cx,cy), self.last_tag[2], 255)
      screen_msg="Tracking (%i,%i) W=%i " % self.last_tag
      cv2.putText(cv_image, screen_msg, (20,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)



    # show the image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError, e:
    #  print e

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
