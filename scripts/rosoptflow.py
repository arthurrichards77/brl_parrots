#!/usr/bin/env python
import roslib
roslib.load_manifest('aapacs_drones')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aapacs_drones.msg import OptflowLK

class image_converter:

  def __init__(self):

    cv2.namedWindow("Optic flow window", 1)
    self.bridge = CvBridge()

    # get the image topic to subscribe to
    rospy.loginfo("Subscribing to " + rospy.resolve_name('image'))
    self.image_sub = rospy.Subscriber('image',Image,self.callback)
    self.data_pub = rospy.Publisher('opticflow',OptflowLK,queue_size=1)

    # running frame counter
    self.frame_ctr = 0

    # number of frames between fresh feature track
    self.corner_interval = rospy.get_param('~corner_interval',10)
    rospy.loginfo("Refreshing corners every %i frames"%self.corner_interval)

    # params for ShiTomasi corner detection
    self.feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

    # Parameters for lucas kanade optical flow
    self.lk_params = dict( winSize  = (15,15),
                           maxLevel = 2,
                           criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Create some random colors
    self.color = np.random.randint(0,255,(100,3))

  def callback(self,data):
    try:
      cv1_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # increment frame counter
    self.frame_ctr = self.frame_ctr+1

    # convert image to grayscale
    cv_image_col = np.asarray(cv1_image)
    cv_image = cv2.cvtColor(cv_image_col, cv2.COLOR_BGR2GRAY)

    # get the size
    imshape = cv_image.shape
    imwidth = imshape[1]
    imheight = imshape[0]

    # trap the first frame to grab corners
    if self.frame_ctr==1:

      # make a mask
      self.mask = np.zeros_like(cv_image_col)
      self.blankmask = self.mask

    else:

      try:

        # blank message for publishing
        outdata = OptflowLK()

        # copy over width, height and timestamp from image
        outdata.header = data.header
        outdata.width = imwidth
        outdata.height = imheight

        # flag for req new features
        newFeatReq = False

        # calc the optic flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_image, cv_image, self.p0, None, **self.lk_params)

        # Select good points
        good_new = p1[st==1]
        good_old = self.p0[st==1]

        # average calcs
        ave_x_exp = 0.0
        
        # draw the tracks
        for i,(new,old) in enumerate(zip(good_new,good_old)):
          a,b = new.ravel()
          c,d = old.ravel()
          cv2.line(self.mask, (a,b),(c,d), self.color[i].tolist(), 2)
          cv2.circle(cv_image_col,(a,b),5,self.color[i].tolist(),-1)
          # append to output message list
          outdata.tags_x = outdata.tags_x + [c]
          outdata.tags_y = outdata.tags_y + [d]
          outdata.tags_dx = outdata.tags_dx + [a-c]
          outdata.tags_dy = outdata.tags_dy + [b-d]
          outdata.tags_count = outdata.tags_count + 1        
          # calculate averages
          ave_x_exp = ave_x_exp + (a-c)/(c - 0.5*imwidth)

        # normalize averages
        if outdata.tags_count == 0:
          ave_x_exp = 0
        else:
          ave_x_exp = ave_x_exp/outdata.tags_count
        print outdata.header.stamp.secs,ave_x_exp


        # store points for next time
        self.p0 = good_new.reshape(-1,1,2)

        # publish
        self.data_pub.publish(outdata)

      except TypeError, e:
        
        # caught here if no matching features
        print "Failed to find any matches!"
        print e
        newFeatReq = True

      # combine latest image with tracks
      img = cv2.add(cv_image_col,self.mask)
      
      # show output and wait for something to happen
      cv2.imshow("Optic flow window", img)
      cv2.waitKey(3)

    # store image for next time
    self.old_image = cv_image.copy()

    # occasional refresh of features
    if self.frame_ctr % self.corner_interval == 1:

      # find corners
      self.p0 = cv2.goodFeaturesToTrack(cv_image, mask = None, **self.feature_params)

      # reset mask
      self.mask = self.blankmask.copy()

def main(args):

  ic = image_converter()
  rospy.init_node('optic_flow', anonymous=False)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
