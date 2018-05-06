#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import message_filters
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError

#This node takes ZED camera information to detect a traffic sign and publishes the position as well as label of the detected sign 
class sign_detector:

  def __init__(self):
    #Setup message filter to synchronize the subscribers (Zed left camera, Registered Point Cloud)
    self.bridge = CvBridge()  
    self.image_sub = message_filters.Subscriber("/zed/left/image_raw_color", Image)
    self.cloud_sub = message_filters.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.cloud_sub], 10,0.3)
    ts.registerCallback(self.callback)
  def callback(self, image, pointcloud):
    #Converts cv_bridge image type to opencv MAT type
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      print(e)

    #Show left stereo camera image
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    #U,V position in the image frame of the sign
    desiredU = 240
    desiredV = 250

    #Acquire the point cloud x,y,z position from the u,v position index
    data_out = pc2.read_points(pointcloud,field_names=None, skip_nans=False, uvs=[[desiredU,desiredV]])
    int_data = next(data_out)
    print(int_data)

def main(args):
  sd = sign_detector()
  
  rospy.init_node('sign_detector', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
