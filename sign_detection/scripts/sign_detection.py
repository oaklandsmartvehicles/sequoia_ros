#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
import rospkg
import cv2
import message_filters
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from sign_detection.msg import SignData, SignDataArray

#This node takes ZED camera information to detect a traffic sign and publishes the position as well as label of the detected sign 
class sign_detector:

  def __init__(self):
    #Find package path    
    rospack = rospkg.RosPack()
    sign_detection_path = rospack.get_path('sign_detection') + "/cascades/" 
    #Load cascade files for sign detection
    self.stop_sign_cascade = cv2.CascadeClassifier(sign_detection_path + "ow_left_arrow.xml")
    print(sign_detection_path) 
    #Setup message filter to synchronize the subscribers (Zed left camera, Registered Point Cloud)
    self.bridge = CvBridge()  
    self.image_sub = message_filters.Subscriber("/zed/left/image_raw_color", Image)
    self.cloud_sub = message_filters.Subscriber("/zed/point_cloud/cloud_registered", PointCloud2)

    self.pub_sign_data = rospy.Publisher('sign_data', SignDataArray, queue_size=1)

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
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    signs = self.stop_sign_cascade.detectMultiScale(gray, 1.3, 5)

    sign_data_array = SignDataArray()
    sign_data_array.header = pointcloud.header

    #Acquire the point cloud x,y,z position from the u,v position index    
    for (x,y,w,h) in signs:
      desiredU = x
      desiredV = y
      data_out = pc2.read_points(pointcloud,field_names=None, skip_nans=False, uvs=[[desiredU,desiredV]])
      int_data = next(data_out)
      print(int_data)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      roi_gray = gray[y:y+h, x:x+w]
      roi_color = cv_image[y:y+h, x:x+w]
      cv2.imshow("Image window", cv_image)
      cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'sign_type'
      sign.sign_position.x = int_data[0]
      sign.sign_position.y = int_data[1]
      sign.sign_position.z = int_data[2]

      sign_data_array.data.append(sign)

    self.pub_sign_data.publish(sign_data_array)




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
