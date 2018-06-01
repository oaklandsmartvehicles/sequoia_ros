#!/usr/bin/env python

from __future__ import print_function
import math
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
    self.stop_sign_cascade = cv2.CascadeClassifier(sign_detection_path + "stopsign_classifier.xml")
    self.ow_left_cascade   = cv2.CascadeClassifier(sign_detection_path + "ow_left_arrow.xml")
    self.ow_right_cascade  = cv2.CascadeClassifier(sign_detection_path + "ow_right_arrow.xml")
    self.closed_cascadeV3  = cv2.CascadeClassifier(sign_detection_path + "road_closed_v3.xml")
    self.no_left_cascade = cv2.CascadeClassifier(sign_detection_path   + "no_trun_left_symbol_v2.xml")
    self.no_right_cascade = cv2.CascadeClassifier(sign_detection_path  + "no_turn_right.xml")
    self.no_turns_cascade = cv2.CascadeClassifier(sign_detection_path  + "no_turns.xml")   

    print(sign_detection_path) 
    #Setup message filter to synchronize the subscribers (Zed left camera, Registered Point Cloud)
    self.bridge = CvBridge()  
    self.image_sub = message_filters.Subscriber("/zed/left/image_raw_color", Image)
    self.depth_sub = message_filters.Subscriber("/zed/depth/depth_registered", Image)


    self.pub_sign_data = rospy.Publisher('sign_data', SignDataArray, queue_size=1)

    ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10,0.3)
    ts.registerCallback(self.callback)

  def callback(self, image, depth_image):
    #Converts cv_bridge image type to opencv MAT type
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")   
      cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")
    except CvBridgeError as e:
      print(e)


    height, width, channels = cv_image.shape
    ROI = cv_image[(0):(height/2),width/2:(width-1)]
    #U,V position in the image frame of the sign
    
    gray = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
    
    ow_left_signs = self.ow_left_cascade.detectMultiScale(gray,3,5) 
    ow_right_signs= self.ow_right_cascade.detectMultiScale(gray,3,5)
    
    closed_signsV3 = self.closed_cascadeV3.detectMultiScale(gray,2,2)

    stop_signs = self.stop_sign_cascade.detectMultiScale(gray,2,2)

    no_left_signs = self.no_left_cascade.detectMultiScale(gray,1.5,2)
    no_right_signs = self.no_right_cascade.detectMultiScale(gray,1.5,2) 
    no_turns_signs = self.no_turns_cascade.detectMultiScale(gray,2,2) 

    sign_data_array = SignDataArray()
    sign_data_array.header = depth_image.header
    font = cv2.FONT_HERSHEY_SIMPLEX
    #Acquire the point cloud x,y,z position from the u,v position index    
    for (x,y,w,h) in no_turns_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2)
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      cv2.putText(cv_image,'No Turns',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'No Turns'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)


    for (x,y,w,h) in no_left_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2)
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      cv2.putText(cv_image,'No Turn Left',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'No Turn Left'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)

    for (x,y,w,h) in no_right_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2) 
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      cv2.putText(cv_image,'No Turn Right',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)      
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'No Turn Right'
      sign.sign_position.x = depth 
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)




    for (x,y,w,h) in closed_signsV3:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      #cv2.imshow("Image window", cv_image)
      cv2.putText(cv_image,'Road Closed',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)        
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'Closed Road'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)



    for (x,y,w,h) in stop_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2)
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      #cv2.imshow("Image window", cv_image)
      cv2.putText(cv_image,'Stop Sign',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)        
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'Stop Sign'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)

    for (x,y,w,h) in ow_left_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2)
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)

      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      #cv2.imshow("Image window", cv_image)
      cv2.putText(cv_image,'One Way Left',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)        
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'One Way Left'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)


    for (x,y,w,h) in ow_right_signs:
    #fix ROI displacment add (width/2) to x
      x = x + (width/2)
      desiredU = x + (w/2)
      desiredV = y + (h/2)
      depth = cv_depth_image[desiredV,desiredU]
      if  math.isinf(depth) or math.isnan(depth):
        continue
      
      #print(depth)
      # Draw position
      cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
      #roi_gray = gray[y:y+h, x:x+w]
      #roi_color = cv_image[y:y+h, x:x+w]
      cv2.putText(cv_image,'One Way Right',(x+w,y+h), font, 0.5, (0,255,255), 2, cv2.LINE_AA)        
      #cv2.waitKey(3)

      sign = SignData()
      sign.sign_type = 'One Way Right'
      sign.sign_position.x = depth
      sign.sign_position.y = 0
      sign.sign_position.z = 0

      sign_data_array.data.append(sign)
    
    #Show left stereo camera image
    #cv2.imshow("Image window", ROI) 
    cv2.waitKey(3)
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
