#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

if __name__ == '__main__':
    try:
        rospy.init_node('pub_camera_info')

        yaml_file = rospy.get_param('~yaml_file')

        # Load data from file
        with open(yaml_file, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]

        rospy.Publisher('camera_info', CameraInfo, latch=True, queue_size=1).publish(camera_info_msg)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
