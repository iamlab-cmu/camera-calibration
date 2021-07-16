"""
Makes the Azure Kinect Overhead Intrinsics File by Subscribing to the Azure Kinect's camera_info topic.
""" 
import argparse
import numpy as np
import os
import rospy
from sensor_msgs.msg import CameraInfo

if __name__ == '__main__':
    # parse args
    parser = argparse.ArgumentParser(description='Register a camera to a robot')
    parser.add_argument('--camera_info_topic_name', type=str, default='/rgb/camera_info', help='topic name for camera_info')
    parser.add_argument('--intrinsics_dir', type=str, default='calib/azure_kinect_overhead/')
    parser.add_argument('--output_file_name', type=str, default='azure_kinect_overhead.intr')
    args = parser.parse_args()
    
    rospy.init_node('register_azure_kinect_overhead', anonymous=True)
    
    camera_info_msg = rospy.wait_for_message(args.camera_info_topic_name, CameraInfo)

