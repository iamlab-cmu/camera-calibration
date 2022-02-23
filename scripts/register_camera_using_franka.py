"""
Adapted from autolab/perception package's register camera script:
Script to register sensors to a chessboard for the YuMi setup
Authors: Jeff Mahler and Brenton Chu
""" 
import argparse
import cv2
import numpy as np
import os
import sys
import time
import traceback
import rospy

from autolab_core import Point, PointCloud, RigidTransform, YamlConfig
from perception import CameraChessboardRegistration, RgbdSensorFactory, CameraIntrinsics

from frankapy import FrankaArm

if __name__ == '__main__': 

    # parse args
    parser = argparse.ArgumentParser(description='Register a camera to a robot')
    parser.add_argument('--config_filename', type=str, default='cfg/register_azure_kinect_with_franka.yaml', help='filename of a YAML configuration for registration')
    parser.add_argument('--intrinsics_dir', type=str, default='calib/azure_kinect.intr')
    args = parser.parse_args()
    config_filename = args.config_filename
    config = YamlConfig(config_filename)

    robot = FrankaArm()
    
    print('Applying 0 force torque control for {}s'.format(20))
    robot.run_guide_mode(20)

    T_ee_world = robot.get_pose()

    # Get T_cb_world by using T_ee_world*T_cb_ee
    # T_cb_ee = RigidTransform(rotation=np.array([[0, 0, 1],[1, 0, 0],[0, 1, 0]]),
    #                          translation=np.array([0.02275, 0, -0.0732]), 
    #                          from_frame='cb', to_frame='franka_tool')
    T_cb_ee = RigidTransform(rotation=np.array([[0, 0, 1],[-1, 0, 0],[0, -1, 0]]),
                             translation=np.array([0.02275, 0, -0.0732]), 
                             from_frame='cb', to_frame='franka_tool')
    # T_cb_ee = RigidTransform(rotation=np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]),
    #                          translation=np.array([0.02275, 0, -0.0732]), 
    #                          from_frame='cb', to_frame='franka_tool')

    T_cb_world = T_ee_world * T_cb_ee

    # get camera sensor object
    for sensor_frame, sensor_data in config['sensors'].items():
        rospy.loginfo('Registering %s' %(sensor_frame))
        sensor_config = sensor_data['sensor_config']
        if 'registration_config' in sensor_data:
            registration_config = sensor_data['registration_config'].copy()
        else:
            registration_config = {}
        registration_config.update(config['chessboard_registration'])
        
        # open sensor
        try:
            sensor_type = sensor_config['type']
            sensor_config['frame'] = sensor_frame
            rospy.loginfo('Creating sensor')
            sensor = RgbdSensorFactory.sensor(sensor_type, sensor_config)
            rospy.loginfo('Starting sensor')
            sensor.start()
            if args.intrinsics_dir:
                ir_intrinsics = CameraIntrinsics.load(args.intrinsics_dir)
            else:
                ir_intrinsics = sensor.ir_intrinsics
            rospy.loginfo('Sensor initialized')

            # register
            reg_result = CameraChessboardRegistration.register(sensor, registration_config)
            print("Robot Transform")
            print(T_ee_world)
            print("Checkerboard in Camera Transform")
            print(reg_result.T_camera_cb)
            T_camera_world = T_cb_world * reg_result.T_camera_cb

            rospy.loginfo('Final Result for sensor %s' %(sensor_frame))
            rospy.loginfo('Rotation: ')
            rospy.loginfo(T_camera_world.rotation)
            rospy.loginfo('Quaternion: ')
            rospy.loginfo(T_camera_world.quaternion)
            rospy.loginfo('Translation: ')
            rospy.loginfo(T_camera_world.translation)

        except Exception as e:
            rospy.logerr('Failed to register sensor {}'.format(sensor_frame))
            traceback.print_exc()
            continue

        # save tranformation arrays based on setup
        output_dir = os.path.join(config['calib_dir'], sensor_frame)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        rospy.loginfo('Saving to {}'.format(output_dir))
        pose_filename = os.path.join(output_dir, '%s_to_world.tf' %(sensor_frame))
        T_camera_world.save(pose_filename)
        if not args.intrinsics_dir:
            intr_filename = os.path.join(output_dir, '%s.intr' %(sensor_frame))
            ir_intrinsics.save(intr_filename)
        f = os.path.join(output_dir, 'corners_cb_%s.npy' %(sensor_frame))
        np.save(f, reg_result.cb_points_cam.data)
                
        sensor.stop()
