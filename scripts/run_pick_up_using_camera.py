from ipaddress import ip_address
from frankapy import FrankaArm
import numpy as np
import argparse
import cv2, time
from cv_bridge import CvBridge
from autolab_core import RigidTransform, Point
from perception import CameraIntrinsics
from utils import *
from pynput import keyboard

from frankapy import FrankaConstants as FC

resume_execution = False
def on_press(key):
    global resume_execution
    try:
        if keyboard.Key.esc:
            print("Esc key pressed. Resume execution")
            resume_execution = True
            return False  # Stop listener
    except AttributeError:
        pass  # Special keys like ctrl, etc.

AZURE_KINECT_INTRINSICS = '/home/saumyas/franka/azure_kinect_calibration/calib/kinect2_front_tilt/ir_intrinsics_azure_kinect_tilt_720.intr'
# AZURE_KINECT_EXTRINSICS = '/home/saumyas/franka/azure_kinect_calibration/calib/kinect2_overhead/kinect2_overhead_to_world.tf'
AZURE_KINECT_EXTRINSICS = '/home/saumyas/franka/azure_kinect_calibration/calib/kinect2_front_tilt/kinect2_front_tilt_to_world.tf'
SENSOR_NAME = 'kinect2_front_tilt'
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics_file_path', type=str, default=AZURE_KINECT_INTRINSICS)
    parser.add_argument('--extrinsics_file_path', type=str, default=AZURE_KINECT_EXTRINSICS) 
    args = parser.parse_args()

    print('Starting robot')
    fa = FrankaArm()    

    print('Opening Grippers')
    #Open Gripper

    #Reset Pose
    fa.reset_joints()
    fa.reset_pose() 

    fa.open_gripper()
    fa.goto_gripper(0.02)
    # fa.close_gripper()

    cv_bridge = CvBridge()
    azure_kinect_intrinsics = CameraIntrinsics.load(args.intrinsics_file_path)
    azure_kinect_to_world_transform = RigidTransform.load(args.extrinsics_file_path)

    azure_kinect_rgb_image = get_azure_kinect_rgb_image(cv_bridge)
    azure_kinect_depth_image = get_azure_kinect_depth_image(cv_bridge)

    object_image_position = np.array([800, 800])

    def onMouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
           print('x = %d, y = %d'%(x, y))
           param[0] = x
           param[1] = y
        
    cv2.namedWindow('Choose grasp point', cv2.WINDOW_NORMAL)
    cv2.imshow('Choose grasp point', azure_kinect_rgb_image)
    cv2.setMouseCallback('Choose grasp point', onMouse, object_image_position)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    object_center_point_in_world = get_object_center_point_in_world(
        object_image_position[0],
        object_image_position[1],
        azure_kinect_depth_image, 
        azure_kinect_intrinsics,
        azure_kinect_to_world_transform,
        sensor_name=SENSOR_NAME
    )

    object_center_pose = fa.get_pose()
    object_z_height = 0.015
    object_center_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1], object_z_height]

    intermediate_robot_pose = object_center_pose.copy()
    intermediate_pose_z_height = 0.19
    intermediate_robot_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1], intermediate_pose_z_height]

    lift_up_pose = object_center_pose.copy()
    lift_up_pose_z_height = 0.12
    lift_up_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1], lift_up_pose_z_height]

    right_pose = object_center_pose.copy()
    right_pose_z_height = 0.10
    right_pose.translation = [object_center_point_in_world[0], object_center_point_in_world[1]+0.25, lift_up_pose_z_height]

    # Move to intermediate robot pose
    print('Moving to intermediate grasp pose')
    fa.goto_pose(intermediate_robot_pose)

    # Move to grasping robot pose
    print('Moving to grasp pose')
    fa.goto_pose(object_center_pose, 10, force_thresholds=[10, 10, 10, 10, 10, 10])

    #Close Gripper
    print('Closing gripper')
    fa.close_gripper()
    
    #Move to intermediate lift pose robot pose
    print('Lifting up')
    fa.goto_pose(lift_up_pose, 5, force_thresholds=[10, 10, 10, 10, 10, 10])
    
    print('Moving right')
    # fa.goto_pose(right_pose, duration=0.6, cartesian_impedances=[650, 650, 650] + FC.DEFAULT_ROTATIONAL_STIFFNESSES)
    fa.goto_pose(right_pose)
    
    #Open Gripper
    print('Opening Grippers')
    fa.goto_gripper(0.02)

    print('Moving to intermediate reset pose')
    intermediate_reset_pose = fa.get_pose()
    intermediate_reset_pose.translation[2] = 0.25
    fa.goto_pose(intermediate_reset_pose, ignore_virtual_walls=True)

    print('Moving to intermediate initial pose')
    fa.goto_pose(intermediate_reset_pose, ignore_virtual_walls=True)

    #Reset Pose
    fa.reset_pose() 
    #Reset Joints
    fa.reset_joints()