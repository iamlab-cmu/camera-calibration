import os
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridgeError
from autolab_core import Point
import open3d as o3d

def get_azure_kinect_rgb_image(cv_bridge, topic='/rgb/image_raw', prefix=''):
    """
    Grabs an RGB image for the topic as argument
    """
    rgb_image_msg = rospy.wait_for_message(prefix + topic, Image)
    try:
        rgb_cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return rgb_cv_image

def get_azure_kinect_depth_image(cv_bridge, topic='/depth_to_rgb/image_raw', prefix=''):
    """
    Grabs an Depth image for the topic as argument
    """
    depth_image_msg = rospy.wait_for_message(topic, Image)
    try:
        depth_cv_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return depth_cv_image

def get_realsense_rgb_image(cv_bridge, topic='/camera/color/image_raw'):
    """
    Grabs an RGB image for the topic as argument
    """
    rgb_image_msg = rospy.wait_for_message(topic, Image)
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_image_msg)
        rgb_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
    except CvBridgeError as e:
        print(e)
    
    return rgb_cv_image

def get_realsense_depth_image(cv_bridge, topic='/camera/aligned_depth_to_color/image_raw'):
    """
    Grabs an Depth image for the topic as argument
    """
    depth_image_msg = rospy.wait_for_message(topic, Image)
    try:
        depth_cv_image = cv_bridge.imgmsg_to_cv2(depth_image_msg)
    except CvBridgeError as e:
        print(e)
    
    return depth_cv_image

def createFolder(directory):
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
    except OSError:
        print ('Error: Creating directory. ' +  directory)

def get_object_center_point_in_world(object_image_center_x, object_image_center_y, depth_image, intrinsics, transform):
    object_center = Point(np.array([object_image_center_x, object_image_center_y]), intrinsics.frame)
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    print("x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
        object_image_center_x, object_image_center_y, object_depth))
    
    object_center_point_in_world = transform * intrinsics.deproject_pixel(object_depth, object_center)    
    print(object_center_point_in_world)

    return object_center_point_in_world 


def get_object_center_point_in_world_realsense(
    object_image_center_x,
    object_image_center_y,
    depth_image,
    intrinsics,
    transform,
    current_pose,
):

    object_center = Point(
        np.array([object_image_center_x, object_image_center_y]),
        "realsense_ee",
    )
    object_depth = depth_image[object_image_center_y, object_image_center_x] * 0.001
    print(
        "x, y, z: ({:.4f}, {:.4f}, {:.4f})".format(
            object_image_center_x, object_image_center_y, object_depth
        )
    )

    object_center_point_in_world = current_pose * transform * intrinsics.deproject_pixel(
        object_depth, object_center
    )
    print(object_center_point_in_world)

    return object_center_point_in_world

def make_o3d_pose(T, size=0.1):
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    mesh.transform(T.matrix)

    return mesh