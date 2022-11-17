 ###################################################################
 #
 # This python script is for generating sets of [cam pose, end-effector pose]
 # for robot hand-eye calibration using the Franka Panda Robot.
 #
 # Usage : Move robot to desired positions then press [enter] to record...repeat. 
 #
 # Output : cam_transform.csv, ee_transform.csv
 # 
 # Input  : /camera/color/image_raw (published through realsense2_camera package)
 #          /tf (published through Frankapy API)
 #          ChAruco board 
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0
 ###################################################################
 
import numpy as np
import rospy

import time
import ChAruco
import cv2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
import csv


import tf

#get multiple frames
desired_num_points_for_calib = 15

# filenames to store saved csv files
f_cam = open('/home/student/data/cam_transform.csv', 'w')
f_ee = open('/home/student/data/ee_transform.csv', 'w')


if __name__ == "__main__":
    
    print(" ----------- start  of script ----------- ")
    
    #cam matrix, dist coeff from RealsenseD435 factory settings
    #read in from /camera/color/camera_info ROS topic
    camera_matrix = np.array([[616.75732421875, 0.,319.7129821777344],[0., 616.644775390625, 246.04098510742188],[0., 0., 1. ]])
    dist_coeff = np.array([0,0,0,0,0])

    #wait for image callback thread before proceeding
    ChAruco_instance = ChAruco.ChAruco()
    while ChAruco_instance.recevied_image == False:
        time.sleep(1)

    #display received image for debugging
    img = ChAruco_instance.get_image()
    img_size = ChAruco_instance.get_image_shape
    cv2.imshow("image", img) 
    cv2.waitKey(0)

    #call guidance mode to Franka robot
    ChAruco_instance.run_guide_robot()

    #create header for Transform Array 
    #not needed b/c won't be publishing directly but through publisher.py file
    cam_tf = TransformArray()
    cam_tf.header.frame_id = "/camera_link"
    cam_tf.header.stamp = rospy.Time.now()
    ee_tf = TransformArray()

    
    # create the csv writer
    writer = csv.writer(f_cam)
    writer_ee = csv.writer(f_ee)

    #For desired number of calib pts, move robot and collect [cam pose, EE pose]
    for i in range (desired_num_points_for_calib):
            print(f"this is {i} index")
            cv2.imshow("image", img)
            cv2.waitKey(0) #<------ stopper until moving robot to desired position

            #get camera pose from camera
            gotpose, rot,trans = ChAruco_instance.get_offset(camera_matrix, dist_coeff)

            # print(f"pose, rot,trans {gotpose,rot,trans}")
            R_np,_ = cv2.Rodrigues(rot)
            R = R.from_matrix(R_np)
            quat = R.as_quat()

            #convert to TF
            cam_transform = Transform()
            cam_transform.translation.x = trans[0]
            cam_transform.translation.y = trans[1]
            cam_transform.translation.z = trans[2]

            cam_transform.rotation.x = quat[0]
            cam_transform.rotation.y = quat[1]
            cam_transform.rotation.z = quat[2]
            cam_transform.rotation.w = quat[3]

            cam_tf.transforms.append(cam_transform)
            print(f"cam_transform: {cam_transform} ")

            # #get EE
            tf_listener = tf.TransformListener()
            rospy.sleep(1)
            (ee_trans,ee_quat) = tf_listener.lookupTransform('/panda_link0', '/panda_end_effector', rospy.Time(0))
            print(f"ee_trans: {ee_trans} ")
            ee_transform = Transform()
            ee_transform.translation.x = ee_trans[0]
            ee_transform.translation.y = ee_trans[1]
            ee_transform.translation.z = ee_trans[2]
            
            ee_transform.rotation.x = ee_quat[0]
            ee_transform.rotation.y = ee_quat[1]
            ee_transform.rotation.z = ee_quat[2]
            ee_transform.rotation.w = ee_quat[3]

            ee_tf.transforms.append(ee_transform)

            cam_row = [trans[0], trans[1], trans[2], quat[0], quat[1], quat[2], quat[3] ]
            # write a row to the csv file
            writer.writerow(cam_row)

            ee_row = [ee_trans[0], ee_trans[1], ee_trans[2], ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3] ]
            # write a row to the csv file
            writer_ee.writerow(ee_row)


    # close the file
    f_cam.close()
    f_ee.close()
    print(" ----------- end of script ----------- ")