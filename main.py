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
import pandas

import tf

#get multiple frames
desired_num_points_for_calib = 35

# filenames to store saved csv files
f_cam = open('./data/cam_transform.csv', 'w')
f_ee = open('./data/ee_transform.csv', 'w')


# ==== Options to set for different configurations ====

# Use guide mode for collecting samples
use_guide_mode = False

# Use fixed q_positions from a csv file with joint positions.
# Will only be used if `use_guide_mode=False`
q_positions_txt = './data/camera_in_hand_back_joint_samples.txt'

if __name__ == "__main__":

    print(" ----------- start  of script ----------- ")

    #cam matrix, dist coeff from RealsenseD435 factory settings
    #read in from /camera/color/camera_info ROS topic
    camera_matrix = np.array([
        [612.920654296875, 0.0, 316.9841003417969],
        [0.0, 612.995361328125, 245.92503356933594],
        [0.0, 0.0, 1.0],
    ])

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

    if use_guide_mode:
        ChAruco_instance.run_guide_robot()
    else:
        q_positions_df = pandas.read_csv(q_positions_txt)
        assert len(q_positions_df.columns) == 7, 'Joint positions txt should have 7 joints'
        q_positions_arr = np.array(q_positions_df)
        assert desired_num_points_for_calib < q_positions_arr.shape[0], (
               'Number of points for calibration should be less than number of sampled joints')

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
        # Go to some position and then record stuff
        if use_guide_mode:
            try:
                print(f"this is {i} index")
                cv2.imshow("image", img)
                cv2.waitKey(0) #<------ stopper until moving robot to desired position
            except KeybboardInterrupt:
                # Stop skill (guide mode) when doing Ctrl-C
                ChAruco_instance.fa.stop_skill()
        else:
            cv2.destroyAllWindows()
            q_i = q_positions_arr[i]
            ChAruco_instance.goto_joints(q_i)

        # get camera pose from camera
        gotpose, rot, trans = ChAruco_instance.get_offset(camera_matrix, dist_coeff, debug=True)
        if trans.ndim > 1:
            assert trans.squeeze().ndim == 1, 'Should have one dimension for (x, y, z)'
            trans = trans.squeeze()

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
        franka_pose = ChAruco_instance.fa.get_pose()
        assert np.all(np.abs(franka_pose.translation - ee_trans) < 1e-4), (
            "Incorrect position between frankapy pose and published pose.")
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
    if run_guide_mode:
        ChAruco_instance.fa.stop_skill()
    print(" ----------- end of script ----------- ")
