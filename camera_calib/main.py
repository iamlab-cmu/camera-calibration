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
 
import cv2
import csv
import click
import numpy as np
import pandas
import rospy
import tf
import time

import camera_calib.ChAruco as ChAruco
from camera_calib.ChAruco import Camera
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray


# filenames to store saved csv files
f_cam = open('./data/cam_transform.csv', 'w')
f_ee = open('./data/ee_transform.csv', 'w')

def log_msg(msg):
    print(msg)
    rospy.loginfo(msg)
    rospy.logdebug(msg)

# q_positions_txt = './data/camera_in_hand_back_joint_samples.txt'
# q_positions_txt = './data/static_camera_top_mount.txt'

@click.command()
@click.option('--guide-mode/--no-guide-mode', default=False, help='Use guide mode to move the robot.')
@click.option('--joint-positions-txt', default='./data/camera_in_hand_back_joint_samples.txt', 
              help='Path to load joints configurations to move robot.')
@click.option('--desired-num-points-for-calib', default=5,
              help='Total number of images to sample for calibration (default: 5')
@click.option('--camera-topic', default='/rgb/image_raw',
              help='Total number of images to sample for calibration (default: 5')
@click.option('--camera-info-topic', default='/rgb/camera_info', help='Camera info topic name')
def move_robot_and_calibrate_camera(
    guide_mode: bool,
    joint_positions_txt: str,
    desired_num_points_for_calib: int,
    camera_topic: str,
    camera_info_topic: str):
    print(" ----------- start  of script ----------- ")

    #wait for image callback thread before proceeding
    ChAruco_instance = ChAruco.ChAruco(camera_img_topic=camera_topic,
                                       camera_info_topic=camera_info_topic)

    while ChAruco_instance.recevied_image == False:
        log_msg("Sleeping")
        time.sleep(1)

    #display received image for debugging
    img = ChAruco_instance.get_image()
    img_size = ChAruco_instance.get_image_shape
    cv2.imshow("image", img) 
    cv2.waitKey(0)
    log_msg("here")

    #call guidance mode to Franka robot
    if guide_mode:
        ChAruco_instance.run_guide_robot()
    else:
        q_positions_df = pandas.read_csv(joint_positions_txt)
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
    # TODO: Save the image as well.
    multi_image_board_info_dict = {}

    #For desired number of calib pts, move robot and collect [cam pose, EE pose]
    for i in range (desired_num_points_for_calib):
        # Go to some position and then record stuff
        if guide_mode:
            print(f"this is {i} index")
            cv2.imshow("image", img)
            cv2.waitKey(0) #<------ stopper until moving robot to desired position
        else:
            cv2.destroyAllWindows()
            q_i = q_positions_arr[i]
            ChAruco_instance.goto_joints(q_i)
        
        # get camera pose from camera
        gotpose, rot_trans_info = ChAruco_instance.get_offset(debug=True)
        if not gotpose:
            breakpoint()
            continue
        
        rot, trans, info = rot_trans_info
        multi_image_board_info_dict[i] = info

        if trans.ndim > 1:
            assert trans.squeeze().ndim == 1, 'Should have one dimension for (x, y, z)'
            trans = trans.squeeze()

        # print(f"pose, rot,trans {gotpose,rot,trans}")
        R_np,_ = cv2.Rodrigues(rot)
        quat = Rotation.from_matrix(R_np).as_quat()

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

    print(" ----------- end of script ----------- ")


if __name__ == '__main__':
    move_robot_and_calibrate_camera()
