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
 
 #
 # Versions :
 # v1.0
 ###################################################################
 
import cv2
import click
import numpy as np
import pandas
import rospy
import tf2_ros
import time

import camera_calib.ChAruco as ChAruco
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray


def log_msg(msg):
    print(msg)
    rospy.loginfo(msg)
    rospy.logdebug(msg)


def get_user_input_to_validate_pose():
    '''Show user the image and ask if the pose is valid.

    Returns:
        Dictionary with keys: valid (bool), retake (bool). Valid is True if the pose is valid,
        retake is True if the user wants to retake the pose.
    '''
    val = input('Is valid pose? Press r to retake aruco pose, s to skip, c to continue:')
    val = val.strip()
    valid_options = ('r', 's', 'c')
    while val not in valid_options:
        val = input('Is valid pose? Press r to retake aruco pose, s to skip, c to continue:')
        val = val.strip()
    
    return {
        'valid': val == 'c',
        'retake': val == 'r',
    }


@click.command()
@click.option('--guide-mode/--no-guide-mode', default=False, help='Use guide mode to move the robot.')
@click.option('--joint-positions-txt', default='./data/camera_in_hand_back_joint_samples.txt', 
              help='Path to load joints configurations to move robot.')
@click.option('--desired-num-points-for-calib', default=5,
              help='Total number of images to sample for calibration (default: 5)')
@click.option('--camera-topic', default='/rgb/image_raw', help='Camera topic name')
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

    # ChAruco_instance.goto_joints(
    #     np.array([-1.4067,-1.11069,1.66861,-2.37646,1.63436,3.02177,0.209308]),
    # )

    while ChAruco_instance.recevied_image == False:
        log_msg("Sleeping")
        time.sleep(1)

    #display received image for debugging
    # log_msg("Will show image")
    img = ChAruco_instance.get_image()
    img_size = ChAruco_instance.get_image_shape()
    print(f"Image size: {img_size}")
    cv2.imshow("image", img) 
    cv2.waitKey(0)
    log_msg("here")

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    #call guidance mode to Franka robot
    if guide_mode:
        ChAruco_instance.run_guide_robot()
    else:
        q_positions_df = pandas.read_csv(joint_positions_txt)
        assert len(q_positions_df.columns) == 7, 'Joint positions txt should have 7 joints'
        q_positions_arr = np.array(q_positions_df)
        assert desired_num_points_for_calib < q_positions_arr.shape[0], (
               'Number of points for calibration should be less than number of sampled joints')

    # Create  the tracking frame transform
    tracking_base_frame = 'rgb_camera_link'
    tracking_marker_frame = 'aruco_marker_base'
    tracking_transform_msg_stmpd = TransformStamped(
        header=Header(frame_id=tracking_base_frame),
        child_frame_id=tracking_marker_frame,
        transform=Transform(translation=Vector3(), rotation=Quaternion(x=0, y=0, z=0, w=1)))
    tracking_transform_msg_stmpd.header.stamp = rospy.Time.now()
    tf_broadcaster.sendTransform(tracking_transform_msg_stmpd)

    #For desired number of calib pts, move robot and collect [cam pose, EE pose]
    for i in range (desired_num_points_for_calib):
        # Go to some position and then record stuff
        if guide_mode:
            print(f"this is {i} index")
            cv2.imshow("image", img)
            cv2.waitKey(0) #<------ stopper until moving robot to desired position
        else:
            cv2.destroyAllWindows()
            print(f"Will go to joint: {i}")
            q_i = q_positions_arr[i]
            ChAruco_instance.goto_joints(q_i)
        
        # get camera pose from camera
        user_inp = None
        num_tries = 0
        while True:
            cv2.destroyAllWindows()
            gotpose, rot_trans_info = ChAruco_instance.get_offset(debug=True)
            if not gotpose:
                if num_tries % 50 == 0:
                    print(f"Did not get pose, will try again: {num_tries}")
                if num_tries > 200:
                    break
                num_tries += 1
                continue
            else:
                # write code that takes user input to determine if the pose is good or not
                # if not, then continue
                user_inp = get_user_input_to_validate_pose()
                if user_inp['retake']:
                    continue
                elif user_inp['valid']:
                    # Wait for the user to take sample in GUI
                    # TODO: This step should be automated

                    frequency = 100
                    rate = rospy.Rate(frequency)

                    while True:
                        val = input("Please press take sample from GUI and then press q to quit: ")
                        val = val.strip()
                        if val == 'q':
                            break
                        start_time = rospy.Time.now()
                        print(f"Will start publish transform, time: {start_time.to_sec():.6f}")
                        while (rospy.Time.now() - start_time) < rospy.Duration(5.0):
                            # Get the image from the camera        
                            rot, trans, info = rot_trans_info
                            if trans.ndim > 1:
                                assert trans.squeeze().ndim == 1, 'Should have one dimension for (x, y, z)'
                                trans = trans.squeeze()

                            # print(f"pose, rot,trans {gotpose,rot,trans}")
                            R_np, _ = cv2.Rodrigues(rot)
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

                            # publish tracking transform
                            tracking_transform_msg_stmpd.header.stamp = rospy.Time.now()
                            tracking_transform_msg_stmpd.transform = cam_transform

                            tf_broadcaster.sendTransform(tracking_transform_msg_stmpd)

                            rate.sleep()
                        print(f"Did end publish transform: {rospy.Time.now().to_sec():.6f}")

                    break
                        
                else:
                    break
        
    print(" ----------- end of script ----------- ")


if __name__ == '__main__':
    move_robot_and_calibrate_camera()
