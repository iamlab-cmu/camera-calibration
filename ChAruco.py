 ###################################################################
 #
 # This helper class is for generating sets of [cam pose, end-effector pose]
 # for robot hand-eye calibration using the Franka Panda Robot.
 # ChAruco generated using the following link:
 # https://calib.io/pages/camera-calibration-pattern-generator
 #
 # Output : 
 # 
 # Input  : /
 #          /
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0
 ###################################################################

import numpy as np
import random
import cv2

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from frankapy import FrankaArm
from frankapy import FrankaConstants as FC


class ChAruco:
    def __init__(self):
 
        #init franka
        print("connecting to franka-interface")
        self.fa = FrankaArm()  
 
        print('will reset joints')
        self.fa.reset_joints()

        self.bridge = CvBridge()
        self.image_cv = None
        self.recevied_image = False

        #self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        #(num row, num col, size of checker, size of aruco tag, aruco definition)
        #self.board = cv2.aruco.CharucoBoard_create(7, 5, 0.03, 0.023, self.aruco_dict)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        # self.board = cv2.aruco.CharucoBoard_create(6, 9, 0.022, 0.017, self.aruco_dict)
        self.board = cv2.aruco.CharucoBoard_create(9, 6, 0.022, 0.017, self.aruco_dict)
        sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.get_pose_rb0)


    def get_pose_rb0(self,image_in):
        # rospy.loginfo('Got image')
        # rospy.logdebug('Got image')
        self.image_ros = image_in

        # Try to convert the ROS Image message to a CV2 Image
        try:
            self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            rospy.loginfo("CvBridge Error")
            rospy.logdebug("CvBridge Error")

        # self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        self.recevied_image = True

    def get_image(self):
        return self.image_cv

    def get_image_shape(self):
        return self.image_cv.shape


    def read_chessboards(self,frames):
        """
        Charuco base pose estimation.
        """
        all_corners = []
        all_ids = []

        for frame in frames:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict)

            if len(corners) > 0:
                ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
                # ret is the number of detected corners
                if ret > 0:
                    all_corners.append(c_corners)
                    all_ids.append(c_ids)
            else:
                print('Failed!')

        imsize = gray.shape
        return all_corners, all_ids, imsize

    def get_intrinsic_distort_matrix(self, all_corners, all_ids, imsize):

        all_corners = [x for x in all_corners if len(x) >= 4]
        all_ids = [x for x in all_ids if len(x) >= 4]

        ret, camera_matrix, dist_coeff, rvec, tvec = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, self.board, imsize, None, None
        )

        return camera_matrix, dist_coeff

    def get_offset(self, camera_matrix, dist_coeff, debug: bool = False):
        frame = self.get_image()
        corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if corners is None or ids is None:
            return None
        if len(corners) != len(ids) or len(corners) == 0:
            return None


        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners,ids, frame, self.board)
        if debug:
            image_copy = cv2.aruco.drawDetectedCornersCharuco(frame, c_corners, c_ids)

        gotpose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, self.board, 
                                                              camera_matrix, dist_coeff, np.empty(1), np.empty(1))
        if gotpose:
            dist_coeff_copy = np.array(dist_coeff, dtype=np.float32).reshape(-1, 1)
            image_disp = cv2.drawFrameAxes(image_copy, camera_matrix, dist_coeff_copy, rvec, tvec, 0.1)
            cv2.imshow("Image", image_disp)
            cv2.waitKey(0)

        return gotpose, rvec, tvec
    
    def run_guide_robot(self):
        guide_duration =  300
        self.fa.run_guide_mode(guide_duration, block=False) 

    def goto_joints(self, joints: np.ndarray):
        self.fa.goto_joints(joints, joint_impedances=FC.DEFAULT_JOINT_IMPEDANCES, ignore_virtual_walls=True)
