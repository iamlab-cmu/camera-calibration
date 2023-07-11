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

from typing import Any
import numpy as np
import random
import cv2

import rospy
from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge, CvBridgeError

from autolab_core import CameraIntrinsics
from frankapy import FrankaArm
from frankapy import FrankaConstants as FC


class Camera:
    def __init__(self,
                 intr: np.ndarray,
                 D: np.ndarray,
                 distr_model: str = 'plumb_bob') -> None:
        self.camera_mat = np.copy(intr)
        self.dist_coeff = np.copy(D)
        self.dist_model = distr_model

        self.camera_intr = CameraIntrinsics(
            frame='camera',
            fx=intr[0, 0],
            fy=intr[1, 1],
            cx=intr[0, 2],
            cy=intr[1, 2],
        )
    
    def undistort_image(self, img):
        return cv2.undistort(img, self.camera_mat, self.dist_coeff)


class ChAruco:
    def __init__(self,
                 camera_img_topic: str ='/camera/color/image_raw',
                 camera_info_topic: str ='/camera/color/camera_info'):
 
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
        # self.board = cv2.aruco.CharucoBoard_create(9, 6, 0.022, 0.017, self.aruco_dict)
        self.board = cv2.aruco.CharucoBoard_create(4, 3, 0.038, 0.030, self.aruco_dict)

        self._sub_image = rospy.Subscriber(camera_img_topic, Image, self.get_pose_rb0)
        self.camera = None    
        self._sub_camera_info = rospy.Subscriber(camera_info_topic, CameraInfo, self.get_camera_info)

    def get_camera_info(self, camera_info):
        if self.camera is None:
            self.camera = Camera(np.array(camera_info.K).reshape((3, 3)),
                                 np.array(camera_info.D))
    
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

    def read_chessboards(self, frames):
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

    def get_offset(self, debug: bool = False):
        frame = self.get_image()
        camera_matrix = self.camera.camera_mat
        dist_coeff = self.camera.dist_coeff
        # dist_coeff = np.zeros_like(self.camera.dist_coeff)

        corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if corners is None or ids is None:
            print("No corners found")
            return False, None
        elif len(corners) != len(ids) or len(corners) == 0:
            print("No corners found")
            return False, None


        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners,ids, frame, self.board)
        if debug:
            image_copy = cv2.aruco.drawDetectedCornersCharuco(frame, c_corners, c_ids)

        gotpose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, self.board, 
                                                                 camera_matrix, dist_coeff, np.empty(1), np.empty(1))
        if gotpose:
            dist_coeff_copy = np.array(dist_coeff, dtype=np.float32).reshape(-1, 1)
            # image_disp = cv2.drawFrameAxes(image_copy, camera_matrix, dist_coeff_copy, rvec, tvec, 0.1)
            # For azure kinect we use rectified images already
            image_disp = cv2.drawFrameAxes(image_copy, camera_matrix, np.zeros_like(dist_coeff_copy), rvec, tvec, 0.1)
            if image_disp.shape[0] > 1000:
                image_disp = cv2.resize(image_disp, (int(image_disp.shape[1] / 2), int(image_disp.shape[0] / 2)))
            cv2.imshow("Image", image_disp)
            cv2.waitKey(0)

        info = {
            'image': frame,
            'corners': c_corners,
            'charuco_identifiers': c_ids,
            'rvec': rvec,
            'tvec': tvec,
        }

        return gotpose, (rvec, tvec, info)
    
    def calibrate_camera(self, all_corners, all_ids, imsize, camera_matrix, dist_coeffs):
        """
        Calibrate camera using list of detected corners and its identifiers from several views.
        """
        flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_ASPECT_RATIO)
        (ret, final_camera_matrix, distortion_coefficients0,
        rotation_vectors, translation_vectors,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics,
        perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
            charucoCorners=all_corners,
            charucoIds=all_ids,
            board=self.board,
            imageSize=imsize,
            cameraMatrix=camera_matrix,
            distCoeffs=dist_coeffs,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))
        breakpoint()
    
    def run_guide_robot(self):
        guide_duration =  300
        self.fa.run_guide_mode(guide_duration, block=False) 

    def goto_joints(self, joints: np.ndarray):
        self.fa.goto_joints(joints, joint_impedances=FC.DEFAULT_JOINT_IMPEDANCES, ignore_virtual_walls=True)
