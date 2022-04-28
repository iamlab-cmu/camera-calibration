import argparse
import numpy as np
import os
import traceback
import rospy
import open3d as o3d

from autolab_core import RigidTransform, YamlConfig, CameraChessboardRegistration
from perception_utils.kinect import KinectSensorBridged
from frankapy import FrankaArm
from utils import make_o3d_pose


if __name__ == '__main__': 
    # parse args
    parser = argparse.ArgumentParser(description='Register a camera to a robot')
    parser.add_argument('--cfg', type=str, default='cfg/register_azure_kinect_with_franka.yaml', help='filename of a YAML cfguration for registration')
    args = parser.parse_args()
    cfg = YamlConfig(args.cfg)

    fa = FrankaArm()
    
    print('Applying 0 force torque control for {}s'.format(20))
    fa.run_guide_mode(20)

    T_ee_world = fa.get_pose()

    # Get T_cb_world by using T_ee_world*T_cb_ee
    T_cb_ee = RigidTransform(rotation=np.array([
                                        [0, 0, 1],
                                        [1, 0, 0],
                                        [0, 1, 0]
                                    ]),
                            # rotation=np.array([
                            #         [0, 0, 1],
                            #         [-1, 0, 0],
                            #         [0, -1, 0]
                            #     ]),
                            # rotation=np.array([
                            #     [1, 0, 0],
                            #     [0, 1, 0],
                            #     [0, 0, 1]
                            # ]),
                            translation=np.array([0.02275, 0, -0.0732]),
                            from_frame='cb', to_frame='franka_tool'
                        )

    T_cb_world = T_ee_world * T_cb_ee

    # get camera sensor object
    for sensor_frame, sensor_cfg in cfg['sensors'].items():
        rospy.loginfo('Registering %s' %(sensor_frame))
        
        # open sensor
        try:
            sensor_cfg['frame'] = sensor_frame
            rospy.loginfo('Creating sensor')
            sensor = KinectSensorBridged(**sensor_cfg['kwargs'])
            rospy.loginfo('Starting sensor')
            sensor.start()
            rospy.loginfo('Sensor initialized')

            # register
            reg_result = CameraChessboardRegistration.register(sensor, cfg['registration'])
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

        T_camera_world = T_cb_world * reg_result.T_camera_cb

        color_im, depth_im, _ = sensor.frames()
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(color_im.data), 
                o3d.geometry.Image(depth_im.data),
                depth_scale=1,
                depth_trunc=1,
                convert_rgb_to_intensity=False
            )
        intr = sensor.ir_intrinsics
        o3d_intr = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_intr)
        pcd.transform(T_camera_world.matrix)

        o3d.visualization.draw_geometries([
            pcd,
            make_o3d_pose(RigidTransform(), size=0.2),
            make_o3d_pose(T_camera_world),
            make_o3d_pose(T_cb_world),
            make_o3d_pose(T_ee_world),
        ])

        # save tranformation arrays based on setup
        output_dir = os.path.join(cfg['calib_dir'], sensor_frame)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        rospy.loginfo('Saving to {}'.format(output_dir))
        pose_filename = os.path.join(output_dir, '%s_to_world.tf' %(sensor_frame))
        T_camera_world.save(pose_filename)
        f = os.path.join(output_dir, 'corners_cb_%s.npy' %(sensor_frame))
        np.save(f, reg_result.cb_points_cam.data)
                
        sensor.stop()
