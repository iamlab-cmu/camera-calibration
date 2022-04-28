from frankapy import FrankaArm
import matplotlib.pyplot as plt
from mpl_point_clicker import clicker

from autolab_core import RigidTransform
from perception_utils.kinect import KinectSensorBridged
from utils import get_object_center_point_in_world


if __name__ == '__main__':
    sensor_name = 'kinect_negx'

    print('Starting robot')
    fa = FrankaArm()    

    # print('Opening Grippers')
    #Open Gripper
    # fa.open_gripper()

    #Reset Pose
    # fa.reset_pose() 
    #Reset Joints
    # fa.reset_joints()

    sensor = KinectSensorBridged(frame=sensor_name, topic_prefix=f'/{sensor_name}')
    sensor.start()
    T_sensor_world = RigidTransform.load(f'calib/{sensor_name}/{sensor_name}_to_world.tf')    

    color_im, depth_im, _ = sensor.frames()

    fig, ax = plt.subplots(constrained_layout=True)
    ax.imshow(color_im.data)
    klicker = clicker(ax, ["click"], markers=["x"])
    plt.show()

    positions = klicker.get_positions()['click'][0].astype('int')

    object_center_point_in_world = get_object_center_point_in_world(positions[0],
                                                                    positions[1],
                                                                    depth_im.data, 
                                                                    sensor.ir_intrinsics,
                                                                    T_sensor_world)

    print(fa.get_pose().translation)
    print(f'\n{__name__}\n'); import IPython; IPython.embed(); exit()
