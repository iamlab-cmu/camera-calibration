![Screenshot](handeye_calibration.png)


## Overview
Takes set of [RGB image, End-effector pose] from realsense D435 camera mounted on Franka Robot Arm, to solve the Ax = xB calibration problem using the ChAurco board where x = extrinsic calibration.

## Install
- python3 and compatiable opencv version to run ChAruco functions. Best found by running it and resolving the missing libaries.
```
$ pip install opencv-contrib-python

```
- VISP ros node package to solve Ax=Bx given pair of [cam,ee pose]. Refer to http://wiki.ros.org/visp_hand2eye_calibration
```
$ sudo-apt get install ros-$ROS_DISTRO-visp-hand2eye-calibration

```
- realsense SDK package. Refer to https://github.com/IntelRealSense/realsense-ros. If installed correctly, should be able to roslaunch realsense2 file
```
$ roslaunch realsense2_camera rs_aligned_depth.launch

```

## Set up
- Place a ChAruco board in FOV of Franka Robot arm with realsense. ChAruco board can be generated from https://calib.io/pages/camera-calibration-pattern-generator.
- Franka robot is turned on and the Frankapy ROS API interface is up and running by calling below command. Franka joint /tf topics should be published and visible on RViz. 
 ```
$ cd Prog/frankapy
$ bash bash_scripts/start_control_pc.sh -u student -i iam-[insert robot name here]

```

## Running
- Modify the main.py file to adjust the path to where you want the generated cam_transform.csv and ee_transform.csv files.  
- Modify the main.py file to adjust the intrinsic camera calibration info. You can do this by rosecho calling the topic /camera/color/camera_info.
- Modify the ChAruco.py file init function to adjust the CharucoBoard_create() parameters with the ChAruco board dimensions you printed.  
- Make sure Franka is powered up and Frankpy ROS API is running, as well as the realsense ROS node (verify by $rostopic list) 

```
$ python main.py

```
- This code will put Franka into Guide Mode. Manually hand move the robot to 15 different positions, press enter in between each position to store data.
- Given the generated two .csv files, modify publisher.py file if needed to see the path to the two .csv files are correct and can be loaded.
- Run the VISP server that will print out camera extrinsic info upon a rosservice call to publish the data. The sequence is as follows in separate terminals:

### Options to configure

Using guide mode to move the robot can sometimes be painful. Since the robot may not be fully static when it captures the image in a guide mode and hence a user needs to stabilize
it. This is not ideal. As an alternative, we provide an option to not use guide mode and use pre-defined poses saved here `./data/`. These joint configurations are loaded and iterated
over and images are recorded at each configuration. 
To enable this set the `use_guide_mode` flag in `main.py` to False [link](https://github.com/iamlab-cmu/camera-calibration/blob/multiview_handeye/main.py#L46)

How many points to sample for calibration? This option can be configured by changing the `desired_num_points_for_calib` parameter in main.py (see [here](https://github.com/iamlab-cmu/camera-calibration/blob/multiview_handeye/main.py#L36)).

How to sample points for calibration? Using half-ellipsoids that densely cover different configurations is the preferred approach.

## Getting the extrinsics

Once we have recorded the marker locations from different sampled images (either using guide mode or pre-specified joint configurations) we need to run a separate server to calculate 
our result. To do this follow the steps below:

- Given the generated two .csv files, modify publisher.py file if needed to see the path to the two .csv files are correct and can be loaded.
- Run the VISP server that will print out camera extrinsic info upon a rosservice call to publish the data. The sequence is as follows in separate terminals:

```
$ rosrun visp_hand2eye_calibration visp_hand2eye_calibration_calibrator
$ python publisher.py
$ rosservice call compute_effector_camera

```
The calibrator node will subscribe to the published [cam,ee pose pairs], then upon a service call, print out the extrinsic calibration matrix.
