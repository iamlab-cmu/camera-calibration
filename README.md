# camera-calibration

## Prerequisites
1. Azure Kinect SDK
2. Azure Kinect ROS Driver
3. Perception

## Running Instructions
1. Start the Azure Kinect Camera
```
roslaunch azure_kinect_ros_driver driver.launch
```
2. Open the calib/azure_kinect_overhead/azure_kinect_overhead.intr file.
3. Rostopic echo /rgb/camera_info once.
4. Copy the first number in K: to after "_fx"
5. Copy the third number in K: to after "_cx"
6. Copy the fifth number in K: to after "_fy"
7. Copy the sixth number in K: to after "_cy"
8. 
