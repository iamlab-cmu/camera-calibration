# camera-calibration

## Installation

1. Clone the following repository:
```
git clone https://github.com/iamlab-cmu/perception.git
cd perception
```

2. Run the following in a virtual environment:
```
pip install -e .
```

3. Clone this repository:
```
git clone git@github.com:iamlab-cmu/camera-calibration.git
cd camera-calibration
```

4. Run the following in a virtual environment:
```
pip install -e .
```

## Running Instructions
1. Start the Azure Kinect Camera
```
roslaunch azure_kinect_ros_driver driver.launch
```
2. Open the calib/azure_kinect.intr file.
3. Rostopic echo /rgb/camera_info once.
4. Copy the first number in K: to after "_fx"
5. Copy the third number in K: to after "_cx"
6. Copy the fifth number in K: to after "_fy"
7. Copy the sixth number in K: to after "_cy"
8. If you are using the large checkerboard for an overhead camera, just run:
```
python scripts/register_camera.py
```
9. If you are using a checkerboard on the hand, first move the robot into a position so that the checkerboard can be seen from the camera at a distance of around 2 feet away.
10. Then start franka-interface and use the following command:
```
python scripts/register_camera_using_franka.py
```
11. Afterwards, check to make sure the calib/azure_kinect_overhead/azure_kinect_overhead_to_world.tf file is similar to calib/example_azure_kinect_overhead_to_world.tf.
12. If the transformations are similar, then test the calibration using a flat surface and an alphabet block and run the following command:
```
python scripts/run_pick_up_using_camera.py
```
