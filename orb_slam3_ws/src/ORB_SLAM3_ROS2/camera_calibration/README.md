# Camera Calibration

## Introduction
If while running ORB_SLAM3 you are having issues with the camera's pose graph
not making sense you can or issues capturing keyframes, it's probably a good
idea to calibrate your camera. To do this, you can use the simple python script
I've provided called calibrate_D435i.py. This script is specifically for the
RealSense D435i camera, however you can easily modify it to receive a different
input image stream (such as a webcam).

## Instructions for Calibrating the Camera
1. Print out the checkerboard.pdf file. This file is originally from here, and is
a 7x10 checkerboard pattern. You can also [create your own pattern](https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html) if you wish.
2. Set up your virtual environment with the following commands:
```bash
python -m venv env
source env/bin/activate
pip install -r requirements.txt
```
3. Run the calibrate_D435i.py script like so:
```bash
python calibrate_D435i.py
```
4. Move the camera around the checkerboard pattern in various orientations. You
should start to see the values in the camera's intrinsic matrix and distortion
parameter matrix start to converge. Once the values have become relatively stable,
you can end the program by pressing ctrl+c.
5. Take the values for the camera's intrinsic matrix and enter them into the
proper ORB_SLAM3 config file. Since this repo only supports monocular and
imu-monocular SLAM, your two choices are
```
ORB_SLAM3_ROS2/config/Monocular-Inertial/RealSenseD435i.yaml
ORB_SLAM3_ROS2/config/Monocular/RealSenseD435i.yaml
```
The intrinsic matrix you get out of opencv looks like this:
```
[[fx, 0, cx],
 [0, fy, cy],
 [0, 0, 1]]
```
So enter the values into the config file accordingly:
```
# RealSense_D435i.yaml
Camera1.fx: 614.67170934
Camera1.fy: 617.63448453
Camera1.cx: 326.22132726
Camera1.cy: 245.58281905
```

## IMU Calibration
Personally, I went with the default values from the original ORB_SLAM3 repo for
my IMU calibration parameters:
```
# IMU noise
IMU.NoiseGyro: 2.44e-4 #1e-3 # rad/s^0.5
IMU.NoiseAcc: 1.47e-3 #1e-2 # m/s^1.5
IMU.GyroWalk: 1e-4 # rad/s^1.5
IMU.AccWalk: 1e-3 # m/s^2.5
IMU.Frequency: 200.0
```
While this is fine, you might also want to update/redo the IMU calibration that
lives on the device itself. This is a fast and easy process thanks to a python
script provided by intel in the librealsense:
```
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/tools/rs-imu-calibration
python rs-imu-calibration.py
```
from there you can follow the instructions, and at the end you should choose the
option to write the calibration to your device.
