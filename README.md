# GL-3 (Mechanical scanning 2D LiDAR)

## Guide
- Installation
```
$ cd ${ROS workspace}/src
$ git clone --recurse-submodules https://github.com/soslab-project/gl_ros_driver.git
$ cd $(ROS workspace)
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```
- Set permission of USB port
```
$ sudo chmod a+rw /dev/ttyUSB0
```
- Set permission of USB port permanently
```
$ sudo usermod -a -G dialout $USER
```
and reboot.
- Run GL-3 publisher node
```
$ roslaunch gl_ros_driver gl_ros_driver.launch
```
- Run GL-3 publisher node with RViz
```
$ roslaunch gl_ros_driver view_gl_ros_driver.launch
```
- Change serial port or frame_id in `gl_ros_driver/launch/gl_ros_driver.launch`

## Published Topics
- _scan_ (sensor_msgs/LaserScan): it publishes scan topic from the laser.

## Test environment
- ROS Melodic Morenia, ROS Kinetic Kame
- Ubuntu 16.04, Ubuntu 18.04
- x86_64 (PC), aarch64 (Jetson series)

## Application demo
- [GL-3, Demo] 2D LiDAR, Mapping (https://youtu.be/AfsqlU_f-Go)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (https://youtu.be/_HBWe95GqXM)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (pulse-width version) (https://youtu.be/q4MeeS9eP4c)
- [GL-3, Demo] Human Tracking Demo of Multiple Mobile Robots (https://youtu.be/ZzEvm8gMPaA)
- [GL-3, Demo] 2D LiDAR Object Detection (https://youtu.be/V29QzU9AcQo)
