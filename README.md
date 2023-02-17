#  Altınay Air

## Air Robot Bringup
To bring up the robot, you can use these command lines:
### Real Hardware
```
roslaunch air_moveit_config air_bringup.launch controller:=ros_control
```
### Fake Hardware 
```
roslaunch air_moveit_config air_bringup.launch
```
Add to command line `bagfile:=true` for working with rosbag(camera).

## Air Robot Launch Files
If you want to run launch files in different terminals use these command lines:

### Air Arm - MoveIt - Hardware Interface
This launch file runs MoveIt and hardware interface.
```
roslaunch air_moveit_config demo_ros_control.launch controller:=ros_control
```
### Realsense Camera
This launch files runs Realsense Camera and nodes.
```
roslaunch realsense2_camera rs_camera.launch
```
### Object Detection 
This launch files runs YoloV7 - object detection node.
```
roslaunch yolov7_ros yolov7.launch
```
### Add Collision Object
This executables generates XYZ values of detected object and adds in planning scene.

This executable gets XY coordinates of detected object from `/yolov7/yolov7` topic and generates XYZ value of that object according to camera. These values are published to `/coordinates_of_object` topic.
```
rosrun air_moveit_config detect_and_add
```
This executable gets the coordinates of object from `/coordinates_of_object` topic and adds to planning scene.
```
rosrun air_moveit_config add_sphere
```

## MoveIt Task Constructor
```
roslaunch air_moveit_config air_mtc.launch
```
