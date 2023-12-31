# Advanced_Lane_Detection
This repository contains a ROS2 package that can be used to manipulate camera data to find out the geometric form of the lane lines to implement a simple lane keeping algorithm.  

Video demonstrating a vehicle completing a lap:


https://github.com/mlopez96/SAE_Solutions/assets/26072511/5c064485-6ffa-477d-bb3c-1fc4401b3e40



# Prerequisites
Running Ubuntu 20.04    
Installed [Gazebo 11](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)  
Installed [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)  
Install [Ackermann messages](http://wiki.ros.org/ackermann_msgs)  
Installed [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)    
Installed ros-noetic dependancies:  
Installed [ROS1_bridge](https://github.com/ros2/ros1_bridge)  
Installed [F1-tenth Simulator](https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public)  
Installed OpenCV  
Installed Qt4  

Remember warnings can be ignored for the ROS_Bridge  

# Instructions
Once build is complete, run three separate terminals  
In the first terminal:
```
$ source ~/ros_source.sh
$ roslaunch racecar_gazebo racecar.launch world_name:=lane_keep x_pos:=1.1 y_pos:=-2.8 angle:=0.0
```
In the second terminal:
```
$ source ~/bridge_source.sh
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
In the third terminal:
```
$ source ~/ros2_source.sh
$ ros2 launch lane_keeping lane_keep.launch.py
```
