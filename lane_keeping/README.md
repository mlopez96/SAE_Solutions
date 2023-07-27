# Advanced_Lane_Keeping
This repository contains a ROS package that can be used to implement a lane keeping algorithm.

Video demonstrating a vehicle completing a lap:
[Insert video link here]

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

# Testing ROS Noetic & ROS_Bridge before build
Should be able to control the racecar with A,S,W,D controls in the terminal
```
$ source ~/ros_source.sh
$ roslaunch race f1_tenth.launch
```
Should display ros_bridge pairs, showing AckermannMessage to indicate a successful bridge
```
$ source ~/bridge_source.sh
$ ros2 run ros1_bridge dynamic_bridge --print-pairs
```


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
