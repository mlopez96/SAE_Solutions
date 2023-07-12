# Wall Following
This is a repository for Wall Following implementing a steering control to keep the vehicle parallel to a curving wall and a certain distance away from it
 - Right Wall Controller
 - Center Wall Controller


https://github.com/mlopez96/SAE_Solutions/assets/26072511/d529906c-620a-443b-8aae-cfdc660f7d08


   
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
Note: You might see some warnings when the building.  This install is not for a beginner, feel free to message me with any questions  
Remember warnings can be ignored for the ROS_Bridge
# Instructions
In the first terminal source the ros_source.sh script and launch the simulator with the track_barca world:  
```
$ source ~/ros_source.sh
$ roslaunch racecar_gazebo racecar.launch world_name:=track_barca
```
In another terminal start the ros1_bridge by following these steps:  
```
$ source ~/bridge_source.sh
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

In the third terminal start the wall_following package:  
```
$ source ~/ros2_source.sh
$ ros2 launch wall_following wall_following.launch.py
```
