# Automatic_Emergency_Braking
This repository contains a ROS package that can be used to implement a P-controller to stop the vehicle before it collides with a wall.  
The two controllers are:  
- The Euclidean distance between vehicle and wall (distance error)
- The Time-to-collision TTC between vehicle and wall (velocity error)   



https://github.com/mlopez96/SAE_Solutions/assets/26072511/d2cfcbcd-406c-4e9f-9c6a-db7bcfcfa38c



 


https://github.com/mlopez96/SAE_Solutions/assets/26072511/c0c69c7d-ec05-425c-a132-69c4067cadae




# Prerequisites
Running Ubuntu 20.04    
Installed [Gazebo 11](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)  
Installed [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)  
Install [Ackermann messages](http://wiki.ros.org/ackermann_msgs)  
Installed [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)    
Installed ros-noetic dependancies:  
```
$ sudo apt install ros-noetic-ros-control ros-noetic-gazebo-ros-control
ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-pid ros-noeticdriver-base
```
Installed [ROS1_bridge](https://github.com/ros2/ros1_bridge)  
Installed [F1-tenth Simulator](https://github.com/SAE-Robotics-Bootcamp/f110-simulator-public)  
Installed OpenCV  
```
$ sudo apt install libopencv-dev python3-opencv  
```
Installed Qt4  
```
$ sudo add-apt-repository ppa:gezakovacs/ppa
$ sudo apt update
$ sudo apt install qt4-default
```
Note: You might see some warnings when the building.  This install is not for a beginner, feel free to message me with any questions  
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
$ roslaunch racecar_gazebo racecar.launch world_name:=racecar_wall
```
In the second terminal:
```
$ source ~/bridge_source.sh
$ ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
In the third terminal:
```
$ source ~/ros2_source.sh
$ ros2 launch automatic_emergency_braking aeb.launch.py
```

To switch to the Distance or Time controllers edit the **sae_aeb.py** (add/remove the '#' on self.dist_control or self.TTC_control)
```
def laser_callback(self, data):
    dist_wall = self.get_distance(data)     #Get the average distance for the front angle
    #self.dist_control(dist_wall)           #Call the distance based controller and publish the speed
    self.TTC_control(dist_wall)             #Call the TTC based controller and publish the speed
```
