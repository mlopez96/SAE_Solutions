# Capstone Project
This repository contains a ROS2 package that completes three missions autonomously.  Wall Following, Line following with object detection, and Go-to-goal using the Navigation Stack.  All code was written in python.

## Task 1:
Wall Following: Implement wall following on the Turtlebot3 Burger. Create a track for the robot to follow with objects you find around the house (or cardboard boxes). The track must have two walls on either side of the vehicle. Make the Turtlebot3 follow the center line between the two walls.

## Task 2:
Line following with object detection: Draw a line on the floor with a tape or strips of paper. The line should be no more than 1.5 inches wide and drawn like a sine wave with at least one 90 degree corner. Make the robot follow this line.

## Task 3:
Go-to-goal using the Navigation Stack: Implement go-to-goal and object avoidance using the Navigation Stack and the Turtlebot. This will require you to map the area, localize your robot within that area using AMCL implement the Simple Commander API node with a local and global planner of your choice. When integrating the three tasks to run the "obstacle course" you will need to set the goal position within the python script.
