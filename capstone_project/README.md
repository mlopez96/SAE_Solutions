# Capstone Project
This repository contains a ROS2 package that completes three missions autonomously.  Wall Following, Line following with object detection, and Go-to-goal using the Navigation Stack.  All code was written in python.

## Task 1:
Wall Following: Implement wall following on the Turtlebot3 Burger. Create a track for the robot to follow with objects you find around the house (or cardboard boxes). The track must have two walls on either side of the vehicle. Make the Turtlebot3 follow the center line between the two walls.

## Task 2:
Line following with object detection: Draw a line on the floor with a tape or strips of paper. The line should be no more than 1.5 inches wide and drawn like a sine wave with at least one 90 degree corner. Make the robot follow this line.

## Task 3:
Go-to-goal using the Navigation Stack: Implement go-to-goal and object avoidance using the Navigation Stack and the Turtlebot. This will require you to map the area, localize your robot within that area using AMCL implement the Simple Commander API node with a local and global planner of your choice. When integrating the three tasks to run the "obstacle course" you will need to set the goal position within the python script.  

## Overview:
Overall, the robot was able to successfully complete all three tasks. The robot was able to follow a wall track, follow a line, and navigate to a goal position. The robot was also able to detect and avoid obstacles.

## Testing:
It is important to test the robot in a real-world environment before deploying it to production. This is because there are a lot of factors that can affect the robot's performance, such as the lighting conditions, the surface that the robot is moving on, and the presence of obstacles. Despite these challenges, we were able to successfully complete this project. We were able to identify and fix a variety of bugs in our code. We were able to use a variety of debugging techniques, such as printing out the values of variables, adding print statements to your code, and using a debugger. 

## Videos

### Final Capstone Run
![Screenshot from 2023-08-08 17-29-32](https://github.com/mlopez96/SAE_Solutions/assets/26072511/c66c8082-dfb4-4517-9cfc-16b1e961150d)  
https://www.youtube.com/shorts/f1H5uRYWIo4

### Part 1:  
![Screenshot from 2023-08-07 20-52-41](https://github.com/mlopez96/SAE_Solutions/assets/26072511/ad8516d3-0f0b-4868-86bb-b1c06db395fa)

https://www.youtube.com/watch?v=E_hD9kfBCuk
### Part 2:
![Screenshot from 2023-08-07 20-53-16](https://github.com/mlopez96/SAE_Solutions/assets/26072511/c2b56226-f2c4-45c0-b515-895f3cb86df5)

https://www.youtube.com/watch?v=KiYYwLnoIsk

## Future Capability
Fail-safes are essential for any robotic program, as they can help to prevent accidents and damage. In the case of a turtlebot3 for ROS2, some possible fail-safes could include:

If the wall follower is no longer detected, the robot could switch to line following. This would allow the robot to continue to navigate even if the wall is no longer visible.  

If the line follower is also not detected, the robot could switch to object detection. This would allow the robot to find landmarks and navigate around obstacles.  

If object detection is not successful, the robot could enter a safe mode. This would involve stopping the robot and waiting for human intervention.  

It is important to carefully consider the possible failure modes of a robotic program and to implement fail-safes that will address these modes. By doing so, you can help to ensure the safety of the robot and the people around it.
Use a variety of sensors to increase the robustness of the program. This will help to ensure that the program can continue to function even if one sensor fails  
