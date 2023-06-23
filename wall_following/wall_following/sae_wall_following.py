#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollowing(Node):
    def __init__(self):
        super().__init__("wall_following")
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.callback, 10)
        self.timer_period = 0.001 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.control) 

        # Vehicle parameters
        self.ANGLE_RANGE = 270           # Hokuyo 10LX has 270 degree scan.
        self.ANGLE_INCREMENT = 0.25        # # Hokuyo 10LX has 0.25 degree angle increment.
        self.VELOCITY = 1             # Maximum Velocity of the vehicle
        self.DISTANCE_THRESHOLD = 0.8	 # (m)
        self.number_previous_errors = 10       # I controller looks at the past error data. This variable sets how far back the controller goes
        self.previous_error = np.zeros((1, self.number_previous_errors))  #store previous errors for the I and D controllers 
        self.previous_error_index = -1  #To index the previous_error array
        self.current_error_index = 0    #To index the previous_error array
        self.previous_distance_left = 0.0  #To store the last distance of the left wall
        self.previous_lookahead_left = 0.0 #To store lookahead distance
        self.previous_distance_right = 0.0  #To store the last distance of the right wall
        self.previous_lookahead_right = 0.0  #To store the last lookahead distance 
        self.Laser_angle = 3 #The controller looks at the range of angles to calculate average distance
        
        self.following_center = 1 #This bit will let the controller know to follow the center line
        self.following_right = 0  #This bit will let the controller know to follow the right wall
        
        # PID-Controller Parameters
        
        if self.following_center == 1:
            self.kp_steer = 1.5
            self.kd_steer = 0.0
            self.ki_steer = 0
        
        if self.following_right == 1:
            self.kp_steer = 3.8
            self.kd_steer = 0
            self.ki_steer = 0
            
        self.kp_vel = 0.13
        self.kd_vel = 0.0
        self.ki_vel = 0.0

        # Container for scan data
        self.data = None
    
    def get_index(self, angle):
        
        Minimum_Angle = self.ANGLE_RANGE/2   #Angle of 270 has a range of [-135:135]
        index = (angle+Minimum_Angle)/self.ANGLE_INCREMENT + 1        #get the index of the 0 angle(Arithmatic Prograssion)
        
        return index
        
    def get_distance(self, index):
        
        data = self.data
        index_range = self.Laser_angle*4  #Looks at 3 degree range
        # Loop through the ranges and find the avg range distance
        index_lower = index - index_range/2     #Minimum angle for the distance filtering
        index_upper = index + index_range/2     #Maximum angle for the distance filtering
        avg_dist = 0.0
        Sum_Valid_index = 0
        for i in range(int(index_lower), int(index_upper), 1):   #Loop over the Angle Range
            try:
                if math.isinf(data.ranges[i]) == False:
                    avg_dist += data.ranges[i]                        #Add the distance to avg_dist for each angle increment in the range
                    Sum_Valid_index += 1                       #Add the index with the valid distance
            except AttributeError:
                return 0.0
        if Sum_Valid_index == 0:
            return 0.0
        else: 
            avg_dist = avg_dist/Sum_Valid_index                #Get the average distance by dividing the sum by number of data points
            return avg_dist
    
    def follow_right(self, angle_right, angle_lookahead):
        
        # Fill in with logic to calculate right wall error
        # Get the corresponding list of indices for given range of angles
        index_right = int(self.get_index(angle_right))
        index_lookahead_right = int(self.get_index(angle_lookahead))
        
        dist_right = self.get_distance(index_right)          #Get the distance from the wall perpendicular from vehicle orientation (b)
        if dist_right == 0:
            dist_right = self.previous_distance_right
        self.previous_distance_right = dist_right
        
        dist_lookahead_right = self.get_distance(index_lookahead_right)  #Get the lookahead distance (a)
        if dist_lookahead_right == 0:
            dist_lookahead_right = self.previous_lookahead_right
        self.previous_lookahead_right = dist_lookahead_right
        
        theta = abs(angle_right - angle_lookahead)*math.pi/180
        
        alpha = math.atan2(dist_lookahead_right*math.cos(theta) - dist_right, dist_lookahead_right*math.sin(theta))
        
        dist_actual = dist_right*math.cos(alpha)
        
        error_r = self.DISTANCE_THRESHOLD - dist_actual
        self.get_logger().info("Distance From the Right Wall: {:.3f}".format(dist_actual))
        
        self.previous_error[0][self.current_error_index] = error_r
        if self.following_right == 1:
            self.previous_error_index = self.current_error_index
            if self.current_error_index == (self.number_previous_errors - 1):
                self.current_error_index = 0
            else:
                self.current_error_index += 1
        return error_r

    def follow_center(self, angle_right, angle_lookahead_right):
    
        # Fill in logic to calculate centerline error
        index_right = int(self.get_index(angle_right))
        index_lookahead_right = int(self.get_index(angle_lookahead_right))
        
        dist_right = self.get_distance(index_right)          #Get the distance from the wall perpendicular from vehicle orientation (b)
        if dist_right == 0:
            dist_right = self.previous_distance_right
        self.previous_distance_right = dist_right
        
        dist_lookahead_right = self.get_distance(index_lookahead_right)  #Get the lookahead distance (a)
        if dist_lookahead_right == 0:
            dist_lookahead_right = self.previous_lookahead_right
        self.previous_lookahead_right = dist_lookahead_right
        
        theta = abs(angle_right - angle_lookahead_right)*math.pi/180
        
        alpha = math.atan2(dist_lookahead_right*math.cos(theta) - dist_right, dist_lookahead_right*math.sin(theta))
        
        dist_to_right_wall = dist_right*math.cos(alpha)
        
        self.get_logger().info("Distance From the Right Wall: {:.3f}".format(dist_to_right_wall))
        
        angle_left = -1*angle_right
        angle_lookahead_left = -1*angle_lookahead_right
        
        index_left = int(self.get_index(angle_left))
        index_lookahead_left = int(self.get_index(angle_lookahead_left))
        
        dist_left = self.get_distance(index_left)          #Get the distance from the wall perpendicular from vehicle orientation (b)
        if dist_left == 0:
            dist_left = self.previous_distance_left
        self.previous_distance_left = dist_left
        
        dist_lookahead_left = self.get_distance(index_lookahead_left)  #Get the lookahead distance (a)
        if dist_lookahead_left == 0:
            dist_lookahead_left = self.previous_lookahead_left
        self.previous_lookahead_left = dist_lookahead_left
        
        theta = abs(angle_left - angle_lookahead_left)*math.pi/180
        
        alpha = math.atan2(dist_lookahead_left*math.cos(theta) - dist_left, dist_lookahead_left*math.sin(theta))
        
        dist_to_left_wall = dist_left*math.cos(alpha)
        
        self.get_logger().info("Distance From the Left Wall: {:.3f}".format(dist_to_left_wall))
        
        error_r = dist_to_left_wall - dist_to_right_wall
        
        self.previous_error[0][self.current_error_index] = error_r

        centerline_error = dist_right - dist_left     # Calculate the center line error(For center line dist_right - dist_left = 0

        self.get_logger().info("Center line error: {:.3f}".format(centerline_error))
        
        self.previous_error[0][self.current_error_index] = centerline_error
        
        self.previous_error_index = self.current_error_index
        if self.current_error_index == (self.number_previous_errors - 1):
            self.current_error_index = 0
        else:
            self.current_error_index += 1
        return centerline_error
        
    def control(self):

        # Pick two rays at two angles
        if self.following_right == 1:
            angle_right = -90
            angle_lookahead = -15
        
        if self.following_center == 1:
            angle_right = 90
            angle_lookahead = 45
        
        # Right wall following 
        if self.following_right == 1:
            P_parameter = self.follow_right(angle_right, angle_lookahead)  #P parameter will be the current error
            I_parameter = np.sum(self.previous_error)                      #I parameter will be the sum of all previous error
            #D parameter will be the rate of change of the error
                
            D_parameter = self.previous_error[0][self.current_error_index] - self.previous_error[0][self.previous_error_index] 
                
        # Center Line following
        if self.following_center == 1: 
            P_parameter = self.follow_center(angle_right, angle_lookahead)  #P parameter will be the current error
            I_parameter = np.sum(self.previous_error)                      #I parameter will be the sum of all previous error
            #D parameter will be the rate of change of the error
        
            D_parameter = self.previous_error[0][self.current_error_index] - self.previous_error[0][self.previous_error_index] 
        
        steering_angle = self.kp_steer*P_parameter + self.ki_steer*I_parameter*self.timer_period + self.kd_steer*D_parameter/self.timer_period

        # Set maximum thresholds for steering angles
        if steering_angle > 0.5:
            steering_angle = 0.5
        elif steering_angle < -0.5:
            steering_angle = -0.5
        
        self.get_logger().info("Steering angle: {:.3f}".format(steering_angle))

        vel = self.VELOCITY - self.kp_vel*np.abs(P_parameter) #

        self.get_logger().info("Velocity: {:.3f}".format(vel))

        msg = AckermannDriveStamped()
        msg.drive.speed = vel
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)

    def callback(self, data):
        self.data = data

def main(args=None):
    print("Wall Following Started")

    rclpy.init(args=args)
    node = WallFollowing()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
