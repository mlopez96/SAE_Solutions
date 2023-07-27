#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

class WallFollower(Node):
    def __init__(self):
        print("Init Node")
        super().__init__("wall_follower")
        self.pub = self.create_publisher(Twist, '/wall_follower/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer_period = 0.01 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.control) 

        self.previous_distance_right = 0
        self.previous_lookahead_right = 0
        self.previous_distance_left = 0
        self.previous_lookahead_left = 0
        
        # Vehicle parameters
        self.ANGLE_RANGE = 360          
        self.ANGLE_INCREMENT = 1        
        self.VELOCITY = 0.1            # Maximum Velocity of the vehicle
        self.Laser_angle = 4 #The controller looks at the range of angles to calculate average distance
        
        
        # P-Controller Parameters
        
        self.kp_steer = 1.2
            
        self.kp_vel = 0 #0.13
        
        # Container for scan data
        self.data = None
    
    def get_index(self, angle):
        
        Minimum_Angle = 0
        index = (angle+Minimum_Angle)/self.ANGLE_INCREMENT        
        
        return index
        
    def get_distance(self, index):
        
        data = self.data
        index_range = self.Laser_angle  #Looks at 4 degree range
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
    
    def follow_center(self, angle_right, angle_lookahead_right, angle_left, angle_lookahead_left):
    
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
        #self.get_logger().info("right alpha: {:.3f}".format(alpha))
        dist_to_right_wall = dist_right*math.cos(alpha)
        
        #self.get_logger().info("Distance From the Right Wall: {:.3f}".format(dist_right))
        
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
        #self.get_logger().info("left alpha: {:.3f}".format(alpha))
        dist_to_left_wall = abs(dist_left*math.cos(alpha))
        
        #self.get_logger().info("Distance From the Left Wall: {:.3f}".format(dist_left))
        
        error_r =  dist_to_right_wall - dist_to_left_wall
        
        centerline_error = dist_right - dist_left     # Calculate the center line error(For center line dist_right - dist_left = 0

        
        return centerline_error
        
    def control(self):
        
        # Pick two rays at two angles
      
        angle_right = 90
        angle_lookahead_right = 120
        angle_left = 270
        angle_lookahead_left = 240
        
                
        # Center Line following
        P_parameter = self.follow_center(angle_right, angle_lookahead_right, angle_left, angle_lookahead_right)  #P parameter will be the current error
        
        steering_angle = -self.kp_steer*P_parameter
        
        # Set maximum thresholds for steering angles
        if steering_angle > 0.5:
            steering_angle = 0.5
        elif steering_angle < -0.5:
            steering_angle = -0.5
        

        vel = self.VELOCITY - self.kp_vel*np.abs(P_parameter) #

        #self.get_logger().info("Velocity: {:.3f}".format(vel))

        twist_object = Twist()
        twist_object.linear.x = 0.1# Set some value for velocity
        twist_object.angular.z = steering_angle
        self.pub.publish(twist_object)
        
    def laser_callback(self, data):
        self.data = data
        

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
