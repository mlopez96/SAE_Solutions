#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class AEB(Node):
    def __init__(self):
        super().__init__("aeb")
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/vesc/odom", self.speed_callback, 10)

        # Vehicle parameters
        self.ANGLE_RANGE = 270              # Hokuyo 10LX has 270 degree scan.
        self.ANGLE_INCREMENT = 0.25         # # Hokuyo 10LX has 0.25 degree angle increment.
        self.VELOCITY = 0.5                # Maximum Velocity of the vehicle
        self.TIME_THRESHOLD = 6            # Time threshold before collision (s)
        self.Start_Brake_TIME_FRONT = 8    # Time threshold where we start breaking (s)
        self.STEERING_ANGLE = 0.0          # Steering angle is uncontrolled
        self.DESIRED_DISTANCE_FRONT = 1    # Distance threshold before collision (m)
        self.Start_Brake_DISTANCE_FRONT = 3  # Distance threshold where we start breaking (m)
        self.Start_Brake_Bit = 0             #Bit showing if the vehicle is breaking or not
        self.previous_speed = 0.0            #Store the previous speed(m/s)
        self.SPEED = 0.5                   # Random initialization of vehicle callback speed.
        
        #Select the control method
        self.control_using_distance = 1
        self.control_using_ttc = 0

        # P-Controller Parameters. Change the kp values for both the controllers for tuning
        self.kp_dist = 0.25
        self.kp_ttc = 0.166

    def get_index(self, angle):
        """
        Implement the logic to find the index of the laser return in the scan data corresponding to the angle we choose.
        """
        Minimum_Angle = self.ANGLE_RANGE/2   #Angle of 270 has a range of [-135:135]
        index = (angle+Minimum_Angle)/self.ANGLE_INCREMENT + 1        #get the index of the 0 angle(Arithmatic Prograssion)
        
        return index

    def get_distance(self, data):
        """
        Implement a function to take filtered distance data corresponding to a vision cone.
        The most basic filtering algorithm we can use is a simple averaging function. Take all the distance returns
        in the vision cone and get the mean to pass as the distance to the control function
        """
        angle_front = 0  
        index_range = 40  #Number of data points to be considered for the filtering
        avg_dist = 0
        
        # Get the corresponding list of indices for given range of angles
        index_front = int(self.get_index(angle_front))
        self.get_logger().info("Front index is = {:.1f}".format(index_front))
        # Loop through the ranges and find the avg range distance
        index_lower = index_front - index_range/2     #Minimum angle for the distance filtering
        index_upper = index_front + index_range/2     #Maximum angle for the distance filtering
        
        Sum_Valid_index = 0
        for i in range(int(index_lower), int(index_upper), 1):   #Loop over the Angle Range
            if math.isinf(data.ranges[i]) == False:
                avg_dist += data.ranges[i]                        #Add the distance to avg_dist for each angle increment in the range
                Sum_Valid_index += 1                       #Add the index with the valid distance 
        avg_dist = avg_dist/Sum_Valid_index                #Get the average distance by dividing the sum by number of data points
        self.get_logger().info("Average Distance = {:.3f}".format(avg_dist))

        return avg_dist

    def TTC_control(self, distance):
        """
        Implement a P-controller based on TTC (time to collision) error
        Time to collision can be computed as:
        time = distance / speed
        where "distance" is the distance to the object and "speed" is the speed of the ego vehicle.
        Then the time error can be calculated to compute the control velocity
        """
        if self.SPEED == 0:   
            TTC = math.inf
        else:
            TTC = distance/self.SPEED
            self.get_logger().info("Time to Collision = {:.3f}".format(TTC))
            
        if TTC < self.Start_Brake_TIME_FRONT:       #Start braking if the TTC is less than Start_Brake_Time
            self.Start_Brake_Bit = 1
            
        if self.Start_Brake_Bit == 0:               #If not Braking, run the vehicle at high speed
            pub_speed = self.VELOCITY
        else:
            pub_speed = self.kp_ttc*(TTC - self.TIME_THRESHOLD)     #If Braking, control the speed by looking at the TTC
            if pub_speed > self.previous_speed:                     #This line prevents the car from accelerating while in the Brake mode
                pub_speed = self.previous_speed
        
        if pub_speed < 0.0:                               #This line prevents the car from moving back and forth
            pub_speed = 0.0
        self.get_logger().info("Speed = {:.3f}".format(pub_speed))
        
        self.previous_speed = pub_speed
        msg = AckermannDriveStamped()
        msg.drive.speed = pub_speed #Input the control velocity here
        msg.drive.steering_angle = self.STEERING_ANGLE
        self.pub.publish(msg)
    
    def dist_control(self, distance):
        """
        Implement a P-controller based on distance based error.
        """
        if distance < self.Start_Brake_DISTANCE_FRONT:       #Start braking if the TTC is less than Start_Brake_Time
            self.Start_Brake_Bit = 1
        
        if self.Start_Brake_Bit == 0:               #If not Braking, run the vehicle at high speed
            pub_speed = self.VELOCITY
        else:
            if (distance - self.DESIRED_DISTANCE_FRONT) > 0:    # Produce the output only when we are far from the desired distance
                pub_speed = self.kp_dist*(distance - self.DESIRED_DISTANCE_FRONT)
            else:
                pub_speed = 0.0
            
        msg = AckermannDriveStamped()

        msg.drive.speed = pub_speed
        msg.drive.steering_angle = self.STEERING_ANGLE
        self.get_logger().info("Speed Input = {:.3f}".format(msg.drive.speed))
        self.pub.publish(msg)


    def laser_callback(self, data):
        """
        Call the relevant controller here. The controller will keep looping according to the callback rate.
        """
        dist_wall = self.get_distance(data)      #Get the average distance for the front angle
        if self.control_using_distance == 1:
            self.dist_control(dist_wall)             #Call the distance based controller and publish the speed
        if self.control_using_ttc == 1:
            self.TTC_control(dist_wall)         ##Call the TTC based controller and publish the speed

    def speed_callback(self, data):
        self.SPEED = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2 + data.twist.twist.linear.z**2)
        return self.SPEED

def main(args=None):
    print("AEB Started")

    rclpy.init(args=args)
    node = AEB()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
