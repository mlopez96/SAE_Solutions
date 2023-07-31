#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from predictor_msgs.msg import Predictor

      
class state_machine(Node):
    def __init__(self):
        super().__init__("State_Machine")
        self.line_follower_twist_sub = self.create_subscription(Twist, '/line_follower/cmd_vel', self.line_follower_callback, 10)
        self.line_follower_detection_sub = self.create_subscription(String, '/line_follower/line_detection', self.line_detection_callback, 10)
        self.wall_follower_twist_sub = self.create_subscription(Twist, '/wall_follower/cmd_vel', self.wall_follower_callback, 10)
        self.go_to_waypoint_twist_sub = self.create_subscription(Twist, '/go_to_waypoint/cmd_vel',self.go_to_waypoint_callback, 10)
        self.pure_pursuit_twist_sub = self.create_subscription(Twist, '/pure_pursuit/cmd_vel', self.pure_pursuit_callback, 10)
        #self.resnet_sub = self.create_subsctription(Predictor, '/object_detector', self.predictor_callback, 10)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.01 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
       
        self.wall_following_started = 1
        self.get_logger().info("Starting Wall Following".format())
        self.wall_following_completed = 0
        self.line_following_started = 0
        self.line_following_completed = 0
        self.go_to_waypoint_started = 0
        self.go_to_waypoint_completed = 0
        self.pure_pursuit_started = 0
        self.pure_pursuit_completed = 0
        self.line_following_completed_counter = 0
        self.line_detection_counter = 0
        self.line_detected = 'Line not Detected'
        

    def line_follower_callback(self, data):
        self.line_follower_data = data
    
    def line_detection_callback(self, data):
        self.line_detected = data.data
    
    def wall_follower_callback(self, data):
        self.wall_follower_data = data
    
    def go_to_waypoint_callback(self, data):
        self.go_to_waypoint_data = data
    
    def pure_pursuit_callback(self,data): 
        self.pure_pursuit_data = data

    #def predictor_callback(self,data):
    #    self.predictor_callback_data = data
    
    def timer_callback(self):
        
        twist_object = Twist()
        try:
            if self.line_detected == 'Line Detected' and self.line_detection_counter < 200:
                self.line_detection_counter += 1
                self.line_following_completed_counter = 0
        
        
            if self.line_detection_counter == 200 and self.line_following_started == 0:
                self.line_following_started = 1
                self.wall_following_completed = 1
                self.get_logger().info("Wall Following Complete".format())
                self.get_logger().info("Starting Line Following".format())
                
            if self.line_follower_data != 'Line not Detected' and self.line_following_started == 1 and self.line_following_completed_counter < 200:
                self.line_detection_counter = 0
                self.line_following_completed_counter += 1

            if self.line_following_completed_counter == 200:
                self.line_following_completed = 1
                self.go_to_waypoint_started = 1
                self.get_logger().info("Line Following Complete".format())
     
            
            if self.go_to_waypoint_started == 1:
                self.get_logger().info("Starting go_to_waypoint".format()) 
                twist_object.linear.x = self.go_to_waypoint_data.linear.x
                twist_object.angular.z = self.go_to_waypoint_data.angular.z      


            if self.wall_following_started == 1 and self.wall_following_completed == 0:
                twist_object.linear.x = self.wall_follower_data.linear.x
                twist_object.angular.z = self.wall_follower_data.angular.z
                
            if self.line_following_started == 1 and self.line_following_completed == 0:
                twist_object.linear.x = self.line_follower_data.linear.x
                twist_object.angular.z = self.line_follower_data.angular.z

            if self.go_to_waypoint_started == 1 and self.line_following_completed == 1:
                twist_object.linear.x = 0.0
                twist_object.angular.z = 0.0
                
            #if self.pure_pursuit_started == 1 and self.pure_pursuit_completed == 0:
            #    twist_object.linear.x = self.pure_pursuit_data.linear.x
            #    twist_object.angular.z = self.pure_pursuit_data.angular.z
                
            if self.pure_pursuit_started == 1 and self.pure_pursuit_completed == 1:
                twist_object.linear.x = 0.0
                twist_object.angular.z = 0.0
        
        except AttributeError:
            twist_object.linear.x = 0.0
            twist_object.angular.z = 0.0
        #self.get_logger().info("Steering angle: {:.3f}".format(twist_object.angular.z))
        self.twist_pub.publish(twist_object)
      
def main(args=None):
    print("Object Recognition Started")

    rclpy.init(args=args)
    node = state_machine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      


