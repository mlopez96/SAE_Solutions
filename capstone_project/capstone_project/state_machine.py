#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from predictor_msgs.msg import Predictor

      
class State_Machine(Node):
    def __init__(self):
        super().__init__("State_Machine")
        self.line_follower_twist_sub = self.create_subscription(Twist, '/line_follower/cmd_vel', self.line_follower_callback, 10)
        self.line_follower_detection_sub = self.create_subscription(String, '/line_follower/line_detection', self.line_detection_callback, 10)
        self.wall_follower_twist_sub = self.create_subscription(Twist, '/wall_follower/cmd_vel', self.wall_follower_callback, 10)
        self.go_to_waypoint_twist_sub = self.create_subscription(Twist, '/go_to_waypoint/cmd_vel',self.go_to_waypoint_callback, 10)
        #self.pure_pursuit_twist_sub = self.create_subscription(Twist, '/pure_pursuit/cmd_vel', self.pure_pursuit_callback, 10)

        #wall folowing
        self.wall_following_completed_pub = self.create_publisher(String, '/state_machine/wf_task', 10)

        #line following/Stoplight
        self.stoplight_completed_pub = self.create_publisher(String, '/state_machine/sl_task', 10)

        #go to waypoint
        
        self.go_to_waypoint_started_pub = self.create_publisher(String, '/state_machine/go_to_waypoint', 10)
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
        self.line_lost_counter = 0
        self.robot_stopped_counter = 0

        self.line_detected = 'Line not Detected'

    def line_follower_callback(self, data):
        self.line_follower_data = data
    
    def line_detection_callback(self, data):
        self.line_detected = data.data
    
    def wall_follower_callback(self, data):
        self.wall_follower_data = data
    
    def go_to_waypoint_callback(self, data):
        self.go_to_waypoint_data = data
    
    #def pure_pursuit_callback(self,data): 
    #    self.pure_pursuit_data = data

    #def predictor_callback(self,data):
    #    self.predictor_callback_data = data
    
    def timer_callback(self):
        
        twist_object = Twist()
        try:
            #Wall Following - velocities from the wall following node are used until that task is completed
            if self.wall_following_started == 1 and self.wall_following_completed == 0:
                twist_object.linear.x = self.wall_follower_data.linear.x
                twist_object.angular.z = self.wall_follower_data.angular.z

            

            #Line Following
            #This tracks the amount of time a line has been visible. Used to transition to the line following task
            if self.line_detected == 'Line Detected' and self.line_detection_counter < 400:
                self.line_detection_counter += 1
                self.line_following_completed_counter = 0
        
            
            #End the wall following task when line detected for 3 seconds, and start line following task
            if self.line_detection_counter == 300 and self.line_following_started == 0:
                self.wall_following_completed = 1
                self.line_following_started = 1

                #Send message to kill line following node
                self.get_logger().info("Wall Following Complete".format())
                msg = String()
                msg.data = 'wf_task_complete'
                self.wall_following_completed_pub.publish(msg)
                self.get_logger().info("Starting Line Following".format())

            #Use the line following node velocities while that task is active, and track the amount of time line following has been active
            if self.line_following_started == 1 and self.line_following_completed == 0:
                twist_object.linear.x = self.line_follower_data.linear.x
                twist_object.angular.z = self.line_follower_data.angular.z
                self.line_following_completed_counter += 1
                self.get_logger().info("Starting counter".format()) 
            
            
            #If line following is active, but no line is visible, track the amount of time that no line has been visible. This is used later to determine when to end the line following task
            if self.line_following_started == 1 and self.line_following_completed == 0 and self.line_detected == 'LineNotDetected':
                self.line_lost_counter += 1               
            
            '''
            #To replace - old transition condition to mark line following complete after a certain period of time. However, the line following node's velocities will continue to be used until the robot comes to a stop.
            if self.line_following_completed == 0 and self.line_following_completed_counter > 1700:
                self.get_logger().info("counter finished".format()) 
                self.line_following_completed = 1
            '''
            
            
            #When the line following task has been active for a sufficient length of time, and no line has been visible for 3 seconds, we mark line following complete, stop the robot, and send a message to kill the line following node
            if self.line_following_completed == 0 and self.line_following_completed_counter > 1700 and self.line_lost_counter > 300:
                self.line_following_completed = 1
                twist_object.linear.x = 0.0
                twist_object.angular.z = 0.0
                msg = String()
                msg.data = 'sl_task_complete'
                self.stoplight_completed_pub.publish(msg)
                self.get_logger().info("Line following complete, pausing robot".format())
            
            
            #Tracks the time the robot has stopped when the line following task is marked complete, and starts taking velocity input from go to waypoint task. This will trigger when the line following task has been active for a sufficient time, and the robot is stopped due to encountering the stoplight (it will remain stopped as the line follower node is killed at that point)
            if twist_object.linear.x == 0.0 and self.line_following_completed == 1:
                self.get_logger().info("Starting go_to_waypoint".format()) 
                msg = String()
                msg.data = 'going_to_waypoint'
                self.go_to_waypoint_started_pub.publish(msg)
                twist_object.linear.x = self.go_to_waypoint_data.linear.x
                twist_object.angular.z = self.go_to_waypoint_data.angular.z
                self.robot_stopped_counter += 1
                
                     
            
            #When the robot has stopped for 5 seconds after completion of the line following task, kill the state machine so that the go to waypoint task can take over
            if self.robot_stopped_counter > 500:
                State_Machine.destroy_node(self)
                rclpy.shutdown(self)




            '''
            if self.line_follower_data == 'Line not Detected' and self.line_following_started == 1 and self.line_following_completed_counter < 200:
                self.line_detection_counter = 0
                self.line_following_completed_counter += 1
                self.get_logger().info("Line Not Detected Complete".format())

            if self.line_following_completed_counter == 200:
                self.line_following_completed = 1
                self.go_to_waypoint_started = 1
                self.go_to_waypoint_text == 'Start Localization'
        
                    
            #StopLight
            if twist_object.linear.x == 0.0:
                robot_stopped_counter += 1
                self.get_logger().info("Starting stopped counter".format())
                self.line_following_completed == 1


                
                
            if twist_object.linear.x == 0.0 and self.line_following_completed == 1 and self.robot_stopped_counter > 500:
                self.line_following_completed = 1
                msg = String()
                msg.data = 'sl_task_complete'
                self.stoplight_completed_pub.publish(msg)
                self.get_logger().info("Starting Line Following".format())


                #msg = String()
                #msg.data = 'wf_task_complete'
                #self.wall_following_completed_pub.publish(msg)
            
            '''

            '''
            #Go to Waypoint
            if self.line_following_completed == 1 and twist_object.angular.z == 0.0:
                self.go_to_waypoint_started = 1
                self.get_logger().info("self.go_to_waypoint_started = 1".format())

                #self.get_logger().info("Line Following Complete".format())
                #self.get_logger().info("Attempting go_to_waypoint".format()) 
                #twist_object.linear.x = 0.0
                #twist_object.angular.z = 0.0

            if self.go_to_waypoint_started == 1 and self.go_to_waypoint_completed == 0:
                self.get_logger().info("Starting go_to_waypoint".format()) 
                msg = String()
                msg.data = 'going to waypoint'
                self.go_to_waypoint_started_pub.publish(msg)

                twist_object.linear.x = self.go_to_waypoint_data.linear.x
                twist_object.angular.z = self.go_to_waypoint_data.angular.z      
                
            if self.go_to_waypoint_started == 1 and self.go_to_waypoint_completed == 1:
                twist_object.linear.x = 0.0
                twist_object.angular.z = 0.0
            '''

        except AttributeError:
            twist_object.linear.x = 0.0
            twist_object.angular.z = 0.0
        #self.get_logger().info("Steering angle: {:.3f}".format(twist_object.angular.z))
        self.twist_pub.publish(twist_object)
      
def main(args=None):
    print("Object Recognition Started")

    rclpy.init(args=args)
    node = State_Machine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
      


