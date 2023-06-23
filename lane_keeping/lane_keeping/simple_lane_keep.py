#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import math
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt


class LaneKeep(Node):
    def __init__(self):
        super().__init__("simple_lane_keep")
        '''
        Create Publisher, Subscribers and the timers:
        We are subscribing to the camera image and publishing AckermannDriveStamped message to control the car
        We are also  creating bridge_object to convert ros images into opencv images.
        '''
        self.image_sub = self.create_subscription(Image, "/camera/zed/rgb/image_rect_color", self.camera_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.timer_period = 0.00001 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.timer_callback) 
        self.bridge_object = CvBridge()
        
        '''
        Set Vehicle Parameters: The only vehicle parameter we will be using is the desired vehicle speed. 
        '''
        self.VELOCITY = 1
        
        '''
        Image Parameters:
        These are the parameters that we will be using to manipulate the image. We want to crop the image so that we don't process the
        unnecessary area of the image.
        The camera image that we get is 480 pixels X 640 pixels. We will crop the image in the vertical direction.
        '''
        self.image_y0 = 287     #The first row of image pixel array that we are interested in
        self.image_y1 = 479     #The last row of image pixel array we are interested in
        
        '''
        Control Parameters
        kp_steer controls the steering angle of the vehicle. The larger the error, the larger the steering angle with lower bound of -0.5 and
        upper bound of 0.5
        kp_vel controls the vehicle speed. The larger the error, the smaller the speed. This will slow down the vehicle on sharp turns.
        '''
        self.kp_steer = 0.0075
        self.kp_vel = 0.003
        
    '''
    The following function takes the cropped image as an input and finds the center line of the image.
    It masks the image to identify the lanes and remove noise, then it uses canny algorithm to find the edges.
    After finding the edges, it usescv2.HoughLinesP to find lines and the averages the slope and intercept to calculate the center line.
    '''    
    def calc_center_line(self, Image):
        height, width, channels = Image.shape     #Get the height of the cropped image
        y0 = 0                                    
        y1 = height - 1

        hsv = cv2.cvtColor(Image, cv2.COLOR_BGR2HSV)   #Convert BGR to HSV 
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)  #Mask the image to reduce noise and segregate lanes clearly from the road
        edges = cv2.Canny(mask, 50, 200)                   #Use Canny Algorithm to find the edges of the lanes
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 40, 5)  #Find the line coordinates for the edges of the lanes
        
        '''
        The following If statement/For loop calculates the slope and intercept of each prrobabilistic lines found in the previous step.
        It then calculates the average slope and intercept of lines on each side of the vehicle.
        Then, it calculates the x coordinates for top and bottom points on each side.
        It then averages out the x coordinates to find the center of the lane.
        '''
        if linesP is not None:
            neg_slope = 0
            neg_intercept = 0
            neg_count = 0
            pos_slope = 0
            pos_intercept = 0
            pos_count = 0
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                m, c = np.polyfit([l[0], l[2]], [l[1], l[3]], 1)  #Find slope and intercept of each line
        
                if m <= 0:           #Sum up slope and intercept of each line on the right side of the vehicle
                    neg_slope += m
                    neg_intercept += c
                    neg_count += 1
                else:                #Sum up slope and intercept of each line on the left side of the vehicle
                    pos_slope += m
                    pos_intercept += c
                    pos_count += 1
            neg_slope /= neg_count            #Find the average slope for the right lane
            neg_intercept /= neg_count        #Find the average intercept for the right lane
            pos_slope /= pos_count            #Find the average slope for the left lane  
            pos_intercept /= pos_count        #Find the average intercept for the right lane
            '''
            Find x coordinates using y = mx + c equation
            '''
            x0l = (y0 - pos_intercept)/pos_slope               
            x1l = (y1 - pos_intercept)/pos_slope
            x0r = (y0 - neg_intercept)/neg_slope
            x1r = (y1 - neg_intercept)/neg_slope
            '''
            Find x coordinates of the center line
            '''
            x0 = (x0l+x0r)/2
            x1 = (x1l+x1r)/2                
            return x0, y0, x1, y1
    
    '''
    The following function finds the uses center line coordinates and publishes the control parameters. 
    '''
    
    def steering_control(self, x0, y0, x1, y1):
        theta = math.atan2(y1 - y0, x1 - x0)   #Calculate center line angel. theta = atan2(slopeof the line). slope = (y1-y0)/(x1-x0) 
        
        error = 180*theta/math.pi - 90    #Calculate the error. We want our vehicle in the center of the lane so the center line should be vertical
            
        steering_angle = self.kp_steer*error #Calculate desired steering angle
        
        '''
        The following if condition puts upper and lower limit of the steering angle.
        '''
        if steering_angle > 0.5:
            steering_angle = 0.5
        elif steering_angle < -0.5:
            steering_angle = -0.5  
        
        self.get_logger().info("Steering angle: {:.3f}".format(steering_angle))

        vel = self.VELOCITY - abs(self.kp_vel*error)    #Calculate the Velocity

        self.get_logger().info("Velocity: {:.3f}".format(vel))
        
        '''
        Publish the message
        '''
        
        msg = AckermannDriveStamped()
        msg.drive.speed = vel
        msg.drive.steering_angle = steering_angle
        self.pub.publish(msg)
    
    
    '''
    The following function gets callback at every timer_period seconds. It takes the image published by the camera and calls necessary functions to 
    publish the control commands.
    '''    
    def timer_callback(self):    
        try:
            cv_image = self.image
            height, width, channels = cv_image.shape    #Find the dimensions of the image
            crop_img = cv_image[self.image_y0:self.image_y1][1:width]   #Crop the image
            
            
            x0, y0, x1, y1 = self.calc_center_line(crop_img)    #Find the center line of the image
            self.steering_control(x0, y0, x1, y1)               #Call the steering angle function to publish the control commands
            cv2.line(cv_image, (int(x0), self.image_y0), (int(x1), self.image_y1), (0,0,255), 3)   #Show the Camera image with the lane center line
            cv2.imshow("Center Line (in red) - Probabilistic Line Transform", cv_image)   
            cv2.waitKey(1)
        except (AttributeError, ZeroDivisionError):
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.get_logger().info("Steering angle: {:.3f}".format(msg.drive.steering_angle))
            self.get_logger().info("Velocity: {:.3f}".format(msg.drive.speed))
        
            self.pub.publish(msg)
            
        
    def camera_callback(self, data):
        """
        This function saves the camera image as self.image wheneever camera publishes the image
        """
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
        except CvBridgeError as e:
            self.get_logger().info("{}".format(e))
        
        
        
def main(args=None):
    print("Lane Keeping Started")

    rclpy.init(args=args)
    node = LaneKeep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
