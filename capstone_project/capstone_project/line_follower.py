#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from predictor_msgs.msg import Predictor
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower")
        self.image_sub = self.create_subscription(Image, '/image_raw', self.camera_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.predictor_sub = self.create_subscription(Predictor, '/object_detector', self.predictor_callback, 10)
        self.pub = self.create_publisher(Twist, 'line_follower/cmd_vel', 10)
        self.line_detection_pub = self.create_publisher(String, '/line_follower/line_detection', 10)
        self.timer_period = 0.01 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.billboard_detected = 0
        self.billboard_start_counter = 0
        self.billboard_stop_counter = 0
        self.bridge_object = CvBridge()
        self.predictor_label = 'Nothing'

    def camera_callback(self, data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
        except CvBridgeError as e:
            self.get_logger().info("{}".format(e))

        
    def predictor_callback(self, data):
        self.predictor_label = data.label
        
    def timer_callback(self):
        if self.predictor_label == 'traffic_light':
            self.get_logger().info("Traffic Light detected".format())
            self.billboard_detected = 1
        if self.billboard_detected == 1 and self.billboard_stop_counter < 200:
            self.billboard_stop_counter += 1
        
        if self.billboard_detected == 1 and self.billboard_start_counter < 300 and self.billboard_stop_counter == 200:
            self.billboard_start_counter += 1
            twist_object = Twist()
            twist_object.linear.x = 0.0
            twist_object.angular.z = 0.0
            #self.get_logger().info("Steering angle: {:.3f}".format(twist_object.angular.z))
            
            self.pub.publish(twist_object)
        else:    
            try:
                cv_image = self.image
                height, width, channels = cv_image.shape
                crop_img = cv_image[int(height/2)+80:int(height/2)+120][1:width]
                height, width, channels = crop_img.shape
                hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
                lower_blue = np.array([80, 50, 50])
                upper_blue = np.array([165,255,255])
                mask = cv2.inRange(hsv, lower_blue, upper_blue)
                m = cv2.moments(mask, False)
                try:
                    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
                    msg = String()
                    msg.data = 'Line Detected'
                    self.line_detection_pub.publish(msg)
                except ZeroDivisionError:
                    cx, cy = width/2, height/2
                    msg = String()
                    msg.data = 'Line not Detected'
                    self.line_detection_pub.publish(msg)
                res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
                cv2.circle(res, (int(cx), int(cy)), 10, (0,0,255), -1)
                cv2.imshow("Original", cv_image)
                cv2.imshow("Resultant", res)
                cv2.waitKey(1)
                error_x =(width / 2) -  cx
                
                kp = 0.003# Tune the P-gain
                twist_object = Twist()
                twist_object.linear.x = 0.1# Set some value for velocity
                twist_object.angular.z = kp*error_x
                #self.get_logger().info("Steering angle: {:.3f}".format(twist_object.angular.z))
                self.pub.publish(twist_object)
            
            except (AttributeError, ZeroDivisionError):
                
                twist_object = Twist()
                twist_object.linear.x = 0.0
                twist_object.angular.z = 0.0
                #self.get_logger().info("Steering angle: {:.3f}".format(twist_object.angular.z))
                self.pub.publish(twist_object)
        

def main(args=None):

    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
