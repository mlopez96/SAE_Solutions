#!/usr/bin/env python3

import os
import math
import rclpy
import atexit
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry

file_name = 'waypoints.csv' # Chnage to appropriate file name
file = open(os.path.join(os.path.expanduser('~'), 'sae_ws/git_ws/bootcamp-assignments/pure_pursuit/waypoints', file_name), 'w')

class PointLogger(Node):
     def __init__(self):
          super().__init__("waypoint_logger")
          self.odom_sub = self.create_subscription(Odometry, "/vesc/odom", self.odom_callback, 10)
          self.ctr = 0
     

     def odom_callback(self, data):
           quaternion = np.array([data.pose.pose.orientation.x,
                				  	data.pose.pose.orientation.y,
                					data.pose.pose.orientation.z,
									data.pose.pose.orientation.w])
           
           euler = self.euler_from_quaternion(quaternion)
           speed = np.linalg.norm(np.array([data.twist.twist.linear.x,
                                            data.twist.twist.linear.y, 
                                            data.twist.twist.linear.z]), 2)
           if data.twist.twist.linear.x > 0.:
                pass
           
           if self.ctr == 10:
                file.write("{:.1f}, {:.1f}, {:.1f}, {:.1f}\n".format(data.pose.pose.position.x, data.pose.pose.position.y, euler[2], speed))
                print("{:.1f}, {:.1f}".format(data.pose.pose.position.x, data.pose.pose.position.y))
                self.ctr = 0
           else:
                self.ctr += 1

     def euler_from_quaternion(self, quat):
          t0 = 2.0 * (quat[3] * quat[0] + quat[1] * quat[2])
          t1 = 1.0 - 2.0 * (quat[0]**2 + quat[1]**2)
          roll = math.atan2(t0, t1)
          
          t2 = 2.0 * (quat[3] * quat[1] - quat[2] * quat[0])
          t2 = 1.0 if t2 > 1.0 else t2
          t2 = -1.0 if t2 < -1.0 else t2
          pitch = math.asin(t2)
          
          t3 = 2.0 * (quat[3] * quat[2] + quat[0] * quat[1])
          t4 = 1.0 - 2.0 * (quat[1]**2 + quat[2]**2)
          yaw = math.atan2(t3, t4)
          
          return np.array([roll, pitch, yaw])
     

def shutdown():
	file.close()
	print('Goodbye')

def main(args=None):
    atexit.register(shutdown)

    print("Saving waypoints...")

    rclpy.init(args=args)
    node = PointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
