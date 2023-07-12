#!/usr/bin/env python3

import os, csv, math, rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from matplotlib import patches
from skspatial.objects import Circle
from skspatial.objects import Line
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.odom_sub = self.create_subscription(Odometry, '/vesc/odom', self.pose_callback, 10)
        self.timer_period = 0.001 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.timer_callback) 
        self.lookahead_distance = 0.1 #
        self.current_waypoint_index = 0
        self.read_points()

        
    def pose_callback(self, data):
        self.xc = data.pose.pose.position.x
        self.yc = data.pose.pose.position.y
        
        quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        euler = self.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

        
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
        
    def read_points(self):
        file_name='waypoints.csv' # Chnage to appropriate file name
        file_path = os.path.join(os.path.expanduser('~'), 'sae_ws/git_ws/bootcamp-assignments/pure_pursuit/waypoints', file_name)
        with open(file_path) as f:
            self.path_points = [tuple(line) for line in csv.reader(f)]
        self.no_of_waypoints = len(self.path_points)
        
    def find_nearest_waypoint(self):
        """Fill with appropriate logic to find nearest waypoint"""
        
        min_distance = math.inf
        for i in len(self.path_points):
            if find_distance_index_based(i) < min_distance:
                nearest_waypoint_index = 1
        return nearest_waypoint_index
        
    def find_distance_index_based(self,idx):
        return math.dist([self.xc, self.yc], [self.path_points[idx][0], self.path_points[idx][1]])
        
    def idx_close_to_lookahead(self, idx):
        """
        What is the index closest to lookahead distance. This will give us the correct waypoint index for interpolation.
        """
        for i in range(idx, self.no_of_waypoints):
            if find_distance_index_based(i) >= self.lookahead_distance:
                break
        return i
       
    
    def interpolate(self, idx):
        """
        Fill with appropriate logic to find the interpolated roots and set the target point to nearest point on the circle
        """
        circle = Circle([self.xc, self.yc], self.lookahead_distance)
        line = Line([self.path_points[idx-1][0], self.path_points[idx-1][1]], [self.path_points[idx][0], self.path_points[idx][1]])
        point_a, point_b = circle.intersect_line(line)
        a = math.dist([self.path_points[idx][0], self.path_points[idx][1]], [point_a[0], point_a[1]])
        b = math.dist([self.path_points[idx][0], self.path_points[idx][1]], [point_b[0], point_b[1]])
        if a > b:
            lookahead_point = point_a
        else:
            lookahead_point = point_b
        return lookahead_point
    
    def find_angle_alpha(self, lookahead_point):
        theta = math.atan2((self.yc - lookahead_point[1]), (self.xc - lookahead_point[0]))
        return (theta - self.yaw)
    
    def find_delta(self, alpha):
        
    
    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
            else:
                plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                          fc=fc, ec=ec, head_width=width, head_length=width)
                plt.plot(x, y)
                patches.Rectangle((self.xc,self.yc), 0.35,0.2)
    
    def follow_waypoints(self):
        """
        Implement the pure pursuit controller based on the target point found.
        Make sure to incorporate error catching in case waypoint list is over
        """
        closest_waypoint = self.find_nearest_waypoint()
        waypoint_at_lookahead = self.idx_close_to_lookahead(closest_waypoint)
        lookahead_point = self.interpolate(waypoint_at_lookahead)
        angle_alpha = self.find_angle_alpha(lookahead_point)
        
        '''
        if self.show_animation:
            plt.cla()
            plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])
            #self.plot_arrow(self.xc, self.yc, self.yaw)
            plt.plot(self.cx, self.cy, "-r", label="course")
            plt.plot(self.xc, self.yc, "-b", label="trajectory")
            plt.plot(target_x, target_y, "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Pure Pursuit Control" + str(1))
            plt.pause(0.001)
        '''   
    def timer_callback(self):
        self.follow_waypoints()

        

def main(args=None):
    print("Pure Pursuit Started\n")

    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
