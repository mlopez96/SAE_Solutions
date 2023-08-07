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
        #Create Publishers, Subscribers and Timers
        self.pub = self.create_publisher(AckermannDriveStamped, '/vesc/ackermann_cmd_mux/input/teleop', 10)
        self.odom_sub = self.create_subscription(Odometry, '/vesc/odom', self.pose_callback, 10)
        self.timer_period = 0.01 # Create timer period
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        #Define Vehicle Parameters
        self.car_wheelbase = 0.325 
        self.vel = 0.5
        self.lookahead_distance = 0.5 #
        self.current_waypoint_index = 0
        
        #Read waypoints.csv file
        self.read_points()

        
    def pose_callback(self, data):
        self.xc = data.pose.pose.position.x
        self.yc = data.pose.pose.position.y
        
        quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        euler = self.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
       
        self.SPEED = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2 + data.twist.twist.linear.z**2)
        self.get_logger().info("X Pose = {:.3f}".format(self.xc))
        self.get_logger().info("Y Pose = {:.3f}".format(self.yc))
        self.get_logger().info("Yaw = {:.3f}".format(self.yaw))

        
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
        self.get_logger().info("no_of_waypoints = {:.3f}".format(self.no_of_waypoints))
        
    '''
    The following function finds the nearest waypoint from the car.
    '''    
    def find_nearest_waypoint(self):
        """Fill with appropriate logic to find nearest waypoint"""
        
        min_distance = math.inf
        nearest_waypoint_index = 0
        for i in range(len(self.path_points)):
            calc_dist = self.find_distance_index_based(i)
            if  calc_dist< min_distance:
                nearest_waypoint_index = i
                min_distance = calc_dist
        self.get_logger().info("nearest_waypoint_index = {:.3f}".format(nearest_waypoint_index))
        return nearest_waypoint_index
        
        
    def find_distance_index_based(self,idx):
        waypoint_x = float(self.path_points[idx][0])
        waypoint_y = float(self.path_points[idx][1])
        return math.dist([self.xc, self.yc], [waypoint_x, waypoint_y])
        
    '''
    The following function finds the index of the waypoint that is closer to the lookahead distance.
    '''    
    def idx_close_to_lookahead(self, idx):
        """
        What is the index closest to lookahead distance. This will give us the correct waypoint index for interpolation.
        """
        for i in range(idx, self.no_of_waypoints):
            if self.find_distance_index_based(i) >= self.lookahead_distance:
                break
        print("Index close to lookahead", i)
        return i
       
    '''
    The following function gets the index of the waypoint closer to the lookahead distance and finds the point along the path
    that is exactly at the lookahead distance from the car.
    '''
    def interpolate(self, idx):
        """
        Fill with appropriate logic to find the interpolated roots and set the target point to nearest point on the circle
        """
        circle = Circle([self.xc, self.yc], self.lookahead_distance)
        point_1 = [float(self.path_points[idx-1][0]), float(self.path_points[idx-1][1])]
        point_2 = [float(self.path_points[idx][0]), float(self.path_points[idx][1])]
        line = Line(point_1, point_2)
        try:
            point_a, point_b = circle.intersect_line(line)
        except ValueError:
            return [float(self.path_points[idx][0]), float(self.path_points[idx][1])]
        a = math.dist(point_2, [point_a[0], point_a[1]])
        b = math.dist(point_2, [point_b[0], point_b[1]])
        if a < b:
            lookahead_point = point_a
        else:
            lookahead_point = point_b
        print("Lookahead Point = ", lookahead_point)
        return lookahead_point
    
    '''
    The following function finds the angle of the line joining the lookahead point with the car center
    '''
    def find_angle_alpha(self, lookahead_point):
        theta = math.atan2((lookahead_point[1] - self.yc), (lookahead_point[0] - self.xc))
        return (theta - self.yaw)
    
    '''
    The following function finds the instantaneous center of rotation.
    '''
    def find_delta(self, alpha):
        delta = math.atan2(2*math.sin(alpha)*self.car_wheelbase, self.lookahead_distance)
        return delta
       
    
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
        delta = self.find_delta(angle_alpha)
        
        msg = AckermannDriveStamped()

        msg.drive.speed = self.vel
        msg.drive.steering_angle = 0.75*delta
        self.get_logger().info("Speed Input = {:.3f}".format(msg.drive.speed))
        self.get_logger().info("Steering Angle = {:.3f}".format(msg.drive.steering_angle))
        self.pub.publish(msg)
        
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
           
    def timer_callback(self):
        try:
            self.follow_waypoints()
        except AttributeError:
            pass

def main(args=None):
    print("Pure Pursuit Started\n")

    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
