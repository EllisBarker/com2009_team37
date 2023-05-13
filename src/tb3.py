#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):

    class scanSubsets():
        def __init__(self):
            self.front = 0.0
            self.r1 = 0.0; self.r2 = 0.0; self.r3 = 0.0; self.r4 = 0.0
            self.l1 = 0.0; self.l2 = 0.0; self.l3 = 0.0; self.l4 = 0.0
        
        def __str__(self):
            msg = f"              l1     front     r1          \n" \
                  f"       l2     {self.l1:<5.3f}  {self.front:^5.3f}  {self.r1:>5.3f}     r2       \n" \
                  f"l3     {self.l2:<5.3f}                       {self.r2:>5.3f}   r3\n" \
                  f"{self.l3:<5.3f}                                   {self.r3:>5.3f}\n" \
                  f"{self.l4:<5.3f} <-- l4                     r4 --> {self.r4:>5.3f}"
            return msg
    
    def laserscan_cb(self, scan_data):

        def min_of_subset(start_index, stop_index):
            range = np.array(scan_data.ranges[start_index: stop_index+1])
            valid_data = range[range>0.1]
            return valid_data.mean() if np.shape(valid_data)[0] > 0 else np.nan
        
        left_arc = scan_data.ranges[0:21]
        right_arc = scan_data.ranges[-20:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        valid = front_arc[front_arc>0.1]
        self.distance.front = valid.mean() if np.shape(valid)[0] > 0 else np.nan
        
        self.min_distance = front_arc.min()
        arc_angles = np.arange(-20, 21)
        self.closest_object_position = arc_angles[np.argmin(front_arc)]

        # right subsets:
        self.distance.r1 = min_of_subset(320, 340)
        self.distance.r2 = min_of_subset(300, 320)
        self.distance.r3 = min_of_subset(275, 290)
        self.distance.r4 = min_of_subset(250, 265)
        
        # left subsets:
        self.distance.l1 = min_of_subset(20, 40)
        self.distance.l2 = min_of_subset(40, 60)
        self.distance.l3 = min_of_subset(70, 85)
        self.distance.l4 = min_of_subset(95, 110)

    def __init__(self):
        self.distance = self.scanSubsets()
        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
