#!/usr/bin/env python3 

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserSub():
    def __init__(self):
        self.min_distance = float('inf')
        self.min_left = float('inf')
        self.min_right = float('inf')
        self.max_right = float('inf')
        self.min_left_45=float('inf')
        self.min_right_45=float('inf')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:30]
        right_arc = scan_data.ranges[-31:]
        front_arc = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_distance = front_arc.min()

        left_90 = scan_data.ranges[80:100]
        right_90 = scan_data.ranges[260:-81]
        self.min_left = np.array(left_90).min()
        self.min_right = np.array(right_90).min()
        self.max_right = np.array(right_90).max()
        left_45 = scan_data.ranges[15:45]
        right_45 = scan_data.ranges[-46:-16]
        self.min_left_45 = np.array(left_45).min()
        self.min_right_45 = np.array(right_45).min()
