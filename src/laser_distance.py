#!/usr/bin/env python3 

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserDistance():
    def __init__(self):
        self.min_front_distance = float('inf')
        self.min_left_distance = float('inf')
        self.min_right_distance = float('inf')
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:30]
        right_arc = scan_data.ranges[-31:]
        front_distance = np.array(left_arc[::-1] + right_arc[::-1])
        self.min_front_distance = front_distance.min()

        left_90 = scan_data.ranges[80:100]
        right_90 = scan_data.ranges[260:-81]
        self.min_left_distance = np.array(left_90).min()
        self.min_right_distance = np.array(right_90).min()