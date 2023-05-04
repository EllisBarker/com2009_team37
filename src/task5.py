#!/usr/bin/env python3

import rospy

# Imaging modules
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Movement and sensor modules
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Need SLAM modules??????

class Exploration():
    def __init__(self):
        pass

if __name__ == '__main__':
    exploration_instance = Exploration()
    try:
        exploration_instance.main()
    except rospy.ROSInterruptException:
        pass