#!/usr/bin/env python3

import rospy
import actionlib
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server", SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # useful publisher/subscriber functions from tb3
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # the 'callback' function of the server
    def action_server_launcher(self, goal: SearchGoal):
        pass

if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()