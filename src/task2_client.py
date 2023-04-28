#!/usr/bin/env python3

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def __init__(self):
        pass

    def feedback_callback(self, feedback_data: SearchFeedback):
        pass

    def shutdown_ops(self):
        pass

    def main_loop(self):
        self.goal.approach_distance = 0.4 # metres
        self.goal_fwd_velocity = 0.15 # m/s
        
if __name__ == '__main__':
    node = SearchActionClient()
    node.main_loop()