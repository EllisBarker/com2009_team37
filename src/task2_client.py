#!/usr/bin/env python3

import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class SearchActionClient():
    goal = SearchGoal()

    def __init__(self):
        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient("/task_2_search", SearchAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdown_ops)

    def feedback_callback(self, feedback_data: SearchFeedback):
        pass

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received shutdown request. Cancelling...")
            self.client.cancel_goal()
            rospy.logwarn("Cancelled...")

        rospy.sleep(1)
        result = self.client.get_result()
        # print results here if need be?

    def main_loop(self):
        self.goal.approach_distance = 0.4 # metres
        self.goal.fwd_velocity = 0.15 # m/s
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
        
if __name__ == '__main__':
    node = SearchActionClient()
    node.main_loop()