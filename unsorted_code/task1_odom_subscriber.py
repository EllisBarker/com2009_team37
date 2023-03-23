#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomSubscriber():
    def __init__(self):
        node_name = "odom_subscriber"
        rospy.init_node(node_name, anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"Task 1: '{node_name}' node is active...")

    def callback(self, topic_data: Odometry):
        pass

    def main_loop(self):
        pass

if __name__ == '__main__':
    subscriber_instance = OdomSubscriber()
    subscriber_instance.main_loop()