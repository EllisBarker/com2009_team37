#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 

class FigureEightPublisher():
    def __init__(self):
        self.node_name = "figure_eight_publisher"
        topic_name = "cmd_vel"
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10) 
        self.ctrl_c = False 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 1: '{self.node_name}' node is active...") 

    def shutdown(self):
        pass

    def main_loop(self):
        pass

if __name__ == '__main__': 
    publisher_instance = FigureEightPublisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass