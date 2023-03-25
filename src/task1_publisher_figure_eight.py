#!/usr/bin/env python3

import rospy, math
from geometry_msgs.msg import Twist 

class FigureEightPublisher():
    def __init__(self):
        self.node_name = "figure_eight_publisher"
        topic_name = "cmd_vel"
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(1) 
        self.ctrl_c = False 
        self.current_loop = 1
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 1: '{self.node_name}' node is active...") 

    def shutdown(self):
        print(f"Stopping the '{self.node_name}' task 1 node at: {rospy.get_time()}")
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0 # m/s
        vel_cmd.angular.z = 0.0 # rad/s
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.10471975512 # m/s ((pi * 1 metre diameter) / 30 seconds)
            if self.current_loop == 1:
                vel_cmd.angular.z = (2 * math.pi) / 30 # rad/s
            elif self.current_loop == 2:
                vel_cmd.angular.z = - (2 * math.pi) / 30 # rad/s
            # CONDITIONAL HERE TO CHANGE THE VALUE OF CURRENT LOOP DEPENDING ON SOMETHING
            self.pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__': 
    publisher_instance = FigureEightPublisher() 
    try:
        publisher_instance.main_loop() 
    except rospy.ROSInterruptException:
        pass