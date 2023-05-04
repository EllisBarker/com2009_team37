#!/usr/bin/env python3
import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
import random

class ObstacleAvoidance():
    def __init__(self):
        self.node_name = "obstacle_avoidance"
        rospy.init_node(self.node_name, anonymous=True)

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        # Information published at 10Hz
        self.rate = rospy.Rate(1000) 
        self.ctrl_c = False 
        rospy.sleep(0.01)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 2: '{self.node_name}' node is active...") 

    def shutdown(self):
        print(f"Stopping the '{self.node_name}' task 2 node.")
        self.vel_controller.set_move_cmd(linear=0, angular=0)
        self.ctrl_c = True

    def main_loop(self):
        print ("TEST")
        while not self.ctrl_c:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            print (self.tb3_lidar.min_distance)
            if self.tb3_lidar.min_distance > 0.5: # approach distance
                self.vel_controller.set_move_cmd(linear=0.2, angular=0.0)
            else:
                self.vel_controller.set_move_cmd(linear=-1.0, angular=0.0)
                rospy.sleep(1)
                self.vel_controller.set_move_cmd(linear=0.0, angular=1.0)
                rospy.sleep(random.random()*3)
            self.vel_controller.publish()

if __name__ == '__main__': 
    node = ObstacleAvoidance() 
    try:
        node.main_loop() 
    except rospy.ROSInterruptException:
        pass