#!/usr/bin/env python3
import rospy
from tb3_task2 import Tb3Move, Tb3Odometry, Tb3LaserScan
import random

class ObstacleAvoidance():
    def __init__(self):
        self.node_name = "obstacle_avoidance"
        rospy.init_node(self.node_name, anonymous=True)

        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        self.rate = rospy.Rate(1000) 
        self.ctrl_c = False 
        self.turning = False
        # Turn direction of 0 implies right, direction of 1 implies left
        self.direction = 0
        rospy.sleep(0.01)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 2: '{self.node_name}' node is active...") 

    def shutdown(self):
        print(f"Stopping the '{self.node_name}' task 2 node.")
        self.vel_controller.set_move_cmd(linear=0, angular=0)
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            if self.tb3_lidar.min_distance > 0.43: # approach distance
                self.vel_controller.set_move_cmd(linear=0.26, angular=0.0)
                self.vel_controller.publish()
                self.turning = False
            else:
                # Only move backwards once upon detecting an obstacle and pick a direction to turn
                if self.turning == False:
                    self.vel_controller.set_move_cmd(linear=-1.3, angular=0.0)
                    self.vel_controller.publish()
                    rospy.sleep(0.5)
                    self.turning = True
                    # If object detected on the left, turn right.
                    if self.closest_object_location >= 0:
                        self.direction = 0
                    # If object detected on the right, turn left.
                    else:
                        self.direction = 1
                # Start rotating for an amount of time between 0 and 1 seconds
                self.vel_controller.set_move_cmd(linear=0.0, angular=[1.0,-1.0][self.direction])
                self.vel_controller.publish()
                rospy.sleep(random.random())
            

if __name__ == '__main__': 
    node = ObstacleAvoidance() 
    try:
        node.main_loop() 
    except rospy.ROSInterruptException:
        pass