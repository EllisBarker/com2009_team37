#!/usr/bin/env python3
import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from laser_distance import LaserDistance
from math import sqrt


class MazeNavigation():
    def __init__(self):
        self.node_name = "maze_navigation"

        rospy.init_node(self.node_name, anonymous=True)
        rate = rospy.Rate(2) # hz

        self.vel_controller = Tb3Move()
        self.tb3_lidar = Tb3LaserScan()
        self.tb3_odm = Tb3Odometry()
        self.laser_sub = LaserDistance()

        # maximum allowed velocities
        self.max_lin_vel = 0.2
        self.max_ang_vel = 0.6

        self.ctrl_c = False 
        rospy.sleep(0.1)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 3: '{self.node_name}' node is active...") 

    def shutdown(self):
        print(f"Stopping the '{self.node_name}' task 3 node.")
        self.vel_controller.set_move_cmd(linear=0, angular=0)
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:

            print(self.tb3_lidar.distance)

            left_wall_rate  = self.tb3_lidar.distance.l3 - self.tb3_lidar.distance.l4
            right_wall_rate = self.tb3_lidar.distance.r3 - self.tb3_lidar.distance.r4
            
            if self.tb3_lidar.min_distance < 0.6: # if obstacle infront

                # if wall infront - slow down
                if self.tb3_lidar.min_distance < 0.4:
                    lin_speed = 0.0
                else:
                    lin_speed = self.max_lin_vel

                # if there is a gap on the right - turn right
                if self.tb3_lidar.distance.r1 > 0.6 or self.tb3_lidar.distance.r3 + self.tb3_lidar.distance.r4 > 1.5:
                    print("Wall detected: Turning right!")
                    self.vel_controller.set_move_cmd(linear=lin_speed , angular=-self.max_ang_vel)

                # if there is a gap on the left - turn left
                elif self.tb3_lidar.distance.l1 > 0.6 or self.tb3_lidar.distance.l3 + self.tb3_lidar.distance.l4 > 1.5:
                    print("Wall detected: Turning left!")
                    self.vel_controller.set_move_cmd(linear=lin_speed , angular=self.max_ang_vel)

                # if there is a dead end
                else:
                    self.vel_controller.set_move_cmd(linear=lin_speed , angular=-self.max_ang_vel)

            else: # if no obstacles infront
                
                # if nothing on the right - turn right
                if right_wall_rate >= 0.6 or self.tb3_lidar.distance.r2 > 0.8:
                    print("Moving Forwards: Turning right!")
                    self.vel_controller.set_move_cmd(linear=self.max_lin_vel, angular=-self.max_ang_vel)                   
                
                # if right wall is flat - move forward 
                elif right_wall_rate < 0.1:
                    wall_rate = right_wall_rate
                    print("Moving Forwards: Using right wall!")
                    if abs(wall_rate) < 0.001:
                        # forwards
                        self.vel_controller.set_move_cmd(linear=self.max_lin_vel, angular=0.0)
                    elif wall_rate < 0:
                        # correct left
                        self.vel_controller.set_move_cmd(linear=self.max_lin_vel, angular=0.1)
                    else:
                        # correct right
                        self.vel_controller.set_move_cmd(linear=self.max_lin_vel, angular=-0.1)

                # if nothing on the left - turn left
                elif left_wall_rate > 0.6 or self.tb3_lidar.distance.l2 > 0.8:
                    print("Moving Forwards: Turning left!")
                    self.vel_controller.set_move_cmd(linear=self.max_lin_vel, angular=self.max_ang_vel)
            
            print(f"{right_wall_rate=:.3f}")
            print(f"{left_wall_rate=:.3f}")
            self.vel_controller.publish()
            rospy.sleep(0.01)
            

if __name__ == '__main__': 
    node = MazeNavigation() 
    try:
        node.main_loop() 
    except rospy.ROSInterruptException:
        pass