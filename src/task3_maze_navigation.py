#!/usr/bin/env python3
import rospy
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt

class MazeNavigation():
    def __init__(self):
        self.node_name = "maze_navigation"

        rospy.init_node(self.node_name, anonymous=True)
        rate = rospy.Rate(2) # hz

        self.vel_controller = Tb3Move()
        self.tb3_lidar = Tb3LaserScan()
        self.tb3_odm = Tb3Odometry()

        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.direction = 0
        self.movement = "move_fwd"
        self.transition = True

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

            if self.transition:
                self.vel_controller.set_move_cmd(linear=0, angular=0) # stop
                # reset all values
                yaw_ref = self.tb3_odm.yaw
                xpos_ref = self.tb3_odm.posx
                ypos_ref = self.tb3_odm.posy
                current_yaw = 0.0
                current_distance = 0.0
                self.transition = False
                print(f"Entering state: {self.movement}")
            elif self.movement == "turn":
                current_yaw = current_yaw + abs(self.tb3_odm.yaw - yaw_ref)
                yaw_ref = self.tb3_odm.yaw

                print("Turning: Rotating",["90 left!","90 right!","180 dead end!"][self.direction])

                if current_yaw >= [90, 90, 180][self.direction]:
                    self.movement = "move_fwd"
                    self.transition = True
                else:
                    self.vel_controller.set_move_cmd(linear=0.0, angular=[0.2,-0.2, 0.2][self.direction])

                print(f"{current_yaw=}")

            elif self.movement == "move_fwd":
                    current_distance = current_distance + sqrt(pow(self.tb3_odm.posy-ypos_ref,2) + pow(self.tb3_odm.posx-xpos_ref,2))
                    xpos_ref = self.tb3_odm.posx
                    ypos_ref = self.tb3_odm.posy
                    
                    if self.tb3_lidar.min_distance < 0.42: # if obstacle infront
                        
                        # if walls on both sides - dead end - turn 180
                        if  self.tb3_lidar.distance.r4 < 0.5 and self.tb3_lidar.distance.l4 < 0.5:
                            self.movement = "turn"
                            self.transition = True
                            self.direction = 2

                        # if nothing on the right - turn 90 right
                        elif self.tb3_lidar.distance.r4 > 0.8:
                            self.movement = "turn"
                            self.transition = True
                            self.direction = 1

                        # if nothing on the left - turn 90 left
                        elif self.tb3_lidar.distance.l4 > 0.8:
                            self.movement = "turn"
                            self.transition = True
                            self.direction = 0

                    else: # move forwards    

                        # prefer the right wall
                        if abs(right_wall_rate) > 0.1:
                            wall_rate = left_wall_rate
                            print("Moving Forwards: Using left wall!")
                            if abs(wall_rate) < 0.001:
                                # forwards
                                self.vel_controller.set_move_cmd(linear=0.2, angular=0.0)
                            elif wall_rate < 0:
                                # correct right
                                self.vel_controller.set_move_cmd(linear=0.2, angular=-0.1)
                            else:
                                # correct left
                                self.vel_controller.set_move_cmd(linear=0.2, angular=0.1)
                        else:
                            wall_rate = right_wall_rate
                            print("Moving Forwards: Using right wall!")
                            if abs(wall_rate) < 0.001:
                                # forwards
                                self.vel_controller.set_move_cmd(linear=0.2, angular=0.0)
                            elif wall_rate < 0:
                                # correct left
                                self.vel_controller.set_move_cmd(linear=0.2, angular=0.1)
                            else:
                                # correct right
                                self.vel_controller.set_move_cmd(linear=0.2, angular=-0.1)

                    print(f"{current_distance=}")
            
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