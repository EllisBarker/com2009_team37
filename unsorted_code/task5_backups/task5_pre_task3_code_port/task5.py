#!/usr/bin/env python3

import rospy, roslaunch

# Imaging modules
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pathlib import Path
import argparse
import random

# Movement and sensor modules
from tb3 import Tb3Move, Tb3Odometry
from tb3_task2 import Tb3LaserScan

class Exploration():
    def __init__(self):
        node_name = "exploration"
        rospy.init_node(node_name)
        
        # Colour and beaconing-related values
        self.m00 = 0
        self.m00_min = 20000
        self.target_colour = ""
        self.colour_ranges = [["green",(40,150,100),(65,255,255)],
                              ["blue",(115,225,100),(130,255,255)],
                              ["red",(0,188,100),(4,255,255)],
                              #["red2",(175,200,100),(180,255,255)],
                              ["yellow",(25,120,100),(35,255,255)]]
        cli = argparse.ArgumentParser()
        cli.add_argument("-target_colour", metavar = "COL", default="blue")
        colour_name = cli.parse_args(rospy.myargv()[1:]).target_colour
        for colour in self.colour_ranges:
            if colour_name in colour:
                self.target_colour = colour
        self.picture_taken = False
        self.picture_signal = False
        self.found_time = 0
        # Turn velocity value (positive is left, negative is right)
        self.turn_vel = 0.6
        self.move_rate = ""

        self.ctrl_c = False 
        self.turning = True
        # Turn direction of 0 implies right, direction of 1 implies left
        self.direction = 0

        self.rate = rospy.Rate(10)
        self.start_time = rospy.get_rostime()

        # Camera-related functionality
        # "/camera/rgb/image_raw" for simulation
        self.camera_topic = "/camera/rgb/image_raw"
        self.camera_subscriber = rospy.Subscriber(self.camera_topic,
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        # Movement and sensor controllers
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        # Shutdown-related operations
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        rospy.loginfo(f"TASK 5 BEACON: The target is {self.target_colour[0]}.")

    def shutdown_ops(self):
        self.vel_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def save_picture(self, img, img_name):
        full_image_path = Path.home().joinpath("catkin_ws/src/com2009_team37/snaps")
        full_image_path.mkdir(parents=True, exist_ok=True)
        full_image_path = full_image_path.joinpath(f"{img_name}.jpg") 
        print(f"Saving the image to '{full_image_path}'...")
        cv2.imwrite(str(full_image_path), img) 
        cv2.waitKey(0) 

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        if self.picture_signal == True:
            self.save_picture(cv_img, img_name = "the_beacon")
            self.picture_signal = False
            self.picture_taken = True

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.target_colour[1], self.target_colour[2])
        if mask.any():
            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)
            print (f"THIS IS CY {self.cy}")
            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

    def main(self):
        while not self.ctrl_c:
            # Blob detected and picture has not yet been taken
            if self.picture_taken == False and self.m00 > self.m00_min:
                self.vel_controller.set_move_cmd(0.0,0.0)
                self.vel_controller.publish()

                # Setting speed to turn at depending on blob position
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                else:
                    self.move_rate = 'slow'
                
                # Publish turn commands to turn robot with turn directions dependent on the position of the blob
                if self.move_rate == 'slow':
                    print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    if self.cy < 560:
                        self.vel_controller.set_move_cmd(0.0, self.turn_vel)
                    elif self.cy > 560:
                        self.vel_controller.set_move_cmd(0.0, -(self.turn_vel))
                # Stop turning once blob of colour is directly ahead
                elif self.move_rate == 'stop':
                    print(f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels...")
                    self.vel_controller.set_move_cmd(0.0, 0.0)
                    self.picture_signal = True
                else:
                    print(f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    if self.cy < 560:
                        self.vel_controller.set_move_cmd(0.0, self.turn_vel)
                    elif self.cy > 560:
                        self.vel_controller.set_move_cmd(0.0, -(self.turn_vel))

                self.vel_controller.publish()
                rospy.sleep(0.5)

            else:
                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                if self.tb3_lidar.min_distance > 0.25: # approach distance
                    self.vel_controller.set_move_cmd(linear=0.23, angular=0.0)
                    self.vel_controller.publish()
                    self.turning = False
                else:
                    # Only move backwards once upon detecting an obstacle and pick a direction to turn
                    if self.turning == False:
                        self.vel_controller.set_move_cmd(linear=-0.3, angular=0.0)
                        self.vel_controller.publish()
                        rospy.sleep(0.75)
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
                    rospy.sleep(0.5)

if __name__ == '__main__':
    exploration_instance = Exploration()
    try:
        exploration_instance.main()
    except rospy.ROSInterruptException:
        pass