#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math
import time
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
from laser_distance import LaserDistance
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import numpy as np
# Import some other modules from within this package
from tb3 import Tb3Move

class ColourSearch(object):

    def __init__(self):
        rospy.init_node('color_detect_node', anonymous=True)
        self.bridge = CvBridge()

        self.vel_controller = Tb3Move()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.laser_sub = LaserDistance()

        self.current_angular_z = 0.0
        self.current_linear_x = 0.0

        self.target_colour = None
        self.target_lower = -999
        self.target_upper = 999
        self.colour_ranges = [["green",(56,173,0),(64,255,255)],
                              ["blue",(112,215,0),(124,255,255)],
                              ["red1",(0,206,0),(4,255,255)],
                              ["red2",(175,182,0),(180,255,255)],
                              ["yellow",(25,63,0),(35,255,255)],
                              ["purple",(143,56,0),(157,255,255)],
                              ["turquoise",(83,79,0),(92,234,255)]]
        self.m00 = 0
        self.m00_min = 10000
        self.hsv_img = None

        self.min_distance = 0.2
        self.max_distance = 0.6

        self.ctrl_c = False
        self.rate = rospy.Rate(100)
        self.start_time = rospy.get_rostime()
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.vel_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))

        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)

        #target_color_contours, channels = cv2.findContours(target_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # if len(target_color_contours) > 0:
        #     # Draw contours on original image
        #     cv2.drawContours(crop_img, target_color_contours, -1, (0, 255, 0), 2)

        #     # Get center of target color contours
        #     target_color_m = cv2.moments(target_color_contours[0])
        #     print("m00 :",target_color_m["m00"])
        #     target_color_cx = int(target_color_m["m10"] / target_color_m["m00"])
        #     target_color_cy = int(target_color_m["m01"] / target_color_m["m00"])
        #     self.m00 = target_color_m["m00"]
        #     self.cx = target_color_cx
        #     if  target_color_cy > 251 or self.m00 > self.m00_max:
        #         center_x = crop_width / 2
        #         # Calculate turn amount proportional to difference
        #         cv2.circle(crop_img, (target_color_cx, target_color_cy), 10, (0, 0, 255), 2)
        #         # Print target color and position
        #         print("Target color: {}, position: ({}, {})".format(self.target_color, target_color_cx, target_color_cy))
        # else:
        #     self.turn_amount=0
        #     # Calculate difference from center

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def find_most_common_color(self):
        colours = []
        for colour in self.colour_ranges:
            mask = cv2.inRange(self.hsv_img, colour[1], colour[2])
            m = cv2.moments(mask, binaryImage = True)
            colours.append([colour[0], m['m00']])
        colours.sort(key=lambda x:x[1], reverse=True)
        colour = colours[0][0]
        return colour

    def find_target_colour(self):
        for colour in self.colour_ranges:
            mask = cv2.inRange(self.hsv_img, colour[1], colour[2])
            print (mask)
            if mask.any():
                self.target_colour = colour[0]

    def main(self):
        while not self.ctrl_c:
            if self.hsv_img is None:
                continue
            if self.target_colour is None:
                self.vel_controller.set_move_cmd(0.0, (math.pi/2))
                self.vel_controller.publish()
                rospy.sleep(2)

                self.vel_controller.set_move_cmd(0.0, 0.0)
                self.vel_controller.publish()
                rospy.sleep(1)
                self.find_target_colour()
                print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour}.")

                self.vel_controller.set_move_cmd(0.0, -(math.pi/2))
                self.vel_controller.publish()
                rospy.sleep(2)

                self.ctrl_c = True
        

if __name__ == '__main__':
    search_instance = ColourSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass