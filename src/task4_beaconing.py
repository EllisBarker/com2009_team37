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

        self.robot_controller = Tb3Move()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.laser_sub = LaserDistance()

        self.current_angle = 0.0
        self.target_angle = 0.0
        #self.turn_vel_fast = -0.5
        #self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.target_color = None
        self.color_ranges = {
            "green": (np.array([0, 180, 0]), np.array([25, 255, 25])),
            "blue": (np.array([0, 0, 82]), np.array([25, 25, 221])),
            "lightblue": (np.array([0, 82, 82]), np.array([25, 122, 122])),
        }
        self.m00 = 0
        self.m00_min = 1000
        self.m00_max = 8000
        self.hsv_img=None
        self.lower = (115, 224, 100)
        self.upper = (130, 255, 255)
        self.cx=0
        self.stop_flag = False

        self.ctrl_c = False
        self.rate = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))

        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)

        # Identify target color
        target_color_mask = cv2.inRange(self.hsv_img, self.lower, self.upper)
        target_color_contours, channels = cv2.findContours(target_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(target_color_contours) > 0:
            # Draw contours on original image
            cv2.drawContours(crop_img, target_color_contours, -1, (0, 255, 0), 2)

            # Get center of target color contours
            target_color_m = cv2.moments(target_color_contours[0])
            print("m00 :",target_color_m["m00"])
            target_color_cx = int(target_color_m["m10"] / target_color_m["m00"])
            target_color_cy = int(target_color_m["m01"] / target_color_m["m00"])
            self.m00 = target_color_m["m00"]
            self.cx=target_color_cx
            if  target_color_cy>251 or self.m00 >self.m00_max:
                center_x = crop_width / 2
                diff_x = target_color_cx - center_x
                # Calculate turn amount proportional to difference
                self.turn_amount = diff_x / 100
                # Set robot move command to turn
                cv2.circle(crop_img, (target_color_cx, target_color_cy), 10, (0, 0, 255), 2)
                print("turn_amount :", self.turn_amount)
                # Print target color and position
                print("Target color: {}, position: ({}, {})".format(self.target_color, target_color_cx, target_color_cy))
        else:
            self.turn_amount=0
            # Calculate difference from center
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def find_most_common_color(self):
        colors = []
        for color in self.color_ranges:
            lower = self.color_ranges[color][0]
            upper = self.color_ranges[color][1]
            mask = cv2.inRange(self.hsv_img, lower, upper)
            count = cv2.countNonZero(mask)
            colors.append((count, color))
        colors.sort(reverse=True)
        color=colors[0][1]
        self.lower = self.color_ranges[color][0]
        self.upper = self.color_ranges[color][1]
        return color

    def main(self):
        move_time=0.5
        t_end = time.time() + 90
        while not self.ctrl_c:
            if self.hsv_img is None:
                continue
            if self.target_color is None:
                self.robot_controller.set_move_cmd(0.0, 0.6)
                rospy.sleep(2)  # adjust time as needed
                self.target_color = self.find_most_common_color()
                print("TARGET DETECTED: Beaconing initiated ", self.target_color)

                # turn back to original orientation
                self.robot_controller.set_move_cmd(0.0,-0.6)
                rospy.sleep(2)  # adjust time as needed
            else:
                pass

if __name__ == '__main__':
    search_instance = ColourSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass