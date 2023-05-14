#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math
import time
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import random
# Import some other modules from within this package
from tb3 import Tb3Move
from tb3_task2 import Tb3LaserScan

class ColourSearch(object):

    def __init__(self):
        rospy.init_node('color_detect_node', anonymous=True)
        self.bridge = CvBridge()

        self.vel_controller = Tb3Move()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.tb3_lidar = Tb3LaserScan()

        self.hsv_img = None
        self.target_colour = "N/A"
        self.colour_ranges = [["green",(40,150,100),(65,255,255)],
                              ["blue",(115,225,100),(130,255,255)],
                              ["red1",(0,188,100),(4,255,255)],
                              ["red2",(175,200,100),(180,255,255)],
                              ["yellow",(25,120,100),(35,255,255)],
                              ["purple",(143,153,100),(157,255,255)],
                              ["turquoise",(84,150,100),(96,255,255)]]
        self.m00_min = 10000
        # Moment related values needed to determine the turning angle needed
        self.cy = -1
        self.cx = -1

        # 3 distinct phases: set target colour, freely roam, approach the target colour
        self.find_target_colour_phase = True
        self.approach_target_phase = False

        self.turning = False
        # Turn direction of 0 implies right, direction of 1 implies left
        self.direction = 0
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

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Can only create mask for target colour if it has been determined
        if self.target_colour != "N/A":
            mask = cv2.cv2.inRange(self.hsv_img, self.target_colour[1], self.target_colour[2])
            # Can only generate moments if the colour is seen, approach phase begins
            if mask.any():
                print("TARGET DETECTED: Beaconing initiated.")
                m = cv2.moments(mask)
                self.cx = m['m10'] / (m['m00'] + 1e-5)
                self.cy = m['m01'] / (m['m00'] + 1e-5)
                if m['m00'] > self.m00_min:
                    cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def set_target_colour(self):
        # Rotate 90 degrees to fill camera vision with a single colour
        self.vel_controller.set_move_cmd(0.0, (math.pi/2))
        self.vel_controller.publish()
        rospy.sleep(1)
        self.vel_controller.set_move_cmd(0.0, 0.0)
        self.vel_controller.publish()

        # Find a mask for any of the colours within the image
        for colour in self.colour_ranges:
            mask = cv2.inRange(self.hsv_img, colour[1], colour[2])
            if mask.any():
                self.target_colour = colour

        # Rotate 90 degrees again to face the original direction
        self.vel_controller.set_move_cmd(0.0, -(math.pi/2))
        self.vel_controller.publish()
        rospy.sleep(1)
        self.vel_controller.set_move_cmd(0.0, 0.0)
        self.vel_controller.publish()
        self.find_target_colour_phase = False
        print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour[0]}.")

    def main(self):
        while not self.ctrl_c:
            if self.find_target_colour_phase == True:
                self.set_target_colour()
            else:
                if self.approach_target_phase == True:
                    # ANGLES BASED ON MOMENTS
                    pass
                else:
                    # OBJECT AVOIDANCE CODE
                    pass
        

if __name__ == '__main__':
    search_instance = ColourSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass