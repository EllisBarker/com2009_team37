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
        self.colour_ranges = [["green",(37,50,100),(72,255,255)],
                            ["blue",(105,50,100),(135,255,255)],
                            ["red1",(0,50,100),(18,255,255)],
                            ["red2",(170,50,100),(180,255,255)],
                            ["yellow",(22,50,100),(35,255,255)],
                            ["purple",(141,50,100),(165,255,255)],
                            ["turquoise",(86,50,100),(94,255,255)]]
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

        for colour in self.colour_ranges:
            mask = cv2.inRange(self.hsv_img, colour[1], colour[2])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # FIND LARGEST CONTOUR AREA (IF THERE IS ONE)
            # COMPARE THIS AREA TO PREVIOUS MOST LARGE CONTOUR AREA
            # ASSIGN COLOUR FOUND BASED ON THIS
        
        # IF CONTOUR FOUND AND TARGET COLOUR IS NOT NONE
            # FIND MOMENTS
            # IF DETECTED COLOUR = TARGET COLOUR
                # LOG MESSAGE
                # SET BOOLEAN THAT ALLOWS BYPASS OF OBSTACLE AVOIDANCE CODE
                # SET VALUES THAT AID THE TURNING BASED ON MOMENTS
            # ELSE
                # it's all chill and stuff

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

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
                #self.vel_controller.set_move_cmd(0.0, (math.pi/2))
                #self.vel_controller.publish()
                #rospy.sleep(2)

                self.vel_controller.set_move_cmd(0.0, 0.0)
                self.vel_controller.publish()
                rospy.sleep(1)
                self.find_target_colour()
                print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour}.")

                #self.vel_controller.set_move_cmd(0.0, -(math.pi/2))
                #self.vel_controller.publish()
                #rospy.sleep(2)

                self.ctrl_c = True

            else:
                pass
                # OBJECT AVOIDANCE CODE FROM TASK 2
                # IF TARGET COLOUR DETECTED THEN START APPROACHING
        

if __name__ == '__main__':
    search_instance = ColourSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass