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
        self.laser_sub = LaserDistance()

        self.find_target_colour_phase = False
        self.target_colour = None
        self.colour_ranges = [["green",(40,150,100),(65,255,255)],
                              ["blue",(115,225,100),(130,255,255)],
                              ["red1",(0,188,100),(4,255,255)],
                              ["red2",(175,200,100),(180,255,255)],
                              ["yellow",(25,120,100),(35,255,255)],
                              ["purple",(143,153,100),(157,255,255)],
                              ["turquoise",(84,150,100),(96,255,255)]]
        self.m00_min = 10000

        self.turning = False
        # Turn direction of 0 implies right, direction of 1 implies left
        self.direction = 0
        self.approach_phase = False
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
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        detected_colour = None
        detected_colour_contour = None
        detected_colour_contour_area = 0
        # Create masks and contours for each colour available to obtain the contour of the greatest area
        for colour in self.colour_ranges:
            mask = cv2.inRange(hsv_img, colour[1], colour[2])
            if colour[0] == "red1":
                mask = mask + cv2.inRange(hsv_img, self.colour_ranges[3][1], self.colour_ranges[3][2])
            if colour[0] == "red2":
                mask = mask + cv2.inRange(hsv_img, self.colour_ranges[2][1], self.colour_ranges[2][2])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) != 0:
                for contour in contours:
                    if cv2.contourArea(contour) > detected_colour_contour_area:
                        detected_colour_contour = contour
                        detected_colour_contour_area = cv2.contourArea(contour)
                        if colour[0] == "red1" or colour[0] == "red2":
                            detected_colour = "red"
                        else:
                            detected_colour = colour[0]
        
        # Check for if the target colour has already been determined or if contour was generated
        if detected_colour_contour is not None and self.target_colour != None:
            if detected_colour == self.target_colour:
                # Use moments-related information to determine appropriate turning angle
                m = cv2.moments(detected_colour_contour)
                cx = m['m10'] / (m['m00'] + 1e-5)
                cy = m['m01'] / (m['m00'] + 1e-5)
                cv2.circle(crop_img, (int(cy), 200), 10, (0, 0, 255), 2)
                if self.approach_phase == False:
                    print("TARGET DETECTED: Beaconing initiated.")
                    self.approach_phase = True
                # SET VALUES THAT AID THE TURNING BASED ON MOMENTS???
        # First-time setup for target colour if it is in the 'find target colour' phase
        elif self.target_colour == None:
            if self.find_target_colour_phase == True:
                self.target_colour = detected_colour
                print(f"SEARCH INITIATED: The target beacon colour is {self.target_colour}.")

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    #def find_target_colour(self):
    #    for colour in self.colour_ranges:
    #        mask = cv2.inRange(hsv_img, colour[1], colour[2])
    #        print (mask)
    #        if mask.any():
    #            self.target_colour = colour[0]

    def main(self):
        while not self.ctrl_c:
            if self.target_colour is None:
                self.vel_controller.set_move_cmd(0.26, 0.0)
                self.vel_controller.publish()
                rospy.sleep(2)

                self.vel_controller.set_move_cmd(0.0, (math.pi/2))
                self.vel_controller.publish()
                rospy.sleep(2)

                self.vel_controller.set_move_cmd(0.0, 0.0)
                self.vel_controller.publish()
                rospy.sleep(1)
                #self.find_target_colour()
                self.find_target_colour_phase = False
                

                self.vel_controller.set_move_cmd(0.0, -(math.pi/2))
                self.vel_controller.publish()
                rospy.sleep(2)

            else:
                if self.approach_phase == False:
                    pass
                    # OBSTACLE AVOIDANCE CODE
                else:
                    pass
                # IF TARGET COLOUR DETECTED THEN START APPROACHING
        

if __name__ == '__main__':
    search_instance = ColourSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass