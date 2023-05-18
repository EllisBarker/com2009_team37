#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math
import time
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
from laser_distance import LaserSub
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import numpy as np
# Import some other modules from within this package
from tb3 import Tb3Move
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class colour_search(object):

    def __init__(self):
        rospy.init_node('color_detect_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odo_sub = rospy.Subscriber('/odom', Odometry, self.odo_callback)
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.move_cmd = Twist()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.target_color = None
        self.stop_flag = False
        self.sub = LaserSub()
        self.color_ranges = {
            "green": (np.array([0, 180, 0]), np.array([25, 255, 25])),
            "blue": (np.array([0, 0, 82]), np.array([25, 25, 221])),
            "lightblue": (np.array([0, 82, 82]), np.array([25, 122, 122])),
        }
        # self.color_ranges = {
        #     "green": (np.array([50, 100, 100]), np.array([70, 255, 255])),
        #     "blue": (np.array([110, 100, 100]), np.array([130, 255, 255])),
        #     "lightblue": (np.array([165, 0, 99]), np.array([195, 250, 255])),
        # }
        self.cvbridge_interface = CvBridge()
        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.min_distance = 0.2
        self.max_distance = 0.6
        self.rate = rospy.Rate(5)
        self.m00 = 0
        self.m00_min = 1000
        self.m00_max = 8000
        self.hsv_img=None
        self.lower = (115, 224, 100)
        self.upper = (130, 255, 255)
        self.cx=0

    def odo_callback(self, odo_msg):
        orientation = odo_msg.pose.pose.orientation
        self.current_angle = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                   1 - 2 * (orientation.y ** 2 + orientation.z ** 2))

    def calculate_angle_diff(self, angle1, angle2):
        diff = angle1 - angle2
        return diff + 2 * math.pi if diff < -math.pi else (diff - 2 * math.pi if diff > math.pi else diff)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, _ = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width / 2) - (crop_width / 2))
        crop_y = int((height / 2) - (crop_height / 2))

        crop_img = cv_img[crop_y:crop_y + crop_height, crop_x:crop_x + crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)

        # Identify target color
        target_color_mask = cv2.inRange(self.hsv_img, self.lower, self.upper)
        target_color_contours, _ = cv2.findContours(target_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            if  target_color_cy>251 or  self.m00 >self.m00_max:

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
                self.move(True, True, 2, linear_vel=0)
                self.target_color = self.find_most_common_color()
                print("TARGET DETECTED: Beaconing initiated ", self.target_color)
                rospy.sleep(2)  # adjust time as needed
                # turn back to original orientation
                self.move(True, False, 2, linear_vel=0)

                rospy.sleep(2)  # adjust time as needed
                self.move(True, True, 0.3, angular_vel=0)
                # self.move(True, False, move_time, linear_vel=0)
            else:
                # Check for obstacles in front
                # If the robot is too close to an obstacle in front, it needs to avoid it
                # Move the robot
                self.move(True, True, 0.2, angular_vel=0)
                # Check for obstacles in front
                # If the robot is too close to an obstacle in front, it needs to avoid it
                if self.turn_amount!=0:
                    if self.cx >= 560-80 and self.cx <= 560+80:
                        self.move(True, True, 0.2, angular_vel=0)
                        if self.sub.min_distance< self.min_distance:
                            self.robot_controller.set_move_cmd(0.0, 0.0)
                            print("BEACONING COMPLETE: The robot has now stopped.")
                            self.ctrl_c = True
                    elif self.turn_amount<-0.3:
                        if abs(self.turn_amount)>2:
                            turn_time=abs(self.turn_amount)/13
                        else:
                            turn_time = 0.2
                        self.move(True, True, turn_time, linear_vel=0)
                    elif self.turn_amount> 0.3:
                        if abs(self.turn_amount) > 2:
                            turn_time = abs(self.turn_amount) / 13
                        else:
                            turn_time = 0.2
                        self.move(True, False, turn_time, linear_vel=0)
                    elif self.sub.min_distance< self.min_distance:
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        self.ctrl_c = True
                    else:
                        self.move(True, True, 0.1, angular_vel=0)
                elif self.sub.min_distance <= self.min_distance:
                    # Move backwards
                    self.move(False, True, 0.7, angular_vel=0)
                    # If the distance on the right side is also close, adjust to the left
                    if self.sub.min_left >= self.sub.min_right:
                        self.move(True, True, move_time, linear_vel=0)
                    # If the distance on the left side is also close, adjust to the right and turn right
                    elif self.sub.min_left <= self.sub.min_right:
                        self.move(True, False, move_time, linear_vel=0)
                elif self.sub.min_distance <= self.max_distance:
                    if self.sub.min_left >= self.sub.min_right :
                        self.move(True, True, move_time, linear_vel=0)
                    # If the distance on the left side is also close, adjust to the right and turn right
                    elif self.sub.min_left <= self.sub.min_right :
                        self.move(True, False, move_time, linear_vel=0)
                else:
                    if self.min_distance>0.8:
                        self.move(True, True, 0.4,linear_vel=0.7, angular_vel=0)
                    else:
                        self.move(True, True, 0.1, angular_vel=0)


            if time.time() >= t_end:
                print("Time is up")
                self.ctrl_c = True


    def move(self, forward_backward, left_right, seconds, linear_vel=0.2, angular_vel=0.6):
        vel = Twist()
        t_end = time.time() + seconds
        if forward_backward:
            print("forward")
            vel.linear.x = linear_vel
            if left_right:
                print("left")
                vel.angular.z = angular_vel
            else:
                print("right")
                vel.angular.z = -angular_vel
        else:
            print("backward")
            vel.linear.x = -linear_vel
            if left_right:
                print("left")
                vel.angular.z = -angular_vel
            else:
                print("right")
                vel.angular.z = angular_vel

        while time.time() < t_end:
            self.pub.publish(vel)
        print(seconds, "seconds have elapsed")

if __name__ == '__main__':
    search_instance = colour_search()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass
