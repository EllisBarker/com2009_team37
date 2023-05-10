#!/usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
from geometry_msgs.msg import Twist
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
import time
# Import some other modules from within this package
from tb3 import Tb3Move


class ColorSearch(object):

    def __init__(self):
        node_name = "color_search"
        topic_name="cmd_vel"
        rospy.init_node(node_name)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
                                                  Image, self.camera_callback, queue_size=1)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.move_rate = "fast"  # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        # color threshold
        self.lower = (115, 224, 100)
        self.upper = (130, 255, 255)
        # minimum moment value to consider a blob detected
        self.m00_min = 10000

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        print("callback")
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
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv_img, self.lower, self.upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        m = cv2.moments(mask)
        self.m00 = m["m00"]
        self.cy = m["m10"] / (m["m00"] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        print(crop_img.shape)
        # cv2.imshow("cropped image", crop_img)
        # cv2.waitKey(1)

    def main(self):
        while not self.ctrl_c:
            self.move(True, True, 0.2, angular_vel=0)
            if self.stop_counter > 0:
                self.stop_counter -= 1

                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560 - 100 and self.cy <= 560 + 100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            self.stop_counter = 30
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'

                if self.move_rate == 'fast':
                    print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print(
                        f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop' and self.stop_counter > 0:
                    print(
                        f"STOPPED: The blob of colour is now dead-ahead at y-position {self.cy:.0f} pixels... Counting down: {self.stop_counter}")
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                else:
                    print(
                        f"MOVING SLOW: A blob of colour of size {self.m00:.0f} pixels is in view at y-position: {self.cy:.0f} pixels.")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

                self.robot_controller.publish()
                self.rate.sleep()

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
    search_instance = ColorSearch()
    try:
        search_instance.main()
    except rospy.ROSInterruptException:
        pass

