#!/usr/bin/env python3

import rospy

# Imaging modules
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Movement and sensor modules
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Need SLAM modules??????

class Exploration():
    def __init__(self):
        node_name = "exploration"
        rospy.init_node(node_name)

        # Camera-related functionality
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = Tb3Move()

        # Shutdown-related operations
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        
        # Colour-related values
        self.m00 = 0
        self.m00_min = 10000
        self.target_colour = rospy.get_param('/task5/target_colour')
        rospy.loginfo(f"TASK 5 BEACON: The target is {self.target_colour}.")

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        # PICK RANGE BASED ON INLINE ARGUMENT FOR COLOUR

    def main(self):
        while not self.ctrl_c:
            pass
            # IF DESIRED OBJECT DETECTED, TURN AND TRY TO APPROACH DETECTED

if __name__ == '__main__':
    exploration_instance = Exploration()
    try:
        exploration_instance.main()
    except rospy.ROSInterruptException:
        pass