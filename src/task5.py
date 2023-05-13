#!/usr/bin/env python3

import rospy, roslaunch

# Imaging modules
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Movement and sensor modules
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

class Exploration():
    def __init__(self):
        node_name = "exploration"
        rospy.init_node(node_name)

        # Camera-related functionality
        # "/camera/rgb/image_raw" for simulation
        self.camera_topic = "/camera/rgb/image_raw"
        self.camera_subscriber = rospy.Subscriber(self.camera_topic,
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        # Map-related functionality
        self.map_path = "com2009_team37/maps/task5_map"

        # Movement and sensor controllers
        self.robot_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        # Shutdown-related operations
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(100)
        
        # Colour-related values
        self.m00 = 0
        self.m00_min = 10000
        self.target_colour = rospy.set_param('/task5/target_colour')
        rospy.loginfo(f"TASK 5 BEACON: The target is {self.target_colour}.")

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def save_image(self):
        rospy.init_node("map_getter", anonymous=True)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        print(f"Saving map at time: {rospy.get_time()}...")
        node = roslaunch.core.Node(package="map_server",
                                   node_type="map_saver",
                                   args=f"-f {self.map_path}")
        process = launch.launch(node)

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        # PICK RANGE BASED ON INLINE ARGUMENT FOR COLOUR

    def main(self):
        while not self.ctrl_c:
            self.save_image()
            # IF DESIRED OBJECT DETECTED, TURN AND TRY TO APPROACH DETECTED

if __name__ == '__main__':
    exploration_instance = Exploration()
    try:
        exploration_instance.main()
    except rospy.ROSInterruptException:
        pass