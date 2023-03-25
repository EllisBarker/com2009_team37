#!/usr/bin/env python3

import rospy, math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class OdomSubscriber():
    def __init__(self):
        node_name = "odom_subscriber"
        rospy.init_node(node_name, anonymous=True)
        # Initialise variables needed for calibration
        self.zero_reading_found = False
        self.init_position = []
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.loginfo(f"Task 1: '{node_name}' node is active...")

    def callback(self, topic_data: Odometry):
        # Extract Odometry data
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation 

        # Define reference measurements immediately to calibrate future readings
        if self.zero_reading_found == False:
            self.init_position = [position.x, position.y, 
                                 (euler_from_quaternion([orientation.x, orientation.y, 
                                                         orientation.z, orientation.w], 
                                                         'sxyz')[2])*(180/math.pi)]
            self.zero_reading_found = True

        # Output robot position and angle information
        print(f"x={(position.x - abs(self.init_position[0])):.2f} [m], y={(position.y - abs(self.init_position[1])):.2f} [m], yaw={((euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')[2]) *(180/math.pi) - abs(self.init_position[2])):.1f} [degrees]")

    def main_loop(self):
        # Subscriber node remains active until closed
        rospy.spin()

if __name__ == '__main__':
    subscriber_instance = OdomSubscriber()
    subscriber_instance.main_loop()