#!/usr/bin/env python3

from logging import shutdown
from multiprocessing.sharedctypes import Value
import rospy
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pi

class FigureEight():
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
                                                          'sxyz')[2])*(180/pi)]
            self.zero_reading_found = True

        # Output robot position and angle information
        print(f"x={(abs(position.x) - abs(self.init_position[0])):.2f} [m], y={(abs(position.y) - abs(self.init_position[1])):.2f} [m], yaw={((abs(euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')[2]) *(180/pi)) - abs(self.init_position[2])):.1f} [degrees]")

    def __init__(self):
        self.node_name = "figure_eight"

        # Initialise variables needed for calibration
        self.zero_reading_found = False
        self.init_position = []

        self.vel = Twist()
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        rospy.init_node(self.node_name, anonymous=True)

        # Information published at 1Hz
        self.rate = rospy.Rate(1) 
        self.ctrl_c = False 
        self.current_loop = 1

        rospy.sleep(0.01)
        self.start_time = rospy.get_rostime()

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(f"Task 1: '{self.node_name}' node is active...") 

    def shutdown(self):
        print(f"Stopping the '{self.node_name}' task 1 node at: {rospy.get_time()}")
        self.pub.publish(Twist())
        self.ctrl_c = True

    def main_loop(self):
        while not self.ctrl_c:
            self.vel.linear.x = (pi * 1) / 30 # m/s ((pi * 1 metre diameter) / 30 seconds)
            # Determine the direction of angular velocity for each loop
            if self.current_loop == 1:
                self.vel.angular.z = (2 * pi) / 30 # rad/s
            elif self.current_loop == 2:
                self.vel.angular.z = - (2 * pi) / 30 # rad/s

            if (rospy.get_rostime().secs - self.start_time.secs) > 31: # originally 29.5 seconds in simulation, adjusted for real robot
                self.current_loop = 2
            if (rospy.get_rostime().secs - self.start_time.secs) > 62: # originally 60 seconds in simulation, adjusted for real robot
                self.shutdown()

            rospy.sleep(0.001)
            self.pub.publish(self.vel)

if __name__ == '__main__': 
    node = FigureEight() 
    try:
        node.main_loop() 
    except rospy.ROSInterruptException:
        pass