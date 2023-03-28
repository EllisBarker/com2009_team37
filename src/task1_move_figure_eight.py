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

        self.x = position.x
        self.y = position.y

        # Define reference measurements immediately to calibrate future readings
        if self.zero_reading_found == False:
            self.init_position = [position.x, position.y, 
                                  (euler_from_quaternion([orientation.x, orientation.y, 
                                                          orientation.z, orientation.w], 
                                                          'sxyz')[2])*(180/pi)]
            self.zero_reading_found = True
            self.x0 = self.x
            self.y0 = self.y

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

        self.x0 = 0
        self.y0 = 0
        self.x = 0
        self.y = 0
        self.distance_travelled = 0

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

            # Change what loop is being travelled across based on the distance travelled (1 circumference or 2 or more)
            try:
                self.distance_travelled += (sqrt(((self.x - self.x0)**2) + ((self.y - self.y0)**2)))
            except ValueError:
                self.distance_travelled += 0
            self.x0 = self.x
            self.y0 = self.y
            if self.distance_travelled >= pi and self.distance_travelled < 2 * pi:
                self.current_loop = 2
            elif self.distance_travelled > 2 * pi:
                self.shutdown()

            self.pub.publish(self.vel)
            self.rate.sleep()

if __name__ == '__main__': 
    node = FigureEight() 
    try:
        node.main_loop() 
    except rospy.ROSInterruptException:
        pass