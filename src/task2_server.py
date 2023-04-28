#!/usr/bin/env python3

import rospy
import actionlib
import random # Please ask the lecturer if we can use this library
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/task_2_search", SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # useful publisher/subscriber functions from tb3
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # the 'callback' function of the server
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        vel = goal.fwd_velocity
        dist = goal.approach_distance

        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        # self.vel_controller.set_move_cmd(vel, 0.0)

        # while true:
            # if not close:
                # update values and publish feedback
            # else if close:
                # publish velocity commands, update values
                ###(CHANGE ANGULAR VELOCITY (also make linear 0) AND TIME OF ROTATION)

        ################# Minh's code (Modify if needed)

        # Get start time (ideally 0s)
        start_time = rospy.get_rostime() 
        # Robot runs for 85 seconds
        while (rospy.get_rostime().secs - start_time.secs) < 85: 
            # While the distance between the robot and the obstacle is more than the threshold, move forward
            if self.closest_object > goal.approach_distance: 
                self.vel_controller.set_move_cmd(vel, 0.0)
            # Stop the robot, make it rotate and move forward after the distance between the 
            # robot and the obstacle is smaller than the threshold
            else:
                # Random duration of turning, ideally ranged between 1 to 3 seconds to avoid wasting time on rotating
                random_duration = random.randrange(1,3) 
                # Capture the moment that the robot stops when it's close to an obstacle
                time_stopped = rospy.get_rostime()
                # Make it rotate for that duration, with a random positive angular velocity
                while (rospy.get_rostime().secs - time_stopped.secs) < random_duration:
                    self.vel_controller.set_move_cmd(0.0, random.random()) 
                # Set angular velocity back to 0 after finish rotating and keep moving forward
                self.vel_controller.set_move_cmd(vel, 0.0)

            
        ################# End of Minh's code
        if success:
            rospy.loginfo("Search completed successfully.")


if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()