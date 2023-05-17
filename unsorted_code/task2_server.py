#! /usr/bin/env python3
# task2_server.py

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib
import random

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3.py
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class SearchActionServer():
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        # Create a "simple action server" with a callback function, and start it...
        self.actionserver = actionlib.SimpleActionServer(
            "/minh_search_server", 
            SearchAction, 
            self.action_server_launcher, 
            auto_start=False
        )
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The 'Search Action Server' is active...")

    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        ## TODO: Implement some checks on the "goal" input parameter(s)
        success = True
        vel = goal.fwd_velocity # m/s
        dist = goal.approach_distance # m

        # if vel > 0.26 or vel < 0:
        #     print("Invalid Velocity!")
        #     success = False
        # if dist < 0.2:
        #     print("Invalid Distance!")
        #     success = False


        if not success:
            # Abort the action server if an invalid goal has been requested...
            self.result.total_distance_travelled = -1
            self.result.closest_object_distance = -1
            self.result.closest_object_angle = -1
            self.actionserver.set_aborted(self.result)
            return

        # Print a message to indicate that the requested goal was valid
        print(f"Search goal received: fwd_velocity = {vel} m/s, approach_distance = {dist} m.")

        # Get the robot's current odometry from the Tb3Odometry() class:
        self.posx0 = self.tb3_odom.posx
        self.posy0 = self.tb3_odom.posy
        # Get information about objects up ahead from the Tb3LaserScan() class:
        self.closest_object = self.tb3_lidar.min_distance
        self.closest_object_location = self.tb3_lidar.closest_object_position

        ## TODO: set the robot's forward velocity (as specified in the "goal")...
        if goal.fwd_velocity<0:
            self.v = random.random()
            if self.v > 0.9:
                self.vel_controller.set_move_cmd(linear=0,angular=1.0)
            else:
                self.vel_controller.set_move_cmd(linear=0,angular=-1.0)
        else:

            self.vel_controller.set_move_cmd(linear=vel,angular=0)

        ## TODO: establish a conditional statement so that the  
        ## while loop continues as long as the distance to the closest object
        ## ahead of the robot is always greater than the "approach distance"
        ## (as specified in the "goal")...
        while self.tb3_lidar.min_distance > dist:
            # update LaserScan data:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            # Publish a velocity command to make the robot start moving 
            self.vel_controller.publish()

            # determine how far the robot has travelled so far:
            self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

            self.result.total_distance_travelled = self.distance
            self.result.closest_object_distance = self.closest_object
            self.result.closest_object_angle = self.closest_object_location
            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                # Take appropriate action if the action is cancelled (pre-empted)...
                print("Pre-empt requested!")
                self.actionserver.set_preempted(self.result)
                self.vel_controller.stop()
                success = False
                # Exit the loop:
                break

            # Update all feedback message values and publish a feedback message:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)

            rate.sleep()

        if success:
            rospy.loginfo("approach completed successfully.")
            ## TODO: Set the action server to "succeeded" and stop the robot...
            self.vel_controller.stop()
            self.actionserver.set_succeeded(self.result)



if __name__ == '__main__':
    rospy.init_node("search_action_server")
    SearchActionServer()
    rospy.spin()

### LEGACY CODE
# import rospy
# import actionlib
# import random # Please ask the lecturer if we can use this library
# from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal
# from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan
# from math import sqrt, pow

# class SearchActionServer():
#     feedback = SearchFeedback() 
#     result = SearchResult()

#     def __init__(self):
#         self.actionserver = actionlib.SimpleActionServer("/task_2_search", SearchAction, self.action_server_launcher, auto_start=False)
#         self.actionserver.start()

#         # useful publisher/subscriber functions from tb3
#         self.vel_controller = Tb3Move()
#         self.tb3_odom = Tb3Odometry()
#         self.tb3_lidar = Tb3LaserScan()

#         rospy.loginfo("The 'Search Action Server' is active...")

#     # the 'callback' function of the server
#     def action_server_launcher(self, goal: SearchGoal):
#         rate = rospy.Rate(10)

#         success = True
#         vel = goal.fwd_velocity
#         dist = goal.approach_distance

#         self.posx0 = self.tb3_odom.posx
#         self.posy0 = self.tb3_odom.posy
#         self.closest_object = self.tb3_lidar.min_distance
#         self.closest_object_location = self.tb3_lidar.closest_object_position

#         # self.vel_controller.set_move_cmd(vel, 0.0)

#         # while true:
#             # if not close:
#                 # update values and publish feedback
#             # else if close:
#                 # publish velocity commands, update values
#                 ###(CHANGE ANGULAR VELOCITY (also make linear 0) AND TIME OF ROTATION)

#         ################# Minh's code (Modify if needed)

#         # Get start time (ideally 0s)
#         start_time = rospy.get_rostime() 
#         # Robot runs for 85 seconds
#         while (rospy.get_rostime().secs - start_time.secs) < 85: 
#             # While the distance between the robot and the obstacle is more than the threshold, move forward
#             if self.closest_object > dist: 
#                 self.vel_controller.set_move_cmd(vel, 0.0)
#             # Stop the robot, make it rotate and move forward after the distance between the 
#             # robot and the obstacle is smaller than the threshold
#             else:
#                 # Random duration of turning, ideally ranged between 1 to 3 seconds to avoid wasting time on rotating
#                 random_duration = random.randrange(1,3) 
#                 # Capture the moment that the robot stops when it's close to an obstacle
#                 time_stopped = rospy.get_rostime()
#                 # Make it rotate for that duration, with a random positive angular velocity
#                 while (rospy.get_rostime().secs - time_stopped.secs) < random_duration:
#                     self.vel_controller.set_move_cmd(0.0, random.random()) 
#                 # Set angular velocity back to 0 after finish rotating and keep moving forward
#                 self.vel_controller.set_move_cmd(vel, 0.0)

            
#         ################# End of Minh's code
#         if success:
#             rospy.loginfo("Search completed successfully.")


# if __name__ == '__main__':
#     rospy.init_node("search_action_server")
#     SearchActionServer()
#     rospy.spin()