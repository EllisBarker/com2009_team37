#! /usr/bin/env python3
# task2_client.py

from tb3 import Tb3LaserScan, Tb3Move, Tb3Odometry
import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback
from math import pi

class SearchActionClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):
        # Print all the data from TB3
        # Get the current distance travelled, from the feedback message
        # and assign this to a class variable...
        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            print(f"Feedback: Distance travelled = {self.distance:.3f} meters.")
            print(f"Robot Yaw: {self.tb3_odom.yaw} degrees.")
            self.msg_counter = 0
        else:
            self.msg_counter += 1

        # turning 
        # goasl must be different
        self.turnRight = True
        if 0<self.tb3_scan.min_distance< 0.5 and self.turnRight:
            self.goal.approach_distance = 0.05 # m 
            self.goal.fwd_velocity = -0.1 # m/s
            self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
            self.stopTurnRight = True
            self.turnRight = False
        # moving 
        if self.msg_counter2 > 2.5:
            self.goal.approach_distance = 0.1 # m 
            self.goal.fwd_velocity = 0.15 # m/s
            self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
            self.msg_counter2 = 0
            self.turnRight = True
        else:
            self.msg_counter2 += 1


    def __init__(self):
        self.distance = 0.0
        self.msg_counter = 0
        self.msg_counter2 = 0

        self.action_complete = False
        rospy.init_node("search_action_client")
        self.rate = rospy.Rate(1)

        ## TODO: setup a "simple action client" with a callback function
        ## and wait for the server to be available...
        self.client = actionlib.SimpleActionClient(
            "/minh_search_server",
            SearchAction
        )
        self.client.wait_for_server()
        
        # create the instance of tb3 functions
        self.tb3_move = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_scan = Tb3LaserScan()


        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            ## TODO: cancel the goal request, if this node is shutdown before the action has completed...
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")

        ## TODO: Print the result here...
        rospy.sleep(1) # Wait for the result to come in 
        result = self.client.get_result()
        print("RESULT:")
        print(f"  * Action State = {self.client.get_state()}")
        print(f"  * Total Distance Travelled = {result.total_distance_travelled:.3f} meters")
        print(f"  * Closest Object Distance = {result.closest_object_distance:.3f} meters")
        print(f"  * Closest Object Angle = {result.closest_object_angle:.1f} degrees.")



    def main_loop(self):
        ## TODO: assign values to all goal parameters
        ## and send the goal to the action server...
        self.goal.approach_distance = 0.4 # m 
        self.goal.fwd_velocity = 0.1 # m/s
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)


        # FOR ASSIGNMENT: This is the part for making the robot runs continuously (instead of stopping after not detecting any obstacle within 2 meters like the code below)
        while self.client.get_state() < 2:
            ## TODO: Construct an if statement and cancel the goal if the 
            ## distance travelled exceeds 2 meters...
            
            #useless
            # if self.distance > 2:
            #     print("STOP: Distance exceeded 2 meters!!!")
            #     # break out of the while loop to stop the node:
            #     break

            self.rate.sleep()

        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    ## TODO: Instantiate the node and call the main_loop() method from it...
    node = SearchActionClient()
    node.main_loop()

### LEGACY CODE
# import rospy
# import actionlib

# from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

# class SearchActionClient():
#     goal = SearchGoal()

#     def __init__(self):
#         self.action_complete = False
#         rospy.init_node("search_action_client")
#         self.rate = rospy.Rate(1)

#         self.client = actionlib.SimpleActionClient("/task_2_search", SearchAction)
#         self.client.wait_for_server()
#         rospy.on_shutdown(self.shutdown_ops)

#     def feedback_callback(self, feedback_data: SearchFeedback):
#         pass

#     def shutdown_ops(self):
#         if not self.action_complete:
#             rospy.logwarn("Received shutdown request. Cancelling...")
#             self.client.cancel_goal()
#             rospy.logwarn("Cancelled...")

#         rospy.sleep(1)
#         result = self.client.get_result()
#         # print results here if need be?

#     def main_loop(self):
#         self.goal.approach_distance = 0.4 # metres
#         self.goal.fwd_velocity = 0.15 # m/s
#         self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
        
# if __name__ == '__main__':
#     node = SearchActionClient()
#     node.main_loop()