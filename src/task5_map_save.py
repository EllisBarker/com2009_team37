#!/usr/bin/env python3

import roslaunch
import rospy
from pathlib import Path

map_path = Path.home().joinpath("catkin_ws/src/com2009_team37/maps/task_5")

rospy.init_node("map_getter", anonymous=True)
start_time = rospy.get_rostime()

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

while (rospy.get_rostime().secs - start_time.secs) < 180:
    print(f"Saving map at time: {rospy.get_time()}...")
    node = roslaunch.core.Node(package="map_server",
                               node_type="map_saver",
                               args=f"-f {map_path}")
    process = launch.launch(node)
