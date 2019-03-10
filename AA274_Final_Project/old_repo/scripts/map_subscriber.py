#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid
from asl_turtlebot.msg import DetectedObject
import tf
import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

def map_callback(msg):
    x_origin = msg.info.origin.position.x
    y_origin = msg.info.origin.position.y
    grid_res = msg.info.resolution # m/cell
    row_ind = grid_res*np.arange(msg.info.width) + x_origin
    col_ind = grid_res*np.arange(msg.info.height) + y_origin
    x_coord, y_coord = np.meshgrid(row_ind, col_ind, sparse=True)
    grid_data = np.resize(msg.data, (msg.info.height, msg.info.width))
    grid_data[grid_data < 0] = -10  # for coloring purposes of unexplored area
    grid_data[grid_data > 0] = 10  # for coloring purposes of explored area
    plt.pcolormesh(x_coord, y_coord, grid_data, cmap=plt.get_cmap('Greys'))
    plt.pause(10)  # this will give an error in ros - uncomment afterwards



def listener():
    rate = rospy.Rate(10) # 10 Hz
    # do stuff
    rate.sleep()

def initialize_food_marker():
    puddle_marker = Marker()
    puddle_marker.header.frame_id = "/puddle"
    puddle_marker.ns = "ellipse"
    puddle_marker.type = Marker.LINE_STRIP
    puddle_marker.scale.x = 0.01
    puddle_marker.frame_locked = True
    puddle_marker.color.g = 1
    puddle_marker.color.a = 1
    return puddle_marker

# function that publishes message to visualizer to mark location
def map_to_visualizer():

pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
# command vel (used for idling)
self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# subscribers
# stop sign detector
rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
# high-level navigation pose
rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)


# if using rviz use pose locations manually entered
if rviz:
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        

if __name__ == '__main__':
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    while not rospy.is_shutdown():
        listener()
