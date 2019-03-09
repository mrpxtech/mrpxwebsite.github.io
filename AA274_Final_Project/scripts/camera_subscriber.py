#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from asl_turtlebot.msg import DetectedObject
import tf
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

def camera_callback(msg):
    image = msg.data  # compressed image rgb
    plt.imshow(image)

def listener():
    rate = rospy.Rate(10)  # necessary only from testing, realistically higher rate
    # do stuff
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('/camera_relay/image/compressed', CompressedImage, camera_callback)
    while not rospy.is_shutdown():
        listener()
