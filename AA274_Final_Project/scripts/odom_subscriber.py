#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

def odom_callback(msg):
    Pass

def loop():
    try:
        rate = rospy.Rate(10) # 10 Hz
        trans_listener = tf.TransformListener()
        origin_frame = "/odom"
        (translation,rotation) = trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        theta = euler[2]
        # do stuff with x, y, and theta
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass

if __name__ == '__main__':
    rospy.init_node('odom_subscriber', anonymous=True)
    while not rospy.is_shutdown():
        loop()
