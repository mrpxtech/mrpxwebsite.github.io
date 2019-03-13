#!/usr/bin/env python

import rospy
import os
# watch out on the order for the next two imports lol
from tf import TransformListener,TransformBroadcaster
import tf as TF
import geometry_msgs.msg
import turtlesim.srv
# import tensorflow as tf
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math

def run(frames, pose, br):
    ### edited by william, added continuous tf broadcaster
    for i in range(len(frames)):
        frame_name = frames[i]
        frame_pose = pose[i]
        br.sendTransform(frame_pose, TF.transformations.quaternion_from_euler(0, 0, 0), rospy.get_rostime(), "/" + frame_name, "/map")


if __name__=='__main__':
    rospy.init_node('test_frame', anonymous=True)

    br = TransformBroadcaster()
    # create test frames
    frames = ['apple', 'banana', 'orange']
    pose = [(3, 2, 0), (3, 0.3, 0), (3.3, 1.2, 0)]

    # added by william
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        run(frames, pose, br)
        rate.sleep()  # may want to do rospy.spinOnce() OR nothing at all
