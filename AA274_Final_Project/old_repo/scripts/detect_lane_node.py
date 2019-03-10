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

# added from lane_detection.py
import cv2
import matplotlib.image as mpimg
import glob
from cv_bridge import CvBridge
from lane_detection import *

class Detect_Lane:

    def __init__(self):
        self.result_image = 0
        self.image_available = 0
        self.deviation = 0
        # initialization files
        self.used_warped = plt.imread('./init.png')
        self.used_warped = apply_color_threshold(used_warped)
        self.used_ret = eval(open('./init.txt').read())

        self.br = CvBridge()

        rospy.init_node('detect_lane_node', anonymous=True)
        self.sub = rospy.Subscriber('/camera_relay/image/compressed', CompressedImage, self.camera_callback)
        self.pub = rospy.Publisher('/detect_lanes/image/compressed', CompressedImage, queue_size=0) # FIFO

    def camera_callback(self, msg):
        # convert compressed image to cv2 image
        image_np = self.br.compressed_imgmsg_to_cv2(msg)
        result, self.used_warped, self.used_ret, self.deviation = process_image(image_np, self.used_warped, self.used_ret)

        # create compressed image
        self.result_image = np.array(cv2.imencode('.jpg', result)[1]).tostring()
        self.image_available = 1

    def loop(self):
        rate = rospy.Rate(10)  # necessary only from testing, realistically higher rate
        if self.image_available == 1:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            msg.data = self.result_image
            self.pub.publish(msg)
        rate.sleep()

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Detect_Lane()
    sup.run()
