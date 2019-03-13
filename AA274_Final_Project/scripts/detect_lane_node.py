#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
import math

from std_msgs.msg import Float32MultiArray, String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from enum import Enum
from rospy_tutorials.msg import Floats

# added from lane_detection_simple.py
import cv2
import glob
from cv_bridge import CvBridge
from lane_detection_simple import *

class Detect_Lane:

    def __init__(self):
        self.result_image = 0
        self.image_available = 0
        self.lane_deviation = 0.0
        self.road_side_flag = 'none'

        # initialization files, make sure to include them
        self.used_warped = plt.imread('/home/AA274_Final_Project/init_data/init.png')
        self.used_warped = apply_color_threshold(self.used_warped)
        self.used_ret = eval(open('/home/AA274_Final_Project/init_data/init.txt').read())

        # command velocities from astar
        self.x_vel = 0
        self.y_vel = 0
        self.kw = 5.0  # k gain for the lane deviation for rotation

        self.mode = 'off'  # set mode to on for lane correction, off for no lane correction

        self.br = CvBridge()

        rospy.init_node('detect_lane_node', anonymous=True)
        self.cam_sub = rospy.Subscriber('/camera_relay/image/compressed', CompressedImage, self.camera_callback)  # sub to camera
        self.nav_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)  # sub for cmd vel
        self.switch_sub = rospy.Subscirber('/lane_mode', String, self.switch_callback)  # sub for lane mode control

        self.pub_image = rospy.Publisher('/detect_lanes/image/compressed', CompressedImage, queue_size=1)
        self.pub_lane_deviaton = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def switch_callback(self, msg):
        self.mode = msg.data

    def vel_callback(self, msg):
        self.x_vel = msg.linear.x
        self.y_vel = msg.linear.y
        # self.w = msg.data.angular.z

    def camera_callback(self, msg):
        # convert compressed image to cv2 image
        image_np = self.br.compressed_imgmsg_to_cv2(msg,'rgb8')
        result, self.used_warped, self.used_ret, self.lane_deviation, self.road_side_flag, self.lane_flag = \
        process_image(image_np, self.used_warped, self.used_ret)

        # create compressed image
        self.result_image = np.array(cv2.imencode('.jpg', result)[1]).tostring()
        self.image_available = 1

    def loop(self):
        if self.image_available == 1:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpeg'
            msg.data = self.result_image
            self.pub_image.publish(msg)

            # reorient if left lane
            if self.mode == 'on' and self.road_side_flag != 'none' and self.lane_flag != 'dashed':
                twist = Twist()
                twist.linear.x = self.x_vel
                twist.linear.y = self.y_vel
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.kw*(self.lane_deviation)
                self.pub_lane_correction.publish(twist)

    def run(self):
        rate = rospy.Rate(50) # 50 Hz, 5x the astar rate
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Detect_Lane()
    sup.run()
