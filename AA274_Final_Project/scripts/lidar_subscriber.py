#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from asl_turtlebot.msg import DetectedObject
import tf
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

def lidar_callback(msg):
    # (is_bigendian, point_step, row_step, data, is_dense) = msg.fields  # note that the data is not calibrated for distance
    laser_data = np.zeros( (msg.width, 5) )
    i = 0
    for point in pc2.read_points(msg, skip_nans=True):
        laser_data[i,0] = point[0]
        laser_data[i,1] = point[1]
        laser_data[i,2] = point[2]
        laser_data[i,3] = point[3]
        laser_data[i,4] = point[4]
        i = i + 1

    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = laser_data[:,0], laser_data[:,1], laser_data[:,2]
    ax.scatter(xs, ys, zs, c='Blue')
    plt.pause(0.001)

def listener():
    rate = rospy.Rate(10)  # necessary only from testing, realistically higher rate
    # do stuff
    rate.sleep()

if __name__ == '__main__':
    rospy.init_node('lidar_subscriber', anonymous=True)
    rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback)
    while not rospy.is_shutdown():
        listener()
