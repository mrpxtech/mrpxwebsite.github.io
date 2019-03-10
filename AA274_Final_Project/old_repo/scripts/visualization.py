#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

class Visualization:

    def __init__(self):
        rospy.init_node('robot_visualization', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        self.visual_publisher = rospy.Publisher('robot_visualization', Marker, queue_size=10)

    def loop(self):
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]

                marker = Marker()
                marker.header.frame_id = '/base_footprint'
                marker.header.stamp = rospy.Time.now();
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.pose.position.x = self.x;
                marker.pose.position.y = self.y;
                # marker.pose.position.z = 0;
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.w = rotation[3]
                marker.pose.position.x = rotation[0]
                marker.pose.position.y = rotation[1]
                marker.pose.position.z = rotation[2]

                marker.lifetime = rospy.Duration();

                # Renumber the marker IDs
                marker.id = 0

                # Publish the MarkerArray
                self.visual_publisher.publish(marker)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    vis = Visualization()
    vis.run()
