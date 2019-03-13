#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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

class Markers:

    def __init__(self):
        rospy.init_node('bot_marker', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0

        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()

        self.mark_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.markerArray = MarkerArray()

        # subscribers
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # if using rviz, we can subscribe to nav goal click
        if rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:

            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta

    def loop(self):

        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        marker = Marker()
        marker.header.frame_id = "/odom" #'/map' or '/odom'?
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.header.stamp = rospy.get_rostime()
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = self.theta
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0
        marker.id = 2

        arrow = Marker()
        arrow.header.frame_id = "/base_link" #'/map' or '/odom'?
        arrow.type = marker.ARROW
        arrow.action = marker.ADD
        arrow.header.stamp = rospy.get_rostime()
        arrow.scale.x = 1
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.a = 0.5
        arrow.color.r = 1
        arrow.color.g = 0
        arrow.color.b = 0
        arrow.pose.orientation.w = self.theta
        arrow.id = 3

        #self.markerArray.markers.append(marker)
        #self.markerArray.markers.append(arrow)

        # Publish the MarkerArray
        self.mark_publisher.publish(marker)
        self.mark_publisher.publish(arrow)
        #self.mark_publisher.publish(self.markerArray)



    ############ your code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    marks = Markers()
    marks.run()
