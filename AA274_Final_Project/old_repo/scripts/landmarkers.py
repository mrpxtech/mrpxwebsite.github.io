#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String
import tf
from visualization_msgs.msg import Marker
import numpy as np

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

print "pose_controller settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping
print "rviz = %s\n" % rviz

class RVmarker:

    def __init__(self):
        rospy.init_node('Rviz_markers', anonymous=True)
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.trans_listener = tf.TransformListener()

    def loop(self):    
        ##Subscribe to location of robot
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                # print('no gazebo')
                # print(self.x)
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        

        # current state used for testing
        # self.x = 0.0
        # self.y = 0.0
        # self.theta = 0.0


        # goal position subscriber
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.markerstate)
        # if using rviz, we can subscribe to nav goal click
        if rviz: # get goal location
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        #Publishers
        # # command pose for controller
        self.publisheruno = rospy.Publisher('/my_location', Marker, queue_size=10)

        self.publisherdos = rospy.Publisher('/my_goalocation', Marker, queue_size=10)

        # self.publishertres = rospy.Publisher('/my_heading', Marker, queue_size=10)

    #implemed markerspace for when using gazebo

    def markerstate_callback(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.id=1
        marker.header.stamp=rospy.Time.now()
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration.from_sec(10.0)
        diameter=0.15
        # marker.scale = Vector3(diameter, diameter, 0.01)

        #add messenger for gazebo

        origin_frame = "/map" if mapping else "/odom"
        (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
        self.x = translation[0]
        self.y = translation[1]
        # print('no gazebo')
        euler = tf.transformations.euler_from_quaternion(rotation)
        self.theta = euler[2]


        marker.pose.orientation.w = self.theta
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        # self.publisheruno.publish(marker) 

        return marker

        
    def rviz_goal_callback(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.id=2
        marker.header.stamp=rospy.Time.now()
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        #self provided
        self.theta_g=0.0
        self.x_g=2.0
        self.y_g=2.0

        marker.pose.orientation.w = self.theta_g
        marker.pose.position.x = self.x_g
        marker.pose.position.y = self.y_g

        return marker

    #do later heading implementation
    # def arrow_callback(self):
    #     marker = Marker()
    #     marker.header.frame_id = "/map"
    #     marker.type = marker.ARROW
    #     marker.action = marker.ADD
    #     marker.id=3
    #     marker.header.stamp=rospy.Time.now()
    #     marker.scale.x = 0.2
    #     marker.scale.y = 0.2
    #     marker.scale.z = 0.2
    #     marker.color.a = 1.0
    #     marker.color.r = 2.0
    #     marker.color.g = 1.0
    #     marker.color.b = 1.0

    #     x=self.x*np.cos(self.theta)
    #     y=self.y*np.sin(self.theta)

    #     marker.pose.orientation.w = self.theta
    #     marker.pose.position.x = x
    #     marker.pose.position.y = y

    #     return marker
        


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():           
            self.loop()
            rate.sleep()
            self.publisheruno.publish(self.markerstate_callback())

            self.publisherdos.publish(self.rviz_goal_callback())

            # self.publishertres.publish(self.arrow_callback())

if __name__ == '__main__':
    mark = RVmarker()
    mark.run()
