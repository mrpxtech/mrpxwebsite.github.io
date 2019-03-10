#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert
 
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

#-------------------------data required------------------------
#topic lane from module control
#topic max_vel from module control


 
 #  <remap from="/control/cmd_vel" to="/cmd_vel" />


#<remap from="/control/lane" to="/detect/lane" />
#self.pub_lane = rospy.Publisher('/detect/lane', Float64, queue_size = 1)
 #self.pub_lane.publish(msg_lane_position)  
 # this is the the center lane position


class ControlLane():
    def __init__(self):
        # subscribe to lane_position topic,float64 message type
        # create instance of class Subscriber called sub_lane, call callback_FollowLane
        self.sub_lane = rospy.Subscriber('lane_position', Float64, self.callback_FollowLane, queue_size = 1)

        # subscribe to max velocity, from control module max_vel topic, float64 message type
        # create instance of class Subscriber called sub_max_vel, call callback_GetMaxVel
        '''self.sub_max_vel = rospy.Subscriber('max_vel', Float64, self.callback_GetMaxVel, queue_size = 1)'''
        
        # publish command velocity, to control module cmd_vel topic, Twist geometry message type
        # create instance of class Publisher called pub_cmd_vel
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        

        self.lastError = 0 # initialize error to zero
        self.MAX_VEL = 0.2 # initialize max velocity to 0.12

        rospy.on_shutdown(self.fnShutDown) # what to execute when node is shutdown, this will stop the robot
        # so it doesn't keep moving after the node is shutdown
    '''
    def callback_GetMaxVel(self, max_vel_msg): #where is this data coming from?
       self.MAX_VEL = max_vel_msg.data # why do we need .data here?????????????'''

    def callback_FollowLane(self, lane_position):
        desired_position = 0.0 # hardcoded desired position, should this come from launchfile?
        position = lane_position.data

        error = desired_position-position 

        Kp = 2.5 # proportional gain
        Kd = 0.7 # derivative gain

        angular_z = Kp * error + Kd * (error - self.lastError) #angular velocity output to motors

        self.lastError = error # update error at (t-1)
        twist = Twist()
        
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0) # calculate theta dot
        twist.linear.x = min(self.MAX_VEL * (1-(twist.angular.z/self.MAX_VEL)), 0.2) # calculate forward velocity
        self.pub_cmd_vel.publish(twist) # publish command velocity
        

    def fnShutDown(self):
        # function to stop the turtlebot
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_controller') # initialize lane controller node
    node = ControlLane() # assign node class to node variable
    node.main() # call main instance of ControlLane class to keep node running