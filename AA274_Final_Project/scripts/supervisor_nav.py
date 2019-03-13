#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64, String
from rospy_tutorials.msg import Floats
import numpy as np
import tf
import math
from enum import Enum

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 5

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 5

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    EXPLORE = 7
    PICKUP = 8
    BASE = 9


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.prevmode = Mode.IDLE
        self.objects = [] #tf frames of objects detected
        self.frontier_goal_dist = 0 # initialized frontier goal distance
        self.foodx = []
        self.foody = []
        self.foodth = []

        #start in idle - go to explore when cmd sent from terminal
        self.mode = Mode.IDLE 

        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        self.points = np.zeros((4,)) #nothing recieved from frontier points, should change when callback called

        #replace below after debugging mode explore
        # self.mode = Mode.IDLE
        # self.last_mode_printed = None
        # self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)

        # exploration subscriber------------------------------------------
        rospy.Subscriber('/detected_points',numpy_msg(Floats),self.frontier_explorer_callback)

        # manual command state subscriber---------------------------------
        rospy.Subscriber('/cmd_state', String, self.manual_command_callback)
        # rostopic pub /cmd_state std_msgs/String "EXPLORE"

        #subscribe to delivery request
        rospy.Subscriber('/delivery_request', String, self.receive_delivery_callback)

    def receive_delivery_callback(self,msg):
        if len(self.objects) ==0:
            self.mode = Mode.PICKUP

            foodString = msg.data.split(',')
            for i in foodString:
                foodString[i] = '/'+ foodString[i]
            self.objects = foodString

            #find coordinates for each food object detected
            #store in different lists because not sure about callback and receiving message for going through list
            #self.objects

            for food in self.objects:
                try:
                    (translation,rotation) = self.trans_listener.lookupTransform('/map', foodString[food], rospy.Time(0))
                    self.foodx.append(translation[0]) 
                    self.foody.append(translation[1])
                    euler = tf.transformations.euler_from_quaternion(rotation)
                    self.foodth.append(euler[2])
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.loginfo(foodString[food],'not detected')
                    print(foodString[food],'not detected')
                    pass
            self.nav_to_food()

    def nav_to_food(self):
        #what happens at empty list
        try:
            nav_g_msg = Pose2D()
            nav_g_msg.x = self.foodx[0]
            nav_g_msg.y = self.foody[0]
            nav_g_msg.theta = self.foodth[0]
            self.nav_goal_publisher.publish(nav_g_msg)
        except(IndexError):
            self.done_picking()

    def has_picked(self):
        #if we are close to the first food goal, remove
        if self.close_to(self.foodx[0],self.foody[0],self.foodth[0]):
            # self.stay_idle()
            #stay idle at the position for 5 seconds...
            self.at_fruit()
            #what happens at an empty list
            try:
                self.foodx.remove(self.foodx[0])
                self.foody.remove(self.foody[0])
                self.foodth.remove(self.foodth[0])
            except(IndexError):
                self.done_picking()
            if len(self.foodx) == 0:
                self.done_picking()
        else:
            self.nav_to_food()
            
    def at_fruit(self):
        self.fruit_time = rospy.get_time()
        self.mode = Mode.IDLE
    def done_picking(self):

        if len(self.objects) > 0:
            return False
        else:
            # self.stay_idle()
            return True


    def manual_command_callback(self, msg):
        msgstr  = msg.data.split()
        rospy.loginfo(msgstr)
        if len(msgstr) == 0: return

        if msgstr[0] == "EXPLORE":
            self.mode = Mode.EXPLORE
        elif msgstr[0] == 'KILL': 
            self.stay_idle()
            self.mode = Mode.IDLE
        elif msgstr[0] == 'PICKUP':
            self.mode = Mode.PICKUP
            self.nav_to_food()
        elif msgstr[0] == 'NAV':
            self.mode = Mode.NAV
            #publish the desired position/pose from the command line
            self.x_g= msgstr[1]
            self.y_g = msgstr[2]
            self.theta_g = msgstr[3]
            self.nav_to_pose()



    def frontier_explorer_callback(self,msg):
        # calculate closest frontier
        self.points = msg.data # frontier points
        #print("Frontier callback executed!")

    def frontier_explorer(self):

        points = np.reshape(self.points,(-1,2))
        print("Received frontier candidate points", points)

        ref = np.array([self.x,self.y]) # current position reference

        # calculate distances
        distances = np.sqrt((points[:,0]-ref[0])**2 + (points[:,1]-ref[1])**2) #1d array of distances
        goal = points[np.argmin(distances),:] # grab x,y row of min distance
        frontier_goal_dist = distances[np.argmin(distances)]
        
        # debugging if statement------
        if frontier_goal_dist < 0.17:
            print("Error frontier too close!")

        print("dist to frontier", distances[np.argmin(distances)] )
        # ---- can remove above if statement in final code
        


        


       

        # Note: rtol and atol are tunning parameters
        # call nav_to_pose if navigation distance has not changed by more than .1 meters
        #print("frontier goal comparision",frontier_goal_dist,self.frontier_goal_dist)
        #print("isclose", np.isclose(frontier_goal_dist, self.frontier_goal_dist, atol=0.2))
        if np.isclose(frontier_goal_dist, self.frontier_goal_dist, atol=0.15):
            pass
        else:
            self.frontier_goal_dist = frontier_goal_dist
            self.x_g = goal[0]
            self.y_g = goal[1]
            self.theta_g = self.theta 
            

         # print robot current goal and position generated by the explore module for debugging
        print("x_goal y_goal",self.x_g,self.y_g)
        print("self.x, self.y",self.x , self.y)
            
            
            
        #print("Exploring: called nav to pose! ")
        self.nav_to_pose() # has to be called constantly to move
        






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
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and (self.mode == Mode.NAV or self.mode == Mode.EXPLORE or self.mode == Mode.PICKUP):
            self.prevmode = self.mode
            self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

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

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            '''self.stay_idle()'''

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = self.prevmode
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g):
               ''' self.mode = Mode.IDLE'''
            else:
                self.nav_to_pose()

        elif self.mode == Mode.EXPLORE:
            self.frontier_explorer()
        
        #implement pickup mode
        elif self.mode == Mode.PICKUP:
            #done picking up all items
            if self.has_picked():
                #return to base
                self.return_to_base()
            else: #continue to pickup items
                self.pickup()

        elif self.mode == Mode.BASE:
            if self.has_returned_to_base():
                self.stay_idle()
            else:
                self.return_to_base()
        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
