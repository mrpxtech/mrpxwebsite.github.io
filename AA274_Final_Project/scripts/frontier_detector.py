#!/usr/bin/env python


#--------Include modules---------------
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier
from rospy.numpy_msg import numpy_msg
from AA274_Final_Project.msg import DetectedObject
import numpy as np
import tf

from rospy_tutorials.msg import Floats
#from Floats_Array.msg import Floats_Array

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
MapData=OccupancyGrid()
global frontier_dist_to_robot_thresh 
frontier_dist_to_robot_thresh = 0.9


def mapCallBack(data):
    global MapData
    MapData = data
    

def marker_init():
    marker = Marker()
    marker.header.frame_id = MapData.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.ns= "markers"
    marker.id = 15
    marker.lifetime = rospy.Duration()
    return marker



#  Define Node----------------------------------------------
def node():
    rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    targetspub = rospy.Publisher('/detected_points', numpy_msg(Floats), queue_size=10)
    pub = rospy.Publisher('shapes', MarkerArray, queue_size=10)
    rospy.init_node('frontier_detector', anonymous=False)
    
    global MapData # global map data variable
    global robot_position
    robot_position = np.array([0,0])
    

  
    # wait until map is received, when a map is received, MapData.header.seq will not be < 1
    while MapData.header.seq<1 or len(MapData.data)<1:
        print("Waiting for map data")
        pass
        
    rate = rospy.Rate(10) # run rate of the node

    
    # -------------initialize marker properties-----------

    marker_array = MarkerArray()  #initialize marker array

   
    while not rospy.is_shutdown():
        
        #---------------------------- get robot posiiton _--------------------------------
        try:
            origin_frame = "/map" #if mapping else "/odom"
            (translation,rotation) = tf.TransformListener().lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
            robot_position = np.array(translation[0],translation[1])
            print("robot position",robot_position)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass








        
        frontiers_pixels = np.array(getfrontier(MapData),dtype=np.float32)
        # filter frontiers on distance from robot to account for un-measurable region (robot size + buffer)
        if frontiers_pixels.shape[0]>0: # if there is data

                # import data from getfrontier.py and convert to numpy array of float32
            
            res =  MapData.info.resolution # map resolution
            origin_offset = np.array([ MapData.info.origin.position.x, MapData.info.origin.position.y])
            frontiers_scaled = frontiers_pixels *res + origin_offset # calculate frontier positions in meters
            #Note this is a tuning parameter!!!

            # calculate distances from current position to frontiers
            distances = np.sqrt((frontiers_scaled[:,0]-robot_position[0])**2 + (frontiers_scaled[:,1]-robot_position[1])**2) #1d array of distances
            mask = (distances>frontier_dist_to_robot_thresh) # create mask for array
            print("unmasked frontiers=",frontiers_scaled)
            frontiers_scaled = frontiers_scaled[mask] # mask rows of frontier to remove the points to close to the robot
            print("masked frontiers=",frontiers_scaled)

            counter = 0 # initialize marker ID counter
            for i in range(len(frontiers_scaled)):
                
                marker = marker_init() # initialize marker
                marker.pose.position.x = frontiers_scaled[i,0]
                marker.pose.position.y = frontiers_scaled[i,1]
                marker.pose.orientation.w = 1
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.id = counter 
                marker_array.markers.append(marker)
                counter = counter + 1 # increment marker counter
     

            # ---------------------    publish topics   --------------------
            # won't publish if there is no data so there should be no index of length zero issues in other code
            # i.e. callbacks in other code will not be initiated

            frontiers_output = np.reshape(frontiers_scaled,-1).astype(np.float32) #must match message type!!!!
            # if dtype does not match message type will get overflow/ unknown behavior!
        
            print("frontiers_output",frontiers_output)
            targetspub.publish(frontiers_output)

        # this will always run whether or not there are frontiers to help clear the markers
        pub.publish(marker_array) #publish marker array
        rate.sleep() 




#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
