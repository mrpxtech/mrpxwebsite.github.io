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

from rospy_tutorials.msg import Floats
#from Floats_Array.msg import Floats_Array

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
MapData=OccupancyGrid()


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

# Node----------------------------------------------
def node():
    #map_topic= rospy.get_param('~map_topic','/robot_1/map')
    #rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    targetspub = rospy.Publisher('/detected_points', numpy_msg(Floats), queue_size=10)
    pub = rospy.Publisher('shapes', MarkerArray, queue_size=10)
    rospy.init_node('frontier_detector', anonymous=False)
    
    global MapData # global map data variable

    #exploration_goal=PointStamped()
  
    # wait until map is received, when a map is received, MapData.header.seq will not be < 1
    while MapData.header.seq<1 or len(MapData.data)<1:
        print("stuck here")
        pass
        
    rate = rospy.Rate(10)
    
    # -------------initialize marker properties-----------
   




    marker_array = MarkerArray()  #initialize marker array

    counter = 0
    while not rospy.is_shutdown():
        #each marker each id
        frontiers = np.array(getfrontier(MapData),dtype=np.float32)

        for point in frontiers:
            res =  MapData.info.resolution
            print("resolution",res)
            marker = marker_init()
           
            marker.pose.position.x = point[0]*res + MapData.info.origin.position.x
            marker.pose.position.y = point[1]*res + MapData.info.origin.position.y
            marker.pose.orientation.w = 1
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.id = counter
            marker_array.markers.append(marker)
            counter = counter + 1
            '''print("pixel x",point[0])
            print("pixel y",point[1])
            print("marker x",marker.pose.position.x)
            print("Marker x origin",MapData.info.origin.position.x)
            print("marker y",marker.pose.position.y)
            print("Marker y origin",MapData.info.origin.position.y)'''

        # ---------------------publish topics--------------------
        print("frontier type",frontiers.dtype)
        frontiers_o = np.reshape(frontiers,-1)
        print("frontier",frontiers_o)
        targetspub.publish(frontiers_o)
        pub.publish(marker_array) #publish marker array
        rate.sleep()
        #print("counter",counter)
        #print("pixel x",point[0])
        counter = 0
        
		

	  	#rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
