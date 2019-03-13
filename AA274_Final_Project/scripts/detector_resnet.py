#!/usr/bin/env python

import rospy
import os
# watch out on the order for the next two imports lol
# from tf import TransformListener,TransformBroadcaster
import tf as TF
import geometry_msgs.msg
import turtlesim.srv
import tensorflow as tf
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math

# path to the trained conv net
PATH_TO_MODEL = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/ssd_resnet_50_fpn_coco.pb')
PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../tfmodels/coco_labels.txt')
#NOT_FOOD_CLASSES = set(['person', 'stop_sign'])
NOT_FOOD_CLASSES = set(['stop_sign'])
FOOD_CLASSES = set(['banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot_dog', 'pizza', 'donut', 'cake', 'potted_plant', 'fruit', 'napkin', 'salad', 'vegetable'])
DEBUG = True
# set to True to use tensorflow and a conv net
# False will use a very simple color thresholding to detect stop signs only
USE_TF = True
# minimum score for positive detection
MIN_SCORE = .3

def load_object_labels(filename):
    """ loads the coco object readable name """

    fo = open(filename,'r')
    lines = fo.readlines()
    fo.close()
    object_labels = {}
    for l in lines:
        object_id = int(l.split(':')[0])
        label = l.split(':')[1][1:].replace('\n','').replace('-','_').replace(' ','_')
        object_labels[object_id] = label

    return object_labels

class ObjectLocator:
    def __init__(self):
        self.objects = {}
        self.br = TF.TransformBroadcaster()
        self.n_stop_sign = 0

        # self.debug_pub = rospy.Publisher('/debug', String, queue_size = 10) # added by william for debug

    def register(self, obj, uncertainty, world_transform_func):
        rospy.logwarn('register object')
        class_ = obj.name
        # TODO: theta sign
        theta = (obj.thetaleft + obj.thetaright) / 2
        distance = obj.distance  # added by william
        #coord = world_transform_func(theta, distance)
        coord, quat_rot = world_transform_func(0, 0)
        x, y, z = coord
        coord = np.array(coord)
        # TODO: implementation #2 where we store the average of the robot pose when we see the object
        # simply store the coordinate of the robot and the pose (euler angles average)
        # (trans,rot) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        # rot = TF.transformations.euler_from_quaternion(rot)  # edited

        if class_ == "stop_sign":  # may need to remove this addition
            #return
            class_ = class_ + '_' + str(self.n_stop_sign)
            self.n_stop_sign += 1
        else:
            if class_ in self.objects:
                # Crappy Kalman function
                (obj2, coord2, quat_rot2, closest_dist2, n) = self.objects[class_]
                if distance >= closest_dist2:
                    distance=closest_dist2
                    coord = coord2
                    quat_rot = quat_rot2

                #coord = (coord * uncertainty2 + coord2 * uncertainty) / (uncertainty + uncertainty2)
                #uncertainty = ((uncertainty2 * n * np.sqrt(n)) + uncertainty)
                n += 1
                #uncertainty /= (n * np.sqrt(n))
                #self.objects[class_] = (obj, coord, uncertainty, n+1)
                self.objects[class_] = (obj, coord, quat_rot, distance, n+1)
            else:
                #self.objects[class_] = (obj, coord, uncertainty, 1)
                self.objects[class_] = (obj, coord, quat_rot, distance, 1)
        self.br.sendTransform(tuple(coord.tolist()), quat_rot, rospy.Time.now(), "/" + class_, "/map")

        # msg = String()  ## added by william for debug warning
        # msg.data = 'sent transform as / ' + class_
        # self.debug_pub.publish(msg)
        # rospy.logwarn(coord)

class Detector:

    def __init__(self):
        rospy.init_node('turtlebot_detector', anonymous=True)
        self.bridge = CvBridge()
        self.br = TF.TransformBroadcaster()  # added by william
        self.object_locator = ObjectLocator()  # added by william
        self.trans_listener = TF.TransformListener()  # added by william

        self.detected_objects_pub = rospy.Publisher('/detector/objects', DetectedObjectList, queue_size=10)

        if USE_TF:
            self.detection_graph = tf.Graph()
            with self.detection_graph.as_default():
                od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def,name='')
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
                config = tf.ConfigProto()
                config.gpu_options.allow_growth = True
            self.sess = tf.Session(graph=self.detection_graph, config=config)
            # self.sess = tf.Session(graph=self.detection_graph)

        # camera and laser parameters that get updated
        self.cx = 0.
        self.cy = 0.
        self.fx = 1.
        self.fy = 1.
        self.laser_ranges = []
        self.laser_angle_increment = 0.01 # this gets updated

        self.object_publishers = {}
        self.object_labels = load_object_labels(PATH_TO_LABELS)
        self.acceptable_class_id = []
        for id_ in self.object_labels:
            id_ = int(id_)
            name = self.object_labels[id_]
            if name in FOOD_CLASSES or name in NOT_FOOD_CLASSES:
                self.acceptable_class_id.append(id_)

        self.tf_listener = TF.TransformListener()
        rospy.Subscriber('/raspicam_node/image_raw', Image, self.camera_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.compressed_camera_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    def run_detection(self, img):
        """ runs a detection method in a given image """

        image_np = self.load_image_into_numpy_array(img)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        if USE_TF:
            # uses MobileNet to detect objects in images
            # this works well in the real world, but requires
            # good computational resources
            with self.detection_graph.as_default():
                (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes,self.d_scores,self.d_classes,self.num_d],
                feed_dict={self.image_tensor: image_np_expanded})

            '''
            print(scores.shape, classes.shape, num.shape)
            print(scores[0])
            print(scores[1])
            '''

            return self.filter(boxes[0], scores, classes[0], num[0])

        else:
            # uses a simple color threshold to detect stop signs
            # this will not work in the real world, but works well in Gazebo
            # with only stop signs in the environment
            R = image_np[:,:,0].astype(np.int) > image_np[:,:,1].astype(np.int) + image_np[:,:,2].astype(np.int)
            Ry, Rx, = np.where(R)
            if len(Ry)>0 and len(Rx)>0:
                xmin, xmax = Rx.min(), Rx.max()
                ymin, ymax = Ry.min(), Ry.max()
                boxes = [[float(ymin)/image_np.shape[1], float(xmin)/image_np.shape[0], float(ymax)/image_np.shape[1], float(xmax)/image_np.shape[0]]]
                scores = [.99]
                classes = [13]
                num = 1
            else:
                boxes = []
                scores = 0
                classes = 0
                num = 0

            return boxes, scores, classes, num

    def world_coordinates(self, ang, distance):
        (trans,rot) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        quat_rot = rot
        rot = TF.transformations.euler_from_quaternion(rot)  # edited
        xr, yr, zr = rot
        assert xr == 0
        assert yr == 0
        x, y, z = trans
        assert z == 0
        ang += zr
        x += distance * np.cos(ang)
        y += distance * np.sin(ang)
        return (x, y, z), quat_rot

    def filter(self, boxes, scores, classes, num):
        """ removes any detected object below MIN_SCORE confidence """

        full_scores = scores
        scores = scores[0]

        f_scores, f_boxes, f_classes = [], [], []
        f_num = 0

        for i in range(num):
            class_string = self.object_labels[int(classes[i])]
            if class_string not in FOOD_CLASSES and class_string not in NOT_FOOD_CLASSES:
                rospy.loginfo("REJECTING IRRELEVANT CLASS: " + str(class_string))
                continue
            if scores[i] >= MIN_SCORE:
                f_scores.append(scores[i])
                f_boxes.append(boxes[i])
                f_classes.append(int(classes[i]))
                f_num += 1
            else:
                rospy.loginfo("REJECTING LOW SCORE CLASS: " + str(class_string) + " " + str(scores[i]))
                break

        return f_boxes, f_scores, f_classes, f_num

    def load_image_into_numpy_array(self, img):
        """ converts opencv image into a numpy array """

        (im_height, im_width, im_chan) = img.shape

        return np.array(img.data).reshape((im_height, im_width, 3)).astype(np.uint8)

    def project_pixel_to_ray(self,u,v):
        """ takes in a pixel coordinate (u,v) and returns a tuple (x,y,z)
        that is a unit vector in the direction of the pixel, in the camera frame.
        This function access self.fx, self.fy, self.cx and self.cy """

        x = (u - self.cx)/self.fx
        y = (v - self.cy)/self.fy
        norm = math.sqrt(x*x + y*y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm

        return (x,y,z)

    def estimate_distance(self, thetaleft, thetaright, ranges, dist_estimate, uncertainty, class_):
        """ estimates the distance of an object in between two angles
        using lidar measurements """

        offset = int(np.pi/self.laser_angle_increment)
        # add offset and wrap to number of points. laserscan should always have the same number of points. (2*offset = len(ranges))
        # See https://github.com/StanfordASL/velodyne/blob/master/velodyne_laserscan/src/VelodyneLaserScan.cpp#L110
        leftray_indx = (offset + min(max(0, int(thetaleft/self.laser_angle_increment)), offset * 2)) % (offset * 2)
        rightray_indx = (offset + min(max(0, int(thetaright/self.laser_angle_increment)), offset * 2)) % (offset * 2)

        if leftray_indx<rightray_indx:
            meas = ranges[rightray_indx:] + ranges[:leftray_indx]
        else:
            meas = ranges[rightray_indx:leftray_indx]

        num_m = 0
        dists = []
        for m in meas:
            if m>0 and m<float('Inf'):
                dists.append(m)
                num_m += 1
        dists_new = []
        for dist in dists:
            if dist <= dist_estimate + uncertainty and dist >= dist_estimate - uncertainty:
                dists_new.append(dist)
        if len(dists_new) > 0:
            dists = dists_new
        else:
            rospy.loginfo("laser return early class " + str(self.object_labels[class_]) + " dist " + str(dist_estimate))
            return dist_estimate

        dists = np.sort(np.array(dists))
        m = min(10, num_m)
        est = np.mean(dists[:m])
        rospy.loginfo("laser class " + str(self.object_labels[class_]) + " dist " + str(est))
        return est

    def estimate_distance_bbox_size(self, thetaup, thetadown, thetaleft, thetaright, class_):
        # ONLY APPLIES TO CLASSES PRINTED ON PAPER
        bounding_box_len = 0.11 # Measured with ruler lol
        if self.object_labels[class_] not in FOOD_CLASSES:
            if self.object_labels[class_] == 'stop_sign':
                bounding_box_len = 0.07
            else:
                return -1, 10000000
        print('up down left right' + str(thetaup) + ' ' + str(thetadown) + ' ' + str(thetaleft) + ' ' + str(thetaright))
        sidedist = abs(thetaleft-thetaright)
        updist = abs(thetaleft-thetaright)
        angledist = max(sidedist, updist)
        rospy.loginfo('side, up, angle ' + str(sidedist) + " " + str(updist) + " " + str(angledist))
        longitudinal_dist = bounding_box_len / math.tan(angledist)
        if longitudinal_dist < 0:
            rospy.loginfo("We have negative distance from camera :(")
            #longitudinal_dist *= -1
        uncertainty = 1.5 * longitudinal_dist
        uncertainty = max(uncertainty, 1.5)
        rospy.loginfo("Class " + str(self.object_labels[class_]) + " bounding box, dist " + str(bounding_box_len) + " " + str(longitudinal_dist))
        return longitudinal_dist, uncertainty

    def camera_callback(self, msg):
        """ callback for camera images """

        # save the corresponding laser scan
        img_laser_ranges = list(self.laser_ranges)

        try:
            img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            img_bgr8 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.camera_common(img_laser_ranges, img, img_bgr8)

    def compressed_camera_callback(self, msg):
        """ callback for camera images """

        # save the corresponding laser scan
        img_laser_ranges = list(self.laser_ranges)

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
            img_bgr8 = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.camera_common(img_laser_ranges, img, img_bgr8)

    def camera_common(self, img_laser_ranges, img, img_bgr8):
        (img_h,img_w,img_c) = img.shape

        # runs object detection in the image
        (boxes, scores, classes, num) = self.run_detection(img)

        if num > 0:
            # create list of detected objects
            detected_objects = DetectedObjectList()

            # some objects were detected
            for (box,sc,cl) in zip(boxes, scores, classes):
                ymin = int(box[0]*img_h)
                xmin = int(box[1]*img_w)
                ymax = int(box[2]*img_h)
                xmax = int(box[3]*img_w)
                xcen = int(0.5*(xmax-xmin)+xmin)
                ycen = int(0.5*(ymax-ymin)+ymin)

                cv2.rectangle(img_bgr8, (xmin,ymin), (xmax,ymax), (255,0,0), 2)

                # computes the vectors in camera frame corresponding to each sides of the box
                rayleft = self.project_pixel_to_ray(xmin,ycen)
                rayright = self.project_pixel_to_ray(xmax,ycen)
                rayup = self.project_pixel_to_ray(xcen,ymin)
                raydown = self.project_pixel_to_ray(xcen,ymax)

                # convert the rays to angles (with 0 poiting forward for the robot)
                thetaleft = math.atan2(-rayleft[0],rayleft[2])
                thetaright = math.atan2(-rayright[0],rayright[2])
                thetaup = math.atan2(-rayup[1],rayup[2])
                thetadown = math.atan2(-raydown[1],raydown[2])

                dist_estimate, uncertainty = self.estimate_distance_bbox_size(thetaup,thetadown, thetaleft,thetaright,cl)
                if thetaleft<0:
                    thetaleft += 2.*math.pi
                if thetaright<0:
                    thetaright += 2.*math.pi
                if thetaup<0:
                    thetaup += 2.*math.pi
                if thetadown<0:
                    thetadown += 2.*math.pi
                # estimate the corresponding distance using the lidar
                dist = self.estimate_distance(thetaleft,thetaright,img_laser_ranges, dist_estimate, uncertainty, cl)
                if math.isnan(dist):
                    dist = dist_estimate
                if dist < 0:
                    continue
                dist = (dist + dist_estimate) / 2.0
                if not self.object_publishers.has_key(cl):
                    self.object_publishers[cl] = rospy.Publisher('/detector/'+self.object_labels[cl],
                        DetectedObject, queue_size=10)

                # publishes the detected object and its location
                object_msg = DetectedObject()
                object_msg.id = cl
                object_msg.name = self.object_labels[cl]
                object_msg.confidence = sc
                object_msg.distance = dist
                object_msg.thetaleft = thetaleft
                object_msg.thetaright = thetaright
                object_msg.corners = [ymin,xmin,ymax,xmax]
                self.object_publishers[cl].publish(object_msg)

                self.object_locator.register(object_msg, uncertainty, self.world_coordinates)

                # add detected object to detected objects list
                detected_objects.objects.append(self.object_labels[cl])
                detected_objects.ob_msgs.append(object_msg)

            self.detected_objects_pub.publish(detected_objects)

        # displays the camera image
        #cv2.imshow("Camera", img_bgr8)
        #cv2.waitKey(1)

    def camera_info_callback(self, msg):
        """ extracts relevant camera intrinsic parameters from the camera_info message.
        cx, cy are the center of the image in pixel (the principal point), fx and fy are
        the focal lengths. Stores the result in the class itself as self.cx, self.cy,
        self.fx and self.fy """

        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.fx = msg.P[0]
        self.fy = msg.P[5]

    def laser_callback(self, msg):
        """ callback for thr laser rangefinder """

        self.laser_ranges = msg.ranges
        self.laser_angle_increment = msg.angle_increment

    def run(self):
        ### edited by william, added continuous tf broadcaster
        for class_ in self.object_locator.objects:
            (obj, coord, quat_rot, closest_dist, n) = self.object_locator.objects[class_]
            self.br.sendTransform(tuple(coord.tolist()), quat_rot, rospy.Time.now(), "/" + class_, "/map")

        # template code for example usage
        # if class_ == "stop_sign":
        #     class_ = class_ + '_' + str(self.n_stop_sign)
        #     self.n_stop_sign += 1
        # else:
        #     if class_ in self.objects:
        #         # Crappy Kalman function
        #         (obj2, coord2, uncertainty2, n) = self.objects[class_]
        #         coord = (coord * uncertainty2 + coord2 * uncertainty) / (uncertainty + uncertainty2)
        #         uncertainty = ((uncertainty2 * n * np.sqrt(n)) + uncertainty)
        #         n += 1
        #         uncertainty /= (n * np.sqrt(n))
        #         self.objects[class_] = (obj, coord, uncertainty, n+1)
        #     else:
        #         self.objects[class_] = (obj, coord, uncertainty, 1)
        # self.br.sendTransform(coord, TF.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/" + class_, "/map")

        # rospy.spin()

'''
class ObjectDetected:
    def __init__(self):
        #this creates the broadcaster portion
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        self.tf_listener = TF.TransformListener()
        self.turtlename = 'sentro'
        rospy.Subscriber('/%s/pose' % self.turtlename, turtlesim.msg.Pose, self.turtle_broadcast_callback,self.turtlename)
        self.parent = '/base_footprint' #possibly base_link
        self.child = '/map'

    #broadcaster callback
    def turtle_broadcast_callback(self,msg):
        br = TF.TransformBroadcaster()
        br.sendTransform((msg.x, msg.y, 0), TF.transformations.quaternion_from_euler(0, 0, msg.theta), rospy.Time.now(), self.turtlename, "world")

    #this is for the listener portion
    #http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
    def tf_call_back(self,msg):
        br = TF.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform(self.child, self.parent, rospy.Time(0))
            except (TF.LookupException, TF.ConnectivityException, TF.ExtrapolationException):
                continue


    #this is for adding the frame of the object
    #http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29



    #this continuously runs the subscribing/broadcasting tf messages
    def run(self):
        rospy.spin()
'''



if __name__=='__main__':
    #o = ObjectDetected()
    d = Detector()
    #o.run()
    # d.run()

    # added by william
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        d.run()
        rate.sleep()  # may want to do rospy.spinOnce() OR nothing at all
