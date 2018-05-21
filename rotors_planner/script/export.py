#!/usr/bin/python

# Extract images from a bag file.

#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# Reading bag filename from command line or roslaunch parameter.
#import os
#import sys


class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag("/home/zhuzd/bag/test.bag", 'r') as bag:  
            for topic,msg,t in bag.read_messages():
                if topic == "/firefly/vi_sensor/camera_depth/camera/image_raw/compressed": 
                        try:
                            #cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			    np_arr = np.fromstring(msg.data, np.uint8)
			    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                        except CvBridgeError as e:
                            print e
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        image_name = timestr+ ".jpg" 
                        cv2.imwrite(image_name, cv_image)  

if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
