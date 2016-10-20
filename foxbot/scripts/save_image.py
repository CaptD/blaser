#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""

Copyright (c) 2015 PAL Robotics SL.
Released under the BSD License.

Created on 7/14/15

@author: Sammy Pfeiffer

test_video_resource.py contains
a testing code to see if opencv can open a video stream
useful to debug if video_stream does not work
"""

import cv2
import sys, time
import roslib
import rospy
import numpy as np
from tf import TransformListener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf.transformations as transform

class ImageLog:
    count = 0;
    period = 10;
    def __init__(self):
	# node initialized and constants described here
	rospy.init_node('image_log_node', anonymous=True)
        self.tf = TransformListener()
        rospy.Subscriber("/camera_1/image_raw", Image, self.saveImage)
	ImageLog.rate = rospy.Rate(0.5)

    def saveImage(self,msg):
	br = CvBridge()
	frame = br.imgmsg_to_cv2(msg)
	ImageLog.count = (ImageLog.count+1)%ImageLog.period
        ImageLog.rate.sleep()
        if self.tf.frameExists("/foxbot_base") and self.tf.frameExists("/foxbot_tool"):
            t = self.tf.getLatestCommonTime("/foxbot_base", "/foxbot_tool")
            position, quaternion = self.tf.lookupTransform("/foxbot_base", "/foxbot_tool", t)
            current_pose = transform.euler_from_quaternion([quaternion[0],quaternion[1],quaternion[2],quaternion[3]])
            print position, current_pose
            fn = '%.4f'%position[0]+'_%.4f'%position[1]+'_%.4f'%position[2]+'_%.4f'%quaternion[0]+'_%.4f'%quaternion[1]+'_%.4f'%quaternion[2]+'_%.4f'%quaternion[3]+'.jpg'
            cv2.imwrite(fn, frame)
            print "image saved"
            ImageLog.rate.sleep()
            
            

	
if __name__ == '__main__':
    im = ImageLog()
    rospy.spin()

