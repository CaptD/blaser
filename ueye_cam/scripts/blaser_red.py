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
import std_msgs.msg

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge

class Blaser:
    table_red = 0
    #table_blue = 0
    #pointcloud_1_pub = 0
    pointcloud_2_pub = 0
    rate = 0
    def __init__(self):
	# node initialized and constants described here
	with open("/home/jin/boeing/catkin_ws/src/blaser/ueye_cam/scripts/red_table.txt") as file:
	    Blaser.table_red = [[float(digit) for digit in line.split(' ')] for line in file]
        #with open("blue_table.txt") as file:
    	#    Blaser.table_blue = [[float(digit) for digit in line.split(' ')] for line in file]
	
	#Blaser.pointcloud_1_pub = rospy.Publisher("/camera_1/PointCloud", PointCloud, queue_size = 5)
	Blaser.pointcloud_2_pub = rospy.Publisher("/camera_2/PointCloud", PointCloud, queue_size = 5)
	rospy.init_node('blaser_red_pub', anonymous=True)
	Blaser.rate = rospy.Rate(100) # 100hz

    def subscribe_image(self):
	#rospy.Subscriber("/camera_1/image_raw", Image, self.generatePointCloud_blue)
	rospy.Subscriber("/camera_2/image_raw", Image, self.generatePointCloud_red)
	rospy.spin()

    def generatePointCloud_red(self,msg):
	br = CvBridge()
	frame = br.imgmsg_to_cv2(msg)
	#cv2.imshow("Display window 1",frame) 
	#print np.max(frame[:,:,0]), np.max(frame[:,:,1]),np.max(frame[:,:,2])
	#cv2.waitKey(0)
        b,g,r = cv2.split(frame)
	lower_red = np.array([0,0,100])
	upper_red = np.array([100,100,255])
	mask = cv2.inRange(frame,lower_red,upper_red)
	temp = np.nonzero(np.transpose(mask))
	pixelpoints = np.array([temp[0],temp[1]])
	diff_col = np.nonzero(pixelpoints[0,1:]-pixelpoints[0,0:len(pixelpoints[1])-1])
	diff_col = np.array(np.append(diff_col[0]+1,len(pixelpoints[0])))
        #print diff_col
	point_cloud = PointCloud()
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'blaser'
	point_cloud.header = header
	for t in range(0,len(diff_col)):
	    if t == 0:
	        temp = np.mean(pixelpoints[:,0:diff_col[t]],axis = 1)
	    else:
	        temp = np.mean(pixelpoints[:,diff_col[t-1]:diff_col[t]],axis = 1)
	    ind = round(temp[0])*720+round(temp[1])
	    point_cloud.points.append(Point32(Blaser.table_red[int(ind)][0]*10,Blaser.table_red[int(ind)][1]*10,Blaser.table_red[int(ind)][2]*10))
	Blaser.pointcloud_2_pub.publish(point_cloud)
	Blaser.rate.sleep()
'''
    def generatePointCloud_blue(self,msg):
	br = CvBridge()
	frame = br.imgmsg_to_cv2(msg)
	cv2.imshow("Display window 2",frame) 
	cv2.waitKey(0); 
    	b,g,r = cv2.split(frame)	
    	lower_red = np.array([0,0,55])
	upper_red = np.array([100,100,255])
	mask = cv2.inRange(frame,lower_red,upper_red)
	temp = np.nonzero(np.transpose(mask))
	pixelpoints = np.array([temp[0],temp[1]])
	    
	diff_col = np.nonzero(pixelpoints[0,1:]-pixelpoints[0,0:len(pixelpoints[1])-1])
	diff_col = np.array(np.append(diff_col[0]+1,len(pixelpoints[0])))
        point_cloud = PointCloud()
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = 'camera_blue'
	point_cloud.header = header
	for t in range(0,len(diff_col)):
	    if t == 0:
	        temp = np.mean(pixelpoints[:,0:diff_col[t]],axis = 1)
	    else:
	        temp = np.mean(pixelpoints[:,diff_col[t-1]:diff_col[t]],axis = 1)
	    ind = round(temp[0])*360+round(temp[1])
	    point_cloud.points.append(Point32(Blaser.table_blue[int(ind)][0],Blaser.table_blue[int(ind)][1],Blaser.table_blue[int(ind)][2]))
	Blaser.pointcloud_2_pub.publish(point_cloud)
	Blaser.rate.sleep()
'''
if __name__ == '__main__':
    try:
        blaser = Blaser()
        blaser.subscribe_image()
    except rospy.ROSInterruptException:
        pass
