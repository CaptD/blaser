#! /usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys, time
import roslib
import rospy
import numpy as np
import tf.transformations as transform
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from tf import TransformListener

class CrossSection:
    def __init__(self):
	# node initialized and constants described here
	rospy.init_node('cross_section_plotter', anonymous=True)
        self.rate = rospy.Rate(1) # 100hz
        self.mean_y = 0
        self.data = np.array([0])
        self.point = np.array([0])
        self.tf = TransformListener()  
        self.blank_image = np.zeros((600,800,3), np.uint8)
        cv2.putText(self.blank_image,'1cm',(110,130), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.line(self.blank_image, (100,100), (200,100),(255,255,255),1) 
        rospy.Subscriber("/camera_1/PointCloud", PointCloud, self.showCrossSection)
	rospy.Subscriber("/camera_2/allScan", PointCloud2, self.updatePointCloud)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/CrossSection",Image,queue_size = 5)
        rospy.spin()
      
    def updateplot(self):
        image = self.blank_image
        print self.data.shape
        if self.data.shape[0] > 2:
            for i in range(self.data.shape[0]):
                cv2.circle(image, (int((4.5-self.data[i,0])*1000),int((0.3-self.data[i,2])*1000)), 1, (0,0,255), -1)
        if self.points.shape[0] > 2:
            
            for i in range(self.points.shape[1]):
                cv2.circle(image, (int((4.5-self.points[0][i])*1000),int((0.3-self.points[2][i])*1000)), 1, (255,0,0), -1)
                #print int((self.points[0][i]-3.8)*1000),int((self.points[2][i]+0.2)*1000)
        #cv2.imshow('im',blank_image)
        #cv2.waitKey(1)
        cv2.putText(image,"y = "+"%.3f" % self.mean_y,(400,120), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),1,cv2.LINE_AA)
	msg = self.bridge.cv2_to_imgmsg(image,"bgr8")
        self.pub.publish(msg)


    def updatePointCloud(self,msg):
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        point = np.asarray(list(gen))
        
        ll = np.array([-10,self.mean_y-0.006,-10])
	ur = np.array([10,self.mean_y+0.006,10])
        mask = np.all(np.logical_and(ll <= point, point <= ur), axis=1)
        self.data = point[mask]
        print "red updated"

    def showCrossSection(self,msg):
        #print msg.header.frame_id
        if self.tf.frameExists("/foxbot_base") and self.tf.frameExists("/blaser"):
            #t = self.tf.getLatestCommonTime("/foxbot_base", "/blaser")
            now = msg.header.stamp
            self.tf.waitForTransform("/foxbot_base","/blaser",now,rospy.Duration(1.0))
            pointCloud = self.tf.transformPointCloud("/foxbot_base",msg)
            size = len(msg.points)
            #print "size = ", size
            self.points = np.zeros((3,size))
            for i in range(size):
                self.points[0,i] = pointCloud.points[i].x
                self.points[1,i] = pointCloud.points[i].y
                self.points[2,i] = pointCloud.points[i].z
            self.mean_y = np.mean(self.points[1])
            print "blue updated"
            self.updateplot()

if __name__ == '__main__':
    try:
        cs = CrossSection()
    except rospy.ROSInterruptException:
        pass
