#! /usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys, time
import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as transform
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from tf import TransformListener


class CrossSection:
    def __init__(self):
	# node initialized and constants described here
	rospy.init_node('cross_section_plotter', anonymous=True)
	#self.rate = rospy.Rate(100) # 10hz
        self.tf = TransformListener()
        rospy.Subscriber("/camera_1/PointCloud", PointCloud, self.showCrossSection)
	rospy.Subscriber("/camera_2/allScan", PointCloud2, self.updatePointCloud)
        self.line_blue = plt.plot(0,0,'bo')
        self.line_red = plt.plot(0,0,'ro')
        self.rate = rospy.Rate(1) # 100hz
        self.data = np.array([0,0,0])
        self.mean_y = 0
        plt.axis('equal')
        plt.axis([3.8, 4.6, -0.3, 0.5])
        plt.show()
        rospy.spin()
      

    def updatePointCloud(self,msg):
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        point = np.asarray(list(gen))
        
        ll = np.array([-10,self.mean_y-0.006,-10])
	ur = np.array([10,self.mean_y+0.006,10])
        mask = np.all(np.logical_and(ll <= point, point <= ur), axis=1)
        self.data = point[mask]
        #mask = self.gen
        #print "red", self.data.shape
        print "red updated"

    def showCrossSection(self,msg):
        #print msg.header.frame_id
        if self.tf.frameExists("/foxbot_base") and self.tf.frameExists("/blaser"):
            #t = self.tf.getLatestCommonTime("/foxbot_base", "/blaser")
            now = msg.header.stamp
            self.tf.waitForTransform("/foxbot_base","/blaser",now,rospy.Duration(3.0))
            pointCloud = self.tf.transformPointCloud("/foxbot_base",msg)
            size = len(msg.points)
            #print "size = ", size
            points = np.zeros((3,size))
            for i in range(size):
                points[0,i] = pointCloud.points[i].x
                points[1,i] = pointCloud.points[i].y
                points[2,i] = pointCloud.points[i].z
            self.mean_y = np.mean(points[1])
            #print self.mean_y
            print "blue updated"
            plt.setp(self.line_blue,'xdata',points[0],'ydata',points[2])
            plt.setp(self.line_red,'xdata',self.data[:,0],'ydata',self.data[:,2])
            plt.show()
            #plt.plot(self.gen[0],self.gen[3],'bo')

if __name__ == '__main__':
    try:
        cs = CrossSection()
    except rospy.ROSInterruptException:
        pass
