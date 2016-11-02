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
	#rospy.Subscriber("/camera_2/allScan", PointCloud2, self.updatePointCloud)
        rospy.spin()

    """
    def updatePointCloud(self,msg):
        if self.tf.frameExists("/foxbot_base") and self.tf.frameExists("/blaser"):
            #t = self.tf.getLatestCommonTime("/foxbot_base", "/blaser")
            cloud = self.tf.transformPointCloud("/foxbot_base",msg)
            self.gen = pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
            print self.gen
    """
            

    def showCrossSection(self,msg):
        if self.tf.frameExists("/foxbot_base") and self.tf.frameExists("/blaser"):
            #t = self.tf.getLatestCommonTime("/foxbot_base", "/blaser")
            now = msg.header.stamp
            self.tf.waitForTransform("/foxbot_base","/blaser",now,rospy.Duration(5.0))
            pointCloud = self.tf.transformPointCloud("/foxbot_base",msg)
            size = len(msg.points)
            print "size = ", size
            points = np.zeros((3,size))
            for i in range(size):
                points[0,i] = msg.points[i].x
                points[1,i] = msg.points[i].y
                points[2,i] = msg.points[i].z
            self.y = np.mean(points[1])
            plt.plot(points[0],points[2],'ro')
            plt.show()
            #plt.plot(self.gen[0],self.gen[3],'bo')

	#self.rate.sleep()

if __name__ == '__main__':
    try:
        cs = CrossSection()
    except rospy.ROSInterruptException:
        pass
