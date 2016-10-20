import rospy 
import sys
import math
import tf.transformations as transform
import numpy as np
import time
from foxbot.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def setJoints(desired_joint):
    rospy.wait_for_service('/foxbot/robot_SetJoints')
    try:
        setJoints_client = rospy.ServiceProxy('/foxbot/robot_SetJoints', robot_SetJoints)
        resp1 = setJoints_client(desired_joint[0], desired_joint[1],desired_joint[2],desired_joint[3],desired_joint[4],desired_joint[5])
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def setCartesian(desired_pose):
    rospy.wait_for_service('/foxbot/robot_SetCartesian')
    try:
        setCartesian_client = rospy.ServiceProxy('/foxbot/robot_SetCartesian', robot_SetCartesian)
        resp1 = setCartesian_client(desired_pose[0], desired_pose[1],desired_pose[2],desired_pose[6],desired_pose[3],desired_pose[4],desired_pose[5],0)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getCartesian():
    rospy.wait_for_service('/foxbot/robot_GetCartesian')
    try:
        getCartesian_client = rospy.ServiceProxy('/foxbot/robot_GetCartesian', robot_GetCartesian)
        resp1 = getCartesian_client()
        current_pose = transform.euler_from_quaternion([resp1.qx,resp1.qy,resp1.qz,resp1.q0])
        return [resp1.x,resp1.y,resp1.z,math.degrees(current_pose[0]),math.degrees(current_pose[1]),math.degrees(current_pose[2])]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":

    desired_joint = np.array([[0.0, 44.6,112.3,2.3,22.8,-0.6],[-2.3, 44.6,112.3,2.3,23.0,-3.3],[-4.9, 44.7,112.0,2.3,23.1,-5.9],[-7.4,44.8,111.7,2.3,23.4,-8.4],[-9.7,44.8,111.7,2.3,23.4,-10.7],[-9.2,45.6,109.6,2.2,24.6,-10.1],[-10.0,43.8,114.4,2.5,21.7,-11.2],[-10.1,43.7,114.8,2.5,21.4,-11.3],[-10.3,43.1,116.4,2.6,20.4,-11.6],[-6.2,45.4,110.2,2.2,24.2,-7.0],[-4.4,45.1,110.9,2.2,23.8,-5.2],[-4.5,44.1,113.6,2.4,22,-5.5],[-4.5,44.1,113.6,2.4,22,-36.5],[-6.6,43.6,115,2.5,21.2,-74.7],[-7.6,43.6,114.8,2.5,21.3,-75.7],[-8.7,44.7,112,2.3,23.1,-76.6],[-8.9,46.1,108.2,2.1,25.5,-76.6],[-9.6,47.4,105.1,2.0,27.4,-77.1],[-9.7,46.4,107.5,2.0,25.9,-52.3],[-4.1,50.3,111.9,2.8,17.4,-3.5],[-3.9,49.3,107.9,1.9,28.3,-0.8],[-3.9,49.6,107.3,1.9,31,-0.5],[-3.8,52,101.5,1.7,40,-0.1],[-3.8,52.3,101,1.7,42.8,0],[-3.7,53.6,97.5,1.7,45,0.1]
    #setJoints(desired_joint)

    #desired_pose = np.array([[320,30,155,180,0,180],[320,-45,155,180,0,180],[330,-45,155,180,0,180],[300,-30,155,180,0,180],[340,-10,155,180,0,180],[320,31,155,180,0,180],[270,20,130,180,30,180],[255,-50,130,180,30,180],[410,-40,130,180,-30,180],[410,20,130,180,-30,180]])
    desired_pose = np.array([[320,0,155,180,0,180],[320,0,155,180,0,0]])
    for i in range(desired_pose.shape[0]):
        current_pose = getCartesian()
        print current_pose
        for j in range(3,6):
            if current_pose[j] - desired_pose[i][j] > 350:
                current_pose[j] = current_pose[j] - 360
            else:
                if current_pose[j] - desired_pose[i][j] < -350:
                    current_pose[j] = current_pose[j] + 360
        step = max(abs(current_pose - desired_pose[i]))*10
	
        for j in range(int(round(step))):
            cmd_pose = np.array(current_pose)*(step-1-j)/(step-1) + desired_pose[i]*j/(step-1)
            print cmd_pose
            desired_quat = transform.quaternion_from_euler(math.radians(cmd_pose[3]),math.radians(cmd_pose[4]),math.radians(cmd_pose[5]),'sxyz')
            setCartesian(cmd_pose[0:3].tolist()+desired_quat.tolist())
            time.sleep(0.01)

