#!/usr/bin/env python
#@package foxbot_node
#This package defines a ROS node for the foxbot robot. In order to run this code, please load the fxscript defined in ../fxscript/ROSnode/ros.pac on the foxbot and then run the script after initializing the robot. Once the script is running, run the rosnode with rosrun foxbot robot_node.py. 
#
#The node spawns 2 socket connections with the ros node behaving as the client. The first socket created is the logger socket which sets up a repeated stream of joint and cartesian configurations and publishes on the JointsLog and CartesianLog topics. The second socket is a service socket. The ROS node advertises services that can then be used to command the robot. 
#


import rospy 
import struct
import threading
import socket
import json 
import sys
import math
import signal
import logging
import numpy as np
from collections import defaultdict
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped,TransformStamped
import tf
import tf.transformations as transform
from foxbot.srv import *
from foxbot.msg import BoolStamped

rospy.init_node('foxbot_node', anonymous=True)

FORMAT_STRING = '<hhHddddddhhddddddh'
MESSAGE_LEN = 108

DEFAULT_FIG = 1
ROBOT_GET_STATE = 2
ROBOT_GET_CARTESIAN = 3
ROBOT_GET_FK = 4
ROBOT_GET_IK = 5
ROBOT_GET_JOINTS = 6 
ROBOT_PING = 7
ROBOT_APPROACH = 8
ROBOT_SET_CARTESIAN = 9
ROBOT_SET_CARTESIAN_J = 10
ROBOT_SET_COMM = 11
ROBOT_SET_DEFAULT = 12
ROBOT_SET_JOINTS = 13
ROBOT_JOG_JOINTS = 21
ROBOT_SET_SPEED = 14
ROBOT_SET_TOOL = 15
ROBOT_SET_TRACK_DISTANCE = 16
ROBOT_SET_TRACKING = 17
ROBOT_STOP_TRACKING = 24
ROBOT_SET_VACUUM_ON = 21
ROBOT_SET_VACUUM_OFF = 22
ROBOT_STOP = 23
ROBOT_SET_ZONE = 18
ROBOT_SET_WORK_OBJ = 19
ROBOT_SPL_CMD = 20

class DelayedKeyboardInterrupt(object):
    def __enter__(self):
        self.signal_received = False
        self.old_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self.handler)

    def handler(self, signal, frame):
        self.signal_received = (signal, frame)
        logging.debug('SIGINT received. Delaying KeyboardInterrupt.')

    def __exit__(self, type, value, traceback):
        signal.signal(signal.SIGINT, self.old_handler)
        if self.signal_received:
            self.old_handler(*self.signal_received)


def pack(data):
    if not data.has_key('IsMoving'):
        data['IsMoving'] = 0
    if not data.has_key('frameId'):
        data['frameId'] = 0
    return struct.pack(FORMAT_STRING,data['tType'],data['tNum'],data['IsMoving'],data['Px'],data['Py'],data['Pz'],data['Rx'],data['Ry'],data['Rz'],data['fig'],data['frameId'],data['J1'],data['J2'],data['J3'],data['J4'],data['J5'],data['J6'],data['rVal'])

def unpack(msg):
    unpacked = struct.unpack(FORMAT_STRING,msg)
    data = dict()
    data['tType'] = unpacked[0]
    data['tNum'] = unpacked[1]
    data['IsMoving'] = unpacked[2]
    data['Px'] = unpacked[3]
    data['Py'] = unpacked[4]
    data['Pz'] = unpacked[5]
    data['Rx'] = unpacked[6]
    data['Ry'] = unpacked[7]
    data['Rz'] = unpacked[8]
    data['fig'] = unpacked[9]
    data['frameId'] = unpacked[10]
    data['J1'] = unpacked[11]
    data['J2'] = unpacked[12]
    data['J3'] = unpacked[13]
    data['J4'] = unpacked[14]
    data['J5'] = unpacked[15]
    data['J6'] = unpacked[16]
    data['rVal'] = unpacked[17]
    return data

class Foxbot():

    #Sockets
    serviceSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    loggerSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #Params
    destinationInfo = ('192.168.1.22', 5000)
    loggerInfo = ('', 5000)
    connectionSleep = 1

    #Threading Locks
    serviceLock = threading.Lock()

    #Transforms
    broadcast = tf.TransformBroadcaster()
    listener = tf.TransformListener()


    def __init__(self):
        self.connect()
        self.advertise()
        rospy.loginfo('Starting Transform Broadcaster')
        self.broadcast.sendTransform((0,0,0),np.array([0,0,0,1]),rospy.Time.now(),'foxbot_base','world')
        #self.broadcast.sendTransform((0,0,0),np.array([0,0,0,1]),rospy.Time.now(),'foxbot_work','foxbot_base')
        #self.broadcast.sendTransform((0,0,0),np.array([0,0,0,1]),rospy.Time.now(),'foxbot_base','foxbot_flange')
        #self.broadcast.sendTransform((0,0,0),np.array([0,0,0,1]),rospy.Time.now(),'foxbot_base','foxbot_tool')
        self.logPublisherJ = rospy.Publisher('/robot_JointsLog', JointState, queue_size=10)
        self.logPublisherC = rospy.Publisher('/robot_CartesianLog', PoseStamped, queue_size=10)
        self.isMovingPublisher = rospy.Publisher('/robot_IsMoving', BoolStamped, queue_size=10)


    def connect(self):
        try:
            
            self.loggerSocket.bind(self.loggerInfo)
            self.loggerSocket.listen(1)
            rospy.loginfo('Waiting for connection on ' + ''.join(str(self.loggerInfo)))
            self.loggerConnection, addr = self.loggerSocket.accept() # blocking
            rospy.loginfo('Logger socket received connection from ' + str(addr))

            self.serviceSocket.connect(self.destinationInfo)
            rospy.loginfo('Service Socket Connected')
            rospy.sleep(self.connectionSleep)

        except:
            rospy.logerr('Unable to Connect to Foxbot Server')
            raise

        rospy.loginfo('Sending Service and Logging Identifiers')
        # self.loggerLock.acquire()
        # self.loggerSocket.sendall('logger')
        # self.loggerLock.release()
        rospy.sleep(self.connectionSleep)
        self.serviceLock.acquire()
        self.serviceSocket.sendall('service')
        self.serviceLock.release()

    def disconnect(self):
        try: 
            rospy.sleep(self.connectionSleep)
            self.serviceSocket.shutdown(1)
            rospy.sleep(self.connectionSleep)
            self.serviceSocket.close()
            self.loggerSocket.shutdown(1)
            rospy.sleep(self.connectionSleep)
            self.loggerSocket.close()
        except:
            rospy.logerr('Error Closing Socket Connections')
    def run_logger(self):
        # try:
        #     with DelayedKeyboardInterrupt():
        #         # self.loggerLock.acquire()
        #         # self.recvBuffer = self.loggerSocket.recv(1024)
        #         # self.loggerLock.release()
        #         data = self.recv(loggerSocket,loggerLock)
        # except:
        #     pass
      try:
        recvMsg = self.loggerConnection.recv(MESSAGE_LEN)
        # print ":".join("{:02x}".format(ord(c)) for c in recvMsg)
        data = unpack(recvMsg)
        now = rospy.get_rostime()

        #Assemble Joints message
        joints = JointState()
        joints.header.stamp = now
        for i in range(0,6):
            joints.name.append('J' + str(i+1))
            joints.position.append(math.radians(data[joints.name[i]]))
            #Assemble Pose Message
        self.logPublisherJ.publish(joints)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.pose.position.x = data['Px']
        pose.pose.position.y = data['Py']
        pose.pose.position.z = data['Pz']
        quat = transform.quaternion_from_euler(math.radians(data['Rx']),math.radians(data['Ry']),math.radians(data['Rz']))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.logPublisherC.publish(pose)
        self.broadcast.sendTransform((pose.pose.position.x/100,pose.pose.position.y/100,pose.pose.position.z/100),quat,now,'foxbot_tool','foxbot_base')
        # unit in 0.1 m
        isMoving = BoolStamped()
        isMoving.header.stamp = now
        isMoving.data = data['IsMoving'] is not 0
        self.isMovingPublisher.publish(isMoving)

        # self.broadcast.sendTransform((data['Px'],data['Py'],data['Pz']),quat,now,'foxbot_base','foxbot_flange')
        # (trans,rot) = self.listener.waitforTransform('foxbot_tool','foxbot_work',now)
        # pose = PoseStamped()
        # pose.header.stamp = now
        # pose.header.frame_id = 'foxbot_work'
        # pose.position.x = trans[0]
        # pose.position.y = trans[1]
        # pose.position.z = trans[2]
        # pose.pose.orientation.x = rot[0]
        # pose.pose.orientation.y = rot[1]
        # pose.pose.orientation.z = rot[2]
        # pose.pose.orientation.w = rot[3]
        # self.logPublisherC.publish(pose)

        # rospy.loginfo('Updated robot info')
      except Exception as ex:
        #  rospy.logerr('Exception receiving log message: ' + str(ex))
        raise ex

    # Service Handlers
    def handle_robot_get_state(self,req):
        response = robot_GetStateResponse()
        response.tcp = 50
        response.ori = 10
        response.zone = 1
        response.vacuum = 1
        response.workx = 1
        response.worky = 2
        response.workz = 1
        return response

    def handle_robot_get_cartesian(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_GET_CARTESIAN
        data['rVal'] = 1
        data['frameId'] = req.frameId
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        quat = transform.quaternion_from_euler(math.radians(rData['Rx']),math.radians(rData['Ry']),math.radians(rData['Rz']))
        return robot_GetCartesianResponse(x=rData['Px'],y=rData['Py'],z=rData['Pz'],q0=quat[3],qx=quat[0], qy=quat[1],qz=quat[2], frameId=rData['frameId'], ret=rData['rVal'], msg='SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')


    def handle_robot_get_fk(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_GET_FK
        data['rVal'] = 1
        data['J1'] = req.j1
        data['J2'] = req.j2
        data['J3'] = req.j3
        data['J4'] = req.j4
        data['J5'] = req.j5
        data['J6'] = req.j6
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        quat = transform.quaternion_from_euler(math.radians(rData['Rx']),math.radians(rData['Ry']),math.radians(rData['Rz']))
        return robot_GetFKResponse(x=rData['Px'],y=rData['Py'],z=rData['Pz'],q0=quat[3],qx=quat[0], qy=quat[1],qz=quat[2], ret=rData['rVal'], msg='SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_get_ik(self,req):
        euler_angles= transform.quaternion_from_euler([req.qx, req.qy, req.qz, req.q0])
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_GET_IK
        data['rVal'] = 1
        data['Px'] = req.x
        data['Py'] = req.y
        data['Pz'] = req.z
        data['Rx'] = euler_angles[0]
        data['Ry'] = euler_angles[1]
        data['Rz'] = euler_angles[2]
        data['fig'] = DEFAULT_FIG
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_GetIKResponse(rData['J1'], rData['J2'], rData['J3'], rData['J4'], rData['J5'], rData['J6'], rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_get_joints(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_GET_JOINTS
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_GetJointsResponse(rData['J1'], rData['J2'], rData['J3'], rData['J4'], rData['J5'], rData['J6'], rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_ping(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_PING
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_PingResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_stop(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_STOP
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_StopResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_approach(self,req):
        return robot_ApproachResponse()

    def handle_robot_set_cartesianJ(self,req):
        euler_angles= transform.euler_from_quaternion([req.qx, req.qy, req.qz, req.q0])
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_SET_CARTESIAN_J
        data['rVal'] = 1
        data['Px'] = req.x
        data['Py'] = req.y
        data['Pz'] = req.z
        data['Rx'] = math.degrees(euler_angles[0])
        data['Ry'] = math.degrees(euler_angles[1])
        data['Rz'] = math.degrees(euler_angles[2])
        data['fig'] = DEFAULT_FIG
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_SetCartesianJResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_set_cartesian(self,req):
        euler_angles= transform.euler_from_quaternion([req.qx, req.qy, req.qz, req.q0])
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_SET_CARTESIAN
        data['rVal'] = 1
        data['Px'] = req.x
        data['Py'] = req.y
        data['Pz'] = req.z
        data['Rx'] = math.degrees(euler_angles[0])
        data['Ry'] = math.degrees(euler_angles[1])
        data['Rz'] = math.degrees(euler_angles[2])
        data['frameId'] = req.frameId
        data['fig'] = DEFAULT_FIG
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_SetCartesianResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_jog_cartesian(self, req):
        curData = defaultdict(int)
        curData['tType'] = 2
        curData['tNum'] = ROBOT_GET_CARTESIAN
        curData['rVal'] = 1
        intData = self.sendrecv(self.serviceSocket,self.serviceLock,curData)
        if intData['rVal'] is not 1:
            return robot_JogCartesianResponse(intData['rVal'], 'SERVER_ERROR')
        intData['tType'] = 2
        intData['tNum'] = ROBOT_SET_CARTESIAN
        intData['Px'] += req.x
        intData['Py'] += req.y
        intData['Pz'] += req.z
	intData['Rx'] += req.rx
	intData['Ry'] += req.ry
	intData['Rz'] += req.rz
        quat = transform.quaternion_from_euler(math.radians(intData['Rx']),math.radians(intData['Ry']),math.radians(intData['Rz']))
        rData = self.sendrecv(self.serviceSocket,self.serviceLock, intData)
        return robot_JogCartesianResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_set_comm(self,req):
        return robot_SetCommResponse()
    def handle_robot_set_defaults(self,req):
        return robot_SetDefaultsResponse()
    def handle_robot_set_joints(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_SET_JOINTS
        data['rVal'] = 1
        data['J1'] = req.j1
        data['J2'] = req.j2
        data['J3'] = req.j3
        data['J4'] = req.j4
        data['J5'] = req.j5
        data['J6'] = req.j6
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_SetJointsResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_jog_joints(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_GET_JOINTS
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        data['tNum'] = ROBOT_SET_JOINTS
        data['J1'] = rData['J1'] + req.j1
        data['J2'] = rData['J2'] + req.j2
        data['J3'] = rData['J3'] + req.j3
        data['J4'] = rData['J4'] + req.j4
        data['J5'] = rData['J5'] + req.j5
        data['J6'] = rData['J6'] + req.j6
	rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_JogJointsResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')

    def handle_robot_set_speed(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_SET_SPEED
        data['rVal'] = 1
        data['Px'] = req.tcp
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_SetSpeedResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_set_tracking(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_SET_TRACKING
        data['rVal'] = 1
        data['Px'] = req.vx
        data['Py'] = req.vy
        data['Pz'] = req.vz
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_SetTrackingResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_stop_tracking(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        data['tNum'] = ROBOT_STOP_TRACKING
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket,self.serviceLock,data)
        return robot_StopTrackingResponse(rData['rVal'], 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_set_tool(self,req):
        return robot_SetToolResponse(1, 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_set_vacuum(self,req):
        data = defaultdict(int)
        data['tType'] = 2
        if req.vacuum:
            data['tNum'] = ROBOT_SET_VACUUM_ON
        else:
            data['tNum'] = ROBOT_SET_VACUUM_OFF
        data['rVal'] = 1
        rData = self.sendrecv(self.serviceSocket, self.serviceLock, data)
        return robot_SetVacuumResponse(1, 'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_set_zone(self,req):
        return robot_SetZoneResponse(1,'SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_set_work_object(self,req):
        return robot_SetWorkObjectResponse('SERVER_OK' if (rData['rVal'] is 1) else 'SERVER_ERROR')
    def handle_robot_special_command(self,req):
        return robot_SpecialCommandResponse()

# Socket Functions
    def sendrecv(self, hSocket, hLock, data):
        sendMsg = pack(data)
        hLock.acquire()
        hSocket.sendall(sendMsg)
        recvMsg = hSocket.recv(MESSAGE_LEN)
        hLock.release()
        rospy.loginfo('Sent {0} bytes, recv {1} bytes'.format(len(data), len(recvMsg)))
        return unpack(recvMsg)


    def advertise(self):
        #Advertise all foxbot services
        try:
            rospy.loginfo('Advertising services')
            self.getStateSrv = rospy.Service('robot_GetState',robot_GetState,self.handle_robot_get_state)
            self.getCartesianSrv = rospy.Service('robot_GetCartesian',robot_GetCartesian,self.handle_robot_get_cartesian)
            self.getFKSrv = rospy.Service('robot_GetFK',robot_GetFK,self.handle_robot_get_fk)
            self.getIKSrv = rospy.Service('robot_GetIK',robot_GetIK,self.handle_robot_get_ik)
            self.getJointsSrv = rospy.Service('robot_GetJoints',robot_GetJoints,self.handle_robot_get_joints)
            self.pingSrv = rospy.Service('robot_Ping',robot_Ping,self.handle_robot_ping)
            self.approachSrv = rospy.Service('robot_Approach',robot_Approach,self.handle_robot_approach)
            self.setCartesianJSrv = rospy.Service('robot_SetCartesianJ',robot_SetCartesianJ,self.handle_robot_set_cartesianJ)
            self.setCartesianSrv = rospy.Service('robot_SetCartesian',robot_SetCartesian,self.handle_robot_set_cartesian)
            self.jogCartesian = rospy.Service('robot_JogCartesian', robot_JogCartesian, self.handle_robot_jog_cartesian)
            self.setCommSrv = rospy.Service('robot_SetComm',robot_SetComm,self.handle_robot_set_comm)
            self.setDefaultsSrv = rospy.Service('robot_SetDefaults',robot_SetDefaults,self.handle_robot_set_defaults)
            self.setJointsSrv = rospy.Service('robot_SetJoints',robot_SetJoints,self.handle_robot_set_joints)
            self.jogJointsSrv = rospy.Service('robot_JogJoints',robot_JogJoints,self.handle_robot_jog_joints)
            self.setSpeedSrv = rospy.Service('robot_SetSpeed',robot_SetSpeed,self.handle_robot_set_speed)
            self.setToolSrv = rospy.Service('robot_SetTool',robot_SetTool,self.handle_robot_set_tool)
            # self.setTrackDistSrv = rospy.Service('robot_SetTrackDist',robot_SetTrackDist,self.handle_robot_set_track_dist)
            self.setTrackingSrv = rospy.Service('robot_SetTracking',robot_SetTracking,self.handle_robot_set_tracking)
            self.stopTrackingSrv = rospy.Service('robot_StopTracking',robot_StopTracking,self.handle_robot_stop_tracking)
            self.setVacuumSrv = rospy.Service('robot_SetVacuum',robot_SetVacuum,self.handle_robot_set_vacuum)
            self.setZoneSrv = rospy.Service('robot_SetZone',robot_SetZone,self.handle_robot_set_zone)
            self.setWorkObjectSrv = rospy.Service('robot_SetWorkObject',robot_SetWorkObject,self.handle_robot_set_work_object)
            self.specialCommandSrv = rospy.Service('robot_SpecialCommand',robot_SpecialCommand,self.handle_robot_special_command)
            self.stopSrv = rospy.Service('robot_Stop',robot_Stop,self.handle_robot_stop)
        except:
            rospy.logerr('Unable to advertise services')
            raise

def main():
    rospy.loginfo('Node Started')
    while not rospy.is_shutdown():
        foxbot = Foxbot()
        try:
            while not rospy.is_shutdown():
                foxbot.run_logger()
        except Exception as ex:
            rospy.logerr("Caught error: " + str(ex))
            foxbot.disconnect()

    rospy.loginfo('Shutting Down')
    foxbot.disconnect()




if __name__ == '__main__':
    main()
