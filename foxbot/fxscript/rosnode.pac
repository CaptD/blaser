Dim ROBOT_LOG%
Dim ROBOT_GET_STATE%
Dim ROBOT_GET_CARTESIAN%
Dim ROBOT_GET_FK%
Dim ROBOT_GET_IK%
Dim ROBOT_GET_JOINTS% 
Dim ROBOT_PING%
Dim ROBOT_APPROACH%
Dim ROBOT_SET_CARTESIAN%
Dim ROBOT_SET_CARTESIAN_J%
Dim ROBOT_SET_COMM%
Dim ROBOT_SET_DEFAULT%
Dim ROBOT_SET_JOINTS%
Dim ROBOT_SET_SPEED%
Dim ROBOT_SET_TOOL%
Dim ROBOT_SET_TRACK_DISTANCE%
Dim ROBOT_SET_VACUUM_ON%
Dim ROBOT_SET_VACUUM_OFF%
Dim ROBOT_SET_ZONE%
Dim ROBOT_SET_WORK_OBJ%
Dim ROBOT_SPL_CMD%
Dim MODE%
Dim LOCK%


Dim intSpeed As Double

ROBOT_LOG = 1
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
ROBOT_SET_SPEED = 14
ROBOT_SET_TOOL = 15
ROBOT_SET_TRACK_DISTANCE = 16

ROBOT_SET_ZONE = 18
ROBOT_SET_WORK_OBJ = 19
ROBOT_SPL_CMD = 20
ROBOT_SET_VACUUM_ON = 21
ROBOT_SET_VACUUM_OFF = 22
MODE = 0
LOCK = 0
intSpeed=0
'Main message type:

Type type1
tType As Integer
tNum As Integer
Px As Double
Py As Double
Pz As Double
Rx As Double
Ry As Double
Rz As Double
fig As Integer
J1 As Double
J2 As Double 
J3 As Double
J4 As Double
J5 As Double
J6 As Double
rVal As Integer
End Type


'Main function call does nothing. All operations are handled in the socket handler threads.
Sub Main
	Do
		Dim posP As PoseP
		Dim posJ As PoseJ
		Dim MovP As PoseP
		Dim retMsg As type1
		Dim recvMsg As type1
		'Acquire
		If TcpRecvEx(0,recvMsg)>0 Then
			MODE=1
			'Print recvMsg.tNum
		End If
		If MODE=1 And recvMsg.tNum<>30322 Then
			Select Case recvMsg.tNum
				Case 7
					retMsg.tType = 1
					retMsg.tNum = ROBOT_PING
					retMsg.rVal = 1
				Case 3
					retMsg.tType = 1
					retMsg.tNum = ROBOT_GET_CARTESIAN
					CurPos posP
					retMsg.Px = posP.x : retMsg.Py = posP.y : retMsg.Pz = posP.z
					retMsg.Rx = posP.rx : retMsg.Ry = posP.ry : retMsg.Rz = posP.rz					
					retMSg.fig = posP.fig
					retMsg.rVal = 1
				Case 6
					retMsg.tType = 1
					retMsg.tNum = ROBOT_GET_JOINTS
					CurPos posJ
					retMsg.J1 = posJ.j1 : retMsg.J2 = posJ.j2 : retMsg.J3 = posJ.j3
					retMsg.J4 = posJ.j4 : retMsg.J5 = posJ.j5 : retMsg.J6 = posJ.j6
					retMsg.rVal = 1
				Case 4
					SetJ posJ,recvMsg.J1,recvMsg.J2,recvMsg.J3,recvMsg.J4,recvMsg.J5,recvMsg.J6
					J2P posJ,posP
					retMsg.tType = 1
					retMsg.tNum = ROBOT_GET_FK
					retMsg.Px = posP.x : retMsg.Py = posP.y : retMsg.Pz = posP.z
					retMsg.Rx = posP.rx : retMsg.Ry = posP.ry : retMsg.Rz = posP.rz
					retMsg.fig = posP.fig
					retMsg.rVal = 1					
				Case 5
					SetP posP, recvMsg.Px, recvMsg.Py, recvMsg.Pz, recvMsg.Rx, recvMsg.Ry, recvMsg.Rz,recvMsg.fig
					P2J posP,posJ 
					retMsg.tType = 1
					retMsg.tNum = ROBOT_GET_IK
					retMsg.J1 = posJ.j1 : retMsg.J2 = posJ.j2 : retMsg.J3 = posJ.j3					
					retMsg.J4 = posJ.j4 : retMsg.J5 = posJ.j5 : retMsg.J6 = posJ.j6
					retMsg.rVal = 1					
				Case 9 'ROBOT_SET_CARTESIAN
					SetP MovP, recvMsg.Px, recvMsg.Py, recvMsg.Pz, recvMsg.Rx, recvMsg.Ry, recvMsg.Rz,recvMsg.fig					
					'SetP MovP, 310, 0, 500, 180, 0, 180,1
					'Print MovP.x
					'Print MovP.y
					'Print MovP.z
					'Print MovP.Rx
					'Print MovP.Ry
					'Print MovP.Rz
					'Print MovP.fig
					Move L, @P, MovP, 100
					retMsg.tNum = ROBOT_SET_CARTESIAN
					retMsg.rVal = 1
				Case 10 'ROBOT_SET_CARTESIAN_J
					SetP MovP, recvMsg.Px, recvMsg.Py, recvMsg.Pz, recvMsg.Rx, recvMsg.Ry, recvMsg.Rz,recvMsg.fig
					Move P, @E, MovP, 100
					retMsg.tType = 1
					retMsg.tNum = ROBOT_SET_CARTESIAN_J
					retMsg.rVal = 1
				Case 13 'ROBOT_SET_JOINTS
					SetJ posJ, recvMsg.J1, recvMsg.J2, recvMsg.J3, recvMsg.J4, recvMsg.J5, recvMsg.J6
					'J2P posJ,MovP
					Move P, @E, posJ, 100
					retMsg.tType = 1
					retMsg.tNum = ROBOT_SET_JOINTS
					retMsg.rVal = 1
				Case 14 'ROBOT_SET_SPEED
					intSpeed = recvMsg.Px
					Speed Vel(intSpeed), False					
					retMsg.rVal = 1
					retMsg.tType = 1
					retMsg.tNum = ROBOT_SET_SPEED
				Case 21 'ROBOT_SET_VACUUM_ON
					SetOutp PB, 15, True 
					retMsg.rVal = 1
					retMsg.tType = 1
					retMsg.tNum = ROBOT_SET_VACUUM_ON	
				Case 22 'ROBOT_SET_VACUUM_OFF
					SetOutp PB, 15, False 
					retMsg.rVal = 1
					retMsg.tType = 1
					retMsg.tNum = ROBOT_SET_VACUUM_OFF			
				Case Else
					Print "Unknown Command"
					retMsg.tType = 1
					retMsg.tNum = recvMsg.tNum					
					retMsg.rval = -1
			End Select
			If TcpSendEx(0,retMsg)>0 Then
			End If
			MODE=0
		End If 
		'Release
	'Print "Main"
	Loop
End Sub

'Need to put in some smarts here. 
Sub Callback(State As Integer)
	Select Case State
		'Add your callback procedures here
		Case STA  'Start
			
		Case STP  'Stop
			
		Case PAU  'Pause
			
		Case CNT  'Continue
			
	End Select
End Sub
	
'Main Callback for TCP/IP connections. 
Sub TcpServer
'	Acquire
	'If TcpRecvEx(0,recvMsg)>0 Then
	'MODE=1
	'End If	
	'Release
End Sub
		
Sub Acquire
	Do While LOCK<>0
	Loop
	LOCK=1
End Sub
Sub Release
	LOCK=0
End Sub
