#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 18:18:18 2020

@author: MarkAPost

This is a robot class for the Puck Robot created by Robert H Peck in 2020
"""

import sim
import time
import numpy as np

class PuckRobot:
	def __init__(self, robotNum):
		#Program Variables
		self.robotNum = robotNum
		self.timeCurrent = time.time()
		print("Created robot %d" %(self.robotNum))

	#Useful public variables
	#Number of robot in simulation
	robotNum = 0
	#Simulation client ID
	clientID = -1
	#Simulation name of robot
	robotObjectName = ''
	#Simulation time of robot
	timeCurrent = 0.0
	#Maximum expected motor speed
	maxSpeed = 1.0
	#Useful array defining where the proximity sensors are pointed
	sensorAngles = [0,
				   30*np.pi/180,
				   60*np.pi/180,
				   90*np.pi/180,
				   135*np.pi/180,
				   180*np.pi/180,
				   -135*np.pi/180,
				   -90*np.pi/180,
				   -60*np.pi/180,
				   -30*np.pi/180]
	#Motor Variables
	leftMotorSpeed = 0.0
	rightMotorSpeed = 0.0
	leftWheelOdom = 0.0
	rightWheelOdom = 0.0
	lastLeftWheelPosition = 0.0
	lastRightWheelPosition = 0.0
	#Size of Wheels
	leftWheelDiameter = 0.09
	rightWheelDiameter = 0.09
	#Distance between wheels for odometry
	interWheelDistance = 0.16
	#Robot localization information
	position = [0,0,0]
	orientation = [0,0,0]
	#Value of food carried
	foodCarried = 0
	#Direction of trail found
	trailBearing = 0
	#Objects in environment
	nest = -1
	wall0_collisionID = -1
	wall1_collisionID = -1
	wall2_collisionID = -1
	wall3_collisionID = -1
	
	#robot parts
	proximitySensor = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
	
	def isConnected(self):
		return sim.simxGetConnectionId(self.clientID) != -1

	def connect(self, portNum):
		#Connect to simulator running on localhost
		#V-REP runs on port 19997
		#Each robot needs a separate port number
		#The sysCall_init() script function should open the remote API on ports starting with 20001 with simRemoteApi.start()
		#Then sysCall_cleanup() should stop the API on the same port with simRemoteApi.stop()
		self.clientID = sim.simxStart('127.0.0.1', portNum, True, True, 20000, 5)
		if self.clientID == -1:
			print('Robot %d could not connect on port %d to remote API server' %(self.robotNum, portNum))
			return -1
		print('Robot %d on port %d connected to remote API server' %(self.robotNum, portNum))
		#Get handles to simulation objects
		#Floor
		res,self.floor = sim.simxGetObjectHandle(self.clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to ResizableFloor_5_25')
		#Floor Colour Sensor
		res,self.floorColourSensor = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_floor_colour_Sensor#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_floor_colour_Sensor#'+str(self.robotNum))
		#Nest Beacon Sensor
		res,self.nestBeaconSensor = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_nest_beacon_sensor#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_nest_beacon_sensor#'+str(self.robotNum))
		#Body
		res,self.body = sim.simxGetObjectParent(self.clientID, self.floorColourSensor, sim.simx_opmode_oneshot_wait)
		#lack of a remote API function to get object associated with script is a HUGE oversight by Coppelia
		self.robotObjectName = 'Puck_robot#'+str(self.robotNum)
		#check to make sure estimated name is the same as the one the script is attached to
		res,bodyCheck = sim.simxGetObjectHandle(self.clientID, self.robotObjectName, sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok or self.body != bodyCheck: print('Could not get handle to '+self.robotObjectName)
		#Proximity Sensors [10]
		sensorMap = {0:'Puck_robot_front_IR#'+str(self.robotNum), 1:'Puck_robot_front_right_IR#'+str(self.robotNum), 
			   2:'Puck_robot_front_right_side_IR#'+str(self.robotNum), 3:'Puck_robot_right_IR#'+str(self.robotNum), 
			   4:'Puck_robot_rear_right_IR#'+str(self.robotNum), 5:'Puck_robot_rear_IR#'+str(self.robotNum), 
			   6:'Puck_robot_rear_left_IR#'+str(self.robotNum), 7:'Puck_robot_left_IR#'+str(self.robotNum), 
			   8:'Puck_robot_front_left_side_IR#'+str(self.robotNum), 9:'Puck_robot_front_left_IR#'+str(self.robotNum)}
		for sensorNum in range(0, len(self.proximitySensor)):
			res,self.proximitySensor[sensorNum] = sim.simxGetObjectHandle(self.clientID, sensorMap[sensorNum], sim.simx_opmode_oneshot_wait)
			if res != sim.simx_return_ok: print('Could not get handle to '+sensorMap[sensorNum])
		#Wheel drive motors
		res,self.leftMotor = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_Left_wheel_joint#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_Left_wheel_joint#'+str(self.robotNum))
		res,self.rightMotor = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_Right_wheel_joint#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_Right_wheel_joint#'+str(self.robotNum))
		#Wheels
		res,self.leftWheel = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_Wheel_left#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_Wheel_left#'+str(self.robotNum))
		res,self.rightWheel = sim.simxGetObjectHandle(self.clientID, 'Puck_robot_Wheel_right#'+str(self.robotNum), sim.simx_opmode_oneshot_wait)
		if res != sim.simx_return_ok: print('Could not get handle to Puck_robot_Wheel_right#'+str(self.robotNum))
		#Walls in the environment
		res, self.nest = sim.simxGetObjectHandle(self.clientID, "NestBeacon", sim.simx_opmode_oneshot_wait)
		#Walls in the environment
		res, self.wall0_collisionID = sim.simxGetCollisionHandle(self.clientID, "left_wall", sim.simx_opmode_oneshot_wait)
		res, self.wall1_collisionID = sim.simxGetCollisionHandle(self.clientID, "right_wall", sim.simx_opmode_oneshot_wait)
		#Calculate useful parameters
		leftWheelDia = \
		sim.simxGetObjectFloatParameter(self.clientID, self.leftWheel, 18, sim.simx_opmode_oneshot)[1] \
		- sim.simxGetObjectFloatParameter(self.clientID, self.leftWheel, 15, sim.simx_opmode_oneshot)[1]
		rightWheelDia = \
		sim.simxGetObjectFloatParameter(self.clientID, self.rightWheel, 18, sim.simx_opmode_oneshot)[1] \
		- sim.simxGetObjectFloatParameter(self.clientID, self.rightWheel, 15, sim.simx_opmode_oneshot)[1]
		axleVector = np.array(sim.simxGetObjectPosition(self.clientID, self.rightWheel, self.leftWheel, sim.simx_opmode_oneshot)[1])
		axleLength = np.linalg.norm(axleVector)
		if leftWheelDia > 0.0: #for some reason simxGetObjectFloatParameter always returns a zero
			self.leftWheelDiameter = leftWheelDia
		if rightWheelDia > 0.0: #for some reason simxGetObjectFloatParameter always returns a zero
			self.rightWheelDiameter = rightWheelDia
		if axleLength > 0.0: #for some reason simxGetObjectPosition always returns a zero vector
			self.interWheelDistance = axleLength
		return self.clientID

	def odometry(self):
		lasttime = self.timeCurrent
		self.timeCurrent = time.perf_counter()

		#Get wheel odometry directly from rotary joints in radians
		self.leftWheelPosition = sim.simxGetJointPosition(self.clientID, self.leftMotor, sim.simx_opmode_oneshot)[1]
		self.rightWheelPosition = sim.simxGetJointPosition(self.clientID, self.rightMotor, sim.simx_opmode_oneshot)[1]
		#Deal with the joint singularity
		dTheta = self.leftWheelPosition - self.lastLeftWheelPosition
		if dTheta > np.pi:
			dTheta -= 2*np.pi
		elif dTheta < -np.pi:
			dTheta += 2*np.pi
		self.leftWheelOdom += dTheta * self.leftWheelDiameter / 2
		self.lastLeftWheelPosition = self.leftWheelPosition
		dTheta = self.rightWheelPosition - self.lastRightWheelPosition
		if dTheta > np.pi:
			dTheta -= 2*np.pi
		elif dTheta < -np.pi:
			dTheta += 2*np.pi
		self.rightWheelOdom += dTheta * self.rightWheelDiameter / 2
		self.lastRightWheelPosition = self.rightWheelPosition

		#Return position tracking information
		self.position = sim.simxGetObjectPosition(self.clientID, self.body, self.floor, sim.simx_opmode_oneshot)[1]
		self.orientation = sim.simxGetObjectOrientation(self.clientID, self.body, self.floor, sim.simx_opmode_oneshot)[1]
		return self.position, self.orientation, self.leftWheelOdom, self.rightWheelOdom, self.timeCurrent - lasttime

	def getRangeSensors(self):
		#Read proximity sensors and place distances that obstacles are detected from sensor into an array
		#If no obstacle is detected the array will contain a "None" type for that sensor
		ranges = []
		for sensor in self.proximitySensor:
			res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
			sim.simxReadProximitySensor(self.clientID, sensor, sim.simx_opmode_oneshot)
			if detectionState:
				ranges.append(np.sqrt(detectedPoint[0]**2+detectedPoint[1]**2))
			else:
				ranges.append(None) 
		return ranges

	def setMotors(self, leftMotorSpeed, RightMotorSpeed):
		#Update internal motor speed variables
		self.leftMotorSpeed = leftMotorSpeed
		self.rightMotorSpeed = RightMotorSpeed

		#Set actuators on mobile robot (need to use internal Lua script to fix robot movement)
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'SetDriveMotors',
			[],
			[self.leftMotorSpeed, self.rightMotorSpeed],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('SetDriveMotors returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def collectFood(self):
		#Try collecting food under the robot using the floor colour and proximity sensors
		#The function outputs zero or the value of the food picked up
		#This is much more easily done with internal lua functions so we call them
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'FoodCollection',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('FoodCollection returned: ',res, outInts, outFloats, outStrings, outBuffer)
		self.foodCarried = outInts[0]
		return self.foodCarried

	def dropFood(self):
		#Drop the food that is being carried
		#Returns a True if successful and False otherwise
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'DropFood',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('DropFood returned: ',res, outInts, outFloats, outStrings, outBuffer)
		if outInts[0] == 1:
			self.foodCarried = 0
			return True
		else:
			return False

	def findNestBeacon(self):
		#Get range and bearing to nest beacon
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'FindNestBeacon',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('FindNestBeacon returned: ',res, outInts, outFloats, outStrings, outBuffer)
		if not outFloats:
			rangeToNest = None
			bearingToNest = None
		else:
			rangeToNest = outFloats[0]
			bearingToNest = outFloats[1]
		return rangeToNest, bearingToNest

	def leaveTrail(self):
		#Leave a blue trail marker with robot's orientation, returns True if successful and False otherwise
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'LeaveTrail',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('LeaveTrail returned: ',res, outInts, outFloats, outStrings, outBuffer)
		if outInts[0] == 1:
			self.foodCarried = 0
			return True
		else:
			return False
		return True

	def findTrail(self):
		#Get last seen pheremone trail bearing
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'GetTrailBearing',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('GetTrailBearing returned: ',res, outInts, outFloats, outStrings, outBuffer)
		self.trailBearing = outFloats[0]
		return self.trailBearing

	def SendToWifi(self, targetNum, message):
		#Sends a text string message to robot number targetNum
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'SendToWifi',
			[self.robotNum, targetNum],
			[],
			[message],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('SendToWifi returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def ReadFromWifi(self):
		#Sends a text string message to robot number targetNum
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'ReadFromWifi',
			[self.robotNum],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('ReadFromWifi returned: ',res, outInts, outFloats, outStrings, outBuffer)
		if len(outInts) >= 2 and len(outStrings) >= 1:
			message = outStrings[0]
			senderNum = outInts[1]
			remainingQueueLength = outInts[2]
			return message,senderNum,remainingQueueLength
		else:
			return '',None,0

	def EnableBeacon(self, numberMessage):
		#Broadcasts an integer number message to all beacons within range
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'EnableBeacon',
			[numberMessage],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('EnableBeacon returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def DisableBeacon(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'DisableBeacon',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('DisableBeacon returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def ReadBeaconRanging(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'ReadBeaconRanging',
			[],
			[],
			[],
			bytearray(10, 0),
			sim.simx_opmode_blocking)
		if res != 0:
			print('ReadBeaconRanging returned: ',res, outInts, outFloats, outStrings, outBuffer)
		readings = outInts[0]
		angle = outFloats[0]
		return readings,angle

	def ReadBeaconMessaging(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'ReadBeaconMessaging',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('ReadBeaconMessaging returned: ',res, outInts, outFloats, outStrings, outBuffer)
		beaconOn = outInts[0]
		senderNum = outInts[1]
		if len(outInts) >= 3:
			numberMessage = outInts[2]
		else:
			numberMessage = []
		return beaconOn,senderNum,numberMessage

	def EnableLongBeacon(self, numberMessage):
		#Broadcasts an integer number message to all beacons within range
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'EnableLongBeacon',
			[numberMessage],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('EnableLongBeacon returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def DisableLongBeacon(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'DisableLongBeacon',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('DisableLongBeacon returned: ',res, outInts, outFloats, outStrings, outBuffer)

	def ReadLongBeaconRanging(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'ReadLongBeaconRanging',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('ReadLongBeaconRanging returned: ',res, outInts, outFloats, outStrings, outBuffer)
		readings = outInts[0]
		angle = outFloats[0]
		return readings,angle

	def ReadLongBeaconMessaging(self):
		#Turns off the beacon broadcast
		res, outInts, outFloats, outStrings, outBuffer = sim.simxCallScriptFunction(
			self.clientID,
			self.robotObjectName,
			sim.sim_scripttype_childscript,
			'ReadLongBeaconMessaging',
			[],
			[],
			[],
			bytearray(),
			sim.simx_opmode_blocking)
		if res != 0:
			print('ReadLongBeaconMessaging returned: ',res, outInts, outFloats, outStrings, outBuffer)
		beaconOn = outInts[0]
		senderNum = outInts[1]
		if len(outInts) >= 3:
			numberMessage = outInts[2]
		else:
			numberMessage = []
		return beaconOn,senderNum,numberMessage

	def isCollisionWithWall(self):
		#judge if collision happens
		for wallID in [self.wall0_collisionID, self.wall1_collisionID, self.wall2_collisionID, self.wall3_collisionID]:
			res, status = sim.simxReadCollision(self.clientID, wallID, sim.simx_opmode_oneshot_wait)
			if status:
				print("Collision has been detected!")
				return True
		return False

	def isCollisionWithNest(self):
		res, targetDistance = sim.simxCheckDistance(self.clientID, self.body, self.nest, sim.simx_opmode_oneshot_wait)
		if not res and targetDistance < 0.3:
			print("Target has been Reached!")
			return True
		return False

	def disconnect(self):
		#Stop robot from moving (important for real robots!)
		self.leftMotorSpeed = 0.0
		self.rightMotorSpeed = 0.0
		self.setMotors(self.leftMotorSpeed, self.rightMotorSpeed)
		#End simulation
		sim.simxFinish(self.clientID)
		print('Client %d Robot %d disconnected' %(self.clientID, self.robotNum))
