#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 20:28:00 2021

@author: MarkAPost

This program goes with the V-REP scenes for the Bug algorithm
It serves as an example for navigation of a simple differential-drive robot
"""
import sim
import time
import signal
import numpy as np
from puck_robot import PuckRobot

#Set up keyboard interrupt handling (to stop with Ctrl-C)
signal.signal(signal.SIGINT, signal.default_int_handler)

#Change this global variable to False to exit
RUN = True

#Change this global variable to True to print debugging information
DEBUG = True

#Amount of time to continue edge following behaviour and timer
FollowTime = 1.0
FollowEndTime = 0.0

def simpleAsyncBug0():

    def TurnToAngle(angle, width):
        robot.maxSpeed = 6
        #Convert the desired turn angle into forward and turning speeds
        forwardSpeed = robot.maxSpeed*np.cos(angle)
        turnSpeed = robot.maxSpeed*np.sin(angle)
        
        #Using differential drive kinematic equations, convert to motor speeds
        leftMotorSpeed = forwardSpeed / 2 + width * turnSpeed
        rightMotorSpeed = forwardSpeed / 2 - width * turnSpeed
        
        if DEBUG:
            print('<Robot ',robot.robotNum,'> Turning ',"{:3.2f}".format(angle*180/np.pi), \
            ' with speeds [',"{:.2f}".format(leftMotorSpeed),",{:.2f}".format(rightMotorSpeed),']')

        #Return the motor speeds to the desired angle
        return leftMotorSpeed,rightMotorSpeed


    def Move(robot):
        global RUN
        
        #The main point of control is to set the drive motor speeds
        leftMotorSpeed = 0
        rightMotorSpeed = 0

        #Find the relative direction of the nest
        rangeToNest, angleToNest = robot.findNestBeacon()
        
        #If robot is within range of the nest, stop
        if(rangeToNest < 0.5):
            print('Reached the goal at ', robot.timeCurrent)
            RUN = False
        #Otherwise, turn towards the goal
        else:
            leftMotorSpeed,rightMotorSpeed = TurnToAngle(angleToNest, robot.interWheelDistance)
            
        #Print debugging information if desired    
        if DEBUG:
            print('<Robot ',robot.robotNum,'> Move(): rangeToNest=',rangeToNest,'angleToNest=',"{:3.2f}".format(angleToNest*180/np.pi))

        #Return the desired motor speeds
        return leftMotorSpeed,rightMotorSpeed


    def Circumnavigate(robot):
        #Start with the previous motor speeds
        leftMotorSpeed = robot.leftMotorSpeed/4
        rightMotorSpeed = robot.rightMotorSpeed/4
        
        #Read the proximity sensors (possibly redundant here but used for clarity)
        distances = robot.getRangeSensors()
        
        #Add the vectors together to get the general vector to where obstacles are
        vectorToObstacle = np.array([0.0, 0.0])
        for sensorNum in range(0, len(distances)):
            # If there is an obstacle detected (not None) add the vector
            if distances[sensorNum]:
                vectorToObstacle = vectorToObstacle + \
                [distances[sensorNum] * np.cos(robot.sensorAngles[sensorNum]), \
                 distances[sensorNum] * np.sin(robot.sensorAngles[sensorNum])]
        
        #Calculate the angle between the robot's x-axis (its forward direction)
        #and the vector PARALLEL to the edge of the obstacle
        if np.arctan2(vectorToObstacle[1], vectorToObstacle[0]) > 0:
            #If vector to obstacle is to the right, follow to the left
            angleAlongEdge = np.arctan2(vectorToObstacle[1], vectorToObstacle[0]) - np.pi*2/3
        else:
            #Otherwise, follow to the right
            angleAlongEdge = np.arctan2(vectorToObstacle[1], vectorToObstacle[0]) + np.pi*2/3
        
        #Turn towards the desired angle
        leftMotorSpeed,rightMotorSpeed = TurnToAngle(angleAlongEdge, robot.interWheelDistance)
        
        #Print debugging information if desired    
        if DEBUG:
            print('碰撞向量:', vectorToObstacle)
            print('<Robot ',robot.robotNum,'> Round(): angleAlongEdge=',"{:3.2f}".format(angleAlongEdge*180/np.pi))
        
        #Return the desired motor speeds
        return leftMotorSpeed/2,rightMotorSpeed/2


    def RobotControl(robot):
        global FollowEndTime, FollowEnd

        #The main point of control is to set the drive motor speeds
        leftMotorSpeed = 0
        rightMotorSpeed = 0

        #Perform odometry and update robot time
        position, orientation, leftWheelOdom, rightWheelOdom, robotTime = robot.odometry()
        #Print out odometry information if desired
        if DEBUG:
            print("<Robot ",robot.robotNum,"> Pos:[", round(position[0], 2), round(position[1], 2), round(position[2], 2), "]", \
                "Rot:[", round(orientation[0], 2), round(orientation[1], 2), round(orientation[2], 2), "]\n", \
                "LWheelOdom:", "{:03.2f}".format(leftWheelOdom), \
                "RWheelOdom:", "{:03.2f}".format(rightWheelOdom), \
                "Time:"+"{:02.1f}".format(robotTime)+"s")
        
        #Read range sensors
        ranges = robot.getRangeSensors()

        #Put your state controller code here
        #If an obstacle is encountered, go around it
        if any(ranges):
            #Set timer for following the edge of obstacles
            FollowEndTime = robot.timeCurrent + FollowTime
        if(robot.timeCurrent < FollowEndTime):
            #Follow the edge of the obstacle based on range sensor input
            leftMotorSpeed,rightMotorSpeed = Circumnavigate(robot)
        #Otherwise, go towards the goal
        else:
            leftMotorSpeed,rightMotorSpeed = Move(robot)

        #Set motor speeds
        robot.setMotors(leftMotorSpeed, rightMotorSpeed)


    if 1:
        global RUN
        #Start control program and just in case, close all opened connections
        sim.simxFinish(-1)
        print('Program started')
        
        #Optional - start simulator running if it is not already
        simulator_clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)
        sim.simxStartSimulation(simulator_clientID, sim.simx_opmode_blocking)
        time.sleep(2)
        
        #Your robots should be numbered starting with 0 each opening remote API port 20000+(robot_number)
        portNumStart = 20000
        cycleTime = 0.1
        numRobots = 1
        robots = []
        connections = []
        
        #Create robot objects and connect to all robots
        for robotNum in range(0, numRobots):
            robots.append(PuckRobot(robotNum))
            robots[-1].connect(portNumStart+robotNum)
            connections.append(True)

        #Start main control loop
        print('Starting control loop')
        while(RUN):
            try:
                for robot in robots:
                    #Run control function if connected
                    if not robot.isConnected():
                        connections[robot.robotNum] = False
                    else:
                        RobotControl(robot)
                #Stop simulation if all robots are disconnected
                if not all(connections):
                    RUN = False
                #Sleep for a time in each cycle so as not to overload the host computer
                time.sleep(cycleTime)
            #Either stop the simulation or use Ctrl-C to stop your control script
            except KeyboardInterrupt:
                RUN = False

        #Disconnect all robots
        for robot in robots:
            robot.disconnect()
            connections[robot.robotNum] = False

        #Clean up all connections and close
        sim.simxFinish(simulator_clientID)
        print('Program ended')
