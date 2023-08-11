import sim
import time
import cv2
import numpy as np
import _thread
from bug0 import simpleAsyncBug0
from core.driver import Driver as D
from service.control import Control
from core.const import const

def asyncExecuteBug0():
    try:
        _thread.start_new_thread(simpleAsyncBug0, () )
        print('bug0 thread create success')
    except Exception as ex:
        print(ex)


def p(*args):
    const.debug and print(args) 


def TurnToAngle(angle, width):
    #Convert the desired turn angle into forward and turning speeds
    forwardSpeed = 5*np.cos(angle)
    turnSpeed = 5*np.sin(angle)
    
    #Using differential drive kinematic equations, convert to motor speeds
    leftMotorSpeed = forwardSpeed / 2 + width * turnSpeed
    rightMotorSpeed = forwardSpeed / 2 - width * turnSpeed
    
    if const.debug:
        #print('<Robot ''> Turning ',"{:3.2f}".format(angle*180/np.pi), \
                #' with speeds [',"{:.2f}".format(leftMotorSpeed),",{:.2f}".format(rightMotorSpeed),']')
        pass

    #Return the motor speeds to the desired angle
    return leftMotorSpeed,rightMotorSpeed

def main ():
    print("wasd控制方向，p暂停汽车+查看日志[打印代码用p()函数]")
    const.debug = 1
    global lastRightWheelPosition
    global lastLeftWheelPosition
    cid = const.cid = -1
    sensorAngles = [0, -180, 90, -90]
    const.leftMotorSpeed = 0.0
    const.rightMotorSpeed = 0.0
    leftWheelOdom = 0.0
    rightWheelOdom = 0.0

    lastLeftWheelPosition = 0.0
    lastRightWheelPosition = 0.0
    D.instance().initConnection('127.0.0.1', 19999)
    cid = const.cid = D.instance().cid
    pioneer = D.instance().get('Pioneer_p3dx')
    res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(cid, 'Pioneer_p3dx', sim.sim_scripttype_childscript, 'start', [0], [0.0], [''], bytearray(), sim.simx_opmode_blocking)

    robot = D.instance().get('Robot')
    camera = D.instance().get('Camera')
    leftWheel = D.instance().get('leftWheel')
    leftMotor = D.instance().get('leftMotor')
    rightWheel = D.instance().get('rightWheel')
    rightMotor = D.instance().get('rightMotor')
    goal = D.instance().get('Goal')
    sensors = []
    sensorsNum = 2
    for i in range(0,sensorsNum):
        _sensor = D.instance().get(f'collisionSensor{i}')
        sensors.append(_sensor)
        res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(cid, sensors[i], sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok:
        raise Exception("connect collisionSensor failed")
    sim.simxGetObjectPosition(cid, goal, robot, sim.simx_opmode_streaming)
    if detectionState:
        L = np.sqrt(detectedPoint[0]**2+detectedPoint[1]**2)
        p(f'检测到障碍物，距离: {L:.3f}')

    
    Control().registerWASD()
    const.leftWheelSpeed= 0.2
    const.rightWheelSpeed= 0.2
    print("start cid: ", cid)
    while (sim.simxGetConnectionId(cid) != -1):
        detect = 0
        collisionVector = np.array([0.0, 0.0])
        for i in range(0,sensorsNum):
            res, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(cid, sensors[i], sim.simx_opmode_oneshot)
            if detectionState:
                L = np.sqrt(detectedPoint[0]**2+detectedPoint[1]**2)
                p(f'sensor [{i}] detected obstacles，distance: {L*100:.2f}cm')
                p(detectedPoint)
                collisionVector += [sensorAngles[i] * np.cos(L), sensorAngles[i] * np.sin(L)]
                detect = 1
            twristAngle = np.arctan2(collisionVector[1], collisionVector[0])
            if twristAngle > 0:
                twristAngle -= np.pi/3
            else:
                twristAngle += np.pi/3
            if detect:
                const.leftMotorSpeed,const.rightMotorSpeed = TurnToAngle(twristAngle, 0.16)
                p(f'twrist: {twristAngle}')
                sim.simxSetJointTargetVelocity(cid, leftMotor, const.leftMotorSpeed, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(cid, rightMotor, const.rightMotorSpeed, sim.simx_opmode_oneshot)
                time.sleep(1)
                sim.simxSetJointTargetVelocity(cid, leftMotor, -2, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(cid, rightMotor, -2, sim.simx_opmode_oneshot)
                time.sleep(2)
        if not detect:
            leftMotorSpeed = 0
            rightMotorSpeed = 0

            res, goalPosition = sim.simxGetObjectPosition(cid, goal, robot, sim.simx_opmode_buffer)
            # 目标，结果正, 偏左，负偏右
            goalAngle = np.arctan2(goalPosition[1],goalPosition[0])
            p('nest: ' , goalPosition[0], goalPosition[1])
            distance = goalPosition[0] ** 2 + goalPosition[1] ** 2
            p(f'distances: {distance}')
            p(f'rotate angle: {goalAngle}')
            const.leftMotorSpeed,const.rightMotorSpeed = TurnToAngle(goalAngle, 0.16)
            if goalPosition[0] and goalPosition[1] and distance < 0.1:
                p("Goal! finish!")
                const.leftMotorSpeed = const.rightMotorSpeed = 0
                sim.simxSetJointTargetVelocity(cid, leftMotor, const.leftMotorSpeed, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(cid, rightMotor, const.rightMotorSpeed, sim.simx_opmode_oneshot)
                return

        leftWheelDiameter = \
        sim.simxGetObjectFloatParameter(cid, leftWheel, 18, sim.simx_opmode_oneshot)[1] \
        - sim.simxGetObjectFloatParameter(cid, leftWheel, 15, sim.simx_opmode_oneshot)[1]
        rightWheelDiameter = \
        sim.simxGetObjectFloatParameter(cid, rightWheel, 18, sim.simx_opmode_oneshot)[1] \
        - sim.simxGetObjectFloatParameter(cid, rightWheel, 15, sim.simx_opmode_oneshot)[1]
        leftWheelPosition = sim.simxGetJointPosition(cid, leftMotor, sim.simx_opmode_oneshot)[1]
        rightWheelPosition = sim.simxGetJointPosition(cid, rightMotor, sim.simx_opmode_oneshot)[1]
        #Deal with the joint singularity
        dTheta = leftWheelPosition - lastLeftWheelPosition
        if dTheta > np.pi:
            dTheta -= 2*np.pi
        elif dTheta < -np.pi:
            dTheta += 2*np.pi
        leftWheelOdom += dTheta * leftWheelDiameter / 2
        lastLeftWheelPosition = leftWheelPosition
        dTheta = rightWheelPosition - lastRightWheelPosition
        if dTheta > np.pi:
            dTheta -= 2*np.pi
        elif dTheta < -np.pi:
            dTheta += 2*np.pi
        rightWheelOdom += dTheta * rightWheelDiameter / 2
        lastRightWheelPosition = rightWheelPosition


        #Set actuators on mobile robot
        sim.simxSetJointTargetVelocity(cid, leftMotor, const.leftMotorSpeed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(cid, rightMotor, const.rightMotorSpeed, sim.simx_opmode_oneshot)
    
    sim.simxFinish(cid)

if __name__ == '__main__':
    asyncExecuteBug0()
    time.sleep(6)
    main()
    while True:
        pass
