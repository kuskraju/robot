#!/usr/bin/env python3
import math

import pybullet as p
import pybullet_data
import time
from random import random

# start the simulation with a GUI (p.DIRECT is without GUI)
p.connect(p.GUI)

# we can load plane and cube from pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load a plane
p.loadURDF("plane.urdf", [0, 0, -0.1], useFixedBase=True)

# setup gravity (without it there is no gravity at all)
p.setGravity(0, 0, -10)

# load our robot definition
robot = p.loadURDF("robot.urdf")

cube_num = 4

# load a cub
cube = [p.loadURDF("cube.urdf", [3/4*(random() - 0.5), random(), -0.1], globalScaling=0.05) for i in range(cube_num)]
for i in range(cube_num):
    p.changeVisualShape(cube[i], -1, rgbaColor=[1, 0.5, 0.7, 1])

# display info about robot joints
numJoints = p.getNumJoints(robot)
for joint in range(numJoints):
    print(p.getJointInfo(robot, joint))

# add four sliders to GUI
p0_id = p.addUserDebugParameter("z", -0.1, 0, 0)
p1_id = p.addUserDebugParameter("y", -1, 1, 0)
p2_id = p.addUserDebugParameter("x", -1, 1, 0)
p3_id = p.addUserDebugParameter("pos", 0.0, 6.28, 0)

p.stepSimulation()

def dot(arr):
  return sum([x ** 2 for x in arr])


def find_worst(cube_pos):
    dist = 0
    id = 0
    for i in range(len(cube_pos)):
        curr = dot(cube_pos[i])
        if curr > dist:
            id = i
            dist = curr
    return id


def finish(cubes_pos):
    stop = True
    for x, c in zip(cubes_pos, cube):
        if dot(x) > 0.05:
            stop = False
    return stop

while True:
    cubes_pos = [p.getBasePositionAndOrientation(cube[i])[0] for i in range(cube_num)]

    if finish(cubes_pos):
        print("SUCCESS !!!")
        break

    cube_id = find_worst(cubes_pos)

    for i in [1, 2]:
        cube_pos = list(p.getBasePositionAndOrientation(cube[cube_id])[0])
        cube_pos.reverse()

        where_1 = 0.01*math.copysign(1, cube_pos[3-i]) + cube_pos[3-i]
        while abs(p.getJointState(robot, 3-i)[0]) < abs(cube_pos[3-i]):
            p.setJointMotorControl2(robot, 3-i, p.POSITION_CONTROL, where_1, maxVelocity=50)
            p.setJointMotorControl2(bodyIndex=robot,
                                    jointIndex=3,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=500)
            p.stepSimulation()
            time.sleep(0.05)  # sometimes pybullet crashes, this line helps a lot

        where_2 = 0.1*math.copysign(1, cube_pos[i]) + cube_pos[i]
        while abs(p.getJointState(robot, i)[0]) < abs(0.05 * math.copysign(1, cube_pos[i]) + cube_pos[i]):
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, where_2, maxVelocity=50)
            p.setJointMotorControl2(bodyIndex=robot,
                                    jointIndex=3,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=500)
            p.stepSimulation()
            time.sleep(0.05)  # sometimes pybullet crashes, this line helps a lot

        while p.getJointState(robot, 0)[0] > -0.05:
            p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, -0.1)
            p.setJointMotorControl2(bodyIndex=robot,
                                    jointIndex=3,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    targetVelocity=0,
                                    force=500)
            p.stepSimulation()
            time.sleep(0.05)  # sometimes pybullet crashes, this line helps a lot

        counter = 0
        prev_cube_pos = cube_pos
        while abs(p.getJointState(robot, i)[0]) > 0.08:
            p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, 0, maxVelocity=0.4)
            p.setJointMotorControl2(robot, 3-i, p.POSITION_CONTROL, prev_cube_pos[3-i], maxVelocity=0.4)
            p.setJointMotorControl2(bodyIndex=robot,
                                    jointIndex=3,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=500)
            p.stepSimulation()
            cube_pos = list(p.getBasePositionAndOrientation(cube[cube_id])[0])
            cube_pos.reverse()
            counter += 1
            if counter > 100 and dot(cube_pos) >= dot(prev_cube_pos):
                break
            else:
                prev_cube_pos = cube_pos
            time.sleep(0.05)  # sometimes pybullet crashes, this line helps a lot

        while p.getJointState(robot, 0)[0] < 0:
            p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, 0.2)
            p.setJointMotorControl2(bodyIndex=robot,
                                    jointIndex=3,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=0,
                                    force=500)
            p.stepSimulation()
            time.sleep(0.05)  # sometimes pybullet crashes, this line helps a lot
time.sleep(100)