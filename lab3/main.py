#!/usr/bin/env python3

import math
import numpy as np
import pybullet as p
import pybullet_data
import random
from time import sleep

# Helper functions for later

def setCubeColor(cube, color = None):
	if color is None:
		color = [1, 0.5, 0.7, 1]
	p.changeVisualShape(cube, -1, rgbaColor = color)

def loadCube(pos, size = 0.05, color = None):
	print(f"Creating cube at {[(f'{x:.2f}') for x in pos]}.")
	ret = p.loadURDF("cube.urdf", pos, globalScaling = size)
	setCubeColor(ret, color)
	return ret

def genPathLinear(start, end, samples = 100):
	delta = ([(e - s) / samples for s, e in zip(start[0], end[0])],
	         [(e - s) / samples for s, e in zip(start[1], end[1])])

	ret = [start]
	for _ in range(samples):
		ret.append(([l + d for l, d in zip(ret[-1][0], delta[0])],
		            [l + d for l, d in zip(ret[-1][1], delta[1])]))

	return ret

def genPathSegments(poses, samples_each = 100):
	ret = []
	for i in range(len(poses) - 1):
		ret += genPathLinear(poses[i], poses[i + 1], samples_each)
	return ret

traceCubes = []
TRACE_MIN_DIST = 0.01
# Call this function with every call to stepSimulation to draw cubes
def leaveTrace(cubes):
	global traceCubes
	for cube in cubes:
		pos, _ = p.getBasePositionAndOrientation(cube)

		if pos[2] > 0.03:
			continue

		hasNeighbour = False
		for trace in traceCubes:
			tracePos, _ = p.getBasePositionAndOrientation(trace)
			if np.linalg.norm([pos[0] - tracePos[0], pos[1] - tracePos[1]]) <= TRACE_MIN_DIST:
				hasNeighbour = True

		if not hasNeighbour:
			newTrace = loadCube(pos, color = [1, 0.5, 0.7, 0.7])
			p.changeDynamics(newTrace, -1, mass = 0)
			p.setCollisionFilterGroupMask(newTrace, -1, 0, 0)
			traceCubes.append(newTrace)

# Basic setup
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, 0])
p.setGravity(0, 0, -10)

# This time we use kuka from pybullet_data
kukaId = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])

# In our robot, end effector link has following index:
KUKA_TIP_IDX = 6

# We will control this number of joints:
KUKA_NUM_JOINTS = 7

W_Y_MAX = 0.7
W_Y_MIN = 0.5

TRAVEL_Z = 0.35
PUSH_Z = 0.29

cubes = [loadCube([0.1, W_Y_MAX, 0.05]), loadCube([0.45, W_Y_MIN, 0.05]), loadCube([0.66, W_Y_MIN + 0.07, 0.05])]

poses = genPathSegments([([0, W_Y_MAX + 0.02, 0.5], [180, 0, 180]), # init

                         ([0.09, W_Y_MAX + 0.02, 0.5], [180, 0, 180]),  # push1 setup
                         ([0.09, W_Y_MAX + 0.02, PUSH_Z], [180, 0, 180]),  # push1 down
                         ([0.12, W_Y_MAX - 0.07, PUSH_Z], [180, 0, 225]),  # push1 start
                         ([0.2, W_Y_MIN, PUSH_Z], [180, 0, 225]),  # push1
                         ([0.18, W_Y_MIN, PUSH_Z], [180, 0, 225]),  # push1 backoff
                         ([0.18, W_Y_MIN, TRAVEL_Z], [180, 0, 225]),  # push1 up

                         ([0.18, W_Y_MIN - 0.06, TRAVEL_Z], [180, 0, 135]),  # push2 setup
                         ([0.18, W_Y_MIN - 0.06, PUSH_Z], [180, 0, 135]),  # push2 down
                         ([0.21, W_Y_MIN - 0.03, PUSH_Z], [180, 0, 160]),  # push2 start
                         ([0.25, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 160]),  # push2
                         ([0.23, W_Y_MAX - 0.07, PUSH_Z], [180, 0, 160]),  # push2 backoff
                         ([0.23, W_Y_MAX - 0.07, TRAVEL_Z], [180, 0, 160]),  # push2 up

                         ([0.26, W_Y_MAX + 0.02, TRAVEL_Z], [180, 0, 160]),  # push3 setup
                         ([0.26, W_Y_MAX + 0.02, PUSH_Z], [180, 0, 160]),  # push3 down
                         ([0.26, W_Y_MAX - 0.03, PUSH_Z], [180, 0, 200]),  # push3 start
                         ([0.28, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 210]),  # push3 start
                         ([0.35, W_Y_MIN, PUSH_Z], [180, 0, 210]),  # push3
                         ([0.33, W_Y_MIN + 0.02, PUSH_Z], [180, 0, 210]),  # push3 backoff
                         ([0.33, W_Y_MIN + 0.02, TRAVEL_Z], [180, 0, 210]),  # push3 up

                         ([0.29, W_Y_MIN - 0.06, TRAVEL_Z], [180, 0, 135]),  # push4 setup
                         ([0.29, W_Y_MIN - 0.06, PUSH_Z], [180, 0, 135]),  # push4 down
                         ([0.36, W_Y_MIN - 0.03, PUSH_Z], [180, 0, 160]),  # push4 start
                         ([0.40, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 160]),  # push4
                         ([0.37, W_Y_MAX - 0.08, PUSH_Z], [180, 0, 160]),  # push4 backoff
                         ([0.37, W_Y_MAX - 0.08, 0.5], [180, 0, 160]),  # push4 up
                         # N
                         ([0.43, W_Y_MIN - 0.06, TRAVEL_Z], [180, 0, 135]),  # push2 setup
                         ([0.43, W_Y_MIN - 0.06, PUSH_Z], [180, 0, 135]),  # push2 down
                         ([0.46, W_Y_MIN - 0.03, PUSH_Z], [180, 0, 160]),  # push2 start
                         ([0.51, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 160]),  # push2
                         ([0.49, W_Y_MAX - 0.07, PUSH_Z], [180, 0, 160]),  # push2 backoff
                         ([0.48, W_Y_MAX - 0.07, TRAVEL_Z], [180, 0, 160]),  # push2 up

                         ([0.46, W_Y_MAX + 0.05, TRAVEL_Z], [180, 0, 160]),  # push3 setup
                         ([0.46, W_Y_MAX + 0.05, PUSH_Z], [180, 0, 160]),  # push3 down
                         ([0.46, W_Y_MAX - 0.03, PUSH_Z], [180, 0, 200]),  # push3 start
                         ([0.53, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 210]),  # push3 start
                         ([0.58, W_Y_MIN, PUSH_Z], [180, 0, 210]),  # push3
                         ([0.56, W_Y_MIN + 0.02, PUSH_Z], [180, 0, 210]),  # push3 backoff
                         ([0.56, W_Y_MIN + 0.02, TRAVEL_Z], [180, 0, 210]),  # push3 up

                         ([0.54, W_Y_MIN - 0.06, TRAVEL_Z], [180, 0, 135]),  # push4 setup
                         ([0.54, W_Y_MIN - 0.06, PUSH_Z], [180, 0, 135]),  # push4 down
                         ([0.57, W_Y_MIN - 0.03, PUSH_Z], [180, 0, 160]),  # push4 start
                         ([0.6, W_Y_MAX - 0.05, PUSH_Z], [180, 0, 160]),  # push4
                         ([0.58, W_Y_MAX - 0.08, PUSH_Z], [180, 0, 160]),  # push4 backoff
                         ([0.58, W_Y_MAX - 0.08, 0.5], [180, 0, 160]),  # push4 up

                         ([0.68, W_Y_MIN + 0.1, 0.5], [180, 0, 180]),  # push5 setup
                         ([0.68, W_Y_MIN + 0.1, PUSH_Z], [180, 0, 180]),  # push5 down
                         ([0.68, W_Y_MIN + 0.05, PUSH_Z], [180, 0, 180]),  # push5 start
                         ([0.66, W_Y_MIN, PUSH_Z], [180, 0, 180]),  # push5
                         ([0.68, W_Y_MIN - 0.04, PUSH_Z], [180, 0, 180]),  # push5
                         ([0.68, W_Y_MIN - 0.08, PUSH_Z], [180, 0, 180]),  # push5 start
                         ([0.68, W_Y_MIN - 0.08, PUSH_Z], [180, 0, 180]),  # push5
                         ([0.68, W_Y_MIN - 0.08, PUSH_Z], [180, 0, 180]),  # push5 backoff
                         ([0.68, W_Y_MIN, TRAVEL_Z], [180, 0, 180]),  # push5 up

                         ([0, W_Y_MAX + 0.02, 0.5], [180, 0, 180]), # init

                        ])

posIdx = 0
while True:
	pos, rpyDeg = poses[posIdx]
	if posIdx < len(poses) - 1:
		posIdx += 1
	rot = p.getQuaternionFromEuler([math.radians(deg) for deg in rpyDeg])

	# IK calculates joint positions from effector position and orientation
	jointPos = p.calculateInverseKinematics(kukaId,
	                                      KUKA_TIP_IDX,
	                                      pos,
	                                      rot)

	# Take calculated joint positions and set as setpoints
	for i in range(KUKA_NUM_JOINTS):
		p.setJointMotorControl2(bodyIndex=kukaId,
		                        jointIndex=i,
		                        controlMode=p.POSITION_CONTROL,
		                        targetPosition=jointPos[i],
		                        targetVelocity=0,
		                        force=500)

	# Keep gripper rigid
	for i in range(KUKA_NUM_JOINTS, 14):
		p.setJointMotorControl2(bodyIndex=kukaId,
		                        jointIndex=i,
		                        controlMode=p.POSITION_CONTROL,
		                        targetPosition=0,
		                        targetVelocity=0,
		                        force=500)



	p.stepSimulation()
	leaveTrace(cubes)
	sleep(0.01)