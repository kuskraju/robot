#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import math
import random
from scipy.spatial.transform import Rotation
import numpy as np
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


def make_boxes():
    pos_1 = [random.random(), random.random(), 0]
    angle = 3.1415 * 2 * random.random()
    pos_2 = [pos_1[0] + math.sin(angle) * 0.6, pos_1[1] - math.cos(angle) * 0.6, 0]
    return pos_1, pos_2


def build_world_with_car(pos):
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf")
    p.resetBasePositionAndOrientation(car, pos[0], pos[1])
    return car


def simulate_car(car):
    inactive_wheels = [3, 5, 7]
    wheels = [2]
    for wheel in inactive_wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    steering = [4, 6]
    maxForce = 10
    targetVelocity = -2
    steeringAngle = 0.174
    steps = 5000
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=targetVelocity,
                                force=maxForce)

    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)
    for i in range(steps):
        p.stepSimulation()
    return p.getBasePositionAndOrientation(car)


start_pose = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
car = build_world_with_car(start_pose)
end_pose = simulate_car(car)
pos_1, pos_2 = make_boxes()

p.stepSimulation()
time.sleep(0.05)

car_rot = Rotation.from_quat(end_pose[1])

a = (pos_1[1] - pos_2[1]) / (pos_1[0] - pos_2[0])
print(pos_1[1], pos_2[1], pos_1[0], pos_2[0])
fi = np.arctan(a) + np.pi / 2
cos = np.cos(fi)
sin = np.sin(fi)

final_rot = Rotation.from_matrix(np.array(
    [[cos, -sin, 0],
     [sin, cos, 0],
     [0, 0, 1]]
))

origin_rot = Rotation.from_matrix(car_rot.inv().as_matrix() @ final_rot.as_matrix())

final_pose = tuple(map(lambda i, j: (i + j) / 2, pos_1, pos_2))

end2_pose = Rotation.from_matrix(car_rot.inv().as_matrix() @ final_rot.as_matrix()).as_matrix() @ np.array(end_pose[0])

# TODO: calculate calculated_pose
calculated_pose = (
    tuple(map(lambda i, j: i - j, final_pose, end2_pose)),
    Rotation.from_matrix(car_rot.inv().as_matrix() @ final_rot.as_matrix()).as_quat())

# The car should end its route with the front wheels between two boxes as in the example

car = build_world_with_car(calculated_pose)
p.loadURDF("cube.urdf", pos_1, globalScaling=0.1)
p.loadURDF("cube.urdf", pos_2, globalScaling=0.1)
simulate_car(car)
time.sleep(50)