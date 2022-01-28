import numpy as np
import cv2
from assignment_2_lib import take_a_photo, drive
import time


def forward_distance(photo):
  image = photo[0:400, :, :]
  lower = np.array([150,0,0,0])
  upper = np.array([255,150,150,255])
  mask = cv2.inRange(image, lower, upper)
  a = np.count_nonzero(mask[np.argmax(np.sum(mask, axis=1))])
  if a == 0:
    return np.inf
  const = 750000
  return int(const / a - 1700)

def forward_distance_column(car):
  photo = take_a_photo(car)
  image = photo[0:400, :, :]
  lower = np.array([0, 0, 150, 0])
  upper = np.array([150, 150, 255, 255])
  mask = cv2.inRange(image, lower, upper)
  wow = np.where(np.sum(mask, axis=0) > 0)[0]
  const = 3000
  if len(wow) == 0:
    return 0
  return int(const / len(wow))

def find_a_ball_center(car):
  photo = take_a_photo(car)
  image = photo[0:400, :, :]
  lower = np.array([150, 0, 0, 0])
  upper = np.array([255, 150, 150, 255])
  mask = cv2.inRange(image, lower, upper)
  return np.argmax(np.sum(mask, axis=0))


def find_a_ball(car):
  iter = 0
  while forward_distance(take_a_photo(car)) > 500:
    bc = find_a_ball_center(car)
    direction = -1 if bc > 320 else 1
    if bc == 320:
      direction = 0
    forward = True
    if np.abs(320 - bc) > 200:
      if iter == 0:
        forward = False
        direction *= -1
    drive(car, forward, direction)
    iter = 1 - iter

def find_a_gate_center(car):
  photo = take_a_photo(car)
  image = photo[0:400, :, :]
  lower = np.array([0, 0, 150, 0])
  upper = np.array([150, 150, 255, 255])
  mask = cv2.inRange(image, lower, upper)
  wow = np.where(np.sum(mask, axis=0) > 0)[0]
  if len(wow) == 0:
    return np.inf, np.inf
  a = min(wow)
  b = max(wow)
  c = len(wow)
  if a < 320 and 641 > b > 320 and c < b - a:
    return a, b
  return np.inf, np.inf


def turn(car, direction):
    drive(car, False, -direction)
    drive(car, True, direction)


def around_a_ball(car):
  for i in range(7):
    drive(car, False, 0)

  for i in range(6):
    drive(car, False, -1)
    drive(car, True, 1)

  count = 0
  while count < 40:
    drive(car, True, -1)
    drive(car, True, 0)
    if 30 > count > 17:
      iter = 0
      bc = find_a_ball_center(car)
      a, b = find_a_gate_center(car)
      while not (300 < bc < 340 or (bc < a + (b - a) / 5 and a != np.inf and bc != 0)):
        turn(car, -1)
        iter += 1
        bc = find_a_ball_center(car)
        a, b = find_a_gate_center(car)
      if a + (b - a) / 5 < bc < b - (b - a) / 5 and 300 < bc < 340 and a != np.inf and bc != 0:
        break
      while iter > 0:
        turn(car, 1)
        iter -= 1
    count += 1

  for i in range(forward_distance_column(car)):
    drive(car, True, 0)

  time.sleep(3)


def move_a_ball(car):
  find_a_ball(car)
  around_a_ball(car)