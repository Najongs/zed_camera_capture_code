import cv2
import time
import numpy as np
import sys
import select

from fairino import Robot

robot = Robot.RPC('192.168.58.2')
speed_set_state = robot.SetSpeed(20)
print("전역 속도 : ", speed_set_state)

tool = 0
user = 0

home_joint = [0.0, -90.0, 0.0, -90.0, -90.0, 0.0]

test_pos1_up_pos= [-723.70, 324.49, 456.16, 180.0, 0.0, 0.0]
test_pos1_up_joint = [-31.53, -44.22, 28.40, -74.13, -90.0, 58.45]
test_pos1_down_pos = [-723.70, 324.49, 232.98, 180.0, 0.0, 0.0]
test_pos1_down_joint = [-31.53, -43.89, 60.62, -106.69, -90.0, 58.45]

test_pos2_up_pos = [-734.26, -312.19, 456.16, 180.0, 0.0, 0.0]
test_pos2_up_joint = [15.68, -42.86, 25.96, -73.03, -90.0, 105.69]
test_pos2_down_pos = [-734.26, -312.19, 232.98, 180.0, 0.0, 0.0]
test_pos2_down_joint = [15.68, -43.16, 59.30, -106.04, -90.0, 105.69]

test_pos3_up_pos = [-343.75, -312.19, 456.16, 180.0, 0.0, 0.0]
test_pos3_up_joint = [29.55, -95.86, 98.55, -92.59, -89.99, 119.55]
test_pos3_down_pos = [-343.75, -312.19, 232.98, 180.0, 0.0, 0.0]
test_pos3_down_joint = [29.55, -84.72, 122.54, -127.70, -90.0, 119.55]

test_pos4_up_pos = [-340.85, 321.33, 456.16, 180.0, 0.0, 0.0]
test_pos4_up_joint = [-55.88, -95.30, 98.08, -92.75, -90.0, 34.11]
test_pos4_down_pos = [-340.85, 321.33, 232.98, 180.0, 0.0, 0.0]
test_pos4_down_joint = [-55.88, -84.20, 121.97, -127.75, -90.0, 34.11]

timeout = 1
num = 0

# 처음 대기 자세로 이동
error = robot.MoveJ(home_joint, tool, user, vel=20)
print("move to home : ", error)

while True:
    if num == 0:
        error = robot.MoveCart(test_pos1_up_pos, tool, user, vel=20)
        print("move to pos1_up : ", error)
    else:
        error = robot.MoveCart(test_pos1_up_pos, tool, user, vel=50)
        print("move to pos1_up : ", error)

    error = robot.MoveL(test_pos1_down_pos, tool, user, vel=40)
    print("move to pos1_down : ", error)

    error = robot.MoveL(test_pos1_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)


    error = robot.MoveCart(test_pos2_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)

    error = robot.MoveL(test_pos2_down_pos, tool, user, vel=40)
    print("move to pos1_down : ", error)

    error = robot.MoveL(test_pos2_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)


    error = robot.MoveCart(test_pos3_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)

    error = robot.MoveL(test_pos3_down_pos, tool, user, vel=40)
    print("move to pos1_down : ", error)

    error = robot.MoveL(test_pos3_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)


    error = robot.MoveCart(test_pos4_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)

    error = robot.MoveL(test_pos4_down_pos, tool, user, vel=40)
    print("move to pos1_down : ", error)

    error = robot.MoveL(test_pos4_up_pos, tool, user, vel=50)
    print("move to pos1_up : ", error)

    num = 1

error = robot.MoveJ(home_joint, tool, user, vel=20)
print("move to home : ", error)

num = 0