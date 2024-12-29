#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from asyncio import set_event_loop
# from ctypes import set_errno
# from re import I
# import rospy
# import rospkg
# from math import cos,sin,pi,sqrt,pow,atan2
# from geometry_msgs.msg import Point, TwistStamped
# from nav_msgs.msg import Odometry,Path
# #from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
# import tf
# from tf.transformations import euler_from_quaternion,quaternion_from_euler
# from erp_driver.msg import erpCmdMsg, erpStatusMsg
# import math
# from std_msgs.msg import Float32, String, Float32MultiArray, Int32
# from geometry_msgs.msg import Point32,PoseStamped, Twist
# import time
# from visualization_msgs.msg import Marker
# from pid_control import pidControl
import matplotlib.pyplot as plt
import re

"""
SEND BYTES
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │ Al  │ ETX0 │ ETX1 │
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴─────┴──────┴──────┘

RECV BYTES
┌─────┬─────┬─────┬─────┬─────┬──────┬───────┬───────┬────┬───────┬─────┬──────┬──────┐
│  S  │  T  │  X  │ A,M │  E  │ Gear │ Speed │ Steer │ Br │  ENC  │ Al  │ ETX0 │ ETX1 │ 
├─────┼─────┼─────┼─────┼─────┼──────┼───────┼───────┼────┼───────┼─────┼──────┼──────┤
│0x53 │0x54 │0x58 │0 , 1│0 , 1│0,1,2 │0 ~ 200│ ±2000 │1~33│ ±2^31 │0~255│ 0x0D │ 0x0A │
└─────┴─────┴─────┴─────┴─────┴──────┴───────┴───────┴────┴───────┴─────┴──────┴──────┘
"""
'''
SPEED : 0 ~ 200 -> 0 ~ 20 (km/h)
STEER : -2000 ~ 2000 -> -28.2 ~ 28.2 (deg) ( we use radians but the degree unit is easier for checking magnitude)
GEAR  : [ D : 0, N : 1, R : 2] 
'''


# READ ME : Above brake range is 1 ~ 33, but the brake range I cheked is 1 ~ 200.


UNIT = 'km/h'

FRAME_RATE = 30


# second velocity pid
P_GAIN = 3
I_GAIN = 0.4
D_GAIN = 0.003
DESIRED_VELOCITY_KPH = 10


# brake parameter
BRAKE_POWER = 0 # 0~200
STOP_BRAKE = 0 # km/h


# State
MANUAL_MODE = 0
AUTO_MODE = 1

# Save File Name
# FILE_NAME = str(DESIRED_VELOCITY_KPH) + '_KM/H_' + 'PGAIN_' + str(P_GAIN) + '_IGAIN_' + str(I_GAIN) + '_DGAIN_' + str(D_GAIN)
# FILE_NAME = re.sub(r'[^\w\s]', '', FILE_NAME).replace(' ', '_')

# FILE_PATH = 'pid_test_folder'
# FULL_FILE = FILE_PATH + '/' + FILE_NAME + '.txt'
# PLOT_FILE = str(DESIRED_VELOCITY_KPH) + '_KM/H_' + 'PGAIN_' + str(P_GAIN) + '_IGAIN_' + str(I_GAIN) + '_DGAIN_' + str(D_GAIN)

# PLOT_FILE = re.sub(r'[^\w\s]', '', PLOT_FILE).replace(' ', '_')


# Analyze
# RISING_THRESHOLD    = 0.8
# OVERSHOOT_THRESHOLD = 0.05


def read_file():
    f = open('brake_test/08.06_1.txt', 'r')
    lines = f.readlines()
    
    curr_vel_list   = []
    target_vel_list = []
    brake_list = []
    
    for line in lines :
        tmp = line.split()
        curr_vel_list.append(float(tmp[0]))
        target_vel_list.append(float(tmp[1]))
        brake_list.append(float(tmp[2]))
 
             
    f.close()
    return np.array(curr_vel_list), np.array(target_vel_list), np.array(brake_list)



def plot_output(curr_vel, target_vel):
    ti = 0
    tf = len(curr_vel)
    
    t  = np.arange(ti, tf) / FRAME_RATE
    
        # curr_vel 점들을 그리는 도중, brake가 0보다 큰 값일동안 빨간색으로 표시
    for i in range(len(curr_vel)):
        if brake[i] > 0:
            plt.scatter(t[i], curr_vel[i], color='red')  # brake가 0보다 클때 
        else:
            plt.scatter(t[i], curr_vel[i], color='blue')  # 나머지 상태
    plt.scatter(t, target_vel, color='orange')
    
    plt.xlabel('time[s]')
    plt.ylabel(f"velocity[{UNIT}]")
    
    plt.legend(['curr_vel', 'target_vel'])
    plt.title(f"P: {P_GAIN}, I: {I_GAIN}, D: {D_GAIN}, BRAKE_POWER : {BRAKE_POWER}, BRAKE_START : {STOP_BRAKE}")
    plt.axhline(y = DESIRED_VELOCITY_KPH, linestyle = '--') 
    plt.axhline(y = 10, linestyle = '--') 
#     plt.savefig(PLOT_FILE)

    plt.show()
    return


# def calculate_metric(curr_vel, target_vel):
def calculate_metric(brake_list):
#     tr_velocity = DESIRED_VELOCITY_KPH * RISING_THRESHOLD
    br_index = np.where(brake_list > 1)[0][0]
    br       = br_index / FRAME_RATE            # tr is 'time rising'
    
#     ov_velocity = DESIRED_VELOCITY_KPH * (1 + OVERSHOOT_THRESHOLD)
#     ov_indices  = np.where(curr_vel > ov_velocity)[0]
#     ov      = len(ov_indices)  / FRAME_RATE              # ov is 'overshoot'
#     max_ov = np.max(curr_vel[ov_indices])
    return br
    

if __name__ == '__main__':
    curr_vel, target_vel, brake = read_file()
    try:
        br = calculate_metric(brake)
        print(f'brake_time : {br}')
#         tr, ov, max_ov = calculate_metric(curr_vel, target_vel)
#         print("Rising time : {:.2f}, Overshoot : {:.2f}, Max_Overshoot : {:.2f}".format(tr, ov, max_ov))
    except:
        print('Error')
    plot_output(curr_vel, target_vel)



