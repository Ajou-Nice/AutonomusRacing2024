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


# Parameter for second_velocity
P_GAIN_2 = 1.1
I_GAIN_2 = 0.45
D_GAIN_2 = 0
SECOND_VELOCITY_KPH = 5



# first_velocity
FIRST_VELOCITY_KPH = 10 #추가
error_rate = 0.2

# first_Velocity parameter
P_GAIN = 1.1
I_GAIN = 0.45
D_GAIN = 0



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
    f = open('brake_test/example1.txt', 'r')
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
    
    plt.scatter(t, curr_vel)
    plt.scatter(t, target_vel)
    
    plt.xlabel('time[s]')
    plt.ylabel(f"velocity[{UNIT}]")
    
    plt.legend(['curr_vel', 'target_vel'])
    plt.title(f"P: {P_GAIN}, I: {I_GAIN}, P_2: {P_GAIN_2}, I_2: {I_GAIN_2}")
    plt.axhline(y = SECOND_VELOCITY_KPH, linestyle = '--') 
    # plt.savefig()

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



