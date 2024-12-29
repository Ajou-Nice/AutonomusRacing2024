#!/usr/bin/env python
# -*- coding: utf-8 -*-

from asyncio import set_event_loop
from ctypes import set_errno
from re import I
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry,Path
#from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from erp_driver.msg import erpCmdMsg, erpStatusMsg
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32
from geometry_msgs.msg import Point32,PoseStamped, Twist
import time
from visualization_msgs.msg import Marker
from pathlib import Path
from pid_control import pidControl

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
# 5 -> 10 -> 5 : 감속하는 부분에서 브레이크 없이 일정 속도를 줘서 속도를 찾는 파일


RAD2DEG = 180/np.pi
DEG2RAD = 1/RAD2DEG
MPS2KPH = 3.6
KPH2MPS = 1/MPS2KPH

UNIT_CONVERSION_SPEED = 1/10
UNIT_CONVERSION_STEER = math.radians(28.2) / 2000

FRAME_RATE = 30

# =================SECOND===============
# Parameter for second_velocity
P_GAIN_2 = 3
I_GAIN_2 = 0.2
D_GAIN_2 = 0.003
SECOND_VELOCITY_KPH = 5

# 두번재 속도 감속할때 줄 일정한 속도값
second_target_vel = 60

# =================FIRST===============
# first_velocity
FIRST_VELOCITY_KPH = 15 #추가
error_rate = 0.3

# first_Velocity parameter
P_GAIN = 2.5
I_GAIN = 0.4
D_GAIN = 0.003

# =================THIRD===============
# Parameter for THIRD_velocity
P_GAIN_3 = 3
I_GAIN_3 = 0.3
D_GAIN_3 = 0.003
THIRD_VELOCITY_KPH = 10



maintain_vel = 250


BRAKE_POWER = 20 # 0~200
STOP_BRAKE = 7 # km/h


# State
MANUAL_MODE = 0
AUTO_MODE = 1
EXPERIMENT_TIME = 2000 # s

# Save File Name
FILE_NAME = 'example1'
FILE_PATH = 'brake_test'
FULL_FILE = FILE_PATH + '/' + FILE_NAME + '.txt'



class Longitudinal_control :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        # rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        # rospy.Subscriber("/vel", TwistStamped, self.velocity_callback)
        
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
 
        # =====================================
        self.erpCmd_msg     = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()

        self.erpCmd_msg.e_stop = False
        self.is_statue = False
        self.is_velo   = False
        self.is_timer  = False

        # =====================================
        self.vehicle_yaw = 0.
        self.velocity    = 0.

        frameRate = FRAME_RATE
        dt        = 1 / frameRate

        p_gain = P_GAIN
        i_gain = I_GAIN
        d_gain = D_GAIN


        p_gain_2 = P_GAIN_2
        i_gain_2 = I_GAIN_2
        d_gain_2 = D_GAIN_2

        p_gain_3 = P_GAIN_3
        i_gain_3 = I_GAIN_3
        d_gain_3 = D_GAIN_3

        self.pid = pidControl(p_gain = p_gain, i_gain = i_gain, d_gain = d_gain, dt = dt)
        self.pid_2 = pidControl(p_gain = p_gain_2, i_gain = i_gain_2, d_gain = d_gain_2, dt = dt)
        self.pid_3 = pidControl(p_gain = p_gain_3, i_gain = i_gain_3, d_gain = d_gain_3, dt = dt)

        # ================================
        self.f = open(FULL_FILE, 'w')

        self.timer = None

        rate = rospy.Rate(frameRate) # 30hz
        

        # ====================================

        self.first_velocity_array = []
        self.second_velocity_array = []
        self.is_first = False
        self.is_second = False

        # ====================================

        while not rospy.is_shutdown():
            if not self.is_statue:
                rospy.loginfo("ERP Status msg do not arrived....")

            # elif not self.is_odom:
            #     rospy.loginfo("Odometry msg do not arrived....")
            
            else:
                if self.erpStatus_msg.control_mode == AUTO_MODE:
                    print('auto_mode')
                    print(self.check_timer())
                    if self.check_timer():
                        # try:
                        self.main()
                        print('main activated')
                        # except:
                        #rospy.loginfo("The system has error, So E-stop turn on")
                            # self.e_stop()
                    else:
                        # self.e_stop()
                        print('fail')
                        pass
                        
                else:
                    self.init_variable()

            rate.sleep()
            
    def main(self):
        steering = 0
        curr_velocity = self.erpStatus_msg.speed
        print(f'curr_velocity : {curr_velocity}')
        target_velocity = int(round(self.pid.pid(target_vel = FIRST_VELOCITY_KPH*10, current_vel = curr_velocity)))
        
        if self.is_first and not self.is_second:


            if curr_velocity/10 < STOP_BRAKE: # 브레이크 x 상태로 두번째 속도 도달 전 -> 세번째 
                target_velocity = int(round(self.pid_2.pid(target_vel = SECOND_VELOCITY_KPH * 10, current_vel = curr_velocity))) 
                # target_velocity = second_target_vel
                self.erpCmd_msg.gear  = 0
                self.erpCmd_msg.steer = steering
                self.erpCmd_msg.brake = 0
                self.erpCmd_msg.speed = target_velocity
                print('velocity2_pid')
            
            else: # 브레이크 시작 -> 두번째
                # target_velocity = int(round(self.pid_2.pid(target_vel = SECOND_VELOCITY_KPH * 10, current_vel = curr_velocity))) 
                # target_velocity = second_target_vel
                target_velocity = 0
                self.erpCmd_msg.gear  = 0
                self.erpCmd_msg.steer = steering           
                self.erpCmd_msg.brake = BRAKE_POWER
                self.erpCmd_msg.speed = target_velocity

            if (( curr_velocity/10 > SECOND_VELOCITY_KPH - SECOND_VELOCITY_KPH * error_rate) & (curr_velocity/10 < SECOND_VELOCITY_KPH + SECOND_VELOCITY_KPH * error_rate)):
                self.second_velocity_array.append(curr_velocity)
            
            # if len(self.second_velocity_array) == maintain_vel: # 3초
            if len(self.second_velocity_array) == 125:
                self.is_second = True
                self.is_first = False
                return

           

        elif not self.is_first and not self.is_second:
            if target_velocity < 0 :
                brake = self.calculate_brakePower(target_velocity)
                target_velocity = 0
            else: # 첫번째 
                self.erpCmd_msg.gear  = 0
                self.erpCmd_msg.steer = steering
                self.erpCmd_msg.speed = target_velocity
                print('velocity1_pid :', target_velocity)
                self.erpCmd_msg.brake = 0
            
            ###################################################################################

            # 현재 속도가 처음 타깃속도 오차범위 내의 값이면 velocity_array에 추가
            if (( curr_velocity/10 > FIRST_VELOCITY_KPH - FIRST_VELOCITY_KPH * error_rate) & (curr_velocity/10 < FIRST_VELOCITY_KPH + FIRST_VELOCITY_KPH * error_rate)):
                self.first_velocity_array.append(curr_velocity)
                print(self.first_velocity_array)

            # 데이터가 90개 모이면 3초
            if len(self.first_velocity_array) == maintain_vel:
                self.is_first = True 
                return
            

        elif not self.is_first and self.is_second: # 세번째 속도

            target_velocity = int(round(self.pid_3.pid(target_vel = THIRD_VELOCITY_KPH * 10, current_vel = curr_velocity)))
            self.erpCmd_msg.gear  = 0
            self.erpCmd_msg.steer = steering
            self.erpCmd_msg.brake = 0
            self.erpCmd_msg.speed = target_velocity
            print('velocity3_pid')
            
            

        if self.erpCmd_msg.speed > 200:
            self.erpCmd_msg.speed = 200

        if self.erpCmd_msg.speed < 0:
            self.erpCmd_msg.speed = 0

        self.erpCmd_msg.e_stop = False
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)
        
        # self.convert_ctrlCmdUnit()
        # self.convert_platformUnit
        data = f"{curr_velocity/10}\t{self.erpCmd_msg.speed/10}\t{self.erpCmd_msg.brake}\n"
        self.f.write(data)
        print(f'cur_vel : {curr_velocity/10}, speed : {self.erpCmd_msg.speed/10}')
        return
    
    def convert_platformUnit(self):
        self.erpStatus_msg.speed *= UNIT_CONVERSION_SPEED # km/h
        self.erpStatus_msg.speed *= KPH2MPS # m/s

        self.erpStatus_msg.steer *= UNIT_CONVERSION_STEER
        
    def convert_ctrlCmdUnit(self):
        self.erpCmd_msg.speed *= MPS2KPH
        self.erpCmd_msg.speed /= UNIT_CONVERSION_SPEED

        # self.erpCmd_msg.steer /= UNIT_CONVERSION_STEER
        
        # if self.erpCmd_msg.steer > 2000:
        #     self.erpCmd_msg.steer = 2000
        # elif self.erpCmd_msg.steer < -2000:
        #     self.erpCmd_msg.steer = -2000
            
    def calculate_brakePower(self, target_velocity):
        brake = 10
        
        return brake
    
    def check_timer(self):
        
        if self.timer is None:
            self.timer = time.time()
            self.is_timer = True

        curr_time = time.time()
        duration  = curr_time - self.timer
        
        if duration < EXPERIMENT_TIME:
            return True
        else:
            
            return False
        
    
    # def e_stop(self):
    #     self.erpCmd_msg.gear    = 0
    #     self.erpCmd_msg.speed   = 0
    #     self.erpCmd_msg.brake   = 200
    #     self.erpCmd_msg.e_stop  = False
        
    #     self.erp_42_ctrl_pub.publish(self.erpCmd_msg)
        
    def init_variable(self):
        self.pid.prev_error = 0
        self.pid.i_control  = 0

        self.pid_2.prev_error = 0
        self.pid_2.i_control  = 0
        
        self.pid_3.prev_error = 0
        self.pid_3.i_control = 0

# ------------------------ callback -------------------------- #
    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z

    def status_callback(self, msg):
        self.is_statue = True
        self.erpStatus_msg = msg

        # self.convert_platformUnit()

    def velocity_callback(self, msg):
        self.is_velo = True

        if not np.isnan(msg.twist.linear.x):
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            self.velocity = math.sqrt(vx**2 + vy**2)
        else:
            self.velocity=0.
            
if __name__ == '__main__':
    Long_Control = Longitudinal_control()
    



        

