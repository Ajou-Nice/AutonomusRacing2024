#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from asyncio import set_event_loop
from re import I
import rospy
from erp_driver.msg import erpCmdMsg, erpStatusMsg
from geometry_msgs.msg import Point, TwistStamped
from nav_msgs.msg import Odometry,Path
#from morai_msgs.msg import CtrlCmd,EgoVehicleStatus
import numpy as np
import math
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import math
from std_msgs.msg import Float32, String, Float32MultiArray, Int32
from geometry_msgs.msg import Point32,PoseStamped
import time
from visualization_msgs.msg import Marker

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
AUTO_MODE = 1   
MANUAL_MODE = 0
frameRate = 30

# case1: laba steer p - 0.8 d - 0.005 ,  speed : 3 -3 -3 degree limit 5.0 , frong_length 0.3
# case2: laba steer p - 0.82 d - 0.005 ,  speed : 5 -3 -3 degree limit 2.5 , frong_length 0.9


class LabaconDrive:
    def __init__(self):
        rospy.init_node('labacon_drive', anonymous=True)
        # =============================== Subscriber =============================== #
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/front_close_utm', Float32MultiArray, self.labacon_callback)
        rospy.Subscriber('/velocity_front_close_utm', Float32MultiArray, self.velocity_labacon_callback)
        # rospy.Subscriber('uturn_point', Float32MultiArray, self.uturn_callback)
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        # =====================================
        self.erpCmd_msg     = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()
        # ====================================
        self.is_laba = False
        self.is_status = False
        # ====================================
        self.prev_mid_point = 0
        
        rate = rospy.Rate(frameRate) # 30hz

        self.pid_laba = pidControl_Laba()
        self.pid = pidControl()

        self.velocity_mid_point_x = 0
        self.velocity_mid_point_y = 0

        while not rospy.is_shutdown():
            if not self.is_status:
                rospy.loginfo("ERP Status msg do not arrived....")

            elif not self.is_laba:
                rospy.loginfo("Labacone midpoint msg do not arrived....")
            
            else:
                if self.erpStatus_msg.control_mode == AUTO_MODE:

                    self.main()
                else:
                    self.pid_laba.init_variable()
                    self.pid.init_variable()
            rate.sleep()


    def main(self):
        steering = self.cal_laba_steer(self.mid_point_y)
        target_velocity = self.control_labacone_velocity(steering)
        brake = 0
        self.erpCmd_msg.gear  = 0
        self.erpCmd_msg.steer = steering
        self.erpCmd_msg.speed = target_velocity
        self.erpCmd_msg.brake = brake


        print('Current_velocity : {}'.format(self.erpStatus_msg.speed))
        print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)

    def cal_laba_steer(self, labacone_point):
        
        keeping_dist = 0.0
        error_dist = keeping_dist-labacone_point
        
        error_point = error_dist # 오른쪽 방향이 음수값 데이터인 경우

        # error_point = -error_dist # 오른쪽 방향이 양수값 데이터인 경우 

        target_steering = self.pid_laba.pid_Laba(error_point)*0.85

        target_steering = int(2000*(target_steering/0.4922))

        return max(min(target_steering, 2000), -2000)
    
    def control_labacone_velocity(self,steering):
        # rospy.logwarn(f'aaaaaaaaaaaaaaaaa')
        
        steer_rad=steering/2000*0.4922
        target_velocity=0

        if abs(steer_rad) < math.radians(2.5):
            target_velocity = 5
        elif abs(steer_rad) < 0.4922:
            target_velocity = 3
        else:
            target_velocity = 3

        output = round(self.pid.pid(target_velocity * 10, self.erpStatus_msg.speed))

        return max(min(output, 200), 0)
        

        # velocity_trigger = self.velocity_control()

        # if velocity_trigger:
        #     target_velocity = 5
        # else:
        #     target_velocity = 3
        
        # output = round(self.pid.pid(target_velocity * 10, self.erpStatus_msg.speed))

        # return max(min(output, 200), 0)

    def velocity_control(self):
        a = abs(self.velocity_mid_point_x)
        b = abs(self.velocity_mid_point_y)
        c = math.atan(b/a) * (180/math.pi)
        velocity_threshold = 75
        if c >= velocity_threshold:
            return True
        else:
            return False

    
    def labacon_callback(self, msg):
        self.is_laba = True
        self.mid_point_x = msg.data[0]
        self.mid_point_y = msg.data[1]

        if math.isnan(self.mid_point_x):
            self.mid_point_x = self.prev_mid_point_x
        if math.isnan(self.mid_point_y):
            self.mid_point_y = self.prev_mid_point_y

        self.prev_mid_point_x = self.mid_point_x
        self.prev_mid_point_y = self.mid_point_y

    def velocity_labacon_callback(self, msg):
        self.is_velocity_laba = True
        self.velocity_mid_point_x = msg.data[0]
        self.velocity_mid_point_y = msg.data[1]

        if math.isnan(self.velocity_mid_point_x):
            self.velocity_mid_point_x = self.velocity_prev_mid_point_x
        if math.isnan(self.velocity_mid_point_y):
            self.velocity_mid_point_y = self.velocity_prev_mid_point_y

        self.velocity_prev_mid_point_x = self.velocity_mid_point_x
        self.velociy_prev_mid_point_y = self.velocity_mid_point_y

    def uturn_callback(self, msg):
        self.is_laba = True
        self.mid_point = msg.data[1]

        if self.mid_point == float('nan'):
            self.mid_point = self.prev_mid_point
        
        self.prev_mid_point = self.mid_point

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
# ============================================== PID =========================================

class pidControl_Laba:
    def __init__(self):
        self.p_gain = 0.82
        self.i_gain = 0.0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 1/frameRate

    def pid_Laba(self,error):

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
    def init_variable(self):
        self.prev_error = 0
        self.i_control = 0

class pidControl:
    def __init__(self):
        self.p_gain = 3.5
        self.i_gain = 0.3
        self.d_gain = 0.003
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1
    
    def pid(self,target_vel, current_vel):
        
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
    def init_variable(self):
        self.prev_error = 0
        self.i_control = 0

if __name__ == '__main__':
    try:
        LabaconDrive()
    except rospy.ROSInterruptException:
        pass