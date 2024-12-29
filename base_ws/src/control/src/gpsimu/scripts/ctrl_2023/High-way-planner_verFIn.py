#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Int32, Bool,String
from geometry_msgs.msg import Twist 
from microstrain_inertial_msgs.msg import FilterHeading
import time
import math
class State_machine:
    
    def __init__(self):


        rospy.init_node('Behavior_decision', anonymous=True)


        #--------------------------------Subscriber------------------------------------
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기
        rospy.Subscriber("/traffic_labacon", Bool, self.rubber_callback) # 지금이 라바콘 미션이면 True, 아니면 False 받게 하기 -> 인지한 라바콘이 3개이면 True 아니면 False
        rospy.Subscriber("/obstacle_state",String,self.obstacle_callback) # 터널 내에서 장애물 판단, 해당 코드에서 터널 구간이 아닐 때는 Safe를 발행하게 해야함.
        #------------------------------------------------------------------------------


        #--------------------------------Publisher--------------------------------------
        self.Desired_velocity_pub = rospy.Publisher('/desired_velocity', Twist, queue_size=1) # 원하는 속도를 제어기에 넘기기
        self.Path_pub = rospy.Publisher('/path_state', String, queue_size= 1) # 전역 경로로 주행하게 하기
        # self.Aloam_pub = rospy.Publisher('/aloam_trigger', Bool, queue_size= 1) # 터널에서 측위할 수 있게 하기
        self.State_pub = rospy.Publisher('/State', String, queue_size= 1)
        #-------------------------------------------------------------------------------


        #-----------------------------Initial_Parameter---------------------------------
        self.State = "Unready"
        self.Status_msg= "Not initialized"
        self.Path_state="Global_path"

        self.is_index = False
        self.is_rubber = False
        self.is_lidar = False
        #-------------------------------------------------------------------------------


        #-----------------------------------Main----------------------------------------
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.State_space()
            self.Action_space()
            self.State_pub.publish(self.State)
            print(f"State: {self.State}, Action: {self.Action}, status: {self.status_msg}")
            rate.sleep()
        #-------------------------------------------------------------------------------


    #------------------------------Callback_function--------------------------------
    def index_callback(self, msg):

        self.is_index = True
        self.index = msg.data

    def rubber_callback(self, msg):

        self.is_rubber = True
        self.rubber_trigger = msg.data

    
    def obstacle_callback(self, msg):

        self.is_lidar = True
        self.obstacle_state = msg.data

    #-------------------------------------------------------------------------------


    #--------------------------------State Space------------------------------------
    def State_space(self):
        
        if self.State == "Unready":
            if self.is_index and self.is_rubber and self.is_lidar: # 추가해야 함.
                self.State = "Drive" #상태 천이
            self.Action = "Unready"

        elif self.State == "Drive":
            self.Drive_state()

        elif self.State == "Rubber_cone_drive":

            if self.rubber_trigger == False: #라바콘 주행이 끝났다면
                self.State = "Drive" # 상태천이

            self.Action = "Rubber_cone_drive"

        elif self.State == "Gps_Dead_zone":
            self.Gps_dead_state()

        
    #-------------------------------------------------------------------------------


    #-------------------------------Action Space------------------------------------
    def Action_space(self):

        if self.Action == "Unready":
            self.status_msg="Sensor Input Check"
            self.stop()

        elif self.Action == "Global_path_drive":
            self.Global_path_drive()

        elif self.Action == "Rubber_cone_drive":
            self.Rubber_cone_drive()

        elif self.Action == "Lane_drive":
            self.Lane_drive()

        elif self.Action == "Obstacle_avoiding":
            self.Obstacle_avoiding()

        elif self.Action == "Estop":
            self.Estop_dynamic()
            self.State_pub.publish("Dynamic")

    #-------------------------------------------------------------------------------


    #-------------------------------State_Area-----------------------------------

    def Drive_state(self):

        if 300 < self.index < 386 : # 특정 영역에서 GPS 음영 구간으로 변경
            self.State = "Gps_Dead_zone" 

        elif self.rubber_trigger : # 라바콘 미션이 True일 때
            self.State = "Rubber_cone_drive"

        else: 
            self.Action = "Global_path_drive" # 전역 경로 주행하게 하기


    def Gps_dead_state(self):

        if self.index > 386: #음영구간 벗어나면
            self.State = "Drive" #상태천이

        else:

            if self.obstacle_state == "Dynamic":
                self.Action = "Estop"

            elif self.obstacle_state == "Static":
                self.Action = "Obstacle_avoiding"

            else:
                self.Action = "Lane_drive"
                self.State = "Gps_Dead_zone"
            
            
    #-------------------------------------------------------------------------------



    #----------------------------------Action_space----------------------------------#
    def Global_path_drive(self):

        self.status_msg="Global Path Drive"
        self.Path_state="Global_path"
        self.Path_pub.publish(self.Path_state)
        self.accel() # 15km/h로 주행하게 하기
    
    def Rubber_cone_drive(self):

        self.status_msg="Rubber Cone Drive Mission"
        self.Path_state="Rubber_cone_path"
        self.Path_pub.publish(self.Path_state)
        # 5km/h로 주행하라고 하기

    def Lane_drive(self):

        self.status_msg="DeadReconing Drive Mission"
        self.Path_state="Dead_zone_path"
        self.Path_pub.publish(self.Path_state)
        self.slow()

    def Obstacle_avoiding(self):

        self.status_msg= "Obstacle Avoiding Mission"
        self.Path_state="Obstacle_avoiding_path"
        self.Path_pub.publish(self.Path_state)
        self.slow() # 5km/h로 주행하라고 하기

    def Estop_dynamic(self):

        self.status_msg= "Estop Dynamic Obstacle Mission"
        self.Path_state="Estop_dynamic_path"
        self.Path_pub.publish(self.Path_state)
        self.stop()

    def accel(self): # 목표 속도 15 km/h

        twist_msg = Twist()
        twist_msg.linear.x = 10
        self.Desired_velocity_pub.publish(twist_msg) 

    def slow(self): # 목표 속도 5 km/h

        twist_msg = Twist()
        twist_msg.linear.x = 5
        self.Desired_velocity_pub.publish(twist_msg) 

    def stop(self): # 정지 명령, 원하는 속도를 0으로 보냄 -> 기어 중립도 해야할까?

        twist_msg = Twist()
        twist_msg.linear.x = 0
        self.Desired_velocity_pub.publish(twist_msg) 


if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass