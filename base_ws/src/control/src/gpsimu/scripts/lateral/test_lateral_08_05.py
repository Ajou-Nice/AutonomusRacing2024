#!/usr/bin/env python
# -*- coding: utf-8 -*-


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
import csv

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

UNIT_CONVERSION_SPEED = 1/10
UNIT_CONVERSION_STEER = math.radians(28.2) / 2000

# ==============================================================================================================================================================
DESIRED_VELOCITY_KPH = 5

# Steer PID Parameter
P_GAIN_STEER = 0.75
I_GAIN_STEER = 0.4
D_GAIN_STEER = 0.002

# Look distance Parameter
fixed_Lp = 1
kpp = 0.2

add_name = 'purever2_i_0.4_d_0.002'

# ==============================================================================================================================================================

# State
MANUAL_MODE = 0
AUTO_MODE = 1

# Data setting
is_lateral_offset_data = True # erp 현재위치와 경로 사이의 차이 계산
is_heading_offset_data = True # purepursuit 헤딩 계산값과 차량의 현재 헤딩값 차이 계산
is_position_data = True # erp 실제 이동경로 gps 데이터
is_pure_pursuit_data = True # Lp_x, Lp_y, Way_x, Way_y 데이터 저장

# Save File Name -> 폴더 생성해놓기
LATERAL_OFFSET_FILE_NAME = 'lateral_offset/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
HEADING_OFFSET_FILE_NAME = 'heading_offset/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
POSITION_DATA_FILE_NAME = 'position_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
LP_DATA_FILE_NAME = 'Lp_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
WAY_DATA_FILE_NAME = 'Way_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'

def make_marker(waypoint,id):
    print(waypoint[0],waypoint[1])
    marker = Marker()
    marker.header.frame_id = "map"  # 마커를 표시할 좌표계
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoint"
    marker.id = id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # 마커 위치 및 크기 설정
    marker.pose.position = Point(waypoint[0], waypoint[1], 1.0)  # 장애물의 위치 (x, y, z)
    marker.pose.orientation.w = 1.0  # 회전 없음
    marker.scale.x = 1.  # 가로 크기
    marker.scale.y = 1.  # 세로 크기
    marker.scale.z = 0.  # 높이

    if marker.scale.x == 0.0:
        marker.scale.x = 0.1
    if marker.scale.y == 0.0:
        marker.scale.y = 0.1
    if marker.scale.z == 0.0:
        marker.scale.z = 0.1

    # 마커 색상 설정 (RGBA)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0  # 투명도

    marker.lifetime = rospy.Duration(0.1)  # 영구적으로 표시

    return marker

class Longitudinal_control :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        # =====================================
        self.erpCmd_msg     = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()

        self.current_position=Point()

        self.vehicle_length = 1.04

        self.is_status = False
        self.is_path   = False
        self.is_odom   =False

        # =====================================
        self.vehicle_yaw = 0.
        self.velocity    = 0.

        frameRate = 30
        dt        = 1 / frameRate


        self.pid_0to5 = pidControl(p_gain = 3.5, i_gain = 0.1, d_gain = 0.003, dt = dt)
        self.pid_5to10 = pidControl(p_gain = 3.5, i_gain = 0.3, d_gain = 0.003, dt = dt)
        self.pid_10to20 = pidControl(p_gain = 3, i_gain = 0.4, d_gain = 0.003, dt = dt)

        p_gain_steer = P_GAIN_STEER
        i_gain_steer = I_GAIN_STEER
        d_gain_steer = D_GAIN_STEER

        self.pid_steer = pidControl(p_gain = p_gain_steer, i_gain = i_gain_steer, d_gain = d_gain_steer, dt = dt)
        # ====================================
        
        self.later_f = open(LATERAL_OFFSET_FILE_NAME, 'w')
        self.heading_f = open(HEADING_OFFSET_FILE_NAME, 'w')
        self.position_f = open(POSITION_DATA_FILE_NAME, 'w')
        self.lp_f = open(LP_DATA_FILE_NAME, 'w')
        self.way_f = open(WAY_DATA_FILE_NAME, 'w')

        self.lateral_writer = csv.writer(self.later_f)
        self.heading_writer = csv.writer(self.heading_f)
        self.position_writer = csv.writer(self.position_f)
        self.lp_writer = csv.writer(self.lp_f)
        self.way_writer = csv.writer(self.way_f)

        # ====================================

        rate = rospy.Rate(frameRate) # 30hz

        while not rospy.is_shutdown():
            if not self.is_status:
                rospy.loginfo("ERP Status msg do not arrived....")

            elif not self.is_odom:
                rospy.loginfo("Odometry msg do not arrived....")

            elif not self.is_path:
                rospy.loginfo("Path msg do not arrived....")
            
            else:
                if self.erpStatus_msg.control_mode == AUTO_MODE:

                    self.main()

                else:
                    self.init_variable()
            rate.sleep()

# ==========================================================================================
    def main(self):

        steering, target_velocity, brake, pure_steering = self.cal_control()

        self.erpCmd_msg.gear  = 0
        self.erpCmd_msg.steer = steering
        self.erpCmd_msg.speed = target_velocity
        self.erpCmd_msg.brake = brake

        self.max_value()
        print('Current_velocity : {}'.format(self.erpStatus_msg.speed))
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)


        # 데이터 생성
        if is_lateral_offset_data:
            self.lateral_offset_data()
        if is_heading_offset_data:
            self.heading_offset_data(pure_steering)
        if is_position_data:
            self.position_data()
        return
    
    def calc_pure_pursuit(self):    # pure_pursuit 계산 값을 이용하며 경로 생성
        curr_velocity = self.erpStatus_msg.speed    # erpStatus_msg.speed로 부터 현재속도값 받아옴
        ref_x = []  
        ref_y = []  

        for pose in self.path.poses:    # Local_path 데이터를 ref_x, ref_y 배열에 추가
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)
        # 추가 내용 : x, y 값은 m단위 , 즉 ref_x, ref_y에 local_path의 값들이 들어와있다.
            
        V_x = curr_velocity / 10 # V_x 는 km/h단위
        Lp = fixed_Lp + kpp*V_x
        # 삼각함수 계산에는 라디안이 사용됨. vehicle_length 도 실제 값
        Lp_x = self.current_position.x + Lp * np.cos(self.vehicle_yaw)
        Lp_y = self.current_position.y + Lp * np.sin(self.vehicle_yaw)

        dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)
        min_index = np.argmin(dis_P2)

        Way_x = ref_x[min_index]
        Way_y = ref_y[min_index]

        x_2 = (Way_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.sin(self.vehicle_yaw)
        y_2 = - (Way_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.cos(self.vehicle_yaw)

        print("================================")
        print("In Local, target postion !!!!!")
        print("x: {:.2f}, y:{:.2f}".format(x_2,y_2))
        print("Current Velocity : {:.2f}".format(V_x))

        L_bar = np.sqrt(x_2**2 +y_2**2)
        sin_alpha = y_2/L_bar

        steering = -self.vehicle_length * 2 * y_2 / (L_bar)**2
        # steering의 단위가 rad 로 나옴 -> yaw 값이 rad 이기 때문에 

        self.visualization_heading_WP([Lp_x, Lp_y],[Way_x, Way_y], self.current_position,min_index)
        print("=================================")
        if steering>0:
            print("RIGHT HANDLING")
        else:
            print("LEFT HANDLING")
            
            print("heading : {:.2f}, steering: {:.2f}".format(math.degrees(self.vehicle_yaw),math.degrees(steering)))

        # Lp와 Way 데이터를 CSV 파일에 저장
        if is_pure_pursuit_data:
            self.lp_writer.writerow([Lp_x, Lp_y])
            self.way_writer.writerow([Way_x, Way_y])

        steering = int(2000*(steering/0.4922))
        return steering
    
        
        
    def max_value(self):
        if self.erpCmd_msg.steer > 2000:
            self.erpCmd_msg.steer = 2000
        elif self.erpCmd_msg.steer < -2000:
            self.erpCmd_msg.steer = -2000

        if self.erpCmd_msg.speed > 200:
            self.erpCmd_msg.speed = 200
        elif self.erpCmd_msg.speed < 0:
            self.erpCmd_msg.speed = 0
            
    def cal_control(self):
        curr_velocity = self.erpStatus_msg.speed
        curr_steer = self.erpStatus_msg.steer
        brake = 0
        target_velocity = 0

        
        # steer calculate ===================================================================
        steering = self.calc_pure_pursuit()
        pure_steering = steering
        steering = int(self.pid_steer.pid(target_vel = steering , current_vel = curr_steer))


        # velocity for steer =================================================================
        if (abs(steering)<math.radians(5.)):
            target_velocity = DESIRED_VELOCITY_KPH * 10
        elif (abs(steering)<0.4922):
            target_velocity = 30 # erpunit
        else:
            target_velocity = 30 # erpunit


        # brake calculate ===================================================================
        if curr_velocity > target_velocity+20:
            target_velocity = 0
            brake = 20
        elif DESIRED_VELOCITY_KPH == 0:
            brake = 200
        else:
            brake = 0

        
        # velocity pid ========================================================================
        if target_velocity >= 0 and target_velocity <= 5:
            output = round(self.pid_0to5.pid(target_vel = target_velocity, current_vel = curr_velocity))
        elif target_velocity <= 10:
            output = round(self.pid_5to10.pid(target_vel = target_velocity, current_vel = curr_velocity))
        else:
            output = round(self.pid_10to20.pid(target_vel = target_velocity, current_vel = curr_velocity))
    
        target_velocity = int(output)
            
        return steering, target_velocity, brake, pure_steering


    
    def e_stop(self):
        self.erpCmd_msg.gear    = 0
        self.erpCmd_msg.speed   = 0
        self.erpCmd_msg.brake   = 200
        self.erpCmd_msg.e_stop  = True
        
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)
        
    def init_variable(self):

        self.pid_0to5.prev_error = 0
        self.pid_0to5.i_control  = 0

        self.pid_5to10.prev_error = 0
        self.pid_5to10.i_control  = 0

        self.pid_10to20.prev_error = 0
        self.pid_10to20.i_control  = 0
    
        self.pid_steer.prev_error = 0
        self.pid_steer.i_control = 0

    def visualization_heading_WP(self, Lp, Way, Ego, idx):

        wp_marker=make_marker(Way,idx)
        

        pursuit_path=Path()
        pursuit_path.header.frame_id='map'

        read_pose = PoseStamped()
        read_pose.pose.position.x=Ego.x
        read_pose.pose.position.y=Ego.y
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)

        read_pose = PoseStamped()
        read_pose.pose.position.x=Lp[0]
        read_pose.pose.position.y=Lp[1]
        read_pose.pose.position.z=1.
        pursuit_path.poses.append(read_pose)

        self.pursuit_pub.publish(pursuit_path)
        self.wp_pub.publish(wp_marker)

        return
# ------------------------ graph ----------------------------- #
    
    def lateral_offset_data(self): # erp의 현재 위치와 localpath 데이터중 첫번째 point와의 거리 차이
        x_error = self.path.poses[0].pose.position.x
        y_error = self.path.poses[0].pose.position.y
        lateral_error = sqrt((self.current_position.x-x_error)**2 + (self.current_position.y-y_error)**2)
        lateral_data = [lateral_error]
        self.lateral_writer.writerow(lateral_data)

        # 5.28 -> 시간에 따른 lateral_error csv 파일, 주행중 현재 위치 x,y 값 csv파일 , heading csv 파일 -> plot하는 파일도 추가
        # heading_offset 데이터는 local_path의 theta 값이 아닌 pure_pursuit 값을 통해 나온 steer 값과 실제 헤딩간의 차이를 나타내는 지표이다.
        # 08.05 -> purepursuit 과 현재 스티어값을 비교해야 한다!!!!!!!!!!!!!!!!
    def heading_offset_data(self, DESIRED_STEER): # purepursuit 계산의 p 제어 출력과 erp 현재 헤딩의 차이
        DESIRED_STEER_DEG = DESIRED_STEER / 2000 * (28.2)
        current_steer_deg = self.erpStatus_msg.steer/2000 * 28.2
        heading_data = [DESIRED_STEER_DEG, current_steer_deg]
        self.heading_writer.writerow(heading_data)

    def position_data(self): # erp 실제 주행 경로 gps 데이터
        position_data = [self.current_position.x, self.current_position.y]
        self.position_writer.writerow(position_data)

# ------------------------ callback -------------------------- #

    def odom_callback(self,msg):
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg

    def path_callback(self,msg):
        
        self.is_path=True
        self.path=msg



class pidControl:
    def __init__(self, p_gain, i_gain, d_gain, dt):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = dt
    
    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    




if __name__ == '__main__':
    Long_Control = Longitudinal_control()
