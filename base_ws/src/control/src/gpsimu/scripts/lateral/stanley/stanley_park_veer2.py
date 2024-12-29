#!/usr/bin/env python3
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
DESIRED_VELOCITY_KPH = 3

# Steer PID Parameter
P_GAIN_STEER = 0
I_GAIN_STEER = 0
D_GAIN_STEER = 0

# P_GAIN_STEER = 0
# I_GAIN_STEER = 0
# D_GAIN_STEER = 0

steer_vel = 3
steer_vel_2 = 3

pid_true = False
# stanley Parameter

k_s = 1.3

add_name = 'paldal_index+5_steervel_10_0.9*pi_t + np.arctan'

# ==============================================================================================================================================================

# State
MANUAL_MODE = 0
AUTO_MODE = 1

#Data setting
# is_lateral_offset_data = True # erp 현재위치와 경로 사이의 차이 계산
# is_heading_offset_data = True # purepursuit 헤딩 계산값과 차량의 현재 헤딩값 차이 계산
# is_position_data = True # erp 실제 이동경로 gps 데이터


# Save File Name -> 폴더 생성해놓기
LATERAL_OFFSET_FILE_NAME = 'lateral_offset/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
HEADING_OFFSET_FILE_NAME = 'heading_offset/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
POSITION_DATA_FILE_NAME = 'position_data/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'

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
        rospy.init_node('stanley', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        # =====================================
        self.erpCmd_msg     = erpCmdMsg()
        self.erpStatus_msg  = erpStatusMsg()

        self.prev_steering = 0
        self.current_position=Point()

        self.vehicle_length = 1.04

        self.is_status = False
        self.is_path   = False
        self.is_odom   =False
        self.is_yaw = False
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
        
        # self.later_f = open(LATERAL_OFFSET_FILE_NAME, 'w')
        # self.heading_f = open(HEADING_OFFSET_FILE_NAME, 'w')
        # self.position_f = open(POSITION_DATA_FILE_NAME, 'w')

        # self.lateral_writer = csv.writer(self.later_f)
        # self.heading_writer = csv.writer(self.heading_f)
        # self.position_writer = csv.writer(self.position_f)

        # ====================================

        rate = rospy.Rate(frameRate) # 30hz

        while not rospy.is_shutdown():
            if not self.is_status:
                rospy.loginfo("ERP Status msg do not arrived....")

            elif not self.is_odom:
                rospy.loginfo("Odometry msg do not arrived....")

            elif not self.is_path:
                rospy.loginfo("Path msg do not arrived....")

            # elif not self.is_yaw:
            #     rospy.loginfo("Yaw msg do not arrived....")
            
            else:
                if self.erpStatus_msg.control_mode == AUTO_MODE:

                    self.main()

                else:
                    self.init_variable()
            rate.sleep()

# ==========================================================================================
    def main(self):

        steering, target_velocity, brake, stanley_steering = self.cal_control()

        self.erpCmd_msg.gear  = 2
        self.erpCmd_msg.steer = steering
        self.erpCmd_msg.speed = target_velocity
        self.erpCmd_msg.brake = brake

        self.max_value()
        print('Current_velocity : {}'.format(self.erpStatus_msg.speed))
        self.erp_42_ctrl_pub.publish(self.erpCmd_msg)


        # 데이터 생성
        # if is_lateral_offset_data:
        #     self.lateral_offset_data()
        # if is_heading_offset_data:
        #     self.heading_offset_data(stanley_steering)
        # if is_position_data:
        #     self.position_data()
        # return
        
    def calc_stanley(self):
        
        k_s = 1.65

        ref_x = []  
        ref_y = []  

        # ==================== local_path 불러오기 ================================

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        # ==================== 후진 주차 스랜리 ==========================
        if self.erpCmd_msg.gear == 2:

            print("ggggggggggggggggggggggggggggggggggggggggg")

            # ==================== 횡방향은 반대 중심, 헤딩은 후륜중심 utm 좌표 구하기 ==========================
            utm_x_for_heading = self.current_position.x + self.vehicle_length*1.25 * np.cos(self.vehicle_yaw)
            utm_y_for_heading = self.current_position.y + self.vehicle_length*1.25 * np.sin(self.vehicle_yaw)
            utm_x_for_lateral = self.current_position.x + (self.vehicle_length*2) * np.cos(self.vehicle_yaw)
            utm_y_for_lateral = self.current_position.y + (self.vehicle_length*2) * np.sin(self.vehicle_yaw)

            # ==================== x(t) 구하기 ========================================

            dis_P2_lateral = np.sqrt((np.array(ref_x) - utm_x_for_lateral)**2 + (np.array(ref_y) - utm_y_for_lateral)**2)
            dis_P2_heading = np.sqrt((np.array(ref_x) - utm_x_for_heading)**2 + (np.array(ref_y) - utm_y_for_heading)**2)
            
            # 최솟값 인덱스 찾기
            min_index_lateral = np.argmin(dis_P2_lateral)
            min_index_heading = np.argmin(dis_P2_heading)

            # 해당 인덱스에 대한 경로 위치 좌표
            Way_x_lateral = ref_x[min_index_lateral]
            Way_y_lateral = ref_y[min_index_lateral]

            # 현재 위치와 경로 위치의 차이 계산
            x_2_lateral = (Way_x_lateral - utm_x_for_lateral) * np.cos(self.vehicle_yaw) + (Way_y_lateral - utm_y_for_lateral) * np.sin(self.vehicle_yaw)
            y_2_lateral = - (Way_x_lateral - utm_x_for_lateral) * np.sin(self.vehicle_yaw) + (Way_y_lateral - utm_y_for_lateral) * np.cos(self.vehicle_yaw)

            if y_2_lateral > 0:
                x_t = -np.min(dis_P2_lateral) 
            else:
                x_t = np.min(dis_P2_lateral) 

            # ====================== pi(t) 구하기 =====================================

            # min_index + 1을 사용하고, 범위를 벗어나는 경우 예외 처리
            if min_index_heading + 1 < len(ref_x):
                delta_east = ref_x[min_index_heading + 1] - ref_x[min_index_heading]
                delta_north = ref_y[min_index_heading + 1] - ref_y[min_index_heading]

            else:  # min_index가 ref_x의 마지막 인덱스일 때
                if min_index_heading - 1 >= 0:
                    delta_east = ref_x[min_index_heading] - ref_x[min_index_heading - 1]
                    delta_north = ref_y[min_index_heading] - ref_y[min_index_heading - 1]
                else: # 경로 데이터가 1개일때
                    delta_east = 0  # min_index가 0일 때 (예외 처리)
                    delta_north = 0

        # ==================== 전진 스랜리 ==========================

        else:
            front_utm_x = self.current_position.x 
            front_utm_y = self.current_position.y

            dis_P2 = np.sqrt((np.array(ref_x) - front_utm_x)**2 + (np.array(ref_y) - front_utm_y)**2)
        
            # 최솟값 인덱스 찾기
            min_index = np.argmin(dis_P2)

            # 해당 인덱스에 대한 경로 위치 좌표
            Way_x = ref_x[min_index]
            Way_y = ref_y[min_index]

            # 현재 위치와 경로 위치의 차이 계산
            x_2 = (Way_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.sin(self.vehicle_yaw)
            y_2 = - (Way_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.cos(self.vehicle_yaw)

            if y_2 > 0:
                x_t = -np.min(dis_P2) 
            else:
                x_t = np.min(dis_P2) 

            # ====================== pi(t) 구하기 =====================================

            # min_index + 1을 사용하고, 범위를 벗어나는 경우 예외 처리
            if min_index + 1 < len(ref_x):
                delta_east = ref_x[min_index + 1] - ref_x[min_index]
                delta_north = ref_y[min_index + 1] - ref_y[min_index]
            else:  # min_index가 ref_x의 마지막 인덱스일 때
                if min_index - 1 >= 0:
                    delta_east = ref_x[min_index] - ref_x[min_index - 1]
                    delta_north = ref_y[min_index] - ref_y[min_index - 1]
                else: # 경로 데이터가 1개일때
                    delta_east = 0  # min_index가 0일 때 (예외 처리)
                    delta_north = 0


        # ==================== 현재 속도 구하기 ===================================

        curr_velocity = self.erpStatus_msg.speed
        curr_velocity_kmh = curr_velocity / 10

        # ======================= 조향각 구하기 ======================================

        path_yaw = math.atan2(delta_north, delta_east)

        pi_t = self.vehicle_yaw - path_yaw

        pi_t = (pi_t + np.pi) % (2 * np.pi) - np.pi

        # ======================= 조향각 구하기 ======================================

        if curr_velocity_kmh != 0:
            steering = 3*pi_t + 0.5*np.arctan(k_s * x_t / curr_velocity_kmh)
        
        else: # 속도가 0일때 예외처리 
            steering = pi_t

        rospy.logwarn(f'\n\n\n x_t : {x_t} \n\n\n\n\n')


        # rad to erp_steer
        steering = int(2000*(steering/0.4922))


        # self.visualization_heading_WP([front_utm_x, front_utm_y],[Way_x, Way_y], self.current_position,min_index)

        print("=================================")

        # if steering>0:
        #     print("RIGHT HANDLING")
        # else:
        #     print("LEFT HANDLING")
            
        steering *= -1

        print("steering: {:.2f}".format(steering))

        print("=================================")
        

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
        steering = self.calc_stanley()
        steering_rad = steering/2000 * 0.4922
        stanley_steering = steering
        if pid_true:
            steering = int(self.pid_steer.pid(target_vel = steering , current_vel = curr_steer))


        # velocity for steer =================================================================
        if (abs(steering_rad)<math.radians(4.)):
            target_velocity = DESIRED_VELOCITY_KPH * 10
        elif (abs(steering_rad)<0.4922):
            target_velocity = steer_vel * 10 # erpunit
        else:
            target_velocity = steer_vel_2 * 10 # erpunit


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
            
        return steering, target_velocity, brake, stanley_steering


    
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
    
    # def lateral_offset_data(self): # erp의 현재 위치와 localpath 데이터중 첫번째 point와의 거리 차이
    #     x_error = self.path.poses[0].pose.position.x
    #     y_error = self.path.poses[0].pose.position.y

    #     x_2 = (x_error - self.current_position.x) * np.cos(self.vehicle_yaw) + (y_error - self.current_position.y) * np.sin(self.vehicle_yaw)
    #     y_2 = - (x_error - self.current_position.x) * np.sin(self.vehicle_yaw) + (y_error - self.current_position.y) * np.cos(self.vehicle_yaw)
    #     lateral_error = y_2
        
    #     lateral_data = [lateral_error]
    #     self.lateral_writer.writerow(lateral_data)

    # def heading_offset_data(self, DESIRED_STEER): # stanley 계산의 p 제어 출력과 erp 현재 헤딩의 차이
    #     DESIRED_STEER_DEG = DESIRED_STEER / 2000 * (28.2)
    #     current_steer_deg = self.erpStatus_msg.steer/2000 * 28.2
    #     heading_data = [DESIRED_STEER_DEG, current_steer_deg]
    #     self.heading_writer.writerow(heading_data)

    # def position_data(self): # erp 실제 주행 경로 gps 데이터
    #     position_data = [self.current_position.x, self.current_position.y]
    #     self.position_writer.writerow(position_data)

# ------------------------ callback -------------------------- #

    def odom_callback(self,msg):
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw = msg.pose.pose.position.z + np.pi

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg

    def path_callback(self,msg):
        
        self.is_path=True
        self.path=msg

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data + np.pi

        if self.vehicle_yaw > np.pi:
            self.vehicle_yaw -= 2*np.pi
        elif self.vehicle_yaw < -np.pi:
            self.vehicle_yaw += 2*np.pi


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
