#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from asyncio import set_event_loop
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
from geometry_msgs.msg import Point32,PoseStamped
import time
from visualization_msgs.msg import Marker

# IMU EXCEPTION and VELOCITY
# No 종방향 pid, No affected by velocity
# integrate global drive and labacone u-turn drive

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


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        #TODO: (1) subscriber, publisher 선언
        # =============================== subscriber =============================== #
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/desired_velocity', Int32, self.desiredVelocity_callback)
        
        rospy.Subscriber('/uturn_point', Float32MultiArray, self.uturnPoint_callback)
        rospy.Subscriber('/avoid_point', Float32MultiArray, self.avoidPoint_callback)
        rospy.Subscriber('/lane_error', Int32, self.lane_callback)
        
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()


 
        # =====================================
        self.is_path=True
        self.is_odom=True 
        self.is_yaw = False         
        self.is_status = False
        self.is_PathState=False
        self.is_desiredVel = False
        # 처음 시작 시 필요한 토픽

        self.is_laba=False
        self.is_lane=False
        self.is_obj=False
        # 미션 별 필요한 토픽
        self.uturn_point = 0

        # =====================================
        self.vehicle_yaw=0.
        self.velocity=0.
        
        # =====================================
        self.current_position=Point()
        self.vehicle_length = 1.04
        self.Path_state="Global_path"

        self.pid_0to5 = pidControl(p_gain = 3.5, i_gain = 0.1, d_gain = 0.003, dt = 1/30)
        self.pid_5to10 = pidControl(p_gain = 3.5, i_gain = 0.3, d_gain = 0.003, dt = 1/30)
        self.pid_10to20 = pidControl(p_gain = 3, i_gain = 0.4, d_gain = 0.003, dt = 1/30)
        self.pid_laba=pidControl_Laba()
        self.pid_lane = pidControl_Lane()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            # print(self.is_path ,self.is_odom, self.is_PathState ,self.is_desiredVel , self.is_status)
            is_ready = self.is_odom and self.is_PathState and self.is_desiredVel and self.is_status

            if is_ready and self.erpStatus_msg.control_mode == 1:

                self.erp_msg.gear = 0

                steering, target_velocity, brake = self.control_state(self.Path_state)

                self.erp_msg.steer = steering
                self.erp_msg.speed = target_velocity
                self.erp_msg.brake = brake

                self.erp_42_ctrl_pub.publish(self.erp_msg)

                print("Current_PATH_STATE : {}".format(self.Path_state))
                print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
                print("Current_Velocity : {:.2f}".format(self.velocity/10)) #km/h
            
            else:
                self.init_variable()
                print('Error')

            rate.sleep()

    def path_callback(self,msg):
        
        self.is_path=True
        self.path=msg

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        # self.vehicle_yaw=msg.pose.pose.position.z

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True
        self.vehicle_yaw = msg.data

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data


    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed

    def uturnPoint_callback(self, msg):

        self.is_laba = True
        self.uturn_point = msg.data[0]

    def avoidPoint_callback(self, msg):

        self.is_obj = True
        self.avoid_point = msg.data[1] # y 값만 불러오는 이유는 y 값이 좌우 정보를 포함하고 있기 때문

    def lane_callback(self, msg):

        self.is_lane = True
        self.lane_error = msg.data

    def desiredVelocity_callback(self, msg):

        # desired_velocity 는 km/h 단위

        self.is_desiredVel = True
        self.desired_velocity = msg.data

    def control_state(self, Path_state):

        brake = 0

        if Path_state == "Global_path":

            steering = self.calc_stanley()
            target_velocity, goal_velocity =self.control_driving_velocity(steering)
            brake = self.control_brake(goal_velocity)


        elif Path_state == "Rubber_cone_path":
            
            if self.is_laba:

                steering = self.cal_laba_steer(self.uturn_point)
                target_velocity, goal_velocity=self.control_labacone_velocity(steering)
                brake = self.control_brake(goal_velocity)
            else:

                steering=0
                target_velocity=0
                brake = 50

        elif Path_state == "Dead_zone_path":

            if self.is_lane == True:

                steering = self.cal_lane_steer(self.lane_error)
                target_velocity, goal_velocity = self.control_lane_velocity(steering)
                brake = self.control_brake(goal_velocity)
            
            else:

                steering = self.calc_stanley()
                target_velocity = 40 # 4km/h

        elif Path_state == "Obstacle_avoiding_path":

            if self.is_obj == True:

                steering = self.cal_laba_steer(self.avoid_point)
                target_velocity, goal_velocity=self.control_avoidingObs_velocity(steering)
                brake = self.control_brake(goal_velocity)

            else : 

                steering = 0
                target_velocity = 0
                brake = 200

        elif Path_state == "Estop_dynamic_path":

            steering = 0
            target_velocity = 0
            brake = 200

        return steering, target_velocity, brake
        
    '''
    def calc_pure_pursuit(self): 
        ref_x = []
        ref_y = []

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        K=0.32; V_x=self.velocity/10

        Lp = 4.9+K*V_x
        Lp_x = self.current_position.x + Lp * np.cos(self.vehicle_yaw)
        Lp_y = self.current_position.y + Lp * np.sin(self.vehicle_yaw)

        dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)
        min_index = np.argmin(dis_P2)

        Way_x = ref_x[min_index]
        Way_y = ref_y[min_index]

        x_2 = (Way_x - self.current_position.x) * np.cos(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.sin(self.vehicle_yaw)
        y_2 = - (Way_x - self.current_position.x) * np.sin(self.vehicle_yaw) + (Way_y - self.current_position.y) * np.cos(self.vehicle_yaw)


        L_bar = np.sqrt(x_2**2 +y_2**2)
        sin_alpha = y_2/L_bar

        steering = -self.vehicle_length * 2 * y_2 / (L_bar)**2

        steering = int(2000*(steering/0.4922))


        if steering > 2000:
            steering = 2000
        elif steering < -2000:
            steering = -2000

        self.visualization_heading_WP([Lp_x, Lp_y],[Way_x, Way_y], self.current_position,min_index)

        return steering
    '''

    def calc_stanley(self):

        k_s = 1.9


        # ==================== 전륜 중심 utm 좌표 구하기 ==========================
        front_utm_x = self.current_position.x + self.vehicle_length * np.cos(self.vehicle_yaw)
        front_utm_y = self.current_position.y + self.vehicle_length * np.sin(self.vehicle_yaw)

        # ==================== 현재 속도 구하기 ===================================

        curr_velocity = self.erpStatus_msg.speed
        curr_velocity_kmh = curr_velocity / 10

        # ==================== local_path 불러오기 ================================

        ref_x = []  
        ref_y = []  

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        
        
        # ==================== x(t) 구하기 ========================================

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

        path_yaw = math.atan2(delta_north, delta_east)

        pi_t = self.vehicle_yaw - path_yaw

        pi_t = (pi_t + np.pi) % (2 * np.pi) - np.pi

        # ======================= 조향각 구하기 ======================================

        if curr_velocity_kmh != 0:

            steering = 1.35*pi_t + np.arctan(k_s * x_t / curr_velocity_kmh)

        
        else: # 속도가 0일때 예외처리 
            steering = pi_t

        rospy.logwarn(f'\n x_t : {x_t} \n pi_t : {pi_t} \n')


        # rad to erp_steer
        steering = int(2000*(steering/0.4922))

        if steering > 2000:
            steering = 2000
        elif steering < -2000:
            steering = -2000
            

        self.visualization_heading_WP([front_utm_x, front_utm_y],[Way_x, Way_y], self.current_position,min_index)

        print("=================================")

        # if steering>0:
        #     print("RIGHT HANDLING")
        # else:
        #     print("LEFT HANDLING")
            
        print("steering: {:.2f}".format(steering))

        print("=================================")
        




        return steering
    
    def cal_laba_steer(self,labacone_point):

        keeping_dist = 0.
        error_dist = keeping_dist-labacone_point
    
        # error_point = error_dist if labacone_point<0 else -error_dist # left, right
        error_point = error_dist    # left
        # error_point = -error_dist # right


        target_steering = self.pid_laba.pid_Laba(error_point)

        target_steering = int(2000*(target_steering/0.4922))

        if target_steering > 2000:
            target_steering = 2000
        
        elif target_steering < -2000:
            target_steering = -2000
        
        return target_steering
    
    def velocity_pid(self, target_velocity):

        if target_velocity >= 0 and target_velocity <= 5:
            output = round(self.pid_0to5.pid(target_vel = target_velocity*10, current_vel = self.velocity))
        elif target_velocity <= 10:
            output = round(self.pid_5to10.pid(target_vel = target_velocity*10, current_vel = self.velocity))
        else:
            output = round(self.pid_10to20.pid(target_vel = target_velocity*10, current_vel = self.velocity))

        return int(output)

    def control_labacone_velocity(self,steering):

        steer_rad=steering/2000*0.4922
        target_velocity=0

        if (abs(steer_rad)<math.radians(15)):
            target_velocity = self.desired_velocity
        elif (abs(steer_rad)<0.4922):
            target_velocity = 5
        else:
            target_velocity = 5

        goal_velocity = target_velocity


        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity, goal_velocity

    def control_driving_velocity(self,steering):

        steer_rad=steering/2000*0.4922
        target_velocity=0

        if (abs(steer_rad)<math.radians(5)):
            target_velocity = self.desired_velocity
        elif (abs(steer_rad)<0.4922):
            target_velocity = 10
        else:
            target_velocity = 7

        goal_velocity = target_velocity

        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity , goal_velocity
    
    def control_avoidingObs_velocity(self,steering):
        steer_rad=steering/2000*0.4922

        if (abs(steer_rad)<math.radians(5)):
            target_velocity = self.desired_velocity
        elif (abs(steer_rad)<0.4922):
            target_velocity = 5
        else:
            target_velocity = 5

        goal_velocity = target_velocity
        
        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity, goal_velocity
    
    def control_lane_velocity(self, steering):
        steer_rad = steering/2000*0.4922

        if (abs(steer_rad)<math.radians(3)):
            target_velocity = self.desired_velocity
        elif (abs(steer_rad)<0.4922):
            target_velocity = 5
        else:
            target_velocity = 5


        goal_velocity = target_velocity
        
        output = self.velocity_pid(target_velocity)

        target_velocity = self.value_check(output)

        return target_velocity, goal_velocity
    
    def cal_lane_steer(self, lane_error):

        error_point = lane_error

        target_steering = self.pid_lane.pid_Lane(error_point) * 0.002

        target_steering = int(2000*(target_steering/0.4922))

        if target_steering > 2000:
            target_steering = 2000

        elif target_steering < -2000:
            target_steering = -2000
        
        return target_steering
    
    
    
    def control_brake(self, goal_velocity):

        if goal_velocity == 5:
            if self.velocity/10 > goal_velocity+2:    
                brake = 5
            else:
                brake = 0
        elif goal_velocity == 7:
            if self.velocity/10 > goal_velocity+2:    
                brake = 20
            else:
                brake = 0

        elif goal_velocity == 4:
            if self.velocity/10 > goal_velocity+2:    
                brake = 5
            else:
                brake = 0

        elif goal_velocity == 10:
            if self.velocity/10 > goal_velocity+2:    
                brake = 20
            else:
                brake = 0
        elif goal_velocity == 15:
            if self.velocity/10 > goal_velocity+2:    
                brake = 5
            else:
                brake = 0
        elif goal_velocity == 20:
            if self.velocity/10 > goal_velocity+2:    
                brake = 5
            else:
                brake = 0
        else:
            rospy.logwarn(f'CHOOSE THE BRAKE MAGNITUDE WITH THIS SPEED')
            if self.velocity/10 > goal_velocity+2:
                brake = 10
            else:
                brake = 0
            
        return brake

    def value_check(self, output):

        if output>200:
            target_velocity=200

        elif output < 0:
            target_velocity = 0

        else:
            target_velocity=int(output)

        return target_velocity
    
    def visualization_heading_WP(self, Lp, Way, Ego,idx):

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
    
    def init_variable(self):

        self.pid_0to5.prev_error = 0
        self.pid_0to5.i_control  = 0

        self.pid_5to10.prev_error = 0
        self.pid_5to10.i_control  = 0

        self.pid_10to20.prev_error = 0
        self.pid_10to20.i_control  = 0
    
        self.pid_laba.prev_error = 0
        self.pid_laba.i_control = 0

        self.pid_lane.prev_error = 0
        self.pid_lane.i_control = 0

class pidControl_Laba:
    def __init__(self):
        self.p_gain = 0.75
        self.i_gain = 0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 1/30

    def pid_Laba(self,error):

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
class pidControl_Lane:
    def __init__(self):
        self.p_gain = 0.75
        self.i_gain = 0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.001

    def pid_Lane(self,error):

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
class pidControl:
    def __init__(self, p_gain, i_gain , d_gain, dt):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = dt
    
    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
