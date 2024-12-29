#!/usr/bin/env python
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
from erp_driver.msg import erpCmdMsg
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
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("vel", TwistStamped, self.velocity_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)

        rospy.Subscriber('/uturn_point', Float32MultiArray, self.uturnPoint_callback)
        rospy.Subscriber('/avoid_point', Float32MultiArray, self.avoidPoint_callback)
        rospy.Subscriber('/lane_error', Int32, self.lane_callback)
        
        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()


 
        # =====================================
        self.is_path=False
        self.is_odom=False          
        self.is_velo=False
        self.is_PathState=False
        # 처음 시작 시 필요한 토픽

        self.is_laba=False
        self.is_lane=False
        self.is_obj=False
        # 미션 별 필요한 토픽

        # =====================================
        self.vehicle_yaw=0.
        self.velocity=0.
        self.linear_velocity_x=0.; self.linear_velocity_y=0.
        
        # =====================================
        self.current_position=Point()
        self.vehicle_length = 2.2
        self.Path_state="Global_path"

        self.pid = pidControl()
        self.pid_laba=pidControl_Laba()
        self.pid_lane = pidControl_Lane()

        rate = rospy.Rate(10) # 30hz
        while not rospy.is_shutdown():

            is_ready = self.is_path and self.is_odom and self.is_PathState

            if is_ready :

                self.erp_msg.gear = 0

                steering, target_velocity, brake = self.control_state(self.Path_state)

                self.erp_msg.steer = steering
                self.erp_msg.speed = target_velocity
                self.erp_msg.brake = brake

                self.erp_42_ctrl_pub.publish(self.erp_msg)

                print("Current_PATH_STATE : {}".format(self.Path_state))
                print("Target_Velocity : {:.2f}, Target_steering : {:.2f}".format(target_velocity/10, math.degrees(steering/2000*0.4922)))
                print("Current_Velocity : {:.2f}".format(self.velocity*3.6)) #km/h


            rate.sleep()

    def path_callback(self,msg):
        
        self.is_path=True
        self.path=msg

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data


    def velocity_callback(self,msg): 
        self.is_velo = True

        if not np.isnan(msg.twist.linear.x):
            self.linear_velocity_x = msg.twist.linear.x
            self.linear_velocity_y = msg.twist.linear.y
            self.velocity = math.sqrt(pow(msg.twist.linear.x, 2) + pow(msg.twist.linear.y, 2))
        else:
            self.linear_velocity_x=0.
            self.linear_velocity_y=0.
            self.velocity=0.

    def uturnPoint_callback(self, msg):

        self.is_laba = True
        self.uturn_point = msg.data[1]

    def avoidPoint_callback(self, msg):

        self.is_obj = True
        self.avoid_point = msg.data[1]

    def lane_callback(self, msg):

        self.is_lane = True
        self.lane_error = msg.data

    def control_state(self, Path_state):

        brake = 0

        if Path_state == "Global_path":

            steering = self.calc_pure_pursuit()
            target_velocity=self.control_driving_velocity(steering)

        elif Path_state == "Rubber_cone_path":
            
            if self.is_laba==True:

                steering = self.cal_laba_steer(self.uturn_point)
                target_velocity=self.control_labacone_velocity(steering)

            else:

                steering=0
                target_velocity=0
                brake = 50

        elif Path_state == "Dead_zone_path":

            if self.is_lane == True:

                steering = self.cal_lane_steer(self.lane_error)
                target_velocity = self.control_lane_velocity(steering)
            
            else:

                steering = self.calc_pure_pursuit()
                target_velocity = 40 # 4km/h

        elif Path_state == "Obstacle_avoiding_path":

            if self.is_obj == True:

                steering = self.cal_laba_steer(self.avoid_point)
                target_velocity=self.control_avoidingObs_velocity(steering)

            else : 

                steering = 0
                target_velocity = 0
                brake = 200

        elif Path_state == "Estop_dynamic_path":

            steering = 0
            target_velocity = 0
            brake = 200

        return steering, target_velocity, brake
        

    def calc_pure_pursuit(self,): 
        ref_x = []
        ref_y = []

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        K=0.32; V_x=self.velocity*3.6

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

    
    def cal_laba_steer(self,labacone_point):

        keeping_dist = 0.
        error_dist = keeping_dist-labacone_point
    
        # error_point = error_dist if labacone_point<0 else -error_dist # left, right
        error_point = error_dist    # left
        # error_point = -error_dist # right


        target_steering = self.pid_laba.pid_Laba(error_point)*0.45

        target_steering = int(2000*(target_steering/0.4922))

        if target_steering > 2000:
            target_steering = 2000
        elif target_steering < -2000:
            target_steering = -2000
        
        return target_steering
    
    def control_labacone_velocity(self,steering):

        steer_rad=steering/2000*0.4922
        target_velocity=0

        if (abs(steer_rad)<math.radians(0.2622)):
            target_velocity = 5
        elif (abs(steer_rad)<0.4922):
            target_velocity = 3
        else:
            target_velocity = 3

        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output>200:
            target_velocity=200
        else:
            target_velocity=int(output)

        return target_velocity

    def control_driving_velocity(self,steering):

        steer_rad=steering/2000*0.4922
        target_velocity=0

        if (abs(steer_rad)<math.radians(5.)):
            target_velocity = 10
        elif (abs(steer_rad)<0.4922):
            target_velocity = 4
        else:
            target_velocity = 3

        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output<0:
            output=0

        if output>200:
            target_velocity=200
        else:
            target_velocity=int(output)

        return target_velocity
    
    def control_avoidingObs_velocity(self,steering):
        steer_rad=steering/2000*0.4922

        target_velocity = 3
        
        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output>200:
            target_velocity=200
        else:
            target_velocity=int(output)

        return target_velocity

    
    def cal_lane_steer(self, lane_error):

        error_point = lane_error

        target_steering = self.pid_lane.pid_Lane(error_point) * 0.002

        target_steering = int(2000*(target_steering/0.4922))

        if target_steering > 2000:
            target_steering = 2000

        elif target_steering < -2000:
            target_steering = -2000
        
        return target_steering
    
    def control_lane_velocity(self, steering):
        steer_rad = steering/2000*0.4922

        target_velocity = 5

        if steer_rad > math.radians(5.):
            target_velocity = 3

        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output<0:
            output=0

        if output>200:
            target_velocity=200

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


class pidControl_Laba:
    def __init__(self):
        self.p_gain = 1
        self.i_gain = 0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1

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
        self.p_gain = 1
        self.i_gain = 0
        self.d_gain = 0.005
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1

    def pid_Lane(self,error):

        #TODO: (4) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output
    
class pidControl:
    def __init__(self):
        self.p_gain = 6
        self.i_gain = 2.2
        self.d_gain = 0
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.1
    
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
