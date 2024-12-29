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
from std_msgs.msg import Float32, String, Float32MultiArray, Int32, Bool
from geometry_msgs.msg import Point32,PoseStamped,Twist
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
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/parking_path", Path, self.parkingPath_callback)
        rospy.Subscriber("/lattice_path", Path, self.latticePath_callback)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/vel", TwistStamped, self.velocity_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/desired_velocity', Twist, self.desiredVelocity_callback)

        rospy.Subscriber('/parking_velocity', Twist, self.parkingVelocity_callback)
        rospy.Subscriber('/parking_gear', Bool, self.parkingGear_callback)

        # ================================== Publisher ================================ $
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size = 1)
        self.pursuit_pub=rospy.Publisher("/pursuit_path", Path, queue_size = 3)
        self.wp_pub = rospy.Publisher('/path_waypoint', Marker, queue_size=5)
        self.erp_msg = erpCmdMsg()


        # =====================================
        self.is_path = False
        self.is_latticePath = False
        self.is_parkingPath = False

        self.is_odom = False          
        self.is_velo = False
        self.is_PathState = False
        self.is_desiredVel = False

        self.is_parkingVel = False
        self.is_parkingGear = False
        # mission
        # =====================================
        self.vehicle_yaw=0.
        self.velocity=0.
        self.linear_velocity_x=0.; self.linear_velocity_y=0.
        
        # =====================================
        self.current_position=Point()
        self.vehicle_length = 2.2
        self.Path_state="Global_path"

        self.pid = pidControl()

        rate = rospy.Rate(10) # 30hz
        while not rospy.is_shutdown():

            is_ready = (self.is_path and self.is_latticePath and 
                        self.is_PathState and self.is_odom and 
                        self.is_desiredVel and self.is_velo)

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

    def localPath_callback(self,msg):
        
        self.is_path=True
        self.local_path=msg

    def parkingPath_callback(self, msg):

        self.is_parkingPath = True
        self.parking_path = msg

    def latticePath_callback(self, msg):
        
        self.is_latticePath = True
        self.lattice_path = msg

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

    def desiredVelocity_callback(self, msg):

        self.is_desiredVel = True
        self.desired_velocity = msg.liinear.x

    def parkingVelocity_callback(self, msg):

        self.is_parkingVel = True
        self.parking_velocity = msg.linear.x

    def parkingGear_callback(self, msg):

        self.is_parkingGear = True
        
        if msg.data :
            self.parking_gear = 2

        else :
            self.parking_gear = 0


    def control_state(self, Path_state):

        brake = 0

        if Path_state == "Global_path":

            self.path = self.local_path

            steering = self.calc_pure_pursuit(Path_state, self.vehicle_yaw)
            target_velocity=self.control_driving_velocity(steering)

        elif Path_state == "Parking_path":

            is_readyParking = self.is_parkingGear and self.is_parkingPath and self.is_parkingVel

            if is_readyParking:
            
                self.path = self.parking_path
                self.erp_msg.gear = self.parking_gear

                steering = self.calc_pure_pursuit(Path_state, self.vehicle_yaw)
                target_velocity = self.control_parking_velocity(steering)

            else:

                steering = 0
                target_velocity = 0
                brake = 0

        elif Path_state == "Obstacle_avoiding_path":
            
            self.path = self.lattice_path

            steering = self.calc_pure_pursuit(Path_state, self.vehicle_yaw)
            target_velocity=self.control_avoidingObs_velocity(steering)

        try :
            brake = self.control_brake()
        except :
            rospy.loginfo("Not ready Msg")
            pass

        return steering, target_velocity, brake
        

    def calc_pure_pursuit(self, Path_state, yaw): 
        ref_x = []
        ref_y = []

        for pose in self.path.poses:
            ref_x.append(pose.pose.position.x)
            ref_y.append(pose.pose.position.y)

        K=0.32; V_x=self.velocity*3.6

        
        if Path_state == "Global_path":
            Lp = 4.9 + K*V_x

        elif Path_state == "Obstacle_avoiding_path":
            Lp = 3 + K*V_x

        elif Path_state == "Parking_path":
            if self.parking_gear == 0:
                Lp = 3

            else:
                yaw += np.pi
                Lp = 2

        
        Lp_x = self.current_position.x + Lp * np.cos(yaw)
        Lp_y = self.current_position.y + Lp * np.sin(yaw)

        dis_P2 = np.sqrt((np.array(ref_x) - Lp_x)**2+(np.array(ref_y) - Lp_y)**2)
        min_index = np.argmin(dis_P2)

        Way_x = ref_x[min_index]
        Way_y = ref_y[min_index]

        x_2 = (Way_x - self.current_position.x) * np.cos(yaw) + (Way_y - self.current_position.y) * np.sin(yaw)
        y_2 = - (Way_x - self.current_position.x) * np.sin(yaw) + (Way_y - self.current_position.y) * np.cos(yaw)


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

    def control_driving_velocity(self,steering):

        steer_rad=steering/2000*0.4922
        target_velocity=0

        if (abs(steer_rad)<math.radians(5.)):
            target_velocity = self.desired_velocity

        elif (abs(steer_rad)<0.4922):
            target_velocity = 4
        else:
            target_velocity = 3

        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output>200:
            target_velocity=200

        elif output < 0:
            target_velocity = 0

        else:
            target_velocity=int(output)

        return target_velocity
    
    def control_parking_velocity(self, steering):
        steer_rad = steering/2000*0.4922

        target_velocity = self.parking_velocity

        if output>200:
            target_velocity=200

        elif output < 0 :
            output = 0

        else:
            target_velocity=int(output)

        return target_velocity

    
    def control_avoidingObs_velocity(self,steering):
        steer_rad=steering/2000*0.4922

        target_velocity = 3
        
        output = int(self.pid.pid(target_velocity,self.velocity*3.6))

        if output>200:
            target_velocity=200

        elif output < 0 :
            output = 0

        else:
            target_velocity=int(output)

        return target_velocity
    
    def control_brake(self):

        if self.Path_state == "Parking_path":

            if self.velocity > self.parking_velocity:
                brake = 20

            elif self.parking_velocity == 0:
                brake = 200
                
            else :
                brake = 0

        else:

            if self.velocity > self.desired_velocity:
                brake = 20

            elif self.desired_velocity == 0:
                brake = 200
                
            else :
                brake = 0

        return brake


    
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
