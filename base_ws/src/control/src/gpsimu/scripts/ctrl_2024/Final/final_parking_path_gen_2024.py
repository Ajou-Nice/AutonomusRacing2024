#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt
import math
import rospy
import rospkg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Float32, String
from erp_driver.msg import erpStatusMsg, erpCmdMsg
import time


class Oblique_parking:

    def __init__(self):
        self.gear = 0
        rospy.init_node('parking_path', anonymous=True)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)  # Localization에서 받는 토픽명으로 바꾸기
        rospy.Subscriber('/parking_plot', Int32, self.park_number_callback)  # 주차공간 인지와 맞추기
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)
        rospy.Subscriber('/path_state', String, self.pathState_callback)
        rospy.Subscriber('/too_far', Bool, self.too_far_callback)
        rospy.Subscriber('/too_close', Bool, self.too_close_callback)
        rospy.Subscriber('/closing', Bool, self.closing_callback)

        self.parkingGear_publisher = rospy.Publisher('/parking_gear', Bool, queue_size=1)
        self.end_mission_pub = rospy.Publisher("/parking_end", Bool, queue_size=1)
        self.parking_publisher = rospy.Publisher('/parking_path', Path, queue_size=10)
        self.parking_velocity = rospy.Publisher('/parking_velocity', Int32, queue_size=1)
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=1)
        self.Desired_brake_pub = rospy.Publisher('/desired_brake', Int32, queue_size=1)
        self.parking_adjust_trigger_pub = rospy.Publisher('/parking_adjust_trigger', Bool, queue_size=1)
        self.start_lidar_pub = rospy.Publisher('/start_lidar', Bool, queue_size=1)
        self.parking_but_going_straight_trigger = rospy.Publisher('/parking_but_going_straight_trigger', Bool, queue_size=1)


        self.erp_msg = erpCmdMsg()
        self.local_path = Path()

        self.pid_0to5 = pidControl(p_gain = 3.5, i_gain = 0.1, d_gain = 0.003, dt = 1/30)

        self.is_odom = False
        self.is_park_num = False
        self.parking_gear = False  # 참이면 후진, 거짓이면 전진
        self.is_yaw = False
        self.is_path = False
        self.straight_yaw = 0
        self.real_yaw = math.radians(-119.5)
        self.is_far = False
        self.is_close = False
        self.is_closing = False
        self.erp = False
        self.too_far = False
        self.too_close = False

        self.end_type = False
        # self.parking_mode = 0  # 0 주차 진행, 1 주차 완료 후 복귀, 2 주차 미션 완료
        self.parking_type = 1  # 0이면 사선, 1이면 평행, 2면 수직
        self.sleep_mode = 0
        # self.is_park_num = True
        # self.parking_num = 2
        # self.velocity = 0
        self.erp_msg.steer = 0
        self.erp_msg.speed = 0
        self.erp_msg.brake = 0
        self.erp_msg.gear = 0
        self.Path_state = "global"


        self.pakr_sure = True

        rate = rospy.Rate(20)  # 20hz
        rospack = rospkg.RosPack()

        while not rospy.is_shutdown():
            if self.Path_state == "Parking_path":
                if self.is_odom and self.is_park_num:

                    position_x = self.x
                    position_y = self.y

                    start_x = []
                    start_y = []
                    parking_x = []
                    parking_y = []
                    return_x = []
                    return_y = []

                    # Load start path
                    pkg_name = 'gpsimu'
                    start_name = self.start_path(self.parking_num)
                    pkg_path = rospack.get_path(pkg_name)
                    full_path = pkg_path + '/path' + '/' + start_name + '.txt'
                    self.load_path(full_path, start_x, start_y)

                    # Load parking path
                    path_name = self.parking_path(self.parking_num, self.parking_type)
                    full_path = pkg_path + '/path' + '/' + path_name + '.txt'
                    self.load_path(full_path, parking_x, parking_y)

                    # Load return path
                    return_name = self.return_path(self.parking_num)
                    full_path = pkg_path + '/path' + '/' + return_name + '.txt'
                    self.load_path(full_path, return_x, return_y)

                    # Generate parking path
                    if self.erp:
                        self.generate_parking_path(position_x, position_y, parking_x, parking_y, start_x, start_y, return_x, return_y)
                        # self.erp_42_ctrl_pub.publish(self.erp_msg)
                    else:
                        self.generate_parking_path(position_x, position_y, parking_x, parking_y, start_x, start_y, return_x, return_y)
                else:
                    rospy.logwarn('\n parking path \n parking lot not detected \n')
                    self.parking_publisher.publish(self.local_path)
                    self.parkingGear_publisher.publish(False)
                    self.parking_velocity.publish(4)
            else:
                rospy.loginfo("Not parking path")
            rate.sleep()

    def too_far_callback(self, msg):
        self.is_far = True
        self.too_far = msg.data
        self.too_close = False

    def too_close_callback(self, msg):
        self.is_close = True
        self.too_close = msg.data
        self.too_far = False

    def closing_callback(self, msg):
        self.is_closing = msg.data

    def load_path(self, full_path, x_list, y_list):
        with open(full_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                tmp = line.split()
                x_list.append(float(tmp[0]))
                y_list.append(float(tmp[1]))

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def localPath_callback(self, msg):
        self.is_path = True
        self.local_path = msg

    def vehicle_yaw_callback(self, msg):
        self.is_yaw = True

        self.straight_yaw = msg.data
        
        if self.gear == 2:
            self.vehicle_yaw = msg.data + np.pi
            if self.vehicle_yaw > np.pi:
                self.vehicle_yaw -= 2*np.pi
            elif self.vehicle_yaw < -np.pi:
                self.vehicle_yaw += 2*np.pi
        else:
            self.vehicle_yaw = msg.data

    def park_number_callback(self, msg):
        self.is_park_num = True
        if self.pakr_sure:
            self.parking_num = msg.data
            self.pakr_sure = False

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed
        self.gear = self.erpStatus_msg.gear

    def pathState_callback(self, msg):
        self.is_PathState=True
        self.Path_state = msg.data

    def linear_equation(self, x1, y1, x2, y2):
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        return slope, intercept

    def error_function(self, x1, y1, x2, y2, yaw):
        local_x = (x2 - x1) * np.cos(yaw) + (y2 - y1) * np.sin(yaw)
        local_y = -(x2 - x1) * np.sin(yaw) + (y2 - y1) * np.cos(yaw)
        distance = sqrt(local_x**2 + local_y**2)
        return distance
    
    def yaw_error_function(self, x, y):
        diff = x - y
        if diff > np.pi:
            diff_yaw = abs(diff - 2*np.pi)
        elif diff < -np.pi:
            diff_yaw = abs(diff + 2*np.pi)
        else:
            diff_yaw = abs(diff)
        return diff_yaw

    def erp_heading_error_cal(self, real_yaw, straight_yaw):
        target_angle = math.degrees(real_yaw)
        current_angle = math.degrees(straight_yaw)

        if target_angle <0:
            target_angle +=360
        if current_angle <0:
            current_angle+=360

        angle_diff = (target_angle-current_angle)%360
        if angle_diff > 180:
            answer = angle_diff-360
        else:
            answer = angle_diff
        return answer

    def parking_path(self, num, type):
        if type == 1:  # 평행 주차
            if num == 1:
                return 'kcity_final_parking_lot1_reverse_fixed'
            elif num == 2:
                return 'kcity_final_parking_lot2_reverse_fixed'
            elif num == 3:
                return 'kcity_final_parking_lot3_reverse_fixed'

    def start_path(self, num):
        if num == 1:
            return 'kcity_final_parking_lot1_forward_fixed'
        elif num == 2:
            return 'kcity_final_parking_lot2_forward_fixed'
        elif num == 3:
            return 'kcity_final_parking_lot3_forward_fixed'

    def return_path(self, num):
        if num == 1:
            return "kcity_final_parking_lot1_return_fixed"
        elif num == 2:
            return "kcity_final_parking_lot2_return_fixed"
        elif num == 3:
            return "kcity_final_parking_lot3_return_fixed"

    def generate_parking_path(self, x, y, path_x, path_y, start_x, start_y, return_x, return_y):
        parking_path_msg = Path()
        parking_path_msg.header.frame_id = '/map'

        start_diff = self.error_function(x, y, start_x[-1], start_y[-1], self.vehicle_yaw)
        path_diff = self.error_function(x, y, path_x[-1], path_y[-1], self.vehicle_yaw)
        return_diff = self.error_function(x, y, return_x[-1], return_y[-1], self.vehicle_yaw)
        yaw_diff = self.yaw_error_function(self.straight_yaw, self.real_yaw)
        int_msg = Int32()

        if self.parking_type == 1:  # 평행 주차
            if self.sleep_mode == 0:
                self.handle_start_path(start_diff, int_msg, start_x, start_y)
                self.parking_but_going_straight_trigger.publish(True)
            elif self.sleep_mode == 1:
                self.parking_but_going_straight_trigger.publish(False)
                self.handle_reverse_path(path_diff, int_msg, path_x, path_y)
            elif self.sleep_mode == 2:
                self.parking_but_going_straight_trigger.publish(True)
                self.yaw_adjust_mode(yaw_diff)
            elif self.sleep_mode == 3:
                self.change_to_controller()
            elif self.sleep_mode == 4:
                self.preparing_for_return()
            elif self.sleep_mode == 5:
                self.handle_return_path(return_diff, int_msg, return_x, return_y)
            elif self.sleep_mode == 6:
                self.end_mission()

    def handle_start_path(self, start_diff, int_msg, start_x, start_y):
        print("mode 1 Going foward")
        print("start diff: " + str(start_diff))
        bool_msg = Bool()
        bool_msg.data = self.parking_gear
        self.parkingGear_publisher.publish(bool_msg)

        int_msg.data = 4

        if start_diff <= 2:
            print("going foward ended")
            int_msg.data = 0
            self.parking_velocity.publish(int_msg)
            self.apply_brake() 
            if self.velocity == 0:
                print("time sleep")
                time.sleep(1)
                self.remove_brake()
                self.sleep_mode = 1
            

        self.parking_velocity.publish(int_msg)
        self.publish_path(start_x, start_y)

    def handle_reverse_path(self, path_diff, int_msg, path_x, path_y):
        print("mode 2 reverse parking")
        print("reverse diff: " + str(path_diff))
        self.parking_gear = True
        int_msg.data = 2

        bool_msg = Bool()
        bool_msg.data = self.parking_gear
        self.parkingGear_publisher.publish(bool_msg)

        if path_diff < 2.85:
            print("reverse parking ended")
            int_msg.data = 0
            self.parking_velocity.publish(int_msg)
            self.apply_brake()
            if self.velocity == 0:
                self.remove_brake()
                time.sleep(1)
                self.parking_adjust_trigger_pub.publish(False)
                self.erp=True
                self.sleep_mode = 2

        self.parking_velocity.publish(int_msg)
        self.publish_path(path_x, path_y)

    def yaw_adjust_mode(self, yaw_diff):
        print("mode 3 adjusting yaw")
        print("yaw_diff = " + str(math.degrees(yaw_diff)))

        # if math.degrees(yaw_diff) <= 7:
        #     print("erp is straight")
        #     print("preparing for return")
        #     self.sleep_mode = 3

        # else:

        if self.erp_heading_error_cal(self.real_yaw, self.straight_yaw) > 0:
            print("erp is on the right side")
            self.erp_msg.steer = -2000
            target_velocity = 1
            output = round(self.pid_0to5.pid(target_vel = target_velocity*10, current_vel = self.velocity))
            if output >= 200:
                output = 200
            elif output <= 0:
                output = 0
            self.erp_msg.speed = output
            # self.erp_msg.speed = 50
            self.erp_msg.brake = 0
            self.erp_msg.gear = 0
            self.erp_42_ctrl_pub.publish(self.erp_msg)

            if math.degrees(yaw_diff) < 2:
                print("adjusting complete")
                self.erp_msg.steer = 0
                self.erp_msg.speed = 0
                self.erp_msg.brake = 21
                self.erp_msg.gear = 0
                self.erp_42_ctrl_pub.publish(self.erp_msg)
                time.sleep(5)
                self.sleep_mode = 3
                self.pid_0to5.init_variable()
            
        elif self.erp_heading_error_cal(self.real_yaw, self.straight_yaw) < 0:
            print("erp is on the left side")
            self.erp_msg.steer = 2000
            
            target_velocity = 1
            output = round(self.pid_0to5.pid(target_vel = target_velocity*10, current_vel = self.velocity))
            if output >= 200:
                output = 200
            elif output <= 0:
                output = 0
            self.erp_msg.speed = output
            # self.erp_msg.speed = 60
            self.erp_msg.brake = 0
            self.erp_msg.gear = 0
            self.erp_42_ctrl_pub.publish(self.erp_msg)
            if math.degrees(yaw_diff) < 2:
                print("adjusting complete")
                self.erp_msg.steer = 0
                self.erp_msg.speed = 0
                self.erp_msg.brake = 21
                self.erp_msg.gear = 0
                self.erp_42_ctrl_pub.publish(self.erp_msg)
                time.sleep(5)
                self.sleep_mode = 3
                self.pid_0to5.init_variable()

    def change_to_controller(self):
        print("removing brake")
        self.erp_msg.steer = 0
        self.erp_msg.speed = 0
        self.erp_msg.brake = 0
        self.erp_msg.gear = 0
        self.erp_42_ctrl_pub.publish(self.erp_msg)
        time.sleep(1)
        self.start_lidar_pub.publish(True)
        self.sleep_mode = 4 

    def preparing_for_return(self):
        print(self.too_far, self.too_close)
        if not self.too_far or self.too_close:
            print("erp is too close")
            self.erp_msg.steer = 0
            target_velocity = 1
            output = round(self.pid_0to5.pid(target_vel = target_velocity*10, current_vel = self.velocity))
            if output >= 200:
                output = 200
            elif output <= 0:
                output = 0
            self.erp_msg.speed = output
            # self.erp_msg.speed = 60
            self.erp_msg.brake = 0
            self.erp_msg.gear = 2
            self.erp_42_ctrl_pub.publish(self.erp_msg)
            self.pid_0to5.init_variable()

        elif self.too_far:
            print("erp is far enough")
            self.erp_msg.steer = 0
            self.erp_msg.speed = 0
            self.erp_msg.brake = 21
            self.erp_msg.gear = 0
            self.erp_42_ctrl_pub.publish(self.erp_msg)
            time.sleep(1)
            self.erp_msg.steer = 0
            self.erp_msg.speed = 0
            self.erp_msg.brake = 0
            self.erp_msg.gear = 0
            self.erp_42_ctrl_pub.publish(self.erp_msg)
            time.sleep(1)
            self.start_lidar_pub.publish(False)
            self.erp = False
            self.sleep_mode = 5

    def handle_return_path(self, return_diff, int_msg, return_x, return_y):
        self.parking_adjust_trigger_pub.publish(True)
        print("mode 4 returning")
        print("return diff: " + str(return_diff))
        self.parking_gear = False
        int_msg.data = 4

        bool_msg = Bool()
        bool_msg.data = self.parking_gear
        self.parkingGear_publisher.publish(bool_msg)

        if return_diff < 3:
            print("returning ended")
            self.sleep_mode = 6

        self.parking_velocity.publish(int_msg)
        self.publish_path(return_x, return_y)

    def publish_path(self, path_x, path_y):
        parking_path_msg = Path()
        parking_path_msg.header.frame_id = '/map'

        for i in range(len(path_x)):
            pose = PoseStamped()
            pose.pose.position.x = path_x[i]
            pose.pose.position.y = path_y[i]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            parking_path_msg.poses.append(pose)

        self.parking_publisher.publish(parking_path_msg)

    def end_mission(self):
        end_msg = Bool()
        self.end_type = True
        end_msg.data = self.end_type
        self.end_mission_pub.publish(end_msg)

    def apply_brake(self):
        int_msg = Int32()
        int_msg.data = 200
        self.Desired_brake_pub.publish(int_msg)
    
    def remove_brake(self):
        int_msg = Int32()
        int_msg.data = 0
        self.Desired_brake_pub.publish(int_msg)
    
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
    
    def init_variable(self):
        self.prev_error = 0
        self.i_control = 0

if __name__ == '__main__':
    try:
        Oblique_parking()
    except rospy.ROSInterruptException:
        pass

