#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt
import rospy
import rospkg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Float32
from erp_driver.msg import erpStatusMsg, erpCmdMsg
import time


class Oblique_parking:

    def __init__(self):
        rospy.init_node('parking_path', anonymous=True)

        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)  # Localization에서 받는 토픽명으로 바꾸기
        rospy.Subscriber('/parking_plot', Int32, self.park_number_callback)  # 주차공간 인지와 맞추기
        rospy.Subscriber("/local_path", Path, self.localPath_callback)
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber("/vehicle_yaw", Float32, self.vehicle_yaw_callback)

        self.parkingGear_publisher = rospy.Publisher('/parking_gear', Bool, queue_size=1)
        self.end_mission_pub = rospy.Publisher("/parking_end", Bool, queue_size=1)
        self.parking_publisher = rospy.Publisher('/parking_path', Path, queue_size=10)
        self.parking_velocity = rospy.Publisher('/parking_velocity', Int32, queue_size=1)
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_ctrl_cmd", erpCmdMsg, queue_size=1)
        self.erp_msg = erpCmdMsg()
        self.local_path = Path()

        self.is_odom = False
        self.is_park_num = False
        self.parking_gear = False  # 참이면 후진, 거짓이면 전진
        self.is_yaw = False
        self.is_path = False

        self.end_type = False
        self.parking_mode = 0  # 0 주차 진행, 1 주차 완료 후 복귀, 2 주차 미션 완료
        self.parking_type = 1  # 0이면 사선, 1이면 평행, 2면 수직
        self.sleep_mode = 0
        # self.is_park_num = True
        # self.parking_num = 1
        # self.velocity = 0


        rate = rospy.Rate(20)  # 20hz
        rospack = rospkg.RosPack()

        while not rospy.is_shutdown():
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
                self.generate_parking_path(position_x, position_y, parking_x, parking_y, start_x, start_y, return_x, return_y)

            else:
                self.parking_publisher.publish(self.local_path)
                self.parkingGear_publisher.publish(False)
                self.parking_velocity.publish(3)
                print("None msg")

            rate.sleep()

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
        self.vehicle_yaw = msg.data

    def park_number_callback(self, msg):
        self.is_park_num = True
        self.parking_num = msg.data

    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.velocity = self.erpStatus_msg.speed

    def linear_equation(self, x1, y1, x2, y2):
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        return slope, intercept

    def error_function(self, x1, y1, x2, y2, yaw):
        local_x = (x2 - x1) * np.cos(yaw) + (y2 - y1) * np.sin(yaw)
        local_y = -(x2 - x1) * np.sin(yaw) + (y2 - y1) * np.cos(yaw)
        distance = sqrt(local_x**2 + local_y**2)
        return distance

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
            return "kcity_final_parking_lot2_reverse_fixed"
        elif num == 3:
            return "kcity_final_parking_lot3_reverse_fixed"

    def generate_parking_path(self, x, y, path_x, path_y, start_x, start_y, return_x, return_y):
        parking_path_msg = Path()
        parking_path_msg.header.frame_id = '/map'

        start_diff = self.error_function(x, y, start_x[-1], start_y[-1], self.vehicle_yaw)
        path_diff = self.error_function(x, y, path_x[-1], path_y[-1], self.vehicle_yaw)
        return_diff = self.error_function(x, y, return_x[-1], return_y[-1], self.vehicle_yaw)

        int_msg = Int32()

        if self.parking_type == 1:  # 평행 주차
            if self.sleep_mode == 0:
                self.handle_start_path(start_diff, int_msg, start_x, start_y)
            elif self.sleep_mode == 1:
                self.handle_reverse_path(path_diff, int_msg, path_x, path_y)
            elif self.sleep_mode == 2:
                self.handle_return_path(return_diff, int_msg, return_x, return_y)
            elif self.sleep_mode == 3:
                self.end_mission()

    def handle_start_path(self, start_diff, int_msg, start_x, start_y):
        print("mode 1 Going foward")
        print("start diff: " + str(start_diff))
        bool_msg = Bool()
        bool_msg.data = self.parking_gear
        self.parkingGear_publisher.publish(bool_msg)

        int_msg.data = 3

        if start_diff <= 2:
            print("going foward ended")
            int_msg.data = 0
            self.parking_velocity.publish(int_msg)
            if self.velocity == 0:
                print("time sleep")
                self.parking_gear = True
                bool_msg = Bool()
                bool_msg.data = self.parking_gear
                self.parkingGear_publisher.publish(bool_msg)
                time.sleep(2)
                self.sleep_mode = 1
            
        self.parking_velocity.publish(int_msg)
        self.publish_path(start_x, start_y)

    def handle_reverse_path(self, path_diff, int_msg, path_x, path_y):
        print("mode 2 reverse parking")
        print("reverse diff: " + str(path_diff))
        int_msg.data = 3

        if path_diff < 1.5:
            print("reverse parking ended")
            int_msg.data = 0
            self.parking_velocity.publish(int_msg)
            if self.velocity == 0:
                time.sleep(2)
                self.parking_gear = False
                bool_msg = Bool()
                bool_msg.data = self.parking_gear
                self.parkingGear_publisher.publish(bool_msg)
                self.sleep_mode = 2
                

        self.parking_velocity.publish(int_msg)
        self.publish_path(path_x, path_y)

    def handle_return_path(self, return_diff, int_msg, return_x, return_y):
        print("mode 3 returning")
        print("return diff: " + str(return_diff))
        int_msg.data = 3

        if return_diff < 1:
            print("returning ended")
            self.sleep_mode = 3

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


if __name__ == '__main__':
    try:
        Oblique_parking()
    except rospy.ROSInterruptException:
        pass

