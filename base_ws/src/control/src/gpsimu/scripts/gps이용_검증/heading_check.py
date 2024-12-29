#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import time
import os
import math

class OdomSubscriber:
    def __init__(self):
        rospy.init_node('odom_subscriber', anonymous=True)
        rospy.Subscriber('/odom_gps', Odometry, self.odom_callback)
        
        self.start_time = None
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.end_x = None
        self.end_y = None
        self.end_yaw = None

        self.set_time = 5 # 시간설정

        self.rate = rospy.Rate(20)  # 20 Hz
        self.data_received = False

        rospy.spin()

    def odom_callback(self, msg):
        current_time = time.time()
        
        # 처음 데이터를 수신할 때 시작 시간과 시작 좌표를 저장
        if not self.data_received:
            self.start_time = current_time
            self.start_x = msg.pose.pose.position.x
            self.start_y = msg.pose.pose.position.y
            self.start_yaw = msg.pose.pose.position.z
            self.data_received = True
            rospy.loginfo("Started receiving data...")
        
        elapsed_time = current_time - self.start_time

        if elapsed_time >= self.set_time:
            self.end_x = msg.pose.pose.position.x
            self.end_y = msg.pose.pose.position.y
            self.end_yaw = msg.pose.pose.position.z

            delta_x = self.end_x - self.start_x
            delta_y = self.end_y - self.start_y
            angle = math.atan2(delta_y, delta_x)

            heading_difference = self.end_yaw - angle

            rospy.loginfo("Angle between start and end position: {:.5f} radians".format(angle))
            rospy.loginfo("Start x: {:.2f}, y: {:.2f}, yaw: {:.2f}".format(self.start_x, self.start_y, self.start_yaw))
            rospy.loginfo("End x: {:.2f}, y: {:.2f}, yaw: {:.2f}".format(self.end_x, self.end_y, self.end_yaw))
            rospy.loginfo("Heading Defference (End yaw - GPS heading) : {:.5f}".format(heading_difference))


            # 노드 종료
            rospy.signal_shutdown(f"Data collected for {self.set_time} second.")
            os._exit(0)  # 강제 종료

if __name__ == '__main__':
    try:
        OdomSubscriber()
    except rospy.ROSInterruptException:
        pass
