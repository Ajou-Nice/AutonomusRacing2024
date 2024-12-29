#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import pow
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32


class global_path_pub:
    def __init__(self, pkg_name='gpsimu', path_name='kcity_highway'):
        rospy.init_node('global_path_pub', anonymous=True)

        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/way_point_index', Int32, queue_size=10)

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)
        full_path = pkg_path + '/path' + '/' + path_name + '.txt'
        self.f = open(full_path, 'r')
        lines = self.f.readlines()

        self.path_x = []
        self.path_y = []

        for line in lines:
            tmp = line.split()

            self.path_x.append(float(tmp[0]))
            self.path_y.append(float(tmp[1]))

        self.f.close()

        while not rospy.is_shutdown():
            self.find_closest_index()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def find_closest_index(self):
        msg = Int32()
        if hasattr(self, 'x') and hasattr(self, 'y'):
            min_distance_squared = float('inf')
            closest_index = None

            for i, (path_x, path_y) in enumerate(zip(self.path_x, self.path_y)):
                distance_squared = pow(self.x - path_x, 2) + pow(self.y - path_y, 2)
                if distance_squared < min_distance_squared:
                    min_distance_squared = distance_squared
                    closest_index = i


            if closest_index is not None:
                print("Closest index:", closest_index)
                msg.data = closest_index
                self.pub.publish(msg)


if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass

