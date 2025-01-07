#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from log_parser import *
from config import *
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA


# 색상 결정 함수
def get_color_from_state(state):
    if state == 1:  # G (초록)
        return ColorRGBA(0, 1, 0, 1)
    elif state == 0:  # R (빨강)
        return ColorRGBA(1, 0, 0, 1)
    else:  # None (회색)
        return ColorRGBA(0.5, 0.5, 0.5, 1)
    
def make_marker( number ):
    global signal_states
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "signals"
    marker.id = number
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = SIGNAL_MARKER_POSION_LIST[number][0] # 위치는 필요에 따라 변경하세요
    marker.pose.position.y = SIGNAL_MARKER_POSION_LIST[number][1] 
    marker.color = get_color_from_state(signal_states[number*2])
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    return marker


 ##  None = -1 G = 1 R =0
signal_states = [

        #200 남서좌 16  
        -1,  0,

        #300 남동직  9  
        -1,  0,

        #400 동직  3
        -1,  0,

        #500 동좌 4
        -1,  0,

        #610 북좌 6
        -1,  0,

        #300 북서직 13
        -1,  0,

        #200 북서직 13
        -1,  0,

    ]

def main():
    # ROS 노드 초기화
    global signal_states
    rospy.init_node('input_to_topic_publisher', anonymous=True)

    # 발행할 토픽 설정 (예: 'input_topic')
    pub = rospy.Publisher('signal_state', Int32MultiArray, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=10)
    pattern_number =rospy.get_param('pattern_number')

    # 사용자로부터 문자를 입력받아 토픽으로 발행
    while not rospy.is_shutdown():
        # 사용자 입력 받기
        signal_states = parse_log_and_json(signal_states ,pub,pattern_number)
        
  
        if  signal_states == False:
            rospy.loginfo("종료합니다.")
            break

        marker_array = MarkerArray()

        for i in range(len(signal_states)//2):
            marker_array.markers.append(make_marker(i))
        marker_pub.publish(marker_array)


        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
