import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time, math
from erp_driver.msg import erpCmdMsg, erpStatusMsg

class Signal_machine:

    def __init__(self):
        rospy.init_node('signal_machine', anonymous=True)

        # Subscriber 설정----------------------------------------------------------------------------------------------------
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기
        
        # Publisher 설정-----------------------------------------------------------------------------------------------------
        self.light_signal_publisher = rospy.Publisher('/light_signal', String, queue_size=10)

        # 파라미터 초기화
        self.is_status = False
        self.brake = 0
        self.Light_signal = "00"  # 기본값 설정
        self.is_index = False
        self.is_position = False
        #주차, 픽업, 첫번째 좌회전 인덱스 재입력 해야함
        self.intersection_index = {'case1' : [433, 522], # 우회전
                                  'case2' : [532, 594], # 좌회전
                                  #'case3' : [-1, -1], #직진
                                  #'case4' : [-1, -1], #직진
                                  'case5' : [1305, 1404], # 좌회전
                                  'case6' : [1529, 1621], # 좌회전
                                  'case7' : [1997, 2055],# 우회전
                                  #'case8' : [-1, -1],
                                  #'case9' : [-1, -1],
                                  'case10' : [200, 263], # 첫번째 좌회전 예상값으로 넣음, 
                                  #'case11' : [-1, -1], # 배달 픽업 쌍깜빡이
                                  'case12' : [263, 346], # 소형 장애물 쌍깜빡이
                                  #'case13' : [1100, 1210], # 배달 도착 쌍깜빡이
                                  #'case14' : [1740, 1806], # 주차 쌍깜빡이
                                  'case15' : [900, 994] #대형 장애물 
                                  }


        # 주 반복문
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            
            #--------------------brake-----------------------------------
            if not self.is_status:
                print("erpStatus_msg doesn't come")
            else:
                if self.brake > 100:
                    self.brake_signal_on()
                else:
                    self.brake_signal_off()
            
            #--------------------intersection----------------------------------
            
            for key, (start, end) in self.intersection_index.items():
                if start <= self.index <= end:
                    self.is_position = True
                    self.intersection_case = key
                    break
                else:
                    self.is_position = False
            
            
            if self.is_position:
                if self.intersection_case == 'case1':
                    self.right_signal_on()
                
                elif self.intersection_case == 'case2':
                    self.left_signal_on()                
                
                elif self.intersection_case == 'case5':
                    self.left_signal_on()
                
                elif self.intersection_case == 'case6':
                    self.left_signal_on()
                
                elif self.intersection_case == 'case7':
                    self.right_signal_on()

                elif self.intersection_case == 'case10':  # 첫 번쨰 좌회전!!!!!!
                    self.left_signal_on()


                elif self.intersection_case == 'case12':  # 소형 장애물 쌍깜빡이
                    self.all_signal_on()

                elif self.intersection_case == 'case15':  # 대형 장애물 쌍깜빡이
                    self.all_signal_on()

                else:
                    pass

            else:
                self.blink_off()

            rate.sleep()

    # 브레이크 신호 켜기
    def brake_signal_on(self):
        self.Light_signal = "Brake_ON"
        self.light_signal_publisher.publish(self.Light_signal)

    # 브레이크 신호 끄기
    def brake_signal_off(self):
        self.Light_signal = "Brake_OFF"
        self.light_signal_publisher.publish(self.Light_signal)

    def right_signal_on(self):
        self.Light_signal = "R_Blink"
        self.light_signal_publisher.publish(self.Light_signal)

    def left_signal_on(self):
        self.Light_signal = "L_Blink"
        self.light_signal_publisher.publish(self.Light_signal)

    def all_signal_on(self): # 비상깜빡이
        self.Light_signal = "Emergency"
        self.light_signal_publisher.publish(self.Light_signal)

    def blink_off(self):
        self.Light_signal = "Blink_OFF"
        self.light_signal_publisher.publish(self.Light_signal)


    # ERP42 상태 메시지를 처리하는 콜백 함수
    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.brake = self.erpStatus_msg.brake
        
    def index_callback(self, msg):

        self.is_index = True
        self.index = msg.data


if __name__ == '__main__':
    try:
        Signal_machine()
    except rospy.ROSInterruptException:
        pass