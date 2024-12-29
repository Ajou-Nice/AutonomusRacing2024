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

        # Subscriber 설정
        rospy.Subscriber("/erp42_status", erpStatusMsg, self.status_callback)

        # Publisher 설정
        self.light_signal_publisher = rospy.Publisher('/light_signal', String, queue_size=10)

        # 파라미터 초기화
        self.is_status = False
        self.brake = 0
        self.Light_signal = "0"  # 기본값 설정

        # 주 반복문
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if not self.is_status:
                print("erpStatus_msg doesn't come")
                self.light_signal_publisher.publish(self.Light_signal)
                print(f"Signal: {self.Light_signal}")
            else:
                if self.brake > 1:
                    self.brake_signal_on()
                else:
                    self.brake_signal_off()

                print(f"Signal: {self.Light_signal}")

            rate.sleep()

    # 브레이크 신호 켜기
    def brake_signal_on(self):
        self.Light_signal = "0"
        self.light_signal_publisher.publish(self.Light_signal)

    # 브레이크 신호 끄기
    def brake_signal_off(self):
        self.Light_signal = "1"
        self.light_signal_publisher.publish(self.Light_signal)

    # ERP42 상태 메시지를 처리하는 콜백 함수
    def status_callback(self, msg):
        self.is_status = True
        self.erpStatus_msg = msg
        self.brake = self.erpStatus_msg.brake

if __name__ == '__main__':
    try:
        Signal_machine()
    except rospy.ROSInterruptException:
        pass