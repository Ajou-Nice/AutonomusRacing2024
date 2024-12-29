import rospy
from std_msgs.msg import Int32, UInt8, Bool
from erp_driver.msg import erpStatusMsg

class StateMachine:

    def __init__(self):
        rospy.init_node('state_machine', anonymous=True)

        # Initialize the publisher for /current_waypoint topic
        self.waypoint_pub = rospy.Publisher('/current_waypoint', Int32, queue_size=1)
        self.current_waypoint = Int32()

        # Initialize the publisher for /erp42_status topic
        self.erp42_pub = rospy.Publisher('/erp42_status', erpStatusMsg, queue_size=1)
        self.erp42_status = erpStatusMsg()

        waypoint_start_time = rospy.get_time()  # 시작 시간을 기록
        brake_start_time = waypoint_start_time  # brake 시작 시간 기록
        rate = rospy.Rate(10)  # 10 Hz
        waypoint_index = 0  # 초기 waypoint 값
        brake_value = 0  # 초기 brake 값

        while not rospy.is_shutdown():
            current_time = rospy.get_time()  # 현재 시간 확인
            elapsed_waypoint_time = current_time - waypoint_start_time  # waypoint 경과 시간 계산
            elapsed_brake_time = current_time - brake_start_time  # brake 경과 시간 계산

            # 5초마다 waypoint 값 변경 (0부터 20까지 반복)
            if elapsed_waypoint_time >= 5:
                waypoint_start_time = current_time  # 시간을 다시 설정
                waypoint_index = (waypoint_index + 1) % 21  # 0부터 20까지 반복

            # 5초마다 brake 값 변경 (0과 200을 번갈아 가며 발행)
            if elapsed_brake_time >= 5:
                brake_start_time = current_time  # 시간을 다시 설정
                brake_value = 0 if brake_value == 200 else 200  # 0과 200 사이를 전환

            # Set ERP42Status message fields
            self.erp42_status.control_mode = 0
            self.erp42_status.e_stop = False
            self.erp42_status.gear = 0
            self.erp42_status.speed = 0
            self.erp42_status.steer = 0
            self.erp42_status.brake = brake_value
            self.erp42_status.encoder = 0
            self.erp42_status.alive = 0

            # Publish the messages
            self.current_waypoint.data = waypoint_index
            self.waypoint_pub.publish(self.current_waypoint)
            self.erp42_pub.publish(self.erp42_status)

            rate.sleep()

if __name__ == '__main__':
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass
