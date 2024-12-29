import rospy

from erp_driver.msg import erpCmdMsg, erpStatusMsg
class Synario:
    
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        
        # Publisher 설정
        self.erp_42_ctrl_pub = rospy.Publisher("/erp42_status", erpStatusMsg, queue_size=10)

        # 메시지 초기화
        self.erp_msg = erpCmdMsg()
        
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # ERP42 제어 명령 메시지 설정
            self.erp_msg.brake = 200  # 브레이크 강도 설정
            self.erp_42_ctrl_pub.publish(self.erp_msg)  # 메시지 퍼블리시

            print(self.erp_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        synario_instance = Synario()
    except rospy.ROSInterruptException:
        pass