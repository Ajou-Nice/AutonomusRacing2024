#!/usr/bin/env python
import rospy
from microstrain_inertial_msgs.msg import FilterHeading  # FilterHeading 메시지 타입을 임포트하세요
import math

class IMUAngularVelocityCalculator:
    def __init__(self):
        self.prev_heading = None  # 이전 헤딩 값 (단위: radians)
        self.prev_time = None     # 이전 시간 (단위: seconds)
        rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)

    def heading_callback(self, msg):
        # 현재 헤딩 값과 시간 가져오기
        current_heading = msg.heading_rad  # 헤딩 값 (radians)
        current_time = rospy.Time.now().to_sec()

        if self.prev_heading is not None:
            # 헤딩 차이 계산 (래핑 처리 포함)
            delta_heading = current_heading - self.prev_heading
            # 헤딩 차이를 -π에서 π 사이로 변환
            delta_heading = (delta_heading + math.pi) % (2 * math.pi) - math.pi

            # 시간 차이 계산
            delta_time = current_time - self.prev_time

            if delta_time > 0:
                # 각속도 계산 (rad/s)
                angular_velocity = delta_heading / delta_time

                # 결과 출력 또는 퍼블리시
                rospy.loginfo("Angular vel: {:.5f} rad/s".format(angular_velocity))
            else:
                rospy.logwarn("Time difference is zero or negative, cannot compute angular velocity.")

        # 이전 헤딩과 시간 업데이트
        self.prev_heading = current_heading
        self.prev_time = current_time

if __name__ == '__main__':
    rospy.init_node('imu_angular_velocity_calculator')
    calculator = IMUAngularVelocityCalculator()
    rospy.spin()
