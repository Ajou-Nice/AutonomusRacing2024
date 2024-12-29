import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
import time, math


class State_machine:

    def __init__(self):

        rospy.init_node('state_machine', anonymous=True)

        self.cnt = 0
        # self.Obstacle_size_pub = rospy.Publisher("/traffic_labacon", Bool, queue_size= 10)
        # self.uturn_pub = rospy.Publisher("/uturn_point", Float32MultiArray, queue_size= 1)
        self.first_utm_pub = rospy.Publisher("/traffic_sign_location", Float32MultiArray, queue_size=1)
        self.second_utm_pub = rospy.Publisher("/traffic_sign_stop_location", Float32MultiArray, queue_size=1)
        # self.obastacle_trigger_pub = rospy.Publisher("/traffic_labacon", Bool, queue_size = 1)
        # self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.obastacle_trigger_pub = Bool()
        self.first_utm = Float32MultiArray()
        self.second_utm = Float32MultiArray()
        # self.uturn_point = Float32MultiArray()
        self.x = 0
        self.y = 0


        # rospy.Subscriber("/traffic_sign_location", Float32MultiArray, self.sign_vision_utm_callback)
        # rospy.Subscriber("/traffic_sign_stop_location", Float32MultiArray, self.sign_vision_stop_utm_callback)

        #-----------------------------------Main----------------------------------------
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # for _ in range(40):
            #     self.cnt += 1
            #     if self.cnt <= 1000:
            #         self.obastacle_trigger_pub.publish(False)
            #     else:
            #         self.obastacle_trigger_pub.publish(False)

            #     bev_msg = PoseArray()
            #     bev_pose = Pose()
            #     bev_pose.position.x = 10
            #     bev_pose.position.y = 10
            #     bev_pose.position.z = 0.0

            #     bev_pose.orientation.x = 10
            #     bev_pose.orientation.y = 10
            #     bev_pose.orientation.z = 10
            #     bev_pose.orientation.w = 10

            #     bev_msg.poses.append(bev_pose)
            self.first_utm.data = [0, 0, 0]
            self.second_utm.data = [0, 0, 0]
            # self.pickup_utm.append(self.x)
            # self.pickup_utm.append(self.y)
            self.first_utm_pub.publish(self.first_utm)
            self.second_utm_pub.publish(self.second_utm)
            print(f"going")

            

            # self.uturn_point.data.append(6.3)
            # self.uturn_pub.publish(self.uturn_point)

            # self.delivery_utm.data.append(self.x)
            # self.delivery_utm.data.append(self.y)

            # self.delivery_utm_pub.publish(self.delivery_utm)
            

            
            rate.sleep()
        #-------------------------------------------------------------------------------

    #-------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass
