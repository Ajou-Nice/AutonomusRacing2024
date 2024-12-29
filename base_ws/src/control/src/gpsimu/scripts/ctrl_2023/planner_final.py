import rospy
import rospkg
from std_msgs.msg import Int32, Bool, Float32MultiArray, String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time, math

'''
표지판 /traffic_sign
parking : 0
delivery_1 : 1
delivery_2 : 2
delivery_3 : 3
static : 4

신호등: /traffic_light
stop : 5
red_left_arrow : 6
green_left_arrow : 7
go : 8
'''

class State_machine:

    def __init__(self):

        rospy.init_node('state_machine', anonymous=True)

        #--------------------------------Subscriber------------------------------------
        rospy.Subscriber('/current_waypoint', Int32, self.index_callback) # ERP42가 경로상 몇 번째 way_point에 위치한지 받아오기 (담담자: 이재훈 -> 김성준님의 코드 쓰기로 함) -> 토픽명 김성준님한테 알아오기
        rospy.Subscriber("/traffic_sign", Int32, self.traffic_sign_callback) # 표지판 인지 정보를 정수형으로 받기 (담당자: 유지상님)
        rospy.Subscriber("/traffic_light", Int32, self.traffic_light_callback) # 신호등 인지 정보를 정수형으로 받기 (담당자: 유지상님)
        rospy.Subscriber("/static_obstacle_trigger", Bool, self.static_callback) # 정적 장애물 미션이면 True, 아니면 False 받게 하기 (담당자: 김성준님)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback) # ERP42 위치 정보 받기 (담당자: 진영완님)
        rospy.Subscriber("/end_parking", Bool, self.end_parking_callback) # 주차 미션이 끝났는가? (담당자: 이재훈)
        rospy.Subscriber("/utm_sign", Float32MultiArray, self.sign_utm_callback) # 라이다로 표지판 utm 좌표 받기 (담당자: 방지윤님)
        rospy.Subscriver("/observed_sign", Bool, self.observed_callback) # 배달 미션에서 언제 멈출것인가(담당자: 유지상님 & 방지윤님)
        rospy.Subscriver("/stopLine", Float32MultiArray, self.stop_callback)# 정지선 유무: 0. -> False, 1. -> True, y픽셀 : float (담당자: 신규민님)
        #------------------------------------------------------------------------------

        #--------------------------------Publisher--------------------------------------
        self.Desired_velocity_publisher = rospy.Publisher('/desired_velocity', Twist, queue_size=1) # 원하는 속도를 제어기에 넘기기(모든 코드에서 속도는 target_velocity로 바꾸기)
        self.utn_cal_trigger_pub = rospy.Publisher("utm_cal_trigger", Bool, queue_size = 1) # 배달 좌표 계산하게 트리거 발행
        self.State_pub = rospy.Publisher("/State", String, queue_size= 10) # 아마 rosbag용?
        self.Path_pub = rospy.Publisher("/path_state", String, queue_size= 10) # 경로에 대한 정보를 문자열로 제어하기
        #-------------------------------------------------------------------------------

        #-----------------------------Initial_Parameter---------------------------------
        
        #↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        self.State = "Unready"
        self.Status_msg = "Not initialized"
        self.Path_state= "Global_path"
         #↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

        #↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        #callback이 정상적으로 들어오나? -> 더 추가할게 있을까?
        self.is_index = False
        self.is_traffic = False
        self.is_odom = False
        self.is_static = False
        self.is_end_parking = False
        #↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

        #↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        #배달 및 주차의 시작과 끝 트리거
        self.pick_end = False
        self.delivery_ready = False
        self.delivery_end = False
        self.pick_up_mode = 0 # 0이면 미션중이고, 1이면 완료
        self.delivery_mode = 0  # 0이면 미션중이고, 1이면 완료
        #↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
        
        #↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
        #픽업 장소 좌표 집어넣기
        self.pick_x = 3333333 # 픽업 utm x 좌표
        self.pick_y = 44444444 # 픽업 utm y 좌표
        #↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
        #-------------------------------------------------------------------------------

        #-----------------------------------Main----------------------------------------
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            self.State_space()
            self.Action_space()
            self.State_pub.publish(self.State)
            self.Path_pub.publish(self.Path_state)
            print(f"State: {self.State}, Action: {self.Action}, status: {self.status_msg}")

            rate.sleep()
        #-------------------------------------------------------------------------------


    #------------------------------Callback_function--------------------------------
    def index_callback(self, msg):

        self.is_index = True
        self.index = msg.data

    def traffic_sign_callback(self, msg):

        self.is_traffic = True
        self.traffic_sign = msg.data

    def traffic_light_callback(self, msg):

        self.is_traffic = True
        self.traffic_light = msg.data

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y

    def static_callback(self, msg):

        self.is_static = True
        self.static_trigger = msg.data

    def end_parking_callback(self, msg):

        self.is_end_parking = True
        self.end_parking = msg.data

    def sign_utm_callback(self, msg):

        data = msg.data
        self.sign_x = data[0]
        self.sign_y = data[1]

    def observed_callback(self, msg):

        self.ready_cal = msg.data

    def stop_callback(self, msg):

        data = msg.data
        self.observed_stop = data[0]
        self.stop_y = data[1]
    #-------------------------------------------------------------------------------

    #--------------------------------State Space------------------------------------
    def State_space(self):

        if self.State == "Unready":

            if self.is_index and self.is_traffic and self.is_odom: # 여기엔 뭘 더 추가해서 제한할것인가? 
                self.State = "Drive"

            self.Action = "Unready"

        elif self.State == "Drive":
            self.drive()

        elif self.State == "Intersection":
            self.Action = "Intersection_drive"

        elif self.State == "Obstacle_avoid":

            if not self.static_trigger: #경로내에 장애물이 없다면
                self.State = "Drive"

            self.Action = "Obstacle_Avoiding"

        elif self.State == "Parking":

            if self.end_parking: # 주차 코드에서 주차 완료 및 복귀까지 끝났다는 신호msg가 True이면
                self.State = "Drive"

            self.Action = "Parking_path"

        elif self.State == "Pick_up": # 전역 경로로 픽업 경로까지 가버리면 되는 상황, 굳이 경로 생성 코드 쓸 이유 없음 

            if self.pick_up_mode == 0 and not self.pick_end: # 아직 미션이 끝나지 않은 상황이라면
                if self.cal_error(self.x, self.y, self.pick_x, self.pick_y) < 0.3: # 목표 지점 n m이내로 들어오면
                    self.stop() # 정지 명령 보내기
                    time.sleep(2) # 미션상 1.5초 정지 해야하는데, 안전하게 2초로
                    self.pick_up_mode = 1 # 픽업 모드 1로 바꿔서 이 조건문으로 다시는 못 들어오게 하기
                    self.pick_end = True # 미션 끝났다고 바꾸기
                    self.State = "Drive" #상태천이
            
            self.Action = "Pick_up_mission"

        elif self.State == "Delivery": # 이건 같이 봐야함 (방지윤님, 유지상님, 이재훈)

            if self.delivery_mode == 0 and self.delivery_ready and not self.delivery_end: # 아직 배달 미션이 끝나지 않았고, 목표 지점 계산이 완료된 상태라면 

                self.global_path_pub() # 전역 경로 주행은 해야하니깐 일단 가라고 하기

                if self.cal_error(self.x, self.y, self.sign_x, self.sign_y) < 0.3: # 목표 지점 안에 들어오면

                    self.stop() # 정지 명령 보내기
                    time.sleep(2) # 미션상 1.5초 정지 해야하는데, 안전하게 2초로
                    self.delivery_mode = 1 # 픽업 모드 1로 바꿔서 이 조건문으로 다시는 못 들어오게 하기
                    self.delivery_end = True # 미션 완료로 바꾸기
                    self.State = "Drive" #상태천이

            self.Action = "Delivery_mission"

    #-------------------------------------------------------------------------------

    #-------------------------------Action Space------------------------------------
    def Action_space(self):

        if self.Action == "Unready":
            self.status_msg="Sensor Input Check"
            self.stop()

        elif self.Action == "Global_Path_Publish":

            self.global_path_pub()

        elif self.Action == "Intersection_drive":

            self.intersection()

        elif self.Action == "Obstacle_Avoiding":

            self.obstacle_path_pub()

        elif self.Action == "Parking_path":

            self.parking_path_pub()

        elif self.Action == "Pick_up_mission":

            self.global_path_pub()
            self.slow()

        elif self.Action == "Delivery_mission":

            self.delivery_path()
                
    #-------------------------------------------------------------------------------

    #-------------------------------Function_Area-----------------------------------


    #To do
    # Waypoint_index 맞게 drive 함수 조건 걸기
    # 교차로 및 배달 미션 유지상, 방지윤님과 말 맞추기
    

    def drive(self):

        if 91< self.index and self.index < 137 and not self.pick_end:
            self.State = "Pick_up"

        elif 743 < self.index and self.index < 804 and not self.delivery_end:
            self.State = "Delivery" 

        elif (351 < self.index and self.index < 376) or (440 < self.index and self.index < 467) or (670 < self.index and self.index < 703) or (855 < self.index and self.index < 884) or (997 < self.index and self.index < 1019) or (1503 < self.index and self.index < 1529) or (1581 < self.index and self.index < 1603):
            self.State = "Intersection"

        elif (192 < self.index and self.index < 240 or  545 < self.index and self.index < 655) and self.static_trigger: # 성준님과 더 말 맞춰보기
            self.State = "Obstacle_avoid"

        elif 1754 < self.index and self.index < 1810 and not self.end_parking: # 방지윤님과 유지상님과 같이 세부적으로 더 조정해줘야함 -> 대회측에서 확인 진행중
            self.State = "Parking"

        else:
            self.Action = "Global_Path_Publish"

    def intersection(self):

        if (351 < self.index and self.index < 376) or (440 < self.index and self.index < 467) or (1503 < self.index and self.index < 1529) or (1581 < self.index and self.index < 1603): # 미션 부분 공지 5, 6, 13, 14 번
            if self.traffic_light == 7:
                self.global_path_pub()
            else:
                if self.observed_stop == 1.:
                    self.stop() # 정지선을 인지하면 멈춤 -> 김성준님 의견 반영

        elif 670 < self.index and self.index < 703: # 미션 부분 공지 8번
            if self.traffic_light == 7 or self.traffic_light == 8:
                self.global_path_pub()
            else:
                if self.observed_stop == 1:
                    self.stop() # 정지선을 인지하면 멈춤 -> 김성준님 의견 반영

        elif (855 < self.index and self.index < 884) or (997 < self.index and self.index < 1019): #미션 부분 공지 10, 11번
            if self.traffic_light == 6:
                self.global_path_pub()
            else:
                if self.observed_stop == 1:
                    self.stop() # 정지선을 인지하면 멈춤 -> 김성준님 의견 반영


    def global_path_pub(self):
        
        self.status_msg = "Global Path Drive"
        self.Path_state = "Global_path"

    def obstacle_path_pub(self):
        
        self.status_msg = "Obstacle Avoiding Mission"
        self.Path_state ="Obstacle_avoiding_path"
            
    def parking_path_pub(self):

        self.status_msg = "Parking Mission"
        self.Path_state = "Parking_path"

    def delivery_path(self):

        self.global_path_pub() # 주행은 하고 있어야 하니깐 

        if self.ready_cal and not self.delivery_ready: # 계산하기 적합한 장소가 True이면서 아직 좌표 계산이 안된 상황이라면 
            self.stop() # 일단 멈추고
            self.utn_cal_trigger_pub.publish(True) # 계산하라고 방지윤님께 보내기
        else:
            self.utn_cal_trigger_pub.publish(False)

        if 302600 < self.sign_x <302609 & 4124012 < self.sign_y < 4124047: # 방지윤님이 계산한 값이 이 범위 내에 있다면
            self.delivery_ready = True # 배달 시작 준비 완료
        else:
            self.delivery_ready = False


    def stop(self): # 정지선이나 정지할 때, 목표속도를 0으로 하는 함수

        twist_msg = Twist()
        twist_msg.linear.x = 0
        self.Desired_velocity_publisher.publish(twist_msg) 

    def slow(self): # 미션 주행시에 적절한 속도

        twist_msg = Twist()
        twist_msg.linear.x = 5
        self.Desired_velocity_publisher.publish(twist_msg) 

    def accel(self): # 평범한 주행 상황

        twist_msg = Twist()
        twist_msg.linear.x = 10
        self.Desired_velocity_publisher.publish(twist_msg) 

    def cal_error(x, y, gx, gy):

        dx = gx - x
        dy = gy -y

        error = math.sqrt(pow(dx,2) + pow(dy,2))

        return error

    #-------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        State_machine()

    except rospy.ROSInterruptException:
        pass