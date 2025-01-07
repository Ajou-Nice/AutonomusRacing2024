#!/usr/bin/python3
# -*- coding: utf-8 -*-

from typing import Tuple, Union, List

from torchvision.transforms import ToTensor
import numpy as np
import rospy
import math

from std_msgs.msg import Int32, Bool ,Float32MultiArray
from std_msgs.msg import Int32, Bool
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from calculate_disance import caculate_local_postion 
import time



TRAFFIC_LABEL_LIST = [ 'red' , 'green' ,'yellow' , 'red_left_arrow', 'green_left_arrow']
LABACON_LABEL_LIST = ['labacon' , 'labacon_1','labacon_2','labacon_3']
SIGNAL_LABEL_LIST = [ 'delivery_1' ,'delivery_2' ,'delivery_3']
WAY_POINT_THERSHOLD = 300

POS_LIST_NUMVER = 50
POS_UPDATE_THERSHOLD =0.5


def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes

class YoloPostProcess:
    def __init__(self, yolo_topic,yolo_traffic_topic, way_point_topic, pub_topic_1, pub_topic_2, pub_topic_3, pub_topic_4, pub_topic_5,img_size, class_labels):
        self.img_size =img_size
        self.class_labels = class_labels

        self.current_time=time.time()
        self.way_pt = 0
        self.way_cnt = 0
        self.prev_sign_id = -1
        self.delivery_id=1
        self.sign_pos_dict ={ 
            1 :[],
            2: [],
            3: []
        }
        self.sign_cnt = 0
        self.prev_labacon_id = -1
        self.labacon_cnt = 0
        self.prev_light_id = -1
        self.light_cnt = 0
        self.uturnStartFlag=False
        self.nolabaConStack =0
        

        self.img_subscriber = rospy.Subscriber(
            yolo_topic, Detection2DArray, self.yolo_post_process
        )

        self.img_subscriber = rospy.Subscriber(
            yolo_traffic_topic, Detection2DArray, self.yolo_traffic_process
        )

        self.way_pts_subscriber = rospy.Subscriber(
            way_point_topic, Int32, self.way_point
        )

        self.traffic_sign_publisher = rospy.Publisher(
            pub_topic_1, Int32, queue_size=1
        )
        self.traffic_labacon_publisher = rospy.Publisher(
            pub_topic_2, Bool, queue_size=1
        )
        self.traffic_sign_location_publisher = rospy.Publisher(
            pub_topic_4, Float32MultiArray, queue_size=1
        )

        self.traffic_sign_stop_location_publisher = rospy.Publisher(
            pub_topic_5, Float32MultiArray, queue_size=1
        )

        self.traffic_light_publisher = rospy.Publisher(
            pub_topic_3, Int32, queue_size=1
        )

        self.point_publisher =rospy.Publisher('/object_position', MarkerArray, queue_size=10)

        rospy.loginfo("YOLOv7 post-processing")

    
    def way_point(self, data_):
        self.way_pt = data_.data
 

    def make_maker(self, pos_x, pos_y ,num):
    
 
        
        # 새로운 마커 생성
        marker = Marker()
        
        # MarkerArray에 추가할 새로운 마커 설정
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "velodyne"  # 좌표 프레임

        marker.ns = "object_positions"  # 네임스페이스 설정
        marker.id = num  # 마커 고유 ID

        marker.type = Marker.SPHERE  # 점을 구로 표현
        marker.action = Marker.ADD  # 추가 액션
        
        # 마커의 위치 설정
        marker.pose.position.x =  pos_y# cm -> m 변환
        marker.pose.position.y = -pos_x  
        marker.pose.position.z = 0  # z축이 없는 경우 0으로 설정
        
        # 마커의 크기 설정 (Sphere의 경우 크기 필요)
        marker.scale.x = 1  # 구의 크기
        marker.scale.y = 1
        marker.scale.z = 1
        
        # 마커 색상 설정 (예: 빨간색 구)
        marker.color.r = 1.0
        marker.color.g = num/3
        marker.color.b = 0.0
        marker.color.a = 1.0  # 투명도

        # 마커 생명주기 (영원히 지속되도록 설정)
        marker.lifetime = rospy.Duration()
        # 메시지 발행
        return marker

   
    def update_pos_dict(self, target,new_pos):
        if len(self.sign_pos_dict[target]) == 0:
            self.sign_pos_dict[target].append(new_pos)
            return
        if(math.dist(self.sign_pos_dict[target][-1], new_pos )> POS_UPDATE_THERSHOLD):
            self.sign_pos_dict[target]= [new_pos]
        else:
            self.sign_pos_dict[target].append(new_pos)

            if len(self.sign_pos_dict[target]) > POS_LIST_NUMVER:
                self.sign_pos_dict[target].pop(0)
        

    def yolo_traffic_process(self,data):
        light_area =[]
        for i in range(len(data.detections)):
            class_id = self.class_labels[data.detections[i].results[0].id]
            if class_id in TRAFFIC_LABEL_LIST: # traffic_light
                light_area.append([class_id, data.detections[i].bbox.size_x * data.detections[i].bbox.size_y])  # x, y, size_x, size_y
        if (len(light_area) ==0 ):
            return
        best_light_idx = np.argmax(np.array(light_area), axis = 0)[1]

        best_light_class = light_area[best_light_idx][0]

        if best_light_class == 'red' or best_light_class == 'yellow':   # STOP      
            self.traffic_light_publisher.publish(5)
        elif best_light_class == 'red_left_arrow':          # LEFT
            self.traffic_light_publisher.publish(6)
        elif best_light_class == 'green_left_arrow':        # GO and LEFT
            self.traffic_light_publisher.publish(7)
        else: # best_light_class == 'green':                   # GO
            self.traffic_light_publisher.publish(8)


    def yolo_post_process(self, data):
        self.current_time=time.time()

        self.data = data
            

        light_area = []
        sign_area = []
        labacon_area = []
        for i in range(len(data.detections)):
            class_id = self.class_labels[data.detections[i].results[0].id]
            if class_id in TRAFFIC_LABEL_LIST: # traffic_light
                light_area.append([i, data.detections[i].bbox.size_x * data.detections[i].bbox.size_y])  # x, y, size_x, size_y
            elif class_id in LABACON_LABEL_LIST:
                labacon_area.append([i, data.detections[i].bbox.size_x * data.detections[i].bbox.size_y])  # x, y, size_x, size_y
            elif class_id in SIGNAL_LABEL_LIST:
                sign_area.append(data.detections[i])         #traffic_sign

 

        if len(light_area) !=0:
            self.light_selection(light_area)

        if len(sign_area)!=0 and self.way_pt < WAY_POINT_THERSHOLD:
            self.sign_recognition(sign_area)

        if len(sign_area)!=0 and self.way_pt > WAY_POINT_THERSHOLD:
            self.sign_selection(sign_area)
        
         
        self.labacon_selection(labacon_area)
   

        rospy.loginfo(f"sign recognition counts : {len(self.sign_pos_dict[1]) ,len(self.sign_pos_dict[2]) , len(self.sign_pos_dict[3]) }")
        rospy.loginfo(f"sgin selection counts : {len(self.sign_pos_dict[1]) ,len(self.sign_pos_dict[2]) , len(self.sign_pos_dict[3]) }")
        rospy.loginfo(f"utrun stare : {self.uturnStartFlag} labacount : {self.labacon_cnt}")

        
    def sign_selection(self, sign_area):
        pos_list =[]
        
        for sign in sign_area:
            sign_id = sign.results[0].id
            y,x = caculate_local_postion(sign)

            for i in range(1,4):
                if self.class_labels[sign_id] == f'delivery_{i}':
                        
                        msg = Float32MultiArray()
                        msg.data =[10+i,-x,y ]
                        self.traffic_sign_location_publisher.publish(msg)
                        self.traffic_sign_publisher.publish(10+i)
                        pos_list.append((x,y,i))
                        self.update_pos_dict(i,(x,y))
            

        msg =MarkerArray()
        msg.markers = [ self.make_maker(pos[0],pos[1],pos[2]) for index,pos in enumerate(pos_list)]
        self.point_publisher.publish(msg)
        
        
        if (len(self.sign_pos_dict[1])>= POS_LIST_NUMVER and self.delivery_id==1) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[1][-1]
            msg.data =[11,-x,y ]
            self.traffic_sign_stop_location_publisher.publish(msg) 
            rospy.logwarn("Find Delivary 1")
        
        elif (len(self.sign_pos_dict[2])>= POS_LIST_NUMVER and self.delivery_id==2) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[2][-1]
            msg.data =[12,-x,y ]
            self.traffic_sign_stop_location_publisher.publish(msg) 
            rospy.logwarn("Find Delivary 2")

        elif (len(self.sign_pos_dict[3])>= POS_LIST_NUMVER and self.delivery_id==3) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[3][-1]
            msg.data =[13,-x,y ]
            self.traffic_sign_stop_location_publisher.publish(msg) 
            rospy.logwarn("Find Delivary 3")


    def sign_recognition(self, sign_area):
        pos_list =[]
        for sign in sign_area:
            sign_id = sign.results[0].id
            y,x = caculate_local_postion(sign)
            for i in range(1,4):
                if self.class_labels[sign_id] == f'delivery_{i}':           
                    msg = Float32MultiArray()
                    msg.data =[10+i,-x,y ]
                    self.traffic_sign_location_publisher.publish(msg)
                    self.traffic_sign_publisher.publish(10+i)
                    pos_list.append((x,y,i))
                    self.update_pos_dict(i,(x,y))
        
        if (len(self.sign_pos_dict[1])>= POS_LIST_NUMVER ) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[1][-1]
            msg.data =[11,-x,y ]
            self.traffic_sign_stop_location_publisher.publish(msg)
            self.delivery_id =1
            self.way_pt= 500
            
            rospy.logwarn("Find Pickup 1")
        
        elif (len(self.sign_pos_dict[2])>= POS_LIST_NUMVER) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[2][-1]
            msg.data =[12,-x,y ]
            self.delivery_id =2
            self.way_pt= 500
        

            self.traffic_sign_stop_location_publisher.publish(msg) 
            rospy.logwarn("Find Pickup 2")


        elif (len(self.sign_pos_dict[3])>= POS_LIST_NUMVER) :
            msg = Float32MultiArray()
            x ,y = self.sign_pos_dict[3][-1]
            msg.data =[13,-x,y ]
            self.delivery_id =3
            self.way_pt= 500
            self.traffic_sign_stop_location_publisher.publish(msg) 
            rospy.logwarn("Find Pickup 3")



    
        

    def labacon_selection(self, labacon_area):

        laba_cnt = 0
     
        # update labacon count
        for i in range(len(labacon_area)):
            labacon = self.data.detections[labacon_area[i][0]]
            labacon_id = labacon.results[0].id
            if self.class_labels[labacon_id] == 'labacon_1' or self.class_labels[labacon_id] == 'labacon_2' or self.class_labels[labacon_id] == 'labacon_3':
              if labacon_area[i][1] > 2000:
                  laba_cnt += 1
            
        
        if laba_cnt >= 3:
            self.labacon_cnt += 1
            
        # update non labacon stack
        if self.uturnStartFlag and len(labacon_area)<=5:
            self.nolabaConStack +=1
            if(self.nolabaConStack>=10):
                self.labacon_cnt =0
                self.uturnStartFlag=False

        elif self.uturnStartFlag and len(labacon_area)>5:
            self.nolabaConStack =0

      
        if self.labacon_cnt >= 5:
            self.uturnStartFlag=True
   
       
        self.traffic_labacon_publisher.publish(self.uturnStartFlag)
        return
       

    def light_selection(self, light_area):
        best_light_idx = np.argmax(np.array(light_area), axis = 0)[1]
        best_light_area = light_area[best_light_idx][1]
        best_light_idx = light_area[best_light_idx][0]
        best_light_detection = self.data.detections[best_light_idx]
        light_id = best_light_detection.results[0].id
        # rospy.loginfo(best_light_area)

        if self.prev_light_id == light_id:
            self.light_cnt += 1
        self.prev_light_id = light_id

        if self.light_cnt >= 5 and best_light_area > 750:
            best_light_class = self.class_labels[light_id]

            if best_light_class == 'red' or best_light_class == 'yellow':   # STOP      
                self.traffic_light_publisher.publish(5)
            elif best_light_class == 'red_left_arrow':          # LEFT
                self.traffic_light_publisher.publish(6)
            elif best_light_class == 'green_left_arrow':        # GO and LEFT
                self.traffic_light_publisher.publish(7)
            else:                   # GO
                self.traffic_light_publisher.publish(8)
            self.light_cnt = 0
  


if __name__ == "__main__":
    rospy.init_node("vision_post_process",log_level=rospy.DEBUG)
    ns = rospy.get_name() + "/"
        
    yolov7_detection = "/yolov7/yolov7_detect"
    way_point = "/current_waypoint"
    yolov7_result_sign = "traffic_sign"
    yolov7_result_labacon = "traffic_labacon"
    yolov7_result_light = "traffic_light"
    yolov7_result_sign_location ="traffic_sign_location"
    yolov7_result_sign_stop_location ="traffic_sign_stop_location"
    yolo_traffic_topic ="/yolov7/yolov7_detect_traffic"
    way_point = "/current_waypoint"
    classes_path = rospy.get_param(ns + "classes_path") 
    classes=parse_classes_file(classes_path)    
    img_size = 640
    rospy.logdebug('processer strat!!')


    publisher = YoloPostProcess(
        yolo_topic=yolov7_detection,
        yolo_traffic_topic=yolo_traffic_topic,
        way_point_topic=way_point,
        pub_topic_1=yolov7_result_sign,
        pub_topic_2=yolov7_result_labacon,
        pub_topic_3=yolov7_result_light,
        pub_topic_4 =yolov7_result_sign_location,
        pub_topic_5 =yolov7_result_sign_stop_location,
        img_size=(img_size, img_size),
        class_labels=classes
    )

    rospy.spin()


