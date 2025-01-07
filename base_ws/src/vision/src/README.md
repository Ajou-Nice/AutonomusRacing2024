# All About Vision


## Data anotation 


robofloW를 사용하여 Data annotation을 진행합니다. 

자세한 사항은 roboflow 홈페이지 docs를 참조하며 진행하며 수월하게 할 수 있습니다.

annotation을 진행한후 원하는 yolo 모델에 맞게 데이터 형식을 다운로드 할 수 있습니다.

다운로드 한 데이터는 datasets에 이름을 변경한 후 저장하면 됩니다.


## Yolo Train

`ulastic library`에서 yolov11을 쉽게 학습할 수 있도록 API를 제공해줍니다.

예시의  `yolo11_train`를 사용해서 학습 할 수 있습니다. 

학습한 모델은 yolo11 폴더안에 저장됩니다.


## Detection 

학습한 모델을 ros에 결합하며 토픽을 발행할 수 있습니다. 

이는 `detect_ros`에서 모든 것이 이루어집니다. 

카메라의 이미지 토픽을 받고 이미지 토픽을 YOLO를 통해 box화 시켜 원하는 물체의 개수 

박스 사이즈, 박스 위치등을 전처리할 수 있도록 만들어 줍니다.


## Post process

`post_process`는 `detect_ros`에서 발행한 토픽을 전처리 하는 코드입니다. 

여기서는 박스의 위치, 물체의 크기를 통해서 해당 물체의 위치를 파악하는 등의 역할을 합니다.

또는 제어팀과의 토픽 규약들을 지켜 알맞는 형식으로 바꾸어 주는 역할 또한 진행합니다.

각 미션에 맞게 필요한 형식으로 변경해주는 코드라고 보면 됩니다.

2024 배달미션에서는 이 코드에서 배달표지판의 위치를 카메라의 초점거리와 실제 배달 

표지판의 크기를 사용해서 실제 표지판의 위치를 특정하도록 하였습니다. 

이떄 카메라의 초점거리를 바꾸게 되면 표지판의 실제 위치가 예측한 위치와 달라지므로 

카메라 칼리브레이션을 끝내고 초점거리를 구하면 카메라의 `초점은 건드리지 않는 것`이 좋습니다. 

여기에 대한 코드는 `calculate_distance` 파일을 참조하면 됩니다.  


## Camera


실제로 객체인식을 하기 위해서는 카메라를 켜야 합니다.

2024 본선 대회를 위해서는 카메라 두 가지가 필요했습니다.

중앙 카메라는 `spinnaker_camerea` 이고 위를 바라보는 카메라는 `usb_cam` 입니다.

가까이서 신호등을 바라보기 위해서는 카메라 usb_cam이 필요합니다. 

이 두 카메라를 키는 명령어는 다음과 같이 키면 됩니다.

```
#f1 으로 치면 동일하게 명령어가 수행됩니다.
roslaunch spinnaker_camera_driver camera.launch

#f2 으로 치면 동일하게 명령어가 수행됩니다.
roslaunch usb_cam usb_cam.launch

```

카메라 두 개를 동작시키기 떄문에 `detect_ros` 노드를 각각 카메라에 맞게 두 개의 같은 코드가 동시에 돌아감을 주의하여 합니다.


## Build & Run

카메라를 킨 상태에서 다음과 같은 명령어를 치면 Detection 기능이 수행됩니다.

```
cd vision

source ~/AutonomusRacing2024/base_ws/src/vision/devel/setup.bash

roslaunch yolov7_ros vision_all.launch

```

## TEST

비전파트는 이미지 파일만으로 기능 수행이 가능하기 때문에 ros bag 파일을 실행하며 해당 기능이 잘 작동하는지 테스트 할 수 있습니다.

테스트 하기에 적합한 rosbag 파일들은 다음과 같은 명령어를 통해 쉽게 수행 할 수 있습니다.

```

#rosbag 파일이 있는 곳으로 진입합니다.
cd /media/ajoucar/_hd/home/ajou/bag2024/0811

#rosbag 파일을 계속 반복합니다.
rosbag play -l 2024_delivery_destination1.bag 

#rviz로 작동상황을 확인합니다.

rivz

#이후 rviz창의 file > Recent configs > lidar_final_rviz.rviz 
#를 클릭하면 라이더와 usb_cam spinnacker 카메라를 동시에 시각화 할수 있습니다.
```


## Utils

밤에 연습하거나 새벽에 연습할때 카메라가 어두워 밝기 조절을 해야할때가 있습니다. 

```
#카메라를 킵니다.
roslaunch spinnaker_camera_driver camera.launch


#rqt reconfigure창을 엽니다
rosrun rqt_reconfigure rqt_reconfigure

#spinnaker 항목을 클릭하고 exposure time을 조절합니다.

```
