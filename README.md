# AutonomusRacing2024

For growth of technical issue


# Overview

![Car Image](img/car.png)
##  1. System Architecrue
- [1.1 Hardware](-hardware)
  -   [1.1.1 computer](-computer)
  -   [1.1.2 lidar](-lidar)
  -   [1.1.3 camera](-camera)
  -  [1.1.4 gps](-gps)
  - [1.1.5 erp](-erp)

- [1.2 Software](-software)
  - [1.2.1 OS](-os)
  - [1.2.2 ROS](-total)
  - [1.2.3 catkine workspace](#catkin-worksapce)
  - [1.2.4 control package](#control-package)
  - [1.2.5 lidar package](#lidar-package)
  - [1.2.6 vision package](#vision-package)
  - [1.2.7 multi vision package](#vision-package)


## 2. How to Use
- [2.1 How to install package](-install)
- [2.2 How to Build](-build)
- [2.3 How to Run](#-Run)
  - [final ](final)
  - [pre ](pre)
  - [labacon](labacon)
  - [util](rviz)
- [2.4 How to develope](#-develope)
- [2.5 How to downlaod data](#-donwlaod-data)
    - [rosbag](rosbag)
- [2.6 How to Test](#-Run)
## 3. Results

- [final results](final)
- [pre results](pre)
- [rviz examples](pacakge)



## 4. Contributors


# 1.1 Hardware

## 1.1.1 Computer
we use two comptuer
## 1.1.2 Lidar
we use velodyne-16
## 1.2.3 Camera
we use usbcam and firm carmera
## 1.2.4 Gps
we use GPS
## 1.2.5 Erp

## 1.2.6 Imu

# 1.2 Software

## 1.2.1 OS
we use OS Ubuntu 20.04 both computer

## 1.2.2 ROS
we use ros noetic and multicore
## 1.2.3  catkin workspace
this is our workspace tree

```
base_ws
└── src
    ├── control
    │   └── README.md
    ├── lidar
    │   ├── Light_signal
    │   ├── README.md
    │   ├── aonehorn_detection
    │   ├── cmake
    │   ├── common
    │   ├── custom_msg
    │   ├── erp_driver
    │   ├── examples
    │   ├── feature_extractors
    │   ├── object_builders_lib
    │   ├── roi_filters
    │   ├── rviz_car_model
    │   ├── segmenters_lib
    │   ├── tracking_lib
    │   ├── tracking_msg
    │   ├── vehicle_msgs
    │   └── velodyne
    └── vision
        ├── README.md
        ├── cits_logger
        ├── usb_cam
        └── yolov7_ros

```

## 1.2.4 Lidar package

<!--lidar packae에 대한 설명을 담아주세요! -->

## 1.2.5 Vision package

<!--vision packae에 대한 설명을 담아주세요! -->

###  spinnaker_camera_driver

spinnaker 의카메라 센서를 키고 조작하기 위한 패키지입니다.

```bash

#spinnaker camerar를 킵니다.

roslaunch spinnaker_camera_driver camera.launch

```

### yolov7_ros


yolov7_ros는 yolo를 사용하며 신호등인식과 표지판인식을 수행하여, 신호등의 값과 표지판의 좌표를 제어프로그램에 전달하기 위한 패키지입니다.

```bash

#yolo와 토픽 발행 모든것을 실행시킵니다.

roslaunch yolov7_ros vision_all

```

## 1.2.6 Control package

<!--control  packae에 대한 설명을 담아주세요! -->

###  catkin_virtualenv

imu 패키지(microstrain_inertial)을 실행을 하기 위해 필요한 패키지입니다.

### erp_driver

erp 플랫폼의 speed, steer, brake, mode 등 정보를 받기위한 패키지입니다.

```bash

#erp usb 권한 설정 ( usb를 꼽는 순서에 따라 변경 필요 )

cd
cd /dev
usb1
# 비밀번호 입력

#erp 관련 데이터 토픽 발생 및 실행

roslaunch erp_driver erp42_base.launch

```

### microstrain_inertial

imu 데이터 정보를 받기위한 패키지입니다.

```bash

#imu 데이터의 토픽 발행을 실행시킵니다.

roslaunch microstrain_inertial_driver microstrain.launch 

```

### nmea_navsat_driver

gps 데이터 정보를 받기위한 패키지입니다.

```bash

#gps usb 권한 설정 ( usb를 꼽는 순서에 따라 변경 필요 )
cd
cd /dev
usb1
# 비밀번호 입력

#gps 센서 초기 설정 1
rosrun nmea_navsat_driver nmea_topic_serial_reader _port:=/dev/ttyUSB0 _baud:=115200

#gps 센서 초기 설정 2
rosrun nmea_navsat_driver nmea_topic_driver

```


### gpsimu

센서 데이터들을 이용하여 작성된 제어 코드를 실행시키기 위한 패키지입니다.

```bash

#utm 좌표 및 imu heading 토픽을 발행시킵니다.

cd ~/control/src/gpsimu/scripts/
python3 gpsimuparser.py

#보정된 heading 토픽을 발행시킵니다.

cd ~/control/src/gpsimu/scripts/
python3 heading_calculator.py

```

# 2. How to use

## 2.1 How to install package

## 2.2 How to  Build

```

git clone https://github.com/Ajou-Nice/AutonomusRacing2024
cd AutonomusRacing2024
catkin_make


```
## 2.3 How to develope

If you  want add more functions or discriptions following below process. we will follow this [example](https://velog.io/@diduya/git-%ED%9A%A8%EC%9C%A8%EC%A0%81%EC%9D%B8-%ED%98%91%EC%97%85%EC%9D%84-%EC%9C%84%ED%95%9C-Git-Flow-%EC%9D%B4%ED%95%B4%ED%95%98%EA%B8%B0-git-branch-repository)

```bash

git clone https://github.com/Ajou-Nice/AutonomusRacing2024
cd AutonomusRacing2024

# develope is our main develope branch
git pull origin develope

# move branch to develope
git checkout develope

# make your branch
git branch yourbranch

# move your branch
git checkout yourbranch

# revise anything or  add package or someting but don't add large file

git add yourfiles

# commit  your revises
git commit -m "your message"

# push remote store your commits and pull request in the github homepage
git push your branch
```



## 2.4 How to  Run

### Pre

```bash

##ros

##control

##lidar

##vision

roslaunch example example.launch

```

### Final
```bash

##ros

##control

##lidar

##vision

## vision
roslaunch usb_cam camera.launch
roslaunch sipinnaker_camera_driver camera.launch
roslaunch yolov7_ros vison_all_v11.launch
roslaunch cits_logger cits.launch

```

### labacone
```bash

roslaunch example example.launch

```

##  2.4 How to downlaod data
##  2.5 How to test
# 3. Results

# 4. Contributors
