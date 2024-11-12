#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>
#include "common/types/type.h"  

// BEV ���� �����ϰ� OccupancyGrid �޽����� �ۺ����ϴ� �Լ�
void fillBEVMap(autosense::PointICloudPtr point_obj,
    ros::Publisher& map_pub,  // �ۺ��Ÿ� �Լ��� ����
    float resolution, float left_range, float right_range,
    float front_range, float rear_range, float dilu);

