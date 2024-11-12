#include <pcl/console/print.h>
#include "aonehorn_detection/cone_detection.hpp"

std::tuple<bool, float, float> detectCone(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    // �α� ������ ERROR�� �����Ͽ� ��� �޽����� ����
    //pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    Eigen::Vector3f axis(0.0, 0.0, 1.0);  // z ����
    seg.setAxis(axis);
    seg.setEpsAngle(50.0f * M_PI / 180.0f);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CONE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(2000);  // �ִ� �ݺ� Ƚ�� ����
    seg.setDistanceThreshold(1);  // �Ÿ� ��� ���� ����
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    pcl::PointIndices::Ptr inliers_cone(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cone(new pcl::ModelCoefficients);
    seg.segment(*inliers_cone, *coefficients_cone);

    if (inliers_cone->indices.size() > 0) {
        float apex_angle = coefficients_cone->values[6];
        float height = coefficients_cone->values[2];

        float angle_deg = apex_angle * 180.0 / M_PI;
        //if (angle_deg >= 5.0 && angle_deg <= 70.0 && height <= 2) {
        //if (coefficients_cone->values[5] > 0) {  // dz ���� ������� Ȯ��
        return std::make_tuple(true, coefficients_cone->values[0], coefficients_cone->values[1]);
        //}
        //}
    }
    else {
        return std::make_tuple(false, 0, 0);
    }
    
}
