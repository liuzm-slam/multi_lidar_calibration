/*
 * @Author: Ziming.Liu 
 * @Date: 2021-06-Fr 03:29:18 
 * @Last Modified by:   Ziming.Liu 
 * @Last Modified time: 2021-06-Fr 03:29:18 
 */

#ifndef MULTI_LIDAR_CALIBRATION_H_
#define MULTI_LIDAR_CALIBRATION_H_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class MultiLidarCalibration
{
public:
    MultiLidarCalibration(ros::NodeHandle &n);
    ~MultiLidarCalibration();

    // 函数处理
    void Run();

private:
    // 订阅的激光话题
    std::string source_lidar_topic_str_;
    std::string target_lidar_topic_str_;

    // 激光雷达坐标系
    std::string source_lidar_frame_str_;
    std::string target_lidar_frame_str_;

    // icp匹配得分
    float icp_score_;

    //  在base_link坐标系下main_laser_link的坐标
    float main_to_base_transform_x_;
    float main_to_base_transform_y_;
    float main_to_base_transform_row_;
    float main_to_base_transform_yaw_;

    // 第一次运行时进行矩阵赋值
    bool is_first_run_;

    // 两个激光间的transfrom，通过tf2获得
    Eigen::Matrix4f transform_martix_;
    // 主激光到base_link的TF
    Eigen::Matrix4f front_to_base_link_;

    ros::NodeHandle nh_;
    // 纠正激光输出，类型pointcloud2
    ros::Publisher final_point_cloud_pub_;
    // 进行激光时间同步和数据缓存
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicyT;
    message_filters::Subscriber<sensor_msgs::LaserScan> *scan_front_subscriber_, *scan_back_subscriber_;
    message_filters::Synchronizer<SyncPolicyT> *scan_synchronizer_;

    // pcl格式的激光数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr main_scan_pointcloud_;
    // 标定雷达数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_scan_pointcloud_;
    // 通过tf2转换后的待标定的激光数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_scan_pointcloud_init_transformed_;

    /**
     * @brief icp 
     * @param final_registration_scan_ 通过icp处理把待标定数据，转换到目标坐标系下的激光点云数据
     */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_registration_scan_;

    // 两个激光坐标系间初始坐标变换
    void GetFrontLasertoBackLaserTf();

    // 订阅main雷达和sub雷达两个激光数据
    void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &in_main_scan_msg, const sensor_msgs::LaserScan::ConstPtr &in_sub_scan_msg);

    // 极坐标转换到笛卡尔坐标，sensor_msgs::LaserScan to pcl::PointCloud<pcl::PointXYZ> (方法很多不限于这一种)
    pcl::PointCloud<pcl::PointXYZ> ConvertScantoPointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

    // 标定后的激光点云进行发布，发布的数据格式是sensor_msgs::PointCloud2
    void PublishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);

    // 对main雷达和sub雷达的激光点云进行icp匹配
    bool ScanRegistration();

    // 发布标定结果
    void PrintResult();

    // 可视化
    void View();
};

#endif //MULTI_LIDAR_CALIBRATION_H_
