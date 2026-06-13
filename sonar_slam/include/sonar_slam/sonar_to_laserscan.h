#ifndef SONAR_TO_LASERSCAN_H
#define SONAR_TO_LASERSCAN_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>  // 使用基础类型而不是具体消息
#include <tf/transform_listener.h>
#include <vector>
#include <string>

// 前向声明，避免编译时依赖问题
#include <mars_msgs/Pose2DArray.h>

namespace sonar_slam {

class SonarToLaserScan
{
public:
    SonarToLaserScan(ros::NodeHandle& nh);
    virtual ~SonarToLaserScan() = default;
    
    // 从声纳点云转换为激光扫描
    sensor_msgs::LaserScan convert(const mars_msgs::Pose2DArray& sonar_points);
    
    // 批量转换
    std::vector<sensor_msgs::LaserScan> convertBatch(
        const std::vector<mars_msgs::Pose2DArray>& sonar_batch);
    
    // 获取激光扫描参数
    void getLaserParams(float& angle_min, float& angle_max, 
                       float& angle_increment, float& range_max) const;
    
private:
    // 配置参数
    void loadParameters();
    
    // 处理单个声纳点
    void processSonarPoint(const geometry_msgs::Pose2D& point, 
                          std::vector<float>& ranges,
                          std::vector<float>& intensities);
    
    // 滤波处理
    void applyFilter(std::vector<float>& ranges);
    
private:
    ros::NodeHandle& nh_;
    
    // 激光扫描参数
    float laser_angle_min_;
    float laser_angle_max_;
    float laser_angle_increment_;
    float laser_time_increment_;
    float laser_scan_time_;
    float laser_range_min_;
    float laser_range_max_;
    
    // 声纳参数
    float sonar_fov_;              // 声纳视场角（弧度）
    float sonar_resolution_;       // 角度分辨率
    int sonar_beams_;              // 波束数量
    
    // 滤波参数
    bool enable_filter_;
    float median_filter_size_;
    float range_filter_max_;
    float range_filter_min_;
    
    // 转换参数
    float min_intensity_;          // 最小强度阈值
    float max_intensity_;          // 最大强度阈值
};

} // namespace sonar_slam

#endif // SONAR_TO_LASERSCAN_H