#include "sonar_slam/sonar_to_laserscan.h"
#include <algorithm>
#include <cmath>

namespace sonar_slam {

SonarToLaserScan::SonarToLaserScan(ros::NodeHandle& nh) : nh_(nh)
{
    loadParameters();
}

void SonarToLaserScan::loadParameters()
{
    // 修复：所有默认值使用float类型，避免double/float类型冲突
    nh_.param("laser_angle_min", laser_angle_min_, -static_cast<float>(M_PI)/2.0f);
    nh_.param("laser_angle_max", laser_angle_max_, static_cast<float>(M_PI)/2.0f);
    nh_.param("laser_angle_increment", laser_angle_increment_, static_cast<float>(M_PI)/180.0f);
    nh_.param("laser_time_increment", laser_time_increment_, 0.0f);
    nh_.param("laser_scan_time", laser_scan_time_, 0.1f);
    nh_.param("laser_range_min", laser_range_min_, 0.2f);
    nh_.param("laser_range_max", laser_range_max_, 15.0f);
    
    // 声纳参数
    nh_.param("sonar_fov", sonar_fov_, static_cast<float>(M_PI));  // 使用float类型
    nh_.param("sonar_resolution", sonar_resolution_, 2.0f * static_cast<float>(M_PI) / 180.0f);
    nh_.param("sonar_beams", sonar_beams_, 200);
    
    // 滤波参数
    nh_.param("enable_filter", enable_filter_, true);
    nh_.param("median_filter_size", median_filter_size_, 3.0f);
    nh_.param("range_filter_max", range_filter_max_, laser_range_max_);
    nh_.param("range_filter_min", range_filter_min_, laser_range_min_);
    
    // 强度参数
    nh_.param("min_intensity", min_intensity_, 100.0f);
    nh_.param("max_intensity", max_intensity_, 255.0f);
    
    ROS_INFO("SonarToLaserScan parameters loaded successfully");
}

sensor_msgs::LaserScan SonarToLaserScan::convert(const mars_msgs::Pose2DArray& sonar_points)
{
    sensor_msgs::LaserScan laser_scan;
    
    // 修复：使用固定frame_id，因为Pose2DArray可能没有header
    laser_scan.header.stamp = ros::Time::now();
    laser_scan.header.frame_id = "sonar_link";  // 使用固定frame_id
    
    laser_scan.angle_min = laser_angle_min_;
    laser_scan.angle_max = laser_angle_max_;
    laser_scan.angle_increment = laser_angle_increment_;
    laser_scan.time_increment = laser_time_increment_;
    laser_scan.scan_time = laser_scan_time_;
    laser_scan.range_min = laser_range_min_;
    laser_scan.range_max = laser_range_max_;
    
    // 计算扫描线数量
    int num_readings = static_cast<int>((laser_scan.angle_max - laser_scan.angle_min) / 
                      laser_scan.angle_increment) + 1;
    
    // 初始化距离和强度数组
    std::vector<float> ranges(num_readings, std::numeric_limits<float>::infinity());
    std::vector<float> intensities(num_readings, 0.0f);
    
    // 处理每个声纳点
    for (const auto& point : sonar_points.poses)
    {
        processSonarPoint(point, ranges, intensities);
    }
    
    // 应用滤波
    if (enable_filter_)
    {
        applyFilter(ranges);
    }
    
    // 设置结果
    laser_scan.ranges = ranges;
    laser_scan.intensities = intensities;
    
    return laser_scan;
}

void SonarToLaserScan::processSonarPoint(const geometry_msgs::Pose2D& point,
                                        std::vector<float>& ranges,
                                        std::vector<float>& intensities)
{
    // 计算距离和角度
    float range = sqrt(point.x * point.x + point.y * point.y);
    float angle = atan2(point.y, point.x);
    
    // 角度范围检查
    if (angle < laser_angle_min_ || angle > laser_angle_max_)
        return;
    
    // 距离范围检查
    if (range < laser_range_min_ || range > laser_range_max_)
        return;
    
    // 强度检查 - 使用point.theta作为强度
    float intensity = point.theta;
    if (intensity < min_intensity_ || intensity > max_intensity_)
        return;
    
    // 强度归一化到0-1范围
    float normalized_intensity = (intensity - min_intensity_) / (max_intensity_ - min_intensity_);
    normalized_intensity = std::max(0.0f, std::min(1.0f, normalized_intensity));
    
    // 找到最近的激光扫描角度索引
    int nearest_index = static_cast<int>((angle - laser_angle_min_) / 
                                        laser_angle_increment_ + 0.5f);
    
    if (nearest_index >= 0 && nearest_index < static_cast<int>(ranges.size()))
    {
        // 取最小距离（最近的点）
        if (range < ranges[nearest_index])
        {
            ranges[nearest_index] = range;
            intensities[nearest_index] = normalized_intensity;
        }
    }
}

void SonarToLaserScan::applyFilter(std::vector<float>& ranges)
{
    std::vector<float> filtered = ranges;
    int filter_size = static_cast<int>(median_filter_size_);
    
    // 确保滤波器大小为奇数
    if (filter_size % 2 == 0) filter_size++;
    if (filter_size < 1) filter_size = 1;
    
    for (size_t i = 0; i < ranges.size(); i++)
    {
        // 中值滤波窗口
        std::vector<float> window;
        int half_size = filter_size / 2;
        
        for (int j = -half_size; j <= half_size; j++)
        {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(ranges.size()) && 
                !std::isinf(ranges[idx]))
            {
                window.push_back(ranges[idx]);
            }
        }
        
        if (window.size() > 0)
        {
            std::sort(window.begin(), window.end());
            filtered[i] = window[window.size() / 2];
        }
        
        // 范围滤波
        if (filtered[i] > range_filter_max_ || filtered[i] < range_filter_min_)
        {
            filtered[i] = std::numeric_limits<float>::infinity();
        }
    }
    
    ranges = filtered;
}

std::vector<sensor_msgs::LaserScan> SonarToLaserScan::convertBatch(
    const std::vector<mars_msgs::Pose2DArray>& sonar_batch)
{
    std::vector<sensor_msgs::LaserScan> laser_scans;
    
    for (const auto& sonar_points : sonar_batch)
    {
        laser_scans.push_back(convert(sonar_points));
    }
    
    return laser_scans;
}

void SonarToLaserScan::getLaserParams(float& angle_min, float& angle_max,
                                     float& angle_increment, float& range_max) const
{
    angle_min = laser_angle_min_;
    angle_max = laser_angle_max_;
    angle_increment = laser_angle_increment_;
    range_max = laser_range_max_;
}

} // namespace sonar_slam