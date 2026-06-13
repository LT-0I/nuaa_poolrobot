#include "sonar_slam/sonar_data_processor.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <mars_msgs/Pose2DArray.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

namespace sonar_slam {

// 2300命令数据结构
struct MsgCmd2300 {
    unsigned char mode;
    unsigned char gain;
    unsigned short angle;
    unsigned short trans_duration;
    unsigned short samplepriead;
    unsigned short frequency;
    unsigned short numbersample;
    unsigned short length;
    unsigned char data[1200];
};

// 声纳数据包结构
struct SonarPacket {
    unsigned char start[4];
    unsigned char msg_time[28];
    struct {
        unsigned char start[2];
        unsigned short len;
        unsigned short cmd;
        unsigned char src;
        unsigned char dst;
        unsigned char data[1300];
        unsigned short checksum;
    } msg;
};

SonarDataProcessor::SonarDataProcessor(ros::NodeHandle& nh) : nh_(nh)
{
    // 读取参数
    std::string data_file;
    nh_.param("data_file", data_file, std::string("test2.bin"));
    nh_.param("max_range", max_range_, 15.0f);
    nh_.param("min_range", min_range_, 0.2f);
    nh_.param("intensity_threshold", intensity_threshold_, 100);
    
    // 发布器
    sonar_points_pub_ = nh_.advertise<mars_msgs::Pose2DArray>("/sonar_points", 10);
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/sonar_scan", 10);
    
    // TF广播
    tf_broadcaster_ = new tf::TransformBroadcaster();
    
    // 读取数据文件
    if (loadSonarData(data_file))
    {
        ROS_INFO("Successfully loaded sonar data file: %s", data_file.c_str());
        ROS_INFO("Total scans: %lu", sonar_scans_.size());
    }
    else
    {
        ROS_ERROR("Failed to load sonar data file: %s", data_file.c_str());
    }
    
    // 定时发布数据
    publish_timer_ = nh_.createTimer(ros::Duration(0.1f),
                                    &SonarDataProcessor::publishCallback, 
                                    this);
    
    // 初始化激光扫描消息
    initLaserScan();
}

SonarDataProcessor::~SonarDataProcessor()
{
    if (tf_broadcaster_) {
        delete tf_broadcaster_;
        tf_broadcaster_ = nullptr;
    }
}

bool SonarDataProcessor::loadSonarData(const std::string& filename)
{
    FILE* fd = fopen(filename.c_str(), "rb");
    if (!fd)
    {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return false;
    }
    
    // 获取文件大小
    fseek(fd, 0, SEEK_END);
    long filelength = ftell(fd);
    fseek(fd, 0, SEEK_SET);
    
    unsigned int loopcnt = 0;
    long i = 0;
    
    while (i < filelength)
    {
        loopcnt++;
        
        // 寻找数据包开始标记 'B'
        unsigned char buf[64];
        int tmpl = 0;
        size_t readlen;
        
        for (tmpl = 0; tmpl < 64; tmpl++)
        {
            readlen = fread(buf + tmpl, 1, 1, fd);
            if (readlen == 0) break;
            if (buf[tmpl] == 'B') break;
            i += readlen;
        }
        
        if (tmpl >= 64) break;
        
        SonarPacket packet;
        
        // 读取时间戳
        int k = tmpl - 1;
        int j = 27;
        for (; j >= 0; j--)
        {
            packet.msg_time[j] = buf[k];
            k--;
            if (k < 0) break;
        }
        
        fseek(fd, -1, SEEK_CUR);
        
        // 读取消息头
        readlen = fread((unsigned char*)&packet.msg, 1, 8, fd);
        i += readlen;
        
        if (readlen != 8 || packet.msg.start[0] != 'B')
        {
            ROS_WARN("Invalid packet header at loop %d", loopcnt);
            continue;
        }
        
        // 读取消息数据
        readlen = fread(packet.msg.data, 1, packet.msg.len, fd);
        i += readlen;
        
        if (readlen != packet.msg.len)
        {
            ROS_WARN("Incomplete packet data at loop %d", loopcnt);
            break;
        }
        
        // 读取校验和
        readlen = fread((unsigned char*)&packet.msg.checksum, 1, 2, fd);
        i += readlen;
        
        // 处理2300命令的数据（声纳扫描数据）
        if (packet.msg.cmd == 2300 && loopcnt > 100)
        {
            MsgCmd2300* cmd = (MsgCmd2300*)packet.msg.data;
            
            // 创建声纳点
            SonarPoint spoint;
            spoint.angle = (cmd->angle * 0.9f) * M_PI / 180.0f;
            
            // 计算距离步长
            float time_per_sample = cmd->samplepriead * 25.0f / 1000.0f;
            float distance_per_sample = time_per_sample * 1.45f / 1000.0f;
            
            // 提取有效数据点
            for (int j = 0; j < cmd->length; j++)
            {
                float distance = distance_per_sample * (j + 1);
                unsigned char intensity = cmd->data[j];
                
                if (distance >= min_range_ && distance <= max_range_ && 
                    intensity > static_cast<unsigned char>(intensity_threshold_))
                {
                    spoint.distances.push_back(distance);
                    spoint.intensities.push_back(intensity);
                }
            }
            
            if (!spoint.distances.empty())
            {
                spoint.timestamp = ros::Time::now();
                sonar_scans_.push_back(spoint);
            }
        }
        
        if (i >= filelength - 1) break;
    }
    
    fclose(fd);
    ROS_INFO("Loaded %lu valid sonar scans", sonar_scans_.size());
    return !sonar_scans_.empty();
}

void SonarDataProcessor::initLaserScan()
{
    laser_scan_.header.frame_id = "sonar_link";
    laser_scan_.angle_min = -static_cast<float>(M_PI) / 2.0f;
    laser_scan_.angle_max = static_cast<float>(M_PI) / 2.0f;
    laser_scan_.angle_increment = 2.0f * static_cast<float>(M_PI) / 180.0f;
    laser_scan_.time_increment = 0.0f;
    laser_scan_.scan_time = 0.1f;
    laser_scan_.range_min = min_range_;
    laser_scan_.range_max = max_range_;
    
    int num_readings = static_cast<int>((laser_scan_.angle_max - laser_scan_.angle_min) / 
                          laser_scan_.angle_increment) + 1;
    laser_scan_.ranges.resize(num_readings, laser_scan_.range_max + 1.0f);
    laser_scan_.intensities.resize(num_readings, 0.0f);
}

void SonarDataProcessor::publishCallback(const ros::TimerEvent& e)
{
    static size_t current_scan_index = 0;
    
    if (current_scan_index >= sonar_scans_.size())
    {
        current_scan_index = 0;
        ROS_INFO("Replaying sonar data...");
    }
    
    publishSonarPoints(current_scan_index);
    publishLaserScan(current_scan_index);
    publishTF();
    
    current_scan_index++;
}

void SonarDataProcessor::publishSonarPoints(size_t index)
{
    if (index >= sonar_scans_.size()) return;
    
    const SonarPoint& scan = sonar_scans_[index];
    mars_msgs::Pose2DArray points_msg;
    
    // 将极坐标转换为笛卡尔坐标
    for (size_t i = 0; i < scan.distances.size(); i++)
    {
        float r = scan.distances[i];
        float theta = scan.angle;
        
        geometry_msgs::Pose2D point;
        point.x = r * cosf(theta);
        point.y = r * sinf(theta);
        point.theta = scan.angle;
        
        points_msg.poses.push_back(point);
    }
    
    // mars_msgs::Pose2DArray 可能没有 header，使用独立的 header 或修改消息定义
    // 目前直接发布数据
    
    sonar_points_pub_.publish(points_msg);
}

void SonarDataProcessor::publishLaserScan(size_t index)
{
    if (index >= sonar_scans_.size()) return;
    
    const SonarPoint& scan = sonar_scans_[index];
    
    std::fill(laser_scan_.ranges.begin(), laser_scan_.ranges.end(), 
             laser_scan_.range_max + 1.0f);
    std::fill(laser_scan_.intensities.begin(), laser_scan_.intensities.end(), 0.0f);
    
    float scan_angle = scan.angle;
    
    int nearest_index = static_cast<int>((scan_angle - laser_scan_.angle_min) / 
                                        laser_scan_.angle_increment + 0.5f);
    
    if (nearest_index >= 0 && nearest_index < static_cast<int>(laser_scan_.ranges.size()))
    {
        float min_distance = laser_scan_.range_max + 1.0f;
        float max_intensity = 0.0f;
        
        for (size_t i = 0; i < scan.distances.size(); i++)
        {
            if (scan.distances[i] < min_distance)
            {
                min_distance = scan.distances[i];
                max_intensity = static_cast<float>(scan.intensities[i]);
            }
        }
        
        if (min_distance <= laser_scan_.range_max)
        {
            laser_scan_.ranges[nearest_index] = min_distance;
            laser_scan_.intensities[nearest_index] = max_intensity;
        }
    }
    
    laser_scan_.header.stamp = scan.timestamp;
    laser_pub_.publish(laser_scan_);
}

void SonarDataProcessor::publishTF()
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "map", "sonar_link"));
}

bool SonarDataProcessor::parseSonarPacket(const unsigned char* data, size_t length, SonarPoint& point)
{
    return false;
}

} // namespace sonar_slam


// sonar_slam/src/sonar_data_processor.cpp 末尾添加：
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar_data_processor");
    ros::NodeHandle nh("~");
    
    sonar_slam::SonarDataProcessor processor(nh);
    
    ROS_INFO("Sonar Data Processor started");
    ros::spin();
    
    return 0;
}