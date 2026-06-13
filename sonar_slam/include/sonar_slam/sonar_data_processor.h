#ifndef SONAR_DATA_PROCESSOR_H
#define SONAR_DATA_PROCESSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mars_msgs/Pose2DArray.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>

namespace sonar_slam {

// 声纳点数据结构
struct SonarPoint
{
    float angle;                    // 角度（弧度）
    std::vector<float> distances;   // 距离数组
    std::vector<unsigned char> intensities; // 强度数组
    ros::Time timestamp;
    
    SonarPoint() : angle(0.0) {}
    
    bool empty() const { return distances.empty(); }
    size_t size() const { return distances.size(); }
};

class SonarDataProcessor
{
public:
    SonarDataProcessor(ros::NodeHandle& nh);  // 构造函数声明
    ~SonarDataProcessor();                     // 析构函数声明
    
    // 加载声纳数据文件
    bool loadSonarData(const std::string& filename);
    
    // 发布数据
    void publishSonarPoints(size_t index);
    void publishLaserScan(size_t index);
    void publishTF();
    
    // 获取声纳扫描数量
    size_t getScanCount() const { return sonar_scans_.size(); }
    
private:
    // 初始化激光扫描消息
    void initLaserScan();
    
    // 从原始数据包解析声纳数据
    bool parseSonarPacket(const unsigned char* data, size_t length, SonarPoint& point);
    
    // 回调函数
    void publishCallback(const ros::TimerEvent& e);
    
private:
    ros::NodeHandle& nh_;
    ros::Publisher sonar_points_pub_;
    ros::Publisher laser_pub_;
    ros::Timer publish_timer_;
    tf::TransformBroadcaster* tf_broadcaster_;
    
    std::vector<SonarPoint> sonar_scans_;
    sensor_msgs::LaserScan laser_scan_;
    
    // 参数
    float max_range_;
    float min_range_;
    int intensity_threshold_;
};

} // namespace sonar_slam

#endif // SONAR_DATA_PROCESSOR_H