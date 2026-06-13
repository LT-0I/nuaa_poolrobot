#ifndef SONAR_MAP_BUILDER_H
#define SONAR_MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <mars_msgs/Pose2DArray.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <cmath>
#include <set>

namespace sonar_slam {

#pragma pack(push, 1)
struct MsgStruct {
    char start[2];      // "BR"
    uint16_t len;       // 消息长度
    uint16_t cmd;       // 消息ID
    uint8_t src;        // 源地址
    uint8_t dst;        // 目的地址
};

struct MsgCmd2300 {
    uint8_t mode;           // 模式
    uint8_t gain_setting;   // 增益设置
    uint16_t angle;         // 角度（0-399，乘以0.9得到真实角度）
    uint16_t trans_duration; // 传输持续时间
    uint16_t sample_period;  // 采样周期
    uint16_t frequency;     // 频率
    uint16_t length;        // 数据长度
    // uint8_t data[];       // 强度数据数组（可变长度）
};
#pragma pack(pop)

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;

struct SonarPoint {
    float heading_angle;          // 声纳指向的中心角度（弧度）
    std::vector<float> angles;    // 每个采样点的实际角度（弧度）
    std::vector<float> ranges;    // 距离值（米）
    std::vector<float> intensities; // 强度值
    ros::Time timestamp;          // 时间戳
    bool valid;                   // 是否有效
    
    SonarPoint() : heading_angle(0.0f), valid(true) {}
    
    bool empty() const { return ranges.empty(); }
    size_t size() const { return ranges.size(); }
    
    void clear() {
        angles.clear();
        ranges.clear();
        intensities.clear();
    }
};

class SonarMapBuilder
{
public:
    SonarMapBuilder(ros::NodeHandle& nh);
    ~SonarMapBuilder();
    
    bool loadSonarData(const std::string& filename);
    void processAndPublish();
    bool isDataLoaded() const { return data_loaded_; }
    
    size_t getPointCount() const { 
        size_t total = 0;
        for (const auto& pt : sonar_points_) {
            total += pt.size();
        }
        return total;
    }
    
    void printDataSummary() const;
    void saveDataToCSV(const std::string& filename) const;
    
    static std::pair<float, float> polarToCartesian(float range, float angle);
    
private:
    // 发布函数
    void publishPointCloud();
    void publishLaserScan();
    void publishOccupancyGrid();
    void publishMarker();
    void publishTF();
    
    // 数据处理函数
    void convertToPointCloud();
    void createOccupancyGrid();
    void filterPoints(float max_range, float min_intensity);
    void enhanceMissingAngles();
    
    // 扇形波束展开
    void applyBeamSpreading(SonarPoint& point, float beam_width_deg = 15.0f);
    
    // 文件解析函数
    bool parseBinaryFile(const std::string& filename);
    
    // 模拟数据生成
    void generateTestData();
    
    // 调试和分析函数
    void validateAngleDistribution();
    
private:
    ros::NodeHandle& nh_;
    
    // 发布器
    ros::Publisher pointcloud_pub_;
    ros::Publisher laser_pub_;
    ros::Publisher occupancy_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher pose_array_pub_;
    
    tf::TransformBroadcaster* tf_broadcaster_;
    
    // 数据存储
    std::vector<SonarPoint> sonar_points_;
    PointCloudI::Ptr pointcloud_;
    nav_msgs::OccupancyGrid occupancy_grid_;
    
    // 处理参数
    float max_range_;             // 最大探测距离
    float min_range_;             // 最小探测距离
    float min_intensity_;         // 最小强度阈值
    float map_resolution_;        // 地图分辨率
    int map_width_;               // 地图宽度（像素）
    int map_height_;              // 地图高度（像素）
    float map_origin_x_;          // 地图原点X坐标
    float map_origin_y_;          // 地图原点Y坐标
    float distance_scale_;        // 距离缩放因子
    bool enable_beam_spreading_;  // 启用扇形展开
    
    std::string data_file_;       // 数据文件路径
    
    bool data_loaded_;            // 数据加载状态
    bool debug_mode_;             // 调试模式

    bool enable_symmetry_; 

    // ==================== 新增：方向性过滤参数 ====================
    float near_side_max_range_;   // 采集侧最大范围
    float far_side_max_range_;    // 对侧最大范围
    float side_wall_max_range_;   // 侧墙最大范围
    
    float near_side_intensity_;   // 采集侧强度阈值
    float far_side_intensity_;    // 对侧强度阈值
    float side_wall_intensity_;   // 侧墙强度阈值
    // ==================== 新增结束 ====================

};

} // namespace sonar_slam

#endif // SONAR_MAP_BUILDER_H