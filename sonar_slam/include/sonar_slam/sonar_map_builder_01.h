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

namespace sonar_slam {

// sonar_map_builder.h 中添加
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
    uint16_t angle;         // 角度（0-399）
    uint16_t trans_duration; // 传输持续时间
    uint16_t sample_period;  // 采样周期（注意：原始代码中是samplepriead）
    uint16_t frequency;     // 频率
    uint16_t length;        // 数据长度
    // uint8_t data[];       // 强度数据数组（可变长度）
};
#pragma pack(pop)

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;

// 声纳点数据结构（增强版）
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
    
    // 清除数据
    void clear() {
        angles.clear();
        ranges.clear();
        intensities.clear();
    }
};

// 声纳帧数据结构
struct SonarFrame {
    uint16_t angle_raw;          // 原始角度值（0-399）
    float angle_deg;             // 转换后的角度（度）
    uint16_t sample_period;      // 采样周期
    uint16_t number_of_samples;  // 采样数
    std::vector<uint8_t> intensity_data; // 强度数据
    ros::Time timestamp;         // 时间戳
};

class SonarMapBuilder
{
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    SonarMapBuilder(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~SonarMapBuilder();
    
    /**
     * @brief 加载声纳数据文件
     * @param filename 数据文件路径
     * @return 成功返回true，失败返回false
     */
    bool loadSonarData(const std::string& filename);
    
    /**
     * @brief 处理数据并发布所有结果
     */
    void processAndPublish();
    
    /**
     * @brief 检查数据是否已加载
     * @return 数据已加载返回true
     */
    bool isDataLoaded() const { return data_loaded_; }
    
    /**
     * @brief 获取数据点数
     * @return 数据点总数
     */
    size_t getPointCount() const { 
        size_t total = 0;
        for (const auto& pt : sonar_points_) {
            total += pt.size();
        }
        return total;
    }
    
    /**
     * @brief 打印数据摘要
     */
    void printDataSummary() const;
    
    /**
     * @brief 保存数据到CSV文件
     * @param filename CSV文件名
     */
    void saveDataToCSV(const std::string& filename) const;
    
    /**
     * @brief 极坐标转笛卡尔坐标
     * @param range 距离
     * @param angle 角度（弧度）
     * @return (x, y)坐标对
     */
    static std::pair<float, float> polarToCartesian(float range, float angle);
    
    /**
     * @brief 设置声纳安装参数
     * @param beam_width 波束宽度（度）
     * @param fov 视场角（度）
     */
    void setSonarParameters(float beam_width, float fov) {
        beam_width_deg_ = beam_width;
        fov_deg_ = fov;
        beam_width_rad_ = beam_width_deg_ * M_PI / 180.0f;
        fov_rad_ = fov_deg_ * M_PI / 180.0f;
    }
    
private:
    // 发布函数
    void publishPointCloud();
    void publishLaserScan();
    void publishOccupancyGrid();
    void publishMarker();
    void publishTF();
    void publishDebugMarkers();
    
    // 数据处理函数
    void convertToPointCloud();
    void createOccupancyGrid();
    void filterPoints(float max_range, float min_intensity);
    void processSonarFrames();
    
    // 文件解析函数
    bool parseBinaryFile(const std::string& filename);
    bool parseMessage2300(const std::vector<char>& msgdata, SonarFrame& frame);
    void processSonarFrame(const SonarFrame& frame);
    
    // 模拟数据生成（用于测试）
    void generateTestData();
    
    // 调试函数
    void visualizeRawData();

    void validateAngleDistribution();

    void enhanceMissingAngles();

    void applyBeamSpreadingToPoint(SonarPoint& point, float beam_width_rad);
    void applyBeamSpreading(float beam_width_deg);
    
private:
    ros::NodeHandle& nh_;                          // ROS节点句柄
    
    // 发布器
    ros::Publisher pointcloud_pub_;               // 点云发布器
    ros::Publisher laser_pub_;                    // 激光扫描发布器
    ros::Publisher occupancy_pub_;                // 占据栅格发布器
    ros::Publisher marker_pub_;                   // 可视化标记发布器
    ros::Publisher pose_array_pub_;               // 位姿数组发布器
    ros::Publisher debug_pub_;                    // 调试发布器
    
    tf::TransformBroadcaster* tf_broadcaster_;    // TF广播器
    
    
    // 数据存储
    std::vector<SonarPoint> sonar_points_;        // 处理后的声纳数据
    std::vector<SonarFrame> raw_frames_;          // 原始声纳帧
    PointCloudI::Ptr pointcloud_;                 // 处理后的点云
    
    // 地图数据
    nav_msgs::OccupancyGrid occupancy_grid_;      // 占据栅格地图

    float distance_scale_;     // 距离缩放因子
    
    // 声纳参数
    float beam_width_deg_;        // 波束宽度（度）
    float beam_width_rad_;        // 波束宽度（弧度）
    float fov_deg_;               // 视场角（度）
    float fov_rad_;               // 视场角（弧度）
    float sound_speed_;           // 声速（m/s）
    
    // 参数
    float max_range_;             // 最大探测距离
    float min_range_;             // 最小探测距离
    float min_intensity_;         // 最小强度阈值
    float map_resolution_;        // 地图分辨率
    int map_width_;               // 地图宽度（像素）
    int map_height_;              // 地图高度（像素）
    float map_origin_x_;          // 地图原点X坐标
    float map_origin_y_;          // 地图原点Y坐标
    
    // 处理参数
    bool enable_intensity_filter_; // 启用强度过滤
    bool enable_range_filter_;     // 启用距离过滤
    bool enable_interpolation_;    // 启用插值
    
    std::string data_file_;       // 数据文件路径
    
    bool data_loaded_;            // 数据加载状态
    bool debug_mode_;             // 调试模式
};

} // namespace sonar_slam

#endif // SONAR_MAP_BUILDER_H