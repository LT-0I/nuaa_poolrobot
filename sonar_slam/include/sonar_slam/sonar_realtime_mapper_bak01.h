/**
 * @file sonar_realtime_mapper.h
 * @brief 实时声纳数据采集与地图构建器
 * 
 * 功能概述：
 * 1. 通过串口与声纳设备实时通信
 * 2. 解析声纳数据消息（msgid=2300）
 * 3. 构建并实时更新点云地图
 * 4. 支持完整的360度扫描
 * 
 * 使用场景：
 * - 将机器人放置在水池中心进行扫描
 * - 实时构建游泳池轮廓地图
 * - 检测水中障碍物
 * - 实现构建整个环境的代价地图
 */

#ifndef SONAR_REALTIME_MAPPER_H
#define SONAR_REALTIME_MAPPER_H

#include <ros/ros.h>
#include <sys/stat.h>  // stat, mkdir
#include <unistd.h>    // getcwd
#include <ctime>  
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <mars_msgs/Pose2DArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>

#include <pcl/registration/icp.h>

#include "AsyncSerial.h"  // 串口通信库

#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>

 
// Eigen头文件 - 新增
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Trigger.h>
#include "sonar_slam/StartPhase1.h"

#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

// C++标准库
#include <iomanip>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <map>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <queue>
#include <cmath>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>
#include <ctime>   


using boost::placeholders::_1;
using boost::placeholders::_2;
namespace sonar_slam {

// ======================== 数据结构定义 ========================

/**
 * @brief 声纳原始数据点
 */
struct SonarRawPoint {
    float distance;    // 距离（米）
    uint8_t intensity; // 强度（0-255）
    
    SonarRawPoint() : distance(0.0f), intensity(0) {}
    SonarRawPoint(float d, uint8_t i) : distance(d), intensity(i) {}
};

/**
 * @brief 处理后的声纳数据点
 */
struct SonarPoint {
    float angle;                    // 角度（弧度）
    std::vector<float> ranges;      // 距离数组（米）
    std::vector<float> intensities; // 强度数组
    ros::Time timestamp;            // 时间戳
    
    SonarPoint() : angle(0.0f) {}
    
    void clear() {
        ranges.clear();
        intensities.clear();
    }
    
    bool empty() const {
        return ranges.empty();
    }
};

/**
 * @brief 代价地图单元
 */
struct CostCell {
    float occupancy;    // 占据概率 0.0-1.0
    float cost;         // 代价 0.0-1.0
    uint32_t hits;      // 命中次数
    uint32_t misses;    // 未命中次数
    
    CostCell() : occupancy(0.5f), cost(0.0f), hits(0), misses(0) {}
    
    void update(bool hit) {
        if (hit) {
            hits++;
            occupancy = std::min(1.0f, occupancy + 0.1f);
        } else {
            misses++;
            occupancy = std::max(0.0f, occupancy - 0.05f);
        }
        
        // 计算代价：高占据概率 = 高代价
        cost = occupancy * occupancy;
    }
};

/**
 * @brief 声纳2300消息结构（与硬件协议一致）
 */
#pragma pack(push, 1)
struct MsgCmd2300 {
    uint8_t mode;               // 字节0: 工作模式
    uint8_t gain_setting;       // 字节1: 增益设置
    uint16_t angle;             // 字节2-3: 角度 (小端序)
    uint16_t transmit_duration; // 字节4-5: 发射持续时间 (us)
    uint16_t sample_period;     // 字节6-7: 采样周期 (25ns为单位)
    uint16_t transmit_frequency;// 字节8-9: 发射频率 (kHz)
    uint16_t number_of_samples; // 字节10-11: 采样点数
    uint16_t data_length;       // 字节12-13: 数据长度
    uint8_t data[1200];         // 字节14+: 强度数据
    
    // 角度转换函数（保持不变）
    float getAngleRad() const {
        return (static_cast<float>(angle) / 399.0f) * 2.0f * M_PI;
    }
    
    float getAngleDeg() const {
        return (static_cast<float>(angle) / 399.0f) * 360.0f;
    }
    
    // 距离计算函数（需要修正）
    // 距离计算函数（就在这里定义）
    float getSampleDistance(int sample_index, float sound_speed = 1500.0f) const {
        // 注意：sample_index从0开始
        // sample_period 单位是25ns
        // 转换为秒: sample_period * 25e-9
        float time_seconds = sample_index * sample_period * 25.0e-9f;
        
        // 距离 = 时间 × 声速 ÷ 2 (往返时间)
        return time_seconds * sound_speed / 2.0f;
    }
    
    float getMaxRange(float sound_speed = 1500.0f) const {
        return getSampleDistance(number_of_samples, sound_speed);
    }
};
#pragma pack(pop)

 
struct Phase1SubMap {
    nav_msgs::OccupancyGrid map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    geometry_msgs::Pose2D robot_pose;
    ros::Time timestamp;
    int index;
    
    // 新增：质心坐标
    float center_x;
    float center_y;
    
    Phase1SubMap() : cloud(new pcl::PointCloud<pcl::PointXYZI>()), 
                     center_x(0.0f), center_y(0.0f) {}
};
 
// ======================== 主类定义 ========================

/**
 * @class SonarRealtimeMapper
 * @brief 实时声纳数据采集与地图构建器
 */
class SonarRealtimeMapper {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit SonarRealtimeMapper(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~SonarRealtimeMapper();
    
    /**
     * @brief 初始化串口通信
     * @param port 串口设备路径
     * @param baudrate 波特率
     * @return 成功返回true
     */
    bool initSerial(const std::string& port, int baudrate = 115200);
    
    /**
     * @brief 开始实时扫描
     */
    void startScanning();
    
    /**
     * @brief 停止扫描
     */
    void stopScanning();
    
    /**
     * @brief 保存当前扫描数据
     * @param filename 输出文件名
     * @return 成功返回true
     */
    bool saveScanData(const std::string& filename);
    
    /**
     * @brief 保存代价地图
     * @param filename 输出文件名
     * @return 成功返回true
     */
    bool saveCostMap(const std::string& filename);
    
    /**
     * @brief 清空所有数据
     */
    void clearScanData();
    
    /**
     * @brief 获取当前扫描角度
     */
    unsigned short getCurrentAngle() const { return current_angle_.load(); }
    
    /**
     * @brief 获取扫描帧数
     */
    size_t getScanFrameCount() const { return sonar_points_.size(); }
    
    /**
     * @brief 检查是否正在扫描
     */
    bool isScanning() const { return is_scanning_.load(); }
    
    /**
     * @brief 检查是否完成360度扫描
     */
    bool isFullScanComplete() const { return full_scan_complete_.load(); }

     // 公共接口
    void startPhase1Mapping(int max_submaps = 2);
    void stopPhase1Mapping();
    bool isPhase1Active() const { return phase1_enabled_; }
    int getPhase1Step() const { return phase1_step_; }
    std::string getPhase1StatusString() const;


    void printPhase1DebugInfo();

    void handlePhase1Mapping();

        /**
     * @brief 发布TF变换
     */
    void publishTF();
    
private:
    // ======================== ROS相关 ========================
    ros::NodeHandle& nh_;
    
    // 发布器
    ros::Publisher pointcloud_pub_;        // 原始点云
    ros::Publisher filtered_pointcloud_pub_;// 滤波后点云
    ros::Publisher occupancy_pub_;         // 占据栅格地图
    ros::Publisher costmap_pub_;           // 代价地图
    ros::Publisher marker_pub_;            // 可视化标记
    ros::Publisher poses_pub_;             // 位姿数组
    
     // 新增发布器
    ros::Publisher dense_pointcloud_pub_;  // 密集点云发布器
    ros::Publisher wall_marker_pub_;       // 墙壁标记发布器
    ros::Publisher corner_marker_pub_;     // 角落标记发布器

     // 添加这2个发布器
    ros::Publisher static_map_done_pub_;
    ros::Publisher initial_occupancy_pub_;

    // TF广播器
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    
    // ======================== 串口通信 ========================
    std::unique_ptr<CallbackAsyncSerial> sonar_serial_;
    
    // 接收缓冲区
    static const size_t RX_BUFFER_SIZE = 4096;
    unsigned char rx_buffer_[RX_BUFFER_SIZE];
    size_t rx_end_index_;
    size_t rx_used_index_;
    
    // ======================== 消息解析状态 ========================
    enum ParseState {
        STATE_IDLE,
        STATE_GOT_START1,          // 收到0x42
        STATE_GOT_START2,          // 收到0x52
        STATE_READING_LENGTH_LOW,  // 读取长度低位
        STATE_READING_LENGTH_HIGH, // 读取长度高位
        STATE_READING_COMMAND_LOW, // 读取命令低位
        STATE_READING_COMMAND_HIGH,// 读取命令高位
        STATE_READING_SOURCE,      // 读取源地址
        STATE_READING_DESTINATION, // 读取目的地址
        STATE_READING_DATA,        // 读取数据
        STATE_READING_CHECKSUM_LOW,// 读取校验和低位
        STATE_READING_CHECKSUM_HIGH// 读取校验和高位
    };
    
    ParseState parse_state_;
    
    // 当前解析的消息
    struct CurrentMessage {
        uint16_t length;
        uint16_t command;
        uint8_t source;
        uint8_t destination;
        std::vector<uint8_t> data;
        uint16_t checksum;
        
        CurrentMessage() : length(0), command(0), source(0), destination(0), checksum(0) {}
        void clear() { length = 0; command = 0; data.clear(); checksum = 0; }
    } current_message_;


     // ==================== 新增：数据累积和增强处理相关 ====================
    
    struct SectorData {
        std::map<int, int> range_accumulator;  // 距离(0.1m单位) -> 计数
        std::vector<float> intensity_accumulator;
        std::vector<ros::Time> timestamps;
    };
    
    std::vector<SectorData> sector_data_;      // 按扇区累积的数据
    
    // 数据累积和密集点云生成函数 - 新增
    void accumulateSectorData(int sector_index, const SonarPoint& point);
    bool shouldGenerateDenseCloud() const;
    void generateDensePointCloud();
    
    // ... 现有的其他函数声明 ...
    
    // ==================== 新增：墙壁检测相关结构体 ====================
    
    struct ContourPoint {
        float x, y;
        float intensity;
        
        ContourPoint(float x_ = 0, float y_ = 0, float i = 0) : x(x_), y(y_), intensity(i) {}
    };
    
    struct WallSegment {
        int start_angle;        // 起始角度（0-360度）
        int end_angle;          // 结束角度
        float avg_distance;     // 平均距离
        float max_intensity;    // 最大强度
        std::vector<ContourPoint> points;  // 属于这个段的点
        
        bool isValid() const { 
            return (end_angle - start_angle) >= 5 && !points.empty(); 
        }
    };

  
    // ======================== 数据存储 ========================
    std::vector<SonarPoint> sonar_points_;           // 处理后的声纳数据
    std::vector<SonarRawPoint> raw_points_buffer_;   // 原始数据缓冲区
    static const int MAX_RAW_POINTS = 1200;          // 最大原始点数
    
    // 点云数据
    typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
    typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudIPtr;
    PointCloudIPtr pointcloud_;                      // 原始点云
    PointCloudIPtr filtered_pointcloud_;             // 滤波后点云
    
    // ======================== 地图数据 ========================
    nav_msgs::OccupancyGrid occupancy_map_;          // 占据栅格地图
    nav_msgs::OccupancyGrid cost_map_;               // 代价地图
    
    // 代价地图数据（更精细的概率管理）
    std::vector<CostCell> cost_cells_;               // 代价单元
    int costmap_width_;                              // 代价地图宽度
    int costmap_height_;                             // 代价地图高度
    float costmap_resolution_;                       // 分辨率
    float costmap_origin_x_;                         // 原点X
    float costmap_origin_y_;                         // 原点Y
    
    // ======================== 扫描控制 ========================
    std::atomic<unsigned short> current_angle_;      // 当前角度
    std::atomic<bool> is_scanning_;                  // 扫描状态
    std::atomic<bool> stop_requested_;               // 停止请求
    std::atomic<bool> full_scan_complete_;           // 完整扫描完成标志
    boost::thread scan_thread_;                      // 扫描线程
    
    // 扫描统计
    unsigned int total_scans_;                       // 总扫描次数
    unsigned int valid_scans_;                       // 有效扫描次数
    unsigned int dropped_scans_;                     // 丢弃的扫描次数
    ros::Time scan_start_time_;                      // 扫描开始时间
    
    // ======================== 参数配置 ========================
    float max_range_;                                // 最大测量距离
    float min_range_;                                // 最小测量距离
    float min_intensity_;                            // 最小强度阈值
    
    float map_resolution_;                           // 地图分辨率
    int map_width_;                                  // 地图宽度
    int map_height_;                                 // 地图高度
    float map_origin_x_;                             // 地图原点X
    float map_origin_y_;                             // 地图原点Y
    
    float sound_speed_;                              // 水中声速
    
    // 代价地图参数
    float hit_probability_;                          // 命中概率增量
    float miss_probability_;                         // 未命中概率减量
    float occupied_threshold_;                       // 占据阈值
    float free_threshold_;                           // 空闲阈值


    // 扫描角度管理
    int scan_start_angle_;            // 当前扫描起始角度
    int last_scan_end_angle_;         // 上次扫描结束角度
    
    // Phase1 状态管理
    int phase1_current_position_;     // 当前位置（1或2）
    int circles_completed_in_position_; // 当前位置完成的圈数
    int required_circles_per_position_; // 每个位置需要的圈数（默认2）
    
    // 位姿管理
    geometry_msgs::Pose2D first_position_pose_;   // 位置1的位姿
    geometry_msgs::Pose2D second_position_pose_;  // 位置2的位姿
 
    
    
    // ======================== 日志记录 ========================
    std::ofstream csv_file_;                         // CSV日志文件
    bool enable_csv_logging_;                        // 是否启用CSV日志

 
 
    std::deque<nav_msgs::OccupancyGrid> recent_scan_maps_; // 最近几次扫描的地图
    std::deque<int> scan_green_point_counts_;


    // 第一阶段状态
    bool phase1_enabled_;
    int phase1_step_;               // 0=未开始, 1=第一次建图, 2=等待移动, 3=第二次建图, 4=完成
    int phase1_submap_count_;
    ros::Time phase1_step_start_time_;
    
    // 子地图存储
    std::vector<Phase1SubMap> phase1_submaps_;
    
    // 配置参数
    float phase1_scan_duration_;
    float phase1_wait_duration_;
    float phase1_move_distance_;    // 估计的移动距离
    
    // 发布器
    ros::Publisher phase1_status_pub_;
    ros::Publisher phase1_submap_pub_;
    


 
    void saveCurrentPhase1Submap();
    void mergePhase1Submaps();
    void publishPhase1Status(const std::string& status = "");
    geometry_msgs::Pose2D getCurrentRobotPose();

  
   void publishRobotPositionMarkers();
    
    // ======================== 私有方法 ========================
    bool getCurrentRobotPoseInMap(geometry_msgs::Pose2D& pose);
    /**
     * @brief 串口数据接收回调
     */
    void receiveDataCallback(const char* data, unsigned int len);
    
    /**
     * @brief 解析接收到的数据包
     */
    void parseDataPacket(const char* buffer, unsigned int length);
    
    /**
     * @brief 处理解析出的完整消息
     */
    void processCompleteMessage();
    
    /**
     * @brief 解析2300命令消息
     */
    bool parseCmd2300Message(const std::vector<uint8_t>& data);
    
    /**
     * @brief 生成控制命令
     */
    void generateControlCommand(unsigned char* buffer, unsigned short angle);
    
    /**
     * @brief 处理单帧扫描数据
     */
    void processScanFrame(int raw_angle, const MsgCmd2300& msg_data);
    
    /**
     * @brief 扫描线程主函数
     */
    void scanningThread();
    
    /**
     * @brief 转换为点云
     */
    void convertToPointCloud();
    
    /**
     * @brief 滤波点云数据
     */
    void filterPointCloud();
    
    /**
     * @brief 创建占据栅格地图
     */
    void createOccupancyGrid();
    
    /**
     * @brief 创建代价地图
     */
    void createCostMap();
    
    /**
     * @brief 更新代价地图单元
     */
    void updateCostCell(int x, int y, bool hit);
    
    /**
     * @brief 发布所有地图数据
     */
    void publishMapData();
    

    
    /**
     * @brief 记录数据到CSV
     */
    void logToCSV(const SonarPoint& point);
    
    /**
     * @brief 计算CRC16校验和
     */
    uint16_t calculateCRC16(const uint8_t* data, size_t length);
    
    /**
     * @brief 验证消息校验和
     */
    bool validateChecksum(const std::vector<uint8_t>& data, uint16_t checksum);
    
    /**
     * @brief 初始化代价地图
     */
    void initializeCostMap();
    
    /**
     * @brief 坐标转换：世界坐标到地图索引
     */
    bool worldToMap(float wx, float wy, int& mx, int& my) const;
    
    /**
     * @brief 坐标转换：地图索引到世界坐标
     */
    void mapToWorld(int mx, int my, float& wx, float& wy) const;


    // 函数声明
    void extractWallSegments(std::vector<WallSegment>& segments);
    void connectWallSegments(std::vector<WallSegment>& segments);
    void fillWallGaps(std::vector<WallSegment>& segments);
    void detectPoolCorners(const std::vector<WallSegment>& segments);
    void addWallSegmentsToContourPoints(const std::vector<WallSegment>& segments,
                                        std::map<int, pcl::PointXYZI>& contour_points);
    
    // 角落检测相关
    void generateInterpolatedPoints(WallSegment& gap_segment,
                                   const WallSegment& prev_segment,
                                   const WallSegment& next_segment);
    
    Eigen::Vector2f computeSegmentDirection(const WallSegment& segment);
    Eigen::Vector2f computeSegmentEndPoint(const WallSegment& segment);
    bool validateCornerPoint(const Eigen::Vector2f& point, 
                            const std::vector<WallSegment>& segments);
    bool analyzeCornerConvexity(const Eigen::Vector2f& corner,
                               const std::vector<WallSegment>& segments);
    std::vector<Eigen::Vector2f> clusterCorners(const std::vector<Eigen::Vector2f>& corners);
    void visualizeCorners(const std::vector<Eigen::Vector2f>& corners);
    void addCornersToCostMap(const std::vector<Eigen::Vector2f>& corners);
    // 添加这个方法
    void checkAndPublishInitialMap();

    void createEnhancedOccupancyGrid();
 
    // 保存阶段地图
    void saveStageMap(const std::string& stage_name);

    bool saveMapAsPGM(const std::string& filename, const nav_msgs::OccupancyGrid& map);
    bool saveMapAsYAML(const std::string& filename, const std::string& pgm_file,
                      const nav_msgs::OccupancyGrid& map);

    // 禁止拷贝和赋值
    SonarRealtimeMapper(const SonarRealtimeMapper&) = delete;
    SonarRealtimeMapper& operator=(const SonarRealtimeMapper&) = delete;
};

} // namespace sonar_slam

#endif // SONAR_REALTIME_MAPPER_H