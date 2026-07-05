#ifndef SONAR_SLAM_MULTI_POSITION_SONAR_MAPPER_H
#define SONAR_SLAM_MULTI_POSITION_SONAR_MAPPER_H
 /**
 * @file multi_position_sonar_mapper.h
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
 
#include <ros/ros.h>
#include <ros/callback_queue.h>  // 守门专用回调队列: /cmd_vel 挂独立队列, 守门只派发它、绝不重入服务回调
#include <sys/stat.h>  // stat, mkdir
#include <unistd.h>    // getcwd
#include <ctime> 

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <mars_msgs/Pose2DArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>  
 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>   // 扫描前泵关+转停守门: 只读观测 /cmd_vel(泵=linear.y, 运动=linear.x/angular.z)


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


#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include "sonar_slam/StartPhase1.h"
#include "sonar_slam/StartPhaseMulti.h"
#include "sonar_slam/MoveStraight.h"
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
 * @class MultiPositionSonarMapper
 * @brief 实时声纳数据采集与地图构建器
 */
class MultiPositionSonarMapper {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit MultiPositionSonarMapper(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~MultiPositionSonarMapper();
    
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
    void openPumpLatch();   // 运行级水泵闩锁: 任务起始置 /move_controller/pump_enable=true(整段持泵)
    void closePumpLatch();  // 任务结束/中止置 false(撤闩, 回落单段策略)
    void spinPumpGateQueue();  // 主循环每拍派发守门专用队列(仅 onCmdVel), 令 last_cmd_* 持续新鲜、无陈旧积压
    void publishSessionDone();  // 采集全完成时发会话目录(触发离线拼图); 仅完成路径调用, enable_stitch_ 门控
    
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

 

        /**
     * @brief 发布TF变换
     */
    void publishTF();

    /**
     * @brief 从累计缓冲(sonar_points_)重建点云/栅格并发布(显示+落盘前刷新)。
     * 把原本每帧(串口I/O线程内)做的重活搬到主线程周期执行: 先在 accum_mutex_ 下把
     * sonar_points_ 拷进 sonar_points_snapshot_(仅阻塞I/O线程这一次拷贝), 释放锁后
     * 在快照上做 convert/filter/grid/cost/publish。主循环 ~2Hz 调(显示), 落盘前强制调一次(内容不变)。
     * 仅主线程调用(主循环 + saveCurrentPhaseMultiSubmap), 两者天然串行, 故 pointcloud_/占据图无需额外互斥。
     */
    void rebuildAndPublishMaps();

     // 1. 启动定位模式（服务调用）
    void startLocalizationMode();

    /**
     * @brief 多位置建图主状态机 (支持3-10个位置)
     */
    void handlePhaseMultiMapping();

    /**
     * @brief 保存当前多位置子地图
     */
    void saveCurrentPhaseMultiSubmap();

    /**
     * @brief 合并多个子地图
     */
    void mergeMultipleSubmaps();

    /**
     * @brief 发布多位置子地图标记
     */
    void publishPhaseMultiSubmapMarkers();

    /**
     * @brief 发布多位置标记
     * @param map_poses 地图坐标系中的位置列表
     */
    void publishPhaseMultiPositionMarkers(const std::vector<geometry_msgs::Pose2D>& map_poses);

    /**
     * @brief 开始多位置建图
     * @param num_positions 位置数量 (3-10)
     */
    void startPhaseMultiMapping(int num_positions);

    /**
     * @brief 停止多位置建图
     */
    void stopPhaseMultiMapping();

    /**
     * @brief 发布多位置状态
     * @param status 状态字符串
     */
    void publishPhaseMultiStatus(const std::string& status);

    /**
     * @brief 获取多位置状态字符串
     * @return 状态字符串
     */
    std::string getPhaseMultiStatusString() const;

    /**
     * @brief 检查多位置模式是否启用
     * @return true 如果启用
     */
    bool isPhaseMultiEnabled() const { return phase_multi_enabled_; }
    
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
    static const size_t RX_BUFFER_SIZE = 40960;
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
    std::vector<SonarPoint> sonar_points_;           // 处理后的声纳数据(I/O线程写, 受 accum_mutex_ 保护)
    std::vector<SonarPoint> sonar_points_snapshot_;  // 主线程重建/落盘用快照(仅主线程访问, 无需锁; 由 rebuildAndPublishMaps 在 accum_mutex_ 下从 sonar_points_ 拷得)
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
    
    // ======================== 角度基准相关 ========================
    int angle_base_;                    // 角度基准
    bool angle_base_initialized_;        // 角度基准是否已初始化
    std::vector<float> recent_angles_debug_;  // 最近角度调试
    std::vector<float> scanned_angles_;       // 已扫描角度
    std::set<int> covered_angles_;            // 已覆盖角度

 
    void saveCurrentPhase1Submap();
 
    void publishPhase1Status(const std::string& status = "");
    geometry_msgs::Pose2D getCurrentRobotPose();

        // 用于累积一圈数据的缓冲区
    struct FullScanBuffer {
        std::vector<float> ranges[400];      // 每个角度多个距离值（0-399）
        std::vector<float> intensities[400]; // 每个角度多个强度值
        bool angle_received[400];             // 标记该角度是否已收到数据
        ros::Time scan_start_time;
        int angles_completed;
        
        FullScanBuffer() {
            reset();
        }
        
        void reset() {
            for (int i = 0; i < 400; i++) {
                ranges[i].clear();
                intensities[i].clear();
                angle_received[i] = false;
            }
            angles_completed = 0;
            scan_start_time = ros::Time::now();
        }
    };
    
    // 在类定义中
    FullScanBuffer full_scan_buffer_;
    bool accumulating_scan_;
    int required_angles_for_full_scan_;   // 可配置参数
    double sweep_pace_sec_ = 0.1;         // 扫描每角步间距(秒); 私有参数 ~sweep_pace_sec, 默认0.1=原硬编码值(零行为变化)



    // 新增发布器
    ros::Publisher standard_map_pub_;

    ros::Publisher phase_multi_status_pub_;
    ros::Publisher phase_multi_submap_pub_;
    


    // ======================== 新增多位置建图成员变量 ========================
    bool phase_multi_enabled_;                    // 多位置建图模式启用标志
    int phase_multi_step_;                         // 多位置建图步骤
    int phase_multi_num_positions_;                 // 要建图的位置总数 (3-10)
    int phase_multi_current_position_;              // 当前正在建图的位置索引 (1-based)
    ros::Time phase_multi_step_start_time_;         // 当前步骤开始时间
    std::vector<Phase1SubMap> phase_multi_submaps_; // 存储所有子地图 (复用Phase1SubMap结构)
    float phase_multi_scan_duration_;               // 每个位置扫描持续时间
    float phase_multi_wait_duration_;                // 等待移动时间
    float phase_multi_move_distance_;                 // 预期移动距离
    int phase_multi_circles_per_position_;           // 每个位置需要完成的扫描圈数


    std::ofstream odom_log_file_;
    bool odom_log_opened_;

    bool phase_multi_auto_move_;

    bool move_completed_;
     // 移动服务客户端
    ros::ServiceClient move_straight_client_;
    ros::ServiceClient find_wall_client_;     // /move_controller/find_wall (acquire_wall_first 用)
    std::string session_dir_;                 // 本次运行智能会话目录 ~/sonar_logs/scan_<时间>/ (maps/raw_sonar/odom 全落此)
    bool acquire_wall_first_ = false;         // 扫描前先找墙对齐(默认关; 沿边+声呐入口置真)
    float acquire_distance_ = 0.5f;           // 找墙对齐后先沿边行走距离(米)
    ros::Publisher pump_enable_pub_;          // /move_controller/pump_enable 运行级水泵闩锁(std_msgs::Bool, latched)
    bool pump_latch_published_open_ = false;  // 幂等闸: 防重复 open/close
    ros::Publisher session_done_pub_;         // /sonar_scan/session_done 采集完成信号(std_msgs::String=会话目录) 触发离线拼图节点; latched
    bool enable_stitch_ = true;               // 拼图开关(私有参数 ~enable_stitch); false=不发完成信号(行为同接入前)

    // ===== 扫描前"泵关+转停"守门 (pump-off / robot-stopped gate before sonar fire) =====
    // 任务要求(用户确认): 声呐扫描前必须关泵(扫描时不吸底) 且 车已停。注意与 CLAUDE.md 宪法安全红线区分——
    //   宪法红线是"声呐绝不在空气中发射(烧换能器)", 由采集/操作规程与入水判定把关; "扫描时关泵"是本任务的作业要求, 非宪法条款。
    // move_controller 已实现"行走开泵/停车关泵/剩0.3m提前关"; 车停 => 泵应自动归零。
    // 本守门只在每次 startScanning 前观测 /cmd_vel 显式确认泵关+转停并 settle 一段, 不发任何指令, 不碰 move_controller/驱动。
    ros::CallbackQueue pump_gate_queue_;      // /cmd_vel 专用回调队列: 守门用 callAvailable 只派发 onCmdVel, 绝不重入本节点的服务回调(防守门期嵌套 startScanning / 吞停止请求)
    ros::Subscriber cmd_vel_sub_;             // 只读观测 /cmd_vel(绝对话题, 挂 pump_gate_queue_) 供守门判泵关+转停
    // last-cmd 观测缓存: onCmdVel 只由主线程派发 pump_gate_queue_ 时写(主循环 spinPumpGateQueue 或守门 callAvailable),
    //   scanningThread 的 spinOnce 只派发全局队列、不含 /cmd_vel; 调用守门时上一个 scan_thread_ 已 join, 读写均在主线程, 无真并发, 普通成员即可。
    double last_cmd_lin_x_ = 0.0;             // 最近 /cmd_vel linear.x (前进差速)
    double last_cmd_lin_y_ = 0.0;             // 最近 /cmd_vel linear.y (泵占空 0~1)
    double last_cmd_ang_z_ = 0.0;             // 最近 /cmd_vel angular.z (转向)
    ros::Time last_cmd_stamp_;                // 最近 /cmd_vel 到达时刻(Twist 无 header, 取接收时刻判新鲜)
    bool   pump_guard_enable_ = true;         // 守门总开关(~pump_guard_enable)
    double pump_settle_sec_ = 2.5;            // 确认(或超时)后物理转停沉降等待秒(~pump_settle_sec); 在 scan_duration 计时前, 不吃扫描时间
    double pump_off_thresh_ = 0.02;           // |linear.y| 判泵关阈值(~pump_off_thresh)
    double pump_stopped_thresh_ = 0.02;       // |linear.x|/|angular.z| 判车停阈值(~pump_stopped_thresh)
    double pump_guard_timeout_sec_ = 6.0;     // 守门确认最长等待秒(~pump_guard_timeout_sec); 到点仍未确认则报错后继续扫(用户拍板: 报错仍扫)
    double pump_guard_stable_sec_ = 0.5;      // 条件需连续保持时长才算确认秒(~pump_guard_stable_sec)
    double pump_cmd_fresh_sec_ = 0.3;         // /cmd_vel 新鲜判据秒(~pump_cmd_fresh_sec); 须 < stable_sec, 否则死掉的发布者其陈旧末值也能凑满 stable 假确认; 构造期钳位保证 < stable
    int    pump_gate_idx_ = 0;                // 守门调用序号(落盘 pump_gate_log.csv 用; 每次 startScanning 守门 +1)

    bool waitPumpOffAndSettle();              // 扫描前守门: 观测确认泵关+车停并 settle; 返回是否已确认(超时也返回并继续扫)
    void onCmdVel(const geometry_msgs::Twist::ConstPtr& msg);  // /cmd_vel 只读观测回调, 刷新 last_cmd_*
    void logPumpGate(int idx, bool confirmed, double elapsed_s, double age_s,
                     double lin_y, double lin_x, double ang_z, const char* note);  // 守门裁决落盘 session_dir_/pump_gate_log.csv

        // 角度信息结构体
    struct AngleInfo {
        float distance;      // 最近距离（米）
        float intensity;     // 对应强度
        bool valid;          // 是否有效（强度>=160）
        
        AngleInfo() : distance(999.0f), intensity(0.0f), valid(false) {}
    };
    
    // 存储每个角度的墙壁信息（0-359度）
    std::map<int, AngleInfo> angle_wall_info_;
    
    // 前方检测参数
    float front_detection_center_;      // 前方中心角度（270°）
    float front_detection_range_;       // 检测范围（度）
    float front_min_intensity_;         // 最小强度阈值
    float front_min_distance_;          // 触发距离阈值（米）
    int front_min_consecutive_;         // 最小连续角度数

     // ========== 线程同步 ==========
    std::mutex pointcloud_mutex_;    // 保护 pointcloud_ 和 filtered_pointcloud_
    std::mutex map_mutex_;
    // 保护 sonar_points_ 累计缓冲: I/O(串口)线程 processScanFrame 写 vs 主线程重建/落盘快照读。
    // 只在"push一帧 / 拷快照 / 清空"这三处短临界区持有; 重活(点云/栅格重建)一律在锁外的快照上做, 绝不持此锁做重活。
    std::mutex accum_mutex_;

 
    bool check_loop_closure_;  // 回环检测标志
    
  
    // 新增服务回调函数声明
    bool startPhaseMultiCallback(sonar_slam::StartPhaseMulti::Request& req,
                                sonar_slam::StartPhaseMulti::Response& res);
    bool stopPhaseMultiCallback(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res);

  
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
     
   

    void createEnhancedOccupancyGrid();
 
    // 保存阶段地图
    void saveStageMap(const std::string& stage_name);

    bool saveMapAsPGM(const std::string& filename, const nav_msgs::OccupancyGrid& map);
    bool saveMapAsYAML(const std::string& filename, const std::string& pgm_file,
                      const nav_msgs::OccupancyGrid& map);

    // 禁止拷贝和赋值
    MultiPositionSonarMapper(const MultiPositionSonarMapper&) = delete;
    MultiPositionSonarMapper& operator=(const MultiPositionSonarMapper&) = delete;

    /**
     * @brief 重置扫描状态，为下一个位置做准备
     */
    void resetForNextPosition();

    /**
     * @brief 检查是否所有位置都已建图完成
     */
    bool isAllPositionsCompleted() const;

    /**
     * @brief 获取下一个要建图的位置索引
     */
    int getNextPositionIndex() const;
 
    // 分析前方墙壁（270°附近）
    bool analyzeFrontWall(float& distance, float& intensity, int& consecutive_deg);

         // 路径管理函数
    std::string getMapsDirectory();
    std::string getRawSonarDirectory();

     bool saveRawSonarData(const std::string& filename, int position);

};

} // namespace sonar_slam

 

#endif // SONAR_SLAM_MULTI_POSITION_SONAR_MAPPER_H