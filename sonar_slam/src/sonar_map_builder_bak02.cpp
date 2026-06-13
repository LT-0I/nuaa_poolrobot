/**
 * @file sonar_realtime_mapper.cpp
 * @brief 实时声纳数据采集与地图构建器实现
 * 
 * 功能概述：
 * 1. 通过串口与声纳设备实时通信
 * 2. 解析声纳数据消息（msgid=2300）
 * 3. 构建并实时更新点云地图
 * 4. 支持完整的360度扫描
 * 5. 构建代价地图
 * 
 *     噪声过滤：添加了统计滤波和半径滤波

    轮廓提取：使用RANSAC算法检测水池边界

    障碍物聚类：使用欧几里得聚类分离障碍物

    可视化优化：不同的点云用不同颜色显示

    参数调整：可以根据实际情况调整过滤阈值

    这样处理后， 应该能看到：

    水池轮廓用红色点显示

    障碍物用绿色点显示

    噪声点被大大减少

    图像更加清晰，便于理解
 */

#include "sonar_slam/sonar_realtime_mapper.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sys/stat.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
 

namespace sonar_slam {

using namespace std::chrono;

// ======================== 常量定义 ========================
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// ======================== 构造函数和析构函数 ========================

SonarRealtimeMapper::SonarRealtimeMapper(ros::NodeHandle& nh)
    : nh_(nh)
    , rx_end_index_(0)
    , rx_used_index_(0)
    , parse_state_(STATE_IDLE)
    , current_angle_(0)
    , is_scanning_(false)
    , stop_requested_(false)
    , full_scan_complete_(false)
    , total_scans_(0)
    , valid_scans_(0)
    , dropped_scans_(0)
    , pointcloud_(new PointCloudI())
    , filtered_pointcloud_(new PointCloudI())
    , enable_csv_logging_(false) 
     , phase1_enabled_(false)
    , phase1_step_(0)
    , phase1_submap_count_(0)
    , phase1_scan_duration_(30.0f)
    , phase1_wait_duration_(100.0f)
    , phase1_move_distance_(1.5f)
    {
    
    ROS_INFO("========================================");
    ROS_INFO("Sonar Realtime Mapper 初始化");
    ROS_INFO("========================================");
    
    // 初始化接收缓冲区
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    
    // 初始化原始数据缓冲区
    raw_points_buffer_.resize(MAX_RAW_POINTS);
    
    // ======================== 读取参数 ========================
    
    // 基本参数
    nh_.param("max_range", max_range_, 10.0f);
    nh_.param("min_range", min_range_, 0.5f);
    nh_.param("min_intensity", min_intensity_, 50.0f); // 最小强度 50
    
    nh_.param("sound_speed", sound_speed_, 1500.0f);
    // ==================== 关键参数（与参考代码一致） ====================
   
    
    
    // 占据栅格地图参数
    
    nh_.param("map_resolution", map_resolution_, 0.05f);
    nh_.param("map_width", map_width_, 600);          // 从200改为600
    nh_.param("map_height", map_height_, 600);        // 从200改为600
    nh_.param("map_origin_x", map_origin_x_, -15.0f); // 从-5.0改为-15.0
    nh_.param("map_origin_y", map_origin_y_, -15.0f); // 从-5.0改为-15.0
        
    // 代价地图参数
    nh_.param("costmap_resolution", costmap_resolution_, 0.1f);
    nh_.param("costmap_width", costmap_width_, 300);       // 从100改为300
    nh_.param("costmap_height", costmap_height_, 300);     // 从100改为300
    nh_.param("costmap_origin_x", costmap_origin_x_, -15.0f); // 从-5.0改为-15.0
    nh_.param("costmap_origin_y", costmap_origin_y_, -15.0f); // 从-5.0改为-15.0
    nh_.param("hit_probability", hit_probability_, 0.7f);
    nh_.param("miss_probability", miss_probability_, 0.4f);
    nh_.param("occupied_threshold", occupied_threshold_, 0.65f);
    nh_.param("free_threshold", free_threshold_, 0.25f);

 
        

    // 日志参数
    nh_.param("enable_csv_logging", enable_csv_logging_, false);
    
    // ======================== 初始化ROS发布器 ========================
    
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sonar/pointcloud", 1);
    filtered_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sonar/filtered_pointcloud", 1);
    occupancy_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("sonar/map", 1, true);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("sonar/costmap", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar/marker", 1);
    poses_pub_ = nh_.advertise<mars_msgs::Pose2DArray>("sonar/poses", 1);


     // 发布器初始化
    dense_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sonar/dense_pointcloud", 1);
    wall_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar/walls", 1);
    corner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar/corners", 1);


    static_map_done_pub_ = nh_.advertise<std_msgs::Bool>("/static_mapping/done", 1, true);
    initial_occupancy_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/initial_occupancy_map", 1, true);
    
    // ======================== 初始化点云 ========================
    
    pointcloud_->header.frame_id = "sonar_frame";
    pointcloud_->is_dense = false;
    
    filtered_pointcloud_->header.frame_id = "sonar_frame";
    filtered_pointcloud_->is_dense = false;
    
    // ======================== 初始化占据栅格地图 ========================
    occupancy_map_.header.frame_id = "map";
    occupancy_map_.info.resolution = map_resolution_;
    
    occupancy_map_.info.width = map_width_;
    occupancy_map_.info.height = map_height_;
    occupancy_map_.info.origin.position.x = map_origin_x_;
    occupancy_map_.info.origin.position.y = map_origin_y_;
    occupancy_map_.info.origin.position.z = 0.0;
    occupancy_map_.info.origin.orientation.w = 1.0;
    occupancy_map_.data.resize(map_width_ * map_height_, -1);
    
    // ======================== 初始化代价地图 ========================
    
    cost_map_.header.frame_id = "map";
    cost_map_.info.resolution = costmap_resolution_;
    cost_map_.info.width = costmap_width_;
    cost_map_.info.height = costmap_height_;
    cost_map_.info.origin.position.x = costmap_origin_x_;
    cost_map_.info.origin.position.y = costmap_origin_y_;
    cost_map_.info.origin.position.z = 0.0;
    cost_map_.info.origin.orientation.w = 1.0;
    cost_map_.data.resize(costmap_width_ * costmap_height_, -1);
    
    // 初始化代价单元
    cost_cells_.resize(costmap_width_ * costmap_height_);
    
    // ======================== 打印配置信息 ========================
    
    ROS_INFO("参数配置:");
    ROS_INFO("  测量范围: %.1f - %.1f 米", min_range_, max_range_);
    ROS_INFO("  最小强度: %.1f", min_intensity_);
    ROS_INFO("  声速: %.1f m/s", sound_speed_);
    ROS_INFO("  占据地图: %dx%d @ %.3f 米/像素", 
             map_width_, map_height_, map_resolution_);
    ROS_INFO("  代价地图: %dx%d @ %.3f 米/像素", 
             costmap_width_, costmap_height_, costmap_resolution_);
    ROS_INFO("========================================");

     // 第一阶段发布器
    phase1_status_pub_ = nh_.advertise<std_msgs::String>("/phase1/status", 1, true);
    phase1_submap_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/phase1/submaps", 1, true);
    
    // 读取第一阶段参数
    bool param_phase1_enabled = false;
    nh_.param("phase1/enabled", param_phase1_enabled, false);
    nh_.param("phase1/scan_duration", phase1_scan_duration_, 30.0f);
    nh_.param("phase1/wait_duration", phase1_wait_duration_, 100.0f);
    nh_.param("phase1/move_distance", phase1_move_distance_, 1.5f);
    
   
    

}

SonarRealtimeMapper::~SonarRealtimeMapper() {
    stopScanning();
    
    if (sonar_serial_) {
        sonar_serial_->close();
    }
    
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
    
    ROS_INFO("Sonar Realtime Mapper 销毁");
}

// ======================== 串口初始化 ========================

bool SonarRealtimeMapper::initSerial(const std::string& port, int baudrate) {
    try {
        sonar_serial_ = std::make_unique<CallbackAsyncSerial>(port, baudrate);
        sonar_serial_->setCallback(
            boost::bind(&SonarRealtimeMapper::receiveDataCallback, this, _1, _2));
        
        // 重置解析状态
        parse_state_ = STATE_IDLE;
        current_message_.clear();
        rx_end_index_ = 0;
        rx_used_index_ = 0;
        
        ROS_INFO("串口初始化成功: %s @ %d 波特", port.c_str(), baudrate);
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("串口初始化失败 %s: %s", port.c_str(), e.what());
        return false;
    }
}

// ======================== 串口数据接收回调 ========================

void SonarRealtimeMapper::receiveDataCallback(const char* data, unsigned int len) {
    if (len == 0 || !is_scanning_.load()) {
        return;
    }
    
    // 将数据添加到缓冲区
    if (rx_end_index_ + len <= RX_BUFFER_SIZE) {
        memcpy(rx_buffer_ + rx_end_index_, data, len);
        rx_end_index_ += len;
    } else {
        ROS_WARN("接收缓冲区溢出，丢弃数据");
        rx_end_index_ = 0;
        rx_used_index_ = 0;
    }
    
    // 解析数据包 - 修复类型转换错误
    parseDataPacket(reinterpret_cast<const char*>(rx_buffer_), rx_end_index_);
}
// ======================== 数据包解析 ========================

void SonarRealtimeMapper::parseDataPacket(const char* buffer, unsigned int length) {
    while (rx_used_index_ < rx_end_index_) {
        unsigned char current_byte = rx_buffer_[rx_used_index_];
        
        switch (parse_state_) {
            case STATE_IDLE:
                if (current_byte == 0x42) { // 'B'
                    parse_state_ = STATE_GOT_START1;
                }
                rx_used_index_++;
                break;
                
            case STATE_GOT_START1:
                if (current_byte == 0x52) { // 'R'
                    parse_state_ = STATE_READING_LENGTH_LOW;
                } else {
                    // 不是有效的消息头，重置状态
                    parse_state_ = STATE_IDLE;
                }
                rx_used_index_++;
                break;
                
            case STATE_READING_LENGTH_LOW:
                current_message_.length = current_byte;
                parse_state_ = STATE_READING_LENGTH_HIGH;
                rx_used_index_++;
                break;
                
            case STATE_READING_LENGTH_HIGH:
                current_message_.length |= (current_byte << 8);
                parse_state_ = STATE_READING_COMMAND_LOW;
                rx_used_index_++;
                break;
                
            case STATE_READING_COMMAND_LOW:
                current_message_.command = current_byte;
                parse_state_ = STATE_READING_COMMAND_HIGH;
                rx_used_index_++;
                break;
                
            case STATE_READING_COMMAND_HIGH:
                current_message_.command |= (current_byte << 8);
                parse_state_ = STATE_READING_SOURCE;
                rx_used_index_++;
                break;
                
            case STATE_READING_SOURCE:
                current_message_.source = current_byte;
                parse_state_ = STATE_READING_DESTINATION;
                rx_used_index_++;
                break;
                
            case STATE_READING_DESTINATION:
                current_message_.destination = current_byte;
                
                if (current_message_.length > 0) {
                    // 开始读取数据
                    current_message_.data.clear();
                    current_message_.data.reserve(current_message_.length);
                    parse_state_ = STATE_READING_DATA;
                } else {
                    // 没有数据，消息直接完成
                    // ==== 修改点1：直接处理消息，跳过校验和 ====
                    processCompleteMessage();
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                }
                rx_used_index_++;
                break;
                
            case STATE_READING_DATA:
                current_message_.data.push_back(current_byte);
                if (current_message_.data.size() >= current_message_.length) {
                    // ==== 修改点2：数据读完，消息直接完成，跳过校验和 ====
                    // 完整消息已接收，处理它
                    processCompleteMessage();
                    
                    // 重置状态机
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                }
                rx_used_index_++;
                break;
                
            // ==== 修改点3：删除 STATE_READING_CHECKSUM_LOW 和 STATE_READING_CHECKSUM_HIGH 两个状态 ====
            // 旧的校验和状态已不再需要
        }
        
        // ==== 修改点4：优化 - 如果状态已重置为IDLE，且缓冲区还有大量未处理数据，
        // 可以提前移动缓冲区内容，避免rx_used_index_一直增长。
        // 这部分逻辑可以后续优化，当前版本保持简单。
    }
    
    // ==== 修改点5：循环结束后，检查并压缩缓冲区 ====
    // 如果已使用的索引超过了缓冲区大小的一半，将未处理的数据移动到缓冲区头部
    if (rx_used_index_ > RX_BUFFER_SIZE / 2) {
        size_t remaining = rx_end_index_ - rx_used_index_;
        if (remaining > 0) {
            memmove(rx_buffer_, rx_buffer_ + rx_used_index_, remaining);
        }
        rx_end_index_ = remaining;
        rx_used_index_ = 0;
    }
}

// ======================== 处理完整消息 ========================

void SonarRealtimeMapper::processCompleteMessage() {
    total_scans_++;
    
    // ====== 关键调试日志：确认进入此函数 ======
    ROS_INFO_ONCE("[DEBUG] 首次进入 processCompleteMessage 函数。");
    ROS_DEBUG_THROTTLE(1.0, "处理消息: cmd=0x%04X, len=%u", current_message_.command, current_message_.length);
    
    // 根据命令字处理消息
    switch (current_message_.command) {
        case 2300: // 声纳数据消息
            // ====== 关键调试日志：确认处理2300消息 ======
            ROS_INFO_THROTTLE(0.5, "[DEBUG] 处理CMD 2300消息，数据大小: %lu", current_message_.data.size());
            if (parseCmd2300Message(current_message_.data)) {
                valid_scans_++;
                // ====== 成功日志 ======
                ROS_DEBUG_THROTTLE(0.5, "成功解析2300消息 (#%u)", valid_scans_);
            } else {
                ROS_WARN("解析2300消息失败");
            }
            break;
            
        default:
            ROS_DEBUG("收到未知命令: 0x%04X", current_message_.command);
            break;
    }
}

// ======================== 解析2300消息 ========================

bool SonarRealtimeMapper::parseCmd2300Message(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(MsgCmd2300)) {
        ROS_WARN("2300消息数据长度不足: %lu < %lu", 
                data.size(), sizeof(MsgCmd2300));
        return false;
    }
    
    // 复制数据到结构体
    MsgCmd2300 msg_data;
    memcpy(&msg_data, data.data(), sizeof(MsgCmd2300));
    
    // 检查数据有效性
    if (msg_data.number_of_samples > MAX_RAW_POINTS) {
        ROS_WARN("采样点数超出范围: %u", msg_data.number_of_samples);
        return false;
    }
    
    // 处理扫描帧
    processScanFrame(msg_data.angle, msg_data);
    
    return true;
}


// ======================== 处理扫描帧 ========================
void SonarRealtimeMapper::processScanFrame(int raw_angle, const MsgCmd2300& msg_data) {
    // ==================== 【1】角度转换修正 ====================
    // 硬件角度范围是0-399，对应0-359.1度
    // 确保正确处理角度归一化
    float angle_deg = static_cast<float>(raw_angle) * 0.9f;

    static std::vector<float> scanned_angles;
    scanned_angles.push_back(angle_deg);
    
    
    // 每扫描10个角度统计一次
    if (scanned_angles.size() % 10 == 0) {
        float min_angle = *std::min_element(scanned_angles.begin(), scanned_angles.end());
        float max_angle = *std::max_element(scanned_angles.begin(), scanned_angles.end());
        float angle_range = max_angle - min_angle;
        
        ROS_INFO("扫描角度范围: %.1f° ~ %.1f° (跨度: %.1f°)", 
                min_angle, max_angle, angle_range);
        
        if (angle_range < 270.0f) {
            ROS_WARN("扫描角度不足！可能需要调整扫描策略");
        }
    }
 
    // 规范化到0-360度
    angle_deg = fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    
    // 【】角度覆盖统计
    static std::set<int> covered_angles;
    int angle_int = static_cast<int>(angle_deg);
    covered_angles.insert(angle_int);
    
    // 每30度输出一次角度覆盖情况
    static int last_angle_report = -1;
    if (angle_int % 30 == 0 && angle_int != last_angle_report) {
        last_angle_report = angle_int;
        ROS_INFO("角度覆盖: %d° (累计 %lu/360 度)", angle_int, covered_angles.size());
    }
    
    float angle_rad = angle_deg * DEG_TO_RAD;
    
    // ==================== 创建声纳点结构 ====================
    SonarPoint sonar_point;
    sonar_point.angle = angle_rad;
    sonar_point.timestamp = ros::Time::now();
    
    // ====================  距离计算修正 ====================
    // 同事的算法详细分析：
    // 1. timeus = cmd->samplepriead * 25 / 1000;  // 单位：微秒(us)
    // 2. distance_per_point = timeus * 1.45;      // 单位：毫米(mm)，这是每个采样点的距离增量
    // 3. dist = distance_per_point * (j + 1) * 0.001; // 单位：米(m)
    
    // 问题分析：你的计算中缺少了乘以(i+1)的环节！
    // 正确计算：
    float time_per_point_us = static_cast<float>(msg_data.sample_period) * 25.0f / 1000.0f; // 微秒
    float distance_per_point_mm = time_per_point_us * 1.45f;  // 毫米
    
    // 【】调试输出距离参数
    static int debug_counter = 0;
    if (debug_counter % 100 == 0) {
        ROS_INFO("距离参数: sample_period=%u, time_per_point=%.3fus, dist_per_point=%.3fmm", 
                msg_data.sample_period, time_per_point_us, distance_per_point_mm);
        debug_counter = 0;
    }
    debug_counter++;
    
    // ==================== 【3】处理每个采样点 ====================
    int valid_samples = 0;
    int use_cnt = 0;  // 使用同事的计数器命名
    
    // 【新增】: 记录该角度的绿色点云最小距离
    static std::map<int, float> angle_to_green_distance; // 角度索引 -> 绿色点云最小距离
    
    // 获取实际的采样点数
    uint16_t actual_samples = std::min(msg_data.number_of_samples, msg_data.data_length);
    
    // 【】计算理论最大距离
    float max_theoretical_range_mm = distance_per_point_mm * actual_samples;
    float max_theoretical_range_m = max_theoretical_range_mm * 0.001f;
    if (max_range_ < max_theoretical_range_m - 2.0f) {
        ROS_WARN_THROTTLE(10.0, "max_range(%.1fm)可能小于硬件能力(%.1fm)", 
                         max_range_, max_theoretical_range_m);
    }
    
    // 【新增】: 统计绿色点云数量
    int green_points_in_this_frame = 0;
 

    for (uint16_t i = 0; i < actual_samples; i++) {
        // 【】计算每个采样点的实际距离
        float distance_mm = distance_per_point_mm * (i + 1);  // 毫米
        float distance_m = distance_mm * 0.001f;              // 米
        
        // 获取强度
        uint8_t raw_intensity = msg_data.data[i];
        float intensity = static_cast<float>(raw_intensity);
        
        // ==================== 使用距离相关的动态阈值过滤 ====================
        // 原则：距离越远，阈值越低（因为信号衰减）

        float min_required_intensity;
        bool is_green_point_candidate = false;

        // 根据距离设置阈值（距离越远，阈值越低）
        if (distance_m <= 0.8f) {
            // 0.8米内：信号最强，要求最高
            min_required_intensity = 160.0f;  // 最低要求
            if (intensity >= 170.0f) {        // 绿色点云要求更高
                is_green_point_candidate = true;
            }
        } 
        else if (distance_m <= 1.5f) {
            // 0.8-1.5米：信号强，要求高
            min_required_intensity = 150.0f;
            if (intensity >= 160.0f) {
                is_green_point_candidate = true;
            }
        } 
        else if (distance_m <= 3.0f) {
            // 1.5-3米：主要范围，信号中等
            min_required_intensity = 140.0f;
            if (intensity >= 150.0f) {
                is_green_point_candidate = true;
            }
        } 
        else if (distance_m <= 6.0f) {
            // 3-6米：远距离，信号弱，降低要求
            min_required_intensity = 130.0f;
            if (intensity >= 140.0f) {
                is_green_point_candidate = true;
            }
        } 
        else {
            // 6米以上：非常远，信号很弱，要求很低
            min_required_intensity = 100.0f;
            if (intensity >= 120.0f) {
                is_green_point_candidate = true;
            }
        }

        // 主要过滤：强度必须达到最低要求
        if (intensity >= min_required_intensity) {
            // 处理绿色点云（高质量墙壁）
            if (is_green_point_candidate) {
                green_points_in_this_frame++;
                
                // 记录最小绿色点云距离
                if (angle_to_green_distance.find(angle_int) == angle_to_green_distance.end() ||
                    distance_m < angle_to_green_distance[angle_int]) {
                    angle_to_green_distance[angle_int] = distance_m;
                }
                
                sonar_point.ranges.push_back(distance_m);
                sonar_point.intensities.push_back(intensity);
                valid_samples++;
                use_cnt++;
                
                ROS_DEBUG_THROTTLE(1.0, "绿色点云: 角度=%d°, 距离=%.2fm, 强度=%.0f", 
                                angle_int, distance_m, intensity);
            }
            // 处理非绿色点云（中低质量点）
            else {
                // 根据距离设置权重（距离越远，权重越低）
                float weight = 1.0f;
                
                if (distance_m <= 1.5f) {
                    weight = 0.8f;  // 近处非绿色点
                }
                else if (distance_m <= 3.0f) {
                    weight = 0.7f;  // 中距离
                }
                else if (distance_m <= 6.0f) {
                    weight = 0.6f;  // 远距离
                    
                    // 检查是否可能是多径反射噪声
                    if (angle_to_green_distance.find(angle_int) != angle_to_green_distance.end()) {
                        float green_dist = angle_to_green_distance[angle_int];
                        if (distance_m > green_dist * 1.5f && intensity < 120.0f) {
                            continue;  // 跳过噪声点
                        }
                    }
                }
                else {
                    weight = 0.5f;  // 非常远距离
                }
                
                sonar_point.ranges.push_back(distance_m);
                sonar_point.intensities.push_back(intensity * weight);
                valid_samples++;
            }
        }
        // 特殊处理：如果完全没有点，补充一个远距离点
        else if (use_cnt == 0 && distance_m > 3.0f && intensity > 80.0f) {
            // 这个角度完全没有有效点，补充一个低质量点
            float weight = 0.3f;
            sonar_point.ranges.push_back(distance_m);
            sonar_point.intensities.push_back(intensity * weight);
            valid_samples++;
        }
    }
    // ==================== 【4】调试信息输出增强 ====================
    static int frame_counter = 0;
    frame_counter++;
    
    if (valid_samples > 0) {
        // 每5帧输出一次详细信息
        if (frame_counter % 5 == 0) {
            float min_range = *std::min_element(sonar_point.ranges.begin(), sonar_point.ranges.end());
            float max_range = *std::max_element(sonar_point.ranges.begin(), sonar_point.ranges.end());
            
            // 统计绿色点云距离范围
            float green_min_range = 1000.0f;
            float green_max_range = 0.0f;
            int green_count = 0;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                if (sonar_point.ranges[i] <= 3.0f && sonar_point.intensities[i] >= 140.0f) {
                    green_count++;
                    if (sonar_point.ranges[i] < green_min_range) green_min_range = sonar_point.ranges[i];
                    if (sonar_point.ranges[i] > green_max_range) green_max_range = sonar_point.ranges[i];
                }
            }
            
            ROS_INFO("Frame %d: 角度=%.1f°, 总点=%lu, 绿点=%d(%.2f-%.2fm), 全距=%.2f-%.2fm", 
                    frame_counter,
                    angle_deg, 
                    sonar_point.ranges.size(),
                    green_count,
                    green_min_range, green_max_range,
                    min_range, max_range);
        }
        
        // 保存到数据数组
        sonar_points_.push_back(sonar_point);
        
        // 记录到CSV
        if (enable_csv_logging_) {
            logToCSV(sonar_point);
        }
        
        // ==================== 【5】优化更新频率 ====================
        // 立即转换点云（保持实时性）
        convertToPointCloud();
        
        // 【修改】: 基于绿色点云数量决定更新频率
        bool has_green_points = (green_points_in_this_frame > 0);
    
        if (has_green_points) {
            // 有绿色点云：及时更新地图
            static int green_update_counter = 0;
            green_update_counter++;
            
            // 每2次有绿色点云就更新一次
            if (green_update_counter % 2 == 0) {
                filterPointCloud();
                createOccupancyGrid();
                createCostMap();
                publishMapData();
                green_update_counter = 0;
            }
        } 
        else if (frame_counter % 20 == 0) {
            // 无绿色点云：降低更新频率
            filterPointCloud();
            createOccupancyGrid();
            createCostMap();
            publishMapData();
        }
        
        // 累计统计
        if (frame_counter % 100 == 0) {
            float total_points = 0;
            int total_green_points = 0;
            
            for (const auto& pt : sonar_points_) {
                total_points += pt.ranges.size();
                for (size_t i = 0; i < pt.ranges.size(); i++) {
                    if (pt.ranges[i] <= 3.0f && pt.intensities[i] >= 140.0f) {
                        total_green_points++;
                    }
                }
            }
            
            ROS_INFO("累计统计: %d帧, 总点=%.0f, 绿点=%d, 当前角度=%.1f°, 覆盖=%lu/360度", 
                    frame_counter, total_points, total_green_points, angle_deg, covered_angles.size());
            
            // 【】检查是否存在角度盲区
            std::vector<int> missing_angles;
            for (int a = 0; a < 360; a += 30) {
                if (covered_angles.find(a) == covered_angles.end()) {
                    missing_angles.push_back(a);
                }
            }
            if (!missing_angles.empty()) {
                ROS_WARN("角度盲区: ");
                for (size_t i = 0; i < std::min(missing_angles.size(), size_t(5)); i++) {
                    ROS_WARN("  %d°", missing_angles[i]);
                }
                if (missing_angles.size() > 5) {
                    ROS_WARN("  ...等%lu个角度", missing_angles.size() - 5);
                }
            }
        }
    } else {
        // 无效帧统计
        dropped_scans_++;
        
        // 【】记录哪些角度没有有效数据
        static std::set<int> no_data_angles;
        no_data_angles.insert(angle_int);
        
        if (dropped_scans_ % 50 == 0) {
            ROS_WARN("已丢弃 %u 帧无效数据, 无数据角度: %lu个", 
                    dropped_scans_, no_data_angles.size());
        }
    }
    
 
   // ==================== 【6】360度扫描检测优化 + 静态建图流程 ====================
    // 使用时间和角度双重检测，确保扫描足够时长
    static ros::Time last_angle_reset_time = ros::Time::now();
    static float last_angle = -1.0f;
    static int near_zero_count = 0;

    if (last_angle < 0) {
        last_angle = angle_deg;
        last_angle_reset_time = ros::Time::now();
    }

    // 检查是否回到起点附近（角度在0度附近）
    if ((angle_deg > 355.0f || angle_deg < 5.0f) && !full_scan_complete_.load()) {
        near_zero_count++;
    } else if (angle_deg > 10.0f && angle_deg < 350.0f) {
        near_zero_count = 0;
    }

    // 计算时间间隔
    ros::Duration time_since_reset = ros::Time::now() - last_angle_reset_time;

    // 需要同时满足：角度在0度附近连续3次，时间超过25秒，有足够的数据点
    if (near_zero_count >= 3 && time_since_reset.toSec() > 90.0f && sonar_points_.size() > 60 && !full_scan_complete_.load()) {
        full_scan_complete_.store(true);
        near_zero_count = 0;
        
        // === 静态建图简化版 ===
        static int total_scans = 0;                    // 总扫描次数
        static std::vector<int> green_counts;         // 有效扫描的绿色点云数
        static std::vector<nav_msgs::OccupancyGrid> scan_maps; // 有效扫描的地图
        
        total_scans++;
        
        // 统计绿色点云
        int green_points = 0;
        for (const auto& pt : sonar_points_) {
            for (size_t i = 0; i < pt.ranges.size(); i++) {
                if (pt.ranges[i] <= 3.0f && pt.intensities[i] >= 140.0f) {
                    green_points++;
                }
            }
        }
        
        ROS_INFO("【静态建图】第%d次扫描完成，耗时%.1f秒，绿色点云: %d", 
                total_scans, time_since_reset.toSec(), green_points);
        
        // 质量检查（放宽阈值，避免过度过滤）
        if (green_points >= 60) {  // 从60降低到40
            green_counts.push_back(green_points);
            scan_maps.push_back(occupancy_map_);
            
            // 保存阶段地图
            saveStageMap("scan_" + std::to_string(green_counts.size()));
            
            ROS_INFO("  有效扫描积累: %lu/3", green_counts.size());
            
            // 如果积累3次有效扫描，检查稳定性
            if (green_counts.size() >= 3) {
                // 简单稳定性检查（放宽条件）
                float avg = 0, min_val = FLT_MAX, max_val = 0;
                for (int count : green_counts) {
                    avg += count;
                    if (count < min_val) min_val = count;
                    if (count > max_val) max_val = count;
                }
                avg /= green_counts.size();
                float variation = (max_val - min_val) / avg;
                
                ROS_INFO("  稳定性检查: 平均=%.1f, 变化=%.1f%%", 
                        avg, variation * 100.0f);
                
                // 放宽稳定性条件
                if (variation < 0.35f && avg > 50) {  // 从0.25放宽到0.35，从50降低到30
                    // 简单合并地图（取最大值）
                    nav_msgs::OccupancyGrid final_map = scan_maps[0];
                    for (size_t i = 1; i < scan_maps.size(); i++) {
                        for (size_t j = 0; j < final_map.data.size(); j++) {
                            if (scan_maps[i].data[j] > final_map.data[j]) {
                                final_map.data[j] = scan_maps[i].data[j];
                            }
                        }
                    }
                    
                    // 发布完成信号
                    std_msgs::Bool done_msg;
                    done_msg.data = true;
                    static_map_done_pub_.publish(done_msg);
                    initial_occupancy_pub_.publish(final_map);
                    
                    ROS_INFO("✓✓✓ 静态建图完成！");
                    ROS_INFO("  总扫描次数: %d", total_scans);
                    ROS_INFO("  有效扫描次数: %lu", green_counts.size());
                    ROS_INFO("  绿色点云: %d, %d, %d", 
                            green_counts[0], green_counts[1], green_counts[2]);
                    
                    // 停止扫描
                    stopScanning();
                    
                    // 标记完成，防止重复处理
                    full_scan_complete_.store(true);
                    return; // 直接返回，不再执行后面的重置
                } else {
                    ROS_WARN("地图不稳定，需要继续扫描");
                    // 移除最早的一次扫描，继续积累新的
                    if (green_counts.size() > 0) {
                        green_counts.erase(green_counts.begin());
                        scan_maps.erase(scan_maps.begin());
                    }
                }
            }
        } else {
            ROS_WARN("扫描质量不佳，绿色点云不足: %d < 40", green_points);
        }
        
        // 重置为下一次扫描准备
        sonar_points_.clear();
        full_scan_complete_.store(false);
        last_angle = angle_deg;
        last_angle_reset_time = ros::Time::now();
    }
    // ==================== 【】数据累积和增强处理 ====================
    // 只有在有高质量点时才进行增强处理
    if (use_cnt > 0) {
        // 角度扇区索引 (每3度一个扇区)
        int sector_index = static_cast<int>(angle_deg / 3.0f);
        
        // 累积数据到扇区
        accumulateSectorData(sector_index, sonar_point);
        
        // 每积累一定数量扇区后生成更密集的点云
        if (shouldGenerateDenseCloud()) {
            generateDensePointCloud();
        }
    }

    checkAndPublishInitialMap();
}


void SonarRealtimeMapper::saveStageMap(const std::string& stage_name) {
    struct stat st;
    std::string maps_dir;
    
    try {
        // 1. 获取当前ROS包的路径
        std::string package_path = ros::package::getPath("sonar_slam");
        maps_dir = package_path + "/maps/";
    }
    catch (const ros::Exception& e) {
        // 回退方案：使用当前目录
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
            maps_dir = std::string(cwd) + "/sonar_maps/";
        } else {
            maps_dir = "/tmp/sonar_maps/";
        }
    }
    
    // 2. 确保目录存在
    if (stat(maps_dir.c_str(), &st) != 0) {
        mkdir(maps_dir.c_str(), 0755);
    }
    
    // 3. 生成时间戳
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    long timestamp_ms = now_ms.time_since_epoch().count();
    
    // 4. 【新增】保存点云数据
    std::string pcd_filename = maps_dir + stage_name + "_" + 
                              std::to_string(timestamp_ms) + ".pcd";
    
    if (!pointcloud_->empty()) {
        if (pcl::io::savePCDFileBinary(pcd_filename, *pointcloud_) == 0) {
            ROS_INFO("保存点云: %s (%lu 点)", pcd_filename.c_str(), pointcloud_->size());
        } else {
            ROS_ERROR("保存点云失败: %s", pcd_filename.c_str());
        }
    }
    
    // 5. 保存PGM地图
    std::string pgm_filename = maps_dir + stage_name + "_" + 
                              std::to_string(timestamp_ms) + ".pgm";
    
    std::ofstream pgm_file(pgm_filename);
    if (pgm_file.is_open()) {
        pgm_file << "P2\n";
        pgm_file << occupancy_map_.info.width << " " << occupancy_map_.info.height << "\n";
        pgm_file << "100\n";
        
        for (int y = occupancy_map_.info.height - 1; y >= 0; y--) {
            for (int x = 0; x < occupancy_map_.info.width; x++) {
                int value = occupancy_map_.data[y * occupancy_map_.info.width + x];
                if (value < 0) value = 50;
                pgm_file << value << " ";
            }
            pgm_file << "\n";
        }
        pgm_file.close();
    }
    
    // 6. 保存YAML配置（包含点云文件引用）
    std::string yaml_filename = maps_dir + stage_name + "_" + 
                               std::to_string(timestamp_ms) + ".yaml";
    
    std::ofstream yaml_file(yaml_filename);
    if (yaml_file.is_open()) {
        yaml_file << "image: " << pgm_filename << "\n";
        yaml_file << "resolution: " << occupancy_map_.info.resolution << "\n";
        yaml_file << "origin: [" << occupancy_map_.info.origin.position.x << ", "
                  << occupancy_map_.info.origin.position.y << ", 0.0]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.25\n";
        
        // 【新增】点云文件信息
        if (!pointcloud_->empty()) {
            yaml_file << "pointcloud: " << pcd_filename << "\n";
            yaml_file << "pointcloud_count: " << pointcloud_->size() << "\n";
        }
        
        // 统计信息
        int occupied = 0, free = 0, unknown = 0;
        for (int val : occupancy_map_.data) {
            if (val < 0) unknown++;
            else if (val == 0) free++;
            else if (val > 50) occupied++;
        }
        
        yaml_file << "\n# Statistics\n";
        yaml_file << "statistics:\n";
        yaml_file << "  occupied_cells: " << occupied << "\n";
        yaml_file << "  free_cells: " << free << "\n";
        yaml_file << "  unknown_cells: " << unknown << "\n";
        
        yaml_file.close();
        ROS_INFO("保存地图: %s", yaml_filename.c_str());
    }
}

bool SonarRealtimeMapper::saveMapAsPGM(const std::string& filename, 
                                       const nav_msgs::OccupancyGrid& map) {
    std::ofstream pgm_file(filename);
    if (!pgm_file.is_open()) {
        ROS_ERROR("无法打开PGM文件: %s", filename.c_str());
        return false;
    }
    
    // PGM文件头
    pgm_file << "P2\n";
    pgm_file << "# Sonar SLAM Map - " << filename << "\n";
    pgm_file << map.info.width << " " << map.info.height << "\n";
    pgm_file << "100\n";
    
    // 地图数据（行倒序，因为PGM是左下角原点）
    for (int y = map.info.height - 1; y >= 0; y--) {
        for (int x = 0; x < map.info.width; x++) {
            int value = map.data[y * map.info.width + x];
            
            // 转换：未知(-1)->50, 自由(0)->0, 占据(100)->100
            if (value < 0) {
                value = 50;  // 未知区域设为中间灰度
            }
            
            pgm_file << value;
            
            // 每行末尾不加空格
            if (x < map.info.width - 1) {
                pgm_file << " ";
            }
        }
        pgm_file << "\n";
    }
    
    pgm_file.close();
    return true;
}

bool SonarRealtimeMapper::saveMapAsYAML(const std::string& filename,
                                        const std::string& pgm_file,
                                        const nav_msgs::OccupancyGrid& map) {
    std::ofstream yaml_file(filename);
    if (!yaml_file.is_open()) {
        ROS_ERROR("无法打开YAML文件: %s", filename.c_str());
        return false;
    }
    
    // 计算统计信息
    int occupied = 0, free = 0, unknown = 0;
    for (int val : map.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }
    
    // YAML内容
    yaml_file << "# Sonar SLAM Map Configuration\n";
    yaml_file << "# Generated at: " << ros::Time::now() << "\n";
    yaml_file << "# ROS Time: " << ros::Time::now().toSec() << "\n\n";
    
    yaml_file << "image: " << pgm_file << "\n";
    yaml_file << "resolution: " << map.info.resolution << "\n";
    yaml_file << "origin: [" << map.info.origin.position.x << ", "
              << map.info.origin.position.y << ", 0.0]\n";
    yaml_file << "negate: 0\n";
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.25\n\n";
    
    // 元数据
    yaml_file << "# Metadata\n";
    yaml_file << "metadata:\n";
    yaml_file << "  width: " << map.info.width << "\n";
    yaml_file << "  height: " << map.info.height << "\n";
    yaml_file << "  timestamp: " << ros::Time::now().toSec() << "\n";
    yaml_file << "  frame_id: \"" << map.header.frame_id << "\"\n\n";
    
    // 统计信息
    yaml_file << "# Statistics\n";
    yaml_file << "statistics:\n";
    yaml_file << "  occupied_cells: " << occupied << "\n";
    yaml_file << "  free_cells: " << free << "\n";
    yaml_file << "  unknown_cells: " << unknown << "\n";
    yaml_file << "  total_cells: " << map.data.size() << "\n";
    yaml_file << "  occupied_percentage: " 
              << (occupied * 100.0 / map.data.size()) << "%\n";
    yaml_file << "  free_percentage: " 
              << (free * 100.0 / map.data.size()) << "%\n";
    yaml_file << "  unknown_percentage: " 
              << (unknown * 100.0 / map.data.size()) << "%\n";
    
    yaml_file.close();
    return true;
}

// 函数：累积扇区数据
void SonarRealtimeMapper::accumulateSectorData(int sector_index, const SonarPoint& point) {
    // 确保扇区容器足够大
    if (sector_data_.size() <= sector_index) {
        sector_data_.resize(sector_index + 1);
    }
    
    // 累积该角度的点
    auto& sector = sector_data_[sector_index];
    for (size_t i = 0; i < point.ranges.size(); i++) {
        sector.range_accumulator[static_cast<int>(point.ranges[i] * 10)]++;  // 0.1m精度
        sector.intensity_accumulator.push_back(point.intensities[i]);
    }
    sector.timestamps.push_back(point.timestamp);
    
    // 保持历史数据大小合理
    if (sector.intensity_accumulator.size() > 1000) {
        sector.intensity_accumulator.erase(sector.intensity_accumulator.begin(), 
                                          sector.intensity_accumulator.begin() + 500);
        sector.range_accumulator.clear();  // 重建范围统计
        for (const auto& intensity : sector.intensity_accumulator) {
            // 重新计算范围分布（简化）
        }
    }
}

// 函数：生成更密集的点云
void SonarRealtimeMapper::generateDensePointCloud() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr dense_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    for (size_t sector_idx = 0; sector_idx < sector_data_.size(); sector_idx++) {
        const auto& sector = sector_data_[sector_idx];
        if (sector.intensity_accumulator.empty()) continue;
        
        float angle_deg = sector_idx * 3.0f;
        float angle_rad = angle_deg * DEG_TO_RAD;
        
        // 计算该扇区的统计信息
        float avg_intensity = 0;
        float max_intensity = 0;
        for (float intensity : sector.intensity_accumulator) {
            avg_intensity += intensity;
            if (intensity > max_intensity) max_intensity = intensity;
        }
        avg_intensity /= sector.intensity_accumulator.size();
        
        // 生成多个距离点的点
        for (const auto& range_pair : sector.range_accumulator) {
            float range = range_pair.first * 0.1f;  // 恢复为米
            int count = range_pair.second;
            
            // 过滤：只有多次检测到的距离才认为是可靠的
            if (count > 3 && avg_intensity > min_intensity_) {
                // 基础点
                pcl::PointXYZI point;
                point.x = range * cosf(angle_rad);
                point.y = range * sinf(angle_rad);
                point.z = 0.0f;
                point.intensity = avg_intensity;
                dense_cloud->points.push_back(point);
                
                // 在周围生成一些点以填充墙壁
                if (max_intensity > 150.0f) {  // 强信号，很可能是墙壁
                    float spread = 0.1f;  // 10cm 扩散
                    for (float dr = -spread; dr <= spread; dr += 0.02f) {
                        for (float da = -0.05f; da <= 0.05f; da += 0.01f) {
                            pcl::PointXYZI fill_point;
                            float fill_range = range + dr;
                            float fill_angle = angle_rad + da;
                            fill_point.x = fill_range * cosf(fill_angle);
                            fill_point.y = fill_range * sinf(fill_angle);
                            fill_point.z = 0.0f;
                            fill_point.intensity = avg_intensity * 0.8f;  // 稍弱强度
                            dense_cloud->points.push_back(fill_point);
                        }
                    }
                }
            }
        }
    }
    
    // 应用滤波
    if (!dense_cloud->empty()) {
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setInputCloud(dense_cloud);
        voxel.setLeafSize(0.03f, 0.03f, 0.03f);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        voxel.filter(*filtered);
        
        // 统计滤波去除噪声
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(filtered);
        sor.setMeanK(20);
        sor.setStddevMulThresh(1.0);
        sor.filter(*dense_cloud);
        
        // 发布密集点云
        sensor_msgs::PointCloud2 dense_msg;
        pcl::toROSMsg(*dense_cloud, dense_msg);
        dense_msg.header.stamp = ros::Time::now();
        dense_msg.header.frame_id = "sonar_frame";
        dense_pointcloud_pub_.publish(dense_msg);   

    }
}

// ======================== 生成控制命令 ========================

void SonarRealtimeMapper::generateControlCommand(unsigned char* buffer, unsigned short angle) {
    // 清零缓冲区
    memset(buffer, 0, 24);
    
    // 消息头 - 不变
    buffer[0] = 0x42;  // 'B'
    buffer[1] = 0x52;  // 'R'
    buffer[2] = 0x0E;  // 长度低位 (14)
    buffer[3] = 0x00;  // 长度高位
    buffer[4] = 0x29;  // 命令低位 (41)
    buffer[5] = 0x0A;  // 命令高位 (10)
    
    // 源地址和目的地址 - 不变
    buffer[6] = 0x00;  // 源地址
    buffer[7] = 0x00;  // 目的地址
    
    // 模式设置 - 不变
    buffer[8] = 0x01;  // 模式
    buffer[9] = 0x00;  // 增益设置
    
    // 角度（小端序）- 不变
    buffer[10] = angle & 0xFF;         // 角度低位
    buffer[11] = (angle >> 8) & 0xFF;  // 角度高位
    
    // ========== 【只需要修改这3个参数】 ==========
    // 发射持续时间 (107us 而不是200us)
    buffer[12] = 0x6B;  // 107 (参考代码: 0x6B)
    buffer[13] = 0x00;
    
    // 采样周期 (888 而不是100) - 最关键！
    buffer[14] = 0x78;  // 120 (888的低字节: 0x0378)
    buffer[15] = 0x03;  // 3   (888的高字节)
    
    // 发射频率 - 不变
    buffer[16] = 0xEE;  // 750
    buffer[17] = 0x02;
    
    // 采样点数 - 不变
    buffer[18] = 0xB0;  // 1200
    buffer[19] = 0x04;
    
    // 数据长度 (1 而不是2)
    buffer[20] = 0x01;  // 1 (参考代码: 0x01)
    buffer[21] = 0x00;
    
    // 计算校验和
    uint16_t checksum = 0;
    for (int i = 0; i < 22; i++) {
        checksum += buffer[i];
    }
    
    buffer[22] = checksum & 0xFF;         // 校验和低位
    buffer[23] = (checksum >> 8) & 0xFF;  // 校验和高位
    
    // 【】添加调试输出
    static bool first_run = true;
    if (first_run) {
        ROS_INFO("控制命令参数调整:");
        ROS_INFO("  采样周期: %u -> 888", (buffer[15] << 8) | buffer[14]);
        ROS_INFO("  发射持续时间: %uus -> 107us", (buffer[13] << 8) | buffer[12]);
        ROS_INFO("  数据长度: %u -> 1", (buffer[21] << 8) | buffer[20]);
        first_run = false;
    }
}

// ======================== 扫描线程 ========================

void SonarRealtimeMapper::scanningThread() {
    ROS_INFO("扫描线程启动");
    
    unsigned short angle = 0;
    scan_start_time_ = ros::Time::now();
    
    while (!stop_requested_.load() && ros::ok()) {
        // 生成并发送控制命令
        unsigned char cmd_buffer[24];
        generateControlCommand(cmd_buffer, angle);
        
        if (sonar_serial_) {
            try {
                sonar_serial_->write(reinterpret_cast<char*>(cmd_buffer), 24);
                ROS_DEBUG("发送控制命令: 角度=%u", angle);
            }
            catch (const std::exception& e) {
                ROS_ERROR("发送命令失败: %s", e.what());
            }
        }
        
        // 更新当前角度
        current_angle_.store(angle);
        
        // 角度递增（每次增加2，约1.8度）
        angle += 1;
        
        // 检查是否完成360度扫描
        if (angle >= 400) {
            angle = 0;
            full_scan_complete_.store(true);
            
            ros::Duration scan_duration = ros::Time::now() - scan_start_time_;
            ROS_INFO("完成360度扫描! 耗时: %.1f 秒, 总帧数: %u", 
                    scan_duration.toSec(), total_scans_);
            
            // 保存完整扫描
            saveScanData("/tmp/full_scan_" + std::to_string(time(nullptr)));
            saveCostMap("/tmp/cost_map_" + std::to_string(time(nullptr)));
            
            scan_start_time_ = ros::Time::now();
            total_scans_ = 0;
            valid_scans_ = 0;
        }
        
        // 控制扫描频率（约5Hz）
        ros::Duration(0.2).sleep();
    }
    
    ROS_INFO("扫描线程停止");
}

// ======================== 点云转换 ========================
void SonarRealtimeMapper::convertToPointCloud() {
    pointcloud_->clear();
   
    // 旋转角度参数（与参考代码一致：-45度）
    const float rotation_angle_deg = -45.0f;
    const float rotation_angle_rad = rotation_angle_deg * DEG_TO_RAD;
 
    
    // 添加调试：显示转换前的原始角度
    for (const auto& sonar_point : sonar_points_) {
        float original_angle_deg = sonar_point.angle * RAD_TO_DEG;
        ROS_INFO_THROTTLE(5.0, "原始角度: %.1f°, 点数: %lu", 
                         original_angle_deg, sonar_point.ranges.size());
    }

    const float cos_theta = cosf(rotation_angle_rad);
    const float sin_theta = sinf(rotation_angle_rad);
    
    // 调试：输出前几个点的变换信息
    static int debug_point_count = 0;
    static const int MAX_DEBUG_POINTS = 10;
    
    for (const auto& sonar_point : sonar_points_) {
        float original_angle_deg = sonar_point.angle * RAD_TO_DEG;
        
        for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
            float range = sonar_point.ranges[i];
            float intensity = sonar_point.intensities[i];
            
            // 1. 计算原始坐标（按照声纳原始坐标系）
            float xx = range * cosf(sonar_point.angle);
            float yy = range * sinf(sonar_point.angle);
            
            // 2. 应用-45度旋转（与参考代码完全一致）
            pcl::PointXYZI point;
            point.x = cos_theta * xx - sin_theta * yy;
            point.y = sin_theta * xx + cos_theta * yy;
            point.z = 0.0f;
            point.intensity = intensity;
           
            pointcloud_->points.push_back(point);
            
            // 调试输出
            if (debug_point_count < MAX_DEBUG_POINTS) {
                ROS_INFO("点云点 %d: 角度=%.1f°, 距离=%.2fm, 原始坐标=(%.2f,%.2f), 旋转后坐标=(%.2f,%.2f)", 
                        debug_point_count, original_angle_deg, range, xx, yy, point.x, point.y);
                debug_point_count++;
            }
        }
    }
   
    pointcloud_->width = pointcloud_->points.size();
    pointcloud_->height = 1;
    pointcloud_->is_dense = false;
   
    pcl_conversions::toPCL(ros::Time::now(), pointcloud_->header.stamp);
    
    // 在函数末尾添加方向标记（不函数）
    static ros::Time last_marker_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    if ((now - last_marker_time).toSec() > 2.0) {
        // 前方标记（红色）
        visualization_msgs::Marker front_marker;
        front_marker.header.frame_id = "sonar_frame";
        front_marker.header.stamp = now;
        front_marker.ns = "directions";
        front_marker.id = 0;
        front_marker.type = visualization_msgs::Marker::ARROW;
        front_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point start, end;
        start.x = 0; start.y = 0; start.z = 0;
        end.x = 2.0; end.y = 0; end.z = 0;  // 前方
        
        front_marker.points.push_back(start);
        front_marker.points.push_back(end);
        front_marker.scale.x = 0.05;
        front_marker.scale.y = 0.1;
        front_marker.color.r = 1.0;
        front_marker.color.g = 0.0;
        front_marker.color.b = 0.0;
        front_marker.color.a = 1.0;
        
        marker_pub_.publish(front_marker);
        
        // 右侧标记（绿色）
        visualization_msgs::Marker right_marker = front_marker;
        right_marker.id = 1;
        right_marker.color.r = 0.0;
        right_marker.color.g = 1.0;
        end.x = 0; end.y = 2.0;  // 右侧
        right_marker.points.clear();
        right_marker.points.push_back(start);
        right_marker.points.push_back(end);
        
        marker_pub_.publish(right_marker);
        
        last_marker_time = now;
        
        ROS_DEBUG("发布方向标记：红色=前方，绿色=右侧");
    }
}

// ======================== 点云滤波 ========================
void SonarRealtimeMapper::filterPointCloud() {
    if (pointcloud_->empty()) {
        return;
    }
    
    // 复制点云
    *filtered_pointcloud_ = *pointcloud_;
    
    // 体素网格滤波（降采样）
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(filtered_pointcloud_);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
    
    PointCloudIPtr temp_cloud(new PointCloudI());
    voxel_filter.filter(*temp_cloud);
    
    // 半径离群点移除
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_filter;
    radius_filter.setInputCloud(temp_cloud);
    radius_filter.setRadiusSearch(0.1f);
    radius_filter.setMinNeighborsInRadius(5);
    
    radius_filter.filter(*filtered_pointcloud_);
    
    ROS_DEBUG("点云滤波: 原始点数=%lu, 滤波后点数=%lu", 
             pointcloud_->size(), filtered_pointcloud_->size());
}

// ======================== 创建占据栅格地图 ========================
/*****
void SonarRealtimeMapper::createOccupancyGrid() {
    // 重置地图数据为未知
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    // 更新时间戳
    occupancy_map_.header.stamp = ros::Time::now();
    
    // 【关键修改】: 只使用原始点云
    if (pointcloud_->empty()) {  
        ROS_DEBUG_THROTTLE(2.0, "原始点云为空，跳过地图更新");
        return;
    }
    // 【关键添加】: 简单统计
    int total_points = 0;
    int ne = 0, nw = 0, sw = 0, se = 0;
    
    for (const auto& point : pointcloud_->points) {
        total_points++;
        
        // 象限统计
        if (point.x >= 0 && point.y >= 0) ne++;
        else if (point.x < 0 && point.y >= 0) nw++;
        else if (point.x < 0 && point.y < 0) sw++;
        else se++;
        
        // 地图更新
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);
        
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            occupancy_map_.data[map_y * map_width_ + map_x] = 100;
        }
    }
    
    // 【关键】: 只输出最重要的信息
    ROS_INFO("地图构建: %d点, 象限分布: NE=%d, NW=%d, SW=%d, SE=%d", 
             total_points, ne, nw, sw, se);
    
    // 如果某个象限完全没数据，可能是扫描问题
    if (ne == 0 || nw == 0 || sw == 0 || se == 0) {
        ROS_WARN("有象限无数据！检查扫描是否完整");
    }
    // 简单统计
    int points_added = 0;
    int points_out_of_bounds = 0;
    
    // 直接处理每个点
    for (const auto& point : pointcloud_->points) {
        // 计算地图坐标
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
        
        // 边界检查
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            // 直接标记为占据
            int index = map_y * map_width_ + map_x;
            occupancy_map_.data[index] = 100;
            points_added++;
            
            // 【】: 使墙壁更连续，标记相邻单元格
            if (map_x > 0) occupancy_map_.data[map_y * map_width_ + (map_x-1)] = 100;
            if (map_x < map_width_-1) occupancy_map_.data[map_y * map_width_ + (map_x+1)] = 100;
            if (map_y > 0) occupancy_map_.data[(map_y-1) * map_width_ + map_x] = 100;
            if (map_y < map_height_-1) occupancy_map_.data[(map_y+1) * map_width_ + map_x] = 100;
        } else {
            points_out_of_bounds++;
        }
    }
    
    // 简单调试输出
    ROS_INFO_THROTTLE(5.0, "地图更新: 添加 %d 个点，%d 个点超出边界", 
                     points_added, points_out_of_bounds);
}
***/


void SonarRealtimeMapper::createOccupancyGrid() {
    // 重置地图数据为未知
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    if (pointcloud_->empty()) {
        ROS_DEBUG_THROTTLE(2.0, "点云为空，跳过地图更新");
        return;
    }
    
    // ==================== 【核心修改】: 水池边界检测 ====================
    // 简单版本: 按角度统计最近的有效点
    std::vector<float> min_dist_per_angle(360, FLT_MAX);
    std::vector<float> max_intensity_per_angle(360, 0);
    
    for (const auto& point : pointcloud_->points) {
        float distance = sqrt(point.x * point.x + point.y * point.y);
        float angle = atan2(point.y, point.x) * RAD_TO_DEG;
        if (angle < 0) angle += 360.0f;
        
        int angle_idx = static_cast<int>(angle) % 360;
        
        // 只考虑质量较高的点
        if (point.intensity >= 140.0f && distance <= 5.0f) {
            if (distance < min_dist_per_angle[angle_idx]) {
                min_dist_per_angle[angle_idx] = distance;
                max_intensity_per_angle[angle_idx] = point.intensity;
            }
        }
    }
    
    // ==================== 根据水池边界过滤点云 ====================
    int inside_pool_points = 0;
    int outside_pool_points = 0;
    
    for (const auto& point : pointcloud_->points) {
        float distance = sqrt(point.x * point.x + point.y * point.y);
        float angle = atan2(point.y, point.x) * RAD_TO_DEG;
        if (angle < 0) angle += 360.0f;
        
        int angle_idx = static_cast<int>(angle) % 360;
        float pool_boundary = min_dist_per_angle[angle_idx];
        
        // 判断是否在水池内
        bool keep_point = true;
        
        // 规则1: 如果有明确的边界，且点远于边界+0.5m，过滤掉
        if (pool_boundary < FLT_MAX - 1.0f) {
            if (distance > pool_boundary + 0.5f) {
                // 可能是水池外的噪声
                if (point.intensity < 160.0f) {  // 强度不够高，很可能是噪声
                    outside_pool_points++;
                    keep_point = false;
                }
            } else {
                inside_pool_points++;
            }
        }
        
        // 规则2: 特别过滤近距离噪声（0.3米内但强度很低）
        if (distance < 0.3f && point.intensity < 120.0f) {
            keep_point = false;
            outside_pool_points++;
        }
        
        // 保留的点
        if (keep_point) {
            int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
            int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
            
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                int index = map_y * map_width_ + map_x;
                
                // 根据强度设置占据值
                int occupancy_value = 0;
                if (point.intensity >= 160.0f) {
                    occupancy_value = 100;  // 墙壁，高置信度
                } else if (point.intensity >= 140.0f) {
                    occupancy_value = 80;   // 疑似墙壁
                } else if (point.intensity >= 120.0f) {
                    occupancy_value = 60;   // 可能的障碍物
                } else {
                    occupancy_value = 40;   // 低置信度
                }
                
                // 保留最高值
                if (occupancy_value > occupancy_map_.data[index]) {
                    occupancy_map_.data[index] = occupancy_value;
                }
                
                // 对于高质量点，增强连续性（墙壁效果）
                if (point.intensity >= 160.0f) {
                    // 简单的3x3填充
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int nx = map_x + dx;
                            int ny = map_y + dy;
                            
                            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                                int nidx = ny * map_width_ + nx;
                                int neighbor_value = occupancy_value * 0.7f;
                                if (neighbor_value > occupancy_map_.data[nidx]) {
                                    occupancy_map_.data[nidx] = neighbor_value;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    // ==================== 可选: 填充水池内部为自由空间 ====================
    // 简单方法: 原点附近标记为自由
    int robot_x = static_cast<int>((0.0 - map_origin_x_) / map_resolution_);
    int robot_y = static_cast<int>((0.0 - map_origin_y_) / map_resolution_);
    
    int free_radius = static_cast<int>(2.0 / map_resolution_);  // 2米半径
    
    for (int y = robot_y - free_radius; y <= robot_y + free_radius; y++) {
        for (int x = robot_x - free_radius; x <= robot_x + free_radius; x++) {
            if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) {
                int idx = y * map_width_ + x;
                float dist_to_robot = sqrt(pow(x - robot_x, 2) + pow(y - robot_y, 2)) * map_resolution_;
                
                // 靠近机器人且未被占据的区域标记为自由
                if (dist_to_robot < 1.5f && occupancy_map_.data[idx] < 30) {
                    occupancy_map_.data[idx] = 0;
                }
            }
        }
    }
    
    // 更新时间戳
    occupancy_map_.header.stamp = ros::Time::now();
    
    // 调试信息
    ROS_INFO_THROTTLE(5.0, "地图更新: 保留池内点=%d, 过滤池外点=%d", 
                     inside_pool_points, outside_pool_points);
    
    // 输出边界统计
    int valid_boundary_angles = 0;
    for (int i = 0; i < 360; i++) {
        if (min_dist_per_angle[i] < FLT_MAX - 1.0f) {
            valid_boundary_angles++;
        }
    }
    ROS_DEBUG_THROTTLE(10.0, "有效边界角度: %d/360", valid_boundary_angles);
}
// ======================== 创建代价地图 ========================
void SonarRealtimeMapper::createCostMap() {
    // 重置代价地图数据
    std::fill(cost_map_.data.begin(), cost_map_.data.end(), -1);
    
    // 更新时间戳
    cost_map_.header.stamp = ros::Time::now();
    
    if (filtered_pointcloud_->empty()) {
        ROS_DEBUG_THROTTLE(1.0, "点云为空，跳过代价地图更新");
        return;
    }
    
    ROS_DEBUG_THROTTLE(2.0, "更新代价地图，输入点云: %lu 点", filtered_pointcloud_->size());
    
    // ==================== 第一步：提取轮廓点 ====================
    // 我们将角度空间分成若干个扇形区域，只保留每个区域内最强的点作为轮廓
    std::map<int, pcl::PointXYZI> sector_contour_points; // 键: 角度扇区索引, 值: 该扇区最强的点
    const float angle_bin_size = 2.0f * DEG_TO_RAD; // 每2度一个扇区（可调整）
    
    for (const auto& point : filtered_pointcloud_->points) {
        // 计算点的角度和距离
        float angle = atan2(point.y, point.x); // 弧度，范围 -π 到 π
        float distance = sqrt(point.x * point.x + point.y * point.y);
        
        // 将角度映射到 [0, 2π) 然后归一化到扇区索引
        float normalized_angle = angle + M_PI; // 转换为 0 到 2π
        int sector_index = static_cast<int>(normalized_angle / angle_bin_size);
        
        // 每个扇区只保留信号最强的点（最可能是墙壁）
        auto it = sector_contour_points.find(sector_index);
        if (it == sector_contour_points.end() || point.intensity > it->second.intensity) {
            sector_contour_points[sector_index] = point;
        }
    }
    
    ROS_DEBUG_THROTTLE(2.0, "提取到 %lu 个扇区轮廓点", sector_contour_points.size());

    // ==================== ：墙壁连续性和端检测 ====================
    std::vector<WallSegment> wall_segments;
    if (!filtered_pointcloud_->empty()) {
        // 第一步：提取可能的墙壁点（高强度，距离一致）
        extractWallSegments(wall_segments);
        
        if (!wall_segments.empty()) {
            ROS_DEBUG("检测到 %lu 个墙壁段", wall_segments.size());
            
            // 第二步：连接相邻的墙壁段
            connectWallSegments(wall_segments);
            
            // 第三步：在墙壁段之间填充
            fillWallGaps(wall_segments);
            
            // 第四步：检测水池角落
            detectPoolCorners(wall_segments);
            
            // 第五步：将墙壁段点添加到轮廓点中，增强墙壁连续性
            addWallSegmentsToContourPoints(wall_segments, sector_contour_points);
        }
    }
    
    // ==================== 第二步：用轮廓点更新代价地图 ====================
    int contour_points_used = 0;
    for (const auto& pair : sector_contour_points) {
        const auto& point = pair.second;
        
        // 计算代价地图坐标
        int costmap_x = static_cast<int>((point.x - costmap_origin_x_) / costmap_resolution_);
        int costmap_y = static_cast<int>((point.y - costmap_origin_y_) / costmap_resolution_);
        
        if (costmap_x >= 0 && costmap_x < costmap_width_ && 
            costmap_y >= 0 && costmap_y < costmap_height_) {
            
            int index = costmap_y * costmap_width_ + costmap_x;
            
            // 更新代价单元 - 对轮廓点给予更高的权重
            CostCell& cell = cost_cells_[index];
            
            // 轮廓点比普通点有更大的影响
            // 击中3次（而不是1次），占据概率增加更多
            cell.hits += 3;
            cell.occupancy = std::min(1.0f, cell.occupancy + 0.3f);
            cell.cost = cell.occupancy * cell.occupancy;
            
            // 根据占据概率设置代价值（0-100）
            int cost_value = static_cast<int>(cell.cost * 100.0f);
            cost_map_.data[index] = std::min(100, std::max(0, cost_value));
            
            contour_points_used++;
            
            // ==================== 第三步：增强墙壁连续性（小范围） ====================
            // 在轮廓点周围小范围内填充，使墙壁更连续
            int fill_radius = 1; // 填充半径（网格数）
            for (int dy = -fill_radius; dy <= fill_radius; dy++) {
                for (int dx = -fill_radius; dx <= fill_radius; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = costmap_x + dx;
                    int ny = costmap_y + dy;
                    
                    if (nx >= 0 && nx < costmap_width_ && ny >= 0 && ny < costmap_height_) {
                        int nidx = ny * costmap_width_ + nx;
                        // 邻居点的代价设为当前点的70%
                        int neighbor_cost = static_cast<int>(cost_value * 0.7f);
                        if (cost_map_.data[nidx] < neighbor_cost) {
                            cost_map_.data[nidx] = neighbor_cost;
                        }
                    }
                }
            }
        }
    }
    
    ROS_DEBUG_THROTTLE(2.0, "使用 %d/%lu 个轮廓点更新代价地图", 
                      contour_points_used, sector_contour_points.size());
    
    // ==================== 第四步：处理自由空间（基于机器人位置） ====================
    // 假设机器人位于地图中心 (0, 0)，从中心向各个方向标记自由空间
    int robot_costmap_x = static_cast<int>((0.0 - costmap_origin_x_) / costmap_resolution_);
    int robot_costmap_y = static_cast<int>((0.0 - costmap_origin_y_) / costmap_resolution_);
    
    if (robot_costmap_x >= 0 && robot_costmap_x < costmap_width_ && 
        robot_costmap_y >= 0 && robot_costmap_y < costmap_height_) {
        
        // 从机器人位置向外标记自由空间
        // 这里简化处理：如果某个单元格代价很低且靠近机器人，标记为自由
        int free_radius = static_cast<int>(2.0 / costmap_resolution_); // 2米半径内
        for (int y = robot_costmap_y - free_radius; y <= robot_costmap_y + free_radius; y++) {
            for (int x = robot_costmap_x - free_radius; x <= robot_costmap_x + free_radius; x++) {
                if (x >= 0 && x < costmap_width_ && y >= 0 && y < costmap_height_) {
                    int idx = y * costmap_width_ + x;
                    float distance_to_robot = sqrt(pow(x - robot_costmap_x, 2) + pow(y - robot_costmap_y, 2)) * costmap_resolution_;
                    
                    // 如果距离机器人较近且代价不高，标记为自由
                    if (distance_to_robot < 1.5f && cost_map_.data[idx] < 30) {
                        cost_map_.data[idx] = 0;
                    }
                }
            }
        }
    }
    
    // ==================== 第五步：后处理 - 平滑与降噪 ====================
    // ：对代价地图进行平滑处理，消除孤立的噪声点
    std::vector<int8_t> smoothed_data = cost_map_.data;
    for (int y = 1; y < costmap_height_ - 1; y++) {
        for (int x = 1; x < costmap_width_ - 1; x++) {
            int idx = y * costmap_width_ + x;
            
            // 如果当前点是占据的，但周围8个邻居大部分都是自由的，则可能是噪声
            if (cost_map_.data[idx] > 60) {
                int occupied_neighbors = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nidx = (y + dy) * costmap_width_ + (x + dx);
                        if (cost_map_.data[nidx] > 40) occupied_neighbors++;
                    }
                }
                
                // 如果周围少于2个占据的邻居，认为是噪声，降低其代价
                if (occupied_neighbors < 2) {
                    smoothed_data[idx] = std::max(0, cost_map_.data[idx] / 2);
                }
            }
        }
    }
    cost_map_.data = smoothed_data;
    
    // ==================== 第六步：统计与日志 ====================
    int occupied_cells = 0;
    int free_cells = 0;
    int unknown_cells = 0;
    
    for (int i = 0; i < costmap_width_ * costmap_height_; i++) {
        if (cost_map_.data[i] < 0) {
            unknown_cells++;
        } else if (cost_map_.data[i] == 0) {
            free_cells++;
        } else if (cost_map_.data[i] > 50) {
            occupied_cells++;
        }
    }
    
    ROS_DEBUG_THROTTLE(5.0, "代价地图统计: 占据=%d, 空闲=%d, 未知=%d", 
                      occupied_cells, free_cells, unknown_cells);
}


// 函数：提取墙壁段
void SonarRealtimeMapper::extractWallSegments(std::vector<WallSegment>& segments) {
    segments.clear();
    
    // 按角度排序的点
    std::map<int, std::vector<ContourPoint>> angle_points;  // 角度索引->点列表
    
    for (const auto& point : filtered_pointcloud_->points) {
        if (point.intensity > 150.0f) {  // 高强度点
            float angle = atan2(point.y, point.x);
            float angle_deg = angle * RAD_TO_DEG;
            if (angle_deg < 0) angle_deg += 360.0f;
            
            int angle_idx = static_cast<int>(angle_deg);
            angle_points[angle_idx].push_back({point.x, point.y, point.intensity});
        }
    }
    
    // 在相邻角度中寻找连续的墙壁
    WallSegment current_segment;
    bool in_segment = false;
    
    for (const auto& entry : angle_points) {
        int angle_idx = entry.first;
        const auto& points = entry.second;
        
        if (points.empty()) {
            if (in_segment) {
                segments.push_back(current_segment);
                current_segment = WallSegment();
                in_segment = false;
            }
            continue;
        }
        
        // 计算该角度的平均距离
        float avg_dist = 0;
        float max_intensity = 0;
        for (const auto& p : points) {
            avg_dist += sqrt(p.x * p.x + p.y * p.y);
            if (p.intensity > max_intensity) max_intensity = p.intensity;
        }
        avg_dist /= points.size();
        
        if (!in_segment) {
            current_segment.start_angle = angle_idx;
            current_segment.avg_distance = avg_dist;
            current_segment.max_intensity = max_intensity;
            current_segment.points = points;
            in_segment = true;
        } else {
            // 检查是否连续（距离差异小于0.5m，角度连续）
            if (abs(avg_dist - current_segment.avg_distance) < 0.5f && 
                angle_idx - current_segment.end_angle <= 5) {
                current_segment.end_angle = angle_idx;
                current_segment.avg_distance = (current_segment.avg_distance + avg_dist) / 2.0f;
                current_segment.max_intensity = std::max(current_segment.max_intensity, max_intensity);
                current_segment.points.insert(current_segment.points.end(), points.begin(), points.end());
            } else {
                segments.push_back(current_segment);
                current_segment = WallSegment();
                current_segment.start_angle = angle_idx;
                current_segment.avg_distance = avg_dist;
                current_segment.max_intensity = max_intensity;
                current_segment.points = points;
                in_segment = true;
            }
        }
        current_segment.end_angle = angle_idx;
    }
    
    if (in_segment) {
        segments.push_back(current_segment);
    }
    
    ROS_DEBUG("检测到 %lu 个墙壁段", segments.size());
}

// 连接墙壁段
void SonarRealtimeMapper::connectWallSegments(std::vector<WallSegment>& segments) {
    if (segments.size() < 2) return;
    
    std::sort(segments.begin(), segments.end(), 
        [](const WallSegment& a, const WallSegment& b) {
            return a.start_angle < b.start_angle;
        });
    
    // 尝试连接相邻的段
    for (size_t i = 0; i < segments.size() - 1; i++) {
        WallSegment& current = segments[i];
        WallSegment& next = segments[i + 1];
        
        // 检查是否可以连接（角度接近，距离相似）
        float angle_gap = next.start_angle - current.end_angle;
        float distance_diff = fabs(next.avg_distance - current.avg_distance);
        
        if (angle_gap <= 10 && distance_diff < 0.5f) { // 可调参数
            // 连接两个段
            current.end_angle = next.end_angle;
            current.avg_distance = (current.avg_distance + next.avg_distance) / 2.0f;
            current.max_intensity = std::max(current.max_intensity, next.max_intensity);
            current.points.insert(current.points.end(), 
                                 next.points.begin(), next.points.end());
            
            // 移除被合并的段
            segments.erase(segments.begin() + i + 1);
            i--; // 重新检查当前位置
        }
    }
}

// 将墙壁段添加到轮廓点
void SonarRealtimeMapper::addWallSegmentsToContourPoints(
    const std::vector<WallSegment>& segments,
    std::map<int, pcl::PointXYZI>& contour_points) {
    
    for (const auto& segment : segments) {
        for (const auto& point : segment.points) {
            float angle = atan2(point.y, point.x);
            float normalized_angle = angle + M_PI;
            const float angle_bin_size = 2.0f * DEG_TO_RAD;
            int sector_index = static_cast<int>(normalized_angle / angle_bin_size);
            
            // 创建点云点
            pcl::PointXYZI pcl_point;
            pcl_point.x = point.x;
            pcl_point.y = point.y;
            pcl_point.z = 0.0f;
            pcl_point.intensity = point.intensity;
            
            // 添加到轮廓点中（如果更强或不存在）
            auto it = contour_points.find(sector_index);
            if (it == contour_points.end() || pcl_point.intensity > it->second.intensity) {
                contour_points[sector_index] = pcl_point;
            }
        }
    }
}

// 填充墙壁间隙
void SonarRealtimeMapper::fillWallGaps(std::vector<WallSegment>& segments) {
    if (segments.size() < 2) return;
    
    // 按起始角度排序
    std::sort(segments.begin(), segments.end(), 
        [](const WallSegment& a, const WallSegment& b) {
            return a.start_angle < b.start_angle;
        });
    
    std::vector<WallSegment> filled_segments;
    
    for (size_t i = 0; i < segments.size(); i++) {
        filled_segments.push_back(segments[i]);
        
        // 检查与下一个段之间的间隙
        if (i < segments.size() - 1) {
            WallSegment& current = segments[i];
            WallSegment& next = segments[i + 1];
            
            int gap_size = next.start_angle - current.end_angle;
            
            // 如果间隙在合理范围内（5-30度），则填充
            if (gap_size > 5 && gap_size < 30) {
                float distance_diff = fabs(next.avg_distance - current.avg_distance);
                
                // 只有距离相似时才填充
                if (distance_diff < 0.8f) {
                    WallSegment gap_segment;
                    gap_segment.start_angle = current.end_angle;
                    gap_segment.end_angle = next.start_angle;
                    
                    // 插值距离和强度
                    gap_segment.avg_distance = (current.avg_distance + next.avg_distance) / 2.0f;
                    gap_segment.max_intensity = std::min(current.max_intensity, next.max_intensity) * 0.7f;
                    
                    // 在间隙中生成插值点
                    generateInterpolatedPoints(gap_segment, current, next);
                    
                    if (gap_segment.points.size() > 0) {
                        filled_segments.push_back(gap_segment);
                        ROS_DEBUG("填充墙壁间隙: %d度 -> %d度 (间隙%d度)", 
                                 current.end_angle, next.start_angle, gap_size);
                    }
                }
            }
        }
    }
    
    segments = filled_segments;
}

// 辅助函数：生成插值点
void SonarRealtimeMapper::generateInterpolatedPoints(
    WallSegment& gap_segment,
    const WallSegment& prev_segment,
    const WallSegment& next_segment) {
    
    if (gap_segment.start_angle >= gap_segment.end_angle) return;
    
    // 角度插值步长（每度一个点）
    float angle_step_deg = 1.0f;
    float angle_step_rad = angle_step_deg * DEG_TO_RAD;
    
    // 起始和结束角度（弧度）
    float start_angle_rad = gap_segment.start_angle * DEG_TO_RAD;
    float end_angle_rad = gap_segment.end_angle * DEG_TO_RAD;
    
    // 线性插值距离
    float start_distance = prev_segment.avg_distance;
    float end_distance = next_segment.avg_distance;
    float total_angles = gap_segment.end_angle - gap_segment.start_angle;
    
    for (int angle_idx = 0; angle_idx <= total_angles; angle_idx += static_cast<int>(angle_step_deg)) {
        float interp_angle_deg = gap_segment.start_angle + angle_idx;
        float t = static_cast<float>(angle_idx) / total_angles;
        
        // 线性插值距离
        float interp_distance = start_distance * (1.0f - t) + end_distance * t;
        
        // 线性插值强度（衰减）
        float interp_intensity = gap_segment.max_intensity * (1.0f - t * 0.5f);
        
        // 转换到笛卡尔坐标
        float interp_angle_rad = interp_angle_deg * DEG_TO_RAD;
        float x = interp_distance * cosf(interp_angle_rad);
        float y = interp_distance * sinf(interp_angle_rad);
        
        // 添加到间隙段
        gap_segment.points.push_back(ContourPoint(x, y, interp_intensity));
    }
    
    ROS_DEBUG("生成 %lu 个插值点填充间隙", gap_segment.points.size());
}


// 检测水池角落
void SonarRealtimeMapper::detectPoolCorners(const std::vector<WallSegment>& segments) {
    if (segments.size() < 2) return;
    
    std::vector<Eigen::Vector2f> detected_corners;
    
    // 分析相邻墙壁段的方向变化
    for (size_t i = 0; i < segments.size(); i++) {
        const WallSegment& current = segments[i];
        const WallSegment& next = segments[(i + 1) % segments.size()];
        
        // 计算两个段的平均方向向量
        Eigen::Vector2f current_dir = computeSegmentDirection(current);
        Eigen::Vector2f next_dir = computeSegmentDirection(next);
        
        // 计算连接点（当前段的终点）
        Eigen::Vector2f connection_point = computeSegmentEndPoint(current);
        
        // 计算方向变化角度
        float dot_product = current_dir.dot(next_dir);
        float angle_diff = acosf(std::max(-1.0f, std::min(1.0f, dot_product))) * RAD_TO_DEG;
        
        // 计算距离差异
        float distance_diff = fabs(current.avg_distance - next.avg_distance);
        
        // 角落检测条件：
        // 1. 方向变化大于30度（锐角角落）
        // 2. 或者方向变化在120-180度之间（钝角/内角）
        // 3. 距离不能相差太大
        bool is_sharp_corner = (angle_diff > 30.0f && angle_diff < 120.0f && distance_diff < 1.0f);
        bool is_inner_corner = (angle_diff > 120.0f && distance_diff < 0.5f);
        
        if (is_sharp_corner || is_inner_corner) {
            // 进一步验证：检查连接点附近是否有实际数据点支持
            if (validateCornerPoint(connection_point, segments)) {
                detected_corners.push_back(connection_point);
                
                ROS_DEBUG("检测到角落: 角度变化=%.1f°, 距离差异=%.2fm, 位置=(%.2f,%.2f)", 
                         angle_diff, distance_diff, 
                         connection_point.x(), connection_point.y());
            }
        }
    }
    
    // 聚类接近的角落点
    std::vector<Eigen::Vector2f> clustered_corners = clusterCorners(detected_corners);
    
    // 发布可视化标记
    visualizeCorners(clustered_corners);
    
    // 将角落点添加到代价地图
    addCornersToCostMap(clustered_corners);
    
    ROS_INFO("检测到 %lu 个水池角落点", clustered_corners.size());
}

// 辅助函数：计算墙壁段的方向向量
Eigen::Vector2f SonarRealtimeMapper::computeSegmentDirection(const WallSegment& segment) {
    if (segment.points.size() < 2) {
        return Eigen::Vector2f(1, 0); // 默认方向
    }
    
    // 使用RANSAC或最小二乘法拟合直线
    // 简化版：使用起始点和结束点的方向
    float start_angle_rad = segment.start_angle * DEG_TO_RAD;
    float end_angle_rad = segment.end_angle * DEG_TO_RAD;
    
    // 计算平均角度
    float mid_angle_rad = (start_angle_rad + end_angle_rad) / 2.0f;
    
    // 方向垂直于径向（墙壁的切线方向）
    float wall_direction_rad = mid_angle_rad + M_PI_2;
    
    return Eigen::Vector2f(cosf(wall_direction_rad), sinf(wall_direction_rad));
}

// 辅助函数：计算段的终点
Eigen::Vector2f SonarRealtimeMapper::computeSegmentEndPoint(const WallSegment& segment) {
    float end_angle_rad = segment.end_angle * DEG_TO_RAD;
    float x = segment.avg_distance * cosf(end_angle_rad);
    float y = segment.avg_distance * sinf(end_angle_rad);
    
    return Eigen::Vector2f(x, y);
}

// 辅助函数：验证角落点
bool SonarRealtimeMapper::validateCornerPoint(const Eigen::Vector2f& point, 
                                             const std::vector<WallSegment>& segments) {
    // 检查点附近是否有足够的实际数据点
    float search_radius = 0.2f; // 20cm搜索半径
    int nearby_points = 0;
    
    for (const auto& segment : segments) {
        for (const auto& contour_point : segment.points) {
            Eigen::Vector2f p(contour_point.x, contour_point.y);
            float distance = (p - point).norm();
            if (distance < search_radius) {
                nearby_points++;
            }
        }
    }
    
    // 还需要检查角落的内外特性
    bool is_valid_corner = (nearby_points >= 2);
    
    if (is_valid_corner) {
        // 进一步检查角落类型（凸角或凹角）
        // 通过分析周围点的分布来判断
        bool is_convex = analyzeCornerConvexity(point, segments);
        ROS_DEBUG("角落验证: 附近点=%d, 类型=%s", 
                 nearby_points, is_convex ? "凸角" : "凹角");
    }
    
    return is_valid_corner;
}

// 辅助函数：分析角落凸凹性
bool SonarRealtimeMapper::analyzeCornerConvexity(const Eigen::Vector2f& corner,
                                                const std::vector<WallSegment>& segments) {
    // 简化实现：检查角落周围点的分布
    // 在实际应用中，可能需要更复杂的几何分析
    
    // 收集角落周围1米范围内的点
    std::vector<Eigen::Vector2f> nearby_points;
    for (const auto& segment : segments) {
        for (const auto& contour_point : segment.points) {
            Eigen::Vector2f p(contour_point.x, contour_point.y);
            if ((p - corner).norm() < 1.0f) {
                nearby_points.push_back(p);
            }
        }
    }
    
    if (nearby_points.size() < 3) return true; // 默认凸角
    
    // 简单判断：如果大部分点都在角落的"外部"，则是凸角
    // 否则是凹角（水池内部的角落）
    int outside_count = 0;
    int inside_count = 0;
    
    for (const auto& p : nearby_points) {
        // 计算点到角落的角度
        Eigen::Vector2f dir = p - corner;
        float angle = atan2(dir.y(), dir.x());
        
        // 简单判断：如果点距离比预期的墙壁距离远，则认为是外部
        float expected_wall_distance = corner.norm(); // 简化
        float actual_distance = p.norm();
        
        if (actual_distance > expected_wall_distance + 0.1f) {
            outside_count++;
        } else {
            inside_count++;
        }
    }
    
    return outside_count > inside_count;
}

// 辅助函数：聚类接近的角落点
std::vector<Eigen::Vector2f> SonarRealtimeMapper::clusterCorners(
    const std::vector<Eigen::Vector2f>& corners) {
    
    if (corners.empty()) return {};
    
    std::vector<Eigen::Vector2f> clustered;
    std::vector<bool> visited(corners.size(), false);
    
    float cluster_radius = 0.3f; // 30cm内的点视为同一角落
    
    for (size_t i = 0; i < corners.size(); i++) {
        if (visited[i]) continue;
        
        visited[i] = true;
        Eigen::Vector2f cluster_center = corners[i];
        int cluster_size = 1;
        
        // 查找邻近点
        for (size_t j = i + 1; j < corners.size(); j++) {
            if (visited[j]) continue;
            
            float distance = (corners[j] - corners[i]).norm();
            if (distance < cluster_radius) {
                visited[j] = true;
                cluster_center += corners[j];
                cluster_size++;
            }
        }
        
        // 计算聚类中心
        cluster_center /= static_cast<float>(cluster_size);
        clustered.push_back(cluster_center);
    }
    
    return clustered;
}

// 辅助函数：可视化角落点
void SonarRealtimeMapper::visualizeCorners(const std::vector<Eigen::Vector2f>& corners) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "sonar_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pool_corners";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15; // 15cm方块
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;  // 红色表示角落
    marker.color.g = 1.0;  // 黄色
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    for (const auto& corner : corners) {
        geometry_msgs::Point p;
        p.x = corner.x();
        p.y = corner.y();
        p.z = 0.0;
        marker.points.push_back(p);
    }
    
    // 需要先添加corner_marker_pub_发布器到类中
     corner_marker_pub_.publish(marker); 
}

// 辅助函数：将角落点添加到代价地图
void SonarRealtimeMapper::addCornersToCostMap(const std::vector<Eigen::Vector2f>& corners) {
    for (const auto& corner : corners) {
        int costmap_x = static_cast<int>((corner.x() - costmap_origin_x_) / costmap_resolution_);
        int costmap_y = static_cast<int>((corner.y() - costmap_origin_y_) / costmap_resolution_);
        
        if (costmap_x >= 0 && costmap_x < costmap_width_ && 
            costmap_y >= 0 && costmap_y < costmap_height_) {
            
            int idx = costmap_y * costmap_width_ + costmap_x;
            
            // 角落点给予更高的代价（更确定是障碍物）
            cost_map_.data[idx] = 100; // 最大值
            
            // 在周围也设置较高代价
            int spread = 2;
            for (int dy = -spread; dy <= spread; dy++) {
                for (int dx = -spread; dx <= spread; dx++) {
                    int nx = costmap_x + dx;
                    int ny = costmap_y + dy;
                    
                    if (nx >= 0 && nx < costmap_width_ && ny >= 0 && ny < costmap_height_) {
                        int nidx = ny * costmap_width_ + nx;
                        int distance = abs(dx) + abs(dy);
                        int corner_value = 100 - distance * 20;
                        if (corner_value > cost_map_.data[nidx]) {
                            cost_map_.data[nidx] = corner_value;
                        }
                    }
                }
            }
        }
    }
}
// ======================== 发布地图数据 ========================

void SonarRealtimeMapper::publishMapData() {
    // 发布原始点云
    if (!pointcloud_->empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pointcloud_, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "sonar_frame";

         // 【】: 发布TF变换
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0, 0, 0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), 
                               "map", "sonar_frame"));
        pointcloud_pub_.publish(cloud_msg);
    }
    
    // 发布滤波后的点云
    if (!filtered_pointcloud_->empty()) {
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_pointcloud_, filtered_cloud_msg);
        filtered_cloud_msg.header.stamp = ros::Time::now();
        filtered_cloud_msg.header.frame_id = "sonar_frame";
        filtered_pointcloud_pub_.publish(filtered_cloud_msg);
    }
    
    // 发布占据栅格地图
    if (!occupancy_map_.data.empty()) {
        occupancy_pub_.publish(occupancy_map_);
    }
    
    // 发布代价地图
    if (!cost_map_.data.empty()) {
        costmap_pub_.publish(cost_map_);
    }
    
    // 发布TF变换
    publishTF();
    
    // 发布可视化标记（）
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sonar";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    if (!filtered_pointcloud_->empty()) {
        for (const auto& point : filtered_pointcloud_->points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            marker.points.push_back(p);
        }
        marker_pub_.publish(marker);
    }
}

// ======================== 发布TF变换 ========================
 
// 修改 sonar_realtime_mapper.cpp 中的 publishTF() 函数
void SonarRealtimeMapper::publishTF() {
    ros::Time current_time = ros::Time::now();
   
    // 1. 发布 base_link -> sonar_frame（这是必须的）
    tf::Transform base_to_sonar;
    base_to_sonar.setOrigin(tf::Vector3(0.1, 0, 0.05));  // 声纳位置
    base_to_sonar.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    tf_broadcaster_.sendTransform(
        tf::StampedTransform(base_to_sonar, current_time, 
                           "base_link", "sonar_frame"));
    
    // 2. 发布 base_link -> imu_link
    // tf::Transform base_to_imu;
    // base_to_imu.setOrigin(tf::Vector3(0, 0, 0.1)); // IMU在机器人上方10cm
    // base_to_imu.setRotation(tf::Quaternion(0, 0, 0, 1));
    
    // tf_broadcaster_.sendTransform(
    //     tf::StampedTransform(base_to_imu, current_time, 
    //                        "base_link", "imu_link"));
 
    
}
 
// ======================== 开始扫描 ========================

void SonarRealtimeMapper::startScanning() {
    if (is_scanning_.load()) {
        ROS_WARN("扫描已经在进行中");
        return;
    }
    
    if (!sonar_serial_) {
        ROS_ERROR("串口未初始化，无法开始扫描");
        return;
    }
    
    stop_requested_.store(false);
    is_scanning_.store(true);
    full_scan_complete_.store(false);
    
    // 启动扫描线程
    scan_thread_ = boost::thread(&SonarRealtimeMapper::scanningThread, this);
    
    ROS_INFO("开始实时扫描");
}

// ======================== 停止扫描 ========================

void SonarRealtimeMapper::stopScanning() {
    if (!is_scanning_.load()) {
        return;
    }
    
    stop_requested_.store(true);
    is_scanning_.store(false);
    
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }
    
    ROS_INFO("停止扫描");
}

// ======================== 保存扫描数据 ========================

bool SonarRealtimeMapper::saveScanData(const std::string& filename) {
    if (pointcloud_->empty()) {
        ROS_WARN("没有点云数据可保存");
        return false;
    }
    
    try {
        std::string pcd_filename = filename + ".pcd";
        if (pcl::io::savePCDFileBinary(pcd_filename, *pointcloud_) == 0) {
            ROS_INFO("保存点云数据到: %s", pcd_filename.c_str());
            return true;
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("保存点云失败: %s", e.what());
    }
    
    return false;
}

// ======================== 保存代价地图 ========================

bool SonarRealtimeMapper::saveCostMap(const std::string& filename) {
    if (cost_map_.data.empty()) {
        ROS_WARN("没有代价地图数据可保存");
        return false;
    }
    
    try {
        // 保存为PGM格式
        std::string pgm_filename = filename + ".pgm";
        std::ofstream pgm_file(pgm_filename);
        
        if (!pgm_file.is_open()) {
            return false;
        }
        
        // PGM文件头
        pgm_file << "P2\n";
        pgm_file << "# Cost Map\n";
        pgm_file << cost_map_.info.width << " " << cost_map_.info.height << "\n";
        pgm_file << "100\n";
        
        // 地图数据（行倒序，因为PGM是左下角原点）
        for (int y = cost_map_.info.height - 1; y >= 0; y--) {
            for (int x = 0; x < cost_map_.info.width; x++) {
                int value = cost_map_.data[y * cost_map_.info.width + x];
                if (value < 0) value = 50; // 未知区域设为中间值
                pgm_file << value << " ";
            }
            pgm_file << "\n";
        }
        
        pgm_file.close();
        
        // 保存YAML元数据
        std::string yaml_filename = filename + ".yaml";
        std::ofstream yaml_file(yaml_filename);
        
        yaml_file << "image: " << pgm_filename << "\n";
        yaml_file << "resolution: " << cost_map_.info.resolution << "\n";
        yaml_file << "origin: [" << cost_map_.info.origin.position.x << ", "
                  << cost_map_.info.origin.position.y << ", 0.0]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: " << occupied_threshold_ << "\n";
        yaml_file << "free_thresh: " << free_threshold_ << "\n";
        
        yaml_file.close();
        
        ROS_INFO("保存代价地图到: %s 和 %s", pgm_filename.c_str(), yaml_filename.c_str());
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("保存代价地图失败: %s", e.what());
        return false;
    }
}

// ======================== 清空数据 ========================

void SonarRealtimeMapper::clearScanData() {
    sonar_points_.clear();
    pointcloud_->clear();
    filtered_pointcloud_->clear();
    raw_points_buffer_.clear();
    
    // 重置地图数据
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    std::fill(cost_map_.data.begin(), cost_map_.data.end(), -1);
    
    // 重置代价单元
    for (auto& cell : cost_cells_) {
        cell = CostCell();
    }
    
    // 重置统计
    total_scans_ = 0;
    valid_scans_ = 0;
    dropped_scans_ = 0;
    
    ROS_INFO("清空所有扫描数据");
}

// ======================== CRC16计算 ========================

uint16_t SonarRealtimeMapper::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

// ======================== 坐标转换 ========================

bool SonarRealtimeMapper::worldToMap(float wx, float wy, int& mx, int& my) const {
    if (wx < map_origin_x_ || wy < map_origin_y_) {
        return false;
    }
    
    mx = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    my = static_cast<int>((wy - map_origin_y_) / map_resolution_);
    
    return (mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_);
}

void SonarRealtimeMapper::mapToWorld(int mx, int my, float& wx, float& wy) const {
    wx = map_origin_x_ + (mx + 0.5f) * map_resolution_;
    wy = map_origin_y_ + (my + 0.5f) * map_resolution_;
}

// ======================== CSV日志记录 ========================

void SonarRealtimeMapper::logToCSV(const SonarPoint& point) {
    if (!enable_csv_logging_) {
        return;
    }
    
    // 第一次打开文件
    if (!csv_file_.is_open()) {
        std::string csv_filename;
        nh_.param("csv_filename", csv_filename, std::string("/tmp/sonar_scan.csv"));
        csv_file_.open(csv_filename, std::ios::out | std::ios::trunc);
        
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,angle_deg,num_points,min_range,max_range,min_intensity,max_intensity\n";
            ROS_INFO("启用CSV日志记录: %s", csv_filename.c_str());
        }
    }
    
    if (csv_file_.is_open()) {
        float angle_deg = point.angle * RAD_TO_DEG;
        
        // 计算统计信息
        float min_range = point.ranges.empty() ? 0.0f : 
                         *std::min_element(point.ranges.begin(), point.ranges.end());
        float max_range = point.ranges.empty() ? 0.0f : 
                         *std::max_element(point.ranges.begin(), point.ranges.end());
        float min_intensity = point.intensities.empty() ? 0.0f : 
                             *std::min_element(point.intensities.begin(), point.intensities.end());
        float max_intensity = point.intensities.empty() ? 0.0f : 
                             *std::max_element(point.intensities.begin(), point.intensities.end());
        
        csv_file_ << std::fixed << std::setprecision(6)
                 << point.timestamp.toSec() << ","
                 << angle_deg << ","
                 << point.ranges.size() << ","
                 << min_range << ","
                 << max_range << ","
                 << min_intensity << ","
                 << max_intensity << "\n";
        
        csv_file_.flush();
    }
}

 
// 或者更智能的判断
bool SonarRealtimeMapper::shouldGenerateDenseCloud() const {
    // 如果有足够的数据累积，就生成密集点云
    if (sector_data_.empty()) return false;
    
    // 检查是否有足够的数据
    int total_points = 0;
    for (const auto& sector : sector_data_) {
        total_points += sector.intensity_accumulator.size();
    }
    
    // 至少积累1000个点才生成
    return total_points > 1000;

    
}


void SonarRealtimeMapper::checkAndPublishInitialMap() {

     if (phase1_enabled_) {
        handlePhase1Mapping();
        return;  // 第一阶段模式下，跳过原有逻辑
    }
    
    
    static bool initial_map_published = false;
    static int stable_counter = 0;
    static int last_green_point_count = 0;
    static int last_total_points = 0;
    static ros::Time scan_start_time = ros::Time::now();
    static ros::Time last_update_time = ros::Time::now();
    static int scan_round_count = 0;
    static std::set<int> all_covered_angles;
    static bool timer_started = false;
    
    // 启动计时器
    if (!timer_started && !sonar_points_.empty()) {
        scan_start_time = ros::Time::now();
        timer_started = true;
        ROS_INFO("静态建图计时开始");
    }
    
    // 强制等待90秒
    if (timer_started) {
        ros::Duration elapsed = ros::Time::now() - scan_start_time;
        
        // ==================== 100秒强制停止逻辑 ====================
        if (elapsed.toSec() > 100.0f && !initial_map_published) {
            ROS_INFO("========================================");
            ROS_INFO("=== 扫描超过100秒，强制生成最终地图 ===");
            ROS_INFO("  扫描时长: %.1f 秒", elapsed.toSec());
            
            // 计算当前数据统计
            int current_total_points = pointcloud_->size();
            int current_green_points = 0;
            int angle_coverage = 0;
            std::set<int> temp_covered_angles;
            
            for (const auto& pt : sonar_points_) {
                float angle_deg = pt.angle * RAD_TO_DEG;
                temp_covered_angles.insert(static_cast<int>(angle_deg) % 360);
                
                for (size_t i = 0; i < pt.ranges.size(); i++) {
                    // 【能量阈值已调整】从150降低到130
                    if (pt.ranges[i] <= 3.0f && pt.intensities[i] >= 130.0f) {
                        current_green_points++;
                    }
                }
            }
            angle_coverage = temp_covered_angles.size();
            
            ROS_INFO("  最终统计: 总点=%d, 绿点=%d, 角度=%d/360", 
                    current_total_points, current_green_points, angle_coverage);
            
            // 生成最终地图
            createEnhancedOccupancyGrid();
            saveStageMap("final_100s_timeout");
            
            // 发布完成信号
            std_msgs::Bool done_msg;
            done_msg.data = true;
            static_map_done_pub_.publish(done_msg);
            
            // 发布初始地图
            occupancy_map_.header.stamp = ros::Time::now();
            initial_occupancy_pub_.publish(occupancy_map_);
            
            // 停止扫描
            stopScanning();
            
            initial_map_published = true;
            
            ROS_INFO("✓ 100秒超时，静态建图强制完成");
            ROS_INFO("========================================");
            return;  // 直接返回，不执行后面的逻辑
        }
        
        // 95秒警告
        if (elapsed.toSec() > 95.0f && elapsed.toSec() <= 100.0f) {
            ROS_WARN_THROTTLE(5.0, "即将达到100秒超时: %.1f/100秒", elapsed.toSec());
        }
        
        if (elapsed.toSec() < 90.0f) {
            ROS_INFO_THROTTLE(10.0, "等待扫描完成: %.1f/90.0秒", elapsed.toSec());
            return;  // 90秒前直接返回
        }
    }
    
    // 只在静态模式下检查，且未发布过初始地图
    if (full_scan_complete_.load() && !initial_map_published) {
        // ==================== 1. 计算当前数据质量 ====================
        int current_total_points = pointcloud_->size();
        
        // 计算绿色点云数量（注意：能量阈值已调整）
        int current_green_points = 0;
        for (const auto& pt : sonar_points_) {
            // 累积角度
            float angle_deg = pt.angle * RAD_TO_DEG;
            all_covered_angles.insert(static_cast<int>(angle_deg) % 360);
            
            for (size_t i = 0; i < pt.ranges.size(); i++) {
                // 【能量阈值调整】从150降低到130
                if (pt.ranges[i] <= 3.0f && pt.intensities[i] >= 130.0f) {
                    current_green_points++;
                }
            }
        }
        
        // 计算角度覆盖度
        int angle_coverage = all_covered_angles.size();
        
        // ==================== 2. 检查稳定性 ====================
        ros::Duration scan_duration = ros::Time::now() - scan_start_time;
        ros::Duration since_last_update = ros::Time::now() - last_update_time;
        
        // 检查绿色点云变化率
        if (last_green_point_count > 0) {
            float green_change = abs(current_green_points - last_green_point_count) / 
                                static_cast<float>(last_green_point_count);
            
            // 检查总点数变化
            float total_change = abs(current_total_points - last_total_points) / 
                                static_cast<float>(std::max(last_total_points, 1));
            
            // 稳定性条件：
            // 1. 绿色点云变化小于15%
            // 2. 总点数变化小于10%
            // 3. 至少5秒没有显著更新
            if (green_change < 0.15f && total_change < 0.10f && 
                since_last_update.toSec() > 5.0f) {
                stable_counter++;
                ROS_DEBUG("稳定性计数: %d/3 (绿色变化: %.1f%%, 总变化: %.1f%%)", 
                         stable_counter, green_change * 100, total_change * 100);
            } else {
                stable_counter = 0;
                ROS_DEBUG("数据不稳定，重置计数器");
            }
        }
        
        // 更新上次数据
        last_green_point_count = current_green_points;
        last_total_points = current_total_points;
        last_update_time = ros::Time::now();
        
        // ==================== 3. 检查完成条件 ====================
        bool completion_conditions_met = false;
        
        // 【修改点1】: 时间至少90秒
        bool time_condition = scan_duration.toSec() > 90.0f;
        
        // 【修改点2】: 降低数据量要求（因为能量阈值降低了）
        bool data_condition = current_total_points > 150 && current_green_points > 60;
        
        // 【修改点3】: 降低角度覆盖要求
        bool coverage_condition = angle_coverage > 280;  // 从306降低到280（78%）
        
        // 【修改点4】: 扫描圈数要求（至少2圈完整扫描）
        bool round_condition = scan_round_count >= 2;
        
        // 数据稳定
        bool stability_condition = stable_counter >= 3;
        
        // 没有新的显著数据
        bool no_new_data_condition = (since_last_update.toSec() > 15.0f);
        
        // 【修改点5】: 完成需要满足所有核心条件
        completion_conditions_met = time_condition && 
                                   data_condition && 
                                   coverage_condition &&
                                   round_condition &&
                                   (stability_condition || no_new_data_condition);
        
        // ==================== 4. 输出调试信息 ====================
        ROS_INFO_THROTTLE(10.0, 
            "建图进度: 时间=%.1f/100秒, 总点=%d, 绿点=%d, 角度=%d/360, 圈数=%d, 稳定=%d/3",
            scan_duration.toSec(), current_total_points, current_green_points,
            angle_coverage, scan_round_count, stable_counter);
        
        // ==================== 5. 如果完成，生成最终地图 ====================
        if (completion_conditions_met && !initial_map_published) {
            ROS_INFO("========================================");
            ROS_INFO("=== 静态建图完成条件满足 ===");
            ROS_INFO("  扫描时长: %.1f 秒", scan_duration.toSec());
            ROS_INFO("  总点数: %d", current_total_points);
            ROS_INFO("  绿色点云: %d", current_green_points);
            ROS_INFO("  角度覆盖: %d/360 度 (%.1f%%)", 
                    angle_coverage, angle_coverage * 100.0 / 360);
            ROS_INFO("  扫描圈数: %d", scan_round_count);
            ROS_INFO("  稳定性: %d/3", stable_counter);
            ROS_INFO("========================================");
            
            // 5.1 生成增强版高质量地图
            createEnhancedOccupancyGrid();
            
            // 5.2 保存最终地图
            saveStageMap("final");
            
            // 5.3 发布完成信号
            std_msgs::Bool done_msg;
            done_msg.data = true;
            static_map_done_pub_.publish(done_msg);
            
            // 5.4 发布当前占据地图作为初始地图
            occupancy_map_.header.stamp = ros::Time::now();
            initial_occupancy_pub_.publish(occupancy_map_);
            
            // 5.5 停止主动扫描
            stopScanning();
            
            // 5.6 设置完成标志
            initial_map_published = true;
            
            // 5.7 输出完成信息
            ROS_INFO("✓✓✓ 静态建图完成！");
            ROS_INFO("  地图已保存并持续发布");
            ROS_INFO("  按Ctrl+C退出程序");
            ROS_INFO("========================================");
        }
        
        // ==================== 6. 预警检查 ====================
        // 如果扫描时间很长但数据质量不好，给出警告
        if (scan_duration.toSec() > 120.0f && current_green_points < 50) {
            ROS_WARN_THROTTLE(30.0, 
                "长时间扫描但绿色点云不足！已扫描%.1f秒，绿点仅%d", 
                scan_duration.toSec(), current_green_points);
        }
        
        // 检查扫描圈数是否足够
        if (scan_duration.toSec() > 60.0f && scan_round_count < 2) {
            ROS_WARN_THROTTLE(30.0, 
                "扫描时间%.1f秒但仅完成%d圈，需要至少2圈完整扫描",
                scan_duration.toSec(), scan_round_count);
        }
    }
    
    // ==================== 7. 即使建图完成，也继续发布地图 ====================
    if (initial_map_published) {
        // 定期更新地图时间戳并发布（保持rviz显示）
        static ros::Time last_publish = ros::Time::now();
        ros::Duration since_last_publish = ros::Time::now() - last_publish;
        
        if (since_last_publish.toSec() > 0.5) {  // 2Hz发布频率
            occupancy_map_.header.stamp = ros::Time::now();
            cost_map_.header.stamp = ros::Time::now();
            
            occupancy_pub_.publish(occupancy_map_);
            costmap_pub_.publish(cost_map_);
            
            // 可选：继续发布点云
            if (!pointcloud_->empty()) {
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*pointcloud_, cloud_msg);
                cloud_msg.header.stamp = ros::Time::now();
                cloud_msg.header.frame_id = "sonar_frame";
                pointcloud_pub_.publish(cloud_msg);
            }
            
            last_publish = ros::Time::now();
            ROS_DEBUG_THROTTLE(30.0, "持续发布静态地图...");
        }
    }
}



void SonarRealtimeMapper::createEnhancedOccupancyGrid() {
    ROS_INFO("生成增强版高质量地图...");
    
    if (pointcloud_->empty() && sonar_points_.empty()) {
        ROS_WARN("没有点云数据，无法生成增强地图");
        return;
    }
    
    // ==================== 1. 创建增强点云 ====================
    PointCloudIPtr enhanced_cloud(new PointCloudI());
    
    // 如果有原始声纳数据，重新处理以获得更好质量
    if (!sonar_points_.empty()) {
        ROS_INFO("使用 %lu 帧原始数据重新生成点云", sonar_points_.size());
        
        for (const auto& sonar_point : sonar_points_) {
            float angle_deg = sonar_point.angle * RAD_TO_DEG;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                float range = sonar_point.ranges[i];
                float intensity = sonar_point.intensities[i];
                
                // 严格的筛选条件
                bool keep_point = false;
                
                // 高质量绿色点云（墙壁）
                if (intensity >= 160.0f && range <= 3.5f) {
                    keep_point = true;
                }
                // 中等质量点，但距离合理
                else if (intensity >= 140.0f && range <= 2.0f) {
                    keep_point = true;
                }
                // 远距离但强度非常高
                else if (intensity >= 180.0f && range <= 6.0f) {
                    keep_point = true;
                }
                
                if (keep_point) {
                    float x = range * cosf(sonar_point.angle);
                    float y = range * sinf(sonar_point.angle);
                    
                    // 应用-45度旋转
                    const float rotation_angle_rad = -45.0f * DEG_TO_RAD;
                    const float cos_theta = cosf(rotation_angle_rad);
                    const float sin_theta = sinf(rotation_angle_rad);
                    
                    pcl::PointXYZI point;
                    point.x = cos_theta * x - sin_theta * y;
                    point.y = sin_theta * x + cos_theta * y;
                    point.z = 0.0f;
                    point.intensity = intensity;
                    
                    enhanced_cloud->points.push_back(point);
                }
            }
        }
    } else {
        *enhanced_cloud = *pointcloud_;
    }
    
    enhanced_cloud->width = enhanced_cloud->points.size();
    enhanced_cloud->height = 1;
    
    ROS_INFO("增强点云生成: %lu 点", enhanced_cloud->size());
    
    // ==================== 2. 应用高级滤波 ====================
    if (!enhanced_cloud->empty()) {
        // 统计滤波去除离群点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud(enhanced_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered);
        
        ROS_DEBUG("统计滤波后: %lu -> %lu 点", enhanced_cloud->size(), filtered->size());
        
        // 半径滤波进一步清理
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
        ror.setInputCloud(filtered);
        ror.setRadiusSearch(0.15f);
        ror.setMinNeighborsInRadius(3);
        ror.filter(*enhanced_cloud);
        
        ROS_DEBUG("半径滤波后: %lu -> %lu 点", filtered->size(), enhanced_cloud->size());
    }
    
    // ==================== 3. 检测水池边界 ====================
    std::vector<float> pool_boundaries(360, FLT_MAX);
    std::vector<float> boundary_intensities(360, 0);
    
    if (!enhanced_cloud->empty()) {
        // 按角度分区
        std::vector<std::vector<pcl::PointXYZI>> angle_buckets(360);
        
        for (const auto& point : enhanced_cloud->points) {
            float distance = sqrt(point.x * point.x + point.y * point.y);
            float angle = atan2(point.y, point.x) * RAD_TO_DEG;
            if (angle < 0) angle += 360.0f;
            
            int angle_idx = static_cast<int>(angle) % 360;
            
            if (distance <= 8.0f) {
                angle_buckets[angle_idx].push_back(point);
            }
        }
        
        // 寻找每个角度的墙壁边界
        for (int angle = 0; angle < 360; angle++) {
            if (!angle_buckets[angle].empty()) {
                // 按距离排序
                std::sort(angle_buckets[angle].begin(), angle_buckets[angle].end(),
                    [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
                        float dist_a = sqrt(a.x * a.x + a.y * a.y);
                        float dist_b = sqrt(b.x * b.x + b.y * b.y);
                        return dist_a < dist_b;
                    });
                
                // 寻找第一个高强度点（很可能是墙壁）
                for (const auto& point : angle_buckets[angle]) {
                    float distance = sqrt(point.x * point.x + point.y * point.y);
                    
                    // 墙壁条件
                    if (point.intensity >= 150.0f && distance >= 0.5f && distance <= 5.0f) {
                        // 检查相邻角度的一致性
                        int consistent_count = 0;
                        
                        for (int offset = -2; offset <= 2; offset++) {
                            if (offset == 0) continue;
                            int neighbor_angle = (angle + offset + 360) % 360;
                            if (pool_boundaries[neighbor_angle] < FLT_MAX - 1.0f) {
                                float neighbor_dist = pool_boundaries[neighbor_angle];
                                if (fabs(distance - neighbor_dist) < 0.5f) {
                                    consistent_count++;
                                }
                            }
                        }
                        
                        // 至少有一个相邻角度有类似距离
                        if (consistent_count > 0) {
                            pool_boundaries[angle] = distance;
                            boundary_intensities[angle] = point.intensity;
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // 平滑边界
    std::vector<float> smoothed_boundaries = pool_boundaries;
    for (int i = 0; i < 360; i++) {
        if (pool_boundaries[i] < FLT_MAX - 1.0f) {
            float sum = pool_boundaries[i];
            int count = 1;
            
            for (int offset = -2; offset <= 2; offset++) {
                if (offset == 0) continue;
                int idx = (i + offset + 360) % 360;
                if (pool_boundaries[idx] < FLT_MAX - 1.0f) {
                    sum += pool_boundaries[idx];
                    count++;
                }
            }
            
            smoothed_boundaries[i] = sum / count;
        }
    }
    
    // ==================== 4. 基于边界的地图生成 ====================
    // 重置地图
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    int total_points_added = 0;
    int points_filtered_out = 0;
    
    if (enhanced_cloud->empty()) {
        ROS_WARN("增强点云为空，使用原始点云");
        enhanced_cloud = pointcloud_;
    }
    
    for (const auto& point : enhanced_cloud->points) {
        float distance = sqrt(point.x * point.x + point.y * point.y);
        float angle = atan2(point.y, point.x) * RAD_TO_DEG;
        if (angle < 0) angle += 360.0f;
        
        int angle_idx = static_cast<int>(angle) % 360;
        float boundary_distance = smoothed_boundaries[angle_idx];
        
        // 判断是否保留该点
        bool keep_point = true;
        
        // 过滤规则1: 水池外噪声
        if (boundary_distance < FLT_MAX - 1.0f) {
            if (distance > boundary_distance + 0.8f) {
                if (point.intensity < 160.0f) {
                    keep_point = false;
                    points_filtered_out++;
                }
            }
        }
        
        // 过滤规则2: 非常近的噪声
        if (distance < 0.3f && point.intensity < 130.0f) {
            keep_point = false;
            points_filtered_out++;
        }
        
        // 保留高质量点
        if (keep_point) {
            int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
            int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
            
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                int index = map_y * map_width_ + map_x;
                
                // 根据强度和质量设置占据值
                int occupancy_value = 0;
                
                // 高质量墙壁点
                if (point.intensity >= 170.0f && distance <= 4.0f) {
                    occupancy_value = 100;
                    
                    // 增强墙壁连续性（高斯权重）
                    for (int dy = -2; dy <= 2; dy++) {
                        for (int dx = -2; dx <= 2; dx++) {
                            int nx = map_x + dx;
                            int ny = map_y + dy;
                            
                            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                                int nidx = ny * map_width_ + nx;
                                float dist = sqrt(dx*dx + dy*dy) * map_resolution_;
                                float weight = exp(-dist * dist / (2 * 0.1f * 0.1f));
                                int8_t neighbor_value_8bit = (int8_t)(occupancy_value * weight);
                                if (neighbor_value_8bit > occupancy_map_.data[nidx]) {
                                    occupancy_map_.data[nidx] = neighbor_value_8bit;
                                }
                            }
                        }
                    }
                }
                // 中等质量点
                else if (point.intensity >= 140.0f) {
                    occupancy_value = 70;
                }
                // 低质量点
                else if (point.intensity >= 110.0f) {
                    occupancy_value = 50;
                }
                
                if (occupancy_value > occupancy_map_.data[index]) {
                    occupancy_map_.data[index] = occupancy_value;
                }
                
                total_points_added++;
            }
        }
    }
    
    // ==================== 5. 形态学后处理 ====================
    // 膨胀操作（使墙壁更连续）
    std::vector<int8_t> temp_map = occupancy_map_.data;
    
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            if (occupancy_map_.data[idx] > 60) {
                // 3x3膨胀
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nidx = (y + dy) * map_width_ + (x + dx);
                        if (temp_map[nidx] < 50) {
                            temp_map[nidx] = 50;
                        }
                    }
                }
            }
        }
    }
    
    // 腐蚀操作（恢复形状但保留小间隙）
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            if (temp_map[idx] > 50) {
                bool has_low_neighbor = false;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nidx = (y + dy) * map_width_ + (x + dx);
                        if (occupancy_map_.data[nidx] < 30) {
                            has_low_neighbor = true;
                            break;
                        }
                    }
                    if (has_low_neighbor) break;
                }
                
                // 可能是噪声，降低置信度
                if (has_low_neighbor && occupancy_map_.data[idx] < 40) {
                    occupancy_map_.data[idx] = std::min((int8_t)occupancy_map_.data[idx], (int8_t)40);
                }
            }
        }
    }
    
    // ==================== 6. 标记自由空间 ====================
    int robot_x = static_cast<int>((0.0 - map_origin_x_) / map_resolution_);
    int robot_y = static_cast<int>((0.0 - map_origin_y_) / map_resolution_);
    
    if (robot_x >= 0 && robot_x < map_width_ && robot_y >= 0 && robot_y < map_height_) {
        int free_radius = static_cast<int>(2.5 / map_resolution_);
        
        for (int y = robot_y - free_radius; y <= robot_y + free_radius; y++) {
            for (int x = robot_x - free_radius; x <= robot_x + free_radius; x++) {
                if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) {
                    int idx = y * map_width_ + x;
                    float dist = sqrt(pow(x - robot_x, 2) + pow(y - robot_y, 2)) * map_resolution_;
                    
                    if (dist < 2.0f && occupancy_map_.data[idx] < 30) {
                        occupancy_map_.data[idx] = 0;
                    }
                }
            }
        }
    }
    
    // ==================== 7. 最终处理 ====================
    occupancy_map_.header.stamp = ros::Time::now();
    
    // 保存当前点云
    *pointcloud_ = *enhanced_cloud;
    
    // 统计信息
    int occupied_cells = 0;
    int free_cells = 0;
    int unknown_cells = 0;
    
    for (int val : occupancy_map_.data) {
        if (val < 0) unknown_cells++;
        else if (val == 0) free_cells++;
        else if (val > 50) occupied_cells++;
    }
    
    int valid_boundaries = 0;
    for (float boundary : smoothed_boundaries) {
        if (boundary < FLT_MAX - 1.0f) {
            valid_boundaries++;
        }
    }
    
    ROS_INFO("======= 增强地图生成完成 =======");
    ROS_INFO("  输入点云: %lu 点", enhanced_cloud->size());
    ROS_INFO("  地图点: %d 添加, %d 过滤", total_points_added, points_filtered_out);
    ROS_INFO("  地图单元: 占据=%d, 空闲=%d, 未知=%d", 
             occupied_cells, free_cells, unknown_cells);
    ROS_INFO("  有效边界: %d/360 角度", valid_boundaries);
    ROS_INFO("================================");
}


// ======================== 第一阶段核心实现 ========================

void SonarRealtimeMapper::handlePhase1Mapping() {
    if (!phase1_enabled_) return;

    ros::Time now = ros::Time::now();
    static ros::Time last_status_time = now;
    
    // 定期发布状态
    if ((now - last_status_time).toSec() > 2.0) {
        publishPhase1Status();
        last_status_time = now;
    }
    
    switch (phase1_step_) {
        case 0: // 准备开始
            if (phase1_enabled_) {
              
                // 等待扫描完全启动
                if (is_scanning_.load()) {
                    ROS_INFO("第一阶段建图开始");
                    phase1_step_ = 1;  // 直接进入步骤1
                    phase1_step_start_time_ = now;
                    publishPhase1Status("开始第一个位置建图");
                }
                // ====================================
            }
            break;
            
        case 1: // 第一个位置建图
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                if (elapsed.toSec() >= phase1_scan_duration_) {
                    saveCurrentPhase1Submap();
                    phase1_step_ = 2;
                    phase1_step_start_time_ = now;
                    stopScanning();
                    
                    ROS_INFO("第一个位置建图完成，请移动机器人");
                    ROS_INFO("等待 %.1f 秒...", phase1_wait_duration_);
                    publishPhase1Status("等待人工移动");
                }
            }
            break;
            
        case 2: // 等待人工移动
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                if (elapsed.toSec() >= phase1_wait_duration_) {
                    phase1_step_ = 3;
                    phase1_step_start_time_ = now;
                    clearScanData();
                    startScanning();
                    
                    ROS_INFO("开始第二个位置建图");
                    publishPhase1Status("第二个位置建图中");
                }
            }
            break;
            
        case 3: // 第二个位置建图
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                if (elapsed.toSec() >= phase1_scan_duration_) {
                    saveCurrentPhase1Submap();
                    phase1_step_ = 4;
                    stopScanning();
                    
                    ROS_INFO("第二个位置建图完成，开始合并");
                    publishPhase1Status("合并地图中");
                    
                    mergePhase1Submaps();
                    
                    ROS_INFO("第一阶段建图完成！");
                    publishPhase1Status("第一阶段完成");
                }
            }
            break;
            
        case 4: // 完成
            // 持续发布合并后的地图
            static ros::Time last_publish = ros::Time::now();
            if ((now - last_publish).toSec() > 0.5) {
                occupancy_pub_.publish(occupancy_map_);
                last_publish = now;
            }
            break;
    }
}

void SonarRealtimeMapper::saveCurrentPhase1Submap() {
    Phase1SubMap submap;
    submap.index = phase1_submap_count_ + 1;
    submap.timestamp = ros::Time::now();
    submap.robot_pose = getCurrentRobotPose();
    submap.map = occupancy_map_;
    
    // 深拷贝点云
    submap.cloud.reset(new PointCloudI(*pointcloud_));
    
    // 保存到列表
    phase1_submaps_.push_back(submap);
    phase1_submap_count_++;
    
    // 保存到文件
    std::string filename = "phase1_submap_" + std::to_string(submap.index);
    saveStageMap(filename);
    
    ROS_INFO("保存子地图 %d: 点云%lu点，位姿(%.2f, %.2f)", 
             submap.index, submap.cloud->size(),
             submap.robot_pose.x, submap.robot_pose.y);
}

geometry_msgs::Pose2D SonarRealtimeMapper::getCurrentRobotPose() {
    geometry_msgs::Pose2D pose;
    // 简化实现：从TF获取或返回默认值
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    
    // 如果有里程计，可以在这里获取真实位姿
    // 暂时使用简化版
    
    return pose;
}

void SonarRealtimeMapper::mergePhase1Submaps() {
    if (phase1_submaps_.size() < 2) {
        ROS_ERROR("需要至少2个子地图才能合并");
        return;
    }
    
    ROS_INFO("开始合并 %lu 个子地图...", phase1_submaps_.size());
    
    // 以第一个地图为基准
    nav_msgs::OccupancyGrid merged_map = phase1_submaps_[0].map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new PointCloudI(*phase1_submaps_[0].cloud));
    
    // 合并第二个地图（根据估计的移动距离）
    if (phase1_submaps_.size() > 1) {
        const auto& submap2 = phase1_submaps_[1];
        
        // 估计相对位姿
        float dx = submap2.robot_pose.x - phase1_submaps_[0].robot_pose.x;
        float dy = submap2.robot_pose.y - phase1_submaps_[0].robot_pose.y;
        float dtheta = submap2.robot_pose.theta - phase1_submaps_[0].robot_pose.theta;
        
        // 如果位姿未知，使用默认移动距离
        if (fabs(dx) < 0.1 && fabs(dy) < 0.1) {
            dx = phase1_move_distance_;  // 默认向前移动
        }
        
        ROS_INFO("相对位姿估计: dx=%.2f, dy=%.2f, dtheta=%.1f°", 
                 dx, dy, dtheta * RAD_TO_DEG);
        
        // 合并点云
        float cos_theta = cos(dtheta);
        float sin_theta = sin(dtheta);
        
        for (const auto& point : submap2.cloud->points) {
            pcl::PointXYZI transformed_point = point;
            float x_rot = point.x * cos_theta - point.y * sin_theta;
            float y_rot = point.x * sin_theta + point.y * cos_theta;
            transformed_point.x = x_rot + dx;
            transformed_point.y = y_rot + dy;
            merged_cloud->points.push_back(transformed_point);
        }
        
        // 合并占据网格（简化：直接叠加）
        for (size_t i = 0; i < merged_map.data.size() && i < submap2.map.data.size(); i++) {
            if (submap2.map.data[i] > merged_map.data[i]) {
                merged_map.data[i] = submap2.map.data[i];
            }
        }
    }
    
    // 更新当前地图
    occupancy_map_ = merged_map;
    *pointcloud_ = *merged_cloud;
    
    // 发布和保存
    occupancy_pub_.publish(occupancy_map_);
    saveStageMap("phase1_final_merged");
    
    ROS_INFO("地图合并完成！总点数: %lu", merged_cloud->size());
}

///

void SonarRealtimeMapper::publishPhase1Status(const std::string& status) {
    std_msgs::String msg;
    if (status.empty()) {
        std::stringstream ss;
        ss << "Phase1: ";
        switch (phase1_step_) {
            case 0: ss << "准备中"; break;
            case 1: ss << "第一个位置建图"; break;
            case 2: ss << "等待移动"; break;
            case 3: ss << "第二个位置建图"; break;
            case 4: ss << "完成"; break;
        }
        msg.data = ss.str();
    } else {
        msg.data = status;
    }
    phase1_status_pub_.publish(msg);
}

// ======================== 公共接口实现 ========================
void SonarRealtimeMapper::startPhase1Mapping(int max_submaps) {
    if (phase1_enabled_) {
        ROS_WARN("第一阶段已在运行中");
        return;
    }
    ROS_INFO("第一阶段建图服务0011:");
    phase1_enabled_ = true;
    phase1_step_ = 0;
    phase1_submap_count_ = 0;
    phase1_submaps_.clear();
    
    // =========== 【仅添加这3行】 ===========
    // 1. 清空现有数据
    clearScanData();
    // 2. 开始扫描
    startScanning();
    // 3. 记录开始时间
    phase1_step_start_time_ = ros::Time::now();
    // ====================================
    ROS_INFO("第一阶段建图服务0022:");
    ROS_INFO("第一阶段建图启动");
    ROS_INFO("流程：位置A(%.1fs) → 等待(%.1fs) → 位置B(%.1fs) → 合并", 
             phase1_scan_duration_, phase1_wait_duration_, phase1_scan_duration_);
}

void SonarRealtimeMapper::stopPhase1Mapping() {
    if (!phase1_enabled_) return;
    
    phase1_enabled_ = false;
    phase1_step_ = 0;
    if (is_scanning_.load()) {
        stopScanning();
    }
    
    ROS_INFO("第一阶段建图已停止");
}

std::string SonarRealtimeMapper::getPhase1StatusString() const {
    std::stringstream ss;
    ss << "Phase1: ";
    if (phase1_enabled_) {
        switch (phase1_step_) {
            case 0: ss << "准备中"; break;
            case 1: ss << "第一个位置建图中"; break;
            case 2: ss << "等待人工移动"; break;
            case 3: ss << "第二个位置建图中"; break;
            case 4: ss << "完成"; break;
        }
        ss << " (" << phase1_submap_count_ << "/2)";
    } else {
        ss << "未启用";
    }
    return ss.str();
}

} // namespace sonar_slam


// 回调函数定义
bool startPhase1Callback(sonar_slam::StartPhase1::Request& req,
                         sonar_slam::StartPhase1::Response& res,
                         sonar_slam::SonarRealtimeMapper* mapper) {
    mapper->startPhase1Mapping(req.max_submaps);
    res.success = true;
    res.message = "第一阶段建图已启动，最大子地图数: " + std::to_string(req.max_submaps);
    return true;
}

bool stopPhase1Callback(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res,
                        sonar_slam::SonarRealtimeMapper* mapper) {
    mapper->stopPhase1Mapping();
    res.success = true;
    res.message = "第一阶段建图已停止";
    return true;
}

bool getStatusCallback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res,
                       sonar_slam::SonarRealtimeMapper* mapper) {
    res.success = true;
    res.message = mapper->getPhase1StatusString();
    return true;
}


// ======================== 主函数 ========================
int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_realtime_mapper");
    
    ROS_INFO("========================================");
    ROS_INFO("Sonar Realtime Mapper 启动");
    ROS_INFO("========================================");
    
    ros::NodeHandle nh("~");
    
    // 读取参数
    std::string serial_port;
    int baudrate;
    
    nh.param("serial_port", serial_port, std::string("/dev/ttyACM0"));
    nh.param("baudrate", baudrate, 115200);
    
    ROS_INFO("串口配置: %s @ %d 波特", serial_port.c_str(), baudrate);
    
    // 创建映射器
    sonar_slam::SonarRealtimeMapper mapper(nh);
    
    // ==================== 注册服务（使用Boost绑定） ====================
    
    // 1. 开始第一阶段建图
    ros::AdvertiseServiceOptions start_opt;
    start_opt.init<sonar_slam::StartPhase1>(
        "start_phase1_mapping",
        boost::bind(&startPhase1Callback, _1, _2, &mapper));
    ros::ServiceServer start_srv = nh.advertiseService(start_opt);
    
    // 2. 停止第一阶段建图
    ros::AdvertiseServiceOptions stop_opt;
    stop_opt.init<std_srvs::Trigger>(
        "stop_phase1_mapping",
        boost::bind(&stopPhase1Callback, _1, _2, &mapper));
    ros::ServiceServer stop_srv = nh.advertiseService(stop_opt);
    
    // 3. 查询状态
    ros::AdvertiseServiceOptions status_opt;
    status_opt.init<std_srvs::Trigger>(
        "get_phase1_status",
        boost::bind(&getStatusCallback, _1, _2, &mapper));
    ros::ServiceServer status_srv = nh.advertiseService(status_opt);
    
    // 初始化串口
    if (!mapper.initSerial(serial_port, baudrate)) {
        ROS_ERROR("串口初始化失败，程序退出");
        return 1;
    }
    
    // 检查是否自动启动第一阶段
    bool auto_start_phase1 = false;
    nh.param("phase1/enabled", auto_start_phase1, false);
    
    if (auto_start_phase1) {
        // 自动启动第一阶段
        int max_submaps = 2;
        nh.param("phase1/max_submaps", max_submaps, 2);
        mapper.startPhase1Mapping(max_submaps);
        ROS_INFO("自动启动第一阶段建图，最大子地图数: %d", max_submaps);
    } else {
        // 原有模式：直接开始扫描
        mapper.startScanning();
        ROS_INFO("实时扫描模式已启动");
    }
    
    // 打印服务信息
    ROS_INFO("========================================");
    ROS_INFO("第一阶段建图服务:");
    ROS_INFO("  启动: rosservice call /start_phase1_mapping max_submaps=2");
    ROS_INFO("  停止: rosservice call /stop_phase1_mapping");
    ROS_INFO("  状态: rosservice call /get_phase1_status");
    ROS_INFO("========================================");
    ROS_INFO("按 Ctrl+C 停止");
    ROS_INFO("========================================");
    
    // 主循环
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    mapper.stopScanning();
    
    ROS_INFO("Sonar Realtime Mapper 关闭");
    return 0;
}