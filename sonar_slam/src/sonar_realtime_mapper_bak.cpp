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
    , enable_csv_logging_(false) {
    
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


     // 新增发布器初始化
    dense_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sonar/dense_pointcloud", 1);
    wall_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar/walls", 1);
    corner_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar/corners", 1);
    
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
    // ==================== 【关键修复1】角度转换修正 ====================
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
    
    // 【新增】角度覆盖统计
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
    
    // 【新增】调试输出距离参数
    static int debug_counter = 0;
    if (debug_counter % 100 == 0) {
        ROS_INFO("距离参数: sample_period=%u, time_per_point=%.3fus, dist_per_point=%.3fmm", 
                msg_data.sample_period, time_per_point_us, distance_per_point_mm);
        debug_counter = 0;
    }
    debug_counter++;
    
    // ==================== 【关键修复3】处理每个采样点 ====================
    int valid_samples = 0;
    int use_cnt = 0;  // 使用同事的计数器命名
    
    // 获取实际的采样点数
    uint16_t actual_samples = std::min(msg_data.number_of_samples, msg_data.data_length);
    
    // 【新增】计算理论最大距离
    float max_theoretical_range_mm = distance_per_point_mm * actual_samples;
    float max_theoretical_range_m = max_theoretical_range_mm * 0.001f;
    if (max_range_ < max_theoretical_range_m - 2.0f) {
        ROS_WARN_THROTTLE(10.0, "max_range(%.1fm)可能小于硬件能力(%.1fm)", 
                         max_range_, max_theoretical_range_m);
    }
    
   
    for (uint16_t i = 0; i < actual_samples; i++) {
        // 【关键修复】计算每个采样点的实际距离
        float distance_mm = distance_per_point_mm * (i + 1);  // 毫米
        float distance_m = distance_mm * 0.001f;              // 米
        
        // 获取强度
        uint8_t raw_intensity = msg_data.data[i];
        float intensity = static_cast<float>(raw_intensity);
        
        // ==================== 【修改】使用距离相关的动态阈值过滤 ====================
        // 条件1：dist > 1.2 米（过滤近距离噪声）
        // 条件2：intensity > 动态阈值（根据距离调整强度要求）
        // 条件3：use_cnt < 2（每个角度只保留最强的2个高质量点）

        // 动态阈值计算：近距离要求高，远距离放宽要求
        float dynamic_threshold;
        if (distance_m <= 3.0f) {
            dynamic_threshold = 140.0f;      // 3米内：严格，保持同事的140
        } else if (distance_m <= 6.0f) {
            dynamic_threshold = 120.0f;      // 3-6米：适度放宽
        } else if (distance_m <= 10.0f) {
            dynamic_threshold = 100.0f;      // 6-10米：进一步放宽
        } else {
            dynamic_threshold = 90.0f;       // 10米以上：保持最低要求
        }

        if (distance_m > 1.2f && intensity > dynamic_threshold && use_cnt < 2) {
            sonar_point.ranges.push_back(distance_m);
            sonar_point.intensities.push_back(intensity);
            use_cnt++;
            valid_samples++;
            
            // 【新增】调试输出带阈值信息
            if (valid_samples <= 2) {
                ROS_DEBUG_THROTTLE(0.5, "选中点: 角度=%.1f°, 距离=%.2fm, 阈值=%.0f, 强度=%.0f, use_cnt=%d",
                                angle_deg, distance_m, dynamic_threshold, intensity, use_cnt);
            }
        }
        // 【新增】专门针对远距离的补充点（5米以上）
        else if (distance_m > 5.0f && intensity > 80.0f && use_cnt < 3) {
            // 远距离补充点：5米以上，强度>80
            sonar_point.ranges.push_back(distance_m);
            sonar_point.intensities.push_back(intensity * 0.7f); // 标记为远距离补充点
            valid_samples++;
            
            ROS_DEBUG_THROTTLE(1.0, "远距离补充点: 距离=%.1fm, 强度=%.0f", distance_m, intensity);
        }
        // 【保留】原有的中等质量点补充（针对所有距离）
        else if (distance_m > 2.0f && intensity > 100.0f && use_cnt < 5) {
            // 如果高质量点太少，添加一些次高质量点
            sonar_point.ranges.push_back(distance_m);
            sonar_point.intensities.push_back(intensity * 0.8f); // 标记为次高质量
            valid_samples++;
        }
    }
    
    // ==================== 【关键修复4】调试信息输出增强 ====================
    static int frame_counter = 0;
    frame_counter++;
    
    if (valid_samples > 0) {
        // 每5帧输出一次详细信息（更频繁，便于调试）
        if (frame_counter % 5 == 0) {
            float min_range = *std::min_element(sonar_point.ranges.begin(), sonar_point.ranges.end());
            float max_range = *std::max_element(sonar_point.ranges.begin(), sonar_point.ranges.end());
            float min_intensity = *std::min_element(sonar_point.intensities.begin(), sonar_point.intensities.end());
            float max_intensity = *std::max_element(sonar_point.intensities.begin(), sonar_point.intensities.end());
            
            ROS_INFO("Frame %d: 角度=%.1f°, 点数=%lu, 距离=%.2f-%.2fm, 强度=%.0f-%.0f, use_cnt=%d", 
                    frame_counter,
                    angle_deg, 
                    sonar_point.ranges.size(),
                    min_range, max_range,
                    min_intensity, max_intensity,
                    use_cnt);
        }
        
        // 保存到数据数组
        sonar_points_.push_back(sonar_point);
        
        // 记录到CSV
        if (enable_csv_logging_) {
            logToCSV(sonar_point);
        }
        
        // ==================== 【关键修复5】优化更新频率 ====================
        // 立即转换点云（保持实时性）
        convertToPointCloud();
        
        // 根据数据质量决定地图更新频率
        bool has_high_quality_point = (use_cnt > 0); // 只要有选中的点就算是高质量
    
        if (has_high_quality_point) {
            // 高质量点：降低更新频率
            static int update_counter = 0;
            update_counter++;
            // 每3次高质量点更新一次
            if (update_counter % 3 == 0) {
                filterPointCloud();
                createOccupancyGrid();
                createCostMap();
                publishMapData();
                update_counter = 0;
            }

                
        } else if (frame_counter % 20 == 0) {  // 从10改为20
            // 低质量点：进一步降低更新频率
            filterPointCloud();
            createOccupancyGrid();
            createCostMap();
            publishMapData();
        }
        
        // 累计统计
        if (frame_counter % 100 == 0) {
            float total_points = 0;
            for (const auto& pt : sonar_points_) {
                total_points += pt.ranges.size();
            }
            
            ROS_INFO("累计统计: %d帧, %.0f点, 当前角度=%.1f°, 覆盖角度=%lu/360", 
                    frame_counter, total_points, angle_deg, covered_angles.size());
            
            // 【新增】检查是否存在角度盲区
            std::vector<int> missing_angles;
            for (int a = 0; a < 360; a += 30) {
                if (covered_angles.find(a) == covered_angles.end()) {
                    missing_angles.push_back(a);
                }
            }
            if (!missing_angles.empty()) {
                ROS_WARN("角度盲区: ");
                for (int a : missing_angles) {
                    ROS_WARN("  %d°", a);
                }
            }
        }
    } else {
        // 无效帧统计
        dropped_scans_++;
        
        // 【新增】记录哪些角度没有有效数据
        static std::set<int> no_data_angles;
        no_data_angles.insert(angle_int);
        
        if (dropped_scans_ % 50 == 0) {
            ROS_WARN("已丢弃 %u 帧无效数据, 无数据角度: %lu个", 
                    dropped_scans_, no_data_angles.size());
            
            // 每100帧输出一次无数据的角度
            if (dropped_scans_ % 100 == 0 && !no_data_angles.empty()) {
                ROS_WARN("无数据角度示例: ");
                int count = 0;
                for (int a : no_data_angles) {
                    if (count++ < 5) {
                        ROS_WARN("  %d°", a);
                    }
                }
                if (no_data_angles.size() > 5) {
                    ROS_WARN("  ...等%lu个角度", no_data_angles.size() - 5);
                }
            }
        }
    }
    
    // ==================== 【关键修复6】360度扫描检测优化 ====================
    // 使用更宽松的条件检测360度完成
    if ((angle_deg > 355.0f || angle_deg < 5.0f) && !full_scan_complete_.load()) {
        static int near_zero_count = 0;
        near_zero_count++;
        
        // 需要连续多次在0度附近才认为完成扫描
        if (near_zero_count >= 3 && sonar_points_.size() > 30) {
            full_scan_complete_.store(true);
            near_zero_count = 0;
            
            // 输出完整的扫描统计
            ROS_INFO("✓ 完成360度扫描！");
            ROS_INFO("  累计帧数: %lu", sonar_points_.size());
            ROS_INFO("  角度覆盖: %lu/360 度", covered_angles.size());
            ROS_INFO("  总点数: %.0f", accumulatePoints(sonar_points_));
            
            // 按象限统计点数
            std::map<std::string, int> quadrant_stats;
            for (const auto& pt : sonar_points_) {
                float pt_angle_deg = pt.angle * RAD_TO_DEG;
                if (pt_angle_deg >= 0 && pt_angle_deg < 90) quadrant_stats["NE"]++;
                else if (pt_angle_deg >= 90 && pt_angle_deg < 180) quadrant_stats["NW"]++;
                else if (pt_angle_deg >= 180 && pt_angle_deg < 270) quadrant_stats["SW"]++;
                else quadrant_stats["SE"]++;
            }
            
            ROS_INFO("  象限分布: NE=%d, NW=%d, SW=%d, SE=%d",
                    quadrant_stats["NE"], quadrant_stats["NW"], 
                    quadrant_stats["SW"], quadrant_stats["SE"]);
            
            // 保存数据
            std::string timestamp = std::to_string(static_cast<int>(ros::Time::now().toSec()));
            saveScanData("/tmp/sonar_scan_full_" + timestamp);
            saveCostMap("/tmp/cost_map_full_" + timestamp);
        }
    } else if (angle_deg > 10.0f && angle_deg < 350.0f) {
        // 离开0度区域时重置计数器
        static int near_zero_count = 0;
        near_zero_count = 0;
    }

    // ==================== 【可选】数据累积和增强处理 ====================
    // 只有在高质量点足够多时才进行增强处理
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
}

// 【新增辅助函数】计算总点数
float SonarRealtimeMapper::accumulatePoints(const std::vector<SonarPoint>& points) {
    float total = 0;
    for (const auto& pt : points) {
        total += pt.ranges.size();
    }
    return total;
}
// 新增函数：累积扇区数据
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

// 新增函数：生成更密集的点云
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
    
    // 【可选】添加调试输出
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
    
    // 在函数末尾添加方向标记（不新增函数）
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
            
            // 【可选】: 使墙壁更连续，标记相邻单元格
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

    // ==================== 新增：墙壁连续性和端检测 ====================
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
    // 可选：对代价地图进行平滑处理，消除孤立的噪声点
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


// 新增函数：提取墙壁段
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

         // 【新增】: 发布TF变换
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
    
    // 发布可视化标记（可选）
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

} // namespace sonar_slam

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
    
    // 注册服务 
    ros::ServiceServer start_srv = nh.advertiseService("start_phase1_mapping", 
        [&mapper](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
            mapper.startPhase1Mapping(2);  // 默认2个子地图
            res.success = true;
            res.message = "第一阶段建图已启动";
            return true;
        });
    
    ros::ServiceServer stop_srv = nh.advertiseService("stop_phase1_mapping",
        [&mapper](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
            mapper.stopPhase1Mapping();
            res.success = true;
            res.message = "第一阶段建图已停止";
            return true;
        });
    
    ros::ServiceServer status_srv = nh.advertiseService("get_phase1_status",
        [&mapper](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
            res.success = true;
            res.message = mapper.getPhase1StatusString();
            return true;
        });
    
    // 初始化串口
    if (!mapper.initSerial(serial_port, baudrate)) {
        ROS_ERROR("串口初始化失败，程序退出");
        return 1;
    }
    
    //  检查是否自动启动第一阶段
    bool auto_start_phase1 = false;
    nh.param("phase1/enabled", auto_start_phase1, false);
    
    if (auto_start_phase1) {
        // 自动启动第一阶段
        int max_submaps = 2;
        nh.param("phase1/max_submaps", max_submaps, 2);
        
        ROS_INFO("自动启动第一阶段建图，最大子地图数: %d", max_submaps);
        ROS_INFO("将在 %.1f 秒后开始第一个位置扫描...", 2.0);
        ros::Duration(2.0).sleep();  // 等待2秒，确保所有组件就绪
        ROS_INFO("第一阶段建图服务0000:");
        mapper.startPhase1Mapping(max_submaps);
        
    } else {
        // 原有模式：直接开始扫描
        mapper.startScanning();
        ROS_INFO("实时扫描模式已启动");
    }
    
    // 打印服务信息
    ROS_INFO("========================================");
    ROS_INFO("第一阶段建图服务:");
    ROS_INFO("  启动: rosservice call /start_phase1_mapping");
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