/**
 * @file phase_fusion_sonar_mapper.cpp
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

#include "sonar_slam/phase_fusion_sonar_mapper.h"
#include "sonar_slam/intelligent_submap_aligner.h"
#include "sonar_slam/sonar_aware_aligner.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <functional>
#include <std_srvs/Trigger.h>
#include "sonar_slam/StartPhase1.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>

#include <locale>
#include <codecvt>

namespace sonar_slam {

using namespace std::chrono;

// ======================== 常量定义 ========================
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// ======================== 构造函数和析构函数 ========================

PhaseFusionSonarMapper::PhaseFusionSonarMapper(ros::NodeHandle& nh)
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
    , phase1_scan_duration_(100.0f)
    , phase1_wait_duration_(60.0f)
    , phase1_move_distance_(0.0f)
    , scan_start_angle_(0)
    , last_scan_end_angle_(0)
    , phase1_current_position_(1)
    , circles_completed_in_position_(0)
    , required_circles_per_position_(1)  // 默认1圈
 
    ,angle_base_(-1)
    ,angle_base_initialized_(false)
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
    // ==================== 参数（与参考代码一致） ====================
   
    
    
    // 占据栅格地图参数
    
    nh_.param("map_resolution", map_resolution_, 0.05f);
    nh_.param("map_width", map_width_, 400);           
    nh_.param("map_height", map_height_, 400);        
    nh_.param("map_origin_x", map_origin_x_, -10.0f);  
    nh_.param("map_origin_y", map_origin_y_, -10.0f);  
        
    // 代价地图参数
    nh_.param("costmap_resolution", costmap_resolution_, 0.1f);
    nh_.param("costmap_width", costmap_width_, 200);       
    nh_.param("costmap_height", costmap_height_, 200);      
    nh_.param("costmap_origin_x", costmap_origin_x_, -1.0f);  
    nh_.param("costmap_origin_y", costmap_origin_y_, -5.0f); 
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
    nh_.param("phase1/scan_duration", phase1_scan_duration_, 100.0f);
    nh_.param("phase1/wait_duration", phase1_wait_duration_, 60.0f);
    nh_.param("phase1/move_distance", phase1_move_distance_, 0.0f);
    
    nh_.param("phase1/required_circles", required_circles_per_position_, 1);
    ROS_INFO("Phase1配置: 每个位置需要完成 %d 圈扫描", required_circles_per_position_);
    

}

PhaseFusionSonarMapper::~PhaseFusionSonarMapper() {
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

bool PhaseFusionSonarMapper::initSerial(const std::string& port, int baudrate) {
    try {
        sonar_serial_ = std::make_unique<CallbackAsyncSerial>(port, baudrate);
        sonar_serial_->setCallback(
            boost::bind(&PhaseFusionSonarMapper::receiveDataCallback, this, _1, _2));
        
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

void PhaseFusionSonarMapper::receiveDataCallback(const char* data, unsigned int len) {
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

void PhaseFusionSonarMapper::parseDataPacket(const char* buffer, unsigned int length) {
    while (rx_used_index_ < rx_end_index_) {
        unsigned char current_byte = rx_buffer_[rx_used_index_];
        
        switch (parse_state_) {
            case STATE_IDLE:
                if (current_byte == 0x42) { // 'B'
                    parse_state_ = STATE_GOT_START1;
                    ROS_DEBUG("找到第一个帧头 0x42");
                }
                rx_used_index_++;
                break;
                
            case STATE_GOT_START1:
                if (current_byte == 0x52) { // 'R'
                    parse_state_ = STATE_READING_LENGTH_LOW;
                    ROS_DEBUG("找到第二个帧头 0x52");
                } else {
                    // 不是预期的第二个头，重新同步
                    ROS_DEBUG("无效的第二个帧头: 0x%02X, 期望 0x52", current_byte);
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
                
                // ========== 验证长度合理性 ==========
                if (current_message_.length > 2000 || current_message_.length < 100) {
                    ROS_WARN("长度异常: %d，可能失去同步，重新搜索", current_message_.length);
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                } else {
                    parse_state_ = STATE_READING_COMMAND_LOW;
                }
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
                    current_message_.data.clear();
                    current_message_.data.reserve(current_message_.length);
                    parse_state_ = STATE_READING_DATA;
                } else {
                    // 没有数据，直接处理消息
                    processCompleteMessage();
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                }
                rx_used_index_++;
                break;
                
            case STATE_READING_DATA:
                current_message_.data.push_back(current_byte);
                
                if (current_message_.data.size() >= current_message_.length) {
                    // 数据收完，现在处理消息
                    processCompleteMessage();
                    
                    // ========== 关键修复：完全重置状态 ==========
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                    
                    // 可以立即开始搜索下一个包
                    ROS_DEBUG("包处理完成，准备接收下一个");
                }
                rx_used_index_++;
                break;
        }
    }
    
    // ========== 处理剩余缓冲区 ==========
    // 检查缓冲区中是否有完整的帧头
    if (parse_state_ == STATE_IDLE && (rx_end_index_ - rx_used_index_) >= 2) {
        // 尝试搜索下一个帧头
        for (size_t i = rx_used_index_; i < rx_end_index_ - 1; i++) {
            if (rx_buffer_[i] == 0x42 && rx_buffer_[i+1] == 0x52) {
                // 找到新的帧头，移动缓冲区
                size_t remaining = rx_end_index_ - i;
                memmove(rx_buffer_, rx_buffer_ + i, remaining);
                rx_end_index_ = remaining;
                rx_used_index_ = 0;
                ROS_DEBUG("在缓冲区中找到新帧头，重新开始解析");
                break;
            }
        }
    }
    
    // 缓冲区压缩（保持不变）
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

void PhaseFusionSonarMapper::processCompleteMessage() {
    total_scans_++;
    
    // ====== 调试日志：确认进入此函数 ======
    ROS_INFO_ONCE("[DEBUG] 首次进入 processCompleteMessage 函数。");
    ROS_DEBUG_THROTTLE(1.0, "处理消息: cmd=0x%04X, len=%u", current_message_.command, current_message_.length);
    ROS_INFO("========== 收到完整消息 ==========");
    ROS_INFO("命令字: 0x%04X (%d)", current_message_.command, current_message_.command);
    ROS_INFO("消息长度: %u", current_message_.length);
    ROS_INFO("源地址: %u", current_message_.source);
    ROS_INFO("目的地址: %u", current_message_.destination);
    ROS_INFO("数据大小: %lu", current_message_.data.size());
    
    // 打印前16字节原始数据（如果有）
    if (!current_message_.data.empty()) {
        std::stringstream hex_ss;
        hex_ss << "数据前16字节 (十六进制): ";
        for (size_t i = 0; i < std::min(current_message_.data.size(), size_t(16)); i++) {
            hex_ss << std::hex << std::setw(2) << std::setfill('0') 
                   << (int)current_message_.data[i] << " ";
        }
        ROS_INFO("%s", hex_ss.str().c_str());
        
        std::stringstream dec_ss;
        dec_ss << "数据前16字节 (十进制): ";
        for (size_t i = 0; i < std::min(current_message_.data.size(), size_t(16)); i++) {
            dec_ss << std::dec << std::setw(3) << (int)current_message_.data[i] << " ";
        }
        ROS_INFO("%s", dec_ss.str().c_str());
    }
    ROS_INFO("==================================");
    // ==========过滤控制命令回显 ==========
    // 检查数据是否以 'B','R' 开头（可能是命令回显）
    if (current_message_.data.size() >= 2 && 
        current_message_.data[0] == 0x42 && 
        current_message_.data[1] == 0x52) {
        ROS_DEBUG("检测到可能的命令回显，跳过处理");
        return;
    }
    
    // 根据命令字处理消息
    switch (current_message_.command) {
        case 2300: // 声纳数据消息
             ROS_INFO_THROTTLE(0.5, "[DEBUG] 处理CMD 2300消息，数据大小: %lu", current_message_.data.size());
            if (parseCmd2300Message(current_message_.data)) {
                valid_scans_++;
                 ROS_DEBUG_THROTTLE(0.5, "成功解析2300消息 (#%u)", valid_scans_);
            } else {
                // ========== 解析失败时的错误恢复 ==========
                ROS_DEBUG("解析2300消息失败，可能是错误的数据包");
                // 不需要特殊处理，状态机已经重置
            }
            break;
            
        default:
            ROS_DEBUG("收到未知命令: 0x%04X", current_message_.command);
            break;
    }
    // 根据命令字处理消息
    
}

// ======================== 解析2300消息 ========================

bool PhaseFusionSonarMapper::parseCmd2300Message(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(MsgCmd2300)) {
        ROS_WARN("2300消息数据长度不足: %lu < %lu", 
                data.size(), sizeof(MsgCmd2300));
        return false;
    }
    
    // 复制数据到结构体
    MsgCmd2300 msg_data;
    memcpy(&msg_data, data.data(), sizeof(MsgCmd2300));

    // ROS_WARN("parseCmd2300Message验证:");
    // ROS_WARN("  angle字段: %u (0x%04X)", msg_data.angle, msg_data.angle);
    // ROS_WARN("  number_of_samples: %u", msg_data.number_of_samples);
    // ROS_WARN("  data_length: %u", msg_data.data_length);
    // ROS_WARN("  sample_period: %u", msg_data.sample_period);
    ROS_WARN("===== 结构体字段与原始数据对应关系 =====");
    ROS_WARN("sizeof(MsgCmd2300): %lu", sizeof(MsgCmd2300));
    
    // 逐字节打印前20个字节的原始值
    ROS_WARN("原始数据索引及值:");
    // for (int i = 0; i < 20 && i < data.size(); i++) {
    //     ROS_WARN("  [%d]: 0x%02X (%u)", i, data[i], data[i]);
    // }
    
    // 直接通过指针访问各个字段，验证偏移
    const uint8_t* ptr = data.data();
    
    uint8_t mode = *ptr;
    uint8_t gain = *(ptr + 1);
    uint16_t angle = *(uint16_t*)(ptr + 2);
    uint16_t tx_duration = *(uint16_t*)(ptr + 4);
    uint16_t sample_period = *(uint16_t*)(ptr + 6);
    uint16_t tx_freq = *(uint16_t*)(ptr + 8);
    uint16_t num_samples = *(uint16_t*)(ptr + 10);
    uint16_t data_len = *(uint16_t*)(ptr + 12);
    
    // ROS_WARN("手动解析字段:");
    // ROS_WARN("  mode[0]: %u", mode);
    // ROS_WARN("  gain[1]: %u", gain);
    // ROS_WARN("  angle[2-3]: %u (0x%04X)", angle, angle);
    // ROS_WARN("  tx_duration[4-5]: %u", tx_duration);
    // ROS_WARN("  sample_period[6-7]: %u", sample_period);
    // ROS_WARN("  tx_freq[8-9]: %u", tx_freq);
    // ROS_WARN("  num_samples[10-11]: %u", num_samples);
    // ROS_WARN("  data_len[12-13]: %u", data_len);

 
      // 角度验证
    if (msg_data.angle >= 400) {
        // ========== 打印完整数据包 ==========
        ROS_ERROR("========================================");
        ROS_ERROR("❌ 角度字段异常: %u (应该是0-399)", msg_data.angle);
        ROS_ERROR("   消息长度: %lu 字节", data.size());
        ROS_ERROR("   命令字: 0x%04X", current_message_.command);
        ROS_ERROR("   消息头信息:");
        ROS_ERROR("     mode: %u", msg_data.mode);
        ROS_ERROR("     gain: %u", msg_data.gain_setting);
        ROS_ERROR("     angle: %u", msg_data.angle);
        ROS_ERROR("     tx_duration: %u", msg_data.transmit_duration);
        ROS_ERROR("     sample_period: %u", msg_data.sample_period);
        ROS_ERROR("     tx_frequency: %u", msg_data.transmit_frequency);
        ROS_ERROR("     num_samples: %u", msg_data.number_of_samples);
        ROS_ERROR("     data_length: %u", msg_data.data_length);
        
        ROS_ERROR("   原始数据 (十六进制):");
        std::stringstream hex_stream;
        for (size_t i = 0; i < std::min(data.size(), size_t(64)); i++) {
            if (i % 16 == 0) {
                if (i > 0) ROS_ERROR("     %s", hex_stream.str().c_str());
                hex_stream.str("");
                hex_stream << std::setw(2) << std::setfill('0') << i << ": ";
            }
            hex_stream << std::hex << std::setw(2) << std::setfill('0') 
                      << (int)data[i] << " ";
        }
        if (!hex_stream.str().empty()) {
            ROS_ERROR("     %s", hex_stream.str().c_str());
        }
        
        ROS_ERROR("   原始数据 (十进制):");
        hex_stream.str("");
        for (size_t i = 0; i < std::min(data.size(), size_t(32)); i++) {
            if (i % 8 == 0) {
                if (i > 0) ROS_ERROR("     %s", hex_stream.str().c_str());
                hex_stream.str("");
                hex_stream << std::setw(2) << std::setfill('0') << i << ": ";
            }
            hex_stream << std::dec << std::setw(3) << (int)data[i] << " ";
        }
        if (!hex_stream.str().empty()) {
            ROS_ERROR("     %s", hex_stream.str().c_str());
        }
        ROS_ERROR("========================================");
        return false;
    }
    
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
void PhaseFusionSonarMapper::processScanFrame(int raw_angle, const MsgCmd2300& msg_data) {
    if (!is_scanning_.load()) {
        return;
    }
    // 使用校正后的角度
    // float angle_deg = static_cast<float>(corrected_raw_angle) * 0.9f;
    // ==================== 1角度转换修正 ====================
    // 硬件角度范围是0-399，对应0-359.1度
    // 确保正确处理角度归一化

    static int current_scan_session = 0;
    static bool in_scan_session = false;
 
 
 
    // 只在第一次扫描时初始化基准
    if (!angle_base_initialized_) {
        if (raw_angle >= 0 && raw_angle < 400) {
            angle_base_ = raw_angle;
            angle_base_initialized_ = true;
            ROS_WARN("角度基准设为: %d (位置%d)", angle_base_, phase1_current_position_);
        } else {
            ROS_ERROR("raw_angle异常: %d，跳过初始化", raw_angle);
            // 使用默认值
            angle_base_ = 0;
            angle_base_initialized_ = true;
        }
    }
    
    
    // 【关键修复】角度校正计算
    int corrected = raw_angle - angle_base_;
    
    // 循环加/减400直到在0-399范围内
    while (corrected < 0) {
        corrected += 400;
    }
    while (corrected >= 400) {
        corrected -= 400;
    }
    
    // 验证校正结果
    if (corrected < 0 || corrected >= 400) {
        ROS_ERROR("角度校正错误: raw=%d, base=%d, corrected=%d", 
                  raw_angle, angle_base_, corrected);
        // 紧急回退：直接使用raw_angle
        corrected = raw_angle;
        while (corrected >= 400) corrected -= 400;
        while (corrected < 0) corrected += 400;
    }
    
    float angle_deg = static_cast<float>(corrected) * 0.9f;
    
    // 角度验证
    if (angle_deg < -360.0f || angle_deg > 360.0f) {
        ROS_ERROR("角度计算异常: raw=%d, base=%d, corrected=%d, angle_deg=%.1f", 
                  raw_angle, angle_base_, corrected, angle_deg);
        angle_deg = fmod(angle_deg, 360.0f);
    }
    
    
    // ==================== 角度复位结束 ====================


    // ========== 角度处理链监测 ==========
    static int frame_count = 0;
    frame_count++;
    
    // 每100帧或在Phase1关键阶段输出
    if (frame_count % 100 == 0 || 
        (phase1_enabled_ && (phase1_step_ == 1 || phase1_step_ == 3))) {
        
        ROS_INFO("角度处理[帧%d]: raw=%d -> corrected=%d -> final=%.1f°, 阶段=%d, 位置=%d", 
                frame_count, raw_angle, corrected, angle_deg, phase1_step_, phase1_current_position_);
    }
    
    // 在位置1和位置2开始时特别记录
    if (phase1_enabled_) {
        static bool pos1_started = false, pos2_started = false;
        
        if (phase1_step_ == 1 && !pos1_started) {
            ROS_WARN(">>>> 位置1开始：第一个角度 raw=%d, corrected=%d, final=%.1f°", 
                    raw_angle, corrected, angle_deg);
            pos1_started = true;
            pos2_started = false;
        }
        else if (phase1_step_ == 3 && !pos2_started) {
            ROS_WARN(">>>> 位置2开始：第一个角度 raw=%d, corrected=%d, final=%.1f°", 
                    raw_angle, corrected, angle_deg);
            pos2_started = true;
        }
    }
    // ========== 角度处理链监测结束 ==========


    // static std::vector<float> recent_angles_debug;
    static int angle_debug_counter = 0;

    // 记录当前角度
    recent_angles_debug_.push_back(angle_deg);
    if (recent_angles_debug_.size() > 100) {
        recent_angles_debug_.erase(recent_angles_debug_.begin());
    }

    angle_debug_counter++;

    // 每50帧输出统计
    if (angle_debug_counter % 50 == 0 && !recent_angles_debug_.empty()) {
        // 计算统计
        float min_angle = *std::min_element(recent_angles_debug_.begin(), recent_angles_debug_.end());
        float max_angle = *std::max_element(recent_angles_debug_.begin(), recent_angles_debug_.end());
        
        float sum = 0;
        for (float ang : recent_angles_debug_) sum += ang;
        float avg_angle = sum / recent_angles_debug_.size();
        
        ROS_INFO("processScanFrame角度统计[最近%lu帧]:", recent_angles_debug_.size());
        ROS_INFO("  最小=%.1f°, 最大=%.1f°, 平均=%.1f°", min_angle, max_angle, avg_angle);
        
        // 检查是否都在-45°附近
        int count_near_minus45 = 0;
        for (float ang : recent_angles_debug_) {
            if (fabs(ang - (-45.0f)) < 5.0f) {
                count_near_minus45++;
            }
        }
        
        float percentage = count_near_minus45 * 100.0f / recent_angles_debug_.size();
        ROS_INFO("  在-45°±5°范围内的帧: %d/%lu (%.1f%%)", 
                count_near_minus45, recent_angles_debug_.size(), percentage);
        
        if (percentage > 80.0f) {
            ROS_WARN("⚠️ 警告: 大部分角度在-45°附近！可能有问题！");
            ROS_WARN("当前帧: raw_angle=%d, corrected=%d, angle_base=%d, angle_deg=%.1f°", 
                    raw_angle, corrected, angle_base_, angle_deg);
        }
        
        // 清空统计
        recent_angles_debug_.clear();
    }

     
    ////////////
    static std::map<int, std::vector<int>> raw_angles_by_step;  // 按阶段记录原始raw_angle
    
    if (phase1_enabled_) {
        raw_angles_by_step[phase1_step_].push_back(raw_angle);
        
        // 每收集10个样本输出统计
        if (raw_angles_by_step[phase1_step_].size() % 10 == 0) {
            int size = raw_angles_by_step[phase1_step_].size();
            ROS_INFO("阶段%d: raw_angle范围 %d-%d, 样本数=%d", 
                    phase1_step_,
                    raw_angles_by_step[phase1_step_].front(),
                    raw_angles_by_step[phase1_step_].back(),
                    size);
        }
    }

    // static std::vector<float> scanned_angles;
    scanned_angles_.push_back(angle_deg);
    
    
    // 每扫描10个角度统计一次
    if (scanned_angles_.size() % 10 == 0) {
        float min_angle = *std::min_element(scanned_angles_.begin(), scanned_angles_.end());
        float max_angle = *std::max_element(scanned_angles_.begin(), scanned_angles_.end());
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
    
    // 角度覆盖统计
    // static std::set<int> covered_angles;
    int angle_int = static_cast<int>(angle_deg);
    covered_angles_.insert(angle_int);
    
    // 每30度输出一次角度覆盖情况
    static int last_angle_report = -1;
    if (angle_int % 30 == 0 && angle_int != last_angle_report) {
        last_angle_report = angle_int;
        ROS_INFO("角度覆盖: %d° (累计 %lu/360 度)", angle_int, covered_angles_.size());
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
    
    // 调试输出距离参数
    static int debug_counter = 0;
    if (debug_counter % 100 == 0) {
        ROS_INFO("距离参数: sample_period=%u, time_per_point=%.3fus, dist_per_point=%.3fmm", 
                msg_data.sample_period, time_per_point_us, distance_per_point_mm);
        debug_counter = 0;
    }
    debug_counter++;
    
    // ==================== 3处理每个采样点 ====================
    int valid_samples = 0;
    int use_cnt = 0;  // 使用同事的计数器命名
    
    // 新增: 记录该角度的绿色点云最小距离
    static std::map<int, float> angle_to_green_distance; // 角度索引 -> 绿色点云最小距离
    
    // 获取实际的采样点数
    uint16_t actual_samples = std::min(msg_data.number_of_samples, msg_data.data_length);
    
    // 计算理论最大距离
    float max_theoretical_range_mm = distance_per_point_mm * actual_samples;
    float max_theoretical_range_m = max_theoretical_range_mm * 0.001f;
    if (max_range_ < max_theoretical_range_m - 2.0f) {
        ROS_WARN_THROTTLE(10.0, "max_range(%.1fm)可能小于硬件能力(%.1fm)", 
                         max_range_, max_theoretical_range_m);
    }
    
    // 新增: 统计绿色点云数量
    int green_points_in_this_frame = 0;
 

    for (uint16_t i = 0; i < actual_samples; i++) {
        // 计算每个采样点的实际距离
        float distance_mm = distance_per_point_mm * (i + 1);  // 毫米
        float distance_m = distance_mm * 0.001f;              // 米
        if (distance_m > max_range_) {
            continue;  // 跳过这个点
        }
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
        else if (distance_m <= 1.5f && distance_m >0.8f) {
            // 0.8-1.5米：信号强，要求高
            min_required_intensity = 150.0f;
            if (intensity >= 160.0f) {
                is_green_point_candidate = true;
            }
        } 
        else if (distance_m <= 3.0f && distance_m >1.5f ) {
            // 1.5-3米：主要范围，信号中等
            min_required_intensity = 140.0f;
            if (intensity >= 150.0f) {
                is_green_point_candidate = true;
            }
        } 
        else if (distance_m <= 6.0f && distance_m >3.0f ) {
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
    // ==================== 4调试信息输出增强 ====================
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
        
        // ==================== 5优化更新频率 ====================
        // 立即转换点云（保持实时性）
        convertToPointCloud();
        
        // 修改: 基于绿色点云数量决定更新频率
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
                    frame_counter, total_points, total_green_points, angle_deg, covered_angles_.size());
            
            // 检查是否存在角度盲区
            std::vector<int> missing_angles;
            for (int a = 0; a < 360; a += 30) {
                if (covered_angles_.find(a) == covered_angles_.end()) {
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
        
        // 记录哪些角度没有有效数据
        static std::set<int> no_data_angles;
        no_data_angles.insert(angle_int);
        
        if (dropped_scans_ % 50 == 0) {
            ROS_WARN("已丢弃 %u 帧无效数据, 无数据角度: %lu个", 
                    dropped_scans_, no_data_angles.size());
        }
    }
    
 
   // ==================== 6360度扫描检测优化 + 静态建图流程 ====================
    // 如果第一阶段启用，跳过原有的静态建图逻辑
    if (phase1_enabled_) {
        // 第一阶段模式不需要检查360度扫描完成
        return;
    }

    // 原有静态建图逻辑（仅当phase1未启用时执行）
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
        
        ROS_INFO("静态建图第%d次扫描完成，耗时%.1f秒，绿色点云: %d", 
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
 
    // ==================== 数据累积和增强处理 ====================
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


void PhaseFusionSonarMapper::saveStageMap(const std::string& stage_name) {
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
    
    // 4. 新增保存点云数据
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
        
        // 新增点云文件信息
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

bool PhaseFusionSonarMapper::saveMapAsPGM(const std::string& filename, 
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

bool PhaseFusionSonarMapper::saveMapAsYAML(const std::string& filename,
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
void PhaseFusionSonarMapper::accumulateSectorData(int sector_index, const SonarPoint& point) {
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
void PhaseFusionSonarMapper::generateDensePointCloud() {
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

void PhaseFusionSonarMapper::generateControlCommand(unsigned char* buffer, unsigned short angle) {
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
    
    // ========== 只需要修改这3个参数 ==========
    // 发射持续时间 (107us 而不是200us)
    buffer[12] = 0x6B;  // 107 (参考代码: 0x6B)
    buffer[13] = 0x00;
    
    // 采样周期 (888 而不是100) - 最！
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
    
    // 添加调试输出
    static bool first_run = true;
    if (first_run) {
        ROS_INFO("控制命令参数调整:");
        ROS_INFO("  采样周期: %u -> 888", (buffer[15] << 8) | buffer[14]);
        ROS_INFO("  发射持续时间: %uus -> 107us", (buffer[13] << 8) | buffer[12]);
        ROS_INFO("  数据长度: %u -> 1", (buffer[21] << 8) | buffer[20]);
        first_run = false;
    }
}

 

void PhaseFusionSonarMapper::scanningThread() {
    ROS_INFO("扫描线程启动");
    
    // 使用成员变量 scan_start_angle_
    unsigned short angle = static_cast<unsigned short>(scan_start_angle_);
    
    // 验证角度有效性
    if (angle >= 400) {
        angle = 0;
        scan_start_angle_ = 0;
        ROS_WARN("起始角度 %d 无效，重置为0", scan_start_angle_);
    }
    
    ROS_INFO("扫描从角度 %u 开始", angle);
    
    scan_start_time_ = ros::Time::now();
    
    // 【修复】使用简单直接的圈数计数
    int circles_completed_in_session = 0;
    
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
        
        // 角度递增
        angle += 1;
        
        // 检查是否完成一圈（400个角度命令）
        if (angle >= 400) {
            angle = 0;
            circles_completed_in_session++;
            
            // 【关键修复】简单的圈数完成检测
            full_scan_complete_.store(true);
            
            ros::Duration scan_duration = ros::Time::now() - scan_start_time_;
            
            if (phase1_enabled_) {
                ROS_INFO("Phase1: 完成第%d圈扫描! 耗时: %.1f 秒", 
                        circles_completed_in_session, scan_duration.toSec());
            } else {
                ROS_INFO("完成第%d圈扫描! 耗时: %.1f 秒, 总帧数: %u", 
                        circles_completed_in_session, scan_duration.toSec(), total_scans_);
                
                // 非Phase1模式保存数据
                saveScanData("/tmp/full_scan_" + std::to_string(time(nullptr)));
                saveCostMap("/tmp/cost_map_" + std::to_string(time(nullptr)));
                
                total_scans_ = 0;
                valid_scans_ = 0;
            }
            
            // 重置计时器
            scan_start_time_ = ros::Time::now();
        }
        
     
        ros::Duration(0.1).sleep();
    }
    
    // 记录结束角度
    last_scan_end_angle_ = current_angle_.load();
    ROS_INFO("扫描线程停止，最后角度: %d", last_scan_end_angle_);
}

// ======================== 点云转换 ========================
void PhaseFusionSonarMapper::convertToPointCloud() {
  
    pointcloud_->clear();

    //重要：声纳硬件坐标系定义
    // 声纳0°方向 = 笛卡尔X+轴（右侧）
    // 但RViz期望：机器人前方 = Y+轴
    // 因此需要旋转90°：X+ → Y+
    const float rotation_angle_deg = 90.0f; 
   
    const float rotation_angle_rad = rotation_angle_deg * DEG_TO_RAD;
    const float cos_theta = cosf(rotation_angle_rad);
    const float sin_theta = sinf(rotation_angle_rad);

    static int call_count = 0;
    call_count++;
    ROS_WARN("[坐标诊断] convertToPointCloud 第%d次被调用. rotation_angle_rad=%.4f (%.1f°)", 
             call_count, rotation_angle_rad, rotation_angle_rad * RAD_TO_DEG);


    // 添加调试输出
    static bool first_debug = true;
    if (first_debug && !sonar_points_.empty()) {
        float original_angle = sonar_points_[0].angle * RAD_TO_DEG;
        ROS_WARN("角度处理: 原始角度=%.1f°, 应用旋转=%.1f°, 最终角度=%.1f°", 
                original_angle, rotation_angle_deg, 
                original_angle + rotation_angle_deg);
        first_debug = false;
    }

    
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
            
            // 2. 应用-90度旋转（）
            pcl::PointXYZI point;
            
            //-90度旋转（）
            point.x = cos_theta * xx - sin_theta * yy;
            point.y = sin_theta * xx + cos_theta * yy;
 
            point.z = 0.0f;
            point.intensity = intensity;
           
            pointcloud_->points.push_back(point);

            static int dir_check_count = 0;
            if (dir_check_count < 10) {
                // 计算点相对于机器人的方向
                float angle = atan2(point.y, point.x) * RAD_TO_DEG;
                
                // 方向分类
                const char* direction = "其他";
                if (angle >= -45 && angle < 45) direction = "前";
                else if (angle >= 45 && angle < 135) direction = "右";
                else if (angle >= 135 || angle < -135) direction = "后";
                else if (angle >= -135 && angle < -45) direction = "左";
                
                ROS_WARN("方向验证[%d]: 坐标(%.3f,%.3f), 角度=%.1f°, 方向=%s",
                        dir_check_count, point.x, point.y, angle, direction);
                dir_check_count++;
            }
                        
            // 调试输出
            if (debug_point_count < MAX_DEBUG_POINTS) {
                // 计算旋转后角度
                float rotated_angle = atan2(point.y, point.x) * RAD_TO_DEG;
                
                ROS_ERROR("【关键验证】点云点 %d:", debug_point_count);
                ROS_ERROR("  输入: 角度=%.1f°, 距离=%.3fm", original_angle_deg, range);
                ROS_ERROR("  原始坐标: (%.4f, %.4f)", xx, yy);
                ROS_ERROR("  旋转后坐标: (%.4f, %.4f), 角度=%.1f°", point.x, point.y, rotated_angle);
                ROS_ERROR("  旋转参数: cos_theta=%.3f, sin_theta=%.3f (%.1f°)", 
                        cos_theta, sin_theta, rotation_angle_deg);
                
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
        // 前方标记（红色）- 注意：由于应用了-90°旋转，前方现在应该是Y+方向
        visualization_msgs::Marker front_marker;
        front_marker.header.frame_id = "sonar_frame";
        front_marker.header.stamp = now;
        front_marker.ns = "directions";
        front_marker.id = 0;
        front_marker.type = visualization_msgs::Marker::ARROW;
        front_marker.action = visualization_msgs::Marker::ADD;
        
        geometry_msgs::Point start, end;
        start.x = 0; start.y = 0; start.z = 0;

        if (rotation_angle_deg == -45.0f) {
        // -45°旋转：前方可能在X和Y之间的方向
            end.x = 1.414; end.y = 1.414; end.z = 0;  // 45°方向
            ROS_DEBUG("发布方向标记：-45°旋转，前方在45°方向");
        } else if (rotation_angle_deg == -90.0f) {
            // -90°旋转：前方在Y+方向
            end.x = 0; end.y = 2.0; end.z = 0;
            ROS_DEBUG("发布方向标记：-90°旋转，前方在Y+方向");
        } else {
            // 其他旋转角度
            float rad = -rotation_angle_rad;  // 注意负号
            end.x = 2.0 * cosf(rad);
            end.y = 2.0 * sinf(rad);
            end.z = 0;
            ROS_DEBUG("发布方向标记：%.1f°旋转", rotation_angle_deg);
        }
        // 由于应用了-90°旋转，原本的前方(X+)现在变成了右方(Y+)
        // 但为了与rviz网格对齐，我们标记Y+为"前方"
        // end.x = 0; end.y = 2.0; end.z = 0;  // 现在Y+是"前方"
        
        front_marker.points.push_back(start);
        front_marker.points.push_back(end);
        front_marker.scale.x = 0.05;
        front_marker.scale.y = 0.1;
        front_marker.color.r = 1.0;  // 红色
        front_marker.color.g = 0.0;
        front_marker.color.b = 0.0;
        front_marker.color.a = 1.0;
        
        marker_pub_.publish(front_marker);
        
        // 右侧标记（绿色）- 原本的右方(Y+)现在变成了后方(-X)
        visualization_msgs::Marker right_marker = front_marker;
        right_marker.id = 1;
        right_marker.color.r = 0.0;
        right_marker.color.g = 1.0;  // 绿色
        end.x = -2.0; end.y = 0; end.z = 0;  // 左侧（原本的前方）
        right_marker.points.clear();
        right_marker.points.push_back(start);
        right_marker.points.push_back(end);
        
        marker_pub_.publish(right_marker);
        
        last_marker_time = now;
        
        ROS_DEBUG("发布方向标记：红色=前方(Y+)，绿色=左侧(-X)");
    }
}

// ======================== 点云滤波 ========================
void PhaseFusionSonarMapper::filterPointCloud() {
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
 

void PhaseFusionSonarMapper::createOccupancyGrid() {
    // 重置地图数据为未知
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    if (pointcloud_->empty()) {
        ROS_DEBUG_THROTTLE(2.0, "点云为空，跳过地图更新");
        return;
    }
    
    // ==================== 核心修改: 水池边界检测 ====================
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
void PhaseFusionSonarMapper::createCostMap() {
    // 新增控制代价地图更新频率，避免频繁更新导致重影
    static ros::Time last_costmap_update = ros::Time::now();
    ros::Duration time_since_update = ros::Time::now() - last_costmap_update;
    
    // 每1.5秒更新一次代价地图（比占据地图慢）
    if (time_since_update.toSec() < 1.5) {
        ROS_DEBUG_THROTTLE(2.0, "代价地图更新频率控制：跳过本次更新");
        return;
    }
    last_costmap_update = ros::Time::now();
    
    // 新增检查是否有足够的数据
    if (filtered_pointcloud_->empty() || filtered_pointcloud_->size() < 100) {
        ROS_DEBUG("点云数据不足，跳过代价地图更新");
        return;
    }
    
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
    std::map<int, pcl::PointXYZI> sector_contour_points;
    const float angle_bin_size = 2.0f * DEG_TO_RAD;
    
    for (const auto& point : filtered_pointcloud_->points) {
        float angle = atan2(point.y, point.x);
        float distance = sqrt(point.x * point.x + point.y * point.y);
        
        // 新增过滤远距离噪声
        if (distance > 10.0f) {
            continue;
        }
        
        float normalized_angle = angle + M_PI;
        int sector_index = static_cast<int>(normalized_angle / angle_bin_size);
        
        auto it = sector_contour_points.find(sector_index);
        if (it == sector_contour_points.end() || point.intensity > it->second.intensity) {
            sector_contour_points[sector_index] = point;
        }
    }
    
    ROS_DEBUG_THROTTLE(2.0, "提取到 %lu 个扇区轮廓点", sector_contour_points.size());
    
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
            
            // 更新代价单元
            CostCell& cell = cost_cells_[index];
            
            // 轮廓点给予更高的权重
            cell.hits += 3;
            cell.occupancy = std::min(1.0f, cell.occupancy + 0.3f);
            cell.cost = cell.occupancy * cell.occupancy;
            
            // 根据占据概率设置代价值（0-100）
            int cost_value = static_cast<int>(cell.cost * 100.0f);
            cost_map_.data[index] = std::min(100, std::max(0, cost_value));
            
            contour_points_used++;
            
            // 增强墙壁连续性（小范围）
            int fill_radius = 1;
            for (int dy = -fill_radius; dy <= fill_radius; dy++) {
                for (int dx = -fill_radius; dx <= fill_radius; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = costmap_x + dx;
                    int ny = costmap_y + dy;
                    
                    if (nx >= 0 && nx < costmap_width_ && ny >= 0 && ny < costmap_height_) {
                        int nidx = ny * costmap_width_ + nx;
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
    
    // ==================== 第三步：处理自由空间 ====================
    int robot_costmap_x = static_cast<int>((0.0 - costmap_origin_x_) / costmap_resolution_);
    int robot_costmap_y = static_cast<int>((0.0 - costmap_origin_y_) / costmap_resolution_);
    
    if (robot_costmap_x >= 0 && robot_costmap_x < costmap_width_ && 
        robot_costmap_y >= 0 && robot_costmap_y < costmap_height_) {
        
        int free_radius = static_cast<int>(2.0 / costmap_resolution_);
        for (int y = robot_costmap_y - free_radius; y <= robot_costmap_y + free_radius; y++) {
            for (int x = robot_costmap_x - free_radius; x <= robot_costmap_x + free_radius; x++) {
                if (x >= 0 && x < costmap_width_ && y >= 0 && y < costmap_height_) {
                    int idx = y * costmap_width_ + x;
                    float distance_to_robot = sqrt(pow(x - robot_costmap_x, 2) + pow(y - robot_costmap_y, 2)) * costmap_resolution_;
                    
                    if (distance_to_robot < 1.5f && cost_map_.data[idx] < 30) {
                        cost_map_.data[idx] = 0;
                    }
                }
            }
        }
    }
    
    // ==================== 第四步：后处理 - 平滑与降噪 ====================
    std::vector<int8_t> smoothed_data = cost_map_.data;
    for (int y = 1; y < costmap_height_ - 1; y++) {
        for (int x = 1; x < costmap_width_ - 1; x++) {
            int idx = y * costmap_width_ + x;
            
            if (cost_map_.data[idx] > 60) {
                int occupied_neighbors = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nidx = (y + dy) * costmap_width_ + (x + dx);
                        if (cost_map_.data[nidx] > 40) occupied_neighbors++;
                    }
                }
                
                if (occupied_neighbors < 2) {
                    smoothed_data[idx] = std::max(0, cost_map_.data[idx] / 2);
                }
            }
        }
    }
    cost_map_.data = smoothed_data;
    
    // ==================== 第五步：统计与日志 ====================
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
    
    ROS_DEBUG_THROTTLE(3.0, "代价地图更新完成: 占据=%d, 空闲=%d, 未知=%d", 
                      occupied_cells, free_cells, unknown_cells);
}

 

// ======================== 发布地图数据 ========================
void PhaseFusionSonarMapper::publishMapData() {
    ros::Time current_time = ros::Time::now();
    // 添加追踪1记录发布次数和阶段
    static int publish_count = 0;
    static int last_phase_step = -1;
    
    if (phase1_step_ != last_phase_step) {
        last_phase_step = phase1_step_;
        ROS_WARN("=== publishMapData阶段变化: %d ===", phase1_step_);
    }
    
    // 添加追踪2检查TF变换的旋转
    if (publish_count % 20 == 0) {  // 每20次发布检查一次
        try {
            // 检查 map->sonar_frame 的变换
            tf::StampedTransform map_to_sonar;
            tf_listener_.lookupTransform("map", "sonar_frame", 
                                        ros::Time(0), map_to_sonar);
            
            float yaw = tf::getYaw(map_to_sonar.getRotation()) * RAD_TO_DEG;
            if (fabs(yaw) > 1.0f) {  // 如果有显著旋转
                ROS_WARN("TF map->sonar_frame有旋转: %.1f°", yaw);
                
                // 输出变换详情
                ROS_WARN("  平移: (%.3f, %.3f, %.3f)", 
                        map_to_sonar.getOrigin().x(),
                        map_to_sonar.getOrigin().y(),
                        map_to_sonar.getOrigin().z());
            }
            
            // 检查 odom->base_link 的变换（来自硬件驱动）
            tf::StampedTransform odom_to_base;
            if (tf_listener_.canTransform("odom", "base_link", ros::Time(0))) {
                tf_listener_.lookupTransform("odom", "base_link", 
                                           ros::Time(0), odom_to_base);
                float odom_yaw = tf::getYaw(odom_to_base.getRotation()) * RAD_TO_DEG;
                ROS_INFO("odom->base_link旋转: %.1f°", odom_yaw);
            }
            
        } catch (tf::TransformException& ex) {
            ROS_DEBUG("TF检查失败: %s", ex.what());
        }
    }
    
    // 添加追踪3检查点云数据的方向
    if (publish_count % 30 == 0 && !pointcloud_->empty()) {
        // 计算点云的主要方向（通过特征值分解或简单统计）
        float sum_x = 0, sum_y = 0;
        float max_dist = 0;
        float farthest_angle = 0;
        
        for (const auto& p : pointcloud_->points) {
            sum_x += p.x;
            sum_y += p.y;
            
            float dist = sqrt(p.x*p.x + p.y*p.y);
            if (dist > max_dist && p.intensity > 140.0f) {
                max_dist = dist;
                farthest_angle = atan2(p.y, p.x) * RAD_TO_DEG;
            }
        }
        
        float avg_x = sum_x / pointcloud_->size();
        float avg_y = sum_y / pointcloud_->size();
        float avg_angle = atan2(avg_y, avg_x) * RAD_TO_DEG;
        
        ROS_INFO("点云统计[阶段%d]: 平均方向=%.1f°, 最远点方向=%.1f°", 
                phase1_step_, avg_angle, farthest_angle);
    }
    
    // 添加追踪4检查地图数据的方向
    if (publish_count % 40 == 0 && !occupancy_map_.data.empty()) {
        // 找到地图中占据值最高的单元格
        int max_occupancy = -1;
        int max_x = 0, max_y = 0;
        
        for (int y = 0; y < map_height_; y++) {
            for (int x = 0; x < map_width_; x++) {
                int val = occupancy_map_.data[y * map_width_ + x];
                if (val > max_occupancy) {
                    max_occupancy = val;
                    max_x = x;
                    max_y = y;
                }
            }
        }
        
        if (max_occupancy > 50) {
            // 转换到世界坐标
            float world_x = map_origin_x_ + (max_x + 0.5) * map_resolution_;
            float world_y = map_origin_y_ + (max_y + 0.5) * map_resolution_;
            float angle = atan2(world_y, world_x) * RAD_TO_DEG;
            
            ROS_INFO("地图最强点[阶段%d]: 坐标(%.2f, %.2f), 角度=%.1f°, 强度=%d", 
                    phase1_step_, world_x, world_y, angle, max_occupancy);
        }
    }
    
    publish_count++;
    // 首先发布必要的TF变换
    try {
        // 1. map -> odom（静态变换）
        tf::Transform map_to_odom;
        map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(map_to_odom, current_time, "map", "odom"));
    
            
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "TF发布异常: %s", e.what());
    }
    
    // 发布原始点云（在sonar_frame中）
    if (!pointcloud_->empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pointcloud_, cloud_msg);
        cloud_msg.header.stamp = current_time;
        cloud_msg.header.frame_id = "map";  // 修改从sonar_frame改为map
        pointcloud_pub_.publish(cloud_msg);
        
        ROS_DEBUG_THROTTLE(2.0, "发布点云: %lu点, 坐标系: map", pointcloud_->size());
    }

    static std::string last_map_frame_id = "";
    if (occupancy_map_.header.frame_id != last_map_frame_id) {
        last_map_frame_id = occupancy_map_.header.frame_id;
        ROS_WARN("地图坐标系变化: %s", last_map_frame_id.c_str());
    }
    
    // 发布占据栅格地图（在map中）
    if (!occupancy_map_.data.empty()) {
        occupancy_map_.header.stamp = current_time;
        occupancy_map_.header.frame_id = "map";
        occupancy_pub_.publish(occupancy_map_);
        
        ROS_DEBUG_THROTTLE(2.0, "发布地图: %dx%d", 
                          occupancy_map_.info.width, occupancy_map_.info.height);
    }
    
    // 发布滤波后的点云
    if (!filtered_pointcloud_->empty()) {
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_pointcloud_, filtered_cloud_msg);
        filtered_cloud_msg.header.stamp = current_time;
        filtered_cloud_msg.header.frame_id = "map";
        filtered_pointcloud_pub_.publish(filtered_cloud_msg);
    }
    
    // 发布代价地图
    if (!cost_map_.data.empty()) {
        cost_map_.header.stamp = current_time;
        cost_map_.header.frame_id = "map";
        costmap_pub_.publish(cost_map_);
    }
    
    // 发布可视化标记
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
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
 
void PhaseFusionSonarMapper::publishTF() {
    ros::Time current_time = ros::Time::now();
    
    try {
        // 只发布两个必要的静态变换
        // 1. map -> odom（总是零变换）
        tf::Transform map_to_odom;
        map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(map_to_odom, current_time, "map", "odom"));
        
        // 2. base_link -> sonar_frame（固定硬件偏移）
        // tf::Transform base_to_sonar;
        // base_to_sonar.setOrigin(tf::Vector3(0.1, 0, 0.05));  // 硬件参数
        // base_to_sonar.setRotation(tf::Quaternion(0, 0, 0, 1));
        // tf_broadcaster_.sendTransform(
        //     tf::StampedTransform(base_to_sonar, current_time, "base_link", "sonar_frame"));
            
        // 不要发布 map->sonar_frame！让TF链自动计算
        
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "TF发布异常: %s", e.what());
    }
}
// ======================== 开始扫描 ========================

void PhaseFusionSonarMapper::startScanning() {
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
    scan_thread_ = boost::thread(&PhaseFusionSonarMapper::scanningThread, this);
    
    ROS_INFO("开始实时扫描");
}

// ======================== 停止扫描 ========================

void PhaseFusionSonarMapper::stopScanning() {
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

bool PhaseFusionSonarMapper::saveScanData(const std::string& filename) {
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

bool PhaseFusionSonarMapper::saveCostMap(const std::string& filename) {
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

void PhaseFusionSonarMapper::clearScanData() {
    ROS_INFO("开始清空所有数据...");
    
    // 清空基础数据
    sonar_points_.clear();
    pointcloud_->clear();
    filtered_pointcloud_->clear();
    raw_points_buffer_.clear();
    
    // 清空扇区数据（如果使用了）
    sector_data_.clear();
    
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
    full_scan_complete_.store(false);
    
    // 重置解析状态
    parse_state_ = STATE_IDLE;
    current_message_.clear();
    rx_end_index_ = 0;
    rx_used_index_ = 0;
 
    
    ROS_INFO("数据清空完成: sonar_points=%lu, pointcloud=%lu", 
             sonar_points_.size(), pointcloud_->size());
}
// ======================== CRC16计算 ========================

uint16_t PhaseFusionSonarMapper::calculateCRC16(const uint8_t* data, size_t length) {
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

// ======================== CSV日志记录 ========================

void PhaseFusionSonarMapper::logToCSV(const SonarPoint& point) {
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
bool PhaseFusionSonarMapper::shouldGenerateDenseCloud() const {
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


void PhaseFusionSonarMapper::checkAndPublishInitialMap() {

    if (phase1_enabled_) {
        // 添加额外的状态检查
        static int last_phase1_step = -1;
        if (phase1_step_ != last_phase1_step) {
            ROS_INFO("Phase1 步骤变化: %d -> %d", last_phase1_step, phase1_step_);
            last_phase1_step = phase1_step_;
        }
        
        handlePhase1Mapping();
        return;  // 第一阶段模式下，跳过原有的静态建图逻辑
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
                    // 能量阈值已调整从150降低到130
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
                // 能量阈值调整从150降低到130
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
        
        // 修改点1: 时间至少90秒
        bool time_condition = scan_duration.toSec() > 90.0f;
        
        // 修改点2: 降低数据量要求（因为能量阈值降低了）
        bool data_condition = current_total_points > 150 && current_green_points > 60;
        
        // 修改点3: 降低角度覆盖要求
        bool coverage_condition = angle_coverage > 280;  // 从306降低到280（78%）
        
        // 修改点4: 扫描圈数要求（至少2圈完整扫描）
        bool round_condition = scan_round_count >= 2;
        
        // 数据稳定
        bool stability_condition = stable_counter >= 3;
        
        // 没有新的显著数据
        bool no_new_data_condition = (since_last_update.toSec() > 15.0f);
        
        // 修改点5: 完成需要满足所有核心条件
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



void PhaseFusionSonarMapper::createEnhancedOccupancyGrid() {
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
                
                
                // 高质量绿色点云（墙壁）
                bool keep_point = false;
                if (range < 0.5f) {
                    if (intensity >= 160.0f) {  // 高质量要求
                        keep_point = true;
                    }
                }
                // 0.5-3.5米：主要范围，中等质量即可
                else  if (range >= 0.5f && range <= 3.5f) {
                    if (intensity >= 150.0f) {  // 中等质量要求
                        keep_point = true;
                    }
                }
                // 3.5-6.0米：中距离，需要稍高质量
                else if (range > 3.5f && range <= 6.0f) {
                    if (intensity >= 140.0f) {  // 较高质量要求
                        keep_point = true;
                    }
                }
                // 6.0-8.0米：远距离，但如果有很强信号可能是墙壁
                else if (range > 6.0f && range <= 8.0f) {
                    if (intensity >= 120.0f) {  // 需要高强度
                        keep_point = true;
                    }
                }
                // 超过8米：很可能是噪声，需要极高强度才考虑
                else if (range > 8.0f && range <= 10.0f) {
                    if (intensity >= 100.0f) {  // 需要极高强度
                        keep_point = true;
                    }
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
                    if (point.intensity >= 140.0f && distance >= 0.5f && distance <= 5.0f) {
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


void PhaseFusionSonarMapper::handlePhase1Mapping() {
    if (!phase1_enabled_) return;

    ros::Time now = ros::Time::now();
    
    if (phase1_step_ == 0 && is_scanning_.load()) {
        ROS_INFO("========================================");
        ROS_INFO("=== Phase1 开始第一阶段建图 ===");
        ROS_INFO("========================================");
        phase1_step_ = 1;
        phase1_step_start_time_ = now;
        
        // 初始化位置1参数
        phase1_current_position_ = 1;
        circles_completed_in_position_ = 0;
        
        // 第一次扫描从0度开始
        scan_start_angle_ = 0;
        ROS_INFO("位置1扫描从角度0开始");
        
        // 确保标志初始状态
        full_scan_complete_.store(false);
        
        publishPhase1Status("位置1建图中");
        return;
    }

    const float TIMEOUT_PER_POSITION = 200.0f;

    switch (phase1_step_) {
        case 1: // 第一个位置建图
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                
                // 【修复】检查是否完成一圈扫描
                if (full_scan_complete_.load()) {
                    // 立即重置标志
                    full_scan_complete_.store(false);
                    
                    circles_completed_in_position_++;
                    
                    // 【重要】添加短暂延迟，确保扫描线程看到标志已重置
                    ros::Duration(0.05).sleep();
                    
                    ROS_INFO("【位置1】确认完成第%d圈扫描（要求：%d圈）", 
                            circles_completed_in_position_, required_circles_per_position_);
                    
                    // 检查是否完成要求的圈数
                    if (circles_completed_in_position_ >= required_circles_per_position_) {
                        ROS_INFO("=== 位置1完成 %d 圈扫描 ===", required_circles_per_position_);
                        
                        // 记录结束角度
                        last_scan_end_angle_ = current_angle_.load();
                        ROS_INFO("位置1扫描结束角度: %d", last_scan_end_angle_);
                        
                        // 保存第一个位置的子地图
                        saveCurrentPhase1Submap();
                        
                        if (is_scanning_.load()) {
                            stopScanning();
                        }
                        
                        phase1_step_ = 2;
                        phase1_step_start_time_ = now;
                        publishPhase1Status("等待移动到第二个位置");
                        
                        // 使用参数设置的等待时间
                        ROS_INFO("请人工移动到第二个位置（直线%.1f米），等待%.0f秒", 
                                phase1_move_distance_, phase1_wait_duration_);
                        return;
                    }
                }
                
                // 超时检查
                bool timeout_condition = (elapsed.toSec() >= TIMEOUT_PER_POSITION);
                
                // 进度输出
                static int last_report_sec = -1;
                int current_sec = static_cast<int>(elapsed.toSec());
                if (current_sec != last_report_sec && current_sec % 10 == 0) {
                    ROS_INFO("位置1: 已进行%d秒, 完成%d/%d圈, 当前角度: %d", 
                            current_sec, circles_completed_in_position_, 
                            required_circles_per_position_, current_angle_.load());
                    last_report_sec = current_sec;
                }
                
                if (timeout_condition) {
                    ROS_WARN("位置1超时（%d秒），强制停止", current_sec);
                    saveCurrentPhase1Submap();
                    
                    if (is_scanning_.load()) {
                        stopScanning();
                    }
                    
                    phase1_step_ = 2;
                    phase1_step_start_time_ = now;
                    publishPhase1Status("等待移动到第二个位置");
                }
            }
            break;
            
        case 2: // 等待移动到第二个位置
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                
                // 每5秒输出一次剩余时间
                static int last_reminder_sec = -1;
                int current_sec = static_cast<int>(elapsed.toSec());
                
                if (current_sec != last_reminder_sec && current_sec % 5 == 0) {
                    float remaining = phase1_wait_duration_ - elapsed.toSec();
                    if (remaining > 0) {
                        ROS_INFO("等待移动: 已等待%d秒，还需等待%.0f秒", 
                                current_sec, remaining);
                    }
                    last_reminder_sec = current_sec;
                }
                
                // 使用 phase1_wait_duration_ 参数
                if (elapsed.toSec() >= phase1_wait_duration_) {
                    ROS_INFO("=== 开始第二个位置建图 ===");
                    ROS_INFO("等待时间结束（%.0f秒）", phase1_wait_duration_);
                    
                    // 初始化位置2参数
                    phase1_current_position_ = 2;
                    circles_completed_in_position_ = 0;
                    
                    // 重置扫描状态
                    full_scan_complete_.store(false);
                    
                    // 位置2也从0度开始
                    scan_start_angle_ = 0;
                    ROS_INFO("位置2扫描从角度 0 开始");
                    
                    // 清空数据
                    sonar_points_.clear();
                    pointcloud_->clear();
                    filtered_pointcloud_->clear();
                    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
                    
                    // 重置扫描统计
                    total_scans_ = 0;
                    valid_scans_ = 0;
                    
                    if (!is_scanning_.load()) {
                        startScanning();
                    }
                    
                    phase1_step_ = 3;
                    phase1_step_start_time_ = now;
                    publishPhase1Status("第二个位置建图中");
                    
                    // 重置提醒时间
                    last_reminder_sec = -1;
                }
            }
            break;
            
        case 3: // 第二个位置建图
            {
                ros::Duration elapsed = now - phase1_step_start_time_;
                
                // 【修复】检查是否完成一圈扫描
                if (full_scan_complete_.load()) {
                    // 立即重置标志
                    full_scan_complete_.store(false);
                    
                    circles_completed_in_position_++;
                    
                    // 【重要】添加短暂延迟
                    ros::Duration(0.05).sleep();
                    
                    ROS_INFO("【位置2】确认完成第%d圈扫描（要求：%d圈）", 
                            circles_completed_in_position_, required_circles_per_position_);
                    
                    // 检查是否达到要求的圈数
                    if (circles_completed_in_position_ >= required_circles_per_position_) {
                        ROS_INFO("=== 位置2完成要求的%d圈扫描 ===", required_circles_per_position_);
                        
                        saveCurrentPhase1Submap();
                        
                        if (is_scanning_.load()) {
                            stopScanning();
                        }
                        
                        phase1_step_ = 4;
                        phase1_step_start_time_ = now;
                        publishPhase1Status("正在进行地图拼接");
                        
                        ROS_INFO("开始拼接两个位置的地图...");
                        mergePhase1Submaps();
                        return;
                    } else {
                        // 继续扫描下一圈
                        ROS_INFO("位置2还需扫描%d圈", 
                                required_circles_per_position_ - circles_completed_in_position_);
                    }
                }
                
                // 超时检查
                bool timeout_condition = (elapsed.toSec() >= TIMEOUT_PER_POSITION);
                
                // 进度输出
                static int last_report_sec = -1;
                int current_sec = static_cast<int>(elapsed.toSec());
                if (current_sec != last_report_sec && current_sec % 10 == 0) {
                    ROS_INFO("位置2: 已扫描%d秒, 完成%d/%d圈, 当前角度: %d", 
                            current_sec, circles_completed_in_position_, 
                            required_circles_per_position_, current_angle_.load());
                    last_report_sec = current_sec;
                }
                
                if (timeout_condition) {
                    ROS_WARN("位置2超时（%d秒），强制停止", current_sec);
                    saveCurrentPhase1Submap();
                    
                    if (is_scanning_.load()) {
                        stopScanning();
                    }
                    
                    phase1_step_ = 4;
                    phase1_step_start_time_ = now;
                    publishPhase1Status("正在进行地图拼接");
                    mergePhase1Submaps();
                }
            }
            break;
            
        case 4: // 拼接完成
            {
                static ros::Time last_map_publish = ros::Time(0);
                
                if ((now - last_map_publish).toSec() > 0.5) {
                    occupancy_map_.header.stamp = now;
                    occupancy_map_.header.frame_id = "map";
                    occupancy_pub_.publish(occupancy_map_);
                    
                    if (!pointcloud_->empty()) {
                        sensor_msgs::PointCloud2 cloud_msg;
                        pcl::toROSMsg(*pointcloud_, cloud_msg);
                        cloud_msg.header.stamp = now;
                        cloud_msg.header.frame_id = "map";
                        pointcloud_pub_.publish(cloud_msg);
                    }
                    
                    last_map_publish = now;
                }
            }
            break;
    }
}


void PhaseFusionSonarMapper::saveCurrentPhase1Submap() {
    if (phase1_submaps_.size() >= 10) {
        ROS_WARN("已经有%lu个子地图，水池建图最多需要2个位置", phase1_submaps_.size());
        return;
    }

    // ========== 新增：子地图关键点检查 ==========

    if (!pointcloud_->empty()) {
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*pointcloud_, min_pt, max_pt);
        ROS_WARN("[点云范围] 位置%d: X[%.3f, %.3f], Y[%.3f, %.3f]", 
                 phase1_current_position_, min_pt.x, max_pt.x, min_pt.y, max_pt.y);
    }

    // 在现有日志后添加边界框信息
    if (!pointcloud_->empty()) {
        // 计算边界框
        float min_x = pointcloud_->points[0].x;
        float max_x = pointcloud_->points[0].x;
        float min_y = pointcloud_->points[0].y;
        float max_y = pointcloud_->points[0].y;
        
        for (const auto& p : pointcloud_->points) {
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
        }
        
        ROS_WARN("  边界框: X[%.3f, %.3f] (宽度:%.3fm), Y[%.3f, %.3f] (宽度:%.3fm)",
                min_x, max_x, max_x - min_x,
                min_y, max_y, max_y - min_y);
    }
    
    int submap_idx = static_cast<int>(phase1_submaps_.size() + 1);
    ROS_WARN(">>>> 保存子地图%d (位置%d):", submap_idx, phase1_current_position_);

    
    if (!pointcloud_->empty()) {
        int point_count = static_cast<int>(pointcloud_->size());
        
        // 检查前3个点的坐标和角度
        ROS_WARN("  前3个点云点(转换后):");
        for (int i = 0; i < std::min(3, point_count); i++) {
            const auto& p = pointcloud_->points[i];
            float angle = atan2(p.y, p.x) * RAD_TO_DEG;
            float dist = sqrt(p.x*p.x + p.y*p.y);
            ROS_WARN("    点%d: (%.4f, %.4f), 距离=%.3f, 角度=%.1f°", 
                    i, p.x, p.y, dist, angle);
        }
        
        // 快速计算质心
        float sum_x = 0, sum_y = 0;
        for (const auto& p : pointcloud_->points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        float center_x = sum_x / pointcloud_->size();
        float center_y = sum_y / pointcloud_->size();
        ROS_WARN("  点云质心: (%.3f, %.3f)", center_x, center_y);
        
        // 检查角度分布
        int front = 0, right = 0, back = 0, left = 0;
        for (const auto& p : pointcloud_->points) {
            float angle = atan2(p.y, p.x) * RAD_TO_DEG;
            if (angle >= -45 && angle < 45) front++;
            else if (angle >= 45 && angle < 135) right++;
            else if (angle >= 135 || angle < -135) back++;
            else left++;
        }
        ROS_WARN("  方向分布: 前%d, 右%d, 后%d, 左%d", front, right, back, left);
        
        // 新增：检查点云统计
        int high_intensity = 0, medium_intensity = 0, low_intensity = 0;
        for (const auto& p : pointcloud_->points) {
            if (p.intensity >= 160.0f) high_intensity++;
            else if (p.intensity >= 140.0f) medium_intensity++;
            else low_intensity++;
        }
        ROS_WARN("  强度分布: 高≥160=%d, 中≥140=%d, 低=%d", 
                high_intensity, medium_intensity, low_intensity);
    } else {
        ROS_WARN("  警告：点云为空！");
    }
    // ========== 新增：子地图关键点检查结束 ==========
    
    Phase1SubMap submap;
    submap.index = phase1_submaps_.size() + 1;
    submap.timestamp = ros::Time::now();
    
    
   // 计算并保存质心
    if (!pointcloud_->empty()) {
        float sum_x = 0, sum_y = 0;
        for (const auto& p : pointcloud_->points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        submap.center_x = sum_x / pointcloud_->size();
        submap.center_y = sum_y / pointcloud_->size();
        
        ROS_WARN("子地图%d质心记录: (%.3f, %.3f)", 
                submap.index, submap.center_x, submap.center_y);
    }
    
    geometry_msgs::Pose2D robot_pose;
    
    if (phase1_current_position_ == 1) {
        // 【修改】位置1：固定为(0,0,0)，不依赖TF
        robot_pose.x = 0.0f;
        robot_pose.y = 0.0f;
        robot_pose.theta = 0.0f;
        
        ROS_INFO("位置1: 设为固定原点(0,0,0)");
        
        // 但仍然记录当前的odom位姿，用于参考
        try {
            tf::StampedTransform odom_to_base;
            ros::Time now = ros::Time::now();
            tf_listener_.lookupTransform("odom", "base_link", now, odom_to_base);
            ROS_INFO("位置1时odom位姿: (%.3f, %.3f, %.1f°)",
                    odom_to_base.getOrigin().x(),
                    odom_to_base.getOrigin().y(),
                    tf::getYaw(odom_to_base.getRotation()) * RAD_TO_DEG);
        } catch (...) {
            // 忽略错误
        }
    } 
    else if (phase1_current_position_ == 2) {
        // 位置2：直接读取当前位置的odom位姿
        // 因为位置1时odom是(0,0,0)，所以当前位置的odom就是相对位移

        tf::StampedTransform transform;
        ros::Time now = ros::Time::now();
        
        // 【恢复】查询 odom->base_link，而不是 map->base_link
        if (tf_listener_.waitForTransform("odom", "base_link", now, ros::Duration(0.5))) {
            tf_listener_.lookupTransform("odom", "base_link", now, transform);
            
            float current_x = transform.getOrigin().x();
            float current_y = transform.getOrigin().y();
            float current_theta = tf::getYaw(transform.getRotation());
            
            ROS_INFO("位置2 odom位姿: (%.3f, %.3f, %.1f°)",
                    current_x, current_y, current_theta * RAD_TO_DEG);
            
            // 【关键】直接使用当前位置的odom作为相对位移
            // 因为位置1时odom是(0,0,0)
            robot_pose.x = current_x;
            robot_pose.y = current_y;
            robot_pose.theta = current_theta;
            
            float distance = sqrt(current_x*current_x + current_y*current_y);
            ROS_INFO(" 使用odom位姿作为相对位移: 距离=%.3f米", distance);
            
            // 【可选】检查位移是否合理
            if (distance < 0.1f) {
                ROS_WARN("⚠️ 位移太小(%.3f米)，可能机器人没移动", distance);
            }
            if (distance > 5.0f) {
                ROS_WARN("⚠️ 位移较大(%.3f米)，确认是否正确", distance);
            }
        }
        else {
            ROS_WARN("无法获取odom位姿，使用预设距离");
            robot_pose.x = phase1_move_distance_;
            robot_pose.y = 0.0f;
            robot_pose.theta = 0.0f;
        }
        
        
    }
    
    submap.robot_pose = robot_pose;
    // 保存当前点云
    submap.cloud.reset(new PointCloudI(*pointcloud_));
    submap.cloud->header.frame_id = "sonar_frame";
    
    // 复制当前占据地图
    submap.map = occupancy_map_;
    submap.map.header.frame_id = "map";
    
    phase1_submaps_.push_back(submap);
    phase1_submap_count_ = phase1_submaps_.size();
    
    // 保存文件
    std::string filename = "phase1_submap_" + std::to_string(submap.index) + 
                          "_pos" + std::to_string(phase1_current_position_) +
                          "_dist" + std::to_string(robot_pose.x);
    saveStageMap(filename);
    
    ROS_INFO(" 保存子地图 %d (位置%d): 位姿(%.3f, %.3f, %.1f°), %zu点", 
             submap.index, phase1_current_position_,
             robot_pose.x, robot_pose.y, robot_pose.theta * RAD_TO_DEG,
             submap.cloud->size());
}

geometry_msgs::Pose2D PhaseFusionSonarMapper::getCurrentRobotPose() {
    geometry_msgs::Pose2D pose;
    // 简化实现：从TF获取或返回默认值
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    
    // 如果有里程计，可以在这里获取真实位姿
    // 暂时使用简化版
    
    return pose;
}

/***
void PhaseFusionSonarMapper::mergePhase1Submaps() {
    if (phase1_submaps_.size() < 2) {
        ROS_ERROR("需要至少2个子地图才能合并，当前有%lu个", phase1_submaps_.size());
        return;
    }

    ROS_INFO("========================================");
    ROS_INFO("基于几何对齐的迭代子图合成优化 - 修正版");
    ROS_INFO("========================================");

    // ==================== 1. 获取子地图数据 ====================
    const auto& submap1 = phase1_submaps_[0];
    const auto& submap2 = phase1_submaps_[1];
    const auto& pose1 = submap1.robot_pose;
    const auto& pose2 = submap2.robot_pose;

    ROS_INFO("子图信息:");
    ROS_INFO("  位置1: %lu点, 质心(%.3f,%.3f)", 
            submap1.cloud->size(), submap1.center_x, submap1.center_y);
    ROS_INFO("  位置2: %lu点, 质心(%.3f,%.3f)", 
            submap2.cloud->size(), submap2.center_x, submap2.center_y);
    ROS_INFO("  位置1 odom: (%.3f,%.3f,%.1f°)", 
            pose1.x, pose1.y, pose1.theta * RAD_TO_DEG);
    ROS_INFO("  位置2 odom: (%.3f,%.3f,%.1f°)", 
            pose2.x, pose2.y, pose2.theta * RAD_TO_DEG);

    // ==================== 2. 计算初始参数 ====================
    float odom_dx = pose2.x - pose1.x;
    float odom_dy = pose2.y - pose1.y;
    float odom_dtheta = pose2.theta - pose1.theta;

    // 质心信息
    float center_x_abs_sum = fabs(submap1.center_x) + fabs(submap2.center_x);
    float center_diff_y = submap2.center_y - submap1.center_y;

    // 使用你的成功公式
    const float SYSTEM_PARAM_K = 0.54f;
    const float estimated_inherent_offset = 0.58f;
    
    float current_dx = odom_dx + center_x_abs_sum + SYSTEM_PARAM_K;
    float current_dy = odom_dy + (center_diff_y * 0.2f) + estimated_inherent_offset;
    
    // 初始旋转决策：仅当odom报告明显旋转时才考虑
    const float INITIAL_ANGLE_THRESHOLD = 3.0f * DEG_TO_RAD;
    float current_theta = (fabs(odom_dtheta) > INITIAL_ANGLE_THRESHOLD) ? odom_dtheta : 0.0f;

    ROS_INFO("初始参数:");
    ROS_INFO("  odom位移: ΔX=%.3fm, ΔY=%.3fm, Δθ=%.1f°", 
            odom_dx, odom_dy, odom_dtheta * RAD_TO_DEG);
    ROS_INFO("  计算位移: dx=%.3fm, dy=%.3fm, dtheta=%.1f°", 
            current_dx, current_dy, current_theta * RAD_TO_DEG);

    // ==================== 3. 创建智能优化器（仅用于几何分析） ====================
    IntelligentSubmapAligner aligner(nh_, false);   
 
    
    // ==================== 4. 迭代优化主循环 ====================
    const int MAX_ITERATIONS = 8;
    const float TRANSLATION_CONVERGENCE_THRESHOLD = 0.02f;  // 2cm
    const float ANGLE_CONVERGENCE_THRESHOLD = 0.3f * DEG_TO_RAD;  // 0.3度

    pcl::PointCloud<pcl::PointXYZI>::Ptr best_merged_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr first_merged_cloud;  // 保留第一次合成结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud;
    float best_dx = current_dx;
    float best_dy = current_dy;
    float best_theta = current_theta;
    float best_alignment_score = -1.0f;

    bool walls_aligned = false;  // 新增：墙壁对齐标志

    // 记录迭代历史
    std::vector<std::string> iteration_logs;

    for (int iteration = 0; iteration < MAX_ITERATIONS && !walls_aligned; iteration++) {
        ROS_INFO("=== 迭代 %d/%d ===", iteration + 1, MAX_ITERATIONS);
        
        // 4.1 使用当前参数合成点云
        merged_cloud = mergePointCloudsWithTransform(
            submap1, submap2, current_dx, current_dy, current_theta);
        
        if (!merged_cloud || merged_cloud->empty()) {
            ROS_WARN("合成点云失败，跳过本次迭代");
            continue;
        }
        
        ROS_INFO("合成点云: %lu点, 参数: dx=%.3f, dy=%.3f, θ=%.1f°", 
                merged_cloud->size(), current_dx, current_dy, current_theta * RAD_TO_DEG);
        
        // 保存第一次合成结果（用于回退）
        if (iteration == 0 && merged_cloud && !merged_cloud->empty()) {
            first_merged_cloud = merged_cloud;
        }        
        
 
        // 4.2 使用智能优化器检测墙壁，但用自己的简单逻辑判断
        
        FourWallAlignment alignment = aligner.checkFourWallAlignment(*merged_cloud);
        
        // 增加墙壁有效性检查
        bool has_valid_x_pos_wall = alignment.x_pos_wall.wall_exists && 
                                alignment.x_pos_wall.average_distance > 0.5f && 
                                alignment.x_pos_wall.average_distance < 3.5f;
        
        bool has_valid_x_neg_wall = alignment.x_neg_wall.wall_exists && 
                                alignment.x_neg_wall.average_distance > 0.5f && 
                                alignment.x_neg_wall.average_distance < 3.5f;
        
        bool has_valid_y_pos_wall = alignment.y_pos_wall.wall_exists && 
                                alignment.y_pos_wall.average_distance > 0.5f && 
                                alignment.y_pos_wall.average_distance < 3.5f;
        
        bool has_valid_y_neg_wall = alignment.y_neg_wall.wall_exists && 
                                alignment.y_neg_wall.average_distance > 0.5f && 
                                alignment.y_neg_wall.average_distance < 3.5f;

        // 添加调试输出
        ROS_INFO("X轴墙壁检测状态: 东墙存在=%s, 西墙存在=%s",
                has_valid_x_pos_wall ? "是" : "否",
                has_valid_x_neg_wall ? "是" : "否");
        // 添加调试输出
 


        
        bool x_axis_walls_aligned = false;
        if (has_valid_x_pos_wall && has_valid_x_neg_wall) {

            float dist_diff = fabs(alignment.x_pos_wall.average_distance - 
                                alignment.x_neg_wall.average_distance);
            float raw_diff = alignment.x_pos_wall.fitted_line.angle - 
                                alignment.x_neg_wall.fitted_line.angle;

            // 第一步：归一化到 [0, 180]
            float angle_diff = fmod(fabs(raw_diff), float(M_PI));

            // 第二步：取锐角（平行可能是0°或180°）
            if (angle_diff > M_PI/2) {
                angle_diff = M_PI - angle_diff;  // 转换为0-90度
            }

            ROS_INFO("X轴方向墙壁检查: 距离差=%.2fm, 实际角度差=%.1f°", 
                    dist_diff, angle_diff * RAD_TO_DEG);
            
            // 【关键修改】使用3度容差，检查0°或180°平行
            const float ANGLE_TOLERANCE = 3.0f * DEG_TO_RAD;
            bool walls_parallel = (angle_diff < ANGLE_TOLERANCE || 
                                fabs(angle_diff - M_PI) < ANGLE_TOLERANCE);
            x_axis_walls_aligned = (dist_diff < 0.3f && walls_parallel);
            ROS_INFO("  X轴对齐: %s", x_axis_walls_aligned ? "是" : "否");
        }


        ROS_INFO("Y轴墙壁详细检查:");
        ROS_INFO("  北墙角度: %.1f°, 距离: %.3fm, 置信度: %.2f", 
        alignment.y_pos_wall.fitted_line.angle * RAD_TO_DEG,
        alignment.y_pos_wall.average_distance,
        alignment.y_pos_wall.alignment_confidence);
        ROS_INFO("  南墙角度: %.1f°, 距离: %.3fm, 置信度: %.2f", 
        alignment.y_neg_wall.fitted_line.angle * RAD_TO_DEG,
        alignment.y_neg_wall.average_distance,
        alignment.y_neg_wall.alignment_confidence);

        bool y_axis_walls_aligned = false;
        if (has_valid_y_pos_wall && has_valid_y_neg_wall) {
            float dist_diff = fabs(alignment.y_pos_wall.average_distance - 
                                alignment.y_neg_wall.average_distance);
            
            float raw_diff = alignment.y_pos_wall.fitted_line.angle - 
                    alignment.y_neg_wall.fitted_line.angle;

            // 第一步：归一化到 [0, 180]
            float angle_diff = fmod(fabs(raw_diff), float(M_PI));
            
            // 第二步：取锐角（平行可能是0°或180°）
            if (angle_diff > M_PI/2) {
                angle_diff = M_PI - angle_diff;  // 转换为0-90度
            }
            
            ROS_INFO("Y轴: 实际角度差=%.1f°, 距离差=%.3fm", 
                    angle_diff * RAD_TO_DEG, dist_diff);
            
            ROS_INFO(" Y轴方向墙壁检查角度差: %.4f rad (%.2f°)", angle_diff, angle_diff * RAD_TO_DEG);
            ROS_INFO(" Y轴方向墙壁检查距离差: %.3fm", dist_diff);
            
            const float ANGLE_TOLERANCE = 3.0f * DEG_TO_RAD;
            bool walls_parallel = (angle_diff < ANGLE_TOLERANCE || 
                                fabs(angle_diff - M_PI) < ANGLE_TOLERANCE);

            ROS_INFO("  墙壁平行: %s (阈值: %.2f°)", 
            walls_parallel ? "是" : "否", ANGLE_TOLERANCE * RAD_TO_DEG);

            y_axis_walls_aligned = (dist_diff < 0.6f && walls_parallel);
            ROS_INFO("  Y轴对齐: %s", y_axis_walls_aligned ? "是" : "否");

        }

        // 【修复】删除重复的alignment_score定义
        float alignment_score = 0.0f;
        if (x_axis_walls_aligned && y_axis_walls_aligned) {
            alignment_score = 0.9f;  // 两个方向都对齐
        } else if (x_axis_walls_aligned || y_axis_walls_aligned) {
            alignment_score = 0.6f;  // 一个方向对齐
        }
        
        // 【关键修改3】更新最佳结果
        if (alignment_score > best_alignment_score) {
            best_alignment_score = alignment_score;
            best_merged_cloud = merged_cloud;
            best_dx = current_dx;
            best_dy = current_dy;
            best_theta = current_theta;
            ROS_INFO("更新最佳结果: 分数 %.1f%%", alignment_score * 100);
        }
        
 

        const float WALL_CONFIDENCE_THRESHOLD = 0.45f;  // 根据日志0.48-0.50设定

        bool x_side_aligned = false;
        bool y_side_aligned = false;

        // X轴检查：只要有任意一侧墙壁吻合即可
        if (has_valid_x_pos_wall) {
            float conf = alignment.x_pos_wall.alignment_confidence;
            x_side_aligned = (conf > WALL_CONFIDENCE_THRESHOLD);
            ROS_INFO("  X+墙壁: 置信度=%.2f, %s", conf, x_side_aligned ? "已吻合" : "未吻合");
        }
        if (!x_side_aligned && has_valid_x_neg_wall) {
            float conf = alignment.x_neg_wall.alignment_confidence;
            x_side_aligned = (conf > WALL_CONFIDENCE_THRESHOLD);
            ROS_INFO("  X-墙壁: 置信度=%.2f, %s", conf, x_side_aligned ? "已吻合" : "未吻合");
        }

        // Y轴检查：只要有任意一侧墙壁吻合即可
        if (has_valid_y_pos_wall) {
            float conf = alignment.y_pos_wall.alignment_confidence;
            y_side_aligned = (conf > WALL_CONFIDENCE_THRESHOLD);
            ROS_INFO("  Y+墙壁: 置信度=%.2f, %s", conf, y_side_aligned ? "已吻合" : "未吻合");
        }
        if (!y_side_aligned && has_valid_y_neg_wall) {
            float conf = alignment.y_neg_wall.alignment_confidence;
            y_side_aligned = (conf > WALL_CONFIDENCE_THRESHOLD);
            ROS_INFO("  Y-墙壁: 置信度=%.2f, %s", conf, y_side_aligned ? "已吻合" : "未吻合");
        }

        // X轴或Y轴至少有一侧对齐即可
        if (x_side_aligned || y_side_aligned) {
            ROS_INFO("✅ %s方向墙壁已吻合 (阈值:%.2f)，停止迭代", 
                    x_side_aligned ? "X轴" : "Y轴", WALL_CONFIDENCE_THRESHOLD);
            walls_aligned = true;
            break;
        } else {
            ROS_INFO("  X轴和Y轴均未吻合 (阈值:%.2f)，继续优化", WALL_CONFIDENCE_THRESHOLD);
        }
                
        // 【关键修改5】简单调整逻辑 - 修复变量名
        float dx_adjust = 0.0f, dy_adjust = 0.0f, theta_adjust = 0.0f;
        
        // Y轴调整 - 处理拉开或重叠
        if (has_valid_y_pos_wall && has_valid_y_neg_wall && !y_axis_walls_aligned) {
            // 计算两侧墙壁的实际距离差
            float dist_pos = alignment.y_pos_wall.average_distance;  // Y+墙壁距离
            float dist_neg = alignment.y_neg_wall.average_distance;  // Y-墙壁距离
            float dist_diff = dist_pos - dist_neg;  // 正表示Y+更远，负表示Y-更远
            
            // 获取两侧墙壁的法向量方向
            float angle_pos = alignment.y_pos_wall.fitted_line.angle;
            float nx_pos = -sin(angle_pos);  // Y+墙壁的法向量
            float ny_pos = cos(angle_pos);
            
            float angle_neg = alignment.y_neg_wall.fitted_line.angle;
            float nx_neg = -sin(angle_neg);  // Y-墙壁的法向量
            float ny_neg = cos(angle_neg);
            
            // 获取置信度
            float conf_pos = alignment.y_pos_wall.alignment_confidence;
            float conf_neg = alignment.y_neg_wall.alignment_confidence;
            
            // 计算调整量
            float adjust_step = 0.03f;  // 3cm步长
            
            if (dist_diff > 0) {
                // 情况1: Y+比Y-远 → 拉开状态
                // Y+需要向内移动（负法向量方向），Y-需要向内移动（正法向量方向）
                dx_adjust = -nx_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) + 
                            nx_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                dy_adjust = -ny_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) + 
                            ny_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                ROS_INFO("Y轴拉开: Y+远%.2fm, 向内调整 (%.3f,%.3f)", 
                        dist_diff, dx_adjust, dy_adjust);
            } else {
                // 情况2: Y+比Y-近 → 重叠状态
                // Y+需要向外移动（正法向量方向），Y-需要向外移动（负法向量方向）
                dx_adjust = nx_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) - 
                            nx_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                dy_adjust = ny_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) - 
                            ny_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                ROS_INFO("Y轴重叠: Y+近%.2fm, 向外调整 (%.3f,%.3f)", 
                        -dist_diff, dx_adjust, dy_adjust);
            }
        }
        
        if (has_valid_x_pos_wall && has_valid_x_neg_wall && !x_axis_walls_aligned) {
            float dist_pos = alignment.x_pos_wall.average_distance;
            float dist_neg = alignment.x_neg_wall.average_distance;
            float dist_diff = dist_pos - dist_neg;
            
            float angle_pos = alignment.x_pos_wall.fitted_line.angle;
            float nx_pos = -sin(angle_pos);
            float ny_pos = cos(angle_pos);
            
            float angle_neg = alignment.x_neg_wall.fitted_line.angle;
            float nx_neg = -sin(angle_neg);
            float ny_neg = cos(angle_neg);
            
            float conf_pos = alignment.x_pos_wall.alignment_confidence;
            float conf_neg = alignment.x_neg_wall.alignment_confidence;
            
            float adjust_step = 0.03f;
            
            if (dist_diff > 0) {
                // X+比X-远 → 拉开
                dx_adjust = -nx_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) + 
                            nx_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                dy_adjust = -ny_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) + 
                            ny_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
            } else {
                // X+比X-近 → 重叠
                dx_adjust = nx_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) - 
                            nx_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
                dy_adjust = ny_pos * adjust_step * (conf_pos/(conf_pos+conf_neg)) - 
                            ny_neg * adjust_step * (conf_neg/(conf_pos+conf_neg));
            }
        }
        
        // 应用调整
        current_dx += dx_adjust;
        current_dy += dy_adjust;
        current_theta += theta_adjust;
        
        // 角度归一化
        current_theta = fmod(current_theta + M_PI, 2 * M_PI) - M_PI;
        
        // 记录日志
        char log_buffer[256];
        snprintf(log_buffer, sizeof(log_buffer), 
                "迭代%d: dx %.3f→%.3f, dy %.3f→%.3f, θ %.1f→%.1f°",
                iteration + 1, current_dx - dx_adjust, current_dx, 
                current_dy - dy_adjust, current_dy,
                (current_theta - theta_adjust) * RAD_TO_DEG, current_theta * RAD_TO_DEG);
        iteration_logs.push_back(log_buffer);
        
        // 短暂延迟
        ros::Duration(0.05).sleep();
    }
 

    pcl::PointCloud<pcl::PointXYZI>::Ptr final_merged_cloud;
    if (walls_aligned && best_merged_cloud && !best_merged_cloud->empty()) {
        // 1. 优先使用对齐成功的结果
        final_merged_cloud = best_merged_cloud;
        ROS_INFO("✅ 使用对齐成功的点云: %lu点", final_merged_cloud->size());
    } else if (first_merged_cloud && !first_merged_cloud->empty()) {
        // 2. 回退到第一次合成结果
        final_merged_cloud = first_merged_cloud;
        ROS_WARN("⚠️ 回退到第一次合成结果: %lu点", final_merged_cloud->size());
    } else {
        // 3. 创建空点云，避免崩溃
        final_merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        ROS_ERROR("❌ 没有可用的点云数据，创建空点云");
    }

    // 【关键：确保指针有效】
    if (!final_merged_cloud) {
        final_merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    // ==================== 6. 生成最终地图 ====================
    if (final_merged_cloud && !final_merged_cloud->empty()) {
        *pointcloud_ = *final_merged_cloud;
        
         // 生成占据地图（完全保留原有逻辑）
        std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
        
        for (const auto& point : merged_cloud->points) {
            int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
            int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
            
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                int index = map_y * map_width_ + map_x;
                int occupancy_value = 0;
                
                if (point.intensity >= 160.0f) occupancy_value = 100;
                else if (point.intensity >= 140.0f) occupancy_value = 80;
                else if (point.intensity >= 120.0f) occupancy_value = 60;
                else occupancy_value = 40;
                
                if (occupancy_value > occupancy_map_.data[index]) {
                    occupancy_map_.data[index] = occupancy_value;
                }
            }
        }
        
        // ==================== 6. 最终结果输出（增加优化详情） ====================
        ROS_INFO("========================================");
        ROS_INFO("地图合并完成");
        ROS_INFO("========================================");
        ROS_INFO("最终参数:");
        ROS_INFO("  dx = %.3f 米", current_dx);
        ROS_INFO("  dy = %.3f 米", current_dy);
        ROS_INFO("  dtheta = %.1f 度", current_theta * RAD_TO_DEG);
        ROS_INFO("合并结果:");
        ROS_INFO("  点云点数: %lu", merged_cloud->size());
        
        // 简单质量统计（完全保留原有逻辑）
        int high_intensity = 0;
        int medium_intensity = 0;
        int low_intensity = 0;
        
        for (const auto& p : merged_cloud->points) {
            if (p.intensity >= 160.0f) high_intensity++;
            else if (p.intensity >= 140.0f) medium_intensity++;
            else if (p.intensity >= 120.0f) low_intensity++;
        }
        
        int total_points = merged_cloud->size();
        ROS_INFO("  高强度点(≥160): %d (%.1f%%)", 
                high_intensity, high_intensity * 100.0 / total_points);
        ROS_INFO("  中强度点(140-159): %d (%.1f%%)", 
                medium_intensity, medium_intensity * 100.0 / total_points);
        ROS_INFO("  低强度点(120-139): %d (%.1f%%)", 
                low_intensity, low_intensity * 100.0 / total_points);
        
        ROS_INFO("========================================");
        
        // 发布结果（完全保留原有代码）
     
        ROS_WARN("=== 发布的地图时间戳: %.3f ===", occupancy_map_.header.stamp.toSec());
        ROS_WARN("=== 当前时间戳: %.3f ===", ros::Time::now().toSec());
        occupancy_map_.header.stamp = ros::Time::now();
        occupancy_map_.header.frame_id = "map";
        occupancy_pub_.publish(occupancy_map_);
        
        if (!merged_cloud->empty()) {
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*merged_cloud, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.header.frame_id = "map";
            pointcloud_pub_.publish(cloud_msg);
        }
        
        // 保存地图
   

        std::string map_name;
        // 使用当前时间戳（秒）
        int64_t timestamp = static_cast<int64_t>(ros::Time::now().toSec());
        map_name = "phase1_intelligently_merged_" + std::to_string(timestamp);
         
      
        saveStageMap(map_name);
        
        // 发布机器人位置标记
        publishRobotPositionMarkers();
        
        ROS_INFO("地图已保存并发布: %s", map_name.c_str());
    }
}
 

pcl::PointCloud<pcl::PointXYZI>::Ptr PhaseFusionSonarMapper::mergePointCloudsWithTransform(
    const Phase1SubMap& submap1,
    const Phase1SubMap& submap2,
    float dx, float dy, float theta) {
    
    PointCloudPtr merged(new PointCloudI());
    merged->header.frame_id = "map";
    merged->is_dense = false;
    
    // 添加子图1（位置1在地图原点）
    *merged += *submap1.cloud;
    
    // 变换并添加子图2
    if (fabs(theta) > 0.001f) {
        // 有旋转：应用旋转+平移
        float cos_theta = cosf(theta);
        float sin_theta = sinf(theta);
        
        for (const auto& point : submap2.cloud->points) {
            pcl::PointXYZI transformed_point;
            transformed_point.x = cos_theta * point.x - sin_theta * point.y + dx;
            transformed_point.y = sin_theta * point.x + cos_theta * point.y + dy;
            transformed_point.z = point.z;
            transformed_point.intensity = point.intensity;
            merged->points.push_back(transformed_point);
        }
    } else {
        // 无旋转：只应用平移
        for (const auto& point : submap2.cloud->points) {
            pcl::PointXYZI transformed_point;
            transformed_point.x = point.x + dx;
            transformed_point.y = point.y + dy;
            transformed_point.z = point.z;
            transformed_point.intensity = point.intensity;
            merged->points.push_back(transformed_point);
        }
    }
    
    merged->width = merged->points.size();
    merged->height = 1;
    
    return merged;
}

****/

void PhaseFusionSonarMapper::mergePhase1Submaps() {
    ROS_INFO("========================================");
    ROS_INFO("=== 声呐感知地图合并（保留网格累加器）===");
    ROS_INFO("========================================");
    
    // ========== 1. 检查子地图数量 ==========
    if (phase1_submaps_.size() < 2) {
        ROS_ERROR("需要至少2个子地图才能合并，当前有%lu个", phase1_submaps_.size());
        return;
    }
    
    // ========== 2. 初始化声呐感知对齐器 ==========
    ros::NodeHandle nh;
    SonarAwareAligner aligner(nh);
    
    // 配置对齐器参数
    aligner.setDebugMode(true);
    aligner.setMinCornerConfidence(0.5f);
    aligner.setMinWallConfidence(0.4f);
    aligner.setMaxTranslationError(0.15f);
    aligner.setMaxRotationError(3.0f * M_PI / 180.0f);
    
    // ========== 3. 获取两个子地图 ==========
    const auto& submap1 = phase1_submaps_[0];  // 第一次扫描
    const auto& submap2 = phase1_submaps_[1];  // 第二次扫描
    
    ROS_INFO("子地图1: %lu 点", submap1.cloud->size());
    ROS_INFO("子地图2: %lu 点", submap2.cloud->size());
    
    // 根据质心推断扫描位置（用于提示）
    std::string hint1 = "unknown";
    std::string hint2 = "unknown";
    
    if (submap1.center_x > 0.5f) hint1 = "right_side";
    else if (submap1.center_x < -0.5f) hint1 = "left_side";
    
    if (submap2.center_x > 0.5f) hint2 = "right_side";
    else if (submap2.center_x < -0.5f) hint2 = "left_side";
    
    // ========== 4. 执行声呐感知对齐 ==========
    ROS_INFO("=== 开始声呐感知对齐 ===");
    ROS_INFO("  提示信息: [%s] -> [%s]", hint1.c_str(), hint2.c_str());
    
    auto result = aligner.align(submap1.cloud, submap2.cloud, hint1, hint2);
    
    // ========== 5. 处理对齐结果 ==========
    float final_dx, final_dy, final_theta;
    
    if (!result.success) {
        ROS_ERROR("❌ 声呐感知对齐失败！");
        ROS_ERROR("  诊断信息: %s", result.diagnostic.c_str());
        
        // 回退到odom（保持原有行为）
        ROS_WARN("尝试使用odom作为紧急备选方案...");
        
        final_dx = submap2.robot_pose.x - submap1.robot_pose.x;
        final_dy = submap2.robot_pose.y - submap1.robot_pose.y;
        final_theta = submap2.robot_pose.theta - submap1.robot_pose.theta;
        
        ROS_WARN("使用odom备选: dx=%.3f, dy=%.3f, dtheta=%.1f°",
                 final_dx, final_dy, final_theta * RAD_TO_DEG);
    } else {
        ROS_INFO("✅ 声呐感知对齐成功！置信度: %.2f", result.confidence);
        
        final_dx = result.transform.dx;
        final_dy = result.transform.dy;
        final_theta = result.transform.dtheta;
        
        ROS_INFO("对齐结果: dx=%.3f, dy=%.3f, dtheta=%.1f°",
                 final_dx, final_dy, final_theta * RAD_TO_DEG);
    }
    
    // ========== 6. 合并点云（使用网格累加器） ==========
    ROS_INFO("=== 合并点云（使用网格累加器）===");
    
    // 创建网格累加器 (与原版完全一致)
    std::map<std::pair<int, int>, std::pair<float, int>> grid_accumulator;
    
    // 添加子地图1的点
    for (const auto& point : submap1.cloud->points) {
        if (point.intensity < 100.0f) continue;
        
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
        
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            auto key = std::make_pair(map_x, map_y);
            auto& entry = grid_accumulator[key];
            
            // 更新最大强度和计数
            if (point.intensity > entry.first) {
                entry.first = point.intensity;
            }
            entry.second++;
        }
    }
    
    // 变换并添加子地图2的点
    float cos_theta = cos(final_theta);
    float sin_theta = sin(final_theta);
    
    for (const auto& point : submap2.cloud->points) {
        if (point.intensity < 100.0f) continue;
        
        // 应用变换
        float tx = point.x * cos_theta - point.y * sin_theta + final_dx;
        float ty = point.x * sin_theta + point.y * cos_theta + final_dy;
        
        int map_x = static_cast<int>((tx - map_origin_x_) / map_resolution_ + 0.5f);
        int map_y = static_cast<int>((ty - map_origin_y_) / map_resolution_ + 0.5f);
        
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            auto key = std::make_pair(map_x, map_y);
            auto& entry = grid_accumulator[key];
            
            // 更新最大强度和计数
            if (point.intensity > entry.first) {
                entry.first = point.intensity;
            }
            entry.second++;
        }
    }
    
    ROS_INFO("网格累加器大小: %lu 个网格", grid_accumulator.size());
    
    // ========== 7. 生成合并点云（用于可视化） ==========
    PointCloudPtr merged_cloud(new PointCloudI());
    
    // 添加子地图1
    *merged_cloud += *submap1.cloud;
    
    // 变换并添加子地图2
    for (const auto& point : submap2.cloud->points) {
        pcl::PointXYZI transformed_point;
        transformed_point.x = point.x * cos_theta - point.y * sin_theta + final_dx;
        transformed_point.y = point.x * sin_theta + point.y * cos_theta + final_dy;
        transformed_point.z = point.z;
        transformed_point.intensity = point.intensity;
        merged_cloud->points.push_back(transformed_point);
    }
    
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    *pointcloud_ = *merged_cloud;
    
    // ========== 8. 从网格累加器生成占据地图 ==========
    ROS_INFO("=== 从网格累加器生成占据地图 ===");
    
    // 重置地图
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    int points_added = 0;
    int high_quality = 0, medium_quality = 0, low_quality = 0;
    
    for (const auto& entry : grid_accumulator) {
        int map_x = entry.first.first;
        int map_y = entry.first.second;
        float max_intensity = entry.second.first;
        int point_count = entry.second.second;
        
        int index = map_y * map_width_ + map_x;
        
        // 根据强度设置占据值（与原版完全一致）
        int occupancy_value = 0;
        if (max_intensity >= 160.0f) {
            occupancy_value = 100;
            high_quality++;
        } else if (max_intensity >= 140.0f) {
            occupancy_value = 80;
            medium_quality++;
        } else if (max_intensity >= 120.0f) {
            occupancy_value = 60;
            low_quality++;
        } else {
            occupancy_value = 40;
            low_quality++;
        }
        
        // 多个点增加置信度（与原版完全一致）
        if (point_count > 1) {
            occupancy_value = std::min(100, occupancy_value + (int)(point_count * 5));
        }
        
        occupancy_map_.data[index] = occupancy_value;
        points_added++;
    }
    
    // ========== 9. 墙壁增强处理 ==========
    ROS_INFO("=== 墙壁增强处理 ===");
    
    std::vector<int8_t> temp_map = occupancy_map_.data;
    int enhanced_cells = 0;
    
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            if (occupancy_map_.data[idx] >= 80) {
                // 3x3膨胀
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nidx = (y + dy) * map_width_ + (x + dx);
                        if (temp_map[nidx] < 60) {
                            temp_map[nidx] = 60;
                            enhanced_cells++;
                        }
                    }
                }
            }
        }
    }
    
    occupancy_map_.data = temp_map;
    ROS_INFO("墙壁增强: 填充 %d 个单元格", enhanced_cells);
    
    // ========== 10. 输出统计信息（与原版格式一致） ==========
    int occupied = 0, free = 0, unknown = 0;
    for (int val : occupancy_map_.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("合并结果:");
    ROS_INFO("  - 点云: %lu点", merged_cloud->size());
    ROS_INFO("  - 网格累加器: %lu个网格", grid_accumulator.size());
    ROS_INFO("  - 地图单元: 占据=%d, 空闲=%d, 未知=%d", occupied, free, unknown);
    ROS_INFO("  - 高质量点(≥160): %d", high_quality);
    ROS_INFO("  - 中质量点(140-159): %d", medium_quality);
    ROS_INFO("  - 低质量点(100-139): %d", low_quality);
    ROS_INFO("  - 位置1在地图原点: (0.0, 0.0)");
    ROS_INFO("  - 位置2在地图: (%.3f, %.3f)", final_dx, final_dy);
    ROS_INFO("  - 旋转角度: %.1f°", final_theta * RAD_TO_DEG);
    
    if (result.success) {
        ROS_INFO("  - 对齐方法: %s", result.method.c_str());
        ROS_INFO("  - 对齐置信度: %.2f", result.confidence);
        if (!result.constraints_used.empty()) {
            ROS_INFO("  - 使用约束:");
            for (const auto& c : result.constraints_used) {
                ROS_INFO("      * %s", c.c_str());
            }
        }
    } else {
        ROS_INFO("  - 对齐方法: odom备选（低置信度）");
    }
    
    ROS_INFO("========================================");
    
    // ========== 11. 保存地图 ==========
    std::string map_name;
    if (result.success && result.confidence > 0.8) {
        map_name = "pool_map_high_confidence_" + 
                   std::to_string(static_cast<int64_t>(ros::Time::now().toSec()));
    } else if (result.success && result.confidence > 0.5) {
        map_name = "pool_map_medium_confidence_" + 
                   std::to_string(static_cast<int64_t>(ros::Time::now().toSec()));
    } else {
        map_name = "phase1_merged_" + 
                   std::to_string(static_cast<int64_t>(ros::Time::now().toSec()));
    }
    
    saveStageMap(map_name);
    
    // ========== 12. 发布结果 ==========
    occupancy_map_.header.stamp = ros::Time::now();
    occupancy_map_.header.frame_id = "map";
    occupancy_pub_.publish(occupancy_map_);
    standard_map_pub_.publish(occupancy_map_);
    
    if (!merged_cloud->empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*merged_cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";
        pointcloud_pub_.publish(cloud_msg);
    }
    
    // 发布机器人位置标记
    publishRobotPositionMarkers();
    
    // ========== 13. 发布完成信号 ==========
    std_msgs::Bool done_msg;
    done_msg.data = true;
    static_map_done_pub_.publish(done_msg);
    
    ROS_INFO("地图合并完成！保存为: %s", map_name.c_str());
}

/**** 
void PhaseFusionSonarMapper::mergePhase1Submaps() {
    if (phase1_submaps_.size() < 2) {
        ROS_ERROR("需要至少2个子地图才能合并，当前有%lu个", phase1_submaps_.size());
        return;
    }

    // ========== 新增：合并前的详细对比 ==========
    ROS_WARN(">>>> 开始地图合并前对比分析：");
    
    for (size_t i = 0; i < phase1_submaps_.size(); i++) {
        int submap_num = static_cast<int>(i + 1);
        const auto& submap = phase1_submaps_[i];
        const auto& cloud = submap.cloud;
        
        ROS_WARN("子地图%d (位置%d):", submap_num, submap_num);
        ROS_WARN("  机器人位姿: (%.3f, %.3f, %.1f°)", 
                submap.robot_pose.x, submap.robot_pose.y, 
                submap.robot_pose.theta * RAD_TO_DEG);
        ROS_WARN("  点云大小: %lu", cloud->size());
        
        if (!cloud->empty() && cloud->size() >= 3) {
            // 检查前3个点的原始坐标（在子地图坐标系中）
            ROS_WARN("  前3个原始点坐标:");
            for (int j = 0; j < 3; j++) {
                const auto& p = cloud->points[j];
                float angle = atan2(p.y, p.x) * RAD_TO_DEG;
                float dist = sqrt(p.x*p.x + p.y*p.y);
                ROS_WARN("    点%d: (%.4f, %.4f), 距离=%.3f, 角度=%.1f°", 
                        j, p.x, p.y, dist, angle);
            }
        }
    }
    // ========== 新增结束 ==========
 
    ROS_INFO("========================================");
    ROS_INFO("开始合并%lu个子地图 - 修正版", phase1_submaps_.size());

    float origin1_x = phase1_submaps_[0].map.info.origin.position.x;
    float origin1_y = phase1_submaps_[0].map.info.origin.position.y;
    float origin2_x = phase1_submaps_[1].map.info.origin.position.x;
    float origin2_y = phase1_submaps_[1].map.info.origin.position.y;
    
    if (fabs(origin1_x - origin2_x) > 0.01f || fabs(origin1_y - origin2_y) > 0.01f) {
        ROS_ERROR("⚠️ 两个子地图原点不一致！");
        ROS_ERROR("  位置1原点: (%.3f, %.3f)", origin1_x, origin1_y);
        ROS_ERROR("  位置2原点: (%.3f, %.3f)", origin2_x, origin2_y);
        ROS_ERROR("  差异: ΔX=%.3f, ΔY=%.3f", origin2_x - origin1_x, origin2_y - origin1_y);
    } else {
        ROS_INFO("✓ 两个子地图原点一致: (%.3f, %.3f)", origin1_x, origin1_y);
    }
    
    // 1. 获取位姿信息
    const auto& pose1 = phase1_submaps_[0].robot_pose;
    const auto& pose2 = phase1_submaps_[1].robot_pose;
    
    ROS_INFO("=== 原始Odom位姿 ===");
    ROS_INFO("位置1 odom位姿: (%.3f, %.3f, %.1f°)", 
             pose1.x, pose1.y, pose1.theta * RAD_TO_DEG);
    ROS_INFO("位置2 odom位姿: (%.3f, %.3f, %.1f°)", 
             pose2.x, pose2.y, pose2.theta * RAD_TO_DEG);
    
    // 2. 计算相对位移 - 关键修改：去除旋转影响
    float actual_dx = pose2.x - pose1.x;
    float actual_dy = pose2.y - pose1.y;
    float actual_distance = sqrt(actual_dx*actual_dx + actual_dy*actual_dy);
    float actual_dtheta = pose2.theta - pose1.theta;
    
    ROS_INFO("Odom相对位移: ΔX=%.3f米, ΔY=%.3f米, 距离=%.3f米, Δθ=%.2f°", 
             actual_dx, actual_dy, actual_distance, actual_dtheta * RAD_TO_DEG);
    
    // 3. 关键修改：解决旋转问题
    // 从您的日志可以看出：odom报告了42.6°旋转，但点云本身已经包含了正确方向
    // 所以合并时应该只平移，不旋转
    ROS_INFO("=== 修正：只使用平移，忽略旋转 ===");

     // 计算质心差异
    float center_diff_x = phase1_submaps_[1].center_x - phase1_submaps_[0].center_x;
    float center_diff_y = phase1_submaps_[1].center_y - phase1_submaps_[0].center_y;

    // ==================== 基于水池几何的位移计算 ==========0=====begin=====
 
    const float POOL_WIDTH = 4.1f;    // 水池宽度（Y轴）
    const float POOL_LENGTH = 6.12f;  // 水池长度（X轴）
    float final_dx_debug = 0.0f;
    float final_dy_debug = 0.0f;
    float final_theta_debug = 0.0f;

    // 分析两个子地图的点云分布
    float pos1_min_x = FLT_MAX, pos1_max_x = -FLT_MAX;
    float pos1_min_y = FLT_MAX, pos1_max_y = -FLT_MAX;
    float pos2_min_x = FLT_MAX, pos2_max_x = -FLT_MAX;
    float pos2_min_y = FLT_MAX, pos2_max_y = -FLT_MAX;

    // 分析位置1点云范围（只分析高质量墙壁点）
    for (const auto& p : phase1_submaps_[0].cloud->points) {
        if (p.intensity >= 160.0f) {  // 只考虑高强度点（墙壁）
            if (p.x < pos1_min_x) pos1_min_x = p.x;
            if (p.x > pos1_max_x) pos1_max_x = p.x;
            if (p.y < pos1_min_y) pos1_min_y = p.y;
            if (p.y > pos1_max_y) pos1_max_y = p.y;
        }
    }

    // 分析位置2点云范围
    for (const auto& p : phase1_submaps_[1].cloud->points) {
        if (p.intensity >= 160.0f) {  // 只考虑高强度点（墙壁）
            if (p.x < pos2_min_x) pos2_min_x = p.x;
            if (p.x > pos2_max_x) pos2_max_x = p.x;
            if (p.y < pos2_min_y) pos2_min_y = p.y;
            if (p.y > pos2_max_y) pos2_max_y = p.y;
        }
    }

    // 计算两个子地图覆盖的总X范围
    float combined_min_x = std::min(pos1_min_x, pos2_min_x);
    float combined_max_x = std::max(pos1_max_x, pos2_max_x);
    float total_x_span = combined_max_x - combined_min_x;

    // 计算当前两个子地图中心的实际距离
    float current_center_distance = phase1_submaps_[0].center_x - phase1_submaps_[1].center_x;

    ROS_WARN("=== 水池几何分析（基于墙壁点） ===");
    ROS_WARN("位置1墙壁范围: X[%.2f, %.2f], Y[%.2f, %.2f]", 
            pos1_min_x, pos1_max_x, pos1_min_y, pos1_max_y);
    ROS_WARN("位置2墙壁范围: X[%.2f, %.2f], Y[%.2f, %.2f]", 
            pos2_min_x, pos2_max_x, pos2_min_y, pos2_max_y);
    ROS_WARN("两个子地图总X范围: %.2f米 (水池宽度应为%.2f米)", total_x_span, POOL_WIDTH);
    ROS_WARN("当前质心距离: %.3f米", current_center_distance);
    ROS_WARN("odom报告位移: ΔX=%.3f米", actual_dx);
 

    // ==================== 基于水池几何的位移计算 =========2======end=====
    
    // 最终位移（位置1在原点）
    // float final_dx = actual_dx;
    // float final_dy = actual_dy;
    // float final_dx = phase1_move_distance_;  // 从参数读取
    // float final_dy = 0.0f;  // 假设是直线移动
    // float final_dx = actual_dx;                
    // float final_dy = actual_dy;      
    ROS_INFO("=== 质心补偿分析 ===");
    ROS_INFO("位置1质心: (%.3f, %.3f)", 
            phase1_submaps_[0].center_x, phase1_submaps_[0].center_y);
    ROS_INFO("位置2质心: (%.3f, %.3f)", 
            phase1_submaps_[1].center_x, phase1_submaps_[1].center_y);

    
    // ====================  质心补偿逻辑 ====================
    bool use_centroid_compensation = true;
    
    // 1. X方向符号检查（放宽条件）
    if (center_diff_x * actual_dx < 0 && fabs(actual_dx) > 0.5f) {
        float angle_diff = fabs(actual_dtheta) * RAD_TO_DEG;
        
        if (angle_diff < 45.0f) {
            ROS_WARN("⚠️ X方向质心符号相反: 质心ΔX=%.3f, OdomΔX=%.3f", 
                     center_diff_x, actual_dx);
            ROS_WARN("  机器人旋转=%.1f°，水池左右两侧可能正常", angle_diff);
        }
        // 注意：不再禁用补偿！只是警告
    }

    // 2. 质心距离检查（提高阈值）
    float center_distance = sqrt(center_diff_x*center_diff_x + center_diff_y*center_diff_y);
    float max_allowed_centroid_distance = 5.0f;  // 从2.0提高到5.0米
    
    if (center_distance > max_allowed_centroid_distance) {
        ROS_WARN("⚠️ 质心差异较大: %.3f米（阈值%.1f米）", 
                 center_distance, max_allowed_centroid_distance);
        ROS_WARN("  继续尝试补偿（水池可能较大）");
        // 不禁用，只是警告
    }

    // 3. 质心/位移比例检查（放宽比例）
    float odom_distance = sqrt(actual_dx*actual_dx + actual_dy*actual_dy);
    if (center_distance > odom_distance * 2.5f && odom_distance > 1.0f) {
        ROS_WARN("⚠️ 质心差异(%.3f) > 位移(%.3f) * 2.5", 
                 center_distance, odom_distance);
        // 不禁用，只是警告
    }
    
    // ==================== 判断是否为水池左右侧建图 ====================
    float pos1_x = phase1_submaps_[0].center_x;
    float pos2_x = phase1_submaps_[1].center_x;
    bool is_left_right_mapping = false;
    
    if (pos1_x > 0.3f && pos2_x < -0.3f) {  // 简单判断：位置1X为正，位置2X为负
        is_left_right_mapping = true;
        ROS_INFO("✅ 检测到水池左右两侧建图模式");
    }
    
    // ==================== 计算最终位移 ====================
    float final_dx = 0.0f;
    float final_dy = 0.0f;
    float final_theta = 0.0f;
    
   
    // 
    const float SYSTEM_PARAM_K = 0.54f;  // 系统参数，通过标定获得
 
    // 计算质心绝对值之和
    float center_x_abs_sum = fabs(phase1_submaps_[0].center_x) + 
                        fabs(phase1_submaps_[1].center_x);

    // 计算最终X位移
    final_dx = actual_dx + center_x_abs_sum + SYSTEM_PARAM_K;
    

    // Y方向简化处理（后续可优化）
    // ==================== Y方向计算  ====================
    // 系统参数（需要标定）
    float estimated_inherent_offset = 0.58f;
    float center_y_diff = phase1_submaps_[1].center_y - phase1_submaps_[0].center_y;
    final_dy = actual_dy + (center_y_diff * 0.2f)+ estimated_inherent_offset;

 
    // 在计算后，记录结果
    ROS_WARN("[Y轴诊断] 计算: center_y_diff=%.3f, 补偿值=%.3f, 最终 final_dy=%.3f", 
            center_y_diff, 
            center_y_diff * 0.2f,
            final_dy);
 
            
    final_theta = 0.0f;

    // ==================== 旋转处理 ====================
    const float ROTATION_THRESHOLD = 5.0f * DEG_TO_RAD;  // 5度阈值
    const float ROTATION_COMPENSATION_RATIO = 0.8f;      // 80%

    if (fabs(actual_dtheta) > ROTATION_THRESHOLD) {
         
        final_theta = actual_dtheta;
 
        ROS_WARN("Odom旋转: %.1f°", actual_dtheta * RAD_TO_DEG);
        // ROS_WARN("补偿比例: %.0f%%", ROTATION_COMPENSATION_RATIO * 100.0f);
        ROS_WARN("设置旋转: %.1f°", final_theta * RAD_TO_DEG);
    } else {
        final_theta = 0.0f;
        ROS_DEBUG("旋转%.1f°小于阈值，忽略", actual_dtheta * RAD_TO_DEG);
    }
    // final_theta = 0.0f;
    // ROS_DEBUG("强制设置不旋转%.1f°小于阈值，忽略", actual_dtheta * RAD_TO_DEG);
    // ==================== 最终结果输出 ====================
    ROS_WARN("=== 位移计算结果 ===");
    ROS_WARN("Odom位移:      (%.3f, %.3f) 距离=%.3f米", 
            actual_dx, actual_dy, odom_distance);
 
    ROS_WARN("最终位移:      (%.3f, %.3f, %.1f°)",
            final_dx, final_dy, final_theta * RAD_TO_DEG);
 
 
    
    // 4. 创建合并点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new PointCloudI());
    merged_cloud->header.frame_id = "map";
    merged_cloud->is_dense = false;
    
    std::map<std::pair<int, int>, std::pair<float, float>> grid_accumulator;
    
    // 5. 处理每个子地图
    for (size_t i = 0; i < phase1_submaps_.size(); i++) {
        const auto& submap = phase1_submaps_[i];
        
        float pose_x, pose_y, pose_theta;
        
        if (i == 0) {
            // 位置1：固定在地图原点
            pose_x = 0.0f;
            pose_y = 0.0f;
            pose_theta = 0.0f;
            ROS_INFO("位置1地图坐标: (0.0, 0.0, 0.0°)");
        } else {
            // 位置2：应用计算出的位移和旋转
            pose_x = final_dx;
            pose_y = final_dy;
            pose_theta = final_theta;  // 注意：这里可能不是0！
            
            // 修正日志输出：显示实际的旋转角度
            ROS_INFO("位置2地图坐标: (%.3f, %.3f, %.1f°)", 
                    pose_x, pose_y, pose_theta * RAD_TO_DEG);
            
            // 添加调试信息
            ROS_WARN("位置2变换详情:");
            ROS_WARN("  平移: (%.3f, %.3f)米", pose_x, pose_y);
            ROS_WARN("  旋转: %.2f° (odom报告%.2f°，应用反向%.2f°)", 
                    pose_theta * RAD_TO_DEG, 
                    actual_dtheta * RAD_TO_DEG,
                    final_theta * RAD_TO_DEG);

        }
        
        int points_added = 0;
        
        // 6. 转换每个点 - 关键修改：简化转换，只平移
        for (const auto& point : submap.cloud->points) {
            pcl::PointXYZI transformed_point;
    

            // 在点云转换循环中：
            if (i == 0) { 
                // 位置1：在地图原点
                transformed_point.x = point.x;
                transformed_point.y = point.y;
                
                if (points_added < 3) {
                    ROS_WARN("位置1点%d: 原始坐标(%.3f,%.3f)", 
                            points_added+1, point.x, point.y);
                }
            } else { 

                if (fabs(pose_theta) > 0.001f) {
                    // 有旋转：应用旋转+平移
                    float cos_theta = cosf(pose_theta);
                    float sin_theta = sinf(pose_theta);
                    
                    transformed_point.x = cos_theta * point.x - sin_theta * point.y + pose_x;
                    transformed_point.y = sin_theta * point.x + cos_theta * point.y + pose_y;
                    
                    if (points_added < 3) {
                        ROS_WARN("位置2点%d【有旋转%.1f°】:", 
                                points_added+1, pose_theta * RAD_TO_DEG);
                        ROS_WARN("  原始: (%.3f,%.3f)", point.x, point.y);
                        ROS_WARN("  旋转矩阵: [%.3f, %.3f; %.3f, %.3f]", 
                                cos_theta, -sin_theta, sin_theta, cos_theta);
                        ROS_WARN("  结果: (%.3f,%.3f)", 
                                transformed_point.x, transformed_point.y);
                    }
                } else {
                    // 无旋转：只应用平移
                    transformed_point.x = point.x + pose_x;
                    transformed_point.y = point.y + pose_y;
                    
                    if (points_added < 3) {
                        ROS_WARN("位置2点%d【无旋转：只应用平移】:", points_added+1);
                        ROS_WARN("  原始: (%.3f,%.3f)", point.x, point.y);
                        ROS_WARN("  位移: (%.3f,%.3f)", pose_x, pose_y);
                        ROS_WARN("  结果: (%.3f,%.3f)", 
                                transformed_point.x, transformed_point.y);
                    }
                }
               
            }

            transformed_point.z = 0.0f;
            transformed_point.intensity = point.intensity;
            
            // 调试输出前几个点
            if (points_added < 3 && i == 1) {  // 位置2的前3个点
                // 计算原始点的极坐标
                float raw_dist = sqrt(point.x*point.x + point.y*point.y);
                float raw_angle = atan2(point.y, point.x) * RAD_TO_DEG;
                
                // 计算转换后相对于位置1的角度
                float map_angle = atan2(transformed_point.y, transformed_point.x) * RAD_TO_DEG;
                
                float map_dist = sqrt(transformed_point.x*transformed_point.x + 
                                    transformed_point.y*transformed_point.y);
                
                // 计算角度变化
                float angle_change = map_angle - raw_angle;
                
                ROS_WARN("位置2点%d:", points_added+1);
                ROS_WARN("  原始: 距离=%.3fm, 角度=%.1f°", raw_dist, raw_angle);
                ROS_WARN("  转换后: 距离=%.3fm, 角度=%.1f°", map_dist, map_angle);
                ROS_WARN("  角度变化: %.1f°", angle_change);
                ROS_WARN("  坐标: (%.3f, %.3f) → (%.3f, %.3f)", 
                        point.x, point.y, transformed_point.x, transformed_point.y);
            }
            
            // 转换到地图网格
            int map_x = static_cast<int>((transformed_point.x - map_origin_x_) / map_resolution_ + 0.5f);
            int map_y = static_cast<int>((transformed_point.y - map_origin_y_) / map_resolution_ + 0.5f);
            
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                std::pair<int, int> grid_key = std::make_pair(map_x, map_y);
                
                // 强强融合：保留最大强度值
                if (grid_accumulator.find(grid_key) == grid_accumulator.end()) {
                    grid_accumulator[grid_key] = std::make_pair(transformed_point.intensity, 1.0f);
                    merged_cloud->points.push_back(transformed_point);
                } else {
                    auto& existing = grid_accumulator[grid_key];
                    if (transformed_point.intensity > existing.first) {
                        existing.first = transformed_point.intensity;
                        // 更新点云中对应点的强度
                        for (auto& mp : merged_cloud->points) {
                            int mx = static_cast<int>((mp.x - map_origin_x_) / map_resolution_ + 0.5f);
                            int my = static_cast<int>((mp.y - map_origin_y_) / map_resolution_ + 0.5f);
                            if (mx == map_x && my == map_y) {
                                mp.intensity = transformed_point.intensity;
                                break;
                            }
                        }
                    }
                    existing.second += 1.0f;
                }
                
                points_added++;
            }
        }
        
        ROS_INFO("  子地图%d转换完成: %d点", static_cast<int>(i+1), points_added);
    }
    
    // 7. 生成合并地图
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    *pointcloud_ = *merged_cloud;
    
    // 重置地图
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    // 从网格累加器生成占据地图
    for (const auto& entry : grid_accumulator) {
        int map_x = entry.first.first;
        int map_y = entry.first.second;
        float max_intensity = entry.second.first;
        float point_count = entry.second.second;
        
        int index = map_y * map_width_ + map_x;
        
        // 根据强度设置占据值
        int occupancy_value = 0;
        if (max_intensity >= 160.0f) occupancy_value = 100;
        else if (max_intensity >= 140.0f) occupancy_value = 80;
        else if (max_intensity >= 120.0f) occupancy_value = 60;
        else occupancy_value = 40;
        
        // 多个点增加置信度
        if (point_count > 1) {
            occupancy_value = std::min(100, occupancy_value + (int)(point_count * 5));
        }
        
        occupancy_map_.data[index] = occupancy_value;
    }
    
    // 8. 输出结果
    int occupied = 0, free = 0, unknown = 0;
    for (int val : occupancy_map_.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("合并结果:");
    ROS_INFO("  - 点云: %lu点", merged_cloud->size());
    ROS_INFO("  - 地图: 占据=%d, 空闲=%d, 未知=%d", occupied, free, unknown);
    ROS_INFO("  - 位置1在地图: (0.0, 0.0)");
    ROS_INFO("  - 位置2在地图: (%.3f, %.3f)", final_dx, final_dy);
    ROS_INFO("  - 使用Odom位移: ΔX=%.3f米, ΔY=%.3f米, Δθ=%.1f°", 
             final_dx, final_dy, final_theta * RAD_TO_DEG);
    ROS_INFO("========================================");
    
    // 9. 发布结果
    occupancy_map_.header.stamp = ros::Time::now();
    occupancy_map_.header.frame_id = "map";
    occupancy_pub_.publish(occupancy_map_);
    
    if (!merged_cloud->empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*merged_cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";
        pointcloud_pub_.publish(cloud_msg);
    }
    
    saveStageMap("phase1_merged");
    publishRobotPositionMarkers();
    
    ROS_INFO(" 地图合并完成（使用Odom位姿）！");
}
 ****/

//发布机器人位置的TF标记
void PhaseFusionSonarMapper::publishRobotPositionMarkers() {
     if (phase1_submaps_.empty()) {
        ROS_WARN("没有子地图数据，无法发布位置标记");
        return;
    }
    visualization_msgs::MarkerArray marker_array;
    // 提前声明变量
    float calib_x = 0, calib_y = 0;
    
    // 1. 显示位置标记
    for (size_t i = 0; i < phase1_submaps_.size(); i++) {
        const auto& submap = phase1_submaps_[i];
        const auto& pose = submap.robot_pose;
        float map_x, map_y, map_theta;
        if (i == 0) {
            // 位置1：地图原点
            map_x = 0.0f;
            map_y = 0.0f;
            map_theta = 0.0f;
        } else {
            // 位置2：计算校准后的位置
            const auto& pose1 = phase1_submaps_[0].robot_pose;
            float odom_dx = pose.x - pose1.x;
            float odom_dy = pose.y - pose1.y;
            
            // 使用校准参数
            map_x = odom_dx;
            map_y = odom_dy;  // √5≈2.236
            map_theta = pose.theta - pose1.theta;
            
            // 保存用于连接线
            calib_x = map_x;
            calib_y = map_y;
        }
        
        // 机器人位置标记
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "map";
        robot_marker.header.stamp = ros::Time::now();
        robot_marker.ns = "robot_positions";
        robot_marker.id = static_cast<int>(i);
        robot_marker.type = visualization_msgs::Marker::SPHERE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.pose.position.x = map_x;
        robot_marker.pose.position.y = map_y;
        robot_marker.pose.position.z = 0.0;
        robot_marker.pose.orientation = tf::createQuaternionMsgFromYaw(map_theta);
        robot_marker.scale.x = 0.3;
        robot_marker.scale.y = 0.3;
        robot_marker.scale.z = 0.3;
        
        // 设置颜色
        if (i == 0) {
            robot_marker.color.r = 0.0;  // 绿色：位置1
            robot_marker.color.g = 1.0;
            robot_marker.color.b = 0.0;
        } else {
            robot_marker.color.r = 1.0;  // 红色：位置2
            robot_marker.color.g = 0.0;
            robot_marker.color.b = 0.0;
        }
        robot_marker.color.a = 0.8;
        robot_marker.lifetime = ros::Duration(0);
        
        marker_array.markers.push_back(robot_marker);
        
        // 文字标记
        visualization_msgs::Marker text_marker = robot_marker;
        text_marker.id = static_cast<int>(i + 100);
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.25;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "位置" << (i+1) 
           << "\n地图:(" << map_x << "," << map_y << ")"
           << "\nodom:(" << pose.x << "," << pose.y << ")";
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
    }
    
    // 2. 连接线（位置1到位置2）
    if (phase1_submaps_.size() >= 2) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "robot_positions";
        line_marker.id = 200;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.05;
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        geometry_msgs::Point p;
        p.x = 0; p.y = 0; p.z = 0.1;
        line_marker.points.push_back(p);
        p.x = calib_x; p.y = calib_y; p.z = 0.1;
        line_marker.points.push_back(p);
        
        marker_array.markers.push_back(line_marker);
    }
    
   
    
    // ========== 3. 发布 ==========
    static ros::Publisher debug_pub = nh_.advertise<visualization_msgs::MarkerArray>(
        "/phase1/debug_markers", 1, true);
    debug_pub.publish(marker_array);
    
    ROS_INFO("已发布调试标记到 /phase1/debug_markers");
}

void PhaseFusionSonarMapper::publishPhase1Status(const std::string& status) {
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
void PhaseFusionSonarMapper::startPhase1Mapping(int max_submaps) {
    if (phase1_enabled_) {
        ROS_WARN("第一阶段已在运行中 (当前步骤: %d)", phase1_step_);
        return;
    }
    
    // 新增确保扫描停止后再开始
    if (is_scanning_.load()) {
        ROS_INFO("停止现有扫描...");
        stopScanning();
        ros::Duration(0.5).sleep();  // 短暂等待确保停止
    }
    
    // 重置所有状态
    ROS_INFO("========================================");
    ROS_INFO("=== 手动启动第一阶段建图 ===");
    
    phase1_enabled_ = true;
    phase1_step_ = 1;  // 重置为初始状态
    phase1_submap_count_ = 0;
    phase1_submaps_.clear();
    
    // 清空旧数据
    ROS_INFO("清空旧扫描数据...");
    clearScanData();
    
    // 确保扫描启动
    if (!is_scanning_.load()) {
        ROS_INFO("启动扫描线程...");
        startScanning();
    } else {
        ROS_INFO("扫描已在运行中");
    }
    
    // 记录开始时间
    phase1_step_start_time_ = ros::Time::now();
    
    publishPhase1Status("准备开始");
    
    // 短暂等待确保状态更新
    ros::Duration(1.0).sleep();
    
    ROS_INFO("第一阶段建图已启动，进入步骤 %d", phase1_step_);
}


void PhaseFusionSonarMapper::stopPhase1Mapping() {
    if (!phase1_enabled_) {
        ROS_WARN("第一阶段未启用");
        return;
    }
    
    ROS_INFO("=== 停止第一阶段建图 ===");
    ROS_INFO("当前步骤: %d", phase1_step_);
    
    // 重置状态
    phase1_enabled_ = false;
    phase1_step_ = 0;
    phase1_submap_count_ = 0;
    
    // 停止扫描
    if (is_scanning_.load()) {
        stopScanning();
    }
    
    publishPhase1Status("已停止");
    
    ROS_INFO("第一阶段建图已停止");
}

std::string PhaseFusionSonarMapper::getPhase1StatusString() const {
    std::stringstream ss;
    
    if (phase1_enabled_) {
        ss << "Phase1: ";
        switch (phase1_step_) {
            case 0: ss << "准备中"; break;
            case 1: ss << "第一个位置建图中"; break;
            case 2: ss << "等待人工移动"; break;
            case 3: ss << "第二个位置建图中"; break;
            case 4: ss << "完成"; break;
            default: ss << "未知步骤 " << phase1_step_; break;
        }
        
        if (phase1_step_ > 0) {
            ros::Duration elapsed = ros::Time::now() - phase1_step_start_time_;
            ss << " (已进行 " << std::fixed << std::setprecision(1) << elapsed.toSec() << " 秒)";
        }
    } else {
        ss << "Phase1: 未启用";
    }
    
    ss << " | 扫描状态: " << (is_scanning_.load() ? "进行中" : "停止");
    
    return ss.str();
}

void PhaseFusionSonarMapper::printPhase1DebugInfo() {
    ROS_INFO("=== Phase1 调试信息 ===");
    ROS_INFO("启用状态: %s", phase1_enabled_ ? "true" : "false");
    ROS_INFO("当前步骤: %d", phase1_step_);
    ROS_INFO("扫描状态: %s", is_scanning_.load() ? "进行中" : "停止");
    ROS_INFO("子地图数量: %d", phase1_submap_count_);
    
    if (phase1_step_ > 0) {
        ros::Duration elapsed = ros::Time::now() - phase1_step_start_time_;
        ROS_INFO("当前步骤已进行: %.1f 秒", elapsed.toSec());
    }
    
    ROS_INFO("扫描时长: %.1f 秒", phase1_scan_duration_);
    ROS_INFO("等待时长: %.1f 秒", phase1_wait_duration_);
    ROS_INFO("移动距离: %.2f 米", phase1_move_distance_);
    ROS_INFO("=========================");
}

 
 
bool PhaseFusionSonarMapper::getCurrentRobotPoseInMap(geometry_msgs::Pose2D& pose) {
    // 尝试从TF链获取：map -> base_link
    try {
        tf::StampedTransform transform;
        
        // 使用正确的变换链：map->base_link
        // 先等待TF链完整
        tf_listener_.waitForTransform("map", "base_link", 
                                     ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("map", "base_link", 
                                    ros::Time(0), transform);
        
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
        pose.theta = tf::getYaw(transform.getRotation());
        
        // 验证位姿合理性
        const float MAX_REASONABLE_DISTANCE = 20.0f;  // 水池最大20米
        
        if (fabs(pose.x) > MAX_REASONABLE_DISTANCE || 
            fabs(pose.y) > MAX_REASONABLE_DISTANCE) {
            ROS_WARN("TF位姿超出水池范围: (%.2f, %.2f)", pose.x, pose.y);
            return false;
        }
        
        ROS_INFO("成功获取map坐标系位姿: (%.2f, %.2f, %.1f°)", 
                 pose.x, pose.y, pose.theta * RAD_TO_DEG);
        return true;
    }
    catch (tf::TransformException& ex) {
        ROS_WARN("无法获取map->base_link位姿: %s", ex.what());
        
        // 备用方案：使用odom坐标系，假设初始位置在地图原点
        try {
            tf::StampedTransform odom_to_base;
            tf_listener_.lookupTransform("odom", "base_link", 
                                        ros::Time(0), odom_to_base);
            
            pose.x = odom_to_base.getOrigin().x();
            pose.y = odom_to_base.getOrigin().y();
            pose.theta = tf::getYaw(odom_to_base.getRotation());
            
            ROS_INFO("使用odom坐标系位姿: (%.2f, %.2f, %.1f°)", 
                     pose.x, pose.y, pose.theta * RAD_TO_DEG);
            return true;
        }
        catch (tf::TransformException& ex2) {
            ROS_WARN("也无法获取odom位姿: %s", ex2.what());
            return false;
        }
    }
}



} // namespace sonar_slam


// 回调函数定义
 
bool startPhase1Callback(sonar_slam::StartPhase1::Request& req,
                         sonar_slam::StartPhase1::Response& res,
                         sonar_slam::PhaseFusionSonarMapper* mapper) {
    ROS_INFO("Service called: /start_phase1_mapping with max_submaps=%d", req.max_submaps);
    mapper->startPhase1Mapping(req.max_submaps);
    res.success = true;
    res.message = "Phase1 mapping started with max_submaps=" + std::to_string(req.max_submaps);
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}

bool stopPhase1Callback(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res,
                        sonar_slam::PhaseFusionSonarMapper* mapper) {
    ROS_INFO("Service called: /stop_phase1_mapping");
    mapper->stopPhase1Mapping();
    res.success = true;
    res.message = "Phase1 mapping stopped";
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}
 

bool getStatusCallback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res,
                       sonar_slam::PhaseFusionSonarMapper* mapper) {
    res.success = true;
    res.message = mapper->getPhase1StatusString();
    return true;
}



// ======================== 主函数 ========================
int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");  // 确保数字格式正确
    ros::init(argc, argv, "phase_fusion_sonar_mapper");
    
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
    sonar_slam::PhaseFusionSonarMapper mapper(nh);
    
    // ==================== 注册服务（使用Boost绑定） ====================
  

     // 创建服务 - 使用全局节点句柄
    ros::NodeHandle nh_global;  // 全局命名空间
    
    ros::ServiceServer start_srv = nh_global.advertiseService<sonar_slam::StartPhase1::Request, sonar_slam::StartPhase1::Response>(
        "start_phase1_mapping",
        boost::bind(&startPhase1Callback, _1, _2, &mapper));
    
    ros::ServiceServer stop_srv = nh_global.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "stop_phase1_mapping",
        boost::bind(&stopPhase1Callback, _1, _2, &mapper));
    
    ros::ServiceServer status_srv = nh_global.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "get_phase1_status",
        boost::bind(&getStatusCallback, _1, _2, &mapper));
    
    ROS_INFO("Services created: /start_phase1_mapping, /stop_phase1_mapping, /get_phase1_status");
    
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
        
        ROS_INFO("=== 自动启动第一阶段建图 ===");
        
        // 等待2秒，确保所有组件就绪
        ros::Duration(2.0).sleep();
        
        // ：启动第一阶段
        mapper.startPhase1Mapping(max_submaps);
        ROS_INFO("第一阶段建图已启动，最大子地图数: %d", max_submaps);
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
 
    int loop_counter = 0;
    ros::Rate rate(10);  // 10Hz

    while (ros::ok()) {
        ros::spinOnce();

        // 确保TF被发布
        mapper.publishTF();
        
        // 确保Phase1状态机被调用
        mapper.handlePhase1Mapping();
        
        // 每5秒输出一次状态
        if (loop_counter % 50 == 0) {
            std::string status = mapper.getPhase1StatusString();
            ROS_INFO("[主循环] %s", status.c_str());
            
            // 如果启用了Phase1，打印更多调试信息
            if (mapper.isPhase1Active()) {
                mapper.printPhase1DebugInfo();
            }
        }
        loop_counter++;
        
        rate.sleep();
    }
    
    
    mapper.stopScanning();
    
    ROS_INFO("Sonar Realtime Mapper 关闭");
    return 0;
}