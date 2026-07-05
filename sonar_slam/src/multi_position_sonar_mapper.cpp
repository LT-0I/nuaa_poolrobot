/**
 * @file multi_position_sonar_mapper.cpp
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

#include "sonar_slam/multi_position_sonar_mapper.h"
#include <sonar_return_home_file_detector/return_home_file_api.h>
 
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
#include <ctime>      // 会话目录时间戳 strftime/localtime
#include <cstdio>     // snprintf(localtime 兜底分支)

#include <locale>
#include <codecvt>
#include <algorithm>  // 如果需要使用 std::min/max
#include <cmath>      // 数学函数
#include <limits>     // FLT_MAX

// 版本横幅(grep 自判): 改行为必改此串。本轮= 沿边+声呐采集入口(智能会话目录 + 可选找墙自启)
#define MULTIMAP_VERSION "wallscan-smartdir-acquire-stitchhook-20260622"

namespace sonar_slam {

using namespace std::chrono;

// ======================== 常量定义 ========================
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// ======================== 构造函数和析构函数 ========================

MultiPositionSonarMapper::MultiPositionSonarMapper(ros::NodeHandle& nh)
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
    , angle_base_(-1)
    , angle_base_initialized_(false)
    , phase_multi_enabled_(false)  // 添加这一行
    , phase_multi_step_(0)
    , phase_multi_num_positions_(3)
    , phase_multi_current_position_(1)
    , accumulating_scan_(true)                // 默认开始累积
    , required_angles_for_full_scan_(360)     // 默认需要360个角度
    , odom_log_opened_(false)
 
 {
    
    ROS_INFO("========================================");
    ROS_INFO("Sonar Realtime Mapper 初始化");
    ROS_INFO("========================================");
    
    // 初始化接收缓冲区
    memset(rx_buffer_, 0, sizeof(rx_buffer_));
    
    // 初始化原始数据缓冲区
    raw_points_buffer_.resize(MAX_RAW_POINTS);
    
  

    // ==================== 读取参数（移除所有硬编码限制） ====================
    nh_.param("min_intensity", min_intensity_, 0.0f);   // 改为0，接受所有强度
    nh_.param("sound_speed", sound_speed_, 1500.0f);

    // 距离范围 - 使用声纳硬件的最大能力
    nh_.param("max_range", max_range_, 100.0f);    // 设大一点，实际由硬件决定
    nh_.param("min_range", min_range_, 0.0f);      // 不限制最小距离

    // 地图参数 - 设置足够大的值，实际点云会自适应
    nh_.param("map_resolution", map_resolution_, 0.05f);
    nh_.param("map_width", map_width_, 4000);        // 200米范围
    nh_.param("map_height", map_height_, 4000);      // 200米范围
    nh_.param("map_origin_x", map_origin_x_, -100.0f);
    nh_.param("map_origin_y", map_origin_y_, -100.0f);

    // 代价地图参数
    nh_.param("costmap_resolution", costmap_resolution_, 0.1f);
    nh_.param("costmap_width", costmap_width_, 2000);
    nh_.param("costmap_height", costmap_height_, 2000);
    nh_.param("costmap_origin_x", costmap_origin_x_, -100.0f);
    nh_.param("costmap_origin_y", costmap_origin_y_, -100.0f);
            

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

    std::string phase_mode = "standard";  // standard, multi
    nh_.param("phase1/mode", phase_mode, std::string("standard"));

 
   
    nh_.param("phase1/num_positions", phase_multi_num_positions_, 3);
    nh_.param("phase1/scan_duration", phase_multi_scan_duration_, 100.0f);
    nh_.param("phase1/wait_duration", phase_multi_wait_duration_, 60.0f);
    nh_.param("phase1/move_distance", phase_multi_move_distance_, 2.0f);
    nh_.param("phase1/required_circles", phase_multi_circles_per_position_, 1);
    // 沿边+声呐入口: 扫描前先找墙对齐(下水自启), 默认关; 由 wall_follow_sonar_scan.launch 置真。命名空间沿用 phase1/
    nh_.param("phase1/acquire_wall_first", acquire_wall_first_, false);
    nh_.param("phase1/acquire_distance", acquire_distance_, 0.5f);
    // 拼图开关(私有 ~enable_stitch): true=采集全部停点完成时发会话目录, 触发离线拼图节点; false=不发(行为同接入前)
    nh_.param("enable_stitch", enable_stitch_, true);
    
 
    
    ROS_INFO("========================================");
    ROS_INFO("多位置建图模式已启用");
    ROS_INFO("  位置数量: %d (3-50)", phase_multi_num_positions_);
    ROS_INFO("  每个位置圈数: %d", phase_multi_circles_per_position_);
    ROS_INFO("  扫描时长: %.1f秒", phase_multi_scan_duration_);
    ROS_INFO("  等待时长: %.1f秒", phase_multi_wait_duration_);
    ROS_INFO("========================================");
   
 
    // 多位置建图发布器
    phase_multi_status_pub_ = nh_.advertise<std_msgs::String>("/phase_multi/status", 1, true);
    phase_multi_submap_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/phase_multi/submaps", 1, true);
    // 运行级水泵闩锁发布器(latched): 任务起始 true / 结束·中止 false → move_controller 整段持泵
    pump_enable_pub_ = nh_.advertise<std_msgs::Bool>("/move_controller/pump_enable", 1, true);
    // 开机主动撤闩一次(幂等, 不置 pump_latch_published_open_): 清除上次进程异常退出可能残留在 move_controller 的持泵闩锁。
    // 本进程已删 openPumpLatch(从不置真), 故这仅清陈旧状态; latched 话题的迟到订阅者(move_controller 后启)也会收到此 false。
    { std_msgs::Bool m; m.data = false; pump_enable_pub_.publish(m); }
    // 采集完成信号(latched): 采集全部停点完成时发本次会话目录(绝对路径), 触发离线拼图节点 sonar_stitch_node
    session_done_pub_ = nh_.advertise<std_msgs::String>("/sonar_scan/session_done", 1, true);
    ROS_INFO("[%s] enable_stitch=%s", MULTIMAP_VERSION, enable_stitch_ ? "true" : "false");
    // 添加标准地图发布器
    standard_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    
    ROS_INFO("标准地图发布器创建: /map");
    
    
     // ===== 从参数服务器读取配置 =====
    nh_.param("required_angles_for_full_scan", required_angles_for_full_scan_, 360);
    nh_.param("accumulating_scan", accumulating_scan_, true);
    // 扫描每角步间距: 原硬编码 0.1s, 每圈 400 步 => 40s/圈固定死时间(回波在等待循环里已收完, 本 sleep 纯角间 idle)。
    // 暴露为私有参数 ~sweep_pace_sec, 默认 0.1 = 与旧版逐字节等(零行为变化)。现场可下调缩短单圈扫描。
    // ⚠ 降 pace 非严格保内容: 一圈完成到存盘有 ~0.2~0.7s 窗口(状态机 sleep + TF 等待), 期间下一圈近 angle-0 的
    //   驻点重复帧会混入本子图(帧数随 pace 下降而增多), 略抬 angle-0 局部密度 -> 喂 MIN_INNER 滤波 -> 拼图非"跨 pace 逐字节等"。
    //   角数/几何不变、默认 0.1 完全不受影响; 但降 pace 后须实测一圈(400/400 角收齐、无单角超时 WARN、拼图正常)再用于 demo。
    nh_.param("sweep_pace_sec", sweep_pace_sec_, 0.1);
    if (sweep_pace_sec_ < 0.02) sweep_pace_sec_ = 0.02;  // 下限0.02(仍~5x提速; 防 pace→0 每帧重建点云打满 OPi5 弱核致单角丢帧超时)
    if (sweep_pace_sec_ > 1.0)  sweep_pace_sec_ = 1.0;   // 上限防误设
    ROS_INFO("[sweep] per-angle pace=%.3fs (~%.0fs per 400-step circle floor); ~sweep_pace_sec default 0.1 clamp[0.02,1.0]", sweep_pace_sec_, sweep_pace_sec_ * 400.0);

    // ===== 扫描前"泵关+转停"守门参数 (私有 ~ 命名空间) =====
    nh_.param("pump_guard_enable",     pump_guard_enable_,     true);
    nh_.param("pump_settle_sec",       pump_settle_sec_,       2.5);
    nh_.param("pump_off_thresh",       pump_off_thresh_,       0.02);
    nh_.param("pump_stopped_thresh",   pump_stopped_thresh_,   0.02);
    nh_.param("pump_guard_timeout_sec",pump_guard_timeout_sec_,6.0);
    nh_.param("pump_guard_stable_sec", pump_guard_stable_sec_, 0.5);
    nh_.param("pump_cmd_fresh_sec",    pump_cmd_fresh_sec_,     0.3);
    // stable<=0 退化配置会令下方 fresh 钳成<=0 -> age<fresh 永假 -> 健康发布者也永不确认(狂刷 ERROR); 先给 stable 一个下限。
    if (pump_guard_stable_sec_ < 0.1) {
        ROS_WARN("[pump-guard] pump_guard_stable_sec(%.3f) too small; floored to 0.1 (else gate can never confirm even with a healthy publisher)", pump_guard_stable_sec_);
        pump_guard_stable_sec_ = 0.1;
    }
    // 新鲜判据须严格 < stable: 否则发布者已死时, 其陈旧末值(pump_off+stopped)仍能在 fresh 未过期的窗口内凑满 stable -> 假确认。
    // 钳到 stable 的 0.6 倍, 确保死掉的发布者在 fresh 过期(=> ok_now 断) 时 stable 尚未满 -> 只能超时(报错仍扫), 不假确认。
    if (pump_cmd_fresh_sec_ >= pump_guard_stable_sec_) {
        double clamped = pump_guard_stable_sec_ * 0.6;
        ROS_WARN("[pump-guard] pump_cmd_fresh_sec(%.3f) >= stable(%.3f); clamped to %.3f to keep fresh<stable (avoid dead-publisher false-confirm)",
                 pump_cmd_fresh_sec_, pump_guard_stable_sec_, clamped);
        pump_cmd_fresh_sec_ = clamped;
    }
    // 只读观测 /cmd_vel(绝对话题, 前导 / 忽略私有 ~ 命名空间 -> 解析为全局 /cmd_vel)。仅供守门判泵关(linear.y)+车停。
    // 挂 pump_gate_queue_ 专用回调队列(非全局队列): 守门用 pump_gate_queue_.callAvailable() 只派发本 onCmdVel, 绝不重入本节点的 6 个服务回调;
    // 主循环每拍另调 spinPumpGateQueue() 持续派发本队列, 令 last_cmd_* 无陈旧积压、新鲜判据可靠。
    {
        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/cmd_vel", 1,
            boost::bind(&MultiPositionSonarMapper::onCmdVel, this, _1),
            ros::VoidConstPtr(), &pump_gate_queue_);
        cmd_vel_sub_ = nh_.subscribe(so);
    }
    ROS_INFO("[pump-guard] enable=%s settle=%.1fs off_thr=%.3f stop_thr=%.3f timeout=%.1fs stable=%.2fs fresh=%.2fs",
             pump_guard_enable_ ? "true" : "false", pump_settle_sec_, pump_off_thresh_,
             pump_stopped_thresh_, pump_guard_timeout_sec_, pump_guard_stable_sec_, pump_cmd_fresh_sec_);

    // 确保缓冲区已重置
    full_scan_buffer_.reset();
    phase1_submaps_.reserve(20);        // 足够安全
    phase_multi_submaps_.reserve(100);   // 足够安全

    // ===== 智能会话目录: 每次运行一个带时间戳子目录, 避免多次运行互相覆盖(maps/raw_sonar/odom 全落本目录) =====
    {
        const char* home_env = getenv("HOME");
        std::string home_dir = home_env ? home_env : "/tmp";
        char tbuf[32] = {0};
        std::time_t tt = std::time(nullptr);
        std::tm* lt = std::localtime(&tt);
        if (lt) std::strftime(tbuf, sizeof(tbuf), "%Y%m%d_%H%M%S", lt);
        else    snprintf(tbuf, sizeof(tbuf), "%u", static_cast<unsigned>(tt));
        std::string base_root = home_dir + "/sonar_logs/";
        session_dir_ = base_root + "scan_" + tbuf + "/";
        struct stat st0;
        if (stat(base_root.c_str(), &st0) != 0)    mkdir(base_root.c_str(), 0755);
        if (stat(session_dir_.c_str(), &st0) != 0) mkdir(session_dir_.c_str(), 0755);
        ROS_INFO("[%s] sonar session dir: %s", MULTIMAP_VERSION, session_dir_.c_str());
    }

    // 打开 odom 日志文件(落在本次会话目录)
    std::string odom_log_path = session_dir_ + "sonar_odom_log.csv";
    odom_log_file_.open(odom_log_path, std::ios::out | std::ios::app);
    if (odom_log_file_.is_open()) {
        odom_log_file_ << "timestamp,submap_index,position,pose_x,pose_y,pose_theta_deg,pointcloud_file\n";
        ROS_INFO("Odom日志文件: %s", odom_log_path.c_str());
    }
 
    ros::NodeHandle nh_global;
    move_straight_client_ = nh_global.serviceClient<sonar_slam::MoveStraight>(
        "/move_controller/move_straight");
    // 找墙服务(可选 acquire_wall_first 用); 复用 MoveStraight srv: distance=对齐后沿边距离, timeout=超时
    find_wall_client_ = nh_global.serviceClient<sonar_slam::MoveStraight>(
        "/move_controller/find_wall");

    ROS_INFO("等待move_controller服务...");
    if (move_straight_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_INFO("✅ move_controller服务已就绪");
    } else {
        ROS_WARN("⚠️ move_controller服务未就绪，将在运行时重试");
    }

    check_loop_closure_ = false;
 

}


MultiPositionSonarMapper::~MultiPositionSonarMapper() {
    closePumpLatch();   // 兜底撤闩(绕过 main 关闭的退出路径); ROS 已关时为 best-effort no-op
    stopScanning();
    
    if (sonar_serial_) {
        sonar_serial_->close();
    }
    
    if (csv_file_.is_open()) {
        csv_file_.close();
    }

    if (odom_log_file_.is_open()) {
        odom_log_file_.close();
    }
    
    ROS_INFO("Sonar Realtime Mapper 销毁");
}

// ======================== 串口初始化 ========================

bool MultiPositionSonarMapper::initSerial(const std::string& port, int baudrate) {
    try {
        sonar_serial_ = std::make_unique<CallbackAsyncSerial>(port, baudrate);
        sonar_serial_->setCallback(
            boost::bind(&MultiPositionSonarMapper::receiveDataCallback, this, _1, _2));
        
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

void MultiPositionSonarMapper::receiveDataCallback(const char* data, unsigned int len) {
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

void MultiPositionSonarMapper::parseDataPacket(const char* buffer, unsigned int length) {
    while (rx_used_index_ < rx_end_index_) {
        unsigned char current_byte = rx_buffer_[rx_used_index_];
        
        switch (parse_state_) {
            case STATE_IDLE:
                if (current_byte == 0x42) { // 'B'
                    parse_state_ = STATE_GOT_START1;
                    // ROS_DEBUG("找到第一个帧头 0x42");
                }
                rx_used_index_++;
                break;
                
            case STATE_GOT_START1:
                if (current_byte == 0x52) { // 'R'
                    parse_state_ = STATE_READING_LENGTH_LOW;
                    // ROS_DEBUG("找到第二个帧头 0x52");
                } else {
                    // 不是预期的第二个头，重新同步
                    // ROS_DEBUG("无效的第二个帧头: 0x%02X, 期望 0x52", current_byte);
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
                    // ROS_DEBUG("包处理完成，准备接收下一个");
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
                // ROS_DEBUG("在缓冲区中找到新帧头，重新开始解析");
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

void MultiPositionSonarMapper::processCompleteMessage() {
    total_scans_++;
    
    // ====== 调试日志：确认进入此函数 ======
    // ROS_INFO_ONCE("[DEBUG] 首次进入 processCompleteMessage 函数。");
    // ROS_DEBUG_THROTTLE(1.0, "处理消息: cmd=0x%04X, len=%u", current_message_.command, current_message_.length);
    // ROS_INFO("========== 收到完整消息 ==========");
    // ROS_INFO("命令字: 0x%04X (%d)", current_message_.command, current_message_.command);
    // ROS_INFO("消息长度: %u", current_message_.length);
    // ROS_INFO("源地址: %u", current_message_.source);
    // ROS_INFO("目的地址: %u", current_message_.destination);
    // ROS_INFO("数据大小: %lu", current_message_.data.size());

    // 打印前16字节原始数据（如果有）
    // if (!current_message_.data.empty()) {
    //     std::stringstream hex_ss;
    //     hex_ss << "数据前16字节 (十六进制): ";
    //     for (size_t i = 0; i < std::min(current_message_.data.size(), size_t(16)); i++) {
    //         hex_ss << std::hex << std::setw(2) << std::setfill('0')
    //                << (int)current_message_.data[i] << " ";
    //     }
    //     ROS_INFO("%s", hex_ss.str().c_str());
    //
    //     std::stringstream dec_ss;
    //     dec_ss << "数据前16字节 (十进制): ";
    //     for (size_t i = 0; i < std::min(current_message_.data.size(), size_t(16)); i++) {
    //         dec_ss << std::dec << std::setw(3) << (int)current_message_.data[i] << " ";
    //     }
    //     ROS_INFO("%s", dec_ss.str().c_str());
    // }
    // ROS_INFO("==================================");
    // ==========过滤控制命令回显 ==========
    // 检查数据是否以 'B','R' 开头（可能是命令回显）
    if (current_message_.data.size() >= 2 && 
        current_message_.data[0] == 0x42 && 
        current_message_.data[1] == 0x52) {
        // ROS_DEBUG("检测到可能的命令回显，跳过处理");
        return;
    }
    
    // 根据命令字处理消息
    switch (current_message_.command) {
        case 2300: // 声纳数据消息
            //  ROS_INFO_THROTTLE(0.5, "[DEBUG] 处理CMD 2300消息，数据大小: %lu", current_message_.data.size());
            if (parseCmd2300Message(current_message_.data)) {
                valid_scans_++;
                //  ROS_DEBUG_THROTTLE(0.5, "成功解析2300消息 (#%u)", valid_scans_);
            } else {
                // ========== 解析失败时的错误恢复 ==========
                // ROS_DEBUG("解析2300消息失败，可能是错误的数据包");
                // 不需要特殊处理，状态机已经重置
            }
            break;
            
        default:
            // ROS_DEBUG("收到未知命令: 0x%04X", current_message_.command);
            break;
    }
    // 根据命令字处理消息
    
}

// ======================== 解析2300消息 ========================

bool MultiPositionSonarMapper::parseCmd2300Message(const std::vector<uint8_t>& data) {
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
    // ROS_WARN("===== 结构体字段与原始数据对应关系 =====");
    // ROS_WARN("sizeof(MsgCmd2300): %lu", sizeof(MsgCmd2300));

    // 逐字节打印前20个字节的原始值
    // ROS_WARN("原始数据索引及值:");
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
    

    // ========== 添加原始角度输出 ==========
    static int msg_count = 0;
    static int min_angle = 400, max_angle = -1;

    if (msg_count < 200) {  // 只记录前200条消息
        if (msg_data.angle < min_angle) min_angle = msg_data.angle;
        if (msg_data.angle > max_angle) max_angle = msg_data.angle;
        
        // if (msg_count % 20 == 0) {
        //     ROS_ERROR("原始角度[%d]: %d (当前范围: %d~%d)",
        //             msg_count, msg_data.angle, min_angle, max_angle);
        // }
        msg_count++;

        // if (msg_count == 200) {
        //     ROS_ERROR("========== 原始角度统计 ==========");
        //     ROS_ERROR("共收到 %d 条消息", msg_count);
        //     ROS_ERROR("原始角度范围: %d ~ %d", min_angle, max_angle);
        //     ROS_ERROR("理论范围应该是 0 ~ 399");
        //     if (min_angle == 0 && max_angle == 399) {
        //         ROS_ERROR("✓ 角度范围正常，覆盖完整360度");
        //     } else {
        //         ROS_ERROR("⚠ 角度范围异常，可能声纳配置有问题");
        //     }
        //     ROS_ERROR("=================================");
        // }
    }
 
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

 
void MultiPositionSonarMapper::processScanFrame(int raw_angle, const MsgCmd2300& msg_data) {
    if (!is_scanning_.load()) {
        return;
    }
    
    // ==================== 原始数据模式 ====================
    // 移除RAW_DATA_MODE开关，直接使用原始模式
    // 所有强度过滤代码全部移除
    
    // ==================== 1. 角度转换修正 ====================
    if (!angle_base_initialized_) {
        if (raw_angle >= 0 && raw_angle < 400) {
            angle_base_ = raw_angle;
            angle_base_initialized_ = true;
            ROS_INFO("角度基准设为: %d (位置%d)", angle_base_, phase1_current_position_);
        } else {
            ROS_ERROR("raw_angle异常: %d，使用默认值0", raw_angle);
            angle_base_ = 0;
            angle_base_initialized_ = true;
        }
    }
    
    int corrected = raw_angle - angle_base_;
    while (corrected < 0) corrected += 400;
    while (corrected >= 400) corrected -= 400;
    
    if (corrected < 0 || corrected >= 400) {
        ROS_ERROR("角度校正错误: raw=%d, base=%d, corrected=%d", 
                  raw_angle, angle_base_, corrected);
        corrected = raw_angle;
        while (corrected >= 400) corrected -= 400;
        while (corrected < 0) corrected += 400;
    }
    
    float angle_deg = static_cast<float>(corrected) * 0.9f;
    if (angle_deg < -360.0f || angle_deg > 360.0f) {
        ROS_ERROR("角度计算异常: raw=%d, base=%d, corrected=%d, angle_deg=%.1f", 
                  raw_angle, angle_base_, corrected, angle_deg);
        angle_deg = fmod(angle_deg, 360.0f);
    }
    
    angle_deg = fmod(angle_deg, 360.0f);
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    
    covered_angles_.insert(static_cast<int>(angle_deg));
    float angle_rad = angle_deg * DEG_TO_RAD;
    int hw_angle_idx = corrected;
    
    // ==================== 2. 距离计算参数 ====================
    float time_per_point_us = static_cast<float>(msg_data.sample_period) * 25.0f / 1000.0f;
    // float distance_per_point_mm = time_per_point_us * 1.45f;
    float distance_per_point_mm = time_per_point_us * 0.73f;
    uint16_t actual_samples = std::min(msg_data.number_of_samples, msg_data.data_length);
    
    // 创建声纳点结构
    SonarPoint sonar_point;
    sonar_point.angle = angle_rad;
    sonar_point.timestamp = ros::Time::now();
    
    // ==================== 3. 处理所有采样点（无强度过滤） ====================
    int valid_samples = 0;
    
    for (uint16_t i = 0; i < actual_samples; i++) {
        float distance_mm = distance_per_point_mm * (i + 1);
        float distance_m = distance_mm * 0.001f;
        
        // 只应用距离范围过滤（硬件限制）
        if (distance_m > max_range_) continue;
        if (distance_m < min_range_) continue;
        
        // 获取原始强度值，不做任何修改
        float intensity = static_cast<float>(msg_data.data[i]);
        
        // 直接添加所有点，不做任何强度判断
        sonar_point.ranges.push_back(distance_m);
        sonar_point.intensities.push_back(intensity);
        valid_samples++;
    }
    
    // ==================== 4. 数据累积 ====================
    if (valid_samples > 0) {
        // 累计缓冲写入(受 accum_mutex_ 保护): 主线程重建/落盘会拷此缓冲, 故写入须持锁。
        // 临界区极短(仅一次 push_back), 不含任何重活。
        {
            std::lock_guard<std::mutex> lk(accum_mutex_);
            sonar_points_.push_back(sonar_point);
        }

        // 累积到全扫描缓冲区
        if (accumulating_scan_) {
            if (!full_scan_buffer_.angle_received[hw_angle_idx]) {
                full_scan_buffer_.angle_received[hw_angle_idx] = true;
                full_scan_buffer_.angles_completed++;
            }
            
            // 存储原始数据到缓冲区
            for (uint16_t i = 0; i < actual_samples; i++) {
                float distance_mm = distance_per_point_mm * (i + 1);
                float distance_m = distance_mm * 0.001f;
                if (distance_m > max_range_) continue;
                if (distance_m < min_range_) continue;
                
                float intensity = static_cast<float>(msg_data.data[i]);
                full_scan_buffer_.ranges[hw_angle_idx].push_back(distance_m);
                full_scan_buffer_.intensities[hw_angle_idx].push_back(intensity);
            }
        }
        
        // 记录到CSV
        if (enable_csv_logging_) {
            logToCSV(sonar_point);
        }

        // ==================== 5. 点云转换和地图更新 ====================
        // [方案A] 原本在此(串口I/O线程内)每帧做 convertToPointCloud/filterPointCloud/
        //   createOccupancyGrid/createCostMap/publishMapData 的重活, 会在低核 OPi5 上把 I/O 线程打满、
        //   拖慢下一角命令出线与回波解析 -> 单角/整圈超时(尤其 sweep_pace 调小时)。
        // 现全部搬出 I/O 线程: 改由主线程周期(~2Hz, rebuildAndPublishMaps) + 落盘前强制刷新执行。
        //   本回调只保留轻活: 解析 + 上面的累计缓冲写入 + full_scan_buffer_ 置位/累积。
        //   注: 角收齐置位(full_scan_buffer_.angle_received/angles_completed)已在上方完成, 与显示刷新解耦,
        //   故 scanningThread 的收角判定不受本改动影响; 发角节拍完全不变。

    } else {
        dropped_scans_++;
        if (dropped_scans_ % 100 == 0) {
            ROS_WARN_THROTTLE(10.0, "已丢弃 %u 帧无有效数据", dropped_scans_);
        }
    }
}


void MultiPositionSonarMapper::saveStageMap(const std::string& stage_name) {
 
     // 空指针检查
    if (!pointcloud_) {
        ROS_ERROR("saveStageMap: pointcloud_ 为空指针");
        return;
    }
    
    // 空数据检查
    if (pointcloud_->empty()) {
        ROS_WARN("saveStageMap: 点云为空，跳过保存 [%s]", stage_name.c_str());
        return;
    }
    struct stat st;
    std::string maps_dir  = getMapsDirectory();
     
    
    // 3. 生成带日期时间的文件名
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tm_info = std::localtime(&now_time_t);
    char datetime_buf[20];
    strftime(datetime_buf, sizeof(datetime_buf), "%Y%m%d_%H%M%S", tm_info);
    std::string datetime_str(datetime_buf);
    
    // 生成基础文件名
    std::string base_name;
    if (stage_name.find("final") != std::string::npos || 
        stage_name.find("merged") != std::string::npos ||
        stage_name.find("pool_map") != std::string::npos) {
        base_name = "sonar360_pool_map_" + datetime_str;  // 添加sonar360前缀
    } else {
        base_name = "sonar360_" + stage_name + "_" + datetime_str;
    }
    
    ROS_INFO("保存声呐地图: %s", base_name.c_str());
    
    // 4. 保存点云数据（添加坐标系信息）
    std::string pcd_filename = maps_dir + base_name + ".pcd";

    if (!pointcloud_->empty()) {
        // 直接保存，不修改frame_id，不创建副本
        if (pcl::io::savePCDFileBinary(pcd_filename, *pointcloud_) == 0) {
            ROS_INFO("保存点云: %s (%lu 点, 坐标系: %s)", 
                    pcd_filename.c_str(), 
                    pointcloud_->size(),
                    pointcloud_->header.frame_id.c_str());  // 输出: sonar_frame
        } else {
            ROS_ERROR("保存点云失败: %s", pcd_filename.c_str());
        }
    }
    
    // 5. 保存PGM地图（使用修改后的标准函数）
    std::string pgm_filename = maps_dir + base_name + ".pgm";
    if (!saveMapAsPGM(pgm_filename, occupancy_map_)) {
        ROS_ERROR("保存PGM失败: %s", pgm_filename.c_str());
    }
    
    // 6. 保存YAML配置（使用修改后的标准函数）
    std::string yaml_filename = maps_dir + base_name + ".yaml";
    if (!saveMapAsYAML(yaml_filename, base_name + ".pgm", occupancy_map_)) {
        ROS_ERROR("保存YAML失败: %s", yaml_filename.c_str());
    }
    
    // 7. 可选：保存一个信息文件，记录声呐参数
    std::string info_filename = maps_dir + base_name + "_info.yaml";
    std::ofstream info_file(info_filename);
    if (info_file.is_open()) {
        info_file << "# Sonar360 Mapping Information\n";
        info_file << "map_file: " << base_name << ".yaml\n";
        info_file << "pointcloud_file: " << base_name << ".pcd\n";
        info_file << "timestamp: " << datetime_str << "\n";
        info_file << "sensor_type: mechanical_scanning_single_beam\n";
        info_file << "scan_resolution_deg: 1.0\n";
        info_file << "max_range: " << max_range_ << "\n";
        info_file << "min_range: " << min_range_ << "\n";
        info_file << "sound_speed: " << sound_speed_ << "\n";
        info_file.close();
        ROS_DEBUG("保存信息文件: %s", info_filename.c_str());
    }
}

// ======================== 修改后的 saveMapAsPGM ========================
bool MultiPositionSonarMapper::saveMapAsPGM(const std::string& filename, 
                                       const nav_msgs::OccupancyGrid& map) {
    std::ofstream pgm_file(filename);
    if (!pgm_file.is_open()) {
        ROS_ERROR("无法打开PGM文件: %s", filename.c_str());
        return false;
    }
    
    // 【修改1】PGM文件头：maxval改为255（ROS标准）
    pgm_file << "P2\n";
    pgm_file << "# Sonar360 SLAM Map - " << filename << "\n";
    pgm_file << "# Generated: " << ros::Time::now() << "\n";
    pgm_file << map.info.width << " " << map.info.height << "\n";
    pgm_file << "255\n";  // 原: 100，现: 255
    
    // 地图数据（行倒序，因为PGM是左下角原点）
    for (int y = map.info.height - 1; y >= 0; y--) {
        for (int x = 0; x < map.info.width; x++) {
            int value = map.data[y * map.info.width + x];
            
            // 【修改2】灰度值映射改为ROS标准
            int pgm_value;
            if (value < 0) {
                // 未知区域：中性灰（127）
                pgm_value = 127;  // 原: 50，现: 127
            } else if (value == 0) {
                // 自由空间：白色（255）
                pgm_value = 255;   // 原: 0，现: 255
            } else {
                // 占据区域：黑色（0）
                pgm_value = 0;     // 原: 100，现: 0
            }
            
            pgm_file << pgm_value;
            
            if (x < map.info.width - 1) {
                pgm_file << " ";
            }
        }
        pgm_file << "\n";
    }
    
    pgm_file.close();
    ROS_DEBUG("PGM保存成功: %s (%dx%d)", filename.c_str(), map.info.width, map.info.height);
    return true;
}

// ======================== 修改后的 saveMapAsYAML ========================
bool MultiPositionSonarMapper::saveMapAsYAML(const std::string& filename,
                                        const std::string& pgm_file,
                                        const nav_msgs::OccupancyGrid& map) {
    std::ofstream yaml_file(filename);
    if (!yaml_file.is_open()) {
        ROS_ERROR("无法打开YAML文件: %s", filename.c_str());
        return false;
    }
    
    // 【修复编译错误】使用 tf::getYaw 代替 tf2::getYaw
    double yaw = tf::getYaw(map.info.origin.orientation);
    
    // 计算统计信息
    int occupied = 0, free = 0, unknown = 0;
    for (int val : map.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }
    
    // YAML内容
    yaml_file << "# Sonar360 SLAM Map Configuration\n";
    yaml_file << "# Generated at: " << ros::Time::now() << "\n";
    yaml_file << "# ROS Time: " << ros::Time::now().toSec() << "\n\n";
    
    yaml_file << "image: " << pgm_file << "\n";
    yaml_file << "resolution: " << map.info.resolution << "\n";
    
    // origin包含yaw（第三分量）
    yaml_file << "origin: [" << map.info.origin.position.x << ", "
              << map.info.origin.position.y << ", " << yaw << "]\n";
    
    yaml_file << "negate: 0\n";
    
    // 阈值改为ROS标准值
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.196\n\n";
    
    // ========== 声呐专用元数据（扩展字段，不影响加载） ==========
    yaml_file << "# ========== Sonar360 Metadata ==========\n";
    yaml_file << "sensor_type: \"mechanical_scanning_single_beam\"\n";
    yaml_file << "scan_resolution_deg: 1.0\n";
    yaml_file << "max_range: " << max_range_ << "\n";
    yaml_file << "min_range: " << min_range_ << "\n";
    yaml_file << "sound_speed: " << sound_speed_ << "\n";
    yaml_file << "frame_id: \"" << map.header.frame_id << "\"\n\n";
    
    // 点云引用（可选，不影响导航，但方便后续点云定位）
    std::string pcd_filename = filename.substr(0, filename.find_last_of('.')) + ".pcd";
    yaml_file << "# Point cloud reference (for ICP/NDT localization)\n";
    yaml_file << "pointcloud: \"" << pcd_filename << "\"\n\n";
    
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
    ROS_INFO("YAML保存成功: %s", filename.c_str());
    return true;
}

// 函数：累积扇区数据
void MultiPositionSonarMapper::accumulateSectorData(int sector_index, const SonarPoint& point) {
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
    if (sector.intensity_accumulator.size() > 2000) {
        sector.intensity_accumulator.erase(sector.intensity_accumulator.begin(), 
                                          sector.intensity_accumulator.begin() + 1000);
        sector.range_accumulator.clear();  // 重建范围统计
        for (const auto& intensity : sector.intensity_accumulator) {
            // 重新计算范围分布（简化）
        }
    }
}

// 函数：生成更密集的点云
void MultiPositionSonarMapper::generateDensePointCloud() {
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

void MultiPositionSonarMapper::generateControlCommand(unsigned char* buffer, unsigned short angle) {
    std::string range_level;      // 局部变量
    nh_.param("range_level", range_level, std::string("s0"));  // 从参数服务器读取
    // 转换为小写
    std::string level_lower = range_level;  // 局部变量

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
    
    // 采样周期 (888) -
    //888 × 25us / 1000 = 22.2ms（每点时间）
    //22.2ms × 1500m/s = 33.3米（声波来回总路程）
    //33.3米 / 2 = 16.65米（单程距离）
    // buffer[14] = 0x78;  // 888 (888的低字节: 0x0378)
    // buffer[15] = 0x03;  // 3   (888的高字节)
    
    // 采样周期 = 1200
    // 1200 × 25us / 1000 = 30ms
    // 30ms × 1500m/s = 45米（来回）
    // 45米 / 2 = 22.5米（单程探测距离）
    // 采样周期 = 1200 (0x04B0)
    //buffer[14] = 0xB0;  // 低字节 176
    //buffer[15] = 0x04;  // 高字节 4
    //根据等级设置采样周期（直接写十六进制值）
  
    // 量程等级定义（从小到大）
    uint16_t samples = 0;

    if (level_lower == "s0") samples = 0x00D6;  // 214点，4.01米
    else if (level_lower == "s1") samples = 0x0140;  // 320点，6.00米（精确6米）
    else if (level_lower == "s2") samples = 0x01AB;  // 427点，8.01米
    else if (level_lower == "s3") samples = 0x0215;  // 533点，9.99米（约10米）
    else if (level_lower == "s4") samples = 0x0280;  // 640点，12.00米
    else if (level_lower == "s5") samples = 0x02EA;  // 746点，13.99米（约14米）
    else if (level_lower == "s6") samples = 0x0320;  // 800点，15.00米
    else if (level_lower == "s7") samples = 0x0355;  // 853点，15.99米（约16米）
    else if (level_lower == "s8") samples = 0x03C0;  // 960点，18.00米
    else if (level_lower == "s9") samples = 0x042B;  // 1067点，20.01米
    else if (level_lower == "s10") samples = 0x0490; // 1168点，21.90米（约22米）
    else if (level_lower == "s11") samples = 0x0500; // 1280点，24.00米
    else samples = 0x0140; // 默认320点，6.00米 

    buffer[14] = samples & 0xFF;
    buffer[15] = (samples >> 8) & 0xFF;
    max_range_ = samples * 25e-6f * 1500.0f / 2.0f;

    // ROS_INFO("量程设置: 等级=%s, 采样点数=%u, 理论距离=%.1f米",
    //         range_level.c_str(), samples, max_range_);


    // ROS_INFO("量程设置: 等级=%s, 采样点数=%u, 理论距离=%.1f米", range_level.c_str(), samples, max_range_);

 
            
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


void MultiPositionSonarMapper::scanningThread() {
    ROS_INFO("扫描线程启动");
    
    unsigned short angle = static_cast<unsigned short>(scan_start_angle_);
    
    if (angle >= 400) {
        angle = 0;
        scan_start_angle_ = 0;
        ROS_WARN("起始角度 %d 无效，重置为0", scan_start_angle_);
    }
    
    ROS_INFO("扫描从角度 %u 开始", angle);
    scan_start_time_ = ros::Time::now();
    
    int circles_completed_in_session = 0;
    
    while (!stop_requested_.load() && ros::ok()) {
        unsigned char cmd_buffer[24];
        generateControlCommand(cmd_buffer, angle);
        
        if (sonar_serial_) {
            try {
                sonar_serial_->write(reinterpret_cast<char*>(cmd_buffer), 24);
            } catch (const std::exception& e) {
                ROS_ERROR("发送命令失败: %s", e.what());
            }
        }
        
        current_angle_.store(angle);
        
        // ===== 等待这个角度的数据返回 =====
        int expected_angle = angle;
        ros::Time wait_start = ros::Time::now();
        const double TIMEOUT = 1.5;  // 固定100ms

        while (!full_scan_buffer_.angle_received[expected_angle] && 
            (ros::Time::now() - wait_start).toSec() < TIMEOUT) {
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }

        if (!full_scan_buffer_.angle_received[expected_angle]) {
            ROS_WARN_THROTTLE(10.0, "角度 %d 数据接收超时", expected_angle);
        }
        // ===== 结束 =====

        angle++;
        
        if (angle >= 400) {
            angle = 0;
            
            // ===== 新增：等待一圈数据接收完成 =====
            ros::Time wait_start = ros::Time::now();
            const double MAX_WAIT_TIME = 2.0;
            
            while (full_scan_buffer_.angles_completed < required_angles_for_full_scan_ && 
                   (ros::Time::now() - wait_start).toSec() < MAX_WAIT_TIME) {
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            
            if (full_scan_buffer_.angles_completed < required_angles_for_full_scan_) {
                ROS_WARN("等待超时，实际收到 %d/%d 个角度", 
                         full_scan_buffer_.angles_completed, 
                         required_angles_for_full_scan_);
            }
            // ===== 新增结束 =====
            
            circles_completed_in_session++;
            full_scan_complete_.store(true);
            
            ros::Duration scan_duration = ros::Time::now() - scan_start_time_;
            ROS_INFO("完成第%d圈扫描! 耗时: %.1f秒", 
                     circles_completed_in_session, scan_duration.toSec());
            
            // 重置缓冲区，为下一圈准备
            full_scan_buffer_.reset();
            
            scan_start_time_ = ros::Time::now();
        }
        
        ros::Duration(sweep_pace_sec_).sleep();   // 角间步进间距(原硬编码0.1); 见 ~sweep_pace_sec
    }

    ROS_INFO("扫描线程停止，最后角度: %d", current_angle_.load());
}
// ======================== [方案A] 快照重建并发布(重活搬出串口I/O线程) ========================
// 仅主线程调用(主循环 ~2Hz 显示 + saveCurrentPhaseMultiSubmap 落盘前强制刷新), 两者天然串行。
// 步骤: 1. accum_mutex_ 下把 sonar_points_ 拷进 sonar_points_snapshot_(I/O线程只被阻塞这一次拷贝);
//       2. 释放锁, 在快照上做 convert/filter/grid/cost/publish(全部重活在锁外, 绝不持 accum_mutex_ 做重活)。
void MultiPositionSonarMapper::rebuildAndPublishMaps() {
    {
        std::lock_guard<std::mutex> lk(accum_mutex_);
        sonar_points_snapshot_ = sonar_points_;   // 深拷贝一次(O(帧数), 远轻于下方重建/发布)
    }
    // 以下全部作用于 sonar_points_snapshot_ / pointcloud_ / 占据图, 仅主线程访问, 与落盘串行 -> 无并发。
    convertToPointCloud();   // 读 sonar_points_snapshot_ 重建 pointcloud_
    filterPointCloud();
    createOccupancyGrid();
    createCostMap();
    publishMapData();
}

// ======================== 点云转换 ========================
void MultiPositionSonarMapper::convertToPointCloud() {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
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
    // ROS_WARN("[坐标诊断] convertToPointCloud 第%d次被调用. rotation_angle_rad=%.4f (%.1f°)",
    //          call_count, rotation_angle_rad, rotation_angle_rad * RAD_TO_DEG);


    // 添加调试输出
    // [方案A] 本函数改为消费主线程快照 sonar_points_snapshot_(由 rebuildAndPublishMaps 在 accum_mutex_
    //   下从 sonar_points_ 拷得), 不再直接读 I/O 线程正在写的 sonar_points_ -> 无数据竞争。逐点数学不变。
    static bool first_debug = true;
    if (first_debug && !sonar_points_snapshot_.empty()) {
        float original_angle = sonar_points_snapshot_[0].angle * RAD_TO_DEG;
        // ROS_WARN("角度处理: 原始角度=%.1f°, 应用旋转=%.1f°, 最终角度=%.1f°",
        //         original_angle, rotation_angle_deg,
        //         original_angle + rotation_angle_deg);
        first_debug = false;
    }

    
    // 调试：输出前几个点的变换信息
    static int debug_point_count = 0;
    static const int MAX_DEBUG_POINTS = 10;

    for (const auto& sonar_point : sonar_points_snapshot_) {   // [方案A] 消费快照, 非 I/O 线程正写的 sonar_points_
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
                
                // ROS_WARN("方向验证[%d]: 坐标(%.3f,%.3f), 角度=%.1f°, 方向=%s",
                //         dir_check_count, point.x, point.y, angle, direction);
                dir_check_count++;
            }
                        
            // 调试输出
            if (debug_point_count < MAX_DEBUG_POINTS) {
                // 计算旋转后角度
                float rotated_angle = atan2(point.y, point.x) * RAD_TO_DEG;
                
                // ROS_ERROR("【关键验证】点云点 %d:", debug_point_count);
                // ROS_ERROR("  输入: 角度=%.1f°, 距离=%.3fm", original_angle_deg, range);
                // ROS_ERROR("  原始坐标: (%.4f, %.4f)", xx, yy);
                // ROS_ERROR("  旋转后坐标: (%.4f, %.4f), 角度=%.1f°", point.x, point.y, rotated_angle);
                // ROS_ERROR("  旋转参数: cos_theta=%.3f, sin_theta=%.3f (%.1f°)",
                //         cos_theta, sin_theta, rotation_angle_deg);

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
            // ROS_DEBUG("发布方向标记：-45°旋转，前方在45°方向");
        } else if (rotation_angle_deg == -90.0f) {
            // -90°旋转：前方在Y+方向
            end.x = 0; end.y = 2.0; end.z = 0;
            // ROS_DEBUG("发布方向标记：-90°旋转，前方在Y+方向");
        } else {
            // 其他旋转角度
            float rad = -rotation_angle_rad;  // 注意负号
            end.x = 2.0 * cosf(rad);
            end.y = 2.0 * sinf(rad);
            end.z = 0;
            // ROS_DEBUG("发布方向标记：%.1f°旋转", rotation_angle_deg);
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

        // ROS_DEBUG("发布方向标记：红色=前方(Y+)，绿色=左侧(-X)");
    }
}

// ======================== 点云滤波 ========================
void MultiPositionSonarMapper::filterPointCloud() {
    if (pointcloud_->empty()) return;
    *filtered_pointcloud_ = *pointcloud_;
}
 

void MultiPositionSonarMapper::createOccupancyGrid() {
    if (pointcloud_->empty()) return;
    
    // 1. 动态计算当前点云的实际范围
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    
    for (const auto& p : pointcloud_->points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }
    
    // 2. 扩展边界，留出余量
    float margin = 5.0f;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;
    
    // 3. 计算新的地图尺寸
    float new_width = max_x - min_x;
    float new_height = max_y - min_y;
    
    int new_map_width = static_cast<int>(new_width / map_resolution_) + 100;
    int new_map_height = static_cast<int>(new_height / map_resolution_) + 100;
    
    // 4. 如果范围变化，重新初始化地图
    bool need_resize = false;
    
    if (new_map_width > map_width_ || new_map_height > map_height_) {
        need_resize = true;
        // ROS_INFO("需要扩展地图: 当前(%d,%d) → 需要(%d,%d)",
        //          map_width_, map_height_, new_map_width, new_map_height);
    }
    
    if (min_x < map_origin_x_ || min_y < map_origin_y_) {
        need_resize = true;
        // ROS_INFO("需要移动原点: 当前(%.1f,%.1f) → 需要(%.1f,%.1f)",
        //          map_origin_x_, map_origin_y_, min_x, min_y);
    }
    
    if (need_resize) {
        // 重新初始化地图
        map_width_ = new_map_width;
        map_height_ = new_map_height;
        map_origin_x_ = min_x;
        map_origin_y_ = min_y;
        
        occupancy_map_.info.width = map_width_;
        occupancy_map_.info.height = map_height_;
        occupancy_map_.info.origin.position.x = map_origin_x_;
        occupancy_map_.info.origin.position.y = map_origin_y_;
        occupancy_map_.data.resize(map_width_ * map_height_, -1);

        // ROS_INFO("地图已动态扩展: %dx%d, 原点(%.1f,%.1f)",
        //          map_width_, map_height_, map_origin_x_, map_origin_y_);
    }
    
    // 5. 填充地图（现在所有点都能落在范围内）
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
    for (const auto& point : pointcloud_->points) {
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
        
        // 现在应该都能在范围内，但为了安全仍做检查
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            int index = map_y * map_width_ + map_x;
            int occ = static_cast<int>(point.intensity * 100.0f / 255.0f);
            if (occ > occupancy_map_.data[index]) {
                occupancy_map_.data[index] = occ;
            }
        }
    }
    
    occupancy_map_.header.stamp = ros::Time::now();
}

// ======================== 创建代价地图 ========================
void MultiPositionSonarMapper::createCostMap() {
    std::fill(cost_map_.data.begin(), cost_map_.data.end(), -1);
    cost_map_.header.stamp = ros::Time::now();
    
    if (filtered_pointcloud_->empty()) {
        return;
    }
    
    // 直接处理所有点，不做轮廓提取
    for (const auto& point : filtered_pointcloud_->points) {
        int costmap_x = static_cast<int>((point.x - costmap_origin_x_) / costmap_resolution_);
        int costmap_y = static_cast<int>((point.y - costmap_origin_y_) / costmap_resolution_);
        
        if (costmap_x >= 0 && costmap_x < costmap_width_ && 
            costmap_y >= 0 && costmap_y < costmap_height_) {
            
            int index = costmap_y * costmap_width_ + costmap_x;
            int cost_value = static_cast<int>(point.intensity * 100.0f / 255.0f);
            cost_value = std::min(100, std::max(0, cost_value));
            
            if (cost_value > cost_map_.data[index]) {
                cost_map_.data[index] = cost_value;
            }
        }
    }
}


// ======================== 发布地图数据 ========================
void MultiPositionSonarMapper::publishMapData() {
    ros::Time current_time = ros::Time::now();
    // 添加追踪1记录发布次数和阶段
    static int publish_count = 0;
    static int last_phase_step = -1;
    
    if (phase1_step_ != last_phase_step) {
        last_phase_step = phase1_step_;
        // ROS_WARN("=== publishMapData阶段变化: %d ===", phase1_step_);
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
    

        // 2. base_link -> sonar_frame（硬件安装变换）
        // 假设声纳安装在机器人前方0.1米，高度0.05米
        tf::Transform base_to_sonar;
        base_to_sonar.setOrigin(tf::Vector3(0.1, 0, 0.05));
        base_to_sonar.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_to_sonar, current_time, "base_link", "sonar_frame"));
            
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

        // ROS_DEBUG_THROTTLE(2.0, "发布点云: %lu点, 坐标系: map", pointcloud_->size());
    }


        // 发布滤波后的点云
    if (!filtered_pointcloud_->empty()) {
        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*filtered_pointcloud_, filtered_cloud_msg);
        filtered_cloud_msg.header.stamp = current_time;
        filtered_cloud_msg.header.frame_id = "map";
        filtered_pointcloud_pub_.publish(filtered_cloud_msg);
    }

    static std::string last_map_frame_id = "";
    if (occupancy_map_.header.frame_id != last_map_frame_id) {
        last_map_frame_id = occupancy_map_.header.frame_id;
        // ROS_WARN("地图坐标系变化: %s", last_map_frame_id.c_str());
    }
    
    // 发布占据栅格地图（在map中）
    if (!occupancy_map_.data.empty()) {
        occupancy_map_.header.stamp = current_time;
        occupancy_map_.header.frame_id = "map";
        occupancy_pub_.publish(occupancy_map_);
        //同时发布到标准话题
        standard_map_pub_.publish(occupancy_map_);  // 标准话题/map
        // ROS_DEBUG_THROTTLE(2.0, "发布地图: %dx%d",
        //                   occupancy_map_.info.width, occupancy_map_.info.height);
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
 
void MultiPositionSonarMapper::publishTF() {
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

void MultiPositionSonarMapper::startScanning() {
    if (is_scanning_.load()) {
        ROS_WARN("扫描已经在进行中");
        return;
    }
    
    if (!sonar_serial_) {
        ROS_ERROR("串口未初始化，无法开始扫描");
        return;
    }

    // 声呐发射前守门: 确认泵关+车停并 settle(覆盖所有 startScanning 入口)。此刻上一 scan_thread_ 已 join。
    // 守门经 pump_gate_queue_ 专用队列 callAvailable 观测 /cmd_vel(只派发 onCmdVel、绝不重入服务回调), 只观测不发指令。见 waitPumpOffAndSettle。
    waitPumpOffAndSettle();

    stop_requested_.store(false);
    is_scanning_.store(true);
    full_scan_complete_.store(false);
    
    // 启动扫描线程
    scan_thread_ = boost::thread(&MultiPositionSonarMapper::scanningThread, this);
    
    ROS_INFO("开始实时扫描");
}

// ======================== 停止扫描 ========================

void MultiPositionSonarMapper::stopScanning() {
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

bool MultiPositionSonarMapper::saveScanData(const std::string& filename) {
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

bool MultiPositionSonarMapper::saveCostMap(const std::string& filename) {
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
 
void MultiPositionSonarMapper::clearScanData() {
    ROS_INFO("开始清空所有数据...");
    
    // 清空基础数据([方案A] 累计缓冲清空受 accum_mutex_ 保护, 防与 I/O 线程写竞争)
    {
        std::lock_guard<std::mutex> lk(accum_mutex_);
        sonar_points_.clear();
        sonar_points_.shrink_to_fit();  // 释放内存
    }
    
    // 强制释放点云内存
    pointcloud_->clear();
    pcl::PointCloud<pcl::PointXYZI> empty_cloud;
    pointcloud_->swap(empty_cloud);  // 交换后原内存释放
    pointcloud_.reset(new PointCloudI());  // 重新创建
    
    filtered_pointcloud_->clear();
    pcl::PointCloud<pcl::PointXYZI> empty_filtered;
    filtered_pointcloud_->swap(empty_filtered);
    filtered_pointcloud_.reset(new PointCloudI());
    
    raw_points_buffer_.clear();
    raw_points_buffer_.shrink_to_fit();
    
    // 清空扇区数据
    sector_data_.clear();
    sector_data_.shrink_to_fit();
    
    // 重置地图数据
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    std::fill(cost_map_.data.begin(), cost_map_.data.end(), -1);
    
    // 重置代价单元
    cost_cells_.clear();
    cost_cells_.resize(costmap_width_ * costmap_height_);
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
    
    // 清空子地图（如果有）
    phase1_submaps_.clear();
    phase_multi_submaps_.clear();
    
    ROS_INFO("数据清空完成（内存已释放）");
}
// ======================== CRC16计算 ========================

uint16_t MultiPositionSonarMapper::calculateCRC16(const uint8_t* data, size_t length) {
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

void MultiPositionSonarMapper::logToCSV(const SonarPoint& point) {
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
bool MultiPositionSonarMapper::shouldGenerateDenseCloud() const {
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


void MultiPositionSonarMapper::createEnhancedOccupancyGrid() {
    // [方案A 警示·死函数] 本函数当前全仓无调用者(死代码)。若日后复用务必注意: 下方 2130+ 直接遍历
    //   sonar_points_ 是无锁访问, 而 sonar_points_ 现由串口 I/O 线程在 accum_mutex_ 下写入 ->
    //   直读活缓冲会与 I/O 写并发 = std::vector 数据竞争/UB。复用前须比照 rebuildAndPublishMaps:
    //   先在 accum_mutex_ 下把 sonar_points_ 拷入 sonar_points_snapshot_, 再遍历快照, 切勿直读活缓冲。
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


void MultiPositionSonarMapper::saveCurrentPhase1Submap() {
    if (phase1_submaps_.size() >= 10) {
        ROS_WARN("已经有%lu个子地图，水池建图最多需要2个位置", phase1_submaps_.size());
        return;
    }

    // ========== 子地图关键点检查 ==========

    if (!pointcloud_->empty()) {
        pcl::PointXYZI min_pt, max_pt;
        pcl::getMinMax3D(*pointcloud_, min_pt, max_pt);
        // ROS_WARN("[点云范围] 位置%d: X[%.3f, %.3f], Y[%.3f, %.3f]",
        //          phase1_current_position_, min_pt.x, max_pt.x, min_pt.y, max_pt.y);
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
        
        // ROS_WARN("  边界框: X[%.3f, %.3f] (宽度:%.3fm), Y[%.3f, %.3f] (宽度:%.3fm)",
        //         min_x, max_x, max_x - min_x,
        //         min_y, max_y, max_y - min_y);
    }
    
    int submap_idx = static_cast<int>(phase1_submaps_.size() + 1);
    // ROS_WARN(">>>> 保存子地图%d (位置%d):", submap_idx, phase1_current_position_);

    
    if (!pointcloud_->empty()) {
        int point_count = static_cast<int>(pointcloud_->size());
        
        // 检查前3个点的坐标和角度
        // ROS_WARN("  前3个点云点(转换后):");
        // for (int i = 0; i < std::min(3, point_count); i++) {
        //     const auto& p = pointcloud_->points[i];
        //     float angle = atan2(p.y, p.x) * RAD_TO_DEG;
        //     float dist = sqrt(p.x*p.x + p.y*p.y);
        //     ROS_WARN("    点%d: (%.4f, %.4f), 距离=%.3f, 角度=%.1f°",
        //             i, p.x, p.y, dist, angle);
        // }
        
        
        // 快速计算质心
        float sum_x = 0, sum_y = 0;
        for (const auto& p : pointcloud_->points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        float center_x = sum_x / pointcloud_->size();
        float center_y = sum_y / pointcloud_->size();
        // ROS_WARN("  点云质心: (%.3f, %.3f)", center_x, center_y);
        
        // 检查角度分布
        int front = 0, right = 0, back = 0, left = 0;
        for (const auto& p : pointcloud_->points) {
            float angle = atan2(p.y, p.x) * RAD_TO_DEG;
            if (angle >= -45 && angle < 45) front++;
            else if (angle >= 45 && angle < 135) right++;
            else if (angle >= 135 || angle < -135) back++;
            else left++;
        }
        // ROS_WARN("  方向分布: 前%d, 右%d, 后%d, 左%d", front, right, back, left);
        
        // 检查点云统计
        int high_intensity = 0, medium_intensity = 0, low_intensity = 0;
        for (const auto& p : pointcloud_->points) {
            if (p.intensity >= 160.0f) high_intensity++;
            else if (p.intensity >= 140.0f) medium_intensity++;
            else low_intensity++;
        }
        // ROS_WARN("  强度分布: 高≥160=%d, 中≥140=%d, 低=%d",
        //         high_intensity, medium_intensity, low_intensity);
    } else {
        // ROS_WARN("  警告：点云为空！");
    }
    // ========== 子地图关键点检查结束 ==========
    
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
        
        // ROS_WARN("子地图%d质心记录: (%.3f, %.3f)",
        //         submap.index, submap.center_x, submap.center_y);
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

geometry_msgs::Pose2D MultiPositionSonarMapper::getCurrentRobotPose() {
    geometry_msgs::Pose2D pose;
    // 简化实现：从TF获取或返回默认值
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;
    
    // 如果有里程计，可以在这里获取真实位姿
    // 暂时使用简化版
    
    return pose;
}
 
 
//发布机器人位置的TF标记
void MultiPositionSonarMapper::publishRobotPositionMarkers() {
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

    // ROS_INFO("已发布调试标记到 /phase1/debug_markers");
}

void MultiPositionSonarMapper::publishPhase1Status(const std::string& status) {
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
void MultiPositionSonarMapper::startPhase1Mapping(int max_submaps) {
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


void MultiPositionSonarMapper::stopPhase1Mapping() {
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

std::string MultiPositionSonarMapper::getPhase1StatusString() const {
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

// void MultiPositionSonarMapper::printPhase1DebugInfo() {
//     ROS_INFO("=== Phase1 调试信息 ===");
//     ROS_INFO("启用状态: %s", phase1_enabled_ ? "true" : "false");
//     ROS_INFO("当前步骤: %d", phase1_step_);
//     ROS_INFO("扫描状态: %s", is_scanning_.load() ? "进行中" : "停止");
//     ROS_INFO("子地图数量: %d", phase1_submap_count_);
//
//     if (phase1_step_ > 0) {
//         ros::Duration elapsed = ros::Time::now() - phase1_step_start_time_;
//         ROS_INFO("当前步骤已进行: %.1f 秒", elapsed.toSec());
//     }
//
//     ROS_INFO("扫描时长: %.1f 秒", phase1_scan_duration_);
//     ROS_INFO("等待时长: %.1f 秒", phase1_wait_duration_);
//     ROS_INFO("移动距离: %.2f 米", phase1_move_distance_);
//     ROS_INFO("=========================");
// }

 
 
bool MultiPositionSonarMapper::getCurrentRobotPoseInMap(geometry_msgs::Pose2D& pose) {
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

 
// ======================== 运行级水泵闩锁 ========================

void MultiPositionSonarMapper::openPumpLatch() {
    // 整段沿边任务起始(第一次扫描/移动前)置闩锁 ON, move_controller 据此跨段持泵(含扫描停顿)。
    // 幂等: 已开则不重发。0.4s 让 latched 话题送达 + move_controller 空闲定时器走一拍, 首次静止扫描即持泵。
    if (pump_latch_published_open_) return;
    std_msgs::Bool m; m.data = true;
    pump_enable_pub_.publish(m);
    pump_latch_published_open_ = true;
    ros::Duration(0.4).sleep();
    ROS_INFO("pump latch OPEN (run start)");
}

void MultiPositionSonarMapper::closePumpLatch() {
    // 任务正常结束/中止时撤闩 OFF; 此刻机器人已静止(扫描间/后), 撤闩后 move_controller 空闲定时器即关泵 → "先停后关泵"。
    // 幂等: 未开则不发。
    if (!pump_latch_published_open_) return;
    std_msgs::Bool m; m.data = false;
    pump_enable_pub_.publish(m);
    pump_latch_published_open_ = false;
    ROS_INFO("pump latch CLOSED (run end)");
}

// ======================== 扫描前"泵关+转停"守门 ========================

void MultiPositionSonarMapper::onCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    // 只读观测: 缓存最近 /cmd_vel 分量 + 到达时刻(Twist 无 header)。不发指令、不改运动状态。
    last_cmd_lin_x_ = msg->linear.x;
    last_cmd_lin_y_ = msg->linear.y;   // 泵占空(0~1); 扫描前须归零
    last_cmd_ang_z_ = msg->angular.z;
    last_cmd_stamp_ = ros::Time::now();
}

void MultiPositionSonarMapper::spinPumpGateQueue() {
    // 主循环每拍派发守门专用队列(仅 onCmdVel), 令 last_cmd_* 持续新鲜、守门入口无陈旧积压。
    // 守门运行时主循环阻塞(单线程同栈), 那期间改由守门自身 callAvailable 派发, 二者不并发。
    pump_gate_queue_.callAvailable(ros::WallDuration(0));
}

void MultiPositionSonarMapper::logPumpGate(int idx, bool confirmed, double elapsed_s, double age_s,
                                           double lin_y, double lin_x, double ang_z, const char* note) {
    // 守门裁决落盘(CLAUDE.md: 数据一律写文件并标注有效性)。尤其"报错仍扫"策略下, 本 CSV 是事后判定
    // "哪些停点是泵未确认关就扫的"的唯一依据(confirmed=0 且 note=TIMEOUT... 即泵未确认)。
    std::string path = session_dir_ + "pump_gate_log.csv";   // session_dir_ 已以 / 结尾, 不再前置 /
    std::ofstream f(path.c_str(), std::ios::app | std::ios::ate);  // ate: open 即定位末尾, tellp 反映真实文件大小
    if (!f.is_open()) { ROS_WARN("[pump-guard] cannot open %s for gate log", path.c_str()); return; }
    if (f.tellp() == std::streampos(0)) {   // 表头按"文件为空"判(不依赖 idx==0; 即使首次 open 失败也不会缺表头)
        f << "gate_idx,confirmed,elapsed_s,last_cmd_age_s,last_lin_y_pump,last_lin_x,last_ang_z,scanned_anyway,note\n";
    }
    f << idx << "," << (confirmed ? 1 : 0) << ","
      << std::fixed << std::setprecision(3) << elapsed_s << "," << age_s << ","
      << lin_y << "," << lin_x << "," << ang_z << ",1," << note << "\n";
    f.close();
}

bool MultiPositionSonarMapper::waitPumpOffAndSettle() {
    // 任务要求(用户确认): 声呐扫描前须泵关且车停。区分 CLAUDE.md 宪法红线——宪法红线是"声呐绝不在空气中发射(烧换能器)",
    //   由入水判定/操作规程把关; "扫描时关泵"是本任务作业要求, 非宪法条款。
    // move_controller 已"行走开泵/停车关泵/剩0.3m提前关"; 车停 => 泵应归零。此处观测 /cmd_vel 显式确认后再 settle,
    // 让电机/泵物理转停到位才交给声呐线程。只读只观测, 不发任何指令、不碰 move_controller/驱动。
    // 重入/线程安全: 观测挂 pump_gate_queue_ 专用队列, 守门用 callAvailable 只派发 onCmdVel, 绝不重入本节点服务回调
    //   (否则守门期挂起的 /start_phase1_mapping 会嵌套再起扫描线程双写声呐串口, /stop_* 会被吞成孤儿扫描线程持续发射)。
    //   调用时上一 scan_thread_ 已 join, 读写 last_cmd_* 均在主线程, 无真并发。
    const int gate_idx = pump_gate_idx_++;
    if (!pump_guard_enable_) {
        logPumpGate(gate_idx, false, 0.0, -1.0, last_cmd_lin_y_, last_cmd_lin_x_, last_cmd_ang_z_, "guard_disabled");  // confirmed=0: 守门关闭=泵未经验证(note 区分于真超时), 免下游"confirmed==1 即已验证"误判
        return true;  // 守门关闭(调参/离线): 直接放行
    }

    ros::Time gate_start = ros::Time::now();
    ros::Rate poll(20.0);                 // 20Hz 轮询
    bool confirmed = false;
    ros::Time stable_since;               // 条件连续满足起点
    bool stable_running = false;
    double age = 999.0, lin_y = last_cmd_lin_y_, lin_x = last_cmd_lin_x_, ang_z = last_cmd_ang_z_;  // 末次观测快照, 循环后落盘用

    while (ros::ok() && (ros::Time::now() - gate_start).toSec() < pump_guard_timeout_sec_) {
        pump_gate_queue_.callAvailable(ros::WallDuration(0));  // 只派发 onCmdVel(专用队列, 不重入服务回调), 刷新 last_cmd_*

        age   = (ros::Time::now() - last_cmd_stamp_).toSec();
        lin_y = last_cmd_lin_y_; lin_x = last_cmd_lin_x_; ang_z = last_cmd_ang_z_;
        bool fresh    = (age < pump_cmd_fresh_sec_);   // /cmd_vel 新鲜(驱动/控制器在发); fresh<stable 保证死掉的发布者不会假确认
        bool pump_off = (std::fabs(lin_y) <= pump_off_thresh_);
        bool stopped  = (std::fabs(lin_x) <= pump_stopped_thresh_) &&
                        (std::fabs(ang_z) <= pump_stopped_thresh_);
        bool ok_now = fresh && pump_off && stopped;

        if (ok_now) {
            if (!stable_running) { stable_running = true; stable_since = ros::Time::now(); }
            if ((ros::Time::now() - stable_since).toSec() >= pump_guard_stable_sec_) {
                confirmed = true;
                break;
            }
        } else {
            stable_running = false;       // 条件中断, 重新计时
        }
        poll.sleep();
    }

    double elapsed = (ros::Time::now() - gate_start).toSec();
    if (confirmed) {
        ROS_INFO("[pump-guard] gate#%d pump-off & stopped CONFIRMED in %.1fs; settling %.1fs before sonar",
                 gate_idx, elapsed, pump_settle_sec_);
        logPumpGate(gate_idx, true, elapsed, age, lin_y, lin_x, ang_z, "confirmed");
    } else {
        ROS_ERROR("[pump-guard] gate#%d could NOT confirm pump-off/stopped in %.1fs (age=%.2f lin_y=%.3f lin_x=%.3f ang_z=%.3f) -- per user policy SCANNING ANYWAY; VERIFY PUMP OFF (sonar should not fire with pump on)",
                  gate_idx, pump_guard_timeout_sec_, age, lin_y, lin_x, ang_z);
        logPumpGate(gate_idx, false, elapsed, age, lin_y, lin_x, ang_z, "TIMEOUT_scanned_pump_UNCONFIRMED");
    }

    // 无论确认与否都物理沉降后返回(让电机/泵转停到位)。此 sleep 在 scan 计时(scan_duration)前, 不吃扫描时间。
    ros::Duration(pump_settle_sec_).sleep();
    return confirmed;
}

void MultiPositionSonarMapper::publishSessionDone() {
    // 采集全部停点完成后调用(仅正常/超时完成路径, 不在中止/失败路径); 发本次会话目录触发离线拼图。
    // enable_stitch_=false 时不发, 行为同接入前。session_dir_ 在构造期已建, 此刻 maps/odom 已全部落盘+flush。
    if (!enable_stitch_) return;
    std_msgs::String m;
    m.data = session_dir_;
    session_done_pub_.publish(m);
    ROS_INFO("[%s] scan complete, published /sonar_scan/session_done = %s",
             MULTIMAP_VERSION, session_dir_.c_str());
}

// ======================== 多位置建图主函数  ========================

void MultiPositionSonarMapper::handlePhaseMultiMapping() {
    if (!phase_multi_enabled_) return;
    ros::Time now = ros::Time::now();
    // 步骤0：初始化
    if (phase_multi_step_ == 0 && is_scanning_.load()) {
        ROS_INFO("==============================================");
        ROS_INFO("=== 多位置建图启动 (共 %d 个位置) ===", phase_multi_num_positions_);
        ROS_INFO("==============================================");

        // 去闩锁: 直行扫描不再整段持泵(整段持泵会令扫描停顿期泵仍在转, 违反"声呐扫描时泵必须关")。
        // 泵回落 move_controller 既有单段策略"行走开泵 / 停车关泵 / 剩0.3m提前关"; 每次移动前 stopScanning、
        // move_straight 阻塞返回后才 startScanning => "正在扫描 <=> 车已停" => 扫描(车停)时泵自动关。
        // (原此处 openPumpLatch() 已删。) 其余各 closePumpLatch() 撤闩点全部保留: pump_latch_published_open_
        // 恒 false 使其幂等 no-op, 作为"中止/结束必撤闩"不变量的防御, 行为不变。

        phase_multi_step_ = 1;
        phase_multi_step_start_time_ = now;
        phase_multi_current_position_ = 1;
        circles_completed_in_position_ = 0;
        phase_multi_submaps_.clear();

        // ==========  强制每位置只扫一圈 ==========
        phase_multi_circles_per_position_ = 1;
        
        scan_start_angle_ = 0;
        full_scan_complete_.store(false);
        
        publishPhaseMultiStatus("位置1建图中");
        ROS_INFO("位置1: 开始建图，需要完成 %d 圈", phase_multi_circles_per_position_);
        return;
    }

    const float TIMEOUT_PER_POSITION = phase_multi_scan_duration_;
    
    // 动态状态机，支持最多50个位置
    int max_steps = phase_multi_num_positions_ * 2;
    
    // 建图步骤：奇数步骤 (1, 3, 5, 7, ...)
    if (phase_multi_step_ % 2 == 1 && phase_multi_step_ < max_steps) {
        int current_pos = (phase_multi_step_ + 1) / 2;
        ros::Duration elapsed = now - phase_multi_step_start_time_;
        
        // 检查是否完成一圈扫描
        if (full_scan_complete_.load()) {
            ros::Duration(0.2).sleep();  // 等待200ms，让最后一帧数据处理完
            full_scan_complete_.store(false);
            circles_completed_in_position_++;
            
            ROS_INFO("【位置%d】完成第%d圈扫描 (需要 %d 圈)", 
                    current_pos, circles_completed_in_position_, 
                    phase_multi_circles_per_position_);
            
            if (circles_completed_in_position_ >= phase_multi_circles_per_position_) {
                ROS_INFO("=== 位置%d 建图完成 ===", current_pos);
                
                // 保存当前子地图
                saveCurrentPhaseMultiSubmap();
                
                // 检查是否所有位置都已建图
                if (current_pos >= phase_multi_num_positions_) {
                    ROS_INFO("==============================================");
                    ROS_INFO("所有 %d 个位置建图完成", phase_multi_num_positions_);
                    
                    if (is_scanning_.load()) {
                        stopScanning();
                    }
                    closePumpLatch();   // 任务正常结束: 机器人已静止, 撤闩关泵
                    publishSessionDone();  // 采集全完成 → 发会话目录触发离线拼图(enable_stitch_ 门控)

                    // 根据位置数量决定是否自动合并
                    if (phase_multi_num_positions_ <= 10) {
                        ROS_INFO("位置数量 ≤ 10，自动进行地图合并");
                        phase_multi_step_ = 100;
                        phase_multi_step_start_time_ = now;
                        
                        publishPhaseMultiStatus("正在合并所有子地图");
                        // mergeMultipleSubmaps();
                        ROS_INFO("采集完成，跳过自动合并");
                    } else {
                        ROS_INFO("==============================================");
                        ROS_INFO("⚠️ 位置数量 > 10，跳过自动合并");
                        ROS_INFO("   已保存 %lu 个子地图到文件", phase_multi_submaps_.size());
                        ROS_INFO("   后续可使用其他工具进行地图合成");
                        ROS_INFO("==============================================");
                        phase_multi_step_ = 200;
                        phase_multi_step_start_time_ = now;
                        publishPhaseMultiStatus("采集完成，等待手动合并");
                    }
                    return;
                } else {
                    // 进入等待移动步骤
                    phase_multi_step_++;
                    phase_multi_step_start_time_ = now;
                    publishPhaseMultiStatus("等待移动到位置" + std::to_string(current_pos + 1));
                    
                    ROS_INFO("准备移动到位置 %d（距离 %.2f 米）", 
                            current_pos + 1, phase_multi_move_distance_);
                    
                    if (is_scanning_.load()) {
                        stopScanning();
                    }
                    return;
                }
            }
        }
        
        // 超时检查
        if (elapsed.toSec() >= TIMEOUT_PER_POSITION) {
            ROS_WARN("位置%d 扫描超时 (%.1f秒)，强制停止", current_pos, elapsed.toSec());
            saveCurrentPhaseMultiSubmap();
            
            if (current_pos >= phase_multi_num_positions_) {
                if (is_scanning_.load()) stopScanning();
                closePumpLatch();   // 任务结束(超时分支): 撤闩关泵
                publishSessionDone();  // 采集全完成(超时分支) → 发会话目录触发离线拼图

                if (phase_multi_num_positions_ <= 10) {
                    phase_multi_step_ = 100;
                    // mergeMultipleSubmaps();
                     ROS_INFO("采集完成，跳过自动合并");
                } else {
                    phase_multi_step_ = 200;
                    ROS_INFO("采集完成（超时），跳过自动合并");
                }
            } else {
                phase_multi_step_++;
                phase_multi_step_start_time_ = now;
            }
            return;
        }
        
        // 进度输出
        static int last_report_sec_multi = -1;
        int current_sec = static_cast<int>(elapsed.toSec());
        if (current_sec != last_report_sec_multi && current_sec % 10 == 0) {
            ROS_INFO("位置%d: 已扫描%d秒, 完成%d/%d圈, 当前角度: %d", 
                    current_pos, current_sec, 
                    circles_completed_in_position_, 
                    phase_multi_circles_per_position_,
                    current_angle_.load());
            last_report_sec_multi = current_sec;
        }
    }
    
    else if (phase_multi_step_ % 2 == 0 && phase_multi_step_ > 0 && phase_multi_step_ < max_steps) {
        int next_pos = phase_multi_step_ / 2 + 1;
        
        ROS_INFO("偶数步骤 %d 执行中，准备移动到位置 %d", phase_multi_step_, next_pos);
        
        // 检查服务是否可用
        if (!move_straight_client_.exists()) {
            ROS_ERROR("move_controller服务不可用，无法进行自动移动");
            ROS_ERROR("多位置建图终止！");
            phase_multi_enabled_ = false;
            phase_multi_step_ = 0;
            closePumpLatch();   // 中止: 撤闩关泵(否则闩锁会令空闲泵长开)
            publishPhaseMultiStatus("建图终止（move_controller不可用）");
            return;
        }

         // ==========  分析前方墙壁 ==========
   
        // 准备服务请求
        sonar_slam::MoveStraight srv;
        srv.request.distance = phase_multi_move_distance_;
        srv.request.timeout = std::max(phase_multi_wait_duration_, 
                                       fabs(phase_multi_move_distance_) * 15.0f);
 
     
        
        ROS_ERROR("调用移动服务: distance=%.2f米, timeout=%.1f秒", 
                srv.request.distance, srv.request.timeout);
        
        // 调用服务
        if (move_straight_client_.call(srv)) {
            if (srv.response.success) {
                ROS_INFO("移动成功: %s", srv.response.message.c_str());
                
                // 移动成功，开始下一位置建图
                ROS_INFO("移动完成，开始位置%d建图", next_pos);
                phase_multi_current_position_ = next_pos;
                circles_completed_in_position_ = 0;
                full_scan_complete_.store(false);
                scan_start_angle_ = 0;
                
                // 清空数据([方案A] 累计缓冲清空持 accum_mutex_)
                {
                    std::lock_guard<std::mutex> lk(accum_mutex_);
                    sonar_points_.clear();
                }
                pointcloud_->clear();
                filtered_pointcloud_->clear();
                
                // 重置扫描状态
                resetForNextPosition();
                
                // 开始扫描
                if (!is_scanning_.load()) {
                    startScanning();
                }
                
                phase_multi_step_++;
                phase_multi_step_start_time_ = ros::Time::now();
                
                std::string status = "位置" + std::to_string(next_pos) + "建图中";
                publishPhaseMultiStatus(status);
            } else {
                ROS_ERROR("移动失败: %s", srv.response.message.c_str());
                ROS_ERROR("多位置建图终止！");
                phase_multi_enabled_ = false;
                phase_multi_step_ = 0;
                closePumpLatch();   // 中止: 撤闩关泵
                publishPhaseMultiStatus("建图终止（移动失败）");
            }
        } else {
            ROS_ERROR("调用移动服务失败");
            ROS_ERROR("多位置建图终止！");
            phase_multi_enabled_ = false;
            phase_multi_step_ = 0;
            closePumpLatch();   // 中止: 撤闩关泵
            publishPhaseMultiStatus("建图终止（服务调用失败）");
        }

        
    }
    // 合并完成后的持续发布（≤10个位置时）
    else if (phase_multi_step_ == 100) {
        static ros::Time last_multi_publish = ros::Time(0);
        
        if ((now - last_multi_publish).toSec() > 0.5) {
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
            
            last_multi_publish = now;
        }
    }
    // 采集完成后的状态（>10个位置时，仅发布提示，不发布地图）
    else if (phase_multi_step_ == 200) {
        static ros::Time last_status_publish = ros::Time(0);
        
        if ((now - last_status_publish).toSec() > 30.0) {
            ROS_INFO("==============================================");
            ROS_INFO("多位置采集完成状态:");
            ROS_INFO("  总位置数: %d", phase_multi_num_positions_);
            ROS_INFO("  已保存子地图: %lu 个", phase_multi_submaps_.size());
            ROS_INFO("  子地图文件保存在: maps/phase_multi_submap_*");
            ROS_INFO("  提示: 使用外部工具进行地图合成");
            ROS_INFO("==============================================");
            last_status_publish = now;
        }
    }
}
 
// ======================== 保存多位置子地图 ========================
void MultiPositionSonarMapper::saveCurrentPhaseMultiSubmap() {
    // ROS_ERROR("=== saveCurrentPhaseMultiSubmap 开始，当前 pointcloud_ 大小: %lu ===",
    //           pointcloud_ ? pointcloud_->size() : 0);
   
    if (phase_multi_submaps_.size() >= phase_multi_num_positions_) {
        ROS_WARN("已经保存了 %lu 个子地图，达到目标位置数 %d", 
                phase_multi_submaps_.size(), phase_multi_num_positions_);
        return;
    }

    int submap_idx = static_cast<int>(phase_multi_submaps_.size() + 1);
    int current_pos = phase_multi_current_position_;
    
    ROS_INFO(">>>> 保存子地图 %d (位置%d) <<<<", submap_idx, current_pos);
    
    // 创建子地图对象
    Phase1SubMap submap;
    submap.index = submap_idx;
    submap.timestamp = ros::Time::now();
    
    // 获取机器人位姿（在锁外执行，不访问 pointcloud_）
    geometry_msgs::Pose2D robot_pose;
    
    if (current_pos == 1) {
        robot_pose.x = 0.0f;
        robot_pose.y = 0.0f;
        robot_pose.theta = 0.0f;
        ROS_INFO("  位置1: 固定原点 (0,0)");
    } else {
        try {
            tf::StampedTransform transform;
            if (tf_listener_.waitForTransform("odom", "base_link", 
                                             ros::Time(0), ros::Duration(0.5))) {
                tf_listener_.lookupTransform("odom", "base_link", 
                                            ros::Time(0), transform);
                
                robot_pose.x = transform.getOrigin().x();
                robot_pose.y = transform.getOrigin().y();
                robot_pose.theta = tf::getYaw(transform.getRotation());
                
                float distance = sqrt(robot_pose.x*robot_pose.x + robot_pose.y*robot_pose.y);
                ROS_INFO("  位置%d odom位姿: (%.3f, %.3f, %.1f°), 距离原点=%.3f米, 第%d个文件", 
                        current_pos, robot_pose.x, robot_pose.y, 
                        robot_pose.theta * RAD_TO_DEG, distance, submap_idx);
            } else {
                robot_pose.x = phase_multi_move_distance_ * (current_pos - 1);
                robot_pose.y = 0.0f;
                robot_pose.theta = 0.0f;
                ROS_WARN("  无法获取odom位姿，使用预设位移: %.3f米", robot_pose.x);
            }
        } catch (const std::exception& e) {
            robot_pose.x = phase_multi_move_distance_ * (current_pos - 1);
            robot_pose.y = 0.0f;
            robot_pose.theta = 0.0f;
            ROS_WARN("  odom查询异常，使用预设位移: %.3f米", robot_pose.x);
        }
    }
    
    submap.robot_pose = robot_pose;

    // [方案A] 落盘前强制从当前累计缓冲重建一次: I/O 线程已不再每帧重建 pointcloud_/occupancy_map_,
    //   若只靠主循环 ~2Hz 刷新, 落盘时刻可能落后最新累计帧。此处显式刷一次, 保证下方读到的
    //   pointcloud_ 与 occupancy_map_ 反映"落盘时刻的全部累计帧"(与现版逐帧重建后落盘的内容口径一致)。
    //   须在下方 pointcloud_mutex_ 锁块之外调用(rebuildAndPublishMaps 内部会各自短暂持有 accum_mutex_/
    //   pointcloud_mutex_ 后即释放), 避免嵌套。
    rebuildAndPublishMaps();

    // ========== 锁内：保存点云和地图，不拷贝 ==========
    {
        std::lock_guard<std::mutex> lock(pointcloud_mutex_);

        if (pointcloud_->empty()) {
            ROS_WARN("点云为空，跳过保存");
            return;
        }
        
        // 计算点云质心
        float sum_x = 0, sum_y = 0;
        for (const auto& p : pointcloud_->points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        submap.center_x = sum_x / pointcloud_->size();
        submap.center_y = sum_y / pointcloud_->size();
        
        ROS_INFO("  点云质心: (%.3f, %.3f)", submap.center_x, submap.center_y);
        ROS_INFO("  点云大小: %lu", pointcloud_->size());
        
        // 输出强度分布
        int high = 0, medium = 0, low = 0;
        for (const auto& p : pointcloud_->points) {
            if (p.intensity >= 160.0f) high++;
            else if (p.intensity >= 140.0f) medium++;
            else low++;
        }
        ROS_INFO("  强度分布: 高=%d, 中=%d, 低=%d", high, medium, low);
        
        // 保存点云到文件（直接使用 pointcloud_，不拷贝）
        std::string filename = "phase_multi_submap_" + std::to_string(submap_idx) + 
                              "_pos" + std::to_string(current_pos);
        std::string pcd_filename = getMapsDirectory() + filename + ".pcd";
        
        if (pcl::io::savePCDFileBinary(pcd_filename, *pointcloud_) == 0) {
            ROS_INFO("保存点云: %s (%lu 点)", pcd_filename.c_str(), pointcloud_->size());
        } else {
            ROS_ERROR("保存点云失败: %s", pcd_filename.c_str());
        }
        
        // 保存地图
        std::string pgm_filename = getMapsDirectory() + filename + ".pgm";
        saveMapAsPGM(pgm_filename, occupancy_map_);
        
        std::string yaml_filename = getMapsDirectory() + filename + ".yaml";
        saveMapAsYAML(yaml_filename, filename + ".pgm", occupancy_map_);

        
        
        // 保存子地图信息（不保存点云副本，只保存元数据）
        submap.cloud.reset();  // 空指针，不保存点云副本
        submap.map = occupancy_map_;
        
        phase_multi_submaps_.push_back(submap);
        
        // 保存完成后清空点云
        pointcloud_->clear();
        filtered_pointcloud_->clear();
        // 强制释放内存
        pcl::PointCloud<pcl::PointXYZI> empty;
        pointcloud_->swap(empty);
        pointcloud_.reset(new PointCloudI());
        scan_start_angle_ = 0;  // 重置起始角度为0
        angle_base_initialized_ = false; 
        ROS_INFO("位置 %d 保存完成，已清空点云释放内存", current_pos);
        
    }  // 锁释放
    
    // 保存子地图后，记录 odom 数据到日志
    if (odom_log_file_.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::tm* tm_info = std::localtime(&now_time_t);
        
        char time_buf[20];
        strftime(time_buf, sizeof(time_buf), "%M:%S", tm_info);
        
        std::string filename = "phase_multi_submap_" + std::to_string(submap_idx) + 
                              "_pos" + std::to_string(current_pos) +
                              "_dist" + std::to_string(robot_pose.x);
        
        odom_log_file_ << time_buf << "." << std::setfill('0') << std::setw(3) << now_ms.count() << ","
                       << submap_idx << ","
                       << current_pos << ","
                       << std::fixed << std::setprecision(3)
                       << robot_pose.x << ","
                       << robot_pose.y << ","
                       << robot_pose.theta * RAD_TO_DEG << ","
                       << filename << ".pcd\n";
        odom_log_file_.flush();
    }

    // 保存原始声呐数据
    std::string raw_filename = "position_" + std::to_string(current_pos) + "_raw";
    saveRawSonarData(raw_filename, current_pos);

    
    // 回环检测扫本次会话目录(智能子目录), 与本次写入的 maps 一致, 不混入历史运行
    std::string sonar_logs_dir = session_dir_;

    ROS_INFO("执行回环检测，目录: %s", sonar_logs_dir.c_str());
    
    auto start = std::chrono::steady_clock::now();
    
    bool result = sonar_return_home_file_detector::detectReturnHomeFromDirectory(sonar_logs_dir);
    
    auto end = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    ROS_INFO("回环检测结果: %s, 耗时: %.3f秒", 
             result ? "true (检测到回环)" : "false (未检测到)", 
             elapsed_ms / 1000.0);
    
    if (result) {
        check_loop_closure_ = result;  // 更新成员变量
        ROS_WARN("========================================");
        ROS_WARN("🎯 检测到回环！建图将在当前保存完成后终止");
        ROS_WARN("========================================");
    }
    // 发布可视化标记
    publishPhaseMultiSubmapMarkers();
    
    ROS_INFO("  子地图 %d 保存完成，共 %lu/%d 个", 
            submap_idx, phase_multi_submaps_.size(), phase_multi_num_positions_);
}

// ======================== 合并多个子地图 ========================
void MultiPositionSonarMapper::mergeMultipleSubmaps() {
    if (phase_multi_submaps_.size() < 2) {
        ROS_ERROR("需要至少2个子地图才能合并，当前有%lu个", phase_multi_submaps_.size());
        return;
    }

    ROS_INFO("==============================================");
    ROS_INFO("开始合并 %lu 个子地图", phase_multi_submaps_.size());
    ROS_INFO("==============================================");

    // 1. 创建合并点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new PointCloudI());
    merged_cloud->header.frame_id = "map";
    merged_cloud->is_dense = false;
    
    // 用于强强融合的网格累加器
    std::map<std::pair<int, int>, std::pair<float, float>> grid_accumulator;
    
    // 2. 存储每个位置的变换信息
    std::vector<geometry_msgs::Pose2D> map_poses;
    
    // 3. 处理每个子地图
    for (size_t i = 0; i < phase_multi_submaps_.size(); i++) {
        auto& submap = phase_multi_submaps_[i];

        if (!submap.cloud || submap.cloud->empty()) {
            std::string filename = getMapsDirectory() + "phase_multi_submap_" + 
                                   std::to_string(submap.index) + 
                                   "_pos" + std::to_string(i+1) + ".pcd";
            submap.cloud.reset(new PointCloudI());
            if (pcl::io::loadPCDFile(filename, *submap.cloud) == -1) {
                ROS_ERROR("无法加载点云文件: %s", filename.c_str());
                continue;
            }
            ROS_INFO("从文件加载点云: %s (%lu点)", filename.c_str(), submap.cloud->size());
        }
        
        float pose_x, pose_y, pose_theta;
        
        if (i == 0) {
            // 第一个位置固定在地图原点
            pose_x = 0.0f;
            pose_y = 0.0f;
            pose_theta = 0.0f;
            ROS_INFO("位置1: 地图坐标 (0.0, 0.0, 0.0°)");
        } else {
            // 其他位置使用累积位移
            // 策略：使用相对于第一个位置的odom位移
            const auto& first_pose = phase_multi_submaps_[0].robot_pose;
            const auto& current_pose = submap.robot_pose;
            
            // 计算相对于第一个位置的位移
            pose_x = current_pose.x - first_pose.x;
            pose_y = current_pose.y - first_pose.y;
            pose_theta = current_pose.theta - first_pose.theta;
            
            ROS_INFO("位置%d: odom位移 (%.3f, %.3f, %.1f°)", 
                    static_cast<int>(i+1), pose_x, pose_y, pose_theta * RAD_TO_DEG);
            
            // 可选：质心补偿（当位移较小时可能有用）
            if (fabs(pose_x) < 0.5f && fabs(pose_y) < 0.5f) {
                float center_diff_x = submap.center_x - phase_multi_submaps_[0].center_x;
                float center_diff_y = submap.center_y - phase_multi_submaps_[0].center_y;
                
                if (fabs(center_diff_x) > 0.5f || fabs(center_diff_y) > 0.5f) {
                    ROS_WARN("  位移过小但质心有较大变化，尝试质心补偿");
                    pose_x = center_diff_x * 0.5f;
                    pose_y = center_diff_y * 0.5f;
                }
            }
        }
        
        // 保存地图位姿用于可视化
        geometry_msgs::Pose2D map_pose;
        map_pose.x = pose_x;
        map_pose.y = pose_y;
        map_pose.theta = pose_theta;
        map_poses.push_back(map_pose);
        
        // 转换点云
        int points_added = 0;
        ROS_INFO("  处理子地图%d点云 (%lu点)...", static_cast<int>(i+1), submap.cloud->size());
        
        for (const auto& point : submap.cloud->points) {
            pcl::PointXYZI transformed_point;
            
            if (i == 0) {
                transformed_point.x = point.x;
                transformed_point.y = point.y;
            } else {
                // 先应用旋转（如果有）
                if (fabs(pose_theta) > 0.001f) {
                    float cos_theta = cosf(pose_theta);
                    float sin_theta = sinf(pose_theta);
                    
                    transformed_point.x = cos_theta * point.x - sin_theta * point.y + pose_x;
                    transformed_point.y = sin_theta * point.x + cos_theta * point.y + pose_y;
                } else {
                    transformed_point.x = point.x + pose_x;
                    transformed_point.y = point.y + pose_y;
                }
            }
            
            transformed_point.z = 0.0f;
            transformed_point.intensity = point.intensity;
            
            // 转换为网格坐标进行强强融合
            int map_x = static_cast<int>((transformed_point.x - map_origin_x_) / map_resolution_ + 0.5f);
            int map_y = static_cast<int>((transformed_point.y - map_origin_y_) / map_resolution_ + 0.5f);
            
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                std::pair<int, int> grid_key = std::make_pair(map_x, map_y);
                
                auto it = grid_accumulator.find(grid_key);
                if (it == grid_accumulator.end()) {
                    grid_accumulator[grid_key] = std::make_pair(transformed_point.intensity, 1.0f);
                    merged_cloud->points.push_back(transformed_point);
                } else {
                    // 强强融合：保留最大强度
                    if (transformed_point.intensity > it->second.first) {
                        it->second.first = transformed_point.intensity;
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
                    it->second.second += 1.0f;
                }
                
                points_added++;
            }
        }
        
        ROS_INFO("  子地图%d转换完成: %d点加入合并点云", static_cast<int>(i+1), points_added);
    }
    
    // 4. 生成合并地图
    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    *pointcloud_ = *merged_cloud;
    
    // 重置地图并填充
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    
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
            occupancy_value = std::min(100, occupancy_value + static_cast<int>(point_count * 5));
        }
        
        occupancy_map_.data[index] = occupancy_value;
    }
    
    // 5. 输出统计信息
    int occupied = 0, free = 0, unknown = 0;
    for (int val : occupancy_map_.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }
    
    ROS_INFO("==============================================");
    ROS_INFO("多位置地图合并完成:");
    ROS_INFO("  子地图数量: %lu", phase_multi_submaps_.size());
    ROS_INFO("  合并点云: %lu点", merged_cloud->size());
    ROS_INFO("  地图单元: 占据=%d, 空闲=%d, 未知=%d", occupied, free, unknown);
    ROS_INFO("==============================================");
    
    // 6. 发布结果
    occupancy_map_.header.stamp = ros::Time::now();
    occupancy_pub_.publish(occupancy_map_);
    
    if (!merged_cloud->empty()) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*merged_cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";
        pointcloud_pub_.publish(cloud_msg);
    }
    
    // 7. 保存最终地图
    saveStageMap("phase_multi_merged_" + std::to_string(phase_multi_submaps_.size()) + "positions");
    
    // 8. 发布位置标记
    publishPhaseMultiPositionMarkers(map_poses);
}


// ======================== 发布多位置标记 ========================
void MultiPositionSonarMapper::publishPhaseMultiSubmapMarkers() {
    visualization_msgs::MarkerArray marker_array;
    
    for (size_t i = 0; i < phase_multi_submaps_.size(); i++) {
        const auto& submap = phase_multi_submaps_[i];
        
        // 子地图位置标记
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "phase_multi_submaps";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0.0;  // 都在原点显示
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.2 * i;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        // 不同位置不同颜色
        float hue = static_cast<float>(i) / phase_multi_num_positions_;
        marker.color.r = sin(hue * 2 * M_PI);
        marker.color.g = sin((hue + 0.33) * 2 * M_PI);
        marker.color.b = sin((hue + 0.67) * 2 * M_PI);
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
        
        // 文字标记
        visualization_msgs::Marker text_marker = marker;
        text_marker.id = static_cast<int>(i + 100);
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        
        std::stringstream ss;
        ss << "位置" << (i+1) << "\n质心:(" 
           << std::fixed << std::setprecision(2) 
           << submap.center_x << "," << submap.center_y << ")";
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
    }
    
    phase_multi_submap_pub_.publish(marker_array);
}

void MultiPositionSonarMapper::publishPhaseMultiPositionMarkers(
    const std::vector<geometry_msgs::Pose2D>& map_poses) {
    
    visualization_msgs::MarkerArray marker_array;
    
    // 位置标记
    for (size_t i = 0; i < map_poses.size(); i++) {
        const auto& pose = map_poses[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "phase_multi_positions";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.x;
        marker.pose.position.y = pose.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        // 颜色渐变
        float hue = static_cast<float>(i) / map_poses.size();
        marker.color.r = sin(hue * 2 * M_PI);
        marker.color.g = sin((hue + 0.33) * 2 * M_PI);
        marker.color.b = sin((hue + 0.67) * 2 * M_PI);
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
        
        // 文字标记
        visualization_msgs::Marker text_marker = marker;
        text_marker.id = static_cast<int>(i + 100);
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z = 0.6;
        text_marker.scale.z = 0.25;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        
        std::stringstream ss;
        ss << "P" << (i+1) << "\n(" 
           << std::fixed << std::setprecision(2) 
           << pose.x << "," << pose.y << ")";
        text_marker.text = ss.str();
        
        marker_array.markers.push_back(text_marker);
    }
    
    // 连接线
    if (map_poses.size() >= 2) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "phase_multi_positions";
        line_marker.id = 200;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.scale.x = 0.05;
        line_marker.color.r = 1.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        
        for (const auto& pose : map_poses) {
            geometry_msgs::Point p;
            p.x = pose.x;
            p.y = pose.y;
            p.z = 0.15;
            line_marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_marker);
    }
    
    static ros::Publisher debug_pub = nh_.advertise<visualization_msgs::MarkerArray>(
        "/phase_multi/debug_markers", 1, true);
    debug_pub.publish(marker_array);
}

// ======================== 多位置建图状态管理 ========================
void MultiPositionSonarMapper::startPhaseMultiMapping(int num_positions) {
    if (phase_multi_enabled_) {
        ROS_WARN("多位置建图已在运行中 (当前步骤: %d)", phase_multi_step_);
        return;
    }
    
 
    ROS_INFO("==============================================");
    ROS_INFO("=== 手动启动多位置建图 (%d个位置) ===", num_positions);
    if (num_positions > 10) {
        ROS_WARN("⚠️ 注意：超过10个位置时，只采集数据，不自动合成地图");
        ROS_WARN("   所有子地图将保存到文件，后续可通过其他工具合成");
    }
    ROS_INFO("==============================================");
    
    // 停止当前扫描
    if (is_scanning_.load()) {
        ROS_INFO("停止现有扫描...");
        stopScanning();
        ros::Duration(0.5).sleep();
    }
    
    // 重置状态
    phase_multi_enabled_ = true;
    phase_multi_step_ = 0;
    phase_multi_num_positions_ = num_positions;
    phase_multi_current_position_ = 1;
    phase_multi_submaps_.clear();
    phase_multi_circles_per_position_ = 1;
    // 清空数据
    clearScanData();

    // 可选: 扫描前先找墙对齐(下水自启找侧壁), 默认关; 沿边+声呐入口可置真。找墙=标准 find_wall 服务(同 step3 独立行为)
    // 注意: 开启时本步会驱动机器人(旋转找墙 + 沿边 acquire_distance 米)后再扫描, 须 step3 找墙已现场验证、右超声映射已标定
    if (acquire_wall_first_) {
        ROS_WARN("[acquire] WILL DRIVE robot: rotate to find wall then follow %.2fm before scan", acquire_distance_);
        sonar_slam::MoveStraight fw;
        fw.request.distance = acquire_distance_;
        fw.request.timeout = 60.0f;
        if (!find_wall_client_.waitForExistence(ros::Duration(5.0)) ||
            !find_wall_client_.call(fw) || !fw.response.success) {
            ROS_ERROR("[acquire] find_wall failed -> abort task (align robot at wall or set phase1/acquire_wall_first=false)");
            closePumpLatch();   // 防御性撤闩(本路径闩锁尚未开, 幂等 no-op; 固化"中止必撤闩"不变量)
            phase_multi_enabled_ = false;
            phase_multi_step_ = 0;
            return;
        }
        ROS_INFO("[acquire] wall acquired, start scan loop");
    }

    // 启动扫描
    ROS_INFO("启动扫描线程...");
    startScanning();
    
    // 记录开始时间
    phase_multi_step_start_time_ = ros::Time::now();
    
    publishPhaseMultiStatus("准备开始");
    
    ROS_INFO("多位置建图已启动，共 %d 个位置", num_positions);
}
  
void MultiPositionSonarMapper::stopPhaseMultiMapping() {
    if (!phase_multi_enabled_) {
        ROS_WARN("多位置建图未启用");
        return;
    }
    
    ROS_INFO("=== 停止多位置建图 ===");
    ROS_INFO("当前步骤: %d, 已完成 %lu/%d 个位置", 
             phase_multi_step_, phase_multi_submaps_.size(), phase_multi_num_positions_);
    
    phase_multi_enabled_ = false;
    phase_multi_step_ = 0;

    if (is_scanning_.load()) {
        stopScanning();
    }
    closePumpLatch();   // 手动停止服务(/stop_phase_multi_mapping)也是一条任务结束路径: 撤闩关泵(否则闩锁令空闲泵长开); 幂等

    publishPhaseMultiStatus("已停止");
}

void MultiPositionSonarMapper::publishPhaseMultiStatus(const std::string& status) {
    std_msgs::String msg;
    
    if (status.empty()) {
        std::stringstream ss;
        ss << "PhaseMulti: ";
        if (phase_multi_step_ == 0) ss << "准备中";
        else if (phase_multi_step_ <= 19) {
            int current_pos = (phase_multi_step_ + 1) / 2;
            if (phase_multi_step_ % 2 == 1) {
                ss << "位置" << current_pos << "建图中";
            } else {
                ss << "等待移动到位置" << (current_pos + 1);
            }
        } else if (phase_multi_step_ == 100) {
            ss << "合并完成";
        }
        
        ss << " (" << phase_multi_submaps_.size() << "/" << phase_multi_num_positions_ << "位置)";
        msg.data = ss.str();
    } else {
        msg.data = "PhaseMulti: " + status;
    }
    
    phase_multi_status_pub_.publish(msg);
}

std::string MultiPositionSonarMapper::getPhaseMultiStatusString() const {
    std::stringstream ss;
    
    if (phase_multi_enabled_) {
        ss << "PhaseMulti: ";
        switch (phase_multi_step_) {
            case 0: ss << "准备中"; break;
            case 1: case 3: case 5: case 7: case 9:
            case 11: case 13: case 15: case 17: case 19:
                ss << "位置" << ((phase_multi_step_ + 1) / 2) << "建图中";
                break;
            case 2: case 4: case 6: case 8: case 10:
            case 12: case 14: case 16: case 18: case 20:
                ss << "等待移动到位置" << (phase_multi_step_ / 2 + 1);
                break;
            case 100: ss << "合并完成"; break;
            default: ss << "未知步骤 " << phase_multi_step_; break;
        }
        
        ss << " (" << phase_multi_submaps_.size() << "/" << phase_multi_num_positions_ << "位置)";
        
        if (phase_multi_step_ > 0 && phase_multi_step_ < 100) {
            ros::Duration elapsed = ros::Time::now() - phase_multi_step_start_time_;
            ss << " 已进行 " << std::fixed << std::setprecision(1) << elapsed.toSec() << "秒";
        }
    } else {
        ss << "PhaseMulti: 未启用";
    }
    
    return ss.str();
}

// ======================== 重置扫描状态函数实现 ========================
void MultiPositionSonarMapper::resetForNextPosition() {
    ROS_INFO("准备切换到下一个位置，重置扫描状态...");
    
    // 清空当前数据([方案A] 累计缓冲清空持 accum_mutex_)
    {
        std::lock_guard<std::mutex> lk(accum_mutex_);
        sonar_points_.clear();
    }
    // pointcloud_->clear();
    // filtered_pointcloud_->clear();
    
    // 重置地图数据（保持地图大小，但清空内容）
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    std::fill(cost_map_.data.begin(), cost_map_.data.end(), -1);
    
    // 重置代价单元
    for (auto& cell : cost_cells_) {
        cell = CostCell();
    }
    
    // 重置扫描统计
    total_scans_ = 0;
    valid_scans_ = 0;
    dropped_scans_ = 0;
    circles_completed_in_position_ = 0;
    
    // 重置扫描完成标志
    full_scan_complete_.store(false);
    
    // 重置扇区数据
    sector_data_.clear();
    
    // 重置原始数据缓冲区
    raw_points_buffer_.clear();
    raw_points_buffer_.resize(MAX_RAW_POINTS);
    
    // 重置解析状态
    parse_state_ = STATE_IDLE;
    current_message_.clear();
    rx_end_index_ = 0;
    rx_used_index_ = 0;
    
    
    angle_base_initialized_ = false;
    angle_base_ = -1;
    recent_angles_debug_.clear();
    scanned_angles_.clear();
    covered_angles_.clear();
 
    ROS_INFO("扫描状态已重置，准备位置 %d 的建图", phase_multi_current_position_);
}

// ======================== 检查是否所有位置都已建图完成 ========================
bool MultiPositionSonarMapper::isAllPositionsCompleted() const {
    return phase_multi_submaps_.size() >= phase_multi_num_positions_;
}

// ======================== 获取下一个要建图的位置索引 ========================
int MultiPositionSonarMapper::getNextPositionIndex() const {
    return phase_multi_current_position_ + 1;
}

std::string MultiPositionSonarMapper::getMapsDirectory() {
    // 智能会话目录: 落在本次运行 scan_<时间>/maps/ (session_dir_ 构造期已建, 防多次运行覆盖)
    std::string maps_dir = session_dir_ + "maps/";
    struct stat st;
    if (stat(maps_dir.c_str(), &st) != 0) {
        mkdir(maps_dir.c_str(), 0755);
    }
    return maps_dir;
}

std::string MultiPositionSonarMapper::getRawSonarDirectory() {
    // 智能会话目录: 落在本次运行 scan_<时间>/raw_sonar/
    std::string raw_dir = session_dir_ + "raw_sonar/";
    struct stat st;
    if (stat(raw_dir.c_str(), &st) != 0) {
        mkdir(raw_dir.c_str(), 0755);
    }
    return raw_dir;
}
  
bool MultiPositionSonarMapper::saveRawSonarData(const std::string& filename, int position) {
    try {
        // 创建目录(智能会话目录 scan_<时间>/raw_sonar/)
        std::string raw_dir = session_dir_ + "raw_sonar/";
        struct stat st;
        if (stat(raw_dir.c_str(), &st) != 0) {
            mkdir(raw_dir.c_str(), 0755);
        }

        std::string filepath = raw_dir + filename + ".csv";
        std::ofstream raw_file(filepath);
        
        if (!raw_file.is_open()) {
            ROS_WARN("无法打开原始数据文件: %s", filepath.c_str());
            return false;
        }
        
        // 写入表头
        raw_file << "angle_deg,point_index,distance_m,intensity\n";

        // [方案A] 累计缓冲改由 accum_mutex_ 保护。此处在锁内拷一份快照后即释放锁, 再在锁外写文件,
        //   避免持锁做磁盘 I/O 拖住串口 I/O 线程。读取时刻仍是本函数被调用时的 sonar_points_(与现版一致)。
        std::vector<SonarPoint> raw_snapshot;
        {
            std::lock_guard<std::mutex> lk(accum_mutex_);
            raw_snapshot = sonar_points_;
        }

        int total_points = 0;
        for (const auto& sonar_point : raw_snapshot) {
            float angle_deg = sonar_point.angle * RAD_TO_DEG;
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                raw_file << std::fixed << std::setprecision(6)
                         << angle_deg << ","
                         << i << ","
                         << sonar_point.ranges[i] << ","
                         << sonar_point.intensities[i] << "\n";
                total_points++;
            }
        }
        
        raw_file.close();
        ROS_INFO("保存原始声呐数据: %s (%d个点, 位置%d)", 
                 filepath.c_str(), total_points, position);
        return true;
        
    } catch (const std::exception& e) {
        ROS_WARN("保存原始声呐数据失败: %s (不影响主流程)", e.what());
        return false;
    } catch (...) {
        ROS_WARN("保存原始声呐数据失败: 未知错误 (不影响主流程)");
        return false;
    }
}

} // namespace sonar_slam


// 回调函数定义
 
bool startPhase1Callback(sonar_slam::StartPhase1::Request& req,
                         sonar_slam::StartPhase1::Response& res,
                         sonar_slam::MultiPositionSonarMapper* mapper) {
    ROS_INFO("Service called: /start_phase1_mapping with max_submaps=%d", req.max_submaps);
    mapper->startPhase1Mapping(req.max_submaps);
    res.success = true;
    res.message = "Phase1 mapping started with max_submaps=" + std::to_string(req.max_submaps);
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}

bool stopPhase1Callback(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res,
                        sonar_slam::MultiPositionSonarMapper* mapper) {
    ROS_INFO("Service called: /stop_phase1_mapping");
    mapper->stopPhase1Mapping();
    res.success = true;
    res.message = "Phase1 mapping stopped";
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}
 

bool getStatusCallback(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res,
                       sonar_slam::MultiPositionSonarMapper* mapper) {
    res.success = true;
    res.message = mapper->getPhase1StatusString();
    return true;
}

// ======================== 多位置建图服务回调 ========================

bool startPhaseMultiCallback(sonar_slam::StartPhaseMulti::Request& req,
                             sonar_slam::StartPhaseMulti::Response& res,
                             sonar_slam::MultiPositionSonarMapper* mapper) {
    ROS_INFO("Service called: /start_phase_multi_mapping with num_positions=%d", req.num_positions);
    
    // 验证参数范围
    if (req.num_positions < 3 || req.num_positions > 10) {
        res.success = false;
        res.message = "位置数量必须在3-10之间";
        ROS_ERROR("%s", res.message.c_str());
        return true;
    }
    
    mapper->startPhaseMultiMapping(req.num_positions);
    res.success = true;
    res.message = "PhaseMulti mapping started with " + std::to_string(req.num_positions) + " positions";
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}

bool stopPhaseMultiCallback(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res,
                            sonar_slam::MultiPositionSonarMapper* mapper) {
    ROS_INFO("Service called: /stop_phase_multi_mapping");
    mapper->stopPhaseMultiMapping();
    res.success = true;
    res.message = "PhaseMulti mapping stopped";
    ROS_INFO("Service response: %s", res.message.c_str());
    return true;
}

bool getPhaseMultiStatusCallback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res,
                                 sonar_slam::MultiPositionSonarMapper* mapper) {
    res.success = true;
    res.message = mapper->getPhaseMultiStatusString();
    return true;
}

// ======================== 主函数 ========================
int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");  // 确保数字格式正确
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
    sonar_slam::MultiPositionSonarMapper mapper(nh);
    
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


    // ==================== 注册多位置建图服务 ====================
    ros::ServiceServer phase_multi_start_srv = nh_global.advertiseService<sonar_slam::StartPhaseMulti::Request, sonar_slam::StartPhaseMulti::Response>(
        "start_phase_multi_mapping",
        boost::bind(&startPhaseMultiCallback, _1, _2, &mapper));

    ros::ServiceServer phase_multi_stop_srv = nh_global.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "stop_phase_multi_mapping",
        boost::bind(&stopPhaseMultiCallback, _1, _2, &mapper));

    ros::ServiceServer phase_multi_status_srv = nh_global.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "get_phase_multi_status",
        boost::bind(&getPhaseMultiStatusCallback, _1, _2, &mapper));

    ROS_INFO("多位置建图服务已创建:");
    ROS_INFO("  /start_phase_multi_mapping (num_positions=3-10)");
    ROS_INFO("  /stop_phase_multi_mapping");
    ROS_INFO("  /get_phase_multi_status");
    
    // 初始化串口
    if (!mapper.initSerial(serial_port, baudrate)) {
        ROS_ERROR("串口初始化失败，程序退出");
        return 1;
    }
    
 

     // 检查是否自动启动第一阶段
   
    bool auto_start_phase1 = false;
    nh.param("phase1/enabled", auto_start_phase1, false);

    if (auto_start_phase1) {
 
        // 自动启动多位置模式
        int num_positions = 2;
        nh.param("phase1/num_positions", num_positions, 2);
        
        ROS_INFO("========================================");
        ROS_INFO("=== 自动启动多位置建图模式 (%d个位置) ===", num_positions);
        ROS_INFO("========================================");
        
        // 等待2秒，确保所有组件就绪
        ros::Duration(2.0).sleep();
        
        // 启动多位置建图
        mapper.startPhaseMultiMapping(num_positions);
         
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
        mapper.spinPumpGateQueue();   // 派发 /cmd_vel 专用队列(仅 onCmdVel), 令守门入口 last_cmd_* 持续新鲜、无陈旧积压

        // 确保TF被发布
        mapper.publishTF();
   
            // 多位置模式
        mapper.handlePhaseMultiMapping();

        // [方案A] 显示层刷新: 原本每帧在串口I/O线程做的点云/栅格重建+发布已删除, 改由主线程在此周期做。
        //   10Hz 主循环里每 5 拍(=~2Hz, 人眼级足够)、且仅扫描中才刷新。落盘另有强制刷新(见 saveCurrentPhaseMultiSubmap)。
        //   与 handlePhaseMultiMapping(含落盘)同在主线程顺序执行 -> 二者绝不并发, pointcloud_/占据图无需额外互斥。
        //   不在 scanningThread 内做, 故不影响发角/收角节拍与 sweep_pace 语义。
        if (loop_counter % 5 == 0 && mapper.isScanning()) {
            mapper.rebuildAndPublishMaps();
        }

        // 每5秒输出一次状态
        if (loop_counter % 50 == 0) {
            if (mapper.isPhaseMultiEnabled()) {
                ROS_INFO("[主循环] %s", mapper.getPhaseMultiStatusString().c_str());
            } else {
                ROS_INFO("[主循环] %s", mapper.getPhase1StatusString().c_str());
            }
        }
        loop_counter++;
            
            rate.sleep();
        }
    
    
    mapper.closePumpLatch();   // Ctrl-C/退出: 撤闩关泵(主循环退出后节点仍在, 能送达)
    mapper.stopScanning();

    ROS_INFO("Sonar Realtime Mapper 关闭");
    return 0;
}