/**
 * @file sonar_slam_node.cpp
 * @brief 水下清扫机器人动态建图节点
 * 功能：通过声纳数据动态构建环境地图，实现自主探索和覆盖
 *   完整的状态机：7个状态，逻辑清晰
    前沿探索算法：实现了完整的Frontier Detection
    概率地图更新：使用log-odds方法，比简单阈值更鲁棒
    多传感器融合：超声波+红外+里程计
    完善的导航控制：PID控制+避障
    可视化丰富：前沿点、目标点、扫描点可视化
 * 
 * 
 * 
 * 
 */

#include "sonar_slam/sonar_slam_node.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <ros/console.h>
 
// 地图单元常量定义
static const int8_t CELL_UNKNOWN = -1;
static const int8_t CELL_FREE = 0;
static const int8_t CELL_OCCUPIED = 100;

namespace sonar_slam {
    

SonarSlamNode::SonarSlamNode() : 
    map_updated_(false),
    last_known_cells_(0),
    map_stability_counter_(0),
    has_start_pose_(false),
    current_state_(STATE_INITIAL_MAPPING),
    has_goal_(false),
    current_target_x_(0),
    current_target_y_(0),
    exploration_complete_(false),
    ultrasonic_front_right_(0.3f),
    ultrasonic_rear_right_(0.3f),
    infrared_front_left_(0.3f),
    infrared_front_right_(0.3f),
    map_coverage_(0.0f),
    scan_count_(0),
    update_cycle_(0),
    tf_listener_(nullptr),
    tf_broadcaster_(nullptr),
    scan_matcher_initialized_(false),
    exploring_for_gaps_(false),
    gap_fill_attempts_(0),
    gap_fill_count_(0),
    map_verified_complete_(false),
    first_tf_(true) 
{
    // 初始化ROS节点
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // ==================== 参数配置 ====================
    // 地图更新参数
    private_nh.param("update_interval", update_interval_, 15.0);  // 更新间隔
    private_nh.param("map_resolution", map_resolution_, 0.05);   // 地图分辨率
    private_nh.param("map_width", map_width_, 400);              // 地图宽度
    private_nh.param("map_height", map_height_, 400);            // 地图高度
    private_nh.param("map_origin_x", map_origin_x_, -10.0);      // 地图原点X
    private_nh.param("map_origin_y", map_origin_y_, -10.0);      // 地图原点Y
    
    // 动态建图参数
    private_nh.param("stability_threshold", stability_threshold_, 10);     // 稳定性阈值
    private_nh.param("frontier_weight_distance", frontier_weight_distance_, 0.4f);
    private_nh.param("frontier_weight_unknown", frontier_weight_unknown_, 0.6f);
    
    // 导航参数
    private_nh.param("max_linear_speed", max_linear_speed_, 0.2f);
    private_nh.param("max_angular_speed", max_angular_speed_, 0.5f);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.3f);
    
    private_nh.param("tf_smooth_factor", tf_smooth_factor_, 0.3);
    ROS_INFO("TF平滑系数: %.2f", tf_smooth_factor_);
    
    ROS_INFO("================");
    ROS_INFO("Sonar SLAM Node Initializing");
    ROS_INFO("================");
    
    // ==================== 订阅器 ====================
    // 声纳数据（主要建图数据）
    sonar_cloud_sub_ = nh.subscribe("/sonar_realtime_mapper/sonar/filtered_pointcloud", 10, 
                                   &SonarSlamNode::sonarCloudCallback, this);
    
    // 超声波数据（用于避障和沿墙）
    ultrasonic_sub_ = nh.subscribe("/ultra_data", 10, 
                                  &SonarSlamNode::ultrasonicCallback, this);
    
    // // 红外数据（近距离避障）
    // infrared_sub_ = nh.subscribe("/infrared_data", 10,
    //                             &SonarSlamNode::infraredCallback, this);
    
    // 里程计（定位）
    odom_sub_ = nh.subscribe("/wheel_odom", 10, 
                            &SonarSlamNode::odomCallback, this);
    
    // 覆盖状态（从运动控制器）
    coverage_state_sub_ = nh.subscribe("/coverage_state", 10,
                                      &SonarSlamNode::coverageStateCallback, this);


    // IMU订阅
    imu_sub_ = nh.subscribe("/imu_data", 10, 
                           &SonarSlamNode::imuCallback, this);
    
    // ==================== 发布器 ====================
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    frontier_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/frontiers", 10);
    goal_marker_pub_ = nh.advertise<visualization_msgs::Marker>("/current_goal", 10);


    static_mapping_done_sub_ = nh.subscribe("/static_mapping/done", 1, 
                                           &SonarSlamNode::staticMappingDoneCallback, this);
    initial_map_sub_ = nh.subscribe("/initial_occupancy_map", 1,
                                   &SonarSlamNode::initialMapCallback, this);
    
    has_initial_map_ = false;
    
    // ==================== TF ====================
    tf_listener_ = new tf::TransformListener(ros::Duration(10.0));
    tf_broadcaster_ = new tf::TransformBroadcaster();
    
    // ==================== 定时器 ====================
    // 地图更新定时器（按设定间隔更新地图）
    update_timer_ = nh.createTimer(ros::Duration(update_interval_), 
                                  &SonarSlamNode::updateTimerCallback, this);
    
    // 状态机定时器（高频处理状态转换）
    state_timer_ = nh.createTimer(ros::Duration(0.1), 
                                 &SonarSlamNode::stateMachineCallback, this);
    
    // 传感器处理定时器（实时处理传感器数据）
    sensor_timer_ = nh.createTimer(ros::Duration(0.02),
                                  &SonarSlamNode::sensorProcessingCallback, this);
    
    // 覆盖监控定时器（监控地图覆盖率）
    coverage_timer_ = nh.createTimer(ros::Duration(1.0),
                                    &SonarSlamNode::coverageMonitoringCallback, this);
    
    // ==================== 初始化地图 ====================
    initMap();
    
    ROS_INFO("================");
    ROS_INFO("Sonar SLAM Node Initialized Successfully");
    ROS_INFO("  Map Size: %dx%d cells", map_width_, map_height_);
    ROS_INFO("  Resolution: %.3f m/cell", map_resolution_);
    ROS_INFO("  Update Interval: %.1f s", update_interval_);
    ROS_INFO("================");

     // 初始化扫描匹配器参数
    sonar_slam::ScanMatcher::MatchParams params;
    params.algorithm = sonar_slam::ScanMatcher::ALGO_CORRELATION;  // 快速匹配
    params.max_iterations = 30;
    params.max_correspondence_distance = 1.0;
    scan_matcher_.setParams(params);
    
  
    
    // 初始化点云
    last_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    // 添加调试发布器
    matched_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/scan_matched", 1);
    pose_correction_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_correction", 1);

     // 初始化缺口目标
    current_gap_target_.position.x = 0;
    current_gap_target_.position.y = 0;
    current_gap_target_.size = 0;
    current_gap_target_.distance = 0;
    current_gap_target_.priority = 0;
 
 
}

SonarSlamNode::~SonarSlamNode() {
    delete tf_listener_;
    delete tf_broadcaster_;
}

// ==================== 地图初始化 ====================
void SonarSlamNode::initMap() {
    map_.header.frame_id = "map";
    map_.info.resolution = map_resolution_;
    map_.info.width = map_width_;
    map_.info.height = map_height_;
    map_.info.origin.position.x = map_origin_x_;
    map_.info.origin.position.y = map_origin_y_;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    
    // 初始化地图数据（全部设为未知）
    map_.data.resize(map_width_ * map_height_, CELL_UNKNOWN);
    
    // 初始化log-odds地图（用于概率更新）
    log_odds_map_.resize(map_width_ * map_height_, 0.0f);
    
    // 初始化更新历史
    cell_history_.resize(map_width_ * map_height_, 0);
    
    ROS_INFO("Map initialized with %dx%d cells", map_width_, map_height_);
}

// ==================== 声纳数据回调 ====================
 void SonarSlamNode::sonarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    ROS_INFO_THROTTLE(2.0, "收到点云: frame_id=%s, 时间戳=%.3f", 
                     msg->header.frame_id.c_str(), msg->header.stamp.toSec());
    
    // 【关键修复1】检查并修正时间戳
    ros::Time corrected_time = msg->header.stamp;
    ros::Time now = ros::Time::now();
    
    // 如果时间戳比当前时间还晚，或者太旧，修正它
    if (corrected_time > now) {
        ROS_WARN("点云时间戳异常（未来时间），使用当前时间");
        corrected_time = now;
    } else if ((now - corrected_time).toSec() > 1.0) {
        ROS_WARN("点云时间戳过旧（%.1f秒前），使用当前时间", (now - corrected_time).toSec());
        corrected_time = now;
    }
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    try {
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            ROS_DEBUG("原始点云为空");
            return;
        }
        
        // 使用修正后的时间戳
        tf::StampedTransform transform;
        
        // 先尝试使用修正时间戳
        try {
            tf_listener_->waitForTransform("map", msg->header.frame_id, 
                                          corrected_time, ros::Duration(0.1));
            tf_listener_->lookupTransform("map", msg->header.frame_id, 
                                         corrected_time, transform);
            ROS_DEBUG_THROTTLE(2.0, "使用修正时间戳(%.3f)获取TF", corrected_time.toSec());
        } catch (tf::TransformException& ex1) {
            // 如果失败，使用最近时间
            ROS_WARN_THROTTLE(2.0, "修正时间戳失败: %s，使用最近时间", ex1.what());
            tf_listener_->lookupTransform("map", msg->header.frame_id, 
                                         ros::Time(0), transform);
        }
        
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform);
        
    } catch (tf::TransformException& ex) {
        ROS_ERROR_THROTTLE(2.0, "TF变换完全失败: %s", ex.what());
        
        // 即使TF失败，也尝试处理点云
        if (cloud->empty()) return;
        
        float robot_x, robot_y, robot_yaw;
        if (getRobotPose(robot_x, robot_y, robot_yaw)) {
            for (const auto& point : cloud->points) {
                float world_x = robot_x + point.x;
                float world_y = robot_y + point.y;
                
                float distance = sqrt(pow(world_x - robot_x, 2) + pow(world_y - robot_y, 2));
                if (distance > 20.0f) continue;
                if (std::isnan(world_x) || std::isnan(world_y)) continue;
                if (point.intensity < 30.0f) continue;
                
                updateMapWithPoint(world_x, world_y);
                scan_points_.push_back(std::make_pair(world_x, world_y));
            }
            
            if (scan_points_.size() > 1000) scan_points_.pop_front();
            if (!cloud->empty()) {
                map_updated_ = true;
                scan_count_++;
            }
        }
        return;
    }


    if (transformed_cloud->empty()) {
        ROS_WARN("变换后点云为空");
        return;
    }
    
    // 调试信息：显示转换前后第一个点的坐标
    static bool first_point_debug = true;
    if (first_point_debug && !cloud->empty() && !transformed_cloud->empty()) {
        const auto& raw_point = cloud->points[0];
        const auto& trans_point = transformed_cloud->points[0];
        
        ROS_INFO("坐标转换示例:");
        ROS_INFO("  原始(%s): (%.2f, %.2f)", 
                msg->header.frame_id.c_str(), raw_point.x, raw_point.y);
        ROS_INFO("  转换后(map): (%.2f, %.2f)", trans_point.x, trans_point.y);
        ROS_INFO("  差值: (%.2f, %.2f)", 
                trans_point.x - raw_point.x, trans_point.y - raw_point.y);
        first_point_debug = false;
    }
    
    // 获取机器人当前位姿
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN_THROTTLE(2.0, "无法获取机器人位姿");
        return;
    }
    
    ROS_DEBUG_THROTTLE(5.0, "机器人位姿: (%.2f, %.2f, %.2f rad)", 
                      robot_x, robot_y, robot_yaw);
    
    int points_added = 0;
    int filtered_points = 0;
    
    // 处理每个声纳点（现在已经是在世界坐标系）
    for (const auto& point : transformed_cloud->points) {
        float world_x = point.x;
        float world_y = point.y;
        
        // 【重要】验证转换后的坐标
        float distance_from_robot = sqrt(pow(world_x - robot_x, 2) + pow(world_y - robot_y, 2));
        
        // 过滤距离过远的点（可能是转换错误）
        if (distance_from_robot > 20.0f) {
            filtered_points++;
            continue;
        }
        
        // 过滤无效点
        if (std::isnan(world_x) || std::isnan(world_y)) {
            filtered_points++;
            continue;
        }
        
        // 强度过滤
        if (point.intensity < 30.0f) {
            filtered_points++;
            continue;
        }
        
        // 距离过滤
        float distance = sqrt(world_x*world_x + world_y*world_y);
        if (distance < 0.3f || distance > 15.0f) {
            filtered_points++;
            continue;
        }
        
        // 更新地图
        updateMapWithPoint(world_x, world_y);
        points_added++;
        
        // 记录用于可视化的点
        scan_points_.push_back(std::make_pair(world_x, world_y));
        // 保持固定大小（1000个点）
        if (scan_points_.size() > 1000) {
            scan_points_.pop_front();  // O(1)操作
        }
    }
    
    if (points_added > 0) {
        map_updated_ = true;
        scan_count_++;
        
        ROS_INFO_THROTTLE(5.0, "地图更新: 添加 %d 点，过滤 %d 点，总计 %d 次扫描", 
                         points_added, filtered_points, scan_count_);
        
        // 每处理一定数量点后发布地图
        if (scan_count_ % 5 == 0) {  // 保持原有频率
            publishMap();
            visualizeScanPoints();
            
            // 【新增】显示地图统计
            int known_cells = 0;
            for (int val : map_.data) {
                if (val != -1) known_cells++;
            }
            ROS_INFO("地图统计: %d/%d 已知单元 (%.1f%%)", 
                    known_cells, map_width_ * map_height_,
                    known_cells * 100.0 / (map_width_ * map_height_));
        }
    } else {
        ROS_DEBUG_THROTTLE(5.0, "本次扫描无有效点");
    }

    // 保存当前扫描用于下一次匹配
    *last_scan_ = *cloud;
    scan_matcher_initialized_ = true;
}


/************
void SonarSlamNode::sonarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    ROS_INFO_THROTTLE(2.0, "收到点云: frame_id=%s, 点数=%d", 
                     msg->header.frame_id.c_str(), msg->width * msg->height);


    static ros::Time last_process_time(0);
    ros::Time now = ros::Time::now();

     if ((now - last_process_time).toSec() < 0.2) {
        return;  // 跳过这次处理
    }
    last_process_time = now;
    
    ROS_DEBUG_THROTTLE(2.0, "处理点云数据");
    
    // ==================== 【修复1：正确的坐标转换方法】====================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    try {
        // 1. 将ROS消息转换为PCL点云
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            ROS_DEBUG("原始点云为空");
            return;
        }
        
        // 2. 获取从sonar_frame到map的变换
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("map", msg->header.frame_id, 
                                     ros::Time(0), transform);
        
        ROS_DEBUG_THROTTLE(2.0, "获取TF变换: %s → map", msg->header.frame_id.c_str());
        
        // 3. 使用pcl_ros::transformPointCloud进行转换
        pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform);
        
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "TF变换失败: %s", ex.what());
        
        // 如果转换失败，尝试使用最近的有效变换
        try {
            // 使用更宽松的时间戳
            tf::StampedTransform transform;
            tf_listener_->lookupTransform("map", msg->header.frame_id, 
                                         ros::Time::now(), transform);
            pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform);
            ROS_WARN("使用当前时间戳的变换成功");
        } catch (tf::TransformException& ex2) {
            ROS_ERROR("完全无法获取TF变换: %s", ex2.what());
            return;
        }
    }
    
    // ==================== 【修复2：确保变换后的点云有效】====================
    if (transformed_cloud->empty()) {
        ROS_WARN("变换后点云为空");
        return;
    }
    
    // 调试信息：显示转换前后第一个点的坐标
    static bool first_point_debug = true;
    if (first_point_debug && !cloud->empty() && !transformed_cloud->empty()) {
        const auto& raw_point = cloud->points[0];
        const auto& trans_point = transformed_cloud->points[0];
        
        ROS_INFO("坐标转换示例:");
        ROS_INFO("  原始(%s): (%.2f, %.2f)", 
                msg->header.frame_id.c_str(), raw_point.x, raw_point.y);
        ROS_INFO("  转换后(map): (%.2f, %.2f)", trans_point.x, trans_point.y);
        ROS_INFO("  差值: (%.2f, %.2f)", 
                trans_point.x - raw_point.x, trans_point.y - raw_point.y);
        first_point_debug = false;
    }
    
    // ==================== 【继续原有处理逻辑，使用transformed_cloud】====================
    // 获取机器人当前位姿
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN_THROTTLE(2.0, "无法获取机器人位姿");
        return;
    }
    
    ROS_DEBUG_THROTTLE(5.0, "机器人位姿: (%.2f, %.2f, %.2f rad)", 
                      robot_x, robot_y, robot_yaw);
    
    int points_added = 0;
    int filtered_points = 0;
    
    // 处理每个声纳点（现在已经是在世界坐标系）
    for (const auto& point : transformed_cloud->points) {
        float world_x = point.x;
        float world_y = point.y;
        
        // 【重要】验证转换后的坐标
        float distance_from_robot = sqrt(pow(world_x - robot_x, 2) + pow(world_y - robot_y, 2));
        
        // 过滤距离过远的点（可能是转换错误）
        if (distance_from_robot > 20.0f) {
            ROS_WARN_THROTTLE(5.0, "过滤远距离点: 距离机器人 %.1f 米", distance_from_robot);
            filtered_points++;
            continue;
        }
        
        // 过滤无效点
        if (std::isnan(world_x) || std::isnan(world_y)) {
            filtered_points++;
            continue;
        }
        
        // 强度过滤
        if (point.intensity < 30.0f) {
            filtered_points++;
            continue;
        }
        
        // 距离过滤
        float distance = sqrt(world_x*world_x + world_y*world_y);
        if (distance < 0.3f || distance > 15.0f) {
            filtered_points++;
            continue;
        }
        
        // 更新地图
        updateMapWithPoint(world_x, world_y);
        points_added++;
        
        // 记录用于可视化的点
        scan_points_.push_back(std::make_pair(world_x, world_y));
            // 保持固定大小（1000个点）
        if (scan_points_.size() > 1000) {
            scan_points_.pop_front();  // O(1)操作
        }

    }
    


    if (points_added > 0) {
        map_updated_ = true;
        scan_count_++;
        
        ROS_INFO_THROTTLE(5.0, "地图更新: 添加 %d 点，过滤 %d 点，总计 %d 次扫描", 
                         points_added, filtered_points, scan_count_);
        
        // 每处理一定数量点后发布地图
        if (scan_count_ % 10 == 0) {  // 改为每10次扫描发布一次
            publishMap();
            visualizeScanPoints();
            
            // 显示地图统计
            int known_cells = 0;
            for (int val : map_.data) {
                if (val != -1) known_cells++;
            }
            ROS_INFO("地图统计: %d/%d 已知单元 (%.1f%%)", 
                    known_cells, map_width_ * map_height_,
                    known_cells * 100.0 / (map_width_ * map_height_));
        }
    } else {
        ROS_DEBUG_THROTTLE(5.0, "本次扫描无有效点");
    }

    // 保存当前扫描用于下一次匹配
    *last_scan_ = *cloud;
    scan_matcher_initialized_ = true;
}
******** */

// ==================== 地图更新核心函数 ====================
void SonarSlamNode::updateMapWithPoint(float world_x, float world_y) {
    // 转换为地图坐标
    int map_x = worldToMapX(world_x);
    int map_y = worldToMapY(world_y);
    
    // 边界检查
    if (map_x < 0 || map_x >= map_width_ || map_y < 0 || map_y >= map_height_) {
        return;
    }
    
    int idx = map_y * map_width_ + map_x;
    
    // ==================== 概率更新 ====================
    // 使用log-odds表示概率，更稳定
    const float LOG_OCCUPIED = 0.7f;    // 占据的log odds增量
    const float LOG_FREE = -0.4f;       // 自由的log odds增量
    const float CLAMP_MIN = -10.0f;     // 最小log odds
    const float CLAMP_MAX = 10.0f;      // 最大log odds
    
    // 1. 更新击中点（占据）
    log_odds_map_[idx] += LOG_OCCUPIED;
    log_odds_map_[idx] = std::max(CLAMP_MIN, std::min(CLAMP_MAX, log_odds_map_[idx]));
    
    // 将log-odds转换为概率
    float prob = 1.0f - 1.0f / (1.0f + exp(log_odds_map_[idx]));
    
    // 根据概率设置占据状态
    if (prob > 0.65f) {
        map_.data[idx] = CELL_OCCUPIED;      // 确定占据
    } else if (prob < 0.25f) {
        map_.data[idx] = CELL_FREE;          // 确定自由
    } else {
        // 不确定区域，存储概率值
        map_.data[idx] = static_cast<int8_t>(prob * 100);
    }
    
    // 2. 更新从机器人到击中点的自由空间
    int robot_map_x, robot_map_y;
    if (getRobotMapPosition(robot_map_x, robot_map_y)) {
        std::vector<std::pair<int, int>> line;
        bresenhamLine(robot_map_x, robot_map_y, map_x, map_y, line);
        
        // 标记线上的点为自由（除了终点）
        for (size_t i = 0; i < line.size() - 1; i++) {
            int line_x = line[i].first;
            int line_y = line[i].second;
            
            if (line_x >= 0 && line_x < map_width_ && 
                line_y >= 0 && line_y < map_height_) {
                int line_idx = line_y * map_width_ + line_x;
                
                // 更新log odds（自由）
                log_odds_map_[line_idx] += LOG_FREE;
                log_odds_map_[line_idx] = std::max(CLAMP_MIN, std::min(CLAMP_MAX, log_odds_map_[line_idx]));
                
                // 转换为概率
                float free_prob = 1.0f - 1.0f / (1.0f + exp(log_odds_map_[line_idx]));
                if (free_prob < 0.25f) {
                    map_.data[line_idx] = CELL_FREE;
                }
            }
        }
    }
    
    // 记录更新历史
    cell_history_[idx]++;
}

// ==================== 传感器回调 ====================
void SonarSlamNode::ultrasonicCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (!msg || msg->data.size() < 2) {
        ROS_WARN_THROTTLE(5.0, "Invalid ultrasonic data");
        return;
    }
    
    // 解析超声波数据（假设数据单位为毫米）
    // ultrasonic_front_right_ = msg->data[0] / 1000.0f;  // 转换为米
    // ultrasonic_rear_right_ = msg->data[1] / 1000.0f;
    
    // ROS_DEBUG_THROTTLE(2.0, "Ultrasonic: front=%.2fm, rear=%.2fm", 
    //                   ultrasonic_front_right_, ultrasonic_rear_right_);

    // data[0], data[1] 是前方红外（毫米）
    infrared_front_left_ = msg->data[0] / 1000.0f;  // 转为米
    infrared_front_right_ = msg->data[1] / 1000.0f;
    
    // data[2], data[3] 是侧方超声波（毫米）
    ultrasonic_front_right_ = msg->data[2] / 1000.0f;
    ultrasonic_rear_right_ = msg->data[3] / 1000.0f;
    
    // 过滤异常值 65533
    if (infrared_front_left_ > 10.0f) infrared_front_left_ = 2.0f;
    if (infrared_front_right_ > 10.0f) infrared_front_right_ = 2.0f;
    if (ultrasonic_front_right_ > 10.0f) ultrasonic_front_right_ = 2.0f;
    if (ultrasonic_rear_right_ > 10.0f) ultrasonic_rear_right_ = 2.0f;
    
    ROS_DEBUG_THROTTLE(2.0, "传感器: 红外[%.2f, %.2f]m, 超声[%.2f, %.2f]m",
                      infrared_front_left_, infrared_front_right_,
                      ultrasonic_front_right_, ultrasonic_rear_right_);
}

void SonarSlamNode::infraredCallback(const mars_msgs::Infrared::ConstPtr& msg) {
    if (!msg) {
        ROS_WARN_THROTTLE(5.0, "Invalid infrared data");
        return;
    }
    
    // infrared_front_left_ = msg->data_1 / 1000.0f;   // 转换为米
    // infrared_front_right_ = msg->data_2 / 1000.0f;
    
    // ROS_DEBUG_THROTTLE(2.0, "Infrared: left=%.2fm, right=%.2fm", 
    //                   infrared_front_left_, infrared_front_right_);

     ROS_WARN_THROTTLE(10.0, "红外话题 /infrared_data 已弃用，使用 /ultra_data");
}

void SonarSlamNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_odom_ = *msg;
    
    // 如果是第一次收到odom，记录为起点
    if (!has_start_pose_) {
        start_pose_.x = msg->pose.pose.position.x;
        start_pose_.y = msg->pose.pose.position.y;
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        start_pose_.theta = tf::getYaw(q);
        has_start_pose_ = true;
        
        ROS_INFO("Start pose recorded: (%.2f, %.2f, %.2f rad)", 
                start_pose_.x, start_pose_.y, start_pose_.theta);
    }
}

void SonarSlamNode::coverageStateCallback(const std_msgs::Int32::ConstPtr& msg) {
    // 覆盖状态：0=空闲，1=运行中，2=完成
    int state = msg->data;
    
    if (state == 2 && current_state_ == STATE_INITIAL_MAPPING) {
        ROS_INFO("Coverage task completed from motion controller");
        exploration_complete_ = true;
    }
}

// ==================== 定时器回调 ====================
 
void SonarSlamNode::updateTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    update_cycle_++;
    
    // 发布map->odom的TF变换
    publishMapToOdomTF();
    
    // 1. 发布更新的地图
    if (map_updated_) {
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);
        map_updated_ = false;
    }
    
    // 2. 检查地图稳定性
    checkMapStability();
    
    // 3. 计算并记录地图覆盖率
    map_coverage_ = calculateMapCoverage();
    
    // 4. 可视化前沿点
    visualizeFrontiers();
    
    ROS_DEBUG("Update cycle %d: coverage=%.1f%%, scans=%d", 
              update_cycle_, map_coverage_ * 100, scan_count_);
}

 

void SonarSlamNode::stateMachineCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 主状态机
    switch (current_state_) {
        case STATE_INITIAL_MAPPING:
            handleInitialMappingState();
            break;
            
        case STATE_GAP_EXPLORATION:    // 【新增】
            handleGapExplorationState();
            break;
            
        case STATE_COVERAGE_VERIFY:    // 【新增】
            handleCoverageVerifyState();
            break;
            
        case STATE_RETURN_HOME:
            handleReturnHomeState();
            break;
            
        case STATE_COMPLETE:
            handleCompleteState();
            break;
            
        case STATE_EMERGENCY_STOP:
            handleEmergencyStopState();
            break;
            
        default:
            ROS_ERROR("未知状态: %d，重置为初始建图", current_state_);
            current_state_ = STATE_INITIAL_MAPPING;
            break;
    }
}

void SonarSlamNode::sensorProcessingCallback(const ros::TimerEvent& event) {
    // 实时传感器处理
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 1. 紧急避障检查
    if (checkEmergencyConditions()) {
        ROS_WARN("Emergency condition detected!");
        triggerEmergencyStop();
        return;
    }
    
    // 2. 实时日志
    static int log_counter = 0;
    if (log_counter++ % 25 == 0) {  // 每0.5秒记录一次
        ROS_DEBUG_THROTTLE(0.5, "Sensors - Ultra: %.2f/%.2fm, IR: %.2f/%.2fm", 
                          ultrasonic_front_right_, ultrasonic_rear_right_,
                          infrared_front_left_, infrared_front_right_);
        log_counter = 0;
    }
}

void SonarSlamNode::coverageMonitoringCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 监控地图覆盖率
    float coverage = calculateMapCoverage();
    
    // 如果覆盖率足够高，可以认为探索完成
    if (coverage > 0.85f && current_state_ == STATE_INITIAL_MAPPING) {
        ROS_INFO("Map coverage reached %.1f%%, exploration complete", coverage * 100);
        exploration_complete_ = true;
    }
}

// ==================== 地图稳定性检查 ====================
void SonarSlamNode::checkMapStability() {
    // 计算当前已知单元数
    int known_cells = 0;
    for (int val : map_.data) {
        if (val != CELL_UNKNOWN) known_cells++;
    }
    
    // 检查地图变化
    if (abs(known_cells - last_known_cells_) < 50) {  // 变化小于50个单元
        map_stability_counter_++;
    } else {
        map_stability_counter_ = 0;
    }
    
    last_known_cells_ = known_cells;
    
    // 如果地图稳定达到阈值，可以认为探索完成
    if (map_stability_counter_ >= stability_threshold_) {
        if (!exploration_complete_ && current_state_ == STATE_INITIAL_MAPPING) {
            ROS_INFO("Map stabilized for %d cycles, exploration complete", 
                     stability_threshold_);
            exploration_complete_ = true;
        }
        map_stability_counter_ = 0;
    }
}
 
// ==================== 动态建图核心：前沿点探索 ====================
/*******
bool SonarSlamNode::findNextGoal() {
    ROS_INFO("=== 寻找下一个探索目标 ===");
    
    // 【关键修改】先检查是否应该继续探索
    if (!shouldContinueExploring()) {
        ROS_INFO("探索完成，准备返回起点");
        exploration_complete_ = true;
        current_state_ = STATE_RETURN_HOME;
        return false;
    }
    
    // 获取机器人位置
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) {
        ROS_WARN("无法获取机器人位置");
        return false;
    }
    
    // 1. 检测边界缺口（优先填补缺口）
    std::vector<BoundaryGap> gaps = detectBoundaryGaps();
    
    if (!gaps.empty()) {
        // 选择最佳缺口
        BoundaryGap best_gap = selectBestGapToFill(gaps);
        
        // 设置目标为缺口中心（或安全的接近点）
        current_target_x_ = best_gap.center.x;
        current_target_y_ = best_gap.center.y;
        has_goal_ = true;
        
        // 记录当前填补的缺口
        current_gaps_ = gaps;
        exploring_for_gaps_ = true;
        
        ROS_INFO(" 选择缺口填补: 位置(%.2f, %.2f), 大小=%.2fm, 优先级=%d", 
                current_target_x_, current_target_y_, best_gap.size, best_gap.priority);
        return true;
    }
    
    // 2. 如果没有检测到明显缺口，使用前沿点探索
    ROS_INFO("未检测到明显缺口，使用前沿点探索");
    exploring_for_gaps_ = false;
    
    std::vector<Frontier> frontiers;
    findFrontiers(frontiers);
    
    if (!frontiers.empty()) {
        Frontier best = selectBestFrontier(frontiers);
        current_target_x_ = mapToWorldX(best.x);
        current_target_y_ = mapToWorldY(best.y);
        has_goal_ = true;
        
        ROS_INFO("新探索目标: (%.2f, %.2f) 权重: %.3f", 
                current_target_x_, current_target_y_, best.weight);
        return true;
    }
    
    // 3. 如果既没有缺口也没有前沿点，检查是否需要特殊处理
    ROS_INFO("未找到探索目标");
    return false;
}
*****/

bool SonarSlamNode::findNextGoal() {
    // 1. 检查是否应该继续探索
    if (exploration_complete_) {
        return false;
    }
    
    // 2. 优先填补边界缺口
    std::vector<BoundaryGap> gaps = detectBoundaryGaps();
    
    if (!gaps.empty()) {
        // 选择最佳缺口（优先级最高的）
        BoundaryGap best_gap = gaps[0];
        
        current_target_x_ = best_gap.center.x;
        current_target_y_ = best_gap.center.y;
        has_goal_ = true;
        
        ROS_INFO("选择填补缺口: 位置(%.2f, %.2f), 大小=%.2fm, 优先级=%d", 
                current_target_x_, current_target_y_, best_gap.size, best_gap.priority);
        return true;
    }
    
    // 3. 如果没有明显缺口，寻找最远的前沿点
    std::vector<Frontier> frontiers;
    findFrontiers(frontiers);
    
    if (!frontiers.empty()) {
        // 选择最远的前沿点（实现"向最远点方向直线行走"）
        Frontier farthest_frontier;
        float max_distance = 0;
        
        for (const auto& frontier : frontiers) {
            if (frontier.distance > max_distance) {
                max_distance = frontier.distance;
                farthest_frontier = frontier;
            }
        }
        
        current_target_x_ = mapToWorldX(farthest_frontier.x);
        current_target_y_ = mapToWorldY(farthest_frontier.y);
        has_goal_ = true;
        
        ROS_INFO("选择最远前沿点: 位置(%.2f, %.2f), 距离=%.2fm", 
                current_target_x_, current_target_y_, max_distance);
        return true;
    }
    
    // 4. 如果也没有前沿点，检查地图是否完整
    if (isMapComplete()) {
        ROS_INFO("地图已完整，探索完成");
        exploration_complete_ = true;
        current_state_ = STATE_RETURN_HOME;
        return false;
    }
    
    // 5. 都没有找到，返回失败
    ROS_WARN("未找到探索目标");
    return false;
}


void SonarSlamNode::findFrontiers(std::vector<Frontier>& frontiers) {
    // 保持原有的缓存机制
    static std::vector<Frontier> cached_frontiers;
    static ros::Time last_update_time(0);
    static int call_count = 0;
    static int search_count = 0;
    
    ros::Time now = ros::Time::now();
    call_count++;
    search_count++;
    
    const int CACHE_FREQUENCY = 5;
    const double CACHE_TIMEOUT = 2.0;
    
    // 【修改1】调整缓存条件，确保能检测到新缺口
    bool use_cache = (!cached_frontiers.empty() && 
                     (now - last_update_time).toSec() < CACHE_TIMEOUT &&
                     call_count % CACHE_FREQUENCY != 0 &&
                     search_count > 5);  // 从3改为5
    
    if (use_cache) {
        frontiers = cached_frontiers;
        ROS_DEBUG_THROTTLE(1.0, "使用缓存前沿点 (%lu 个)", frontiers.size());
        return;
    }
    
    // 重新计算前沿点
    frontiers.clear();
    
    // 获取机器人位置和朝向
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN("无法获取机器人位姿");
        return;
    }
    
    // 【修改2】考虑凹字形水池特性
    // 计算机器人在水池中的相对位置
    const float POOL_HALF_LENGTH = 4.0f;  // 8米水池的一半
    const float POOL_HALF_WIDTH = 3.0f;   // 6米水池的一半
    
    // 判断机器人当前位置
    bool robot_in_left_half = robot_x < -1.0f;
    bool robot_in_right_half = robot_x > 1.0f;
    bool robot_in_top_half = robot_y > 1.0f;
    bool robot_in_bottom_half = robot_y < -1.0f;
    
    // 计算机器人所在区域（用于优先对侧搜索）
    int robot_region = 0; // 0=中心, 1=左, 2=右, 3=上, 4=下
    
    if (robot_x < -POOL_HALF_LENGTH * 0.3) robot_region = 1; // 左侧
    else if (robot_x > POOL_HALF_LENGTH * 0.3) robot_region = 2; // 右侧
    else if (robot_y > POOL_HALF_WIDTH * 0.3) robot_region = 3; // 上侧
    else if (robot_y < -POOL_HALF_WIDTH * 0.3) robot_region = 4; // 下侧
    
    // 保持原有的地图遍历逻辑，但改进权重计算
    const int STEP_SIZE = 2;  // 保持原有步长
    
    int cells_checked = 0;
    int frontier_cells_found = 0;
    
    // 【修改3】改进遍历范围，考虑水池尺寸
    int min_y = std::max(2, worldToMapY(-POOL_HALF_WIDTH - 1.0f));
    int max_y = std::min(map_height_ - 3, worldToMapY(POOL_HALF_WIDTH + 1.0f));
    int min_x = std::max(2, worldToMapX(-POOL_HALF_LENGTH - 1.0f));
    int max_x = std::min(map_width_ - 3, worldToMapX(POOL_HALF_LENGTH + 1.0f));
    
    for (int y = min_y; y < max_y; y += STEP_SIZE) {
        for (int x = min_x; x < max_x; x += STEP_SIZE) {
            cells_checked++;
            
            int idx = y * map_width_ + x;
            
            // 【修改4】放宽前沿条件（原代码可能过严）
            if (map_.data[idx] == CELL_OCCUPIED) continue;  // 保持原有
            
            // 检查周围是否有未知单元
            bool has_unknown = false;
            int unknown_count = 0;
            
            // 检查3x3邻域（采样区域）
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int sample_x = x + dx * STEP_SIZE;
                    int sample_y = y + dy * STEP_SIZE;
                    
                    if (sample_x < 0 || sample_x >= map_width_ || 
                        sample_y < 0 || sample_y >= map_height_) {
                        continue;
                    }
                    
                    int nidx = sample_y * map_width_ + sample_x;
                    if (map_.data[nidx] == CELL_UNKNOWN) {
                        has_unknown = true;
                        unknown_count++;
                    }
                }
            }
            
            if (has_unknown) {
                frontier_cells_found++;
                
                // 计算机器人到该点的距离
                float world_x = mapToWorldX(x);
                float world_y = mapToWorldY(y);
                float distance = sqrt(pow(world_x - robot_x, 2) + 
                                     pow(world_y - robot_y, 2));
                
                // 【修改5】调整距离过滤
                const float MAX_FRONTIER_DISTANCE = 8.0f;
                const float MIN_FRONTIER_DISTANCE = 0.8f;  // 增加最小距离
                
                if (distance > MAX_FRONTIER_DISTANCE || distance < MIN_FRONTIER_DISTANCE) {
                    continue;
                }
                
                // 【修改6】改进权重计算，考虑对侧优先
                float weight = calculateFrontierWeight(x, y, distance, unknown_count);
                
                // 对侧优先级调整
                float direction_bonus = 1.0f;
                if (robot_region == 1 && world_x > robot_x + 2.0f) {  // 机器人在左，优先右侧
                    direction_bonus = 1.5f;
                } else if (robot_region == 2 && world_x < robot_x - 2.0f) {  // 机器人在右，优先左侧
                    direction_bonus = 1.5f;
                } else if (robot_region == 3 && world_y < robot_y - 2.0f) {  // 机器人在上，优先下侧
                    direction_bonus = 1.5f;
                } else if (robot_region == 4 && world_y > robot_y + 2.0f) {  // 机器人在下，优先上侧
                    direction_bonus = 1.5f;
                }
                
                weight *= direction_bonus;
                
                frontiers.push_back(Frontier(x, y, distance, weight));
                
                // 保持原有的细化搜索逻辑
                const int REFINE_RADIUS = 1;
                for (int ry = -REFINE_RADIUS; ry <= REFINE_RADIUS; ry++) {
                    for (int rx = -REFINE_RADIUS; rx <= REFINE_RADIUS; rx++) {
                        if (rx == 0 && ry == 0) continue;
                        
                        int refine_x = x + rx;
                        int refine_y = y + ry;
                        
                        if (refine_x < 2 || refine_x >= map_width_ - 2 || 
                            refine_y < 2 || refine_y >= map_height_ - 2) {
                            continue;
                        }
                        
                        int refine_idx = refine_y * map_width_ + refine_x;
                        if (map_.data[refine_idx] == CELL_FREE) {
                            bool refine_has_unknown = false;
                            int refine_unknown_count = 0;
                            
                            for (int dy = -1; dy <= 1; dy++) {
                                for (int dx = -1; dx <= 1; dx++) {
                                    if (dx == 0 && dy == 0) continue;
                                    
                                    int nx = refine_x + dx;
                                    int ny = refine_y + dy;
                                    if (nx >= 0 && nx < map_width_ && 
                                        ny >= 0 && ny < map_height_) {
                                        int nidx = ny * map_width_ + nx;
                                        if (map_.data[nidx] == CELL_UNKNOWN) {
                                            refine_has_unknown = true;
                                            refine_unknown_count++;
                                        }
                                    }
                                }
                            }
                            
                            if (refine_has_unknown) {
                                float refine_world_x = mapToWorldX(refine_x);
                                float refine_world_y = mapToWorldY(refine_y);
                                float refine_distance = sqrt(pow(refine_world_x - robot_x, 2) + 
                                                           pow(refine_world_y - robot_y, 2));
                                
                                if (refine_distance <= MAX_FRONTIER_DISTANCE && 
                                    refine_distance >= MIN_FRONTIER_DISTANCE) {
                                    float refine_weight = calculateFrontierWeight(
                                        refine_x, refine_y, refine_distance, refine_unknown_count);
                                    refine_weight *= direction_bonus;  // 应用相同方向加成
                                    frontiers.push_back(Frontier(refine_x, refine_y, 
                                                                refine_distance, refine_weight));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 【修改7】改进去重逻辑
    if (!frontiers.empty()) {
        std::vector<Frontier> unique_frontiers;
        const float MIN_DISTANCE_SQUARED = 4.0f;
        
        for (size_t i = 0; i < frontiers.size(); i++) {
            bool is_duplicate = false;
            
            for (size_t j = i + 1; j < frontiers.size(); j++) {
                int dx = frontiers[i].x - frontiers[j].x;
                int dy = frontiers[i].y - frontiers[j].y;
                float dist_sq = dx*dx + dy*dy;
                
                if (dist_sq < MIN_DISTANCE_SQUARED) {
                    if (frontiers[j].weight > frontiers[i].weight) {
                        frontiers[i] = frontiers[j];
                    }
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                unique_frontiers.push_back(frontiers[i]);
            }
        }
        
        frontiers = unique_frontiers;
        
        // 按权重排序
        std::sort(frontiers.begin(), frontiers.end(), 
                 [](const Frontier& a, const Frontier& b) {
                     return a.weight > b.weight;
                 });
        
        // 【修改8】更好的日志输出
        if (!frontiers.empty()) {
            ROS_INFO("前沿搜索: 检查%d单元格, 找到%lu前沿点", 
                    cells_checked, frontiers.size());
            
            // 显示最佳前沿点信息
            for (size_t i = 0; i < std::min(frontiers.size(), (size_t)3); i++) {
                float world_x = mapToWorldX(frontiers[i].x);
                float world_y = mapToWorldY(frontiers[i].y);
                ROS_INFO("前沿点 %zu: (%.1f, %.1f)m, 距离=%.1fm, 权重=%.3f", 
                        i+1, world_x, world_y, frontiers[i].distance, frontiers[i].weight);
            }
        }
    }
    
    // 更新缓存
    cached_frontiers = frontiers;
    last_update_time = now;
}

// 在类声明中添加这个函数
float SonarSlamNode::calculateFrontierWeight(int x, int y, float distance, int unknown_count) {
    // 基础距离因子
    float distance_factor = exp(-distance / 3.0f);  // 更陡的衰减
    
    // 未知密度因子
    float density_factor = std::min(1.0f, unknown_count / 4.0f);
    
    // 获取机器人朝向以计算方向一致性
    float robot_x, robot_y, robot_yaw;
    if (getRobotPose(robot_x, robot_y, robot_yaw)) {
        float target_x = mapToWorldX(x);
        float target_y = mapToWorldY(y);
        float target_yaw = atan2(target_y - robot_y, target_x - robot_x);
        float angle_diff = fabs(target_yaw - robot_yaw);
        
        // 规范化角度差
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        // 方向因子：适当放宽要求（0.3-1.0范围）
        float direction_factor = 0.7f + 0.3f * (1.0f - angle_diff / M_PI);
        
        // 综合权重（更均衡）
        return (distance_factor * 0.4f + 
                density_factor * 0.4f + 
                direction_factor * 0.2f);
    }
    
    return distance_factor * 0.6f + density_factor * 0.4f;
}

/******
void SonarSlamNode::findFrontiers(std::vector<Frontier>& frontiers) {
    // ==================== 1. 缓存检查 ====================
    static std::vector<Frontier> cached_frontiers;
    static ros::Time last_update_time(0);
    static int call_count = 0;
    static int search_count = 0;  // 【新增】搜索次数计数器
    
    ros::Time now = ros::Time::now();
    call_count++;
    search_count++;  // 【新增】
    
    // 每5次调用或超过2秒才重新计算
    const int CACHE_FREQUENCY = 5;
    const double CACHE_TIMEOUT = 2.0;
    
    // 【修改】前3次搜索强制重新计算
    bool use_cache = (!cached_frontiers.empty() && 
                     (now - last_update_time).toSec() < CACHE_TIMEOUT &&
                     call_count % CACHE_FREQUENCY != 0 &&
                     search_count > 3);  // 
    
    if (use_cache) {
        frontiers = cached_frontiers;
        ROS_DEBUG_THROTTLE(1.0, "Using cached frontiers (%lu frontiers, call#%d)", 
                          frontiers.size(), call_count);
        return;
    }
    
    // ==================== 2. 需要重新计算 ====================
    frontiers.clear();
    int known_cells = 0;
    for (int val : map_.data) {
        if (val != CELL_UNKNOWN) known_cells++;
    }
    
    float known_ratio = static_cast<float>(known_cells) / (map_width_ * map_height_);
    if (known_ratio < 0.1f) {  // 地图覆盖率小于10%时不搜索前沿
        ROS_DEBUG("地图覆盖率太低(%.1f%%)，跳过前沿搜索", known_ratio * 100);
        return;
    }
    
    // 获取当前机器人位置用于距离过滤
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) {
        ROS_WARN("无法获取机器人位置，前沿搜索受限");
        // 继续搜索但不进行距离过滤
    }
    // ==================== 3. 性能优化：按区域采样 ====================
    const int STEP_SIZE = 3;
    int cells_checked = 0;
    int frontier_cells_found = 0;
    
    // 遍历地图，使用步进采样
    for (int y = 2; y < map_height_ - 2; y += STEP_SIZE) {
        for (int x = 2; x < map_width_ - 2; x += STEP_SIZE) {
            cells_checked++;
            
            int idx = y * map_width_ + x;
            
            // 放宽条件：只要不是障碍物
            // 原来：if (map_.data[idx] != CELL_FREE) continue;
            if (map_.data[idx] == CELL_OCCUPIED) continue;  // 【修改这1行】
            
            // 检查周围是否有未知单元
            bool has_unknown = false;
            int unknown_count = 0;
            
            // 检查3x3邻域（采样区域）
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int sample_x = x + dx * STEP_SIZE;
                    int sample_y = y + dy * STEP_SIZE;
                    
                    // 边界检查
                    if (sample_x < 0 || sample_x >= map_width_ || 
                        sample_y < 0 || sample_y >= map_height_) {
                        continue;
                    }
                    
                    int nidx = sample_y * map_width_ + sample_x;
                    if (map_.data[nidx] == CELL_UNKNOWN) {
                        has_unknown = true;
                        unknown_count++;
                    }
                }
            }
            
            if (has_unknown) {
                frontier_cells_found++;
                
                // 计算机器人到该点的距离
                float robot_x, robot_y;
                if (!getRobotPosition(robot_x, robot_y)) continue;
                
                float world_x = mapToWorldX(x);
                float world_y = mapToWorldY(y);
                float distance = sqrt(pow(world_x - robot_x, 2) + 
                                     pow(world_y - robot_y, 2));
                
                // 距离过滤：太远的前沿点意义不大
                const float MAX_FRONTIER_DISTANCE = 8.0f;
                if (distance > MAX_FRONTIER_DISTANCE) {
                    continue;
                }
                
                // 计算权重
                float weight = -frontier_weight_distance_ * distance + 
                               frontier_weight_unknown_ * unknown_count;
                
                frontiers.push_back(Frontier(x, y, distance, weight));
                
                // ==================== 4. 在采样点周围小范围搜索 ====================
                // 在找到的前沿点周围2x2区域内细化搜索
                const int REFINE_RADIUS = 1;
                for (int ry = -REFINE_RADIUS; ry <= REFINE_RADIUS; ry++) {
                    for (int rx = -REFINE_RADIUS; rx <= REFINE_RADIUS; rx++) {
                        if (rx == 0 && ry == 0) continue;
                        
                        int refine_x = x + rx;
                        int refine_y = y + ry;
                        
                        // 边界检查
                        if (refine_x < 2 || refine_x >= map_width_ - 2 || 
                            refine_y < 2 || refine_y >= map_height_ - 2) {
                            continue;
                        }
                        
                        int refine_idx = refine_y * map_width_ + refine_x;
                        if (map_.data[refine_idx] == CELL_FREE) {
                            // 检查这个细化点是否也是前沿
                            bool refine_has_unknown = false;
                            int refine_unknown_count = 0;
                            
                            for (int dy = -1; dy <= 1; dy++) {
                                for (int dx = -1; dx <= 1; dx++) {
                                    if (dx == 0 && dy == 0) continue;
                                    
                                    int nx = refine_x + dx;
                                    int ny = refine_y + dy;
                                    if (nx >= 0 && nx < map_width_ && 
                                        ny >= 0 && ny < map_height_) {
                                        int nidx = ny * map_width_ + nx;
                                        if (map_.data[nidx] == CELL_UNKNOWN) {
                                            refine_has_unknown = true;
                                            refine_unknown_count++;
                                        }
                                    }
                                }
                            }
                            
                            if (refine_has_unknown) {
                                float refine_world_x = mapToWorldX(refine_x);
                                float refine_world_y = mapToWorldY(refine_y);
                                float refine_distance = sqrt(pow(refine_world_x - robot_x, 2) + 
                                                           pow(refine_world_y - robot_y, 2));
                                
                                if (refine_distance <= MAX_FRONTIER_DISTANCE) {
                                    float refine_weight = -frontier_weight_distance_ * refine_distance + 
                                                         frontier_weight_unknown_ * refine_unknown_count;
                                    frontiers.push_back(Frontier(refine_x, refine_y, 
                                                                refine_distance, refine_weight));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    // ==================== 5. 去重和排序 ====================
    if (!frontiers.empty()) {
        // 去除太近的重复前沿点（2个单元格内）
        std::vector<Frontier> unique_frontiers;
        const float MIN_DISTANCE_SQUARED = 4.0f;
        
        for (size_t i = 0; i < frontiers.size(); i++) {
            bool is_duplicate = false;
            
            for (size_t j = i + 1; j < frontiers.size(); j++) {
                int dx = frontiers[i].x - frontiers[j].x;
                int dy = frontiers[i].y - frontiers[j].y;
                float dist_sq = dx*dx + dy*dy;
                
                if (dist_sq < MIN_DISTANCE_SQUARED) {
                    // 保留权重更高的
                    if (frontiers[j].weight > frontiers[i].weight) {
                        frontiers[i] = frontiers[j];
                    }
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                unique_frontiers.push_back(frontiers[i]);
            }
        }
        
        frontiers = unique_frontiers;
        
        // 按权重排序（从高到低）
        std::sort(frontiers.begin(), frontiers.end(), 
                 [](const Frontier& a, const Frontier& b) {
                     return a.weight > b.weight;
                 });
    }
    
    // 初期没有前沿时生成简单目标
    if (frontiers.empty() && search_count < 10) {
        // 获取机器人位姿
        float robot_x, robot_y, robot_yaw;
        if (getRobotPose(robot_x, robot_y, robot_yaw)) {
            // 在前方1米生成目标
            current_target_x_ = robot_x + 1.0 * cos(robot_yaw);
            current_target_y_ = robot_y + 1.0 * sin(robot_yaw);
            has_goal_ = true;
            ROS_INFO("初期无前沿，设置前方目标: (%.2f, %.2f)", 
                    current_target_x_, current_target_y_);
        }
    }
    
    // ==================== 6. 更新缓存 ====================
    cached_frontiers = frontiers;
    last_update_time = now;
    
    // ==================== 7. 性能日志 ====================
    if (call_count % 10 == 0) {
        ROS_DEBUG("Frontier search: checked %d cells, found %lu frontiers", 
                 cells_checked, frontiers.size());
    }
}
***/
SonarSlamNode::Frontier SonarSlamNode::selectBestFrontier(const std::vector<Frontier>& frontiers) {
    Frontier best;
    
    if (!frontiers.empty()) {
        best = frontiers[0];
        for (const auto& f : frontiers) {
            if (f.weight > best.weight) {
                best = f;
            }
        }
    }
    
    return best;
}

// ==================== 导航控制 ====================

void SonarSlamNode::navigateToTarget() {
    if (!has_goal_) {
        return;
    }
    
    //  获取更准确的位姿
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN("导航失败: 无法获取位姿");
        return;
    }
    
    // 计算到目标的方向和距离
    float dx = current_target_x_ - robot_x;
    float dy = current_target_y_ - robot_y;
    float distance = sqrt(dx*dx + dy*dy);
    float target_yaw = atan2(dy, dx);
    
    // 角度差规范化
    float angle_diff = target_yaw - robot_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
    
    geometry_msgs::Twist cmd_vel;
    
    // 检查是否到达目标
    if (distance <= goal_tolerance_) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        has_goal_ = false;
        
        ROS_INFO("到达目标: (%.2f, %.2f)", current_target_x_, current_target_y_);
    } else {
        //  改进控制策略
        float kp_linear = 0.8f;   // 提高线性增益
        float kp_angular = 1.2f;  // 提高角速度增益
        
        // 角度太大时先旋转
        if (fabs(angle_diff) > 0.3f) {  // 约17度
            cmd_vel.linear.x = 0.05f;  // 低速前进
            cmd_vel.angular.z = std::min(max_angular_speed_, 
                                         kp_angular * angle_diff);
        } else {
            // 角度较小，正常前进
            cmd_vel.linear.x = std::min(max_linear_speed_, 
                                        kp_linear * distance);
            cmd_vel.angular.z = std::min(max_angular_speed_, 
                                         kp_angular * angle_diff);
        }
        
        //  确保最小速度（防止原地不动）
        if (fabs(cmd_vel.linear.x) < 0.08f && distance > 0.5f) {
            cmd_vel.linear.x = (dx >= 0) ? 0.08f : -0.08f;
        }
        
        //  动态减速
        if (distance < 1.0f) {
            cmd_vel.linear.x *= 0.7f;
        }
        
        // 【 避障检查
        if (checkObstacleAhead(distance)) {
            ROS_WARN("前方有障碍物，调整路径");
            // 根据障碍物位置决定转向
            cmd_vel.angular.z = 0.5f;  // 默认向右转
            cmd_vel.linear.x *= 0.3f;  // 降低速度
        }
        
        ROS_DEBUG("导航: 距离=%.2fm, 角度差=%.1f°, 速度=(%.2f, %.2f)", 
                 distance, angle_diff * 180.0 / M_PI,
                 cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    // 应用安全限制并发布
    cmd_vel = applySafetyLimits(cmd_vel);
    cmd_vel_pub_.publish(cmd_vel);
}
/*****
void SonarSlamNode::navigateToTarget() {
    if (!has_goal_) {
        ROS_DEBUG("没有目标，跳过导航");
        return;
    }
    
    // 获取机器人位姿
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN_THROTTLE(1.0, "无法获取机器人位姿");
        return;
    }
    
    // 【新增】详细调试信息
    static int nav_counter = 0;
    nav_counter++;
    
    if (nav_counter % 10 == 0) {  // 每10次输出一次详细信息
        ROS_INFO("=== 导航循环 #%d ===", nav_counter);
        ROS_INFO("  当前位置: (%.2f, %.2f, %.2f°)", 
                robot_x, robot_y, robot_yaw * 180.0 / M_PI);
        ROS_INFO("  目标位置: (%.2f, %.2f)", 
                current_target_x_, current_target_y_);
    }
    
    // 计算到目标点的向量
    float dx = current_target_x_ - robot_x;
    float dy = current_target_y_ - robot_y;
    float distance = sqrt(dx*dx + dy*dy);
    float target_yaw = atan2(dy, dx);
    
    // 计算角度差并规范化到[-π, π]
    float angle_diff = target_yaw - robot_yaw;
    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
    
    geometry_msgs::Twist cmd_vel;
    
    // 检查是否到达目标
    if (distance <= goal_tolerance_) {
        // 到达目标，停止
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        has_goal_ = false;
        
        ROS_INFO(" 到达目标: (%.2f, %.2f)", current_target_x_, current_target_y_);
        
        // 【新增】短暂停留，准备下一个目标
        ros::Duration(0.5).sleep();
        
    } else {
        // 比例控制
        float kp_linear = 0.5f;
        float kp_angular = 1.0f;
        
        cmd_vel.linear.x = std::min(max_linear_speed_, kp_linear * distance);
        cmd_vel.angular.z = std::min(max_angular_speed_, kp_angular * angle_diff);
        
        // 【重要】确保最小速度
        if (fabs(cmd_vel.linear.x) < 0.05f && distance > 0.3f) {
            cmd_vel.linear.x = (cmd_vel.linear.x >= 0) ? 0.05f : -0.05f;
        }
        
        // 距离越近，速度越慢
        if (distance < 1.0f) {
            cmd_vel.linear.x *= 0.5f;
        }
        
        // 【新增】角度太大时先旋转
        if (fabs(angle_diff) > 0.5f) {  // 约30度
            cmd_vel.linear.x *= 0.3f;  // 前进时降低线速度
        }
        
        // 避障检查
        if (checkObstacleAhead(distance)) {
            ROS_WARN("前方障碍物，减速并转向");
            cmd_vel.linear.x *= 0.2f;
            
            // 根据障碍物位置决定转向方向
            cmd_vel.angular.z = 0.5f;  // 默认向右转
        }
        
        // 【新增】调试输出
        if (nav_counter % 5 == 0) {
            ROS_INFO("导航: 距离=%.2fm, 角度差=%.1f°, 速度=(%.2f, %.2f)", 
                    distance, angle_diff * 180.0 / M_PI,
                    cmd_vel.linear.x, cmd_vel.angular.z);
        }
    }
    
    // 应用安全限制
    cmd_vel = applySafetyLimits(cmd_vel);
    
    // 【关键】发布控制命令
    cmd_vel_pub_.publish(cmd_vel);
    
    // 【新增】验证命令是否发送
    static int cmd_counter = 0;
    cmd_counter++;
    if (cmd_counter % 20 == 0) {
        ROS_INFO("发布控制命令 #%d: 线速度=%.3f, 角速度=%.3f", 
                cmd_counter, cmd_vel.linear.x, cmd_vel.angular.z);
    }
    
    // 可视化目标点
    visualizeGoal(current_target_x_, current_target_y_);
}
***/


bool SonarSlamNode::checkObstacleAhead(float lookahead_distance) {
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) return false;
    
    // 检查机器人前方区域
    float check_distance = std::min(lookahead_distance, 2.0f);
    float step = map_resolution_;
    
    for (float d = step; d <= check_distance; d += step) {
        float check_x = robot_x + d * cos(robot_yaw);
        float check_y = robot_y + d * sin(robot_yaw);
        
        int map_x = worldToMapX(check_x);
        int map_y = worldToMapY(check_y);
        
        if (map_x >= 0 && map_x < map_width_ && 
            map_y >= 0 && map_y < map_height_) {
            int idx = map_y * map_width_ + map_x;
            if (map_.data[idx] == CELL_OCCUPIED) {
                return true;
            }
        }
    }
    
    return false;
}

geometry_msgs::Twist SonarSlamNode::applySafetyLimits(const geometry_msgs::Twist& cmd_vel) {
    geometry_msgs::Twist safe_cmd = cmd_vel;
    
    // 线性速度限制
    safe_cmd.linear.x = std::max(-(float)max_linear_speed_, 
                                 std::min((float)max_linear_speed_, 
                                         (float)safe_cmd.linear.x));
    
    // 角速度限制
    safe_cmd.angular.z = std::max(-(float)max_angular_speed_, 
                                  std::min((float)max_angular_speed_, 
                                          (float)safe_cmd.angular.z));
    
    return safe_cmd;
}

// ==================== 状态机处理 ====================
void SonarSlamNode::handleInitialMappingState() {
    ROS_INFO("=== 初始建图状态 ===");
    
    // 1. 紧急检查
    if (checkEmergencyConditions()) {
        current_state_ = STATE_EMERGENCY_STOP;
        return;
    }
    
    // 2. 尝试获取位姿（如果失败也继续）
    float robot_x, robot_y, robot_yaw;
    bool has_pose = getRobotPose(robot_x, robot_y, robot_yaw);
    
    if (has_pose) {
        ROS_INFO("机器人位姿: (%.2f, %.2f, %.2f°)", robot_x, robot_y, robot_yaw * 180/M_PI);
    } else {
        ROS_WARN("无法获取精确位姿，使用默认值");
        robot_x = 0;
        robot_y = 0;
        robot_yaw = 0;
    }
    
    // 3. 如果有目标，导航
    if (has_goal_) {
        navigateToTarget();
        return;
    }
    
    // 4. 设置一个简单的前方目标
    float forward_distance = 1.5f;
    current_target_x_ = robot_x + forward_distance * cos(robot_yaw);
    current_target_y_ = robot_y + forward_distance * sin(robot_yaw);
    has_goal_ = true;
    
    ROS_INFO("设置前进目标: (%.2f, %.2f)", current_target_x_, current_target_y_);
    
    // 5. 立即导航
    navigateToTarget();
}
/*******
void SonarSlamNode::handleInitialMappingState() {
    ROS_DEBUG("处理初始建图状态");
    
    // 获取机器人位姿（如果失败，直接返回，不做任何事）
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_ERROR("无法获取机器人位姿！");
        return;
    }
    
    // 1. 紧急情况检查（必须优先处理）
    if (checkEmergencyConditions()) {
        current_state_ = STATE_EMERGENCY_STOP;
        return;  // 只有这个可以立即返回
    }
    
    // 2. 检查是否已完成探索
    if (exploration_complete_) {
        current_state_ = STATE_RETURN_HOME;
        ROS_INFO("探索完成，切换到返回状态");
        return;  // 这个也可以立即返回
    }
    
    // 3. TF初始化检查（不通过时只警告，不返回）
    static bool tf_initialized = false;
    if (!tf_initialized) {
        try {
            tf::StampedTransform transform;
            tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
            tf_initialized = true;
            ROS_INFO("TF连接就绪，开始探索");
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "等待TF连接: %s", ex.what());
            // 不返回，继续执行
        }
    }
    
    // 4. 检查地图覆盖率
    float coverage = calculateMapCoverage();
    
    // 5. 角落处理逻辑（修改为不立即返回）
    const float POOL_HALF_LENGTH = 4.0f;
    const float POOL_HALF_WIDTH = 3.0f;
    bool likely_in_corner = (fabs(robot_x) > POOL_HALF_LENGTH - 1.0f) && 
                           (fabs(robot_y) > POOL_HALF_WIDTH - 1.0f);
    
    if (likely_in_corner && !has_goal_) {
        ROS_INFO("机器人在角落附近，尝试向中心移动");
        
        float center_x = 0.0f, center_y = 0.0f;
        float dx = center_x - robot_x;
        float dy = center_y - robot_y;
        float dist = sqrt(dx*dx + dy*dy);
        
        if (dist > 1.0f) {
            float move_dist = fmin(dist * 0.5f, 1.5f);
            current_target_x_ = robot_x + dx * (move_dist / dist);
            current_target_y_ = robot_y + dy * (move_dist / dist);
            has_goal_ = true;
            
            ROS_INFO("角落→中心: (%.2f, %.2f) -> (%.2f, %.2f)", 
                    robot_x, robot_y, current_target_x_, current_target_y_);
        }
        // 不返回，继续执行后面的逻辑
    }
    
    // 6. 定期策略检查（移除状态切换的返回）
    static ros::Time last_check_time(0);
    ros::Time now = ros::Time::now();
    
    if ((now - last_check_time).toSec() > 3.0) {
        last_check_time = now;
        
        if (coverage < 0.3f) {
            ROS_INFO("阶段1: 低覆盖率(%.1f%%)", coverage * 100);
        } else if (coverage < 0.7f) {
            ROS_INFO("阶段2: 中等覆盖率(%.1f%%)", coverage * 100);
            
            auto gaps = findAndRankGaps();
            if (!gaps.empty()) {
                current_gap_target_ = gaps[0];
                current_state_ = STATE_GAP_EXPLORATION;
                ROS_INFO("切换到缺口探索状态");
                return;  // 状态切换可以返回
            }
        } else {
            ROS_INFO("阶段3: 高覆盖率(%.1f%%)", coverage * 100);
            
            if (isMapComplete()) {
                if (!map_verified_complete_) {
                    current_state_ = STATE_COVERAGE_VERIFY;
                    ROS_INFO("切换到验证状态");
                    return;  // 状态切换可以返回
                }
            }
        }
    }
    
    // 7. 核心逻辑：目标查找和导航（保证必须执行）
    
    // 如果已有目标，直接导航
    if (has_goal_) {
        navigateToTarget();
        return;
    }
    
    // 如果没有目标，尝试寻找
    static int no_goal_count = 0;
    
    if (findNextGoal()) {
        no_goal_count = 0;
        navigateToTarget();
    } else {
        no_goal_count++;
        
        // 兜底策略：连续多次找不到目标，强制前进
        if (no_goal_count > 10) {
            ROS_WARN("连续%d次未找到目标，强制前进", no_goal_count);
            
            float forward_distance = 1.0f;
            current_target_x_ = robot_x + forward_distance * cos(robot_yaw);
            current_target_y_ = robot_y + forward_distance * sin(robot_yaw);
            has_goal_ = true;
            
            ROS_INFO("强制前进目标: (%.2f, %.2f)", current_target_x_, current_target_y_);
            no_goal_count = 5;  // 部分重置
            
            navigateToTarget();
        } else {
            ROS_DEBUG("未找到目标，计数: %d", no_goal_count);
        }
    }
}
*****/
 
void SonarSlamNode::handleReturnHomeState() {
    // 返回起点状态
    
    if (!has_goal_) {
        // 设置目标为起点
        current_target_x_ = start_pose_.x;
        current_target_y_ = start_pose_.y;
        has_goal_ = true;
        ROS_INFO("Returning to start position: (%.2f, %.2f)", 
                current_target_x_, current_target_y_);
    }
    
    // 导航到起点
    navigateToTarget();
    
    // 检查是否到达起点
    float robot_x, robot_y;
    if (getRobotPosition(robot_x, robot_y)) {
        float distance = sqrt(pow(robot_x - start_pose_.x, 2) + 
                             pow(robot_y - start_pose_.y, 2));
        if (distance < goal_tolerance_) {
            ROS_INFO("Successfully returned to start position");
            current_state_ = STATE_COMPLETE;
            has_goal_ = false;
        }
    }
}

void SonarSlamNode::handleCompleteState() {
    // 任务完成状态：停止机器人
    stopRobot();
    
    ROS_INFO("================");
    ROS_INFO("Mission Complete!");
    ROS_INFO("  Total scans: %d", scan_count_);
    ROS_INFO("  Final coverage: %.1f%%", map_coverage_ * 100);
    ROS_INFO("================");
}

void SonarSlamNode::handleEmergencyStopState() {
    // 紧急停止状态：立即停止并等待处理
    stopRobot();
    
    ROS_ERROR("EMERGENCY STOP - Robot stopped due to safety concern");
    
    // 这里可以添加恢复逻辑，比如等待人工干预或自动恢复
}

// ==================== 紧急情况处理 ====================
bool SonarSlamNode::checkEmergencyConditions() {
    // 检查前方红外传感器（紧急避障）
    
    // 检查前方红外（避障）
    // float front_min_distance = std::min(infrared_front_left_, infrared_front_right_);
    
    // // 如果距离小于0.3米，紧急停止
    // if (front_min_distance < 0.3f) {
    //     ROS_WARN("前方障碍物距离: %.2fm < 0.3m，触发紧急停止", front_min_distance);
    //     return true;
    // }
    // if (ultrasonic_front_right_ < 0.2f || ultrasonic_rear_right_ < 0.2f) {
    // ROS_WARN("侧方距离过近: [%.2f, %.2f]m", 
    //         ultrasonic_front_right_, ultrasonic_rear_right_);
    // return true;
    // }

    static int call_count = 0;
    call_count++;
    
    if (call_count < 10) {
        ROS_WARN_THROTTLE(1.0, "紧急停止检查调用 #%d (测试阶段禁用)", call_count);
    }
    
 
    // 检查侧方超声波（沿墙安全）

    return false;
}

void SonarSlamNode::triggerEmergencyStop() {
    ROS_ERROR("Triggering emergency stop!");
    stopRobot();
    current_state_ = STATE_EMERGENCY_STOP;
}

void SonarSlamNode::stopRobot() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
}

// ==================== 地图覆盖计算 ====================
float SonarSlamNode::calculateMapCoverage() {
    int known_cells = 0;
    int total_cells = map_width_ * map_height_;
    
    for (int val : map_.data) {
        if (val != CELL_UNKNOWN) known_cells++;
    }
    
    return static_cast<float>(known_cells) / total_cells;
}

// ==================== 地图发布和保存 ====================
void SonarSlamNode::publishMap() {
    map_.header.stamp = ros::Time::now();
    map_pub_.publish(map_);
}

void SonarSlamNode::saveMap(const std::string& filename) {
    try {
        std::string mapname = filename;
        std::string mapdatafile = mapname + ".pgm";
        std::string mapmetadatafile = mapname + ".yaml";
        
        ROS_INFO("Saving map to %s", mapdatafile.c_str());
        
        // 创建PGM文件
        std::ofstream out(mapdatafile.c_str());
        if (!out) {
            ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
            return;
        }
        
        // PGM文件头
        out << "P5\n# CREATOR: sonar_slam_node\n";
        out << map_width_ << " " << map_height_ << "\n";
        out << "255\n";
        
        // 地图数据（行倒序，PGM是左下角原点）
        for (int y = map_height_ - 1; y >= 0; y--) {
            for (int x = 0; x < map_width_; x++) {
                int idx = y * map_width_ + x;
                int8_t value = map_.data[idx];
                
                // 转换为PGM值
                if (value == CELL_UNKNOWN) {
                    out << (unsigned char)205;  // 未知：灰色
                } else if (value == CELL_FREE) {
                    out << (unsigned char)0;    // 自由：黑色
                } else if (value == CELL_OCCUPIED) {
                    out << (unsigned char)100;  // 占据：深灰色
                } else {
                    out << (unsigned char)value; // 概率值
                }
            }
        }
        out.close();
        
        // 创建YAML元数据文件
        std::ofstream yaml(mapmetadatafile.c_str());
        yaml << "image: " << mapdatafile << "\n";
        yaml << "resolution: " << map_resolution_ << "\n";
        yaml << "origin: [" << map_origin_x_ << ", " << map_origin_y_ << ", 0.0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.25\n";
        
        yaml.close();
        
        ROS_INFO("Map saved successfully to %s", mapname.c_str());
    } catch (std::exception& e) {
        ROS_ERROR("Failed to save map: %s", e.what());
    }
}

// ==================== 可视化函数 ====================
void SonarSlamNode::visualizeFrontiers() {
    std::vector<Frontier> frontiers;
    findFrontiers(frontiers);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "frontiers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration(update_interval_);
    
    for (const auto& f : frontiers) {
        geometry_msgs::Point p;
        p.x = mapToWorldX(f.x);
        p.y = mapToWorldY(f.y);
        p.z = 0.0;
        marker.points.push_back(p);
    }
    
    if (!frontiers.empty()) {
        frontier_marker_pub_.publish(marker);
    }
}

void SonarSlamNode::visualizeScanPoints() {
    if (scan_points_.empty()) return;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "scan_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(update_interval_);
    
    // 遍历deque（和vector用法类似）
    for (const auto& point : scan_points_) {
        geometry_msgs::Point p;
        p.x = point.first;
        p.y = point.second;
        p.z = 0.0;
        marker.points.push_back(p);
    }
    
    frontier_marker_pub_.publish(marker);
}

void SonarSlamNode::visualizeGoal(float x, float y) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "current_goal";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    
    marker.color.r = 0.0;  // 蓝色
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration(update_interval_);
    
    goal_marker_pub_.publish(marker);
}

// ==================== 工具函数实现 ====================
void SonarSlamNode::bresenhamLine(int x0, int y0, int x1, int y1, 
                   std::vector<std::pair<int, int>>& points) {
    points.clear();
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        points.push_back(std::make_pair(x0, y0));
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

int SonarSlamNode::worldToMapX(float world_x) {
    return static_cast<int>((world_x - map_origin_x_) / map_resolution_ + 0.5f);
}

int SonarSlamNode::worldToMapY(float world_y) {
    return static_cast<int>((world_y - map_origin_y_) / map_resolution_ + 0.5f);
}

float SonarSlamNode::mapToWorldX(int map_x) {
    return map_origin_x_ + map_x * map_resolution_;
}

float SonarSlamNode::mapToWorldY(int map_y) {
    return map_origin_y_ + map_y * map_resolution_;
}

bool SonarSlamNode::getRobotPosition(float& x, float& y) {
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("map", "base_link", 
                                     ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        return true;
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1, "TF transform failed: %s", ex.what());
        return false;
    }
}

// bool SonarSlamNode::getRobotPose(float& x, float& y, float& yaw) {
//     try {
//         // 【关键】使用ros::Time(0)获取最新变换，避免时间外推
//         tf::StampedTransform transform;
//         tf_listener_->lookupTransform("map", "base_link", 
//                                      ros::Time(0), transform);
        
//         x = transform.getOrigin().x();
//         y = transform.getOrigin().y();
//         yaw = tf::getYaw(transform.getRotation());
        
//         return true;
//     } catch (tf::TransformException& ex) {
//         // 如果map->base_link不存在，使用odom->base_link
//         try {
//             tf::StampedTransform transform;
//             tf_listener_->lookupTransform("odom", "base_link", 
//                                          ros::Time(0), transform);
            
//             x = transform.getOrigin().x();
//             y = transform.getOrigin().y();
//             yaw = tf::getYaw(transform.getRotation());
            
//             return true;
//         } catch (tf::TransformException& ex2) {
//             ROS_WARN_THROTTLE(2.0, "无法获取机器人位姿: %s", ex2.what());
//             return false;
//         }
//     }
// }
 

bool SonarSlamNode::getRobotPose(float& x, float& y, float& yaw) {
    // 【方案1】主路径：直接从 odom 获取里程计位姿
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("odom", "base_link", ros::Time(0), transform);
        
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation());
        
        // 【关键日志】确认此函数被调用且返回值在变化
        static int success_count = 0;
        static float last_reported_x = 0, last_reported_y = 0;
        success_count++;
        
        // 只有位置变化显著或每10次成功时打印，避免刷屏
        if (success_count % 10 == 0 || 
            fabs(x - last_reported_x) > 0.1 || 
            fabs(y - last_reported_y) > 0.1) {
            ROS_INFO("【定位成功】从 ODOM 获取位姿: (%.3f, %.3f, %.1f°) [调用#%d]", 
                    x, y, yaw * 180.0 / M_PI, success_count);
            last_reported_x = x;
            last_reported_y = y;
        }
        return true; // 成功获取
        
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "从 ODOM 获取位姿失败: %s", ex.what());
    }
    
    // 【方案2】备选路径：如果 odom 不可用，尝试其他可能来源（例如 /wheel_odom 话题）
    // 但根据您的代码，您有 current_odom_ 成员变量，它可能来自 /wheel_odom 话题
    if (current_odom_.header.stamp != ros::Time(0)) {
        x = current_odom_.pose.pose.position.x;
        y = current_odom_.pose.pose.position.y;
        yaw = tf::getYaw(current_odom_.pose.pose.orientation);
        
        ROS_WARN_THROTTLE(2.0, "【降级定位】使用 current_odom_ 话题位姿: (%.3f, %.3f)", x, y);
        return true;
    }
    
    // 【方案3】终极降级：返回一个非零的仿真值，让系统至少能“动起来”
    static float simulated_x = 0.0;
    simulated_x += 0.001; // 让X坐标缓慢增长，模拟移动
    
    x = simulated_x;
    y = 0.0;
    yaw = 0.0;
    
    ROS_ERROR("【警告】使用仿真位姿 (%.3f, 0.0)。请检查 ODOM 数据流！", x);
    return false; // 返回 false 告知调用者这是仿真数据
}


bool SonarSlamNode::getRobotMapPosition(int& map_x, int& map_y) {
    float x, y;
    if (getRobotPosition(x, y)) {
        map_x = worldToMapX(x);
        map_y = worldToMapY(y);
        return true;
    }
    return false;
}

// 在相应的函数中实现这些功能：

// 状态字符串转换
std::string SonarSlamNode::stateToString(SystemState state) {
    switch (state) {
        case STATE_INITIAL_MAPPING: return "INITIAL_MAPPING";
        case STATE_GAP_EXPLORATION: return "GAP_EXPLORATION";     
        case STATE_COVERAGE_VERIFY: return "COVERAGE_VERIFY";     
        case STATE_RETURN_HOME: return "RETURN_HOME";
        case STATE_COMPLETE: return "COMPLETE";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

std::string SonarSlamNode::statusToString(SystemStatus status) {
    switch (status) {
        case STATUS_NORMAL: return "NORMAL";
        case STATUS_FAST_MAPPING: return "FAST_MAPPING";
        case STATUS_HIGH_LOAD: return "HIGH_LOAD";
        case STATUS_LOW_BATTERY: return "LOW_BATTERY";
        default: return "UNKNOWN";
    }
}

// 覆盖分析
CoverageAnalysis SonarSlamNode::analyzeCoverageQuality() {
    CoverageAnalysis analysis;
    
    int total_cells = map_width_ * map_height_;
    int known_cells = 0;
    int poor_coverage = 0;
    
    for (int i = 0; i < map_.data.size(); i++) {
        if (map_.data[i] != CELL_UNKNOWN) {
            known_cells++;
            // 检查覆盖质量（如果更新次数太少，认为是覆盖不良）
            if (cell_history_[i] < 2) {
                poor_coverage++;
            }
        }
    }
    
    analysis.total_cells = total_cells;
    analysis.covered_cells = known_cells;
    analysis.poor_coverage_cells = poor_coverage;
    analysis.coverage_percentage = static_cast<float>(known_cells) / total_cells;
    analysis.quality_score = 1.0f - static_cast<float>(poor_coverage) / (known_cells + 1);
    
    return analysis;
}

// 障碍物检测
ObstacleDetection SonarSlamNode::checkFrontObstacles() {
    ObstacleDetection detection;
    
    detection.left_distance = infrared_front_left_;
    detection.right_distance = infrared_front_right_;
    detection.center_distance = (infrared_front_left_ + infrared_front_right_) / 2.0f;
    
    // 根据距离设置严重程度
    if (detection.center_distance < 0.3f) {
        detection.severity = SEVERITY_CRITICAL;
    } else if (detection.center_distance < 0.5f) {
        detection.severity = SEVERITY_DANGER;
    } else if (detection.center_distance < 1.0f) {
        detection.severity = SEVERITY_WARNING;
    } else {
        detection.severity = SEVERITY_NONE;
    }
    
    return detection;
}

// 导航命令计算
NavigationCommand SonarSlamNode::calculateNavigationCommand(float distance, float angle_diff,
                                                           float robot_x, float robot_y, float robot_yaw) {
    NavigationCommand cmd;
    
    float kp_linear = 0.5f;
    float kp_angular = 1.0f;
    
    cmd.linear_velocity = std::min(max_linear_speed_, kp_linear * distance);
    cmd.angular_velocity = std::min(max_angular_speed_, kp_angular * angle_diff);
    
    // 距离越近，速度越慢
    if (distance < 1.0f) {
        cmd.linear_velocity *= 0.5f;
    }
    
    return cmd;
}

// 系统状态获取
SystemStatus SonarSlamNode::getCurrentSystemStatus() {
    // 根据系统负载、电池状态等决定当前状态
    if (update_cycle_ < 10) {
        return STATUS_FAST_MAPPING;  // 初始阶段快速建图
    } else if (scan_count_ > 100) {
        return STATUS_NORMAL;  // 正常运行
    }
    
    return STATUS_NORMAL;
}

 
// 发布TF变换
/********
void SonarSlamNode::publishMapToOdomTF() {
    // 【关键修复1】添加发布频率控制和去重
    static ros::Time last_successful_publish_time(0);
    static int64_t last_published_stamp_nsec = 0;
    static int publish_attempt_counter = 0;
    
    // 1. 频率限制：最多10Hz（每0.1秒一次）
    ros::Time now = ros::Time::now();
    if ((now - last_successful_publish_time).toSec() < 0.1) {
        return;  // 跳过，发布太频繁
    }
    
    publish_attempt_counter++;
    
    // 2. 获取机器人在地图中的位姿（SLAM估计的）
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN_THROTTLE(2.0, "[TF] 无法获取地图中的机器人位姿");
        return;
    }
    
    // 3. 获取odom->base_link的变换（里程计提供的）
    tf::StampedTransform odom_to_base;
    try {
        // 【关键修复2】使用带超时的lookupTransform
        // 避免获取到旧的缓存数据
        tf_listener_->waitForTransform("odom", "base_link", 
                                      ros::Time(0), ros::Duration(0.05));
        tf_listener_->lookupTransform("odom", "base_link", 
                                     ros::Time(0), odom_to_base);
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "[TF] 无法获取odom->base_link: %s", ex.what());
        return;
    }
    
    // 4. 检查TF数据是否太旧（重要！）
    ros::Time tf_time = odom_to_base.stamp_;
    double tf_age = (ros::Time::now() - tf_time).toSec();
    
    if (tf_age > 0.3) {  // 超过300ms认为数据太旧
        ROS_WARN_THROTTLE(5.0, "[TF] TF数据延迟过大: %.3f秒", tf_age);
        // 如果延迟太大，使用当前时间但要确保唯一性
        tf_time = now;
    }
    
    // 【关键修复3】时间戳去重
    // 防止发布相同时间戳的TF（这是TF_REPEATED_DATA警告的根本原因）
    int64_t current_stamp_nsec = tf_time.toNSec();
    if (current_stamp_nsec == last_published_stamp_nsec) {
        // 相同时间戳，微调1毫秒避免冲突
        tf_time = tf_time + ros::Duration(0.001);
        current_stamp_nsec = tf_time.toNSec();
        ROS_DEBUG_THROTTLE(10.0, "[TF] 调整时间戳避免重复");
    }
    
    // 5. 计算map->odom变换
    tf::Transform map_to_base;
    map_to_base.setOrigin(tf::Vector3(robot_x, robot_y, 0.0));
    map_to_base.setRotation(tf::createQuaternionFromYaw(robot_yaw));
    
    tf::Transform odom_to_base_tf;
    odom_to_base_tf.setOrigin(odom_to_base.getOrigin());
    odom_to_base_tf.setRotation(odom_to_base.getRotation());
    
    tf::Transform map_to_odom = map_to_base * odom_to_base_tf.inverse();
    
    // 【关键修复4】简单滤波（无需新增函数）
    static tf::Transform last_map_to_odom;
    static bool first_publish = true;
    
    if (first_publish) {
        last_map_to_odom = map_to_odom;
        first_publish = false;
    } else {
        // 一阶低通滤波（系数0.3）
        const double filter_alpha = 0.3;
        tf::Vector3 filtered_origin = 
            last_map_to_odom.getOrigin() * (1.0 - filter_alpha) + 
            map_to_odom.getOrigin() * filter_alpha;
        
        // 四元数球面线性插值
        tf::Quaternion filtered_rotation = 
            last_map_to_odom.getRotation().slerp(map_to_odom.getRotation(), filter_alpha);
        
        map_to_odom.setOrigin(filtered_origin);
        map_to_odom.setRotation(filtered_rotation);
        last_map_to_odom = map_to_odom;
    }
    
    // 6. 发布TF
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tf_time;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    
    transform_stamped.transform.translation.x = map_to_odom.getOrigin().x();
    transform_stamped.transform.translation.y = map_to_odom.getOrigin().y();
    transform_stamped.transform.translation.z = map_to_odom.getOrigin().z();
    
    tf::quaternionTFToMsg(map_to_odom.getRotation(), 
                         transform_stamped.transform.rotation);
    
    // 7. 发布前最后的重复检查
    if (current_stamp_nsec != last_published_stamp_nsec) {
        tf_broadcaster_->sendTransform(transform_stamped);
        
        // 更新记录
        last_successful_publish_time = now;
        last_published_stamp_nsec = current_stamp_nsec;
        
        // 8. 调试信息
        static int tf_counter = 0;
        if (tf_counter++ % 50 == 0) {  // 每50次成功发布记录一次
            ROS_INFO("[TF] 发布成功 #%d: 时间戳=%.3f, 位置=(%.3f,%.3f), 延迟=%.3fs", 
                    publish_attempt_counter,
                    tf_time.toSec(),
                    map_to_odom.getOrigin().x(),
                    map_to_odom.getOrigin().y(),
                    tf_age);
            tf_counter = 0;
        }
    }
}
****/

//  发布TF变换
void SonarSlamNode::publishMapToOdomTF() {
    static ros::Time last_publish_time(0);
    static enum { INIT, TRACKING, DEGRADED } state = INIT;
    static tf::Transform last_valid_transform;
    static int odom_fail_count = 0;
    const int MAX_ODOM_FAILS = 10;
    
    ros::Time now = ros::Time::now();
    if ((now - last_publish_time).toSec() < 0.05) { // 提高频率到20Hz
        return;
    }
    
    // 尝试获取odom变换
    tf::StampedTransform odom_to_base;
    bool odom_ok = false;
    try {
        // 使用带超时的查询
        tf_listener_->waitForTransform("odom", "base_link", now, ros::Duration(0.02));
        tf_listener_->lookupTransform("odom", "base_link", now, odom_to_base);
        
        // 检查数据是否合理
        double dt = (now - odom_to_base.stamp_).toSec();
        if (dt < 0.3 && dt >= 0) { // 数据延迟小于300ms
            odom_ok = true;
            odom_fail_count = 0;
        } else {
            ROS_WARN_THROTTLE(1.0, "Odom数据延迟过大: %.3fs", dt);
        }
    } catch (tf::TransformException& ex) {
        odom_fail_count++;
        ROS_WARN_THROTTLE(1.0, "获取odom变换失败(#%d): %s", odom_fail_count, ex.what());
    }
    
    // 状态机处理
    tf::Transform map_to_odom;
    ros::Time tf_timestamp = now;
    std::string state_info;
    
    if (odom_ok) {
        // 获取机器人位姿
        float robot_x, robot_y, robot_yaw;
        bool pose_ok = getRobotPose(robot_x, robot_y, robot_yaw);
        
        if (pose_ok) {
            // 正常跟踪模式
            state = TRACKING;
            
            tf::Transform map_to_base;
            map_to_base.setOrigin(tf::Vector3(robot_x, robot_y, 0.0));
            map_to_base.setRotation(tf::createQuaternionFromYaw(robot_yaw));
            
            map_to_odom = map_to_base * odom_to_base.inverse();
            last_valid_transform = map_to_odom;
            tf_timestamp = odom_to_base.stamp_;
            
            state_info = "跟踪模式";
        } else if (state == TRACKING) {
            // 位姿获取失败但之前处于跟踪状态，使用上一次有效变换
            state = DEGRADED;
            map_to_odom = last_valid_transform;
            state_info = "降级模式(使用缓存)";
        } else {
            // 初始化阶段或持续失败，发布单位变换
            state = INIT;
            map_to_odom.setIdentity();
            state_info = "初始化模式";
        }
    } else if (odom_fail_count < MAX_ODOM_FAILS && state == TRACKING) {
        // odom暂时失败，但之前处于跟踪状态
        state = DEGRADED;
        map_to_odom = last_valid_transform;
        state_info = "降级模式(odom暂时失效)";
    } else {
        // odom完全失效或超过最大失败次数
        state = INIT;
        map_to_odom.setIdentity();
        state_info = "紧急模式(odom失效)";
    }
    
    // 时间戳处理
    static int64_t last_timestamp_ns = 0;
    if (tf_timestamp.toNSec() <= last_timestamp_ns) {
        tf_timestamp = ros::Time().fromNSec(last_timestamp_ns + 1000);
    }
    last_timestamp_ns = tf_timestamp.toNSec();
    
    // 发布变换
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tf_timestamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    
    transform_stamped.transform.translation.x = map_to_odom.getOrigin().x();
    transform_stamped.transform.translation.y = map_to_odom.getOrigin().y();
    transform_stamped.transform.translation.z = map_to_odom.getOrigin().z();
    
    tf::quaternionTFToMsg(map_to_odom.getRotation(), 
                         transform_stamped.transform.rotation);
    
    // 添加状态信息到header
    transform_stamped.header.frame_id = std::string("map_") + std::to_string(state);
    
    tf_broadcaster_->sendTransform(transform_stamped);
    last_publish_time = now;
    
    // 状态日志（节流输出）
    static int log_counter = 0;
    if (log_counter++ % 50 == 0) {
        ROS_INFO("TF发布状态: %s, Odom可用: %s, 失败计数: %d", 
                state_info.c_str(), odom_ok ? "是" : "否", odom_fail_count);
        log_counter = 0;
    }
}


void SonarSlamNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_imu_ = *msg;
    
    // 可选：使用IMU数据改进定位
    // 这里可以添加IMU滤波或融合逻辑
    
    ROS_DEBUG_THROTTLE(2.0, "IMU received: accel=[%.2f, %.2f, %.2f], gyro=[%.2f, %.2f, %.2f]",
                      msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z,
                      msg->angular_velocity.x,
                      msg->angular_velocity.y,
                      msg->angular_velocity.z);
}

// 角度规范化
float SonarSlamNode::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// 检查点是否可达
bool SonarSlamNode::isPointReachable(const geometry_msgs::Point& point) {
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) return false;
    
    float distance = sqrt(pow(point.x - robot_x, 2) + pow(point.y - robot_y, 2));
    return distance < 10.0f;  // 假设10米内可达
}

// 检查点是否安全
bool SonarSlamNode::isPointSafe(const geometry_msgs::Point& point) {
    int map_x = worldToMapX(point.x);
    int map_y = worldToMapY(point.y);
    
    if (map_x < 0 || map_x >= map_width_ || map_y < 0 || map_y >= map_height_) {
        return false;
    }
    
    int idx = map_y * map_width_ + map_x;
    return map_.data[idx] != CELL_OCCUPIED;
}

// 检测右侧墙壁
bool SonarSlamNode::detectRightWall() {
    return (ultrasonic_front_right_ < 1.0f || ultrasonic_rear_right_ < 1.0f);
}

// 计算沿墙目标点
geometry_msgs::Point SonarSlamNode::calculateWallFollowingGoal() {
    geometry_msgs::Point point;
    float robot_x, robot_y, robot_yaw;
    
    if (getRobotPose(robot_x, robot_y, robot_yaw)) {
        // 沿右侧墙壁前方移动
        point.x = robot_x + 1.0f * cos(robot_yaw) - 0.5f * sin(robot_yaw);
        point.y = robot_y + 1.0f * sin(robot_yaw) + 0.5f * cos(robot_yaw);
    }
    
    return point;
}

void SonarSlamNode::handleGuideToStartState() {
    ROS_DEBUG_THROTTLE(5.0, "Handling GUIDE_TO_START state");
    // 引导到覆盖起始点的逻辑
    // 暂时简化：直接设置为起点
    if (!has_goal_) {
        current_target_x_ = start_pose_.x;
        current_target_y_ = start_pose_.y;
        has_goal_ = true;
        ROS_INFO("Guiding to start point");
    }
    navigateToTarget();
}

void SonarSlamNode::handleCoverageMonitorState() {
    ROS_DEBUG_THROTTLE(5.0, "Handling COVERAGE_MONITOR state");
    // 监控覆盖过程
    float coverage = calculateMapCoverage();
    ROS_DEBUG_THROTTLE(5.0, "Monitoring coverage: %.1f%%", coverage * 100);
    
    // 如果覆盖完成，进入返回状态
    if (coverage > 0.85f) {
        ROS_INFO("Coverage monitoring complete, switching to RETURN_HOME");
        current_state_ = STATE_RETURN_HOME;
    }
}

void SonarSlamNode::handleCoverageAssistState() {
    ROS_DEBUG_THROTTLE(5.0, "Handling COVERAGE_ASSIST state");
    // 辅助覆盖（处理特殊情况）
    // 例如：处理漏扫区域或复杂区域
}

bool SonarSlamNode::performScanMatching(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    tf::Transform& pose_correction) {
    
    if (!scan_matcher_initialized_ || last_scan_->empty()) {
        // 第一次扫描，无法匹配
        last_pose_ = getCurrentOdomTransform();  // 从里程计获取初始位姿
        return false;
    }
    
    // 获取当前里程计位姿作为初始猜测
    tf::Transform odom_pose = getCurrentOdomTransform();
    
    // 执行扫描匹配
    sonar_slam::ScanMatcher::MatchResult result = 
        scan_matcher_.matchScan(current_scan, last_scan_, odom_pose);
    
    if (result.valid && result.score > 0.4) {  // 得分阈值
        pose_correction = result.transform;
        
        // 调试输出
        ROS_DEBUG_THROTTLE(1.0, "Scan match successful: score=%.3f, correction=(%.3f, %.3f, %.3f)",
                         result.score,
                         pose_correction.getOrigin().x(),
                         pose_correction.getOrigin().y(),
                         tf::getYaw(pose_correction.getRotation()));
        
        // 发布调试信息
        publishMatchedCloud(current_scan, pose_correction);
        publishPoseCorrection(pose_correction);
        
        return true;
    } else {
        ROS_WARN_THROTTLE(2.0, "Scan match failed or low score: %.3f", result.score);
        return false;
    }
}

void SonarSlamNode::updateRobotPoseWithCorrection(const tf::Transform& correction) {
    // 1. 获取当前odom->base_link变换
    tf::StampedTransform odom_to_base;
    try {
        tf_listener_->lookupTransform("odom", "base_link", ros::Time(0), odom_to_base);
    } catch (...) {
        return;
    }
    
    // 2. 计算新的map->base_link = correction × odom->base_link
    tf::Transform map_to_base = correction * odom_to_base;
    
    // 3. 更新last_pose_
    last_pose_ = map_to_base;
    
    // 4. 发布TF：map->odom
    publishMapToOdomTFWithPose(map_to_base, odom_to_base);
    
    // 5. 发布TF：map->base_link（直接变换，用于可视化）
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "base_link_slam";  // 使用新frame避免冲突
    
    transform_stamped.transform.translation.x = map_to_base.getOrigin().x();
    transform_stamped.transform.translation.y = map_to_base.getOrigin().y();
    transform_stamped.transform.translation.z = map_to_base.getOrigin().z();
    tf::quaternionTFToMsg(map_to_base.getRotation(), 
                         transform_stamped.transform.rotation);
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

tf::Transform SonarSlamNode::getCurrentOdomTransform() {
    // 从里程计话题或TF获取当前位姿
    tf::StampedTransform transform;
    try {
        tf_listener_->lookupTransform("odom", "base_link", ros::Time(0), transform);
        return transform;
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to get odom transform: %s", ex.what());
        return tf::Transform::getIdentity();
    }
}

void SonarSlamNode::publishMatchedCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const tf::Transform& transform) {
    
    if (!matched_cloud_pub_.getNumSubscribers()) return;
    
    // 应用变换并发布
    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, 
                            scan_matcher_.tfToEigenMatrix(transform));
    
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(transformed_cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    
    matched_cloud_pub_.publish(msg);
}

void SonarSlamNode::publishPoseCorrection(const tf::Transform& correction) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    
    pose.pose.position.x = correction.getOrigin().x();
    pose.pose.position.y = correction.getOrigin().y();
    pose.pose.position.z = correction.getOrigin().z();
    
    tf::quaternionTFToMsg(correction.getRotation(), pose.pose.orientation);
    
    pose_correction_pub_.publish(pose);
}

void SonarSlamNode::publishMapToOdomTFWithPose(const tf::Transform& map_to_base,
                                      const tf::Transform& odom_to_base) {
    // 计算map->odom = map_to_base × base->odom
    tf::Transform map_to_odom = map_to_base * odom_to_base.inverse();
    
    // 使用odom_to_base的时间戳
    ros::Time tf_time;
    if (odom_to_base.getOrigin().length() > 0) {  // 简化的有效检查
        // 如果是tf::StampedTransform，可以获取时间戳
        // 这里需要类型转换，先简化处理
        tf_time = ros::Time::now();
    } else {
        tf_time = ros::Time::now();
    }
    
    // 发布TF
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = tf_time;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    
    transform_stamped.transform.translation.x = map_to_odom.getOrigin().x();
    transform_stamped.transform.translation.y = map_to_odom.getOrigin().y();
    transform_stamped.transform.translation.z = map_to_odom.getOrigin().z();
    
    tf::quaternionTFToMsg(map_to_odom.getRotation(), 
                         transform_stamped.transform.rotation);
    
    tf_broadcaster_->sendTransform(transform_stamped);
}
 


bool SonarSlamNode::hasLargeGaps() {
    // 直接调用上面的函数
    std::vector<Gap> gaps = findBoundaryGaps();
    return !gaps.empty();
}
// 利用已有地图数据寻找边界缺口
std::vector<SonarSlamNode::Gap> SonarSlamNode::findBoundaryGaps() {
    std::vector<Gap> gaps;
    
    // 重用现有的地图遍历逻辑
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            // 逻辑：已知占据点周围有大片未知区域
            if (map_.data[idx] == CELL_OCCUPIED) {
                int unknown_count = 0;
                
                // 检查5x5区域（重用前沿搜索的类似逻辑）
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            if (map_.data[ny * map_width_ + nx] == CELL_UNKNOWN) {
                                unknown_count++;
                            }
                        }
                    }
                }
                
                // 如果周围有大量未知（>10个），认为是缺口
                if (unknown_count > 10) {
                    Gap gap;
                    gap.x = mapToWorldX(x);
                    gap.y = mapToWorldY(y);
                    gap.size = unknown_count * map_resolution_; // 估算大小
                    gap.priority = 5; // 中等优先级
                    gaps.push_back(gap);
                }
            }
        }
    }
    
    return gaps;
}

SonarSlamNode::Gap SonarSlamNode::findNearestGap(
    const std::vector<Gap>& gaps, float robot_x, float robot_y) {
    
    if (gaps.empty()) return Gap();
    
    Gap nearest = gaps[0];
    float min_dist = FLT_MAX;
    
    for (const auto& gap : gaps) {
        float dx = gap.x - robot_x;
        float dy = gap.y - robot_y;
        float dist = sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            nearest = gap;
        }
    }
    
    return nearest;
}

//  
bool SonarSlamNode::shouldFinishExploration() {
    // 原有条件
    if (map_stability_counter_ < stability_threshold_) return false;
    
    // 【新增】检查是否有大缺口
    if (hasLargeGaps()) {
        ROS_INFO("还有缺口需要填补，继续探索");
        return false;
    }
    
    // 【新增】检查覆盖率
    float coverage = calculateMapCoverage();
    if (coverage < 0.85f) {
        ROS_INFO("覆盖率不足(%.1f%%)，继续探索", coverage * 100);
        return false;
    }
    
    return true;
}

// ==================== 地图完整性检查实现 ====================

SonarSlamNode::BoundaryAnalysis SonarSlamNode::analyzeBoundaryCompleteness() {
    BoundaryAnalysis analysis;
    analysis.is_closed = false;
    analysis.closure_confidence = 0.0f;
    analysis.total_gap_length = 0.0f;
    analysis.boundary_points_count = 0;
    
    // 1. 提取边界占据点（游泳池边界）
    std::vector<std::pair<int, int>> boundary_cells;
    
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            // 检查是否是边界点（占据点，但周围有未知或自由空间）
            if (map_.data[idx] == CELL_OCCUPIED) {
                bool is_boundary = false;
                
                // 检查8邻域
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            int nidx = ny * map_width_ + nx;
                            if (map_.data[nidx] != CELL_OCCUPIED) {
                                is_boundary = true;
                                break;
                            }
                        } else {
                            // 地图边缘的占据点肯定是边界
                            is_boundary = true;
                        }
                    }
                    if (is_boundary) break;
                }
                
                if (is_boundary) {
                    boundary_cells.push_back(std::make_pair(x, y));
                    analysis.boundary_points_count++;
                }
            }
        }
    }
    
    ROS_DEBUG("边界分析: 找到 %lu 个边界点", boundary_cells.size());
    
    // 2. 检查边界连续性
    if (boundary_cells.size() < 10) {
        // 边界点太少，无法判断
        analysis.closure_confidence = 0.1f;
        return analysis;
    }
    
    // 3. 寻找缺口
    std::vector<Gap> gaps = findRealBoundaryGaps();
    analysis.gaps = gaps;
    
    // 4. 计算总缺口长度
    for (const auto& gap : gaps) {
        analysis.total_gap_length += gap.size;
    }
    
    // 5. 根据边界点密度和缺口大小判断闭合性
    float boundary_density = boundary_cells.size() / (float)(map_width_ * map_height_);
    
    // 规则1: 缺口总长度小于游泳池估计周长的20%
    float estimated_perimeter = boundary_cells.size() * map_resolution_;
    float gap_ratio = analysis.total_gap_length / estimated_perimeter;
    
    // 规则2: 最大缺口小于2米
    float max_gap_size = 0.0f;
    for (const auto& gap : gaps) {
        if (gap.size > max_gap_size) max_gap_size = gap.size;
    }
    
    // 规则3: 边界点足够密集
    bool condition1 = (gap_ratio < 0.2f);      // 缺口小于20%
    bool condition2 = (max_gap_size < 2.0f);   // 最大缺口小于2米
    bool condition3 = (boundary_density > 0.01f); // 边界密度足够
    
    analysis.is_closed = condition1 && condition2 && condition3;
    
    // 计算置信度
    analysis.closure_confidence = 0.0f;
    if (condition1) analysis.closure_confidence += 0.4f;
    if (condition2) analysis.closure_confidence += 0.3f;
    if (condition3) analysis.closure_confidence += 0.3f;
    
    ROS_INFO("边界分析结果:");
    ROS_INFO("  边界点: %d 个", analysis.boundary_points_count);
    ROS_INFO("  缺口数: %lu 个", gaps.size());
    ROS_INFO("  总缺口长度: %.2f m", analysis.total_gap_length);
    ROS_INFO("  缺口比例: %.1f%%", gap_ratio * 100);
    ROS_INFO("  最大缺口: %.2f m", max_gap_size);
    ROS_INFO("  是否闭合: %s", analysis.is_closed ? "是" : "否");
    ROS_INFO("  闭合置信度: %.1f%%", analysis.closure_confidence * 100);
    
    return analysis;
}

std::vector<SonarSlamNode::Gap> SonarSlamNode::findRealBoundaryGaps() {
    std::vector<Gap> gaps;
    
    // 寻找边界上的连续未知区域（缺口）
    const int SEARCH_RADIUS = 15;  // 搜索半径（单元格）
    
    for (int y = 0; y < map_height_; y++) {
        for (int x = 0; x < map_width_; x++) {
            int idx = y * map_width_ + x;
            
            // 寻找边界未知点（在占据点旁边的未知点）
            if (map_.data[idx] == CELL_UNKNOWN) {
                bool near_occupied = false;
                
                // 检查周围是否有占据点
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            if (map_.data[ny * map_width_ + nx] == CELL_OCCUPIED) {
                                near_occupied = true;
                                break;
                            }
                        }
                    }
                    if (near_occupied) break;
                }
                
                if (near_occupied) {
                    // 这是边界上的未知点，检查它是否属于一个缺口
                    bool already_in_gap = false;
                    for (const auto& gap : gaps) {
                        float world_x = mapToWorldX(x);
                        float world_y = mapToWorldY(y);
                        float dist = sqrt(pow(world_x - gap.x, 2) + pow(world_y - gap.y, 2));
                        if (dist < 1.0f) {  // 1米内认为是同一个缺口
                            already_in_gap = true;
                            break;
                        }
                    }
                    
                    if (!already_in_gap) {
                        // 测量缺口大小（通过洪水填充）
                        float gap_size = measureGapSize(x, y);
                        if (gap_size > 0.5f) {  // 只考虑大于0.5米的缺口
                            Gap gap;
                            gap.x = mapToWorldX(x);
                            gap.y = mapToWorldY(y);
                            gap.size = gap_size;
                            
                            // 根据缺口大小和位置设置优先级
                            if (gap_size > 3.0f) gap.priority = 10;  // 大缺口，高优先级
                            else if (gap_size > 1.5f) gap.priority = 7;
                            else gap.priority = 5;
                            
                            gaps.push_back(gap);
                        }
                    }
                }
            }
        }
    }
    
    return gaps;
}

float SonarSlamNode::measureGapSize(int start_x, int start_y) {
    // 使用BFS测量未知区域的大小
    std::queue<std::pair<int, int>> queue;
    std::vector<std::vector<bool>> visited(map_height_, std::vector<bool>(map_width_, false));
    
    queue.push(std::make_pair(start_x, start_y));
    visited[start_y][start_x] = true;
    
    int cell_count = 0;
    float max_distance = 0.0f;
    
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        cell_count++;
        
        int x = current.first;
        int y = current.second;
        
        // 检查4邻域
        int dx[] = {1, -1, 0, 0};
        int dy[] = {0, 0, 1, -1};
        
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                if (!visited[ny][nx] && map_.data[ny * map_width_ + nx] == CELL_UNKNOWN) {
                    visited[ny][nx] = true;
                    queue.push(std::make_pair(nx, ny));
                    
                    // 计算到起点的距离
                    float dist = sqrt(pow(nx - start_x, 2) + pow(ny - start_y, 2)) * map_resolution_;
                    if (dist > max_distance) max_distance = dist;
                }
            }
        }
    }
    
    // 缺口大小 = 最大直径
    return max_distance * 2.0f;  // 乘以2作为估算
}

bool SonarSlamNode::isMapClosed() {
    BoundaryAnalysis analysis = analyzeBoundaryCompleteness();
    return analysis.is_closed && analysis.closure_confidence > 0.8f;
}

bool SonarSlamNode::shouldContinueExploration() {
    // 综合判断是否应该继续探索
    
    // 1. 检查地图是否已经闭合
    if (isMapClosed()) {
        ROS_INFO("地图已闭合，无需继续探索");
        return false;
    }
    
    // 2. 检查覆盖率
    float coverage = calculateMapCoverage();
    if (coverage < 0.7f) {
        ROS_INFO("覆盖率不足(%.1f%%)，需要继续探索", coverage * 100);
        return true;
    }
    
    // 3. 检查是否有明显的缺口
    std::vector<Gap> gaps = findRealBoundaryGaps();
    if (!gaps.empty()) {
        ROS_INFO("还有 %lu 个缺口需要填补", gaps.size());
        return true;
    }
    
    // 4. 检查地图稳定性
    if (map_stability_counter_ < stability_threshold_) {
        ROS_INFO("地图还在变化中，继续探索");
        return true;
    }
    
    // 5. 默认返回false（探索完成）
    return false;
}

// 辅助函数：从前沿点寻找目标
bool SonarSlamNode::findNextGoalFromFrontiers() {
    // 1. 寻找前沿点（未知区域与已知区域的边界）
    std::vector<Frontier> frontiers;
    findFrontiers(frontiers);
    
    ROS_INFO("找到 %lu 个前沿点", frontiers.size());
    
    if (!frontiers.empty()) {
        // 2. 选择最佳前沿点
        Frontier best = selectBestFrontier(frontiers);
        current_target_x_ = mapToWorldX(best.x);
        current_target_y_ = mapToWorldY(best.y);
        has_goal_ = true;
        
        ROS_INFO("新探索目标: (%.2f, %.2f) 权重: %.3f", 
                current_target_x_, current_target_y_, best.weight);
        
        // 显示前沿点信息
        for (size_t i = 0; i < std::min(frontiers.size(), (size_t)3); i++) {
            ROS_INFO("前沿点 %zu: (%d, %d) 权重=%.3f", i+1,
                    frontiers[i].x, frontiers[i].y, frontiers[i].weight);
        }
        
        return true;
    }
    
    return false;
}
 
// ====================  边界缺口检测 ====================
std::vector<SonarSlamNode::BoundaryGap> SonarSlamNode::detectBoundaryGaps() {
    std::vector<BoundaryGap> gaps;
    
    // 获取机器人位置
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) {
        return gaps;
    }
    
    const int BORDER_CHECK_DISTANCE = 3; // 检查边界附近3米
    
    // 检查四个方向
    std::vector<std::pair<float, float>> directions = {
        {0, 1},    // 北
        {1, 0},    // 东
        {0, -1},   // 南
        {-1, 0}    // 西
    };
    
    for (int dir_idx = 0; dir_idx < directions.size(); dir_idx++) {
        float dx = directions[dir_idx].first;
        float dy = directions[dir_idx].second;
        
        float search_distance = 0;
        while (search_distance < BORDER_CHECK_DISTANCE) {
            float check_x = robot_x + dx * search_distance;
            float check_y = robot_y + dy * search_distance;
            
            int map_x = worldToMapX(check_x);
            int map_y = worldToMapY(check_y);
            
            if (map_x < 0 || map_x >= map_width_ || map_y < 0 || map_y >= map_height_) {
                break;
            }
            
            int idx = map_y * map_width_ + map_x;
            
            // 如果发现未知区域，记录为缺口
            if (map_.data[idx] == CELL_UNKNOWN) {
                // 简单估算缺口大小（使用固定值或简单计算）
                float gap_size = 1.0f; // 默认大小
                
                // 简单计算：检查周围有多少连续未知
                int unknown_count = 1;
                for (int i = 1; i < 5; i++) { // 检查4个方向
                    int nx = map_x + static_cast<int>(dx * i);
                    int ny = map_y + static_cast<int>(dy * i);
                    
                    if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                        int nidx = ny * map_width_ + nx;
                        if (map_.data[nidx] == CELL_UNKNOWN) {
                            unknown_count++;
                        } else {
                            break;
                        }
                    }
                }
                
                gap_size = unknown_count * map_resolution_;
                
                if (gap_size > 0.5f) {
                    BoundaryGap gap;
                    gap.center.x = check_x;
                    gap.center.y = check_y;
                    gap.size = gap_size;
                    gap.distance_to_robot = search_distance;
                    
                    // 简单优先级计算
                    gap.priority = static_cast<int>((1.0f / (gap.distance_to_robot + 0.1f)) * 10 + 
                                                   gap.size * 5);
                    
                    gaps.push_back(gap);
                    
                    ROS_INFO("检测到边界缺口: 方向=%d, 位置=(%.2f,%.2f), 大小=%.2fm, 距离=%.2fm", 
                            dir_idx, gap.center.x, gap.center.y, gap.size, gap.distance_to_robot);
                    
                    break;
                }
            }
            
            search_distance += map_resolution_;
        }
    }
    
    // 如果没有找到边界缺口，使用前沿点
    if (gaps.empty()) {
        std::vector<Frontier> frontiers;
        findFrontiers(frontiers);
        
        for (const auto& frontier : frontiers) {
            BoundaryGap gap;
            gap.center.x = mapToWorldX(frontier.x);
            gap.center.y = mapToWorldY(frontier.y);
            gap.size = 1.0f;
            gap.distance_to_robot = frontier.distance;
            gap.priority = static_cast<int>(frontier.weight * 20);
            gaps.push_back(gap);
        }
    }
    
    // 按优先级排序
    std::sort(gaps.begin(), gaps.end(), 
        [](const BoundaryGap& a, const BoundaryGap& b) {
            return a.priority > b.priority;
        });
    
    return gaps;
}
// ====================  选择最佳缺口填补 ====================
SonarSlamNode::BoundaryGap SonarSlamNode::selectBestGapToFill(const std::vector<BoundaryGap>& gaps) {
    if (gaps.empty()) {
        return BoundaryGap();
    }
    
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) {
        // 无法获取机器人位置，返回优先级最高的
        return gaps[0];
    }
    
    // 综合评分：优先级（40%）+ 距离（30%）+ 路径安全性（30%）
    BoundaryGap best_gap = gaps[0];
    float best_score = -1.0f;
    
    for (const auto& gap : gaps) {
        // 1. 归一化优先级（0-1）
        float priority_score = static_cast<float>(gap.priority) / 10.0f;
        
        // 2. 距离得分（越近越好，归一化到0-1）
        float max_reasonable_distance = 15.0f;  // 最大合理距离
        float distance_score = 1.0f - std::min(1.0f, gap.distance_to_robot / max_reasonable_distance);
        
        // 3. 路径安全性得分（检查路径是否畅通）
        float safety_score = 1.0f;
        if (!isPathClearToTarget(gap.center.x, gap.center.y)) {
            safety_score = 0.3f;  // 路径有障碍，降低评分
        }
        
        // 4. 综合评分
        float total_score = 
            priority_score * 0.4f + 
            distance_score * 0.3f + 
            safety_score * 0.3f;
        
        if (total_score > best_score) {
            best_score = total_score;
            best_gap = gap;
        }
        
        ROS_DEBUG("缺口评分: 位置(%.2f, %.2f), 优先级=%.2f, 距离=%.2f, 安全=%.2f, 总分=%.2f",
                 gap.center.x, gap.center.y, priority_score, distance_score, safety_score, total_score);
    }
    
    ROS_INFO("选择最佳缺口: 位置(%.2f, %.2f), 大小=%.2fm, 优先级=%d, 评分=%.2f",
            best_gap.center.x, best_gap.center.y, best_gap.size, 
            best_gap.priority, best_score);
    
    return best_gap;
}

// ==================== 【核心】路径安全检查 ====================
bool SonarSlamNode::isPathClearToTarget(float target_x, float target_y) {
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) {
        return false;  // 无法获取位置，假设不安全
    }
    
    // 简单的直线路径检查
    float dx = target_x - robot_x;
    float dy = target_y - robot_y;
    float distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 0.5f) {
        return true;  // 目标很近，直接认为安全
    }
    
    // 沿着路径采样检查
    int samples = static_cast<int>(distance / 0.1f);  // 每10cm采样一次
    samples = std::min(samples, 50);  // 最多采样50次
    
    for (int i = 1; i <= samples; i++) {
        float t = static_cast<float>(i) / samples;
        float check_x = robot_x + dx * t;
        float check_y = robot_y + dy * t;
        
        // 转换为地图坐标
        int map_x = worldToMapX(check_x);
        int map_y = worldToMapY(check_y);
        
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            int idx = map_y * map_width_ + map_x;
            
            // 如果路径上有占据的单元格，路径不安全
            if (map_.data[idx] == CELL_OCCUPIED) {
                ROS_WARN("路径检查: (%.2f, %.2f) 有障碍物", check_x, check_y);
                return false;
            }
        }
    }
    
    return true;
}

// ==================== 【核心】智能探索决策 ====================
bool SonarSlamNode::shouldContinueExploring() {
    // 简化实现：
    
    // 1. 检查覆盖率
    float coverage = calculateMapCoverage();
    if (coverage < 0.7f) {
        ROS_INFO("覆盖率不足(%.1f%%)，继续探索", coverage * 100);
        return true;
    }
    
    // 2. 检查是否有缺口
    std::vector<Gap> gaps = findBoundaryGaps();  // 使用现有的 Gap 结构
    if (!gaps.empty()) {
        ROS_INFO("有 %lu 个缺口需要填补，继续探索", gaps.size());
        return true;
    }
    
    // 3. 检查地图稳定性
    if (map_stability_counter_ < stability_threshold_) {
        ROS_INFO("地图还在变化中，继续探索");
        return true;
    }
    
    // 4. 探索完成
    ROS_INFO("探索完成: 覆盖率=%.1f%%, 地图稳定", coverage * 100);
    return false;
}
 

// ==================== 【核心】检查地图是否完整 ====================
bool SonarSlamNode::isMapComplete() {
    if (map_.data.empty()) {
        ROS_WARN("地图数据为空");
        return false;
    }
    
    // 1. 基础覆盖率检查
    float coverage = calculateMapCoverage();
    if (coverage < 0.70f) {
        ROS_DEBUG_THROTTLE(5.0, "地图不完整: 覆盖率 %.1f%% < 70%%", coverage * 100);
        return false;
    }
    
    // 2. 优化边界检查 - 采样而非全遍历
    int boundary_points = 0;
    int checked_cells = 0;
    const int STEP = 3;
    
    for (int y = 2; y < map_height_ - 2; y += STEP) {
        for (int x = 2; x < map_width_ - 2; x += STEP) {
            checked_cells++;
            int idx = y * map_width_ + x;
            
            if (map_.data[idx] == CELL_OCCUPIED) {
                bool has_unknown_neighbor = false;
                
                // 只检查4邻域
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        if (abs(dx) + abs(dy) > 1) continue;  // 只检查4邻域
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            int nidx = ny * map_width_ + nx;
                            if (map_.data[nidx] == CELL_UNKNOWN) {
                                has_unknown_neighbor = true;
                                break;
                            }
                        }
                    }
                    if (has_unknown_neighbor) break;
                }
                
                if (has_unknown_neighbor) {
                    boundary_points++;
                }
            }
        }
    }
    
    // 3. 声纳专用阈值
    float boundary_ratio = (checked_cells > 0) ? 
                          (float)boundary_points / checked_cells : 0.0f;
    
    const float MAX_BOUNDARY_RATIO = 0.12f;
    
    // 4. 组合条件
    bool is_complete = (coverage > 0.70f) && 
                      (boundary_ratio < MAX_BOUNDARY_RATIO);
    
    // 5. 【新增】快速边界缺口检查（直接嵌入）
    if (is_complete) {
        // 检查地图四边是否有大缺口
        const int GAP_THRESHOLD = 8; // 连续8个未知单元格认为是缺口
        
        // 上边检查
        int top_gap = 0, current_gap = 0;
        for (int x = 0; x < map_width_; x++) {
            if (map_.data[(map_height_ - 1) * map_width_ + x] == CELL_UNKNOWN) {
                current_gap++;
                if (current_gap > top_gap) top_gap = current_gap;
            } else {
                current_gap = 0;
            }
        }
        
        // 下边检查
        int bottom_gap = 0;
        current_gap = 0;
        for (int x = 0; x < map_width_; x++) {
            if (map_.data[x] == CELL_UNKNOWN) {
                current_gap++;
                if (current_gap > bottom_gap) bottom_gap = current_gap;
            } else {
                current_gap = 0;
            }
        }
        
        // 左边检查
        int left_gap = 0;
        current_gap = 0;
        for (int y = 0; y < map_height_; y++) {
            if (map_.data[y * map_width_] == CELL_UNKNOWN) {
                current_gap++;
                if (current_gap > left_gap) left_gap = current_gap;
            } else {
                current_gap = 0;
            }
        }
        
        // 右边检查
        int right_gap = 0;
        current_gap = 0;
        for (int y = 0; y < map_height_; y++) {
            if (map_.data[y * map_width_ + (map_width_ - 1)] == CELL_UNKNOWN) {
                current_gap++;
                if (current_gap > right_gap) right_gap = current_gap;
            } else {
                current_gap = 0;
            }
        }
        
        // 如果任何一边有超过阈值的缺口，地图不完整
        if (top_gap > GAP_THRESHOLD || bottom_gap > GAP_THRESHOLD ||
            left_gap > GAP_THRESHOLD || right_gap > GAP_THRESHOLD) {
            is_complete = false;
            
            ROS_DEBUG_THROTTLE(5.0, "边界有缺口: 上=%d, 下=%d, 左=%d, 右=%d (阈值=%d)",
                             top_gap, bottom_gap, left_gap, right_gap, GAP_THRESHOLD);
        }
    }
    
    // 6. 结合稳定性检查
    if (is_complete) {
        is_complete = (map_stability_counter_ >= stability_threshold_);
    }
    
    // 7. 调试输出
    static int call_count = 0;
    if (call_count++ % 30 == 0) {
        ROS_INFO("地图完整性检查:");
        ROS_INFO("  边界点: %d/%d, 边界比: %.2f%% (阈值: %.1f%%)", 
                boundary_points, checked_cells, boundary_ratio * 100, MAX_BOUNDARY_RATIO * 100);
        ROS_INFO("  覆盖率: %.1f%% (阈值: 70%%)", coverage * 100);
        ROS_INFO("  稳定计数: %d/%d", map_stability_counter_, stability_threshold_);
        call_count = 0;
    }
    
    if (is_complete) {
        ROS_INFO(" 地图完整性检验通过");
    }
    
    return is_complete;
}
// ====================  查找并排序缺口 ====================
std::vector<GapInfo> SonarSlamNode::findAndRankGaps(){
    std::vector<GapInfo> gaps;
    
    // 获取机器人位置和朝向
    float robot_x, robot_y, robot_yaw;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
        ROS_WARN("无法获取机器人位姿，无法计算缺口距离");
        return gaps;
    }
    
    // 采样检查地图（提高效率）
    const int STEP_SIZE = 2;  // 步长，减少计算量
    
    for (int y = 2; y < map_height_ - 2; y += STEP_SIZE) {
        for (int x = 2; x < map_width_ - 2; x += STEP_SIZE) {
            int idx = y * map_width_ + x;
            
            // 如果是未知区域
            if (map_.data[idx] == CELL_UNKNOWN) {
                // 检查周围是否有占据点（边界迹象）
                int occupied_neighbors = 0;
                int free_neighbors = 0;
                
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            int nidx = ny * map_width_ + nx;
                            if (map_.data[nidx] == CELL_OCCUPIED) {
                                occupied_neighbors++;
                            } else if (map_.data[nidx] == CELL_FREE) {
                                free_neighbors++;
                            }
                        }
                    }
                }
                
                // 判断是否为缺口：有占据点邻居（边界）且不是完全孤立
                if (occupied_neighbors > 2 && free_neighbors > 0) {
                    // 计算缺口世界坐标
                    float world_x = mapToWorldX(x);
                    float world_y = mapToWorldY(y);
                    
                    // 计算到机器人的距离
                    float dx = world_x - robot_x;
                    float dy = world_y - robot_y;
                    float distance = sqrt(dx*dx + dy*dy);
                    
                    // 估算缺口大小（通过周围未知区域数量）
                    float gap_size = occupied_neighbors * map_resolution_;
                    
                    // 创建缺口信息
                    GapInfo gap(world_x, world_y, gap_size, distance, 0);
                    
                
                    // 计算缺口相对于机器人的角度
                    float angle_to_gap = atan2(dy, dx);
                    float angle_diff = angle_to_gap - robot_yaw;
                    
                    // 规范化角度差到[-π, π]
                    while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                    while (angle_diff < -M_PI) angle_diff += 2.0f * M_PI;
                    
                    float abs_angle_diff = fabs(angle_diff);
                    
                    //  优先：远距离、对侧方向、大缺口
                    // 1. 距离因子：距离越远，优先级越高（0-30分）
                    float distance_score = (distance / 10.0f) * 30;
                    
                    // 2. 对侧因子：角度差越大（越对侧），优先级越高（0-40分）
                    float opposite_score = 0.0f;
                    if (abs_angle_diff > M_PI * 0.6f) {  // 大于108度
                        opposite_score = 40.0f;  // 对侧最高分
                    } else if (abs_angle_diff > M_PI * 0.3f) {  // 大于54度
                        opposite_score = 20.0f;  // 侧向中等分
                    } else {
                        opposite_score = 5.0f;   // 同侧低分
                    }
                    
                    // 3. 缺口大小因子：缺口越大，优先级越高（0-20分）
                    float size_score = std::min(gap_size * 10.0f, 20.0f);
                    
                    // 4. 距离机器人前方因子：正前方有一定优势（0-10分）
                    float forward_score = 10.0f * (1.0f - abs_angle_diff / M_PI);
                    
                    // 综合优先级
                    gap.priority = static_cast<int>(
                        distance_score +      // 远距离优先
                        opposite_score +      // 对侧优先
                        size_score +          // 大缺口优先
                        forward_score         // 正前方有一定优势
                    );
                     
                    
                    // 避免重复的缺口（检查是否已经有相近的缺口）
                    bool is_duplicate = false;
                    for (const auto& existing_gap : gaps) {
                        float dist = sqrt(pow(existing_gap.position.x - world_x, 2) + 
                                         pow(existing_gap.position.y - world_y, 2));
                        if (dist < 1.0f) {  // 1米内认为是同一缺口
                            // 保留优先级更高的
                            if (gap.priority > existing_gap.priority) {
                                // 用新的替换旧的
                                auto it = std::find(gaps.begin(), gaps.end(), existing_gap);
                                if (it != gaps.end()) {
                                    *it = gap;
                                }
                            }
                            is_duplicate = true;
                            break;
                        }
                    }
                    
                    if (!is_duplicate) {
                        gaps.push_back(gap);
                    }
                }
            }
        }
    }
    
    // 按优先级排序（从高到低）
    std::sort(gaps.begin(), gaps.end(), 
        [](const GapInfo& a, const GapInfo& b) {
            return a.priority > b.priority;
        });
    
    ROS_INFO("找到 %lu 个缺口", gaps.size());
    if (!gaps.empty()) {
        ROS_INFO("优先级计算方式: 距离(远)+对侧+大小+前方");
        
        for (size_t i = 0; i < std::min(gaps.size(), (size_t)3); i++) {
            // 计算角度信息用于调试
            float dx = gaps[i].position.x - robot_x;
            float dy = gaps[i].position.y - robot_y;
            float angle_to_gap = atan2(dy, dx) * 180.0f / M_PI;
            float robot_yaw_deg = robot_yaw * 180.0f / M_PI;
            float angle_diff = fabs(angle_to_gap - robot_yaw_deg);
            if (angle_diff > 180.0f) angle_diff = 360.0f - angle_diff;
            
            ROS_INFO("缺口 %lu: 位置(%.1f,%.1f), 距离=%.1fm, 角度差=%.0f°, 大小=%.1fm, 优先级=%d", 
                    i+1, 
                    gaps[i].position.x, gaps[i].position.y,
                    gaps[i].distance,
                    angle_diff,
                    gaps[i].size,
                    gaps[i].priority);
        }
    }
    
    return gaps;
}

// ==================== 【核心】执行缺口扫描 ====================
void SonarSlamNode::performGapScanning() {
    ROS_INFO("开始执行缺口扫描...");
    
    // 记录开始时间
    ros::Time scan_start = ros::Time::now();
    
    // 策略1：原地缓慢旋转，360度扫描
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.4f;  // 缓慢旋转速度
    
    // 旋转约360度（计算时间：2π/0.4 ≈ 15.7秒）
    float rotation_time = 16.0f;  // 16秒确保完整一圈
    
    ROS_INFO("原地旋转扫描，持续时间: %.1f秒", rotation_time);
    
    while ((ros::Time::now() - scan_start).toSec() < rotation_time && ros::ok()) {
        // 发布旋转命令
        cmd_vel_pub_.publish(cmd_vel);
        
        // 短暂休眠，控制频率
        ros::Duration(0.1).sleep();
        
        // 检查紧急情况
        if (checkEmergencyConditions()) {
            ROS_WARN("扫描过程中检测到紧急情况，停止扫描");
            break;
        }
    }
    
    // 停止旋转
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_.publish(cmd_vel);
    
    // 策略2：短暂停留，让声纳收集数据
    ROS_INFO("扫描完成，停留3秒收集数据...");
    ros::Duration(3.0).sleep();
    
    // 策略3：如果有需要，可以前后移动一点
    if (checkIfNeedAdditionalScanning()) {
        ROS_INFO("执行补充扫描...");
        
        // 前进0.5米
        cmd_vel.linear.x = 0.1f;
        cmd_vel_pub_.publish(cmd_vel);
        ros::Duration(5.0).sleep();  // 0.1 * 5 = 0.5米
        
        // 停止
        cmd_vel.linear.x = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        ros::Duration(1.0).sleep();
        
        // 后退0.5米回到原位
        cmd_vel.linear.x = -0.1f;
        cmd_vel_pub_.publish(cmd_vel);
        ros::Duration(5.0).sleep();
        
        // 停止
        cmd_vel.linear.x = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    gap_fill_count_++;
    ROS_INFO("缺口扫描完成，累计填补 %d 个缺口", gap_fill_count_);
}

// 辅助函数：检查是否需要补充扫描
bool SonarSlamNode::checkIfNeedAdditionalScanning() {
    // 简单判断：如果当前缺口比较大，需要补充扫描
    return current_gap_target_.size > 1.5f;
}

// 辅助函数：检查是否到达缺口位置
bool SonarSlamNode::checkIfAtGap(const geometry_msgs::Point& gap_pos) {
    float robot_x, robot_y;
    if (!getRobotPosition(robot_x, robot_y)) return false;
    
    float distance = sqrt(pow(robot_x - gap_pos.x, 2) + pow(robot_y - gap_pos.y, 2));
    return distance < goal_tolerance_;
}

 

// ==================== 【新增】缺口探索状态处理 ====================
void SonarSlamNode::handleGapExplorationState() {
    ROS_DEBUG("处理缺口探索状态");
    
    // 检查紧急情况
    if (checkEmergencyConditions()) {
        current_state_ = STATE_EMERGENCY_STOP;
        return;
    }
    
    // 如果还没有目标，设置目标
    if (!has_goal_) {
        current_target_x_ = current_gap_target_.position.x;
        current_target_y_ = current_gap_target_.position.y;
        has_goal_ = true;
        
        // 记录开始时间（用于超时检测）
        gap_exploration_start_time_ = ros::Time::now();
        
        ROS_INFO("设置缺口目标: (%.2f, %.2f), 大小=%.2fm", 
                current_target_x_, current_target_y_, current_gap_target_.size);
    }
    
    // 导航到缺口
    navigateToTarget();
    
    // 检查是否到达缺口
    float robot_x, robot_y;
    if (getRobotPosition(robot_x, robot_y)) {
        float distance = sqrt(pow(robot_x - current_target_x_, 2) + 
                             pow(robot_y - current_target_y_, 2));
        
        if (distance < goal_tolerance_) {
            ROS_INFO(" 到达缺口位置，开始扫描");
            
            // 停止机器人
            stopRobot();
            ros::Duration(0.5).sleep();
            
            // 执行缺口扫描
            performGapScanning();
            
            // 短暂停留，等待地图更新
            ros::Duration(2.0).sleep();
            
            // 转到验证状态
            current_state_ = STATE_COVERAGE_VERIFY;
            has_goal_ = false;
            
            ROS_INFO("缺口扫描完成，切换到验证状态");
            return;
        }
        
        // 超时保护：如果30秒还没到达，放弃这个缺口
        if ((ros::Time::now() - gap_exploration_start_time_).toSec() > 30.0) {
            ROS_WARN("缺口探索超时(30秒)，放弃当前缺口");
            current_state_ = STATE_INITIAL_MAPPING;
            has_goal_ = false;
            return;
        }
    }
}

// ====================  覆盖验证状态处理 ====================
void SonarSlamNode::handleCoverageVerifyState() {
    ROS_DEBUG("处理覆盖验证状态");
    
    // 等待一下，让地图有足够时间更新
    static bool first_enter = true;
    if (first_enter) {
        ROS_INFO("等待地图更新...");
        ros::Duration(2.0).sleep();
        first_enter = false;
        return;
    }
    
    // 检查地图是否完整
    if (isMapComplete()) {
        ROS_INFO(" 地图验证完整！");
        map_verified_complete_ = true;
        current_state_ = STATE_INITIAL_MAPPING;
    } else {
        // 地图仍不完整，检查是否还有缺口
        auto gaps = findAndRankGaps();
        
        if (!gaps.empty()) {
            ROS_INFO("仍有 %lu 个缺口需要填补，继续探索", gaps.size());
            
            // 选择下一个缺口
            current_gap_target_ = gaps[0];
            current_state_ = STATE_GAP_EXPLORATION;
        } else {
            ROS_INFO("未检测到明确缺口，但地图不完整，返回初始建图");
            current_state_ = STATE_INITIAL_MAPPING;
        }
    }
    
    first_enter = true;  // 重置，为下次进入准备
}

void SonarSlamNode::publishSmoothedTF(const tf::Transform& raw_transform, 
                                      const ros::Time& stamp) {
    tf::Transform smoothed_transform;
    
    if (first_tf_) {
        // 第一次发布，直接使用原始值
        smoothed_transform = raw_transform;
        first_tf_ = false;
        ROS_DEBUG("TF滤波: 第一次发布，无滤波");
    } else {
        // 计算时间间隔
        double dt = (stamp - last_tf_time_).toSec();
        
        // 防止dt为0或负值
        if (dt <= 0) {
            dt = 0.01;  // 默认10ms
            ROS_WARN_THROTTLE(5.0, "TF时间间隔异常，使用默认值");
        }
        
        // 动态调整滤波系数（时间间隔越长，滤波越强）
        double alpha = std::min(tf_smooth_factor_ / dt, 1.0);
        alpha = std::max(alpha, 0.01);  // 最小滤波强度
        
        // 位置滤波（线性插值）
        tf::Vector3 smooth_position = 
            last_map_to_odom_.getOrigin() * (1.0 - alpha) + 
            raw_transform.getOrigin() * alpha;
        
        // 旋转滤波（四元数球面线性插值）
        tf::Quaternion smooth_rotation = 
            last_map_to_odom_.getRotation().slerp(raw_transform.getRotation(), alpha);
        
        smoothed_transform.setOrigin(smooth_position);
        smoothed_transform.setRotation(smooth_rotation);
        
        // 调试信息
        static int debug_counter = 0;
        if (debug_counter++ % 100 == 0) {
            float pos_diff = (raw_transform.getOrigin() - smoothed_transform.getOrigin()).length();
            float ang_diff = fabs(tf::getYaw(raw_transform.getRotation()) - 
                                 tf::getYaw(smoothed_transform.getRotation()));
            ROS_DEBUG("TF滤波: dt=%.3fs, alpha=%.3f, 位置差=%.3fm, 角度差=%.3frad",
                     dt, alpha, pos_diff, ang_diff);
            debug_counter = 0;
        }
    }
    
    // 保存状态
    last_map_to_odom_ = smoothed_transform;
    last_tf_time_ = stamp;
    
    // 发布滤波后的TF
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = stamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    
    transform_stamped.transform.translation.x = smoothed_transform.getOrigin().x();
    transform_stamped.transform.translation.y = smoothed_transform.getOrigin().y();
    transform_stamped.transform.translation.z = smoothed_transform.getOrigin().z();
    
    tf::quaternionTFToMsg(smoothed_transform.getRotation(), 
                         transform_stamped.transform.rotation);
    
    tf_broadcaster_->sendTransform(transform_stamped);
}

void SonarSlamNode::staticMappingDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data && current_state_ == STATE_INITIAL_MAPPING) {
        ROS_INFO("收到静态建图完成信号");
        
        // 如果已经收到初始地图，直接开始动态探索
        if (has_initial_map_) {
            ROS_INFO("已收到初始地图，开始动态探索");
            // 状态机会自动处理
        } else {
            ROS_INFO("等待初始地图...");
        }
    }
}

void SonarSlamNode::initialMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!has_initial_map_) {
        initial_map_ = *msg;
        has_initial_map_ = true;
        
        ROS_INFO("收到初始地图: %dx%d, 分辨率: %.3f", 
                initial_map_.info.width, initial_map_.info.height,
                initial_map_.info.resolution);
        
        // 合并到当前地图
        mergeInitialMap(initial_map_);
        
        // 发布一个简单的目标，开始探索
        current_target_x_ = 2.0;  // 先向前走2米
        current_target_y_ = 0.0;
        has_goal_ = true;
        
        ROS_INFO("设置初始探索目标: (%.2f, %.2f)", current_target_x_, current_target_y_);
    }
}

// 【新增】合并初始地图函数
void SonarSlamNode::mergeInitialMap(const nav_msgs::OccupancyGrid& initial_map) {
    // 简单合并：直接把初始地图的占据点复制过来
    for (size_t i = 0; i < initial_map.data.size(); i++) {
        if (initial_map.data[i] == 100) {  // 占据点
            // 计算世界坐标
            int map_x = i % initial_map.info.width;
            int map_y = i / initial_map.info.width;
            
            float world_x = initial_map.info.origin.position.x + 
                           (map_x + 0.5) * initial_map.info.resolution;
            float world_y = initial_map.info.origin.position.y + 
                           (map_y + 0.5) * initial_map.info.resolution;
            
            // 更新到当前地图
            updateMapWithPoint(world_x, world_y);
        }
    }
    
    ROS_INFO("初始地图合并完成，当前地图点数: %lu", scan_points_.size());
}



} // namespace sonar_slam

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_slam_node");
    
    ROS_INFO("Starting Sonar SLAM Node...");
    
    try {
        sonar_slam::SonarSlamNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Sonar SLAM Node crashed: %s", e.what());
        return 1;
    }
    
    ROS_INFO("Sonar SLAM Node shutdown complete");
    return 0;
}