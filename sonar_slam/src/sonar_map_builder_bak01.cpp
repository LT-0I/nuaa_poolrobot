/**
 * @file sonar_map_builder.cpp
 * @brief 独立节点：从声纳数据文件构建点云地图
 * 用法：rosrun sonar_slam sonar_map_builder _data_file:="your_data_file.bin"
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <mars_msgs/Pose2DArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iomanip>  // 添加头文件
// 包含头文件
#include "sonar_slam/sonar_map_builder.h"

// 现在整个实现应该在命名空间内
namespace sonar_slam {

SonarMapBuilder::SonarMapBuilder(ros::NodeHandle& nh) : 
    nh_(nh),
    data_loaded_(false),
    pointcloud_(new PointCloudI()),
    // 新增成员初始化
    enable_beam_spreading_(true),
    // ==================== 新增：初始化方向性参数 ====================
    near_side_max_range_(3.0f),
    far_side_max_range_(5.0f),
    side_wall_max_range_(4.0f),
    near_side_intensity_(0.15f),
    far_side_intensity_(0.08f),
    side_wall_intensity_(0.12f)
{
    // 读取参数
    nh_.param("data_file", data_file_, std::string(""));
    nh_.param("max_range", max_range_, 4.0f);     
    nh_.param("min_range", min_range_, 0.5f);
    nh_.param("min_intensity", min_intensity_, 80.0f);
    nh_.param("map_resolution", map_resolution_, 0.05f);
    nh_.param("distance_scale", distance_scale_, 0.4f);
    nh_.param("map_width", map_width_, 140);
    nh_.param("map_height", map_height_, 120);
    nh_.param("map_origin_x", map_origin_x_, -3.5f);
    nh_.param("map_origin_y", map_origin_y_, -3.0f);
    
    // 新增参数读取
    nh_.param("enable_beam_spreading", enable_beam_spreading_, true);
 
    nh_.param("enable_symmetry", enable_symmetry_, false);  // 新增对称参数
    
    ROS_INFO("  enable_beam_spreading: %s", enable_beam_spreading_ ? "true" : "false");
    ROS_INFO("  enable_symmetry: %s", enable_symmetry_ ? "true" : "false");  // 新增日志

        // ==================== 新增：方向性过滤参数 ====================
    nh_.param("near_side_max_range", near_side_max_range_, 3.0f);   // 采集侧最大范围
    nh_.param("far_side_max_range", far_side_max_range_, 5.0f);     // 对侧最大范围
    nh_.param("side_wall_max_range", side_wall_max_range_, 4.0f);   // 侧墙最大范围
    
    nh_.param("near_side_intensity", near_side_intensity_, 0.15f);  // 采集侧强度阈值
    nh_.param("far_side_intensity", far_side_intensity_, 0.08f);    // 对侧强度阈值
    nh_.param("side_wall_intensity", side_wall_intensity_, 0.12f);  // 侧墙强度阈值
    
    ROS_INFO("  Directional filtering enabled:");
    ROS_INFO("    Near side: %.1fm, >%.1f%%", near_side_max_range_, near_side_intensity_*100);
    ROS_INFO("    Far side: %.1fm, >%.1f%%", far_side_max_range_, far_side_intensity_*100);
    ROS_INFO("    Side walls: %.1fm, >%.1f%%", side_wall_max_range_, side_wall_intensity_*100);
    // ==================== 新增结束 ====================
    
    // 初始化发布器
    pointcloud_pub_ = nh_.advertise<PointCloudI>("sonar_map_pointcloud", 1, true);
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("sonar_map_scan", 1, true);
    occupancy_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("sonar_occupancy_map", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sonar_map_marker", 1, true);
    pose_array_pub_ = nh_.advertise<mars_msgs::Pose2DArray>("sonar_map_poses", 1, true);
    
    // TF广播
    tf_broadcaster_ = new tf::TransformBroadcaster();
    
    ROS_INFO("========================================");
    ROS_INFO("Sonar Map Builder - Independent Node");
    ROS_INFO("========================================");
    ROS_INFO("Parameters:");
    ROS_INFO("  data_file: %s", data_file_.c_str());
    ROS_INFO("  max_range: %.1f", max_range_);
    ROS_INFO("  min_range: %.1f", min_range_);
    ROS_INFO("  min_intensity: %.1f", min_intensity_);
    ROS_INFO("  map_resolution: %.3f", map_resolution_);
    ROS_INFO("  enable_beam_spreading: %s", enable_beam_spreading_ ? "true" : "false");
    ROS_INFO("========================================");
    
    // 如果指定了数据文件，自动加载
    if (!data_file_.empty()) {
        if (loadSonarData(data_file_)) {
            ROS_INFO("Successfully loaded sonar data from: %s", data_file_.c_str());
            ROS_INFO("Total points: %lu", getPointCount());
            
            // 关键：验证数据角度分布
            validateAngleDistribution();

            // 处理并发布数据
            processAndPublish();
        } else {
            ROS_ERROR("Failed to load sonar data from: %s", data_file_.c_str());
            ROS_WARN("Generating test data for demonstration...");
            generateTestData();
            processAndPublish();
        }
    } else {
        ROS_WARN("No data file specified. Usage:");
        ROS_WARN("  rosrun sonar_slam sonar_map_builder _data_file:=/path/to/your_data.bin");
        ROS_WARN("Generating test data for demonstration...");
        generateTestData();
        processAndPublish();
    }
}

SonarMapBuilder::~SonarMapBuilder()
{
    if (tf_broadcaster_) {
        delete tf_broadcaster_;
        tf_broadcaster_ = nullptr;
    }
    ROS_INFO("SonarMapBuilder destroyed");
}

bool SonarMapBuilder::loadSonarData(const std::string& filename)
{
    ROS_INFO("Loading sonar data from: %s", filename.c_str());
    
    // 先尝试解析二进制文件
    if (parseBinaryFile(filename)) {
        ROS_INFO("Successfully parsed binary file");
        data_loaded_ = true;
        return true;
    }
    
    ROS_WARN("Binary parsing failed, file may be in custom format");
    ROS_WARN("You may need to modify parseBinaryFile() to match your actual file format");
    
    data_loaded_ = false;
    return false;
}

void SonarMapBuilder::validateAngleDistribution()
{
    if (sonar_points_.empty()) return;
    
    std::map<int, int> angle_bins; // 每1度一个桶
    
    for (const auto& point : sonar_points_) {
        float angle_deg = point.heading_angle * 180.0 / M_PI;
        int bin = static_cast<int>(angle_deg);
        angle_bins[bin]++;
    }
    
    // 统计完整度
    int total_bins = 360;
    int occupied_bins = angle_bins.size();
    float coverage = static_cast<float>(occupied_bins) / total_bins * 100.0f;
    
    ROS_INFO("Angle coverage: %.1f%% (%d/%d degrees)", 
             coverage, occupied_bins, total_bins);
    
    // 打印缺失的角度范围
    std::vector<int> missing_bins;
    for (int deg = 0; deg < 360; deg++) {
        if (angle_bins.find(deg) == angle_bins.end()) {
            missing_bins.push_back(deg);
        }
    }
    
    if (!missing_bins.empty()) {
        ROS_WARN("Missing angles: %lu degree bins", missing_bins.size());
        // 打印前10个缺失的角度
        for (size_t i = 0; i < std::min(missing_bins.size(), size_t(10)); i++) {
            ROS_WARN("  - %d°", missing_bins[i]);
        }
    }
}

bool SonarMapBuilder::parseBinaryFile(const std::string& filename) 
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return false;
    }
    
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    ROS_INFO("File size: %lu bytes", file_size);
    
    // 跳过214字节头部（与Python代码一致）
    file.seekg(214, std::ios::beg);
    
    sonar_points_.clear();
    int total_frames = 0;
    int msg2300_count = 0;
    int invalid_angle_count = 0;  // 无效角度计数
    
    ROS_INFO("Starting to parse binary file (ignoring cmd=4)...");
    
    while (file.tellg() < file_size) {
        // 搜索'B'字符
        bool found_B = false;
        char ch;
        
        // 搜索'B'（最多100字节）
        for (int i = 0; i < 100; i++) {
            if (!file.read(&ch, 1)) break;
            if (ch == 'B') {
                found_B = true;
                break;
            }
        }
        
        if (!found_B) {
            if (total_frames % 100 == 0) {
                std::streampos pos = file.tellg();
                ROS_WARN("Failed to find 'B' marker at position %lld", static_cast<long long>(pos));
            }
            break;
        }
        
        // 检查下一个字符是否为'R'
        if (!file.read(&ch, 1)) break;
        if (ch != 'R') {
            // 不是'BR'，继续搜索
            continue;
        }
        
        total_frames++;
        
        // ========== 读取消息结构 ==========
        MsgStruct msg_header;
        
        // 已经读了'B'和'R'，现在读剩余部分
        if (!file.read(reinterpret_cast<char*>(&msg_header.len), sizeof(msg_header.len))) break;
        if (!file.read(reinterpret_cast<char*>(&msg_header.cmd), sizeof(msg_header.cmd))) break;
        if (!file.read(reinterpret_cast<char*>(&msg_header.src), sizeof(msg_header.src))) break;
        if (!file.read(reinterpret_cast<char*>(&msg_header.dst), sizeof(msg_header.dst))) break;
        
        // ========== 忽略cmd=4消息（通信握手包） ==========
        if (msg_header.cmd == 4) {
            // 跳过cmd=4消息数据
            file.seekg(msg_header.len + 2, std::ios::cur);
            continue;
        }
        
        // ========== 只处理cmd=2300消息 ==========
        if (msg_header.cmd != 2300) {
            // 跳过非2300且非4的消息（包括数据 + 校验和）
            file.seekg(msg_header.len + 2, std::ios::cur);
            continue;
        }
        
        msg2300_count++;
        
        // ========== 读取2300消息数据 ==========
        std::vector<uint8_t> msgdata(msg_header.len);
        if (!file.read(reinterpret_cast<char*>(msgdata.data()), msg_header.len)) {
            ROS_WARN("Failed to read msgdata for frame %d", total_frames);
            break;
        }
        
        // 跳过校验和
        file.seekg(2, std::ios::cur);
        
        // ========== 解析2300消息 ==========
        if (msgdata.size() < sizeof(MsgCmd2300)) {
            ROS_WARN("Frame %d: msg2300 data too short", total_frames);
            continue;
        }
        
        MsgCmd2300* cmd = reinterpret_cast<MsgCmd2300*>(msgdata.data());
        
        SonarPoint point;
        
        // ========== 关键修复：使用cmd->angle作为角度（乘以0.9） ==========
        float angle_deg = static_cast<float>(cmd->angle) * 0.9f;
        
        // 检查是否是无效角度值（6144）
        if (cmd->angle == 6144) {
            invalid_angle_count++;
            if (invalid_angle_count <= 10) {
                ROS_WARN("Frame %d: Invalid angle value 6144, skipping", total_frames);
            }
            continue;  // 跳过这一帧
        }
        
        // 规范化到0-360度
        while (angle_deg >= 360.0f) angle_deg -= 360.0f;
        while (angle_deg < 0.0f) angle_deg += 360.0f;
        
        point.heading_angle = angle_deg * M_PI / 180.0f;
        
        // ========== 距离计算 ==========
        float sample_time_us = static_cast<float>(cmd->sample_period) * 25.0f / 1000.0f;
        float distance_per_sample_mm = sample_time_us * 1.45f;
        float distance_per_sample_m = distance_per_sample_mm * 0.001f;
        
        // ========== 解析强度数据 ==========
        size_t data_start = sizeof(MsgCmd2300);
        size_t max_samples = std::min(static_cast<size_t>(cmd->length), 
                                     msgdata.size() - data_start);
        
        int valid_samples = 0;
        
        // 处理每个采样点
        for (size_t i = 0; i < max_samples; i++) {
            uint8_t intensity_raw = msgdata[data_start + i];
            float intensity = static_cast<float>(intensity_raw);
            
            // 计算距离（米）
            float distance = distance_per_sample_m * (i + 1) * distance_scale_;
            
            // 应用过滤条件
            if (distance < min_range_ || distance > max_range_ || 
                intensity < min_intensity_) {
                continue;
            }
            
            // 添加到点
            point.angles.push_back(point.heading_angle);
            point.ranges.push_back(distance);
            point.intensities.push_back(intensity);
            valid_samples++;
        }
        
        if (valid_samples > 0) {
            point.timestamp = ros::Time::now();
            point.valid = true;
            
            // 应用扇形波束展开
            if (enable_beam_spreading_) {
                applyBeamSpreading(point, 15.0f);
            }
            
            sonar_points_.push_back(point);
            
            // 减少日志输出频率
            if (sonar_points_.size() % 50 == 0 && msg2300_count <= 100) {
                ROS_INFO("Frame %d: msg2300#%d, angle=%.1f°, samples=%d, range=%.2f-%.2fm", 
                        total_frames, msg2300_count, angle_deg, valid_samples,
                        point.ranges.front(), point.ranges.back());
            }
        }
        
        // 进度显示
        if (total_frames % 100 == 0) {
            float progress = static_cast<float>(file.tellg()) / file_size * 100.0f;
            ROS_INFO("Progress: %.1f%%, Frames: %d, msg2300: %d, Valid: %lu, Invalid angles: %d", 
                    progress, total_frames, msg2300_count, sonar_points_.size(), invalid_angle_count);
        }
        
        // 只保留文件结束的检查
        if (file.tellg() >= file_size) {
            break;
        }
    }
    
    file.close();
    
    if (sonar_points_.empty()) {
        ROS_ERROR("No valid sonar points parsed from file");
        return false;
    }
    
    data_loaded_ = true;
    
    // ========== 详细的统计信息 ==========
    ROS_INFO("========================================");
    ROS_INFO("PARSING COMPLETED!");
    ROS_INFO("========================================");
    ROS_INFO("Total frames scanned: %d", total_frames);
    ROS_INFO("CMD=2300 frames (sonar data): %d", msg2300_count);
    ROS_INFO("Valid sonar frames: %lu", sonar_points_.size());
    ROS_INFO("Invalid angle values (6144): %d", invalid_angle_count);
    
    // 声纳帧角度分布
    if (!sonar_points_.empty()) {
        std::vector<float> angles;
        for (const auto& point : sonar_points_) {
            angles.push_back(point.heading_angle * 180.0f / M_PI);
        }
        
        std::sort(angles.begin(), angles.end());
        ROS_INFO("Sonar angle range: %.1f° to %.1f°", angles.front(), angles.back());
        
        // 角度分布（每10度）
        std::map<int, int> angle_dist;
        for (float angle : angles) {
            int bin = static_cast<int>(angle / 10.0f) * 10;
            angle_dist[bin]++;
        }
        
        ROS_INFO("Sonar angle distribution (10° bins):");
        for (int deg = 0; deg < 360; deg += 10) {
            if (angle_dist[deg] > 0) {
                ROS_INFO("  %3d°-%3d°: %3d frames", deg, deg+9, angle_dist[deg]);
            } else {
                ROS_WARN("  %3d°-%3d°: MISSING!", deg, deg+9);
            }
        }
        
        // 统计角度覆盖度
        int covered_bins = 0;
        for (int deg = 0; deg < 360; deg += 10) {
            if (angle_dist[deg] > 0) covered_bins++;
        }
        float coverage = static_cast<float>(covered_bins) / 36.0f * 100.0f;
        ROS_INFO("Angle coverage: %.1f%% (%d/36 bins)", coverage, covered_bins);
    }
    
    ROS_INFO("========================================");
    
    return true;
}

void SonarMapBuilder::generateTestData()
{
    ROS_INFO("Generating test data (rectangular pool)...");
    
    sonar_points_.clear();
    
    // 生成模拟的游泳池数据（10x5米的矩形）
    float pool_width = 10.0f;
    float pool_height = 5.0f;
    
    // 从中心点向四周扫描
    for (int angle_deg = 0; angle_deg < 360; angle_deg += 5) {
        SonarPoint point;
        point.heading_angle = angle_deg * M_PI / 180.0f;
        point.timestamp = ros::Time::now();
        
        // 计算与游泳池边界的交点
        float angle_rad = point.heading_angle;
        
        // 检查与四条边的交点
        std::vector<float> intersections;
        
        // 与上下边（y = ±pool_height/2）的交点
        if (fabs(sin(angle_rad)) > 1e-6) {
            float t_top = (pool_height/2) / sin(angle_rad);
            float x_top = t_top * cos(angle_rad);
            if (fabs(x_top) <= pool_width/2) {
                float range = sqrt(x_top*x_top + (pool_height/2)*(pool_height/2));
                if (range <= max_range_ && range >= min_range_) {
                    intersections.push_back(range);
                }
            }
            
            float t_bottom = (-pool_height/2) / sin(angle_rad);
            float x_bottom = t_bottom * cos(angle_rad);
            if (fabs(x_bottom) <= pool_width/2) {
                float range = sqrt(x_bottom*x_bottom + (-pool_height/2)*(-pool_height/2));
                if (range <= max_range_ && range >= min_range_) {
                    intersections.push_back(range);
                }
            }
        }
        
        // 与左右边（x = ±pool_width/2）的交点
        if (fabs(cos(angle_rad)) > 1e-6) {
            float t_left = (-pool_width/2) / cos(angle_rad);
            float y_left = t_left * sin(angle_rad);
            if (fabs(y_left) <= pool_height/2) {
                float range = sqrt((-pool_width/2)*(-pool_width/2) + y_left*y_left);
                if (range <= max_range_ && range >= min_range_) {
                    intersections.push_back(range);
                }
            }
            
            float t_right = (pool_width/2) / cos(angle_rad);
            float y_right = t_right * sin(angle_rad);
            if (fabs(y_right) <= pool_height/2) {
                float range = sqrt((pool_width/2)*(pool_width/2) + y_right*y_right);
                if (range <= max_range_ && range >= min_range_) {
                    intersections.push_back(range);
                }
            }
        }
        
        // 取最近的有效交点
        if (!intersections.empty()) {
            float min_range = *std::min_element(intersections.begin(), intersections.end());
            point.ranges.push_back(min_range);
            point.intensities.push_back(200.0f);  // 假设墙壁反射强
            
            // 添加一些噪声点
            for (int i = 0; i < 5; i++) {
                float noise_range = min_range + (i+1)*0.1f;
                if (noise_range <= max_range_) {
                    point.ranges.push_back(noise_range);
                    point.intensities.push_back(50.0f + i*10.0f);
                }
            }
        }
        
        if (!point.ranges.empty()) {
            sonar_points_.push_back(point);
        }
    }
    
    data_loaded_ = !sonar_points_.empty();
    ROS_INFO("Generated %lu test sonar frames", sonar_points_.size());
}

void SonarMapBuilder::processAndPublish()
{
    if (!data_loaded_) {
        ROS_ERROR("No data loaded to process");
        return;
    }
    
    ROS_INFO("Processing sonar data and building map...");
    
    // 转换为点云
    convertToPointCloud();

    // 增强：填补缺失的角度
    enhanceMissingAngles();
    
    // 过滤点云
    filterPoints(max_range_, min_intensity_);
    
    // 创建占据栅格地图
    createOccupancyGrid();
    
    // 发布所有数据
    publishPointCloud();
    publishLaserScan();
    publishOccupancyGrid();
    publishMarker();
    publishTF();
    
    ROS_INFO("========================================");
    ROS_INFO("Map Building Complete!");
    ROS_INFO("========================================");
    ROS_INFO("Results:");
    ROS_INFO("  - Point cloud size: %lu points", pointcloud_->size());
    ROS_INFO("  - Map size: %dx%d cells", map_width_, map_height_);
    ROS_INFO("  - Map resolution: %.3f m/cell", map_resolution_);
    ROS_INFO("========================================");
    ROS_INFO("Published topics:");
    ROS_INFO("  - PointCloud2: /sonar_map_pointcloud");
    ROS_INFO("  - LaserScan: /sonar_map_scan");
    ROS_INFO("  - OccupancyGrid: /sonar_occupancy_map");
    ROS_INFO("  - Marker: /sonar_map_marker");
    ROS_INFO("========================================");
    
    // 保存点云到文件（可选）
    if (!pointcloud_->empty()) {
        std::string pcd_file = "/tmp/sonar_map.pcd";
        pcl::io::savePCDFileASCII(pcd_file, *pointcloud_);
        ROS_INFO("Point cloud saved to: %s", pcd_file.c_str());
    }
}

// 扇形波束展开函数
// 修改扇形展开，考虑反射角度
void SonarMapBuilder::applyBeamSpreading(SonarPoint& point, float beam_width_deg)
{
    if (point.ranges.empty() || !enable_beam_spreading_) {
        return;
    }
    
    float beam_width_rad = beam_width_deg * M_PI / 180.0f;
    const int num_subdivisions = 7;  // 增加子束数量
    
    std::vector<float> new_angles;
    std::vector<float> new_ranges;
    std::vector<float> new_intensities;
    
    // 计算角度修正（考虑墙壁反射特性）
    float angle_correction_factor = 1.0f;
    
    // 根据角度调整展开策略
    float angle_deg = point.heading_angle * 180.0f / M_PI;
    
    // 对于直接面对的墙壁（0-30°, 150-210°, 330-360°），减少展开
    if ((angle_deg >= 0 && angle_deg < 30) || 
        (angle_deg >= 150 && angle_deg < 210) || 
        (angle_deg >= 330 && angle_deg < 360)) {
        beam_width_rad *= 0.7f;  // 减小波束宽度
    }
    // 对于侧面的墙壁，增加展开
    else if ((angle_deg >= 60 && angle_deg < 120) || 
             (angle_deg >= 240 && angle_deg < 300)) {
        beam_width_rad *= 1.3f;  // 增加波束宽度
    }
    
    // 处理每个点
    for (size_t i = 0; i < point.ranges.size(); i++) {
        float base_angle = point.heading_angle;
        if (i < point.angles.size()) {
            base_angle = point.angles[i];
        }
        
        float range = point.ranges[i];
        float intensity = point.intensities[i];
        
        // 根据距离调整展开强度
        float distance_factor = 1.0f - (range / max_range_) * 0.3f;
        
        for (int sub = 0; sub < num_subdivisions; sub++) {
            float offset_ratio = (sub - (num_subdivisions - 1) / 2.0f) / (num_subdivisions - 1);
            float offset = offset_ratio * beam_width_rad / 2.0f;
            
            // 距离越远，展开越大（模拟散射）
            offset *= (1.0f + range / max_range_ * 0.2f);
            
            new_angles.push_back(base_angle + offset);
            new_ranges.push_back(range);
            
            // 强度衰减：中心强，边缘弱，远距离衰减更多
            float intensity_factor = 1.0f - fabs(offset_ratio) * 0.4f;
            intensity_factor *= distance_factor;  // 距离衰减
            
            // 对于弱信号，降低衰减
            if (intensity < min_intensity_ * 2.0f) {
                intensity_factor = 1.0f - fabs(offset_ratio) * 0.2f;
            }
            
            new_intensities.push_back(intensity * intensity_factor);
        }
    }
    
    point.angles = new_angles;
    point.ranges = new_ranges;
    point.intensities = new_intensities;
}

void SonarMapBuilder::convertToPointCloud() 
{
    pointcloud_->clear();
    pointcloud_->header.frame_id = "sonar_map_frame";
    pointcloud_->width = 0;
    pointcloud_->height = 1;
    pointcloud_->is_dense = false;
    
    size_t total_points = 0;
    
    // 调试：查看实际使用的角度校正值
    ROS_INFO("=== 角度校正测试开始 ===");
    
    // 测试几个常见的校正值
    std::vector<float> candidate_corrections = {
        0.0f,      // 0°
        -M_PI_2,   // -90° (当前使用的)
        M_PI_2,    // 90°
        M_PI,      // 180°
        -M_PI,     // -180°
        M_PI/4,    // 45°
        -M_PI/4    // -45°
    };
    
    std::vector<std::string> correction_names = {
        "0°", "-90°", "90°", "180°", "-180°", "45°", "-45°"
    };
    
    PointCloudI::Ptr best_cloud;
    float best_score = -1.0f;
    float best_correction = 0.0f;
    std::string best_name = "0°";
    
    for (size_t idx = 0; idx < candidate_corrections.size(); idx++) {
        float angle_correction = candidate_corrections[idx];
        std::string name = correction_names[idx];
        
        PointCloudI::Ptr test_cloud(new PointCloudI());
        test_cloud->header.frame_id = "sonar_map_frame";
        
        int points_in_cloud = 0;
        float sum_x = 0.0f, sum_y = 0.0f;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        // 转换点云
        for (const auto& sonar_point : sonar_points_) {
            if (!sonar_point.valid || sonar_point.empty()) continue;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                float range = sonar_point.ranges[i];
                float angle = sonar_point.angles[i];
                float intensity = sonar_point.intensities[i];
                
                if (range < min_range_ || range > max_range_) continue;
                
                float corrected_angle = angle + angle_correction;
                
                pcl::PointXYZI point;
                point.x = range * cosf(corrected_angle);
                point.y = range * sinf(corrected_angle);
                point.z = 0.0f;
                point.intensity = intensity;
                
                test_cloud->points.push_back(point);
                points_in_cloud++;
                
                // 统计信息
                sum_x += point.x;
                sum_y += point.y;
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }
        }
        
        if (points_in_cloud > 0) {
            test_cloud->width = test_cloud->points.size();
            test_cloud->height = 1;
            
            // 计算评分：对称性 + 面积 + 点数密度
            float center_x = sum_x / points_in_cloud;
            float center_y = sum_y / points_in_cloud;
            float width = max_x - min_x;
            float height = max_y - min_y;
            
            // 对称性评分（中心越接近0分越高）
            float symmetry_score = 100.0f / (1.0f + fabs(center_x) + fabs(center_y));
            
            // 面积评分
            float area = width * height;
            float area_score = area / 100.0f; // 标准化
            
            // 点数密度评分
            float density_score = static_cast<float>(points_in_cloud) / area;
            
            // 综合评分（对称性最重要）
            float total_score = symmetry_score * 0.6f + area_score * 0.2f + density_score * 0.2f;
            
            ROS_INFO("校正 %s: %d点, 中心(%.3f,%.3f), 宽%.2f×高%.2f, 对称性%.1f, 总分%.2f",
                    name.c_str(), points_in_cloud, center_x, center_y, 
                    width, height, symmetry_score, total_score);
            
            if (total_score > best_score) {
                best_score = total_score;
                best_cloud = test_cloud;
                best_correction = angle_correction;
                best_name = name;
            }
        }
    }
    
    ROS_INFO("=== 选择最佳校正: %s (%.1f°) ===", best_name.c_str(), best_correction * 180.0 / M_PI);
    // ==================== 添加对称增强代码开始 ====================
    ROS_INFO("=== 开始对称增强处理 ===");
    
    // 计算点云质心
    float sum_x = 0, sum_y = 0;
    for (const auto& point : best_cloud->points) {
        sum_x += point.x;
        sum_y += point.y;
    }
    float center_x = sum_x / best_cloud->size();
    float center_y = sum_y / best_cloud->size();
    
    ROS_INFO("点云质心: (%.2f, %.2f)", center_x, center_y);
    
    // 检查是否需要对称增强
    bool enable_symmetry = false;
    nh_.param("enable_symmetry", enable_symmetry, false);
    
    PointCloudI::Ptr final_cloud = best_cloud;
    
    if (enable_symmetry && (fabs(center_x) > 1.0f || fabs(center_y) > 1.0f)) {
        ROS_WARN("数据采集位置明显偏移 (%.2f, %.2f)，进行对称增强", center_x, center_y);
        
        PointCloudI::Ptr enhanced_cloud(new PointCloudI());
        enhanced_cloud->header = best_cloud->header;
        
        // 方法：镜像对称增强
        int added_points = 0;
        for (const auto& point : best_cloud->points) {
            // 保留原始点
            enhanced_cloud->points.push_back(point);
            
            // 添加对称点（关于原点对称）- 只对强信号点
            if (point.intensity > min_intensity_ * 2.0f) {
                pcl::PointXYZI sym_point;
                sym_point.x = -point.x * 0.7f;   // 对称且稍微减弱
                sym_point.y = -point.y * 0.7f;
                sym_point.z = 0.0f;
                sym_point.intensity = point.intensity * 0.5f;  // 降低强度
                enhanced_cloud->points.push_back(sym_point);
                added_points++;
            }
        }
        
        enhanced_cloud->width = enhanced_cloud->size();
        enhanced_cloud->height = 1;
        final_cloud = enhanced_cloud;
        
        ROS_INFO("对称增强完成: %lu -> %lu 点，新增 %d 对称点", 
                 best_cloud->size(), enhanced_cloud->size(), added_points);
    } else if (enable_symmetry) {
        ROS_INFO("点云质心接近中心，跳过对称增强");
    }
    // 使用最佳校正
    if (best_cloud && !best_cloud->empty()) {
        pointcloud_ = best_cloud;
        total_points = pointcloud_->size();
    } else {
        ROS_ERROR("没有生成有效的点云！使用默认校正");
        // 回退到默认
        float default_correction = 0.0f;
        for (const auto& sonar_point : sonar_points_) {
            if (!sonar_point.valid || sonar_point.empty()) continue;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                float range = sonar_point.ranges[i];
                float angle = sonar_point.angles[i];
                float intensity = sonar_point.intensities[i];
                
                if (range < min_range_ || range > max_range_) continue;
                
                float corrected_angle = angle + default_correction;
                
                pcl::PointXYZI point;
                point.x = range * cosf(corrected_angle);
                point.y = range * sinf(corrected_angle);
                point.z = 0.0f;
                point.intensity = intensity;
                
                pointcloud_->points.push_back(point);
                total_points++;
            }
        }
        pointcloud_->width = pointcloud_->size();
    }
    
    // 计算最终边界
    if (!pointcloud_->empty()) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& point : pointcloud_->points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        
        ROS_INFO("最终点云: %lu 点, 边界 x=[%.2f, %.2f], y=[%.2f, %.2f]",
                total_points, min_x, max_x, min_y, max_y);
        ROS_INFO("点云中心: (%.3f, %.3f)", (min_x + max_x)/2.0f, (min_y + max_y)/2.0f);
    }
}

void SonarMapBuilder::enhanceMissingAngles()
{
    if (pointcloud_->empty()) return;
    
    // 统计角度分布
    std::map<int, int> angle_histogram; // 每度统计
    
    for (const auto& point : pointcloud_->points) {
        float angle = atan2(point.y, point.x) * 180.0 / M_PI;
        if (angle < 0) angle += 360.0f;
        int deg = static_cast<int>(angle);
        angle_histogram[deg]++;
    }
    
    // 找出缺失的角度
    std::vector<int> missing_angles;
    for (int deg = 0; deg < 360; deg++) {
        if (angle_histogram[deg] == 0) {
            missing_angles.push_back(deg);
        }
    }
    
    if (!missing_angles.empty()) {
        ROS_INFO("发现 %lu 个缺失角度，进行插值填充", missing_angles.size());
        
        PointCloudI::Ptr enhanced_cloud(new PointCloudI(*pointcloud_));
        
        // 对每个缺失角度进行插值
        for (int missing_deg : missing_angles) {
            // 找到前后最近的有效角度
            int prev_deg = missing_deg - 1;
            int next_deg = missing_deg + 1;
            
            while (prev_deg >= 0 && angle_histogram[prev_deg] == 0) prev_deg--;
            while (next_deg < 360 && angle_histogram[next_deg] == 0) next_deg++;
            
            if (prev_deg >= 0 && next_deg < 360) {
                // 收集前后角度的点
                std::vector<pcl::PointXYZI> prev_points, next_points;
                for (const auto& point : pointcloud_->points) {
                    float angle = atan2(point.y, point.x) * 180.0 / M_PI;
                    if (angle < 0) angle += 360.0f;
                    int deg = static_cast<int>(angle);
                    
                    if (deg == prev_deg) prev_points.push_back(point);
                    else if (deg == next_deg) next_points.push_back(point);
                }
                
                // 插值生成新点
                if (!prev_points.empty() && !next_points.empty()) {
                    size_t num_to_add = std::min(prev_points.size(), next_points.size());
                    float missing_rad = missing_deg * M_PI / 180.0f;
                    
                    for (size_t i = 0; i < num_to_add; i++) {
                        float avg_range = (sqrt(prev_points[i].x*prev_points[i].x + prev_points[i].y*prev_points[i].y) +
                                         sqrt(next_points[i].x*next_points[i].x + next_points[i].y*next_points[i].y)) / 2.0f;
                        float avg_intensity = (prev_points[i].intensity + next_points[i].intensity) / 2.0f;
                        
                        pcl::PointXYZI new_point;
                        new_point.x = avg_range * cosf(missing_rad);
                        new_point.y = avg_range * sinf(missing_rad);
                        new_point.z = 0.0f;
                        new_point.intensity = avg_intensity * 0.5f; // 降低插值点强度
                        
                        enhanced_cloud->points.push_back(new_point);
                    }
                }
            }
        }
        
        enhanced_cloud->width = enhanced_cloud->size();
        enhanced_cloud->height = 1;
        
        pointcloud_ = enhanced_cloud;
        ROS_INFO("角度插值完成: 新增 %lu 点", enhanced_cloud->size() - pointcloud_->size());
    }
}

void SonarMapBuilder::filterPoints(float max_range, float min_intensity)
{
    PointCloudI::Ptr filtered(new PointCloudI());
    filtered->header = pointcloud_->header;
    
    // 统计直方图用于过滤
    std::map<int, int> angle_histogram;
    const int BIN_SIZE = 5; // 5度一个桶
    
    // 先收集所有点
    for (const auto& point : pointcloud_->points) {
        float distance = sqrtf(point.x * point.x + point.y * point.y);
        float angle = atan2(point.y, point.x) * 180.0 / M_PI;
        int bin = static_cast<int>((angle + 180.0) / BIN_SIZE); // -180~180转0~360
        
        // 基本过滤
        if (distance <= max_range && point.intensity >= min_intensity) {
            filtered->points.push_back(point);
            angle_histogram[bin]++;
        }
    }
    
    // 计算平均点数
    int total_filtered_points = filtered->points.size();
    int num_bins = 360 / BIN_SIZE;
    int avg_points = num_bins > 0 ? total_filtered_points / num_bins : 0;
    
    // 找出稀疏的角度区域
    std::vector<int> sparse_bins;
    for (const auto& bin : angle_histogram) {
        if (avg_points > 0 && bin.second < avg_points / 5) { // 少于平均值的1/5
            sparse_bins.push_back(bin.first);
        }
    }
    
    ROS_INFO("Filtered to %lu points, sparse angle bins: %lu", 
            filtered->points.size(), sparse_bins.size());
    
    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;
    
    pointcloud_ = filtered;
}



void SonarMapBuilder::createOccupancyGrid()
{
    if (pointcloud_->empty()) {
        ROS_WARN("No points to create occupancy grid");
        return;
    }
    
    // 1. 计算点云的边界
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    
    for (const auto& point : pointcloud_->points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    
    ROS_INFO("Point cloud bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]", 
            min_x, max_x, min_y, max_y);
    
    // 2. 设置合适的地图参数
    float cloud_width = max_x - min_x;
    float cloud_height = max_y - min_y;
    
    // 留出1米的边界
    float border = 1.0f;
    map_width_ = static_cast<int>((cloud_width + 2*border) / map_resolution_);
    map_height_ = static_cast<int>((cloud_height + 2*border) / map_resolution_);
    
    // 设置地图原点
    map_origin_x_ = min_x - border;
    map_origin_y_ = min_y - border;
    
    ROS_INFO("Adjusted map: size=%dx%d (%.1fx%.1fm), origin=(%.2f, %.2f)", 
            map_width_, map_height_, 
            map_width_ * map_resolution_, map_height_ * map_resolution_,
            map_origin_x_, map_origin_y_);
    
    // 3. 初始化占据栅格地图
    occupancy_grid_.header.frame_id = "map";
    occupancy_grid_.header.stamp = ros::Time::now();
    occupancy_grid_.info.resolution = map_resolution_;
    occupancy_grid_.info.width = map_width_;
    occupancy_grid_.info.height = map_height_;
    occupancy_grid_.info.origin.position.x = map_origin_x_;
    occupancy_grid_.info.origin.position.y = map_origin_y_;
    occupancy_grid_.info.origin.position.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;
    
    // 初始化地图数据（-1表示未知，0表示空闲）
    occupancy_grid_.data.resize(map_width_ * map_height_, -1);
    
    int occupied_cells = 0;
    
    // 4. 先标记空闲区域（机器人可能的位置）
    // 假设机器人从中心开始扫描，中心区域应该是空闲的
    int center_x = map_width_ / 2;
    int center_y = map_height_ / 2;
    int free_radius = static_cast<int>(0.5f / map_resolution_); // 0.5米半径内空闲
    
    for (int y = center_y - free_radius; y <= center_y + free_radius; y++) {
        for (int x = center_x - free_radius; x <= center_x + free_radius; x++) {
            if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_) {
                int index = y * map_width_ + x;
                if (occupancy_grid_.data[index] == -1) { // 只有未知区域才标记为空闲
                    occupancy_grid_.data[index] = 0;
                }
            }
        }
    }
    
    // 5. 将点云投影到栅格地图（障碍物）
    for (const auto& point : pointcloud_->points) {
        float distance = sqrt(point.x * point.x + point.y * point.y);
        // ==================== 新增：智能距离过滤 ====================
        // 不对称过滤：根据点的方向决定距离阈值

        // 计算点的角度和方向
        float angle = atan2(point.y, point.x) * 180.0f / M_PI;
        if (angle < 0) angle += 360.0f;

        // 定义方向区域（基于你的采集位置在左侧）
        // 假设：0°=正前方，90°=右侧，180°=后方，270°=左侧
        bool is_near_side = false;      // 采集侧（近侧）
        bool is_far_side = false;       // 对侧（远侧）
        bool is_side_wall = false;      // 侧墙

        // 根据角度判断点的方向
        if ((angle >= 315 && angle <= 360) || (angle >= 0 && angle <= 45)) {
            // 前方区域（0°±45°）- 可能是对侧墙壁
            is_far_side = true;
        } else if (angle >= 135 && angle <= 225) {
            // 后方区域（180°±45°）- 可能是采集侧墙壁
            is_near_side = true;
        } else {
            // 侧方区域 - 可能是侧墙
            is_side_wall = true;
        }

        // 动态距离阈值
        float dynamic_max_range = max_range_;  // 默认使用max_range_

        if (is_near_side) {
            // 采集侧：使用较小的范围（避免房间墙壁）
            dynamic_max_range = std::min(max_range_, 3.0f);  // 限制在3米内
        } else if (is_far_side) {
            // 对侧：保持较大范围（确保能探测到）
            dynamic_max_range = max_range_;  // 使用完整的5米
        } else if (is_side_wall) {
            // 侧墙：中等范围
            dynamic_max_range = std::min(max_range_, 4.0f);  // 限制在4米内
        }

 
        // 只处理有效距离内的点
        if (distance < min_range_ || distance > max_range_) {
            continue;
        }
        
        // 转换为地图坐标
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);
        
        // 检查边界
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            int index = map_y * map_width_ + map_x;
            
            // 根据强度设置占据概率
            float intensity_ratio = point.intensity / 255.0f;

            // ==================== 新增：方向性强度阈值 ====================
            // 动态强度阈值：不同方向要求不同信号强度
            float dynamic_intensity_threshold = 0.1f;  // 默认10%

            if (is_near_side) {
                // 采集侧：提高阈值（减少近场噪声）
                dynamic_intensity_threshold = 0.15f;  // 15%
            } else if (is_far_side) {
                // 对侧：降低阈值（捕捉弱信号）
                dynamic_intensity_threshold = 0.08f;  // 8%
            } else if (is_side_wall) {
                // 侧墙：中等阈值
                dynamic_intensity_threshold = 0.12f;  // 12%
            }
            
            // 强度阈值调整为更合理的值 
            if (intensity_ratio > dynamic_intensity_threshold) {   
            // if (intensity_ratio > 0.1f) {  // 10%强度以上认为是障碍
                int occupancy = static_cast<int>(intensity_ratio * 100.0f);
                occupancy = std::min(100, std::max(1, occupancy));  // 限制在1-100
                
                // 重要：只覆盖未知区域，不覆盖已标记为空闲的区域
                if (occupancy_grid_.data[index] <= 0) { // -1(未知)或0(空闲)
                    occupancy_grid_.data[index] = occupancy;
                    occupied_cells++;
                }
                
                // 填充周围区域（创建连续的墙壁）
                if (occupancy > 50) {
                    // 小范围填充，避免过度填充
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = map_x + dx;
                            int ny = map_y + dy;
                            
                            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                                int nidx = ny * map_width_ + nx;
                                if (occupancy_grid_.data[nidx] <= 0) { // 只覆盖未知或空闲
                                    int neighbor_occ = occupancy / 2;
                                    if (neighbor_occ > 10) {
                                        occupancy_grid_.data[nidx] = neighbor_occ;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 6. 增强墙壁连续性（霍夫变换思路简化版）
    std::vector<int8_t> enhanced_data = occupancy_grid_.data;
    
    // 检测水平墙壁
    for (int y = 1; y < map_height_ - 1; y++) {
        for (int x = 1; x < map_width_ - 1; x++) {
            int idx = y * map_width_ + x;
            
            // 如果当前点可能是墙壁
            // if (occupancy_grid_.data[idx] > 30) {

            //     // 检查水平连续性
            //     int left_idx = y * map_width_ + (x - 1);
            //     int right_idx = y * map_width_ + (x + 1);
                
            //     // 如果左右都有墙壁点，加强连续性
            //     if ((occupancy_grid_.data[left_idx] > 20 || occupancy_grid_.data[right_idx] > 20)) {
            //         // 填充中间缺失的点
            //         for (int dx = -2; dx <= 2; dx++) {
            //             int nx = x + dx;
            //             if (nx >= 0 && nx < map_width_) {
            //                 int nidx = y * map_width_ + nx;
            //                 if (enhanced_data[nidx] < occupancy_grid_.data[idx]) {
            //                     enhanced_data[nidx] = occupancy_grid_.data[idx] * 0.8f;
            //                 }
            //             }
            //         }
            //     }
                
            //     // 检查垂直连续性
            //     int up_idx = (y - 1) * map_width_ + x;
            //     int down_idx = (y + 1) * map_width_ + x;
                
            //     if ((occupancy_grid_.data[up_idx] > 20 || occupancy_grid_.data[down_idx] > 20)) {
            //         // 填充垂直方向
            //         for (int dy = -2; dy <= 2; dy++) {
            //             int ny = y + dy;
            //             if (ny >= 0 && ny < map_height_) {
            //                 int nidx = ny * map_width_ + x;
            //                 if (enhanced_data[nidx] < occupancy_grid_.data[idx]) {
            //                     enhanced_data[nidx] = occupancy_grid_.data[idx] * 0.8f;
            //                 }
            //             }
            //         }
            //     }


            // }

            if (enhanced_data[idx] > 30) {
                // 检查这个点是否可能是游泳池墙壁（而不是房间墙壁）
                // 计算该网格的世界坐标
                float world_x = map_origin_x_ + x * map_resolution_;
                float world_y = map_origin_y_ + y * map_resolution_;
                float cell_distance = sqrt(world_x*world_x + world_y*world_y);
                float cell_angle = atan2(world_y, world_x) * 180.0f / M_PI;
                if (cell_angle < 0) cell_angle += 360.0f;
                
                // 判断是否可能是游泳池墙壁
                bool is_likely_pool_wall = false;
                
                // 规则1：距离在合理范围内
                if (cell_distance > 1.5f && cell_distance < 4.5f) {
                    // 规则2：角度接近直角（0°,90°,180°,270°）
                    for (int base_angle : {0, 90, 180, 270}) {
                        float angle_diff = fabs(cell_angle - base_angle);
                        angle_diff = std::min(angle_diff, 360.0f - angle_diff);
                        if (angle_diff < 20.0f) {  // ±20°范围内
                            is_likely_pool_wall = true;
                            break;
                        }
                    }
                }
                
                // 只对可能是游泳池墙壁的点进行连续性增强
                if (is_likely_pool_wall) {
                    // 检查水平连续性
                    int left_idx = y * map_width_ + (x - 1);
                    int right_idx = y * map_width_ + (x + 1);
                    
                    // 如果左右都有墙壁点，加强连续性
                    if ((enhanced_data[left_idx] > 20 || enhanced_data[right_idx] > 20)) {
                        // 填充中间缺失的点
                        for (int dx = -2; dx <= 2; dx++) {
                            int nx = x + dx;
                            if (nx >= 0 && nx < map_width_) {
                                int nidx = y * map_width_ + nx;
                                if (enhanced_data[nidx] < enhanced_data[idx]) {
                                    enhanced_data[nidx] = enhanced_data[idx] * 0.8f;
                                }
                            }
                        }
                    }
                    
                    // 检查垂直连续性
                    int up_idx = (y - 1) * map_width_ + x;
                    int down_idx = (y + 1) * map_width_ + x;
                    
                    if ((enhanced_data[up_idx] > 20 || enhanced_data[down_idx] > 20)) {
                        // 填充垂直方向
                        for (int dy = -2; dy <= 2; dy++) {
                            int ny = y + dy;
                            if (ny >= 0 && ny < map_height_) {
                                int nidx = ny * map_width_ + x;
                                if (enhanced_data[nidx] < enhanced_data[idx]) {
                                    enhanced_data[nidx] = enhanced_data[idx] * 0.8f;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    occupancy_grid_.data = enhanced_data;
    
    // 7. 统计并输出结果
    int total_cells = map_width_ * map_height_;
    float occupied_percent = static_cast<float>(occupied_cells) / total_cells * 100.0f;
    
    // 统计未知、空闲、占据的单元格数
    int unknown_cells = 0;
    int free_cells = 0;
    for (int i = 0; i < total_cells; i++) {
        if (occupancy_grid_.data[i] == -1) unknown_cells++;
        else if (occupancy_grid_.data[i] == 0) free_cells++;
    }
    
    ROS_INFO("Map statistics:");
    ROS_INFO("  Total cells: %d", total_cells);
    ROS_INFO("  Occupied cells: %d (%.1f%%)", occupied_cells, occupied_percent);
    ROS_INFO("  Free cells: %d (%.1f%%)", free_cells, static_cast<float>(free_cells)/total_cells*100.0f);
    ROS_INFO("  Unknown cells: %d (%.1f%%)", unknown_cells, static_cast<float>(unknown_cells)/total_cells*100.0f);

    
}


void SonarMapBuilder::publishPointCloud()
{
    if (pointcloud_->empty()) {
        ROS_WARN("No point cloud data to publish");
        return;
    }
    
    // 设置时间戳
    pointcloud_->header.stamp = pcl_conversions::toPCL(ros::Time::now());
    
    // 发布点云
    pointcloud_pub_.publish(pointcloud_);
    
    ROS_INFO("Published point cloud with %lu points", pointcloud_->size());
}

void SonarMapBuilder::publishLaserScan()
{
    if (sonar_points_.empty()) {
        ROS_WARN("No sonar data to publish as laser scan");
        return;
    }
    
    // 创建激光扫描消息
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "sonar_map_frame";
    scan.header.stamp = ros::Time::now();
    
    // 参数设置
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = 2.0f * M_PI / 360.0f;
    scan.time_increment = 0.0f;
    scan.scan_time = 0.1f;
    scan.range_min = min_range_;
    scan.range_max = max_range_;
    
    // 初始化范围数组
    size_t num_readings = static_cast<size_t>((scan.angle_max - scan.angle_min) / 
                                              scan.angle_increment);
    scan.ranges.resize(num_readings, scan.range_max + 1.0f);
    scan.intensities.resize(num_readings, 0.0f);
    
    // 角度偏移
    const float angle_offset = -M_PI_2;
    
    // 填充扫描数据
    for (const auto& sonar_point : sonar_points_) {
        if (!sonar_point.valid) continue;
        
        // 使用中心角度
        float angle = sonar_point.heading_angle + angle_offset;
        
        // 规范化角度到[-π, π]
        while (angle > M_PI) angle -= 2.0f * M_PI;
        while (angle < -M_PI) angle += 2.0f * M_PI;
        
        // 找到最近的扫描角度索引
        int nearest_index = static_cast<int>((angle - scan.angle_min) / 
                                            scan.angle_increment + 0.5f);
        
        if (nearest_index >= 0 && nearest_index < static_cast<int>(scan.ranges.size())) {
            // 取最小距离（最近的点）
            float min_range = scan.range_max + 1.0f;
            float max_intensity = 0.0f;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                if (sonar_point.ranges[i] < min_range) {
                    min_range = sonar_point.ranges[i];
                    max_intensity = sonar_point.intensities[i];
                }
            }
            
            if (min_range <= scan.range_max) {
                scan.ranges[nearest_index] = min_range;
                scan.intensities[nearest_index] = max_intensity;
            }
        }
    }
    
    laser_pub_.publish(scan);
    ROS_INFO("Published laser scan with %lu readings", scan.ranges.size());
}

void SonarMapBuilder::publishOccupancyGrid()
{
    occupancy_grid_.header.stamp = ros::Time::now();
    occupancy_pub_.publish(occupancy_grid_);
    ROS_INFO("Published occupancy grid map");
}

void SonarMapBuilder::publishMarker()
{
    // 创建可视化标记
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sonar_map";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 点大小
    marker.scale.y = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration();
    
    // 添加点
    for (const auto& point : pointcloud_->points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        marker.points.push_back(p);
    }
    
    marker_pub_.publish(marker);
    ROS_INFO("Published visualization marker with %lu points", marker.points.size());
}

// void SonarMapBuilder::publishTF()
// {
//     ros::Time current_time = ros::Time::now();
    
//     // 发布完整的TF链
//     tf::Transform transform;
//     tf::Quaternion q;
//     q.setRPY(0, 0, 0);
    
//     // 1. map -> odom (恒等变换)
//     transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//     transform.setRotation(q);
//     tf_broadcaster_->sendTransform(
//         tf::StampedTransform(transform, current_time, "map", "odom"));
    
//     // 2. odom -> base_link (根据你的数据质心调整位置)
//     // 从你的日志看，数据质心在(1.65, -1.48)
//     // 为了在rviz中居中显示，我们可能需要调整这个偏移
//     float x_offset = 0.0;  // 先试0.0，不行再调整
//     float y_offset = 0.0;
//     transform.setOrigin(tf::Vector3(x_offset, y_offset, 0.0));
//     transform.setRotation(q);
//     tf_broadcaster_->sendTransform(
//         tf::StampedTransform(transform, current_time, "odom", "base_link"));
    
//     // 3. base_link -> sonar_map_frame (保持原来的)
//     transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//     transform.setRotation(q);
//     tf_broadcaster_->sendTransform(
//         tf::StampedTransform(transform, current_time, "base_link", "sonar_map_frame"));
    
//     ROS_INFO("Published TF chain: map->odom->base_link->sonar_map_frame");
// }

void SonarMapBuilder::publishTF()
{
    ros::Time current_time = ros::Time::now();
    
    // 发布TF链
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    
    // 1. map -> odom（恒等变换）
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, current_time, "map", "odom"));
    
    // 2. odom -> base_link（机器人位于地图中心）
    // 根据点云边界，计算地图中心
    if (!pointcloud_->empty()) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& point : pointcloud_->points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        
        // 地图中心坐标
        float center_x = (min_x + max_x) / 2.0f;
        float center_y = (min_y + max_y) / 2.0f;
        
        // 如果中心不在(0,0)，调整base_link位置
        if (fabs(center_x) > 0.1f || fabs(center_y) > 0.1f) {
            ROS_WARN("Adjusting base_link position to center of map: (%.2f, %.2f)", 
                    -center_x, -center_y);
            transform.setOrigin(tf::Vector3(-center_x, -center_y, 0.0));
        } else {
            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        }
    } else {
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    }
    
    transform.setRotation(q);
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, current_time, "odom", "base_link"));
    
    // 3. base_link -> sonar_map_frame（声纳传感器偏移）
    // 假设声纳在机器人正前方0.1米处
    transform.setOrigin(tf::Vector3(0.1, 0.0, 0.0));
    transform.setRotation(q);
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, current_time, "base_link", "sonar_map_frame"));
    
    ROS_INFO("Published TF chain: map->odom->base_link->sonar_map_frame");
}

void SonarMapBuilder::printDataSummary() const
{
    if (sonar_points_.empty()) {
        ROS_WARN("No data to summarize");
        return;
    }
    
    ROS_INFO("========================================");
    ROS_INFO("Data Summary");
    ROS_INFO("========================================");
    ROS_INFO("Total frames: %lu", sonar_points_.size());
    
    // 统计信息
    size_t total_points = 0;
    float min_range = std::numeric_limits<float>::max();
    float max_range = std::numeric_limits<float>::min();
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::min();
    
    for (const auto& point : sonar_points_) {
        total_points += point.ranges.size();
        for (size_t i = 0; i < point.ranges.size(); i++) {
            min_range = std::min(min_range, point.ranges[i]);
            max_range = std::max(max_range, point.ranges[i]);
            min_intensity = std::min(min_intensity, point.intensities[i]);
            max_intensity = std::max(max_intensity, point.intensities[i]);
        }
    }
    
    ROS_INFO("Total data points: %lu", total_points);
    ROS_INFO("Range: %.3f - %.3f m", min_range, max_range);
    ROS_INFO("Intensity: %.1f - %.1f", min_intensity, max_intensity);
    
    // 角度分布
    std::vector<int> angle_histogram(36, 0); // 每10度一个桶
    for (const auto& point : sonar_points_) {
        int angle_bin = static_cast<int>((point.heading_angle * 180.0 / M_PI) / 10.0);
        if (angle_bin >= 0 && angle_bin < 36) {
            angle_histogram[angle_bin]++;
        }
    }
    
    ROS_INFO("Angle distribution:");
    for (int i = 0; i < 36; i++) {
        if (angle_histogram[i] > 0) {
            ROS_INFO("  %3d°-%3d°: %d frames", i*10, (i+1)*10, angle_histogram[i]);
        }
    }
    ROS_INFO("========================================");
}

void SonarMapBuilder::saveDataToCSV(const std::string& filename) const
{
    std::ofstream csv_file(filename);
    if (!csv_file.is_open()) {
        ROS_ERROR("Cannot open CSV file: %s", filename.c_str());
        return;
    }
    
    // 写入CSV头部
    csv_file << "frame_id,heading_angle_deg,point_angle_deg,range,intensity,x,y" << std::endl;
    
    // 角度偏移
    const float angle_offset = -M_PI_2;
    
    // 写入数据
    for (size_t frame_id = 0; frame_id < sonar_points_.size(); frame_id++) {
        const auto& point = sonar_points_[frame_id];
        float heading_deg = point.heading_angle * 180.0f / M_PI;
        
        for (size_t i = 0; i < point.ranges.size(); i++) {
            float range = point.ranges[i];
            float intensity = point.intensities[i];
            
            // 获取实际角度或中心角度
            float angle = point.heading_angle;
            if (i < point.angles.size()) {
                angle = point.angles[i];
            }
            
            // 应用角度偏移并计算笛卡尔坐标
            float adjusted_angle = angle + angle_offset;
            float x = range * cosf(adjusted_angle);
            float y = range * sinf(adjusted_angle);
            
            float angle_deg = angle * 180.0f / M_PI;
            
            csv_file << frame_id << ","
                    << std::fixed << std::setprecision(2) << heading_deg << ","
                    << std::fixed << std::setprecision(2) << angle_deg << ","
                    << std::fixed << std::setprecision(3) << range << ","
                    << std::fixed << std::setprecision(1) << intensity << ","
                    << std::fixed << std::setprecision(3) << x << ","
                    << std::fixed << std::setprecision(3) << y << std::endl;
        }
    }
    
    csv_file.close();
    ROS_INFO("Data saved to CSV: %s", filename.c_str());
}

} // namespace sonar_slam

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar_map_builder");
    
    // 使用私有节点句柄来获取参数
    ros::NodeHandle nh("~");
    
    // 添加额外的调试参数
    bool debug_mode, save_csv;
    std::string csv_filename;
    nh.param("debug_mode", debug_mode, false);
    nh.param("save_csv", save_csv, false);
    nh.param("csv_filename", csv_filename, std::string("/tmp/sonar_data.csv"));
    
    sonar_slam::SonarMapBuilder builder(nh);
    
    // 如果启用了调试模式，打印详细数据摘要
    if (debug_mode) {
        builder.printDataSummary();
        
        if (save_csv && builder.isDataLoaded()) {
            builder.saveDataToCSV(csv_filename);
        }
    }
    
    ROS_INFO("Sonar Map Builder node ready");
    ROS_INFO("Press Ctrl+C to exit");
    
    // 保持节点运行以持续发布TF等
    ros::spin();
    
    return 0;
}