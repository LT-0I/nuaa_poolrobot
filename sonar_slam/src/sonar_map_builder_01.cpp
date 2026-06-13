// sonar_slam/src/sonar_map_builder.cpp
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
// 包含头文件
#include "sonar_slam/sonar_map_builder.h"

// 现在整个实现应该在命名空间内
namespace sonar_slam {  // 添加命名空间开始



 

SonarMapBuilder::SonarMapBuilder(ros::NodeHandle& nh) : 
    nh_(nh),
    data_loaded_(false),
    pointcloud_(new PointCloudI())
{
    // 读取参数
    // nh_.param("data_file", data_file_, std::string(""));
    // nh_.param("max_range", max_range_, 15.0f);
    // nh_.param("min_range", min_range_, 0.2f);
    // nh_.param("min_intensity", min_intensity_, 100.0f);
    // nh_.param("map_resolution", map_resolution_, 0.05f);
    // nh_.param("map_width", map_width_, 100);
    // nh_.param("map_height", map_height_, 100);
    // nh_.param("map_origin_x", map_origin_x_, 0.0f);   
    // nh_.param("map_origin_y", map_origin_y_, 0.0f);

     // 读取参数
    nh_.param("data_file", data_file_, std::string(""));
    nh_.param("max_range", max_range_, 4.0f);     
    nh_.param("min_range", min_range_, 0.5f);
    nh_.param("min_intensity", min_intensity_, 80.0f);
    nh_.param("map_resolution", map_resolution_, 0.05f);

    // 添加距离缩放参数
    nh_.param("distance_scale", distance_scale_, 0.4f);
    
    // 关键修改：根据最大范围自动计算地图尺寸
    // nh_.param("map_width", map_width_, 400);       // 增加地图尺寸
    // nh_.param("map_height", map_height_, 400);

    nh_.param("map_width", map_width_, 140);       // 140*0.05=7米
    nh_.param("map_height", map_height_, 120);     // 120*0.05=6米
    
    // 关键修改：设置负的原点，使地图中心在(0,0)
    // nh_.param("map_origin_x", map_origin_x_, -10.0f);   // 负值，使地图居中
    // nh_.param("map_origin_y", map_origin_y_, -10.0f);
    nh_.param("map_origin_x", map_origin_x_, -3.5f);
    nh_.param("map_origin_y", map_origin_y_, -3.0f);
    
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
 

// 添加验证函数
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
    
    ROS_INFO("Starting to parse binary file with CORRECT STRUCTURE...");
    std::map<int, int> raw_angle_counts;  // 原始角度值统计
    // 参考第二份代码的处理逻辑
    while (file.tellg() < file_size && total_frames < 5000) {
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
            ROS_WARN("Failed to find 'B' marker");
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
        
        // 检查命令ID
        if (msg_header.cmd != 2300) {
            // 跳过非2300消息
            file.seekg(msg_header.len + 2, std::ios::cur); // 数据 + 校验和
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
        
        // ========== 关键修复：正确的角度转换 ==========
        // 根据第二份代码：angle * 0.9 得到角度（度）
        float angle_deg = static_cast<float>(cmd->angle) * 0.9f;
        
        // 规范化到0-360度
        while (angle_deg >= 360.0f) angle_deg -= 360.0f;
        while (angle_deg < 0.0f) angle_deg += 360.0f;
        
        point.heading_angle = angle_deg * M_PI / 180.0f;
        
        // ========== 关键修复：正确的距离计算 ==========
        // 根据第二份代码：
        // timeus = cmd->samplepriead * 25 / 1000;  // 微秒
        // distance = timeus * 1.45; // mm
        // dist = (distance * (j + 1)) * 0.001; // 米
        
        float sample_time_us = static_cast<float>(cmd->sample_period) * 25.0f / 1000.0f; // 微秒
        float distance_per_sample_mm = sample_time_us * 1.45f; // 毫米
        float distance_per_sample_m = distance_per_sample_mm * 0.001f; // 米
        
        // ========== 解析强度数据 ==========
        size_t data_start = sizeof(MsgCmd2300);
        size_t max_samples = std::min(static_cast<size_t>(cmd->length), 
                                     msgdata.size() - data_start);
        
        int valid_samples = 0;
        
        // 生成扇形扫描数据
        const float BEAM_WIDTH_DEG = 15.0f; // 波束宽度15度
        const float BEAM_WIDTH_RAD = BEAM_WIDTH_DEG * M_PI / 180.0f;
        
        // 处理每个采样点
        for (size_t i = 0; i < max_samples; i++) {
            uint8_t intensity_raw = msgdata[data_start + i];
            float intensity = static_cast<float>(intensity_raw);
            
            // 计算距离（米）
            // float distance = distance_per_sample_m * (i + 1);

            float SCALE_FACTOR = 0.4f; // 尝试0.4、0.3、0.5等值
            float distance = distance_per_sample_m * (i + 1) * SCALE_FACTOR;
            
            // 应用过滤条件
            if (distance < min_range_ || distance > max_range_ || 
                intensity < min_intensity_) {
                continue;
            }
            
            // 关键修复：确保angles数组与ranges数组一一对应
            // 方法1：使用heading_angle作为所有点的角度
            point.angles.push_back(point.heading_angle);
            point.ranges.push_back(distance);
            point.intensities.push_back(intensity);
            valid_samples++;
        }
        
        if (valid_samples > 0) {
            point.timestamp = ros::Time::now();
            point.valid = true;
            sonar_points_.push_back(point);
            
            // if (sonar_points_.size() % 10 == 0 || msg2300_count <= 20) {
            //     ROS_INFO("Frame %d: msg2300#%d, angle=%.1f°, samples=%d, range=%.2f-%.2fm", 
            //             total_frames, msg2300_count, angle_deg, valid_samples,
            //             point.ranges.front(), point.ranges.back());
            // }

            // 调整日志频率
            if (sonar_points_.size() % 50 == 0) {  // 改为每50帧输出一次
                ROS_INFO("Frame %d: msg2300#%d, angle=%.1f°, samples=%d, range=%.2f-%.2fm", 
                        total_frames, msg2300_count, angle_deg, valid_samples,
                        point.ranges.front(), point.ranges.back());
            }
        }
        
        // 进度显示
        if (total_frames % 100 == 0) {
            float progress = static_cast<float>(file.tellg()) / file_size * 100.0f;
            ROS_INFO("Progress: %.1f%%, Frames: %d, msg2300: %d, Valid: %lu", 
                    progress, total_frames, msg2300_count, sonar_points_.size());
        }

        if (msg_header.cmd == 2300) {
            MsgCmd2300* cmd = reinterpret_cast<MsgCmd2300*>(msgdata.data());
            
            // 统计原始角度
            raw_angle_counts[cmd->angle]++;
            
            // 打印原始角度分布（前100帧）
            if (msg2300_count <= 100) {
                ROS_INFO("原始角度: %u -> %.1f°", cmd->angle, 
                        static_cast<float>(cmd->angle) * 0.9f);
            }
        }
        
        // 收集足够数据后停止
        // if (sonar_points_.size() >= 180) {
        //     ROS_INFO("Collected 180 sonar frames, stopping");
        //     break;
        // }

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
    
    // 统计信息
    ROS_INFO("========================================");
    ROS_INFO("PARSING COMPLETED!");
    ROS_INFO("========================================");
    ROS_INFO("Total frames: %d", total_frames);
    ROS_INFO("msg2300 frames: %d", msg2300_count);
    ROS_INFO("Valid sonar frames: %lu", sonar_points_.size());
    
    if (!sonar_points_.empty()) {
        std::vector<float> angles;
        for (const auto& point : sonar_points_) {
            angles.push_back(point.heading_angle * 180.0f / M_PI);
        }
        
        std::sort(angles.begin(), angles.end());
        ROS_INFO("Angle range: %.1f° to %.1f°", angles.front(), angles.back());
        
        // 角度分布
        std::map<int, int> angle_dist;
        for (float angle : angles) {
            int bin = static_cast<int>(angle / 10.0f) * 10;
            angle_dist[bin]++;
        }
        
        ROS_INFO("Angle distribution:");
        for (int deg = 0; deg < 360; deg += 10) {
            if (angle_dist[deg] > 0) {
                ROS_INFO("  %3d°-%3d°: %3d frames", deg, deg+9, angle_dist[deg]);
            }
        }
    }

    if (!raw_angle_counts.empty()) {
        ROS_INFO("原始角度分布统计:");
        ROS_INFO("  总不同角度: %lu", raw_angle_counts.size());
        
        // 找出最小和最大原始角度
        auto min_angle = std::min_element(raw_angle_counts.begin(), raw_angle_counts.end(),
                                         [](const auto& a, const auto& b) { return a.first < b.first; });
        auto max_angle = std::max_element(raw_angle_counts.begin(), raw_angle_counts.end(),
                                         [](const auto& a, const auto& b) { return a.first < b.first; });
        
        ROS_INFO("  原始角度范围: %d 到 %d", min_angle->first, max_angle->first);
        ROS_INFO("  对应角度范围: %.1f° 到 %.1f°", 
                min_angle->first * 0.9f, max_angle->first * 0.9f);
        
        // 检查是否有完整扫描
        if (max_angle->first - min_angle->first < 399) {
            ROS_WARN("警告：原始角度范围不足！可能不是完整360度扫描。");
            ROS_WARN("  期望: 0-399 (360度)");
            ROS_WARN("  实际: %d-%d (%.1f度)", 
                    min_angle->first, max_angle->first,
                    (max_angle->first - min_angle->first) * 0.9f);
        }
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
 

// 添加新函数：填补缺失角度
void SonarMapBuilder::enhanceMissingAngles()
{
    if (pointcloud_->empty()) return;
    
    PointCloudI::Ptr enhanced(new PointCloudI());
    enhanced->header = pointcloud_->header;
    
    // 复制原始点
    for (const auto& point : pointcloud_->points) {
        enhanced->points.push_back(point);
    }
    
    // 统计角度分布
    std::map<int, int> angle_bins; // 每10度一个桶
    for (const auto& point : pointcloud_->points) {
        float angle = atan2(point.y, point.x) * 180.0 / M_PI;
        if (angle < 0) angle += 360.0f;
        int bin = static_cast<int>(angle / 10.0f) * 10;
        angle_bins[bin]++;
    }
    
    // 找出缺失的角度区域
    std::vector<int> missing_angles;
    for (int deg = 0; deg < 360; deg += 10) {
        if (angle_bins[deg] < 5) { // 少于5个点
            missing_angles.push_back(deg);
        }
    }
    
    ROS_INFO("Missing angle regions: %lu", missing_angles.size());
    
    // 对每个缺失角度，从相邻角度插值
    for (int missing_deg : missing_angles) {
        // 找前后最近的有数据的角度
        int prev_deg = missing_deg - 10;
        int next_deg = missing_deg + 10;
        
        while (prev_deg >= 0 && angle_bins[prev_deg] < 5) prev_deg -= 10;
        while (next_deg < 360 && angle_bins[next_deg] < 5) next_deg += 10;
        
        if (prev_deg >= 0 && next_deg < 360) {
            // 插值生成点
            float prev_rad = prev_deg * M_PI / 180.0f;
            float next_rad = next_deg * M_PI / 180.0f;
            float missing_rad = missing_deg * M_PI / 180.0f;
            
            // 收集前后角度的点
            std::vector<pcl::PointXYZI> prev_points, next_points;
            for (const auto& point : pointcloud_->points) {
                float angle = atan2(point.y, point.x);
                if (angle < 0) angle += 2*M_PI;
                float angle_deg = angle * 180.0 / M_PI;
                
                if (fabs(angle_deg - prev_deg) < 5.0) {
                    prev_points.push_back(point);
                } else if (fabs(angle_deg - next_deg) < 5.0) {
                    next_points.push_back(point);
                }
            }
            
            // 简单平均插值
            if (!prev_points.empty() && !next_points.empty()) {
                // 取每个距离上的平均点
                for (size_t i = 0; i < std::min(prev_points.size(), next_points.size()); i++) {
                    float avg_range = (sqrt(prev_points[i].x*prev_points[i].x + prev_points[i].y*prev_points[i].y) +
                                      sqrt(next_points[i].x*next_points[i].x + next_points[i].y*next_points[i].y)) / 2.0f;
                    float avg_intensity = (prev_points[i].intensity + next_points[i].intensity) / 2.0f;
                    
                    pcl::PointXYZI new_point;
                    new_point.x = avg_range * cosf(missing_rad);
                    new_point.y = avg_range * sinf(missing_rad);
                    new_point.z = 0.0f;
                    new_point.intensity = avg_intensity * 0.7f; // 降低强度表示插值点
                    
                    enhanced->points.push_back(new_point);
                }
            }
        }
    }
    
    enhanced->width = enhanced->points.size();
    enhanced->height = 1;
    enhanced->is_dense = false;
    
    pointcloud_ = enhanced;
    ROS_INFO("Enhanced point cloud: %lu points (added %lu points)", 
             pointcloud_->size(), pointcloud_->size() - enhanced->width);
}

/****
void SonarMapBuilder::convertToPointCloud()
{
    pointcloud_->clear();
    pointcloud_->header.frame_id = "sonar_map_frame";
    pointcloud_->width = 0;
    pointcloud_->height = 1;
    pointcloud_->is_dense = false;
    
    size_t total_points = 0;
    size_t error_count = 0;
    
    // 移除中心化计算，直接转换
    for (const auto& sonar_point : sonar_points_) {
        if (!sonar_point.valid || sonar_point.empty()) continue;
        
        // 安全检查：确保数组大小匹配
        size_t point_count = sonar_point.ranges.size();
        if (sonar_point.angles.size() != point_count) {
            ROS_WARN("Array size mismatch: angles=%lu, ranges=%lu", 
                    sonar_point.angles.size(), point_count);
            error_count++;
            continue;
        }
        
        for (size_t i = 0; i < point_count; i++) {
            // 安全检查：确保索引有效
            if (i >= sonar_point.ranges.size() || i >= sonar_point.angles.size() || 
                i >= sonar_point.intensities.size()) {
                ROS_WARN("Array index out of bounds: i=%lu, sizes: ranges=%lu, angles=%lu, intensities=%lu",
                        i, sonar_point.ranges.size(), sonar_point.angles.size(),
                        sonar_point.intensities.size());
                error_count++;
                break;
            }
            
            float range = sonar_point.ranges[i];
            float angle = sonar_point.angles[i];
            float intensity = sonar_point.intensities[i];
            
            // 移除角度偏移测试，直接使用原始角度
            // const float ANGLE_OFFSET = -M_PI_2; // -90度
            // float corrected_angle = angle + ANGLE_OFFSET;
            
            pcl::PointXYZI point;
            point.x = range * cosf(angle);  // 使用原始角度
            point.y = range * sinf(angle);
            point.z = 0.0f;
            point.intensity = intensity;
            
            pointcloud_->points.push_back(point);
            total_points++;
        }
    }
    
    pointcloud_->width = pointcloud_->points.size();
    ROS_INFO("Converted %lu points to point cloud (errors: %lu)", total_points, error_count);
    
    // 调试信息
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
        
        ROS_INFO("Point cloud bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]", 
                min_x, max_x, min_y, max_y);
    }
}
****/

void SonarMapBuilder::convertToPointCloud() {
    pointcloud_->clear();
    pointcloud_->header.frame_id = "sonar_map_frame";
    pointcloud_->width = 0;
    pointcloud_->height = 1;
    pointcloud_->is_dense = false;
    
    // 新增：详细角度分布诊断
    ROS_INFO("=== 开始角度分布诊断 ===");
    
    // 统计所有角度
    std::vector<float> all_angles;
    for (const auto& sonar_point : sonar_points_) {
        if (!sonar_point.valid || sonar_point.empty()) continue;
        
        float angle_deg = sonar_point.heading_angle * 180.0 / M_PI;
        all_angles.push_back(angle_deg);
        
        // 打印前几个点的详细信息
        if (all_angles.size() <= 5) {
            ROS_INFO("点 %lu: 角度=%.1f°, 距离数=%lu", 
                    all_angles.size(), angle_deg, sonar_point.ranges.size());
            if (!sonar_point.ranges.empty()) {
                ROS_INFO("  最近距离=%.2fm, 最远距离=%.2fm", 
                        sonar_point.ranges.front(), sonar_point.ranges.back());
            }
        }
    }
    
    if (!all_angles.empty()) {
        // 角度范围
        auto min_max = std::minmax_element(all_angles.begin(), all_angles.end());
        ROS_INFO("角度范围: %.1f° 到 %.1f°", *min_max.first, *min_max.second);
        
        // 角度直方图（每45度）
        int bins[8] = {0}; // 0-45, 45-90, ..., 315-360
        for (float angle : all_angles) {
            while (angle < 0) angle += 360.0f;
            while (angle >= 360.0f) angle -= 360.0f;
            
            int bin = static_cast<int>(angle / 45.0f);
            if (bin >= 0 && bin < 8) bins[bin]++;
        }
        
        ROS_INFO("角度分布（每45度）:");
        for (int i = 0; i < 8; i++) {
            ROS_INFO("  %3d°-%3d°: %d 个点", i*45, (i+1)*45, bins[i]);
        }
        
        // 找出主要角度
        std::map<int, int> angle_counts;
        for (float angle : all_angles) {
            int int_angle = static_cast<int>(angle + 0.5f);
            angle_counts[int_angle]++;
        }
        
        ROS_INFO("主要角度（前10个）:");
        std::vector<std::pair<int, int>> sorted_angles(angle_counts.begin(), angle_counts.end());
        std::sort(sorted_angles.begin(), sorted_angles.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });
        
        for (size_t i = 0; i < std::min(size_t(10), sorted_angles.size()); i++) {
            ROS_INFO("  %3d°: %d 个点", sorted_angles[i].first, sorted_angles[i].second);
        }
    }
    ROS_INFO("=== 角度分布诊断结束 ===");
    
    // 原来的转换代码...
    size_t total_points = 0;
    
    // 测试不同角度校正值
    std::vector<float> test_corrections = {0.0f, -M_PI_2, M_PI_2, M_PI, -M_PI};
    
    for (float angle_correction : test_corrections) {
        ROS_INFO("测试角度校正: %.1f°", angle_correction * 180.0 / M_PI);
        
        PointCloudI::Ptr test_cloud(new PointCloudI());
        test_cloud->header.frame_id = "sonar_map_frame";
        
        for (const auto& sonar_point : sonar_points_) {
            if (!sonar_point.valid || sonar_point.empty()) continue;
            
            for (size_t i = 0; i < sonar_point.ranges.size(); i++) {
                float range = sonar_point.ranges[i];
                float angle = sonar_point.angles[i];
                float intensity = sonar_point.intensities[i];
                
                float corrected_angle = angle + angle_correction;
                
                pcl::PointXYZI point;
                point.x = range * cosf(corrected_angle);
                point.y = range * sinf(corrected_angle);
                point.z = 0.0f;
                point.intensity = intensity;
                
                test_cloud->points.push_back(point);
            }
        }
        
        test_cloud->width = test_cloud->points.size();
        test_cloud->height = 1;
        
        if (!test_cloud->empty()) {
            // 计算边界
            float min_x = std::numeric_limits<float>::max();
            float max_x = std::numeric_limits<float>::lowest();
            float min_y = std::numeric_limits<float>::max();
            float max_y = std::numeric_limits<float>::lowest();
            
            for (const auto& point : test_cloud->points) {
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }
            
            ROS_INFO("  校正 %.1f°: %lu 点, 边界 x=[%.2f, %.2f], y=[%.2f, %.2f]", 
                    angle_correction * 180.0 / M_PI,
                    test_cloud->size(),
                    min_x, max_x, min_y, max_y);
            
            // 选择最合适的校正（边界最大）
            float width = max_x - min_x;
            float height = max_y - min_y;
            float area = width * height;
            
            if (area > total_points) {  // 使用面积作为选择标准
                total_points = area;
                pointcloud_ = test_cloud;
                ROS_INFO("  选择此校正值");
            }
        }
    }
    
    pointcloud_->width = pointcloud_->points.size();
    ROS_INFO("最终点云: %lu 点", pointcloud_->size());
    
    if (pointcloud_->empty()) {
        ROS_ERROR("没有点被转换！");
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

/****
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
    
    // 2. 如果使用自动计算的地图尺寸
    if (map_width_ == 400 && map_height_ == 400) { // 如果是默认值
        // 根据点云范围动态调整地图
        float cloud_width = max_x - min_x;
        float cloud_height = max_y - min_y;
        
        // 留出20%的边界
        map_width_ = static_cast<int>(cloud_width / map_resolution_ * 1.2f) + 1;
        map_height_ = static_cast<int>(cloud_height / map_resolution_ * 1.2f) + 1;
        
        // 设置原点点云居中
        map_origin_x_ = min_x - (cloud_width * 0.1f);
        map_origin_y_ = min_y - (cloud_height * 0.1f);
        
        ROS_INFO("Auto-adjusted map: size=%dx%d, origin=(%.2f, %.2f)", 
                map_width_, map_height_, map_origin_x_, map_origin_y_);
    }
    
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
    
    // 初始化地图数据（-1表示未知）
    occupancy_grid_.data.resize(map_width_ * map_height_, -1);
    
    int occupied_cells = 0;
    
    // 4. 将点云投影到栅格地图（改进算法）
    for (const auto& point : pointcloud_->points) {
        // 转换为地图坐标（不加0.5，直接截断）
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);
        
        // 检查边界
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            int index = map_y * map_width_ + map_x;
            
            // 根据强度设置占据概率（0-100）
            int occupancy = static_cast<int>((point.intensity / 255.0f) * 100.0f);
            occupancy = std::min(100, std::max(0, occupancy));
            
            // 如果单元格已有值，取最大值
            if (occupancy_grid_.data[index] < occupancy) {
                occupancy_grid_.data[index] = occupancy;
            }
            occupied_cells++;
            
            // 可选：填充周围单元格，使墙壁更连续
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int nx = map_x + dx;
                    int ny = map_y + dy;
                    
                    if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                        int nidx = ny * map_width_ + nx;
                        // 周围单元格概率减半
                        int neighbor_occ = occupancy / 2;
                        if (occupancy_grid_.data[nidx] < neighbor_occ) {
                            occupancy_grid_.data[nidx] = neighbor_occ;
                        }
                    }
                }
            }
        }
    }
    
    ROS_INFO("Occupancy grid: %d cells occupied (%.1f%%)", 
            occupied_cells, occupied_cells * 100.0 / (map_width_ * map_height_));
}
****/

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
    // 根据点云范围计算地图大小（留10%边界）
    float cloud_width = max_x - min_x;
    float cloud_height = max_y - min_y;
    
    // 限制最大地图尺寸（例如10m×10m）
    const float MAX_MAP_SIZE = 10.0f; // 米
    
    if (cloud_width > MAX_MAP_SIZE) {
        cloud_width = MAX_MAP_SIZE;
        min_x = -MAX_MAP_SIZE/2;
        max_x = MAX_MAP_SIZE/2;
    }
    
    if (cloud_height > MAX_MAP_SIZE) {
        cloud_height = MAX_MAP_SIZE;
        min_y = -MAX_MAP_SIZE/2;
        max_y = MAX_MAP_SIZE/2;
    }
    
    // 计算地图尺寸（单元格）
    map_width_ = static_cast<int>(cloud_width / map_resolution_) + 20; // 加10个单元格边界
    map_height_ = static_cast<int>(cloud_height / map_resolution_) + 20;
    
    // 设置地图原点使点云居中
    map_origin_x_ = min_x - 10 * map_resolution_; // 留10个单元格边界
    map_origin_y_ = min_y - 10 * map_resolution_;
    
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
    
    // 初始化地图数据（-1表示未知）
    occupancy_grid_.data.resize(map_width_ * map_height_, -1);
    
    int occupied_cells = 0;
    
    // 4. 优化点云到地图的投影
    for (const auto& point : pointcloud_->points) {
        // 转换为地图坐标
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_);
        
        // 放宽边界检查，确保边缘点也被包含
        if (map_x >= -1 && map_x <= map_width_ && map_y >= -1 && map_y <= map_height_) {
            // 限制在有效范围内
            map_x = std::max(0, std::min(map_width_ - 1, map_x));
            map_y = std::max(0, std::min(map_height_ - 1, map_y));
            
            int index = map_y * map_width_ + map_x;
            
            // 根据强度设置占据概率
            // 降低阈值，确保弱信号也被显示
            float intensity_ratio = point.intensity / 255.0f;
            int occupancy = static_cast<int>(intensity_ratio * 100.0f);
            
            // 降低阈值到20
            if (occupancy > 20) {
                if (occupancy_grid_.data[index] < occupancy) {
                    occupancy_grid_.data[index] = occupancy;
                }
                
                // 对于强信号点，填充周围区域
                if (occupancy > 50) {
                    for (int dx = -3; dx <= 3; dx++) {
                        for (int dy = -3; dy <= 3; dy++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = map_x + dx;
                            int ny = map_y + dy;
                            
                            if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                                int nidx = ny * map_width_ + nx;
                                float distance = sqrt(dx*dx + dy*dy);
                                int neighbor_occ = static_cast<int>(occupancy * exp(-distance*0.3f));
                                if (occupancy_grid_.data[nidx] < neighbor_occ) {
                                    occupancy_grid_.data[nidx] = neighbor_occ;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 5. 填充空白区域（如果周围有足够的点）
    std::vector<int8_t> temp_data = occupancy_grid_.data;
    for (int y = 1; y < map_height_-1; y++) {
        for (int x = 1; x < map_width_-1; x++) {
            int idx = y * map_width_ + x;
            
            // 如果是未知区域
            if (occupancy_grid_.data[idx] == -1) {
                // 检查周围8个单元格
                int occupied_neighbors = 0;
                int total_occupancy = 0;
                
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        int nidx = (y + dy) * map_width_ + (x + dx);
                        if (occupancy_grid_.data[nidx] > 0) {
                            occupied_neighbors++;
                            total_occupancy += occupancy_grid_.data[nidx];
                        }
                    }
                }
                
                // 如果周围有足够多的占据点，填充这个单元格
                if (occupied_neighbors >= 4) {
                    temp_data[idx] = total_occupancy / occupied_neighbors / 2;
                }
            }
        }
    }
    
    occupancy_grid_.data = temp_data;
    
    ROS_INFO("Occupancy grid: %d cells occupied (%.1f%%)", 
            occupied_cells, occupied_cells * 100.0 / (map_width_ * map_height_));
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


void SonarMapBuilder::publishTF()
{
    ros::Time current_time = ros::Time::now();
    
    // 发布完整的TF链
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    
    // 1. map -> odom (恒等变换)
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, current_time, "map", "odom"));
    
    // 2. odom -> base_link (根据你的数据质心调整位置)
    // 从你的日志看，数据质心在(1.65, -1.48)
    // 为了在rviz中居中显示，我们可能需要调整这个偏移
    float x_offset = 0.0;  // 先试0.0，不行再调整
    float y_offset = 0.0;
    transform.setOrigin(tf::Vector3(x_offset, y_offset, 0.0));
    transform.setRotation(q);
    tf_broadcaster_->sendTransform(
        tf::StampedTransform(transform, current_time, "odom", "base_link"));
    
    // 3. base_link -> sonar_map_frame (保持原来的)
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
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