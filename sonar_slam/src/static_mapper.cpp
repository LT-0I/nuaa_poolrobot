#include "sonar_slam/static_mapper.h"
#include <iostream>
#include <cmath>
#include <mutex>
#include <algorithm>

namespace sonar_slam {

StaticMapper::StaticMapper() : 
    nh_("~"),
    tf_listener_(tf_buffer_),
    map_initialized_(false)
{
    // 参数 - 修复所有float/double类型不匹配问题
    nh_.param("map_resolution", map_resolution_, 0.05f);     
    nh_.param("origin_x", origin_x_, -5.0f);                 
    nh_.param("origin_y", origin_y_, -5.0f);                 
    nh_.param("prob_hit", prob_hit_, 0.7f);                 
    nh_.param("prob_miss", prob_miss_, 0.4f);              
    nh_.param("min_prob", min_prob_, 0.2f);                  
    nh_.param("max_prob", max_prob_, 0.8f);                 
    nh_.param("max_range", max_range_, 15.0f);             
    nh_.param("min_range", min_range_, 0.2f);                
    nh_.param("enable_smoothing", enable_smoothing_, true);
    nh_.param("smoothing_kernel_size", smoothing_kernel_size_, 3);
    nh_.param("min_neighbors", min_neighbors_, 2);
    
    // 计算地图尺寸
    map_width_ = static_cast<int>(2.0f * fabs(origin_x_) / map_resolution_);
    map_height_ = static_cast<int>(2.0f * fabs(origin_y_) / map_resolution_);
    
    // 订阅扫描
    scan_sub_ = nh_.subscribe("/sonar_scan", 10, &StaticMapper::scanCallback, this);
    
    // 发布地图
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/sonar_map", 1, true);
    
    // 初始化地图
    initializeMap();
    
    ROS_INFO("Static Mapper initialized");
    ROS_INFO("Map size: %d x %d, resolution: %.3f", map_width_, map_height_, map_resolution_);
}

void StaticMapper::initializeMap()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    map_.header.frame_id = "map";
    map_.info.resolution = map_resolution_;
    map_.info.width = map_width_;
    map_.info.height = map_height_;
    map_.info.origin.position.x = origin_x_;
    map_.info.origin.position.y = origin_y_;
    map_.info.origin.position.z = 0.0;
    map_.info.origin.orientation.w = 1.0;
    
    // 初始化log-odds值
    log_odds_map_.resize(map_width_ * map_height_, 0.0f);
    map_.data.resize(map_width_ * map_height_, -1);
    
    map_initialized_ = true;
}

void StaticMapper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if (!map_initialized_) return;
    
    try
    {
        // 获取声纳在map坐标系下的位姿
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer_.lookupTransform("map", scan->header.frame_id,
                                              scan->header.stamp,
                                              ros::Duration(1.0));
        
        // 更新地图
        updateMap(*scan, transform);
        
        // 发布地图
        publishMap();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void StaticMapper::updateMap(const sensor_msgs::LaserScan& scan, 
                   const geometry_msgs::TransformStamped& transform)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    
    float sensor_x = transform.transform.translation.x;
    float sensor_y = transform.transform.translation.y;
    
    // 将传感器位姿转换到地图坐标
    int sensor_mx = worldToMapX(sensor_x);
    int sensor_my = worldToMapY(sensor_y);
    
    // 处理每个有效的扫描点
    for (size_t i = 0; i < scan.ranges.size(); i++)
    {
        float range = scan.ranges[i];
        
        // 跳过无效测量
        if (range < scan.range_min || range > scan.range_max || 
            range > max_range_ || std::isnan(range))
            continue;
        
        // 计算扫描点的角度
        float angle = scan.angle_min + i * scan.angle_increment;
        
        // 计算扫描点的世界坐标
        float wx = sensor_x + range * cos(angle);
        float wy = sensor_y + range * sin(angle);
        
        // 将世界坐标转换为地图坐标
        int mx = worldToMapX(wx);
        int my = worldToMapY(wy);
        
        // 确保坐标在地图范围内
        if (!isInMap(mx, my)) continue;
        
        // 更新占用栅格（使用Bresenham算法画线）
        updateLine(sensor_mx, sensor_my, mx, my, range);
        
        // 终点设为占用
        int cell_idx = my * map_width_ + mx;
        updateCell(cell_idx, true);
    }
    
    // 可选：应用滤波
    if (enable_smoothing_)
    {
        applySmoothingFilter();
    }
}

void StaticMapper::updateLine(int x0, int y0, int x1, int y1, float range)
{
    // Bresenham直线算法
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    while (true)
    {
        // 更新当前单元格为空闲
        int cell_idx = y0 * map_width_ + x0;
        updateCell(cell_idx, false);
        
        // 如果到达终点，退出
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void StaticMapper::updateCell(int cell_idx, bool occupied)
{
    if (cell_idx < 0 || cell_idx >= static_cast<int>(log_odds_map_.size())) return;
    
    float log_odds_update = occupied ? logOddsFromProb(prob_hit_) : 
                                       logOddsFromProb(prob_miss_);
    
    // 更新log-odds值
    log_odds_map_[cell_idx] += log_odds_update;
    
    // 限制log-odds值范围
    log_odds_map_[cell_idx] = std::max(-10.0f, std::min(10.0f, log_odds_map_[cell_idx]));
    
    // 转换为概率值（0-100）
    float prob = probabilityFromLogOdds(log_odds_map_[cell_idx]);
    
    // 将概率限制在min_prob_和max_prob_之间
    prob = std::max(min_prob_, std::min(max_prob_, prob));
    
    // 转换为0-100的整数值
    map_.data[cell_idx] = static_cast<int8_t>((prob - min_prob_) / (max_prob_ - min_prob_) * 100.0f);
    
    // 确保值在有效范围内
    map_.data[cell_idx] = std::max(-1, std::min(100, static_cast<int>(map_.data[cell_idx])));
}

float StaticMapper::logOddsFromProb(float prob) const
{
    if (prob <= 0.0f || prob >= 1.0f) return 0.0f;
    return logf(prob / (1.0f - prob));  // 使用logf而不是log
}

float StaticMapper::probabilityFromLogOdds(float log_odds) const
{
    return 1.0f - 1.0f / (1.0f + expf(log_odds));  // 使用expf而不是exp
}

int StaticMapper::worldToMapX(float wx) const
{
    return static_cast<int>((wx - origin_x_) / map_resolution_ + 0.5f);
}

int StaticMapper::worldToMapY(float wy) const
{
    return static_cast<int>((wy - origin_y_) / map_resolution_ + 0.5f);
}

bool StaticMapper::isInMap(int mx, int my) const
{
    return mx >= 0 && mx < map_width_ && my >= 0 && my < map_height_;
}

void StaticMapper::publishMap()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_.header.stamp = ros::Time::now();
    map_pub_.publish(map_);
}

void StaticMapper::applySmoothingFilter()
{
    std::vector<float> smoothed_log_odds = log_odds_map_;
    
    for (int y = 1; y < map_height_ - 1; y++)
    {
        for (int x = 1; x < map_width_ - 1; x++)
        {
            int idx = y * map_width_ + x;
            float sum = 0.0f;
            int count = 0;
            
            // 3x3邻域
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    int nidx = ny * map_width_ + nx;
                    
                    if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_)
                    {
                        sum += log_odds_map_[nidx];
                        count++;
                    }
                }
            }
            
            smoothed_log_odds[idx] = sum / static_cast<float>(count);
        }
    }
    
    log_odds_map_ = smoothed_log_odds;
    
    // 更新地图数据
    for (size_t i = 0; i < log_odds_map_.size(); i++)
    {
        float prob = probabilityFromLogOdds(log_odds_map_[i]);
        prob = std::max(min_prob_, std::min(max_prob_, prob));
        map_.data[i] = static_cast<int8_t>((prob - min_prob_) / (max_prob_ - min_prob_) * 100.0f);
        
        // 确保值在有效范围内
        map_.data[i] = std::max(-1, std::min(100, static_cast<int>(map_.data[i])));
    }
}

nav_msgs::OccupancyGrid StaticMapper::getMap() const
{
    std::lock_guard<std::mutex> lock(map_mutex_);  // 因为mutex是mutable的，这里可以直接使用
    return map_;
}

bool StaticMapper::saveMap(const std::string& filename)
{
    // TODO: 实现地图保存
    ROS_WARN("saveMap not implemented yet");
    return false;
}

bool StaticMapper::loadMap(const std::string& filename)
{
    // TODO: 实现地图加载
    ROS_WARN("loadMap not implemented yet");
    return false;
}

void StaticMapper::resetMap()
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::fill(log_odds_map_.begin(), log_odds_map_.end(), 0.0f);
    std::fill(map_.data.begin(), map_.data.end(), static_cast<int8_t>(-1));
}

} // namespace sonar_slam

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_mapper");
    sonar_slam::StaticMapper mapper;
    ros::spin();
    return 0;
}