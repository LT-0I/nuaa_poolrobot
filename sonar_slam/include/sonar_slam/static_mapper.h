#ifndef STATIC_MAPPER_H
#define STATIC_MAPPER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>  // 添加缺少的头文件
#include <vector>
#include <string>
#include <mutex>

namespace sonar_slam {

class StaticMapper
{
public:
    StaticMapper();
    
    // 初始化地图
    void initializeMap();
    
    // 更新地图
    void updateMap(const sensor_msgs::LaserScan& scan,
                   const geometry_msgs::TransformStamped& transform);
    
    // 发布地图
    void publishMap();
    
    // 获取当前地图
    nav_msgs::OccupancyGrid getMap() const;
    
    // 保存地图到文件
    bool saveMap(const std::string& filename);
    
    // 加载地图从文件
    bool loadMap(const std::string& filename);
    
    // 重置地图
    void resetMap();
    
private:
    // 回调函数
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    // 地图更新方法
    void updateLine(int x0, int y0, int x1, int y1, float range);
    void updateCell(int cell_idx, bool occupied);
    
    // 坐标转换
    int worldToMapX(float wx) const;
    int worldToMapY(float wy) const;
    bool isInMap(int mx, int my) const;
    
    // 概率转换
    float logOddsFromProb(float prob) const;
    float probabilityFromLogOdds(float log_odds) const;
    
    // 滤波方法
    void applySmoothingFilter();
    void removeIsolatedPoints();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher map_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::OccupancyGrid map_;
    std::vector<float> log_odds_map_;
    
    // 地图参数
    float map_resolution_;
    int map_width_;
    int map_height_;
    float origin_x_;
    float origin_y_;
    
    // 概率参数
    float prob_hit_;
    float prob_miss_;
    float min_prob_;
    float max_prob_;
    
    // 传感器参数
    float max_range_;
    float min_range_;
    
    // 状态
    bool map_initialized_;
    mutable std::mutex map_mutex_;  // 添加mutable关键字
    
    // 滤波参数
    bool enable_smoothing_;
    int smoothing_kernel_size_;
    int min_neighbors_;
};

} // namespace sonar_slam

#endif // STATIC_MAPPER_H