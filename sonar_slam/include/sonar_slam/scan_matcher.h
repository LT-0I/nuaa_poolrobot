#ifndef SONAR_SLAM_SCAN_MATCHER_H
#define SONAR_SLAM_SCAN_MATCHER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Core>
#include <vector>
#include <mutex>

namespace sonar_slam {

/**
 * @brief 扫描匹配器 - 用于SLAM中的定位校正
 * 
 * 支持多种匹配算法：
 * 1. 相关性扫描匹配（Correlation Scan Matching）
 * 2. ICP（Iterative Closest Point）
 * 3. NDT（Normal Distributions Transform）
 */
class ScanMatcher {
public:
    enum Algorithm {
        ALGO_CORRELATION,  // 相关性匹配（快速，适合低分辨率）
        ALGO_ICP,          // ICP（精确，计算量大）
        ALGO_NDT           // NDT（平衡精度和速度）
    };
    
    struct MatchResult {
        tf::Transform transform;      // 最优变换
        double score;                 // 匹配得分（0-1，越高越好）
        bool valid;                   // 匹配是否有效
        int iterations;               // 迭代次数
        double match_time;            // 匹配耗时（秒）
        
        MatchResult() : score(0.0), valid(false), iterations(0), match_time(0.0) {}
    };
    
    struct MatchParams {
        Algorithm algorithm;          // 匹配算法
        double max_correspondence_distance;  // 最大对应点距离（m）
        int max_iterations;           // 最大迭代次数
        double transformation_epsilon; // 变换收敛阈值
        double fitness_epsilon;       // 拟合收敛阈值
        bool use_reciprocal_correspondences; // 使用双向对应
        
        // 相关性匹配参数
        double correlation_search_range_x;   // X方向搜索范围（m）
        double correlation_search_range_y;   // Y方向搜索范围（m）
        double correlation_search_range_yaw; // Yaw搜索范围（rad）
        double correlation_resolution_x;     // X方向分辨率（m）
        double correlation_resolution_y;     // Y方向分辨率（m）
        double correlation_resolution_yaw;   // Yaw分辨率（rad）
        
        MatchParams() {
            algorithm = ALGO_CORRELATION;
            max_correspondence_distance = 0.5;
            max_iterations = 50;
            transformation_epsilon = 1e-8;
            fitness_epsilon = 1e-6;
            use_reciprocal_correspondences = false;
            
            correlation_search_range_x = 0.5;
            correlation_search_range_y = 0.5;
            correlation_search_range_yaw = 0.2;
            correlation_resolution_x = 0.05;
            correlation_resolution_y = 0.05;
            correlation_resolution_yaw = 0.02;
        }
    };
    
    ScanMatcher();
    ~ScanMatcher();
    
    /**
     * @brief 设置匹配参数
     */
    void setParams(const MatchParams& params);
    
    /**
     * @brief 扫描匹配主函数
     * @param current_scan 当前扫描点云
     * @param reference_points 参考点云或地图
     * @param initial_guess 初始位姿猜测
     * @return 匹配结果
     */
    MatchResult matchScan(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_points,
        const tf::Transform& initial_guess);
    
    /**
     * @brief 使用占据栅格地图进行匹配（快速）
     */
    MatchResult matchScanToGrid(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        const nav_msgs::OccupancyGrid& grid_map,
        const tf::Transform& initial_guess);
    
    /**
     * @brief 更新参考地图（用于连续匹配）
     */
    void updateReferenceMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud);
    
    /**
     * @brief 重置匹配器
     */
    void reset();
    
    /**
     * @brief 获取匹配统计数据
     */
    void getStatistics(int& total_matches, int& successful_matches, 
                      double& avg_time, double& success_rate);


    
    Eigen::Matrix4f tfToEigenMatrix(const tf::Transform& transform);
    tf::Transform eigenMatrixToTf(const Eigen::Matrix4f& matrix);
        
    
private:
    // 三种匹配算法的实现
    MatchResult matchByCorrelation(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        const nav_msgs::OccupancyGrid& grid_map,
        const tf::Transform& initial_guess);
    
    MatchResult matchByICP(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_cloud,
        const tf::Transform& initial_guess);
    
    MatchResult matchByNDT(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_cloud,
        const tf::Transform& initial_guess);
    
    /**
     * @brief 将点云转换为占据栅格用于快速匹配
     */
    void pointCloudToGrid(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        std::vector<std::vector<bool>>& grid,
        float& origin_x, float& origin_y,
        float resolution, int& width, int& height);
    
    /**
     * @brief 计算两个点云的相关性得分
     */
    double computeCorrelationScore(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan1,
        const std::vector<std::vector<bool>>& grid2,
        float grid_origin_x, float grid_origin_y,
        float resolution, const tf::Transform& transform);
    
    /**
     * @brief 从占据栅格地图提取点云
     */
    void extractPointsFromGrid(
        const nav_msgs::OccupancyGrid& grid_map,
        pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud,
        int occupied_threshold = 50);
    

    // 成员变量
    MatchParams params_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr reference_map_;
    
    // ICP和NDT对象
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_;
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;


    
    
    // 统计
    int total_matches_;
    int successful_matches_;
    double total_match_time_;
    
    // 线程安全
    std::mutex mutex_;
    
    // 调试
    bool enable_debug_;
    ros::Publisher debug_cloud_pub_;
    ros::Publisher aligned_cloud_pub_;
};

} // namespace sonar_slam

#endif // SONAR_SLAM_SCAN_MATCHER_H