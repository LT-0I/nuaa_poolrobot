#include "sonar_slam/scan_matcher.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <chrono>
#include <cmath>
#include <ros/console.h>

namespace sonar_slam {

using namespace std::chrono;

ScanMatcher::ScanMatcher() 
    : total_matches_(0)
    , successful_matches_(0)
    , total_match_time_(0.0)
    , enable_debug_(false) {
    
    // 初始化ICP参数
    icp_.setMaximumIterations(params_.max_iterations);
    icp_.setMaxCorrespondenceDistance(params_.max_correspondence_distance);
    icp_.setTransformationEpsilon(params_.transformation_epsilon);
    icp_.setEuclideanFitnessEpsilon(params_.fitness_epsilon);
    icp_.setUseReciprocalCorrespondences(params_.use_reciprocal_correspondences);
    
    // 初始化NDT参数
    ndt_.setMaximumIterations(params_.max_iterations);
    ndt_.setTransformationEpsilon(params_.transformation_epsilon);
    ndt_.setStepSize(0.1);
    ndt_.setResolution(1.0);
    
    // 初始化参考地图
    reference_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    ROS_DEBUG("ScanMatcher initialized");
}

ScanMatcher::~ScanMatcher() {
    ROS_DEBUG("ScanMatcher destroyed. Statistics: %d/%d successful matches (%.1f%%)",
             successful_matches_, total_matches_,
             total_matches_ > 0 ? 100.0 * successful_matches_ / total_matches_ : 0.0);
}

void ScanMatcher::setParams(const MatchParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    
    // 更新ICP参数
    icp_.setMaximumIterations(params.max_iterations);
    icp_.setMaxCorrespondenceDistance(params.max_correspondence_distance);
    icp_.setTransformationEpsilon(params.transformation_epsilon);
    icp_.setEuclideanFitnessEpsilon(params.fitness_epsilon);
    
    ROS_INFO("ScanMatcher parameters updated");
}

ScanMatcher::MatchResult ScanMatcher::matchScan(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_points,
    const tf::Transform& initial_guess) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    total_matches_++;
    
    MatchResult result;
    
    if (!current_scan || current_scan->empty()) {
        ROS_WARN("ScanMatcher: current_scan is empty");
        return result;
    }
    
    if (!reference_points || reference_points->empty()) {
        ROS_WARN("ScanMatcher: reference_points is empty");
        return result;
    }
    
    auto start_time = high_resolution_clock::now();
    
    try {
        switch (params_.algorithm) {
            case ALGO_CORRELATION:
                // 相关性匹配需要栅格地图，这里使用ICP作为后备
                result = matchByICP(current_scan, reference_points, initial_guess);
                break;
                
            case ALGO_ICP:
                result = matchByICP(current_scan, reference_points, initial_guess);
                break;
                
            case ALGO_NDT:
                result = matchByNDT(current_scan, reference_points, initial_guess);
                break;
                
            default:
                ROS_ERROR("Unknown matching algorithm: %d", params_.algorithm);
                return result;
        }
        
        auto end_time = high_resolution_clock::now();
        duration<double> elapsed = end_time - start_time;
        result.match_time = elapsed.count();
        
        // 检查匹配质量
        if (result.valid && result.score > 0.3) {  // 得分阈值
            successful_matches_++;
            total_match_time_ += result.match_time;
            
            ROS_DEBUG("Scan match successful: score=%.3f, time=%.3fs, iterations=%d",
                     result.score, result.match_time, result.iterations);
        } else {
            ROS_WARN("Scan match failed or low score: score=%.3f", result.score);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Scan matching failed: %s", e.what());
    }
    
    return result;
}

ScanMatcher::MatchResult ScanMatcher::matchScanToGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    const nav_msgs::OccupancyGrid& grid_map,
    const tf::Transform& initial_guess) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    total_matches_++;
    
    MatchResult result;
    
    if (!current_scan || current_scan->empty()) {
        ROS_WARN("ScanMatcher: current_scan is empty");
        return result;
    }
    
    if (grid_map.data.empty()) {
        ROS_WARN("ScanMatcher: grid_map is empty");
        return result;
    }
    
    auto start_time = high_resolution_clock::now();
    
    try {
        // 使用相关性匹配（最适合栅格地图）
        result = matchByCorrelation(current_scan, grid_map, initial_guess);
        
        auto end_time = high_resolution_clock::now();
        duration<double> elapsed = end_time - start_time;
        result.match_time = elapsed.count();
        
        if (result.valid && result.score > 0.3) {
            successful_matches_++;
            total_match_time_ += result.match_time;
            
            ROS_DEBUG("Grid match successful: score=%.3f, time=%.3fs",
                     result.score, result.match_time);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Grid matching failed: %s", e.what());
    }
    
    return result;
}

ScanMatcher::MatchResult ScanMatcher::matchByCorrelation(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    const nav_msgs::OccupancyGrid& grid_map,
    const tf::Transform& initial_guess) {
    
    MatchResult result;
    
    // 1. 提取初始猜测参数
    double init_x = initial_guess.getOrigin().x();
    double init_y = initial_guess.getOrigin().y();
    double init_yaw = tf::getYaw(initial_guess.getRotation());
    
    // 2. 定义搜索空间
    double best_score = -1.0;
    double best_x = init_x;
    double best_y = init_y;
    double best_yaw = init_yaw;
    
    // 3. 将栅格地图转换为bool矩阵用于快速查询
    std::vector<std::vector<bool>> occupancy_grid;
    float grid_origin_x = grid_map.info.origin.position.x;
    float grid_origin_y = grid_map.info.origin.position.y;
    float resolution = grid_map.info.resolution;
    int width = grid_map.info.width;
    int height = grid_map.info.height;
    
    occupancy_grid.resize(height, std::vector<bool>(width, false));
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            occupancy_grid[y][x] = (grid_map.data[idx] > 50);  // 占据阈值
        }
    }
    
    // 4. 网格搜索
    int search_steps_x = static_cast<int>(params_.correlation_search_range_x / 
                                         params_.correlation_resolution_x);
    int search_steps_y = static_cast<int>(params_.correlation_search_range_y / 
                                         params_.correlation_resolution_y);
    int search_steps_yaw = static_cast<int>(params_.correlation_search_range_yaw / 
                                           params_.correlation_resolution_yaw);
    
    for (int dx = -search_steps_x; dx <= search_steps_x; dx++) {
        for (int dy = -search_steps_y; dy <= search_steps_yaw; dy++) {
            for (int dtheta = -search_steps_yaw; dtheta <= search_steps_yaw; dtheta++) {
                double test_x = init_x + dx * params_.correlation_resolution_x;
                double test_y = init_y + dy * params_.correlation_resolution_y;
                double test_yaw = init_yaw + dtheta * params_.correlation_resolution_yaw;
                
                // 创建变换
                tf::Transform test_transform;
                test_transform.setOrigin(tf::Vector3(test_x, test_y, 0));
                test_transform.setRotation(tf::createQuaternionFromYaw(test_yaw));
                
                // 计算相关性得分
                double score = computeCorrelationScore(
                    current_scan, occupancy_grid,
                    grid_origin_x, grid_origin_y, resolution,
                    test_transform);
                
                if (score > best_score) {
                    best_score = score;
                    best_x = test_x;
                    best_y = test_y;
                    best_yaw = test_yaw;
                }
            }
        }
    }
    
    // 5. 设置结果
    if (best_score > 0) {
        result.transform.setOrigin(tf::Vector3(best_x, best_y, 0));
        result.transform.setRotation(tf::createQuaternionFromYaw(best_yaw));
        result.score = best_score;
        result.valid = true;
        result.iterations = (2 * search_steps_x + 1) * (2 * search_steps_y + 1) * (2 * search_steps_yaw + 1);
    }
    
    return result;
}

ScanMatcher::MatchResult ScanMatcher::matchByICP(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_cloud,
    const tf::Transform& initial_guess) {
    
    MatchResult result;
    
    // 1. 降采样以提高速度
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_current(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_reference(new pcl::PointCloud<pcl::PointXYZI>());
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
    
    voxel_filter.setInputCloud(current_scan);
    voxel_filter.filter(*filtered_current);
    
    voxel_filter.setInputCloud(reference_cloud);
    voxel_filter.filter(*filtered_reference);
    
    if (filtered_current->empty() || filtered_reference->empty()) {
        ROS_WARN("ICP: Filtered clouds are empty");
        return result;
    }
    
    // 2. 移除离群点
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outlier_filter;
    outlier_filter.setRadiusSearch(0.1);
    outlier_filter.setMinNeighborsInRadius(5);
    
    outlier_filter.setInputCloud(filtered_current);
    outlier_filter.filter(*filtered_current);
    
    outlier_filter.setInputCloud(filtered_reference);
    outlier_filter.filter(*filtered_reference);
    
    // 3. 设置初始变换
    Eigen::Matrix4f init_guess_mat = tfToEigenMatrix(initial_guess);
    icp_.setInputSource(filtered_current);
    icp_.setInputTarget(filtered_reference);
    
    // 4. 执行ICP
    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    icp_.align(aligned_cloud, init_guess_mat);
    
    // 5. 检查结果
    if (icp_.hasConverged()) {
        Eigen::Matrix4f final_transform = icp_.getFinalTransformation();
        
        result.transform = eigenMatrixToTf(final_transform);
        result.score = 1.0 / (1.0 + icp_.getFitnessScore());  // 转换得分
        result.valid = true;

        // 兼容不同PCL版本
        #if PCL_VERSION_COMPARE(>=, 1, 11, 0)
            result.iterations = icp_.getFinalNumIteration();
        #else
            result.iterations = icp_.nr_iterations_;
        #endif
        // result.iterations = icp_.getFinalNumIteration();
        
        // 调试输出
        if (enable_debug_) {
            ROS_DEBUG("ICP converged: fitness=%.4f, iterations=%d",
                     icp_.getFitnessScore(), result.iterations);
        }
    } else {
        ROS_WARN("ICP did not converge");
        result.score = 0.0;
    }
    
    return result;
}

 

ScanMatcher::MatchResult ScanMatcher::matchByNDT(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& reference_cloud,
    const tf::Transform& initial_guess) {
    
    MatchResult result;
    
    // 1. 降采样
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_current(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_reference(new pcl::PointCloud<pcl::PointXYZI>());
    
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    
    voxel_filter.setInputCloud(current_scan);
    voxel_filter.filter(*filtered_current);
    
    voxel_filter.setInputCloud(reference_cloud);
    voxel_filter.filter(*filtered_reference);
    
    if (filtered_current->empty() || filtered_reference->empty()) {
        ROS_WARN("NDT: Filtered clouds are empty");
        return result;
    }
    
    // 2. 设置初始变换
    Eigen::Matrix4f init_guess_mat = tfToEigenMatrix(initial_guess);
    ndt_.setInputSource(filtered_current);
    ndt_.setInputTarget(filtered_reference);
    
    // 3. 执行NDT
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    ndt_.align(output_cloud, init_guess_mat);
    
    // 4. 检查结果
    if (ndt_.hasConverged()) {
        Eigen::Matrix4f final_transform = ndt_.getFinalTransformation();
        
        result.transform = eigenMatrixToTf(final_transform);
        result.score = ndt_.getTransformationProbability();  // NDT提供概率
        result.valid = true;
        result.iterations = ndt_.getFinalNumIteration();
        
        if (enable_debug_) {
            ROS_DEBUG("NDT converged: probability=%.4f, iterations=%d",
                     result.score, result.iterations);
        }
    } else {
        ROS_WARN("NDT did not converge");
        result.score = 0.0;
    }
    
    return result;
}

double ScanMatcher::computeCorrelationScore(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const std::vector<std::vector<bool>>& grid,
    float grid_origin_x, float grid_origin_y,
    float resolution, const tf::Transform& transform) {
    
    int matches = 0;
    int total_points = 0;
    
    for (const auto& point : scan->points) {
        // 应用变换
        tf::Vector3 point_vec(point.x, point.y, point.z);
        tf::Vector3 transformed_vec = transform * point_vec;
        
        // 转换到栅格坐标
        int grid_x = static_cast<int>((transformed_vec.x() - grid_origin_x) / resolution);
        int grid_y = static_cast<int>((transformed_vec.y() - grid_origin_y) / resolution);
        
        // 检查边界
        if (grid_y >= 0 && grid_y < static_cast<int>(grid.size()) &&
            grid_x >= 0 && grid_x < static_cast<int>(grid[0].size())) {
            
            total_points++;
            if (grid[grid_y][grid_x]) {
                matches++;
            }
        }
    }
    
    if (total_points == 0) return 0.0;
    return static_cast<double>(matches) / total_points;
}

void ScanMatcher::updateReferenceMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (map_cloud && !map_cloud->empty()) {
        *reference_map_ = *map_cloud;
        ROS_DEBUG("Reference map updated with %lu points", reference_map_->size());
    }
}

void ScanMatcher::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    reference_map_->clear();
    total_matches_ = 0;
    successful_matches_ = 0;
    total_match_time_ = 0.0;
    
    ROS_INFO("ScanMatcher reset");
}

void ScanMatcher::getStatistics(int& total_matches, int& successful_matches, 
                               double& avg_time, double& success_rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    total_matches = total_matches_;
    successful_matches = successful_matches_;
    
    if (successful_matches_ > 0) {
        avg_time = total_match_time_ / successful_matches_;
    } else {
        avg_time = 0.0;
    }
    
    if (total_matches_ > 0) {
        success_rate = 100.0 * successful_matches_ / total_matches_;
    } else {
        success_rate = 0.0;
    }
}

// 工具函数
Eigen::Matrix4f ScanMatcher::tfToEigenMatrix(const tf::Transform& transform) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    matrix(0, 3) = transform.getOrigin().x();
    matrix(1, 3) = transform.getOrigin().y();
    matrix(2, 3) = transform.getOrigin().z();
    
    tf::Matrix3x3 rot(transform.getRotation());
    matrix(0, 0) = rot[0][0]; matrix(0, 1) = rot[0][1]; matrix(0, 2) = rot[0][2];
    matrix(1, 0) = rot[1][0]; matrix(1, 1) = rot[1][1]; matrix(1, 2) = rot[1][2];
    matrix(2, 0) = rot[2][0]; matrix(2, 1) = rot[2][1]; matrix(2, 2) = rot[2][2];
    
    return matrix;
}

tf::Transform ScanMatcher::eigenMatrixToTf(const Eigen::Matrix4f& matrix) {
    tf::Transform transform;
    
    transform.setOrigin(tf::Vector3(matrix(0, 3), matrix(1, 3), matrix(2, 3)));
    
    tf::Matrix3x3 rot(
        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        matrix(2, 0), matrix(2, 1), matrix(2, 2)
    );
    
    tf::Quaternion q;
    rot.getRotation(q);
    transform.setRotation(q);
    
    return transform;
}

} // namespace sonar_slam