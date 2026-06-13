#ifndef SONAR_SLAM_SONAR_SLAM_NODE_H
#define SONAR_SLAM_SONAR_SLAM_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <mars_msgs/Infrared.h>
#include <mars_msgs/MarsStates.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
 
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>


#include "sonar_slam/scan_matcher.h"

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <fstream>
#include <string>
#include <utility>
#include <memory>
#include <deque>

namespace sonar_slam {



// 系统健康状态
enum SystemStatus {
    STATUS_NORMAL,
    STATUS_FAST_MAPPING,
    STATUS_HIGH_LOAD,
    STATUS_LOW_BATTERY
};

// 覆盖状态
enum CoverageState {
    COVERAGE_IDLE,
    COVERAGE_RUNNING,
    COVERAGE_PAUSED,
    COVERAGE_COMPLETE
};

// 障碍物严重程度
enum ObstacleSeverity {
    SEVERITY_NONE,
    SEVERITY_WARNING,
    SEVERITY_DANGER,
    SEVERITY_CRITICAL
};


// ==================== 扫描匹配相关 ====================
enum ScanMatchResult {
    MATCH_SUCCESS,
    MATCH_LOW_SCORE,
    MATCH_NO_REFERENCE,
    MATCH_FAILED
};

// ==================== 系统状态枚举 ====================
enum SystemState {
    STATE_INITIAL_MAPPING = 0,  // 初始建图
    STATE_GAP_EXPLORATION,      // 缺口探索
    STATE_COVERAGE_VERIFY,      // 覆盖验证
    STATE_RETURN_HOME,          // 返回起点
    STATE_COMPLETE,             // 完成
    STATE_EMERGENCY_STOP        // 紧急停止
};

 

// 地图单元常量
static const int8_t CELL_UNKNOWN = -1;
static const int8_t CELL_FREE = 0;
static const int8_t CELL_OCCUPIED = 100;

// ==================== 结构体定义 ====================

// 导航命令结构
struct NavigationCommand {
    float linear_velocity;
    float angular_velocity;
    
    NavigationCommand() : linear_velocity(0.0f), angular_velocity(0.0f) {}
    NavigationCommand(float lin, float ang) : linear_velocity(lin), angular_velocity(ang) {}
};

// 墙壁检测结果
struct WallDetectionResult {
    bool detected;
    float distance;
    float angle;
    float confidence;
    
    WallDetectionResult() : detected(false), distance(0.0f), angle(0.0f), confidence(0.0f) {}
};

// 障碍物检测结果
struct ObstacleDetection {
    ObstacleSeverity severity;
    float left_distance;
    float right_distance;
    float center_distance;
    geometry_msgs::Point position;
    
    ObstacleDetection() : severity(SEVERITY_NONE), left_distance(0.0f), 
                         right_distance(0.0f), center_distance(0.0f) {}
};

// 覆盖分析结果
struct CoverageAnalysis {
    float coverage_percentage;
    float quality_score;
    int total_cells;
    int covered_cells;
    int poor_coverage_cells;
    float estimated_time_remaining;
    
    CoverageAnalysis() : coverage_percentage(0.0f), quality_score(0.0f),
                        total_cells(0), covered_cells(0), poor_coverage_cells(0),
                        estimated_time_remaining(0.0f) {}
};

// 覆盖优化
struct CoverageOptimization {
    std::vector<geometry_msgs::Point> new_path;
    float improvement;
    float estimated_time_saving;
    
    CoverageOptimization() : improvement(0.0f), estimated_time_saving(0.0f) {}
};


// ==================== 扫描匹配相关 ====================
struct ScanMatchInfo {
    tf::Transform correction;
    float score;
    ScanMatchResult result;
    int iterations;
    double match_time;
    
    ScanMatchInfo() : score(0.0f), result(MATCH_FAILED), 
                     iterations(0), match_time(0.0) {}
};

 
// ====================  缺口信息结构体 ====================
struct GapInfo {
    geometry_msgs::Point position;  // 缺口位置
    float size;                     // 缺口大小（估算）
    float distance;                 // 到机器人距离
    int priority;                   // 优先级
    
    GapInfo() : size(0), distance(0), priority(0) {
        position.x = 0;
        position.y = 0;
        position.z = 0;
    }
    
    GapInfo(float x, float y, float s, float d, int p) : 
        size(s), distance(d), priority(p) {
        position.x = x;
        position.y = y;
        position.z = 0;
    }
    
    // 添加相等运算符（必须）
    bool operator==(const GapInfo& other) const {
        float dx = position.x - other.position.x;
        float dy = position.y - other.position.y;
        float dist_sq = dx*dx + dy*dy;
        
        // 如果位置很接近（0.5米内），认为是同一个缺口
        return dist_sq < 0.25f;  // 0.5^2 = 0.25
    }
    
    // 添加不相等运算符（建议）
    bool operator!=(const GapInfo& other) const {
        return !(*this == other);
    }
    
    //  如果需要排序，添加小于运算符
    bool operator<(const GapInfo& other) const {
        // 按优先级降序，然后按距离升序
        if (priority != other.priority) {
            return priority > other.priority;  // 优先级高的在前
        }
        return distance < other.distance;  // 距离近的在前
    }
};

// ==================== 卡尔曼滤波器 ====================

class KalmanFilter {
private:
    float Q;  // 过程噪声协方差
    float R;  // 测量噪声协方差
    float P;  // 估计误差协方差
    float X;  // 状态估计
    float K;  // 卡尔曼增益

public:
    KalmanFilter(float q = 0.1, float r = 0.1) : Q(q), R(r), P(1.0), X(0), K(0) {}
    
    float update(float measurement) {
        // 预测
        P = P + Q;
        
        // 更新
        K = P / (P + R);
        X = X + K * (measurement - X);
        P = (1 - K) * P;
        
        return X;
    }
};

// ==================== 主类定义 ====================

class SonarSlamNode
{
private:
    // 前沿点结构
    struct Frontier {
        int x, y;           // 地图坐标
        float distance;     // 到机器人的距离
        float weight;       // 权重（距离 + 未知区域大小）
        
        Frontier(int x = 0, int y = 0, float dist = 0.0f, float w = 0.0f) 
            : x(x), y(y), distance(dist), weight(w) {}
        
        bool operator<(const Frontier& other) const {
            return weight < other.weight;
        }
    };
    
    // 机器人位姿结构
    struct Pose2D {
        float x, y, theta;
        Pose2D() : x(0), y(0), theta(0) {}
        Pose2D(float x, float y, float theta) : x(x), y(y), theta(theta) {}
    };
 
    struct Gap {
        float x;        // 世界坐标 X
        float y;        // 世界坐标 Y  
        float size;     // 缺口大小（米）
        int priority;   // 优先级 1-10
        
        Gap() : x(0), y(0), size(0), priority(1) {}
        Gap(float _x, float _y, float _s, int _p) 
            : x(_x), y(_y), size(_s), priority(_p) {}
    };

    // ==================== 地图完整性检查 ====================
    struct BoundaryAnalysis {
        bool is_closed;              // 边界是否闭合
        float closure_confidence;    // 闭合置信度 0-1
        std::vector<Gap> gaps;       // 所有缺口
        float total_gap_length;      // 总缺口长度
        int boundary_points_count;   // 边界点数
    };

 
    // ==================== 地图完整性相关结构体 ====================
    // 边界缺口结构
    struct BoundaryGap {
        geometry_msgs::Point center;      // 缺口中心
        float size;                       // 缺口大小（米）
        int priority;                     // 优先级 1-10
        float distance_to_robot;          // 到机器人距离
        
        BoundaryGap() : size(0), priority(1), distance_to_robot(0) {
            center.x = 0;
            center.y = 0;
            center.z = 0;
        }
        
        BoundaryGap(float x, float y, float s, int p) : 
            size(s), priority(p), distance_to_robot(0) {
            center.x = x;
            center.y = y;
            center.z = 0;
        }
    };

    // 地图完整性评估
    struct MapCompleteness {
        float coverage_percentage;        // 覆盖率
        float boundary_closure;           // 边界闭合度 0-1
        std::vector<BoundaryGap> critical_gaps; // 关键缺口
        bool is_closed;                   // 是否完全闭合
        
        MapCompleteness() : coverage_percentage(0), boundary_closure(0), is_closed(false) {}
    };
 
 

public:
    SonarSlamNode();
    ~SonarSlamNode();
    
private:
    // ==================== 核心回调函数 ====================
    void sonarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void ultrasonicCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void infraredCallback(const mars_msgs::Infrared::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void coverageStateCallback(const std_msgs::Int32::ConstPtr& msg);
    
    // ==================== 定时器回调 ====================
    void updateTimerCallback(const ros::TimerEvent& event);
    void stateMachineCallback(const ros::TimerEvent& event);
    void sensorProcessingCallback(const ros::TimerEvent& event);
    void coverageMonitoringCallback(const ros::TimerEvent& event);
    
    // ==================== 地图管理 ====================
    void initMap();
    void initCostMap();
    void updateMapWithPoint(float world_x, float world_y);
    void updateCostMapWithUltrasonic(const WallDetectionResult& wall_result);
    void publishMap();
    void publishCompressedMap();
    void saveMap(const std::string& filename);
    float calculateMapCoverage();
    void checkMapStability();
    SystemStatus getCurrentSystemStatus();
    
    // ==================== 动态建图核心 ====================
    bool findNextGoal();  // 寻找下一个探索目标
    void findFrontiers(std::vector<Frontier>& frontiers);  // 寻找前沿点
    Frontier selectBestFrontier(const std::vector<Frontier>& frontiers);  // 选择最佳前沿点
    void navigateToTarget();  // 导航到目标点
    void returnToStart();     // 返回起点
    void stopRobot();         // 停止机器人
    
    // ==================== 导航控制 ====================
    NavigationCommand calculateNavigationCommand(float distance, float angle_diff,
                                                float robot_x, float robot_y, float robot_yaw);
    bool checkObstacleAhead(float lookahead_distance);  // 前方障碍物检查
    void handleObstacleAvoidance(geometry_msgs::Twist& cmd_vel, float distance, float angle_diff);
    geometry_msgs::Twist applySafetyLimits(const geometry_msgs::Twist& cmd_vel);  // 速度安全限制
    
    // ==================== 状态机处理 ====================
    void handleInitialMappingState();      // 初始建图状态
    void handleGuideToStartState();        // 引导到起点状态
    void handleCoverageMonitorState();     // 覆盖监控状态
    void handleCoverageAssistState();      // 覆盖辅助状态
    void handleReturnHomeState();          // 返回起点状态
    void handleCompleteState();            // 完成状态
    void handleEmergencyStopState();       // 紧急停止状态
    
    // ==================== 传感器处理 ====================
    bool validateUltrasonicData(const std_msgs::Int32MultiArray::ConstPtr& msg);
    float applyKalmanFilter(float current_value, float new_value, const std::string& sensor_name);
    bool isUltrasonicDataValid(float front_value, float rear_value);
    WallDetectionResult detectRightWallDetailed();
    float convertInfraredData(int raw_value);
    ObstacleDetection checkFrontObstacles();
    bool checkEmergencyConditions();       // 检查紧急情况
    void triggerEmergencyStop();           // 触发紧急停止
    void executeEmergencyProtocol();       // 执行紧急协议
    
    // ==================== 覆盖相关 ====================
    CoverageAnalysis analyzeCoverageQuality();
    CoverageOptimization optimizeCoveragePath();
    void applyCoverageOptimization(const CoverageOptimization& optimization);
    bool shouldAssistCoverage();
    geometry_msgs::Point calculateCoverageAssistPoint();
    void updateNavigationStrategy(const WallDetectionResult& wall_result);
    
    // ==================== 工具函数 ====================
    bool getRobotPosition(float& x, float& y);
    bool getRobotPose(float& x, float& y, float& yaw);
    bool getRobotMapPosition(int& map_x, int& map_y);
    int worldToMapX(float world_x);
    int worldToMapY(float world_y);
    float mapToWorldX(int map_x);
    float mapToWorldY(int map_y);
    void bresenhamLine(int x0, int y0, int x1, int y1, 
                       std::vector<std::pair<int, int>>& points);
    float normalizeAngle(float angle);
    bool isPointReachable(const geometry_msgs::Point& point);
    bool isPointSafe(const geometry_msgs::Point& point);
    bool detectRightWall();
    geometry_msgs::Point calculateWallFollowingGoal();
    
    // ==================== 可视化 ====================
    void visualizeFrontiers();
    void visualizeScanPoints();
    void visualizeGoal(float x, float y);
    void visualizeNavigationState(float robot_x, float robot_y, float robot_yaw,
                                 float target_x, float target_y);
    
    // ==================== 系统管理 ====================
    void publishSystemStatistics();
    void publishSensorStatus(const WallDetectionResult& wall_result);
    void recordSensorData(const std::string& sensor_type, float value1, float value2 = 0.0f);
    void publishStaticTransforms();
    //TF发布函数
    void publishMapToOdomTF();
    
    // ==================== 辅助函数 ====================
    static std::string stateToString(SystemState state);
    static std::string statusToString(SystemStatus status);

    // ==================== 扫描匹配 ====================
    bool performScanMatching(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,
        tf::Transform& pose_correction);

    void updateRobotPoseWithCorrection(const tf::Transform& correction);
    tf::Transform getCurrentOdomTransform();
    void publishMatchedCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const tf::Transform& transform);
    void publishPoseCorrection(const tf::Transform& correction);

    // TF发布函数（需要重载版本）
    void publishMapToOdomTFWithPose(const tf::Transform& map_to_base,
                        const tf::Transform& odom_to_base);

 
    std::vector<Gap> findBoundaryGaps();
    bool hasLargeGaps();
    Gap findNearestGap(const std::vector<Gap>& gaps, float robot_x, float robot_y);
    bool shouldFinishExploration();  // 如果没有这个函数
 
    BoundaryAnalysis analyzeBoundaryCompleteness();
    float measureGapSize(int start_x, int start_y);
    std::vector<Gap> findRealBoundaryGaps();
    bool isMapClosed();
    bool shouldContinueExploration();
    bool findNextGoalFromFrontiers();  // 新函数，从前沿点寻找目标


    // 地图完整性检查相关
   
    std::vector<BoundaryGap> detectBoundaryGaps();
    bool isBoundaryClosed();
    BoundaryGap selectBestGapToFill(const std::vector<BoundaryGap>& gaps);
    bool shouldContinueExploring();
    
    // 智能导航辅助
    bool isPathClearToTarget(float target_x, float target_y);
    geometry_msgs::Point findSafeApproachPoint(const BoundaryGap& gap);
    
    // 缺口检测相关
    bool isMapComplete();
    std::vector<GapInfo> findAndRankGaps();
    void performGapScanning();
    
    // 新增状态处理
    void handleGapExplorationState();
    void handleCoverageVerifyState();
    
    // 辅助函数
    bool checkIfAtGap(const geometry_msgs::Point& gap_pos);
    void updateGapPriority(GapInfo& gap);

    bool checkIfNeedAdditionalScanning(); 

    void publishSmoothedTF(const tf::Transform& raw_transform, const ros::Time& stamp);

    float calculateFrontierWeight(int x, int y, float distance, int unknown_count);

    void staticMappingDoneCallback(const std_msgs::Bool::ConstPtr& msg);
    void initialMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void mergeInitialMap(const nav_msgs::OccupancyGrid& initial_map);
   

private:
    // ==================== ROS组件 ====================
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber sonar_cloud_sub_;      // 声纳点云
    ros::Subscriber ultrasonic_sub_;       // 超声波
    ros::Subscriber infrared_sub_;         // 红外
    ros::Subscriber odom_sub_;             // 里程计
    ros::Subscriber imu_sub_;              // IMU
    ros::Subscriber coverage_state_sub_;   // 覆盖状态
    
    // 发布器
    ros::Publisher cmd_vel_pub_;           // 速度控制
    ros::Publisher map_pub_;               // 地图
    ros::Publisher costmap_pub_;           // 代价地图
    ros::Publisher frontier_marker_pub_;   // 前沿点可视化
    ros::Publisher goal_marker_pub_;       // 目标点可视化
    ros::Publisher coverage_marker_pub_;   // 覆盖路径可视化
    ros::Publisher sensor_marker_pub_;     // 传感器数据可视化
    ros::Publisher system_state_pub_;      // 系统状态
    ros::Publisher coverage_progress_pub_; // 覆盖进度

        // 添加发布器用于调试
    ros::Publisher matched_cloud_pub_;
    ros::Publisher pose_correction_pub_;
    
    // 定时器
    ros::Timer update_timer_;              // 地图更新定时器
    ros::Timer state_timer_;               // 状态机定时器
    ros::Timer sensor_timer_;              // 传感器处理定时器
    ros::Timer coverage_timer_;            // 覆盖监控定时器


    ros::Subscriber static_mapping_done_sub_;
    ros::Subscriber initial_map_sub_;
    
    // TF
    tf::TransformListener* tf_listener_;
    tf::TransformBroadcaster* tf_broadcaster_;
    
    // ==================== 地图数据 ====================
    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid costmap_;
    std::vector<float> log_odds_map_;      // 概率地图
    std::vector<int> cell_history_;        // 单元更新历史
    bool map_updated_;
    int last_known_cells_;
    int map_stability_counter_;
    
    // ==================== 机器人状态 ====================
    nav_msgs::Odometry current_odom_;
    sensor_msgs::Imu current_imu_;
    Pose2D start_pose_;
    bool has_start_pose_;
    // std::vector<std::pair<float, float>> scan_points_;
    std::deque<std::pair<float, float>> scan_points_; 
    
    // ==================== 动态建图状态 ====================
    SystemState current_state_;
    CoverageState coverage_state_;
    bool exploration_complete_;
    bool has_goal_;
    bool is_moving_;
    float current_target_x_, current_target_y_;
    
    // ==================== 传感器数据 ====================
    float ultrasonic_front_right_;
    float ultrasonic_rear_right_;
    float infrared_front_left_;
    float infrared_front_right_;

   // ==================== 传感器数据 ====================
    sonar_slam::ScanMatcher scan_matcher_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_scan_;
    tf::Transform last_pose_;
    bool scan_matcher_initialized_;
    

    
    // ==================== 建图统计 ====================
    float map_coverage_;
    float coverage_quality_;
    float ultrasonic_spacing_;
    int scan_count_;
    int update_cycle_;
    ros::Time last_update_time_;
    ros::Time last_state_time_;

    
    
    // ==================== 参数配置 ====================
    double update_interval_;
    int fast_update_threshold_;
    double map_resolution_;
    int map_width_, map_height_;
    double map_origin_x_, map_origin_y_;
    float coverage_complete_threshold_;
    float coverage_quality_threshold_;
    int stability_threshold_;
    float frontier_weight_distance_;
    float frontier_weight_unknown_;
    float max_linear_speed_;
    float max_angular_speed_;
    float goal_tolerance_;
    float wall_following_distance_;
    
    // 探索状态
    bool exploring_for_gaps_;          // 是否在填补缺口模式
    std::vector<BoundaryGap> current_gaps_; // 当前检测到的缺口
    int gap_fill_attempts_;            // 缺口填补尝试次数

     // ====================  成员变量 ====================
    GapInfo current_gap_target_;              // 当前目标缺口
    ros::Time gap_exploration_start_time_;    // 缺口探索开始时间
    int gap_fill_count_;                      // 缺口填补次数
    bool map_verified_complete_;              // 地图已验证完整

    // ==================== TF滤波相关成员====================
    tf::Transform last_map_to_odom_;
    ros::Time last_tf_time_;
    bool first_tf_;
    double tf_smooth_factor_;  // 滤波系数，可调整

    bool has_initial_map_;
    nav_msgs::OccupancyGrid initial_map_;
 
    // ==================== 线程安全 ====================
    std::mutex data_mutex_;
};

} // namespace sonar_slam

#endif // SONAR_SLAM_SONAR_SLAM_NODE_H