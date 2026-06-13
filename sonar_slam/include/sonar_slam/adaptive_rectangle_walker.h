#ifndef ADAPTIVE_RECTANGLE_WALKER_H
#define ADAPTIVE_RECTANGLE_WALKER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <functional>
#include <vector>
#include <deque>
#include <string>

namespace sonar_slam {

// 前向声明
class AdaptiveRectangleWalker;

// 全局指针
extern AdaptiveRectangleWalker* g_walker;
void signalHandler(int sig);

// 状态枚举
enum class SystemState {
    IDLE, INITIALIZING, MOVING, WAITING, TURNING, CALIBRATING, COMPLETED, ERROR, PAUSED
};

enum class MoveSubState {
    NORMAL, APPROACHING_WALL, AT_WALL, EMERGENCY_STOP
};

// 边信息结构
struct SideInfo {
    int index;
    std::string name;
    double design_length;
    double actual_length;
    bool is_long_side;
    bool is_first_of_type;
    double start_odom;
    double target_heading;
    
    SideInfo() 
        : index(0), design_length(0), actual_length(0), 
          is_long_side(false), is_first_of_type(false), start_odom(0), target_heading(0) {}
};

// 转角位置结构
struct CornerPosition {
    double x, y;
    std::string side;
    ros::Time timestamp;
    
    CornerPosition() : x(0), y(0) {}
    CornerPosition(double _x, double _y, const std::string& _side)
        : x(_x), y(_y), side(_side), timestamp(ros::Time::now()) {}
};

// PID控制器类
class PIDController {
public:
    PIDController();
    PIDController(double kp, double ki, double kd);
    double update(double error);
    void reset();
    void setGains(double kp, double ki, double kd);
    
private:
    double kp_, ki_, kd_;
    double integral_;
    double last_error_;
    ros::Time last_time_;
};

/**
 * @brief 自适应矩形行走控制器
 */
class AdaptiveRectangleWalker {
public:
    explicit AdaptiveRectangleWalker(ros::NodeHandle& nh);
    ~AdaptiveRectangleWalker();
    
    bool start(double width = 15.0, double height = 30.0, double wait_time = 55.0);
    void stop();
    void pause();
    void resume();
    std::string getStatus() const;
    double getProgress() const;
    std::string getCurrentWaypoint() const;
    bool isRunning() const { return is_running_; }
    void setStateCallback(std::function<void(const std::string&)> callback) {
        state_callback_ = callback;
    }

private:
    // 配置参数结构
    struct Config {
        double design_width;
        double design_height;
        double start_offset;
        double step_distance;
        double turn_threshold;
        double wall_safe_distance;
        double walk_speed;
        double approach_speed;
        double turn_speed;
        double wait_time;
        double timeout_per_side;
        double timeout_per_turn;
        double angle_tolerance;
        double distance_tolerance;
        double calibration_interval;
        double wall_detect_distance;
        double emergency_distance;
        int wall_detect_samples;
        int max_calibration_attempts;
        bool enable_auto_calibration;
        
        Config()
            : design_width(15.0)
            , design_height(30.0)
            , start_offset(2.0)
            , step_distance(4.1)
            , turn_threshold(4.0)
            , wall_safe_distance(0.3)
            , walk_speed(0.25)
            , approach_speed(0.1)
            , turn_speed(0.3)
            , wait_time(55.0)
            , timeout_per_side(120.0)
            , timeout_per_turn(15.0)
            , angle_tolerance(0.0175)
            , distance_tolerance(0.05)
            , calibration_interval(5.0)
            , wall_detect_distance(0.35)
            , emergency_distance(0.10)
            , wall_detect_samples(3)
            , max_calibration_attempts(3)
            , enable_auto_calibration(true)
        {}
    };
    
    // 实际测量数据
    struct ActualMeasurements {
        double left_side;
        double top_side;
        double right_side;
        double bottom_side;
        int calibration_count;
        
        ActualMeasurements()
            : left_side(0), top_side(0), right_side(0), bottom_side(0), calibration_count(0) {}
        
        void reset() {
            left_side = top_side = right_side = bottom_side = 0;
            calibration_count = 0;
        }
    };
    
    // 停留点结构
    struct Waypoint {
        int index;
        double distance;
        std::string side;
        std::string description;
        
        Waypoint() : index(0), distance(0) {}
        Waypoint(int idx, double dist, const std::string& s, int num)
            : index(idx), distance(dist), side(s) {
            description = side + std::to_string(num);
        }
    };
    
    // 位姿结构
    struct Pose2D {
        double x, y, theta;
        Pose2D() : x(0), y(0), theta(0) {}
    };
    
    // 传感器数据结构
    struct SensorData {
        double front_left;
        double front_right;
        double front_min;
        double left_front;
        double left_back;
        
        SensorData()
            : front_left(10.0), front_right(10.0), front_min(10.0),
              left_front(10.0), left_back(10.0) {}
        
        void update(double f_l, double f_r, double l_f, double l_b) {
            front_left = (f_l > 0.01 && f_l < 10.0) ? f_l : 10.0;
            front_right = (f_r > 0.01 && f_r < 10.0) ? f_r : 10.0;
            front_min = std::min(front_left, front_right);
            left_front = (l_f > 0.01 && l_f < 10.0) ? l_f : 10.0;
            left_back = (l_b > 0.01 && l_b < 10.0) ? l_b : 10.0;
        }
        
        bool isWallDetected(double threshold = 0.35) const {
            return front_min < threshold;
        }
        
        bool isEmergency(double threshold = 0.10) const {
            return front_min < threshold;
        }
        
        double getWallDistance() const { return front_min; }
    };
    
    // 初始化函数
    void loadParameters();
    void setupROS();
    
    // 主要执行流程
    bool executeRectangleWalk();
    bool walkSide(const SideInfo& side);
    bool turnAtCorner(const SideInfo& current_side, const SideInfo& next_side);
    bool waitAtWaypoint(const Waypoint& wp);
    
    // 行走控制
    bool rotateToAngle(double target_angle);
    bool stopMoving();
    bool walkDistance(double distance, double speed, bool enable_wall_detection = true);
    
    // 停留点生成
    std::vector<Waypoint> generateWaypoints(double side_length, 
                                             const std::string& side_name,
                                             bool use_actual_length = true);
    
    // 自适应校准
    bool checkWallConsistently();
    double calibrateSideLength(double traveled, bool is_long_side);
    void saveMeasurement(bool is_long_side, double actual_length, int side_index);
    
    // 墙壁检测与对齐
    bool alignToWall();
    double measureWallAngle();
    bool backupForTurn(double distance);
    
    // 里程计管理
    double getOdomDistance();
    void updateOdomDistance();
    void resetOdomDistance();
    
    // 传感器管理
    void updateSensorData();
    void updateCurrentPose();
    
    // 辅助函数
    double calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    double normalizeAngle(double angle);
    void publishVelocity(double linear_x, double angular_z);
    void publishZeroVelocity();
    void printMeasurementReport();
    
    // ROS回调
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void distDatasCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    
    // ROS接口
    ros::NodeHandle& nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber dist_sub_;
    
    // 配置
    Config config_;
    ActualMeasurements actual_measurements_;
    
    // 状态管理
    SystemState system_state_;
    MoveSubState move_sub_state_;
    bool is_running_;
    bool is_paused_;
    mutable std::mutex mutex_;
    std::thread execution_thread_;
    
    // 当前行走信息
    SideInfo current_side_;
    std::vector<Waypoint> current_waypoints_;
    int current_waypoint_index_;
    
    // 位置数据
    Pose2D current_pose_;
    double odom_distance_;
    double last_odom_x_;
    double last_odom_y_;
    bool odom_initialized_;
    bool odom_received_;
    
    // 传感器数据
    SensorData sensor_data_;
    std::deque<double> front_distance_history_;
    
    // 时间统计
    ros::Time side_start_time_;
    ros::Time wait_start_time_;
    ros::Time turn_start_time_;
    
    // PID控制器
    PIDController heading_pid_;
    PIDController turn_pid_;
    double target_heading_;
    
    // 转角记录
    std::deque<CornerPosition> corner_positions_;
    
    // 进度统计
    double distance_walked_;
    double total_distance_;
    int total_waypoints_;
    
    // 回调函数
    std::function<void(const std::string&)> state_callback_;
};

} // namespace sonar_slam

#endif // ADAPTIVE_RECTANGLE_WALKER_H