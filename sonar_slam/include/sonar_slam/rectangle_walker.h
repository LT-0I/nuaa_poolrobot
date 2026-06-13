#ifndef RECTANGLE_WALKER_H
#define RECTANGLE_WALKER_H

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

namespace sonar_slam {

/**
 * @brief 独立的口字型行走控制器
 * 支持红外测距防碰撞、精确里程计位移控制
 */
class RectangleWalker {
public:
    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit RectangleWalker(ros::NodeHandle& nh);
    
    /**
     * @brief 析构函数
     */
    ~RectangleWalker();
    
    /**
     * @brief 开始口字型行走
     * @param side_length 边长（米），默认4米
     * @param sides 边数，默认4条边
     * @param wait_time 每个角停留时间（秒），默认55秒
     * @return 成功返回true
     */
    bool start(float side_length = 4.0f, int sides = 4, float wait_time = 55.0f);
    
    /**
     * @brief 停止行走
     */
    void stop();
    
    /**
     * @brief 获取当前状态
     */
    std::string getStatus() const;
    
    /**
     * @brief 获取进度百分比
     */
    float getProgress() const;
    
    /**
     * @brief 是否正在运行
     */
    bool isRunning() const { return is_running_; }

private:
    // 状态枚举
    enum class State {
        IDLE,           // 空闲
        WALKING,        // 行走中
        TURNING,        // 旋转中
        WAITING,        // 等待中
        COMPLETED,      // 完成
        ERROR,          // 错误
        OBSTACLE_AVOID  // 避障中
    };
    
    // 行走子状态
    enum class WalkSubState {
        NORMAL,         // 正常行走
        APPROACH_WALL,  // 接近墙壁
        AT_WALL,        // 已到达墙壁
        EMERGENCY_STOP  // 紧急停止
    };
    
    // 配置参数（支持命令行传入）
    struct Config {
        // 行走参数
        float side_length;          // 边长（米）
        int sides;                  // 边数
        float wait_time;            // 停留时间（秒）
        float walk_speed;           // 行走速度（米/秒）
        float turn_speed;           // 旋转速度（弧度/秒）
        
        // 红外防碰撞参数
        float wall_detect_distance;     // 墙壁检测距离（米）
        float wall_safe_distance;       // 墙壁安全距离（米）
        float emergency_distance;       // 紧急停止距离（米）
        int wall_detect_samples;        // 墙壁检测需要的连续采样数
        float wall_approach_speed;      // 接近墙壁时的速度（米/秒）
        
        // 行走精度参数
        float angle_tolerance;          // 角度容差（弧度）
        float distance_tolerance;       // 距离容差（米）
        float timeout_per_side;         // 每条边超时（秒）
        float timeout_per_turn;         // 每次旋转超时（秒）
        
        Config() 
            : side_length(4.0f)
            , sides(4)
            , wait_time(55.0f)
            , walk_speed(0.2f)
            , turn_speed(0.3f)
            , wall_detect_distance(0.35f)
            , wall_safe_distance(0.20f)
            , emergency_distance(0.10f)
            , wall_detect_samples(3)
            , wall_approach_speed(0.08f)
            , angle_tolerance(0.0175f)
            , distance_tolerance(0.05f)
            , timeout_per_side(60.0f)
            , timeout_per_turn(10.0f)
        {}
    };
    
    // 位姿结构
    struct Pose2D {
        double x, y, theta;
        Pose2D() : x(0), y(0), theta(0) {}
    };
    
    // 传感器数据结构
    struct SensorData {
        float front_left;      // 左前红外 (前左)
        float front_right;     // 右前红外 (前右)
        float front_min;       // 前方最小距离
        float left_front;      // 左前侧
        float left_back;       // 左后侧
        
        SensorData() : front_left(10.0), front_right(10.0), front_min(10.0),
                       left_front(10.0), left_back(10.0) {}
        
        void update(float f_l, float f_r, float l_f, float l_b) {
            front_left = (f_l > 0.01 && f_l < 10.0) ? f_l : 10.0;
            front_right = (f_r > 0.01 && f_r < 10.0) ? f_r : 10.0;
            front_min = std::min(front_left, front_right);
            left_front = (l_f > 0.01 && l_f < 10.0) ? l_f : 10.0;
            left_back = (l_b > 0.01 && l_b < 10.0) ? l_b : 10.0;
        }
        
        bool isWallDetected() const {
            return front_min < 0.35f;
        }
        
        bool isEmergency() const {
            return front_min < 0.10f;
        }
        
        float getWallDistance() const {
            return front_min;
        }
    };
    
    // 主要执行函数
    bool executeRectangleWalk();
    bool walkCurrentSide();
    bool turnAtCorner();
    bool waitAtCorner();
    
    // 行走控制函数（基于里程计位移）
    bool walkDistance(float distance, float speed, bool enable_wall_detection = true);
    bool rotateToAngle(float target_angle);
    bool stopMoving();
    
    // 里程计位移管理
    double getOdomDistance();
    void updateOdomDistance();
    void resetOdomDistance();
    
    // 红外防碰撞检测
    bool isWallNearby();
    bool isEmergencyStop();
    float getFrontWallDistance();
    void updateSensorData();
    bool checkWallConsistently();
    
    // 辅助函数
    void updateCurrentPose();
    double calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    double normalizeAngle(double angle);
    void publishVelocity(double linear_x, double angular_z);
    void publishZeroVelocity();
    void notifyState(const std::string& info);
    void loadParameters();
    
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
    
    // 状态管理
    State current_state_;
    WalkSubState walk_sub_state_;
    Config config_;
    int current_side_;
    bool is_running_;
    mutable std::mutex mutex_;
    std::thread execution_thread_;
    
    // 传感器数据
    SensorData sensor_data_;
    std::deque<float> front_distance_history_;  // 前方距离历史记录
    int valid_sensor_count_;                    // 有效传感器计数
    
    // 里程计位移数据
    double odom_distance_;           // 总行走距离（米）
    double last_odom_x_;             // 上一次里程计X位置
    double last_odom_y_;             // 上一次里程计Y位置
    bool odom_initialized_;          // 里程计是否已初始化
    
    // 位置数据
    Pose2D current_pose_;
    double side_start_odom_;         // 当前边起始里程计距离
    bool odom_received_;
    
    // 进度统计
    double distance_walked_;
    ros::Time side_start_time_;
    ros::Time wait_start_time_;
    ros::Time turn_start_time_;
    
    // PID控制器（用于直线纠偏）
    struct PID {
        double kp, ki, kd;
        double integral;
        double last_error;
        ros::Time last_time;
        
        PID() : kp(0.6), ki(0.02), kd(0.1), integral(0), last_error(0) {}
        
        double update(double error, const ros::Time& current_time) {
            double dt = (current_time - last_time).toSec();
            if (dt <= 0) dt = 0.02;
            if (dt > 0.1) dt = 0.02;
            
            integral += error * dt;
            // 积分限幅
            if (integral > 0.5) integral = 0.5;
            if (integral < -0.5) integral = -0.5;
            
            double derivative = (error - last_error) / dt;
            double output = kp * error + ki * integral + kd * derivative;
            
            last_error = error;
            last_time = current_time;
            
            // 输出限幅
            if (output > 0.25) output = 0.25;
            if (output < -0.25) output = -0.25;
            
            return output;
        }
        
        void reset() {
            integral = 0;
            last_error = 0;
            last_time = ros::Time::now();
        }
    };
    
    PID heading_pid_;        // 航向PID
    double target_heading_;  // 目标航向角
    
    // 回调函数
    std::function<void(const std::string&)> state_callback_;
};

} // namespace sonar_slam

#endif // RECTANGLE_WALKER_H