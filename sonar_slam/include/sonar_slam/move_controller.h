#ifndef SONAR_SLAM_MOVE_CONTROLLER_H  
#define SONAR_SLAM_MOVE_CONTROLLER_H  

#pragma once

#include <ros/ros.h>
#include "sonar_slam/MoveStraight.h"
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <fstream>      
#include <vector>        
#include <sys/stat.h>    
#include <chrono>       
#include <iomanip>      
#include <unistd.h>
#include <cstdint>
#include <deque>

namespace sonar_slam {

class MoveController {
public:
    MoveController();
    ~MoveController() {}
    
    // ========== 对外接口（ROS服务调用） ==========
    /**
     * @brief 直线移动（ROS服务接口）
     * @param distance 行走距离（米，正数为前进，负数为后退）
     * @param timeout 超时时间（秒）
     * @return true 成功，false 失败
     */
    bool moveStraight(float distance, float timeout = 30.0f);
    
    /**
     * @brief 停止机器人运动
     */
    void stop();
    
    /**
     * @brief 获取当前状态字符串
     */
    std::string getStatus() const;
    
    // ========== 服务回调 ==========
    bool moveStraightCallback(sonar_slam::MoveStraight::Request& req,
                              sonar_slam::MoveStraight::Response& res);

    void initWalkLog(const std::string& log_type, float target_distance);
    void recordWalkData(float current_distance, float target_distance);
    void saveWalkLog(bool success);
    std::string getWalkLogDirectory();
    
private:
    // ========== 核心功能 ==========
    /**
     * @brief 高级直线行走（带前方池壁检测与避让）
     * @param distance 行走距离（米，正数为前进，负数为后退）
     * @param max_speed 最大速度（米/秒）
     * @param timeout 超时时间（秒）
     * @return true 成功，false 失败
     */
    bool walkStraightWithWallAvoidanceAdvanced(float distance, float max_speed, float timeout);
    
    // ========== 初始化 ==========
    void initKeyboardSimulator();
    
    // ========== 键盘模拟 ==========
    void applyKeyboardCommand(char key);
    void keyboardTimerCallback(const ros::TimerEvent& event);
    
    // ========== 对齐功能 ==========
    void alignToRightWallOnly();
    
    // ========== 传感器回调 ==========
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    
    // ========== 辅助函数 ==========
    void updateCurrentPose();
    void updateCurrentPoseFromTF();
    bool waitForOdom(float timeout);
    float calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    float normalizeAngle(float angle);
    void stopMoving();
    
    // ========== 键盘模拟器结构 ==========
    struct KeyboardSimulator {
        float current_vx;
        float target_vx;
        float target_angular;
        float current_vy;
        
        float linear_inc;
        float angular_inc;
        float max_linear;
        float max_angular;
        
        int idle_counter;
        int idle_threshold;
        
        char current_key;
        bool key_pressed;
        ros::Time key_press_time;
        
        void reset() {
            current_vx = 0.0f;
            target_vx = 0.0f;
            target_angular = 0.0f;
            current_vy = 0.0f;
            linear_inc = 0.01f;
            angular_inc = 0.3f;
            max_linear = 0.3f;
            max_angular = 0.3f;
            idle_counter = 0;
            idle_threshold = 4;
            current_key = 0;
            key_pressed = false;
            key_press_time = ros::Time::now();
        }
    };
    
    // ========== 传感器数据结构 ==========
    struct WallSensors {
        float ultra_right_front;
        float ultra_right_rear;
        float infra_front_left;
        float infra_front_right;
        
        WallSensors() : ultra_right_front(999.0f), ultra_right_rear(999.0f),
                        infra_front_left(999.0f), infra_front_right(999.0f) {}
    };
    
    // ========== 成员变量 ==========
    // ROS接口
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ultra_sub_;
    ros::ServiceServer move_straight_srv_;
    ros::Timer keyboard_timer_;
    tf::TransformListener tf_listener_;
    
    // 状态变量
    geometry_msgs::Pose2D current_pose_;
    nav_msgs::Odometry current_odom_;
    bool odom_received_;
    int move_state_;  // 0:空闲, 1:行走中, 2:错误
    
    // 传感器数据
    WallSensors wall_sensors_;
    std::mutex sensor_mutex_;
    std::mutex keyboard_mutex_;
    
    // 键盘模拟器
    KeyboardSimulator keyboard_sim_;
    bool disabling_timer_;  // 禁用键盘定时器标志
    
    // 水泵控制
 
    float config_speed_;      // 配置的行走速度
    float config_pump_speed_; // 配置的水泵速度
    float config_wall_align_;   // 目标距右墙距离(m)，沿墙距离环用
    float user_config_pump_speed_;  // 用户命令行配置的水泵速度
    // 控制参数
    float linear_tolerance_;  // 位置容差

     // 日志记录相关
    std::ofstream walk_log_file_;
    std::string current_log_filename_;
    ros::Time last_log_time_;
    ros::Time walk_start_time_;
    
    // 日志数据结构
    struct WalkLogData {
        ros::Time timestamp;
        float right_front;
        float right_rear;
        float infra_left;
        float infra_right;
        float pose_x;
        float pose_y;
        float pose_theta;
        float target_distance;
        float current_distance;
    };
    std::vector<WalkLogData> log_buffer_;
 
    std::deque<float> front_buffer_;
    std::deque<float> rear_buffer_;
    float movingAverageFilter(float new_value, std::deque<float>& buffer, int window_size = 3);

    bool rotateToTargetAngle(float target_angle_deg, float timeout = 8.0f);

    // 右侧超声"新鲜度"跟踪：每通道真实 ~0.83Hz，原始值变化即视为收到新帧
    // (修复拐角在 20Hz 循环里反复读 0.83Hz 旧值导致提前停的根因)
    uint32_t ultra_right_seq_;     // 右侧任一通道原始值变化时自增
    ros::Time ultra_right_stamp_;  // 右侧最近一次值变化时间(陈旧看门狗用)
    int last_ultra_raw_rf_;        // 上次原始 data[2]
    int last_ultra_raw_rr_;        // 上次原始 data[3]

};
} // namespace sonar_slam

#endif // SONAR_SLAM_MOVE_CONTROLLER_H