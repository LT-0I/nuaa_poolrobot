#ifndef SONAR_SLAM_ROBOT_WALL_CLEANER_H
#define SONAR_SLAM_ROBOT_WALL_CLEANER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <std_msgs/Int32MultiArray.h>

namespace sonar_slam {

// 键盘模拟器结构（复用）
struct KeyboardSimulator {
    float linear_inc = 0.01f;
    float angular_inc = 0.3f;
    float max_linear = 0.35f;
    float max_angular = 0.5f;
    int idle_threshold = 50;
    
    float current_vx = 0.0f;
    float current_vy = 0.0f;
    float target_vx = 0.0f;
    float target_angular = 0.0f;
    int idle_counter = 0;
    bool key_pressed = false;
    char current_key = 0;
    ros::Time key_press_time;
    
    void reset() {
        current_vx = 0.0f;
        current_vy = 0.0f;
        target_vx = 0.0f;
        target_angular = 0.0f;
        idle_counter = 0;
        key_pressed = false;
        current_key = 0;
        key_press_time = ros::Time::now();
    }
};

class RobotWallCleaner {
public:
    RobotWallCleaner(ros::NodeHandle& nh);
    ~RobotWallCleaner();
    
    // 主要功能
    bool findPoolWall(float timeout = 60.0f);
    bool cleanPoolWallLoop(float timeout = 300.0f);
    void stop();
    std::string getStatus() const;
    
    // 设置水泵速度
    void setPumpSpeed(float speed);

private:
    // ROS接口
    ros::NodeHandle& nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ultra_sub_;
    ros::ServiceServer find_wall_service_;
    ros::ServiceServer clean_service_;
    ros::Timer keyboard_timer_;
    
    // 键盘模拟器
    KeyboardSimulator keyboard_sim_;
    std::mutex keyboard_mutex_;
    
    // 状态
    enum CleanerState {
        STATE_IDLE = 0,
        STATE_FINDING_WALL,
        STATE_CLEANING,
        STATE_ERROR
    };
    CleanerState cleaner_state_;
    
    // 里程计数据
    bool odom_received_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Pose2D current_pose_;
    
    // 传感器数据
    bool ultra_received_;
    float right_front_dist_;   // 右侧前超声波
    float right_rear_dist_;    // 右侧后超声波
    float infra_front_left_;   // 前方左红外
    float infra_front_right_;  // 前方右红外
    
    // 水泵
    float current_pump_speed_;
    
    // 控制标志
    bool disabling_timer_;
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void keyboardTimerCallback(const ros::TimerEvent& event);
    
    // 服务回调
    bool findWallServiceCallback(std_srvs::Trigger::Request& req,
                                  std_srvs::Trigger::Response& res);
    bool cleanServiceCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);
    
    // 工具函数
    void updateCurrentPose();
    void applyKeyboardCommand(char key);
    float calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    bool waitForOdom(float timeout);
    void initKeyboardSimulator();
    float normalizeAngle(float angle);
    
    // 核心功能函数
    bool executeFindPoolWall(float timeout);
    bool executeCleanPoolWallLoop(float timeout);
    
    // 辅助检测函数
    bool checkFrontWallDetected();   // 前方池壁检测（红外）
    bool checkRightWallDetected();   // 右侧池壁检测（超声波）
    bool isOutOfWater();              // 露出水面检测（待实现）
};

} // namespace sonar_slam

#endif // SONAR_SLAM_ROBOT_WALL_CLEANER_H