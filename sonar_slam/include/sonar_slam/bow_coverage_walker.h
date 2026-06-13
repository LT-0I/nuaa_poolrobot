#ifndef BOW_COVERAGE_WALKER_H
#define BOW_COVERAGE_WALKER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#include <string>

namespace sonar_slam {

// 弓字形行走状态机
enum BowState {
    BOW_IDLE = 0,
    BOW_SEEK_START,      // 沿右边墙寻找右下角起点
    BOW_ALIGN_START,     // 对齐起点方向
    BOW_FIND_WALL,       // 寻找起始墙
    BOW_ALIGN_TO_WALL,   // 对齐墙边
    BOW_STRAIGHT_GO,     // 直线前进
    BOW_TURN_180,        // 180度掉头
    BOW_LATERAL_SHIFT,   // 横向平移
    BOW_FINISHED         // 完成全覆盖
};

// 沿墙动作状态（参考 MotionController）
enum FindWallAction {
    ROTATION_IN_PLACE,
    NEAR_WALL_ROTATION,
    NEAR_WALL_STRAIGHT,
    ALIGNED_WALL
};

// 键盘模拟器数据结构（从 robot_walker 复制）
struct KeyboardSimulator {
    double linear_inc = 0.01;
    double angular_inc = 0.3;
    double max_linear = 0.2;
    double max_angular = 0.5;
    int idle_threshold = 200;
    
    double current_vx = 0.0;
    double target_vx = 0.0;
    double target_angular = 0.0;
    int idle_counter = 0;
    bool key_pressed = false;
    ros::Time key_press_time;
};

class BowCoverageWalker {
public:
    BowCoverageWalker(ros::NodeHandle& nh);
    ~BowCoverageWalker();

    bool startBowCoverage(float length, float width, float robot_width, float max_speed = 0.15f);
    void stop();
    std::string getStatus() const;
    void update();

private:
    ros::NodeHandle& nh_;
    
    // ROS 通信
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ultra_data_sub_;
    ros::Timer keyboard_timer_;  // 20Hz 定时器（从 robot_walker 复制）
    ros::ServiceServer wall_follow_srv_;

    ros::ServiceServer set_wall_distance_srv_;
    
    // 键盘模拟器（从 robot_walker 复制）
    std::mutex keyboard_mutex_;
    KeyboardSimulator keyboard_sim_;
    
    // 里程计数据
    std::mutex odom_mutex_;
    nav_msgs::Odometry current_odom_;
    bool odom_received_;
    
    // 传感器数据
    std::mutex sensor_mutex_;
    double dis_l_b_, dis_l_f_, dis_f_l_, dis_f_r_, dis_f_min_;
    
    // 行走状态
    BowState bow_state_;
    FindWallAction find_wall_action_;
    int step_counter_;
    int turn_count_;
    int row_direction_;

    
    
    // 参数
    double robot_width_;
    double coverage_length_;
    double coverage_width_;
    double step_width_;
    double max_speed_;
    double wall_distance_;
    
    // 位置记录
    geometry_msgs::Pose2D start_pose_;
    geometry_msgs::Pose2D current_pose_;
    geometry_msgs::Pose2D turn_start_pose_;
    double target_distance_;
    double target_shift_distance_;
    
    // 时间控制
    ros::Time state_start_time_;
    double state_timeout_;
    
    // PID 参数
    double kp_linear_, ki_linear_;
    double kp_angular_, ki_angular_;

 
    double target_theta_;  // 新增：目标角度

    double wall_follow_distance_;  // 沿墙距离，默认0.12米
    bool wall_follow_enabled_;     // 是否启用沿墙模式
    
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ultraDataCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void keyboardTimerCallback(const ros::TimerEvent& event);  // 20Hz 定时器
    
    // 运动控制函数（从 robot_walker 复制）
    void initKeyboardSimulator();
    void applyKeyboardCommand(char key);
    void publishVelocity(double linear_x, double angular_z);
    void publishZeroVelocity();
    void setTargetVelocity(double linear_x, double angular_z);
    
    // 核心行走函数（从 robot_walker 复制并改进）
    bool walkStraight(double distance, double max_speed, double timeout);
    bool turnInPlace(double target_angle, double max_angular_speed = 0.3);

    bool executeWallFollow();
    
    // 传感器处理
    void updateCurrentPose();
    double getYaw();
    double normalizeAngle(double angle);
    double calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    bool waitForOdom(double timeout);
    
    // 沿边跟踪（参考 MotionController）
    void traceWall(double target_dist, double& pos_err_y, double& pos_err_theta);
    
    // 状态执行
    bool executeSeekStart();
    bool executeAlignStart();
    bool executeFindWall();
    bool executeAlignToWall();
    bool executeStraightGo();
    bool executeTurn180();
    bool executeLateralShift();
    
    bool executeWallFollowOnly();
    
    void resetState();
};

} // namespace sonar_slam

#endif