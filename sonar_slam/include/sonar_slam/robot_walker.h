#ifndef SONAR_SLAM_ROBOT_WALKER_H
#define SONAR_SLAM_ROBOT_WALKER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <std_msgs/Int32MultiArray.h>
 
#include <fstream>
#include <vector>
#include <sys/stat.h>
#include <chrono>
#include <iomanip>
#include <deque>

namespace sonar_slam {

// 直接复用 MoveController 的成功键盘模拟机制
struct KeyboardSimulator {
    float linear_inc = 0.01f;      // 线速度增量
    float angular_inc = 0.3f;      // 角速度增量
    float max_linear = 0.5f;       // 最大线速度
    float max_angular = 0.5f;      // 最大角速度
    int idle_threshold = 5;        // 空闲阈值
    
    // 状态变量
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

class RobotWalker {
public:
    RobotWalker(ros::NodeHandle& nh);
    ~RobotWalker();
    
    bool walkStraight(float distance, float max_speed = 0.15f, float timeout = 30.0f);
    void stop();
    std::string getStatus() const;
 
    bool walkStraightWithWallAvoidanceAdvanced(float distance, float max_speed, float timeout);

    bool findPoolWall(float timeout = 60.0f);

    
    
private:
    // ROS接口
    ros::NodeHandle& nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer walk_service_;
    ros::Timer keyboard_timer_;  // 键盘模拟定时器（复用成功的20Hz机制）
    ros::Subscriber ultra_sub_;
    
    // 键盘模拟器（关键：复用MoveController的成功机制）
    KeyboardSimulator keyboard_sim_;
    std::mutex keyboard_mutex_;
    
    // 状态
    bool odom_received_;
    nav_msgs::Odometry current_odom_;
    geometry_msgs::Pose2D current_pose_;

    float current_pump_speed_;  // 当前水泵速度 (0-1)
    
    // 行走状态
    enum WalkState {
        STATE_IDLE,
        STATE_WALKING,
        STATE_ERROR
    };
    WalkState walk_state_;
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool walkServiceCallback(std_srvs::Trigger::Request& req, 
                            std_srvs::Trigger::Response& res);
    void keyboardTimerCallback(const ros::TimerEvent& event);  // 关键：20Hz定时器
    
    // 工具函数
    void updateCurrentPose();
    void applyKeyboardCommand(char key);  // 复用MoveController的键盘命令
    float calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    bool waitForOdom(float timeout);
    
    // 核心行走函数
    bool executeStraightWalk(float distance, float max_speed, float timeout);
    
    // 初始化键盘模拟器
    void initKeyboardSimulator();

    float normalizeAngle(float angle);
 
    bool alignToRightWallWithDistance(float target_distance);  // 右侧距离+平行度对齐
    void alignToRightWallOnly();
    // 成员变量
    bool pre_rotate_90_;
    bool enable_parallel_align_;
    float right_front_dist_;
    float right_rear_dist_;
    bool ultra_received_;
    float infra_front_left_;   // 左前红外距离（米）
    float infra_front_right_;  // 右前红外距离（米）
    
    bool disabling_timer_;
    // 回调声明
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

    // 新增智能沿边对齐函数
    bool smartAlignToWall(float target_distance, float timeout = 15.0f);
 

    // 沿边对齐状态机枚举
    enum AlignState {
        STATE_DETECT,           // 检测环境（判断是角落/边缘/空旷）
        STATE_ROTATE_TO_WALL,   // 旋转寻找池壁
        STATE_ALIGN_PARALLEL,   // 平行度对齐
        STATE_ADJUST_DISTANCE,  // 调整到目标距离
        STATE_TURN_TO_WALL,     // 转向池壁（用于角落场景）
        STATE_COMPLETE          // 完成
    };
    //日志数据
    std::ofstream walk_log_file_;
    std::string current_log_filename_;
    ros::Time last_log_time_;
    ros::Time walk_start_time_;
    
    // 日志数据结构
    struct WalkLogData {
        ros::Time timestamp;
        float right_front;      // 右侧前超声波
        float right_rear;       // 右侧后超声波
        float infra_left;       // 前方左红外
        float infra_right;      // 前方右红外
        float pose_x;           // 里程计X
        float pose_y;           // 里程计Y
        float pose_theta;       // 里程计角度(弧度)
        float target_distance;  // 目标行走距离
        float current_distance; // 当前已走距离（基于里程计）
    };
    std::vector<WalkLogData> log_buffer_;
    
    // 日志方法
    void initWalkLog(const std::string& log_type, float target_distance);
    void recordWalkData(float current_distance, float target_distance);
    void saveWalkLog(bool success);
    
    std::string getWalkLogDirectory();

    float movingAverageFilter(float new_value, std::deque<float>& buffer, int window_size = 3);


};

} // namespace sonar_slam

#endif // SONAR_SLAM_ROBOT_WALKER_H