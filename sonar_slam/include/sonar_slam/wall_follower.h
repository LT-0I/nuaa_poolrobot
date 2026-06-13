#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>

namespace sonar_slam {

// 沿边行走状态机
enum WallFollowState {
    STATE_IDLE = 0,           // 空闲
    STATE_ROTATING,           // 旋转寻找墙面
    STATE_ALIGNING,           // 对齐墙面
    STATE_FOLLOWING,          // 沿边行走
    STATE_STOPPING,           // 停止等待
    STATE_TURNING,            // 转弯
    STATE_FRONT_WALL,    // 前方墙壁处理
    STATE_GAP_HANDLING,  // 凹入区域处理
    STATE_COMPLETED           // 完成
};

// 旋转子状态
enum RotateSubState {
    ROTATE_FULL_CIRCLE = 0,   // 完整旋转一圈
    ROTATE_TO_TARGET          // 旋转到目标角度
};

// 沿边子状态
enum FollowSubState {
    FOLLOW_STRAIGHT = 0,      // 直行
    FOLLOW_ADJUSTING          // 调整中
};

 
class ScopedAction {
public:
    explicit ScopedAction(bool& flag) : flag_(flag) {
        flag_ = true;
    }
    ~ScopedAction() {
        flag_ = false;
    }
private:
    bool& flag_;
};

class WallFollower {
public:
    WallFollower(ros::NodeHandle& nh);
    ~WallFollower();

    // 开始沿边行走
    bool start(double wall_distance = 0.3,    // 沿边距离(米)
               double speed = 0.15,           // 行走速度(米/秒)
               double segment_distance = 2.0, // 每段行走距离(米)
               double stop_duration = 45.0,   // 停止时间(秒)
               double turn_distance = 1.5,    // 检测到墙的转弯距离(米)
               double complete_distance = 3.0,
               double finish_distance= 1.0,
               double pump_speed = 0.5); // 完成距离(米)

    // 停止
    void stop();

    // 获取当前状态
    WallFollowState getState() const { return state_; }
    
    // 主循环更新函数
    void update();

private:
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    
    // 核心功能函数
    bool waitForOdom(float timeout = 5.0f);
    void updateCurrentPose();
    float calculateYawFromQuaternion(const geometry_msgs::Quaternion& q);
    float normalizeAngle(float angle);
    
    // 状态机核心函数
    void updateRotating();      // 旋转寻找墙面
    void updateAligning();      // 对齐墙面
    void updateFollowing();     // 沿边行走
    void updateStopping();      // 停止等待
    void updateTurning();       // 转弯
    
    void updateFrontWall();
    void updateGapHandling();
    // 沿边控制函数
    void computeWallErrors();   // 计算与墙面的误差
    void applyFollowControl();  // 应用沿边控制
    
    // 按键模拟控制
    void applyKeyboardCommand(char key);
    void stopMoving();
    void setLinearSpeed(float speed);
    void setAngularSpeed(float speed);
    
    // 辅助函数
    float getRightUltrasoundMin();  // 获取右侧超声波最小值
    bool checkFrontObstacle();       // 检查前方障碍物
    float calculateTurnAngle();      // 计算转弯角度
    bool checkCompletion();          // 检查是否完成
    void updateFrontAlign();

    bool  checkFrontObstacleStable();

   
    
    // ROS接口
    ros::NodeHandle& nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ultra_sub_;
    
    // 状态
    WallFollowState state_;
    RotateSubState rotate_sub_state_;
    FollowSubState follow_sub_state_;
    bool odom_received_;
    int front_align_step_;
 
    ros::Time last_decision_time_;
    std::string last_decision_;
    int decision_stable_count_;
    ros::Time last_state_update_;

    float aligned_yaw_;           // 对齐完成时的角度
    bool alignment_completed_;    // 是否已完成对齐
    ros::Time last_alignment_time_;

    // 搜索状态变量
    int search_state_;
    float turn_start_yaw_; // 转向起始角度
    ros::Time walk_start_time_;

     // 动作执行标志（防止状态机重入）
    bool executing_action_;
   
    int turn_direction_;  // 转弯方向: 1=左转, -1=右转

    float turn_target_yaw_;    // 转向目标角度
    bool turning_active_;  // 是否正在转向
 
    ros::Time turn_start_time_;
    float accumulated_yaw_; // 累积转角

    // 参数
    struct {
        float wall_distance;      // 沿边距离(米)
        float speed;              // 行走速度(米/秒)
        float segment_distance;   // 每段行走距离(米)
        float stop_duration;      // 停止时间(秒)
        float turn_distance;      // 转弯检测距离(米)
        float complete_distance;  // 完成距离(米)
        float finish_distance;
    } params_;
    
    // 沿边控制参数
    struct {
        float kp_distance;        // 距离PID比例
        float kp_angle;           // 角度PID比例
        float target_distance;    // 目标距离
        float current_distance;   // 当前距离
        float angle_error;        // 角度误差
        float distance_error;     // 距离误差
    } wall_control_;
    
    // 行走状态
    struct {
        float segment_start_x;    // 段起始X
        float segment_start_y;    // 段起始Y
        float segment_traveled;   // 已行走距离
        float total_traveled;     // 总行走距离
        ros::Time stop_start_time; // 停止开始时间
        float start_yaw;          // 起始偏航角
        geometry_msgs::Pose2D start_pose;  // 起始位姿
    } travel_;
    
    // 传感器数据
    struct {
        float ultra_right_front;  // 右侧前超声波(米) - data[2]
        float ultra_right_rear;   // 右侧后超声波(米) - data[3]
        float infra_front_left;   // 前方左红外(米) - data[1]
        float infra_front_right;  // 前方右红外(米) - data[0]
        ros::Time last_update;
    } sensors_;
        
    // 旋转寻找墙面数据
    struct {
        float min_distance;       // 最小距离
        float min_distance_yaw;   // 最小距离对应的角度
        float target_yaw;         // 目标角度
        float start_yaw;          // 起始角度
        bool found_wall;
        bool need_random_walk; 
    } wall_search_;

 
    // 里程计数据
    geometry_msgs::Pose2D current_pose_;
    nav_msgs::Odometry current_odom_;
    
    // 互斥锁
    std::mutex data_mutex_;
 
    ros::Subscriber twist_sub_;
    geometry_msgs::Twist current_twist_;

    // 在 wall_search_ 结构体后面添加
    float pump_speed_;           // 水泵速度 0.0-1.0
    float follow_start_yaw_;     // IMU纠偏起始角度
    bool follow_yaw_initialized_; // IMU纠偏是否初始化
    
 
    
    // 调试
    int debug_counter_;
};

} // namespace sonar_slam

#endif // WALL_FOLLOWER_H