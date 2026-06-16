#ifndef SONAR_SLAM_MOVE_CONTROLLER_H  
#define SONAR_SLAM_MOVE_CONTROLLER_H  

#pragma once

#include <ros/ros.h>
#include "sonar_slam/MoveStraight.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
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
    
    // ========== 传感器回调 ==========
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void imuHeadingCallback(const std_msgs::Float32::ConstPtr& msg);   // /imu_heading 融合航向(度)
    void pumpEnableCallback(const std_msgs::Bool::ConstPtr& msg);      // /move_controller/pump_enable 运行级水泵闩锁

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
    ros::Subscriber imu_heading_sub_;   // /imu_heading 融合航向(度)，串级直行内环用
    ros::Subscriber pump_enable_sub_;   // /move_controller/pump_enable 运行级水泵闩锁(mapper 整段持泵)
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
    float config_wall_stale_sec_;  // 右侧超声陈旧看门狗(s)，超此时长无新鲜帧→退 odom 兜底；须≥右侧刷新周期，随驱动 ultra_poll_period 调
    float user_config_pump_speed_;  // 水泵速度(ROS 参数 pump_speed, 默认 0.4; 夹[0,1])
    // 运行级水泵闩锁: mapper 在整段沿边任务前置 true(/move_controller/pump_enable)、结束/中止置 false。
    // true=跨段持泵(空闲定时器/段末停车/扫描停顿都不撤泵); false=回落单段策略(原行为, 单测无 mapper 时即此)。
    // 单回调队列线程: 仅 pumpEnableCallback 写、定时器/主循环读 → 无锁(见下方 AsyncSpinner 铁律)。
    bool run_pump_latched_ = false;
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
        // 串级直行调参字段
        float imu_yaw_deg;
        float yaw_base_deg;
        float e_head_deg;
        float wall_d;
        float e_par;
        float ctrl_C;
        float ctrl_u;
        int   ctrl_mode;   // 0=无IMU(odom兜底) 1=IMU+超声 2=IMU守航向(丢墙)
    };
    std::vector<WalkLogData> log_buffer_;

    // 右侧超声"新鲜度"跟踪。超声实测 MCU 推流 ~10Hz/通道、/ultra_data 话题 ~20Hz 转发(ZOH)。
    // seq=拐角 stop-and-settle 用(只在原始值真变化时推进, 避免对 ZOH 重复值重复精修);
    // stamp=陈旧看门狗用, 每次消息到达即更新、与值变化【解耦】(平直墙真实距离恒定时不会被误判 stale)。
    uint32_t ultra_right_seq_;     // 右侧任一通道原始值变化时自增(拐角精修用)
    ros::Time ultra_right_stamp_;  // 右侧超声最近一次消息到达时间(陈旧看门狗用; 每帧更新, 与值变化解耦)
    int last_ultra_raw_rf_;        // 上次原始 data[2]
    int last_ultra_raw_rr_;        // 上次原始 data[3]

    // ⚠铁律: 本节点单线程回调队列(main 用 ros::Rate+spinOnce, 阻塞 move 循环内用嵌套 spinOnce)。
    //   hs_yaw_*/hs_new_sample_/wall_sensors_/ultra_* 等跨"回调↔主循环"共享态据此免锁(或仅 sensor_mutex_ 快照)。
    //   严禁改用 AsyncSpinner/MultiThreadedSpinner, 否则这些无锁 double/float 共享会撕裂、须全量补锁。
    // ===== 串级直行: IMU 融合航向(/imu_heading) 内环 PD + 超声距墙外环 =====
    // /imu_heading 状态(与主循环 spinOnce 同线程、回调亦在该线程执行, 无需加锁)
    bool      hs_have_yaw_   = false;
    double    hs_yaw_rad_    = 0.0;   // 最新融合航向(rad)
    ros::Time hs_yaw_stamp_;          // 最新一帧到达时间
    bool      hs_new_sample_ = false; // 有新帧待用于微分
    // 内环 PD 状态(微分=样本驱动一阶低通, 积分=条件抗饱和)
    double    hs_d_filt_ = 0.0;
    double    hs_e_prev_ = 0.0;
    ros::Time hs_t_prev_sample_;
    double    hs_I_      = 0.0;
    // 串级参数(ROS 全局参数: 启动前 rosparam set /xxx; 默认=0614实测好值)
    double config_heading_kp_          = 1.0;
    double config_heading_kd_          = 1.0;
    double config_heading_ki_          = 0.0;
    double config_heading_i_clamp_     = 0.05;
    double config_heading_omega_limit_ = 0.30;  // 总角速度命令限幅(cmd; ×6.4 才是 rad/s); Q3 实测直行值 0.30
    double config_heading_tau_d_       = 0.2;   // 微分低通时间常数(s)
    double config_heading_yaw_timeout_ = 0.5;   // /imu_heading 陈旧阈值(s), 超则退 odom 兜底
    int    config_heading_sign_        = 1;     // 航向误差极性(0614 实测 +1=负反馈)
    double config_wall_k_dist_         = 0.20;  // 距墙误差→航向偏置 (rad/m); 超声实测 10Hz(非旧推断0.83Hz)→比旧0.03激进, off_max 兜底
    double config_wall_k_par_          = 0.60;  // 平行误差→航向偏置 (rad/m)
    double config_wall_off_max_        = 0.10;  // 外环航向偏置限幅(rad≈5.7°); 距墙环安全上界, 限制最大转向激进度
    // ⚠ 稳定性依赖右前/右后纵向基距 L: 特征根过阻尼需 L<1/k_par≈1.67m(k_par=0.6)。Orange Pi 小车 L<1m 必满足、安全;
    //   若换大车/调大 k_par, 须复核 L。另: off_max=0.10 使 |d-target|>0.5m 时外环按限幅 slew(开环段), 仿真"44s收敛"假定起步已近目标。
    float  config_speed_max_           = 0.30f; // 前进速度安全上限(防履带打滑临界~0.27)
    float  config_softstart_sec_       = 1.5f;  // 缓启动斜坡(s): 起步/拐角后速度【与外环C权限】在此时长内线性 0→目标
    // 外环新墙/平行一致性闸门: |右后-右前|≥此 → 判畸变(拐角凹角/一传感器照旧墙一照新墙)→弃距墙外环、纯IMU守航向。
    // 兼治拐角畸变区误锁(评审 BLOCKER)与物理不可能的平行读数。
    float  config_wall_consistency_m_  = 0.5f;

    // ===== P1 隔离 + P2 拐角(停-转-融合航向闭环)参数 =====
    // 全局 nh_.param, 构造期读一次(终端2 裸 rosrun 无 launch → 默认即操作值, 仅【启动前】rosparam set 生效)。
    bool   config_enable_corner_         = true;   // false=跳过前墙拐角块走纯串级直行(直行段单独整定 + 作回归基线)
    float  config_corner_front_trigger_m_= 1.5f;   // 前超声(data0前左/data1前右)min<此且>0.05m 触发拐角; 保 1.5 给停车物理裕度
    int    config_corner_front_confirm_  = 3;      // 连续确认帧(>0.05m 近场地板拒噪, 上限内全有效)
    float  config_corner_settle_sec_     = 1.0f;   // 停稳取入航向时长(持续 20Hz 发零运动+泵, 喂看门狗)
    float  config_corner_turn_deg_       = 90.0f;  // 闭环左转目标(融合航向转过幅值)
    float  config_corner_turn_omega_     = 0.2f;   // 粗转角速度(cmd; 左转 angular>0; ×6.4≈1.28rad/s)
    float  config_corner_turn_omega_fine_= 0.10f;  // 近目标降速(⚠水下若低于电机起转阈会停在~88°; 首跑量, 必要时0.12~0.15)
    float  config_corner_slow_deg_       = 15.0f;  // 距目标此角内降到 fine
    float  config_corner_turn_timeout_   = 8.0f;   // 转向硬超时(s)
    float  config_corner_turn_cap_deg_   = 130.0f; // odom 角兜底上限(默认 turn+40°宽裕度): 仅兜 IMU 大故障/防过转, 不与 IMU 到位判据争紧裕度(评审 HIGH)

    // 串级调参快照(主循环每拍更新, recordWalkData 1Hz 采样进 CSV)
    float dbg_imu_yaw_deg_  = 0.0f;
    float dbg_yaw_base_deg_ = 0.0f;
    float dbg_e_head_deg_   = 0.0f;
    float dbg_wall_d_       = -1.0f;
    float dbg_e_par_        = 0.0f;
    float dbg_C_            = 0.0f;
    float dbg_u_            = 0.0f;
    int   dbg_mode_         = 0;

};
} // namespace sonar_slam

#endif // SONAR_SLAM_MOVE_CONTROLLER_H