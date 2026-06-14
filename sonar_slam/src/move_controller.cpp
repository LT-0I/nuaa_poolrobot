#include "sonar_slam/move_controller.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <thread>
#include <locale>
#include <deque> 

namespace sonar_slam {

const float PI_M = 3.14159265358979323846f;
const float DEG_TO_RAD = PI_M / 180.0f;
const float RAD_TO_DEG = 180.0f / PI_M;

// ==================== 构造函数 ====================
MoveController::MoveController() : 
    odom_received_(false),
    move_state_(0),
    disabling_timer_(false),
    linear_tolerance_(0.05f),
    config_speed_(0.25f),
    config_pump_speed_(0.0f),           
    user_config_pump_speed_(0.0f),      
    config_wall_align_(2.0f),
    config_wall_stale_sec_(1.5f),
    ultra_right_seq_(0),
    last_ultra_raw_rf_(-1),
    last_ultra_raw_rr_(-1)
{
    ROS_INFO("=== MoveController 初始化开始 ===");
    
    // 初始化传感器数据
    wall_sensors_.ultra_right_front = 999.0f;
    wall_sensors_.ultra_right_rear = 999.0f;
    wall_sensors_.infra_front_left = 999.0f;
    wall_sensors_.infra_front_right = 999.0f;
    
    // 发布到 /cmd_vel
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ROS_INFO("发布器: /cmd_vel");
    
    // 订阅 /wheel_odom
    odom_sub_ = nh_.subscribe("/wheel_odom", 1000, &MoveController::odomCallback, this);
    ROS_INFO("订阅器: /wheel_odom");
    
    // 订阅超声波和红外数据
    ultra_sub_ = nh_.subscribe("/ultra_data", 100, &MoveController::ultraCallback, this);
    ROS_INFO("订阅器: /ultra_data");

    // 订阅 IMU 融合航向 /imu_heading(度) —— 串级直行内环用(由 mars_serial_server 驱动发布;
    // 老驱动无此话题时控制器自动退化为 odom 兜底, 不会失效)
    imu_heading_sub_ = nh_.subscribe("/imu_heading", 50, &MoveController::imuHeadingCallback, this);
    ROS_INFO("订阅器: /imu_heading");

    // 创建服务服务器
    move_straight_srv_ = nh_.advertiseService(
        "/move_controller/move_straight", 
        &MoveController::moveStraightCallback, this);
    ROS_INFO("服务服务器: /move_controller/move_straight");
    
    // 初始化键盘模拟器
    initKeyboardSimulator();
    
    // 启动键盘模拟定时器（20Hz）
    keyboard_timer_ = nh_.createTimer(
        ros::Duration(0.05),
        &MoveController::keyboardTimerCallback,
        this
    );
    
    // 读取水泵速度参数
    // 默认 0.25(0614 实测能走直的速度; 高于 ~0.27 履带打滑临界会侧滑); 实际再夹 speed_max
    nh_.param("speed", config_speed_, 0.25f);
    float temp_pump = 0.0f;
    nh_.param("pump_speed", temp_pump, 0.1f);
    user_config_pump_speed_ = 0.4f;
    nh_.param("wall_align", config_wall_align_, 2.0f);  // 目标距墙距离(m)，沿墙距离环用
    // 右侧超声陈旧看门狗(s)：默认1.5(配 D-41 驱动右侧~0.3s/路、≈5×周期)；夹在[0.5,5.0]防误设。
    // 下限0.5≥3×最快右侧刷新周期(驱动 ultra_poll_period 最小0.05→右0.15s)，留抖动裕度。
    // ⚠ 须≥右侧实际刷新周期：若驱动 ultra_poll_period 调大或回退旧1.2s/路，须同步上调本值，否则超声环↔odom兜底抖动。
    nh_.param("wall_stale_sec", config_wall_stale_sec_, 1.5f);
    if (config_wall_stale_sec_ < 0.5f) config_wall_stale_sec_ = 0.5f;
    if (config_wall_stale_sec_ > 5.0f) config_wall_stale_sec_ = 5.0f;
    config_pump_speed_ = 0.0f;

    // ===== 串级直行参数(ROS 全局参数: 启动前 rosparam set /xxx; 默认=0614实测好值) =====
    // 内环 = 0614 实测能走直的 IMU 航向 PD; 外环 = doc 分支超声距墙/平行环(折成航向偏置)
    nh_.param("heading_kp",          config_heading_kp_,          1.0);
    nh_.param("heading_kd",          config_heading_kd_,          1.0);
    nh_.param("heading_ki",          config_heading_ki_,          0.0);
    nh_.param("heading_i_clamp",     config_heading_i_clamp_,     0.05);
    nh_.param("heading_omega_limit", config_heading_omega_limit_, 0.15);
    nh_.param("heading_tau_d",       config_heading_tau_d_,       0.2);
    nh_.param("heading_yaw_timeout", config_heading_yaw_timeout_, 0.5);
    nh_.param("heading_sign",        config_heading_sign_,        1);
    config_heading_sign_ = (config_heading_sign_ >= 0) ? 1 : -1;
    nh_.param("wall_k_dist",         config_wall_k_dist_,         0.03);
    nh_.param("wall_k_par",          config_wall_k_par_,          0.30);
    nh_.param("wall_off_max",        config_wall_off_max_,        0.08);
    nh_.param("speed_max",           config_speed_max_,           0.30f);
    nh_.param("softstart_sec",       config_softstart_sec_,       1.5f);
    if (config_softstart_sec_ < 0.0f) config_softstart_sec_ = 0.0f;
    if (config_softstart_sec_ > 10.0f) config_softstart_sec_ = 10.0f;
    // 夹限防误设(总角速度 ≤0.3 cmd; 外环偏置 ≤0.3; 速度上限 ≤0.35)
    if (config_heading_omega_limit_ < 0.0) config_heading_omega_limit_ = 0.0;
    if (config_heading_omega_limit_ > 0.3) config_heading_omega_limit_ = 0.3;
    if (config_heading_tau_d_       < 0.02) config_heading_tau_d_      = 0.02;
    if (config_wall_off_max_        < 0.0) config_wall_off_max_        = 0.0;
    if (config_wall_off_max_        > 0.3) config_wall_off_max_        = 0.3;
    if (config_speed_max_           < 0.05f) config_speed_max_         = 0.05f;
    if (config_speed_max_           > 0.35f) config_speed_max_         = 0.35f;

    // 版本闸门: ASCII 横幅, 操作单 grep 自判是否在跑新二进制(防跑旧编译)
    ROS_WARN("MOVE_CTRL_VERSION = wallfollow-imu-cascade-20260614 (IMU-heading inner + ultra-wall outer + softstart)");
    ROS_INFO("  cascade params: heading_kp=%.2f kd=%.2f ki=%.2f omega_limit=%.2f sign=%d",
             config_heading_kp_, config_heading_kd_, config_heading_ki_, config_heading_omega_limit_, config_heading_sign_);
    ROS_INFO("  wall outer: k_dist=%.3f k_par=%.3f off_max=%.3f | speed_max=%.2f softstart=%.2f",
             config_wall_k_dist_, config_wall_k_par_, config_wall_off_max_, config_speed_max_, config_softstart_sec_);
    ROS_INFO("========================================");
    ROS_INFO("MoveController 配置参数:");
    ROS_INFO("  speed = %.2f m/s", config_speed_);
    ROS_INFO("  user_config_pump_speed = %.2f (用户设置)", user_config_pump_speed_);
    ROS_INFO("  config_pump_speed = %.2f (实际输出，行走时才生效)", config_pump_speed_);
    ROS_INFO("  temp_pump = %.2f (用户实际设置)", temp_pump);
    ROS_INFO("  wall_align = %.2f m", config_wall_align_);
    ROS_INFO("  wall_stale_sec = %.2f s (right ultra staleness watchdog)", config_wall_stale_sec_);
    ROS_INFO("========================================");
 
    
 
    ROS_INFO("========================================");
    ROS_INFO("MoveController 初始化完成");
    ROS_INFO("========================================");
}


void MoveController::stop() {
    stopMoving();
}

std::string MoveController::getStatus() const {
    switch (move_state_) {
        case 0: return "空闲";
        case 1: return "行走中";
        case 2: return "错误";
        default: return "未知";
    }
}

// ==================== 对外接口 ====================
bool MoveController::moveStraight(float distance, float timeout) {
    ROS_INFO("moveStraight 调用: distance=%.3f, timeout=%.1f, speed=%.2f", 
             distance, timeout, config_speed_);
    // 使用配置的速度调用高级行走函数
    return walkStraightWithWallAvoidanceAdvanced(distance, config_speed_, timeout);
}

// ==================== 服务回调 ====================
bool MoveController::moveStraightCallback(sonar_slam::MoveStraight::Request& req,
                                           sonar_slam::MoveStraight::Response& res) {
    ROS_INFO("收到移动服务请求: distance=%.3f, timeout=%.1f", 
             req.distance, req.timeout);
    
    float timeout = (req.timeout > 0) ? req.timeout : 60.0f;
    bool success = moveStraight(req.distance, timeout);
    
    res.success = success;
    res.message = success ? "移动成功" : "移动失败";
    res.actual_distance = success ? fabs(req.distance) : 0.0f;
    res.final_error = 0.0f;
    
    return true;
}

// ==================== 核心功能：高级直线行走（带避让）====================

bool MoveController::walkStraightWithWallAvoidanceAdvanced(float distance, float max_speed, float timeout) {
    if (move_state_ != 0) {
        ROS_ERROR("机器人正在行走中，无法开始新的行走");
        return false;
    }
    
    // ========== 初始化行走日志 ==========
    initWalkLog("move_advanced_walk", fabs(distance));

    ROS_ERROR("========================================");
    ROS_ERROR("高级直线行走开始（带行走中池壁检测与避让）");
    ROS_ERROR("  距离: %.3f米", distance);
    ROS_ERROR("  速度: %.3f米/秒", max_speed);
    ROS_ERROR("  超时: %.1f秒", timeout);
    ROS_ERROR("  水泵速度: %.1f", user_config_pump_speed_);
    ROS_ERROR("========================================");
    
    if (user_config_pump_speed_ > 0.01f) {
        config_pump_speed_ = user_config_pump_speed_;
        ROS_INFO("✅ 行走开始，激活水泵，速度: %.0f%%", config_pump_speed_ * 100);
    }
    
    move_state_ = 1;  // 行走中
    disabling_timer_ = true;  // 禁用键盘定时器
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;

    float total_target = fabs(distance);
    char move_direction = (distance >= 0) ? 'w' : 'x';
    // 速度: 用传入 max_speed(=ROS参数 speed, 默认0.25), 夹到安全上限 config_speed_max_(默认0.30, 防履带打滑)
    // 修正原 doc/main 把 cmd_speed 写死 0.35(忽略参数, 且 >打滑临界~0.27 → 侧滑): 现可调且限幅
    float cmd_speed = std::min(std::max(max_speed, 0.0f), config_speed_max_);
    
    // 清除键盘模拟器状态
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.current_vx = 0.0f;
        keyboard_sim_.current_vy = 0.0f;
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.idle_counter = 0;
        keyboard_sim_.key_pressed = false;
    }
    
    // 等待odom数据
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        disabling_timer_ = false;
        move_state_ = 2;
        return false;
    }
    
    // 强制刷新里程计
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    updateCurrentPose();
    
    // 记录起点（仅用于角度参考）
    geometry_msgs::Pose2D start_pose = current_pose_;
    float start_yaw = current_pose_.theta;
    
    // ========== 累积路程变量 ==========
    float total_accumulated_distance = 0.0f;
    geometry_msgs::Pose2D last_pose = current_pose_;
    
    ROS_ERROR("起始状态: 位置(%.2f,%.2f), 角度=%.1f°", 
              start_pose.x, start_pose.y, start_yaw * RAD_TO_DEG);
    
    float pump_speed = config_pump_speed_;
    bool pump_on = (pump_speed > 0.01f);
    if (pump_on) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = 0.0f;   // 缓启动: 起泵阶段不前冲, 前进速度由主循环斜坡渐起
        pump_cmd.linear.y = pump_speed;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();
    }
    
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool wall_avoiding = false;
    int front_wall_detect_count = 0;
    bool pump_closed_early = false;
    
    // 角度累积变量
    float total_yaw_change = 0.0f;
    float last_theta = start_yaw;
    
    // 纠偏相关变量
    ros::Time correction_start_time = ros::Time::now();
    bool correction_enabled = false;
    ros::Time last_correction_time = ros::Time::now();
    float current_angular_correction = 0.0f;

    // 串级直行: 锁定 IMU 航向基准 yaw_base(=当前/imu_heading; 段首与每次拐角对齐后重锁) + 复位内环 PD
    // 段首先等 IMU 首帧再锁(最多1s), 防首帧晚到把基准锁成绝对航向 0.0 → 大初始误差饱和打舵(评审 HIGH)。
    // IMU 在跑时首帧 <50ms 即到、几乎不停留; 等不到(老驱动无/imu_heading)则降级, 主循环内首个新鲜帧到达时"懒锁"。
    for (int i = 0; i < 20 && ros::ok() && !hs_have_yaw_; i++) { ros::spinOnce(); ros::Duration(0.05).sleep(); }
    bool yaw_base_locked = (hs_have_yaw_ &&
                            (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);
    if (!yaw_base_locked) ROS_WARN("沿墙(串级): 段首未等到 /imu_heading 新鲜帧 → 暂退 odom 兜底, 待首帧到再懒锁航向基准");
    double yaw_base = yaw_base_locked ? hs_yaw_rad_ : 0.0;
    bool imu_fresh_prev = yaw_base_locked;   // 跟踪 IMU 新鲜态翻转, 掉线恢复时复位微分锚防尖峰
    hs_I_ = 0.0; hs_d_filt_ = 0.0; hs_e_prev_ = 0.0;
    hs_t_prev_sample_ = yaw_base_locked ? hs_yaw_stamp_ : ros::Time::now();
    hs_new_sample_ = false;
    // 缓启动斜坡起点(起步; 每次拐角恢复后重置)
    ros::Time ramp_start_time = ros::Time::now();

    // ========== 设置目标速度 ==========
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        if (move_direction == 'w') {
            keyboard_sim_.target_vx = cmd_speed;
            keyboard_sim_.current_vx = cmd_speed;
        } else {
            keyboard_sim_.target_vx = -cmd_speed;
            keyboard_sim_.current_vx = -cmd_speed;
        }
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.idle_counter = 0;
        keyboard_sim_.key_pressed = true;
    }
    
    // ========== 主循环 ==========
    while (ros::ok() && move_state_ == 1) {
        updateCurrentPose();
        
        // 超时检查
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("行走超时");
            break;
        }
        
        // ========== 使用累积路程 ==========
        float dx = current_pose_.x - last_pose.x;
        float dy = current_pose_.y - last_pose.y;
        float delta = sqrt(dx*dx + dy*dy);

        if (delta > 0.001f && delta < 0.5f) {
            total_accumulated_distance += delta;
        }
        
        last_pose = current_pose_;
        float remaining = total_target - total_accumulated_distance;
        
        // 累积角度变化（用于纠偏）
        float delta_yaw = normalizeAngle(current_pose_.theta - last_theta);
        total_yaw_change += delta_yaw;
        last_theta = current_pose_.theta;

        recordWalkData(total_accumulated_distance, total_target);
        
        // ==================== 池壁检测 ====================
        if (!wall_avoiding && move_direction == 'w') {
            // 前方超声波检测（使用修正后的传感器数据）
            float front_left_dist = wall_sensors_.infra_front_left;
            float front_right_dist = wall_sensors_.infra_front_right;
            float front_dist = std::min(front_left_dist, front_right_dist);
            bool has_front_wall = (front_dist > 0.3f && front_dist < 1.5f);
            
            if (has_front_wall) {
                front_wall_detect_count++;
                if (front_wall_detect_count >= 3) {
                    ROS_WARN("行走中检测到前方池壁(%.2f米)，执行避让！", front_dist);
                    wall_avoiding = true;
                    
                    // ----- 1. 停止 -----
                    stop();
                    ros::Duration(0.3).sleep();
                    ros::spinOnce();
                    
                    // 等待1.5秒后开始旋转
                    ROS_INFO("等待1.5秒后开始旋转...");
                    ros::Duration(1.5).sleep();
                    ros::spinOnce();
                    updateCurrentPose();
                    
                    // ----- 2. 左转至右侧平行于池壁 -----
                    ROS_INFO("左转至右侧平行于池壁");
                    float turn_start = current_pose_.theta;
                    ros::Time turn_start_time = ros::Time::now();
                    // 拐角左转：odom 粗转(快、相对角准) + "新鲜"右侧超声平行精修(stop-and-settle)
                    // 修复②：旧码在 20Hz 循环反复读 0.83Hz 旧值→提前停→没转够→机头朝墙
                    const float COARSE_DEG = 80.0f;          // 开环粗转到此角度(odom)
                    const float CAP_DEG = 95.0f;             // 角度兜底上限
                    const float TURN_PARALLEL_TOL = 0.025f;  // 平行容差(m)
                    const int   PARALLEL_CONFIRM_NEEDED = 2; // 连续N个"新鲜"帧确认平行
                    const float FINE_ANGULAR = 0.10f;        // 精修脉冲角速度(cmd,实际×6.4)
                    const float FINE_PULSE_SEC = 0.15f;      // 单次脉冲时长(限制单次旋转~5°)
                    int parallel_confirm = 0;
                    uint32_t last_ultra_seq;
                    { std::lock_guard<std::mutex> lk(sensor_mutex_); last_ultra_seq = ultra_right_seq_; }
                    bool phaseB_entered = false;       // 进入精修阶段标志
                    ros::Time last_fresh_time;         // 阶段B最近一次新帧时间(无回波兜底用)

                    while (ros::ok()) {
                        ros::spinOnce();
                        updateCurrentPose();

                        float yaw_change = current_pose_.theta - turn_start;
                        if (yaw_change > M_PI) yaw_change -= 2 * M_PI;
                        if (yaw_change < -M_PI) yaw_change += 2 * M_PI;
                        float abs_yaw = fabs(yaw_change);

                        if ((ros::Time::now() - turn_start_time).toSec() > 8.0f) {
                            stop();
                            ROS_WARN("旋转超时，强制停止 (转过%.1f°)", abs_yaw * RAD_TO_DEG);
                            break;
                        }

                        // 阶段A：开环粗转到 COARSE_DEG(odom 相对角，快)
                        if (abs_yaw < COARSE_DEG * DEG_TO_RAD) {
                            geometry_msgs::Twist cmd;
                            cmd.linear.x = 0.0f;
                            cmd.linear.y = pump_speed;
                            cmd.angular.z = 0.25f;
                            cmd_vel_pub_.publish(cmd);
                            ros::Duration(0.05).sleep();
                            continue;
                        }

                        // 角度兜底：精修拿不到有效回波时防空转
                        if (abs_yaw > CAP_DEG * DEG_TO_RAD) {
                            stop();
                            ROS_WARN("兜底：累积转到%.0f°停止", abs_yaw * RAD_TO_DEG);
                            break;
                        }

                        // 进入阶段B：标记起点(用于"长时间无新帧"兜底计时)
                        if (!phaseB_entered) { phaseB_entered = true; last_fresh_time = ros::Time::now(); }

                        // 阶段B：只用"新鲜"右侧超声精修(seq 未推进则原地保持等新帧)
                        uint32_t seq; float front, rear;
                        {
                            std::lock_guard<std::mutex> lk(sensor_mutex_);
                            seq = ultra_right_seq_;
                            front = wall_sensors_.ultra_right_front;
                            rear  = wall_sensors_.ultra_right_rear;
                        }
                        if (seq == last_ultra_seq) {
                            // 长时间无新帧(超声疑似无回波)：慢速开环补转，由 CAP_DEG 收口退出，避免空等到8s
                            if ((ros::Time::now() - last_fresh_time).toSec() > 2.5f) {
                                geometry_msgs::Twist cmd;
                                cmd.linear.x = 0.0f;
                                cmd.linear.y = pump_speed;
                                cmd.angular.z = 0.12f;
                                cmd_vel_pub_.publish(cmd);
                                ros::Duration(0.05).sleep();
                                continue;
                            }
                            geometry_msgs::Twist hold;
                            hold.linear.x = 0.0f;
                            hold.linear.y = pump_speed;
                            hold.angular.z = 0.0f;
                            cmd_vel_pub_.publish(hold);
                            ros::Duration(0.05).sleep();
                            continue;
                        }
                        last_ultra_seq = seq;  // 收到新帧
                        last_fresh_time = ros::Time::now();

                        bool data_valid = (front > 0.05f && front < 5.0f &&
                                           rear > 0.05f && rear < 5.0f);
                        if (!data_valid) {
                            // 新帧但无回波：补一点开环转，交给角度兜底收口
                            parallel_confirm = 0;
                            geometry_msgs::Twist cmd;
                            cmd.linear.x = 0.0f;
                            cmd.linear.y = pump_speed;
                            cmd.angular.z = FINE_ANGULAR;
                            ros::Time t0 = ros::Time::now();
                            while ((ros::Time::now() - t0).toSec() < FINE_PULSE_SEC && ros::ok()) {
                                cmd_vel_pub_.publish(cmd);
                                ros::spinOnce();
                                ros::Duration(0.05).sleep();
                            }
                            stop();
                            continue;
                        }

                        float parallel_error = rear - front;  // >0:机头朝右墙(欠转)
                        ROS_INFO("旋转精修(新鲜帧): 前=%.2f 后=%.2f 误差=%.3f 角度=%.1f°",
                                 front, rear, parallel_error, abs_yaw * RAD_TO_DEG);

                        if (fabs(parallel_error) < TURN_PARALLEL_TOL) {
                            parallel_confirm++;
                            if (parallel_confirm >= PARALLEL_CONFIRM_NEEDED) {
                                stop();
                                ROS_INFO("平行达标(连续%d帧新鲜确认)，停止旋转 (转过%.1f°)",
                                         parallel_confirm, abs_yaw * RAD_TO_DEG);
                                break;
                            }
                            geometry_msgs::Twist hold;
                            hold.linear.x = 0.0f;
                            hold.linear.y = pump_speed;
                            hold.angular.z = 0.0f;
                            cmd_vel_pub_.publish(hold);
                            ros::Duration(0.05).sleep();
                            continue;
                        }

                        // 未平行：朝减小误差方向发限时脉冲，再回到"停-等新帧"
                        parallel_confirm = 0;
                        float w = (parallel_error > 0) ? FINE_ANGULAR : -FINE_ANGULAR;
                        geometry_msgs::Twist cmd;
                        cmd.linear.x = 0.0f;
                        cmd.linear.y = pump_speed;
                        cmd.angular.z = w;
                        ros::Time t0 = ros::Time::now();
                        while ((ros::Time::now() - t0).toSec() < FINE_PULSE_SEC && ros::ok()) {
                            cmd_vel_pub_.publish(cmd);
                            ros::spinOnce();
                            ros::Duration(0.05).sleep();
                        }
                        stop();
                    }
                    
                    ros::Duration(0.3).sleep();
                    
                    // ----- 3. 平行度对齐 -----
                    ROS_INFO("右侧平行度对齐");
                    alignToRightWallOnly();
                    ros::Duration(0.3).sleep();
                    
                    // ----- 4. 重置状态 -----
                    total_yaw_change = 0.0f;
                    last_theta = current_pose_.theta;
                    start_time = ros::Time::now();
                    wall_avoiding = false;
                    front_wall_detect_count = 0;
                    correction_enabled = false;
                    correction_start_time = ros::Time::now();
                    current_angular_correction = 0.0f;
                    // 串级: 拐角对齐后(已平行于新墙)重锁 IMU 航向基准 + 复位内环 PD + 重置缓启动斜坡
                    // 重锁须校 IMU 新鲜度, 否则可能锁到陈旧航向(评审 MEDIUM)
                    bool f_relock = (hs_have_yaw_ &&
                                     (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);
                    if (f_relock) { yaw_base = hs_yaw_rad_; yaw_base_locked = true; }
                    else { ROS_WARN("拐角重锁: IMU 不新鲜, 沿用旧 yaw_base(或退化 odom 兜底)"); }
                    imu_fresh_prev = f_relock;
                    hs_I_ = 0.0; hs_d_filt_ = 0.0; hs_e_prev_ = 0.0;
                    hs_t_prev_sample_ = f_relock ? hs_yaw_stamp_ : ros::Time::now();
                    hs_new_sample_ = false;
                    ramp_start_time = ros::Time::now();

                    // 恢复前进速度
                    {
                        std::lock_guard<std::mutex> lock(keyboard_mutex_);
                        if (move_direction == 'w') {
                            keyboard_sim_.target_vx = cmd_speed;
                            keyboard_sim_.current_vx = cmd_speed;
                        }
                        keyboard_sim_.target_angular = 0.0f;
                        keyboard_sim_.idle_counter = 0;
                        keyboard_sim_.key_pressed = true;
                    }
                    
                    ROS_INFO("避让完成，已走: %.2f米, 剩余: %.2f米, 当前角度: %.1f°", 
                            total_accumulated_distance, remaining, current_pose_.theta * RAD_TO_DEG);
                    continue;
                }
            } else {
                front_wall_detect_count = 0;
            }
        }
        
        // ==================== 到达目标 ====================
        if (remaining < 0.05f) {
            stop();
            success = true;
            ROS_ERROR("✅ 行走完成！总路程: %.3f米", total_accumulated_distance);
            break;
        }
        
        // ==================== 速度: 缓启动斜坡(起步/拐角后) ∩ 到点减速区 ====================
        float current_speed = cmd_speed;
        // 缓启动: 起步与每次拐角恢复后, 速度在 softstart_sec 内线性 0→cmd_speed(消除起步瞬跳引起的侧偏)
        if (config_softstart_sec_ > 0.01f) {
            float ramp_t = (ros::Time::now() - ramp_start_time).toSec();
            float ramp = std::min(1.0f, std::max(0.0f, ramp_t / config_softstart_sec_));
            current_speed *= ramp;
        }
        // 到点减速区(取更小者)
        float slow_down_dist = 0.4f;
        if (remaining < slow_down_dist && remaining > 0) {
            current_speed = std::min(current_speed, cmd_speed * std::max(0.3f, remaining / slow_down_dist));
        }

        // ==================== 提前关泵 ====================
        if (pump_on && !pump_closed_early && remaining < 0.3f && remaining > 0) {
            pump_on = false;
            pump_closed_early = true;
            config_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地，水泵已关闭");
        }
        
        // ==================== 沿墙纠偏：串级(IMU航向内环 + 超声距墙外环) ====================
        // 内环: /imu_heading(融合航向, 0614实测好的直行PD)守住航向基准 yaw_base(段首/拐角对齐后锁定)。
        // 外环: 右侧超声"新鲜且有效"时, 把 距墙误差e_dist + 平行误差e_par 折成有界航向偏置 C
        //       (同 doc 分支号: 太近/朝墙→正→左转远离), 叠到航向误差上; 丢墙时 C=0 → 纯 IMU 守 yaw_base(直行)。
        // 退化: /imu_heading 不可用/陈旧(老驱动或掉线) → 回退 doc 分支原逻辑(超声直接出角速度 + odom兜底)。
        // 增益为 cmd 单位, 实际角速度 = cmd ×6.4 (见驱动 hardware_driver.cpp); 现场按 CSV(e_head/d/e_par/u) 整定。
        float relative_theta = total_yaw_change;  // odom 兜底/日志用

        if (!correction_enabled && (ros::Time::now() - correction_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动沿墙纠偏(串级)");
        }

        if (correction_enabled && !wall_avoiding) {
            // 快照右侧超声(加锁) + 新鲜度; IMU 航向同线程读, 无需锁
            // 陈旧看门狗 config_wall_stale_sec_(ROS 参数 wall_stale_sec): 须≥右侧刷新周期, 随驱动 ultra_poll_period 调
            float front, rear; ros::Time ultra_stamp;
            {
                std::lock_guard<std::mutex> lk(sensor_mutex_);
                front = wall_sensors_.ultra_right_front;
                rear  = wall_sensors_.ultra_right_rear;
                ultra_stamp = ultra_right_stamp_;
            }
            bool front_valid = (front > 0.05f && front < 5.0f);
            bool rear_valid  = (rear  > 0.05f && rear  < 5.0f);
            bool ultra_fresh = (!ultra_stamp.isZero() &&
                                (ros::Time::now() - ultra_stamp).toSec() < config_wall_stale_sec_);
            bool ultra_ok = front_valid && rear_valid && ultra_fresh;
            bool imu_fresh = (hs_have_yaw_ &&
                              (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);

            // ---- 外环: 距墙/平行 → 有界航向偏置 C ----
            // C 单位 = rad(等效航向偏置, 叠进航向误差 e_total); K_DIST[rad/m] K_PAR[rad/m]; off_max 0.08≈4.6°。仅新鲜有效超声时刷新。
            float d = -1.0f, e_par = 0.0f, C = 0.0f;
            if (ultra_ok) {
                d = 0.5f * (front + rear);              // 距右墙
                e_par = rear - front;                   // >0:机头朝墙(欠转)
                float e_dist = config_wall_align_ - d;  // >0:离墙太近→需离墙
                C = (float)config_wall_k_dist_ * e_dist + (float)config_wall_k_par_ * e_par;
                C = std::max(-(float)config_wall_off_max_, std::min((float)config_wall_off_max_, C));
            } else if (front_valid != rear_valid) {
                C = rear_valid ? -0.05f : 0.05f;        // 单侧有墙: 轻偏重捕获(右前丢→右转(-); 右后丢→左转(+))
            }

            if (imu_fresh) {
                // 懒锁: 段首没等到 IMU → 此刻首帧到, 用它锁 yaw_base 并复位 PD(防以绝对航向0为基准)
                if (!yaw_base_locked) {
                    yaw_base = hs_yaw_rad_; yaw_base_locked = true;
                    hs_I_ = 0.0; hs_d_filt_ = 0.0; hs_e_prev_ = 0.0;
                    hs_t_prev_sample_ = hs_yaw_stamp_; hs_new_sample_ = false;
                    ROS_WARN("沿墙(串级): IMU 首帧到, 懒锁 yaw_base=%.1fdeg", hs_yaw_rad_ * RAD_TO_DEG);
                } else if (!imu_fresh_prev) {
                    // IMU 掉线后恢复: 不动 yaw_base, 只复位微分锚, 跳过本拍微分防跨间隙尖峰
                    hs_d_filt_ = 0.0;
                    hs_e_prev_ = (double)config_heading_sign_ * (double)normalizeAngle((float)(hs_yaw_rad_ - yaw_base));
                    hs_t_prev_sample_ = hs_yaw_stamp_; hs_new_sample_ = false;
                }
                // ======== 内环: 0614 实测好的航向 PD(误差 = IMU航向偏差 + 外环偏置C) ========
                double e_head = (double)config_heading_sign_ *
                                (double)normalizeAngle((float)(hs_yaw_rad_ - yaw_base));
                // 微分: 仅新 IMU 样本上、按样本 dt、一阶低通(tau_d); 只对航向求导, 不含慢变 C
                if (hs_new_sample_) {
                    hs_new_sample_ = false;
                    double dts = (hs_yaw_stamp_ - hs_t_prev_sample_).toSec();
                    if (dts < 0.005) dts = 0.005;
                    if (dts > 0.5)   dts = 0.5;
                    double raw_d = (double)normalizeAngle((float)(e_head - hs_e_prev_)) / dts;
                    double alpha = dts / (config_heading_tau_d_ + dts);
                    hs_d_filt_ += alpha * (raw_d - hs_d_filt_);
                    hs_e_prev_ = e_head;
                    hs_t_prev_sample_ = hs_yaw_stamp_;
                }
                double e_total = e_head + (double)C;    // 外环偏置进入误差(串级)
                double u_unsat = config_heading_kp_ * e_total
                               + config_heading_kd_ * hs_d_filt_ + hs_I_;
                // 条件抗饱和: 仅对航向稳态(e_head)积分, 不积分 C(否则会抵消距墙环)
                if (fabs(u_unsat) < config_heading_omega_limit_) {
                    hs_I_ += config_heading_ki_ * e_head * 0.05;   // loop dt≈0.05s(20Hz)
                    if (hs_I_ >  config_heading_i_clamp_) hs_I_ =  config_heading_i_clamp_;
                    if (hs_I_ < -config_heading_i_clamp_) hs_I_ = -config_heading_i_clamp_;
                }
                double u = u_unsat;
                if (u >  config_heading_omega_limit_) u =  config_heading_omega_limit_;
                if (u < -config_heading_omega_limit_) u = -config_heading_omega_limit_;
                current_angular_correction = (float)u;

                // 贴墙时保持 odom 兜底零点新鲜(以当前已对齐航向为零), 以防 IMU 后续掉线退化时航向错
                if (ultra_ok || (front_valid != rear_valid)) {
                    total_yaw_change = 0.0f; last_theta = current_pose_.theta;
                }

                dbg_imu_yaw_deg_  = (float)(hs_yaw_rad_ * RAD_TO_DEG);
                dbg_yaw_base_deg_ = (float)(yaw_base * RAD_TO_DEG);
                dbg_e_head_deg_   = (float)(e_head * RAD_TO_DEG);
                dbg_wall_d_ = d; dbg_e_par_ = e_par; dbg_C_ = C; dbg_u_ = (float)u;
                dbg_mode_ = ultra_ok ? 1 : 2;
                ROS_INFO_THROTTLE(1.0,
                    "沿墙(串级): d=%.2f e_par=%.3f C=%.3f | imu_e=%.1fdeg u=%.3f mode=%d",
                    d, e_par, C, dbg_e_head_deg_, dbg_u_, dbg_mode_);
            } else {
                // ======== 退化: /imu_heading 不可用 → doc 分支原逻辑(超声直接出角速度 + odom兜底) ========
                const float K_ODOM_HOLD = 1.0f;
                const float MAX_ODOM_CORR = 0.10f;
                float angular_cmd;
                if (ultra_ok || (front_valid != rear_valid)) {
                    angular_cmd = std::max(-(float)config_wall_off_max_,
                                           std::min((float)config_wall_off_max_, C));
                    total_yaw_change = 0.0f; last_theta = current_pose_.theta;  // 贴墙→刷新odom零点
                } else {
                    angular_cmd = -K_ODOM_HOLD * relative_theta;
                    angular_cmd = std::max(-MAX_ODOM_CORR, std::min(MAX_ODOM_CORR, angular_cmd));
                }
                current_angular_correction = angular_cmd;
                dbg_imu_yaw_deg_ = 0.0f; dbg_yaw_base_deg_ = 0.0f; dbg_e_head_deg_ = 0.0f;
                dbg_wall_d_ = d; dbg_e_par_ = e_par; dbg_C_ = C; dbg_u_ = angular_cmd; dbg_mode_ = 0;
                ROS_WARN_THROTTLE(1.0, "沿墙(IMU不可用→退化): u=%.3f (odom偏角=%.1f°)",
                                  angular_cmd, relative_theta * RAD_TO_DEG);
            }
            imu_fresh_prev = imu_fresh;   // 记录本拍 IMU 新鲜态(掉线恢复检测用)
        }
        
        // ========== 发送移动命令 ==========
        geometry_msgs::Twist cmd;
        cmd.linear.x = (move_direction == 'w') ? current_speed : -current_speed;
        cmd.linear.y = pump_on ? pump_speed : 0.0f;
        cmd.angular.z = current_angular_correction;
        cmd_vel_pub_.publish(cmd);
        
        // 进度打印
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0) {
            float progress = (total_accumulated_distance / total_target) * 100.0f;
            ROS_INFO("进度: %.1f%%, 已走%.2f/%.2f米, 偏角:%.1f°", 
                     progress, total_accumulated_distance, total_target, relative_theta * RAD_TO_DEG);
            last_print = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // ==================== 结束处理 ====================
    stop();
    if (pump_on) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.0f;
        cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(cmd);
        config_pump_speed_ = 0.0f;
    }
    
 
    
    disabling_timer_ = false;
    move_state_ = success ? 0 : 2;
    
    // ========== 保存行走日志 ==========
    saveWalkLog(success);
    return success;
}
// ==================== 右侧平行度对齐 ====================
 
void MoveController::alignToRightWallOnly() {
    const float TOLERANCE = 0.035f;  // 放宽容差到3.5cm
    ros::Time start_time = ros::Time::now();
    const float KP = 2.0f;           // 增大比例增益
    const float MAX_SPEED = 0.25f;   // 增大最大角速度
    const float TIMEOUT = 5.0f;      // 减少超时时间到5秒
    
    ROS_INFO("开始平行度对齐（改进版）");
    
    // 辅助函数：判断超声波数据是否有效
    auto isValid = [](float dist) -> bool {
        return (dist > 0.05f && dist < 5.0f);
    };
    
    // ========== 移动平均滤波缓冲区 ==========
    std::deque<float> front_buffer;
    std::deque<float> rear_buffer;
    const int WINDOW_SIZE = 3;
    
    // 记录上次有效值，用于跳变检测
    float last_front = 0, last_rear = 0;
    bool has_last = false;
    
    // ========== 阶段1：主要对齐 ==========
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > TIMEOUT) {
            ROS_WARN("平行度对齐超时(%0.f秒)，强制退出", TIMEOUT);
            break;
        }
        
        updateCurrentPose();
        
        float raw_front = wall_sensors_.ultra_right_front;
        float raw_rear = wall_sensors_.ultra_right_rear;
        
        bool front_valid = isValid(raw_front);
        bool rear_valid = isValid(raw_rear);
        
        // ========== 传感器跳变检测 ==========
        if (has_last && front_valid && rear_valid) {
            if (fabs(raw_front - last_front) > 0.3f || fabs(raw_rear - last_rear) > 0.3f) {
                ROS_WARN_THROTTLE(1.0, "传感器跳变 (前:%.2f→%.2f, 后:%.2f→%.2f)，忽略本次",
                                 last_front, raw_front, last_rear, raw_rear);
                ros::Duration(0.05).sleep();
                continue;
            }
        }
        last_front = raw_front;
        last_rear = raw_rear;
        has_last = true;
        
        // 应用滤波
        float front = movingAverageFilter(raw_front, front_buffer, WINDOW_SIZE);
        float rear = movingAverageFilter(raw_rear, rear_buffer, WINDOW_SIZE);
        
        // 数据无效处理
        if (!front_valid && !rear_valid) {
            ROS_WARN_THROTTLE(1.0, "两个超声波都无效，可能远离池壁");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
            continue;
        }
        
        if (!front_valid && rear_valid) {
            ROS_WARN_THROTTLE(1.0, "偏左（右前无效），右转纠正");
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0f;
            cmd.linear.y = config_pump_speed_;
            cmd.angular.z = -0.15f;  // 增大角速度
            cmd_vel_pub_.publish(cmd);
            ros::Duration(0.08).sleep();
            ros::spinOnce();
            continue;
        }
        
        if (front_valid && !rear_valid) {
            ROS_WARN_THROTTLE(1.0, "偏右（右后无效），左转纠正");
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0f;
            cmd.linear.y = config_pump_speed_;
            cmd.angular.z = 0.15f;   // 增大角速度
            cmd_vel_pub_.publish(cmd);
            ros::Duration(0.08).sleep();
            ros::spinOnce();
            continue;
        }
        
        // 两个都有效，使用PID控制
        float diff = rear - front;
        
        // 如果误差已经很小，直接退出
        if (fabs(diff) < TOLERANCE) {
            stop();
            ROS_INFO("平行度初步达标 (前后差=%.3fm)", diff);
            break;
        }
        
        // 比例控制
        float angular_speed = diff * KP;
        angular_speed = std::max(-MAX_SPEED, std::min(MAX_SPEED, angular_speed));
        
        // 减小日志频率
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {
            ROS_INFO("对齐: 前=%.3f, 后=%.3f, 差=%.3fm, 角速度=%.2f", 
                     front, rear, diff, angular_speed);
        }
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = config_pump_speed_;
        cmd.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd);
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    stop();
    ros::Duration(0.2).sleep();
    
    // ========== 阶段2：二次验证和微调 ==========
    ROS_INFO("开始二次验证...");
    
    // 重新获取稳定读数（取多次平均，增加采样数）
    const int SAMPLE_COUNT = 10;
    float front_sum = 0, rear_sum = 0;
    int valid_count = 0;
    
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        ros::spinOnce();
        updateCurrentPose();
        
        float f = wall_sensors_.ultra_right_front;
        float r = wall_sensors_.ultra_right_rear;
        
        if (isValid(f) && isValid(r)) {
            front_sum += f;
            rear_sum += r;
            valid_count++;
        }
        ros::Duration(0.1).sleep();
    }
    
    if (valid_count > 0) {
        float final_front = front_sum / valid_count;
        float final_rear = rear_sum / valid_count;
        float final_diff = final_rear - final_front;
        
        ROS_INFO("二次验证: 前=%.3f, 后=%.3f, 差=%.3fm", 
                 final_front, final_rear, final_diff);
        
        // 放宽微调阈值
        if (fabs(final_diff) > 0.025f) {
            ROS_WARN("需要微调，当前差=%.3fm", final_diff);
            
            // 微调方向
            float micro_speed = (final_diff > 0) ? 0.08f : -0.08f;
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0f;
            cmd.linear.y = config_pump_speed_;
            cmd.angular.z = micro_speed;
            
            // 根据偏差大小决定微调时间
            float duration = std::min(0.8f, fabs(final_diff) / 0.15f);
            ros::Time adjust_start = ros::Time::now();
            
            while ((ros::Time::now() - adjust_start).toSec() < duration) {
                cmd_vel_pub_.publish(cmd);
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
            stop();
            
            // 最终验证
            ros::Duration(0.3).sleep();
            updateCurrentPose();
            
            // 再次采样验证
            front_sum = 0; rear_sum = 0; valid_count = 0;
            for (int i = 0; i < 5; i++) {
                float f = wall_sensors_.ultra_right_front;
                float r = wall_sensors_.ultra_right_rear;
                if (isValid(f) && isValid(r)) {
                    front_sum += f;
                    rear_sum += r;
                    valid_count++;
                }
                ros::Duration(0.1).sleep();
            }
            
            if (valid_count > 0) {
                float final_check_front = front_sum / valid_count;
                float final_check_rear = rear_sum / valid_count;
                ROS_INFO("微调最终: 前=%.3f, 后=%.3f, 差=%.3fm", 
                         final_check_front, final_check_rear, 
                         final_check_rear - final_check_front);
            }
        } else {
            ROS_INFO("二次验证通过，无需微调");
        }
    }
    
    ROS_INFO("平行度对齐完成");
}
// ==================== 键盘模拟器初始化 ====================
void MoveController::initKeyboardSimulator() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    keyboard_sim_.reset();
    ROS_INFO("键盘模拟器初始化完成");
}

// ==================== 应用键盘命令 ====================
void MoveController::applyKeyboardCommand(char key) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    keyboard_sim_.idle_counter = 0;
    keyboard_sim_.key_pressed = true;
    keyboard_sim_.current_key = key;
    keyboard_sim_.key_press_time = ros::Time::now();
    
    switch (key) {
        case 'w':
            keyboard_sim_.target_vx += keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
        case 'x':
            keyboard_sim_.target_vx -= keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
        case 'a':
            keyboard_sim_.target_angular = keyboard_sim_.angular_inc;
            break;
        case 'd':
            keyboard_sim_.target_angular = -keyboard_sim_.angular_inc;
            break;
        case 's':
            keyboard_sim_.target_vx = 0.0f;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.current_vx = 0.0f;
            keyboard_sim_.key_pressed = false;
            break;
        default:
            break;
    }
}

// ==================== 键盘定时器回调 ====================
void MoveController::keyboardTimerCallback(const ros::TimerEvent& event) {
    if (disabling_timer_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    keyboard_sim_.idle_counter++;
    
    if (keyboard_sim_.idle_counter > keyboard_sim_.idle_threshold) {
        if (keyboard_sim_.key_pressed) {
            keyboard_sim_.key_pressed = false;
            keyboard_sim_.current_key = 0;
            keyboard_sim_.target_vx = 0.0f;
            keyboard_sim_.target_angular = 0.0f;
        }
    }
    
    float current_angular = keyboard_sim_.target_angular;
    
    if (keyboard_sim_.current_vx > keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = keyboard_sim_.max_linear;
    }
    if (keyboard_sim_.current_vx < -keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = -keyboard_sim_.max_linear;
    }
    
    if (current_angular > keyboard_sim_.max_angular) {
        current_angular = keyboard_sim_.max_angular;
    }
    if (current_angular < -keyboard_sim_.max_angular) {
        current_angular = -keyboard_sim_.max_angular;
    }
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = keyboard_sim_.current_vx;
    cmd.angular.z = current_angular;

     // ========== 只有行走状态才输出水泵速度 ==========
    if (move_state_ == 1) {  // 行走中
        cmd.linear.y = config_pump_speed_;
    } else {
        cmd.linear.y = 0.0f;
    }
    
    cmd_vel_pub_.publish(cmd);
}

// ==================== 传感器回调 ====================
void MoveController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    current_odom_ = *msg;
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.theta = calculateYawFromQuaternion(msg->pose.pose.orientation);
    
    if (!odom_received_) {
        ROS_INFO("✅ 首次收到odom数据！位置: (%.3f, %.3f, %.1f°)",
                current_pose_.x, current_pose_.y, current_pose_.theta * RAD_TO_DEG);
    }
    odom_received_ = true;
}

void MoveController::ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    if (msg->data.size() >= 4) {
        // 新鲜度：右侧任一原始值变化即视为收到新帧(每通道真实 ~0.83Hz)
        int raw_rf = msg->data[2];
        int raw_rr = msg->data[3];
        if (raw_rf != last_ultra_raw_rf_ || raw_rr != last_ultra_raw_rr_) {
            ultra_right_seq_++;
            ultra_right_stamp_ = ros::Time::now();
            last_ultra_raw_rf_ = raw_rf;
            last_ultra_raw_rr_ = raw_rr;
        }

        wall_sensors_.infra_front_left = msg->data[0] / 1000.0f;   // 前左超声波
        wall_sensors_.infra_front_right = msg->data[1] / 1000.0f;  // 前右超声波
        wall_sensors_.ultra_right_front = msg->data[2] / 1000.0f;  // 右前超声波
        wall_sensors_.ultra_right_rear = msg->data[3] / 1000.0f;   // 右后超声波
        
        // 前方超声波过滤
        if (wall_sensors_.infra_front_left > 5.0f || wall_sensors_.infra_front_left < 0.05f) {
            wall_sensors_.infra_front_left = 999.0f;
        }
        if (wall_sensors_.infra_front_right > 5.0f || wall_sensors_.infra_front_right < 0.05f) {
            wall_sensors_.infra_front_right = 999.0f;
        }
        
        // 右侧超声波过滤
        if (wall_sensors_.ultra_right_front > 5.0f || wall_sensors_.ultra_right_front < 0.05f) {
            wall_sensors_.ultra_right_front = 999.0f;
        }
        if (wall_sensors_.ultra_right_rear > 5.0f || wall_sensors_.ultra_right_rear < 0.05f) {
            wall_sensors_.ultra_right_rear = 999.0f;
        }
    }
}

void MoveController::imuHeadingCallback(const std_msgs::Float32::ConstPtr& msg) {
    // /imu_heading = IMU 融合航向(度, D-19 取 imu_encoder.roll 轴, 由 mars_serial_server 驱动发布)。
    // 与主循环 spinOnce 同线程执行, 无需加锁。无 header 时间戳 → 用到达时刻作样本时间(用于微分 dt)。
    hs_yaw_rad_    = (double)msg->data * (double)DEG_TO_RAD;
    hs_yaw_stamp_  = ros::Time::now();
    hs_have_yaw_   = true;
    hs_new_sample_ = true;
}

// ==================== 辅助函数 ====================
void MoveController::updateCurrentPose() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (odom_received_) {
        current_pose_.x = current_odom_.pose.pose.position.x;
        current_pose_.y = current_odom_.pose.pose.position.y;
        current_pose_.theta = calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    } else {
        updateCurrentPoseFromTF();
    }
}

void MoveController::updateCurrentPoseFromTF() {
    try {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
        current_pose_.x = transform.getOrigin().x();
        current_pose_.y = transform.getOrigin().y();
        current_pose_.theta = tf::getYaw(transform.getRotation());
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF变换失败: %s", ex.what());
    }
}

bool MoveController::waitForOdom(float timeout) {
    if (odom_received_) {
        return true;
    }
    
    ros::Time start_time = ros::Time::now();
    ROS_INFO("等待 /wheel_odom 数据，超时: %.1f秒", timeout);
    
    while (ros::ok() && !odom_received_) {
        ros::spinOnce();
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("等待odom超时");
            return false;
        }
        ros::Duration(0.01).sleep();
    }
    
    ROS_INFO("✅ odom数据就绪");
    return true;
}

float MoveController::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (fabs(yaw) < 0.0001f) yaw = 0.0;
    return static_cast<float>(yaw);
}

float MoveController::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void MoveController::stopMoving() {
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.current_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.key_pressed = false;
        keyboard_sim_.idle_counter = keyboard_sim_.idle_threshold + 1;
    }
    
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0f;
    stop_cmd.linear.y = 0.0f;
    stop_cmd.angular.z = 0.0f;
    
    for (int i = 0; i < 3; i++) {
        cmd_vel_pub_.publish(stop_cmd);
        ros::Duration(0.05).sleep();
    }
}

// ==================== 日志记录函数 ====================

std::string MoveController::getWalkLogDirectory() {
    std::string home_dir = getenv("HOME");
    std::string base_dir = home_dir + "/sonar_logs/";
    std::string walk_dir = base_dir + "move_logs/";
    
    struct stat st;
    if (stat(base_dir.c_str(), &st) != 0) {
        mkdir(base_dir.c_str(), 0755);
    }
    if (stat(walk_dir.c_str(), &st) != 0) {
        mkdir(walk_dir.c_str(), 0755);
    }
    
    return walk_dir;
}

void MoveController::initWalkLog(const std::string& log_type, float target_distance) {
    log_buffer_.clear();
    
    std::string dir = getWalkLogDirectory();
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tm_info = std::localtime(&now_time_t);
    char time_buf[64];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", tm_info);
    
    current_log_filename_ = dir + log_type + "_" + std::string(time_buf) + 
                            "_dist" + std::to_string(target_distance) + ".csv";
    
    if (walk_log_file_.is_open()) {
        walk_log_file_.close();
    }
    walk_log_file_.open(current_log_filename_);
    
    if (walk_log_file_.is_open()) {
        walk_log_file_ << "elapsed_sec,right_front_m,right_rear_m,infra_left_m,infra_right_m,"
                       << "pose_x_m,pose_y_m,pose_theta_deg,target_distance_m,current_distance_m,"
                       << "imu_yaw_deg,yaw_base_deg,e_head_deg,wall_d_m,e_par_m,ctrl_C,ctrl_u,ctrl_mode,status\n";
        walk_log_file_.flush();
        ROS_INFO("MoveController行走日志已创建: %s", current_log_filename_.c_str());
    } else {
        ROS_ERROR("无法创建行走日志文件: %s", current_log_filename_.c_str());
    }
    
    last_log_time_ = ros::Time::now();
    walk_start_time_ = ros::Time::now();
}

void MoveController::recordWalkData(float current_distance, float target_distance) {
    ros::Time now = ros::Time::now();
    
    if ((now - last_log_time_).toSec() < 1.0) {
        return;
    }
    
    WalkLogData data;
    data.timestamp = now;
    
    // 获取传感器数据
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    data.right_front = wall_sensors_.ultra_right_front;
    data.right_rear = wall_sensors_.ultra_right_rear;
    data.infra_left = wall_sensors_.infra_front_left;
    data.infra_right = wall_sensors_.infra_front_right;
    
    // 获取位姿数据
    data.pose_x = current_pose_.x;
    data.pose_y = current_pose_.y;
    data.pose_theta = current_pose_.theta;
    data.target_distance = target_distance;
    data.current_distance = current_distance;
    // 串级调参快照(主循环每拍更新)
    data.imu_yaw_deg  = dbg_imu_yaw_deg_;
    data.yaw_base_deg = dbg_yaw_base_deg_;
    data.e_head_deg   = dbg_e_head_deg_;
    data.wall_d       = dbg_wall_d_;
    data.e_par        = dbg_e_par_;
    data.ctrl_C       = dbg_C_;
    data.ctrl_u       = dbg_u_;
    data.ctrl_mode    = dbg_mode_;

    log_buffer_.push_back(data);
    last_log_time_ = now;
}

void MoveController::saveWalkLog(bool success) {
    if (!walk_log_file_.is_open()) {
        ROS_WARN("行走日志文件未打开，无法保存");
        return;
    }
    
    for (size_t i = 0; i < log_buffer_.size(); i++) {
        const auto& data = log_buffer_[i];
        double elapsed = (data.timestamp - walk_start_time_).toSec();
        
        walk_log_file_ << std::fixed << std::setprecision(3)
                       << elapsed << ","
                       << data.right_front << ","
                       << data.right_rear << ","
                       << data.infra_left << ","
                       << data.infra_right << ","
                       << data.pose_x << ","
                       << data.pose_y << ","
                       << data.pose_theta * RAD_TO_DEG << ","
                       << data.target_distance << ","
                       << data.current_distance << ","
                       << data.imu_yaw_deg << ","
                       << data.yaw_base_deg << ","
                       << data.e_head_deg << ","
                       << data.wall_d << ","
                       << data.e_par << ","
                       << data.ctrl_C << ","
                       << data.ctrl_u << ","
                       << data.ctrl_mode << ","
                       << (success ? "SUCCESS" : "FAILED") << "\n";
    }
    
    walk_log_file_.flush();
    walk_log_file_.close();
    
    ROS_INFO("MoveController行走日志已保存: %s (共%zu条记录)", 
             current_log_filename_.c_str(), log_buffer_.size());
}

 
float MoveController::movingAverageFilter(float new_value, std::deque<float>& buffer, int window_size) {
    buffer.push_back(new_value);
    if (buffer.size() > window_size) {
        buffer.pop_front();
    }
    float sum = 0.0f;
    for (float v : buffer) {
        sum += v;
    }
    return sum / buffer.size();
}

bool MoveController::rotateToTargetAngle(float target_angle_deg, float timeout) {
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float ANGLE_TOLERANCE = 3.0f;  // 允许3度误差
    
    float target_rad = target_angle_deg * DEG_TO_RAD;
    float start_yaw = current_pose_.theta;
    
    // 计算需要旋转的方向和角度
    float delta = normalizeAngle(target_rad - start_yaw);
    float abs_delta = fabs(delta);
    
    if (abs_delta < ANGLE_TOLERANCE * DEG_TO_RAD) {
        ROS_INFO("已接近目标角度(%.1f°)，无需旋转", target_angle_deg);
        return true;
    }
    
    // 确定旋转方向
    float angular_speed = (delta > 0) ? 0.25f : -0.25f;
    
    ROS_INFO("旋转: 当前角度=%.1f°, 目标=%.1f°, 需要转%.1f°, 方向=%s",
             start_yaw * RAD_TO_DEG, target_angle_deg, abs_delta * RAD_TO_DEG,
             (delta > 0) ? "左转" : "右转");
    
    ros::Time start_time = ros::Time::now();
    float last_error = abs_delta;
    int stall_count = 0;   
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_WARN("旋转超时");
            break;
        }
        
        updateCurrentPose();
        
        float current_error = normalizeAngle(target_rad - current_pose_.theta);
        float abs_error = fabs(current_error);
        
        // 到达目标
        if (abs_error < ANGLE_TOLERANCE * DEG_TO_RAD) {
            stop();
            ROS_INFO("旋转完成: 当前角度=%.1f°, 误差=%.1f°", 
                     current_pose_.theta * RAD_TO_DEG, abs_error * RAD_TO_DEG);
            return true;
        }
        
        // 接近目标时减速
        float current_speed = angular_speed;
        if (abs_error < 15.0f * DEG_TO_RAD) {
            current_speed = angular_speed * 0.5f;
        }
        
        // 检测是否卡住
        if (abs_error >= last_error - 0.5f * DEG_TO_RAD) {
            static int stall_count = 0;
            stall_count++;
            if (stall_count > 20) {
                current_speed = angular_speed * 1.5f;  // 卡住时增加速度
                ROS_WARN_THROTTLE(1.0, "旋转卡住，增加速度");
                stall_count = 0;
            }
        } else {
            stall_count = 0;
        }
        last_error = abs_error;
        
        // 发送旋转命令
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = config_pump_speed_;
        cmd.angular.z = current_speed;
        cmd_vel_pub_.publish(cmd);
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    stop();
    ROS_WARN("旋转未完成: 当前角度=%.1f°, 目标=%.1f°", 
             current_pose_.theta * RAD_TO_DEG, target_angle_deg);
    return false;
}

} // namespace sonar_slam
 
// ==================== main 函数 ====================
int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ros::init(argc, argv, "move_controller_node");
    
    ROS_INFO("========================================");
    ROS_INFO("MoveController 节点启动");
    ROS_INFO("========================================");
    
    ros::NodeHandle nh("~");
    
    sonar_slam::MoveController controller;
    
    ROS_INFO("服务已就绪: /move_controller/move_straight");
    ROS_INFO("========================================");
    
    // ========== 自动测试参数（可选） ==========
    bool auto_start = false;
    nh.param("auto_start", auto_start, false);
    
    float test_distance = 4.1f;
    float test_speed = 0.25f;   // auto_start 调试用默认; 与构造函数 /speed 默认一致(0.25, 防履带打滑)。注: 此 setParam 走私有 nh, 与控制器读的全局 /speed 不同命名空间, 实际仍以全局 /speed 为准
    float test_timeout = 60.0f;
    float test_pump = 0.4f;
    
    nh.param("test_distance", test_distance, test_distance);
    nh.param("test_speed", test_speed, test_speed);
    nh.param("test_timeout", test_timeout, test_timeout);
    nh.param("test_pump", test_pump, test_pump);
    
    // 设置参数（自动测试时会使用）
    nh.setParam("speed", test_speed);
    nh.setParam("pump_speed", test_pump);
    
    if (auto_start) {
        ROS_INFO("========================================");
        ROS_INFO("自动测试模式启动");
        ROS_INFO("  距离: %.2f 米", test_distance);
        ROS_INFO("  速度: %.2f m/s", test_speed);
        ROS_INFO("  超时: %.1f 秒", test_timeout);
        ROS_INFO("  水泵: %.1f", test_pump);
        ROS_INFO("========================================");
        
        // 等待1秒让节点完全初始化
        ros::Duration(1.0).sleep();
        
        bool success = controller.moveStraight(test_distance, test_timeout);
        
        if (success) {
            ROS_INFO("✅ 自动行走测试成功");
        } else {
            ROS_ERROR("❌ 自动行走测试失败");
        }
        
        // 自动测试完成后可以选择退出或继续运行
        // return 0;  // 如果测试完就退出
        // 如果不退出，继续运行等待服务调用
    }
    
    // ========== 主循环（同时支持服务调用） ==========
    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("MoveController 节点关闭");
    return 0;
}
// int main(int argc, char** argv) {
//     std::setlocale(LC_ALL, "");
//     std::setlocale(LC_NUMERIC, "C");
    
//     ros::init(argc, argv, "move_controller_node");
    
//     ROS_INFO("========================================");
//     ROS_INFO("MoveController 节点启动");
//     ROS_INFO("========================================");
    
//     ros::NodeHandle nh("~");
     
  
    
//     sonar_slam::MoveController controller;
    
//     ROS_INFO("服务已就绪: /move_controller/move_straight");
//     ROS_INFO("========================================");
    
//     ros::Rate rate(20);
    
//     while (ros::ok()) {
//         ros::spinOnce();
//         rate.sleep();
//     }
    
//     ROS_INFO("MoveController 节点关闭");
//     return 0;
// }

