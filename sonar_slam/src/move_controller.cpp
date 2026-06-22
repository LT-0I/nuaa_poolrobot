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
    config_wall_stale_sec_(0.5f),
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

    // 订阅运行级水泵闩锁 /move_controller/pump_enable(std_msgs::Bool):
    // true=整段沿边任务持泵(mapper 任务起始前置位)、false=撤闩回落单段策略。
    // 单测(无 mapper)时无人发布、保持 false=原单段行为。
    pump_enable_sub_ = nh_.subscribe("/move_controller/pump_enable", 10, &MoveController::pumpEnableCallback, this);
    ROS_INFO("订阅器: /move_controller/pump_enable");

    // 创建服务服务器
    move_straight_srv_ = nh_.advertiseService(
        "/move_controller/move_straight",
        &MoveController::moveStraightCallback, this);
    ROS_INFO("服务服务器: /move_controller/move_straight");

    // 下水自启找侧壁服务: 旋转找侧壁 → 对齐 → 进入沿边
    find_wall_srv_ = nh_.advertiseService(
        "/move_controller/find_wall",
        &MoveController::findWallCallback, this);
    ROS_INFO("服务服务器: /move_controller/find_wall");
    
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
    // 水泵速度读 ROS 参数 pump_speed(默认 0.4 = 改前实际值); 夹 [0,1] 匹配驱动 linear.y->spin 映射。
    float temp_pump = 0.4f;
    nh_.param("pump_speed", temp_pump, 0.4f);
    if (temp_pump < 0.0f) temp_pump = 0.0f;
    if (temp_pump > 1.0f) temp_pump = 1.0f;
    user_config_pump_speed_ = temp_pump;
    // 目标距右墙距离(m)，沿墙距离环用。默认2.0(据实际实验取2m); 同时驱动拐角触发距离与找墙正前净空门。
    // ⚠头号下水未知: 右前/右后超声在掠射平直池壁能否拿到稳定回波(掠射存亡); 拿不到→串级退化纯IMU守航向(直行,
    //   距墙环失效)。下水前先静态实测(距右墙2.0m平行静止读 data[2]/[3] 是否稳定有效); 拿不到则降到掠射可见最大距离(如1.5)。
    nh_.param("wall_align", config_wall_align_, 2.0f);
    // 右侧超声陈旧看门狗(s)：默认0.5(超声实测 MCU 推流 ~10Hz=0.1s/路、≈5×周期)；夹在[0.3,5.0]防误设。
    // 实测(2026-06-14 g2a_map CSV)四路各 ~10Hz, 比旧推断的 0.83Hz 快得多→可比旧 1.5s 收紧, 丢回波盲走更短。
    // ⚠ 须≥右侧实际刷新周期：若 MCU 推流率下降或驱动异常导致刷新变慢, 须同步上调本值, 否则超声环↔odom兜底抖动。
    nh_.param("wall_stale_sec", config_wall_stale_sec_, 0.5f);
    if (config_wall_stale_sec_ < 0.3f) config_wall_stale_sec_ = 0.3f;
    if (config_wall_stale_sec_ > 5.0f) config_wall_stale_sec_ = 5.0f;
    config_pump_speed_ = 0.0f;

    // ===== 串级直行参数(ROS 全局参数: 启动前 rosparam set /xxx; 默认=0614实测好值) =====
    // 内环 = 0614 实测能走直的 IMU 航向 PD; 外环 = doc 分支超声距墙/平行环(折成航向偏置)
    nh_.param("heading_kp",          config_heading_kp_,          1.0);
    nh_.param("heading_kd",          config_heading_kd_,          1.0);
    nh_.param("heading_ki",          config_heading_ki_,          0.0);
    nh_.param("heading_i_clamp",     config_heading_i_clamp_,     0.05);
    nh_.param("heading_omega_limit", config_heading_omega_limit_, 0.30);  // Q3 实测直行值 0.30(夹限[0,0.3]上界)
    nh_.param("heading_tau_d",       config_heading_tau_d_,       0.2);
    nh_.param("heading_yaw_timeout", config_heading_yaw_timeout_, 0.5);
    nh_.param("heading_sign",        config_heading_sign_,        1);
    config_heading_sign_ = (config_heading_sign_ >= 0) ? 1 : -1;
    nh_.param("wall_k_dist",         config_wall_k_dist_,         0.20);  // 超声实测10Hz→比旧0.03激进; off_max 兜底
    nh_.param("wall_k_par",          config_wall_k_par_,          0.60);
    nh_.param("wall_off_max",        config_wall_off_max_,        0.10);
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

    // ===== P1 隔离 + P2 拐角(停-转-融合航向闭环)参数 (全局 nh_.param, 构造期读一次) =====
    nh_.param("enable_corner",          config_enable_corner_,          true);
    nh_.param("corner_front_trigger_m", config_corner_front_trigger_m_, 1.5f);
    nh_.param("corner_use_wall_dist",   config_corner_use_wall_dist_,   true);   // 前墙触发距离是否跟随 wall_align(默认开: 走够离墙距即停转)
    nh_.param("corner_front_confirm",   config_corner_front_confirm_,   3);
    nh_.param("corner_settle_sec",      config_corner_settle_sec_,      1.0f);
    nh_.param("corner_turn_deg",        config_corner_turn_deg_,        90.0f);
    nh_.param("corner_turn_omega",      config_corner_turn_omega_,      0.2f);
    nh_.param("corner_turn_omega_fine", config_corner_turn_omega_fine_, 0.10f);
    nh_.param("corner_slow_deg",        config_corner_slow_deg_,        15.0f);
    nh_.param("corner_turn_timeout",    config_corner_turn_timeout_,    8.0f);
    nh_.param("corner_turn_cap_deg",    config_corner_turn_cap_deg_,    130.0f);
    nh_.param("wall_consistency_m",     config_wall_consistency_m_,     0.5f);
    // 夹限防误设
    if (config_corner_front_confirm_   < 1)     config_corner_front_confirm_   = 1;
    if (config_corner_front_trigger_m_ < 0.4f)  config_corner_front_trigger_m_ = 0.4f;
    if (config_corner_settle_sec_      < 0.0f)  config_corner_settle_sec_      = 0.0f;
    if (config_corner_settle_sec_      > 10.0f) config_corner_settle_sec_      = 10.0f;
    if (config_corner_turn_deg_        < 10.0f) config_corner_turn_deg_        = 10.0f;
    if (config_corner_turn_deg_        > 170.0f)config_corner_turn_deg_        = 170.0f;
    if (config_corner_turn_cap_deg_    < config_corner_turn_deg_ + 20.0f) config_corner_turn_cap_deg_ = config_corner_turn_deg_ + 20.0f;
    if (config_corner_turn_omega_      < 0.0f)  config_corner_turn_omega_      = 0.0f;
    if (config_corner_turn_omega_      > 0.3f)  config_corner_turn_omega_      = 0.3f;
    if (config_corner_turn_omega_fine_ < 0.0f)  config_corner_turn_omega_fine_ = 0.0f;
    if (config_corner_turn_omega_fine_ > config_corner_turn_omega_) config_corner_turn_omega_fine_ = config_corner_turn_omega_;
    if (config_corner_turn_timeout_    < 1.0f)  config_corner_turn_timeout_    = 1.0f;
    if (config_wall_consistency_m_     < 0.05f) config_wall_consistency_m_     = 0.05f;

    // ===== 下水自启找侧壁(bootstrap) 参数 (全局 nh_.param, 启动前 rosparam set /xxx) =====
    nh_.param("findwall_omega",       config_findwall_omega_,       0.18f);
    nh_.param("findwall_turn_left",   config_findwall_turn_left_,   true);
    nh_.param("findwall_confirm_sec", config_findwall_confirm_sec_, 0.4f);
    nh_.param("findwall_max_rot_deg", config_findwall_max_rot_deg_, 540.0f);
    nh_.param("findwall_timeout",     config_findwall_timeout_,     25.0f);
    if (config_findwall_omega_       < 0.05f)   config_findwall_omega_       = 0.05f;
    if (config_findwall_omega_       > 0.30f)   config_findwall_omega_       = 0.30f;
    if (config_findwall_confirm_sec_ < 0.0f)    config_findwall_confirm_sec_ = 0.0f;
    if (config_findwall_confirm_sec_ > 5.0f)    config_findwall_confirm_sec_ = 5.0f;
    if (config_findwall_max_rot_deg_ < 90.0f)   config_findwall_max_rot_deg_ = 90.0f;
    if (config_findwall_max_rot_deg_ > 1080.0f) config_findwall_max_rot_deg_ = 1080.0f;
    if (config_findwall_timeout_     < 3.0f)    config_findwall_timeout_     = 3.0f;
    if (config_findwall_timeout_     > 120.0f)  config_findwall_timeout_     = 120.0f;

    // 版本闸门: ASCII 横幅, 操作单 grep 自判是否在跑新二进制(防跑旧编译)
    ROS_WARN("MOVE_CTRL_VERSION = wallfollow-cascade-findwall-frontclear-fix-wa2m-20260622 (wall_align default 2.0m; corner trigger follows wall_align; findwall front-clear uses SAME trig as corner so acquire hands off without spurious turn; per-run log to $HOME/sonar_logs/move_logs)");
    ROS_INFO("  cascade params: heading_kp=%.2f kd=%.2f ki=%.2f omega_limit=%.2f sign=%d",
             config_heading_kp_, config_heading_kd_, config_heading_ki_, config_heading_omega_limit_, config_heading_sign_);
    ROS_INFO("  wall outer: k_dist=%.3f k_par=%.3f off_max=%.3f | speed_max=%.2f softstart=%.2f consistency=%.2f",
             config_wall_k_dist_, config_wall_k_par_, config_wall_off_max_, config_speed_max_, config_softstart_sec_, config_wall_consistency_m_);
    ROS_INFO("  corner: enable=%d front_trig=%.2fm confirm=%d settle=%.2fs turn=%.0fdeg omega=%.2f(fine=%.2f) slow=%.0fdeg cap=%.0fdeg timeout=%.1fs",
             (int)config_enable_corner_, config_corner_front_trigger_m_, config_corner_front_confirm_, config_corner_settle_sec_,
             config_corner_turn_deg_, config_corner_turn_omega_, config_corner_turn_omega_fine_, config_corner_slow_deg_,
             config_corner_turn_cap_deg_, config_corner_turn_timeout_);
    ROS_INFO("  findwall(bootstrap): omega=%.2f turn_left=%d confirm=%.2fs max_rot=%.0fdeg timeout=%.1fs | acquire: right both valid & |rear-front|<%.2fm & front>=%.2fm(eff; =wall_align when corner_use_wall_dist)",
             config_findwall_omega_, (int)config_findwall_turn_left_, config_findwall_confirm_sec_,
             config_findwall_max_rot_deg_, config_findwall_timeout_, config_wall_consistency_m_,
             (config_corner_use_wall_dist_ ? config_wall_align_ : config_corner_front_trigger_m_));
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

// ==================== 服务回调: 下水自启找侧壁 ====================
bool MoveController::findWallCallback(sonar_slam::MoveStraight::Request& req,
                                      sonar_slam::MoveStraight::Response& res) {
    // 复用 MoveStraight 服务类型: distance=找到墙对齐后沿边行走距离(默认 50m≈绕池足够), timeout=沿边阶段超时。
    // 找墙旋转阶段另由 findwall_* 参数约束(见构造函数)。
    float ftimeout = (req.timeout > 0) ? req.timeout : 60.0f;
    float fdist    = (fabs(req.distance) > 0.01f) ? fabs(req.distance) : 50.0f;
    ROS_INFO("[findwall] service request: follow_distance=%.2f timeout=%.1f", fdist, ftimeout);
    bool success = findWallAndFollow(fdist, ftimeout);
    res.success = success;
    res.message = success ? "find_wall+follow ok" : "find_wall+follow failed";
    res.actual_distance = success ? fdist : 0.0f;
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
    bool corner_armed = true;             // 拐角触发武装位: 触发即下锁, 须前方重新净空才再武装(防欠转后老前墙仍在视野立即重触发->累计U转)
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

        // ==================== 前墙拐角: 停-转-融合航向闭环 (P2) ====================
        // enable_corner=false → 整块跳过, 走纯串级直行(直行段单独整定 / 回归基线)。
        // 触发: 前超声 data0(前左)/data1(前右) min < corner_trig(默认跟随 wall_align: 走够离墙距即停转; 否则独立 corner_front_trigger_m) 连续 confirm 帧。
        // 看门狗铁律: 所有相位 ≤0.05s 持续 publish, 无盲 sleep; 泵全程 linear.y=pump_speed(看门狗只停电机不停泵)。
        if (config_enable_corner_ && !wall_avoiding && move_direction == 'w') {
            // 前墙读取持 sensor_mutex_ 快照(与 ultraCallback 写侧锁规则一致)
            float front_left_dist, front_right_dist;
            {
                std::lock_guard<std::mutex> lk(sensor_mutex_);
                front_left_dist  = wall_sensors_.infra_front_left;
                front_right_dist = wall_sensors_.infra_front_right;
            }
            float front_dist = std::min(front_left_dist, front_right_dist);
            // 地板取有效量程下限 0.05(而非 0.3): 近场 [0.05,0.3) 也判有墙, 防机器人已逼到 <0.3m 时落进盲带漏触发撞墙
            // (无效/无回波在 ultraCallback 已兜成 999 > trigger, 自然排除)。评审 PARTIAL 安全项。
            // 前墙触发距离: corner_use_wall_dist=true 时跟随 wall_align(走够离墙距即停转, 用户指定逻辑); 否则用独立 corner_front_trigger_m
            float corner_trig = config_corner_use_wall_dist_ ? config_wall_align_ : config_corner_front_trigger_m_;
            bool has_front_wall = (front_dist > 0.05f && front_dist < corner_trig);

            if (!has_front_wall) {
                front_wall_detect_count = 0;
                corner_armed = true;   // 前方净空 -> 重新武装
            } else if (corner_armed && ++front_wall_detect_count >= config_corner_front_confirm_) {
                corner_armed = false;  // 触发即下锁; 须前方重新净空才再武装(评审D MAJOR: 防转向欠转后老前墙仍<trig时立即重触发->累计U转)
                ROS_WARN("[corner] front wall %.2fm (confirm %d) -> STOP-and-turn", front_dist, front_wall_detect_count);
                wall_avoiding = true;

                // ---------- SETTLE: 20Hz 持续发零运动(泵保持)停稳取入航向, 喂看门狗(替换原盲 sleep) ----------
                {
                    ros::Time s0 = ros::Time::now();
                    while (ros::ok() && (ros::Time::now() - s0).toSec() < config_corner_settle_sec_) {
                        geometry_msgs::Twist z;
                        z.linear.x = 0.0f; z.linear.y = pump_speed; z.angular.z = 0.0f;
                        cmd_vel_pub_.publish(z);
                        ros::spinOnce();
                        ros::Duration(0.05).sleep();
                    }
                    updateCurrentPose();
                }

                // ---------- TURN: 融合航向闭环左转 corner_turn_deg, 持续发布 ----------
                // 入口同刻捕获融合航向 h0 与 odom 入角 odom0(同基准, 评审 HIGH); IMU 全程不可用→纯 odom 兜底转(降级)。
                bool   imu_entry = (hs_have_yaw_ &&
                                    (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);
                double h0    = imu_entry ? hs_yaw_rad_ : 0.0;
                float  odom0 = current_pose_.theta;
                float  target_rad = config_corner_turn_deg_    * DEG_TO_RAD;
                float  cap_rad    = config_corner_turn_cap_deg_ * DEG_TO_RAD;
                float  slow_rad   = config_corner_slow_deg_     * DEG_TO_RAD;
                ros::Time turn_t0 = ros::Time::now();
                ros::Time last_imu_fresh = ros::Time::now();
                int   reach_confirm = 0;
                float imu_anchor  = 0.0f;                   // 最近一帧 IMU 测得的已转角(幅值,rad); 帧间用 odom 增量插值
                float odom_anchor = odom0;                  // 该 IMU 帧时刻的 odom theta(插值基准)
                float last_progress_delta = 0.0f;           // 增量进度高水位(替代一次性 latch, 覆盖 fine 段停转, 评审 HIGH)
                ros::Time last_progress_time = ros::Time::now();
                bool  moving_started = false;               // 见证已起转(分离"从静止起转延迟"与"转中停转", 评审 LOW)
                if (!imu_entry) ROS_WARN("[corner] no fresh /imu_heading at entry -> odom fallback turn (degraded)");

                while (ros::ok()) {
                    ros::spinOnce();
                    updateCurrentPose();
                    bool new_imu = hs_new_sample_;          // 本拍是否有新 IMU 帧(消费); 用于 IMU 绝对角重锚
                    if (new_imu) hs_new_sample_ = false;

                    if ((ros::Time::now() - turn_t0).toSec() > config_corner_turn_timeout_) {
                        ROS_WARN("[corner] turn timeout %.1fs -> stop", config_corner_turn_timeout_);
                        break;
                    }

                    bool  imu_fresh_now = (hs_have_yaw_ &&
                                           (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);
                    float odom_delta = fabs(normalizeAngle(current_pose_.theta - odom0));
                    float delta;
                    if (imu_entry) {
                        if (imu_fresh_now) {
                            last_imu_fresh = ros::Time::now();
                        } else if ((ros::Time::now() - last_imu_fresh).toSec() > config_heading_yaw_timeout_) {
                            // 入口有 IMU 但中途陈旧超阈(有效率<2Hz) → 保守硬停(不乐观续转到 CAP, 评审 HIGH)
                            ROS_WARN("[corner] IMU went stale mid-turn -> conservative stop (~%.0f deg)",
                                     (imu_anchor + fabs(normalizeAngle(current_pose_.theta - odom_anchor))) * RAD_TO_DEG);
                            break;
                        }
                        // 新帧到 → 用 IMU 绝对角重锚(去 odom 累积漂移)+记 odom 锚; 帧间 delta=锚+odom增量。
                        // 这样 reach 判据不再是 10Hz ZOH 冻结值(帧间持续转却读旧角→过转, 评审 MEDIUM): odom 插值使 delta 每拍平滑跟随真航向,
                        // 过转上界压到 ~1 个 odom 拍(几度), 与 IMU 推流率无关; 同时每帧 IMU 重锚消除 odom 累积漂移。
                        if (new_imu) {
                            float cand = fabs(normalizeAngle((float)(hs_yaw_rad_ - h0)));
                            // 单帧前跳钳位(评审 HIGH): 正常帧 cand≈插值估计 pred(差=漂移几度); IMU 单帧尖峰使 cand 暴增→截到 pred+8°,
                            // 防尖峰把 delta 拉到/越过 target 提前误停(/imu_heading 是融合航向、尖峰本不应大, 此为无数据依赖的兜底)。
                            float pred = imu_anchor + fabs(normalizeAngle(current_pose_.theta - odom_anchor));
                            if (cand > pred + 8.0f * DEG_TO_RAD) cand = pred + 8.0f * DEG_TO_RAD;
                            imu_anchor  = cand;
                            odom_anchor = current_pose_.theta;
                        }
                        delta = imu_anchor + fabs(normalizeAngle(current_pose_.theta - odom_anchor));
                    } else {
                        delta = odom_delta;   // 纯 odom 兜底
                    }

                    // odom 角兜底(宽裕度, 仅兜 IMU 大故障/防过转, 默认 cap=turn+40°): 不与到位判据争紧裕度(评审 HIGH)。
                    if (odom_delta > cap_rad) {
                        ROS_WARN("[corner] odom CAP %.0f deg -> stop", odom_delta * RAD_TO_DEG);
                        break;
                    }
                    // 达标确认【先于停转判定】(防 fine 段慢逼近被误判停转, 评审 MEDIUM): delta 已 odom 插值平滑, 连续 2 拍拒单拍噪声。
                    if (delta >= target_rad) {
                        if (++reach_confirm >= 2) {
                            ROS_INFO("[corner] reached %.0f deg (confirmed) -> stop turn", delta * RAD_TO_DEG);
                            break;
                        }
                    } else {
                        reach_confirm = 0;
                    }
                    // 起转/停转见证【基于裸 odom_delta=实际物理转角】(单调、无重锚台阶, 评审 MEDIUM): 物理推进刷新高水位;
                    //   已起转后 >1.5s 无 >2° 物理推进→停转(覆盖 fine 弱命令水下停转, 防空转到 8s timeout, 评审 HIGH);
                    //   从静止 >2.5s 仍未转→停(分离"从静止起转延迟"与"转中停转", 防慢起转被 0° 误判, 评审 LOW)。
                    if (odom_delta > last_progress_delta + 2.0f * DEG_TO_RAD) {
                        last_progress_delta = odom_delta;
                        last_progress_time = ros::Time::now();
                        moving_started = true;
                    } else if (moving_started && (ros::Time::now() - last_progress_time).toSec() > 1.5f) {
                        ROS_WARN("[corner] turn stalled (no >2deg physical rotation 1.5s at %.0f deg; motor stall / odom-imu stale?) -> stop",
                                 delta * RAD_TO_DEG);
                        break;
                    } else if (!moving_started && (ros::Time::now() - turn_t0).toSec() > 2.5f) {
                        ROS_WARN("[corner] turn never started (no physical rotation in 2.5s; motor stall?) -> stop");
                        break;
                    }
                    // 发左转命令(angular>0 = 左转, 见驱动电机混合); 近目标降 fine 速; 持续 20Hz 发布喂看门狗
                    float w = ((target_rad - delta) < slow_rad) ? config_corner_turn_omega_fine_
                                                                : config_corner_turn_omega_;
                    geometry_msgs::Twist tc;
                    tc.linear.x = 0.0f; tc.linear.y = pump_speed; tc.angular.z = w;
                    cmd_vel_pub_.publish(tc);
                    ros::Duration(0.05).sleep();
                }

                // 停转: 持续发零(非盲 sleep)稳定
                {
                    ros::Time q0 = ros::Time::now();
                    while (ros::ok() && (ros::Time::now() - q0).toSec() < 0.3) {
                        geometry_msgs::Twist z;
                        z.linear.x = 0.0f; z.linear.y = pump_speed; z.angular.z = 0.0f;
                        cmd_vel_pub_.publish(z);
                        ros::spinOnce();
                        ros::Duration(0.05).sleep();
                    }
                    updateCurrentPose();
                }

                // ---------- LOCK + RESET: 转完即以当前融合航向重锁基准, 复位内环+兜底零点+软启动 ----------
                // 重锁直接用 IMU 转完那刻的融合航向(对抗 odom 漂移它最可信), 不靠超声平行精修(拐角凹角畸变会锁到错墙, 评审 BLOCKER);
                // 新墙的距墙/平行对齐交给恢复直行后的串级外环(已加一致性闸门 + 软启动渐入)自然完成。
                bool f_relock = (hs_have_yaw_ &&
                                 (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);
                if (f_relock) { yaw_base = hs_yaw_rad_; yaw_base_locked = true; }
                else { ROS_WARN("[corner] relock: IMU not fresh -> keep old yaw_base (or odom fallback)"); }
                imu_fresh_prev = f_relock;
                hs_I_ = 0.0; hs_d_filt_ = 0.0; hs_e_prev_ = 0.0;
                hs_t_prev_sample_ = f_relock ? hs_yaw_stamp_ : ros::Time::now();
                hs_new_sample_ = false;
                total_yaw_change = 0.0f; last_theta = current_pose_.theta;  // 刷新 odom 兜底零点(评审 MEDIUM)
                // 不重置 start_time: 它是整条腿的全局超时锚(主循环 timeout); 每拐角重置会抹掉总运行时长上限,
                // 跨多拐角可在水中无限超时运行(评审 MEDIUM)。拐角自身由 corner_turn_timeout 单独约束。
                correction_enabled = false; correction_start_time = ros::Time::now();
                current_angular_correction = 0.0f;
                ramp_start_time = ros::Time::now();   // 速度 + 外环 C 双软启动重置
                front_wall_detect_count = 0;
                wall_avoiding = false;
                // corner_armed 此处刻意不复位: 转向欠转时老前墙仍<trig, 须靠上方 !has_front_wall 数据驱动重新武装(勿在此加 =true 否则重开欠转 U 转洞)

                // 恢复前进速度
                {
                    std::lock_guard<std::mutex> lock(keyboard_mutex_);
                    keyboard_sim_.target_vx = cmd_speed;
                    keyboard_sim_.current_vx = cmd_speed;
                    keyboard_sim_.target_angular = 0.0f;
                    keyboard_sim_.idle_counter = 0;
                    keyboard_sim_.key_pressed = true;
                }
                ROS_INFO("[corner] done: walked %.2fm remaining %.2fm theta=%.1fdeg -> resume straight (new wall via cascade)",
                         total_accumulated_distance, remaining, current_pose_.theta * RAD_TO_DEG);
                continue;
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

        // ==================== 提前关泵(仅单段; 整段闩锁时不提前撤泵) ====================
        if (pump_on && !pump_closed_early && !run_pump_latched_ && remaining < 0.3f && remaining > 0) {
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
            // 陈旧看门狗 config_wall_stale_sec_(ROS 参数 wall_stale_sec): 须≥右侧刷新周期; 超声实测 MCU 推流 ~10Hz=0.1s/路, 默认 0.5s
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
            // 一致性闸门: 两侧都有效但 |右后-右前| 过大 = 畸变(拐角凹角/一传感器照旧墙一照新墙/物理不可能)→ 弃外环, 纯 IMU 守航向。
            bool ultra_consistent = (fabs(rear - front) < config_wall_consistency_m_);
            bool ultra_ok = front_valid && rear_valid && ultra_fresh && ultra_consistent;
            bool imu_fresh = (hs_have_yaw_ &&
                              (ros::Time::now() - hs_yaw_stamp_).toSec() < config_heading_yaw_timeout_);

            // ---- 外环: 距墙/平行 → 有界航向偏置 C ----
            // C 单位 = rad(等效航向偏置, 叠进航向误差 e_total); K_DIST[rad/m] K_PAR[rad/m]; off_max 0.10≈5.7°(安全上界)。仅新鲜有效超声时刷新。
            float d = -1.0f, e_par = 0.0f, C = 0.0f;
            if (ultra_ok) {
                d = 0.5f * (front + rear);              // 距右墙
                e_par = rear - front;                   // >0:机头朝墙(欠转)
                float e_dist = config_wall_align_ - d;  // >0:离墙太近→需离墙
                C = (float)config_wall_k_dist_ * e_dist + (float)config_wall_k_par_ * e_par;
                C = std::max(-(float)config_wall_off_max_, std::min((float)config_wall_off_max_, C));
                // 外环软启动(评审 BLOCKER): 起步/拐角后 softstart_sec 内把【双侧距墙/平行】外环权限 0→full 渐入,
                // 治"起步距墙误差大→C 瞬间钉满 off_max 叠加 omega_limit=0.30 打舵"。只软此双侧 C, 不软单侧捕获(评审 MEDIUM)、不动内环 PD(Q3 已验)。
                if (config_softstart_sec_ > 0.01f) {
                    float cr = (float)(ros::Time::now() - ramp_start_time).toSec() / config_softstart_sec_;
                    C *= std::min(1.0f, std::max(0.0f, cr));
                }
            } else if (front_valid != rear_valid) {
                C = rear_valid ? -0.05f : 0.05f;        // 单侧有墙: 轻偏重捕获(右前丢→右转(-); 右后丢→左转(+)); 全权限不被软启动压制, 尽快回墙
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
        cmd.linear.y = (run_pump_latched_ || pump_on) ? pump_speed : 0.0f;  // 整段闩锁时即使单段 pump_on 翻假也持泵
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
    if (pump_on && !run_pump_latched_) {  // 整段闩锁时段末不撤泵(交由闩锁跨段持泵)
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
// ==================== 下水自启: 旋转找侧壁 → 对齐 → 进入沿边 ====================
// 安全铁律: 本相位纯原地偏航(linear.x=0 绝不前进), 全程 ≤0.05s 持续 publish(doc 沿边轨无 /cmd_vel 看门狗→失败即 stop() 发零, 不留残命令)。
// 朝墙角 = 右得墙但前被挡 → 拒绝该航向继续转; 仅"右两路有效一致【且前净空】"才锁定 → 起步绝不冲墙(对应用户硬要求)。
// 采集用"停-核验": 慢扫到候选航向先停转再核验, 使扫速不影响确认(镜面壁窄法向窗也不会被扫过漏掉)。
bool MoveController::findWallAndFollow(float follow_distance, float timeout) {
    if (move_state_ != 0) {
        ROS_ERROR("[findwall] busy (move_state=%d) -> reject", move_state_);
        return false;
    }
    const float DEG2RAD = M_PI / 180.0f;
    const float RAD2DEG = 180.0f / M_PI;

    ROS_WARN("========================================");
    ROS_WARN("[findwall] bootstrap: rotate -> acquire RIGHT wall -> enter wall-follow");
    ROS_WARN("  follow=%.2fm follow_timeout=%.1fs | omega=%.2f left=%d confirm=%.2fs max_rot=%.0fdeg acq_timeout=%.1fs",
             follow_distance, timeout, config_findwall_omega_, (int)config_findwall_turn_left_,
             config_findwall_confirm_sec_, config_findwall_max_rot_deg_, config_findwall_timeout_);
    ROS_WARN("  acquire: right_front&right_rear both in(0.05,5.0)m & |rear-front|<%.2fm(fresh) & front min>=%.2fm(eff; =wall_align when corner_use_wall_dist)",
             config_wall_consistency_m_,
             (config_corner_use_wall_dist_ ? config_wall_align_ : config_corner_front_trigger_m_));
    ROS_WARN("========================================");

    disabling_timer_ = true;   // 接管 /cmd_vel 发布, 抑制键盘定时器(同 walkStraight)
    move_state_ = 1;

    if (!waitForOdom(5.0f)) {  // 需 odom 测累计转角(旋转上限兜底)
        ROS_ERROR("[findwall] wait odom timeout -> abort (no motion committed)");
        stop(); disabling_timer_ = false; move_state_ = 2;
        return false;
    }
    for (int i = 0; i < 5 && ros::ok(); i++) { ros::spinOnce(); ros::Duration(0.05).sleep(); }
    updateCurrentPose();

    const float max_rot_rad = config_findwall_max_rot_deg_ * DEG2RAD;
    const float omega = config_findwall_turn_left_ ? config_findwall_omega_ : -config_findwall_omega_;
    // 泵(linear.y): ACQUIRE 为纯偏航侦察 → 旋转/停稳全程【强制关泵(0)、与是否闩锁无关】(评审 MEDIUM): 杜绝泵推力在原地转时
    //   产生前向漂移、把离壁很近的机体顶上墙(本分支无 /cmd_vel 看门狗)。不改 run_pump_latched_ 闩锁态本身——失败 stop() 与
    //   交回 walkStraight 时各自按 live 闩锁恢复泵, 故此处恒置 0 不扰闩锁。
    const float pump = 0.0f;

    ros::Time t0 = ros::Time::now();
    float     rotated = 0.0f;
    float     last_theta = current_pose_.theta;
    bool      verifying = false;
    ros::Time verify_start;
    bool      acquired = false;
    float     acq_d = -1.0f, acq_front = 999.0f;
    // 进度看门狗状态(对齐拐角块 cpp 起转/停转见证): 命令在转却无物理推进 → 快速失败
    bool      moving_started    = false;
    float     last_progress_rot = 0.0f;
    ros::Time last_progress_t   = t0;

    while (ros::ok()) {
        updateCurrentPose();
        // 转角累计【无条件每拍, 含核验停转期】: odom 真实航向变化都计入(评审 HIGH: 原仅 SWEEP 计、漏核验期漂移→1.5圈上限偏乐观)。
        // 仅作旋转上限兜底; dth<0.5rad 滤角度回卷(±π)突跳(正常每拍≈3°)。
        {
            float dth = fabs(normalizeAngle(current_pose_.theta - last_theta));
            if (dth < 0.5f) rotated += dth;
            last_theta = current_pose_.theta;
            if (rotated > last_progress_rot + 2.0f * DEG2RAD) {   // 物理推进高水位(进度看门狗用)
                last_progress_rot = rotated;
                last_progress_t   = ros::Time::now();
                moving_started    = true;
            }
        }
        // 快照右侧+前方超声(与 ultraCallback 写侧同锁规则)
        float rf, rr, fl, fr; ros::Time ustamp;
        {
            std::lock_guard<std::mutex> lk(sensor_mutex_);
            rf = wall_sensors_.ultra_right_front;
            rr = wall_sensors_.ultra_right_rear;
            fl = wall_sensors_.infra_front_left;
            fr = wall_sensors_.infra_front_right;
            ustamp = ultra_right_stamp_;
        }
        bool rf_valid = (rf > 0.05f && rf < 5.0f);
        bool rr_valid = (rr > 0.05f && rr < 5.0f);
        bool ufresh   = (!ustamp.isZero() &&
                         (ros::Time::now() - ustamp).toSec() < config_wall_stale_sec_);
        bool consistent = (fabs(rr - rf) < config_wall_consistency_m_);
        float front_dist = std::min(fl, fr);
        // 前净空: front min 不落在 (0.05, trig) 危险带; 999=无回波=净空。trig 须同 walkStraight 拐角触发口径
        // (corner_use_wall_dist? wall_align : corner_front_trigger_m), 否则净空阈(旧钉死1.5)< 拐角阈(3.0)时,
        // 找墙会在 front∈[1.5,3.0) 处判净空锁定 -> 交回沿边 has_front_wall=true 立即误触发拐角、把刚找到的墙转开。
        float front_trig = config_corner_use_wall_dist_ ? config_wall_align_ : config_corner_front_trigger_m_;
        bool front_clear = !(front_dist > 0.05f && front_dist < front_trig);
        bool right_hint = (rf_valid || rr_valid) && ufresh;             // 触发核验(宽: 任一路先现回波)
        bool right_ok   = rf_valid && rr_valid && ufresh && consistent; // 锁定判据(严: 与外环 ultra_ok 同口径)

        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;        // 铁律: 找墙阶段绝不前进
        cmd.linear.y = pump;
        cmd.angular.z = 0.0f;

        if (!verifying) {
            // ---- SWEEP: 原地偏航(转角累计/进度看门狗已在循环顶部统一处理) ----
            cmd.angular.z = omega;
            if (right_hint && front_clear) {  // 右有墙且前净空 = 候选位姿 → 停转核验
                verifying = true; verify_start = ros::Time::now();
                cmd.angular.z = 0.0f;
                ROS_INFO("[findwall] candidate @%.0fdeg (rf=%.2f rr=%.2f front=%.2f) -> stop & verify",
                         rotated * RAD2DEG, rf, rr, front_dist);
            } else if (right_hint && !front_clear) {
                ROS_INFO_THROTTLE(1.0, "[findwall] right wall but front blocked(%.2fm)=facing corner -> keep rotating", front_dist);
            } else {
                ROS_INFO_THROTTLE(1.0, "[findwall] sweep @%.0fdeg (rf=%.2f rr=%.2f front=%.2f)",
                                  rotated * RAD2DEG, rf, rr, front_dist);
            }
        } else {
            // ---- VERIFY: 停转, 严判据须持续 confirm_sec 才锁定; 中断则回 SWEEP ----
            last_progress_t = ros::Time::now();   // 核验=有意停转: 每拍重置进度看门狗计时, 不误判为电机 stall
            cmd.angular.z = 0.0f;
            if (right_ok && front_clear) {
                if ((ros::Time::now() - verify_start).toSec() >= config_findwall_confirm_sec_) {
                    acquired = true; acq_d = 0.5f * (rf + rr); acq_front = front_dist;
                    break;   // 末态零速由下方停转稳定段发出
                }
            } else {
                verifying = false; cmd.angular.z = omega;   // 核验中断(右不稳/不一致/前被挡)→继续转
                ROS_INFO_THROTTLE(1.0, "[findwall] verify broke (right_ok=%d front_clear=%d) -> resume sweep",
                                  (int)right_ok, (int)front_clear);
            }
        }

        cmd_vel_pub_.publish(cmd);

        // 上限兜底(不盲走/不冲墙): 转满上限或超时仍未锁定 → 失败
        if (rotated >= max_rot_rad) {
            ROS_ERROR("[findwall] rotated %.0fdeg (cap %.0f) without acquiring a usable side wall -> abort",
                      rotated * RAD2DEG, config_findwall_max_rot_deg_);
            break;
        }
        if ((ros::Time::now() - t0).toSec() > config_findwall_timeout_) {
            ROS_ERROR("[findwall] acquire timeout %.1fs -> abort", config_findwall_timeout_);
            break;
        }
        // 进度看门狗(评审 CONFIRMED-HIGH + MEDIUM): 命令在转却无物理推进 → 快速失败, 不在近壁久转。
        //   覆盖: ① omega 可能低于水下电机起转阈(转不动)→ 原本要空转到 25s 超时才停; ② odom 冻结使旋转上限永不触发。
        //   核验停转期已每拍重置 last_progress_t, 故不会把"有意停转"误判 stall。失败路径仍 stop()+不前进, 安全。
        if (moving_started && (ros::Time::now() - last_progress_t).toSec() > 1.5) {
            ROS_ERROR("[findwall] rotation STALLED (no >2deg yaw for 1.5s; motor below water start threshold? raise findwall_omega) -> abort");
            break;
        } else if (!moving_started && (ros::Time::now() - t0).toSec() > 2.5) {
            ROS_ERROR("[findwall] rotation NEVER STARTED (no physical yaw in 2.5s; omega too low / motor stall) -> abort");
            break;
        }

        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    // 停转稳定: 持续发零(非盲 sleep)
    {
        ros::Time q0 = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - q0).toSec() < 0.3) {
            geometry_msgs::Twist z;
            z.linear.x = 0.0f; z.linear.y = pump; z.angular.z = 0.0f;
            cmd_vel_pub_.publish(z);
            ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
        updateCurrentPose();
    }

    if (!acquired) {
        stop();
        disabling_timer_ = false;
        move_state_ = 2;
        ROS_ERROR("[findwall] FAILED to acquire a usable side wall (no forward motion committed). Reposition robot near a wall and retry.");
        return false;
    }

    ROS_WARN("[findwall] ACQUIRED right wall: d~%.2fm, front clear(%.2fm). Hand off to wall-follow for %.2fm.",
             acq_d, acq_front, follow_distance);
    // 交回沿边串级: 复位 move_state_ 让 walkStraight 守卫通过(disabling_timer_ 保持真, walkStraight 内自再置真→末了置假)。
    // 用户要的"移动修正右侧超声距离"由 walkStraight 外环在前进中完成(wall_align-d 折成有界航向偏置渐收敛),
    // 拐角块继续防前墙撞击; 复用成品、不另写易脆盲横移闭距控制器。
    move_state_ = 0;
    return walkStraightWithWallAvoidanceAdvanced(follow_distance, config_speed_, timeout);
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

     // ========== 水泵: 运行级闩锁优先(跨段/扫描停顿持泵), 否则按行走状态(单段回退) ==========
    if (run_pump_latched_) {  // 整段沿边任务: 空闲(含扫描停顿)也持泵, 电机仍为 0
        cmd.linear.y = user_config_pump_speed_;  // 闩锁下即 run 级泵速(为0则本就该关泵)
    } else if (move_state_ == 1) {  // 行走中(单段回退)
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
        // 新鲜度看门狗(陈旧判定用): 每次收到 /ultra_data(话题稳定 ~20Hz 转发、底层 MCU 推流 ~10Hz)即更新时间戳,
        //   与原始值是否变化【解耦】。否则平直墙面下真实距离恒定(可数秒不变)会被误判 stale → 退 odom 盲走(评审 HIGH-1 实测 7×误回退)。
        ultra_right_stamp_ = ros::Time::now();
        // seq(拐角 stop-and-settle 用): 仅在右侧原始值真变化时推进, 避免对 ZOH 重复值重复精修(拐角旋转时值必变, 不受影响)。
        int raw_rf = msg->data[2];
        int raw_rr = msg->data[3];
        if (raw_rf != last_ultra_raw_rf_ || raw_rr != last_ultra_raw_rr_) {
            ultra_right_seq_++;
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

void MoveController::pumpEnableCallback(const std_msgs::Bool::ConstPtr& msg) {
    // 运行级水泵闩锁。与主循环 spinOnce 同线程执行(单回调队列), 无需加锁。
    // true=整段持泵(跨段/扫描停顿不撤泵); false=撤闩回落单段策略。ASCII 打印(机上无中文字库)。
    run_pump_latched_ = msg->data;
    ROS_INFO("pump latch %s", run_pump_latched_ ? "ON" : "OFF");
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
    // 整段闩锁时停车保泵(段末/扫描停顿不撤泵, 消除每段末 ~0.2s 断泵抖动); 否则全停(含泵)。
    // 仅段末(disabling_timer_ 仍真、空闲定时器关)调用 → 与定时器不并发, 不构成双写。
    stop_cmd.linear.y = run_pump_latched_ ? user_config_pump_speed_ : 0.0f;
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
    
    // 文件名带运行时间+关键整定参数(wall_align/k_dist/k_par), 便于复盘对应整定 (用户要求"按运行时间+参数")
    current_log_filename_ = dir + log_type + "_" + std::string(time_buf) +
                            "_dist" + std::to_string(target_distance) +
                            "_wa" + std::to_string(config_wall_align_) +
                            "_kd" + std::to_string(config_wall_k_dist_) +
                            "_kp" + std::to_string(config_wall_k_par_) + ".csv";
    
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

