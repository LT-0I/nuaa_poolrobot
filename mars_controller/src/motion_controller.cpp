#include <motion_controller.h>
#include <cmath>
#include <iomanip>
#include <csignal>      // [heading-straight-pd] signal/SIG_IGN/SIGINT/SIGTERM；SIGHUP/SIGPIPE 由 Linux <signal.h> 提供

// [heading-straight-pd] 版本闸门：改功能必升版本，操作单 grep 自判
#define CODE_VERSION "heading-straight-pd-v7-defaults-20260621"

// [heading-straight-pd] 自管 SIGINT/SIGTERM/SIGHUP：退出前补发零速，防驱动保持最后指令（驱动无看门狗，D-29）
static volatile sig_atomic_t g_hs_stop = 0;
static void hsSigintHandler(int) { g_hs_stop = 1; }

MotionController::MotionController()
{
    key_t key = ftok(PATH_NAME, PROJ_ID);
    int shmid = shmget(key, sizeof(ares_cmd), 0666 | IPC_CREAT);
    ares_cmd = (AresCmd *)shmat(shmid, 0, 0);

    

    trace_points_ok = false;
    closest_point_index = 0;
    step_index = 0;

    motion_pose_state.x = 0.0f;
    motion_pose_state.y = 0.0f;
    motion_pose_state.theta = 0.0f;

    motion_twist_cmd.linear.x = 0.0f;
    motion_twist_cmd.linear.y = 0.0f;
    motion_twist_cmd.angular.z = 0.0f;

    ares_cmd->mars_os_cmd = MARS_BOOT_OS_CMD;
    mars_state.mars_ctlmode_sta = MARS_SYSTEM_MODE_STA;
    ares_cmd->mars_ctlmode_cmd = MARS_GET_SYSTEM_CMD;

    ares_cmd->task_command.mission = CMD_NO_TASK;
    task_state.mission = STATE_NO_TASK;

    flag_loctaion = false;
    odom_jounery = 0.0;
    odom_angle = 0.0;
    flag_new_paths = true;
    base_to_curr = false;
    ref_flag = false;

    location_states = LOCATION_STATE_NONE;
    stop_flag = false;
    ares_cmd->avoid_action_cmd.aviod_match_cmd = NO_OBSTACLE;
    ares_cmd->avoid_action_cmd.aviod_scan_cmd = NO_OBSTACLE;

    control_params.x.Kp = 0.6;
    control_params.x.Ki = 1.0;
    control_params.x.Kd = 0.0;
    control_params.y.Kp = 6.0;
    control_params.y.Ki = 0.95;
    control_params.theta.Kp = 1.0;
    control_params.theta.Ki = 1.2;
    control_params.theta.Kd = 0.0;

    sweep_action = STRAIGHT_ON;
    find_wall_action = ROTATION_IN_PLACE;
    session = SES_WALL_STATE;
    cover_action = SEARCH_START_POINT;
    climb_wall_action = STA_0;

    wheel_pose_state.theta = 0.0;

    // 电机补偿初始化
    motor_compensation_left = 1.0;
    motor_compensation_right = 1.0;
    motor_calibration_factor = 1.0;
    is_calibrating = false;
    debug_loop_counter = 0;
    last_debug_time = ros::Time::now();

    // 尝试从参数服务器加载历史补偿
 
 
    
    // 初始化校准数据结构
    calibration_data.straight_yaw_start = 0.0;
    calibration_data.straight_distance_start = 0.0;
    calibration_data.yaw_drift_sum = 0.0;
    calibration_data.sample_count = 0;

    // 尝试从参数服务器加载历史补偿
    nh_.getParam("motor_compensation_left", motor_compensation_left);
    nh_.getParam("motor_compensation_right", motor_compensation_right);
    
    ROS_INFO("Motor compensation: L=%.3f, R=%.3f", 
             motor_compensation_left, motor_compensation_right);

    if (nh_.getParam("map_file_path", map_file_path))
    {
        cout << "map_file_path " << map_file_path.c_str() << endl;
        ROS_INFO("MAP file path: %s", map_file_path.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get MAP file path parameter");
    }
}
MotionController::~MotionController()
{
    exit(0);
}

void MotionController::rosNode()
{
    int wheel_resolution;
    vector<double> xPid, yPid, thetaPid;

    if (nh_.getParam("xPid", xPid))
    {
        ROS_INFO("-> X Pid          : [ %.3f  %.3f  %.3f ]", xPid[0], xPid[1], xPid[2]);
        memcpy(&control_params.x, &xPid[0], sizeof(PID));
    }
    else
    {
        ROS_INFO("-> Error: didn't find xPid");
    }

    if (nh_.getParam("yPid", yPid))
    {
        ROS_INFO("-> Y Pid           : [ %.3f  %.3f  %.3f]", yPid[0], yPid[1], yPid[1]);
        memcpy(&control_params.y, &yPid[0], sizeof(PID));
    }
    else
    {
        ROS_INFO("-> Error: didn't find yPid");
    }

    if (nh_.getParam("thetaPid", thetaPid))
    {
        ROS_INFO("-> theta Pid      : [ %.3f  %.3f  %.3f]", thetaPid[0], thetaPid[1], thetaPid[2]);
        memcpy(&control_params.theta, &thetaPid[0], sizeof(PID));
    }
    else
    {
        ROS_INFO("-> Error: didn't find thetaPid");
    }

    if (nh_.getParam("obstacle_rect", obstacle_rect_))
    {
    }
    else
    {
        ROS_INFO("-> Error: didn't find obstacle_rect");
    }

    nh_.param("limit_speed", motion_params.limit_speed, 0.2);
    nh_.param("limit_omega", motion_params.limit_omega, 0.3);

    wheelTwist_sta_sub_ = nh_.subscribe("/wheelTwist_sta", 100, &MotionController::handleWheelTwistStates, this);
    sub_wheel_pose = nh_.subscribe("/wheelPose_sta", 100, &MotionController::WheelPoseCallback, this);
    sub_scan_1 = nh_.subscribe("/lidar1/scan", 100, &MotionController::scanCallback_1, this);
    sub_scan_2 = nh_.subscribe("/lidar2/scan", 100, &MotionController::scanCallback_2, this);
    sub_locationState = nh_.subscribe("/location_state", 100, &MotionController::LocationStateCallback, this);
    sub_vslam_pose = nh_.subscribe("/robot_pose", 100, &MotionController::vslamPoseCallback, this);
    dist_datas_sub_ = nh_.subscribe("/ultra_data", 100, &MotionController::distDatasCallback, this);

    motionTwist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("autoVel_cmd", 1, true);

      // 添加校准服务
    calibrate_service = nh_.advertiseService("calibrate_motors", 
        &MotionController::handleCalibrateRequest, this);
    
    ROS_INFO("Motor calibration service available: /calibrate_motors");

    // ===================== [heading-straight-pd] 航向保持走直线模式装配 =====================
    // 全部参数走私有句柄 ~ → 命令行 _name:=value 可覆盖、免重编。
    std::string control_mode;
    pnh_.param<std::string>("control_mode", control_mode, std::string("legacy"));
    // [rect-z] heading 家族: heading_straight(原单段) / rectangle(矩形) / zigzag(Z弓字形)
    // 三者共用同一套 PD 直行+航向保持机理; rectangle/zigzag 仅在段末加原地转向(段序列)。
    heading_mode_ = (control_mode == std::string("heading_straight") ||
                     control_mode == std::string("rectangle") ||
                     control_mode == std::string("zigzag"));
    if      (control_mode == std::string("rectangle")) path_mode_ = PATH_RECT;
    else if (control_mode == std::string("zigzag"))    path_mode_ = PATH_ZIGZAG;
    else                                               path_mode_ = PATH_SINGLE;

    pnh_.param<double>("heading_kp",        h_kp_,               1.0);   // 现场实测默认: PD 比例
    pnh_.param<double>("heading_kd",        h_kd_,               1.0);   // 现场实测默认: PD 微分(输出仍受 omega_limit 钳)
    pnh_.param<double>("heading_ki",        h_ki_,               0.0);
    pnh_.param<double>("i_clamp",           h_i_clamp_,          0.05);
    pnh_.param<double>("straight_speed",    h_speed_,            0.25);  // 现场实测默认: 履带打滑临界以内
    pnh_.param<double>("omega_limit",       h_omega_limit_,      0.1);
    pnh_.param<double>("pump_speed",        h_pump_speed_,       0.0);   // 泵占空比 0~1; 默认0(陆地安全); 下水正式跑 _pump_speed:=0.2
    pnh_.param<double>("settle_seconds",    h_settle_s_,         20.0);
    pnh_.param<double>("still_max_pp_deg",  h_still_pp_deg_,     0.5);
    pnh_.param<double>("run_seconds",       h_run_seconds_,      60.0);
    pnh_.param<double>("run_distance",      h_run_distance_,     0.0);
    pnh_.param<double>("pause_every_s",     h_pause_every_s_,    150.0);
    pnh_.param<double>("pause_duration_s",  h_pause_duration_s_, 4.0);
    pnh_.param<double>("softstart_seconds", h_softstart_s_,      2.0);   // 缓启动斜坡(s); 0=关闭(瞬跳)
    pnh_.param<double>("yaw_timeout",       h_yaw_timeout_,      0.5);
    pnh_.param<double>("deadband_u",        h_deadband_u_,       0.0);
    pnh_.param<int>   ("heading_sign",      h_sign_,             1);
    pnh_.param<bool>  ("drift_comp",        h_drift_comp_,       false);
    pnh_.param<std::string>("yaw_topic",    h_yaw_topic_,        std::string("/imu_heading"));
    pnh_.param<std::string>("cmd_topic",    h_cmd_topic_,        std::string("/cmd_vel"));
    pnh_.param<std::string>("log_path",     h_log_path_,         std::string("/tmp/heading_straight_log.csv"));
    h_sign_ = (h_sign_ >= 0) ? 1 : -1;

    // [rect-z] 段序列(矩形/Z) + 原地转向参数。仅 rectangle/zigzag 用; 启动前 rosparam set, 构造期只读一次。
    pnh_.param<double>("rect_side_a",      rect_side_a_,        2.0);
    pnh_.param<double>("rect_side_b",      rect_side_b_,        rect_side_a_);   // 默认正方形
    pnh_.param<int>   ("rect_laps",        rect_laps_,          1);
    pnh_.param<double>("zig_leg",          zig_leg_,            3.0);
    pnh_.param<double>("zig_shift",        zig_shift_,          0.5);
    pnh_.param<int>   ("zig_legs",         zig_legs_,           4);
    pnh_.param<int>   ("turn_dir",         turn_dir_,           1);
    pnh_.param<double>("turn_deg",         turn_deg_,           90.0);
    pnh_.param<double>("turn_omega",       hs_turn_omega_,      0.20);
    pnh_.param<double>("turn_omega_fine",  hs_turn_omega_fine_, 0.08);
    pnh_.param<double>("turn_settle_s",    hs_turn_settle_s_,   1.0);
    pnh_.param<double>("turn_timeout_s",   hs_turn_timeout_s_,  25.0);
    pnh_.param<double>("turn_stall_s",     hs_turn_stall_s_,    3.0);
    double turn_tol_deg, turn_fine_deg, turn_progress_eps_deg;
    pnh_.param<double>("turn_tol_deg",          turn_tol_deg,          3.0);
    pnh_.param<double>("turn_fine_deg",         turn_fine_deg,         15.0);
    pnh_.param<double>("turn_progress_eps_deg", turn_progress_eps_deg, 2.0);
    turn_dir_             = (turn_dir_ >= 0) ? 1 : -1;
    hs_turn_tol_          = turn_tol_deg          * PI_M / 180.0;
    hs_turn_fine_band_    = turn_fine_deg         * PI_M / 180.0;
    hs_turn_progress_eps_ = turn_progress_eps_deg * PI_M / 180.0;

    // [rect-z] 段几何/转角参数防御性钳位(评审收敛 MINOR/NIT): 负边长/横移会让 hypot 距离判据首拍即真→空跑收尾;
    // turn_deg 超界 wrap 后日志误导; settle 过大会让段间永久停滞且无超时(评审 B MAJOR-2)。均钳到安全区。
    if (rect_side_a_ < 0.0) rect_side_a_ = 0.0;
    if (rect_side_b_ < 0.0) rect_side_b_ = 0.0;
    if (zig_leg_     < 0.0) zig_leg_     = 0.0;
    if (zig_shift_   < 0.0) zig_shift_   = 0.0;
    if (turn_deg_ <   0.0) turn_deg_ =   0.0;
    if (turn_deg_ > 180.0) turn_deg_ = 180.0;
    if (hs_turn_settle_s_  < 0.0)  hs_turn_settle_s_  = 0.0;
    if (hs_turn_settle_s_  > 10.0) hs_turn_settle_s_  = 10.0;
    if (hs_turn_timeout_s_ < 1.0)  hs_turn_timeout_s_ = 1.0;
    if (hs_turn_stall_s_   < 0.5)  hs_turn_stall_s_   = 0.5;

    // 命令行速度/角速度硬钳, 防操作单笔误一位(0.4→4.0)在控制器侧就直达 /cmd_vel。
    // straight_speed 上限 0.4: 为测"加缓启动后 >0.3(0.3~0.4) 能否压住履带打滑"开放到 0.4; 0.4 直行峰值
    // 轨速 ~0.59 m/s、电机命令 ~136, 正常工作区内, 远未触及驱动兜底阈值(max_track_speed=1.0/max_motor_cmd=300),
    // 且笔误(0.4→4.0)被钳到 0.4。下界 0 保留 _straight_speed:=0 的原地 sign 测试路径。omega_limit 仍钳 0.3
    // (差速航向权限充裕、与前进速度解耦)。横幅打印的是钳后值(grep 自判按钳后值)。
    {
        const double sp0 = h_speed_, om0 = h_omega_limit_, pm0 = h_pump_speed_;
        if (h_speed_       < 0.0) h_speed_       = 0.0;
        if (h_speed_       > 0.4) h_speed_       = 0.4;
        if (h_omega_limit_ < 0.0) h_omega_limit_ = 0.0;
        if (h_omega_limit_ > 0.3) h_omega_limit_ = 0.3;
        if (h_pump_speed_  < 0.0) h_pump_speed_  = 0.0;   // 泵占空比钳 [0,1]
        if (h_pump_speed_  > 1.0) h_pump_speed_  = 1.0;
        if (h_softstart_s_ < 0.0)  h_softstart_s_ = 0.0;  // 缓启动斜坡 [0,10]s
        if (h_softstart_s_ > 10.0) h_softstart_s_ = 10.0;
        if (hs_turn_omega_      < 0.0) hs_turn_omega_      = 0.0;   // [rect-z] 转速钳 [0,0.3]，同 omega 安全上限
        if (hs_turn_omega_      > 0.3) hs_turn_omega_      = 0.3;
        if (hs_turn_omega_fine_ < 0.0) hs_turn_omega_fine_ = 0.0;
        if (hs_turn_omega_fine_ > hs_turn_omega_) hs_turn_omega_fine_ = hs_turn_omega_;   // 慢转速不超主转速
        if (sp0 != h_speed_ || om0 != h_omega_limit_ || pm0 != h_pump_speed_)
            ROS_WARN("[hs] param out of range, clamped: straight_speed %.3f->%.3f, omega_limit %.3f->%.3f, pump_speed %.3f->%.3f",
                     sp0, h_speed_, om0, h_omega_limit_, pm0, h_pump_speed_);
    }

    if (heading_mode_)
    {
        // CSV 默认路径加墙钟时间戳，避免连续两次运行覆盖同一文件（评审 LOW：防数据丢失）。
        if (h_log_path_ == "/tmp/heading_straight_log.csv") {
            char buf[80];
            snprintf(buf, sizeof(buf), "/tmp/heading_straight_log_%u.csv", (unsigned)ros::WallTime::now().sec);
            h_log_path_ = buf;
        }
        imu_heading_sub_ = nh_.subscribe(h_yaw_topic_, 50, &MotionController::imuHeadingCallback, this);
        heading_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(h_cmd_topic_, 10);
        hs_phase_        = HS_WAIT_YAW;
        hs_phase_start_  = ros::Time::now();
        hs_t_prev_loop_  = ros::Time::now();
        // 启动横幅：CODE_VERSION + 全部生效参数（grep 自判，防 _param 拼错静默回默认）
        ROS_WARN("==================== [heading-straight-pd] ENABLED ====================");
        ROS_WARN("CODE_VERSION=%s", CODE_VERSION);
        ROS_WARN("control_mode=%s  yaw_topic=%s  cmd_topic=%s",
                 (path_mode_ == PATH_SINGLE ? "heading_straight" : (path_mode_ == PATH_RECT ? "rectangle" : "zigzag")),
                 h_yaw_topic_.c_str(), h_cmd_topic_.c_str());
        ROS_WARN("Kp=%.4f Kd=%.4f Ki=%.4f i_clamp=%.3f heading_sign=%d",
                 h_kp_, h_kd_, h_ki_, h_i_clamp_, h_sign_);
        ROS_WARN("straight_speed=%.3f omega_limit=%.3f pump_speed=%.3f deadband_u=%.4f",
                 h_speed_, h_omega_limit_, h_pump_speed_, h_deadband_u_);
        ROS_WARN("settle_seconds=%.1f still_max_pp_deg=%.3f softstart_seconds=%.2f", h_settle_s_, h_still_pp_deg_, h_softstart_s_);
        ROS_WARN("run_seconds=%.1f run_distance=%.2f", h_run_seconds_, h_run_distance_);
        ROS_WARN("pause_every_s=%.1f pause_duration_s=%.1f drift_comp=%d",
                 h_pause_every_s_, h_pause_duration_s_, (int)h_drift_comp_);
        ROS_WARN("yaw_timeout=%.2f log_path=%s", h_yaw_timeout_, h_log_path_.c_str());
        // [rect-z] 矩形/Z 模式: 构建段表 + 打印生效参数 (ASCII, 机上无中文字库; grep 自判防 _param 拼错回默认)
        if (path_mode_ != PATH_SINGLE) {
            hsBuildSegments();
            ROS_WARN("PATH_MODE=%s segments=%d turn_dir=%d turn_deg=%.0f turn_omega=%.2f/%.2f tol=%.1fdeg settle=%.1fs timeout=%.1fs",
                     (path_mode_ == PATH_RECT ? "RECTANGLE" : "ZIGZAG"), (int)hs_segs_.size(),
                     turn_dir_, turn_deg_, hs_turn_omega_, hs_turn_omega_fine_,
                     hs_turn_tol_ * 180.0 / PI_M, hs_turn_settle_s_, hs_turn_timeout_s_);
            if (path_mode_ == PATH_RECT)
                ROS_WARN("RECT side_a=%.2f side_b=%.2f laps=%d (legs=%d)", rect_side_a_, rect_side_b_, rect_laps_, (int)hs_segs_.size());
            else
                ROS_WARN("ZIGZAG leg=%.2f shift=%.2f legs=%d (segments=%d)", zig_leg_, zig_shift_, zig_legs_, (int)hs_segs_.size());
            // [rect-z] 估算整条路径最短耗时, 据此校验 run_seconds 全局兜底是否够大(评审 A/B 收敛 MAJOR):
            // 默认 60s 会砍断多段路径=演示半途急停。est 用命令转速/速度算, 是下界(水下转向更慢), 故按 1.3x 留余量告警。
            double total_dist = 0.0;
            for (size_t i = 0; i < hs_segs_.size(); ++i) total_dist += hs_segs_[i].distance;
            double drive_t  = (h_speed_ > 0.0) ? (total_dist / h_speed_) : 0.0;
            int    n_turns  = (int)hs_segs_.size() - 1; if (n_turns < 0) n_turns = 0;
            double per_turn = ((hs_turn_omega_ > 0.0) ? (turn_deg_ * PI_M / 180.0 / hs_turn_omega_) : 0.0) + hs_turn_settle_s_;
            double est_t    = drive_t + (double)n_turns * per_turn;
            ROS_WARN("EST path time >= ~%.0fs (lower bound: drive %.0fs + %d turns x ~%.0fs; underwater turns slower).",
                     est_t, drive_t, n_turns, per_turn);
            if (h_run_seconds_ > 0.0 && h_run_seconds_ < est_t * 1.3)
                ROS_ERROR("!!! run_seconds=%.1f < 1.3x est %.0fs: GLOBAL backstop will CUT path mid-way -> raise _run_seconds (or set <=0, then ONLY Ctrl+C stops). !!!",
                          h_run_seconds_, est_t);
            ROS_WARN("NOTE: per-segment stop by distance; TURN keeps pump OFF (no forward); run_seconds=%.1f is GLOBAL backstop -> set big enough to cover whole path; CUT POWER guards all.", h_run_seconds_);
        }
        // 评审 LOW(Fable5)：两个自动停车兜底都被关→仅 Ctrl+C 可停(驱动无看门狗)，大字告警。
        if (h_run_seconds_ <= 0.0 && h_run_distance_ <= 0.0)
            ROS_WARN("!!! NO auto-stop (run_seconds<=0 and run_distance<=0): only Ctrl+C/SIGTERM stops, GUARD THE POWER !!!");
        ROS_WARN("======================================================================");
    }
    else
    {
        ROS_INFO("[hs] control_mode=%s (legacy behavior, heading mode OFF)", control_mode.c_str());
    }
}

void MotionController::LocationStateCallback(const std_msgs::Int8 &state)
{
    location_states = state.data;
    // cout << "location_states = " << location_states << endl;
}

void MotionController::vslamPoseCallback(const geometry_msgs::Pose2D vslam_pose_stamsg)
{
    // motion_pose_state = vslam_pose_stamsg;
    // cout << "motion_pose_state : " << motion_pose_state.x << "," << motion_pose_state.y << "," << motion_pose_state.theta <<endl;
}

// 订阅速度
void MotionController::handleWheelTwistStates(const geometry_msgs::Twist wheel_twist_stamsg)
{
    motion_twist_state.linear.x = wheel_twist_stamsg.linear.x;
    motion_twist_state.linear.y = 0;
    motion_twist_state.angular.z = wheel_twist_stamsg.angular.z;
    // cout << "motion_twist_state.angular.z = " << motion_twist_state.angular.z << endl;
}

// 订阅imu
void MotionController::WheelPoseCallback(const geometry_msgs::Pose2D msg)
{
    wheel_pose_state = msg;
}

// 后激光
void MotionController::scanCallback_1(const sensor_msgs::LaserScan &scanmsg)
{
    if (std::isfinite(scanmsg.ranges[450]))
    {
        // dis_r_b = scanmsg.ranges[450];
        // cout << "后激光距离 = " << scanmsg.ranges[450] << endl;
    }
}

// 前激光
void MotionController::scanCallback_2(const sensor_msgs::LaserScan &scanmsg)
{
    if (std::isfinite(scanmsg.ranges[1350]))
    {
        dis_l_f = scanmsg.ranges[1350];
        // cout << "前激光距离 = " << scanmsg.ranges[450] << endl;
    }

    if (std::isfinite(scanmsg.ranges[900]))
    {
        dis_f_l = scanmsg.ranges[900];
    }
}

void MotionController::distDatasCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    static int debug_count = 0;
    if (debug_count++ % 50 == 0) {  // 每50次回调输出一次
        ROS_INFO_THROTTLE(1.0, "=== Ultrasound Data ===");
        ROS_INFO_THROTTLE(1.0, "Raw [0]=%d(FR) [1]=%d(FL) [2]=%d(LF) [3]=%d(LB) (mm)", 
                         msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }

    
    double temp_dis_f_l, temp_dis_f_r;
    dis_l_b = msg->data[3] / 1000.0; // 左后
    dis_l_f = msg->data[2] / 1000.0; // 左前
    temp_dis_f_l = msg->data[1] / 1000.0; // 前左
    temp_dis_f_r = msg->data[0] / 1000.0; // 前右

    static int cnt_l = 0;
    static int cnt_r = 0;
    if(temp_dis_f_l > 0){
        cnt_l++;
    }else{
        cnt_l = 0;
    }

    if(temp_dis_f_r > 0){
        cnt_r++;
    }else{
        cnt_r = 0;
    }


    // cnt_l = (temp_dis_f_l > 0) ? cnt_l++ : 0;
    // cnt_r = (temp_dis_f_r > 0) ? cnt_r++ : 0;

    dis_f_l = (cnt_l < 3) ? 1 : temp_dis_f_l;
    dis_f_r = (cnt_r < 3) ? 1 : temp_dis_f_r;
    
    dis_f_min = (dis_f_r > dis_f_l) ? dis_f_l : dis_f_r; // 前传感器最小距离

    if (debug_count % 50 == 0) {
        ROS_INFO_THROTTLE(1.0, "Processed (m): FL=%.3f, FR=%.3f, LF=%.3f, LB=%.3f", 
                         dis_f_l, dis_f_r, dis_l_f, dis_l_b);
        ROS_INFO_THROTTLE(1.0, "Front min: %.3f m, Counters: L=%d, R=%d", 
                         dis_f_min, cnt_l, cnt_r);
    }


    if(dist_r_f_datas.size() == 10){
        if(dis_l_f < 10.0){
            dist_r_f_datas.erase(dist_r_f_datas.begin());
            dist_r_f_datas.push_back(dis_l_f);
        }
    }else{
        dist_r_f_datas.push_back(dis_l_f);
    }
    
 
}

void MotionController::traceZero()
{
    motion_twist_cmd.linear.x = 0.0;
    motion_twist_cmd.linear.y = 0.0;
    motion_twist_cmd.angular.z = 0.0;
    motionTwist_cmd_pub_.publish(motion_twist_cmd);
}

// 寻找墙面
void MotionController::FindWall()
{
    static double start_theta = wheel_pose_state.theta;
    static double end_theta = wheel_pose_state.theta + PI_M * 2 - 0.017;
    static double target_theta = 0.0;
    static double wall_theta = 0.0;
    static double min_dist = 99999;

    if (find_wall_action == ROTATION_IN_PLACE)
    {
        double err_theta = end_theta - wheel_pose_state.theta;
        err_theta = emath.angleConstrainTo2pi(err_theta);
        if (min_dist > dis_f_l)
        {
            min_dist = dis_f_l;
            target_theta = wheel_pose_state.theta;
        }

        // cout << "wheel_pose_state.theta = " << wheel_pose_state.theta << endl;
        // cout << "end_theta = " << end_theta << endl;
        // cout << "err_theta = " << err_theta << endl;
        // cout << "-------------" << endl;

        if (fabs(err_theta) < 0.05)
        {
            find_wall_action = NEAR_WALL_ROTATION;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            cout << "target_theta = " << target_theta << endl;
            cout << "min_dist = " << min_dist << endl;
        }
        else
        {
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta);
        }
    }
    else if (find_wall_action == NEAR_WALL_ROTATION)
    {
        double err_theta = target_theta - wheel_pose_state.theta;
        err_theta = emath.angleConstrain(err_theta);

        // cout << "wheel_pose_state.theta = " << wheel_pose_state.theta << endl;
        // cout << "target_theta = " << target_theta << endl;
        // cout << "err_theta = " << err_theta << endl;
        // cout << "-------------" << endl;

        if (fabs(err_theta) < 0.05)
        {
            find_wall_action = NEAR_WALL_STRAIGHT;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta);
        }
    }
    else if (find_wall_action == NEAR_WALL_STRAIGHT)
    {
        double target_dist = dis_f_l - 0.3;
        if (target_dist < 0.05)
        {
            find_wall_action = ALIGNED_WALL;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            wall_theta = wheel_pose_state.theta + PI_M / 2;
        }
        else
        {
            motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist);
            motion_twist_cmd.angular.z = 0.0;
        }
    }
    else if (find_wall_action == ALIGNED_WALL)
    {
        double err_theta = wall_theta - wheel_pose_state.theta;
        err_theta = emath.angleConstrain(err_theta);

        if (fabs(err_theta) < 0.05)
        {
            session = SES_SWEEP_STATE;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta);
        }
    }
    motionTwist_cmd_pub_.publish(motion_twist_cmd);
}

 

// 沿边跟踪
void MotionController::TraceWall()
{   
    ROS_INFO_THROTTLE(1.0, "=== TraceWall ===");
    ROS_INFO_THROTTLE(1.0, "Sweep Action: %d", sweep_action);
    ROS_INFO_THROTTLE(1.0, "Sensor data: LF=%.3f, LB=%.3f, FL=%.3f, FR=%.3f", 
                      dis_l_f, dis_l_b, dis_f_l, dis_f_r);


    const double dis = 0.0; // 目标距离
    

    double wave_dis_f_x = 0.07; // 右前方传感器纵向安装距离到车中心
    double wave_dis_b_x = 0.07; // 右后方传感器纵向安装距离
    double wave_dis_y = 0.150;  // 右侧传感器横向安装距离

    double pos_err_theta = 0.0;
    double pos_err_y = 0.0;

    static int cnt_20 = 0;

   

    double ab = wave_dis_f_x + wave_dis_b_x;
    double bc = dis_l_b - dis_l_f;
    // 计算斜边AC长度（勾股定理）
    double ac = sqrt(ab * ab + bc * bc);

    // 计算角A的正切值：tan(A) = 对边BC / 邻边AB
    double tanA = bc / ab;

    // 使用反正切计算弧度
    pos_err_theta = atan(tanA);
    // pos_err_theta = (dis_l_f - dis_r_b) * 15;
    // pos_err_y = dis_l_f - dis;
    pos_err_theta = 0.0;
  

    if(dis_l_b < 6.0 && dis_l_f < 6.0 && (fabs(dis_l_b - dis_l_f) < 0.1)){
        pos_err_y = dis_l_f - dis_l_b;
    }else{
        pos_err_y = 0.0;
    }

    if(fabs(pos_err_y) < 0.005){
        pos_err_y = 0.0;
    }

    ROS_INFO_THROTTLE(1.0, "Geometry: bc(LB-LF)=%.3f, theta=%.3f rad", bc, pos_err_theta);
    ROS_INFO_THROTTLE(1.0, "Pos errors: y=%.3f, theta=%.3f", pos_err_y, pos_err_theta);

    

    static double theta_detour = 0.0;
    static double dis_turning = 0.0; // 里程计
    static int dis_turning_cnt = 0;
    static int speed_cnt = 0;
    static int omega_cnt = 0;


     if (debug_loop_counter++ % 100 == 0) {  // 每1秒输出一次
        ROS_INFO_THROTTLE(1.0, "[TraceWall] State: %d, Pos_err: y=%.3f, θ=%.3f", 
                         sweep_action, pos_err_y, pos_err_theta);
    }

    if (sweep_action == STRAIGHT_ON)
    {
        motion_twist_cmd.linear.z = 10;
        if (speed_cnt < 100)
        {
            speed_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            double target_dist = 1.0;
            // cout << "dis_l_f = " << dis_l_f << endl;
            // cout << "target_dist = " << target_dist << endl;
            if (dis_f_min < 0.1)
            {
                 ROS_WARN("Detected obstacle! Front min distance: %.3f m", dis_f_min);
                ROS_WARN("Starting detour, target theta: %.3f rad", theta_detour);
                sweep_action = DETOUR;
                omega_cnt = 0;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                theta_detour = wheel_pose_state.theta + PI_M / 2;
            }
            else
            {
                if (dis_l_f > 0.4)
                {
                    dis_turning_cnt++;
                    cout << "dis_l_f = " << dis_l_f << endl;
                }
                else
                {
                    dis_turning_cnt = 0;
                }

                if (dis_turning_cnt > 10)
                {
                    cout << "STRA_BEFORE_TURNING" << endl;
                    sweep_action = STRA_BEFORE_TURNING;
                    dis_turning_cnt = 0;
                    dis_turning = wheel_pose_state.x;
                    motion_twist_cmd.angular.z = 0.0;
                }else{
                    double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.2);
                    double speed_error = (-1.0 / PI_M) * fabs(pos_err_theta * 3.0) + 1.0;
                    if (speed_error < 0.0)
                    {
                        speed_error = 0.0;
                    }
                    if (target_dist < 0.3)
                    {
                        target_dist = 0.3;
                    }
                    motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist) * (speed_error);
                    motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * omega_error);
                    // cout << "dis_l_b = " << dis_l_b << endl;
                    // cout << "dis_l_f = " << dis_l_f << endl;
                    // cout << "pos_err_y = " << pos_err_y << endl;
                    // cout << "fabs(dis_l_b - dis_l_f) = " << fabs(dis_l_b - dis_l_f) << endl;
                    // cout << "motion_twist_cmd.angular.z = " << motion_twist_cmd.angular.z << endl;
                    if(motion_twist_cmd.angular.z < 0.01 && motion_twist_cmd.angular.z > -0.01){
                        motion_twist_cmd.angular.z = 0.0;
                    }
                }
                // motion_twist_cmd.angular.z = 0.0;
            }
        }

        
    }
    else if (sweep_action == DETOUR)
    {
        motion_twist_cmd.linear.z = 30;
        double err_theta = theta_detour - wheel_pose_state.theta;
        err_theta = emath.angleConstrain(err_theta);
        // cout << "theta_detour = " << theta_detour << endl;
        // cout << "wheel_pose_state.theta = " << wheel_pose_state.theta << endl;
        // cout << "err_theta = " << err_theta << endl;
        // double err_theta = -PI_M / 2;
        ROS_INFO("DETOUR: target=%.3f, current=%.3f, error=%.3f, cmd_z=%.3f", 
             theta_detour, wheel_pose_state.theta, err_theta, motion_twist_cmd.angular.z);

        if (omega_cnt < 200)
        {
            omega_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            // cout << "=========================== = " << endl;
        }
        else
        {
            if (fabs(err_theta) < 0.03)
            {
                sweep_action = STRAIGHT_ON;
                dist_r_f_datas.clear();
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                speed_cnt = 0;
                // cout << "纵向偏差 = " << dis_f_l << endl;
                // cout << "角度偏差 = " << pos_err_theta << endl;
            }
            else
            {
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta) * 0.2;
                if(motion_twist_cmd.angular.z < 0.1 && motion_twist_cmd.angular.z > 0.0){
                    motion_twist_cmd.angular.z = 0.1;
                }else if(motion_twist_cmd.angular.z > -0.1 && motion_twist_cmd.angular.z < 0.0){
                    motion_twist_cmd.angular.z = -0.1;
                }
            }
        }
    }
    else if (sweep_action == STRA_BEFORE_TURNING)
    {
        double target_dist = 0.2 - (wheel_pose_state.x - dis_turning);
        // cout << "target_dist = " << target_dist << endl;
        // cout << "wheel_pose_state.x = " << wheel_pose_state.x << endl;
        // cout << "dis_turning = " << dis_turning << endl;
        if (target_dist < 0.1)
        {
            sweep_action = TURNING;
            omega_cnt = 0;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            theta_detour = wheel_pose_state.theta - PI_M / 2;
            cout << "TURNING" << endl;
        }
        else
        {
            double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.2);
            double speed_error = (-1.0 / PI_M) * fabs(pos_err_theta * 3.0) + 1.0;
            if (speed_error < 0.0)
            {
                speed_error = 0.0;
            }
            if (target_dist < 0.3)
            {
                target_dist = 0.3;
            }
            motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist) * (speed_error);
            motion_twist_cmd.angular.z = 0.0;
        }
    }
    else if (sweep_action == TURNING)
    {
        double err_theta = theta_detour - wheel_pose_state.theta;
        err_theta = emath.angleConstrain(err_theta);
        // cout << "角度偏差 = " << err_theta << endl;
        ROS_INFO("TURNING: target=%.3f, current=%.3f, error=%.3f, cmd_z=%.3f", 
             theta_detour, wheel_pose_state.theta, err_theta, motion_twist_cmd.angular.z);
        if (omega_cnt < 200)
        {
            omega_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            if (fabs(err_theta) < 0.03)
            {
                sweep_action = STRA_AFTER_TURNING;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                speed_cnt = 0;
                dis_turning = wheel_pose_state.x;
                // cout << "纵向偏差 = " << dis_f_l << endl;
                // cout << "角度偏差 = " << pos_err_theta << endl;
                cout << "STRA_AFTER_TURNING" << endl;
            }
            else
            {
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta) * 0.2;
                if(motion_twist_cmd.angular.z < 0.1 && motion_twist_cmd.angular.z > 0.0){
                    motion_twist_cmd.angular.z = 0.1;
                }else if(motion_twist_cmd.angular.z > -0.1 && motion_twist_cmd.angular.z < 0.0){
                    motion_twist_cmd.angular.z = -0.1;
                }
            }
        }
    }
    else if (sweep_action == STRA_AFTER_TURNING)
    {
        if (speed_cnt < 200)
        {
            speed_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            // cout << "speed_cnt = " << speed_cnt << endl;
        }
        else
        {
            double target_dist = 0.4 - (wheel_pose_state.x - dis_turning);
            // cout << "wheel_pose_state.x = " << wheel_pose_state.x << endl;
            // cout << "dis_turning = " << dis_turning << endl;
            // cout << "target_dist = " << target_dist << endl;

            // if(dis_f_l < 0.6){

            // }else{
            if (dis_l_f <= 0.8 || dis_f_l < 0.3)
            {
                sweep_action = STRAIGHT_ON;
                dist_r_f_datas.clear();
                omega_cnt = 0;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                cout << "STRAIGHT_ON" << endl;
            }
            else
            {
                double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.2);
                double speed_error = (-1.0 / PI_M) * fabs(pos_err_theta * 3.0) + 1.0;
                if (speed_error < 0.0)
                {
                    speed_error = 0.0;
                }
                if (target_dist < 0.3)
                {
                    target_dist = 0.3;
                }
                motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist) * (speed_error);
                motion_twist_cmd.angular.z = 0.0;
            }
            // }
        }
    }
 
// 应用电机补偿
    applyMotorCompensation(motion_twist_cmd);
    
    // 发布控制命令
    motionTwist_cmd_pub_.publish(motion_twist_cmd);
    
}

void MotionController::FullCoverage()
{
    double pos_err_theta = 0.0;
    double pos_err_y = 0.0;
    static double theta_detour = 0.0;
    static int speed_cnt = 0;
    static int omega_cnt = 0;
    double robot_width;
    static double dis_turning = 0.0;

    // if(dist_r_f_datas.size() == 10 && fabs((dist_r_f_datas[9] - dist_r_f_datas[0])) < 0.2){
    //     pos_err_y = (dist_r_f_datas[9] - dist_r_f_datas[0]);
    // }else{
    //     pos_err_y = 0;
    // }

    double wave_dis_f_x = 0.07, wave_dis_b_x = 0.07;
    double ab = wave_dis_f_x + wave_dis_b_x;
    double bc = dis_l_f - dis_l_b;

    // 计算斜边AC长度
    double ac = sqrt(ab * ab + bc * bc);

    // 计算角A的正切值：tan(A) = 对边BC / 邻边AB
    double tanA = bc / ab;

    // 使用反正切计算弧度
    pos_err_theta = atan(tanA);
    pos_err_theta = 0.0;
    
    if(dis_l_b < 6.0 && dis_l_f < 6.0 && (fabs(dis_l_b - dis_l_f) < 0.1)){
        pos_err_y = dis_l_f - dis_l_b;
    }else{
        pos_err_theta = wheel_pose_state.theta - cover_turning_theta;
        pos_err_y = 0.0;
    }

    if(fabs(pos_err_y) < 0.005){
        pos_err_y = 0.0;
    }
    // else if (dis_l_b > 10.0 || dis_l_f > 10.0){
    //     if(dist_r_f_datas.size() == 10 && fabs((dist_r_f_datas[9] - dist_r_f_datas[0])) < 0.2){
    //         pos_err_y = (dist_r_f_datas[9] - dist_r_f_datas[0]);
    //     }else{
    //         pos_err_y = 0;
    //     }
    // }

    // if(dis_l_b > 10.0 || dis_l_f > 10.0 || fabs(dis_l_b - dis_l_f) > 1.0){
    //     pos_err_theta = 0.0;
    //     pos_err_y = 0.0;
    // }

    // cover_action = CLIMB_WALL;

    if (cover_action == SEARCH_START_POINT)
    {
        motion_twist_cmd.linear.z = 10;
        double target_dist = dis_f_min - 0.2;

        // if(dis_f_l < 0.2 || dis_f_r < 0.2){
        //     target_dist = 0.0;
        // }

        if (target_dist < 0.1)
        {
            cover_action = COVER_TURNING;
            // cover_action = COVER_OVER;
            omega_cnt = 0;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
            goal_pose.theta = 0.0;
            motion_pose_state.x = 0.0;
            motion_pose_state.y = 0.0;
            motion_pose_state.theta = -PI_M / 2;
            goal_pose.x = pool_x;
            goal_pose.y = 0.0;
            theta_detour = wheel_pose_state.theta + goal_pose.theta - motion_pose_state.theta;
            frame_state = FRAME_X;
            cout << "COVER_TURNING" << endl;
            cout << "theta_detour = " << theta_detour << endl;
        }
        else
        {
            double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.0);
            double speed_error = (-1.0 / PI_M) * fabs(pos_err_theta * 3.0) + 1.0;
            if (speed_error < 0.0)
            {
                speed_error = 0.0;
            }
            motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * omega_error);
            // if (target_dist < 0.3)
            // {
            //     target_dist = 0.3;
            //     motion_twist_cmd.angular.z = 0.0;
            // }
            motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist) * (speed_error);
            if(motion_twist_cmd.angular.z < 0.01 && motion_twist_cmd.angular.z > -0.01){
                motion_twist_cmd.angular.z = 0.0;
            }
            // if(motion_twist_cmd.angular.z < 0.01 && motion_twist_cmd.angular.z > -0.01){
            //     motion_twist_cmd.angular.z = 0.0;
            // }
        }
    }else if (cover_action == COVER_GO_STRAIGHT){
        motion_twist_cmd.linear.z = 10;
        if (speed_cnt < 100)
        {
            speed_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            double target_dist = 0.0;
            double min_dist = (dis_f_l > dis_f_r) ? dis_f_r : dis_f_l;
            if(motion_pose_state.theta == PI_M / 2){
                target_dist = pool_dis_h - (wheel_pose_state.x - dis_turning);
                pos_err_y = 0.0;
                // cout << "dis_turning = " << dis_turning << endl;
                // cout << "wheel_pose_state.x = " << wheel_pose_state.x << endl;
                if(target_dist > min_dist){
                    target_dist = min_dist - 0.01;
                }
                // cout << "target_dist = " << target_dist << endl;
            }else
            {
                target_dist = 1.0;
                cout << "dis_f_r = " << dis_f_r << endl;
                cout << "dis_f_l = " << dis_f_l << endl;
                cout << "dis_f_min = " << dis_f_min << endl;
                cout << "target_dist = " << target_dist << endl;

                // if(dis_f_l < 0.2 || dis_f_r < 0.2){
                //     cover_action = CLIMB_WALL;
                //     climb_wall_action = STA_3;
                //     cout << "CLIMB_WALL" << endl;
                // }
                // else if(dis_f_l < 0.085 || dis_f_r < 0.085){
                //     if(fabs(dis_f_l - dis_f_r) > 0.3){
                //         cover_action = CLIMB_WALL;
                //         climb_wall_action = STA_3;
                //     }
                // }

                // cout << "dis_f_l = " << dis_f_l << endl;
                // cout << "target_dist = " << target_dist << endl;
            }

            if (dis_f_min < 0.2 || target_dist < 0.1)
            {
                cover_action = COVER_TURNING;
                omega_cnt = 0;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                motion_pose_state.x = goal_pose.x;
                motion_pose_state.y = goal_pose.y;
                if(motion_pose_state.theta == PI_M / 2){
                    if(goal_pose.x == 0.0){
                        goal_pose.x = pool_x;
                    }else{
                        goal_pose.x = 0.0;
                    }
                    goal_pose.y = motion_pose_state.y;
                }else if(motion_pose_state.theta == 0.0){
                    goal_pose.x = motion_pose_state.x;
                    goal_pose.y = motion_pose_state.y + pool_dis_h;
                }else if(motion_pose_state.theta == PI_M){
                    goal_pose.x = motion_pose_state.x;
                    goal_pose.y = motion_pose_state.y + pool_dis_h;
                }
                double dx = goal_pose.x - motion_pose_state.x;
                double dy = goal_pose.y - motion_pose_state.y;
                goal_pose.theta = std::atan2(dy, dx);
                theta_detour = goal_pose.theta - motion_pose_state.theta + wheel_pose_state.theta;
                theta_detour = emath.angleConstrain(theta_detour);
                cout << "GO_STRAIGHT TO COVER_TURNING" << endl;
                cout << "motion_pose_state.x = " << motion_pose_state.x << endl;
                cout << "motion_pose_state.y = " << motion_pose_state.y << endl;
                cout << "motion_pose_state.theta = " << motion_pose_state.theta << endl;
                cout << "goal_pose.x = " << goal_pose.x << endl;
                cout << "goal_pose.y = " << goal_pose.y << endl;
                cout << "goal_pose.theta = " << goal_pose.theta << endl;
                cout << "theta_detour = " << theta_detour << endl;
                cout << "theta_error = " << goal_pose.theta - motion_pose_state.theta << endl;
                cout << "----------------" << endl;
            }
            else
            {
                pos_err_theta = 0.0;
                double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.0);
                double speed_error = (-1.0 / PI_M) * fabs(pos_err_theta * 3.0) + 1.0;
                if (speed_error < 0.0)
                {
                    speed_error = 0.0;
                }
                motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * omega_error) * 2;
                if (target_dist < 0.3)
                {
                    target_dist = 0.3;
                    motion_twist_cmd.angular.z = 0.0;
                }
                motion_twist_cmd.linear.x = control_params.x.Kp * tanh(control_params.x.Ki * target_dist) * (speed_error);
                if(motion_twist_cmd.angular.z < 0.01 && motion_twist_cmd.angular.z > -0.01){
                    motion_twist_cmd.angular.z = 0.0;
                }
                // if(pos_err_theta != 0.0){
                cout << "dis_l_f = " << dis_l_f << endl;
                cout << "dis_l_b = " << dis_l_b << endl;
                cout << "pos_err_y = " << pos_err_y << endl;
                cout << "wheel_pose_state.theta = " << wheel_pose_state.theta << endl;
                cout << "cover_turning_theta = " << cover_turning_theta << endl;
                cout << "pos_err_theta = " << pos_err_theta << endl;
                cout << "motion_twist_cmd.angular.z = " << motion_twist_cmd.angular.z << endl;
                cout << "----------------" << endl;
                // }
                pos_err_theta = wheel_pose_state.theta - cover_turning_theta;
            }
        }
    }else if (cover_action == COVER_TURNING){
        motion_twist_cmd.linear.z = 30;
        double err_theta = theta_detour - wheel_pose_state.theta;
        err_theta = emath.angleConstrain(err_theta);
        // cout << "err_theta = " << err_theta << endl;
        // cout << "omega_cnt = " << omega_cnt << endl;
        if (omega_cnt < 200)
        {
            omega_cnt++;
            motion_twist_cmd.linear.x = 0.0;
            motion_twist_cmd.angular.z = 0.0;
        }
        else
        {
            if (fabs(err_theta) < 0.03)
            {
                if(dis_f_l < 0.3 && motion_pose_state.y > pool_y / 2){
                    cover_action = COVER_OVER;
                    cout << "COVER_OVER" << endl;
                    return;
                }
                cover_action = COVER_GO_STRAIGHT;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                speed_cnt = 0;
                dis_turning = wheel_pose_state.x;
                motion_pose_state.theta = goal_pose.theta;
                cout << "COVER_GO_STRAIGHT" << endl;
                cover_turning_theta = wheel_pose_state.theta;
            }
            else
            {
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * err_theta) * 0.2;
                if(motion_twist_cmd.angular.z < 0.1 && motion_twist_cmd.angular.z > 0.0){
                    motion_twist_cmd.angular.z = 0.1;
                }else if(motion_twist_cmd.angular.z > -0.1 && motion_twist_cmd.angular.z < 0.0){
                    motion_twist_cmd.angular.z = -0.1;
                }
            }
        }
    }else if (cover_action == CLIMB_WALL){
        // cout << "CLIMB_WALL" << endl;
        if(climb_wall_action == STA_0){
            if(dis_f_l > 0.2 && dis_f_r > 0.2){
                climb_wall_action = STA_1;
                cout << "STA_1" << endl;
            }
            motion_twist_cmd.linear.x = 0.1;
            motion_twist_cmd.linear.z = 90;
            motion_twist_cmd.angular.z = 0.0;
        }else if(climb_wall_action == STA_1){
            if(dis_f_l < 0.1 && dis_f_r < 0.1){
                climb_wall_action = STA_2;
                cout << "STA_2" << endl;
                motion_twist_cmd.linear.x = -0.1;
                motion_twist_cmd.linear.z = 90;
                motion_twist_cmd.angular.z = 0.0;
            }
        }else if(climb_wall_action == STA_2){
            if(dis_f_l > 0.2 && dis_f_r > 0.2){
                climb_wall_action = STA_3;
                cout << "STA_3" << endl;
            }
        }else if(climb_wall_action == STA_3){
            // cout << "dis_f_l = " << dis_f_l << endl;
            if(dis_f_l < 0.2 || dis_f_r < 0.2){
                climb_wall_action = STA_4;
                cout << "STA_4" << endl;
                cover_action = COVER_TURNING;
                omega_cnt = 0;
                motion_twist_cmd.linear.x = 0.0;
                motion_twist_cmd.linear.z = 0.0;
                motion_twist_cmd.angular.z = 0.0;
                motion_pose_state.x = goal_pose.x;
                motion_pose_state.y = goal_pose.y;
                if(motion_pose_state.theta == PI_M / 2){
                    if(goal_pose.x == 0.0){
                        goal_pose.x = pool_x;
                    }else{
                        goal_pose.x = 0.0;
                    }
                    goal_pose.y = motion_pose_state.y;
                }else if(motion_pose_state.theta == 0.0){
                    goal_pose.x = motion_pose_state.x + pool_dis_h;
                    goal_pose.y = motion_pose_state.y;
                }else if(motion_pose_state.theta == PI_M){
                    goal_pose.x = motion_pose_state.x + pool_dis_h;
                    goal_pose.y = motion_pose_state.y;
                }
                double dx = goal_pose.x - motion_pose_state.x;
                double dy = goal_pose.y - motion_pose_state.y;
                goal_pose.theta = std::atan2(dy, dx);
                theta_detour = goal_pose.theta - motion_pose_state.theta + wheel_pose_state.theta;
                theta_detour = emath.angleConstrain(theta_detour);
            }
        }
    }else if (cover_action == COVER_OVER){
        motion_twist_cmd.linear.x = 0.0;
        motion_twist_cmd.angular.z = 0.0;
    }
    motionTwist_cmd_pub_.publish(motion_twist_cmd);
}

void MotionController::Run()
{
    // [heading-straight-pd] 门控：开启航向保持走直线时，完全绕开老 TraceWall 状态机
    // （在 mars_state 机之前 return → 免疫 ares_cmd 共享内存模式翻转）。默认关=老逻辑一字不动。
    if (heading_mode_) { RunHeadingStraight(); return; }

   // 首先更新校准（如果正在校准）
    if (is_calibrating) {
        updateMotorCalibration();
        
        // 如果是校准模式，直接发布校准速度并返回
        motionTwist_cmd_pub_.publish(motion_twist_cmd);
        return;  // 校准期间不执行其他控制逻辑
    }
    
    // 定期输出调试信息
    if ((ros::Time::now() - last_debug_time).toSec() > 2.0) {
        ROS_INFO_THROTTLE(2.0, "=== System Status ===");
        ROS_INFO_THROTTLE(2.0, "Session: %d, SweepAction: %d", session, sweep_action);
        ROS_INFO_THROTTLE(2.0, "Motor comp: L=%.3f, R=%.3f", 
                         motor_compensation_left, motor_compensation_right);
        ROS_INFO_THROTTLE(2.0, "Wheel pose: (%.3f, %.3f, %.3f)", 
                         wheel_pose_state.x, wheel_pose_state.y, wheel_pose_state.theta);
        last_debug_time = ros::Time::now();
    }

    if (mars_state.mars_ctlmode_sta == MARS_SYSTEM_MODE_STA)
    {
        if (ares_cmd->mars_ctlmode_cmd == MARS_GET_MANUAL_CMD)
        {
            traceZero();
            if (fabs(motion_twist_state.linear.x) < 0.01)
            {
                mars_state.mars_ctlmode_sta = MARS_MANUAL_MODE_STA;
                step_index = 0;
                motion_steps.clear();
                cout << "MANUAL_CONTROL" << endl;
            }
            return;
        }

        // if (session == SES_WALL_STATE)
        // {
        //     FindWall();
        // }
        // else if (session == SES_SWEEP_STATE)
        // {
          TraceWall();
        // }else if (session == SES_COVER_STATE){
        //FullCoverage();
        // }else if (session == SES_FREE_STATE){
            // traceZero();
        //}
    }
    else if (mars_state.mars_ctlmode_sta == MARS_MANUAL_MODE_STA)
    {
        // cout << "wheel_pose_state.theta = " << wheel_pose_state.theta << endl;
        // cout << "wheel_pose_state.x = " << wheel_pose_state.x << endl;
        TraceWall();
        // cout << "dis_l_f = " << dis_l_f << endl;
        // cout << "dist_r_f_datas = " << endl;
        // for(int i = 0; i < dist_r_f_datas.size(); i++){
            //     cout << " " << dist_r_f_datas[i];
            // }
            // cout << endl;
            
        // double pos_err_y = 0.0;
        // if(dis_l_b < 6.0 && dis_l_f < 6.0){
        //     pos_err_y = dis_l_f - dis_l_b;
        //     cout << "dis_l_b = " << dis_l_b << endl;
        //     cout << "dis_l_f = " << dis_l_f << endl;
        //     cout << "pos_err_y = " << pos_err_y << endl;
        //     cout << "fabs(dis_l_b - dis_l_f) = " << fabs(dis_l_b - dis_l_f) << endl;
        // }else{
        //     pos_err_y = 0.0;
        //     cout << "-------------------- " << endl;
        // }
        // double omega_error = atan(pos_err_y * 1.0);
        // motion_twist_cmd.angular.z = control_params.theta.Kp * tanh(control_params.theta.Ki * omega_error);
        // cout << "dis_l_f = " << dis_l_f << endl;
        // cout << "dis_l_b = " << dis_l_b << endl;
        // cout << "motion_twist_cmd.angular.z = " << motion_twist_cmd.angular.z << endl;
        // cout << "----------------" << endl;
    }

    if (ares_cmd->mars_ctlmode_cmd == MARS_GET_SYSTEM_CMD && mars_state.mars_ctlmode_sta != MARS_SYSTEM_MODE_STA)
    {
        mars_state.mars_ctlmode_sta = MARS_SYSTEM_MODE_STA;
        cout << "SYSTEM_CONTROL" << endl;
    }


}


// 校准服务回调
bool MotionController::handleCalibrateRequest(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res)
{
    // [heading-straight-pd] heading 模式下拒绝电机校准：校准会置 is_calibrating，
    // 而我们在 Run() 顶部已 return，校准逻辑不会被驱动 → 会卡死标志。
    if (heading_mode_) {
        res.success = false;
        res.message = "calibration disabled in heading_straight mode";
        ROS_WARN("[hs] calibrate service refused (heading mode)");
        return true;
    }
    startMotorCalibration();
    res.success = true;
    res.message = "Motor calibration started";
    ROS_INFO("Motor calibration triggered via service");
    return true;
}

// 开始校准
void MotionController::startMotorCalibration()
{
    ROS_INFO("🚀 Starting automatic motor calibration...");
    
    is_calibrating = true;
    calibration_start_time = ros::Time::now();
    
    // 记录起始状态
    calibration_data.straight_yaw_start = wheel_pose_state.theta;
    calibration_data.straight_distance_start = wheel_pose_state.x;
    calibration_data.yaw_drift_sum = 0.0;
    calibration_data.sample_count = 0;
    
    // 重置运动状态
    sweep_action = STRAIGHT_ON;  // 确保在校准状态
    
    // 设置校准速度 - 直行
    motion_twist_cmd.linear.x = 0.2;  // 0.2 m/s 的校准速度
    motion_twist_cmd.angular.z = 0.0;
    motion_twist_cmd.linear.y = 0.0;
    motion_twist_cmd.linear.z = 99;   // 特殊标记，用于区分校准模式
    
    ROS_INFO("Calibration: Starting at yaw=%.3f, x=%.3f", 
             calibration_data.straight_yaw_start, 
             calibration_data.straight_distance_start);
    ROS_INFO("Calibration: Moving straight at 0.2 m/s for 5 seconds...");
}

// 更新校准
void MotionController::updateMotorCalibration()
{
    if (!is_calibrating) return;
    
    double elapsed = (ros::Time::now() - calibration_start_time).toSec();
    
    // 收集偏航数据（每10次循环收集一次，避免数据过多）
    static int collect_counter = 0;
    if (collect_counter++ % 10 == 0) {
        double current_yaw = wheel_pose_state.theta;
        double yaw_drift = emath.angleConstrain(current_yaw - calibration_data.straight_yaw_start);
        
        calibration_data.yaw_drift_sum += yaw_drift;
        calibration_data.sample_count++;
        
        // 实时输出校准进度
        if (calibration_data.sample_count % 10 == 0) {
            double avg_yaw_drift = calibration_data.yaw_drift_sum / calibration_data.sample_count;
            ROS_INFO_THROTTLE(0.5, "Calibrating... %.1f/5.0s, Avg yaw drift: %.4f rad", 
                             elapsed, avg_yaw_drift);
        }
    }
    
    // 校准完成条件：走了足够时间
    if (elapsed >= 5.0) {  // 校准5秒
        is_calibrating = false;
        
        // 计算最终偏航漂移
        double avg_yaw_drift = 0.0;
        if (calibration_data.sample_count > 0) {
            avg_yaw_drift = calibration_data.yaw_drift_sum / calibration_data.sample_count;
        }
        
        double distance_traveled = wheel_pose_state.x - calibration_data.straight_distance_start;
        double yaw_drift_per_meter = (distance_traveled > 0.1) ? 
                                     avg_yaw_drift / distance_traveled : 0.0;
        
        ROS_INFO("========================================");
        ROS_INFO("📊 MOTOR CALIBRATION COMPLETE");
        ROS_INFO("========================================");
        ROS_INFO("Calibration time: %.1f seconds", elapsed);
        ROS_INFO("Distance traveled: %.3f meters", distance_traveled);
        ROS_INFO("Average yaw drift: %.4f radians", avg_yaw_drift);
        ROS_INFO("Yaw drift per meter: %.4f rad/m", yaw_drift_per_meter);
        
        // 判断是否需要调整
        if (fabs(yaw_drift_per_meter) > 0.01) {  // 阈值：0.01 rad/m
            double old_left = motor_compensation_left;
            double old_right = motor_compensation_right;
            
            if (avg_yaw_drift > 0) {
                // 顺时针偏航 → 左轮更快或右轮更慢
                ROS_WARN("⚠️  Robot drifts RIGHT (clockwise)");
                ROS_WARN("   Left motor is FASTER or Right motor is SLOWER");
                
                // 计算调整量（基于偏航率）
                double adjustment = fabs(yaw_drift_per_meter) * 0.5;  // 调整系数
                adjustment = std::min(adjustment, 0.1);  // 最大调整10%
                
                motor_compensation_left *= (1.0 - adjustment);
                motor_compensation_right *= (1.0 + adjustment);
                
                ROS_INFO("Adjustment factor: %.3f", adjustment);
            } else {
                // 逆时针偏航 → 右轮更快或左轮更慢
                ROS_WARN("⚠️  Robot drifts LEFT (counter-clockwise)");
                ROS_WARN("   Right motor is FASTER or Left motor is SLOWER");
                
                double adjustment = fabs(yaw_drift_per_meter) * 0.5;
                adjustment = std::min(adjustment, 0.1);
                
                motor_compensation_left *= (1.0 + adjustment);
                motor_compensation_right *= (1.0 - adjustment);
                
                ROS_INFO("Adjustment factor: %.3f", adjustment);
            }
            
            // 限制补偿系数范围
            motor_compensation_left = std::max(0.7, std::min(1.3, motor_compensation_left));
            motor_compensation_right = std::max(0.7, std::min(1.3, motor_compensation_right));
            
            ROS_INFO("Compensation changed:");
            ROS_INFO("   Left:  %.3f → %.3f", old_left, motor_compensation_left);
            ROS_INFO("   Right: %.3f → %.3f", old_right, motor_compensation_right);
            
            // 保存到参数服务器
            nh_.setParam("motor_compensation_left", motor_compensation_left);
            nh_.setParam("motor_compensation_right", motor_compensation_right);
            ROS_INFO("✅ Parameters saved to ROS parameter server");
            
            // 测试校准效果
            ROS_INFO("Testing calibration for 3 seconds...");
            
            // 短暂测试
            ros::Time test_start = ros::Time::now();
            double test_yaw_start = wheel_pose_state.theta;
            
            // 等待3秒
            while ((ros::Time::now() - test_start).toSec() < 3.0) {
                motion_twist_cmd.linear.x = 0.2;
                motion_twist_cmd.angular.z = 0.0;
                motionTwist_cmd_pub_.publish(motion_twist_cmd);
                ros::Duration(0.1).sleep();
            }
            
            double test_yaw_end = wheel_pose_state.theta;
            double test_yaw_drift = emath.angleConstrain(test_yaw_end - test_yaw_start);
            ROS_INFO("Test result: yaw drift = %.4f rad (%.2f deg)", 
                    test_yaw_drift, test_yaw_drift * 180.0 / PI_M);
            
            if (fabs(test_yaw_drift) < 0.05) {  // 小于3度
                ROS_INFO("🎉 Calibration SUCCESSFUL!");
            } else {
                ROS_WARN("Calibration may need further adjustment");
            }
        } else {
            ROS_INFO("✅ Motors are already well balanced!");
        }
        
        ROS_INFO("========================================");
        
        // 停止运动
        motion_twist_cmd.linear.x = 0.0;
        motion_twist_cmd.angular.z = 0.0;
        motion_twist_cmd.linear.z = 0.0;  // 清除校准标记
        
        // 重置运动状态
        sweep_action = STRAIGHT_ON;
        
        // 立即发布停止命令
        motionTwist_cmd_pub_.publish(motion_twist_cmd);
    }
}
// 应用电机补偿
void MotionController::applyMotorCompensation(geometry_msgs::Twist& cmd)
{
    // 只在直线行走时应用补偿，且不是在校准模式下
    if (!is_calibrating && 
        (sweep_action == STRAIGHT_ON || 
         sweep_action == STRA_BEFORE_TURNING || 
         sweep_action == STRA_AFTER_TURNING)) {
        
        static double last_yaw = wheel_pose_state.theta;
        static double yaw_error_integral = 0.0;
        
        // 计算实时偏航误差
        double current_yaw = wheel_pose_state.theta;
        double yaw_error = current_yaw - last_yaw;
        last_yaw = current_yaw;
        
        // 积分误差（用于持续偏航检测）
        yaw_error_integral += yaw_error;
        
        // 动态调整补偿因子（小幅度实时调整）
        const double Kp_yaw = 0.005;   // 比例系数
        const double Ki_yaw = 0.0005;  // 积分系数
        
        // 如果有明显偏航趋势
        if (fabs(yaw_error_integral) > 0.1) {  // 积分阈值
            if (yaw_error_integral > 0) {
                // 顺时针偏航累积
                double adjustment = Kp_yaw * yaw_error_integral + Ki_yaw * yaw_error_integral;
                motor_compensation_left -= adjustment;
                motor_compensation_right += adjustment;
                ROS_INFO_THROTTLE(10.0, "📈 Auto-adjust: CW drift, L-%.4f, R+%.4f", 
                                 adjustment, adjustment);
            } else {
                // 逆时针偏航累积
                double adjustment = Kp_yaw * fabs(yaw_error_integral) + Ki_yaw * fabs(yaw_error_integral);
                motor_compensation_left += adjustment;
                motor_compensation_right -= adjustment;
                ROS_INFO_THROTTLE(10.0, "📈 Auto-adjust: CCW drift, L+%.4f, R-%.4f", 
                                 adjustment, adjustment);
            }
            
            // 限制范围
            motor_compensation_left = std::max(0.8, std::min(1.2, motor_compensation_left));
            motor_compensation_right = std::max(0.8, std::min(1.2, motor_compensation_right));
            
            // 重置积分器
            yaw_error_integral = 0.0;
        }
        
        // 应用补偿到控制命令
        const double WHEEL_BASE = 0.5;  // 轮距，根据实际测量修改
        
        double v = cmd.linear.x;
        double w = cmd.angular.z;
        
        // 转换为左右轮速
        double v_left = v - (w * WHEEL_BASE / 2.0);
        double v_right = v + (w * WHEEL_BASE / 2.0);
        
        // 应用补偿系数
        v_left *= motor_compensation_left;
        v_right *= motor_compensation_right;
        
        // 转换回线速度和角速度
        cmd.linear.x = (v_left + v_right) / 2.0;
        cmd.angular.z = (v_right - v_left) / WHEEL_BASE;
        
        // 调试输出
        static int comp_debug = 0;
        if (comp_debug++ % 200 == 0 && fabs(v) > 0.05) {
            ROS_INFO_THROTTLE(2.0, "Motor compensation applied: L=%.3f, R=%.3f", 
                             motor_compensation_left, motor_compensation_right);
            ROS_INFO_THROTTLE(2.0, "  Before: v=%.3f, w=%.3f", v, w);
            ROS_INFO_THROTTLE(2.0, "  After:  v=%.3f, w=%.3f", cmd.linear.x, cmd.angular.z);
        }
    }
}

// ======================= [heading-straight-pd] 航向保持走直线实现 =======================

// 角度 wrap 到 [-pi, pi]（atan2(sin(-pi),cos(-pi))==-pi，两端皆有效，对调节器无碍）
double MotionController::wrapPi(double a)
{
    return atan2(sin(a), cos(a));
}

// /imu_heading 回调（度 → rad）。与 Run() 同线程（spinOnce），无需加锁。
void MotionController::imuHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
    hs_yaw_rad_    = (double)msg->data * PI_M / 180.0;
    hs_yaw_stamp_  = ros::Time::now();   // Float32 无 header，用同机接收时刻
    hs_have_yaw_   = true;
    hs_new_sample_ = true;
}

void MotionController::openLogIfNeeded()
{
    if (hs_log_open_) return;
    hs_log_.open(h_log_path_.c_str(), std::ios::out | std::ios::trunc);
    if (hs_log_.is_open()) {
        hs_log_ << "# CODE_VERSION=" << CODE_VERSION
                << " Kp=" << h_kp_ << " Kd=" << h_kd_ << " Ki=" << h_ki_
                << " speed=" << h_speed_ << " omega_limit=" << h_omega_limit_
                << " sign=" << h_sign_ << " yaw0_deg=" << hs_yaw0_ * 180.0 / PI_M << "\n";
        hs_log_ << "t,phase,yaw_deg,yaw_age_ms,yaw0_deg,e_rad,P,I,D,u,vx,odom_x,odom_y,odom_theta,driven_m\n";
        hs_log_open_ = true;
        ROS_WARN("[hs] CSV log: %s", h_log_path_.c_str());
    } else {
        ROS_ERROR("[hs] cannot open log %s (keep running, no logging)", h_log_path_.c_str());
    }
}

// 退出前补发零速（main 在 Ctrl+C 后调用；驱动无看门狗，否则保持最后指令）
void MotionController::shutdownStop()
{
    if (!heading_mode_) return;
    geometry_msgs::Twist z;   // 全 0
    ros::Rate r(50);
    int sent = 0;
    for (int i = 0; i < 20 && ros::ok(); i++) {
        heading_cmd_pub_.publish(z);
        sent++;
        r.sleep();
    }
    if (hs_log_open_) { hs_log_.flush(); hs_log_.close(); hs_log_open_ = false; }
    // 评审 MEDIUM(Fable5)：rosnode kill 路径下 ros::ok() 已假→sent==0，绝不能撒谎报"已补发零速"。
    // 如实报告：发满 20 条才算成功，否则 ERROR 提示可能仍在运动、立即断电。
    if (sent == 20)
        ROS_WARN("[hs] exit: zero-vel resent %d/20", sent);
    else
        ROS_ERROR("[hs] exit: only sent %d/20 zero-vel (ros::ok went false early, e.g. rosnode kill), "
                  "robot MAY STILL MOVE, CUT POWER NOW!", sent);
}

void MotionController::RunHeadingStraight()
{
    ros::Time now = ros::Time::now();
    double loop_dt = (now - hs_t_prev_loop_).toSec();
    if (loop_dt < 0.001) loop_dt = 0.001;
    if (loop_dt > 0.05)  loop_dt = 0.05;
    hs_t_prev_loop_ = now;

    geometry_msgs::Twist cmd;   // 默认全 0

    // ---------- WAIT_YAW：等 /imu_heading（兼版本闸门：新控制器+旧驱动=无话题=不动）----------
    if (hs_phase_ == HS_WAIT_YAW)
    {
        if (!hs_have_yaw_ || (now - hs_yaw_stamp_).toSec() > h_yaw_timeout_)
        {
            ROS_WARN_THROTTLE(2.0, "[hs] WAIT_YAW: no fresh %s (is the new driver running?)",
                              h_yaw_topic_.c_str());
            heading_cmd_pub_.publish(cmd);   // 发零
            return;
        }
        hs_phase_       = HS_STILL;
        hs_phase_start_ = now;
        hs_sin_sum_ = hs_cos_sum_ = 0.0; hs_still_n_ = 0; hs_still_started_ = false;
        hs_ls_t_ = hs_ls_y_ = hs_ls_tt_ = hs_ls_ty_ = 0.0;
        ROS_WARN("[hs] -> STILL: capture yaw0 for %.1fs (keep robot still)", h_settle_s_);
    }

    // ---------- STILL：静止采 yaw0（圆均值=去零偏）+ 静止门 + 漂移最小二乘 ----------
    if (hs_phase_ == HS_STILL)
    {
        heading_cmd_pub_.publish(cmd);   // 全程发零

        if (hs_new_sample_)
        {
            hs_new_sample_ = false;
            double y = hs_yaw_rad_;
            hs_sin_sum_ += sin(y);
            hs_cos_sum_ += cos(y);
            hs_still_n_++;
            double t_rel = (now - hs_phase_start_).toSec();
            if (!hs_still_started_) {
                hs_still_first_ = y; hs_still_min_ = 0.0; hs_still_max_ = 0.0;
                hs_still_started_ = true;
            } else {
                double d = wrapPi(y - hs_still_first_);
                if (d < hs_still_min_) hs_still_min_ = d;
                if (d > hs_still_max_) hs_still_max_ = d;
            }
            double yy = wrapPi(y - hs_still_first_);   // 相对首样本展开（小量），用于漂移拟合
            hs_ls_t_  += t_rel;        hs_ls_y_  += yy;
            hs_ls_tt_ += t_rel * t_rel; hs_ls_ty_ += t_rel * yy;
        }

        if ((now - hs_phase_start_).toSec() >= h_settle_s_)
        {
            if (hs_still_n_ < 5) {
                ROS_ERROR("[hs] STILL done but too few samples(%ld): %s too slow? back to WAIT_YAW.",
                          hs_still_n_, h_yaw_topic_.c_str());
                hs_phase_ = HS_WAIT_YAW; hs_have_yaw_ = false; return;
            }
            double pp_deg = (hs_still_max_ - hs_still_min_) * 180.0 / PI_M;
            if (pp_deg > h_still_pp_deg_) {
                ROS_WARN("[hs] not still: peak-peak %.3fdeg > %.3fdeg, re-settle.", pp_deg, h_still_pp_deg_);
                hs_phase_start_ = now;
                hs_sin_sum_ = hs_cos_sum_ = 0.0; hs_still_n_ = 0; hs_still_started_ = false;
                hs_ls_t_ = hs_ls_y_ = hs_ls_tt_ = hs_ls_ty_ = 0.0;
                return;
            }
            hs_yaw0_ = atan2(hs_sin_sum_, hs_cos_sum_);   // 圆均值参考航向
            hs_base_yaw_ = hs_yaw0_;   // [rect-z] 起始航向基准(段0目标=基准本身; 单段模式无副作用)
            double n = (double)hs_still_n_;
            double denom = (n * hs_ls_tt_ - hs_ls_t_ * hs_ls_t_);
            double drift = (fabs(denom) > 1e-9) ? (n * hs_ls_ty_ - hs_ls_t_ * hs_ls_y_) / denom : 0.0; // rad/s
            ROS_WARN("[hs] yaw0=%.3fdeg (zero-offset) still_pp=%.3fdeg drift=%.5fdeg/s %s",
                     hs_yaw0_ * 180.0 / PI_M, pp_deg, drift * 180.0 / PI_M,
                     h_drift_comp_ ? "(drift_comp logged only, no comp)" : "(logged only, no comp)");
            hs_phase_         = HS_DRIVE;
            hs_phase_start_   = now;
            hs_drive_start_   = now;
            hs_last_pause_    = now;
            hs_ramp_start_    = now;   // 缓启动斜坡起点(起步)
            hs_I_ = 0.0; hs_d_filt_ = 0.0; hs_e_prev_ = 0.0;
            hs_t_prev_sample_ = now;
            hs_x0_ = wheel_pose_state.x; hs_y0_ = wheel_pose_state.y;
            hs_seg_idx_ = 0;   // [rect-z] 段索引归零(段0即此处进入的直行; 单段模式忽略)
            openLogIfNeeded();
            ROS_WARN("[hs] -> DRIVE: speed=%.3f Kp=%.4f sign=%d (start straight)",
                     h_speed_, h_kp_, h_sign_);
        }
        return;
    }

    // ---------- DONE：持续发零（keep-alive，对抗驱动保持最后指令）----------
    if (hs_phase_ == HS_DONE)
    {
        heading_cmd_pub_.publish(cmd);
        return;
    }

    // ---------- [rect-z] TURN：原地转向到下一段目标航向 ----------
    // 闭环到 /imu_heading 真航向(融合, 不吃陀螺漂移); 复用直行同号约定(h_sign_)保证转对方向;
    // 转向期 linear.x=0 + 泵关(linear.y=0) 绝不前进(安全); 超时/失速双看门狗失败一律停车不盲转。
    if (hs_phase_ == HS_TURN)
    {
        // /imu_heading 陈旧 -> 立即停车中止(与 DRIVE 一致)
        if (!hs_have_yaw_ || (now - hs_yaw_stamp_).toSec() > h_yaw_timeout_) {
            ROS_ERROR_THROTTLE(1.0, "[hs] TURN: %s stale %.2fs -> STOP abort",
                               h_yaw_topic_.c_str(), (now - hs_yaw_stamp_).toSec());
            heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;
        }
        // 转完停稳子阶段: 持续发零, 稳够 settle 后进入下一段直行
        if (hs_turn_settling_) {
            heading_cmd_pub_.publish(cmd);   // 零
            if ((now - hs_turn_settle_start_).toSec() >= hs_turn_settle_s_)
                hsBeginSegment(hs_seg_idx_, now);
            return;
        }
        double err = wrapPi(hs_yaw_rad_ - hs_turn_target_);   // 到目标航向误差(rad), 趋 0
        // 超时: 转不到位 -> 停(失败朝安全方向, 绝不前冲)
        if ((now - hs_turn_start_).toSec() > hs_turn_timeout_s_) {
            ROS_ERROR("[hs] TURN seg%d timeout %.1fs (|err|=%.1fdeg) -> STOP",
                      hs_seg_idx_, hs_turn_timeout_s_, fabs(err) * 180.0 / PI_M);
            heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;
        }
        // 进度看门狗: 航向必须持续转动(转不起来=水下原地转失败 -> 停)
        if (hs_new_sample_) {
            hs_new_sample_ = false;
            double moved = fabs(wrapPi(hs_yaw_rad_ - hs_turn_last_yaw_));
            if (moved > hs_turn_progress_eps_) { hs_turn_last_yaw_ = hs_yaw_rad_; hs_turn_last_prog_t_ = now; }
        }
        if ((now - hs_turn_last_prog_t_).toSec() > hs_turn_stall_s_) {
            ROS_ERROR("[hs] TURN seg%d STALLED (no yaw progress %.1fs, cannot rotate?) -> STOP",
                      hs_seg_idx_, hs_turn_stall_s_);
            heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;
        }
        // 到位: 进入停稳
        if (fabs(err) <= hs_turn_tol_) {
            hs_turn_settling_ = true; hs_turn_settle_start_ = now;
            heading_cmd_pub_.publish(cmd);   // 零
            ROS_WARN("[hs] TURN seg%d reached (err=%.1fdeg) -> settle %.1fs",
                     hs_seg_idx_, err * 180.0 / PI_M, hs_turn_settle_s_);
            return;
        }
        // 旋转命令: 同号约定保证转对方向; 近目标(fine_band 内)降速防过冲; 泵关、不前进。
        double mag = (fabs(err) > hs_turn_fine_band_) ? hs_turn_omega_ : hs_turn_omega_fine_;
        double az  = (double)h_sign_ * ((err > 0.0) ? 1.0 : -1.0) * mag;
        cmd.linear.x  = 0.0;
        cmd.linear.y  = 0.0;   // 泵关: 转向期绝不前进(安全; 与直行"仅前进时开泵"一致)
        cmd.linear.z  = 0.0;
        cmd.angular.z = az;
        heading_cmd_pub_.publish(cmd);
        if (hs_log_open_) {
            double yaw_age_ms = (now - hs_yaw_stamp_).toSec() * 1000.0;
            hs_log_ << std::fixed << std::setprecision(6)
                    << now.toSec() << ",TURN," << hs_yaw_rad_ * 180.0 / PI_M << ',' << yaw_age_ms << ','
                    << hs_turn_target_ * 180.0 / PI_M << ',' << err << ",0,0,0," << az << ",0,"
                    << wheel_pose_state.x << ',' << wheel_pose_state.y << ',' << wheel_pose_state.theta << ",0\n";
        }
        ROS_INFO_THROTTLE(1.0, "[hs] TURN seg%d err=%.1fdeg az=%.3f", hs_seg_idx_, err * 180.0 / PI_M, az);
        return;
    }

    // ---------- DRIVE / PAUSE ----------
    // 陈旧看门狗：/imu_heading 停了就立刻停车中止
    if (!hs_have_yaw_ || (now - hs_yaw_stamp_).toSec() > h_yaw_timeout_)
    {
        ROS_ERROR_THROTTLE(1.0, "[hs] %s stale %.2fs -> STOP abort",
                           h_yaw_topic_.c_str(), (now - hs_yaw_stamp_).toSec());
        heading_cmd_pub_.publish(cmd);   // 零
        hs_phase_ = HS_DONE;
        return;
    }

    // 自停：时间 / 距离（先到先停）。drive_elapsed 含暂停时间→作为总时长上限是保守的。
    double driven        = hypot(wheel_pose_state.x - hs_x0_, wheel_pose_state.y - hs_y0_);
    double drive_elapsed = (now - hs_drive_start_).toSec();
    if (h_run_seconds_ > 0.0 && drive_elapsed >= h_run_seconds_) {
        ROS_WARN("[hs] reached time %.1fs -> stop done", drive_elapsed);
        heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;
    }
    if (path_mode_ == PATH_SINGLE) {
        if (h_run_distance_ > 0.0 && driven >= h_run_distance_) {
            ROS_WARN("[hs] reached dist %.2fm -> stop done", driven);
            heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;
        }
    } else {
        // [rect-z] 段距离到 -> 原地转向到下一段; 末段走完 -> 收尾停车。run_seconds 仍是全局时间兜底(上面)。
        if (hs_segs_.empty() || hs_seg_idx_ >= (int)hs_segs_.size()) {
            heading_cmd_pub_.publish(cmd); hs_phase_ = HS_DONE; return;   // 兜底: 段表异常即停
        }
        if (driven >= hs_segs_[hs_seg_idx_].distance) {
            ROS_WARN("[hs] seg%d/%d leg done (driven=%.2fm)", hs_seg_idx_, (int)hs_segs_.size(), driven);
            heading_cmd_pub_.publish(cmd);   // 本拍发零, 下一拍进入转向/收尾
            int next = hs_seg_idx_ + 1;
            if (next >= (int)hs_segs_.size()) {
                ROS_WARN("[hs] all %d segments done -> stop", (int)hs_segs_.size());
                hs_phase_ = HS_DONE; return;
            }
            hs_seg_idx_ = next;
            hsBeginTurn(wrapPi(hs_base_yaw_ + hs_segs_[next].cum_off), now);
            return;
        }
    }

    // 周期暂停（IMU ≤3min/≥3s 合规；**不重采 yaw0**，否则会抹掉要测的累计误差）
    bool in_pause = false;
    if (hs_phase_ == HS_PAUSE) {
        if ((now - hs_phase_start_).toSec() >= h_pause_duration_s_) {
            hs_phase_ = HS_DRIVE; hs_last_pause_ = now;
            hs_ramp_start_ = now;   // 缓启动斜坡起点(暂停恢复后重新渐起, 因机器人已静止)
            // 恢复行驶：重置微分状态，避免暂停期间的姿态变化在 D 项上产生尖峰（评审 MEDIUM-1）。
            hs_d_filt_ = 0.0;
            hs_e_prev_ = (double)h_sign_ * wrapPi(hs_yaw_rad_ - hs_yaw0_);
            hs_t_prev_sample_ = hs_yaw_stamp_;
        } else {
            in_pause = true;
        }
    } else { // HS_DRIVE
        // [rect-z] 周期暂停只在单段模式; 矩形/Z 段短(数十秒内)无需暂停, 且避免与段计时/转向交互。
        if (path_mode_ == PATH_SINGLE && h_pause_every_s_ > 0.0 && (now - hs_last_pause_).toSec() >= h_pause_every_s_) {
            hs_phase_ = HS_PAUSE; hs_phase_start_ = now; in_pause = true;
            ROS_WARN("[hs] periodic pause %.1fs (IMU compliance, no yaw0 re-capture)", h_pause_duration_s_);
        }
    }

    // 航向误差（始终计算，便于暂停时也记录）
    double e = (double)h_sign_ * wrapPi(hs_yaw_rad_ - hs_yaw0_);

    // 微分：仅在新 yaw 样本上、按样本间 dt、一阶低通（融合yaw 0.01°量化+低于100Hz，逐loop差分会尖峰）
    if (hs_new_sample_) {
        hs_new_sample_ = false;
        double dts = (hs_yaw_stamp_ - hs_t_prev_sample_).toSec();
        if (dts < 0.005) dts = 0.005;
        if (dts > 0.5)   dts = 0.5;
        double raw_d = wrapPi(e - hs_e_prev_) / dts;
        double alpha = dts / (0.2 + dts);     // tau_d = 0.2s
        hs_d_filt_ += alpha * (raw_d - hs_d_filt_);
        hs_e_prev_ = e;
        hs_t_prev_sample_ = hs_yaw_stamp_;
    }

    double u_unsat = h_kp_ * e + h_kd_ * hs_d_filt_ + hs_I_;

    // 条件抗饱和：仅在 DRIVE 且未饱和时积分
    if (!in_pause && fabs(u_unsat) < h_omega_limit_) {
        hs_I_ += h_ki_ * e * loop_dt;
        if (hs_I_ >  h_i_clamp_) hs_I_ =  h_i_clamp_;
        if (hs_I_ < -h_i_clamp_) hs_I_ = -h_i_clamp_;
    }

    double u = u_unsat;
    if (u >  h_omega_limit_) u =  h_omega_limit_;
    if (u < -h_omega_limit_) u = -h_omega_limit_;
    if (h_deadband_u_ > 0.0 && fabs(u) < h_deadband_u_) u = 0.0;   // 仅 /autoVel_cmd 备用，默认 0 不动作

    // 缓启动斜坡: DRIVE 起步/暂停恢复后, linear.x 在 softstart_seconds 内线性 0->h_speed_。
    // 只缓速度(前进)，不动航向环 u —— 消除起步瞬跳引起的侧偏(0.3 起步弯诊断), 不影响纠偏。
    double hs_ramp = 1.0;
    if (h_softstart_s_ > 1e-3) {
        double rt = (now - hs_ramp_start_).toSec();
        hs_ramp = rt / h_softstart_s_;
        if (hs_ramp < 0.0) hs_ramp = 0.0;
        if (hs_ramp > 1.0) hs_ramp = 1.0;
    }
    cmd.linear.x  = in_pause ? 0.0 : (h_speed_ * hs_ramp);
    // 泵(/cmd_vel linear.y=占空比)：仅"真正前进"时开——暂停 或 speed<=0(sign 测试) 一律关泵，防干转(陆地/原地)。
    // 默认 h_pump_speed_=0 → 陆地前进测试也不开泵；下水正式跑传 _pump_speed:=0.2。
    cmd.linear.y  = (in_pause || h_speed_ <= 0.0) ? 0.0 : h_pump_speed_;
    cmd.linear.z  = 0.0;
    cmd.angular.z = in_pause ? 0.0 : u;
    heading_cmd_pub_.publish(cmd);

    if (hs_log_open_) {
        double yaw_age_ms = (now - hs_yaw_stamp_).toSec() * 1000.0;
        hs_log_ << std::fixed << std::setprecision(6)
                << now.toSec() << ',' << (in_pause ? "PAUSE" : "DRIVE") << ','
                << hs_yaw_rad_ * 180.0 / PI_M << ',' << yaw_age_ms << ','
                << hs_yaw0_ * 180.0 / PI_M << ',' << e << ',' << (h_kp_ * e) << ','
                << hs_I_ << ',' << (h_kd_ * hs_d_filt_) << ',' << u << ',' << cmd.linear.x << ','
                << wheel_pose_state.x << ',' << wheel_pose_state.y << ',' << wheel_pose_state.theta << ','
                << driven << '\n';
        // 评审 LOW(Fable5)：每 ~100 行(≈1s @100Hz)落盘一次，把断电/SIGKILL 最坏丢失约束到 ~1s
        // (恰是最接近事故时刻的数据)。正常退出 shutdownStop 仍会 flush+close。
        static unsigned hs_log_rows = 0;
        if ((++hs_log_rows % 100) == 0) hs_log_.flush();
    }

    ROS_INFO_THROTTLE(1.0, "[hs] %s e=%.2fdeg u=%.3f I=%.3f driven=%.2fm t=%.1fs",
                      in_pause ? "PAUSE" : "DRIVE", e * 180.0 / PI_M, u, hs_I_, driven, drive_elapsed);
}

// [rect-z] 据 control_mode 生成段表: 每段 {直行距离, 相对起始航向的累计转角(rad)}。
// 转角累计不在此 wrap; 用时 wrapPi(base + cum_off) 得该段目标航向。
void MotionController::hsBuildSegments()
{
    hs_segs_.clear();
    const double td = turn_deg_ * PI_M / 180.0;
    if (path_mode_ == PATH_RECT) {
        int laps = (rect_laps_ > 0) ? rect_laps_ : 1;
        int n = 4 * laps;
        double cum = 0.0;
        for (int i = 0; i < n; ++i) {
            double dist = (i % 2 == 0) ? rect_side_a_ : rect_side_b_;   // 邻边交替 a,b,a,b
            hs_segs_.push_back({dist, cum});
            cum += (double)turn_dir_ * td;     // 本段走完后转 turn_deg(朝下一段)
        }
    } else if (path_mode_ == PATH_ZIGZAG) {
        int legs = (zig_legs_ > 0) ? zig_legs_ : 1;
        double cum = 0.0;
        hs_segs_.push_back({zig_leg_, cum});                        // 第一条长边
        for (int k = 1; k < legs; ++k) {
            // U 形换行: 两个同向转角(中间夹一段横移); 换行方向逐条交替, 形成弓字形覆盖
            double ud = (double)turn_dir_ * ((k % 2 == 1) ? 1.0 : -1.0) * td;
            cum += ud; hs_segs_.push_back({zig_shift_, cum});       // 横移(转一个角后)
            cum += ud; hs_segs_.push_back({zig_leg_,   cum});       // 下一条长边(再转一个角, 方向反转)
        }
    }
    if (hs_segs_.empty()) hs_segs_.push_back({0.0, 0.0});   // 兜底: 空表退化为零距离段(立即收尾)
}

// [rect-z] 进入第 idx 段直行: 设该段目标航向为 hs_yaw0_, 复位 PD/距离锚/软启动, 转 DRIVE。
void MotionController::hsBeginSegment(int idx, const ros::Time& now)
{
    if (idx < 0 || idx >= (int)hs_segs_.size()) { hs_phase_ = HS_DONE; return; }   // 防御: 越界即安全收尾(评审 A/B NIT)
    hs_yaw0_ = wrapPi(hs_base_yaw_ + hs_segs_[idx].cum_off);   // 该段目标航向(供 DRIVE 的 PD 保持)
    hs_x0_   = wheel_pose_state.x; hs_y0_ = wheel_pose_state.y;   // 距离锚归零
    hs_ramp_start_ = now;                                         // 软启动重新渐起(已静止)
    hs_I_ = 0.0; hs_d_filt_ = 0.0;
    hs_e_prev_ = (double)h_sign_ * wrapPi(hs_yaw_rad_ - hs_yaw0_);
    hs_t_prev_sample_ = hs_yaw_stamp_;
    hs_phase_ = HS_DRIVE;
    ROS_WARN("[hs] -> DRIVE seg%d/%d dist=%.2fm target=%.1fdeg",
             idx, (int)hs_segs_.size(), hs_segs_[idx].distance, hs_yaw0_ * 180.0 / PI_M);
}

// [rect-z] 进入原地转向到 target_yaw(闭环到 /imu_heading)。
void MotionController::hsBeginTurn(double target_yaw, const ros::Time& now)
{
    hs_turn_target_      = target_yaw;
    hs_turn_start_       = now;
    hs_turn_settling_    = false;
    hs_turn_last_yaw_    = hs_yaw_rad_;
    hs_turn_last_prog_t_ = now;
    hs_phase_ = HS_TURN;
    double dyaw = wrapPi(target_yaw - hs_yaw_rad_) * 180.0 / PI_M;
    ROS_WARN("[hs] -> TURN to seg%d target=%.1fdeg (need ~%.0fdeg, omega=%.2f, pump OFF)",
             hs_seg_idx_, target_yaw * 180.0 / PI_M, dyaw, hs_turn_omega_);
}

int main(int argc, char **argv)
{
    // [heading-straight-pd] NoSigintHandler + 自管 SIGINT/SIGTERM/SIGHUP：退出前能补发零速。
    // 评审 HIGH(Fable5·失控级)：仅接管 SIGINT 不够——pkill/kill 默认发 SIGTERM、池边 SSH/WiFi 掉线发 SIGHUP，
    // 这两路若不接管会直接杀进程、一条零速不发，叠加驱动无 /cmd_vel 看门狗(D-29)=机器人按末指令续跑直至断电。
    // handler 只置 volatile sig_atomic_t，异步信号安全；三信号共用同一退出路径(while→shutdownStop)。
    ros::init(argc, argv, "motion_controller_node", ros::init_options::NoSigintHandler);
    signal(SIGINT,  hsSigintHandler);
    signal(SIGTERM, hsSigintHandler);
    signal(SIGHUP,  hsSigintHandler);
    signal(SIGPIPE, SIG_IGN);   // master 断连写管道→默认 SIGPIPE 终止，忽略之以便走优雅退出
    ROS_INFO("Hello motion_control!");
    MotionController motion_controller;
    motion_controller.rosNode();
    ros::Rate loop_rate(100);

    while (ros::ok() && !g_hs_stop)
    {
        motion_controller.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 退出前补发零速（仅 heading 模式动作；legacy 为 no-op）
    motion_controller.shutdownStop();
    ros::shutdown();
    return 0;
}
