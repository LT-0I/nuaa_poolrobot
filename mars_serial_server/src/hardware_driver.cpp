#include <hardware_driver.h>
#include <std_msgs/Float32.h>   // [heading-straight-pd] 发布 IMU 融合航向 /imu_heading

using namespace std;
using namespace cv;

// [vcu] 简化版 VCU 限幅辅助（文件局部，避免新增 include 依赖；截断/夹限不改符号）
static inline double vcu_clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int    vcu_clampi(int v, int lo, int hi)         { return v < lo ? lo : (v > hi ? hi : v); }
 
double pose_xx = 0.0, pose_yy = 0.0;
HardwareDriver::HardwareDriver(CallbackAsyncSerial *serial)
{
    key_t key = ftok(PATH_NAME, PROJ_ID);
    int shmid = shmget(key, sizeof(ares_cmd), 0666 | IPC_CREAT);
    ares_cmd = (AresCmd *)shmat(shmid, 0, 0);

    hardware_serial_ = serial;
    memset(&recv_msg, 0, sizeof(recv_msg));
    memset(&send_msg, 0, sizeof(send_msg));
    last_speed_command = 0.0;
    last_omega_command = 0.0;
    memset(&motor_encoder, 0, sizeof(MotorEncoder));  // ← 关键，防未初始化

    

    obst_slow = false;
    obst_match_stop = false;
    obst_scan_stop = false;
    received_motion_pose = false;

    base_pose.x = 0.0;
    base_pose.y = 0.0;
    base_pose.theta = 0.0;

    for (int i = 0; i < 100; i++)
    {
        dtheta_array[i] = 0.0f;
    }
    imu_flag = 0;
    imu_theta = 0.0;
    theta_last = 0.0;
    dtheta_sum = 0.0;
    theta_filter = 0.0;
    update_nums = 0;

    imu_ax = 0.0;
    imu_ay = 0.0;
    imu_az = 9.8;
    imu_wz = 0.0;

    imu_ax_last = 0.0;
    imu_ay_last = 0.0;
    imu_az_last = 9.8;
    imu_wz_last = 0.0;

    encoder_synced_=false;
    skip_encoder_count_=0;

    deviceOnoff_sta.baterry_onoff_sta = BATERRY_ON_STA;

    // right_curr_encoder = 0;
    // left_curr_encoder = 0;
    // right_last_encoder = 0;
    // left_last_encoder = 0;

    // cap1 = VideoCapture(6,CAP_V4L2);
    // cap2 = VideoCapture(7,CAP_V4L2);

    // // cap1.set(CAP_PROP_FOCUS, 0);
    // // cap2.set(CAP_PROP_FOCUS, 2);
    // cap1.set(CAP_PROP_FOURCC,VideoWriter::fourcc('Y','U','Y','V'));
    // cap2.set(CAP_PROP_FOURCC,VideoWriter::fourcc('Y','U','Y','V'));

    // cap1.set(CAP_PROP_FRAME_WIDTH,640);
    // cap1.set(CAP_PROP_FRAME_HEIGHT,480);

    // cap2.set(CAP_PROP_FRAME_WIDTH,640);
    // cap2.set(CAP_PROP_FRAME_HEIGHT,480);

    // cap = VideoCapture(0,CAP_V4L2);
    // if(!cap.isOpened() ){
    //     std::cerr<<"ERROR! Unable to open camera"<<std::endl;
    //     // return 0
    // }
    // // cap.set(CAP_PROP_FOCUS, 0);
    // cap.set(CAP_PROP_FOURCC,VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    // // cap.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
    // cap.set(CAP_PROP_FRAME_WIDTH,3840);
    // cap.set(CAP_PROP_FRAME_HEIGHT,1280);
    dist_datas.data.resize(4);
    // dist_datas.data.push_back(0);
    // dist_datas.data.push_back(0);
    // dist_datas.data.push_back(0);

    cout << "========== 构造函数完成，base_pose已重置 ==========" << endl;
    cout << "base_pose.x = " << base_pose.x << endl;
    cout << "base_pose.y = " << base_pose.y << endl;
    cout << "base_pose.theta = " << base_pose.theta << endl;
    cout << "==================================================" << endl;
}

HardwareDriver::~HardwareDriver()
{
}
void HardwareDriver::rosNode()
{

    // ==========  节点启动时重置里程计 ==========
    base_pose.x = 0.0;
    base_pose.y = 0.0;
    base_pose.theta = 0.0;


     
    // 重置编码器相关标志
    encoder_updated = false;
    wheel_flag = 0;
    imu_flag = 0;

     
    // ========== 重置同步标志 ==========
    encoder_synced_ = false;
    skip_encoder_count_ = 0;
    
    // ==========同步当前编码器值作为基准 ==========
    {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        last_right_econd = (double)motor_encoder.right_speeder / 720.0f;
        last_left_econd = (double)motor_encoder.left_speeder / 720.0f;
        encoder_updated = false;
    }
    
    ROS_INFO("===== 里程计已重置为0 =====");

    nh_.param("wheel_resolution", machine_params.wheel_resolution, 2731);
    nh_.param("wheel_reduction", machine_params.wheel_reduction, 20);
    nh_.param("wheel_diameter", machine_params.wheel_diameter, 0.162);
    nh_.param("wheel_spacing", machine_params.wheel_spacing, 0.2);

    nh_.param("limit_speed", motion_params.limit_speed, 0.5);
    nh_.param("limit_omega", motion_params.limit_omega, 0.5);
    nh_.param("max_acce", motion_params.max_acce, 1.0);
    nh_.param("max_alpha", motion_params.max_alpha, 1.0);

    // ========== [vcu] 简化版 VCU 差速补偿 + /cmd_vel 看门狗 参数 ==========
    // 默认值刻意=保持现行为：omega_gain=6.4 / track_bias=0 / motor_scale=230
    //   -> 与旧码逐位等价（cmd_vel->*6.4->*230、不补偿），0613 直线测试锚定参数不受影响。
    // 改这些参数走 launch / rosparam，现场不需重编。
    nh_.param("omega_gain",      omega_gain_,      6.4);
    nh_.param("track_bias",      track_bias_,      0.0);
    nh_.param("motor_scale",     motor_scale_,     230.0);
    nh_.param("max_track_speed", max_track_speed_, 1.0);
    nh_.param("max_motor_cmd",   max_motor_cmd_,   300);
    nh_.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
    // 安全夹限：track_bias 限到 +-0.15；其余防误设 0/负锁死或失控。
    if (track_bias_ >  0.15) track_bias_ =  0.15;
    if (track_bias_ < -0.15) track_bias_ = -0.15;
    if (max_track_speed_ <= 0.0) max_track_speed_ = 1.0;
    if (max_motor_cmd_   <= 0)   max_motor_cmd_   = 300;
    if (cmd_vel_timeout_ <= 0.0) cmd_vel_timeout_ = 0.5;
    cmd_vel_received_ = false;
    // 版本 banner（ASCII，供现场 grep 自判闸门：VCU-v1-20260613）
    ROS_WARN("[vcu] hardware_driver VCU-v1-20260613 omega_gain=%.2f track_bias=%.3f motor_scale=%.1f max_track_speed=%.2f max_motor_cmd=%d cmd_vel_timeout=%.2f",
             omega_gain_, track_bias_, motor_scale_, max_track_speed_, max_motor_cmd_, cmd_vel_timeout_);

    keyboardVel_cmd_sub_ = nh_.subscribe("cmd_vel", 100, &HardwareDriver::sendKeyboardVelCommand, this);
    autoVel_cmd_sub_ = nh_.subscribe("/autoVel_cmd", 100, &HardwareDriver::sendAutoVelCommand, this);
    // imu_data_sub_           = nh_.subscribe("/handsfree/imu",   1000, &HardwareDriver::GrabImuData,             this);
 
     

    Base_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("wheelPose_sta", 1, true);
    Base_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("wheelTwist_sta", 1, true);
    wheel_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 1, true);
    battery_dat_pub_ = nh_.advertise<mars_msgs::BatteryDatas>("battery_data", 1, true);
    infrared_data_pub_ = nh_.advertise<mars_msgs::Infrared>("infrared_data", 1, true);

    // speed_twist_pub_        = nh_.advertise<geometry_msgs::Twist>("speed_cmd",         1, true);

    imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 1, true);
    sonarA_dat_pub_ = nh_.advertise<sensor_msgs::Range>("sonarA_dat", 1, true);
    sonarB_dat_pub_ = nh_.advertise<sensor_msgs::Range>("sonarB_dat", 1, true);
    Odom_delta_pub = nh_.advertise<mars_msgs::Point>("odom_delta", 1, true);
    ultra_data_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("ultra_data", 1, true);
    // [heading-straight-pd] 附加：发布 IMU 融合航向（度）。纯附加，不改任何现有话题语义。
    imu_heading_pub_ = nh_.advertise<std_msgs::Float32>("imu_heading", 10);
    // advertise 完成后置就绪标志(release)，与 RX 线程的 acquire-load 建立 happens-before，
    // 保证 RX 看到标志为真时 imu_heading_pub_ 已完整构造。此后该发布器单次赋值、不再修改，并发只读安全。
    imu_heading_ready_.store(true, std::memory_order_release);

    image_transport::ImageTransport it(nh_);

    // pub1 = it.advertiseCamera("right/image", 1);
    // pub2 = it.advertiseCamera("left/image", 1);
    pub1 = it.advertise("left/image", 1);
    pub2 = it.advertise("right/image", 1);

    sonarA_datas.header.frame_id = "sonarA";
    sonarA_datas.radiation_type = 0;
    sonarA_datas.field_of_view = 0.7;
    sonarA_datas.min_range = 0.19;
    sonarA_datas.max_range = 4.2;

    sonarB_datas.header.frame_id = "sonarB";
    sonarB_datas.radiation_type = 0;
    sonarB_datas.field_of_view = 0.7;
    sonarB_datas.min_range = 0.19;
    sonarB_datas.max_range = 4.2;
 

    
    ros::Timer imu_timer = nh_.createTimer(
        ros::Duration(0.01),  // 100Hz
        [this](const ros::TimerEvent&) {
            this->UpdateImu();
        }
    );

    ros::Timer ultra_timer = nh_.createTimer(
        ros::Duration(0.3),  // 300ms请求一次
        [this](const ros::TimerEvent&) {
            this->sendUltraCommand();
        }
    );

    // [vcu] /cmd_vel 看门狗 Timer：与 cmd_vel 回调同走单线程 spinner（天然串行化、sendMessage 不竞态）。
    //       20Hz 检查；超 cmd_vel_timeout_ 无新指令则持续下发零速（含泵）-> 治“控制器死/Ctrl+C 后续跑”。
    ros::Timer cmd_watchdog_timer = nh_.createTimer(
        ros::Duration(0.05),  // 20Hz
        [this](const ros::TimerEvent&) {
            this->cmdVelWatchdog();
        }
    );

    ros::spin();
}

bool HardwareDriver::unpack_data(HardwareMessage *recv_msg, uint8_t recv_data)
{
    recv_msg->cnt++;
    // cout << endl;
    //    cout << "C" << endl;
    switch (recv_state)
    {

    case STATE_RECV_FLAGA:
        /* first confirm receive flow */
        if (recv_data == 90)
        {
            memset(recv_msg, 0, sizeof(*recv_msg));
            recv_msg->head.flaga = recv_data;
            recv_msg->check ^= recv_data;
            recv_state = STATE_RECV_FLAGB;
        }
        else
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_FLAGB:
        /* second confirm receive flow */
        if (recv_data == 165)
        {
            if (recv_msg->cnt == (uint8_t)1)
            {
                recv_msg->head.flagb = recv_data;
                recv_msg->check ^= recv_data;
                recv_state = STATE_RECV_MSTYPE;
            }
            else
                recv_state = STATE_RECV_FLAGA;
        }
        else
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_MSTYPE:
        /* data in one group, valid*/
        if (recv_data < DEV_TYPE_MAX)
        {
            recv_msg->head.msgtype = recv_data;
            recv_msg->check ^= recv_data;
            recv_state = STATE_RECV_MSGID;
        }
        /* data not in any group, invalid*/
        else
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_MSGID:
        /* data in one group, valid*/
        if (recv_data < DEV_ID_MAX)
        {
            recv_msg->head.msgid = recv_data;
            recv_msg->check ^= recv_data;
            recv_state = STATE_RECV_LEN;
        }
        /* data not in any group, invalid*/
        else
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_LEN:
        
        recv_msg->head.length = recv_data;
        recv_msg->check ^= recv_data;
        if (recv_msg->head.length == 0)
            recv_state = STATE_RECV_CHECK;
        else if (recv_msg->head.length > 100)
            recv_state = STATE_RECV_FLAGA;
        else
            recv_state = STATE_RECV_PAYLOAD;
        break;

    case STATE_RECV_PAYLOAD:
        recv_msg->payload[recv_msg->payload_count++] = recv_data;
        recv_msg->check ^= recv_data;
        if (recv_msg->payload_count == recv_msg->head.length)
            recv_state = STATE_RECV_CHECK;
        if (recv_msg->payload_count > 100)
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_CHECK:

        if (recv_msg->check == recv_data)
            recv_state = STATE_RECV_TLAGA;
        else
            recv_state = STATE_RECV_FLAGA;
        break;

    case STATE_RECV_TLAGA:
        if (recv_data == 92)
        {
            recv_state = STATE_RECV_TLAGB;
            recv_msg->tail.flaga = recv_data;
        }
        else
        {
            recv_state = STATE_RECV_FLAGA;
        }

        break;

    case STATE_RECV_TLAGB:
        recv_state = STATE_RECV_FLAGA;
        if (recv_data == 197)
        {
            recv_msg->tail.flagb = recv_data;
            return true;
        }

        break;

    default:
        recv_state = STATE_RECV_FLAGA;
        // memset(&recv_msg, 0, sizeof(recv_msg));
    }

    return false;
}

void HardwareDriver::receiveMessage(const char data[], unsigned int len)
{
    uint8_t recv_data;
    ros::Time receive_start_time;
    receive_start_time = ros::Time::now();
    // camera_end_time = ros::Time::now();
    double interval_a = (receive_start_time - receive_end_time).toSec();
    receive_end_time = receive_start_time;

    int aaaa = 0;
    // cout <<"time: "<< interval_a<<endl;
    for (int i = 0; i < len; i++)
    {
        recv_data = data[i];
        // cout<< "recv_data" <<(int)recv_data<< endl;

        if (unpack_data(&recv_msg, recv_data))
        {
            MSG_Type rs_type = (MSG_Type)recv_msg.head.msgtype;
            MSG_ID rs_id = (MSG_ID)recv_msg.head.msgid;

            if (rs_type == CM_DEV_CTL_TYPE)
            {
                switch (rs_id)
                {
                case CM_BATTERY_DATA_ID:
                {
                    //  aaaa = recv_msg.head.length ;
                    // cout << "length" << aaaa << endl;
                    // for(int i = 0; i<8;i++){
                    //     cout <<i<<" "<< (int)recv_msg.payload[i] << endl;
                    // }
                    // cout << "receive motor_encoder " << endl;

                    // memcpy(&motor_encoder, recv_msg.payload, sizeof(MotorEncoder));
                    // cout << "receiveMessage right_speeder"<<motor_encoder.right_speeder << endl;
                    // cout << "receiveMessage left_speeder"<<motor_encoder.left_speeder << endl;
                    // cout << " " << endl;
                    // receiveWheelDatas();
                    //   receiveWheelStates();
                    std::lock_guard<std::mutex> lock(encoder_mutex);
                    memcpy(&motor_encoder, recv_msg.payload, sizeof(MotorEncoder));
                    // ========== 新增 cout 日志 ==========
                    static int encoder_cnt = 0;
                    encoder_cnt++;
                    if (encoder_cnt % 50 == 0) {
                        cout << "[编码器] right=" << motor_encoder.right_speeder 
                            << ", left=" << motor_encoder.left_speeder
                            << ", flagR=" << flagdirR 
                            << ", flagL=" << flagdirL << endl;
                    }
                    // ==================================
                    encoder_updated = true;
                    last_encoder_stamp_ = ros::Time::now();
                    break;
                }
                case CM_MOTOR_CTL_ID:
                    
                    memcpy(&battery_datas, recv_msg.payload, sizeof(mars_msgs::BatteryDatas));
                    // receiveBatteryDatas();
                    break;
                case CM_IMU_DATA_ID:
                    // cout << "receive imu data" << endl;
                    memcpy(&imu_encoder, recv_msg.payload, sizeof(IMUEncoder));
                    receiveImuDatas();
                    //  receiveSensorStates();
                    break;
                case CM_SENSOR_DATA_ID:
                    memcpy(&deviceOnoff_sta, recv_msg.payload, sizeof(mars_msgs::DeviceOnoff));
                    // cout << "deviceOnoff_sta.baterry_onoff_sta = " << (int)deviceOnoff_sta.baterry_onoff_sta <<endl;
                    break;
                case CM_ULTRA_DATA_ID:
                    memcpy(&ultra_datas, recv_msg.payload, sizeof(UltraDatas));
                    // cout << "receive ultra_datas" << endl;
                    // for(int i = 0; i<sizeof(recv_msg.payload);i++){
                    //     cout <<i<<" "<< (int)recv_msg.payload[i] << endl;
                    // }
                    if((int)ultra_datas.ultra_id == 1){
                        dist_datas.data[0] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 2){
                        dist_datas.data[1] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 3){
                        dist_datas.data[2] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 4){
                        dist_datas.data[3] = (int)ultra_datas.ultra_dist;
                    }

                    
                    // cout << "ultra_datas.ultra_id = " << (int)ultra_datas.ultra_id <<endl;
                    // cout << "ultra_datas.ultra_dist = " << (int)ultra_datas.ultra_dist <<endl;
                    // cout << endl;
                    break;
                }
            }
            else if (rs_type == CM_CMD_STA_TYPE)
            {
                switch (rs_id)
                {
                case 7:
                    memcpy(&infrared_datas, recv_msg.payload, 8);
                    // cout << "infrared_datas[1] = " << (int)infrared_datas.data_1<< endl;
                    // cout << "infrared_datas[2] = " << (int)infrared_datas.data_2<< endl;
                    // cout << "infrared_datas[3] = " << (int)infrared_datas.data_3<< endl;
                    // cout << "infrared_datas[4] = " << (int)infrared_datas.data_4<< endl;
                    // cout << "infrared_datas[5] = " << (int)infrared_datas.data_5<< endl;
                    // cout << "infrared_datas[6] = " << (int)infrared_datas.data_6<< endl;
                    // cout << "infrared_datas[7] = " << (int)infrared_datas.data_7<< endl;
                    // cout << "infrared_datas[8] = " << (int)infrared_datas.data_8<< endl;
                    // cout << "----------------------------------" << endl;
                }
            }
        }
    }
}

void HardwareDriver::receiveImuDatas()
{

    // ros::Time current_time = ros::Time::now();

    // if (imu_flag == 0)
    // {
    //     last_time = current_time;
    //     imu_flag = 1;
    // }

    // double dt = (current_time - last_time).toSec();
    // dt = (dt < 0) ? 0.0 : dt;
    // last_time = current_time;

    // float dt=0.01;
    imu_ax = (float)(imu_encoder.x_acc)/1000.0f * 9.8;
    imu_ay = (float)(imu_encoder.y_acc)/1000.0f * 9.8;
    imu_az = (float)(imu_encoder.z_acc)/1000.0f * 9.8;
    imu_wx = (float)(imu_encoder.omg_x)/100.0f;
    imu_wy = (float)(imu_encoder.omg_y)/100.0f;
    imu_wz = -(float)(imu_encoder.omg_z)/100.0f * 1.08;
    // double imu_omega_sta = imu_wz / 180 * PI_M / dt / 10;
    imu_roll = (float)(imu_encoder.roll) / 100.0f;
    imu_pitch = (float)(imu_encoder.pitch) / 100.0f;
    imu_yaw = (float)(imu_encoder.yaw) / 100.0f;
    static int log_cnt = 0;
    if (log_cnt++ % 100 == 0) {
        cout << "IMU att: roll=" << imu_roll
            << ", pitch=" << imu_pitch
            << ", yaw=" << imu_yaw << endl;
    }

    // [heading-straight-pd] 发布融合航向。D-19：本机 roll/yaw 轴互换，真航向=imu_encoder.roll(=imu_roll)，
    // 不是 imu_encoder.yaw。度数原样发，控制器侧做 deg->rad + atan2 wrap。
    // 本发布在串口 RX 线程，advertise 在 rosNode 线程；用 acquire-load 与 advertise 后的 release-store 配对，
    // 建立 happens-before。仅当就绪标志为真(发布器已完整构造)才发，消除发布器竞态读 UB。
    // (注：驱动既有的跨线程共享态/析构竞态非本次改动引入，不在此处理。)
    std_msgs::Float32 imu_heading_msg;
    imu_heading_msg.data = imu_roll;
    if (imu_heading_ready_.load(std::memory_order_acquire))
        imu_heading_pub_.publish(imu_heading_msg);

    // cout << " " << endl;
    // cout << "imu_ax = " << imu_ax << endl;
    // cout << "imu_ay = " << imu_ay << endl;
    // cout << "imu_az = " << imu_az << endl;
    // cout << "ag " << sqrt(imu_ax*imu_ax + imu_ay*imu_ay + imu_az*imu_az) << endl;

    // cout << "imu_wx = " << imu_wx << endl;
    // cout << "imu_wy = " << imu_wy << endl;
    // cout << "imu_wz = " << imu_wz << endl;
    // cout << endl;

    // imu_theta=imu_theta+imu_wz*dt;
    // imu_cnt++;
}

void HardwareDriver::receiveInfDatas()
{
    try
    {
        infrared_data_pub_.publish(infrared_datas);
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }
    // cout << "Infrared_buf "<< Infrared_buf.size() << endl;
    // Infrared_buf.push_back(infrared_datas);
    // if (Infrared_buf.size() > 10)
    // {
    //     Infrared_buf.erase(Infrared_buf.begin());
    // }
}

void HardwareDriver::GrabImuData(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imu_ax = -imu_msg->linear_acceleration.y;
    imu_ay = -imu_msg->linear_acceleration.z;
    imu_az = imu_msg->linear_acceleration.x;

    imu_wx = -imu_msg->angular_velocity.y;
    imu_wy = -imu_msg->angular_velocity.z;
    imu_wz = imu_msg->angular_velocity.x;
}

void HardwareDriver::UpdateImage()
{
    cv::Mat left_frame;
    cv::Mat right_frame;
 
}
 
void HardwareDriver::UpdateUltra()
{
    // cout << "dist_datas " << dist_datas.data[0] <<  " " << dist_datas.data[1] << " " << dist_datas.data[2] << endl;
     // 过滤异常值 65533
    std_msgs::Int32MultiArray safe_data = dist_datas;
    
    for (size_t i = 0; i < safe_data.data.size(); i++) {
        if (safe_data.data[i] == 65533 || safe_data.data[i] > 10000) {
            
            /*safe_data.data[i] = 2000;*/
            ROS_WARN_THROTTLE(5.0, "超声波传感器%d异常设置为安全距离2米", (int)(i+1));
        }
    }
    ultra_data_pub_.publish(safe_data);
}

 /*******  
void HardwareDriver::UpdateImu()
{
    tf::Quaternion imu_quat;

    ros::Time current_time = ros::Time::now();

    // ========== 编码器同步==========
    if (!encoder_synced_) {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        
        // 同步编码器基准值，避免第一帧产生假位移
        last_right_econd = (double)motor_encoder.right_speeder / 720.0f;
        last_left_econd = (double)motor_encoder.left_speeder / 720.0f;
        
        encoder_updated = false;
        encoder_synced_ = true;
        
        ROS_WARN("编码器已同步，丢弃初始数据");
        return;  // 第一帧不产生位移
    }


 
    static int imu_count = 0;
    static int encoder_count = 0;
    static int last_encoder_value = 0;

    imu_count++;
    if (motor_encoder.right_speeder != last_encoder_value) {
        encoder_count++;
        last_encoder_value = motor_encoder.right_speeder;
    }

    // 每秒钟打印一次统计
    if (imu_count % 100 == 0) {
        cout << "IMU cnt: " << imu_count << ", encoder upd cnt: " << encoder_count << endl;
    }

    static bool first_check = true;
    if (first_check) {
        first_check = false;
        if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
            ROS_ERROR("⚠️ base_pose初始值异常！重置为0");
            ROS_ERROR("  当前值: x=%.3f, y=%.3f", base_pose.x, base_pose.y);
            base_pose.x = 0.0;
            base_pose.y = 0.0;
            base_pose.theta = 0.0;
        }
    }

    // 零漂过滤
    if (imu_flag == 0)
    {   
        cout << "【状态】imu_flag == 0，初始化中，跳过计算" << endl;
        last_time = current_time;
        imu_flag = 1;
        return; // 第一次初始化，跳过计算
    }

    // ========== 关键修复：时间间隔保护 ==========
    double dt = (current_time - last_time).toSec();
    
    // 防御性编程：防止ROS时间异常
    if (dt <= 0) {
        // 时间异常：小于等于0
        ROS_WARN_THROTTLE(1.0, "时间异常: dt=%.6f <= 0，使用默认值0.01s", dt);
        cout << "【警告】dt <= 0，跳过计算" << endl;
        dt = 0.01;
        last_time = current_time;
        return;
    }
    
    if (dt > 0.2) {
        // ROS时间可能发生跳跃（如重启、时间同步等）
        ROS_ERROR("时间跳跃异常: dt=%.3fs > 0.2s，跳过本次更新", dt);
        last_time = current_time;
        cout << "【警告】dt > 0.2，跳过计算" << endl;
        // 重置编码器状态，避免错误积分
        wheel_flag = 0;
        imu_flag = 0;
        return;
    }
    
    if (dt > 0.1) {
        // 时间间隔较大但仍在可接受范围
        ROS_WARN_THROTTLE(5.0, "时间间隔较大: dt=%.3fs，限制为0.1s", dt);
        
        dt = 0.1;
    }

    last_time = current_time;

    imu_datas.header.stamp = ros::Time::now();
    imu_datas.header.frame_id = "imu_link";

    double cur_right_econd = (double)motor_encoder.right_speeder/720.0f;
    double cur_left_econd = (double)motor_encoder.left_speeder/720.0f;

 
    
    // 检查编码器值是否异常（防止数据解析错误）
    if (fabs(cur_right_econd) > 10000.0 || fabs(cur_left_econd) > 10000.0) {
        std::cout << "编码器值异常: right=" << cur_right_econd 
              << ", left=" << cur_left_econd 
              << "，使用上次值" << std::endl;
        // cur_right_econd = last_right_econd;
        // cur_left_econd = last_left_econd;
        encoder_updated = false;  // 关键：不更新
        return;
    }

    if(wheel_flag == 0){
        wheel_flag = 1;
        
        last_right_econd = cur_right_econd;
        last_left_econd = cur_left_econd;
        cout << "【状态】wheel_flag == 0，初始化编码器，跳过计算" << endl;
        return; // 第一次初始化编码器，跳过计算
    }
    
    // 检查编码器突变（单次变化过大）
    double right_diff = fabs(cur_right_econd - last_right_econd);
    double left_diff = fabs(cur_left_econd - last_left_econd);

  
    if (right_diff > 100.0 || left_diff > 100.0) {
 

       std::cout << "编码器突变: right_diff=" << right_diff 
              << ", left_diff=" << left_diff 
              << "，可能数据错误" << std::endl;
        // 使用有限的变化量
        double max_diff = 10.0;  // 最大允许单次变化
        if (right_diff > max_diff) {
            cur_right_econd = last_right_econd + 
                ((cur_right_econd > last_right_econd) ? max_diff : -max_diff);
        }
        if (left_diff > max_diff) {
            cur_left_econd = last_left_econd + 
                ((cur_left_econd > last_left_econd) ? max_diff : -max_diff);
        }
    }

   

    // double dright_econd, dleft_econd;
    // if(flagdirR == 1){
    //     dright_econd = cur_right_econd - last_right_econd;
    // }else{
    //     dright_econd = -(cur_right_econd - last_right_econd);
    // }

    // if(flagdirL == 1){
    //     dleft_econd  = cur_left_econd - last_left_econd;
    // }else{
    //     dleft_econd  = -(cur_left_econd - last_left_econd);
    // }

    // cout << "dright_econd =   " << dright_econd << endl;
    // cout << "dleft_econd =   " << dleft_econd << endl;

    double dright_econd = 0.0;
    double dleft_econd = 0.0;
    bool do_update = false;

    {
        std::lock_guard<std::mutex> lock(encoder_mutex);  // 注意：用 encoder_mutex

        if (encoder_updated) {
            double encoder_age = (current_time - last_encoder_stamp_).toSec();
            if (encoder_age < 0.15) {  // 150ms内的数据才认为有效

                if(flagdirR == 1){
                    dright_econd = cur_right_econd - last_right_econd;
                }else{
                    dright_econd = -(cur_right_econd - last_right_econd);
                }

                if(flagdirL == 1){
                    dleft_econd  = cur_left_econd - last_left_econd;
                }else{
                    dleft_econd  = -(cur_left_econd - last_left_econd);
                }

                // ========== 异常 delta 过滤  ==========
                if (fabs(dright_econd) > 1.0 || fabs(dleft_econd) > 1.0) {
                    ROS_ERROR("异常大delta，丢弃: dr=%.3f dl=%.3f", dright_econd, dleft_econd);
                    encoder_updated = false;
                    // 不更新位置，只更新时间
                    last_time = current_time;
                    publishTfTransforms();
                    return;
                }

                last_right_econd = cur_right_econd;
                last_left_econd = cur_left_econd;
                do_update = true;
            }
            else {
                ROS_WARN_THROTTLE(5.0, "编码器数据过旧，丢弃: age=%.3fs", encoder_age);
            }
 

            encoder_updated = false;
             
        }
    }

    if (!do_update) {
        // 没新编码器数据 → 不更新位置，只更新角度
        base_pose.theta += imu_wz * PI_M / 180.0f * dt;
        while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
        while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;
        
        publishTfTransforms();
        // 发布里程计（位置不变）
        wheel_odom.header.stamp = current_time;
        wheel_odom.header.frame_id = "odom";
        wheel_odom.child_frame_id = "base_link";
        wheel_odom.pose.pose.position.x = base_pose.x;
        wheel_odom.pose.pose.position.y = base_pose.y;
        wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);
        wheel_odom_pub_.publish(wheel_odom);
        return;
    }

    cout << "dright_econd =   " << dright_econd << endl;
    cout << "dleft_econd =   " << dleft_econd << endl;

    
    // // 检查增量是否合理
    double max_single_move = 0.2;  // 米
    // double wheel_circumference = 1.5;  // 应该是轮子周长（米/圈）
    double wheel_circumference = M_PI * machine_params.wheel_diameter;  

    // dright_econd 单位是圈数，直接乘以周长得到米
    double abs_right_move = fabs(dright_econd * wheel_circumference);
    double abs_left_move = fabs(dleft_econd * wheel_circumference);

    if (abs_right_move > max_single_move || abs_left_move > max_single_move) {
        cout << "单次移动过大: right=" << abs_right_move 
            << "m, left=" << abs_left_move << "m" << endl;
        
        // 压缩圈数
        double scale = max_single_move / fmax(abs_right_move, abs_left_move);
        dright_econd *= scale;
        dleft_econd *= scale;
    }

    // 计算位移（米）
    base_dist_x = (dright_econd + dleft_econd) / 2.0 * wheel_circumference;
 

    // 检查base_dist_x是否合理
    if (fabs(base_dist_x) > max_single_move) {
        std::cout << "base_dist_x异常: " << base_dist_x << "m > " << max_single_move 
              << "m，跳过本次位置更新" << std::endl;
        base_dist_x = (base_dist_x > 0) ? max_single_move : -max_single_move;
    }

 
 
    static float dist_a = 0.0, dist_b = 0.0;
    dist_a = dright_econd + dist_a;
    dist_b = dleft_econd + dist_b;

    last_right_econd = cur_right_econd;
    last_left_econd = cur_left_econd;

    imu_datas.angular_velocity.x = 0.0;
    imu_datas.angular_velocity.y = 0.0;
    imu_datas.angular_velocity.z = imu_wz * PI_M / 180.0f;

    imu_datas.linear_acceleration.x = -imu_ay - 0.3;
    imu_datas.linear_acceleration.y = -9.81;
    imu_datas.linear_acceleration.z = imu_ax + 0.2;

    cout << "=== 位置更新 ===" << endl;
    cout << "base_dist_x = " << base_dist_x << endl;
    cout << "当前角度 = " << base_pose.theta * 180.0/M_PI << "°" << endl;

    double original_x = base_pose.x;
    double original_y = base_pose.y;
    cout << "原始位置: X=" << original_x << ", Y=" << original_y << endl;

    // 保存当前角度
    double current_theta = base_pose.theta;
    
    // 计算位移
    double delta_x = base_dist_x * cos(current_theta);
    double delta_y = base_dist_x * sin(current_theta);
    
    // ========== 关键修复：位置更新保护 ==========
    bool skip_position_update = false;
    bool reset_odometry = false;
    
    // 1. 检查位移大小
    double max_delta_per_update = 0.2;
    double delta_magnitude = sqrt(delta_x*delta_x + delta_y*delta_y);
    
    if (delta_magnitude > max_delta_per_update) {
        ROS_ERROR("位移异常！大小=%.3fm, dt=%.3fs", delta_magnitude, dt);
        skip_position_update = true;
    }
    
    // 2. 计算新位置
    double new_x = original_x + delta_x;
    double new_y = original_y + delta_y;
    
    // 3. 检查数值有效性
    if (!std::isfinite(new_x) || !std::isfinite(new_y)) {
        ROS_ERROR("位置值非有限数！");
        skip_position_update = true;
    }
    
    // 4. 检查位置突变（核心保护）
    double max_position_jump = 2.0;  // 改为2.0米更严格
    if (!skip_position_update && 
        (fabs(new_x - original_x) > max_position_jump || 
         fabs(new_y - original_y) > max_position_jump)) {
        ROS_ERROR("位置突变异常！ΔX=%.3f, ΔY=%.3f", new_x-original_x, new_y-original_y);
        skip_position_update = true;
    }
    
    // 更新位置（如果检查通过）
    if (!skip_position_update) {
        base_pose.x = new_x;
        base_pose.y = new_y;
    }
    
    // 总是更新角度
    base_pose.theta += imu_wz * PI_M / 180.0f * dt;
    
    // 角度规范化
    while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
    while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;
    
    // 最终检查：如果位置异常，重置
    if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
        ROS_ERROR("位置值超出范围！重置里程计");
        base_pose.x = 0.0;
        base_pose.y = 0.0;
        base_pose.theta = 0.0;
        wheel_flag = 0;
        imu_flag = 0;
        last_right_econd = cur_right_econd;
        last_left_econd = cur_left_econd;
        reset_odometry = true;
    }
    // ============================================

    // 调试输出
    cout << "位移: ΔX=" << delta_x << ", ΔY=" << delta_y << endl;
    if (skip_position_update) {
        cout << "⚠️ 跳过位置更新" << endl;
    }
    if (reset_odometry) {
        cout << "⚠️ 里程计已重置" << endl;
    }
    cout << "新位置: X=" << std::fixed << std::setprecision(6) << base_pose.x 
         << ", Y=" << base_pose.y << endl;
    cout << "新角度: " << base_pose.theta * 180.0/M_PI << "°" << endl;
     cout << "新弧度: " << base_pose.theta << endl;
    // 融合IMU和编码器数据计算角度变化
   
    // double imu_theta_diff = imu_wz * PI_M / 180.0f * dt;
    // double encoder_theta_diff = (dright_econd - dleft_econd) / machine_params.wheel_spacing;

    // // 加权融合：IMU权重0.8，编码器权重0.2
    // // 这样IMU负责快速响应，编码器负责长期稳定性
    // double fused_theta_diff = 0.8 * imu_theta_diff + 0.2 * encoder_theta_diff;

    // base_pose.theta += fused_theta_diff;

    // // 规范化角度到[-π, π]范围
    // while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
    // while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;



    // base_pose.theta = base_pose.theta + imu_wz * PI_M / 180.0f * dt;
    // // 角度规范化（可选，但建议保留）
    // while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
    // while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;

    // base_twist_state.linear.x = base_dist_x/dt;
    // imu_datas.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);


    // cout << "wx: " << imu_datas.angular_velocity.x << endl;
    // cout << "wy: " << imu_datas.angular_velocity.y << endl;
    // cout << "wz: " << imu_datas.angular_velocity.z << endl;
    // cout << "ax: " << imu_datas.linear_acceleration.x << endl;
    // cout << "ay: " << imu_datas.linear_acceleration.y << endl;
    // cout << "az: " << imu_datas.linear_acceleration.z << endl;

    imu_datas.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_datas.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_datas.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_data_pub_.publish(imu_datas);

    // cout << "dt " << dt << endl;
    cout << "base_pose.x  " << base_pose.x << endl;
    cout << "dist_datas " << dist_datas.data[0] <<  " " << dist_datas.data[1] << " " << dist_datas.data[2] << " " <<dist_datas.data[3] << endl;
    // // // // cout << "base_pose.y  " << base_pose.y << endl;
    cout << "base_pose.theta  " << base_pose.theta * 180.0/M_PI << endl;
    cout << "base_pose.theta  rd" << base_pose.theta << endl;
    // cout << "dright_econd = " << dright_econd << endl;
    // cout << "dleft_econd = " << dleft_econd << endl;
    // cout << "dist_a = " << dist_a << endl;
    // cout << "dist_b = " << dist_b << endl;
    // cout << "motor_encoder.right_speeder = " << motor_encoder.right_speeder << endl;
    // cout << "motor_encoder.left_speeder = " << motor_encoder.left_speeder << endl;
    // cout << "base_twist_state.linear.x " << base_twist_state.linear.x << endl;
    cout << "-------------------------------\n" << endl;

    static int debug_count = 0;
    if (debug_count++ % 50 == 0) {
        cout << "=== 坐标系方向测试 ===" << endl;
        cout << "机器人实际向前走时：" << endl;
        cout << "base_pose.x（应该增加）: " << base_pose.x << endl;
        cout << "base_pose.y（应该接近0）: " << base_pose.y << endl;
        cout << "base_pose.theta（方向角）: " << base_pose.theta*180/M_PI << "°" << endl;
        
        // 建议：向前走时，base_pose.x应该增加，base_pose.y接近0
        // 如果base_pose.y在增加，说明X/Y轴定义反了
    }

    wheel_odom.header.stamp = current_time;
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_link";
    wheel_odom.pose.pose.position.x = base_pose.x;
    wheel_odom.pose.pose.position.y = base_pose.y;
    wheel_odom.pose.pose.position.z = 0.0f;
    wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);

    wheel_odom.twist.twist.linear.x = base_twist_state.linear.x;
    wheel_odom.twist.twist.linear.y = 0.0;
    wheel_odom.twist.twist.angular.z = imu_wz * PI_M / 180.0f;
    base_twist_state.angular.z = imu_wz * PI_M / 180.0f;

    wheel_odom_pub_.publish(wheel_odom);
    Base_twist_pub_.publish(base_twist_state);
    Base_pose_pub_.publish(base_pose);

 

     //  发布TF变换
    publishTfTransforms();
}
 ***/

void HardwareDriver::UpdateImu()
{
    tf::Quaternion imu_quat;

    ros::Time current_time = ros::Time::now();

    // ========== 编码器同步==========
    if (!encoder_synced_) {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        
        // 同步编码器基准值，避免第一帧产生假位移
        last_right_econd = (double)motor_encoder.right_speeder / 720.0f;
        last_left_econd = (double)motor_encoder.left_speeder / 720.0f;
        
        // [NEW] 平行变量：600脉冲版本的基准值初始化
        last_right_econd_t0 = (double)motor_encoder.right_speeder / 600.0f;
        last_left_econd_t0 = (double)motor_encoder.left_speeder / 600.0f;
        
        encoder_updated = false;
        encoder_synced_ = true;
        
        ROS_WARN("编码器已同步，丢弃初始数据");
        return;  // 第一帧不产生位移
    }

 
    static int imu_count = 0;
    static int encoder_count = 0;
    static int last_encoder_value = 0;

    imu_count++;
    if (motor_encoder.right_speeder != last_encoder_value) {
        encoder_count++;
        last_encoder_value = motor_encoder.right_speeder;
    }

    // 每秒钟打印一次统计
    if (imu_count % 100 == 0) {
        cout << "IMU cnt: " << imu_count << ", encoder upd cnt: " << encoder_count << endl;
    }

    static bool first_check = true;
    if (first_check) {
        first_check = false;
        if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
            ROS_ERROR("⚠️ base_pose初始值异常！重置为0");
            ROS_ERROR("  当前值: x=%.3f, y=%.3f", base_pose.x, base_pose.y);
            base_pose.x = 0.0;
            base_pose.y = 0.0;
            base_pose.theta = 0.0;
        }
    }

    // 零漂过滤
    if (imu_flag == 0)
    {   
        cout << "【状态】imu_flag == 0，初始化中，跳过计算" << endl;
        last_time = current_time;
        imu_flag = 1;
        return; // 第一次初始化，跳过计算
    }

    // ========== 关键修复：时间间隔保护 ==========
    double dt = (current_time - last_time).toSec();
    
    // 防御性编程：防止ROS时间异常
    if (dt <= 0) {
        // 时间异常：小于等于0
        ROS_WARN_THROTTLE(1.0, "时间异常: dt=%.6f <= 0，使用默认值0.01s", dt);
        cout << "【警告】dt <= 0，跳过计算" << endl;
        dt = 0.01;
        last_time = current_time;
        return;
    }
    
    if (dt > 0.2) {
        // ROS时间可能发生跳跃（如重启、时间同步等）
        ROS_ERROR("时间跳跃异常: dt=%.3fs > 0.2s，跳过本次更新", dt);
        last_time = current_time;
        cout << "【警告】dt > 0.2，跳过计算" << endl;
        // 重置编码器状态，避免错误积分
        wheel_flag = 0;
        imu_flag = 0;
        return;
    }
    
    if (dt > 0.1) {
        // 时间间隔较大但仍在可接受范围
        ROS_WARN_THROTTLE(5.0, "时间间隔较大: dt=%.3fs，限制为0.1s", dt);
        
        dt = 0.1;
    }

    last_time = current_time;

    imu_datas.header.stamp = ros::Time::now();
    imu_datas.header.frame_id = "imu_link";

    // [NEW] 原方案：720脉冲
    double cur_right_econd = (double)motor_encoder.right_speeder / 720.0f;
    double cur_left_econd = (double)motor_encoder.left_speeder / 720.0f;
    
    // [NEW] 平行方案：600脉冲
    double cur_right_econd_t0 = (double)motor_encoder.right_speeder /700.0f;
    double cur_left_econd_t0 = (double)motor_encoder.left_speeder / 700.0f;

 
    
    // 检查编码器值是否异常（防止数据解析错误）
    if (fabs(cur_right_econd) > 10000.0 || fabs(cur_left_econd) > 10000.0) {
        std::cout << "编码器值异常: right=" << cur_right_econd 
              << ", left=" << cur_left_econd 
              << "，使用上次值" << std::endl;
        // cur_right_econd = last_right_econd;
        // cur_left_econd = last_left_econd;
        encoder_updated = false;  // 关键：不更新
        return;
    }

    if(wheel_flag == 0){
        wheel_flag = 1;
        
        last_right_econd = cur_right_econd;
        last_left_econd = cur_left_econd;
        // [NEW] 平行变量基准值初始化
        last_right_econd_t0 = cur_right_econd_t0;
        last_left_econd_t0 = cur_left_econd_t0;
        
        cout << "【状态】wheel_flag == 0，初始化编码器，跳过计算" << endl;
        return; // 第一次初始化编码器，跳过计算
    }
    
    // 检查编码器突变（单次变化过大）
    double right_diff = fabs(cur_right_econd - last_right_econd);
    double left_diff = fabs(cur_left_econd - last_left_econd);

    
    // if (right_diff > 100.0 || left_diff > 100.0) {
 

    //    std::cout << "编码器突变: right_diff=" << right_diff 
    //           << ", left_diff=" << left_diff 
    //           << "，可能数据错误" << std::endl;
    //     // 使用有限的变化量
    //     double max_diff = 10.0;  // 最大允许单次变化
    //     if (right_diff > max_diff) {
    //         cur_right_econd = last_right_econd + 
    //             ((cur_right_econd > last_right_econd) ? max_diff : -max_diff);
    //         // [NEW] 平行变量同步修正
    //         cur_right_econd_t0 = last_right_econd_t0 + 
    //             ((cur_right_econd_t0 > last_right_econd_t0) ? max_diff : -max_diff);
    //     }
    //     if (left_diff > max_diff) {
    //         cur_left_econd = last_left_econd + 
    //             ((cur_left_econd > last_left_econd) ? max_diff : -max_diff);
    //         // [NEW] 平行变量同步修正
    //         cur_left_econd_t0 = last_left_econd_t0 + 
    //             ((cur_left_econd_t0 > last_left_econd_t0) ? max_diff : -max_diff);
    //     }
    // }

    double dright_econd = 0.0;
    double dleft_econd = 0.0;
    // [NEW] 平行变量差值
    double dright_econd_t0 = 0.0;
    double dleft_econd_t0 = 0.0;
    bool do_update = false;

    {
        std::lock_guard<std::mutex> lock(encoder_mutex);  // 注意：用 encoder_mutex

        if (encoder_updated) {
            double encoder_age = (current_time - last_encoder_stamp_).toSec();
            if (encoder_age < 0.15) {  // 150ms内的数据才认为有效

                // [NEW] 原方案差值计算
                if(flagdirR == 1){
                    dright_econd = cur_right_econd - last_right_econd;
                }else{
                    dright_econd = -(cur_right_econd - last_right_econd);
                }

                if(flagdirL == 1){
                    dleft_econd  = cur_left_econd - last_left_econd;
                }else{
                    dleft_econd  = -(cur_left_econd - last_left_econd);
                }
                
                // [NEW] 平行方案差值计算（600脉冲版本）
                if(flagdirR == 1){
                    dright_econd_t0 = cur_right_econd_t0 - last_right_econd_t0;
                }else{
                    dright_econd_t0 = -(cur_right_econd_t0 - last_right_econd_t0);
                }

                if(flagdirL == 1){
                    dleft_econd_t0 = cur_left_econd_t0 - last_left_econd_t0;
                }else{
                    dleft_econd_t0 = -(cur_left_econd_t0 - last_left_econd_t0);
                }

                // ========== 异常 delta 过滤  ==========
                if (fabs(dright_econd) > 1.0 || fabs(dleft_econd) > 1.0) {
                    ROS_ERROR("异常大delta，丢弃: dr=%.3f dl=%.3f", dright_econd, dleft_econd);
                    encoder_updated = false;
                    // 不更新位置，只更新时间
                    last_time = current_time;
                    publishTfTransforms();
                    return;
                }

                last_right_econd = cur_right_econd;
                last_left_econd = cur_left_econd;
                // [NEW] 平行变量基准值更新
                last_right_econd_t0 = cur_right_econd_t0;
                last_left_econd_t0 = cur_left_econd_t0;
                do_update = true;
            }
            else {
                ROS_WARN_THROTTLE(5.0, "编码器数据过旧，丢弃: age=%.3fs", encoder_age);
            }
 

            encoder_updated = false;
             
        }
    }

    if (!do_update) {
        // 没新编码器数据 → 不更新位置，只更新角度
        base_pose.theta += imu_wz * PI_M / 180.0f * dt;
        while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
        while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;
        
        publishTfTransforms();
        // 发布里程计（位置不变）
        wheel_odom.header.stamp = current_time;
        wheel_odom.header.frame_id = "odom";
        wheel_odom.child_frame_id = "base_link";
        wheel_odom.pose.pose.position.x = base_pose.x;
        wheel_odom.pose.pose.position.y = base_pose.y;
        wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);
        wheel_odom_pub_.publish(wheel_odom);
        return;
    }

    cout << "dright_econd =   " << dright_econd << endl;
    cout << "dleft_econd =   " << dleft_econd << endl;
    // [NEW] 打印平行变量差值
    cout << "dright_econd_t0 =   " << dright_econd_t0 << endl;
    cout << "dleft_econd_t0 =   " << dleft_econd_t0 << endl;

    
    // // 检查增量是否合理
    double max_single_move = 0.2;  // 米
    // double wheel_circumference = 1.5;  // 应该是轮子周长（米/圈）
    double wheel_circumference = M_PI * machine_params.wheel_diameter* 0.86;//0.85
    
    // [NEW] 平行方案：周长0.82倍
    double wheel_circumference_t0 = M_PI * machine_params.wheel_diameter * 0.82* (4.1 / 4.7);

    // dright_econd 单位是圈数，直接乘以周长得到米
    double abs_right_move = fabs(dright_econd * wheel_circumference);
    double abs_left_move = fabs(dleft_econd * wheel_circumference);

    if (abs_right_move > max_single_move || abs_left_move > max_single_move) {
        cout << "单次移动过大: right=" << abs_right_move 
            << "m, left=" << abs_left_move << "m" << endl;
        
        // 压缩圈数
        double scale = max_single_move / fmax(abs_right_move, abs_left_move);
        dright_econd *= scale;
        dleft_econd *= scale;
        // [NEW] 平行变量同步压缩
        dright_econd_t0 *= scale;
        dleft_econd_t0 *= scale;
    }

    // 计算位移（米）- 原方案
    base_dist_x = (dright_econd + dleft_econd) / 2.0 * wheel_circumference;
    // ========== 新增 cout 日志 ==========
    static int imu_cnt = 0;
    imu_cnt++;
    if (imu_cnt % 20 == 0) {
        cout << "[IMU] dr=" << dright_econd 
            << ", dl=" << dleft_econd
            << ", dist=" << base_dist_x 
            << ", pose_x=" << base_pose.x << endl;
    }
    // [NEW] 平行方案1：600脉冲 + 原周长
    double base_dist_x_t0 = (dright_econd_t0 + dleft_econd_t0) / 2.0 * wheel_circumference;
    
    // [NEW] 平行方案2：720脉冲 + 周长0.82倍
    double base_dist_x_t1 = (dright_econd + dleft_econd) / 2.0 * wheel_circumference_t0;
 

    // 检查base_dist_x是否合理
    if (fabs(base_dist_x) > max_single_move) {
        std::cout << "base_dist_x异常: " << base_dist_x << "m > " << max_single_move 
              << "m，跳过本次位置更新" << std::endl;
        base_dist_x = (base_dist_x > 0) ? max_single_move : -max_single_move;
        // [NEW] 平行变量同步修正
        base_dist_x_t0 = (base_dist_x_t0 > 0) ? max_single_move : -max_single_move;
        base_dist_x_t1 = (base_dist_x_t1 > 0) ? max_single_move : -max_single_move;
    }

 
 
    static float dist_a = 0.0, dist_b = 0.0;
    dist_a = dright_econd + dist_a;
    dist_b = dleft_econd + dist_b;

    last_right_econd = cur_right_econd;
    last_left_econd = cur_left_econd;
    // [NEW] 平行变量基准值更新（已在上方do_update中更新，此处补充）
    last_right_econd_t0 = cur_right_econd_t0;
    last_left_econd_t0 = cur_left_econd_t0;

    imu_datas.angular_velocity.x = 0.0;
    imu_datas.angular_velocity.y = 0.0;
    imu_datas.angular_velocity.z = imu_wz * PI_M / 180.0f;

    imu_datas.linear_acceleration.x = -imu_ay - 0.3;
    imu_datas.linear_acceleration.y = -9.81;
    imu_datas.linear_acceleration.z = imu_ax + 0.2;

    cout << "=== 位置更新 ===" << endl;
    cout << "base_dist_x = " << base_dist_x << endl;
    // [NEW] 打印平行变量位移
    cout << "base_dist_x_t0(600脉冲) = " << base_dist_x_t0 << endl;
    cout << "base_dist_x_t1(周长0.82) = " << base_dist_x_t1 << endl;
    cout << "当前角度 = " << base_pose.theta * 180.0/M_PI << "°" << endl;

    double original_x = base_pose.x;
    double original_y = base_pose.y;
    cout << "原始位置: X=" << original_x << ", Y=" << original_y << endl;

    // 保存当前角度
    double current_theta = base_pose.theta;
    
    // 计算位移
    double delta_x = base_dist_x * cos(current_theta);
    double delta_y = base_dist_x * sin(current_theta);
    
    // ========== 关键修复：位置更新保护 ==========
    bool skip_position_update = false;
    bool reset_odometry = false;
    
    // 1. 检查位移大小
    double max_delta_per_update = 0.2;
    double delta_magnitude = sqrt(delta_x*delta_x + delta_y*delta_y);
    
    if (delta_magnitude > max_delta_per_update) {
        ROS_ERROR("位移异常！大小=%.3fm, dt=%.3fs", delta_magnitude, dt);
        skip_position_update = true;
    }
    
    // 2. 计算新位置
    double new_x = original_x + delta_x;
    double new_y = original_y + delta_y;
    
    // 3. 检查数值有效性
    if (!std::isfinite(new_x) || !std::isfinite(new_y)) {
        ROS_ERROR("位置值非有限数！");
        skip_position_update = true;
    }
    
    // 4. 检查位置突变（核心保护）
    double max_position_jump = 2.0;  // 改为2.0米更严格
    if (!skip_position_update && 
        (fabs(new_x - original_x) > max_position_jump || 
         fabs(new_y - original_y) > max_position_jump)) {
        ROS_ERROR("位置突变异常！ΔX=%.3f, ΔY=%.3f", new_x-original_x, new_y-original_y);
        skip_position_update = true;
    }
    
    // 更新位置（如果检查通过）
    if (!skip_position_update) {
        base_pose.x = new_x;
        base_pose.y = new_y;
    }
    
    // 总是更新角度
    base_pose.theta += imu_wz * PI_M / 180.0f * dt;
    
    // 角度规范化
    while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
    while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;
    
    // 最终检查：如果位置异常，重置
    if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
        ROS_ERROR("位置值超出范围！重置里程计");
        base_pose.x = 0.0;
        base_pose.y = 0.0;
        base_pose.theta = 0.0;
        wheel_flag = 0;
        imu_flag = 0;
        last_right_econd = cur_right_econd;
        last_left_econd = cur_left_econd;
        last_right_econd_t0 = cur_right_econd_t0;
        last_left_econd_t0 = cur_left_econd_t0;
        reset_odometry = true;
    }
    // ============================================

    // 调试输出
    cout << "位移: ΔX=" << delta_x << ", ΔY=" << delta_y << endl;
    if (skip_position_update) {
        cout << "⚠️ 跳过位置更新" << endl;
    }
    if (reset_odometry) {
        cout << "⚠️ 里程计已重置" << endl;
    }
    cout << "新位置: X=" << std::fixed << std::setprecision(6) << base_pose.x 
         << ", Y=" << base_pose.y << endl;
    cout << "新角度: " << base_pose.theta * 180.0/M_PI << "°" << endl;
    cout << "新弧度: " << base_pose.theta << endl;

    // [NEW] 累积统计和对比输出
    static double sum_original = 0.0;   // 原方案累积
    static double sum_t0 = 0.0;         // 600脉冲累积
    static double sum_t1 = 0.0;         // 周长0.82累积
    static int test_frame = 0;
    
    sum_original += fabs(base_dist_x);
    sum_t0 += fabs(base_dist_x_t0);
    sum_t1 += fabs(base_dist_x_t1);
    test_frame++;
    
    // 每100帧打印对比（约2秒）
    if (test_frame % 100 == 0) {
        cout << "===== 平行对比 =====" << endl;
        cout << "[原方案] 720脉冲+原周长: " << sum_original << " 米" << endl;
        cout << "[猜测A] 600脉冲+原周长: " << sum_t0 << " 米" << endl;
        cout << "[猜测B] 720脉冲+周长0.82: " << sum_t1 << " 米" << endl;
        cout << "当前 base_pose.x = " << base_pose.x << " 米" << endl;
        cout << "==================" << endl;
    }
    
    // 检测运动停止，打印最终结果
    static double last_x_test = 0;
    static int stop_test_cnt = 0;
    if (fabs(base_dist_x) < 0.001) {
        stop_test_cnt++;
        if (stop_test_cnt > 50 && fabs(base_pose.x - last_x_test) < 0.01) {
            cout << "########## 最终验证结果 ##########" << endl;
            cout << "[原方案] 累积位移 = " << sum_original << " 米" << endl;
            cout << "[猜测A] 600脉冲累积 = " << sum_t0 << " 米" << endl;
            cout << "[猜测B] 周长0.82累积 = " << sum_t1 << " 米" << endl;
            cout << "当前 base_pose.x = " << base_pose.x << " 米" << endl;
            cout << "################################" << endl;
        }
    } else {
        stop_test_cnt = 0;
        last_x_test = base_pose.x;
    }

    imu_datas.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_datas.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_datas.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_data_pub_.publish(imu_datas);

    // cout << "dt " << dt << endl;
    cout << "base_pose.x  " << base_pose.x << endl;
    cout << "dist_datas " << dist_datas.data[0] <<  " " << dist_datas.data[1] << " " << dist_datas.data[2] << " " <<dist_datas.data[3] << endl;
    // // // // cout << "base_pose.y  " << base_pose.y << endl;
    cout << "base_pose.theta  " << base_pose.theta * 180.0/M_PI << endl;
    cout << "base_pose.theta  rd" << base_pose.theta << endl;
    cout << "-------------------------------\n" << endl;

    static int debug_count = 0;
    if (debug_count++ % 50 == 0) {
        cout << "=== 坐标系方向测试 ===" << endl;
        cout << "机器人实际向前走时：" << endl;
        cout << "base_pose.x（应该增加）: " << base_pose.x << endl;
        cout << "base_pose.y（应该接近0）: " << base_pose.y << endl;
        cout << "base_pose.theta（方向角）: " << base_pose.theta*180/M_PI << "°" << endl;
        
        // 建议：向前走时，base_pose.x应该增加，base_pose.y接近0
        // 如果base_pose.y在增加，说明X/Y轴定义反了
    }

 
 

    wheel_odom.header.stamp = current_time;
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_link";
    wheel_odom.pose.pose.position.x = base_pose.x;
    wheel_odom.pose.pose.position.y = base_pose.y;
    wheel_odom.pose.pose.position.z = 0.0f;
    wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);

    wheel_odom.twist.twist.linear.x = base_twist_state.linear.x;
    wheel_odom.twist.twist.linear.y = 0.0;
    wheel_odom.twist.twist.angular.z = imu_wz * PI_M / 180.0f;
    base_twist_state.angular.z = imu_wz * PI_M / 180.0f;

    wheel_odom_pub_.publish(wheel_odom);
    Base_twist_pub_.publish(base_twist_state);
    Base_pose_pub_.publish(base_pose);

 

     //  发布TF变换
    publishTfTransforms();
}
 

// 实现TF发布函数
 
/*********/
void HardwareDriver::publishTfTransforms() {
    ros::Time current_time = ros::Time::now();
    
    static float last_base_theta = 0.0f;
    static bool first_time = true;
    
    // ========== 关键修复：发布前的数据检查 ==========
    
    // 1. 检查位置值是否有效
    if (!std::isfinite(base_pose.x) || !std::isfinite(base_pose.y) || 
        !std::isfinite(base_pose.theta)) {
        ROS_ERROR_THROTTLE(1.0, "TF发布：位置/角度值非有限数，跳过TF发布");
        return;  // 直接返回，不发布异常TF
    }
    
    // 2. 检查位置值是否合理（防止984023.05这样的异常）
    if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
        ROS_ERROR_THROTTLE(1.0, "TF发布：位置值异常 x=%.1f, y=%.1f，跳过TF发布", 
                          base_pose.x, base_pose.y);
        return;  // 直接返回，不发布异常TF
    }
    
    // 3. 检查角度是否合理
    if (fabs(base_pose.theta) > 10.0 * M_PI) {  // 绝对值大于10π（5圈）
        ROS_ERROR_THROTTLE(1.0, "TF发布：角度异常 %.1f°，跳过TF发布", 
                          base_pose.theta * 180.0/M_PI);
        return;
    }
    
    // 4. 角度突变检测（改进版）
    if (!first_time) {
        float theta_diff = base_pose.theta - last_base_theta;
        // 规范化到[-π, π]
        while (theta_diff > M_PI) theta_diff -= 2*M_PI;
        while (theta_diff < -M_PI) theta_diff += 2*M_PI;
        
        if (fabs(theta_diff) > 0.5f) {  // 角度变化超过0.5弧度（约28.6度）
            ROS_WARN_THROTTLE(1.0, "TF发布：角度突变 %.1f° -> %.1f° (差值=%.1f°)，使用上次角度", 
                            last_base_theta * 180.0/M_PI,
                            base_pose.theta * 180.0/M_PI,
                            theta_diff * 180.0/M_PI);
            // 实际处理：使用上次的角度值
            base_pose.theta = last_base_theta;
        }
    } else {
        first_time = false;
    }
    
    last_base_theta = base_pose.theta;
    // ============================================
    
    // 1. 发布odom->base_link
    try {
        tf::Transform odom_to_base;
        odom_to_base.setOrigin(tf::Vector3(
            base_pose.x, 
            base_pose.y, 
            0.0));
        
        tf::Quaternion q = tf::createQuaternionFromYaw(base_pose.theta);
        odom_to_base.setRotation(q);
        
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(odom_to_base, current_time, "odom", "base_link"));
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "发布odom->base_link TF失败: %s", e.what());
    }
    
    // 2. 发布map->odom（固定变换）
    try {
        tf::Transform map_to_odom;
        map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
        map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
        
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(map_to_odom, current_time, "map", "odom"));
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "发布map->odom TF失败: %s", e.what());
    }
    
    // 3. 发布base_link->sonar_frame（固定变换）
    try {
        tf::Transform base_to_sonar;
        base_to_sonar.setOrigin(tf::Vector3(0.1, 0, 0.05));  
        base_to_sonar.setRotation(tf::Quaternion(0, 0, 0, 1));
        
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_to_sonar, current_time, "base_link", "sonar_frame"));
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "发布base_link->sonar_frame TF失败: %s", e.what());
    }
    
    // 4. 发布base_link->imu_link（固定变换）
    try {
        tf::Transform base_to_imu;
        base_to_imu.setOrigin(tf::Vector3(0, 0, 0.1));  
        base_to_imu.setRotation(tf::Quaternion(0, 0, 0, 1));
        
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(base_to_imu, current_time, "base_link", "imu_link"));
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "发布base_link->imu_link TF失败: %s", e.what());
    }
}

bool HardwareDriver::packData(HardwareMessage *send_msg, uint8_t *send_data)
{
    uint8_t pay_len = send_msg->head.length;
    if (send_msg == NULL)
        return false;
    if (send_msg->head.length > RS_PAYLOAD_MAX)
        return false;

    memset(send_data, 0, RS_DATAFLOW_MAX);

    send_data[0] = send_msg->head.flaga;
    send_data[1] = send_msg->head.flagb;
    send_data[2] = send_msg->head.msgtype;
    send_data[3] = send_msg->head.msgid;
    send_data[4] = send_msg->head.length;
    // cout << "send_data[4]" << (int)send_data[4] << endl;
    memcpy(&send_data[PACKET_PAYLOAD_INDEX], send_msg->payload, pay_len);
    // cout << "send_data[5]" << (int)send_data[5] << endl;
    send_data[PACKET_PAYLOAD_INDEX + pay_len] = send_msg->check;
    send_data[PACKET_PAYLOAD_INDEX + pay_len + 1] = send_msg->tail.flaga;
    send_data[PACKET_PAYLOAD_INDEX + pay_len + 2] = send_msg->tail.flagb;

    return true;
}

void HardwareDriver::sendMessage(int msgtype, int msgid, uint8_t *payload, uint8_t len)
{
    unsigned short send_length = 5 + len + 1 + 2;
    uint8_t send_data[RS_DATAFLOW_MAX];
    // cout<<"send_data"<<endl;
    HardwareMessage send_msg(msgtype, msgid, payload, len);
    if (packData(&send_msg, send_data))
    {
        hardware_serial_->write((char *)send_data, send_length);
        // cout << "aaaaaaaaaaa" << endl;
        //    for(int i = 0; i<30;i++)
        //     {
        //         cout<<"send_data"<<i<<"   "<<(int)send_data[i]<<endl;
        //     }
    }
}

/**
 * @brief :  Speed Smooth(线速度平滑)
 *
 * @param :     expected_speed(期望线速度), trace_acce(所用线加速度)
 *
 * @return :    current_speed_command(下发线速度)
 */
double
HardwareDriver::speedSmooth(double expected_speed, double used_acce)
{
    double current_speed_command = 0, range = 0.02;
    if ((expected_speed - last_speed_command) > used_acce * range)
    {
        current_speed_command = last_speed_command + used_acce * range;
    }
    else if ((expected_speed - last_speed_command) < -used_acce * range)
    {
        current_speed_command = last_speed_command - used_acce * range;
    }
    else
    {
        current_speed_command = expected_speed;
    }
    last_speed_command = current_speed_command;

    return current_speed_command;
}

/**
 * @brief :  Omega Smooth(线速度平滑)
 *
 * @param :     expected_speed(期望线速度), trace_acce(所用线加速度)
 *
 * @return :    current_speed_command(下发线速度)
 */
double
HardwareDriver::omegaSmooth(double expected_omega, double used_alpha)
{
    double current_omega_command, range = 0.02;
    if ((expected_omega - last_omega_command) > used_alpha * range)
    {
        current_omega_command = last_omega_command + used_alpha * range;
    }
    else if ((expected_omega - last_omega_command) < -used_alpha * range)
    {
        current_omega_command = last_omega_command - used_alpha * range;
    }
    else
    {
        current_omega_command = expected_omega;
    }
    last_omega_command = current_omega_command;
    return current_omega_command;
}

void HardwareDriver::receiveWheelDatas()
{
    // double cur_right_econd = (double)motor_encoder.right_speeder/37000.0f;
    // double cur_left_econd = (double)motor_encoder.left_speeder/37000.0f;
    // if(wheel_flag == 0){
    //     wheel_flag = 1;
    //     last_right_econd = cur_right_econd;
    //     last_left_econd  = cur_left_econd;
    // }

    // double dright_econd = cur_right_econd - last_right_econd;
    // double dleft_econd  = cur_left_econd - last_left_econd;

    // base_dist_x = (dright_econd + dleft_econd)/2.0;

    // last_right_econd = cur_right_econd;
    // last_left_econd = cur_left_econd;

    // curr_time = ros::Time::now();
    // double dt = (curr_time - las_time).toSec();
    // las_time = curr_time;

    // cout << "dright_econd " << dright_econd << endl;
    // cout << "dleft_econd " << dleft_econd << endl;
    // cout << "dist = " << dist << endl;
    // cout << endl;
    // cout << "dis = " << dis << endl;
}

void HardwareDriver::sendAutoVelCommand(const geometry_msgs::Twist &cmd_vel_msg)
{
    // [vcu] 喂看门狗：autoVel 也是电机指令源，刷新时间戳避免看门狗误停 autoVel（本路径其余逻辑本轮不改）
    last_cmd_vel_time_ = ros::Time::now();
    cmd_vel_received_  = true;
    geometry_msgs::Twist cmd_vel = cmd_vel_msg;
    if(cmd_vel.linear.x < 0.5 && cmd_vel.linear.x > 0.0){
        cmd_vel.linear.x = 0.5;
    }else if(cmd_vel.linear.x > -0.5 && cmd_vel.linear.x < 0.0){
        cmd_vel.linear.x = -0.5;
    }
    if(cmd_vel.angular.z < 0.05 && cmd_vel.angular.z > 0.0){
        cmd_vel.angular.z = 0.05;
    }else if(cmd_vel.angular.z > -0.05 && cmd_vel.angular.z < 0.0){
        cmd_vel.angular.z = -0.05;
    }
    curr_time = ros::Time::now();
    double dt = (curr_time - las_time).toSec();
    las_time = curr_time;
    // cout << "dt " << dt << endl;

    double current_speed_command = cmd_vel.linear.x * 0.2;
    double current_omega_command = cmd_vel.angular.z * 6.4;

    // cout << "keybot dt " << dt << endl;
    // cout << "cmd_vel.linear.x = " << cmd_vel.linear.x << endl;
    // cout << "base_twist_state.linear.x = " << base_twist_state.linear.x << endl;

    // cout << "current_omega_command = " << current_omega_command << endl;
    // cout << "base_twist_state.angular.z = " << base_twist_state.angular.z << endl;

    // speed_receive = current_speed_command;
    // omega_receive = current_omega_command;

    double right_speed = current_speed_command + current_omega_command * machine_params.wheel_spacing / 2;
    double left_speed = current_speed_command - current_omega_command * machine_params.wheel_spacing / 2;

    
    if(right_speed >= 0){
        flagdirR = 1;
    }else{
        flagdirR = -1;
    }

    if(left_speed >= 0){
        flagdirL = 1;
    }else{
        flagdirL = -1;
    }
    // cout << "current_speed_command " << current_speed_command << endl;
    // cout << "current_omega_command" << current_omega_command << endl;
    // cout << "right_speed " << right_speed << endl;
    // cout << "left_speed " << left_speed << endl;

    motor_command.right_speed = right_speed * 230; //-left_speed * 230;
    motor_command.left_speed = -left_speed * 230; //right_speed * 230;
    // motor_command.spin_speed = fabs((current_speed_command)*150) + fabs((current_omega_command)*50);
    //motor_command.spinl_speed = cmd_vel.linear.z;
    //motor_command.spinr_speed = cmd_vel.linear.z;
    // if(motor_command.spinl_speed > 90){
    //     motor_command.spinl_speed = 90;
    // }
    // motor_command.spinl_speed = 0;

    // cout << " " << endl;
    // cout << "motor_command.right_speed = " << motor_command.right_speed << endl;
    // cout << "motor_command.left_speed = " << motor_command.left_speed << endl;
    // cout << "motor_command.spin_speed = " << motor_command.spin_speed << endl;
    // cout << "dis = " << dis << endl;
    // cout << "------------" << endl;

    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}

void HardwareDriver::sendKeyboardVelCommand(const geometry_msgs::Twist &cmd_vel)
{
    ares_cmd->mars_ctlmode_cmd = MARS_GET_MANUAL_CMD;
    curr_time = ros::Time::now();
    double dt = (curr_time - las_time).toSec();
    las_time = curr_time;

    // [vcu] 喂看门狗：每收到一条 /cmd_vel 刷新时间戳（线程1 spinner，单线程安全）
    last_cmd_vel_time_ = curr_time;
    cmd_vel_received_  = true;

    // 1. 上层速度命令；转向增益由写死 6.4 改为可调 omega_gain_（默认 6.4，保持历史标定）
    const double v = cmd_vel.linear.x;
    const double w = cmd_vel.angular.z * omega_gain_;

    // 2. 差速分解
    const double half_track = machine_params.wheel_spacing / 2.0;
    double right_speed = v + w * half_track;
    double left_speed  = v - w * half_track;

    // 3. 左右履带静态偏置补偿（track_bias_ 已在 rosNode() 夹到 +-0.15；默认 0 -> 不补偿）
    //    track_bias>0：右减左增（治“总向左偏”）；<0 反之。
    right_speed *= (1.0 - track_bias_);
    left_speed  *= (1.0 + track_bias_);

    // 4. 方向标志（喂里程计）：用补偿后的速度符号，与实际下发命令一致；|bias|<=0.15 不翻号。
    flagdirR = (right_speed >= 0) ? 1 : -1;
    flagdirL = (left_speed  >= 0) ? 1 : -1;

    // 5. 履带目标速度限幅（默认 max_track_speed_=1.0；实测最坏正常峰值 ~0.65-0.82 m/s，仅兜失控、正常不触发）
    right_speed = vcu_clampd(right_speed, -max_track_speed_, max_track_speed_);
    left_speed  = vcu_clampd(left_speed,  -max_track_speed_, max_track_speed_);

    // 6. 转电机命令：230 -> 可调 motor_scale_；左轮取负（与旧码一致）。
    //    (int) 截断向零，与旧 `motor_command.right_speed = double` 的隐式 double->int 转换等价 -> 默认参数逐位相同。
    //    max_motor_cmd_ 是最终电机命令上限：默认 300 下、轨速限幅(1.0×230=230)先于它触发，故它是
    //    motor_scale_ 被人为调高(调试)时的兜底；MCU 真实安全上限待现场/固件确认(见 OPEN_ITEMS)。
    int right_cmd = vcu_clampi((int)( right_speed * motor_scale_), -max_motor_cmd_, max_motor_cmd_);
    int left_cmd  = vcu_clampi((int)(-left_speed  * motor_scale_), -max_motor_cmd_, max_motor_cmd_);

    motor_command.right_speed = right_cmd;
    motor_command.left_speed  = left_cmd;

    // 7. 水泵：cmd_vel.linear.y in [0,1] -> spin [0,90]（保持原逻辑；打印改 ASCII）
    motor_command.spinl_speed = 0;
    motor_command.spinr_speed = 0;
    double pump_speed = cmd_vel.linear.y;
    if (pump_speed > 0.01) {
        motor_command.spinl_speed = (int)(pump_speed * 90.0);
        motor_command.spinr_speed = (int)(pump_speed * 90.0);
        if (motor_command.spinl_speed > 90) motor_command.spinl_speed = 90;
        if (motor_command.spinr_speed > 90) motor_command.spinr_speed = 90;
        static int log_count = 0;
        if (log_count++ % 50 == 0) {
            ROS_INFO("[vcu] pump on: %.0f%%", pump_speed * 100);
        }
    }

    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}

// [vcu] 统一停车帧：左右电机 + 水泵全部置零。仅在线程1（看门狗 Timer / 关停 join 后）调用，单线程安全。
void HardwareDriver::sendStopCommand()
{
    motor_command.right_speed = 0;
    motor_command.left_speed  = 0;
    motor_command.spinl_speed = 0;
    motor_command.spinr_speed = 0;
    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}

// [vcu] /cmd_vel 看门狗回调（20Hz，spinner 线程）：超时无新指令则下发零速。
// 从未收到过指令时不主动发帧（避免开机/idle 抢串口）；autoVel 也刷新时间戳故不会误停 autoVel。
// 边沿触发——进入超时态立即停一次 + 之后 ~5Hz 持续重申(幂等、防丢帧)，而非每拍 20Hz 灌帧；
//   正常 idle 待命不再被持续高频干预。
// ⚠ 安全边界：本看门狗只在 PC 端发"速度=0"帧；"零帧=真停车"取决于 MCU 行为、未经现场证实，
//   不消除失控风险，须现场实测(杀控制器→看是否真停+录像)，唯一可靠停车仍是断电。
void HardwareDriver::cmdVelWatchdog()
{
    if (!cmd_vel_received_) return;
    double age = (ros::Time::now() - last_cmd_vel_time_).toSec();
    if (age > cmd_vel_timeout_) {
        if (!wd_timed_out_ || (++wd_tick_ % 4 == 0)) {   // 进入即停 + 之后 20Hz/4≈5Hz 重申
            sendStopCommand();
        }
        if (!wd_timed_out_) {
            wd_timed_out_ = true;
            ROS_WARN("[vcu] cmd_vel watchdog: no cmd for %.2fs, motors+pump stopped (re-assert ~5Hz)", age);
        }
    } else {
        wd_timed_out_ = false;
        wd_tick_ = 0;
    }
}

void HardwareDriver::sendUltraCommand(){
    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = 4;
    uint8_t ultra_command = 1;
    
    // 轮流请求4个超声波传感器
    static int ultra_counter = 0;  // 计数器 0,1,2,3 轮流
    
    // 根据计数器选择要请求的超声波ID
    if (ultra_counter == 0) {
        ultra_command = 1;  // 请求超声波1 (右侧前)
        // cout << "send ultra 1 (右侧前)" << endl;
    } else if (ultra_counter == 1) {
        ultra_command = 2;  // 请求超声波2 (右侧后)
        // cout << "send ultra 2 (右侧后)" << endl;
    } else if (ultra_counter == 2) {
        ultra_command = 3;  // 请求超声波3 (前方左红外)
        // cout << "send ultra 3 (前方左红外)" << endl;
    } else {
        ultra_command = 4;  // 请求超声波4 (前方右红外)
        // cout << "send ultra 4 (前方右红外)" << endl;
    }
    
    // 发送命令
    sendMessage(send_type, send_id, (uint8_t *)&ultra_command, (uint8_t)sizeof(ultra_command));
    
    // 计数器递增，实现轮流
    ultra_counter++;
    if (ultra_counter >= 4) {
        ultra_counter = 0;  // 重置为0，重新开始
    }
}



void HardwareDriver::sendTestVelCommand()
{

    static int test_ward = 0;

    if (test_ward == 0)
    {
        test_speed = test_speed + 0.0005;
        test_omega = test_omega;
        if (test_speed > 0.5)
        {
            test_ward = 1;
        }
    }
    else
    {
        test_speed = test_speed - 0.0005;
        test_omega = test_omega;
        if (test_speed < -0.5)
        {
            test_ward = 0;
        }
    }

    // cout << "current_speed_command = " << test_speed << endl;
    // cout << "current_omega_command = " << test_omega << endl;
    // speed_receive = current_speed_command;
    // omega_receive = current_omega_command;

    double right_speed = test_speed + (test_omega * 0.37 / 2);
    double left_speed = test_speed - (test_omega * 0.37 / 2);

    motor_command.right_speed = right_speed * 80;
    motor_command.left_speed = -left_speed * 80;

    // cout << "motor_command.right_speed = " << motor_command.right_speed << endl;
    // cout << "motor_command.left_speed = " << motor_command.left_speed << endl;
    // cout << "dis = " << dis << endl;
    // cout << "------------" << endl;

    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}