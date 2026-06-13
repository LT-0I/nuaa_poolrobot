#include <hardware_driver.h>

const int HardwareDriver::STARTUP_PROTECT_THRESHOLD = 50;
const int HardwareDriver::STATIONARY_THRESHOLD = 20;
const double HardwareDriver::MOTION_THRESHOLD = 0.01;

using namespace std;
using namespace cv;

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

    last_right_econd = 0.0;
    last_left_econd = 0.0;

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


 

      // 添加日志初始化（在构造函数末尾）
    anomaly_count_ = 0;
    
    // 创建日志文件，以时间戳命名
    char filename[256];
    time_t now = time(0);
    struct tm *tm_now = localtime(&now);
    // 方法1：用户主目录（推荐）
    const char* home_dir = getenv("HOME");
    if (home_dir) {
        sprintf(filename, "%s/.ros/anomaly_logs/anomaly_log_%04d%02d%02d_%02d%02d%02d.txt",
                home_dir,
                tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday,
                tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
        
        // 确保目录存在
        char dir_path[256];
        sprintf(dir_path, "%s/.ros/anomaly_logs", home_dir);
        mkdir(dir_path, 0755);
    } else {
        // 回退方案
        sprintf(filename, "/var/log/anomaly_log_%04d%02d%02d_%02d%02d%02d.txt",
                tm_now->tm_year+1900, tm_now->tm_mon+1, tm_now->tm_mday,
                tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
    }

    
    anomaly_log_.open(filename, std::ios::out | std::ios::app);
    if (anomaly_log_.is_open()) {
        anomaly_log_ << "=== 异常日志开始 ===" << std::endl;
        anomaly_log_ << "时间戳,异常类型,异常值,base_pose.x,base_pose.y,dt,"
                    << "右编码器,左编码器,imu_wz,备注" << std::endl;
    }


    // ==========IMU数据包日志初始化 ==========
    // ==========环形缓冲区和异常触发状态初始化 ==========
    imu_ring_buffer_.clear();
    is_logging_anomaly_ = false;
    anomaly_log_remaining_frames_ = 0;
    anomaly_trigger_count_ = 0;
    
    // ========== 先初始化异常触发日志 ==========
    initAnomalyTriggerLog();  // 这个函数内部会创建自己的日志文件
    
    // ========== 然后初始化常规IMU日志 ==========
    imu_packet_count_ = 0;
    imu_anomaly_count_ = 0;

    // ========== 初始化标志 ==========
    encoder_initialized_ = false;
    last_encoder_time_ = ros::Time::now();
    
    // 初始化编码器值为0
    motor_encoder.right_speeder = 0;
    motor_encoder.left_speeder = 0;
    
    // 生成常规IMU日志文件名
    time_t now_time = time(0);
    struct tm *time_info = localtime(&now_time);
    const char* home_path = getenv("HOME");
    
    std::string log_dir;
    if (home_path) {
        log_dir = std::string(home_path) + "/.ros/imu_packet_logs/";
        mkdir(log_dir.c_str(), 0755);
    } else {
        log_dir = "/var/log/imu_packet_logs/";
        mkdir(log_dir.c_str(), 0755);
    }
    
    char log_filename[256];
    snprintf(log_filename, sizeof(log_filename),
             "%s/imu_packets_%04d%02d%02d_%02d%02d%02d.csv",
             log_dir.c_str(),
             time_info->tm_year+1900, time_info->tm_mon+1, time_info->tm_mday,
             time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
    
    // 打开常规IMU日志文件
    imu_log_.open(log_filename, std::ios::out);
    
    if (imu_log_.is_open()) {
        imu_log_ << "=== IMU + Encoder Raw Data Packet Log (Continuous) ===" << std::endl;
        imu_log_ << "Note: This is continuous data stream. For anomaly-triggered detailed logs," << std::endl;
        imu_log_ << "      check anomaly_trigger_*.csv files." << std::endl;
        imu_log_ << "==================================" << std::endl;
        imu_log_ << "PacketNo,SystemTime,ROSTime,Status,AnomalyType,"
                 << "x_acc_raw,y_acc_raw,z_acc_raw,"
                 << "omg_x_raw,omg_y_raw,omg_z_raw,"
                 << "right_encoder_raw,left_encoder_raw,"   
                 << "right_speed_rpm,left_speed_rpm,"      
                 << "Comments" << std::endl;
        
        ROS_INFO("常规IMU+编码器数据包日志已创建: %s", log_filename);
    } else {
        ROS_ERROR("无法创建IMU数据包日志文件: %s", log_filename);
    }
    
    last_valid_pose_.x = 0;
    last_valid_pose_.y = 0;
    last_valid_pose_.theta = 0;
    last_valid_time_ = ros::Time::now();

    imu_bias_ = 0.0;
    bias_calib_count_ = 0;
    bias_calibrated_ = false;
    stationary_count_ = 0;
    is_stationary_ = false;
    startup_protect_count_ = 0;

    current_cmd_linear_x = 0.0;
    current_cmd_angular_z = 0.0;
}

HardwareDriver::~HardwareDriver()
{
    if (anomaly_log_.is_open()) {
        anomaly_log_ << "=== 异常日志结束，共记录 " << anomaly_count_ << " 次异常 ===" << std::endl;
        anomaly_log_.close();
    }

    if (imu_log_.is_open()) {
        // 获取当前时间
        time_t raw_time;
        struct tm *timeinfo;
        char time_buffer[80];
        time(&raw_time);
        timeinfo = localtime(&raw_time);
        strftime(time_buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
        
        float anomaly_rate = 0.0f;
        if (imu_packet_count_ > 0) {
            anomaly_rate = (imu_anomaly_count_ * 100.0f) / imu_packet_count_;
        }
        
        // 使用英文结束信息
        imu_log_ << "=== IMU Raw Data Packet Log End ===" << std::endl;
        imu_log_ << "TotalPackets: " << imu_packet_count_ << std::endl;
        imu_log_ << "AnomalyPackets: " << imu_anomaly_count_ << std::endl;
        imu_log_ << "AnomalyRate: " << std::fixed << std::setprecision(2) << anomaly_rate << "%" << std::endl;
        imu_log_ << "EndTime: " << time_buffer << std::endl;
        
        imu_log_.close();
        
        ROS_INFO("IMU原始数据包日志已关闭，共记录 %d 个数据包，异常率 %.2f%%",
                imu_packet_count_, anomaly_rate);
    }
}
void HardwareDriver::rosNode()
{
    nh_.param("wheel_resolution", machine_params.wheel_resolution, 2731);
    nh_.param("wheel_reduction", machine_params.wheel_reduction, 20);
    nh_.param("wheel_diameter", machine_params.wheel_diameter, 0.162);
    nh_.param("wheel_spacing", machine_params.wheel_spacing, 0.2);

    nh_.param("limit_speed", motion_params.limit_speed, 0.5);
    nh_.param("limit_omega", motion_params.limit_omega, 0.5);
    nh_.param("max_acce", motion_params.max_acce, 1.0);
    nh_.param("max_alpha", motion_params.max_alpha, 1.0);

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

    if (nh_.getParam("laser_vector3", laser_vector3_))
        ROS_INFO("-> Laser Pos vector : [ %.3f  %.3f  %.3f ]", laser_vector3_[0], laser_vector3_[1], laser_vector3_[2]);
    else
        ROS_INFO("-> Error: didn't find laser_vector");

    if (nh_.getParam("laser_quaternion", laser_quaternion_))
        ROS_INFO("-> Laser Pos quater : [ %.3f  %.3f  %.3f  %.3f ]", laser_quaternion_[0], laser_quaternion_[1],
                 laser_quaternion_[2], laser_quaternion_[3]);
    else
        ROS_INFO("-> Error: didn't find laser_quaternion");

    
    ros::Timer imu_timer = nh_.createTimer(
        ros::Duration(0.01),  // 100Hz
        [this](const ros::TimerEvent&) {
            this->UpdateImu();
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
/*****
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
                    //  aaaa = recv_msg.head.length ;
                    // cout << "length" << aaaa << endl;
                    // for(int i = 0; i<8;i++){
                    //     cout <<i<<" "<< (int)recv_msg.payload[i] << endl;
                    // }
                    // cout << "receive motor_encoder " << endl;
                    memcpy(&motor_encoder, recv_msg.payload, sizeof(MotorEncoder));
                    // cout << "right_speeder"<<motor_encoder.right_speeder << endl;
                    // cout << "left_speeder"<<motor_encoder.left_speeder << endl;
                    // cout << " " << endl;
                    // receiveWheelDatas();
                    //   receiveWheelStates();
                    break;
                case CM_MOTOR_CTL_ID:
                    
                    memcpy(&battery_datas, recv_msg.payload, sizeof(mars_msgs::BatteryDatas));
                    // receiveBatteryDatas();
                    break;
                case CM_IMU_DATA_ID:
                {
                    // ========== 新增：IMU数据包日志记录 ==========
                
                    ros::Time imu_receive_time = ros::Time::now();
                    
                    // 复制数据
                    IMUEncoder imu_copy;
                    memcpy(&imu_copy, recv_msg.payload, sizeof(IMUEncoder));

                    // ==========  将IMU数据加入环形缓冲区 ==========
                    addToRingBuffer(imu_copy, imu_receive_time);
                    
                    // 验证数据包
                    bool is_valid = validateImuPacket(imu_copy);
                    bool has_anomaly = false;
                    std::string anomaly_reason = "";
                    std::string english_anomaly_reason = "";   
                    // 检查常见异常
                    if (!is_valid) {
                        has_anomaly = true;
                        anomaly_reason = "数据范围异常";
                        english_anomaly_reason = "OutOfRange"; 
                    }
                    // 检查零值异常（所有值都为0）
                    else if (imu_copy.x_acc == 0 && imu_copy.y_acc == 0 && imu_copy.z_acc == 0 &&
                             imu_copy.omg_x == 0 && imu_copy.omg_y == 0 && imu_copy.omg_z == 0) {
                        has_anomaly = true;
                        anomaly_reason = "全零数据包";
                        english_anomaly_reason = "AllZero";
                    }
                    
                    // 记录日志
                    logImuPacket(imu_copy, imu_receive_time, has_anomaly, anomaly_reason);
                    
                    // 每100个数据包输出一次状态
                     if (is_logging_anomaly_ && anomaly_trigger_log_.is_open() && anomaly_log_remaining_frames_ > 0) {
                        
                        // 计算转速
                        double right_speed_rpm = (double)motor_encoder.right_speeder / 720.0;
                        double left_speed_rpm = (double)motor_encoder.left_speeder / 720.0;
                        
                        // 获取触发时间（从文件指针位置获取，简单起见用当前时间）
                        time_t raw_time;
                        struct tm *timeinfo;
                        char trigger_time[80];
                        time(&raw_time);
                        timeinfo = localtime(&raw_time);
                        strftime(trigger_time, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
                        
                        // 写入触发后数据
                        anomaly_trigger_log_ << trigger_time << ","
                                            << "POST_ANOMALY" << ","
                                            << anomaly_trigger_count_ << ","
                                            << (ANOMALY_LOG_FRAMES - anomaly_log_remaining_frames_ + 1) << ","
                                            << std::fixed << std::setprecision(3) 
                                            << (imu_receive_time - ros::Time::now()).toSec() << ","
                                            << imu_copy.x_acc << ","
                                            << imu_copy.y_acc << ","
                                            << imu_copy.z_acc << ","
                                            << imu_copy.omg_x << ","
                                            << imu_copy.omg_y << ","
                                            << imu_copy.omg_z << ","
                                            << motor_encoder.right_speeder << ","
                                            << motor_encoder.left_speeder << ","
                                            << std::setprecision(2) << right_speed_rpm << ","
                                            << left_speed_rpm << ","
                                            << flagdirR << ","
                                            << flagdirL << ","
                                            << std::setprecision(6) << base_pose.x << ","
                                            << base_pose.y << ","
                                            << base_pose.theta << std::endl;
                        
                        anomaly_trigger_log_.flush();
                        anomaly_log_remaining_frames_--;
                        
                        if (anomaly_log_remaining_frames_ == 0) {
                            is_logging_anomaly_ = false;
                            anomaly_trigger_log_ << "--- 异常数据记录结束 ---\n" << std::endl;
                            ROS_INFO("异常 #%d 数据记录完成", anomaly_trigger_count_);
                        }
                    }

                    
                    // cout << "receive imu data" << endl;
                    memcpy(&imu_encoder, recv_msg.payload, sizeof(IMUEncoder));
                    receiveImuDatas();
                    //  receiveSensorStates();
                     break;
                }
                   
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
*****/
void HardwareDriver::receiveMessage(const char data[], unsigned int len)
{
    uint8_t recv_data;
    ros::Time receive_time = ros::Time::now();

    for (int i = 0; i < len; i++)
    {
        recv_data = data[i];

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
                    // 编码器数据（5Hz）
                    MotorEncoder new_encoder;
                    memcpy(&new_encoder, recv_msg.payload, sizeof(MotorEncoder));
                    
                    // 检查数据有效性
                    bool encoder_valid = true;
                    if (new_encoder.right_speeder == -647298784 || 
                        new_encoder.left_speeder == -647298784 ||
                        abs(new_encoder.right_speeder) > 200000 ||
                        abs(new_encoder.left_speeder) > 200000) {
                        encoder_valid = false;
                    }
                    
                    if (encoder_valid) {
                        boost::mutex::scoped_lock lock(queue_mutex_);
                        
                        // 创建同步数据点
                        SyncData sync_data;
                        sync_data.timestamp = receive_time;
                        sync_data.encoder = new_encoder;
                        sync_data.has_encoder = true;
                        sync_data.has_imu = false;
                        
                        // 如果有最近的IMU数据，也保存进来
                        if (latest_imu_) {
                            sync_data.imu = latest_imu_.get();
                            sync_data.has_imu = true;
                        }
                        
                        data_queue_.push_back(sync_data);
                        
                        // 限制队列大小
                        while (data_queue_.size() > MAX_QUEUE_SIZE) {
                            data_queue_.pop_front();
                        }
                        
                        // 更新motor_encoder供其他函数使用
                        motor_encoder = new_encoder;
                        last_encoder_time_ = receive_time;
                        
                        if (!encoder_initialized_) {
                            encoder_initialized_ = true;
                            ROS_ERROR("【编码器】首次有效数据: %d, %d", 
                                     new_encoder.right_speeder, new_encoder.left_speeder);
                        }
                    }
                    
                    logMotorEncoderData("BATTERY_DATA", recv_msg.head.length);
                    break;
                }
                    
                case CM_MOTOR_CTL_ID:
                    memcpy(&battery_datas, recv_msg.payload, sizeof(mars_msgs::BatteryDatas));
                    break;
                    
                case CM_IMU_DATA_ID:
                {
                    // IMU数据（100Hz）
                    IMUEncoder new_imu;
                    memcpy(&new_imu, recv_msg.payload, sizeof(IMUEncoder));
                    
                    // 保存最新的IMU数据
                    latest_imu_ = new_imu;
                    
                    // 检查编码器数据时效性（用于日志）
                    double encoder_age = (receive_time - last_encoder_time_).toSec();
                    bool encoder_stale = !encoder_initialized_ || encoder_age > ENCODER_TIMEOUT;
                    
                    // 记录日志
                    logImuEncoderData(new_imu, recv_msg.head.length, encoder_stale);
                    
                    boost::mutex::scoped_lock lock(queue_mutex_);
                    
                    // 创建同步数据点
                    SyncData sync_data;
                    sync_data.timestamp = receive_time;
                    sync_data.imu = new_imu;
                    sync_data.has_imu = true;
                    sync_data.has_encoder = false;
                    
                    data_queue_.push_back(sync_data);
                    
                    while (data_queue_.size() > MAX_QUEUE_SIZE) {
                        data_queue_.pop_front();
                    }
                    
                    // 原有的IMU处理
                    addToRingBuffer(new_imu, receive_time);
                    
                    bool is_valid = validateImuPacket(new_imu);
                    bool has_anomaly = false;
                    std::string anomaly_reason = "";
                    
                    if (!is_valid) {
                        has_anomaly = true;
                        anomaly_reason = "数据范围异常";
                    }
                    else if (new_imu.x_acc == 0 && new_imu.y_acc == 0 && new_imu.z_acc == 0 &&
                             new_imu.omg_x == 0 && new_imu.omg_y == 0 && new_imu.omg_z == 0) {
                        has_anomaly = true;
                        anomaly_reason = "全零数据包";
                    }
                    
                    logImuPacket(new_imu, receive_time, has_anomaly, anomaly_reason);
                    
                    // 异常触发日志
                    if (is_logging_anomaly_ && anomaly_trigger_log_.is_open() && anomaly_log_remaining_frames_ > 0) {
                        
                        double right_speed_rpm = (double)motor_encoder.right_speeder / 720.0;
                        double left_speed_rpm = (double)motor_encoder.left_speeder / 720.0;
                        
                        time_t raw_time;
                        struct tm *timeinfo;
                        char trigger_time[80];
                        time(&raw_time);
                        timeinfo = localtime(&raw_time);
                        strftime(trigger_time, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
                        
                        anomaly_trigger_log_ << trigger_time << ","
                                            << "POST_ANOMALY" << ","
                                            << anomaly_trigger_count_ << ","
                                            << (ANOMALY_LOG_FRAMES - anomaly_log_remaining_frames_ + 1) << ","
                                            << std::fixed << std::setprecision(3) 
                                            << (receive_time - ros::Time::now()).toSec() << ","
                                            << new_imu.x_acc << ","
                                            << new_imu.y_acc << ","
                                            << new_imu.z_acc << ","
                                            << new_imu.omg_x << ","
                                            << new_imu.omg_y << ","
                                            << new_imu.omg_z << ","
                                            << motor_encoder.right_speeder << ","
                                            << motor_encoder.left_speeder << ","
                                            << std::setprecision(2) << right_speed_rpm << ","
                                            << left_speed_rpm << ","
                                            << flagdirR << ","
                                            << flagdirL << ","
                                            << std::setprecision(6) << base_pose.x << ","
                                            << base_pose.y << ","
                                            << base_pose.theta << std::endl;
                        
                        anomaly_trigger_log_.flush();
                        anomaly_log_remaining_frames_--;
                        
                        if (anomaly_log_remaining_frames_ == 0) {
                            is_logging_anomaly_ = false;
                            anomaly_trigger_log_ << "--- 异常数据记录结束 ---\n" << std::endl;
                            ROS_INFO("异常 #%d 数据记录完成", anomaly_trigger_count_);
                        }
                    }

                    memcpy(&imu_encoder, recv_msg.payload, sizeof(IMUEncoder));
                    receiveImuDatas();
                    break;
                }
                   
                case CM_SENSOR_DATA_ID:
                    memcpy(&deviceOnoff_sta, recv_msg.payload, sizeof(mars_msgs::DeviceOnoff));
                    break;
                    
                case CM_ULTRA_DATA_ID:
                    memcpy(&ultra_datas, recv_msg.payload, sizeof(UltraDatas));
                    if((int)ultra_datas.ultra_id == 1){
                        dist_datas.data[0] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 2){
                        dist_datas.data[1] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 3){
                        dist_datas.data[2] = (int)ultra_datas.ultra_dist;
                    }else if((int)ultra_datas.ultra_id == 4){
                        dist_datas.data[3] = (int)ultra_datas.ultra_dist;
                    }
                    break;
                }
            }
            else if (rs_type == CM_CMD_STA_TYPE)
            {
                switch (rs_id)
                {
                case 7:
                    memcpy(&infrared_datas, recv_msg.payload, 8);
                    break;
                }
            }
        }
    }
}
 
// ==========电机编码器日志记录函数 ==========
void HardwareDriver::logMotorEncoderData(const std::string& data_source, int packet_length)
{
    // 如果日志文件未打开，则创建
    if (!motor_encoder_log_.is_open()) {
        // 生成日志文件名
        time_t now_time = time(0);
        struct tm *time_info = localtime(&now_time);
        const char* home_path = getenv("HOME");
        
        std::string log_dir;
        if (home_path) {
            log_dir = std::string(home_path) + "/.ros/motor_encoder_logs/";
            mkdir(log_dir.c_str(), 0755);
        } else {
            log_dir = "/var/log/motor_encoder_logs/";
            mkdir(log_dir.c_str(), 0755);
        }
        
        char log_filename[256];
        snprintf(log_filename, sizeof(log_filename),
                 "%s/motor_encoder_%04d%02d%02d_%02d%02d%02d.csv",
                 log_dir.c_str(),
                 time_info->tm_year+1900, time_info->tm_mon+1, time_info->tm_mday,
                 time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
        
        motor_encoder_log_.open(log_filename, std::ios::out);
        
        if (motor_encoder_log_.is_open()) {
            motor_encoder_log_ << "=== 电机编码器数据日志 ===" << std::endl;
            // ========== 修改列名：分开Timestamp和SystemTime ==========
            motor_encoder_log_ << "date,time,rostime,data_source,packet_length,"
                              << "right_speeder_raw,left_speeder_raw,"
                              << "right_speed_rpm,left_speed_rpm,"
                              << "flagdirR,flagdirL,"
                              << "base_pose_x,base_pose_y,base_pose_theta" << std::endl;
            
            ROS_INFO("电机编码器日志已创建: %s", log_filename);
        }
    }
    
    if (!motor_encoder_log_.is_open()) return;
    
    // 获取系统时间 - 分别获取日期和时间部分
    time_t raw_time;
    struct tm *timeinfo;
    char date_buffer[20];  // 仅日期 YYYY-MM-DD
    char time_buffer[20];  // 仅时间 HH:MM:SS
    
    time(&raw_time);
    timeinfo = localtime(&raw_time);
    strftime(date_buffer, sizeof(date_buffer), "%Y-%m-%d", timeinfo);
    strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", timeinfo);
    
    // ROS时间
    ros::Time ros_time = ros::Time::now();
    
    // 计算转速
    double right_speed_rpm = (double)motor_encoder.right_speeder / 720.0;
    double left_speed_rpm = (double)motor_encoder.left_speeder / 720.0;
    
    // ==========分开写入日期和时间 ==========
    motor_encoder_log_ << date_buffer << ","  // date列
                      << time_buffer << ","    // time列
                      << std::fixed << std::setprecision(3) << ros_time.toSec() << ","  // rostime
                      << data_source << ","
                      << packet_length << ","
                      << motor_encoder.right_speeder << ","
                      << motor_encoder.left_speeder << ","
                      << std::setprecision(2) << right_speed_rpm << ","
                      << left_speed_rpm << ","
                      << flagdirR << ","
                      << flagdirL << ","
                      << std::setprecision(6) << base_pose.x << ","
                      << base_pose.y << ","
                      << base_pose.theta << std::endl;
    
    // 每100条记录flush一次
    static int motor_log_counter = 0;
    motor_log_counter++;
    if (motor_log_counter % 100 == 0) {
        motor_encoder_log_.flush();
    }
}

 
// ========== IMU+编码器数据日志记录函数（添加encoder_stale参数） ==========
void HardwareDriver::logImuEncoderData(const IMUEncoder& imu_data, int packet_length, bool encoder_stale)
{
    // 如果日志文件未打开，则创建
    if (!imu_encoder_log_.is_open()) {
        // 生成日志文件名
        time_t now_time = time(0);
        struct tm *time_info = localtime(&now_time);
        const char* home_path = getenv("HOME");
        
        std::string log_dir;
        if (home_path) {
            log_dir = std::string(home_path) + "/.ros/imu_encoder_logs/";
            mkdir(log_dir.c_str(), 0755);
        } else {
            log_dir = "/var/log/imu_encoder_logs/";
            mkdir(log_dir.c_str(), 0755);
        }
        
        char log_filename[256];
        snprintf(log_filename, sizeof(log_filename),
                 "%s/imu_encoder_%04d%02d%02d_%02d%02d%02d.csv",
                 log_dir.c_str(),
                 time_info->tm_year+1900, time_info->tm_mon+1, time_info->tm_mday,
                 time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
        
        imu_encoder_log_.open(log_filename, std::ios::out);
        
        if (imu_encoder_log_.is_open()) {
            imu_encoder_log_ << "=== IMU+编码器数据日志 ===" << std::endl;
            // ========== 添加encoder_stale列 ==========
            imu_encoder_log_ << "date,time,rostime,packet_length,encoder_stale,"
                            << "x_acc_raw,y_acc_raw,z_acc_raw,"
                            << "omg_x_raw,omg_y_raw,omg_z_raw,"
                            << "right_speeder_raw,left_speeder_raw,"
                            << "right_speed_rpm,left_speed_rpm,"
                            << "flagdirR,flagdirL,"
                            << "base_pose_x,base_pose_y,base_pose_theta" << std::endl;
            
            ROS_INFO("IMU+编码器日志已创建: %s", log_filename);
        }
    }
    
    if (!imu_encoder_log_.is_open()) return;
    
    // 获取系统时间
    time_t raw_time;
    struct tm *timeinfo;
    char date_buffer[20];
    char time_buffer[20];
    
    time(&raw_time);
    timeinfo = localtime(&raw_time);
    strftime(date_buffer, sizeof(date_buffer), "%Y-%m-%d", timeinfo);
    strftime(time_buffer, sizeof(time_buffer), "%H:%M:%S", timeinfo);
    
    // ROS时间
    ros::Time ros_time = ros::Time::now();
    
    // 计算转速（如果编码器数据异常，转速也标记为0）
    double right_speed_rpm = 0.0;
    double left_speed_rpm = 0.0;
    
    if (!encoder_stale && encoder_initialized_) {
        right_speed_rpm = (double)motor_encoder.right_speeder / 720.0;
        left_speed_rpm = (double)motor_encoder.left_speeder / 720.0;
    }
    
    // 写入日志
    imu_encoder_log_ << date_buffer << ","
                    << time_buffer << ","
                    << std::fixed << std::setprecision(3) << ros_time.toSec() << ","
                    << packet_length << ","
                    << (encoder_stale ? "STALE" : "FRESH") << ","  // 添加时效性标记
                    << imu_data.x_acc << ","
                    << imu_data.y_acc << ","
                    << imu_data.z_acc << ","
                    << imu_data.omg_x << ","
                    << imu_data.omg_y << ","
                    << imu_data.omg_z << ","
                    << motor_encoder.right_speeder << ","
                    << motor_encoder.left_speeder << ","
                    << std::setprecision(2) << right_speed_rpm << ","
                    << left_speed_rpm << ","
                    << flagdirR << ","
                    << flagdirL << ","
                    << std::setprecision(6) << base_pose.x << ","
                    << base_pose.y << ","
                    << base_pose.theta << std::endl;
    
    // 每100条记录flush一次
    static int imu_encoder_log_counter = 0;
    imu_encoder_log_counter++;
    if (imu_encoder_log_counter % 100 == 0) {
        imu_encoder_log_.flush();
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
            
            safe_data.data[i] = 2000;
            ROS_WARN_THROTTLE(5.0, "超声波传感器%d异常设置为安全距离2米", (int)(i+1));
        }
    }
    ultra_data_pub_.publish(dist_datas);
}


/***
void HardwareDriver::UpdateImu()
{
    bool encoder_valid = true;
    if (motor_encoder.right_speeder == -647298784 || 
        motor_encoder.left_speeder == -647298784 ||
        abs(motor_encoder.right_speeder) > 200000 ||
        abs(motor_encoder.left_speeder) > 200000) {
        encoder_valid = false;
    }
    
    // 编码器无效时直接返回，不进行任何处理
    if (!encoder_valid) {
        return;
    }
    // ========== 新增：检查编码器数据是否有效 ==========
    if (!encoder_initialized_) {
        ROS_WARN_THROTTLE(1.0, "编码器未初始化，等待有效数据...");
        return;
    }
    
    double encoder_age = (ros::Time::now() - last_encoder_time_).toSec();
    if (encoder_age > ENCODER_TIMEOUT) {
        ROS_WARN_THROTTLE(1.0, "编码器数据超时(%.2fs)，使用上次值", encoder_age);
        // 继续使用上次的编码器值，但给出警告
    }

    static int imu_call_count = 0;
    imu_call_count++;
    
    if (imu_call_count % 50 == 0) {
        ROS_ERROR("【硬件状态】wheel_flag=%d, imu_flag=%d, flagdirL=%d, flagdirR=%d, base_pose=(%.3f,%.3f,%.1f°)",
                  wheel_flag, imu_flag, flagdirL, flagdirR,
                  base_pose.x, base_pose.y, base_pose.theta * 180.0/M_PI);
        
        ROS_ERROR("【编码器】right=%d, left=%d, last_right=%.1f, last_left=%.1f",
                  motor_encoder.right_speeder, motor_encoder.left_speeder,
                  last_right_econd, last_left_econd);
    }

    static int idle_counter = 0;
    
    if (motor_command.right_speed == 0 && motor_command.left_speed == 0) {
        idle_counter++;
        
        // 2. 空闲超过阈值（如1秒），自动恢复默认方向
        if (idle_counter > 50) {  // 假设50Hz控制频率，1秒=50次
            if (flagdirL != 1 || flagdirR != 1) {
                flagdirL = 1;
                flagdirR = 1;
                ROS_WARN("电机空闲超时，编码器方向自动重置为正向");
            }
            idle_counter = 0;
        }
    } else {
        idle_counter = 0;  // 有运动时复位计数器
    }

    tf::Quaternion imu_quat;

    ros::Time current_time = ros::Time::now();

    double dt = 0.0;   
    static bool first_check = true;
    if (first_check) {
        first_check = false;
        if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
            ROS_ERROR("⚠️ base_pose初始值异常！重置为0");
            ROS_ERROR("  当前值: x=%.3f, y=%.3f", base_pose.x, base_pose.y);
            logAnomaly("INIT_POSE_ABNORMAL", base_pose.x, "初始位置异常，重置为0", dt);
            base_pose.x = 0.0;
            base_pose.y = 0.0;
            base_pose.theta = 0.0;
        }
    }

    // 零漂过滤
    if (imu_flag == 0)
    {
        last_time = current_time;
        imu_flag = 1;
        return; // 第一次初始化，跳过计算
    }

    // ========== 关键修复：时间间隔保护 ==========
    dt = (current_time - last_time).toSec();
    
    // 防御性编程：防止ROS时间异常
    if (dt <= 0) {
        // 时间异常：小于等于0
        ROS_WARN_THROTTLE(1.0, "时间异常: dt=%.6f <= 0，使用默认值0.01s", dt);
        logAnomaly("DT_NEGATIVE", dt, "时间间隔为负值或零，使用默认值0.01", dt);
        dt = 0.01;
        last_time = current_time;
        return;
    }
    
    if (dt > 0.2) {
        // ROS时间可能发生跳跃（如重启、时间同步等）
        ROS_ERROR("时间跳跃异常: dt=%.3fs > 0.2s，跳过本次更新", dt);
        logAnomaly("DT_TOO_LARGE", dt, "时间间隔过大(>0.2s)，重置编码器状态", dt);
        last_time = current_time;
        
        // 重置编码器状态，避免错误积分
        wheel_flag = 0;
        imu_flag = 0;
        return;
    }
    
    if (dt > 0.1) {
        // 时间间隔较大但仍在可接受范围
        ROS_WARN_THROTTLE(5.0, "时间间隔较大: dt=%.3fs，限制为0.1s", dt);
        logAnomaly("DT_WARNING", dt, 
                  "时间间隔较大(>0.1s)，限制为0.1s", dt);
        dt = 0.1;
    }

    last_time = current_time;

    imu_datas.header.stamp = ros::Time::now();
    imu_datas.header.frame_id = "imu_link";

    double cur_right_econd = (double)motor_encoder.right_speeder/720.0f;
    double cur_left_econd = (double)motor_encoder.left_speeder/720.0f;

 
    
    // 检查编码器值是否异常（防止数据解析错误）
    if (fabs(cur_right_econd) > 10000.0 || fabs(cur_left_econd) > 10000.0) {
        ROS_WARN_THROTTLE(1.0, "编码器值异常: right=%.1f, left=%.1f，使用上次值", 
                         cur_right_econd, cur_left_econd);
        
         // 记录编码器原始值异常
        std::stringstream ss;
        ss << "右编码器值=" << motor_encoder.right_speeder 
           << "(" << cur_right_econd << "), "
           << "左编码器值=" << motor_encoder.left_speeder 
           << "(" << cur_left_econd << ")";
        logAnomaly("ENCODER_VALUE_ABNORMAL", 
                  fmax(fabs(cur_right_econd), fabs(cur_left_econd)),
                  ss.str(), dt);
        cur_right_econd = last_right_econd;
        cur_left_econd = last_left_econd;
    }
    
    // 检查编码器突变（单次变化过大）
    double right_diff = fabs(cur_right_econd - last_right_econd);
    double left_diff = fabs(cur_left_econd - last_left_econd);

  
    if (right_diff > 100.0 || left_diff > 100.0) {
 

        ROS_WARN("编码器突变: right_diff=%.1f, left_diff=%.1f，可能数据错误", 
                right_diff, left_diff);
          // 记录编码器突变
        logAnomaly("ENCODER_JUMP", 
                  fmax(right_diff, left_diff),
                  "右编码器变化=" + std::to_string(right_diff) + 
                  ", 左编码器变化=" + std::to_string(left_diff), dt);
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

    if(wheel_flag == 0){
        wheel_flag = 1;
        
        last_right_econd = cur_right_econd;
        last_left_econd = cur_left_econd;
        
        return; // 第一次初始化编码器，跳过计算
    }

    double dright_econd, dleft_econd;
    if(flagdirR == 1){
        dright_econd = cur_right_econd - last_right_econd;
    }else{
        dright_econd = -(cur_right_econd - last_right_econd);
        ROS_WARN("右轮方向反转! flagdirR=%d", flagdirR);
    }

    if(flagdirL == 1){
        dleft_econd  = cur_left_econd - last_left_econd;
    }else{
        dleft_econd  = -(cur_left_econd - last_left_econd);
        ROS_WARN("左轮方向反转! flagdirL=%d", flagdirL);
    }

    cout << "dright_econd =   " << dright_econd << endl;
    cout << "dleft_econd =   " << dleft_econd << endl;

    // 检查增量是否合理
    double max_single_move = 0.2;
    double abs_right_move = fabs(dright_econd / 720.0f / 1.5);
    double abs_left_move = fabs(dleft_econd / 720.0f / 1.5);


    
    if (abs_right_move > max_single_move || abs_left_move > max_single_move) {
        ROS_WARN_THROTTLE(1.0, "单次移动过大: right=%.3fm, left=%.3fm", 
                         abs_right_move, abs_left_move);
        
         // 记录增量异常
        logAnomaly("ENCODER_INCREMENT_TOO_LARGE",
                  fmax(abs_right_move, abs_left_move),
                  "右轮增量=" + std::to_string(dright_econd) +
                  ", 左轮增量=" + std::to_string(dleft_econd), dt);
        
        // 限制增量
        double scale = max_single_move / fmax(abs_right_move, abs_left_move);
        dright_econd *= scale;
        dleft_econd *= scale;
    }

    base_dist_x = (dright_econd + dleft_econd)/2.0/1.5;
 

    // 检查base_dist_x是否合理
    if (fabs(base_dist_x) > max_single_move) {
        ROS_WARN_THROTTLE(1.0, "base_dist_x异常: %.3fm > %.3fm", 
                         base_dist_x, max_single_move);
         // 记录位移异常
        logAnomaly("BASE_DIST_X_ABNORMAL", base_dist_x,"计算得到的base_dist_x过大", dt);
        base_dist_x = (base_dist_x > 0) ? max_single_move : -max_single_move;
    }

    static bool first_displacement_after_restart = true;
    if (first_displacement_after_restart) {
        first_displacement_after_restart = false;
        base_dist_x = 0.0;  // 强制清零
        ROS_WARN("重启后第一次位移清零");
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
          // 记录位移大小异常
        logAnomaly("DELTA_MAGNITUDE_ABNORMAL", delta_magnitude,
                  "dt=" + std::to_string(dt) + 
                  ", delta_x=" + std::to_string(delta_x) +
                  ", delta_y=" + std::to_string(delta_y), dt);
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
        
        // 记录位置突变
        std::stringstream ss;
        ss << "ΔX=" << (new_x - original_x) 
           << ", ΔY=" << (new_y - original_y)
           << ", 从(" << original_x << "," << original_y << ")"
           << " 到(" << new_x << "," << new_y << ")";
        logAnomaly("POSITION_JUMP", fmax(fabs(new_x - original_x), fabs(new_y - original_y)),
                  ss.str(), dt);
 

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
        
         // 记录致命的位置异常
        std::stringstream ss;
        ss << "x=" << base_pose.x << ", y=" << base_pose.y 
           << ", 重置里程计到原点";
        logAnomaly("POSITION_OUT_OF_RANGE", base_pose.x, ss.str(), dt);

        base_pose.x = 0.0;
        base_pose.y = 0.0;
        base_pose.theta = 0.0;
        wheel_flag = 0;
        imu_flag = 0;
        reset_odometry = true;

        // 同时重置安全位置
        last_valid_pose_.x = 0.0;
        last_valid_pose_.y = 0.0;
        last_valid_pose_.theta = 0.0;
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
    cout << "base_pose.theta  " << base_pose.theta << endl;
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
         // 每50次更新记录一次正常状态，用于对比
        if (anomaly_log_.is_open()) {
            time_t rawtime;
            struct tm *timeinfo;
            char time_buffer[80];
            time(&rawtime);
            timeinfo = localtime(&rawtime);
            strftime(time_buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
            
            anomaly_log_ << time_buffer << ",NORMAL,0," 
                        << base_pose.x << "," << base_pose.y << ","
                        << dt << "," << motor_encoder.right_speeder << ","
                        << motor_encoder.left_speeder << "," << imu_wz
                        << ",正常状态记录" << std::endl;
        }
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
    // ========== 1. 从队列获取同步数据 ==========
    SyncData current_data;
    bool has_data = false;
    
    {
        boost::mutex::scoped_lock lock(queue_mutex_);
        if (!data_queue_.empty()) {
            current_data = data_queue_.front();
            data_queue_.pop_front();
            has_data = true;
        }
    }
    
    if (!has_data) return;

    // ========== 2. 启动保护 ==========
    static bool startup_complete = false;
    
    if (!startup_complete) {
        if (startup_protect_count_ < STARTUP_PROTECT_THRESHOLD) {
            startup_protect_count_++;
            ROS_WARN_THROTTLE(1.0, "【启动保护】%d/%d", startup_protect_count_, STARTUP_PROTECT_THRESHOLD);
            return;
        } else {
            startup_complete = true;
            ROS_ERROR("【启动完成】");
        }
    }

    // ========== 3. 时间计算 ==========
    ros::Time current_time = ros::Time::now();
    static ros::Time last_time = current_time;
    double dt = (current_time - last_time).toSec();

    if (dt <= 0 || dt > 0.1) dt = 0.01;
    last_time = current_time;

    // ========== 4. IMU数据 ==========
    double imu_wz_raw = -(double)current_data.imu.omg_z / 100.0f * 1.08;
    if (!std::isfinite(imu_wz_raw) || fabs(imu_wz_raw) > 500.0) {
        ROS_WARN_THROTTLE(1.0, "IMU数据异常: %.3f", imu_wz_raw);
        return;
    }

    // ========== 5. 编码器数据处理 ==========
    double translation = 0.0;
    double rotation = 0.0;
    double dr = 0.0, dl = 0.0;
    bool has_encoder_motion = false;
    static int encoder_error_count = 0;
    double raw_dr = 0.0, raw_dl = 0.0; 

    // ========== 阈值常量 ==========
    const double ENCODER_DEADZONE = 0.01;      // 1cm死区
    const double ENCODER_MOTION_THRESH = 0.01; // 运动检测阈值
    const double ROTATION_CMD_THRESH = 30;     // 旋转指令阈值

    // 动态阈值：根据最近的运动状态调整
    static double recent_motion = 0.0;

    // ========== 零偏校准（不阻塞正常更新） ==========
    static double imu_bias_ = 0.0;
    static int bias_calib_count_ = 0;
    static bool bias_calibrated_ = false;
    
    if (!bias_calibrated_ && startup_complete) {
        if (bias_calib_count_ < 100) {
            imu_bias_ += imu_wz_raw;
            bias_calib_count_++;
            ROS_WARN_THROTTLE(1.0, "【IMU校准中】%d/100", bias_calib_count_);
        } else {
            imu_bias_ /= 100.0;
            bias_calibrated_ = true;
            ROS_ERROR("【IMU校准完成】零偏=%.3f °/s", imu_bias_);
        }
    }
    
    double imu_wz_used = bias_calibrated_ ? (imu_wz_raw - imu_bias_) : imu_wz_raw;

    // ==============================

    if (current_data.has_encoder) {
        double cur_right = (double)current_data.encoder.right_speeder / 720.0f;
        double cur_left = (double)current_data.encoder.left_speeder / 720.0f;
        
        // 编码器值范围检查
        if (fabs(cur_right) > 10000.0 || fabs(cur_left) > 10000.0) {
            encoder_error_count++;
            if (encoder_error_count > 10) {
                ROS_ERROR("编码器持续异常，重置里程计");
                base_pose.x = 0;
                base_pose.y = 0;
                base_pose.theta = 0;
                wheel_flag = 0;
                encoder_error_count = 0;
            }
            return;
        }
        encoder_error_count = 0;
        
        if (wheel_flag == 0) {
            wheel_flag = 1;
            last_right_econd = cur_right;
            last_left_econd = cur_left;
            ROS_WARN("【轮计数】初始化完成");
            return;
        }
        
        // ========== 计算原始增量 ==========
        raw_dr = cur_right - last_right_econd;   
        raw_dl = cur_left - last_left_econd;

        // 计算方向校正后的增量
        dr = (flagdirR == 1) ? (cur_right - last_right_econd) : -(cur_right - last_right_econd);
        dl = (flagdirL == 1) ? (cur_left - last_left_econd) : -(cur_left - last_left_econd);
        
        // 死区滤波
        if (fabs(dr) < ENCODER_DEADZONE) dr = 0.0;
        if (fabs(dl) < ENCODER_DEADZONE) dl = 0.0;
        
        // 突变检测
        if (fabs(dr) > 100.0 || fabs(dl) > 100.0) {
            ROS_WARN_THROTTLE(1.0, "编码器突变: dr=%.1f, dl=%.1f", dr, dl);
        }
        
        // 检查电机指令
        bool has_rotation_cmd = (abs(motor_command.right_speed - motor_command.left_speed) > ROTATION_CMD_THRESH);
        bool has_translation_cmd = (abs(motor_command.right_speed + motor_command.left_speed) > ROTATION_CMD_THRESH);
        
        // 运动检测
        has_encoder_motion = (fabs(dr) > ENCODER_MOTION_THRESH || fabs(dl) > ENCODER_MOTION_THRESH);
        
        // 获取当前按键状态
        bool is_pressing_a_or_d = (fabs(current_cmd_angular_z) > 0.01);
        bool is_pressing_w_or_s = (fabs(current_cmd_linear_x) > 0.01);

        // ========== 核心逻辑修改：取消旋转时的强制清零 ==========
        // 无论什么模式，都计算原始位移
        double raw_trans = (dr + dl) / 2.0 / 1.5;
        
        if (is_pressing_a_or_d && !is_pressing_w_or_s) {
            // 场景1：纯旋转 - 使用小阈值，不再强制为0
            if (fabs(raw_trans) > 0.0015) {  // 1.5mm阈值
                translation = raw_trans;
            } else {
                translation = 0.0;
            }
            // 重要：不再清零 dr 和 dl
            has_encoder_motion = (fabs(dr) > 0.005 || fabs(dl) > 0.005);
            
            // 旋转测试统计（保持原打印）
            static double rot_sum = 0.0;
            static int rot_count = 0;
            static double last_rot_theta = base_pose.theta;
            
            rot_count++;
            rot_sum += fabs(raw_trans);
            
            if (fabs(base_pose.theta - last_rot_theta) > M_PI/2) {
                ROS_WARN("【旋转测试】旋转90°，平均位移=%.6fm", rot_sum/rot_count);
                rot_sum = 0.0;
                rot_count = 0;
                last_rot_theta = base_pose.theta;
            }
        }
        else if (is_pressing_w_or_s) {
            // 场景2：有平移（可能同时旋转）- 正常计算
            translation = raw_trans;
            
            // 平移测试统计（保持原打印）
            static double trans_sum = 0.0;
            static int trans_count = 0;
            static double last_trans_x = base_pose.x;
            
            trans_count++;
            trans_sum += fabs(translation);
            
            if (fabs(base_pose.x - last_trans_x) > 0.1) {
                ROS_WARN("【平移测试】移动0.1m，平均步长=%.6fm", trans_sum/trans_count);
                trans_sum = 0.0;
                trans_count = 0;
                last_trans_x = base_pose.x;
            }
        }
        else {
            // 场景3：无按键（惯性/静止）- 保持原样
            if (has_encoder_motion) {
                recent_motion = recent_motion * 0.9 + fabs(raw_trans) * 0.1;
                
                if (recent_motion > 0.01) {
                    translation = raw_trans;
                    cout << "recent_motion > 0.01 translation = " << raw_trans << endl;
                } else {
                    translation = 0.0;
                    cout << "recent_motion < 0.01 translation = " << translation << endl;
                }
            } else {
                translation = 0.0;
                recent_motion = 0.0;
                cout << "has_encoder_motion ! translation = " << translation << endl;
            }
        }

        // 单步位移限制
        if (fabs(translation) > 0.2) {
            ROS_WARN_THROTTLE(1.0, "位移过大: %.3f", translation);
            translation = (translation > 0) ? 0.2 : -0.2;
        }
        
        last_right_econd = cur_right;
        last_left_econd = cur_left;
        
        // ========== 调试输出（保持原样） ==========
        cout << "dright_econd = " << dr << endl;
        cout << "dleft_econd = " << dl << endl;
        cout << "translation = " << translation << " m" << endl;
        cout << ", has_rotation_cmd=" << has_rotation_cmd
            << ", has_translation_cmd=" << has_translation_cmd << endl;
    }

 
    // ========== 6. 静止检测 ==========
    static int stationary_count = 0;
    static const int STATIONARY_THRESHOLD = 50;
    static double last_angle_for_check = base_pose.theta;
    static ros::Time last_motion_time = ros::Time::now();

    // 重新计算这三个值，确保是最新的
    if (current_data.has_encoder) {
        has_encoder_motion = (fabs(dr) > 0.01 || fabs(dl) > 0.01);
    } else {
        has_encoder_motion = false;  // 没有编码器数据时强制false
    }

    bool has_motion_cmd = (abs(motor_command.right_speed) > 50 || 
                        abs(motor_command.left_speed) > 50);
    double angle_change = fabs(base_pose.theta - last_angle_for_check);
    last_angle_for_check = base_pose.theta;

    // ========== 关键：强制重新计算，不受外部影响 ==========
    bool has_any_motion = has_encoder_motion;  // 只依赖编码器

    // 打印调试信息，确认实际值
    cout << "【静止检测调试】has_encoder_motion=" << has_encoder_motion 
        << ", has_motion_cmd=" << has_motion_cmd
        << ", 计算出的has_any_motion=" << has_any_motion << endl;

    if (has_any_motion) {
        stationary_count = 0;
        last_motion_time = ros::Time::now();
    } else {
        double idle_time = (ros::Time::now() - last_motion_time).toSec();
        if (idle_time > 0.2) {
            stationary_count = min(stationary_count + 1, STATIONARY_THRESHOLD + 10);
        }
    }
    bool is_stationary = (stationary_count >= STATIONARY_THRESHOLD);
 

    cout << "静止检测: 计数=" << stationary_count << ", 状态=" << (is_stationary ? "静止" : "运动") 
         << ", 角度变化=" << angle_change * 180.0/M_PI << "°" << endl;
    cout << "has_any_motion=" << has_any_motion 
         << ", stationary_count=" << stationary_count 
         << ", is_stationary=" << is_stationary << endl;
    cout << "编码器运动=" << has_encoder_motion 
         << ", 电机指令=" << has_motion_cmd
         << ", dr=" << dr 
         << ", dl=" << dl << endl;

    // ========== 7. 角度更新（关键修改：IMU+编码器融合）==========
 
    if (!is_stationary) {
        double imu_dtheta = imu_wz_used * PI_M / 180.0f * dt;
        
        double encoder_dtheta = 0.0;
        if (current_data.has_encoder && fabs(dr - dl) > 0.001) {
            encoder_dtheta = (dr - dl) / machine_params.wheel_spacing;
        }
        
        // 判断运动类型
        double trans_mag = fabs(dr + dl);
        double rot_mag = fabs(dr - dl);
        
        double fused_dtheta;
        const char* motion_type = "";
        
        if (rot_mag < 0.005) {
            // 纯平移：角度不应变化
            fused_dtheta = 0.0;
            motion_type = "平移";
        }
        else if (trans_mag < 0.005) {
            // 纯旋转：IMU为主，编码器辅助
            fused_dtheta = 0.9 * imu_dtheta + 0.1 * encoder_dtheta;
            motion_type = "旋转";
            
            // 限幅2°/步
            if (fabs(fused_dtheta) > 2.0 * PI_M / 180.0) {
                fused_dtheta = (fused_dtheta > 0) ? 2.0 * PI_M/180 : -2.0 * PI_M/180;
            }
        }
        else {
            // 弧线运动：平衡融合
            fused_dtheta = 0.8 * imu_dtheta + 0.2* encoder_dtheta;
            motion_type = "弧线";
            
            // 限幅1°/步
            if (fabs(fused_dtheta) > 1.0 * PI_M / 180.0) {
                fused_dtheta = (fused_dtheta > 0) ? 1.0 * PI_M/180 : -1.0 * PI_M/180;
            }
        }
        
        // 限幅保护（防止跳变）
        const double MAX_THETA_PER_STEP = 2.0 * PI_M / 180.0;  // 2°
        if (fabs(fused_dtheta) > MAX_THETA_PER_STEP) {
            cout << "【角度跳变】" << motion_type << "模式: "
                << fused_dtheta * 180/M_PI << "° -> 限制为2°" << endl;
            fused_dtheta = (fused_dtheta > 0) ? MAX_THETA_PER_STEP : -MAX_THETA_PER_STEP;
        }
        
        base_pose.theta += fused_dtheta;
        
        // 每50次输出一次角度详情
        static int angle_debug = 0;
        angle_debug++;
        if (angle_debug % 50 == 0) {
            cout << "【角度】" << motion_type << ": IMU=" << imu_dtheta * 180/M_PI 
                << "°, 编码器=" << encoder_dtheta * 180/M_PI 
                << "°, 融合=" << fused_dtheta * 180/M_PI 
                << "°, 当前=" << base_pose.theta * 180/M_PI << "°" << endl;
        }
    }
    
    // 角度规范化
    while (base_pose.theta > PI_M) base_pose.theta -= 2*PI_M;
    while (base_pose.theta < -PI_M) base_pose.theta += 2*PI_M;
 
    // ========== 8. 位置更新（实时更新，不等待静止）==========
    if (fabs(translation) > 0.0001) {  // 只要有位移就更新
        double delta_x = translation * cos(base_pose.theta);
        double delta_y = translation * sin(base_pose.theta);
        
        double max_delta_per_update = 0.2;
        double delta_magnitude = sqrt(delta_x*delta_x + delta_y*delta_y);
        
        if (delta_magnitude <= max_delta_per_update) {
            base_pose.x += delta_x;
            base_pose.y += delta_y;
        } else {
            ROS_ERROR("位移异常！大小=%.3fm", delta_magnitude);
        }
    }
 
    // ========== 10. 最终位置检查 ==========
    if (fabs(base_pose.x) > 1000.0 || fabs(base_pose.y) > 1000.0) {
        ROS_ERROR("位置超出范围，重置里程计");
        base_pose.x = 0;
        base_pose.y = 0;
        base_pose.theta = 0;
        wheel_flag = 0;
        imu_flag = 0;
    }

    // ========== 11. 输出调试信息（保持原样） ==========
    cout << "-------------------------------" << endl;
    cout << "base_pose.x = " << base_pose.x << endl;
    cout << "base_pose.y = " << base_pose.y << endl;
    cout << "base_pose.theta = " << base_pose.theta * 180.0/M_PI << "°" << endl;
    cout << "dist_datas " << dist_datas.data[0] << " " << dist_datas.data[1] << " " 
         << dist_datas.data[2] << " " << dist_datas.data[3] << endl;
    cout << "-------------------------------\n" << endl;

    // ========== 12. 发布数据 ==========
    imu_datas.header.stamp = current_time;
    imu_datas.header.frame_id = "imu_link";
    imu_datas.angular_velocity.z = imu_wz_used * PI_M / 180.0f;
    imu_datas.linear_acceleration.x = -imu_ay - 0.3;
    imu_datas.linear_acceleration.y = -9.81;
    imu_datas.linear_acceleration.z = imu_ax + 0.2;
    imu_data_pub_.publish(imu_datas);

    wheel_odom.header.stamp = current_time;
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_link";
    wheel_odom.pose.pose.position.x = base_pose.x;
    wheel_odom.pose.pose.position.y = base_pose.y;
    wheel_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(base_pose.theta);
    wheel_odom_pub_.publish(wheel_odom);
    Base_pose_pub_.publish(base_pose);

    publishTfTransforms();
}
// 实现TF发布函数
/*********/
void HardwareDriver::publishTfTransforms() {
    ros::Time current_time = ros::Time::now();

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
   
    if (msgtype == CM_DEV_CTL_TYPE && msgid == RCRID_MOTOR_COMMUN && len == sizeof(MotorCommand)) {
        MotorCommand* mc = (MotorCommand*)payload;
        ROS_ERROR("【硬件发送】msgtype=%d, msgid=%d", msgtype, msgid);
        ROS_ERROR("  电机命令: right=%d, left=%d, spinl=%d, spinr=%d",
                 mc->right_speed, mc->left_speed, mc->spinl_speed, mc->spinr_speed);
        ROS_ERROR("  时间戳: %.6f", ros::Time::now().toSec());
    }

    HardwareMessage send_msg(msgtype, msgid, payload, len);
    if (packData(&send_msg, send_data))
    {

        ros::Time write_start = ros::Time::now();

        hardware_serial_->write((char *)send_data, send_length);
        
        
        ros::Duration write_duration = ros::Time::now() - write_start;
        if (write_duration.toSec() > 0.001) {  // 超过1ms
            ROS_WARN("串口写入耗时: %.3fms", write_duration.toSec() * 1000.0);
        }
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
    ares_cmd->mars_ctlmode_cmd = MARS_GET_MANUAL_CMD;  // 设为1
    deviceOnoff_sta.baterry_onoff_sta = BATERRY_ON_STA;

    geometry_msgs::Twist cmd_vel = cmd_vel_msg;
 
    curr_time = ros::Time::now();
    double dt = (curr_time - las_time).toSec();
    las_time = curr_time;
    // cout << "dt " << dt << endl;

    double current_speed_command = cmd_vel.linear.x;
    double current_omega_command = cmd_vel.angular.z * 6.4;
    if (fabs(cmd_vel.angular.z) < 0.001f) {
        current_omega_command = 0.0f;
    }
 

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
 

    motor_command.right_speed = right_speed * 230;     
    motor_command.left_speed = -left_speed * 230; 

 


    motor_command.spinl_speed = fabs((current_speed_command)*240);
    motor_command.spinr_speed = fabs((current_speed_command)*260);
    if(motor_command.spinl_speed > 90){
        motor_command.spinl_speed = 90;
    }
    motor_command.spinl_speed = 0;
    motor_command.spinr_speed = 0;
    ROS_ERROR("=== 电机命令详细调试 ===");
 
    ROS_ERROR("主电机: right=%d, left=%d", motor_command.right_speed, motor_command.left_speed);
    ROS_ERROR("spin速度: left=%d, right=%d", motor_command.spinl_speed, motor_command.spinr_speed);
    ROS_ERROR("方向标记: flagdirR=%d, flagdirL=%d", flagdirR, flagdirL);


    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}

void HardwareDriver::sendKeyboardVelCommand(const geometry_msgs::Twist &cmd_vel)
{
    static int cmd_count = 0;
    cmd_count++;
    // 保存当前指令用于运动类型判断
    current_cmd_linear_x = cmd_vel.linear.x;
    current_cmd_angular_z = cmd_vel.angular.z;
    if (cmd_vel.linear.x != 0 || cmd_vel.angular.z != 0) {
        ROS_ERROR("【键盘命令】vx=%.3f, wz=%.3f, flagdirL=%d, flagdirR=%d",
                  cmd_vel.linear.x, cmd_vel.angular.z, flagdirL, flagdirR);
    }
   
    ares_cmd->mars_ctlmode_cmd = MARS_GET_MANUAL_CMD;
    deviceOnoff_sta.baterry_onoff_sta = BATERRY_ON_STA;
    curr_time = ros::Time::now();
    double dt = (curr_time - las_time).toSec();
    las_time = curr_time;
    // cout << "dt " << dt << endl;

    double current_speed_command = cmd_vel.linear.x;
    double current_omega_command = cmd_vel.angular.z * 6.4 ;

 
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
    // if(fabs(current_speed_command) == 0) {
        motor_command.spinl_speed = fabs((current_speed_command)*240);
        motor_command.spinr_speed = fabs((current_speed_command)*260);
    // }else{
    //     motor_command.spin_speed = 90;
    // }
    // cout << "motor_command.spinl_speed  " << motor_command.spinl_speed  << endl;
    if(motor_command.spinl_speed > 90){
        motor_command.spinl_speed = 90;
    }
    motor_command.spinl_speed = 0;
    motor_command.spinr_speed = 0;
    // cout << "cmd_vel.linear.y " << cmd_vel.linear.y << endl;
    // if(cmd_vel.linear.y != 0){
    //     motor_command.right_speed = 0;
    //     motor_command.left_speed = 0;
    //     motor_command.spin_speed = 99;
    // }
    //motor_command.spin_speed = 0.0;

    // cout << " " << endl;
    // cout << "motor_command.right_speed = " << motor_command.right_speed << endl;
    // cout << "motor_command.left_speed = " << motor_command.left_speed << endl;
    // cout << "motor_command.spin_speed = " << motor_command.spin_speed << endl;
    // cout << "dis = " << dis << endl;
    // cout << "------------" << endl;
    ROS_ERROR("sendKeyboardVelCommand: vx=%.3f, spinl=%d, spinr=%d", 
              cmd_vel.linear.x, motor_command.spinl_speed, motor_command.spinr_speed);

    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = RCRID_MOTOR_COMMUN;
    sendMessage(send_type, send_id, (uint8_t *)&motor_command, (uint8_t)sizeof(motor_command));
}

void HardwareDriver::sendUltraCommand(){
    MSG_Type send_type = CM_DEV_CTL_TYPE;
    int send_id = 4;
    uint8_t ultra_command = 1;
    // cout << "send ultra_command" << endl;
    static int aaa = 0;
    if(flag_ultra == 0){
        ultra_command = 1;
        // cout << "send ultra 1" << endl;
        sendMessage(send_type, send_id, (uint8_t *)&ultra_command, (uint8_t)sizeof(ultra_command));
    }
    if(flag_ultra == 1){
        ultra_command = 2;
        // cout << "send ultra 2" << endl;
        sendMessage(send_type, send_id, (uint8_t *)&ultra_command, (uint8_t)sizeof(ultra_command));
    }
    if(flag_ultra == 2){
        // flag_ultra =0;
        ultra_command = 3;
        // cout << "send ultra 3" << endl;
        aaa++;
        sendMessage(send_type, send_id, (uint8_t *)&ultra_command, (uint8_t)sizeof(ultra_command));
    }
    if(aaa >5){
        aaa = 0;
        flag_ultra =0;
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

bool HardwareDriver::validateImuPacket(const IMUEncoder& imu_data) {
    // 根据IMUEncoder实际结构调整
    // 假设IMUEncoder定义如下（根据接收函数判断）：
    // struct IMUEncoder {
    //     int16_t x_acc, y_acc, z_acc;
    //     int16_t omg_x, omg_y, omg_z;
    //     // 可能没有温度字段
    // };
    
    // 检查加速度值是否在合理范围内
    const int32_t ACC_MAX = 16000;    // 16g * 1000
    const int32_t ACC_MIN = -16000;
    
    // 检查角速度值是否在合理范围内
    const int32_t GYRO_MAX = 20000;   // 200°/s * 100
    const int32_t GYRO_MIN = -20000;
    
    // 验证加速度（转换为int32防止溢出）
    int32_t acc_x = imu_data.x_acc;
    int32_t acc_y = imu_data.y_acc;
    int32_t acc_z = imu_data.z_acc;
    
    if (acc_x < ACC_MIN || acc_x > ACC_MAX ||
        acc_y < ACC_MIN || acc_y > ACC_MAX ||
        acc_z < ACC_MIN || acc_z > ACC_MAX) {
        return false;
    }
    
    // 验证角速度
    int32_t gyro_x = imu_data.omg_x;
    int32_t gyro_y = imu_data.omg_y;
    int32_t gyro_z = imu_data.omg_z;
    
    if (gyro_x < GYRO_MIN || gyro_x > GYRO_MAX ||
        gyro_y < GYRO_MIN || gyro_y > GYRO_MAX ||
        gyro_z < GYRO_MIN || gyro_z > GYRO_MAX) {
        return false;
    }
    
    return true;
}

// ========== 新增：初始化异常触发日志 ==========
void HardwareDriver::initAnomalyTriggerLog()
{
    // 生成日志文件名
    time_t now_time = time(0);
    struct tm *time_info = localtime(&now_time);
    const char* home_path = getenv("HOME");
    
    std::string log_dir;
    if (home_path) {
        log_dir = std::string(home_path) + "/.ros/anomaly_trigger_logs/";
        mkdir(log_dir.c_str(), 0755);
    } else {
        log_dir = "/var/log/anomaly_trigger_logs/";
        mkdir(log_dir.c_str(), 0755);
    }
    
    char log_filename[256];
    snprintf(log_filename, sizeof(log_filename),
             "%s/anomaly_trigger_%04d%02d%02d_%02d%02d%02d.csv",
             log_dir.c_str(),
             time_info->tm_year+1900, time_info->tm_mon+1, time_info->tm_mday,
             time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
    
    anomaly_trigger_log_.open(log_filename, std::ios::out);
    
    if (anomaly_trigger_log_.is_open()) {
        anomaly_trigger_log_ << "=== 异常触发IMU+编码器详细日志 ===" << std::endl;
        anomaly_trigger_log_ << "TriggerTime,TriggerReason,AnomalyValue,"
                            << "PacketNo,RelativeTime,"
                            << "x_acc_raw,y_acc_raw,z_acc_raw,"
                            << "omg_x_raw,omg_y_raw,omg_z_raw,"
                            << "right_encoder_raw,left_encoder_raw,"
                            << "right_speed_rpm,left_speed_rpm,"
                            << "flagdirR,flagdirL,"
                            << "base_pose_x,base_pose_y,base_pose_theta" << std::endl;
        
        ROS_INFO("异常触发日志已创建: %s", log_filename);
    }
}

// ========== 新增：添加到环形缓冲区 ==========
void HardwareDriver::addToRingBuffer(const IMUEncoder& imu_data, const ros::Time& receive_time)
{
    ImuPacketRecord record;
    record.receive_time = receive_time;
    memcpy(&record.imu_data, &imu_data, sizeof(IMUEncoder));
    memcpy(&record.encoder_data, &motor_encoder, sizeof(MotorEncoder));  // 保存当前编码器值
    record.is_recorded = true;
    
    imu_ring_buffer_.push_back(record);
    
    // 保持缓冲区大小
    while (imu_ring_buffer_.size() > IMU_RING_BUFFER_SIZE) {
        imu_ring_buffer_.pop_front();
    }
}

// ========== 新增：将环形缓冲区数据写入日志 ==========
void HardwareDriver::dumpRingBufferToLog(const std::string& anomaly_reason, double anomaly_value)
{
    if (!anomaly_trigger_log_.is_open()) {
        ROS_ERROR("异常触发日志未打开，无法写入");
        return;
    }
    
    anomaly_trigger_count_++;
    
    // 获取当前时间作为触发时间
    time_t raw_time;
    struct tm *timeinfo;
    char trigger_time[80];
    time(&raw_time);
    timeinfo = localtime(&raw_time);
    strftime(trigger_time, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
    
    ros::Time trigger_ros_time = ros::Time::now();
    
    // 写入分隔线和触发信息
    anomaly_trigger_log_ << "\n=== 异常触发 #" << anomaly_trigger_count_ << " ===" << std::endl;
    anomaly_trigger_log_ << "TriggerTime," << trigger_time << ",,"
                        << "TriggerROS," << std::fixed << std::setprecision(6) 
                        << trigger_ros_time.toSec() << ",,"
                        << "Reason," << anomaly_reason << ",,"
                        << "Value," << anomaly_value << ",,\n";
    
    // 写入表头（仅作参考）
    anomaly_trigger_log_ << "--- 触发前数据 (最近" << imu_ring_buffer_.size() << "个包) ---" << std::endl;
    
    // 写入环形缓冲区中的所有数据
    int packet_offset = -imu_ring_buffer_.size();
    for (const auto& record : imu_ring_buffer_) {
        // 计算相对于触发时间的偏移（秒）
        double time_offset = (trigger_ros_time - record.receive_time).toSec();
        
        // 计算转速
        double right_speed_rpm = (double)record.encoder_data.right_speeder / 720.0;
        double left_speed_rpm = (double)record.encoder_data.left_speeder / 720.0;
        
        anomaly_trigger_log_ << trigger_time << ","
                            << anomaly_reason << ","
                            << anomaly_value << ","
                            << packet_offset << ","  // 相对包序号
                            << std::fixed << std::setprecision(3) << time_offset << ","
                            << record.imu_data.x_acc << ","
                            << record.imu_data.y_acc << ","
                            << record.imu_data.z_acc << ","
                            << record.imu_data.omg_x << ","
                            << record.imu_data.omg_y << ","
                            << record.imu_data.omg_z << ","
                            << record.encoder_data.right_speeder << ","
                            << record.encoder_data.left_speeder << ","
                            << std::setprecision(2) << right_speed_rpm << ","
                            << left_speed_rpm << ","
                            << flagdirR << ","  // 注意：这里记录的是当前方向，不是历史方向
                            << flagdirL << ","
                            << std::setprecision(6) << base_pose.x << ","
                            << base_pose.y << ","
                            << base_pose.theta << std::endl;
        
        packet_offset++;
    }
    
    anomaly_trigger_log_ << "--- 触发后数据 (将记录" << ANOMALY_LOG_FRAMES << "个包) ---" << std::endl;
    anomaly_trigger_log_.flush();
    
    // 设置异常日志状态，开始记录后续数据
    is_logging_anomaly_ = true;
    anomaly_log_remaining_frames_ = ANOMALY_LOG_FRAMES;
    
    ROS_WARN("异常 #%d 触发，已导出 %zu 个历史数据包，将记录后续 %d 个包",
             anomaly_trigger_count_, imu_ring_buffer_.size(), ANOMALY_LOG_FRAMES);
}

// ========== 新增：检查并触发异常日志 ==========
void HardwareDriver::checkAndTriggerAnomalyLog(const std::string& reason, double abnormal_value, 
                                              const std::string& extra_info, double dt)
{
    // 只针对编码器相关的异常触发IMU详细日志
    if (reason.find("ENCODER") != std::string::npos || 
        reason.find("ENCODER_JUMP") != std::string::npos ||
        reason.find("ENCODER_VALUE_ABNORMAL") != std::string::npos ||
        reason.find("ENCODER_INCREMENT_TOO_LARGE") != std::string::npos ||
        reason.find("POSITION_JUMP") != std::string::npos) {
        
        // 避免频繁触发（10秒内只触发一次）
        static ros::Time last_trigger_time;
        ros::Time now = ros::Time::now();
        
        if ((now - last_trigger_time).toSec() < 10.0) {
            return;  // 10秒内不重复触发
        }
        last_trigger_time = now;
        
        // 触发异常日志记录
        dumpRingBufferToLog(reason, abnormal_value);
    }
}

// ========== 修改logAnomaly函数，添加触发机制 ==========
void HardwareDriver::logAnomaly(const std::string& reason, double abnormal_value, 
                               const std::string& extra_info, double dt) {
    if (!anomaly_log_.is_open()) return;
    
    anomaly_count_++;
    
    // 获取当前时间
    time_t rawtime;
    struct tm *timeinfo;
    char time_buffer[80];
    
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
    
    // 获取ROS时间
    ros::Time ros_time = ros::Time::now();
    
    // 写入日志（CSV格式）
    anomaly_log_ << time_buffer << ","  // 系统时间
                << reason << ","       // 异常类型
                << abnormal_value << ","  // 异常值
                << base_pose.x << ","  // 当前X位置
                << base_pose.y << ","  // 当前Y位置
                << dt << ","   
                << motor_encoder.right_speeder << ","  // 右编码器原始值
                << motor_encoder.left_speeder << ","   // 左编码器原始值
                << imu_wz << ","       // IMU角速度
                << extra_info          // 额外信息
                << std::endl;
    
    // 立即写入文件
    anomaly_log_.flush();
    
    // ========== 新增：检测到异常时触发IMU详细日志 ==========
    checkAndTriggerAnomalyLog(reason, abnormal_value, extra_info, dt);
    
    // 同时在ROS日志中记录
    ROS_WARN("异常记录 #%d: %s, 异常值=%.1f, 位置=(%.1f,%.1f)",
            anomaly_count_, reason.c_str(), abnormal_value,
            base_pose.x, base_pose.y);
}

 
// ========== logImuPacket函数，记录常规数据 ==========
void HardwareDriver::logImuPacket(const IMUEncoder& imu_data, 
                                 const ros::Time& receive_time,
                                 bool is_anomaly,
                                 const std::string& anomaly_reason) {
    if (!imu_log_.is_open()) return;
    
    imu_packet_count_++;
    
    // 获取系统时间
    time_t raw_time;
    struct tm *timeinfo;
    char time_buffer[80];
    time(&raw_time);
    timeinfo = localtime(&raw_time);
    strftime(time_buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
    
    // ROS时间
    double rostime_seconds = receive_time.toSec();
    
    // 计算编码器转速
    double right_speed_rpm = (double)motor_encoder.right_speeder / 720.0;
    double left_speed_rpm = (double)motor_encoder.left_speeder / 720.0;
    
    // 写入常规日志（每一条都记录）
    imu_log_ << imu_packet_count_ << ","
             << time_buffer << ","
             << std::fixed << std::setprecision(3) << rostime_seconds << ","
             << (is_anomaly ? "ABNORMAL" : "NORMAL") << ","
             << (is_anomaly ? anomaly_reason : "") << ","
             << imu_data.x_acc << ","
             << imu_data.y_acc << ","
             << imu_data.z_acc << ","
             << imu_data.omg_x << ","
             << imu_data.omg_y << ","
             << imu_data.omg_z << ","
             << motor_encoder.right_speeder << ","
             << motor_encoder.left_speeder << ","
             << std::setprecision(2) << right_speed_rpm << ","
             << left_speed_rpm << ","
             << (is_anomaly ? anomaly_reason : "Normal") << std::endl;
    
    // 每500个包flush一次，避免频繁IO
    if (imu_packet_count_ % 500 == 0) {
        imu_log_.flush();
    }
}