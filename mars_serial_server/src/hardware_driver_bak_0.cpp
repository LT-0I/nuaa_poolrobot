#include <hardware_driver.h>

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

    memset(&motor_encoder, 0, sizeof(MotorEncoder));  // ← 关键，防未初始化
	memset(&imu_encoder,   0, sizeof(IMUEncoder));

    encoder_updated = false;
    wheel_flag = 0;
    imu_flag = 0;
    encoder_initialized_ = false; 

 
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
}

HardwareDriver::~HardwareDriver()
{
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
                    // memcpy(&motor_encoder, recv_msg.payload, sizeof(MotorEncoder));
                    std::lock_guard<std::mutex> lock(data_mutex);
					if (recv_msg.head.length == sizeof(MotorEncoder)) {   // ← 长度校验！防错包
						memcpy(&motor_encoder, recv_msg.payload, sizeof(MotorEncoder));
						encoder_updated = true;
						last_encoder_time_ = ros::Time::now(); 
					} else {
						ROS_WARN("CM_BATTERY_DATA_ID 长度错误！期望%zu 实际%d", sizeof(MotorEncoder), recv_msg.head.length);
 
					}

                    break;
                }
                case CM_MOTOR_CTL_ID:
                    
                    memcpy(&battery_datas, recv_msg.payload, sizeof(mars_msgs::BatteryDatas));
                    // receiveBatteryDatas();
                    break;
                case CM_IMU_DATA_ID:
                {
                    // cout << "receive imu data" << endl;
                    // memcpy(&imu_encoder, recv_msg.payload, sizeof(IMUEncoder));
                    // receiveImuDatas();
                    std::lock_guard<std::mutex> lock(data_mutex);
					if (recv_msg.head.length == sizeof(IMUEncoder)) {
						memcpy(&imu_encoder, recv_msg.payload, sizeof(IMUEncoder));
						receiveImuDatas();
					}
                
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



void HardwareDriver::UpdateImu()
{
    tf::Quaternion imu_quat;

    ros::Time current_time = ros::Time::now();
 
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
        cout << "IMU次数: " << imu_count << ", 编码器更新次数: " << encoder_count << endl;
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
        dt = 0.01;
        last_time = current_time;
        return;
    }
    
    if (dt > 0.2) {
        // ROS时间可能发生跳跃（如重启、时间同步等）
        ROS_ERROR("时间跳跃异常: dt=%.3fs > 0.2s，跳过本次更新", dt);
        last_time = current_time;
        
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
    /*******
    double cur_right_econd = (double)motor_encoder.right_speeder/720.0f;
    double cur_left_econd = (double)motor_encoder.left_speeder/720.0f;

 
    
    // 检查编码器值是否异常（防止数据解析错误）
    if (fabs(cur_right_econd) > 10000.0 || fabs(cur_left_econd) > 10000.0) {
        std::cout << "编码器值异常: right=" << cur_right_econd 
              << ", left=" << cur_left_econd 
              << "，使用上次值" << std::endl;
        cur_right_econd = last_right_econd;
        cur_left_econd = last_left_econd;
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
    ***/

    // ==================== 异步保护逻辑 ====================
    double cur_right_econd = last_right_econd;   // 默认使用上次有效值
    double cur_left_econd  = last_left_econd;
    bool has_new_encoder = false;  // 新增局部变量

    {
        std::lock_guard<std::mutex> lock(data_mutex);

        if (encoder_updated) {                    // 只有收到新编码器包时才读取
            encoder_updated = false;              // 立即消费，防止重复使用
            has_new_encoder = true;

            cur_right_econd = (double)motor_encoder.right_speeder / 720.0f;
            cur_left_econd  = (double)motor_encoder.left_speeder  / 720.0f;

            // 检查编码器值是否异常（防止数据解析错误）
            if (fabs(cur_right_econd) > 10000.0 || fabs(cur_left_econd) > 10000.0) {
                std::cout << "编码器值异常: right=" << cur_right_econd
                      << ", left=" << cur_left_econd
                      << "，使用上次值" << std::endl;
                cur_right_econd = last_right_econd;
                cur_left_econd  = last_left_econd;
            }
           
            // 检查编码器突变（单次变化过大）
            double right_diff = fabs(cur_right_econd - last_right_econd);
            double left_diff  = fabs(cur_left_econd  - last_left_econd);
         
            if (right_diff > 100.0 || left_diff > 100.0) {
               std::cout << "编码器突变: right_diff=" << right_diff
                      << ", left_diff=" << left_diff
                      << "，可能数据错误" << std::endl;
                // 使用有限的变化量
                double max_diff = 10.0;
                if (right_diff > max_diff) {
                    cur_right_econd = last_right_econd +
                        ((cur_right_econd > last_right_econd) ? max_diff : -max_diff);
                }
                if (left_diff > max_diff) {
                    cur_left_econd = last_left_econd +
                        ((cur_left_econd > last_left_econd) ? max_diff : -max_diff);
                }
            }
        }
    }   // 锁在这里自动释放

    // 如果本次没有新编码器数据，则跳过位置积分（防止用旧数据或垃圾值爆炸）
    if (!has_new_encoder && wheel_flag != 0) {
        goto continue_imu_publish;   // ← 跳转到下面标记位，继续执行IMU和发布部分
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
    }

    if(flagdirL == 1){
        dleft_econd  = cur_left_econd - last_left_econd;
    }else{
        dleft_econd  = -(cur_left_econd - last_left_econd);
    }

    cout << "dright_econd =   " << dright_econd << endl;
    cout << "dleft_econd =   " << dleft_econd << endl;
   
    continue_imu_publish:
    
    // 检查增量是否合理
    	/****
    double max_single_move = 0.2;  // 米
    double wheel_circumference = 1.5;  // 应该是轮子周长（米/圈）

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
	**/
	
	double wheel_circumference = M_PI * machine_params.wheel_diameter;  // ← 关键修正！使用真实周长
    double max_single_move = 0.3;   // 可以适当放宽到 0.25~0.3（水下可能有轻微打滑）

    // dright_econd 单位是圈数，直接乘以周长得到米
    double abs_right_move = fabs(dright_econd * wheel_circumference);
    double abs_left_move  = fabs(dleft_econd  * wheel_circumference);

    if (abs_right_move > max_single_move || abs_left_move > max_single_move) {
        cout << "单次移动过大: right=" << abs_right_move
             << "m, left=" << abs_left_move << "m (周长=" 
             << wheel_circumference << "m)" << endl;
       
        // 压缩圈数
        double scale = max_single_move / fmax(abs_right_move, abs_left_move);
        dright_econd *= scale;
        dleft_econd  *= scale;
    }

    // 计算位移（米）
    base_dist_x = (dright_econd + dleft_econd) / 2.0 * wheel_circumference;
 



    // 检查base_dist_x是否合理
    if (fabs(base_dist_x) > max_single_move) {
        std::cout << "base_dist_x异常: " << base_dist_x << "m > " << max_single_move 
              << "m，跳过本次位置更新" << std::endl;
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
    /*******/
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

    motor_command.right_speed = -left_speed * 230;
    motor_command.left_speed = right_speed * 230;
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
    // cout << "dt " << dt << endl;

    double current_speed_command = cmd_vel.linear.x;
    double current_omega_command = cmd_vel.angular.z * 6.4 ;

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