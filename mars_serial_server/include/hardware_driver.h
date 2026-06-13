#ifndef __HARDWARE_DRIVER_H__
#define __HARDWARE_DRIVER_H__

#include <fstream>      
#include <sstream>
#include <string>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <atomic>       // [heading-straight-pd] imu_heading_ready_ 标志(advertise→publish happens-before)

#include <AsyncSerial.h>
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>

#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Int32MultiArray.h>

#include <mars_msgs/BatteryDatas.h>
#include <mars_msgs/Pose.h>
#include <mars_msgs/Twist.h>
#include <mars_msgs/Odom.h>
#include <mars_msgs/Point.h>
#include <mars_msgs/MotionStates.h>
#include <mars_msgs/DeviceStates.h>
#include <mars_msgs/DeviceOnoff.h>
#include <mars_msgs/Infrared.h>

#include "tf/transform_broadcaster.h"
#include <emath.h>
#include <public.h>
#include <queue>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
 

#define PI_M       3.1415926535897932384626433832795029
#pragma pack(1)


using namespace cv;

typedef struct{
    int right_speeder;
    int left_speeder;
}MotorEncoder;

typedef struct{
    int16_t x_acc;
    int16_t y_acc;
    int16_t z_acc;
    int16_t omg_x;
    int16_t omg_y;
    int16_t omg_z;
    int16_t roll;   // 横滚角
    int16_t pitch;  // 俯仰角  
    int16_t yaw;    // 偏航角
}IMUEncoder;
typedef struct{
    int8_t     ultra_id;
    uint16_t    ultra_dist;
}UltraDatas;

typedef struct{
    int     right_speed;
    int     left_speed;
    int     spinl_speed;
    int     spinr_speed;
}MotorCommand;

typedef struct{
    float theta;
    float imu[6];
    float sonar[4];
}SensorDatas;

typedef struct{
    float battery_percentage;			// Ê£ÓàµçÁ¿, µ¥Î»%
    float battery_voltage;              // µç³ØµçÑ¹, µ¥Î»V
    float battery_current;				// µç³ØµçÁ÷, µ¥Î»A
    float battery_temperature;			// µç³ØÎÂ¶È, µ¥Î»¡æ
    float battery_remain_time;			// ¿ÉÓÃÊ±¼ä, µ¥Î»min
}BatteryDatas;
#pragma pack(0)
struct Head
{
   uint8_t  flaga;
   uint8_t  flagb;
   uint8_t  msgtype;
   uint8_t  msgid;
   uint8_t  length;
};

struct Tail
{
   uint8_t  flaga;
   uint8_t  flagb;
};

struct Infrared
{
   uint8_t  data_1;
   uint8_t  data_2;
   uint8_t  data_3;
   uint8_t  data_4;
   uint8_t  data_5;
   uint8_t  data_6;
   uint8_t  data_7;
   uint8_t  data_8;
};

static const unsigned short RS_PAYLOAD_MAX = 125;
static const unsigned short RS_DATAFLOW_MAX = 255;
#define DEV_TYPE_MAX		 3
#define DEV_ID_MAX			 10
#define PACKET_PAYLOAD_INDEX 5
typedef enum {
    STATE_RECV_FLAGA = 0,
    STATE_RECV_FLAGB,
    STATE_RECV_MSTYPE,
    STATE_RECV_MSGID,
    STATE_RECV_LEN,
    STATE_RECV_PAYLOAD,
    STATE_RECV_CHECK,
    STATE_RECV_TLAGA,
    STATE_RECV_TLAGB,
}RECEIVE_STATE;

struct HardwareMessage
{
    struct Head head;
    uint8_t payload[RS_PAYLOAD_MAX];
    uint8_t check;
    struct Tail tail;
    uint8_t payload_count;
    uint8_t cnt;
    HardwareMessage(){}
    HardwareMessage(uint8_t msgtype, uint8_t msgid, uint8_t* pay_load=0, uint8_t len=0){
        head.flaga = 0x5A;
        head.flagb = 0xA5;
        head.msgtype = msgtype;
        head.msgid = msgid;
        head.length = len;


        tail.flaga = 0x5C;
        tail.flagb = 0xC5;

        check = 0;

        if(pay_load != 0 && len !=0)
            memcpy(this->payload, pay_load, len);

        uint8_t* _send_message = (uint8_t*)this;

        unsigned int i = 0;
        for (i = 0; i < sizeof(head)+head.length; i++)
            check ^= _send_message[i];
    }
};


class HardwareDriver
{
public:
    HardwareDriver(CallbackAsyncSerial* serial);
    ~HardwareDriver();
    void receiveMessage(const char *data, unsigned int len);
    void rosNode();
    void UpdateImu();
    void UpdateImage();
    void UpdateUltra();

    void sendAutoChargeCommand();
    void sendTestVelCommand();
    void sendUltraCommand();
    void sendStopCommand();        // [vcu] 统一停车帧（关停时由 main 调用 + 看门狗调用），公有故 serial_starup 可访问

    void receiveInfDatas();
    
    AresCmd *ares_cmd;
private:
    bool unpack_data(HardwareMessage *recv_msg, uint8_t recv_data);
    void receiveImuDatas();
    void receiveWheelDatas(); 

    void GrabImuData(const sensor_msgs::ImuConstPtr &imu_msg);
    
    bool packData(HardwareMessage *send_msg, uint8_t *send_data);
    void sendMessage(int msgtype, int msgid, uint8_t* payload, uint8_t len);
    
    
    void sendKeyboardVelCommand(const geometry_msgs::Twist& cmd_vel);
    void sendAutoVelCommand(const geometry_msgs::Twist &auto_vel_cmdmsg);
    void cmdVelWatchdog();         // [vcu] /cmd_vel 看门狗回调（私有，仅内部 Timer 调用）

    double speedSmooth(double  expected_speed,  double used_acce);
    double omegaSmooth(double  expected_omega,  double used_alpha);
 
    void publishTfTransforms();


      // ========== 电机编码器日志函数 ==========
    void logMotorEncoderData(const std::string& data_source, int packet_length);
    
    // ========== IMU+编码器日志函数 ==========
    void logImuEncoderData(const IMUEncoder& imu_data, int packet_length, bool encoder_stale = false);

     // 日志记录函数（可以放在类定义的末尾）
    void logAnomaly(const std::string& reason, double abnormal_value = 0.0, 
                const std::string& extra_info = "", double dt = 0.0);
    
        // IMU日志方法
    void logImuPacket(const IMUEncoder& imu_data, 
                     const ros::Time& receive_time,
                     bool is_anomaly = false, 
                     const std::string& anomaly_reason = "");
    
    bool validateImuPacket(const IMUEncoder& imu_data);
    
 

private:
    CallbackAsyncSerial*    hardware_serial_;
    RECEIVE_STATE           recv_state;
    HardwareMessage         recv_msg;
    HardwareMessage         send_msg;

    MachineParams           machine_params;
    MotionParams            motion_params;
    MotorEncoder            motor_encoder;
    IMUEncoder              imu_encoder;
    SensorDatas             sensor_datas;
    UltraDatas              ultra_datas;
    
    mars_msgs::Infrared     infrared_datas;
    mars_msgs::DeviceOnoff  deviceOnoff_sta;

    MotorCommand            motor_command;

    // ========== [vcu] 简化版 VCU 差速补偿 + 看门狗 参数/状态（默认值=保持现行为）==========
    double                  omega_gain_       = 6.4;    // 替代写死 6.4
    double                  track_bias_       = 0.0;    // 左右履带静态偏置补偿，夹到 [-0.15, +0.15]
    double                  motor_scale_      = 230.0;  // 替代写死 230
    double                  max_track_speed_  = 1.0;    // 履带目标速度限幅(m/s)，仅兜失控
    int                     max_motor_cmd_    = 300;    // 电机命令限幅，仅兜失控
    double                  cmd_vel_timeout_  = 0.5;    // /cmd_vel 看门狗超时(s)
    ros::Time               last_cmd_vel_time_;         // 最近一次 cmd_vel 到达时刻
    bool                    cmd_vel_received_ = false;  // 是否已收到过任何 cmd_vel
    bool                    wd_timed_out_     = false;  // 看门狗是否已进入超时态(边沿触发用)
    int                     wd_tick_          = 0;      // 看门狗重申节流计数(20Hz/4≈5Hz)

    ros::NodeHandle         nh_;
    Emath                   emath;


     // 添加这些日志成员（在最前面或最后面）
    std::ofstream anomaly_log_;
    int anomaly_count_;
    geometry_msgs::Pose2D last_valid_pose_;
    ros::Time last_valid_time_;


    //添加编码器序号变量
    int encoder_seq_;
    int last_encoder_seq_;

    // ========== 专用日志文件对象 ==========
    std::ofstream motor_encoder_log_;    // 电机编码器专用日志
    std::ofstream imu_encoder_log_;       // IMU+编码器专用日志


    // ========== \编码器同步和时间戳相关 ==========
    bool encoder_synced_;           // 编码器是否已同步
    ros::Time last_encoder_stamp_;  // 上次编码器数据的时间戳
    int skip_encoder_count_;        // 跳过的编码器帧数（可选）


    float imu_ax;
    float imu_ay;
    float imu_az;
    float imu_wx;
    float imu_wy;
    float imu_wz;
    float imu_theta;
    float imu_roll;   // 横滚角
    float imu_pitch;  // 俯仰角
    float imu_yaw;    // 偏航角

    float imu_ax_last;
    float imu_ay_last;
    float imu_az_last;
    float imu_wz_last ;

    vector<Infrared> Infrared_buf;
    std_msgs::Int32MultiArray dist_datas;

    int   update_nums;
    float theta_last;
    float delta_theta_last;
    float dtheta_sum;
    float theta_filter;

    float dtheta_array[100];
    sensor_msgs::Imu        imu_datas;

    float                   omega_last;
    double last_right_econd, last_left_econd;
    int wheel_flag = 0;

    geometry_msgs::Twist    base_twist_state;

    geometry_msgs::Pose2D   base_pose;

    // geometry_msgs::PoseWithCovarianceStamped   current_state_ekf;  

    nav_msgs::Odometry      base_odom_state;


    nav_msgs::Odometry      wheel_odom;

    sensor_msgs::Range      sonarA_datas;
    sensor_msgs::Range      sonarB_datas;

    mars_msgs::BatteryDatas battery_datas;
    mars_msgs::Point        odom_delta_state;

    double base_dist_x = 0.0;

   // mars_msgs::DeviceOnff   device_onff_state;

    

    // ----------------------------- Publisher -----------------------------
    // wheel
    ros::Publisher      Base_pose_pub_;
    ros::Publisher      Base_twist_pub_;
    ros::Publisher      wheel_odom_pub_;
    ros::Publisher      Odom_delta_pub;

    // seneor
    ros::Publisher      imu_data_pub_;
    ros::Publisher      sonarA_dat_pub_;
    ros::Publisher      sonarB_dat_pub_;
    // battery
    ros::Publisher      battery_dat_pub_;
    ros::Publisher      infrared_data_pub_;
    ros::Publisher      ultra_data_pub_;
    ros::Publisher      imu_heading_pub_;   // [heading-straight-pd] 融合航向 /imu_heading (std_msgs/Float32, 度)
    // advertise 在 rosNode 线程、首帧 publish 在串口 RX 线程，无 happens-before；
    // 原 if(imu_heading_pub_) guard 是对 ros::Publisher 内部 shared_ptr 的竞态读=C++11 UB。
    // 用 atomic 标志 + release/acquire 建立同步：advertise 完成后 store(true,release)，RX 侧 load(acquire) 为真才发。
    std::atomic<bool>   imu_heading_ready_{false};

    // ----------------------------- Subscriber ----------------------------
    ros::Subscriber     manualVel_cmd_sub_;
    ros::Subscriber     autoVel_cmd_sub_;
    ros::Subscriber   	motion_state_sub_;
    ros::Subscriber   	ekfPose_sta_sub_;
    ros::Subscriber     keyboardVel_cmd_sub_;
    ros::Subscriber     imu_data_sub_;

    int     right_counter_last;
    int     left_counter_last;
    double  last_speed_command;
    double  last_omega_command;
    double  last_rotate_command;

    bool    obst_slow;
    bool    obst_match_stop;
    bool    obst_scan_stop;
    bool    received_motion_pose;

    ros::Time current_time;
    ros::Time last_time;
    double last_imu_time;
    int   imu_flag = 0;
    // Emath                   emath;
    ros::Time receive_end_time;
    ros::Time curr_time;
    ros::Time las_time;

    std::vector<double>         laser_vector3_;
    std::vector<double>         laser_quaternion_;
    tf::TransformBroadcaster    bc_laser2base;      // for laser -> base tf
    tf::TransformBroadcaster    bc_scan2base; 
    tf::TransformBroadcaster    bc_imu2base; 
    tf::TransformBroadcaster    bc_scan2imu; 
    
    // TF广播器
    tf::TransformBroadcaster   tf_broadcaster_;
     
    int imu_cnt = 0;
    int flag_ultra = 0;
    
    cv::VideoCapture cap;
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
    cv::Mat frame;
    cv::Mat frame1, frame2;
    cv::Mat GrayA, GrayB;
    image_transport::Publisher pub1;
    image_transport::Publisher pub2;
    double test_speed = 0.0, test_omega = 0.0;

    int flagdirR =1;
    int flagdirL =1;
    // image_transport::CameraPublisher pub1;
    // image_transport::CameraPublisher pub2;
 
    std::ofstream imu_log_;
    int imu_packet_count_;      // 接收的IMU数据包总数
    int imu_anomaly_count_;     // 异常的IMU数据包数

    bool encoder_initialized_;              // 编码器是否已初始化
    ros::Time last_encoder_time_;           // 上次有效编码器时间
    static constexpr double ENCODER_TIMEOUT = 0.5;  // 编码器超时时间0.5秒


    std::mutex encoder_mutex;      // 互斥锁
    bool encoder_updated = false;   // 编码器是否更新
    double last_right_rev = 0.0;    // 上次右编码器圈数
    double last_left_rev = 0.0;     // 上次左编码器圈数

  // 新增：平行变量
    double last_right_econd_t0;
    double last_left_econd_t0;
 

     // ========== IMU数据环形缓冲区，用于异常触发记录 ==========
    struct ImuPacketRecord {
        ros::Time receive_time;
        IMUEncoder imu_data;
        MotorEncoder encoder_data;  // 同时保存编码器数据
        bool is_recorded;
        
        ImuPacketRecord() : is_recorded(false) {
            memset(&imu_data, 0, sizeof(IMUEncoder));
            memset(&encoder_data, 0, sizeof(MotorEncoder));
        }
    };
    
    std::deque<ImuPacketRecord> imu_ring_buffer_;
    const int IMU_RING_BUFFER_SIZE = 500;  // 可配置：500个包 ≈ 5秒（假设100Hz）
    
    // 异常触发后的日志文件
    std::ofstream anomaly_trigger_log_;
    bool is_logging_anomaly_;
    int anomaly_log_remaining_frames_;  // 异常后还需记录多少帧
    const int ANOMALY_LOG_FRAMES = 200; // 异常触发后额外记录200帧（约2秒）
    
    // 异常触发计数器
    int anomaly_trigger_count_;
    
    // 新增方法
    void initAnomalyTriggerLog();
    void addToRingBuffer(const IMUEncoder& imu_data, const ros::Time& receive_time);
    void dumpRingBufferToLog(const std::string& anomaly_reason, double anomaly_value);
    void checkAndTriggerAnomalyLog(const std::string& reason, double abnormal_value, 
                                  const std::string& extra_info, double dt);
    
 
    // ========== 数据同步相关 ==========
    struct SyncData {
        ros::Time timestamp;
        IMUEncoder imu;
        MotorEncoder encoder;
        bool has_encoder;  // 是否有新的编码器数据
        bool has_imu;      // 是否有IMU数据
    };

    std::deque<SyncData> data_queue_;
    boost::mutex queue_mutex_;
    const int MAX_QUEUE_SIZE = 200;

    // ========== 零漂校准相关 ==========
    double imu_bias_;
    int bias_calib_count_;
    bool bias_calibrated_;
    boost::optional<IMUEncoder> latest_imu_;

    // ========== 运动状态检测 ==========
    int stationary_count_;
    bool is_stationary_;
    ros::Time last_motion_time_;

    double current_cmd_linear_x;   // w/s 键的值（前后）
    double current_cmd_angular_z;  // a/d 键的值（左右旋转）

    // ========== 启动保护（修改这里） ==========
    int startup_protect_count_;
    static const int STARTUP_PROTECT_THRESHOLD;  // 去掉=50
    static const int STATIONARY_THRESHOLD;       // 去掉=20
    static const double MOTION_THRESHOLD;         // 去掉=0.01

};

#endif


