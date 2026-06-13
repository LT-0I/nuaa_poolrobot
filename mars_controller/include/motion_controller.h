#ifndef __MOTION_CONTROLLER_H__
#define __MOTION_CONTROLLER_H__

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>   // [heading-straight-pd] 融合航向 /imu_heading

#include <mars_msgs/MarsStates.h>
#include <mars_msgs/MotionStates.h>
#include <mars_msgs/ModuleOnoff.h>
#include <mars_msgs/ModuleAction.h>
#include <mars_msgs/ModuleStates.h>
#include <mars_msgs/TaskState.h>
#include <mars_msgs/Point.h>
#include <mars_msgs/Path.h>
#include <mars_msgs/GlobalPath.h>
#include <mars_msgs/Infrared.h>

#include <std_srvs/Trigger.h>  // 添加这行

#include <emath.h>
#include <public.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <thread>

using namespace std;
// using namespace tf;

#define LOCATION_STATE_NONE 0x00
#define LOCATION_STATE_SUCCESS 0x11
#define LOCATION_STATE_ERROR 0x22

#define SYSTEM_CONTROL 0
#define MANUAL_CONTROL 1

#define NAV_DISTANCE_RESOLUTION 0.005
#define NAV_ANGLE_RESOLUTION 0.02

typedef struct _MotionStep
{
    enum ActionType
    {
        MT_NO_ACTION = 0,
        MT_TO_TRACE_PATH = 1,
        MT_TO_TRACE_ANGLE = 2,
    };
    ActionType action;
    int path_num;
    int start_index;
} MotionStep;

// 工作阶段
enum Session : uint8_t
{
    SES_FREE_STATE  = 0, // 空闲
    SES_ERROR_STATE = 1, // 错误
    SES_STOP_STATE  = 2, // 停止
    SES_PAUSE_STATE = 3, // 暂停
    SES_WALL_STATE  = 4, // 寻墙
    SES_SWEEP_STATE = 5, // 扫描
    SES_COVER_STATE = 6, // 覆盖
};

enum SweepAction : uint8_t
{
    STRAIGHT_ON = 0,         // 直行
    DETOUR = 1,              // 绕障
    STRA_BEFORE_TURNING = 2, // 转弯前直行一个车身位
    TURNING = 3,             // 转弯
    STRA_AFTER_TURNING = 4,  // 转弯后直行一个车身位
};

enum FindWallAction : uint8_t
{
    ROTATION_IN_PLACE     = 0, // 原地旋转
    NEAR_WALL_ROTATION    = 1, // 靠近墙面
    NEAR_WALL_STRAIGHT    = 2, // 靠近墙面
    ALIGNED_WALL          = 3, // 对齐墙面
};

enum ClimbWallAction : uint8_t
{
    STA_0 = 0, // 初始平行于水面(小于85时)
    STA_1 = 1, // 贴近墙面(大于200时)
    STA_2 = 2, // 靠近水面(小于100时)
    STA_3 = 3, // 靠近水底(大于200时)
    STA_4 = 4, // 已离开墙面(小于100时)
};

enum CoverAction : uint8_t
{
    SEARCH_START_POINT = 0,     // 寻找起始点
    COVER_GO_STRAIGHT  = 1,     // 直行
    CLIMB_WALL         = 2,     // 爬墙
    COVER_TURNING      = 3,     // 转弯
    COVER_OVER         = 4,     // 覆盖完成
};

enum FrameState : uint8_t
{
    FRAME_X = 0,
    FRAME_Y = 0,
};

// [heading-straight-pd] 航向保持走直线模式的内部阶段
enum HsPhase : uint8_t
{
    HS_WAIT_YAW = 0,  // 等待 /imu_heading 到达（兼版本闸门）
    HS_STILL    = 1,  // 静止采 yaw0（去零偏）
    HS_DRIVE    = 2,  // 行驶 + PD 航向保持
    HS_PAUSE    = 3,  // 周期暂停（IMU 合规 ≤3min/≥3s，不重采 yaw0）
    HS_DONE     = 4,  // 结束：持续发零
};

class MotionController
{
public:
    MotionController();
    ~MotionController();
    void rosNode();
    void Run();
    void shutdownStop();   // [heading-straight-pd] 退出前补发零（main 调用）
    void transPlanPathToActionStep();
    void wheel_odom_test(int test_type);
    void update_path_msg();
    AresCmd *ares_cmd;

private:
    void get_segment_action();
    void transActionStepToTracePoints();
    void computeMotionCommands(MotionStep::ActionType action);
    void sloveAngleOmega(double theta_error);
    void slovePathSpeed(Point goal_point);
    bool traceAngle();
    bool tracePath();
    void traceZero();

    void laserMatchCloudCallback(const sensor_msgs::PointCloud2 &laser_match_cldmsg);
    void LocationStateCallback(const std_msgs::Int8 &state);
    void scanCallback_1(const sensor_msgs::LaserScan &scanmsg);
    void scanCallback_2(const sensor_msgs::LaserScan &scanmsg);
    void distDatasCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void getLaserPoseInMap();
    bool getBasePoseInMap();
    void task_state_switch();
    bool PinpointNavigation();

    void handleWheelTwistStates(const geometry_msgs::Twist wheel_twist_stamsg);
    void handleWheelPoseStates(const mars_msgs::Point odom_delta);
    void handleInfraredDatas(const mars_msgs::Infrared msg);
    void ekfPoseCallback(const geometry_msgs::Pose2D ekf_pose_stamsg);
    void WheelPoseCallback(const geometry_msgs::Pose2D msg);
    void vslamPoseCallback(const geometry_msgs::Pose2D vslam_pose_stamsg);
    void vslam3DPoseCallback(const nav_msgs::Odometry odom_msg);
    void stationCallback(const std_msgs::Int8 msg);

    bool isGoalReached(); // 判断是否到达终点
    void projectAndFilterLaserPoints(vector<Point> &laser_points, geometry_msgs::Pose2D robot_pose, double max_forward_distance, double max_lateral_distance);
    void sendAutoChargeCommand();

    Eigen::Array2i GetCellIndex(Point &point);
    void ParseOptions2DFile();

    void TraceWall();
    void FindWall();
    void FullCoverage();
     
    //
    void printDebugInfo();
    void startMotorCalibration();
    void updateMotorCalibration();
    void applyMotorCompensation(geometry_msgs::Twist& cmd);
    bool handleCalibrateRequest(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res);

    // [heading-straight-pd] 航向保持走直线（融合yaw PD）
    void RunHeadingStraight();
    void imuHeadingCallback(const std_msgs::Float32::ConstPtr& msg);
    void openLogIfNeeded();
    static double wrapPi(double a);   // 角度 wrap 到 [-pi, pi]


private:
    ros::NodeHandle nh_;
    ros::Subscriber wheelTwist_sta_sub_;
    ros::Subscriber wheelPose_sta_sub_;
    ros::Subscriber infrared_data_sub_;
    ros::Subscriber sub_cloud_outmap;
    ros::Subscriber sub_locationState;
    ros::Subscriber sub_scan_1;
    ros::Subscriber sub_scan_2;
    ros::Subscriber dist_datas_sub_;
    ros::Subscriber sub_ekf_pose;
    ros::Subscriber sub_vslam_pose;
    ros::Subscriber sub_vslam_3d_pose;
    ros::Subscriber cmd_station_edit_sub_;
    ros::Subscriber sub_wheel_pose;

    ros::Publisher motion_state_pub_;
    ros::Publisher task_state_pub_;
    ros::Publisher motionTwist_cmd_pub_;
    ros::Publisher naviga_mode_pub_;
    ros::Publisher path_upload_pub_;

    tf::TransformListener tf_listener;

    ros::Time curr_time;
    ros::Time las_time;

    geometry_msgs::Pose2D laser_to_map;
    geometry_msgs::Pose2D map_to_laser;
    geometry_msgs::Pose2D motion_pose_state;
    geometry_msgs::Pose2D wheel_pose_state;
    geometry_msgs::Pose2D odom_pose_state;
    geometry_msgs::Pose2D reflect_pos;
    geometry_msgs::Pose2D goal_pose;
    geometry_msgs::Twist motion_twist_state;
    geometry_msgs::Twist motion_twist_cmd;
    geometry_msgs::Twist moiton_last_twist;

    mars_msgs::MarsStates mars_state;
    mars_msgs::TaskState task_state;
    mars_msgs::TaskState task_state_detail;
    mars_msgs::GlobalPath mars_path_msg;

    MotionParams motion_params;
    Emath emath;
    ControlParams control_params;
    
    vector<MotionStep> motion_steps;
    vector<double> obstacle_rect_;
    vector<Point> obstacle_points;
    vector<mars_msgs::Infrared> Infrared_buf;

    double omega_error_last;
    bool trace_points_ok;
    bool flag_loctaion;
    bool base_to_curr;
    double odom_jounery;
    double odom_angle;
    int location_states;
    bool stop_flag;
    int closest_point_index;
    int step_index;
    bool ref_flag;
    bool flag_new_paths;

    std::string map_file_path;
    int MapLimits;
    double resolution;
    double mMax;
    float fmax_x;
    float fmax_y;
    int goal_node_id, current_node_id; // 目标点id 当前点id


    Session session;
    SweepAction sweep_action;
    FindWallAction find_wall_action;
    CoverAction cover_action;
    FrameState frame_state;
    ClimbWallAction climb_wall_action;
    double dis_l_f = 0.0, dis_l_b = 0.0, dis_f_r = 0.4, dis_f_l = 0.4, dis_f_min = 0.4;
    vector<double> dist_r_f_datas;
    double pool_x = 3, pool_y = 5;
    double cover_turning_theta = 0.0;
    double pool_dis_h = 0.5; // 横向移动距离
      

    ////////
    double motor_compensation_left;
    double motor_compensation_right;
    double motor_calibration_factor;
    bool is_calibrating;
    ros::Time calibration_start_time;
    
    struct MotorCalibrationData {
        double straight_yaw_start;
        double straight_distance_start;
        double yaw_drift_sum;
        int sample_count;
    } calibration_data;
    
    // 添加服务服务器
    ros::ServiceServer calibrate_service;
    
    // 调试相关
    ros::Time last_debug_time;
    int debug_loop_counter;

    // ===================== [heading-straight-pd] 航向保持走直线模式 =====================
    ros::NodeHandle pnh_{"~"};          // 私有句柄：参数走命令行 _name:=value，免重编
    ros::Subscriber imu_heading_sub_;   // /imu_heading (融合yaw, 度)
    ros::Publisher  heading_cmd_pub_;   // 输出 /cmd_vel（无地板价，区别于 autoVel_cmd）

    bool   heading_mode_ = false;       // control_mode==heading_straight 时 true
    // —— 可调参数（私有句柄读取，含默认）——
    double h_kp_ = 0.15, h_kd_ = 0.0, h_ki_ = 0.0, h_i_clamp_ = 0.05;
    double h_speed_ = 0.1, h_omega_limit_ = 0.1, h_pump_speed_ = 0.0;   // pump_speed 占空比 0~1, 默认0(陆地安全)
    double h_settle_s_ = 20.0, h_still_pp_deg_ = 0.5;
    double h_run_seconds_ = 60.0, h_run_distance_ = 0.0;
    double h_pause_every_s_ = 150.0, h_pause_duration_s_ = 4.0;
    double h_yaw_timeout_ = 0.5, h_deadband_u_ = 0.0;
    int    h_sign_ = 1;
    bool   h_drift_comp_ = false;
    std::string h_yaw_topic_ = "/imu_heading", h_cmd_topic_ = "/cmd_vel";
    std::string h_log_path_ = "/tmp/heading_straight_log.csv";

    // —— 运行态 ——
    HsPhase   hs_phase_ = HS_WAIT_YAW;
    double    hs_yaw_rad_ = 0.0;        // 最新融合yaw（rad），回调写
    ros::Time hs_yaw_stamp_;            // 最新yaw接收时刻
    bool      hs_have_yaw_ = false;
    bool      hs_new_sample_ = false;
    double    hs_yaw0_ = 0.0;           // 去零偏参考航向（rad）
    double    hs_I_ = 0.0;              // 积分累加
    double    hs_d_filt_ = 0.0;         // 滤波后的微分
    double    hs_e_prev_ = 0.0;         // 上个样本的误差
    ros::Time hs_t_prev_loop_, hs_t_prev_sample_;
    ros::Time hs_phase_start_;          // 当前阶段进入时刻
    // STILL 窗口累加（圆均值 + 静止门 + 漂移最小二乘）
    double    hs_sin_sum_ = 0.0, hs_cos_sum_ = 0.0;
    long      hs_still_n_ = 0;
    double    hs_still_first_ = 0.0, hs_still_min_ = 0.0, hs_still_max_ = 0.0;
    bool      hs_still_started_ = false;
    double    hs_ls_t_ = 0.0, hs_ls_y_ = 0.0, hs_ls_tt_ = 0.0, hs_ls_ty_ = 0.0;  // 漂移最小二乘
    // DRIVE 距离/时间锚 + 暂停计时
    double    hs_x0_ = 0.0, hs_y0_ = 0.0;
    ros::Time hs_drive_start_, hs_last_pause_;
    std::ofstream hs_log_;
    bool      hs_log_open_ = false;
};
#endif
