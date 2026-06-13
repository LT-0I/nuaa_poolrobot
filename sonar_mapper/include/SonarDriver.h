#ifndef SONARDRIVER_H
#define SONARDRIVER_H

#include <vector>
#include <cstring>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <AsyncSerial.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <mars_msgs/Pose2DArray.h>

#include <eigen3/Eigen/Dense>  // 使用正确的Eigen包含路径
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#define PI_M 3.1415926535897932384626433832795029

// 声纳点结构
struct SonarPoint {
    float angle;
    std::vector<float> distances;
    std::vector<unsigned char> intensities;
    std::vector<float> usedist;
    std::vector<unsigned char> useintens;
};

typedef struct Msgstruct {
    unsigned char start[2];
    unsigned short len;
    unsigned short cmd;
    unsigned char src;
    unsigned char dst;
    unsigned char data[1300];
    unsigned short checksum;
} MsgStruct;

typedef struct Sonadata_Struct {
    unsigned char start_title[4];
    unsigned char msg_time[28];
    MsgStruct msgs;
} Sonadata;

typedef struct point {
    double distance;
    unsigned char intensity;
} sonapoint;

typedef struct msgcmd2300 {
    unsigned char mode;
    unsigned char gain;
    unsigned short angle;
    unsigned short trans_duration;
    unsigned short samplepriead;
    unsigned short franquency;
    unsigned short numbersample;
    unsigned short length;
    unsigned char data[1200];
} MsgCmd2300;

class SonarDriver
{
public:
    SonarDriver(CallbackAsyncSerial* serial, ros::NodeHandle n);
    virtual ~SonarDriver();  // 析构函数声明
    
    int UnpackData(const char* buf, unsigned int len);
    void receiveMessage(const char data[], unsigned int len);
    void SendMessage();
    void rosNode();

    void AddImuDataMessage(const sensor_msgs::ImuConstPtr &imu_msg);
    void AddOdomEtryMessage(const nav_msgs::Odometry odom_msg);
    
    void Initialize();
    void GrabImageMono(const sensor_msgs::ImageConstPtr &img_msg);
    void Run();

private:
    bool LoadSonarDataFile();
    void Genecmd(unsigned char* buf, unsigned short adata);
    int parsesona(const char* buf, unsigned int len);
    int getdistance(sonapoint* sonadata);
    
    // 可视化函数
    void visualizePoints(const std::vector<Eigen::Vector3f>& points);
    
private:
    // ROS相关
    ros::NodeHandle nh_;
    cv::VideoCapture cap;
    CallbackAsyncSerial* sonar_serial_;

    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher location_state_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher sonar_poses_pub_;

    // 点云存储
    std::vector<Eigen::Vector3f> sonar_points_;
    std::vector<unsigned char> sonar_intensities_;

    // 串口数据缓冲区
    unsigned char rxbuf[2048];
    int rxendindex;
    int rxusedindex;
    unsigned int parsestep;
    unsigned int parret;           // 解析结果
    MsgStruct msg;

    // 声纳数据
    sonapoint sonadataarry[1400];
    sonapoint* psonarray = sonadataarry;
    MsgCmd2300* msgreplay;         // 消息回复指针

    // 角度和图像
    unsigned int angle = 0;
    cv::Mat frame;
    cv::Mat imRectMono;
    ros::Subscriber img_left_sub_;
    image_transport::Publisher pub_img;

    int number = 0;

    // 扫描ID
    int ScanId = 0;
    
    // 当前位姿
    double curodomX = 0.0;
    double curodomY = 0.0; 
    double curTheta = 0.0;

    // 上次位姿 - 只在头文件中声明一次
    double LastodomX = 0.0;
    double LastodomY = 0.0; 
    double LastTheta = 0.0;

    Eigen::Vector3f robotPose_;
    ros::Time last_time;
};

#endif // SONARDRIVER_H