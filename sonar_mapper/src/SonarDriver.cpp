/**
 * @file ark_reflexxes_controller.cpp
 * LINEAR position velocity acceleration contoller
 *  串口通信：通过 AsyncSerial 与声纳硬件通信
 * 消息解析：解析 msgid=2300 的消息，包含角度、距离、强度数据
 * 坐标系转换：将极坐标转换为笛卡尔坐标
 * 数据发布：发布 /sonar_poses 话题
 * 控制逻辑：发送控制命令改变声纳角度
 * @author 
 * @author 
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <thread>
#include <eigen3/Eigen/Dense>   

#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include "SonarDriver.h"
#include <stdio.h>
// #include <cpuid.h>
using namespace cv;
using namespace std;
using namespace cv::ximgproc;

FILE *csv_file = fopen("/home/orangepi/Documents/sansy_ws/src/bin/aaaa.csv", "w");
SonarDriver::SonarDriver(CallbackAsyncSerial* serial, ros::NodeHandle n):nh_(n)
{
    
    sonar_serial_ = serial;

    memset(rxbuf, 0, 2048);
    rxendindex = 0;
    rxusedindex = 0;
    parsestep = 0;

    unsigned char sendbuf[64];
    Genecmd(sendbuf, angle);
    sonar_serial_->write((char *)sendbuf, 24);

    // mpViewer = new Viewer();
    // mptViewer = new thread(&Viewer::Run, mpViewer);

    // LoadSonarDataFile();

    ScanId = 0;
    curodomX = 0.0;
    curodomY = 0.0; 
    curTheta = 0.0;

    LastodomX = 0.0;
    LastodomY = 0.0; 
    LastTheta = 0.0;

    robotPose_(0) = 0.0;
    robotPose_(1) = 0.0;
    robotPose_(2) = 0.0;
    fprintf(csv_file, "ScanId_%d\n", ScanId);
} 

SonarDriver::~SonarDriver()
{
    ROS_INFO("Destroying SonarDriver...");
    
    // 清理文件句柄
    if (csv_file != nullptr) {
        fclose(csv_file);
        csv_file = nullptr;
        ROS_INFO("CSV file closed");
    }
    
    // 清理串口资源（注意：serial在main函数中管理，这里不要delete）
    if (sonar_serial_ != nullptr) {
        // sonar_serial_->close();  // 如果需要关闭串口
    }
    
    // 清理点云数据
    sonar_points_.clear();
    sonar_intensities_.clear();
    
    ROS_INFO("SonarDriver destroyed");
}

void SonarDriver::rosNode()
{
    imu_sub_ = nh_.subscribe("/imu_data", 100, &SonarDriver::AddImuDataMessage, this);
    odom_sub_ = nh_.subscribe("/wheel_odom", 100, &SonarDriver::AddOdomEtryMessage, this);

    location_state_pub_ = nh_.advertise<std_msgs::Int8>("/location_state", 100);
    robot_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/robot_pose", 100);
    sonar_poses_pub_ = nh_.advertise<mars_msgs::Pose2DArray>("/sonar_poses", 100);

    // std_msgs::Int8 m_state;
    // m_state.data = local_build_.m_LocationState;
    // location_state_pub_.publish(m_state);

    // geometry_msgs::Pose2D robot_pose_stamsg;
    // robot_pose_stamsg.x = local_build_.robotPose_.x;
    // robot_pose_stamsg.y = local_build_.robotPose_.y;
    // robot_pose_stamsg.theta = local_build_.robotPose_.th;
    // robot_pose_pub_.publish(robot_pose_stamsg);
    // publish
    ros::spin();
}

bool SonarDriver::LoadSonarDataFile()
{
    // Viewer viewer;
    // std::thread view_thread(&Viewer::Run,&viewer);

    FILE * fd;
    fd = fopen("/home/oldking/fsb_ws/src/sonar_driver/bin/test2.bin", "rb");

    if (fd <= 0)
    {
        printf("open file error \n");
        return 0;
    }

    long filelength;
    fseek(fd, 0, SEEK_END);
    filelength=ftell(fd);
    fseek(fd, 0, SEEK_SET);

    long i;
    i = 0;
    int j,k,tmpl;
    unsigned int loopcnt;
    size_t readlen;
    unsigned char buf[64];

    loopcnt = 0;
    Sonadata  sonadata;
    MsgStruct *msgs;
    MsgCmd2300 * cmd;
    int timeus;
    double distance;
    int angle_cnt;

    while(1)
    {
        loopcnt++;

        
        for ( tmpl = 0; tmpl < 64; tmpl++)
        {
            readlen = fread(buf+tmpl, 1, 1, fd);
            if (readlen == 0)
                break;
            if (buf[tmpl] == 'B')
                break;
            i = i + readlen;
        }
        if ((tmpl < 64)&&(tmpl>20))
        {
            k = tmpl-1;
            j = 27;
            for (; j >= 0; j--)
            {
                sonadata.msg_time[j] = buf[k];
                k = k - 1;
                if (k == 0)
                    break;
            }
        }
        fseek(fd, -1, SEEK_CUR);
        readlen = fread((unsigned char*)&sonadata.msgs, 1, 8, fd);
        i = i + readlen;
        if (readlen != 8)
            printf("error read sonadata.msgs \n");
        if (sonadata.msgs.start[0] != 'B')
        {
            printf( "msgs start error\n");
            printf (" loopi =%d \n" ,loopcnt);
            //printf("i=%d\n", i);;
            break;
        }
        readlen = fread(sonadata.msgs.data, 1, sonadata.msgs.len, fd);
        i = i + readlen;
        if (readlen != sonadata.msgs.len)
        {
            printf( "error read sonadata.msgs.data \n");
            printf("%d \n", sonadata.msgs.len);
            
            break;
        }
        readlen = fread((unsigned char *)&sonadata.msgs.checksum, 1, 2, fd);
        i = i + readlen;
        if (readlen != 2)
        {
            if (readlen == 0)
            {
                printf("error read sonadata.msgs.checksum \n");
                break;
            }
            else
            {
                readlen = fread((unsigned char*)&sonadata.msgs.checksum+1, 1, 1, fd);
                if (readlen == 0)
                {
                    printf( "error read sonadata.msgs.checksum \n");
                    break;
                }
                i = i + readlen;
            }
        }
        msgs = &sonadata.msgs;
        if (msgs->cmd == 2300 &&loopcnt >3420)
        {
            SonarPoint spoint;
            cmd = (MsgCmd2300*)msgs->data;
            spoint.angle = (cmd->angle*0.9)/180.0*3.1415926;
            //cmd->length;
            printf("angle= %f \n",cmd->angle*0.9);
            cout << "loopcnt " << loopcnt << endl;
            //cmd->trans_duration;
            timeus = cmd->samplepriead * 25 / 1000;
            distance = timeus * 1.45; // mm 
            int use_cnt = 0;
            for (j = 0; j < cmd->length; j++)
            {
                double dist =  (distance * (j + 1))*0.001;
                int inte = (int) cmd->data[j];
                if(inte >100  &&  dist >2.0 && dist < 25.0){
                    if(inte >190 && use_cnt < 2){
                        spoint.usedist.push_back(dist);
                        spoint.useintens.push_back(inte);
                        use_cnt++;
                    }
                    spoint.distances.push_back(dist);  // 修复：改为distances
                    spoint.intensities.push_back(inte); // 修复：改为intensities
                    // cout << dist  << " " << inte << endl;
                }
                // cout << dist  << " " << inte << endl;

            }
            // cout << spoint.usedist.size() << endl;
            // mpViewer->sonarPointCloud.push_back(spoint);
            // cout << "-----------------------------\n" << endl;
            
        }
        
        if (i > filelength - 1)
        break;
        
    }
    fclose(fd);
    return true;
}


void SonarDriver::AddOdomEtryMessage(const nav_msgs::Odometry odom_msg)
{
    geometry_msgs::PoseStamped odometer_position_;
    odometer_position_.pose = odom_msg.pose.pose;
    odometer_position_.header = odom_msg.header;

    curodomX = odometer_position_.pose.position.x;
    curodomY = odometer_position_.pose.position.y;
    // std::cout << " curodomX: " << curodomX << std::endl;
    // std::cout << " curodomY: " << curodomY << std::endl;
    // cout << endl;
}
   
void SonarDriver::AddImuDataMessage(const sensor_msgs::ImuConstPtr &imu_msg)
{
    geometry_msgs::Quaternion quat_msg = imu_msg->orientation;

    // mark 没用到
    double imu_wz = imu_msg->angular_velocity.z;

    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(quat_msg, quat_tf);

    curTheta = tf::getYaw(quat_tf);
    // 将四元数转换为欧拉角（偏航角yaw，翻滚角roll，俯仰角pitch）
    // cout << "imu_wz " << imu_wz << endl;
    // std::cout << "curTheta : " << curTheta   << std::endl;
    // cout << endl;
    // std::cout << "imu_wz: " << imu_wz  << std::endl;
}


int SonarDriver::UnpackData(const char* buf, unsigned int len)
{
    uint8_t recv_data;
    // for(int i=0; i<len; i++)
    // {
    //     recv_data = buf[i];
    //     cout << "recv_data" <<(int) recv_data << endl;
    // }

    unsigned endwhile = 0;
    if (len + rxendindex < 2048)
    {
        memcpy(rxbuf + rxendindex, buf, len);
        rxendindex = rxendindex + len;
    }else{
        memcpy(rxbuf, buf, 2048);
        rxendindex = 2048;
    }
    if (len + rxendindex < 4)
        return -1;
    while (1){
    switch (parsestep)
    {
        case 0:
            if (rxbuf[rxusedindex] == 0x42)
                parsestep = 1;
            break;
        case 1:
            if (rxbuf[rxusedindex] == 0x52)
                parsestep = 2;
            break;
        case 2:
            msg.len = rxbuf[rxusedindex];
            parsestep = 3;
            break;
        case 3:
            msg.len = rxbuf[rxusedindex] * 256 + msg.len;
            parsestep = 4;
            break;
        case 4:
            msg.cmd = rxbuf[rxusedindex];
            parsestep = 5;
            break;
        case 5:
            msg.cmd = rxbuf[rxusedindex] * 256 + msg.cmd;
            // cout << "msg.cmd " << msg.cmd << endl;
            parsestep = 6;
            break;
        case 6:
            // cout << "parsestep1 " << parsestep << endl;
        case 7:
            // cout << "parsestep2 " << parsestep << endl;
            parsestep = parsestep + 1;
            break;
        case 8:
            if ((rxusedindex + msg.len) <= rxendindex)
            {
                if (msg.cmd == 2300)
                {
                    memcpy(msg.data, rxbuf + rxusedindex, msg.len);
                    rxusedindex = rxusedindex + msg.len - 1;
                }
                parsestep = 9;
                // cout << "parsestep " << parsestep << endl;
            }
            else
                endwhile = 1;;
            break;
        case 9:
            parsestep = 10;
            break;
        case 10:
            if (rxusedindex + 1 == rxendindex)
            {
                rxusedindex = 0;
                parsestep = 0;
                rxendindex = 0;
                endwhile = 1;
            }
            else if ((rxusedindex + 1) < rxendindex)
            {
                for (int j = 0; j < rxendindex - (rxusedindex + 1); j++)
                    rxbuf[j] = rxbuf[rxusedindex + 1 + j];
                //  memcpy(rxbuf, rxbuf + rxusedindex + 1, rxendindex - (rxusedindex + 1));
                endwhile = 1;
                rxusedindex = 0;
                parsestep = 0;
                rxendindex = rxendindex - 1 - rxusedindex;
            }
            break;
        }
        if (endwhile)
            break;
        rxusedindex++;
        if (rxusedindex >= rxendindex)
            break;
    }
    return parsestep;
}



void SonarDriver::receiveMessage(const char data[], unsigned int len){
    // ros::Time receive_start_time;
    // receive_start_time = ros::Time::now();
    // double interval_a = (receive_start_time - receive_end_time).toSec();
    // receive_end_time  = receive_start_time;
    if(len > 0){
        parret = UnpackData(data, len);
    }
    // cout << "parret " << parret << endl;
}

void SonarDriver::Genecmd(unsigned char* buf, unsigned short adata)
{
    int i;
    short checksum;
    unsigned char orgcmd[24] = { 0x42, 0x52, 0x0e, 0x00, 0x29, 0x0a, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x6b, 0x00, 0x78, 0x03, 0xee, 0x02, 0xb0, 0x04, 0x01, 0x00, 0x61, 0x03

    };
    for (i = 0; i < 10; i++)
        buf[i] = orgcmd[i];
    buf[10] = adata & 0x00FF;
    buf[11] = (adata >> 8) & 0xFF;
    for (i = 12; i < 22; i++)
        buf[i] = orgcmd[i];
    checksum = 0;
    for (i = 0; i < 22; i++)
    {
        checksum = checksum + buf[i];
    }
    buf[22] = checksum & 0xFF;
    buf[23] = (checksum >> 8) & 0xFF;
}


void
SonarDriver::GrabImageMono(const sensor_msgs::ImageConstPtr &img_msg){
    // cout << "receive left image" << endl;
    cv_bridge::CvImageConstPtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    imRectMono = cv_ptr->image;
    // cv::remap(imRectLeft,imRectLeft,M1l,M2l,cv::INTER_LINEAR);
    // timestamp = img_msg->header.stamp.toSec();
}


int SonarDriver::getdistance(sonapoint* sonadata)
{
    int i;
    double distanceperstep;
    if (msg.cmd != 2300)
        return -1;
    msgreplay = (MsgCmd2300*)msg.data;
    distanceperstep = msgreplay->samplepriead * 0.025 * 1.47;
    for (i = 0; i < msgreplay->numbersample; i++)
    {
        sonadata[i].distance = i * distanceperstep;
        sonadata[i].intensity = msgreplay->data[i];
    }
    
    return msgreplay->angle;

}

void SonarDriver::SendMessage()
{
    uint8_t sendbuf[64];
    unsigned int reangle=0;
    
    robotPose_(2) += curTheta -LastTheta;
    robotPose_(0) += cos(robotPose_(2)) * (curodomX - LastodomX);
    robotPose_(1) += sin(robotPose_(2)) * (curodomX - LastodomX);
    LastodomX = curodomX ;
    LastodomY = curodomY;
    LastTheta = curTheta;

    // cout << "\n"<< endl;
    // cout << "-----------------------------------" << endl;
    // cout << "robot pose " << robotPose_(0) << " " << robotPose_(1)  << " " << robotPose_(2)  << endl;

    if(parret == 0){

        // cout << "START HANDLE ONE ANGLE SONAR DATA" << endl;
        reangle = getdistance(psonarray);

        // cout << "-----------------------------------" << endl;
        // cout << "angle " << angle << endl;
        // cout << "reangle " << reangle << endl;

        SonarPoint spoint;
        spoint.angle = angle*0.9/180.0*3.1415926;
        int use_cnt = 0;
        for (int i = 0; i < 1400; i++)
        {
            double dist =  psonarray[i].distance*0.001;
            int inte = psonarray[i].intensity;
            
            // if(dist > 0.5 && dist < 20.0){
            //     cout << i << " " <<dist  << " " << inte << endl;
            // }
            // if(inte >100 && dist < 40 && dist >2.0 && use_cnt < 5)
            // if(inte >10 && dist < 10 && dist >0.2 && use_cnt < 20)
            // if(dist > 2.0 && inte >30 && use_cnt < 20)
            if(dist > 1.2 && inte >140 && use_cnt < 2)
            {
                // if(inte >190 && use_cnt < 2){
                //     spoint.usedist.push_back(dist);
                //     spoint.useintens.push_back(inte);
                    use_cnt++;
                // }
                // dist = dist * 0.5;
                spoint.distances.push_back(dist);    // 修复：改为distances
                spoint.intensities.push_back(inte);   // 修复：改为intensities
                // cout << i << " " <<dist  << " " << inte << endl;
            }
            // cout << i << " " <<dist  << " " << inte << endl;
        }

        // cout << "spoint.size() " << spoint.distances.size() << endl;
        
        if(spoint.distances.size() > 0 && number == 1) {  // 修复：改为distances
            for(size_t i = 0; i < spoint.distances.size(); i++){  // 修复：改为distances

                Eigen::Vector3f tmpoint;
                double xx = cos(spoint.angle) * spoint.distances[i];  // 修复：改为distances
                double yy = sin(spoint.angle) * spoint.distances[i];  // 修复：改为distances
                float angle_dgree = -45.0/180.0* PI_M;
                tmpoint(0) = cos(angle_dgree) * xx - sin(angle_dgree) * yy ;
                tmpoint(1) = sin(angle_dgree) * xx + cos(angle_dgree) * yy ;
                tmpoint(2) = 0.0;

                Eigen::Vector3f mappoint;
                


                // mpViewer->localCP.mvPoints.push_back(tmpoint);
                // mpViewer->localCP.mvIntensits.push_back(spoint.intensities[i]);  // 修复：改为intensities

                fprintf(csv_file, "angle_%7.3f\n", spoint.angle);
                fprintf(csv_file, "%7.3f", spoint.distances[0]);  // 修复：改为distances
                for (size_t i = 1; i < spoint.distances.size(); ++i){  // 修复：改为distances
                    fprintf(csv_file, ",%7.3f", spoint.distances[i]);  // 修复：改为distances
                }
                // fprintf(csv_file, "\n%d", (int)spoint.intensities[0]);  // 修复：改为intensities
                // for (unsigned i = 1; i <spoint.intensities.size(); i++)  // 修复：改为intensities
                // {
                //     fprintf(csv_file, ",%d", (int)spoint.intensities[i]);  // 修复：改为intensities
                // }
                fprintf(csv_file, "\n");

                // geometry_msgs::Pose2D sonar_pose;
                // sonar_pose.x = xx;
                // sonar_pose.y = yy;
                // sonar_poses.poses.push_back(sonar_pose);
            }
            // mpViewer->sonarPointCloud.push_back(spoint);
            // sonar_poses_pub_.publish(sonar_poses);
        }
        // cout << spoint.usedist.size() << endl;
        


        Genecmd(sendbuf, angle);
        sonar_serial_->write((char *)sendbuf, 24);
        angle=angle+2;
        // cout << "OVER HANDLE" << endl;
    }
    // cout << "angle " << angle << endl;
    // cout << "number " << number << endl;

    // if(angle%100 == 0){

        cout << " angle " << angle << endl;
        // cout << "mpViewer->localCP.mvPoints.size() " << mpViewer->localCP.mvPoints.size() << endl;
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        cout << "dt " << dt << endl;

        mars_msgs::Pose2DArray sonar_poses;
        // for(int i = 0 ; i < mpViewer->localCP.mvPoints.size(); i++){
        //     // cout << " mpViewer->localCP.mvPoints.size() " <<  mpViewer->localCP.mvPoints.size() << endl;
        //     geometry_msgs::Pose2D sonar_pose;
        //     Eigen::Vector3f pos;
        //     pos = mpViewer->localCP.mvPoints[i];
        //     sonar_pose.x = pos(0);
        //     sonar_pose.y = pos(1);
        //     sonar_poses.poses.push_back(sonar_pose);
        // }
        sonar_poses_pub_.publish(sonar_poses);

        robotPose_(0) = 0.0;
        robotPose_(1) = 0.0;
        robotPose_(2) = 0.0;
        // mpViewer->localCP.mvPoints.clear();
        // mpViewer->localCP.mvIntensits.clear();
    // }




    if(angle >= 400 && number == 1){
        angle = 0;
        // number = 0;
        fprintf(csv_file, "robot_pose\n");
        fprintf(csv_file, "%7.3f", robotPose_(0));
        fprintf(csv_file, "%7.3f", robotPose_(1));
        fprintf(csv_file, "%7.3f\n", robotPose_(2));
        fprintf(csv_file, "\n");
        
        ScanId++;
        fprintf(csv_file, "ScanId_%d\n", ScanId);


        // mpViewer->localCP.mvPoints.clear();
        // mpViewer->localCP.mvIntensits.clear();

        // spoint.clear();
        // mpViewer->sonarPointCloud.clear();
        // printf("%f,%d \n", psonarray[0].distance, psonarray->intensity);
    }
}

void SonarDriver::Run()
{
    while(1){
        // cout <<  "number pre " << number << endl;
        std::cin >> number;
        // cout <<  "number now" << number << endl;
        number = 1;
        if (number == 4) {
            break;
        }
    }
}



struct CPUVendorID {
    unsigned int ebx;
    unsigned int edx;
    unsigned int ecx;

    string toString() const {
        return string(reinterpret_cast<const char *>(this), 12);
    }
};

 

// mark Respect Destan        
int main(int argc, char **argv)
{

    ROS_INFO_STREAM("welcome to mars sonar server,please feel free as home!");

	ros::init(argc, argv, "sonar_driver");
    ros::start();

    ros::NodeHandle n;



    // unsigned int level = 0;
    // unsigned int eax = 0;
    // unsigned int ebx;
    // unsigned int ecx;
    // unsigned int edx;

    // __get_cpuid(level, &eax, &ebx, &ecx, &edx);

    // CPUVendorID vendorID { .ebx = ebx, .edx = edx, .ecx = ecx };

    // map<string, string> vendorIdToName;
    // vendorIdToName["GenuineIntel"] = "Intel";
    // vendorIdToName["AuthenticAMD"] = "AMD";
    // vendorIdToName["CyrixInstead"] = "Cyrix";
    // vendorIdToName["CentaurHauls"] = "Centaur";
    // vendorIdToName["SiS SiS SiS "] = "SiS";
    // vendorIdToName["NexGenDriven"] = "NexGen";
    // vendorIdToName["GenuineTMx86"] = "Transmeta";
    // vendorIdToName["RiseRiseRise"] = "Rise";
    // vendorIdToName["UMC UMC UMC "] = "UMC";
    // vendorIdToName["Geode by NSC"] = "National Semiconductor";

    // string vendorIDString = vendorID.toString();

    // auto it = vendorIdToName.find(vendorIDString);
    // string vendorName = (it == vendorIdToName.end()) ? "Unknown" : it->second;

    // cout << "Max instruction ID: " << eax << endl;
    // cout << "Max  ID: " << ecx << endl;
    // cout << "Max  ID: " << edx << endl;



    // cout << "Vendor ID: " << vendorIDString << endl;
    // cout << "Vendor name: " << vendorName << endl;

    // char buf[30];
    // snprintf(buf, sizeof(buf), "%04X-%04X-%04X-%04X-%04X-%04X",
    //     eax>> 16, eax & 0xffff,
    //     edx>> 16, edx & 0xffff,
    //     ecx>> 16, ecx & 0xffff);
    // printf("%s\n", buf);



    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyACM0");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    ROS_INFO_STREAM("port:" << port << " baud:" << baud);

    try {
        CallbackAsyncSerial serial(port,baud);
        SonarDriver sonar_driver(&serial, n);
        serial.setCallback(boost::bind(&SonarDriver::receiveMessage, &sonar_driver, _1, _2));
        boost::thread rosnode_thread(boost::bind(&SonarDriver::rosNode, &sonar_driver));
        std::thread sonar_thread(&SonarDriver::Run,&sonar_driver);
        ros::Duration(1).sleep();

            // int number;
            // std::cin >> number;
            // cout <<  "number " << number << endl;

        ros::Rate r(10);  //发布周期为50hz
        while (ros::ok())
        {
           sonar_driver.SendMessage();
          
          r.sleep();
        }
        quit:
            serial.close();  
    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        ros::shutdown();
        return 1;
    }
    ros::shutdown();
    return 0;

    // SonarDriver sonar_driver(n);
    // std::thread sync_thread(&SonarDriver::Run,&sonar_driver);
    // std::thread view_thread(&Viewer::Run,&viewer);
    // ros::spin();
    // return 0;
}