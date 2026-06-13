 
#include <ros/ros.h>
#include "sonar_slam/sonar_data_processor.h"
#include "sonar_slam/static_mapper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar_slam_system");
    
    // 使用全局节点句柄
    ros::NodeHandle nh;
    
    ROS_INFO("Starting Sonar SLAM System...");
    
    // 注意：由于 sonar_data_processor.cpp 和 static_mapper.cpp 已有自己的main函数
    // 这里不能直接创建对象，除非您修改那些文件
    // 建议的方案：
    // 1. 删除 sonar_data_processor.cpp 和 static_mapper.cpp 中的 main 函数
    // 2. 或者，运行独立的节点
    
    ROS_INFO("Sonar SLAM System main node started");
    ROS_INFO("Note: Run sonar_data_processor and static_mapper as separate nodes");
    
    ros::spin();
    
    return 0;
}