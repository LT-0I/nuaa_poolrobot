#include <ros/ros.h>
#include <sonar_return_home_file_detector/return_home_file_api.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <locale>
#include <codecvt>

int main(int argc, char** argv) {
	std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");  // 确保数字格式正确
    ros::init(argc, argv, "test_detector");
    ros::NodeHandle nh;
    
    // 使用与 multi_position_sonar_mapper.cpp 完全相同的路径逻辑
    std::string home_dir = getenv("HOME");
    std::string log_dir = home_dir + "/sonar_logs_n1/";
    
    // 如果命令行有参数，使用参数指定的路径（用于测试不同路径）
    std::string test_path = log_dir;
    if (argc > 1) {
        test_path = argv[1];
    }
    
    ROS_INFO("========================================");
    ROS_INFO("测试回环检测函数");
    ROS_INFO("默认路径: %s", log_dir.c_str());
    ROS_INFO("测试路径: %s", test_path.c_str());
    ROS_INFO("========================================");
    
    // 记录开始时间
    auto start = std::chrono::steady_clock::now();
    
    // 调用检测函数
    bool result = sonar_return_home_file_detector::detectReturnHomeFromDirectory(test_path);
    
    // 记录结束时间
    auto end = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    auto elapsed_sec = elapsed_ms / 1000.0;
    
    // 输出结果
    ROS_INFO("========================================");
    ROS_INFO("检测结果: %s", result ? "true (检测到回环)" : "false (未检测到)");
    ROS_INFO("耗时: %.3f 秒 (%ld 毫秒)", elapsed_sec, elapsed_ms);
    ROS_INFO("========================================");
    
    return 0;
}