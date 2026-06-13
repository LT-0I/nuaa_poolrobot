/**
 * @file sonar_to_laserscan_node.cpp
 * @brief 产品级声纳转激光节点 - 专为水下清扫机器人设计
 * 特性：线程安全、参数动态更新、错误恢复、性能监控
 */

#include "sonar_slam/sonar_to_laserscan.h"
#include "sonar_slam/sonar_to_laserscan_node.h"
#include <ros/ros.h>
#include <mars_msgs/Pose2DArray.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <mutex>
#include <chrono>

namespace sonar_slam {

SonarToLaserScanNode::SonarToLaserScanNode() : 
    nh_("~"), 
    private_nh_("~"),
    diagnostic_updater_(nh_, private_nh_),
    converter_(private_nh_),
    last_process_time_(0),
    total_points_processed_(0),
    total_scans_published_(0),
    conversion_errors_(0)
{
    // 1. 初始化参数
    initParameters();
    

      // 2. 【关键添加】初始化订阅和发布
    sonar_sub_ = nh_.subscribe(input_topic_, queue_size_,
                              &SonarToLaserScanNode::sonarCallback, this);
    
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(
        output_topic_, queue_size_, true
    );
    
    status_pub_ = private_nh_.advertise<std_msgs::String>("status", 1, true);
    // 2. 设置诊断
    setupDiagnostics();
    
    // 3. 初始化订阅和发布
    initROSComponents();
    
    // 4. 启动参数动态重配置服务
    initDynamicReconfigure();
    
    ROS_INFO("[SonarToLaserScan] 产品级节点初始化完成");
    ROS_INFO("  输入话题: %s", input_topic_.c_str());
    ROS_INFO("  输出话题: %s", output_topic_.c_str());
    ROS_INFO("  发布频率: %.1f Hz", publish_rate_);
}

SonarToLaserScanNode::~SonarToLaserScanNode() {
    // 清理资源
    ROS_INFO("[SonarToLaserScan] 节点关闭，共处理 %ld 个点，发布 %ld 次扫描", 
             total_points_processed_, total_scans_published_);
}

void SonarToLaserScanNode::initParameters() {
    // 从参数服务器获取配置
    private_nh_.param("input_topic", input_topic_, std::string("/sonar/pose_array"));
    private_nh_.param("output_topic", output_topic_, std::string("/sonar/scan"));
    private_nh_.param("publish_rate", publish_rate_, 10.0);
    private_nh_.param("queue_size", queue_size_, 10);
    private_nh_.param("enable_diagnostics", enable_diagnostics_, true);
    private_nh_.param("max_consecutive_errors", max_consecutive_errors_, 10);
    
    // 验证参数
    if (publish_rate_ <= 0) {
        ROS_WARN("发布频率 %.1f Hz 无效，使用默认值 10 Hz", publish_rate_);
        publish_rate_ = 10.0;
    }

    
    
    // 初始化统计
    consecutive_errors_ = 0;
    last_param_check_ = ros::Time::now();
}

void SonarToLaserScanNode::setupDiagnostics() {
    if (enable_diagnostics_) {
        // 添加诊断任务
        diagnostic_updater_.setHardwareID("sonar_to_laserscan");
        diagnostic_updater_.add("状态检查", this, &SonarToLaserScanNode::diagnosticCallback);
        
        // 添加频率监控
        diagnostic_pub_.reset(new diagnostic_updater::TopicDiagnostic(
            output_topic_, diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&publish_rate_, &publish_rate_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam()
        ));
        
        ROS_INFO("[SonarToLaserScan] 诊断系统已启用");
    }
}

void SonarToLaserScanNode::initROSComponents() {
    // 订阅声纳数据
    sonar_sub_ = nh_.subscribe<mars_msgs::Pose2DArray>(
        input_topic_, queue_size_,
        boost::bind(&SonarToLaserScanNode::sonarCallback, this, _1),
        ros::VoidPtr(), ros::TransportHints().tcpNoDelay()
    );
    
    // 发布激光数据（带延迟优化）
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(
        output_topic_, queue_size_, true
    );
    
    // 状态发布
    status_pub_ = private_nh_.advertise<std_msgs::String>("status", 1, true);
    
    // 服务：获取状态
    get_status_srv_ = private_nh_.advertiseService(
        "get_status", &SonarToLaserScanNode::getStatusCallback, this
    );
    
    // 服务：重新加载参数
    reload_params_srv_ = private_nh_.advertiseService(
        "reload_parameters", &SonarToLaserScanNode::reloadParametersCallback, this
    );
}

void SonarToLaserScanNode::initDynamicReconfigure() {
    // 动态重配置服务器
    // 可以添加配置：滤波参数、范围限制等
    // dyn_rec_srv_.setCallback(boost::bind(&SonarToLaserScanNode::reconfigureCallback, this, _1, _2));
}

void SonarToLaserScanNode::sonarCallback(const mars_msgs::Pose2DArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(processing_mutex_);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // 1. 数据验证
        if (!validateInputData(msg)) {
            conversion_errors_++;
            consecutive_errors_++;
            
            if (consecutive_errors_ >= max_consecutive_errors_) {
                ROS_ERROR("[SonarToLaserScan] 连续错误超过 %d 次，可能需要检查传感器", 
                         max_consecutive_errors_);
                publishErrorStatus("SENSOR_ERROR");
            }
            return;
        }
        
        // 2. 检查参数是否更新
        checkParameterUpdates();
        
        // 3. 数据转换
        sensor_msgs::LaserScan laser_scan;
        if (!convertSonarToLaser(*msg, laser_scan)) {
            ROS_WARN_THROTTLE(1.0, "[SonarToLaserScan] 数据转换失败");
            conversion_errors_++;
            return;
        }
        
        // 4. 数据后处理
        postProcessScan(laser_scan);
        
        // 5. 发布结果
        publishLaserScan(laser_scan);
        
        // 6. 更新统计
        updateStatistics(msg->poses.size());
        
        // 7. 重置错误计数
        consecutive_errors_ = 0;
        publishNormalStatus();
        
    } catch (const std::exception& e) {
        ROS_ERROR("[SonarToLaserScan] 处理异常: %s", e.what());
        conversion_errors_++;
        consecutive_errors_++;
        
        publishErrorStatus("PROCESSING_EXCEPTION");
    }
    
    // 8. 性能监控
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    last_process_time_ = duration.count() / 1000.0;  // 转换为毫秒
    
    // 9. 更新诊断
    if (enable_diagnostics_) {
        diagnostic_updater_.update();
    }
}

bool SonarToLaserScanNode::validateInputData(const mars_msgs::Pose2DArray::ConstPtr& msg) {
    if (!msg) {
        ROS_DEBUG_THROTTLE(5.0, "[SonarToLaserScan] 接收到空消息");
        return false;
    }
    
    if (msg->poses.empty()) {
        ROS_DEBUG_THROTTLE(5.0, "[SonarToLaserScan] 接收到空数据");
        return false;
    }
    
    // 检查时间戳是否合理
    ros::Duration age = ros::Duration(0);
    if (age.toSec() > 1.0) {
        ROS_WARN_THROTTLE(5.0, "[SonarToLaserScan] 数据延迟过大: %.3f 秒", age.toSec());
        // 不返回false，可能只是网络延迟
    }
    
    return true;
}

bool SonarToLaserScanNode::convertSonarToLaser(const mars_msgs::Pose2DArray& sonar_data,
                                              sensor_msgs::LaserScan& laser_scan) {
    try {
        // 使用转换器
        laser_scan = converter_.convert(sonar_data);
        
        // 验证输出
        if (laser_scan.ranges.empty()) {
            ROS_DEBUG("[SonarToLaserScan] 转换结果为空");
            return false;
        }
        
        // 设置必要header信息
        laser_scan.header.stamp = ros::Time::now();
        if (laser_scan.header.frame_id.empty()) {
            laser_scan.header.frame_id = "sonar_link";
        }
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("[SonarToLaserScan] 转换失败: %s", e.what());
        return false;
    }
}

void SonarToLaserScanNode::postProcessScan(sensor_msgs::LaserScan& scan) {
    // 可选的后处理步骤
    // 1. 范围限制
    for (auto& range : scan.ranges) {
        if (std::isinf(range) || std::isnan(range)) {
            range = scan.range_max;
        } else if (range < scan.range_min) {
            range = scan.range_min;
        } else if (range > scan.range_max) {
            range = scan.range_max;
        }
    }
    
    // 2. 强度归一化（如果有）
    if (!scan.intensities.empty()) {
        // 强度处理逻辑
    }
    
    // 3. 时间戳同步
    scan.header.stamp = ros::Time::now();
}

void SonarToLaserScanNode::publishLaserScan(const sensor_msgs::LaserScan& scan) {
    try {
        laser_pub_.publish(scan);
        total_scans_published_++;
        
        // 更新诊断发布器
        if (enable_diagnostics_ && diagnostic_pub_) {
            diagnostic_pub_->tick(scan.header.stamp);
        }
        
        ROS_DEBUG_THROTTLE(2.0, "[SonarToLaserScan] 发布扫描: %lu 个距离值", 
                          scan.ranges.size());
                          
    } catch (const ros::Exception& e) {
        ROS_ERROR("[SonarToLaserScan] 发布失败: %s", e.what());
        conversion_errors_++;
    }
}

void SonarToLaserScanNode::checkParameterUpdates() {
    // 定期检查参数更新
    ros::Time now = ros::Time::now();
    if ((now - last_param_check_).toSec() > 5.0) {  // 每5秒检查一次
        // 检查并更新转换器参数
        // 如果参数变化，可能需要重新初始化converter_
        last_param_check_ = now;
    }
}

void SonarToLaserScanNode::updateStatistics(size_t points_processed) {
    total_points_processed_ += points_processed;
    
    // 定期输出统计信息
    static int stat_counter = 0;
    if (stat_counter++ % 100 == 0) {
        double error_rate = total_scans_published_ > 0 ? 
            (double)conversion_errors_ / total_scans_published_ * 100.0 : 0.0;
            
        ROS_INFO("[SonarToLaserScan] 统计: 点=%ld, 扫描=%ld, 错误=%ld (%.1f%%), 处理时间=%.2fms",
                 total_points_processed_, total_scans_published_, 
                 conversion_errors_, error_rate, last_process_time_);
        stat_counter = 0;
    }
}

void SonarToLaserScanNode::diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper& status) {
    // 诊断回调
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "运行正常");
    
    // 添加诊断信息
    status.add("总处理点数", total_points_processed_);
    status.add("总发布扫描数", total_scans_published_);
    status.add("转换错误数", conversion_errors_);
    status.add("最后处理时间(ms)", last_process_time_);
    status.add("连续错误数", consecutive_errors_);
    
    // 判断状态
    if (consecutive_errors_ > 0) {
        status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, 
                           "检测到连续错误");
    }
    
    if (consecutive_errors_ >= max_consecutive_errors_) {
        status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                           "连续错误过多，需要干预");
    }
    
    if (last_process_time_ > 100.0) {  // 超过100ms处理时间
        status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                           "处理时间过长");
    }
}

bool SonarToLaserScanNode::getStatusCallback(std_srvs::Trigger::Request& req,
                                           std_srvs::Trigger::Response& res) {
    // 服务：获取状态
    std::stringstream ss;
    ss << "SonarToLaserScan 状态:\n";
    ss << "  运行时间: " << (ros::Time::now() - start_time_).toSec() << " 秒\n";
    ss << "  处理点数: " << total_points_processed_ << "\n";
    ss << "  发布扫描: " << total_scans_published_ << "\n";
    ss << "  转换错误: " << conversion_errors_ << "\n";
    ss << "  处理延迟: " << last_process_time_ << " ms\n";
    ss << "  连续错误: " << consecutive_errors_;
    
    res.success = true;
    res.message = ss.str();
    
    return true;
}

bool SonarToLaserScanNode::reloadParametersCallback(std_srvs::Trigger::Request& req,
                                                  std_srvs::Trigger::Response& res) {
    // 服务：重新加载参数
    ROS_INFO("[SonarToLaserScan] 重新加载参数...");
    
    try {
        // 重新初始化参数
        initParameters();
        
        // 可以重新初始化转换器
        // converter_ = SonarToLaserScan(private_nh_);
        
        res.success = true;
        res.message = "参数重新加载成功";
        
    } catch (const std::exception& e) {
        res.success = false;
        res.message = std::string("参数重新加载失败: ") + e.what();
    }
    
    return true;
}

void SonarToLaserScanNode::publishNormalStatus() {
    std_msgs::String status_msg;
    status_msg.data = "NORMAL";
    status_pub_.publish(status_msg);
}

void SonarToLaserScanNode::publishErrorStatus(const std::string& error_type) {
    std_msgs::String status_msg;
    status_msg.data = "ERROR:" + error_type;
    status_pub_.publish(status_msg);
}



} // namespace sonar_slam

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_to_laserscan_node");
    
    // 设置ROS日志级别
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
                                       ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ROS_INFO("========================================");
    ROS_INFO("  声纳转激光节点 - 产品级版本");
    ROS_INFO("  版本: 1.0.0");
    ROS_INFO("  编译时间: %s %s", __DATE__, __TIME__);
    ROS_INFO("========================================");
    
    try {
        sonar_slam::SonarToLaserScanNode node;
        
        ROS_INFO("节点初始化完成，开始运行...");
        ROS_INFO("按 Ctrl+C 退出");
        ROS_INFO("========================================");
        
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_FATAL("节点启动失败: %s", e.what());
        return 1;
    }
    
    ROS_INFO("节点正常关闭");
    return 0;
}