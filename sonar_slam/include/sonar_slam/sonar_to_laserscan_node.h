#ifndef SONAR_SLAM_SONAR_TO_LASERSCAN_NODE_H
#define SONAR_SLAM_SONAR_TO_LASERSCAN_NODE_H

#include <ros/ros.h>
#include <mars_msgs/Pose2DArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <mutex>
#include <memory>
#include <string>

namespace sonar_slam {

class SonarToLaserScanNode {
public:
    SonarToLaserScanNode();
    ~SonarToLaserScanNode();
    
    // 禁止拷贝和赋值
    SonarToLaserScanNode(const SonarToLaserScanNode&) = delete;
    SonarToLaserScanNode& operator=(const SonarToLaserScanNode&) = delete;
    
private:
    // 初始化函数
    void initParameters();
    void setupDiagnostics();
    void initROSComponents();
    void initDynamicReconfigure();
    
    // 回调函数
    void sonarCallback(const mars_msgs::Pose2DArray::ConstPtr& msg);
    
    // 数据处理函数
    bool validateInputData(const mars_msgs::Pose2DArray::ConstPtr& msg);
    bool convertSonarToLaser(const mars_msgs::Pose2DArray& sonar_data,
                            sensor_msgs::LaserScan& laser_scan);
    void postProcessScan(sensor_msgs::LaserScan& scan);
    void publishLaserScan(const sensor_msgs::LaserScan& scan);
    
    // 参数和状态管理
    void checkParameterUpdates();
    void updateStatistics(size_t points_processed);
    
    // 诊断和服务回调
    void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper& status);
    bool getStatusCallback(std_srvs::Trigger::Request& req,
                          std_srvs::Trigger::Response& res);
    bool reloadParametersCallback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);
    
    // 状态发布
    void publishNormalStatus();
    void publishErrorStatus(const std::string& error_type);
    
    // ROS组件
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sonar_sub_;
    ros::Publisher laser_pub_;
    ros::Publisher status_pub_;
    ros::ServiceServer get_status_srv_;
    ros::ServiceServer reload_params_srv_;
    
    // 诊断
    diagnostic_updater::Updater diagnostic_updater_;
    std::shared_ptr<diagnostic_updater::TopicDiagnostic> diagnostic_pub_;
    
    // 数据处理
    SonarToLaserScan converter_;
    std::mutex processing_mutex_;
    
    // 参数
    std::string input_topic_;
    std::string output_topic_;
    double publish_rate_;
    int queue_size_;
    bool enable_diagnostics_;
    int max_consecutive_errors_;
    
    // 统计和状态
    ros::Time start_time_;
    ros::Time last_param_check_;
    long total_points_processed_;
    long total_scans_published_;
    long conversion_errors_;
    int consecutive_errors_;
    double last_process_time_;  // 毫秒
};

} // namespace sonar_slam

#endif // SONAR_SLAM_SONAR_TO_LASERSCAN_NODE_H