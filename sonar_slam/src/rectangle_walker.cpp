#include "sonar_slam/rectangle_walker.h"
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <signal.h>

namespace sonar_slam {

// 全局指针用于信号处理
RectangleWalker* g_walker = nullptr;

void signalHandler(int sig) {
    if (g_walker) {
        ROS_WARN("收到信号 %d，停止矩形行走", sig);
        g_walker->stop();
    }
    ros::shutdown();
}

RectangleWalker::RectangleWalker(ros::NodeHandle& nh)
    : nh_(nh)
    , current_state_(State::IDLE)
    , walk_sub_state_(WalkSubState::NORMAL)
    , current_side_(0)
    , is_running_(false)
    , valid_sensor_count_(0)
    , odom_distance_(0.0)
    , last_odom_x_(0.0)
    , last_odom_y_(0.0)
    , odom_initialized_(false)
    , odom_received_(false)
    , distance_walked_(0.0)
{
    // 加载参数
    loadParameters();
    
    // 初始化ROS发布器和订阅器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/autoVel_cmd", 1);
    odom_sub_ = nh_.subscribe("/wheelPose_sta", 100, &RectangleWalker::odomCallback, this);
    imu_sub_ = nh_.subscribe("/imu/data", 100, &RectangleWalker::imuCallback, this);
    dist_sub_ = nh_.subscribe("/ultra_data", 100, &RectangleWalker::distDatasCallback, this);
    
    ROS_INFO("========================================");
    ROS_INFO("RectangleWalker 初始化完成");
    ROS_INFO("  行走参数: 边长=%.2f米, 边数=%d, 速度=%.2f米/秒", 
             config_.side_length, config_.sides, config_.walk_speed);
    ROS_INFO("  停留参数: 时间=%.1f秒", config_.wait_time);
    ROS_INFO("  防碰撞: 检测距离=%.2f米, 安全距离=%.2f米, 紧急距离=%.2f米",
             config_.wall_detect_distance, config_.wall_safe_distance, 
             config_.emergency_distance);
    ROS_INFO("  精度参数: 角度容差=%.2f°, 距离容差=%.2f米",
             config_.angle_tolerance * 180.0 / M_PI, config_.distance_tolerance);
    ROS_INFO("========================================");
}

RectangleWalker::~RectangleWalker() {
    stop();
}

void RectangleWalker::loadParameters() {
    // 行走参数
    nh_.param("side_length", config_.side_length, 4.0f);
    nh_.param("sides", config_.sides, 4);
    nh_.param("wait_time", config_.wait_time, 55.0f);
    nh_.param("walk_speed", config_.walk_speed, 0.2f);
    nh_.param("turn_speed", config_.turn_speed, 0.3f);
    
    // 防碰撞参数
    nh_.param("wall_detect_distance", config_.wall_detect_distance, 0.35f);
    nh_.param("wall_safe_distance", config_.wall_safe_distance, 0.20f);
    nh_.param("emergency_distance", config_.emergency_distance, 0.10f);
    nh_.param("wall_approach_speed", config_.wall_approach_speed, 0.08f);
    nh_.param("wall_detect_samples", config_.wall_detect_samples, 3);
    
    // 精度参数
    nh_.param("angle_tolerance", config_.angle_tolerance, 0.0175f);
    nh_.param("distance_tolerance", config_.distance_tolerance, 0.05f);
    nh_.param("timeout_per_side", config_.timeout_per_side, 60.0f);
    nh_.param("timeout_per_turn", config_.timeout_per_turn, 10.0f);
    
    // 打印加载的参数
    ROS_INFO("参数加载完成:");
    ROS_INFO("  side_length = %.2f", config_.side_length);
    ROS_INFO("  sides = %d", config_.sides);
    ROS_INFO("  wait_time = %.1f", config_.wait_time);
    ROS_INFO("  walk_speed = %.2f", config_.walk_speed);
    ROS_INFO("  turn_speed = %.2f", config_.turn_speed);
    ROS_INFO("  wall_detect_distance = %.2f", config_.wall_detect_distance);
    ROS_INFO("  wall_safe_distance = %.2f", config_.wall_safe_distance);
    ROS_INFO("  emergency_distance = %.2f", config_.emergency_distance);
}

bool RectangleWalker::start(float side_length, int sides, float wait_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (is_running_) {
        ROS_WARN("矩形行走已在运行中");
        return false;
    }
    
    // 更新配置
    config_.side_length = side_length;
    config_.sides = sides;
    config_.wait_time = wait_time;
    
    // 重置状态
    current_state_ = State::IDLE;
    current_side_ = 0;
    is_running_ = true;
    distance_walked_ = 0.0;
    front_distance_history_.clear();
    
    // 重置里程计
    resetOdomDistance();
    
    ROS_INFO("========================================");
    ROS_INFO("开始口字型行走");
    ROS_INFO("  边长: %.2f米", side_length);
    ROS_INFO("  边数: %d", sides);
    ROS_INFO("  每点停留: %.1f秒", wait_time);
    ROS_INFO("========================================");
    
    // 启动执行线程
    execution_thread_ = std::thread([this]() {
        bool success = executeRectangleWalk();
        std::lock_guard<std::mutex> lock(mutex_);
        is_running_ = false;
        if (success) {
            current_state_ = State::COMPLETED;
            ROS_INFO("✅ 矩形行走完成");
        } else {
            current_state_ = State::ERROR;
            ROS_ERROR("❌ 矩形行走失败");
        }
    });
    
    return true;
}

void RectangleWalker::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_running_) return;
    
    ROS_WARN("停止矩形行走");
    is_running_ = false;
    publishZeroVelocity();
    
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
}

std::string RectangleWalker::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    switch (current_state_) {
        case State::IDLE: return "空闲";
        case State::WALKING: return "行走中";
        case State::TURNING: return "旋转中";
        case State::WAITING: return "作业中";
        case State::COMPLETED: return "已完成";
        case State::ERROR: return "错误";
        case State::OBSTACLE_AVOID: return "避障中";
        default: return "未知";
    }
}

float RectangleWalker::getProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (config_.sides == 0) return 0.0f;
    
    float progress = (float)(current_side_ - 1) / config_.sides;
    if (current_state_ == State::WALKING && config_.side_length > 0) {
        progress += (distance_walked_ / config_.side_length) / config_.sides;
    }
    
    return std::min(progress * 100.0f, 100.0f);
}

bool RectangleWalker::executeRectangleWalk() {
    // 等待里程计数据
    ros::Time start_wait = ros::Time::now();
    while (ros::ok() && !odom_received_ && 
           (ros::Time::now() - start_wait).toSec() < 5.0) {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    if (!odom_received_) {
        ROS_ERROR("无法获取里程计数据");
        return false;
    }
    
    // 主循环：执行每条边
    for (current_side_ = 1; current_side_ <= config_.sides && is_running_; current_side_++) {
        ROS_INFO("========================================");
        ROS_INFO("开始第 %d/%d 条边", current_side_, config_.sides);
        ROS_INFO("目标距离: %.2f米", config_.side_length);
        ROS_INFO("========================================");
        
        // 1. 行走当前边（带红外防碰撞）
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_state_ = State::WALKING;
            walk_sub_state_ = WalkSubState::NORMAL;
            side_start_time_ = ros::Time::now();
            distance_walked_ = 0.0;
        }
        notifyState("开始行走第" + std::to_string(current_side_) + "条边");
        
        // 重置里程计起始点
        resetOdomDistance();
        side_start_odom_ = 0.0;
        
        if (!walkCurrentSide()) {
            ROS_ERROR("第 %d 条边行走失败", current_side_);
            return false;
        }
        
        ROS_INFO("✅ 第 %d 条边行走完成，实际行走: %.2f米", 
                 current_side_, distance_walked_);
        
        // 2. 如果不是最后一条边，进行转弯和停留
        if (current_side_ < config_.sides && is_running_) {
            // 2.1 停留作业
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_state_ = State::WAITING;
                wait_start_time_ = ros::Time::now();
            }
            notifyState("停留" + std::to_string(config_.wait_time) + "秒进行作业");
            
            if (!waitAtCorner()) {
                ROS_WARN("停留被中断");
                if (!is_running_) return false;
            }
            
            // 2.2 旋转90度
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_state_ = State::TURNING;
                turn_start_time_ = ros::Time::now();
            }
            notifyState("旋转90度");
            
            if (!turnAtCorner()) {
                ROS_ERROR("旋转失败");
                return false;
            }
            
            ROS_INFO("✅ 旋转完成");
            
            // 旋转后短暂停顿
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }
    
    return true;
}

bool RectangleWalker::walkCurrentSide() {
    // 使用精确的里程计位移控制行走
    return walkDistance(config_.side_length, config_.walk_speed, true);
}

bool RectangleWalker::walkDistance(float distance, float speed, bool enable_wall_detection) {
    // 记录起始里程计距离
    double start_odom = getOdomDistance();
    double target_odom = start_odom + fabs(distance);
    double target_heading = current_pose_.theta;
    
    // 方向处理
    float actual_speed = (distance >= 0) ? fabs(speed) : -fabs(speed);
    float abs_distance = fabs(distance);
    
    ROS_INFO("开始行走: 目标=%.2f米, 速度=%.3f米/秒", abs_distance, fabs(actual_speed));
    ROS_INFO("起始里程计: %.3f米", start_odom);
    
    // 重置PID
    heading_pid_.reset();
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);  // 20Hz控制频率
    
    while (ros::ok() && is_running_) {
        updateCurrentPose();
        updateSensorData();
        
        // 1. 红外防碰撞检测（最高优先级）
        if (enable_wall_detection) {
            float front_dist = sensor_data_.front_min;
            
            // 紧急停止
            if (sensor_data_.isEmergency()) {
                ROS_ERROR("🚨 紧急停止！前方距离: %.2f米 < %.2f米", 
                          front_dist, config_.emergency_distance);
                stopMoving();
                return false;
            }
            
            // 墙壁检测
            if (sensor_data_.isWallDetected()) {
                // 连续采样确认
                front_distance_history_.push_back(front_dist);
                if (front_distance_history_.size() > config_.wall_detect_samples) {
                    front_distance_history_.pop_front();
                }
                
                bool wall_confirmed = checkWallConsistently();
                
                if (wall_confirmed && walk_sub_state_ != WalkSubState::AT_WALL) {
                    ROS_WARN("检测到墙壁！距离: %.2f米", front_dist);
                    walk_sub_state_ = WalkSubState::APPROACH_WALL;
                }
            } else {
                // 没有墙壁，清空历史
                if (!front_distance_history_.empty()) {
                    front_distance_history_.clear();
                }
                if (walk_sub_state_ != WalkSubState::NORMAL) {
                    walk_sub_state_ = WalkSubState::NORMAL;
                    ROS_INFO("恢复正常行走");
                }
            }
        }
        
        // 2. 获取当前里程计距离
        double current_odom = getOdomDistance();
        double traveled = current_odom - start_odom;
        
        // 根据子状态调整速度
        float current_speed = actual_speed;
        
        if (walk_sub_state_ == WalkSubState::APPROACH_WALL) {
            // 接近墙壁：减速
            current_speed = (distance >= 0) ? config_.wall_approach_speed : -config_.wall_approach_speed;
            
            // 检查是否到达安全距离
            if (sensor_data_.front_min <= config_.wall_safe_distance) {
                walk_sub_state_ = WalkSubState::AT_WALL;
                ROS_INFO("已到达墙壁安全距离: %.2f米", sensor_data_.front_min);
                stopMoving();
                distance_walked_ = traveled;
                return true;
            }
        } else if (walk_sub_state_ == WalkSubState::AT_WALL) {
            // 已到达墙壁
            stopMoving();
            distance_walked_ = traveled;
            return true;
        }
        
        // 3. 检查是否到达目标距离
        if (traveled >= abs_distance - config_.distance_tolerance) {
            ROS_INFO("到达目标距离: 已走 %.3f/%.2f米", traveled, abs_distance);
            stopMoving();
            distance_walked_ = traveled;
            return true;
        }
        
        // 4. 检查超时
        double timeout = abs_distance / fabs(actual_speed) * 1.5;
        if (timeout < config_.timeout_per_side) timeout = config_.timeout_per_side;
        
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("行走超时 (%.1f秒)", timeout);
            return false;
        }
        
        // 5. 航向纠偏（PID控制）
        double heading_error = normalizeAngle(target_heading - current_pose_.theta);
        double angular_correction = heading_pid_.update(heading_error, ros::Time::now());
        
        // 根据侧向传感器微调（保持与墙壁平行）
        if (sensor_data_.left_front < 5.0 && sensor_data_.left_back < 5.0) {
            double wall_angle = atan2(sensor_data_.left_back - sensor_data_.left_front, 0.14);
            angular_correction += wall_angle * 0.3;
        }
        
        // 6. 发布速度命令
        publishVelocity(current_speed, angular_correction);
        
        // 7. 进度报告
        static ros::Time last_report = ros::Time::now();
        if ((ros::Time::now() - last_report).toSec() > 2.0) {
            float progress = (traveled / abs_distance) * 100.0f;
            ROS_INFO("进度: %.1f%%, 已走 %.2f/%.2f米, 前方: %.2f米, 航向误差: %.2f°",
                     progress, traveled, abs_distance,
                     sensor_data_.front_min, heading_error * 180.0 / M_PI);
            last_report = ros::Time::now();
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    return false;
}

bool RectangleWalker::turnAtCorner() {
    updateCurrentPose();
    float start_theta = current_pose_.theta;
    
    // 右转90度
    float target_theta = normalizeAngle(start_theta - M_PI_2);
    
    ROS_INFO("右转90度: 从 %.1f° 到 %.1f°", 
             start_theta * 180.0 / M_PI, target_theta * 180.0 / M_PI);
    
    bool success = rotateToAngle(target_theta);
    
    if (success) {
        target_heading_ = target_theta;
        publishZeroVelocity();
        ros::Duration(0.3).sleep();
    }
    
    return success;
}

bool RectangleWalker::rotateToAngle(float target_angle) {
    ros::Time start_time = ros::Time::now();
    
    // 旋转PID控制器
    PID turn_pid;
    turn_pid.kp = 1.0;
    turn_pid.ki = 0.0;
    turn_pid.kd = 0.1;
    turn_pid.reset();
    
    while (ros::ok() && is_running_) {
        updateCurrentPose();
        float angle_error = normalizeAngle(target_angle - current_pose_.theta);
        
        // 检查是否到达
        if (fabs(angle_error) < config_.angle_tolerance) {
            ROS_INFO("旋转完成，误差: %.2f°", angle_error * 180.0 / M_PI);
            return true;
        }
        
        // 检查超时
        if ((ros::Time::now() - start_time).toSec() > config_.timeout_per_turn) {
            ROS_ERROR("旋转超时，当前误差: %.2f°", angle_error * 180.0 / M_PI);
            return false;
        }
        
        // PID控制
        float angular_speed = turn_pid.update(angle_error, ros::Time::now());
        
        // 限幅
        if (angular_speed > config_.turn_speed) angular_speed = config_.turn_speed;
        if (angular_speed < -config_.turn_speed) angular_speed = -config_.turn_speed;
        
        // 最小速度确保转动
        if (fabs(angular_speed) < 0.05 && fabs(angle_error) > 0.1) {
            angular_speed = (angle_error > 0) ? 0.05 : -0.05;
        }
        
        publishVelocity(0.0, angular_speed);
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return false;
}

bool RectangleWalker::waitAtCorner() {
    ROS_INFO("开始停留 %.1f 秒", config_.wait_time);
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);
    
    // 确保机器人停止
    publishZeroVelocity();
    ros::Duration(0.2).sleep();
    
    while (ros::ok() && is_running_ && 
           (ros::Time::now() - start_time).toSec() < config_.wait_time) {
        
        double elapsed = (ros::Time::now() - start_time).toSec();
        double remaining = config_.wait_time - elapsed;
        
        // 每10秒输出一次进度
        static int last_progress = -1;
        int current_progress = (int)(elapsed / config_.wait_time * 100);
        if (current_progress % 10 == 0 && current_progress != last_progress) {
            ROS_INFO("作业中: %d%% 完成, 剩余 %.1f 秒", current_progress, remaining);
            last_progress = current_progress;
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    return is_running_;
}

bool RectangleWalker::checkWallConsistently() {
    if (front_distance_history_.size() < config_.wall_detect_samples) {
        return false;
    }
    
    for (float d : front_distance_history_) {
        if (d >= config_.wall_detect_distance) {
            return false;
        }
    }
    return true;
}

double RectangleWalker::getOdomDistance() {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_distance_;
}

void RectangleWalker::updateOdomDistance() {
    if (odom_initialized_) {
        double dx = current_pose_.x - last_odom_x_;
        double dy = current_pose_.y - last_odom_y_;
        double delta = sqrt(dx*dx + dy*dy);
        
        // 限制最大增量，避免跳变
        if (delta < 0.5 && delta >= 0) {
            odom_distance_ += delta;
        }
    }
    
    last_odom_x_ = current_pose_.x;
    last_odom_y_ = current_pose_.y;
    odom_initialized_ = true;
}

void RectangleWalker::resetOdomDistance() {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_distance_ = 0.0;
    odom_initialized_ = false;
}

bool RectangleWalker::stopMoving() {
    publishZeroVelocity();
    return true;
}

void RectangleWalker::publishVelocity(double linear_x, double angular_z) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = 0.0;
    cmd.angular.z = angular_z;
    cmd_vel_pub_.publish(cmd);
}

void RectangleWalker::publishZeroVelocity() {
    geometry_msgs::Twist zero_cmd;
    zero_cmd.linear.x = 0.0;
    zero_cmd.linear.y = 0.0;
    zero_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(zero_cmd);
}

void RectangleWalker::updateCurrentPose() {
    // 在回调中已经更新
}

double RectangleWalker::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double RectangleWalker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void RectangleWalker::updateSensorData() {
    // 传感器数据在回调中更新
}

void RectangleWalker::notifyState(const std::string& info) {
    if (state_callback_) {
        state_callback_(info);
    }
    ROS_INFO("[RectangleWalker] %s", info.c_str());
}

// ========== ROS回调函数 ==========

void RectangleWalker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 更新位姿
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.theta = calculateYawFromQuaternion(msg->pose.pose.orientation);
    
    // 更新里程计位移
    updateOdomDistance();
    odom_received_ = true;
}

void RectangleWalker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // IMU数据可用于辅助航向，暂不使用
}

void RectangleWalker::distDatasCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 根据您的代码格式: [右前, 前左, 前右, 左后, 左前]
    if (msg->data.size() >= 5) {
        float front_left_raw = msg->data[1] / 1000.0f;   // 前左
        float front_right_raw = msg->data[2] / 1000.0f;  // 前右
        float left_back = msg->data[3] / 1000.0f;        // 左后
        float left_front_raw = msg->data[4] / 1000.0f;   // 左前
        
        sensor_data_.update(front_left_raw, front_right_raw, 
                           left_front_raw, left_back);
        
        // 调试输出（可选）
        static int debug_count = 0;
        if (debug_count++ % 100 == 0 && sensor_data_.front_min < 1.0) {
            ROS_DEBUG("红外: 前左=%.2f, 前右=%.2f, 最小=%.2f", 
                      sensor_data_.front_left, sensor_data_.front_right,
                      sensor_data_.front_min);
        }
    }
}

} // namespace sonar_slam

// ========== main函数 ==========
int main(int argc, char** argv) {
    // 设置locale
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ros::init(argc, argv, "rectangle_walker_node");
    ros::NodeHandle nh("~");  // 使用私有节点句柄，支持命令行参数
    
    ROS_INFO("========================================");
    ROS_INFO("RectangleWalker 启动");
    ROS_INFO("========================================");
    
    // 设置信号处理
    signal(SIGINT, sonar_slam::signalHandler);
    signal(SIGTERM, sonar_slam::signalHandler);
    
    // 创建矩形行走控制器
    sonar_slam::RectangleWalker rect_walker(nh);
    sonar_slam::g_walker = &rect_walker;
    
    // 从参数服务器读取配置（支持命令行传入）
    bool auto_start = false;
    float side_length = 4.0f;
    int sides = 4;
    float wait_time = 55.0f;
    
    nh.param("auto_start", auto_start, false);
    nh.param("side_length", side_length, 4.0f);
    nh.param("sides", sides, 4);
    nh.param("wait_time", wait_time, 55.0f);
    
    ROS_INFO("启动参数:");
    ROS_INFO("  auto_start = %d", auto_start);
    ROS_INFO("  side_length = %.2f m", side_length);
    ROS_INFO("  sides = %d", sides);
    ROS_INFO("  wait_time = %.1f s", wait_time);
    
    if (auto_start) {
        ROS_INFO("自动开始矩形行走");
        rect_walker.start(side_length, sides, wait_time);
    } else {
        ROS_INFO("等待启动命令，可通过服务调用启动");
        ROS_INFO("示例: rosservice call /rectangle_walker/start");
    }
    
    // 主循环
    ROS_INFO("========================================");
    ROS_INFO("控制器准备就绪");
    ROS_INFO("  状态: %s", rect_walker.getStatus().c_str());
    ROS_INFO("========================================");
    
    ros::Rate rate(10);
    while (ros::ok()) {
        if (rect_walker.isRunning()) {
            static ros::Time last_report = ros::Time::now();
            if ((ros::Time::now() - last_report).toSec() > 10.0) {
                ROS_INFO("状态: %s, 进度: %.1f%%", 
                         rect_walker.getStatus().c_str(),
                         rect_walker.getProgress());
                last_report = ros::Time::now();
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("RectangleWalker 关闭");
    return 0;
}