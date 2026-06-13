#include "sonar_slam/adaptive_rectangle_walker.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <deque>
#include <signal.h>

namespace sonar_slam {

// 全局指针
AdaptiveRectangleWalker* g_walker = nullptr;

void signalHandler(int sig) {
    if (g_walker) {
        ROS_WARN("收到信号 %d，停止行走", sig);
        g_walker->stop();
    }
    ros::shutdown();
}

// ==================== PIDController 实现 ====================

PIDController::PIDController() 
    : kp_(0.6), ki_(0.02), kd_(0.1), integral_(0), last_error_(0) {
    last_time_ = ros::Time::now();
}

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), last_error_(0) {
    last_time_ = ros::Time::now();
}

double PIDController::update(double error) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if (dt <= 0) dt = 0.02;
    if (dt > 0.1) dt = 0.02;
    
    integral_ += error * dt;
    if (integral_ > 0.5) integral_ = 0.5;
    if (integral_ < -0.5) integral_ = -0.5;
    
    double derivative = (error - last_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    
    if (output > 0.25) output = 0.25;
    if (output < -0.25) output = -0.25;
    
    last_error_ = error;
    last_time_ = now;
    
    return output;
}

void PIDController::reset() {
    integral_ = 0;
    last_error_ = 0;
    last_time_ = ros::Time::now();
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// ==================== AdaptiveRectangleWalker 实现 ====================

AdaptiveRectangleWalker::AdaptiveRectangleWalker(ros::NodeHandle& nh)
    : nh_(nh)
    , system_state_(SystemState::IDLE)
    , move_sub_state_(MoveSubState::NORMAL)
    , is_running_(false)
    , is_paused_(false)
    , current_waypoint_index_(0)
    , odom_distance_(0.0)
    , last_odom_x_(0.0)
    , last_odom_y_(0.0)
    , odom_initialized_(false)
    , odom_received_(false)
    , distance_walked_(0.0)
    , total_distance_(0.0)
    , total_waypoints_(0)
    , target_heading_(0.0)
{
    loadParameters();
    setupROS();
    
    heading_pid_.setGains(0.6, 0.02, 0.1);
    turn_pid_.setGains(1.0, 0.0, 0.1);
    
    ROS_INFO("========================================");
    ROS_INFO("自适应矩形行走控制器初始化完成");
    ROS_INFO("  设计尺寸: %.1f x %.1f 米", config_.design_width, config_.design_height);
    ROS_INFO("  行走参数: 步长=%.2f米, 速度=%.2f米/秒", config_.step_distance, config_.walk_speed);
    ROS_INFO("  停留时间: %.1f秒/点", config_.wait_time);
    ROS_INFO("========================================");
}

AdaptiveRectangleWalker::~AdaptiveRectangleWalker() {
    stop();
}

void AdaptiveRectangleWalker::loadParameters() {
    nh_.param("width", config_.design_width, 15.0);
    nh_.param("height", config_.design_height, 30.0);
    nh_.param("start_offset", config_.start_offset, 2.0);
    nh_.param("step_distance", config_.step_distance, 4.1);
    nh_.param("turn_threshold", config_.turn_threshold, 4.0);
    nh_.param("wall_safe_distance", config_.wall_safe_distance, 0.3);
    nh_.param("walk_speed", config_.walk_speed, 0.25);
    nh_.param("approach_speed", config_.approach_speed, 0.1);
    nh_.param("turn_speed", config_.turn_speed, 0.3);
    nh_.param("wait_time", config_.wait_time, 55.0);
    nh_.param("timeout_per_side", config_.timeout_per_side, 120.0);
    nh_.param("timeout_per_turn", config_.timeout_per_turn, 15.0);
    nh_.param("angle_tolerance", config_.angle_tolerance, 0.0175);
    nh_.param("distance_tolerance", config_.distance_tolerance, 0.05);
    nh_.param("calibration_interval", config_.calibration_interval, 5.0);
    nh_.param("wall_detect_distance", config_.wall_detect_distance, 0.35);
    nh_.param("emergency_distance", config_.emergency_distance, 0.10);
    nh_.param("wall_detect_samples", config_.wall_detect_samples, 3);
    nh_.param("enable_auto_calibration", config_.enable_auto_calibration, true);
}

void AdaptiveRectangleWalker::setupROS() {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/autoVel_cmd", 1);
    odom_sub_ = nh_.subscribe("/wheelPose_sta", 100, &AdaptiveRectangleWalker::odomCallback, this);
    imu_sub_ = nh_.subscribe("/imu/data", 100, &AdaptiveRectangleWalker::imuCallback, this);
    dist_sub_ = nh_.subscribe("/ultra_data", 100, &AdaptiveRectangleWalker::distDatasCallback, this);
}

bool AdaptiveRectangleWalker::start(double width, double height, double wait_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (is_running_) {
        ROS_WARN("行走控制器已在运行中");
        return false;
    }
    
    config_.design_width = width;
    config_.design_height = height;
    config_.wait_time = wait_time;
    
    actual_measurements_.reset();
    
    system_state_ = SystemState::INITIALIZING;
    move_sub_state_ = MoveSubState::NORMAL;
    is_running_ = true;
    is_paused_ = false;
    distance_walked_ = 0.0;
    corner_positions_.clear();
    front_distance_history_.clear();
    
    resetOdomDistance();
    
    ROS_INFO("========================================");
    ROS_INFO("开始自适应矩形行走");
    ROS_INFO("  设计尺寸: %.1f x %.1f 米", width, height);
    ROS_INFO("  停留时间: %.1f秒/点", wait_time);
    ROS_INFO("========================================");
    
    execution_thread_ = std::thread([this]() {
        bool success = executeRectangleWalk();
        std::lock_guard<std::mutex> lock(mutex_);
        is_running_ = false;
        if (success) {
            system_state_ = SystemState::COMPLETED;
            printMeasurementReport();
            ROS_INFO("✅ 自适应矩形行走完成");
        } else {
            system_state_ = SystemState::ERROR;
            ROS_ERROR("❌ 自适应矩形行走失败");
        }
    });
    
    return true;
}

void AdaptiveRectangleWalker::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_running_) return;
    
    ROS_WARN("停止行走");
    is_running_ = false;
    publishZeroVelocity();
    
    if (execution_thread_.joinable()) {
        execution_thread_.join();
    }
}

void AdaptiveRectangleWalker::pause() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_running_ || is_paused_) return;
    
    is_paused_ = true;
    system_state_ = SystemState::PAUSED;
    publishZeroVelocity();
    ROS_WARN("行走已暂停");
}

void AdaptiveRectangleWalker::resume() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_running_ || !is_paused_) return;
    
    is_paused_ = false;
    system_state_ = SystemState::MOVING;
    ROS_INFO("恢复行走");
}

std::string AdaptiveRectangleWalker::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    switch (system_state_) {
        case SystemState::IDLE: return "空闲";
        case SystemState::INITIALIZING: return "初始化中";
        case SystemState::MOVING: return "移动中";
        case SystemState::WAITING: return "作业中";
        case SystemState::TURNING: return "旋转中";
        case SystemState::CALIBRATING: return "校准中";
        case SystemState::COMPLETED: return "已完成";
        case SystemState::ERROR: return "错误";
        case SystemState::PAUSED: return "已暂停";
        default: return "未知";
    }
}

double AdaptiveRectangleWalker::getProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (total_distance_ <= 0) return 0.0;
    return (distance_walked_ / total_distance_) * 100.0;
}

std::string AdaptiveRectangleWalker::getCurrentWaypoint() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_waypoint_index_ < (int)current_waypoints_.size()) {
        return current_waypoints_[current_waypoint_index_].description;
    }
    return "无";
}

bool AdaptiveRectangleWalker::executeRectangleWalk() {
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
    
    system_state_ = SystemState::MOVING;
    
    std::vector<SideInfo> sides(4);
    
    sides[0].index = 0;
    sides[0].name = "左侧";
    sides[0].design_length = config_.design_height;
    sides[0].is_long_side = true;
    sides[0].is_first_of_type = true;
    
    sides[1].index = 1;
    sides[1].name = "上侧";
    sides[1].design_length = config_.design_width;
    sides[1].is_long_side = false;
    sides[1].is_first_of_type = true;
    
    sides[2].index = 2;
    sides[2].name = "右侧";
    sides[2].design_length = config_.design_height;
    sides[2].is_long_side = true;
    sides[2].is_first_of_type = false;
    
    sides[3].index = 3;
    sides[3].name = "下侧";
    sides[3].design_length = config_.design_width;
    sides[3].is_long_side = false;
    sides[3].is_first_of_type = false;
    
    total_distance_ = 0;
    total_waypoints_ = 0;
    for (const auto& side : sides) {
        total_distance_ += side.design_length;
        auto waypoints = generateWaypoints(side.design_length, side.name, false);
        total_waypoints_ += waypoints.size();
    }
    
    ROS_INFO("总行程: %.1f米, 总停留点: %d个", total_distance_, total_waypoints_);
    
    for (int i = 0; i < 4 && is_running_; i++) {
        current_side_ = sides[i];
        
        ROS_INFO("========================================");
        ROS_INFO("开始第 %d/4 条边: %s", i+1, current_side_.name.c_str());
        ROS_INFO("设计长度: %.2f米", current_side_.design_length);
        ROS_INFO("========================================");
        
        current_waypoints_ = generateWaypoints(current_side_.design_length, 
                                                current_side_.name, true);
        current_waypoint_index_ = 0;
        
        resetOdomDistance();
        updateCurrentPose();
        current_side_.target_heading = current_pose_.theta;
        target_heading_ = current_side_.target_heading;
        heading_pid_.reset();
        
        if (!walkSide(current_side_)) {
            ROS_ERROR("%s 行走失败", current_side_.name.c_str());
            return false;
        }
        
        ROS_INFO("✅ %s 完成", current_side_.name.c_str());
        distance_walked_ += current_side_.actual_length;
        
        if (i < 3 && is_running_) {
            if (!turnAtCorner(current_side_, sides[i+1])) {
                ROS_ERROR("转弯失败");
                return false;
            }
        }
    }
    
    return true;
}

bool AdaptiveRectangleWalker::walkSide(const SideInfo& side) {
    double start_odom = getOdomDistance();
    double last_calibration = 0;
    double actual_length = side.design_length;
    
    MoveSubState local_state = MoveSubState::NORMAL;
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);
    
    ROS_INFO("开始行走 %s: 目标 %.2f米", side.name.c_str(), side.design_length);
    
    while (ros::ok() && is_running_ && !is_paused_) {
        updateCurrentPose();
        updateSensorData();
        
        double current_odom = getOdomDistance();
        double traveled = current_odom - start_odom;
        double remaining = actual_length - traveled;
        
        if (sensor_data_.isEmergency(config_.emergency_distance)) {
            ROS_ERROR("🚨 紧急停止！前方距离: %.2f米", sensor_data_.front_min);
            stopMoving();
            return false;
        }
        
        if (sensor_data_.isWallDetected(config_.wall_detect_distance)) {
            front_distance_history_.push_back(sensor_data_.front_min);
            if (front_distance_history_.size() > (size_t)config_.wall_detect_samples) {
                front_distance_history_.pop_front();
            }
            
            bool wall_confirmed = checkWallConsistently();
            
            if (wall_confirmed && local_state != MoveSubState::AT_WALL) {
                actual_length = traveled + sensor_data_.front_min;
                ROS_INFO("检测到墙壁！实际长度: %.2f米", actual_length);
                saveMeasurement(side.is_long_side, actual_length, side.index);
                local_state = MoveSubState::APPROACHING_WALL;
            }
        } else {
            if (!front_distance_history_.empty()) {
                front_distance_history_.clear();
            }
        }
        
        if (config_.enable_auto_calibration && local_state == MoveSubState::NORMAL) {
            if (traveled - last_calibration >= config_.calibration_interval) {
                double calibrated = calibrateSideLength(traveled, side.is_long_side);
                if (calibrated > 0 && fabs(calibrated - actual_length) > 0.2) {
                    actual_length = calibrated;
                    ROS_INFO("长度校准: %.2f米", actual_length);
                }
                last_calibration = traveled;
            }
        }
        
        if (local_state == MoveSubState::APPROACHING_WALL) {
            double speed = config_.approach_speed;
            
            if (sensor_data_.front_min <= config_.wall_safe_distance) {
                ROS_INFO("到达墙壁安全距离: %.2f米", sensor_data_.front_min);
                stopMoving();
                local_state = MoveSubState::AT_WALL;
                current_side_.actual_length = traveled + sensor_data_.front_min;
                break;
            }
            
            double heading_error = normalizeAngle(target_heading_ - current_pose_.theta);
            double angular = heading_pid_.update(heading_error);
            publishVelocity(speed, angular);
        }
        else if (local_state == MoveSubState::NORMAL) {
            if (traveled >= actual_length - config_.distance_tolerance) {
                ROS_INFO("到达目标距离: 已走 %.3f/%.2f米", traveled, actual_length);
                stopMoving();
                current_side_.actual_length = traveled;
                break;
            }
            
            if (remaining < config_.turn_threshold && remaining > 0) {
                ROS_INFO("剩余 %.2f米 < %.1f米，提前进入转弯准备", remaining, config_.turn_threshold);
                local_state = MoveSubState::APPROACHING_WALL;
                continue;
            }
            
            if (current_waypoint_index_ < (int)current_waypoints_.size()) {
                double next_stop = current_waypoints_[current_waypoint_index_].distance;
                
                if (traveled >= next_stop - config_.distance_tolerance) {
                    ROS_INFO("到达停留点 %s", 
                             current_waypoints_[current_waypoint_index_].description.c_str());
                    stopMoving();
                    
                    system_state_ = SystemState::WAITING;
                    if (!waitAtWaypoint(current_waypoints_[current_waypoint_index_])) {
                        return false;
                    }
                    system_state_ = SystemState::MOVING;
                    current_waypoint_index_++;
                }
            }
            
            double speed = config_.walk_speed;
            if (remaining < 1.0 && remaining > 0) {
                speed = config_.walk_speed * (remaining / 1.0);
                if (speed < 0.05) speed = 0.05;
            }
            
            double heading_error = normalizeAngle(target_heading_ - current_pose_.theta);
            double angular = heading_pid_.update(heading_error);
            publishVelocity(speed, angular);
        }
        else if (local_state == MoveSubState::AT_WALL) {
            break;
        }
        
        if ((ros::Time::now() - start_time).toSec() > config_.timeout_per_side) {
            ROS_ERROR("行走超时");
            return false;
        }
        
        static ros::Time last_report = ros::Time::now();
        if ((ros::Time::now() - last_report).toSec() > 3.0) {
            double progress = (traveled / actual_length) * 100.0;
            ROS_INFO("%s 进度: %.1f%%, 已走 %.2f/%.2f米, 前方: %.2f米",
                     side.name.c_str(), progress, traveled, actual_length,
                     sensor_data_.front_min);
            last_report = ros::Time::now();
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    return true;
}

bool AdaptiveRectangleWalker::turnAtCorner(const SideInfo& current_side, 
                                            const SideInfo& next_side) {
    ROS_INFO("========== 开始转角 (%s -> %s) ==========", 
             current_side.name.c_str(), next_side.name.c_str());
    
    system_state_ = SystemState::TURNING;
    
    double wall_angle = measureWallAngle();
    ROS_INFO("墙壁角度偏差: %.2f°", wall_angle * 180.0 / M_PI);
    
    if (!alignToWall()) {
        ROS_WARN("墙壁对齐失败，继续执行");
    }
    
    if (!backupForTurn(0.3)) {
        ROS_WARN("后退失败");
    }
    
    updateCurrentPose();
    double start_theta = current_pose_.theta;
    double target_theta = normalizeAngle(start_theta - M_PI_2 + wall_angle);
    
    ROS_INFO("旋转: %.1f° -> %.1f°", start_theta * 180.0 / M_PI, target_theta * 180.0 / M_PI);
    
    bool success = rotateToAngle(target_theta);
    
    if (success) {
        target_heading_ = target_theta;
        corner_positions_.push_back(CornerPosition(current_pose_.x, current_pose_.y, current_side_.name));
        if (corner_positions_.size() > 4) corner_positions_.pop_front();
        ROS_INFO("✅ 转角完成");
    } else {
        ROS_ERROR("❌ 转角失败");
    }
    
    system_state_ = SystemState::MOVING;
    return success;
}

bool AdaptiveRectangleWalker::rotateToAngle(double target_angle) {
    ros::Time start_time = ros::Time::now();
    turn_pid_.reset();
    
    while (ros::ok() && is_running_ && !is_paused_) {
        updateCurrentPose();
        double error = normalizeAngle(target_angle - current_pose_.theta);
        
        if (fabs(error) < config_.angle_tolerance) {
            ROS_INFO("旋转完成，误差: %.2f°", error * 180.0 / M_PI);
            publishZeroVelocity();
            return true;
        }
        
        if ((ros::Time::now() - start_time).toSec() > config_.timeout_per_turn) {
            ROS_ERROR("旋转超时，误差: %.2f°", error * 180.0 / M_PI);
            return false;
        }
        
        double angular_speed = turn_pid_.update(error);
        if (angular_speed > config_.turn_speed) angular_speed = config_.turn_speed;
        if (angular_speed < -config_.turn_speed) angular_speed = -config_.turn_speed;
        
        if (fabs(angular_speed) < 0.05 && fabs(error) > 0.1) {
            angular_speed = (error > 0) ? 0.05 : -0.05;
        }
        
        publishVelocity(0.0, angular_speed);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return false;
}

std::vector<AdaptiveRectangleWalker::Waypoint> AdaptiveRectangleWalker::generateWaypoints(
    double side_length, const std::string& side_name, bool use_actual_length) {
    
    std::vector<Waypoint> waypoints;
    
    double actual_length = side_length;
    if (use_actual_length) {
        if (side_name == "左侧" && actual_measurements_.left_side > 0) {
            actual_length = actual_measurements_.left_side;
        } else if (side_name == "上侧" && actual_measurements_.top_side > 0) {
            actual_length = actual_measurements_.top_side;
        } else if (side_name == "右侧" && actual_measurements_.right_side > 0) {
            actual_length = actual_measurements_.right_side;
        } else if (side_name == "下侧" && actual_measurements_.bottom_side > 0) {
            actual_length = actual_measurements_.bottom_side;
        }
    }
    
    double current = config_.start_offset;
    int point_num = 1;
    
    while (current + config_.step_distance < actual_length - config_.turn_threshold) {
        waypoints.emplace_back(point_num, current, side_name, point_num);
        current += config_.step_distance;
        point_num++;
    }
    
    ROS_INFO("%s 生成 %zu 个停留点 (实际长度: %.2f米)", 
             side_name.c_str(), waypoints.size(), actual_length);
    
    return waypoints;
}

bool AdaptiveRectangleWalker::waitAtWaypoint(const Waypoint& wp) {
    ROS_INFO("========================================");
    ROS_INFO("到达 %s, 开始作业 %.1f 秒", wp.description.c_str(), config_.wait_time);
    ROS_INFO("========================================");
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);
    
    while (ros::ok() && is_running_ && !is_paused_ && 
           (ros::Time::now() - start_time).toSec() < config_.wait_time) {
        
        double elapsed = (ros::Time::now() - start_time).toSec();
        double remaining = config_.wait_time - elapsed;
        
        static int last_progress = -1;
        int progress = (int)((elapsed / config_.wait_time) * 100);
        if (progress % 10 == 0 && progress != last_progress) {
            ROS_INFO("%s 作业中: %d%% 完成, 剩余 %.1f 秒", 
                     wp.description.c_str(), progress, remaining);
            last_progress = progress;
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    ROS_INFO("%s 作业完成", wp.description.c_str());
    return true;
}

bool AdaptiveRectangleWalker::walkDistance(double distance, double speed, bool enable_wall_detection) {
    double start_odom = getOdomDistance();
    double abs_distance = fabs(distance);
    double target_heading = current_pose_.theta;
    float actual_speed = (distance >= 0) ? fabs(speed) : -fabs(speed);
    
    ROS_INFO("行走: 目标=%.2f米, 速度=%.3f米/秒", abs_distance, fabs(actual_speed));
    
    heading_pid_.reset();
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);
    
    while (ros::ok() && is_running_ && !is_paused_) {
        updateCurrentPose();
        updateSensorData();
        
        if (enable_wall_detection && sensor_data_.isEmergency(config_.emergency_distance)) {
            ROS_ERROR("🚨 紧急停止！前方距离: %.2f米", sensor_data_.front_min);
            stopMoving();
            return false;
        }
        
        double current_odom = getOdomDistance();
        double traveled = current_odom - start_odom;
        
        if (traveled >= abs_distance - config_.distance_tolerance) {
            ROS_INFO("到达目标距离: 已走 %.3f/%.2f米", traveled, abs_distance);
            stopMoving();
            return true;
        }
        
        if ((ros::Time::now() - start_time).toSec() > abs_distance / fabs(actual_speed) + 10.0) {
            ROS_ERROR("行走超时");
            return false;
        }
        
        double heading_error = normalizeAngle(target_heading - current_pose_.theta);
        double angular = heading_pid_.update(heading_error);
        
        double current_speed = actual_speed;
        double remaining = abs_distance - traveled;
        if (remaining < 0.5 && remaining > 0) {
            current_speed = actual_speed * (remaining / 0.5);
            if (fabs(current_speed) < 0.05) current_speed = (actual_speed > 0) ? 0.05 : -0.05;
        }
        
        publishVelocity(current_speed, angular);
        rate.sleep();
        ros::spinOnce();
    }
    
    return false;
}

bool AdaptiveRectangleWalker::checkWallConsistently() {
    if (front_distance_history_.size() < (size_t)config_.wall_detect_samples) {
        return false;
    }
    
    for (double d : front_distance_history_) {
        if (d >= config_.wall_detect_distance) {
            return false;
        }
    }
    return true;
}

double AdaptiveRectangleWalker::calibrateSideLength(double traveled, bool is_long_side) {
    if (sensor_data_.isWallDetected()) {
        return traveled + sensor_data_.front_min;
    }
    
    if (sensor_data_.left_front < 5.0 && sensor_data_.left_back < 5.0) {
        double angle = atan2(sensor_data_.left_back - sensor_data_.left_front, 0.14);
        if (fabs(angle) < 0.2) {
            double avg_dist = (sensor_data_.left_front + sensor_data_.left_back) / 2;
            return traveled + avg_dist / cos(angle);
        }
    }
    
    return 0;
}

void AdaptiveRectangleWalker::saveMeasurement(bool is_long_side, double actual_length, int side_index) {
    if (is_long_side) {
        if (side_index == 0) {
            actual_measurements_.left_side = actual_length;
        } else {
            actual_measurements_.right_side = actual_length;
        }
    } else {
        if (side_index == 1) {
            actual_measurements_.top_side = actual_length;
        } else {
            actual_measurements_.bottom_side = actual_length;
        }
    }
    
    actual_measurements_.calibration_count++;
}

bool AdaptiveRectangleWalker::alignToWall() {
    ROS_INFO("对齐墙壁...");
    
    std::deque<double> angles;
    ros::Time start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        updateSensorData();
        
        if (sensor_data_.left_front < 5.0 && sensor_data_.left_back < 5.0) {
            double angle = atan2(sensor_data_.left_back - sensor_data_.left_front, 0.14);
            angles.push_back(angle);
            if (angles.size() > 5) angles.pop_front();
            
            double sum = 0;
            for (double a : angles) sum += a;
            double avg_angle = sum / angles.size();
            
            if (fabs(avg_angle) < 0.03) {
                ROS_INFO("墙壁对齐完成");
                return true;
            }
            
            double angular = avg_angle * 0.5;
            if (angular > 0.1) angular = 0.1;
            if (angular < -0.1) angular = -0.1;
            publishVelocity(0.0, angular);
        }
        
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    return false;
}

double AdaptiveRectangleWalker::measureWallAngle() {
    std::deque<double> angles;
    
    for (int i = 0; i < 10 && ros::ok(); i++) {
        updateSensorData();
        
        if (sensor_data_.left_front < 5.0 && sensor_data_.left_back < 5.0) {
            double angle = atan2(sensor_data_.left_back - sensor_data_.left_front, 0.14);
            angles.push_back(angle);
        }
        
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    if (angles.empty()) return 0;
    
    double sum = 0;
    for (double a : angles) sum += a;
    return sum / angles.size();
}

bool AdaptiveRectangleWalker::backupForTurn(double distance) {
    ROS_INFO("后退 %.2f米，为转弯预留空间...", distance);
    
    double start_odom = getOdomDistance();
    
    while (ros::ok() && is_running_) {
        double current_odom = getOdomDistance();
        if (current_odom - start_odom >= distance - 0.05) {
            stopMoving();
            ROS_INFO("后退完成");
            return true;
        }
        
        publishVelocity(-0.1, 0.0);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return false;
}

double AdaptiveRectangleWalker::getOdomDistance() {
    std::lock_guard<std::mutex> lock(mutex_);
    return odom_distance_;
}

void AdaptiveRectangleWalker::updateOdomDistance() {
    if (odom_initialized_) {
        double dx = current_pose_.x - last_odom_x_;
        double dy = current_pose_.y - last_odom_y_;
        double delta = sqrt(dx*dx + dy*dy);
        
        if (delta < 0.5 && delta >= 0) {
            odom_distance_ += delta;
        }
    }
    
    last_odom_x_ = current_pose_.x;
    last_odom_y_ = current_pose_.y;
    odom_initialized_ = true;
}

void AdaptiveRectangleWalker::resetOdomDistance() {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_distance_ = 0.0;
    odom_initialized_ = false;
}

void AdaptiveRectangleWalker::updateSensorData() {
    // 数据在回调中更新
}

void AdaptiveRectangleWalker::updateCurrentPose() {
    // 数据在回调中更新
}

double AdaptiveRectangleWalker::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double AdaptiveRectangleWalker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void AdaptiveRectangleWalker::publishVelocity(double linear_x, double angular_z) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.linear.y = 0.0;
    cmd.angular.z = angular_z;
    cmd_vel_pub_.publish(cmd);
}

void AdaptiveRectangleWalker::publishZeroVelocity() {
    geometry_msgs::Twist zero_cmd;
    zero_cmd.linear.x = 0.0;
    zero_cmd.linear.y = 0.0;
    zero_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(zero_cmd);
}

bool AdaptiveRectangleWalker::stopMoving() {
    publishZeroVelocity();
    return true;
}

void AdaptiveRectangleWalker::printMeasurementReport() {
    ROS_INFO("========================================");
    ROS_INFO("水池实际尺寸测量报告:");
    ROS_INFO("  设计尺寸: %.1f x %.1f 米", config_.design_width, config_.design_height);
    ROS_INFO("----------------------------------------");
    
    if (actual_measurements_.left_side > 0) {
        double error = actual_measurements_.left_side - config_.design_height;
        ROS_INFO("  左侧实际: %.2f米 (偏差: %+.2f米)", actual_measurements_.left_side, error);
    }
    if (actual_measurements_.top_side > 0) {
        double error = actual_measurements_.top_side - config_.design_width;
        ROS_INFO("  上侧实际: %.2f米 (偏差: %+.2f米)", actual_measurements_.top_side, error);
    }
    if (actual_measurements_.right_side > 0) {
        double error = actual_measurements_.right_side - config_.design_height;
        ROS_INFO("  右侧实际: %.2f米 (偏差: %+.2f米)", actual_measurements_.right_side, error);
    }
    if (actual_measurements_.bottom_side > 0) {
        double error = actual_measurements_.bottom_side - config_.design_width;
        ROS_INFO("  下侧实际: %.2f米 (偏差: %+.2f米)", actual_measurements_.bottom_side, error);
    }
    
    ROS_INFO("========================================");
    ROS_INFO("校准次数: %d", actual_measurements_.calibration_count);
    ROS_INFO("========================================");
}

void AdaptiveRectangleWalker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.theta = calculateYawFromQuaternion(msg->pose.pose.orientation);
    updateOdomDistance();
    odom_received_ = true;
}

void AdaptiveRectangleWalker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // IMU数据可用于辅助航向
}

void AdaptiveRectangleWalker::distDatasCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (msg->data.size() >= 5) {
        double front_left_raw = msg->data[1] / 1000.0;
        double front_right_raw = msg->data[2] / 1000.0;
        double left_back = msg->data[3] / 1000.0;
        double left_front_raw = msg->data[4] / 1000.0;
        
        sensor_data_.update(front_left_raw, front_right_raw, left_front_raw, left_back);
    }
}

} // namespace sonar_slam

// ==================== main函数 ====================

int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ros::init(argc, argv, "adaptive_rectangle_walker");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("自适应矩形行走控制器启动");
    ROS_INFO("========================================");
    
    signal(SIGINT, sonar_slam::signalHandler);
    signal(SIGTERM, sonar_slam::signalHandler);
    
    sonar_slam::AdaptiveRectangleWalker walker(nh);
    sonar_slam::g_walker = &walker;
    
    bool auto_start = false;
    double width = 15.0, height = 30.0, wait_time = 55.0;
    
    nh.param("auto_start", auto_start, false);
    nh.param("width", width, 15.0);
    nh.param("height", height, 30.0);
    nh.param("wait_time", wait_time, 55.0);
    
    ROS_INFO("启动参数:");
    ROS_INFO("  auto_start = %d", auto_start);
    ROS_INFO("  width = %.1f m", width);
    ROS_INFO("  height = %.1f m", height);
    ROS_INFO("  wait_time = %.1f s", wait_time);
    
    if (auto_start) {
        walker.start(width, height, wait_time);
    } else {
        ROS_INFO("等待启动命令");
    }
    
    ros::Rate rate(10);
    while (ros::ok()) {
        if (walker.isRunning()) {
            static ros::Time last_report = ros::Time::now();
            if ((ros::Time::now() - last_report).toSec() > 10.0) {
                ROS_INFO("状态: %s, 进度: %.1f%%, 当前: %s",
                         walker.getStatus().c_str(),
                         walker.getProgress(),
                         walker.getCurrentWaypoint().c_str());
                last_report = ros::Time::now();
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("控制器关闭");
    return 0;
}