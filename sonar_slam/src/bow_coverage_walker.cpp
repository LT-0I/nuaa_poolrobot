#include "sonar_slam/bow_coverage_walker.h"
#include <tf/transform_datatypes.h>
#include <iostream>
#include <signal.h>
#include <csignal>
#include <locale>

namespace sonar_slam {

BowCoverageWalker::BowCoverageWalker(ros::NodeHandle& nh)
    : nh_(nh)
    , odom_received_(false)
    , bow_state_(BOW_IDLE)
    , find_wall_action_(ROTATION_IN_PLACE)
    , step_counter_(0)
    , turn_count_(0)
    , row_direction_(1)
    , robot_width_(0.5)
    , coverage_length_(6.0)
    , coverage_width_(4.0)
    , step_width_(0.25)
    , max_speed_(0.25)
    , wall_distance_(0.35)
    , state_timeout_(30.0)
    , kp_linear_(0.6)
    , ki_linear_(1.0)
    , kp_angular_(1.0)
    , ki_angular_(1.2)
    , wall_follow_distance_(0.12)   // 默认0.12米
    , wall_follow_enabled_(true)    // 默认启用
{
    // 初始化传感器数据
    dis_l_b_ = dis_l_f_ = dis_f_l_ = dis_f_r_ = dis_f_min_ = 10.0;
    
    // 初始化位姿
    current_pose_.x = current_pose_.y = current_pose_.theta = 0.0;
    
    // 从参数服务器读取配置
    nh_.param("robot_width", robot_width_, 0.5);
    nh_.param("coverage_length", coverage_length_, 6.0);
    nh_.param("coverage_width", coverage_width_, 4.0);
    nh_.param("max_speed", max_speed_, 0.15);
    nh_.param("wall_distance", wall_distance_, 0.35);
    
    step_width_ = robot_width_ * 0.6;
    
    // 创建发布器
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 订阅里程计
    odom_sub_ = nh_.subscribe("/wheel_odom", 100, &BowCoverageWalker::odomCallback, this);
    
    // 订阅传感器
    ultra_data_sub_ = nh_.subscribe("/ultra_data", 100, 
                                    &BowCoverageWalker::ultraDataCallback, this);
    
    // 初始化键盘模拟器（从 robot_walker 复制）
    initKeyboardSimulator();
    
    // 创建 20Hz 定时器（与 robot_walker 一致）
    keyboard_timer_ = nh_.createTimer(ros::Duration(0.05), 
                                      &BowCoverageWalker::keyboardTimerCallback, this);
 
    // 从参数服务器读取
    nh_.param("wall_follow_distance", wall_follow_distance_, 0.12);
    nh_.param("wall_follow_enabled", wall_follow_enabled_, true);
    
    ROS_INFO("沿墙参数: 距离=%.2f米, 启用=%d", wall_follow_distance_, wall_follow_enabled_);                                    
    
    ROS_INFO("========================================");
    ROS_INFO("弓字形全覆盖行走控制器");
    ROS_INFO("  机器人宽度: %.2f m", robot_width_);
    ROS_INFO("  水池长度: %.2f m, 宽度: %.2f m", coverage_length_, coverage_width_);
    ROS_INFO("  平移步长: %.2f m", step_width_);
    ROS_INFO("  最大速度: %.2f m/s", max_speed_);
    ROS_INFO("  20Hz 控制频率（复用 robot_walker 机制）");
    ROS_INFO("========================================");
}

BowCoverageWalker::~BowCoverageWalker() {
    stop();
}

// ========== 键盘模拟器初始化（从 robot_walker 复制） ==========
void BowCoverageWalker::initKeyboardSimulator() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    keyboard_sim_ = KeyboardSimulator();
    ROS_INFO("键盘模拟器初始化完成: 线速度增量=%.3f, 角速度增量=%.3f", 
             keyboard_sim_.linear_inc, keyboard_sim_.angular_inc);
}

// ========== 应用键盘命令（从 robot_walker 复制） ==========
void BowCoverageWalker::applyKeyboardCommand(char key) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    keyboard_sim_.idle_counter = 0;
    keyboard_sim_.key_pressed = true;
    keyboard_sim_.key_press_time = ros::Time::now();
    
    switch (key) {
        case 'w':  // 前进
            keyboard_sim_.target_vx += keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0;
            break;
        case 'x':  // 后退
            keyboard_sim_.target_vx -= keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0;
            break;
        case 'a':  // 左转
            keyboard_sim_.target_angular = keyboard_sim_.angular_inc;
            break;
        case 'd':  // 右转
            keyboard_sim_.target_angular = -keyboard_sim_.angular_inc;
            break;
        case 's':  // 停止
            keyboard_sim_.target_vx = 0.0;
            keyboard_sim_.target_angular = 0.0;
            keyboard_sim_.current_vx = 0.0;
            keyboard_sim_.key_pressed = false;
            break;
        default:
            break;
    }
    
    // 限幅
    if (keyboard_sim_.target_vx > keyboard_sim_.max_linear) 
        keyboard_sim_.target_vx = keyboard_sim_.max_linear;
    if (keyboard_sim_.target_vx < -keyboard_sim_.max_linear) 
        keyboard_sim_.target_vx = -keyboard_sim_.max_linear;
}

// ========== 20Hz 定时器回调（从 robot_walker 复制核心逻辑） ==========
void BowCoverageWalker::keyboardTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    // 更新空闲计数器
    keyboard_sim_.idle_counter++;
    
    // 空闲超时检查
    if (keyboard_sim_.idle_counter > keyboard_sim_.idle_threshold) {
        if (keyboard_sim_.key_pressed) {
            keyboard_sim_.key_pressed = false;
            keyboard_sim_.target_vx = 0.0;
            keyboard_sim_.target_angular = 0.0;
        }
    }
    
    // 速度平滑过渡（关键：避免突变）
    double target_vx = keyboard_sim_.target_vx;
    double current_vx = keyboard_sim_.current_vx;
    
    if (current_vx < target_vx) {
        current_vx += keyboard_sim_.linear_inc;
        if (current_vx > target_vx) current_vx = target_vx;
    } else if (current_vx > target_vx) {
        current_vx -= keyboard_sim_.linear_inc;
        if (current_vx < target_vx) current_vx = target_vx;
    }
    keyboard_sim_.current_vx = current_vx;
    
    // 角速度直接设置
    double current_angular = keyboard_sim_.target_angular;
    
    // 限幅
    if (current_angular > keyboard_sim_.max_angular) 
        current_angular = keyboard_sim_.max_angular;
    if (current_angular < -keyboard_sim_.max_angular) 
        current_angular = -keyboard_sim_.max_angular;
    
    // 发布命令
    geometry_msgs::Twist cmd;
    cmd.linear.x = keyboard_sim_.current_vx;
    cmd.angular.z = current_angular;
    cmd_vel_pub_.publish(cmd);
}

// ========== 设置目标速度（供内部调用） ==========
void BowCoverageWalker::setTargetVelocity(double linear_x, double angular_z) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    keyboard_sim_.target_vx = linear_x;
    keyboard_sim_.target_angular = angular_z;
    keyboard_sim_.current_vx = linear_x;
    keyboard_sim_.key_pressed = true;
    keyboard_sim_.idle_counter = 0;
}

void BowCoverageWalker::publishVelocity(double linear_x, double angular_z) {
    setTargetVelocity(linear_x, angular_z);
}

void BowCoverageWalker::publishZeroVelocity() {
    setTargetVelocity(0.0, 0.0);
}

// ========== 直线行走（基于键盘模拟器机制） ==========
bool BowCoverageWalker::walkStraight(double distance, double max_speed, double timeout) {
    if (distance == 0.0) return true;
    if (!waitForOdom(5.0)) {
        ROS_ERROR("等待里程计超时");
        return false;
    }
    
    // 记录起始位置
    updateCurrentPose();
    geometry_msgs::Pose2D start_pose = current_pose_;
    double target_distance = fabs(distance);
    int direction = (distance > 0) ? 1 : -1;
    double speed = std::min(max_speed, keyboard_sim_.max_linear);
    
    ROS_INFO("直线行走: 距离=%.2f米, 速度=%.2f m/s", distance, speed);
    
    // 设置目标速度
    setTargetVelocity(speed * direction, 0.0);
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);
    
    while (ros::ok() && bow_state_ == BOW_STRAIGHT_GO) {
        updateCurrentPose();
        
        // 计算已走距离
        double traveled = fabs(current_pose_.x - start_pose.x);
        
        // 检查是否到达
        if (traveled >= target_distance - 0.03) {
            publishZeroVelocity();
            ROS_INFO("直线行走完成，实际距离: %.3f 米", traveled);
            return true;
        }
        
        // 检查超时
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("直线行走超时");
            publishZeroVelocity();
            return false;
        }
        
        // 简单的方向纠偏
        double angle_error = normalizeAngle(current_pose_.theta - start_pose.theta);
        double angular_z = -0.5 * angle_error;
        angular_z = std::max(-0.2, std::min(0.2, angular_z));
        setTargetVelocity(speed * direction, angular_z);
        
        rate.sleep();
        ros::spinOnce();
    }
    
    publishZeroVelocity();
    return false;
}

// ========== 原地旋转（基于键盘模拟器机制） ==========
bool BowCoverageWalker::turnInPlace(double target_angle, double max_angular_speed) {
    if (!waitForOdom(3.0)) {
        ROS_ERROR("等待里程计超时");
        return false;
    }
    
    updateCurrentPose();
    double start_theta = current_pose_.theta;
    double target_theta = start_theta + target_angle;
    double max_angular = std::min(max_angular_speed, keyboard_sim_.max_angular);
    
    ROS_INFO("原地旋转: 目标角度=%.1f度", target_angle * 180.0 / M_PI);
    
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(20);
    
    while (ros::ok()) {
        updateCurrentPose();
        double err_theta = normalizeAngle(target_theta - current_pose_.theta);
        
        if (fabs(err_theta) < 0.05) {
            publishZeroVelocity();
            ROS_INFO("旋转完成");
            return true;
        }
        
        if ((ros::Time::now() - start_time).toSec() > 10.0) {
            ROS_ERROR("旋转超时");
            publishZeroVelocity();
            return false;
        }
        
        double angular = std::max(-max_angular, std::min(max_angular, 0.5 * err_theta));
        setTargetVelocity(0.0, angular);
        
        rate.sleep();
        ros::spinOnce();
    }
    
    publishZeroVelocity();
    return false;
}

// ========== 沿边跟踪（参考 MotionController） ==========
void BowCoverageWalker::traceWall(double target_dist, double& pos_err_y, double& pos_err_theta) {
    double wave_dis_f_x = 0.07, wave_dis_b_x = 0.07, wave_dis_y = 0.150;
    
    double ab = wave_dis_f_x + wave_dis_b_x;
    double bc = dis_l_b_ - dis_l_f_;
    double tanA = bc / ab;
    pos_err_theta = atan(tanA);
    
    double numerator = (dis_l_f_ * dis_l_f_ - wave_dis_f_x * wave_dis_f_x) + 
                       (dis_l_b_ * dis_l_b_ - wave_dis_b_x * wave_dis_b_x);
    double denominator = 2 * (wave_dis_f_x + wave_dis_b_x);
    
    if (denominator != 0) {
        double current_dis = wave_dis_y + numerator / denominator;
        pos_err_y = target_dist - current_dis;
    } else {
        pos_err_y = 0.0;
    }
    
    if (!std::isfinite(pos_err_y) || !std::isfinite(pos_err_theta)) {
        pos_err_y = pos_err_theta = 0.0;
    }
}

// ========== 寻找墙 ==========
bool BowCoverageWalker::executeFindWall() {
    static double start_theta = 0.0, end_theta = 0.0, target_theta = 0.0, wall_theta = 0.0;
    static double min_dist = 99999.0;
    
    if (find_wall_action_ == ROTATION_IN_PLACE) {
        if (step_counter_ == 0) {
            start_theta = current_pose_.theta;
            end_theta = start_theta + 2 * M_PI - 0.017;
            min_dist = 99999.0;
            ROS_INFO("开始旋转寻找墙壁...");
        }
        
        double err_theta = normalizeAngle(end_theta - current_pose_.theta);
        
        if (dis_f_min_ < min_dist) {
            min_dist = dis_f_min_;
            target_theta = current_pose_.theta;
        }
        
        if (fabs(err_theta) < 0.05) {
            find_wall_action_ = NEAR_WALL_ROTATION;
            publishZeroVelocity();
            ROS_INFO("找到墙壁，角度: %.1f°, 距离: %.2f m", 
                     target_theta * 180.0 / M_PI, min_dist);
            step_counter_ = 0;
            return true;
        } else {
            double angular = kp_angular_ * tanh(ki_angular_ * err_theta);
            angular = std::max(-0.3, std::min(0.3, angular));
            setTargetVelocity(0.0, angular);
            step_counter_++;
            return false;
        }
    }
    else if (find_wall_action_ == NEAR_WALL_ROTATION) {
        double err_theta = normalizeAngle(target_theta - current_pose_.theta);
        
        if (fabs(err_theta) < 0.05) {
            find_wall_action_ = NEAR_WALL_STRAIGHT;
            publishZeroVelocity();
            step_counter_ = 0;
            return true;
        } else {
            double angular = kp_angular_ * tanh(ki_angular_ * err_theta);
            angular = std::max(-0.3, std::min(0.3, angular));
            setTargetVelocity(0.0, angular);
            return false;
        }
    }
    else if (find_wall_action_ == NEAR_WALL_STRAIGHT) {
        double target_dist = dis_f_min_ - wall_distance_;
        
        if (target_dist < 0.05) {
            find_wall_action_ = ALIGNED_WALL;
            publishZeroVelocity();
            wall_theta = current_pose_.theta + M_PI / 2;
            step_counter_ = 0;
            return true;
        } else {
            double speed = kp_linear_ * tanh(ki_linear_ * target_dist);
            speed = std::max(0.0, std::min(max_speed_, speed));
            setTargetVelocity(speed, 0.0);
            return false;
        }
    }
    else if (find_wall_action_ == ALIGNED_WALL) {
        double err_theta = normalizeAngle(wall_theta - current_pose_.theta);
        
        if (fabs(err_theta) < 0.05) {
            find_wall_action_ = ROTATION_IN_PLACE;
            publishZeroVelocity();
            return true;
        } else {
            double angular = kp_angular_ * tanh(ki_angular_ * err_theta);
            angular = std::max(-0.3, std::min(0.3, angular));
            setTargetVelocity(0.0, angular);
            return false;
        }
    }
    
    return false;
}

// ========== 沿边对齐 ==========
bool BowCoverageWalker::executeAlignToWall() {
    double pos_err_y = 0.0, pos_err_theta = 0.0;
    traceWall(wall_distance_, pos_err_y, pos_err_theta);
    
    double omega_error = 0.6 * pos_err_theta + atan(pos_err_y * 1.2);
    double speed_error = (-1.0 / M_PI) * fabs(pos_err_theta * 3.0) + 1.0;
    if (speed_error < 0.0) speed_error = 0.0;
    
    double target_dist = dis_f_min_ - 0.3;
    if (target_dist < 0.05) target_dist = 0.05;
    
    double speed = kp_linear_ * tanh(ki_linear_ * target_dist) * speed_error;
    double angular = kp_angular_ * tanh(ki_angular_ * omega_error);
    
    speed = std::max(0.0, std::min(max_speed_ * 0.5, speed));
    angular = std::max(-0.3, std::min(0.3, angular));
    
    setTargetVelocity(speed, angular);
    
    static int stable_count = 0;
    if (fabs(pos_err_y) < 0.03 && fabs(pos_err_theta) < 0.03) {
        stable_count++;
        if (stable_count > 50) {
            stable_count = 0;
            ROS_INFO("墙面对齐完成");
            publishZeroVelocity();
            return true;
        }
    } else {
        stable_count = 0;
    }
    
    return false;
}

 
// ========== 修改后的沿墙寻边：先靠近墙，然后沿墙走固定距离 ==========
bool BowCoverageWalker::executeSeekStart() {
    enum SeekStartPhase {
        SEEK_MOVE_TO_WALL,    // 阶段1：靠近右侧墙
        SEEK_FOLLOW_WALL,     // 阶段2：沿墙走固定距离
        SEEK_COMPLETE         // 阶段3：完成
    };
    
    static SeekStartPhase phase = SEEK_MOVE_TO_WALL;
    static double start_x = 0.0, start_y = 0.0;
    static double follow_distance = 1.5;  // 沿墙行走1.5米后认为找到起点
    
    if (step_counter_ == 0) {
        ROS_INFO("=== 沿墙寻找起点 ===");
        phase = SEEK_MOVE_TO_WALL;
        start_x = current_pose_.x;
        start_y = current_pose_.y;
        step_counter_++;
    }
    
    updateCurrentPose();
    
    float right_dist = dis_l_f_;  // 右侧距离
    float front_dist = dis_f_min_;
    float target_dist = wall_follow_distance_;  // 目标距离0.12米
    
    switch (phase) {
        case SEEK_MOVE_TO_WALL:
            // 阶段1：如果离墙太远，先向右平移靠近墙
            if (right_dist > target_dist + 0.1) {
                // 向右平移（原地右转+前进的组合）
                float angular = 0.25f;  // 右转
                float speed = 0.08f;     // 慢速前进
                publishVelocity(speed, angular);
                
                ROS_INFO_THROTTLE(1, "靠近墙壁: 右距=%.2f米, 目标=%.2f米", right_dist, target_dist);
                return false;
            } else {
                // 已经靠近墙，进入沿墙阶段
                ROS_INFO("已靠近墙壁(右距=%.2f米)，开始沿墙行走", right_dist);
                phase = SEEK_FOLLOW_WALL;
                start_x = current_pose_.x;
                start_y = current_pose_.y;
                return false;
            }
            break;
            
        case SEEK_FOLLOW_WALL:
            {
                // 阶段2：沿墙行走固定距离
                double traveled = sqrt(pow(current_pose_.x - start_x, 2) + 
                                      pow(current_pose_.y - start_y, 2));
                
                // 检查完成条件：走了足够距离 OR 碰到角落
                bool distance_reached = (traveled >= follow_distance);
                bool corner_reached = (front_dist < 0.25f);
                
                if (distance_reached || corner_reached) {
                    ROS_INFO("沿墙完成: 行程=%.2f米, 前方距离=%.2f米", traveled, front_dist);
                    publishZeroVelocity();
                    phase = SEEK_COMPLETE;
                    ros::Duration(0.5).sleep();
                    return true;  // 完成寻边
                }
                
                // PID控制保持右侧距离
                float error = right_dist - target_dist;
                float angular = 0.0f;
                
                if (error > 0.05f) {
                    angular = 0.2f;  // 右转靠近
                } else if (error < -0.05f) {
                    angular = -0.2f; // 左转远离
                } else {
                    angular = error * 1.5f;
                    angular = std::max(-0.1f, std::min(0.1f, angular));
                }
                
                float speed = max_speed_ * 0.5f;
                speed = std::max(0.08f, std::min(0.12f, speed));
                
                publishVelocity(speed, angular);
                
                static int debug_count = 0;
                if (debug_count++ % 50 == 0) {
                    ROS_INFO("沿墙寻边: 行程=%.2f/%.2f, 右距=%.2f, 前距=%.2f", 
                             traveled, follow_distance, right_dist, front_dist);
                }
                return false;
            }
            break;
            
        default:
            return true;
    }
    
    return false;
}

// 新增：纯沿墙行走模式（不转弓字形）
bool BowCoverageWalker::executeWallFollowOnly() {
    if (step_counter_ == 0) {
        ROS_INFO("=== 纯沿墙行走模式（保持右侧距离%.2f米）===", wall_follow_distance_);
        step_counter_++;
    }
    
    updateCurrentPose();
    
    float right_dist = dis_l_f_;
    float target_dist = wall_follow_distance_;
    float error = right_dist - target_dist;
    float front_dist = dis_f_min_;
    
    // 速度
    float speed = max_speed_;
    if (front_dist < 0.35f) {
        speed = max_speed_ * 0.4f;  // 前方有障碍减速
    }
    
    // PID控制
    float angular = 0.0f;
    float deadzone = 0.03f;
    
    if (error > deadzone) {
        angular = 0.3f;
    } else if (error < -deadzone) {
        angular = -0.3f;
    } else {
        angular = error * 2.0f;
        angular = std::max(-0.15f, std::min(0.15f, angular));
    }
    
    publishVelocity(speed, angular);
    
    static int debug = 0;
    if (debug++ % 50 == 0) {
        ROS_INFO("沿墙: 目标=%.2f, 实际=%.2f, 速度=%.2f, 角速度=%.2f",
                 target_dist, right_dist, speed, angular);
    }
    
    return false;  // 持续运行
}

// ========== 对齐起点方向 ==========
// 修改 executeAlignStart() 为左转90度
bool BowCoverageWalker::executeAlignStart() {
    if (step_counter_ == 0) {
        ROS_INFO("=== 到达角落，左转90度 ===");
        step_counter_++;
        
        // 先停止
        publishZeroVelocity();
        ros::Duration(0.3).sleep();
        
        // 目标角度：当前角度 + 90度
        start_pose_ = current_pose_;
        target_theta_ = current_pose_.theta + M_PI / 2;  // 左转90度
        return false;
    }
    
    updateCurrentPose();
    
    double err_theta = normalizeAngle(target_theta_ - current_pose_.theta);
    
    if (fabs(err_theta) < 0.05) {
        ROS_INFO("左转90度完成");
        publishZeroVelocity();
        ros::Duration(0.3).sleep();
        return true;
    }
    
    // 旋转
    double angular = 0.3 * err_theta;
    angular = std::max(-0.3, std::min(0.3, angular));
    publishVelocity(0.0, angular);
    
    return false;
}
// ========== 沿边行走 ==========
bool BowCoverageWalker::executeWallFollow() {
    if (step_counter_ == 0) {
        ROS_INFO("=== 沿边行走模式（保持右侧距离0.12米）===");
        step_counter_++;
    }
    
    updateCurrentPose();
    
    float right_dist = dis_l_f_;
    float target_dist = 0.12f;
    float error = right_dist - target_dist;
    float front_dist = dis_f_min_;
    
    // 速度：使用 max_speed_ 参数
    float speed = max_speed_;
    if (front_dist < 0.35f) {
        speed = max_speed_ * 0.4f;
    }
    
    // 控制角速度
    float angular = 0.0f;
    if (error > 0.05f) {
        angular = 0.3f;
    } else if (error < -0.05f) {
        angular = -0.3f;
    } else {
        angular = error * 2.0f;
        angular = std::max(-0.15f, std::min(0.15f, angular));
    }
    
    publishVelocity(speed, angular);
    return false;
}

// ========== 直线前进 ==========
bool BowCoverageWalker::executeStraightGo() {
    if (step_counter_ == 0) {
        start_pose_ = current_pose_;
        
        if (turn_count_ % 2 == 0) {
            row_direction_ = 1;
            target_distance_ = coverage_length_;
            ROS_INFO("第%d行: 向右前进 %.2f 米", turn_count_ + 1, coverage_length_);
        } else {
            row_direction_ = -1;
            target_distance_ = coverage_length_;
            ROS_INFO("第%d行: 向左前进 %.2f 米", turn_count_ + 1, coverage_length_);
        }
        
        step_counter_++;
        
        // 启动直线行走
        if (!walkStraight(target_distance_, max_speed_, 60.0)) {
            ROS_ERROR("直线行走失败");
            return false;
        }
        return false;
    }
    
    updateCurrentPose();
    
    double traveled = fabs(current_pose_.x - start_pose_.x);
    if (traveled >= target_distance_ - 0.05) {
        ROS_INFO("第%d行完成", turn_count_ + 1);
        return true;
    }
    
    if (dis_f_min_ < 0.25) {
        ROS_WARN("前方距离 %.2f m，遇到凹槽边缘，提前结束", dis_f_min_);
        publishZeroVelocity();
        return true;
    }
    
    return false;
}

// ========== 180度掉头 ==========
bool BowCoverageWalker::executeTurn180() {
    if (step_counter_ == 0) {
        turn_start_pose_ = current_pose_;
        ROS_INFO("开始180度掉头...");
        step_counter_++;
        return false;
    }
    
    double target_theta = turn_start_pose_.theta + M_PI;
    double err_theta = normalizeAngle(target_theta - current_pose_.theta);
    
    if (fabs(err_theta) < 0.05) {
        ROS_INFO("180度掉头完成");
        publishZeroVelocity();
        return true;
    } else {
        double angular = kp_angular_ * tanh(ki_angular_ * err_theta);
        angular = std::max(-0.3, std::min(0.3, angular));
        setTargetVelocity(0.0, angular);
        return false;
    }
}

// ========== 横向平移 ==========
bool BowCoverageWalker::executeLateralShift() {
    if (step_counter_ == 0) {
        turn_start_pose_ = current_pose_;
        target_shift_distance_ = step_width_;
        ROS_INFO("横向平移: %.2f 米", step_width_);
        step_counter_++;
        
        // 使用 walkStraight 沿 Y 轴移动
        // 注意：需要先旋转90度，这里简化处理
        if (!turnInPlace(M_PI / 2, 0.3)) {
            ROS_ERROR("横向平移：旋转失败");
            return false;
        }
        return false;
    }
    
    if (step_counter_ == 1) {
        if (!walkStraight(step_width_, max_speed_ * 0.6, 20.0)) {
            ROS_ERROR("横向平移：前进失败");
            return false;
        }
        step_counter_++;
        return false;
    }
    
    if (step_counter_ == 2) {
        if (!turnInPlace(-M_PI / 2, 0.3)) {
            ROS_ERROR("横向平移：回正失败");
            return false;
        }
        step_counter_++;
        return false;
    }
    
    ROS_INFO("横向平移完成");
    turn_count_++;
    
    double total_covered = turn_count_ * step_width_;
    if (total_covered >= coverage_width_) {
        ROS_INFO("========================================");
        ROS_INFO("弓字形全覆盖完成！共 %d 行", turn_count_);
        ROS_INFO("========================================");
        bow_state_ = BOW_FINISHED;
    }
    
    return true;
}
 
// ========== 主循环更新 ==========
void BowCoverageWalker::update() {
    if (bow_state_ == BOW_IDLE || bow_state_ == BOW_FINISHED) return;
    
    updateCurrentPose();
    
    if ((ros::Time::now() - state_start_time_).toSec() > state_timeout_) {
        ROS_ERROR("状态超时");
        stop();
        return;
    }
    
    bool success = false;
    switch (bow_state_) {
        case BOW_SEEK_START:    success = executeSeekStart(); break;
        case BOW_ALIGN_START:   success = executeAlignStart(); break;
        case BOW_FIND_WALL:     success = executeFindWall(); break;
        case BOW_ALIGN_TO_WALL: success = executeAlignToWall(); break;
        case BOW_STRAIGHT_GO:   success = executeStraightGo(); break;
        case BOW_TURN_180:      success = executeTurn180(); break;
        case BOW_LATERAL_SHIFT: success = executeLateralShift(); break;
        default: break;
    }
    
    if (success) {
        state_start_time_ = ros::Time::now();
        step_counter_ = 0;
        
        if (bow_state_ == BOW_SEEK_START) {
            // 寻边完成 → 直接开始弓字形行走，不需要额外对齐
            bow_state_ = BOW_STRAIGHT_GO;
            turn_count_ = 0;
            row_direction_ = 1;
            ROS_INFO("寻边完成，开始弓字形行走");
        }
        else if (bow_state_ == BOW_ALIGN_START) {
            bow_state_ = BOW_STRAIGHT_GO;
            turn_count_ = 0;
            ROS_INFO("ALIGN_START完成，转到STRAIGHT_GO");
        }
        else if (bow_state_ == BOW_FIND_WALL) {
            bow_state_ = BOW_ALIGN_TO_WALL;
        }
        else if (bow_state_ == BOW_ALIGN_TO_WALL) {
            bow_state_ = BOW_STRAIGHT_GO;
            turn_count_ = 0;
        }
        else if (bow_state_ == BOW_STRAIGHT_GO) {
            bow_state_ = BOW_TURN_180;
        }
        else if (bow_state_ == BOW_TURN_180) {
            bow_state_ = BOW_LATERAL_SHIFT;
        }
        else if (bow_state_ == BOW_LATERAL_SHIFT) {
            turn_count_++;
            if (turn_count_ * step_width_ >= coverage_width_) {
                bow_state_ = BOW_FINISHED;
                ROS_INFO("全覆盖完成！");
            } else {
                bow_state_ = BOW_STRAIGHT_GO;
            }
        }
    }
}
// ========== 回调函数 ==========
void BowCoverageWalker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
    odom_received_ = true;
}

void BowCoverageWalker::ultraDataCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (msg->data.size() >= 4) {
        dis_l_b_ = msg->data[3] / 1000.0;
        dis_l_f_ = msg->data[2] / 1000.0;
        double temp_f_l = msg->data[1] / 1000.0;
        double temp_f_r = msg->data[0] / 1000.0;
        
        static int cnt_l = 0, cnt_r = 0;
        if (temp_f_l > 0) cnt_l++; else cnt_l = 0;
        if (temp_f_r > 0) cnt_r++; else cnt_r = 0;
        
        dis_f_l_ = (cnt_l < 3) ? 1.0 : temp_f_l;
        dis_f_r_ = (cnt_r < 3) ? 1.0 : temp_f_r;
        dis_f_min_ = (dis_f_r_ > dis_f_l_) ? dis_f_l_ : dis_f_r_;
    }
}

// ========== 辅助函数 ==========
bool BowCoverageWalker::waitForOdom(double timeout) {
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        if (odom_received_) return true;
        if ((ros::Time::now() - start_time).toSec() > timeout) return false;
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return false;
}

void BowCoverageWalker::updateCurrentPose() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) {
        current_pose_.x = current_odom_.pose.pose.position.x;
        current_pose_.y = current_odom_.pose.pose.position.y;
        current_pose_.theta = calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    }
}

double BowCoverageWalker::getYaw() {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_received_) {
        return calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    }
    return 0.0;
}

double BowCoverageWalker::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double BowCoverageWalker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool BowCoverageWalker::startBowCoverage(float length, float width, float robot_width, float max_speed) {
    if (bow_state_ != BOW_IDLE) {
        ROS_ERROR("弓字形行走正在进行中");
        return false;
    }
    
    coverage_length_ = length;
    coverage_width_ = width;
    robot_width_ = robot_width;
    step_width_ = robot_width_ * 0.6;
    max_speed_ = max_speed;
    
    if (!waitForOdom(5.0)) {
        ROS_ERROR("等待里程计数据超时");
        return false;
    }
    
    resetState();
    
    bool use_seek_start = true;
    nh_.param("use_seek_start", use_seek_start, true);
    
    if (use_seek_start) {
        bow_state_ = BOW_SEEK_START;
        ROS_INFO("沿右边墙寻找右下角起点");
    } else {
        bow_state_ = BOW_FIND_WALL;
        ROS_INFO("原地旋转找墙模式");
    }
    
    state_start_time_ = ros::Time::now();
    
    ROS_INFO("========================================");
    ROS_INFO("开始弓字形全覆盖行走");
    ROS_INFO("  长度: %.2f m, 宽度: %.2f m", coverage_length_, coverage_width_);
    ROS_INFO("  平移步长: %.2f m", step_width_);
    ROS_INFO("========================================");
    
    return true;
}

void BowCoverageWalker::resetState() {
    step_counter_ = 0;
    turn_count_ = 0;
    find_wall_action_ = ROTATION_IN_PLACE;
    publishZeroVelocity();
}

void BowCoverageWalker::stop() {
    bow_state_ = BOW_IDLE;
    publishZeroVelocity();
    ROS_INFO("弓字形行走已停止");
}

std::string BowCoverageWalker::getStatus() const {
    switch (bow_state_) {
        case BOW_IDLE: return "空闲";
        case BOW_SEEK_START: return "寻找起点";
        case BOW_ALIGN_START: return "对齐起点";
        case BOW_FIND_WALL: return "寻找墙面";
        case BOW_ALIGN_TO_WALL: return "对齐墙面";
        case BOW_STRAIGHT_GO: return "直线前进";
        case BOW_TURN_180: return "180度掉头";
        case BOW_LATERAL_SHIFT: return "横向平移";
        case BOW_FINISHED: return "已完成";
        default: return "未知";
    }
}

 

} // namespace sonar_slam

// ========== main 函数 ==========
sonar_slam::BowCoverageWalker* g_bow_walker = nullptr;

void signalHandler(int sig) {
    if (g_bow_walker) g_bow_walker->stop();
    ros::shutdown();
}

int main(int argc, char** argv) {

    // 设置locale
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");

    ros::init(argc, argv, "bow_coverage_walker");
    ros::NodeHandle nh("~");
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    sonar_slam::BowCoverageWalker bow_walker(nh);
    g_bow_walker = &bow_walker;
    
    bool auto_start = false;
    double pool_length = 6.0, pool_width = 4.0, robot_width = 0.5, max_speed = 0.15;
    
    nh.param("auto_start", auto_start, false);
    nh.param("pool_length", pool_length, 6.0);
    nh.param("pool_width", pool_width, 4.0);
    nh.param("robot_width", robot_width, 0.5);
    nh.param("max_speed", max_speed, 0.15);
    
    ROS_INFO("参数: 长度=%.1f, 宽度=%.1f, 机器人宽度=%.2f", pool_length, pool_width, robot_width);
    
    if (auto_start) {
        ros::Duration(2.0).sleep();
        bow_walker.startBowCoverage(pool_length, pool_width, robot_width, max_speed);
    }
    
    ros::Rate rate(20);
    while (ros::ok()) {
        bow_walker.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}