#include "sonar_slam/robot_wall_cleaner.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <signal.h>
#include <csignal>
#include <locale>

namespace sonar_slam {

RobotWallCleaner* g_cleaner = nullptr;

void signalHandler(int sig) {
    if (g_cleaner) {
        g_cleaner->stop();
    }
    ros::shutdown();
}

RobotWallCleaner::RobotWallCleaner(ros::NodeHandle& nh) 
    : nh_(nh)
    , odom_received_(false)
    , cleaner_state_(STATE_IDLE)
    , current_pump_speed_(0.0f)
    , ultra_received_(false)
    , right_front_dist_(999.0f)
    , right_rear_dist_(999.0f)
    , infra_front_left_(999.0f)
    , infra_front_right_(999.0f)
    , disabling_timer_(false)
{
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ROS_INFO("========================================");
    ROS_INFO("RobotWallCleaner 初始化（池壁清洗器）");
    ROS_INFO("========================================");
    
    // 初始化ROS接口
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/wheel_odom", 1000, &RobotWallCleaner::odomCallback, this);
    ultra_sub_ = nh_.subscribe("/ultra_data", 100, &RobotWallCleaner::ultraCallback, this);
    
    // 服务
    find_wall_service_ = nh_.advertiseService("/cleaner/find_wall", 
                                              &RobotWallCleaner::findWallServiceCallback, this);
    clean_service_ = nh_.advertiseService("/cleaner/start_cleaning", 
                                          &RobotWallCleaner::cleanServiceCallback, this);
    
    // 20Hz键盘定时器
    keyboard_timer_ = nh_.createTimer(
        ros::Duration(0.05),
        &RobotWallCleaner::keyboardTimerCallback,
        this
    );
    
    // 读取参数
    nh_.param("cleaner_pump_speed", current_pump_speed_, 0.5f);
    
    // 初始化键盘模拟器
    initKeyboardSimulator();
    
    ROS_INFO("ROS接口初始化完成:");
    ROS_INFO("  发布: /cmd_vel");
    ROS_INFO("  订阅: /wheel_odom, /ultra_data");
    ROS_INFO("  服务: /cleaner/find_wall, /cleaner/start_cleaning");
    ROS_INFO("========================================");
}

RobotWallCleaner::~RobotWallCleaner() {
    stop();
    ROS_INFO("RobotWallCleaner 关闭");
}

void RobotWallCleaner::initKeyboardSimulator() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    keyboard_sim_.reset();
}

void RobotWallCleaner::applyKeyboardCommand(char key) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    keyboard_sim_.idle_counter = 0;
    keyboard_sim_.key_pressed = true;
    keyboard_sim_.current_key = key;
    keyboard_sim_.key_press_time = ros::Time::now();
    
    switch (key) {
        case 'w':
            keyboard_sim_.target_vx += keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
        case 'x':
            keyboard_sim_.target_vx -= keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
        case 'a':
            keyboard_sim_.target_angular = keyboard_sim_.angular_inc;
            break;
        case 'd':
            keyboard_sim_.target_angular = -keyboard_sim_.angular_inc;
            break;
        case 's':
            keyboard_sim_.target_vx = 0.0f;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.current_vx = 0.0f;
            keyboard_sim_.current_vy = 0.0f;
            keyboard_sim_.key_pressed = false;
            break;
        default:
            break;
    }
}

void RobotWallCleaner::keyboardTimerCallback(const ros::TimerEvent& event) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    if (disabling_timer_) {
        return;
    }
    
    keyboard_sim_.idle_counter++;
    
    if (keyboard_sim_.idle_counter > keyboard_sim_.idle_threshold) {
        if (keyboard_sim_.key_pressed) {
            keyboard_sim_.key_pressed = false;
            keyboard_sim_.current_key = 0;
            keyboard_sim_.target_vx = 0.0f;
            keyboard_sim_.target_angular = 0.0f;
        }
    }
    
    // 速度平滑
    float diff_vx = keyboard_sim_.target_vx - keyboard_sim_.current_vx;
    keyboard_sim_.current_vx += diff_vx * 0.3f;
    
    float current_angular = keyboard_sim_.target_angular;
    
    // 限幅
    if (keyboard_sim_.current_vx > keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = keyboard_sim_.max_linear;
    }
    if (keyboard_sim_.current_vx < -keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = -keyboard_sim_.max_linear;
    }
    if (current_angular > keyboard_sim_.max_angular) {
        current_angular = keyboard_sim_.max_angular;
    }
    if (current_angular < -keyboard_sim_.max_angular) {
        current_angular = -keyboard_sim_.max_angular;
    }
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = keyboard_sim_.current_vx;
    cmd.linear.y = current_pump_speed_;
    cmd.angular.z = current_angular;
    cmd_vel_pub_.publish(cmd);
}

void RobotWallCleaner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    current_odom_ = *msg;
    odom_received_ = true;
}

void RobotWallCleaner::ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (msg->data.size() >= 4) {
        infra_front_right_ = msg->data[0] / 1000.0f;
        infra_front_left_ = msg->data[1] / 1000.0f;
        right_rear_dist_ = msg->data[2] / 1000.0f;
        right_front_dist_ = msg->data[3] / 1000.0f;
        
        // 过滤异常值
        if (infra_front_right_ > 0.95f || infra_front_right_ < 0.05f) {
            infra_front_right_ = 999.0f;
        }
        if (infra_front_left_ > 0.95f || infra_front_left_ < 0.05f) {
            infra_front_left_ = 999.0f;
        }
        if (right_front_dist_ > 5.0f || right_front_dist_ < 0.05f) {
            right_front_dist_ = 999.0f;
        }
        if (right_rear_dist_ > 5.0f || right_rear_dist_ < 0.05f) {
            right_rear_dist_ = 999.0f;
        }
        
        ultra_received_ = true;
    }
}

void RobotWallCleaner::updateCurrentPose() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (odom_received_) {
        current_pose_.x = current_odom_.pose.pose.position.x;
        current_pose_.y = current_odom_.pose.pose.position.y;
        current_pose_.theta = calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    }
}

float RobotWallCleaner::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

float RobotWallCleaner::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

bool RobotWallCleaner::waitForOdom(float timeout) {
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            if (odom_received_) {
                return true;
            }
        }
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("等待odom超时");
            return false;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    return false;
}

void RobotWallCleaner::stop() {
    applyKeyboardCommand('s');
    ROS_INFO("机器人已停止");
}

void RobotWallCleaner::setPumpSpeed(float speed) {
    current_pump_speed_ = std::max(0.0f, std::min(1.0f, speed));
    ROS_INFO("水泵速度设置为: %.0f%%", current_pump_speed_ * 100);
}

std::string RobotWallCleaner::getStatus() const {
    switch (cleaner_state_) {
        case STATE_IDLE: return "空闲";
        case STATE_FINDING_WALL: return "寻池壁中";
        case STATE_CLEANING: return "清洗中";
        case STATE_ERROR: return "错误";
        default: return "未知";
    }
}

bool RobotWallCleaner::checkFrontWallDetected() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    float left_dist = infra_front_left_;
    float right_dist = infra_front_right_;
    float min_dist = std::min(left_dist, right_dist);
    return (min_dist > 0.08f && min_dist < 0.80f);
}

bool RobotWallCleaner::checkRightWallDetected() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    const float DETECT_THRESHOLD = 1.5f;
    float front = right_front_dist_;
    float rear = right_rear_dist_;
    return (front < DETECT_THRESHOLD && front > 0.1f) ||
           (rear < DETECT_THRESHOLD && rear > 0.1f);
}

bool RobotWallCleaner::isOutOfWater() {
    // TODO: 根据实际IMU数据实现
    // 目前临时使用传感器组合判断
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    bool right_wall_lost = (right_front_dist_ > 5.0f || right_front_dist_ < 0.05f) &&
                           (right_rear_dist_ > 5.0f || right_rear_dist_ < 0.05f);
    bool infra_invalid = (infra_front_left_ > 0.95f || infra_front_left_ < 0.05f) ||
                         (infra_front_right_ > 0.95f || infra_front_right_ < 0.05f);
    
    return (right_wall_lost && infra_invalid);
}

bool RobotWallCleaner::findWallServiceCallback(std_srvs::Trigger::Request& req,
                                                std_srvs::Trigger::Response& res) {
    ROS_INFO("收到寻池壁服务请求");
    bool success = findPoolWall(60.0f);
    res.success = success;
    res.message = success ? "寻池壁成功" : "寻池壁失败";
    return true;
}

bool RobotWallCleaner::cleanServiceCallback(std_srvs::Trigger::Request& req,
                                             std_srvs::Trigger::Response& res) {
    ROS_INFO("收到池壁清洗服务请求");
    bool success = cleanPoolWallLoop(300.0f);
    res.success = success;
    res.message = success ? "清洗完成" : "清洗失败";
    return true;
}

bool RobotWallCleaner::findPoolWall(float timeout) {
    ROS_INFO("========================================");
    ROS_INFO("寻池壁功能启动");
    ROS_INFO("  超时: %.1f秒", timeout);
    ROS_INFO("========================================");
    
    // 如果正在清洗或行走，先停止
    if (cleaner_state_ != STATE_IDLE) {
        ROS_WARN("机器人正在工作中，先停止再寻池壁");
        stop();
        ros::Duration(0.5).sleep();
    }
    
    cleaner_state_ = STATE_FINDING_WALL;
    bool success = executeFindPoolWall(timeout);
    cleaner_state_ = success ? STATE_IDLE : STATE_ERROR;
    
    if (success) {
        ROS_ERROR("✅ 寻池壁成功，机器人已正对池壁");
    } else {
        ROS_ERROR("❌ 寻池壁失败");
    }
    
    return success;
}

bool RobotWallCleaner::executeFindPoolWall(float timeout) {
    // 等待传感器数据
    ros::Time wait_start = ros::Time::now();
    while (ros::ok() && !ultra_received_) {
        if ((ros::Time::now() - wait_start).toSec() > 5.0f) {
            ROS_WARN("等待传感器数据超时");
            return false;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        updateCurrentPose();
    }
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    
    // 先停止，关闭水泵
    stop();
    setPumpSpeed(0.0f);
    ros::Duration(0.3).sleep();
    
    int total_rotated = 0;      // 已累积旋转角度
    int rotate_count = 0;        // 已完成90度旋转次数
    bool turn_started = false;
    float turn_start_yaw = 0.0f;
    ros::Time turn_start_time;
    bool move_started = false;
    
    ros::Time start_time = ros::Time::now();
    
    // 先检查起始位置是否已经检测到池壁
    updateCurrentPose();
    bool front_detected = checkFrontWallDetected();
    bool right_detected = checkRightWallDetected();
    
    if (front_detected) {
        ROS_INFO("起始位置已检测到前方池壁，寻壁成功");
        return true;
    }

    
    ROS_INFO("开始寻池壁，当前角度: %.1f°", current_pose_.theta * 180.0f / M_PI);
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
        ros::spinOnce();
        updateCurrentPose();
        
        front_detected = checkFrontWallDetected();
        right_detected = checkRightWallDetected();
        
        // ========== 检测到池壁 ==========
        if (front_detected || right_detected) {
            // 停止旋转
            stop();
            ros::Duration(0.2).sleep();
            
            // 条件判断A
            if (front_detected) {
                ROS_INFO("红外检测到池壁，寻壁成功");
                return true;
            }
            else if (right_detected && !front_detected) {
                ROS_INFO("右侧超声检测到池壁，开始平行度对齐");
                // 平行度对齐
                ros::Time align_start = ros::Time::now();
                bool aligned = false;
                
                while (ros::ok() && (ros::Time::now() - align_start).toSec() < 8.0f) {
                    ros::spinOnce();
                    updateCurrentPose();
                    
                    float front = right_front_dist_;
                    float rear = right_rear_dist_;
                    
                    if (front > 5.0f || rear > 5.0f) {
                        ROS_WARN("超声波数据无效");
                        break;
                    }
                    
                    float parallel_error = rear - front;
                    
                    if (fabs(parallel_error) < 0.03f) {
                        stop();
                        ROS_INFO("平行度对齐完成");
                        aligned = true;
                        break;
                    }
                    
                    // 连续旋转，不停止
                    geometry_msgs::Twist align_cmd;
                    align_cmd.linear.x = 0.0f;
                    align_cmd.linear.y = 0.0f;
                    if (parallel_error > 0) {
                        align_cmd.angular.z = 0.15f;
                    } else {
                        align_cmd.angular.z = -0.15f;
                    }
                    cmd_vel_pub_.publish(align_cmd);
                    
                    ros::Duration(0.05).sleep();
                }
                
                if (!aligned) {
                    stop();
                    ROS_WARN("平行度对齐超时");
                }
                
                ros::Duration(0.3).sleep();
                
                // 继续右转90度
                ROS_INFO("继续右转90度");
                disabling_timer_ = true;
                
                float target_yaw = normalizeAngle(current_pose_.theta - M_PI / 2.0f);
                ros::Time turn_start = ros::Time::now();
                
                while (ros::ok()) {
                    ros::spinOnce();
                    updateCurrentPose();
                    
                    float yaw_error = normalizeAngle(current_pose_.theta - target_yaw);
                    if (fabs(yaw_error) < 5.0f * M_PI / 180.0f ||
                        (ros::Time::now() - turn_start).toSec() > 10.0f) {
                        stop();
                        ROS_INFO("右转完成");
                        break;
                    }
                    
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = 0.0f;
                    cmd.linear.y = 0.0f;
                    cmd.angular.z = -0.25f;
                    cmd_vel_pub_.publish(cmd);
                    ros::Duration(0.05).sleep();
                }
                
                disabling_timer_ = false;
                ros::Duration(0.3).sleep();
                
                // 直行靠近池壁
                ROS_INFO("直行靠近池壁");
                {
                    std::lock_guard<std::mutex> lock(keyboard_mutex_);
                    keyboard_sim_.target_vx = 0.15f;
                    keyboard_sim_.current_vx = 0.15f;
                    keyboard_sim_.target_angular = 0.0f;
                    keyboard_sim_.key_pressed = true;
                }
                
                ros::Time approach_start = ros::Time::now();
                bool front_triggered = false;
                
                while (ros::ok() && (ros::Time::now() - approach_start).toSec() < 15.0f) {
                    ros::spinOnce();
                    updateCurrentPose();
                    if (checkFrontWallDetected()) {
                        front_triggered = true;
                        break;
                    }
                    ros::Duration(0.05).sleep();
                }
                
                stop();
                
                if (front_triggered) {
                    ROS_INFO("红外触发，寻壁成功");
                    return true;
                } else {
                    ROS_WARN("红外未触发");
                    return false;
                }
            }
        }
        
        // ========== 未检测到池壁，执行旋转扫描 ==========
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_start_time = ros::Time::now();
            turn_started = true;
            ROS_INFO("开始右转90度扫描，当前角度: %.1f°", turn_start_yaw * 180.0f / M_PI);
        }
        
        // 计算已旋转角度
        float yaw_change = normalizeAngle(turn_start_yaw - current_pose_.theta);
        float abs_yaw_change = fabs(yaw_change);
        
        // 执行旋转
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.0f;
        cmd.angular.z = -0.25f;
        cmd_vel_pub_.publish(cmd);
        
        // 检查是否完成90度旋转
        if (abs_yaw_change >= 85.0f * DEG_TO_RAD ||
            (ros::Time::now() - turn_start_time).toSec() > 15.0f) {
            stop();
            turn_started = false;
            rotate_count++;
            total_rotated += 90;
            ROS_INFO("完成第%d次90度旋转，累积=%d°", rotate_count, total_rotated);
            ros::Duration(0.3).sleep();
            
            // 旋转到位后，再次检测池壁
            ros::spinOnce();
            updateCurrentPose();
            front_detected = checkFrontWallDetected();
            right_detected = checkRightWallDetected();
            
            if (front_detected || right_detected) {
                ROS_INFO("旋转到位后检测到池壁");
                continue;
            }
            
            // 检查是否累积旋转360度
            if (total_rotated >= 360) {
                ROS_INFO("累积旋转360度未找到池壁，前进1米");
                
                // ========== 前进1米 ==========
                {
                    std::lock_guard<std::mutex> lock(keyboard_mutex_);
                    keyboard_sim_.target_vx = 0.2f;
                    keyboard_sim_.current_vx = 0.2f;
                    keyboard_sim_.target_angular = 0.0f;
                    keyboard_sim_.key_pressed = true;
                }
                
                // 记录起始位置
                updateCurrentPose();
                geometry_msgs::Pose2D forward_start_pose = current_pose_;
                ros::Time forward_start_time = ros::Time::now();
                bool forward_wall_found = false;
                
                while (ros::ok() && (ros::Time::now() - forward_start_time).toSec() < 10.0f) {
                    ros::spinOnce();
                    updateCurrentPose();
                    
                    // 计算已前进距离
                    float dx = current_pose_.x - forward_start_pose.x;
                    float dy = current_pose_.y - forward_start_pose.y;
                    float traveled = sqrt(dx*dx + dy*dy);
                    
                    // 前进过程中持续检测池壁
                    front_detected = checkFrontWallDetected();
                    right_detected = checkRightWallDetected();
                    
                    if (front_detected || right_detected) {
                        stop();
                        ROS_INFO("前进1米中检测到池壁: %s, 前进距离=%.2f米", 
                                 front_detected ? "前方" : "右侧", traveled);
                        forward_wall_found = true;
                        break;
                    }
                    
                    // 到达1米
                    if (traveled >= 1.0f) {
                        ROS_INFO("前进1米完成，未检测到池壁");
                        break;
                    }
                    
                    ros::Duration(0.05).sleep();
                }
                
                stop();
                ros::Duration(0.3).sleep();
                
                if (forward_wall_found) {
                    // 前进中找到了池壁，重新进入条件判断A
                    ROS_INFO("前进中找到池壁，进入条件判断A");
                    // 重置状态，继续循环处理检测到的池壁
                    total_rotated = 0;
                    rotate_count = 0;
                    turn_started = false;
                    continue;
                }
                
                // 前进1米后仍未找到，重置旋转计数，重新开始旋转扫描
                ROS_INFO("前进1米后仍未找到池壁，重置旋转计数，重新开始扫描");
                total_rotated = 0;
                rotate_count = 0;
                turn_started = false;
                continue;
            }
            
            // 继续下一次90度旋转
            continue;
        }
        
        ros::Duration(0.05).sleep();
    }
    
    stop();
    ROS_ERROR("寻池壁失败");
    return false;
}

bool RobotWallCleaner::cleanPoolWallLoop(float timeout) {
    ROS_INFO("========================================");
    ROS_INFO("池壁往复清洗开始");
    ROS_INFO("  超时: %.1f秒", timeout);
    ROS_INFO("========================================");
    
    if (cleaner_state_ != STATE_IDLE) {
        ROS_ERROR("机器人正在工作中");
        return false;
    }
    
    cleaner_state_ = STATE_CLEANING;
    bool success = executeCleanPoolWallLoop(timeout);
    cleaner_state_ = success ? STATE_IDLE : STATE_ERROR;
    
    return success;
}

bool RobotWallCleaner::executeCleanPoolWallLoop(float timeout) {
    const float SPEED_CALIBRATION = 0.885f;
    
    // 爬壁阶段参数
    float climb_speed = 0.15f;           // 爬壁速度
    float climb_pump = 0.1f;             // 爬壁时小水泵
    float climb_duration = 2.0f;         // 爬壁时间（秒）
    float climb_distance = 0.3f;         // 爬壁距离（米）
    
    // 正常清洗参数
    float pump_max = 0.99f;              // 清洗时水泵最大
    float pump_min = 0.3f;               // 转向时水泵
    float walk_speed = 0.35f;             // 清洗速度

     nh_.param("pump_max", pump_max, 0.99f);
    
    ROS_INFO("清洗参数: pump_max=%.2f, pump_min=%.2f, walk_speed=%.2f", 
             pump_max, pump_min, walk_speed);

    float cmd_speed = walk_speed / SPEED_CALIBRATION;
    if (cmd_speed > 0.35f) cmd_speed = 0.35f;
    
    ros::Time start_time = ros::Time::now();
    int cycle_count = 0;
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
        cycle_count++;
        ROS_INFO("========== 第%d段清洗开始 ==========", cycle_count);
        
               // ========== 阶段1：爬壁（整体爬上池壁） ==========
        ROS_INFO("阶段1: 爬壁阶段 - 低速前进，不开水泵");
        
        // 记录爬壁起始位置（红外触发的位置）
        updateCurrentPose();
        geometry_msgs::Pose2D climb_start_pose = current_pose_;
        ros::Time climb_start_time = ros::Time::now();
        
        // 记录起始红外距离
        float start_infra = std::min(infra_front_left_, infra_front_right_);
        ROS_INFO("爬壁起点: 红外距离=%.2f米", start_infra);
        
        // 爬壁参数
        const float CLIMB_DURATION = 3.0f;   // 3秒
        const float CLIMB_DISTANCE = 0.5f;   // 0.5米
        const float CLIMB_SPEED = 0.30f;     // 爬壁速度
        const float CLIMB_PUMP = 0.0f;       // 爬壁时不开水泵
        
        // 设置爬壁速度
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            keyboard_sim_.target_vx = CLIMB_SPEED;
            keyboard_sim_.current_vx = CLIMB_SPEED;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.key_pressed = true;
        }
        
        // 发送爬壁命令（不开水泵）
        geometry_msgs::Twist climb_cmd;
        climb_cmd.linear.x = CLIMB_SPEED;
        climb_cmd.linear.y = CLIMB_PUMP;  // 爬壁时不开水泵
        climb_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(climb_cmd);
        
        // 等待爬壁完成（时间或距离）
        bool climb_completed = false;
        while (ros::ok()) {
            ros::spinOnce();
            updateCurrentPose();
            
            // 计算已前进距离
            float dx = current_pose_.x - climb_start_pose.x;
            float dy = current_pose_.y - climb_start_pose.y;
            float traveled = sqrt(dx*dx + dy*dy);
            
            float elapsed = (ros::Time::now() - climb_start_time).toSec();
            
            // 当前红外距离
            float current_infra = std::min(infra_front_left_, infra_front_right_);
            
            ROS_INFO_THROTTLE(0.5, "爬壁中: 时间=%.2f/%.0fs, 距离=%.2f/%.1fm, 红外=%.2f->%.2f", 
                              elapsed, CLIMB_DURATION, traveled, CLIMB_DISTANCE, 
                              start_infra, current_infra);
            
            // 检查是否完成爬壁（达到时间或距离）
            if (elapsed >= CLIMB_DURATION || traveled >= CLIMB_DISTANCE) {
                climb_completed = true;
                ROS_INFO("爬壁完成: 时间=%.2fs, 距离=%.2fm, 红外从%.2f米变为%.2f米",
                         elapsed, traveled, start_infra, current_infra);
                break;
            }
            
            ros::Duration(0.05).sleep();
        }
        
   
        // ========== 阶段2：正常清洗（开启大水泵） ==========
        ROS_INFO("阶段2: 正常清洗 - 全速前进，大水泵");
   
    
        current_pump_speed_ = pump_max;
        
        // 初始化 keyboard_sim 的值
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            keyboard_sim_.target_vx = cmd_speed;
            keyboard_sim_.current_vx = cmd_speed;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.key_pressed = true;
        }
        
        ROS_INFO("贴壁清洗: 速度=%.2f m/s, 水泵=%.0f%%", walk_speed, pump_max * 100);
        
        bool out_of_water = false;
        ros::Time segment_start = ros::Time::now();
        int log_count = 0;
        
        // 行走直到露出水面
        while (ros::ok() && cleaner_state_ == STATE_CLEANING) {
            ros::spinOnce();
            updateCurrentPose();
            
            // ========== 关键：每次循环都发送速度命令 ==========
            geometry_msgs::Twist clean_cmd;
            clean_cmd.linear.x = cmd_speed;
            clean_cmd.linear.y = pump_max;
            clean_cmd.angular.z = keyboard_sim_.target_angular;  // 使用纠偏角速度
            cmd_vel_pub_.publish(clean_cmd);
            
            // 同时更新 keyboard_sim 的值，保持同步
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                keyboard_sim_.target_vx = cmd_speed;
                keyboard_sim_.current_vx = cmd_speed;
                // keyboard_sim_.target_angular 已在纠偏中设置
            }
            
            if ((ros::Time::now() - segment_start).toSec() > 180.0f) {
                ROS_WARN("本段行走超时");
                break;
            }
            
            // 打印速度信息（每50次循环打印一次，约2.5秒一次）
            if (log_count++ % 50 == 0) {
                ROS_INFO("贴壁清洗中: cmd_speed=%.3f m/s, 实际速度=%.3f m/s, 水泵=%.0f%%", 
                         cmd_speed, keyboard_sim_.current_vx, current_pump_speed_ * 100);
            }
            
            // 保持右侧贴壁（微小纠偏）
            float right_front = right_front_dist_;
            float right_rear = right_rear_dist_;
            if (right_front > 0.1f && right_front < 1.5f && 
                right_rear > 0.1f && right_rear < 1.5f) {
                float parallel_error = right_rear - right_front;
                if (parallel_error > 0.03f) {
                    applyKeyboardCommand('a');
                    keyboard_sim_.target_angular = 0.15f;
                } else if (parallel_error < -0.03f) {
                    applyKeyboardCommand('d');
                    keyboard_sim_.target_angular = -0.15f;
                } else {
                    keyboard_sim_.target_angular = 0.0f;
                }
            }
            
            // 检测露出水面
            if (isOutOfWater()) {
                ROS_INFO("检测到露出水面，本段结束");
                out_of_water = true;
                break;
            }
            
            ros::Duration(0.05).sleep();
        }
        
        stop();
        
        // 检查是否是露出水面导致的退出，还是超时导致的
        bool is_out_of_water = isOutOfWater();  // 重新检测一次
        
        if (!is_out_of_water) {
            ROS_WARN("本段结束但未检测到露出水面，可能超时，结束清洗");
            break;
        }
        // ========== 露出水面后的转向流程（后续回到阶段1爬壁） ==========
        ROS_INFO("========== 执行转向流程 ==========");
        
        // 1. 水泵减速
        current_pump_speed_ = pump_min;
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = 0.0f;
        pump_cmd.linear.y = pump_min;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ros::Duration(0.3).sleep();
        
        // 2. 后退直到前方检测到池壁0.5米
        ROS_INFO("后退，直到前方检测到池壁0.5米");
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            keyboard_sim_.target_vx = -0.15f;
            keyboard_sim_.current_vx = -0.15f;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.key_pressed = true;
        }
        
        ros::Time backup_start = ros::Time::now();
        bool front_wall_detected = false;
        
        while (ros::ok() && (ros::Time::now() - backup_start).toSec() < 10.0f) {
            ros::spinOnce();
            updateCurrentPose();
            
            float infra_left = infra_front_left_;
            float infra_right = infra_front_right_;
            float min_front = std::min(infra_left, infra_right);
            
            if (min_front > 0.08f && min_front < 0.50f) {
                front_wall_detected = true;
                ROS_INFO("前方检测到池壁: %.2f米", min_front);
                break;
            }
            
            ros::Duration(0.05).sleep();
        }
        
        stop();
        ros::Duration(0.3).sleep();
        
        if (!front_wall_detected) {
            ROS_WARN("后退未检测到池壁，跳过转向");
            continue;
        }
        
        // 3. 左旋转90度
        ROS_INFO("左旋转90度");
        disabling_timer_ = true;
        
        float turn_start_yaw = current_pose_.theta;
        ros::Time turn_start = ros::Time::now();
        
        while (ros::ok()) {
            ros::spinOnce();
            updateCurrentPose();
            
            float yaw_change = normalizeAngle(current_pose_.theta - turn_start_yaw);
            if (fabs(yaw_change) > 85.0f * M_PI / 180.0f ||
                (ros::Time::now() - turn_start).toSec() > 10.0f) {
                stop();
                ROS_INFO("左转完成");
                break;
            }
            
            geometry_msgs::Twist rotate_cmd;
            rotate_cmd.linear.x = 0.0f;
            rotate_cmd.linear.y = pump_min;
            rotate_cmd.angular.z = 0.25f;
            cmd_vel_pub_.publish(rotate_cmd);
            ros::Duration(0.05).sleep();
        }
        
        ros::Duration(0.3).sleep();
        
        // 4. 行走0.5米
        ROS_INFO("行走0.5米");
        {
            std::lock_guard<std::mutex> lock(keyboard_mutex_);
            keyboard_sim_.target_vx = 0.15f;
            keyboard_sim_.current_vx = 0.15f;
            keyboard_sim_.target_angular = 0.0f;
            keyboard_sim_.key_pressed = true;
        }
        
        updateCurrentPose();
        geometry_msgs::Pose2D move_start = current_pose_;
        ros::Time move_start_time = ros::Time::now();
        
        while (ros::ok()) {
            ros::spinOnce();
            updateCurrentPose();
            
            float dx = current_pose_.x - move_start.x;
            float dy = current_pose_.y - move_start.y;
            float traveled = sqrt(dx*dx + dy*dy);
            
            if (traveled >= 0.48f || (ros::Time::now() - move_start_time).toSec() > 5.0f) {
                stop();
                ROS_INFO("行走0.5米完成");
                break;
            }
            ros::Duration(0.05).sleep();
        }
        
        ros::Duration(0.3).sleep();
        
        // 5. 右旋转90度直到前方检测到池壁
        ROS_INFO("右旋转90度，直到前方检测到池壁");
        turn_start_yaw = current_pose_.theta;
        turn_start = ros::Time::now();
        
        while (ros::ok()) {
            ros::spinOnce();
            updateCurrentPose();
            
            if (checkFrontWallDetected()) {
                stop();
                ROS_INFO("右转中检测到前方池壁");
                break;
            }
            
            float yaw_change = normalizeAngle(turn_start_yaw - current_pose_.theta);
            if (fabs(yaw_change) > 85.0f * M_PI / 180.0f) {
                ROS_INFO_THROTTLE(1.0, "右转中: 转角=%.1f°", fabs(yaw_change) * 180.0f / M_PI);
            }
            
            if ((ros::Time::now() - turn_start).toSec() > 15.0f) {
                ROS_WARN("右转超时");
                stop();
                break;
            }
            
            geometry_msgs::Twist rotate_cmd;
            rotate_cmd.linear.x = 0.0f;
            rotate_cmd.linear.y = pump_min;
            rotate_cmd.angular.z = -0.25f;
            cmd_vel_pub_.publish(rotate_cmd);
            ros::Duration(0.05).sleep();
        }
        
        disabling_timer_ = false;
        ros::Duration(0.5).sleep();
        
        ROS_INFO("========== 转向完成，返回开始下一段爬壁 ==========");
        // 继续循环，回到阶段1（爬壁）
    }
    
    // 最终停止并关闭水泵
    stop();
    current_pump_speed_ = 0.0f;
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0f;
    stop_cmd.linear.y = 0.0f;
    stop_cmd.angular.z = 0.0f;
    cmd_vel_pub_.publish(stop_cmd);
    
    ROS_INFO("========================================");
    ROS_INFO("池壁往复清洗结束，共完成%d段", cycle_count);
    ROS_INFO("========================================");
    
    return true;
}

} // namespace sonar_slam

// ========== main函数 ==========
int main(int argc, char** argv) {
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ros::init(argc, argv, "robot_wall_cleaner");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("RobotWallCleaner 启动（池壁清洗器）");
    ROS_INFO("========================================");
    
    std::signal(SIGINT, sonar_slam::signalHandler);
    std::signal(SIGTERM, sonar_slam::signalHandler);
    
    sonar_slam::RobotWallCleaner cleaner(nh);
    sonar_slam::g_cleaner = &cleaner;
    
    // 读取参数
    bool auto_find_wall = true;
    bool auto_clean = true;
    // nh.param("auto_find_wall", auto_find_wall, false);
    // nh.param("auto_clean", auto_clean, false);
    
    float find_timeout = 900.0f;
    float clean_timeout = 6000000.0f;
    nh.param("find_timeout", find_timeout, 900.0f);
    nh.param("clean_timeout", clean_timeout, 6000000.0f);
    
    ROS_INFO("参数配置:");
    ROS_INFO("  auto_find_wall = %d", auto_find_wall);
    ROS_INFO("  auto_clean = %d", auto_clean);
    ROS_INFO("  find_timeout = %.1f s", find_timeout);
    ROS_INFO("  clean_timeout = %.1f s", clean_timeout);
    
    if (auto_find_wall) {
        ROS_INFO("开始自动寻池壁");
        bool success = cleaner.findPoolWall(find_timeout);
        if (success) {
            ROS_INFO("✅ 寻池壁成功");
            ROS_INFO("  auto_clean = %d", auto_clean);
            if (auto_clean) {
                ROS_INFO("开始自动池壁清洗");
                success = cleaner.cleanPoolWallLoop(clean_timeout);
                if (success) {
                    ROS_INFO("✅ 池壁清洗完成");
                } else {
                    ROS_ERROR("❌ 池壁清洗失败");
                }
            }
        } else {
            ROS_ERROR("❌ 寻池壁失败");
        }
    } else {
        ROS_INFO("等待服务调用:");
        ROS_INFO("  寻池壁: rosservice call /cleaner/find_wall");
        ROS_INFO("  开始清洗: rosservice call /cleaner/start_cleaning");
    }
    
    ROS_INFO("========================================");
    ROS_INFO("清洗器准备就绪");
    ROS_INFO("  状态: %s", cleaner.getStatus().c_str());
    ROS_INFO("========================================");
    
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("池壁清洗器关闭");
    return 0;
}