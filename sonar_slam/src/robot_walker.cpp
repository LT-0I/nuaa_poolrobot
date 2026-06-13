#include "sonar_slam/robot_walker.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <signal.h>
#include <csignal>
#include <locale>
#include <deque>

namespace sonar_slam {

RobotWalker* g_walker = nullptr;

void signalHandler(int sig) {
    if (g_walker) {
        g_walker->stop();
    }
    ros::shutdown();
}

RobotWalker::RobotWalker(ros::NodeHandle& nh) 
    : nh_(nh)
    , odom_received_(false)
    , walk_state_(STATE_IDLE)
    , current_pump_speed_(0.0f) 
{
    
    // 设置locale
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ROS_INFO("========================================");
    ROS_INFO("RobotWalker 初始化 (基于MoveController键盘模拟器)");
    ROS_INFO("========================================");
    
    // 初始化ROS接口 - 使用和MoveController完全相同的话题
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/wheel_odom", 1000, &RobotWalker::odomCallback, this);
    walk_service_ = nh_.advertiseService("/walker/walk_straight", 
                                        &RobotWalker::walkServiceCallback, this);
    
    // 使用和MoveController完全相同的20Hz键盘定时器
    keyboard_timer_ = nh_.createTimer(
        ros::Duration(0.05),  // 20Hz，与Python键盘控制和MoveController一致
        &RobotWalker::keyboardTimerCallback,
        this
    );
    
    // 初始化键盘模拟器
    initKeyboardSimulator();

    // 订阅超声波
    ultra_sub_ = nh_.subscribe("/ultra_data", 100, &RobotWalker::ultraCallback, this);

    // 读取参数
    nh_.param("pre_rotate_90", pre_rotate_90_, false);
    nh_.param("enable_parallel_align", enable_parallel_align_, false);
        
    // 水泵
    nh_.param("walker_pump_speed", current_pump_speed_, 0.5f);   

    // 初始化超声波数据
    right_front_dist_ = 999.0f;
    right_rear_dist_ = 999.0f;
    ultra_received_ = false;
    infra_front_left_ = 999.0f;
    infra_front_right_ = 999.0f;
    disabling_timer_ = false;
    
    ROS_INFO("ROS接口初始化完成:");
    ROS_INFO("  发布: /cmd_vel (复用MoveController的成功机制)");
    ROS_INFO("  订阅: /wheel_odom");
    ROS_INFO("  服务: /walker/walk_straight");
    ROS_INFO("  键盘定时器: 20Hz (复用MoveController的成功频率)");
    ROS_INFO("========================================");
}

RobotWalker::~RobotWalker() {
    stop();
    ROS_INFO("RobotWalker 关闭");
}

// ========== 复用MoveController的键盘模拟器初始化 ==========
void RobotWalker::initKeyboardSimulator() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    keyboard_sim_.reset();
    
    ROS_INFO("键盘模拟器初始化完成:");
    ROS_INFO("  线速度增量: %.3f (与MoveController一致)", keyboard_sim_.linear_inc);
    ROS_INFO("  角速度增量: %.3f (与MoveController一致)", keyboard_sim_.angular_inc);
    ROS_INFO("  最大速度: %.3f (与MoveController一致)", keyboard_sim_.max_linear);
    ROS_INFO("  空闲阈值: %d (与MoveController一致)", keyboard_sim_.idle_threshold);
}

// ==========   ==========
void RobotWalker::applyKeyboardCommand(char key) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    
    // 重置空闲计数器（模仿Python的count = 0）
    keyboard_sim_.idle_counter = 0;
    keyboard_sim_.key_pressed = true;
    keyboard_sim_.current_key = key;
    keyboard_sim_.key_press_time = ros::Time::now();
    
    // 完全模仿Python和MoveController的moveBindings
    switch (key) {
        case 'w':  // 前进：x = 0.01
            keyboard_sim_.target_vx += keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
            
        case 'x':  // 后退：x = -0.01
            keyboard_sim_.target_vx -= keyboard_sim_.linear_inc;
            keyboard_sim_.target_angular = 0.0f;
            break;
            
        case 'a':  // 左转：th = 0.3
            keyboard_sim_.target_angular = keyboard_sim_.angular_inc;
            break;
            
        case 'd':  // 右转：th = -0.3
            keyboard_sim_.target_angular = -keyboard_sim_.angular_inc;
            break;
            
        case 's':  // 停止
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

// ========== 复用MoveController的20Hz键盘定时器 ==========
void RobotWalker::keyboardTimerCallback(const ros::TimerEvent& event) {

    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (disabling_timer_) {
        return;  // 禁用键盘定时器
    } 
    // 1. 更新空闲计数器
    keyboard_sim_.idle_counter++;
    
    // 2. 检查空闲超时
    if (keyboard_sim_.idle_counter > keyboard_sim_.idle_threshold) {
        if (keyboard_sim_.key_pressed) {
            keyboard_sim_.key_pressed = false;
            keyboard_sim_.current_key = 0;
            keyboard_sim_.target_vx = 0.0f;
            keyboard_sim_.target_angular = 0.0f;
        }
    }
    
 
    // 4. 角速度直接设置（非累积）
    float current_angular = keyboard_sim_.target_angular;
    
    // 5. 限幅保护
    if (keyboard_sim_.current_vx > keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = keyboard_sim_.max_linear;
    }
    if (keyboard_sim_.current_vx < -keyboard_sim_.max_linear) {
        keyboard_sim_.current_vx = -keyboard_sim_.max_linear;
    }
    
    // 角速度限幅
    if (current_angular > keyboard_sim_.max_angular) {
        current_angular = keyboard_sim_.max_angular;
    }
    if (current_angular < -keyboard_sim_.max_angular) {
        current_angular = -keyboard_sim_.max_angular;
    }
    
    // 6. 发布控制命令 - 使用和MoveController完全相同的命令格式
    geometry_msgs::Twist cmd;
    cmd.linear.x = keyboard_sim_.current_vx;
    cmd.angular.z = current_angular;
    cmd.linear.y = current_pump_speed_;
    // 调试输出
    static int debug_count = 0;
    if (debug_count++ % 200 == 0) {  // 每10秒输出一次（20Hz * 200 = 10秒）
        ROS_INFO("键盘模拟器: 速度=%.3f, 目标=%.3f, 角速度=%.3f, 空闲=%d",
                 keyboard_sim_.current_vx, keyboard_sim_.target_vx,
                 cmd.angular.z, keyboard_sim_.idle_counter);
    }
    
    cmd_vel_pub_.publish(cmd);
}

bool RobotWalker::walkStraight(float distance, float max_speed, float timeout) {
    if (walk_state_ != STATE_IDLE) {
        ROS_ERROR("机器人正在行走中，无法开始新的行走");
        return false;
    }
    
    ROS_ERROR("========================================");
    ROS_ERROR("直线行走开始");
    ROS_ERROR("  距离: %.3f米", distance);
    ROS_ERROR("  超时: %.1f秒", timeout);
    ROS_ERROR("========================================");
        // ========== 新增：强制刷新里程计 ==========
    ROS_INFO("等待里程计数据更新...");
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    updateCurrentPose();
    ROS_ERROR("当前里程计: x=%.3f, y=%.3f, theta=%.1f°", 
              current_pose_.x, current_pose_.y, 
              current_pose_.theta * 180.0 / M_PI);
    // =========================================
    
    ROS_ERROR("========================================");
    // ========== 启动前准备 ==========
    
    // 1. 如果启用预旋转90度，先左转
    if (pre_rotate_90_) {
        ROS_INFO("执行启动前左转90度");
        
        // 等待odom数据
        if (!waitForOdom(5.0f)) {
            ROS_ERROR("等待odom数据超时");
            return false;
        }
        
        // 停止当前运动
        stop();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        
        // 记录起始角度
        updateCurrentPose();
        float start_yaw = current_pose_.theta;
        float target_yaw = normalizeAngle(start_yaw + M_PI / 2.0f);  // 左转90度
        ros::Time turn_start_time = ros::Time::now();
        bool turn_completed = false;
        
        // 左转循环
        while (ros::ok()) {
            updateCurrentPose();
            
            // 计算累计转角（左转为正）
            float yaw_change = current_pose_.theta - start_yaw;
            // 处理360°跨越
            if (yaw_change > M_PI) {
                yaw_change = yaw_change - 2 * M_PI;
            }
            float abs_yaw_change = fabs(yaw_change);
            float elapsed = (ros::Time::now() - turn_start_time).toSec();
            
            ROS_INFO_THROTTLE(0.5, "左转: 累计转角=%.1f°", abs_yaw_change * 180.0f / M_PI);
            
            // 检查是否完成（85-95度或超时）
            if (abs_yaw_change > 85.0f * M_PI / 180.0f || elapsed > 10.0f) {
                stop();
                ROS_INFO("左转完成: 累计转角=%.1f°", abs_yaw_change * 180.0f / M_PI);
                turn_completed = true;
                break;
            }
            
            // 发送左转命令
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0f;
            cmd.linear.y = 0.0f;
            cmd.angular.z = 0.25f;  // 左转速度
            cmd_vel_pub_.publish(cmd);
            
            ros::Duration(0.05).sleep();
            ros::spinOnce();
        }
        
        if (!turn_completed) {
            ROS_ERROR("左转90度失败");
            return false;
        }
        
        ros::Duration(0.5).sleep();
    }
    
    // 2. 进行右侧平行度对齐 
    if (enable_parallel_align_) {
        ROS_INFO("执行启动前平行度对齐");
        
        // 等待超声波数据
        ros::Time start_wait = ros::Time::now();
        while (ros::ok() && !ultra_received_) {
            if ((ros::Time::now() - start_wait).toSec() > 5.0f) {
                ROS_WARN("等待超声波数据超时，跳过平行度对齐");
                break;
            }
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        
        if (ultra_received_) {
            const float TOLERANCE = 0.02f;  // 平行度公差
            ros::Time align_start = ros::Time::now();
            const float ALIGN_TIMEOUT = 5.0f;
            
            while (ros::ok()) {
                // 检查超时
                if ((ros::Time::now() - align_start).toSec() > ALIGN_TIMEOUT) {
                    ROS_WARN("平行度对齐超时");
                    break;
                }
                
                // 获取最新距离
                float front_dist = right_front_dist_;
                float rear_dist = right_rear_dist_;
                
                if (front_dist > 6.0f || rear_dist > 6.0f) {
                    ROS_WARN_THROTTLE(2.0, "超声波数据无效: 前=%.2f, 后=%.2f", front_dist, rear_dist);
                    ros::Duration(0.5).sleep();
                    ros::spinOnce();
                    continue;
                }
                
                float parallel_error = rear_dist - front_dist;  // 前近后远为正
                
                ROS_INFO_THROTTLE(1.0, "平行度: 前=%.3f, 后=%.3f, 误差=%.3f", 
                                  front_dist, rear_dist, parallel_error);
                
                // 平行度达标
                if (fabs(parallel_error) < TOLERANCE) {
                    stop();
                    ROS_INFO("平行度对齐完成");
                    break;
                }
                
                // 调整平行度
                if (parallel_error > 0) {
                    // 前近后远，左转
                    applyKeyboardCommand('a');
                    ROS_INFO_THROTTLE(1.0, "前近后远(%.3f)，左转", parallel_error);
                } else {
                    // 前远后近，右转
                    applyKeyboardCommand('d');
                    ROS_INFO_THROTTLE(1.0, "前远后近(%.3f)，右转", -parallel_error);
                }
                
                // 短暂旋转后停止，重新检测
                ros::Duration(0.15).sleep();  // 从0.3秒改为0.15秒
                stop();
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            }
        }
        
        ros::Duration(0.3).sleep();
    }
    
    // ========== 原有行走逻辑 ==========
    walk_state_ = STATE_WALKING;
    bool success = executeStraightWalk(distance, max_speed, timeout);
    walk_state_ = success ? STATE_IDLE : STATE_ERROR;


    if (success && enable_parallel_align_) {
        ROS_INFO("========================================");
        ROS_INFO("执行停止后平行度对齐");
        ROS_INFO("========================================");
        
        ros::Duration(0.5).sleep();  // 等待完全静止
        ros::spinOnce();
        
        // 等待超声波数据（如果之前有数据，直接使用）
        if (!ultra_received_) {
            ros::Time start_wait = ros::Time::now();
            while (ros::ok() && !ultra_received_) {
                if ((ros::Time::now() - start_wait).toSec() > 5.0f) {
                    ROS_WARN("等待超声波数据超时，跳过停止后对齐");
                    break;
                }
                ros::Duration(0.1).sleep();
                ros::spinOnce();
            }
        }
        
        if (ultra_received_) {
            const float TOLERANCE = 0.005f;
            ros::Time align_start = ros::Time::now();
            const float ALIGN_TIMEOUT = 5.0f;
    
            // 调用对齐函数（它会持续调整直到达标）
            alignToRightWallOnly();
            
            if ((ros::Time::now() - align_start).toSec() > ALIGN_TIMEOUT) {
                ROS_WARN("停止后平行度对齐超时");
            }
        } else {
            ROS_WARN("无超声波数据，跳过停止后对齐");
        }
        
        ROS_INFO("停止后平行度对齐结束");
    }
    
    return success;
}
/******
bool RobotWalker::executeStraightWalk(float distance, float max_speed, float timeout) {
    

    // ========== 校准参数（根据测试数据得出） ==========
    // 速度校准：命令速度 vs 实际速度
    // 命令0.20 → 实际0.1676，比例 = 0.1676/0.20 = 0.838
    const float SPEED_CALIBRATION = 0.995f;  // 实际速度/命令速度
    
    // 距离校准：里程计读数 vs 实际地面距离
 
    const float ODOM_CALIBRATION = 1.0f;  // 实际距离/里程计读数
    
    // 1. 等待odom数据
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        return false;
    }

    // ========== 修改2：强制刷新并打印调试信息 ==========
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    updateCurrentPose();
    
    ROS_ERROR("========== 修改2：行走开始前里程计状态 ==========");
    ROS_ERROR("  odom_received_ = %s", odom_received_ ? "true" : "false");
    ROS_ERROR("  current_pose_.theta = %.3f rad (%.1f°)", 
              current_pose_.theta, current_pose_.theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.x = %.3f, current_pose_.y = %.3f", 
              current_pose_.x, current_pose_.y);
    ROS_ERROR("================================================");

    
    // ========== 清除累积状态 ==========
    ROS_WARN("准备行走：清除累积状态");
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.current_vx = 0.0f;
        keyboard_sim_.current_vy = 0.0f;
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.idle_counter = 0;
        keyboard_sim_.key_pressed = false;
        keyboard_sim_.current_key = 0;
        keyboard_sim_.key_press_time = ros::Time::now();
    }
    
    applyKeyboardCommand('s');
    for (int i = 0; i < 5; i++) {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }
    
    // 2. 初始化键盘模拟器
    initKeyboardSimulator();
    
    // 3. 记录起始位置
    updateCurrentPose();
    
    // ========== 保存原始角度用于相对纠偏 ==========
    float original_theta = current_pose_.theta;
      // ========== 修改4：验证代码（放在 original_theta 定义之后） ==========
    ROS_ERROR("========================================");
    ROS_ERROR("【修改4验证】original_theta 记录:");
    ROS_ERROR("  original_theta = %.3f rad (%.1f°)", 
              original_theta, original_theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.theta = %.3f rad (%.1f°)", 
              current_pose_.theta, current_pose_.theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.x = %.3f, current_pose_.y = %.3f", 
              current_pose_.x, current_pose_.y);
    
    if (fabs(original_theta) < 1.57 && fabs(original_theta) > 0.1) {
        ROS_ERROR(" 警告: original_theta=%.1f° (绝对值小于90°)，里程计可能未更新！", 
                  original_theta * 180.0 / M_PI);
    }
    ROS_ERROR("========================================");

    geometry_msgs::Pose2D start_pose;
    start_pose.x = current_pose_.x;
    start_pose.y = current_pose_.y;
    start_pose.theta = 0.0f;  // 相对角度为0
    
    ROS_WARN("全局起始角度: %.1f°, 使用相对角度坐标系", 
             original_theta * 180.0f / M_PI);
    
    // 4. 确定方向键
    char move_key = (distance >= 0) ? 'w' : 'x';
    float abs_distance = fabs(distance);
    
    // ========== 设置速度（应用速度校准） ==========
    // 放开速度上限到0.35，让用户设置的max_speed生效
    float desired_speed = std::min(max_speed, 0.35f);  // 期望的实际速度
    float cmd_speed = desired_speed / SPEED_CALIBRATION;  // 需要发送的命令速度
    
    // 限制命令速度最大值
    if (cmd_speed > 0.35f) cmd_speed = 0.35f;
    
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        if (move_key == 'w') {
            keyboard_sim_.target_vx = cmd_speed;
            keyboard_sim_.current_vx = cmd_speed; 
            ROS_WARN("前进: 期望速度=%.3f, 命令速度=%.3f (校准因子=%.3f)", 
                     desired_speed, cmd_speed, SPEED_CALIBRATION);
        } else {
            keyboard_sim_.target_vx = -cmd_speed;
            keyboard_sim_.current_vx = -cmd_speed; 
            ROS_WARN("后退: 期望速度=%.3f, 命令速度=%.3f (校准因子=%.3f)", 
                     desired_speed, cmd_speed, SPEED_CALIBRATION);
        }
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.key_pressed = true;       // 添加这行
        keyboard_sim_.idle_counter = 0;         // 添加这行，重置空闲计数
    }
    
    // ========== 开启水泵 ==========
    float pump_speed = 0.5f;  // 水泵速度50%，可调
    nh_.param("walker_pump_speed", pump_speed, 0.5f);
    
    bool pump_is_on = false;  // 水泵状态标志
    
    if (pump_speed > 0.01f) {
        geometry_msgs::Twist pump_cmd;
        current_pump_speed_ = pump_speed;
        pump_cmd.linear.x = cmd_speed;
        pump_cmd.linear.y = pump_speed;  // 开启水泵
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        pump_is_on = true;
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();  // 等待水泵启动
    }
    
    // 5. 主控制循环
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool pump_closed_early = false;  // 是否已提前关闭水泵
    
    ROS_INFO("开始行走，期望速度: %.2f m/s, 命令速度: %.2f m/s, 按键: %c", 
             desired_speed, cmd_speed, move_key);
    
    while (ros::ok() && walk_state_ == STATE_WALKING) {
        static float compensation_start_distance = 0.5f;
        static bool compensation_enabled = false;
        static float left_compensation = 1.0f;
        static float right_compensation = 1.0f;
        static float accumulated_yaw_error = 0.0f;
        static int yaw_sample_count = 0;
        static float last_adjust_distance = 0.5f;

        // 检查超时
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("移动超时");
            break;
        }
        
        // 更新当前位置
        updateCurrentPose();
        
        // ========== 计算已移动距离（应用距离校准） ==========
        float dx = current_pose_.x - start_pose.x;
        float dy = current_pose_.y - start_pose.y;
        float raw_distance = sqrt(dx*dx + dy*dy);           // 里程计原始读数
        // float real_distance = raw_distance ;  // 校准后的实际距离
        // 方向不同的校准系数
        // float odom_cal = (move_key == 'w') ? 0.891f : 0.89f;
        float odom_cal =  0.955f;

        float real_distance = raw_distance * odom_cal;
        float distance_remaining = abs_distance - real_distance;
        
        // ========== 新增：快到目的地时提前关闭水泵 ==========
        float pump_off_distance = 0.3f;  // 剩余0.3米时关闭水泵
        nh_.param("walker_pump_off_distance", pump_off_distance, 0.3f);
        
        if (pump_is_on && !pump_closed_early && distance_remaining < pump_off_distance && distance_remaining > 0) {
            geometry_msgs::Twist pump_cmd;
            pump_cmd.linear.x = cmd_speed;
            pump_cmd.linear.y = 0.0f;  // 关闭水泵
            pump_cmd.angular.z = 0.0f;
            cmd_vel_pub_.publish(pump_cmd);
            pump_closed_early = true;
            current_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地(剩余%.2f米)，水泵已关闭", distance_remaining);
        }

        // ========== 减速区（防止冲过头或提前停） ==========
        float slow_down_distance = 0.3f;  // 最后0.3米开始减速
        
        if (distance_remaining < slow_down_distance && distance_remaining > 0) {
            float speed_ratio = distance_remaining / slow_down_distance;
            speed_ratio = std::max(0.3f, speed_ratio);  // 最低30%速度
            
            float slowed_speed = cmd_speed * speed_ratio;
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                if (move_key == 'w') {
                    keyboard_sim_.target_vx = slowed_speed;
                    keyboard_sim_.current_vx = slowed_speed;
                } else {
                    keyboard_sim_.target_vx = -slowed_speed;
                    keyboard_sim_.current_vx = -slowed_speed;
                }
            }
            ROS_INFO_THROTTLE(1.0, "减速区: 剩余%.2fm, 速度%.3fm/s", 
                            distance_remaining, slowed_speed);
        }

 
            
        // 检查是否到达（使用校准后的实际距离）
        if (distance_remaining < 0.05f) {
            success = true;
            
            // 1. 记录停车前的角度
            float final_theta = current_pose_.theta;
            ROS_WARN("停车前角度: %.1f°", final_theta * 180.0f/M_PI);
            
            // 2. 先停止前进
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                keyboard_sim_.target_vx = 0.0f;
                keyboard_sim_.target_angular = 0.0f;
            }
            applyKeyboardCommand('s');
            for (int i = 0; i < 10; i++) {
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
            
            // 3. 等待0.5秒让机器人完全静止
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            updateCurrentPose();
            
            // 4. 计算停车后的角度误差
            float stop_yaw_error = normalizeAngle(current_pose_.theta - final_theta);
            
 
            // 5. 如果偏了，主动转回来（分步回正）
            if (fabs(stop_yaw_error) > 0.035f) {// 约2度
                ROS_WARN("刹车后偏转 %.1f°，主动回正", stop_yaw_error * 180.0f/M_PI);
                
                float target_theta = final_theta;
                float current_error = stop_yaw_error;
                
                // 分两步回正，避免过冲
                for (int step = 0; step < 2; step++) {
                    updateCurrentPose();
                    current_error = normalizeAngle(current_pose_.theta - target_theta);
                    
                    if (fabs(current_error) < 0.03f) break;  // 已经很小了
                    
                    // 第一次转70%，第二次转剩余的
                    float ratio = (step == 0) ? 0.7f : 1.0f;
                    float target_correction = current_error * ratio;
                    
                    // 根据需要的转角计算旋转时间（预判）
                    float angular_speed = (target_correction > 0) ? -0.12f : 0.12f;
                    float duration = fabs(target_correction) / fabs(angular_speed) * 0.9f;  // 0.9系数避免过冲
                    
                    ROS_INFO("回正第%d步: 需要转%.1f°, 用时%.2f秒", 
                            step + 1, target_correction * 180.0f/M_PI, duration);
                    
                    // 执行旋转
                    geometry_msgs::Twist rotate_cmd;
                    rotate_cmd.linear.x = 0.0;
                    rotate_cmd.angular.z = angular_speed;
                    cmd_vel_pub_.publish(rotate_cmd);
                    
                    ros::Time step_start = ros::Time::now();
                    while ((ros::Time::now() - step_start).toSec() < duration) {
                        ros::Duration(0.02).sleep();
                        ros::spinOnce();
                    }
                    
                    // 停止
                    rotate_cmd.angular.z = 0.0;
                    cmd_vel_pub_.publish(rotate_cmd);
                    ros::Duration(0.1).sleep();
                }
                
                // 最终微调
                updateCurrentPose();
                float final_error = normalizeAngle(current_pose_.theta - target_theta);
                if (fabs(final_error) > 0.02f) {
                    ROS_WARN("最终微调: %.1f°", final_error * 180.0f/M_PI);
                    geometry_msgs::Twist fine_cmd;
                    fine_cmd.linear.x = 0.0;
                    fine_cmd.angular.z = (final_error > 0) ? -0.05f : 0.05f;
                    cmd_vel_pub_.publish(fine_cmd);
                    ros::Duration(fabs(final_error) / 0.05f * 0.8f).sleep();
                    fine_cmd.angular.z = 0.0;
                    cmd_vel_pub_.publish(fine_cmd);
                }
                
                ROS_WARN("回正完成");
            }
            
            // 6. 最终停止（确保完全静止）
            applyKeyboardCommand('s');
            for (int i = 0; i < 5; i++) {
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
            
            ROS_ERROR("✅ 移动完成！里程计读数: %.3f米, 校准后实际: %.3f米", 
                      raw_distance, real_distance);
            break;
        }
        
        float relative_theta = normalizeAngle(current_pose_.theta - original_theta);
        static ros::Time loop_start_time = ros::Time::now();
        static bool correction_enabled = false;

        // 行走2秒后开始启用纠偏（更稳定，不受打滑影响）
        if (!correction_enabled && (ros::Time::now() - loop_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动连续按键纠偏");
        }

        
        if (correction_enabled) {
            static ros::Time last_correction_time = ros::Time::now();
            
            // 每秒更新一次角速度
            if ((ros::Time::now() - last_correction_time).toSec() > 1.0) {
                
                // 根据前进/后退自动适配左右键方向
                if (move_key == 'w') {  // 前进模式
                    if (relative_theta < -0.05f) {  // 向右偏
                        applyKeyboardCommand('a');
                        keyboard_sim_.target_angular = 0.3f;
                        ROS_WARN("→ 向右偏 %.1f°，按住 a 左转", 
                                relative_theta * 180.0f/M_PI);
                    }
                    else if (relative_theta > 0.05f) {  // 向左偏
                        applyKeyboardCommand('d');
                        keyboard_sim_.target_angular = -0.3f;
                        ROS_WARN("→ 向左偏 %.1f°，按住 d 右转", 
                                relative_theta * 180.0f/M_PI);
                    }
                    
                }
                else {  
                    // 后退模式 (move_key == 'x')
                    float actual_error = -relative_theta;
    
                    if (actual_error < -0.05f) {  // 实际向右偏
                        applyKeyboardCommand('d');  // 按右转
                        keyboard_sim_.target_angular = -0.3f;
                        ROS_WARN("→ 后退实际向右偏 %.1f° (里程计%.1f°)，按住 d 右转",
                                actual_error * 180.0f/M_PI,
                                relative_theta * 180.0f/M_PI);
                    }
                    else if (actual_error > 0.05f) {  // 实际向左偏
                        applyKeyboardCommand('a');  // 按左转
                        keyboard_sim_.target_angular = 0.3f;
                        ROS_WARN("→ 后退实际向左偏 %.1f° (里程计%.1f°)，按住 a 左转",
                                actual_error * 180.0f/M_PI,
                                relative_theta * 180.0f/M_PI);
                    }
                    
                }
                
                last_correction_time = ros::Time::now();
            }
        }

       
        // 进度报告（使用校准后的实际距离）
        static ros::Time last_report = ros::Time::now();
        if ((ros::Time::now() - last_report).toSec() > 0.5) {
            float progress = (real_distance / abs_distance) * 100.0f;
            float current_speed = 0.0f;
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                current_speed = keyboard_sim_.current_vx;
            }
            ROS_INFO("进度: %.0f%%, 已走%.3f/%.3f米 (里程计:%.3f米), 剩余:%.2f米, 命令速度=%.3fm/s", 
                     progress, real_distance, abs_distance, raw_distance, distance_remaining,
                     fabs(current_speed));
            last_report = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // ========== 确保水泵关闭（如果还没关） ==========
    if (pump_is_on && !pump_closed_early) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = 0.0f;
        pump_cmd.linear.y = 0.0f;
        pump_cmd.angular.z = 0.0f;
        current_pump_speed_ = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已关闭（最终确认）");
    }
    
    // 最终停止
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
    }
    applyKeyboardCommand('s');
    for (int i = 0; i < 10; i++) {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return success;
}

bool RobotWalker::executeStraightWalk(float distance, float max_speed, float timeout) {
    
    // ========== 累加距离变量 ==========
    float total_raw_distance = 0.0f;
    geometry_msgs::Pose2D last_pose;
    disabling_timer_ = false;
    // ========== 校准参数 ==========
    const float SPEED_CALIBRATION = 0.995f;
    const float ODOM_CALIBRATION = 1.0f;
    
    // 1. 等待odom数据
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        return false;
    }

    // 强制刷新并打印调试信息
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    updateCurrentPose();
    
    ROS_ERROR("========== 行走开始前里程计状态 ==========");
    ROS_ERROR("  odom_received_ = %s", odom_received_ ? "true" : "false");
    ROS_ERROR("  current_pose_.theta = %.3f rad (%.1f°)", 
              current_pose_.theta, current_pose_.theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.x = %.3f, current_pose_.y = %.3f", 
              current_pose_.x, current_pose_.y);
    ROS_ERROR("================================================");

    // 清除累积状态
    ROS_WARN("准备行走：清除累积状态");
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.current_vx = 0.0f;
        keyboard_sim_.current_vy = 0.0f;
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.idle_counter = 0;
        keyboard_sim_.key_pressed = false;
        keyboard_sim_.current_key = 0;
        keyboard_sim_.key_press_time = ros::Time::now();
    }
    
    applyKeyboardCommand('s');
    for (int i = 0; i < 5; i++) {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }
    
    // 初始化键盘模拟器
    initKeyboardSimulator();
    
    // 记录起始位置
    updateCurrentPose();
    
    // 初始化累加变量
    last_pose = current_pose_;
    
    // 保存原始角度用于相对纠偏
    float original_theta = current_pose_.theta;
    
    ROS_ERROR("========================================");
    ROS_ERROR("【修改4验证】original_theta 记录:");
    ROS_ERROR("  original_theta = %.3f rad (%.1f°)", 
              original_theta, original_theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.theta = %.3f rad (%.1f°)", 
              current_pose_.theta, current_pose_.theta * 180.0 / M_PI);
    ROS_ERROR("  current_pose_.x = %.3f, current_pose_.y = %.3f", 
              current_pose_.x, current_pose_.y);
    
    if (fabs(original_theta) < 1.57 && fabs(original_theta) > 0.1) {
        ROS_ERROR(" 警告: original_theta=%.1f° (绝对值小于90°)，里程计可能未更新！", 
                  original_theta * 180.0 / M_PI);
    }
    ROS_ERROR("========================================");

    geometry_msgs::Pose2D start_pose;
    start_pose.x = current_pose_.x;
    start_pose.y = current_pose_.y;
    start_pose.theta = 0.0f;
    
    ROS_WARN("全局起始角度: %.1f°, 使用相对角度坐标系", 
             original_theta * 180.0f / M_PI);
    
    // 确定方向键
    char move_key = (distance >= 0) ? 'w' : 'x';
    float abs_distance = fabs(distance);
    
    // 设置速度
    float desired_speed = std::min(max_speed, 0.35f);
    float cmd_speed = desired_speed / SPEED_CALIBRATION;
    if (cmd_speed > 0.35f) cmd_speed = 0.35f;
    
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        if (move_key == 'w') {
            keyboard_sim_.target_vx = cmd_speed;
            keyboard_sim_.current_vx = cmd_speed; 
            ROS_WARN("前进: 期望速度=%.3f, 命令速度=%.3f (校准因子=%.3f)", 
                     desired_speed, cmd_speed, SPEED_CALIBRATION);
        } else {
            keyboard_sim_.target_vx = -cmd_speed;
            keyboard_sim_.current_vx = -cmd_speed; 
            ROS_WARN("后退: 期望速度=%.3f, 命令速度=%.3f (校准因子=%.3f)", 
                     desired_speed, cmd_speed, SPEED_CALIBRATION);
        }
        keyboard_sim_.target_angular = 0.0f;
        keyboard_sim_.key_pressed = true;
        keyboard_sim_.idle_counter = 0;
    }
    
    // 开启水泵
    float pump_speed = 0.5f;
    nh_.param("walker_pump_speed", pump_speed, 0.5f);
    
    bool pump_is_on = false;
    
    if (pump_speed > 0.01f) {
        geometry_msgs::Twist pump_cmd;
        current_pump_speed_ = pump_speed;
        pump_cmd.linear.x = cmd_speed;
        pump_cmd.linear.y = pump_speed;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        pump_is_on = true;
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();
    }
    
    // 主控制循环
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool pump_closed_early = false;
    
    // 校准系数
    // float odom_cal = (move_key == 'w') ? 0.963f : 0.82f;
    float odom_cal =1.0f;
    
    ROS_INFO("开始行走，期望速度: %.2f m/s, 命令速度: %.2f m/s, 按键: %c, 校准系数: %.3f", 
             desired_speed, cmd_speed, move_key, odom_cal);
    
    while (ros::ok() && walk_state_ == STATE_WALKING) {
        // 检查超时
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("移动超时");
            break;
        }
        
        // 更新当前位置
        updateCurrentPose();
        
        // ========== 计算每帧位移（累加法） ==========
        float dx = current_pose_.x - last_pose.x;
        float dy = current_pose_.y - last_pose.y;
        float delta = sqrt(dx*dx + dy*dy);
        
        // 累加原始里程
        if (delta > 0.001f && delta < 0.5f) {
            total_raw_distance += delta;
        }
        last_pose = current_pose_;
        
        // ========== 日志输出 ==========
        static int walker_cnt = 0;
        walker_cnt++;
        if (walker_cnt % 50 == 0) {
            ROS_ERROR("[Walker] dx=%.5f, dy=%.5f, delta=%.5f, total_raw=%.4f, pose_x=%.4f", 
                    dx, dy, delta, total_raw_distance, current_pose_.x);
        }
        
        // ========== 计算校准后的实际距离 ==========
        float raw_distance = total_raw_distance;  // 累加的原始里程
        float real_distance = raw_distance * odom_cal;
        float distance_remaining = abs_distance - real_distance;
        
        // 快到目的地时提前关闭水泵
        float pump_off_distance = 0.3f;
        nh_.param("walker_pump_off_distance", pump_off_distance, 0.3f);
        
        if (pump_is_on && !pump_closed_early && distance_remaining < pump_off_distance && distance_remaining > 0) {
            geometry_msgs::Twist pump_cmd;
            pump_cmd.linear.x = cmd_speed;
            pump_cmd.linear.y = 0.0f;
            pump_cmd.angular.z = 0.0f;
            cmd_vel_pub_.publish(pump_cmd);
            pump_closed_early = true;
            current_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地(剩余%.2f米)，水泵已关闭", distance_remaining);
        }

        // 减速区
        float slow_down_distance = 0.3f;
        
        if (distance_remaining < slow_down_distance && distance_remaining > 0) {
            float speed_ratio = distance_remaining / slow_down_distance;
            speed_ratio = std::max(0.3f, speed_ratio);
            float slowed_speed = cmd_speed * speed_ratio;
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                if (move_key == 'w') {
                    keyboard_sim_.target_vx = slowed_speed;
                    keyboard_sim_.current_vx = slowed_speed;
                } else {
                    keyboard_sim_.target_vx = -slowed_speed;
                    keyboard_sim_.current_vx = -slowed_speed;
                }
            }
            ROS_INFO_THROTTLE(1.0, "减速区: 剩余%.2fm, 速度%.3fm/s", 
                            distance_remaining, slowed_speed);
        }
        
        // 检查是否到达
        if (distance_remaining < 0.05f) {
            success = true;
            
            // 记录停车前的角度
            float final_theta = current_pose_.theta;
            ROS_WARN("停车前角度: %.1f°", final_theta * 180.0f/M_PI);
            
            // 先停止前进
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                keyboard_sim_.target_vx = 0.0f;
                keyboard_sim_.target_angular = 0.0f;
            }
            applyKeyboardCommand('s');
            for (int i = 0; i < 10; i++) {
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
            
            // 等待0.5秒让机器人完全静止
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            updateCurrentPose();
            
            // 计算停车后的角度误差
            float stop_yaw_error = normalizeAngle(current_pose_.theta - final_theta);
            
            // 如果偏了，主动转回来
            if (fabs(stop_yaw_error) > 0.03f) {
                ROS_WARN("刹车后偏转 %.1f°，主动回正", stop_yaw_error * 180.0f/M_PI);
                
                float target_theta = final_theta;
                float current_error = stop_yaw_error;
                
                for (int step = 0; step < 2; step++) {
                    updateCurrentPose();
                    current_error = normalizeAngle(current_pose_.theta - target_theta);
                    
                    if (fabs(current_error) < 0.03f) break;
                    
                    float ratio = (step == 0) ? 0.7f : 1.0f;
                    float target_correction = current_error * ratio;
                    
                    float angular_speed = (target_correction > 0) ? -0.12f : 0.12f;
                    float duration = fabs(target_correction) / fabs(angular_speed) * 0.9f;
                    
                    ROS_INFO("回正第%d步: 需要转%.1f°, 用时%.2f秒", 
                            step + 1, target_correction * 180.0f/M_PI, duration);
                    
                    geometry_msgs::Twist rotate_cmd;
                    rotate_cmd.linear.x = 0.0;
                    rotate_cmd.angular.z = angular_speed;
                    cmd_vel_pub_.publish(rotate_cmd);
                    
                    ros::Time step_start = ros::Time::now();
                    while ((ros::Time::now() - step_start).toSec() < duration) {
                        ros::Duration(0.02).sleep();
                        ros::spinOnce();
                    }
                    
                    rotate_cmd.angular.z = 0.0;
                    cmd_vel_pub_.publish(rotate_cmd);
                    ros::Duration(0.1).sleep();
                }
                
                updateCurrentPose();
                float final_error = normalizeAngle(current_pose_.theta - target_theta);
                if (fabs(final_error) > 0.02f) {
                    ROS_WARN("最终微调: %.1f°", final_error * 180.0f/M_PI);
                    geometry_msgs::Twist fine_cmd;
                    fine_cmd.linear.x = 0.0;
                    fine_cmd.angular.z = (final_error > 0) ? -0.05f : 0.05f;
                    cmd_vel_pub_.publish(fine_cmd);
                    ros::Duration(fabs(final_error) / 0.05f * 0.8f).sleep();
                    fine_cmd.angular.z = 0.0;
                    cmd_vel_pub_.publish(fine_cmd);
                }
                
                ROS_WARN("回正完成");
            }
            
            applyKeyboardCommand('s');
            for (int i = 0; i < 5; i++) {
                ros::Duration(0.05).sleep();
                ros::spinOnce();
            }
            
            // ========== 最终统计 ==========
            ROS_ERROR("✅ 移动完成！累加里程: %.3f米, 校准后实际: %.3f米", 
                      raw_distance, real_distance);
            ROS_ERROR("=== 最终统计 ===");
            ROS_ERROR("total_raw_distance = %.4f 米", total_raw_distance);
            ROS_ERROR("start_pose.x = %.4f 米", start_pose.x);
            ROS_ERROR("end_pose.x = %.4f 米", current_pose_.x);
            ROS_ERROR("pose.x 差值 = %.4f 米", (current_pose_.x - start_pose.x));
            ROS_ERROR("odom_cal = %.4f", odom_cal);
            ROS_ERROR("real_distance = %.4f 米", real_distance);
            break;
        }
        
        // 角度纠偏
        float relative_theta = normalizeAngle(current_pose_.theta - original_theta);
        static ros::Time loop_start_time = ros::Time::now();
        static bool correction_enabled = false;

        if (!correction_enabled && (ros::Time::now() - loop_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动连续按键纠偏");
        }
        
        if (correction_enabled) {
            static ros::Time last_correction_time = ros::Time::now();
            
            if ((ros::Time::now() - last_correction_time).toSec() > 1.0) {
                if (move_key == 'w') {
                    if (relative_theta < -0.03f) {
                        applyKeyboardCommand('a');
                        keyboard_sim_.target_angular = 0.3f;
                        ROS_WARN("→ 向右偏 %.1f°，按住 a 左转", 
                                relative_theta * 180.0f/M_PI);
                    }
                    else if (relative_theta > 0.03f) {
                        applyKeyboardCommand('d');
                        keyboard_sim_.target_angular = -0.3f;
                        ROS_WARN("→ 向左偏 %.1f°，按住 d 右转", 
                                relative_theta * 180.0f/M_PI);
                    }
                }
                else {
                    float actual_error = -relative_theta;
                    if (actual_error < -0.03f) {
                        applyKeyboardCommand('d');
                        keyboard_sim_.target_angular = -0.3f;
                        ROS_WARN("→ 后退实际向右偏 %.1f°，按住 d 右转", actual_error * 180.0f/M_PI);
                    }
                    else if (actual_error > 0.03f) {
                        applyKeyboardCommand('a');
                        keyboard_sim_.target_angular = 0.3f;
                        ROS_WARN("→ 后退实际向左偏 %.1f°，按住 a 左转", actual_error * 180.0f/M_PI);
                    }
                }
                last_correction_time = ros::Time::now();
            }
        }
        
        // 进度报告
        static ros::Time last_report = ros::Time::now();
        if ((ros::Time::now() - last_report).toSec() > 0.5) {
            float progress = (real_distance / abs_distance) * 100.0f;
            float current_speed = 0.0f;
            {
                std::lock_guard<std::mutex> lock(keyboard_mutex_);
                current_speed = keyboard_sim_.current_vx;
            }
            ROS_INFO("进度: %.0f%%, 已走%.3f/%.3f米 (累加里程:%.3f米), 剩余:%.2f米, 命令速度=%.3fm/s", 
                     progress, real_distance, abs_distance, raw_distance, distance_remaining,
                     fabs(current_speed));
            last_report = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // 确保水泵关闭
    if (pump_is_on && !pump_closed_early) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = 0.0f;
        pump_cmd.linear.y = 0.0f;
        pump_cmd.angular.z = 0.0f;
        current_pump_speed_ = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已关闭（最终确认）");
    }
    
    // 最终停止
    {
        std::lock_guard<std::mutex> lock(keyboard_mutex_);
        keyboard_sim_.target_vx = 0.0f;
        keyboard_sim_.target_angular = 0.0f;
    }
    applyKeyboardCommand('s');
    for (int i = 0; i < 10; i++) {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    return success;
}
****/

bool RobotWalker::executeStraightWalk(float distance, float max_speed, float timeout) {
    
         // 初始化日志
    initWalkLog("straight_walk", fabs(distance));
    // ========== 参数初始化 ==========
    const float SPEED_CALIBRATION = 0.995f;
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    
    // 方向判断
    char move_direction = (distance >= 0) ? 'w' : 'x';
    float target_distance = fabs(distance);
    float abs_distance = target_distance;

    
    // 速度设置
    float desired_speed = std::min(max_speed, 0.35f);
    float cmd_speed = desired_speed / SPEED_CALIBRATION;
    if (cmd_speed > 0.35f) cmd_speed = 0.35f;
    
    // ========== 1. 等待并获取初始状态 ==========
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        return false;
    }
    
    // 强制刷新里程计
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    updateCurrentPose();
    
    // 记录起始状态
    geometry_msgs::Pose2D start_pose = current_pose_;
    float start_yaw = current_pose_.theta;
    float total_traveled = 0.0f;
    float total_yaw_change = 0.0f;      // 累积角度变化（用于纠偏）
    geometry_msgs::Pose2D last_pose = start_pose;
    float last_theta = start_yaw;
    
    ROS_ERROR("起始状态: 位置(%.2f,%.2f), 角度=%.1f°", 
              start_pose.x, start_pose.y, start_yaw * RAD_TO_DEG);
    
    // ========== 2. 禁用键盘定时器，使用直接控制 ==========
    bool old_disabling_state = disabling_timer_;
    disabling_timer_ = true;
    
    // ========== 3. 开启水泵 ==========
    float pump_speed = current_pump_speed_;
    bool pump_on = (pump_speed > 0.01f);
    if (pump_on) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = cmd_speed;
        pump_cmd.linear.y = pump_speed;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();
    }
    
    // ========== 4. 主控制循环 ==========
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool pump_closed_early = false;
    bool correction_enabled = false;
    ros::Time correction_start_time = ros::Time::now();
    ros::Time last_correction_time = ros::Time::now();
    float current_angular_correction = 0.0f;
    
    ROS_INFO("开始行走，方向: %c, 速度: %.2f m/s, 目标: %.2f米", 
             move_direction, cmd_speed, target_distance);
    
    while (ros::ok() && walk_state_ == STATE_WALKING) {
        updateCurrentPose();
        
        // 超时检查
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("行走超时");
            break;
        }
        
        // ========== 计算已走距离（累加法，不受打滑影响） ==========
        float dx = current_pose_.x - last_pose.x;
        float dy = current_pose_.y - last_pose.y;
        float delta = sqrt(dx*dx + dy*dy);
        if (delta > 0.001f && delta < 0.5f) {
            total_traveled += delta;
        }
        last_pose = current_pose_;
 
        // ========== 计算累积角度变化（用于纠偏） ==========
        float delta_yaw = normalizeAngle(current_pose_.theta - last_theta);
        total_yaw_change += delta_yaw;
        last_theta = current_pose_.theta;

        // 每秒记录日志
        recordWalkData(total_traveled, abs_distance);
        
        // 更新日志缓冲区中的目标距离
        if (!log_buffer_.empty()) {
            log_buffer_.back().target_distance = abs_distance;
        }
        
        float remaining = target_distance - total_traveled;
        
        // ========== 到达判断 ==========
        if (remaining < 0.05f) {
            success = true;
            ROS_ERROR("✅ 行走完成！实际距离: %.3f米, 累积偏角: %.1f°", 
                      total_traveled, total_yaw_change * RAD_TO_DEG);
            break;
        }
        
        // ========== 减速区 ==========
        float slow_down_dist = 0.3f;
        float current_speed = cmd_speed;
        if (remaining < slow_down_dist && remaining > 0) {
            float speed_ratio = std::max(0.3f, remaining / slow_down_dist);
            current_speed = cmd_speed * speed_ratio;
            ROS_INFO_THROTTLE(1.0, "减速区: 剩余%.2fm, 速度%.3fm/s", remaining, current_speed);
        }
        
        // ========== 提前关泵 ==========
        float pump_off_dist = 0.3f;
        if (pump_on && !pump_closed_early && remaining < pump_off_dist && remaining > 0) {
            pump_on = false;
            pump_closed_early = true;
            current_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地，水泵已关闭");
        }
        
 
        // ========== 角度纠偏（使用累积偏航，高频响应） ==========
        if (!correction_enabled && (ros::Time::now() - correction_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动实时纠偏, 当前累积偏角=%.1f°", total_yaw_change * RAD_TO_DEG);
        }

        if (correction_enabled) {
            float relative_theta = total_yaw_change;
            
            if (move_direction == 'w') {  // 前进
                // 修正：relative_theta > 0 表示实际左转，需要右转（负角速度）
                if (relative_theta > 0.02f) {  // 实际左转
                    current_angular_correction = -std::min(0.30f, relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 左转 %.1f°，右转速度=%.2f", 
                                    relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else if (relative_theta < -0.02f) {  // 实际右转
                    current_angular_correction = std::min(0.30f, -relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 右转 %.1f°，左转速度=%.2f", 
                                    -relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else {
                    current_angular_correction = 0.0f;
                }
            }
            else if (move_direction == 'x') {  // 后退模式
                if (relative_theta > 0.02f) {  // 后退时左偏
                    current_angular_correction = -std::min(0.30f, relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 后退左转 %.1f°，右转速度=%.2f", 
                                    relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else if (relative_theta < -0.02f) {  // 后退时右偏
                    current_angular_correction = std::min(0.30f, -relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 后退右转 %.1f°，左转速度=%.2f", 
                                    -relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else {
                    current_angular_correction = 0.0f;
                }
            }
            
            // 基础补偿（暂时注释，调试确认后再启用）
            // current_angular_correction += -0.08f;
            
            // 限幅
            current_angular_correction = std::max(-0.35f, std::min(0.35f, current_angular_correction));
        }
        
        // ========== 直接发布控制命令 ==========
        geometry_msgs::Twist cmd;
        cmd.linear.x = (move_direction == 'w') ? current_speed : -current_speed;
        cmd.linear.y = pump_on ? pump_speed : 0.0f;
        cmd.angular.z = current_angular_correction;
        cmd_vel_pub_.publish(cmd);
        
        // ========== 进度打印 ==========
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0) {
            float progress = (total_traveled / target_distance) * 100.0f;
            ROS_INFO("进度: %.0f%%, 已走%.2f/%.2f米, 偏角:%.1f°, 纠正角速度:%.2f", 
                     progress, total_traveled, target_distance, 
                     total_yaw_change * RAD_TO_DEG, current_angular_correction);
            last_print = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();  // 20Hz 控制频率
    }
    
    // ========== 5. 停止并清理 ==========
    stop();
    
    if (pump_on) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.0f;
        cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(cmd);
        current_pump_speed_ = 0.0f;
        ROS_INFO("水泵已关闭");
    }
    
    // 恢复键盘定时器状态
    disabling_timer_ = old_disabling_state;
    
    walk_state_ = success ? STATE_IDLE : STATE_ERROR;
    // 保存日志
    saveWalkLog(success);
    return success;
}
// ========== 辅助函数 ==========
void RobotWalker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    current_odom_ = *msg;
    odom_received_ = true;
}

void RobotWalker::updateCurrentPose() {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (odom_received_) {
        current_pose_.x = current_odom_.pose.pose.position.x;
        current_pose_.y = current_odom_.pose.pose.position.y;
        current_pose_.theta = calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    } else {
        static int warn_count = 0;
        if (warn_count++ % 100 == 0) {
            ROS_WARN_THROTTLE(2.0, "updateCurrentPose: 没有里程计数据！");
        }
    }
}

float RobotWalker::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

bool RobotWalker::waitForOdom(float timeout) {
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

void RobotWalker::stop() {
    applyKeyboardCommand('s');
    ROS_INFO("机器人已停止");
}

bool RobotWalker::walkServiceCallback(std_srvs::Trigger::Request& req, 
                                     std_srvs::Trigger::Response& res) {
    ROS_INFO("收到行走服务请求");
    
    // 使用硬编码参数进行测试
    float distance = 1.0f;      // 先测试1米
    float max_speed = 0.1f;     // 低速测试
    float timeout = 20.0f;
    
    bool success = walkStraight(distance, max_speed, timeout);
    
    res.success = success;
    res.message = success ? "行走成功" : "行走失败";
    
    return true;
}

std::string RobotWalker::getStatus() const {
    switch (walk_state_) {
        case STATE_IDLE: return "空闲";
        case STATE_WALKING: return "行走中";
        case STATE_ERROR: return "错误";
        default: return "未知";
    }
}

float RobotWalker::normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

 
void RobotWalker::ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(keyboard_mutex_);
    if (msg->data.size() >= 4) {
        // 根据实际硬件映射
        infra_front_left_ = msg->data[0] / 1000.0f;   // data[0] = 前左超声波
        infra_front_right_ = msg->data[1] / 1000.0f;  // data[1] = 前右超声波
        right_front_dist_ = msg->data[2] / 1000.0f;   // data[2] = 右前超声波
        right_rear_dist_ = msg->data[3] / 1000.0f;    // data[3] = 右后超声波
        
        // 前方超声波过滤（有效范围 0.05m ~ 5.0m）
        if (infra_front_left_ > 5.0f || infra_front_left_ < 0.05f) {
            infra_front_left_ = 999.0f;
        }
        if (infra_front_right_ > 5.0f || infra_front_right_ < 0.05f) {
            infra_front_right_ = 999.0f;
        }
        
        // 右侧超声波过滤（有效范围 0.05m ~ 5.0m）
        if (right_front_dist_ > 5.0f || right_front_dist_ < 0.05f) {
            right_front_dist_ = 999.0f;
        }
        if (right_rear_dist_ > 5.0f || right_rear_dist_ < 0.05f) {
            right_rear_dist_ = 999.0f;
        }
        
        ultra_received_ = true;
    }
}

/******* 
bool RobotWalker::walkStraightWithWallAvoidanceAdvanced(float distance, float max_speed, float timeout) {
    if (walk_state_ != STATE_IDLE) {
        ROS_ERROR("机器人正在行走中，无法开始新的行走");
        return false;
    }
    
    ROS_ERROR("========================================");
    ROS_ERROR("高级直线行走开始（带行走中池壁检测与避让）");
    ROS_ERROR("  距离: %.3f米", distance);
    ROS_ERROR("  速度: %.3f米/秒", max_speed);
    ROS_ERROR("  超时: %.1f秒", timeout);
    ROS_ERROR("========================================");
    
    walk_state_ = STATE_WALKING;
    
    // ========== 禁用键盘定时器，避免与直接命令冲突 ==========
    disabling_timer_ = true;
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    
    // ========== 校准参数 ==========
    const float SPEED_CALIBRATION = 0.885f;  // 实际速度/命令速度
    
    // ========== 初始化参数 ==========
    float target_distance = fabs(distance);
    char move_direction = (distance >= 0) ? 'w' : 'x';
    float desired_speed = std::min(max_speed, 0.35f);
    float cmd_speed = desired_speed / SPEED_CALIBRATION;
    if (cmd_speed > 0.35f) cmd_speed = 0.35f;
    
    // 等待odom数据
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        disabling_timer_ = false;
        walk_state_ = STATE_ERROR;
        return false;
    }
    
    // 强制刷新里程计
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    updateCurrentPose();
    
    // 记录起始位置
    geometry_msgs::Pose2D start_pose = current_pose_;
    float start_yaw = current_pose_.theta;
    float total_traveled = 0.0f;
    float total_yaw_change = 0.0f;  // 累积角度变化
    geometry_msgs::Pose2D last_pose = start_pose;
    geometry_msgs::Pose2D last_angle_pose = start_pose;
    
    ROS_ERROR("起始状态: 位置(%.2f,%.2f), 角度=%.1f°", 
              start_pose.x, start_pose.y, start_yaw * RAD_TO_DEG);
    
    // 水泵控制
    float pump_speed = current_pump_speed_;   
    bool pump_on = (pump_speed > 0.01f);
    if (pump_on) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = cmd_speed;
        pump_cmd.linear.y = pump_speed;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();
    }
    
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool wall_avoiding = false;
    int front_wall_detect_count = 0;
    bool pump_closed_early = false;
    
    // 纠偏相关变量
    ros::Time correction_start_time = ros::Time::now();
    bool correction_enabled = false;
    ros::Time last_correction_time = ros::Time::now();
    float current_angular_correction = 0.0f;
    
    // ========== 主循环 ==========
    while (ros::ok() && walk_state_ == STATE_WALKING) {
        updateCurrentPose();
        
        // 超时检查
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("行走超时");
            break;
        }
        
        // ========== 计算已走距离（累积） ==========
        float dx = current_pose_.x - last_pose.x;
        float dy = current_pose_.y - last_pose.y;
        float delta = sqrt(dx*dx + dy*dy);
        if (delta > 0.001f && delta < 0.5f) {
            total_traveled += delta;
        }
        last_pose = current_pose_;
        
        // ========== 累积计算角度变化（避免角度跳变问题） ==========
        float delta_yaw = normalizeAngle(current_pose_.theta - last_angle_pose.theta);
        total_yaw_change += delta_yaw;
        last_angle_pose.theta = current_pose_.theta;
        
        float remaining = target_distance - total_traveled;
        
        // ========== 前方池壁检测 ==========
        if (!wall_avoiding && move_direction == 'w') {
            float front_left_dist = infra_front_left_;
            float front_right_dist = infra_front_right_;
            float front_dist = std::min(front_left_dist, front_right_dist);
            bool has_front_wall = (front_dist > 0.1f && front_dist < 1.0f);
            
            if (has_front_wall) {
                front_wall_detect_count++;
                if (front_wall_detect_count >= 3) {
                    ROS_WARN("行走中检测到前方池壁(%.2f米)，执行避让！", front_dist);
                    wall_avoiding = true;
                    
                    // ========== 1. 停止 ==========
                    stop();
                    ros::Duration(0.3).sleep();
                    ros::spinOnce();
                    updateCurrentPose();
                    
                    // ========== 2. 后退0.5米 ==========
                    ROS_INFO("后退 0.5 米");
                    float backup_speed = 0.20f;
                    float backup_time = 0.5f / backup_speed;
                    ros::Time backup_start = ros::Time::now();
                    
                    while (ros::ok()) {
                        float elapsed = (ros::Time::now() - backup_start).toSec();
                        if (elapsed >= backup_time) break;
                        
                        geometry_msgs::Twist cmd;
                        cmd.linear.x = -backup_speed;
                        cmd.linear.y = pump_speed;
                        cmd.angular.z = 0.0f;
                        cmd_vel_pub_.publish(cmd);
                        ros::Duration(0.02).sleep();
                    }
                    
                    stop();
                    ros::Duration(0.3).sleep();
                    updateCurrentPose();
                    
                    // ========== 3. 左转90度 ==========
                    ROS_INFO("左转90度");
                    float turn_start_yaw = current_pose_.theta;
                    ros::Time turn_start_time = ros::Time::now();
                    bool turn_completed = false;
                    
                    while (ros::ok()) {
                        ros::spinOnce();
                        updateCurrentPose();
                        
                        float yaw_change = current_pose_.theta - turn_start_yaw;
                        if (yaw_change > M_PI) yaw_change -= 2 * M_PI;
                        float abs_yaw_change = fabs(yaw_change);
                        
                        if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
                            (ros::Time::now() - turn_start_time).toSec() > 10.0f) {
                            stop();
                            turn_completed = true;
                            break;
                        }
                        
                        geometry_msgs::Twist cmd;
                        cmd.linear.x = 0.0f;
                        cmd.linear.y = pump_speed;
                        cmd.angular.z = 0.25f;
                        cmd_vel_pub_.publish(cmd);
                        ros::Duration(0.05).sleep();
                    }
                    
                    if (!turn_completed) {
                        ROS_ERROR("左转90度失败");
                        walk_state_ = STATE_ERROR;
                        disabling_timer_ = false;
                        return false;
                    }
                    ros::Duration(0.3).sleep();
                    
                    // ========== 4. 右侧平行度对齐 ==========
                    ROS_INFO("右侧平行度对齐");
                    alignToRightWallOnly();
                    ros::Duration(0.3).sleep();
                    
                    // ========== 5. 更新状态 ==========
                    target_distance = fabs(distance) - total_traveled;
                    start_pose = current_pose_;
                    start_yaw = current_pose_.theta;
                    last_pose = current_pose_;
                    last_angle_pose = current_pose_;
                    total_yaw_change = 0.0f;  // 重置累积角度
                    start_time = ros::Time::now();
                    wall_avoiding = false;
                    front_wall_detect_count = 0;
                    correction_enabled = false;
                    correction_start_time = ros::Time::now();

                    ROS_INFO("避让完成，剩余: %.2f米，当前角度: %.1f°", 
                            target_distance, current_pose_.theta * RAD_TO_DEG);
                    continue;
                }
            } else {
                front_wall_detect_count = 0;
            }
        }
        
        // 到达目标
        if (remaining < 0.05f) {
            success = true;
            ROS_ERROR("✅ 行走完成！实际距离: %.3f米", total_traveled);
            break;
        }
        
        // ========== 减速区 ==========
        float slow_down_dist = 0.4f;
        float current_speed = cmd_speed;
        if (remaining < slow_down_dist && remaining > 0) {
            float speed_ratio = std::max(0.3f, remaining / slow_down_dist);
            current_speed = cmd_speed * speed_ratio;
            ROS_INFO_THROTTLE(1.0, "减速区: 剩余%.2fm, 速度%.3fm/s", remaining, current_speed);
        }
        
        // ========== 提前关泵 ==========
        float pump_off_dist = 0.3f;
        if (pump_on && !pump_closed_early && remaining < pump_off_dist && remaining > 0) {
            pump_on = false;
            pump_closed_early = true;
            current_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地，水泵已关闭");
        }
        
        // ========== 角度纠偏（使用累积角度） ==========
        float relative_theta = total_yaw_change;
        
        // 行走2秒后开始启用纠偏
        if (!correction_enabled && (ros::Time::now() - correction_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动连续按键纠偏, 当前累积偏角=%.1f°", relative_theta * RAD_TO_DEG);
        }
        
        if (correction_enabled && !wall_avoiding) {
            // 每秒更新一次角速度
            if ((ros::Time::now() - last_correction_time).toSec() > 0.8) {  // 提高到0.8秒响应更快
                if (move_direction == 'w') {  // 前进模式
                    if (relative_theta < -0.03f) {  // 向右偏（降低阈值到1.7度）
                        current_angular_correction = 0.20f;  // 左转
                        ROS_WARN_THROTTLE(1.0, "→ 向右偏 %.1f°，左转", 
                                          -relative_theta * RAD_TO_DEG);
                    }
                    else if (relative_theta > 0.03f) {  // 向左偏
                        current_angular_correction = -0.20f;  // 右转
                        ROS_WARN_THROTTLE(1.0, "→ 向左偏 %.1f°，右转", 
                                          relative_theta * RAD_TO_DEG);
                    }
                    else {
                        current_angular_correction = 0.0f;
                    }
                }
                else {  // 后退模式
                    float actual_error = -relative_theta;
                    if (actual_error < -0.03f) {
                        current_angular_correction = -0.20f;
                    } else if (actual_error > 0.03f) {
                        current_angular_correction = 0.20f;
                    } else {
                        current_angular_correction = 0.0f;
                    }
                }
                last_correction_time = ros::Time::now();
            }
        }
        
        // 小角度时逐渐减小角速度，避免过冲
        if (fabs(relative_theta) < 0.05f && fabs(current_angular_correction) > 0.05f) {
            current_angular_correction *= 0.5f;
        }
        
        // ========== 发送移动命令 ==========
        geometry_msgs::Twist cmd;
        cmd.linear.x = (move_direction == 'w') ? current_speed : -current_speed;
        cmd.linear.y = pump_on ? pump_speed : 0.0f;
        cmd.angular.z = current_angular_correction;
        cmd_vel_pub_.publish(cmd);
        
        // 进度打印
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0) {
            float progress = (total_traveled / fabs(distance)) * 100.0f;
            ROS_INFO("进度: %.1f%%, 已走%.2f/%.2f米, 累积偏角:%.1f°, 角速度:%.2f", 
                     progress, total_traveled, fabs(distance), 
                     relative_theta * RAD_TO_DEG, current_angular_correction);
            last_print = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // ========== 停车回正（闭环控制） ==========
    stop();
    ros::Duration(0.3).sleep();
    updateCurrentPose();
    
    float final_error = normalizeAngle(current_pose_.theta - start_yaw - total_yaw_change);
    if (fabs(final_error) > 0.035f) {  // 大于2度
        ROS_WARN("停车后偏转 %.1f°，主动回正", final_error * RAD_TO_DEG);
        
        geometry_msgs::Twist rotate_cmd;
        rotate_cmd.linear.x = 0.0;
        ros::Time rotate_start = ros::Time::now();
        
        while (ros::ok()) {
            updateCurrentPose();
            float current_error = normalizeAngle(current_pose_.theta - start_yaw - total_yaw_change);
            
            if (fabs(current_error) < 0.025f) break;  // 小于1.4度退出
            if ((ros::Time::now() - rotate_start).toSec() > 2.0) break;
            
            // 动态角速度
            float angular_speed = (current_error > 0) ? -0.12f : 0.12f;
            if (fabs(current_error) < 0.05f) angular_speed *= 0.5f;
            
            rotate_cmd.angular.z = angular_speed;
            cmd_vel_pub_.publish(rotate_cmd);
            ros::Duration(0.03).sleep();
            ros::spinOnce();
        }
        
        rotate_cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(rotate_cmd);
        ROS_WARN("回正完成");
    }
    
    // 最终停止并关泵
    stop();
    if (pump_on) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.0f;
        cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(cmd);
        current_pump_speed_ = 0.0f;
        ROS_INFO("水泵已关闭");
    }
    
    // 恢复键盘定时器
    disabling_timer_ = false;
    
    walk_state_ = success ? STATE_IDLE : STATE_ERROR;
    return success;
}
*****/

bool RobotWalker::walkStraightWithWallAvoidanceAdvanced(float distance, float max_speed, float timeout) {
    if (walk_state_ != STATE_IDLE) {
        ROS_ERROR("机器人正在行走中，无法开始新的行走");
        return false;
    }

    
    ROS_ERROR("========================================");
    ROS_ERROR("高级直线行走开始（带行走中池壁检测与避让）");
    ROS_ERROR("  距离: %.3f米", distance);
    ROS_ERROR("  速度: %.3f米/秒", max_speed);
    ROS_ERROR("  超时: %.1f秒", timeout);
    ROS_ERROR("========================================");

    // ========== 初始化行走日志 ==========
    initWalkLog("advanced_walk", fabs(distance));
    
    walk_state_ = STATE_WALKING;
    disabling_timer_ = true;
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    const float SPEED_CALIBRATION = 0.995f;
    
    float total_target = fabs(distance);
    char move_direction = (distance >= 0) ? 'w' : 'x';
    float desired_speed = std::min(max_speed, 0.5f);
    float cmd_speed = desired_speed / SPEED_CALIBRATION;
    if (cmd_speed > 0.5f) cmd_speed = 0.5f;
    
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待odom数据超时");
        disabling_timer_ = false;
        walk_state_ = STATE_ERROR;
        return false;
    }
    
    for (int i = 0; i < 5; i++) {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    updateCurrentPose();
    
    // 记录起点（仅用于角度参考）
    geometry_msgs::Pose2D start_pose = current_pose_;
    float start_yaw = current_pose_.theta;
    
    // ========== 新增：累积路程变量 ==========
    float total_accumulated_distance = 0.0f;
    geometry_msgs::Pose2D last_pose = current_pose_;
    
    ROS_ERROR("起始状态: 位置(%.2f,%.2f), 角度=%.1f°", 
              start_pose.x, start_pose.y, start_yaw * RAD_TO_DEG);
    
    float pump_speed = current_pump_speed_;   
    bool pump_on = (pump_speed > 0.01f);
    if (pump_on) {
        geometry_msgs::Twist pump_cmd;
        pump_cmd.linear.x = cmd_speed;
        pump_cmd.linear.y = pump_speed;
        pump_cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(pump_cmd);
        ROS_INFO("水泵已开启，速度: %.0f%%", pump_speed * 100);
        ros::Duration(0.3).sleep();
    }
    
    ros::Time start_time = ros::Time::now();
    bool success = false;
    bool wall_avoiding = false;
    int front_wall_detect_count = 0;
    bool pump_closed_early = false;
    
    // 角度累积变量
    float total_yaw_change = 0.0f;
    float last_theta = start_yaw;
    
    // 纠偏相关变量
    ros::Time correction_start_time = ros::Time::now();
    bool correction_enabled = false;
    ros::Time last_correction_time = ros::Time::now();
    float current_angular_correction = 0.0f;
    
    while (ros::ok() && walk_state_ == STATE_WALKING) {
        updateCurrentPose();
        
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("行走超时");
            break;
        }
        
        // ========== 修改：使用累积路程（只增不减） ==========
        float dx = current_pose_.x - last_pose.x;
        float dy = current_pose_.y - last_pose.y;
        float delta = sqrt(dx*dx + dy*dy);

        if (delta > 0.001f && delta < 0.5f) {
            total_accumulated_distance += delta;
        }

        last_pose = current_pose_;
        float remaining = total_target - total_accumulated_distance;
        
        // 累积角度变化（用于纠偏）
        float delta_yaw = normalizeAngle(current_pose_.theta - last_theta);
        total_yaw_change += delta_yaw;
        last_theta = current_pose_.theta;

        // ========== 每秒记录日志 ==========
        recordWalkData(total_accumulated_distance,total_target);
        
                // 更新日志缓冲区中的目标距离
        if (!log_buffer_.empty()) {
            log_buffer_.back().target_distance = total_target;
        }
        
        
        // ==================== 池壁检测 ====================
        if (!wall_avoiding && move_direction == 'w') {
            float front_left_dist = infra_front_left_;   // 前左超声波
            float front_right_dist = infra_front_right_; // 前右超声波
            float front_dist = std::min(front_left_dist, front_right_dist);
            // 前方超声波有效范围改为 0.3m ~ 1.6m
 
            bool has_front_wall = (front_dist > 0.5f && front_dist < 1.6f);
            
            if (has_front_wall) {
                front_wall_detect_count++;
                if (front_wall_detect_count >= 3) {
                    ROS_WARN("行走中检测到前方池壁(%.2f米)，执行避让！", front_dist);
                    wall_avoiding = true;
                    
                    // ----- 1. 停止 -----
                    stop();
                    ros::Duration(0.3).sleep();
                    ros::spinOnce();
                    // updateCurrentPose();
                    
                    // ========== 等待0.5秒 ==========
                    ROS_INFO("等待0.5秒后开始旋转...");
                    ros::Duration(1.5).sleep();
                    ros::spinOnce();
                    updateCurrentPose();
                    // ----- 3. 左转90度 -----
                    ROS_INFO("左转90度");
                    
                    // ----- 2. 左转直到右侧平行 -----
                                
                    ROS_INFO("左转至右侧平行于池壁");
                    float turn_start = current_pose_.theta;
                    ros::Time turn_start_time = ros::Time::now();
                    bool parallel_check_enabled = false;  // 是否启用平行度检测
                    
                    while (ros::ok()) {
                        ros::spinOnce();
                        updateCurrentPose();
                        
                        // 计算已转角度
                        float yaw_change = current_pose_.theta - turn_start;
                        if (yaw_change > M_PI) yaw_change -= 2 * M_PI;
                        float abs_yaw = fabs(yaw_change);
                        
                        // 超时保护
                        if ((ros::Time::now() - turn_start_time).toSec() > 8.0f) {
                            stop();
                            ROS_WARN("旋转超时，强制停止");
                            break;
                        }
                        
                        // ========== 阶段1：纯旋转，不做平行检测 ==========
                        if (abs_yaw < 70.0f * DEG_TO_RAD) {
                            geometry_msgs::Twist cmd;
                            cmd.angular.z = 0.25f;
                            cmd_vel_pub_.publish(cmd);
                            ros::Duration(0.05).sleep();
                            continue;
                        }
                        
                        // ========== 阶段2：超过60度，启用平行检测 ==========
                        if (!parallel_check_enabled) {
                            parallel_check_enabled = true;
                            ROS_INFO("已转60°，启用平行度检测");
                        }
                        
                        float front = right_front_dist_;
                        float rear = right_rear_dist_;
                        
                        bool data_valid = (front > 0.05f && front < 5.0f && 
                                        rear > 0.05f && rear < 5.0f);
                        
                        if (data_valid) {
                            float parallel_error = rear - front;
                            
                            ROS_INFO_THROTTLE(0.3, "旋转对齐: 前=%.2f 后=%.2f 误差=%.2f 角度=%.1f°", 
                                            front, rear, parallel_error, abs_yaw * RAD_TO_DEG);
                            
                            if (fabs(parallel_error) < 0.02f) {
                                stop();
                                ROS_INFO("平行度达标，停止旋转 (转过%.1f°)", abs_yaw * RAD_TO_DEG);
                                break;
                            }
                            
        
                            // parallel_error > 0 (后远前近) → 机头朝右 → 需要左转 → 正角速度
                            // parallel_error < 0 (后近前远) → 机头朝左 → 需要右转 → 负角速度
                            float base_speed = (fabs(parallel_error) < 0.1f) ? 0.08f : 0.15f;
                            float angular_speed = (parallel_error > 0) ? base_speed : -base_speed;
                            geometry_msgs::Twist cmd;
                            cmd.angular.z = angular_speed;
                            cmd_vel_pub_.publish(cmd);
                        } else {
                            // 无有效数据，继续旋转并用角度兜底
                            if (abs_yaw > 88.0f * DEG_TO_RAD) {
                                stop();
                                ROS_WARN("兜底：累积转到88°停止");
                                break;
                            }
                            
                            geometry_msgs::Twist cmd;
                            cmd.angular.z = 0.15f;
                            cmd_vel_pub_.publish(cmd);
                        }
                        
                        ros::Duration(0.05).sleep();
                    }
 

                    
                    ros::Duration(0.3).sleep();
                    
                    // ----- 4. 平行度对齐 -----
                    ROS_INFO("右侧平行度对齐");
                    alignToRightWallOnly();
                    ros::Duration(0.3).sleep();
                    
                    // ----- 5. 重置状态（不重置累积路程）-----
                    total_yaw_change = 0.0f;
                    last_theta = current_pose_.theta;
                    start_time = ros::Time::now();
                    wall_avoiding = false;
                    front_wall_detect_count = 0;
                    correction_enabled = false;
                    correction_start_time = ros::Time::now();
                    current_angular_correction = 0.0f;
                    // 注意：不重置 total_accumulated_distance 和 last_pose！
                    
                    ROS_INFO("避让完成，已走: %.2f米, 剩余: %.2f米, 当前角度: %.1f°", 
                            total_accumulated_distance, remaining, current_pose_.theta * RAD_TO_DEG);
                    continue;
                }
            } else {
                front_wall_detect_count = 0;
            }
        }
        
        // ==================== 到达目标 ====================
        if (remaining < 0.05f) {
            success = true;
            ROS_ERROR("✅ 行走完成！总路程: %.3f米", total_accumulated_distance);
            break;
        }
        
        // ==================== 减速区 ====================
        float slow_down_dist = 0.4f;
        float current_speed = cmd_speed;
        if (remaining < slow_down_dist && remaining > 0) {
            current_speed = cmd_speed * std::max(0.3f, remaining / slow_down_dist);
        }
        
        // ==================== 提前关泵 ====================
        if (pump_on && !pump_closed_early && remaining < 0.3f && remaining > 0) {
            pump_on = false;
            pump_closed_early = true;
            current_pump_speed_ = 0.0f;
            ROS_INFO("快到目的地，水泵已关闭");
        }
        
 
        // ==================== 角度纠偏 ====================
        float relative_theta = total_yaw_change;

        if (!correction_enabled && (ros::Time::now() - correction_start_time).toSec() > 2.0) {
            correction_enabled = true;
            ROS_WARN("启动连续按键纠偏, 当前累积偏角=%.1f°", relative_theta * RAD_TO_DEG);
        }

        if (correction_enabled && !wall_avoiding) {
            if (move_direction == 'w') {
                // 修正：relative_theta > 0 表示实际左转，需要右转（负角速度）
                if (relative_theta > 0.02f) {  // 实际左转
                    current_angular_correction = -std::min(0.35f, relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 左转 %.1f°，右转速度=%.2f", 
                                    relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else if (relative_theta < -0.02f) {  // 实际右转
                    current_angular_correction = std::min(0.35f, -relative_theta * 2.0f);
                    ROS_INFO_THROTTLE(0.5, "→ 右转 %.1f°，左转速度=%.2f", 
                                    -relative_theta * RAD_TO_DEG, current_angular_correction);
                }
                else {
                    current_angular_correction = 0.0f;
                }
            }
            else {  // 后退模式
                float actual_error = -relative_theta;
                if (actual_error > 0.02f) {  // 后退时左转
                    current_angular_correction = -std::min(0.35f, actual_error * 2.0f);
                } else if (actual_error < -0.02f) {  // 后退时右转
                    current_angular_correction = std::min(0.35f, -actual_error * 2.0f);
                } else {
                    current_angular_correction = 0.0f;
                }
            }
            
            // 微小误差时衰减
            if (fabs(relative_theta) < 0.03f && fabs(current_angular_correction) > 0.05f) {
                current_angular_correction *= 0.5f;
            }
        }

 
        
        // ========== 发送移动命令 ==========
        geometry_msgs::Twist cmd;
        cmd.linear.x = (move_direction == 'w') ? current_speed : -current_speed;
        cmd.linear.y = pump_on ? pump_speed : 0.0f;
        cmd.angular.z = current_angular_correction;
        cmd_vel_pub_.publish(cmd);
        

        
        // 进度打印
        static ros::Time last_print = ros::Time::now();
        if ((ros::Time::now() - last_print).toSec() > 1.0) {
            float progress = (total_accumulated_distance / total_target) * 100.0f;
            ROS_INFO("进度: %.1f%%, 已走%.2f/%.2f米, 偏角:%.1f°", 
                     progress, total_accumulated_distance, total_target, relative_theta * RAD_TO_DEG);
            last_print = ros::Time::now();
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // ==================== 结束处理 ====================
    stop();
    if (pump_on) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = 0.0f;
        cmd.angular.z = 0.0f;
        cmd_vel_pub_.publish(cmd);
        current_pump_speed_ = 0.0f;
    }
    
 
    
    disabling_timer_ = false;
    walk_state_ = success ? STATE_IDLE : STATE_ERROR;

    // ========== 保存行走日志 ==========
    saveWalkLog(success);
    return success;
}


bool RobotWalker::alignToRightWallWithDistance(float target_distance) {
    ROS_INFO("开始右侧对齐，目标距离: %.2f米", target_distance);
    
    // 等待超声波数据有效
    ros::Time wait_start = ros::Time::now();
    while (ros::ok() && (right_front_dist_ > 6.0f || right_rear_dist_ > 6.0f)) {
        if ((ros::Time::now() - wait_start).toSec() > 5.0f) {
            ROS_WARN("等待超声波数据超时");
            return false;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        updateCurrentPose();
    }
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    
    int step = 0;           // 对齐步骤
    float gap = 0.0f;       // 距离差距
    bool turn_started = false;
    float turn_start_yaw = 0.0f;
    ros::Time turn_start_time;
    ros::Time move_start_time;
    bool move_started = false;
    float pump_speed = current_pump_speed_;
    
 
    while (ros::ok()) {
        updateCurrentPose();
        
        float right_front = right_front_dist_;
        float right_rear = right_rear_dist_;
        float right_avg = (right_front + right_rear) / 2.0f;
        float parallel_error = right_rear - right_front;
        
        ROS_INFO_THROTTLE(0.5, "对齐步骤%d: 前=%.2f 后=%.2f 平行=%.2f 平均=%.2f", 
                          step, right_front, right_rear, parallel_error, right_avg);
        if (right_front > 5.0f || right_rear > 5.0f) {
            ROS_WARN("右侧无有效池壁数据，使用前方红外调整距离");
            // 使用前方红外距离作为参考
            float front_dist = std::min(infra_front_left_, infra_front_right_);
            if (front_dist > 0.1f && front_dist < 1.0f) {
                gap = target_distance - front_dist;  // 用前方距离
                step = 1;
                continue;
            } else {
                return true;  // 无有效数据，跳过
            }
        }
        // step0: 调平行度
        if (step == 0) {
            if (fabs(parallel_error) < 0.02f) {
                gap = target_distance - right_avg;
                ROS_INFO("平行达标，距离差距=%.2f", gap);
                
                if (fabs(gap) < 0.05f) {
                    ROS_INFO("对齐完成");
                    return true;
                }
                step = 1;
                ROS_INFO("需要%s%.2f米", gap > 0 ? "前进" : "后退", fabs(gap));
                continue;
            }
            
            // 调整平行度
            if (parallel_error > 0) {
                applyKeyboardCommand('a');  // 前近后远 → 左转
                ROS_INFO("前近后远，左转");
            } else {
                applyKeyboardCommand('d');  // 前远后近 → 右转
                ROS_INFO("前远后近，右转");
            }
            ros::Duration(0.3f).sleep();
            stop();
            ros::Duration(0.1f).sleep();
            continue;
        }
        
        // step1: 左转90度（调整距离前需要转向）
        if (step == 1) {
            if (!turn_started) {
                turn_start_yaw = current_pose_.theta;
                turn_start_time = ros::Time::now();
                turn_started = true;
                ROS_INFO("开始左转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
            }
            
            float yaw_change = normalizeAngle(current_pose_.theta - turn_start_yaw);
            
            if (yaw_change > 85.0f * DEG_TO_RAD || 
                (ros::Time::now() - turn_start_time).toSec() > 5.0f) {
                stop();
                step = 2;
                turn_started = false;
                ROS_INFO("左转完成: 累计转角=%.1f°", yaw_change * RAD_TO_DEG);
            } else {
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0f;
                cmd.linear.y = pump_speed;
                cmd.angular.z = 0.25f;
                cmd_vel_pub_.publish(cmd);
            }
            ros::Duration(0.05).sleep();
            continue;
        }
        
        // step2: 移动差距距离
        if (step == 2) {
            if (!move_started) {
                move_start_time = ros::Time::now();
                move_started = true;
                ROS_INFO("开始移动%.3f米", fabs(gap));
            }
            
            float elapsed = (ros::Time::now() - move_start_time).toSec();
            float speed = 0.15f;  // 移动速度
            float traveled = speed * elapsed;
            float target = fabs(gap);
            
            ROS_INFO_THROTTLE(0.3, "移动: 已走%.3f/%.3f米", traveled, target);
            
            if (traveled >= target) {
                stop();
                step = 3;
                move_started = false;
                ROS_INFO("移动%.3f米完成", target);
            } else if (gap > 0) {
                applyKeyboardCommand('w');  // 前进
            } else {
                applyKeyboardCommand('x');  // 后退
            }
            ros::Duration(0.05).sleep();
            continue;
        }
        
        // step3: 右转90度
        if (step == 3) {
            if (!turn_started) {
                turn_start_yaw = current_pose_.theta;
                turn_start_time = ros::Time::now();
                turn_started = true;
                ROS_INFO("开始右转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
            }
            
            float yaw_change = normalizeAngle(current_pose_.theta - turn_start_yaw);
            float abs_yaw_change = fabs(yaw_change);
            
            if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
                (ros::Time::now() - turn_start_time).toSec() > 5.0f) {
                stop();
                step = 4;
                turn_started = false;
                ROS_INFO("右转完成: 累计转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
            } else {
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0f;
                cmd.linear.y = pump_speed;
                cmd.angular.z = -0.25f;
                cmd_vel_pub_.publish(cmd);
            }
            ros::Duration(0.05).sleep();
            continue;
        }
        
        // step4: 最终平行度检查
        if (step == 4) {
            if (fabs(parallel_error) < 0.02f) {
                ROS_INFO("最终对齐完成");
                return true;
            }
            
            // 平行度不达标，调整
            if (parallel_error > 0) {
                applyKeyboardCommand('a');
                ROS_INFO("最终对齐: 前近后远，左转");
            } else {
                applyKeyboardCommand('d');
                ROS_INFO("最终对齐: 前远后近，右转");
            }
            ros::Duration(0.3f).sleep();
            stop();
            ros::Duration(0.1f).sleep();
            continue;
        }
    }
    
    return false;
}

// 只调整平行度（不调整距离）
/*******
void RobotWalker::alignToRightWallOnly() {
    const float TOLERANCE = 0.015f;  // 放宽到0.015 rad (约0.86°)，避免无限震荡
    ros::Time start_time = ros::Time::now();
    float pump_speed = current_pump_speed_;
    
    ROS_INFO("开始平行度对齐（连续PID控制）");
    
    // 保存并临时禁用键盘定时器
    bool old_disabling_state = disabling_timer_;
    if (!old_disabling_state) {
        disabling_timer_ = true;
    }
    
    // 传感器纵向距离
    const float SENSOR_SPACING = 0.14f;
    
    // PID参数
    const float KP = 2.5f;      // 增大比例增益，响应更快
    const float MAX_SPEED = 0.25f;  // 最大角速度 0.25 rad/s ≈ 14°/s
    
    // 积分项（消除静差）
    static float integral = 0.0f;
    integral = 0.0f;  // 每次对齐重置积分
    
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > 8.0f) {
            ROS_WARN("平行度对齐超时");
            break;
        }
        
        updateCurrentPose();
        
        float front = right_front_dist_;
        float rear = right_rear_dist_;
        
        // 检查数据有效性
        if (front > 5.0f || rear > 5.0f) {
            ROS_WARN_THROTTLE(1.0, "超声波数据无效，等待...");
            ros::Duration(0.05).sleep();
            continue;
        }
        
        // 计算平行度误差（弧度）
        float diff = rear - front;
        float angle_error = atan2(diff, SENSOR_SPACING);
        
        // 打印详细日志
        ROS_INFO("对齐: 前=%.3f, 后=%.3f, 误差=%.3fm, 角度=%.2f°", 
                 front, rear, diff, angle_error * 180.0f / M_PI);
        
        // 平行度达标
        if (fabs(angle_error) < TOLERANCE) {
            stop();
            ROS_INFO("平行度对齐完成 (误差=%.2f°)", angle_error * 180.0f / M_PI);
            break;
        }
        
        // P控制 + 微分项（减小过冲）
        float angular_speed = angle_error * KP;
        
        // 小误差时增加死区，避免震荡
        if (fabs(angle_error) < 0.03f) {
            angular_speed = angle_error * KP * 0.5f;  // 接近目标时减速
        }
        
        // 限制最大角速度
        angular_speed = std::max(-MAX_SPEED, std::min(MAX_SPEED, angular_speed));
        
        // 如果误差很小但角速度也很小，直接走最短时间
        static float last_error = 0.0f;
        float error_change = fabs(angle_error - last_error);
        last_error = angle_error;
        
        // 如果误差基本不变（卡住），强制增加角速度
        static int stuck_count = 0;
        if (error_change < 0.002f && fabs(angle_error) > TOLERANCE) {
            stuck_count++;
            if (stuck_count > 20) {  // 约1秒没变化
                angular_speed = (angle_error > 0) ? MAX_SPEED : -MAX_SPEED;
                ROS_WARN("检测到卡住，强制旋转");
                stuck_count = 0;
            }
        } else {
            stuck_count = 0;
        }
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = pump_speed;
        cmd.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd);
        
        // 缩短控制周期，响应更快
        ros::Duration(0.03).sleep();
        ros::spinOnce();
    }
    
    stop();
    disabling_timer_ = old_disabling_state;
}


void RobotWalker::alignToRightWallOnly() {
    const float TOLERANCE = 0.015f;  // 放宽到0.015 rad (约0.86°)，避免无限震荡
    ros::Time start_time = ros::Time::now();
    float pump_speed = current_pump_speed_;
    
    ROS_INFO("开始平行度对齐（增强版：支持超声波异常检测）");
    
    // 保存并临时禁用键盘定时器
    bool old_disabling_state = disabling_timer_;
    if (!old_disabling_state) {
        disabling_timer_ = true;
    }
    
    // 传感器纵向距离
    const float SENSOR_SPACING = 0.14f;
    
    // PID参数
    const float KP = 2.5f;      // 增大比例增益，响应更快
    const float MAX_SPEED = 0.25f;  // 最大角速度 0.25 rad/s ≈ 14°/s
    
    // 积分项（消除静差）
    static float integral = 0.0f;
    integral = 0.0f;  // 每次对齐重置积分
    
    // 辅助函数：判断超声波数据是否有效
    auto isValid = [](float dist) -> bool {
        return (dist > 0.05f && dist < 5.0f);  // 有效范围：0.05m ~ 5.0m
    };
    
    // 异常状态标志
    enum BiasState {
        NORMAL = 0,      // 两个都有效
        BIAS_LEFT,       // 偏左（右前无效，右后有效）
        BIAS_RIGHT,      // 偏右（右前有效，右后无效）
        NO_WALL          // 远离池壁（两个都无效）
    };
    
    BiasState bias_state = NORMAL;
    ros::Time last_bias_log = ros::Time::now();
    
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > 8.0f) {
            ROS_WARN("平行度对齐超时");
            break;
        }
        
        updateCurrentPose();
        
        float front = right_front_dist_;
        float rear = right_rear_dist_;
        
        bool front_valid = isValid(front);
        bool rear_valid = isValid(rear);
        
        // ========== 新增：超声波异常检测逻辑 ==========
        if (!front_valid && !rear_valid) {
            // 两个都无效 → 远离池壁
            bias_state = NO_WALL;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN(" 两个超声波都无效(front=%.1f, rear=%.1f)！可能远离池壁或角度过大", front, rear);
                last_bias_log = ros::Time::now();
            }
        } 
        else if (!front_valid && rear_valid) {
            // 右前无效、右后有效 → 偏左（机头朝左）
            bias_state = BIAS_LEFT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("检测到偏左：右前异常(%.1f), 右后正常(%.2f)", front, rear);
                last_bias_log = ros::Time::now();
            }
        }
        else if (front_valid && !rear_valid) {
            // 右前有效、右后无效 → 偏右（机头朝右）
            bias_state = BIAS_RIGHT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("检测到偏右：右前正常(%.2f), 右后异常(%.1f)", front, rear);
                last_bias_log = ros::Time::now();
            }
        }
        else {
            // 两个都有效 → 正常模式
            bias_state = NORMAL;
        }
        
        // ========== 根据状态执行不同策略 ==========
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = pump_speed;
        
        if (bias_state == NORMAL) {
            // ========== 正常模式：使用PID精确控制 ==========
            float diff = rear - front;
            float angle_error = atan2(diff, SENSOR_SPACING);
            
            // 打印详细日志
            ROS_INFO_THROTTLE(0.5, "对齐: 前=%.3f, 后=%.3f, 误差=%.3fm, 角度=%.2f°", 
                             front, rear, diff, angle_error * 180.0f / M_PI);
            
            // 平行度达标
            if (fabs(angle_error) < TOLERANCE) {
                stop();
                ROS_INFO("平行度对齐完成 (误差=%.2f°)", angle_error * 180.0f / M_PI);
                break;
            }
            
            // P控制 + 微分项（减小过冲）
            float angular_speed = angle_error * KP;
            
            // 小误差时增加死区，避免震荡
            if (fabs(angle_error) < 0.03f) {
                angular_speed = angle_error * KP * 0.5f;  // 接近目标时减速
            }
            
            // 限制最大角速度
            angular_speed = std::max(-MAX_SPEED, std::min(MAX_SPEED, angular_speed));
            
            // 如果误差基本不变（卡住），强制增加角速度
            static float last_error = 0.0f;
            float error_change = fabs(angle_error - last_error);
            last_error = angle_error;
            
            static int stuck_count = 0;
            if (error_change < 0.002f && fabs(angle_error) > TOLERANCE) {
                stuck_count++;
                if (stuck_count > 20) {  // 约1秒没变化
                    angular_speed = (angle_error > 0) ? MAX_SPEED : -MAX_SPEED;
                    ROS_WARN("检测到卡住，强制旋转");
                    stuck_count = 0;
                }
            } else {
                stuck_count = 0;
            }
            
            cmd.angular.z = angular_speed;
            cmd_vel_pub_.publish(cmd);
        }
        else if (bias_state == BIAS_LEFT) {
            // ========== 偏左：需要右转 ==========
            // 使用固定角速度右转，直到恢复正常
            cmd.angular.z = -0.15f;  // 右转
            cmd_vel_pub_.publish(cmd);
            
            // 短暂等待后继续循环，让传感器重新检测
            ros::Duration(0.05).sleep();
            ros::spinOnce();
            continue;
        }
        else if (bias_state == BIAS_RIGHT) {
            // ========== 偏右：需要左转 ==========
            // 使用固定角速度左转，直到恢复正常
            cmd.angular.z = 0.15f;   // 左转
            cmd_vel_pub_.publish(cmd);
            
            // 短暂等待后继续循环，让传感器重新检测
            ros::Duration(0.05).sleep();
            ros::spinOnce();
            continue;
        }
        else if (bias_state == NO_WALL) {
            // ========== 远离池壁：需要向池壁方向移动 ==========
            ROS_WARN_THROTTLE(1.0, "远离池壁，执行靠近动作");
            
            // 停止当前动作
            stop();
            ros::Duration(0.2).sleep();
            
            // 记录起始角度
            float start_yaw = current_pose_.theta;
            
            // 1. 右转45度
            ROS_INFO("右转45度");
            ros::Time turn_start = ros::Time::now();
            while (ros::ok()) {
                updateCurrentPose();
                float yaw_change = normalizeAngle(current_pose_.theta - start_yaw);
                if (fabs(yaw_change) > 45.0f * M_PI / 180.0f ||
                    (ros::Time::now() - turn_start).toSec() > 3.0f) {
                    break;
                }
                geometry_msgs::Twist turn_cmd;
                turn_cmd.linear.x = 0.0f;
                turn_cmd.linear.y = pump_speed;
                turn_cmd.angular.z = -0.2f;  // 右转
                cmd_vel_pub_.publish(turn_cmd);
                ros::Duration(0.05).sleep();
            }
            stop();
            ros::Duration(0.2).sleep();
            
            // 2. 前进1米
            ROS_INFO("前进靠近池壁");
            ros::Time move_start = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - move_start).toSec() < 2.0f) {
                geometry_msgs::Twist move_cmd;
                move_cmd.linear.x = 0.15f;
                move_cmd.linear.y = pump_speed;
                move_cmd.angular.z = 0.0f;
                cmd_vel_pub_.publish(move_cmd);
                ros::Duration(0.05).sleep();
            }
            stop();
            ros::Duration(0.2).sleep();
            
            // 3. 左转45度回正
            ROS_INFO("左转回正");
            turn_start = ros::Time::now();
            while (ros::ok()) {
                updateCurrentPose();
                float yaw_change = normalizeAngle(current_pose_.theta - start_yaw);
                if (fabs(yaw_change) < 5.0f * M_PI / 180.0f ||
                    (ros::Time::now() - turn_start).toSec() > 3.0f) {
                    break;
                }
                geometry_msgs::Twist turn_cmd;
                turn_cmd.linear.x = 0.0f;
                turn_cmd.linear.y = pump_speed;
                turn_cmd.angular.z = 0.2f;   // 左转
                cmd_vel_pub_.publish(turn_cmd);
                ros::Duration(0.05).sleep();
            }
            stop();
            ros::Duration(0.3).sleep();
            
            // 重置计时器，继续对齐循环
            start_time = ros::Time::now();
            continue;
        }
        
        // 缩短控制周期，响应更快
        ros::Duration(0.03).sleep();
        ros::spinOnce();
    }
    
    stop();
    disabling_timer_ = old_disabling_state;
}

void RobotWalker::alignToRightWallOnly() {
    const float TOLERANCE = 0.02f;
    ros::Time start_time = ros::Time::now();
    float pump_speed = current_pump_speed_;
    
    ROS_INFO("开始平行度对齐（修正版）");
    
    bool old_disabling_state = disabling_timer_;
    if (!old_disabling_state) {
        disabling_timer_ = true;
    }
    
    const float SENSOR_SPACING = 0.14f;
    const float KP = 1.5f;
    const float MAX_SPEED = 0.2f;
    
    // ========== 修改：超声波有效范围 0.05m ~ 5.0m ==========
    auto isValid = [](float dist) -> bool {
        return (dist > 0.05f && dist < 5.0f);  // 超声波统一使用这个范围
    };
    
    enum BiasState { NORMAL, BIAS_LEFT, BIAS_RIGHT, NO_WALL };
    BiasState bias_state = NORMAL;
    ros::Time last_bias_log = ros::Time::now();
    
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > 15.0f) {
            ROS_WARN("平行度对齐超时");
            break;
        }
        
        updateCurrentPose();
        
        float front = right_front_dist_;   // 右侧前超声波
        float rear = right_rear_dist_;     // 右侧后超声波
        
        bool front_valid = isValid(front);
        bool rear_valid = isValid(rear);
        
        // ========== 异常检测（只依赖右侧两个超声波） ==========
        if (!front_valid && !rear_valid) {
            bias_state = NO_WALL;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN(" 两个右侧超声波都无效(front=%.1f, rear=%.1f)，可能远离池壁", front, rear);
                last_bias_log = ros::Time::now();
            }
        } 
        else if (!front_valid && rear_valid) {
            bias_state = BIAS_LEFT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("偏左（右前无效，右后有效），需要右转");
                last_bias_log = ros::Time::now();
            }
        }
        else if (front_valid && !rear_valid) {
            bias_state = BIAS_RIGHT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("偏右（右前有效，右后无效），需要左转");
                last_bias_log = ros::Time::now();
            }
        }
        else {
            bias_state = NORMAL;
        }
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = pump_speed;
        
        if (bias_state == NORMAL) {
            float diff = rear - front;
            
            // diff > 0 (后远前近) → 机头朝右 → 需要左转 → 正角速度
            // diff < 0 (后近前远) → 机头朝左 → 需要右转 → 负角速度
            float angular_speed = 0.0f;
            
            if (fabs(diff) < 0.02f) {
                stop();
                ROS_INFO("平行度对齐完成 (前后差=%.3fm)", diff);
                break;
            }
            
            angular_speed = diff * KP;
            angular_speed = std::max(-MAX_SPEED, std::min(MAX_SPEED, angular_speed));
            
            ROS_INFO_THROTTLE(0.5, "对齐: 前=%.3f, 后=%.3f, 差=%.3fm, 角速度=%.2f", 
                             front, rear, diff, angular_speed);
            
            cmd.angular.z = angular_speed;
            cmd_vel_pub_.publish(cmd);
        }
        else if (bias_state == BIAS_LEFT) {
            // 偏左 → 右转
            cmd.angular.z = -0.12f;
            cmd_vel_pub_.publish(cmd);
            ROS_INFO_THROTTLE(0.5, "偏左，右转纠正");
        }
        else if (bias_state == BIAS_RIGHT) {
            // 偏右 → 左转
            cmd.angular.z = 0.12f;
            cmd_vel_pub_.publish(cmd);
            ROS_INFO_THROTTLE(0.5, "偏右，左转纠正");
        }
        else if (bias_state == NO_WALL) {
            ROS_WARN_THROTTLE(1.0, "远离池壁，停止对齐");
            break;
        }
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    stop();
    disabling_timer_ = old_disabling_state;
}
***/ 
void RobotWalker::alignToRightWallOnly() {
    const float TOLERANCE = 0.02f;
    ros::Time start_time = ros::Time::now();
    float pump_speed = current_pump_speed_;
    
    ROS_INFO("开始平行度对齐（移动平均滤波版）");
    
    bool old_disabling_state = disabling_timer_;
    if (!old_disabling_state) {
        disabling_timer_ = true;
    }
    
    const float SENSOR_SPACING = 0.14f;
    const float KP = 1.5f;
    const float MAX_SPEED = 0.2f;
    
    // ========== 新增：滤波缓冲区 ==========
    std::deque<float> front_buffer;
    std::deque<float> rear_buffer;
    const int WINDOW_SIZE = 3;  // 3点移动平均
    
    auto isValid = [](float dist) -> bool {
        return (dist > 0.05f && dist < 5.0f);
    };
    
    enum BiasState { NORMAL, BIAS_LEFT, BIAS_RIGHT, NO_WALL };
    BiasState bias_state = NORMAL;
    ros::Time last_bias_log = ros::Time::now();
    
    while (ros::ok()) {
        if ((ros::Time::now() - start_time).toSec() > 5.0f) {
            ROS_WARN("平行度对齐超时");
            break;
        }
        
        updateCurrentPose();
        
        float raw_front = right_front_dist_;
        float raw_rear = right_rear_dist_;
        
        // ========== 应用滤波 ==========
        float front = movingAverageFilter(raw_front, front_buffer, WINDOW_SIZE);
        float rear = movingAverageFilter(raw_rear, rear_buffer, WINDOW_SIZE);
        
        bool front_valid = isValid(raw_front);
        bool rear_valid = isValid(raw_rear);
        
        // ========== 异常检测 ==========
        if (!front_valid && !rear_valid) {
            bias_state = NO_WALL;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN(" 两个超声波都无效(front=%.1f, rear=%.1f)", raw_front, raw_rear);
                last_bias_log = ros::Time::now();
            }
        } 
        else if (!front_valid && rear_valid) {
            bias_state = BIAS_LEFT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("偏左（右前无效），右转");
                last_bias_log = ros::Time::now();
            }
        }
        else if (front_valid && !rear_valid) {
            bias_state = BIAS_RIGHT;
            if ((ros::Time::now() - last_bias_log).toSec() > 1.0) {
                ROS_WARN("偏右（右后无效），左转");
                last_bias_log = ros::Time::now();
            }
        }
        else {
            bias_state = NORMAL;
        }
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0f;
        cmd.linear.y = pump_speed;
        
        if (bias_state == NORMAL) {
            float diff = rear - front;
            
            if (fabs(diff) < 0.02f) {
                stop();
                ROS_INFO("平行度对齐完成 (前后差=%.3fm)", diff);
                break;
            }
            
            // diff > 0 → 左转，diff < 0 → 右转
            float angular_speed = diff * KP;
            angular_speed = std::max(-MAX_SPEED, std::min(MAX_SPEED, angular_speed));
            
            // 打印原始值和滤波后的值对比
            ROS_INFO_THROTTLE(0.5, "滤波前(%.3f,%.3f) → 滤波后(%.3f,%.3f) 差=%.3f 角速度=%.2f", 
                             raw_front, raw_rear, front, rear, diff, angular_speed);
            
            cmd.angular.z = angular_speed;
            cmd_vel_pub_.publish(cmd);
        }
        else if (bias_state == BIAS_LEFT) {
            cmd.angular.z = -0.12f;
            cmd_vel_pub_.publish(cmd);
            ROS_INFO_THROTTLE(0.5, "偏左，右转纠正");
        }
        else if (bias_state == BIAS_RIGHT) {
            cmd.angular.z = 0.12f;
            cmd_vel_pub_.publish(cmd);
            ROS_INFO_THROTTLE(0.5, "偏右，左转纠正");
        }
        else if (bias_state == NO_WALL) {
            ROS_WARN_THROTTLE(1.0, "远离池壁，停止对齐");
            break;
        }
        
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    }
    
    stop();
    disabling_timer_ = old_disabling_state;
}

bool RobotWalker::findPoolWall(float timeout) {
    ROS_INFO("========================================");
    ROS_INFO("寻池壁功能启动");
    ROS_INFO("  超时: %.1f秒", timeout);
    ROS_INFO("========================================");
    
    if (walk_state_ != STATE_IDLE) {
        ROS_ERROR("机器人正在行走中，无法执行寻池壁");
        return false;
    }
    
    walk_state_ = STATE_WALKING;
    
    // 等待传感器数据有效
    ros::Time wait_start = ros::Time::now();
    while (ros::ok() && !ultra_received_) {
        if ((ros::Time::now() - wait_start).toSec() > 5.0f) {
            ROS_WARN("等待传感器数据超时");
            walk_state_ = STATE_ERROR;
            return false;
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        updateCurrentPose();
    }
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    
    int total_rotated = 0;      // 已累积旋转角度
    int step = 0;               // 0:旋转扫描, 1:直行找壁
    bool turn_started = false;
    float turn_start_yaw = 0.0f;
    ros::Time turn_start_time;
    ros::Time move_start_time;
    bool move_started = false;
    float pump_speed = current_pump_speed_;
    
    ros::Time start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
        updateCurrentPose();
        
        float right_front = right_front_dist_;
        float right_rear = right_rear_dist_;
        float infra_left = infra_front_left_;
        float infra_right = infra_front_right_;
        
        bool front_detected = (infra_left > 0.08f && infra_left < 0.80f) ||
                              (infra_right > 0.08f && infra_right < 0.80f);
        bool right_detected = (right_front < 1.5f && right_front > 0.1f) ||
                              (right_rear < 1.5f && right_rear > 0.1f);
        
        ROS_INFO_THROTTLE(0.5, "寻壁状态: 步=%d, 累积旋转=%d°, 前红外=%.2f/%.2f, 右超声=%.2f/%.2f", 
                          step, total_rotated, infra_left, infra_right, right_front, right_rear);
        
        // ========== 步骤0: 旋转扫描模式 ==========
        if (step == 0) {
            // 检查是否检测到池壁
            if (front_detected || right_detected) {
                // 条件判断A: 区分检测来源
                if (front_detected) {
                    // 红外检测到（优先），已正对池壁
                    stop();
                    ROS_INFO("红外检测到池壁，寻壁成功");
                    walk_state_ = STATE_IDLE;
                    return true;
                } 
                else if (right_detected && !front_detected) {
                    // 仅右侧超声检测到
                    ROS_INFO("右侧超声检测到池壁，开始平行度对齐");
                    stop();
                    ros::Duration(0.3).sleep();
                    
                    // 平行度对齐
                    alignToRightWallOnly();
                    ros::Duration(0.5).sleep();
                    
                    // 继续右转90度，使机器人正对池壁
                    ROS_INFO("继续右转90度");
                    disabling_timer_ = true;
                    
                    turn_start_yaw = current_pose_.theta;
                    turn_start_time = ros::Time::now();
                    turn_started = true;
                    
                    while (ros::ok()) {
                        ros::spinOnce();
                        updateCurrentPose();
                        
                        float yaw_change = normalizeAngle(turn_start_yaw - current_pose_.theta);
                        float abs_yaw_change = fabs(yaw_change);
                        
                        if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
                            (ros::Time::now() - turn_start_time).toSec() > 10.0f) {
                            stop();
                            ROS_INFO("右转完成: 转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
                            break;
                        }
                        
                        geometry_msgs::Twist cmd;
                        cmd.linear.x = 0.0f;
                        cmd.linear.y = pump_speed;
                        cmd.angular.z = -0.25f;
                        cmd_vel_pub_.publish(cmd);
                        
                        ros::Duration(0.05).sleep();
                    }
                    
                    disabling_timer_ = false;
                    ros::Duration(0.3).sleep();
                    
                    // 直行靠近池壁，直到红外触发
                    ROS_INFO("直行靠近池壁，等待红外触发");
                    
                    // 设置前进速度
                    {
                        std::lock_guard<std::mutex> lock(keyboard_mutex_);
                        keyboard_sim_.target_vx = 0.15f;
                        keyboard_sim_.current_vx = 0.15f;
                        keyboard_sim_.target_angular = 0.0f;
                        keyboard_sim_.key_pressed = true;
                    }
                    
                    if (pump_speed > 0.01f) {
                        geometry_msgs::Twist pump_cmd;
                        pump_cmd.linear.x = 0.15f;
                        pump_cmd.linear.y = pump_speed;
                        cmd_vel_pub_.publish(pump_cmd);
                    }
                    
                    ros::Time approach_start = ros::Time::now();
                    bool front_triggered = false;
                    
                    while (ros::ok() && (ros::Time::now() - approach_start).toSec() < 15.0f) {
                        ros::spinOnce();
                        updateCurrentPose();
                        
                        infra_left = infra_front_left_;
                        infra_right = infra_front_right_;
                        if ((infra_left > 0.08f && infra_left < 0.80f) ||
                            (infra_right > 0.08f && infra_right < 0.80f)) {
                            front_triggered = true;
                            break;
                        }
                        
                        ros::Duration(0.05).sleep();
                    }
                    
                    stop();
                    
                    if (front_triggered) {
                        ROS_INFO("红外触发，寻壁成功");
                        walk_state_ = STATE_IDLE;
                        return true;
                    } else {
                        ROS_WARN("红外未触发，继续寻壁");
                        step = 0;
                        total_rotated = 0;
                        continue;
                    }
                }
            }
            else {
                // 未检测到池壁，继续旋转
                if (!turn_started) {
                    turn_start_yaw = current_pose_.theta;
                    turn_start_time = ros::Time::now();
                    turn_started = true;
                    ROS_INFO("开始右转90度扫描");
                }
                
                float yaw_change = normalizeAngle(turn_start_yaw - current_pose_.theta);
                float abs_yaw_change = fabs(yaw_change);
                
                if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
                    (ros::Time::now() - turn_start_time).toSec() > 15.0f) {
                    // 完成一次90度旋转
                    stop();
                    turn_started = false;
                    total_rotated += 90;
                    ROS_INFO("完成90度旋转，累积=%d°", total_rotated);
                    
                    if (total_rotated >= 180) {
                        // 累积180度未找到，改为直行模式
                        ROS_INFO("旋转180度未找到池壁，改为直行寻找");
                        step = 1;
                        total_rotated = 0;
                        move_started = false;
                    }
                    // 继续下一次旋转
                    ros::Duration(0.2).sleep();
                    continue;
                }
                
                // 执行旋转
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.0f;
                cmd.linear.y = pump_speed;
                cmd.angular.z = -0.25f;
                cmd_vel_pub_.publish(cmd);
                
                ros::Duration(0.05).sleep();
                continue;
            }
        }
        
        // ========== 步骤1: 直行找壁模式 ==========
        if (step == 1) {
            if (!move_started) {
                stop();
                ros::Duration(0.3).sleep();
                
                // 设置前进速度
                {
                    std::lock_guard<std::mutex> lock(keyboard_mutex_);
                    keyboard_sim_.target_vx = 0.15f;
                    keyboard_sim_.current_vx = 0.15f;
                    keyboard_sim_.target_angular = 0.0f;
                    keyboard_sim_.key_pressed = true;
                }
                
                if (pump_speed > 0.01f) {
                    geometry_msgs::Twist pump_cmd;
                    pump_cmd.linear.x = 0.15f;
                    pump_cmd.linear.y = pump_speed;
                    cmd_vel_pub_.publish(pump_cmd);
                }
                
                move_started = true;
                ROS_INFO("开始直行寻找池壁");
            }
            
            // 直行过程中检测池壁
            if (front_detected || right_detected) {
                stop();
                ROS_INFO("直行中检测到池壁，进入条件判断A");
                ros::Duration(0.3).sleep();
                
                // 重置step为0，让下一次循环进入条件判断A
                step = 0;
                turn_started = false;
                move_started = false;
                continue;
            }
            
            // 检查超时
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_ERROR("寻池壁超时");
                break;
            }
            
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    
    stop();
    walk_state_ = STATE_ERROR;
    ROS_ERROR("寻池壁失败");
    return false;
}

bool RobotWalker::smartAlignToWall(float target_distance, float timeout) {
    ROS_INFO("========================================");
    ROS_INFO("智能沿边对齐开始");
    ROS_INFO("  目标距离: %.2f米", target_distance);
    ROS_INFO("  超时: %.1f秒", timeout);
    ROS_INFO("========================================");
    
    // 保存并禁用键盘定时器
    bool old_disabling_state = disabling_timer_;
    disabling_timer_ = true;
    
    const float DEG_TO_RAD = M_PI / 180.0f;
    const float RAD_TO_DEG = 180.0f / M_PI;
    
    // 等待传感器数据有效
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 3.0f) {
        ros::spinOnce();
        updateCurrentPose();
        if (ultra_received_) break;
        ros::Duration(0.1).sleep();
    }
    
    // ========== 1. 环境检测 ==========
    bool has_right_wall = (right_front_dist_ > 0.1f && right_front_dist_ < 3.0f) ||
                          (right_rear_dist_ > 0.1f && right_rear_dist_ < 3.0f);
    bool has_front_wall = (infra_front_left_ > 0.1f && infra_front_left_ < 1.0f) ||
                          (infra_front_right_ > 0.1f && infra_front_right_ < 1.0f);
    
    float right_dist = (right_front_dist_ + right_rear_dist_) / 2.0f;
    float front_dist = std::min(infra_front_left_, infra_front_right_);
    
    ROS_INFO("环境检测: 右侧池壁=%s (%.2fm), 前方池壁=%s (%.2fm)", 
             has_right_wall ? "有" : "无", right_dist,
             has_front_wall ? "有" : "无", front_dist);
    
    // 决策变量
    int state = 0;  // 0:检测, 1:旋转找壁, 2:平行对齐, 3:调整距离, 4:转向池壁, 5:完成
    bool success = false;
    float pump_speed = current_pump_speed_;
    
    // 决策：根据传感器情况选择策略
    if (has_front_wall && has_right_wall) {
        // 场景1：角落（前+右都有池壁）
        ROS_INFO("场景: 角落检测到（前方+右侧都有池壁）");
        state = 4;  // STATE_TURN_TO_WALL
    } 
    else if (has_right_wall && !has_front_wall) {
        // 场景2：边缘中间（只有右侧池壁）
        ROS_INFO("场景: 边缘中间（仅右侧有池壁）");
        
        // 检查距离是否合适
        if (right_dist > target_distance + 0.1f) {
            ROS_INFO("距离太远(%.2fm > %.2fm)，需要靠近", right_dist, target_distance);
            state = 3;  // STATE_ADJUST_DISTANCE
        } 
        else if (right_dist < target_distance - 0.1f) {
            ROS_INFO("距离太近(%.2fm < %.2fm)，需要远离", right_dist, target_distance);
            state = 3;  // STATE_ADJUST_DISTANCE
        }
        else {
            ROS_INFO("距离合适，进行平行度对齐");
            state = 2;  // STATE_ALIGN_PARALLEL
        }
    }
    else {
        // 场景3：空旷（无池壁），需要旋转寻找
        ROS_INFO("场景: 空旷区域，开始旋转寻找池壁");
        state = 1;  // STATE_ROTATE_TO_WALL
    }
    
    // ========== 2. 状态机主循环 ==========
    ros::Time state_start_time = ros::Time::now();
    float turn_start_yaw = 0.0f;
    bool turn_started = false;
    float target_gap = 0.0f;
    
    // 在循环外声明的变量，避免 switch 跨初始化问题
    float yaw_change = 0.0f;
    float abs_yaw_change = 0.0f;
    float current_dist = 0.0f;
    float error = 0.0f;
    float move_speed = 0.0f;
    float angle_error = 0.0f;
    float angular_speed = 0.0f;
    ros::Time turn_start_time;
    
    while (ros::ok() && state != 5) {
        // 超时检查
        if ((ros::Time::now() - start_time).toSec() > timeout) {
            ROS_ERROR("智能对齐超时");
            break;
        }
        
        updateCurrentPose();
        
        // 更新传感器数据
        has_right_wall = (right_front_dist_ > 0.1f && right_front_dist_ < 3.0f) ||
                         (right_rear_dist_ > 0.1f && right_rear_dist_ < 3.0f);
        has_front_wall = (infra_front_left_ > 0.1f && infra_front_left_ < 1.0f) ||
                         (infra_front_right_ > 0.1f && infra_front_right_ < 1.0f);
        right_dist = (right_front_dist_ + right_rear_dist_) / 2.0f;
        front_dist = std::min(infra_front_left_, infra_front_right_);
        
        float parallel_error = right_rear_dist_ - right_front_dist_;
        
        switch (state) {
            case 0:  // STATE_DETECT
                state = 5;
                break;
                
            case 1:  // STATE_ROTATE_TO_WALL - 右转寻找池壁
                if (!turn_started) {
                    turn_start_yaw = current_pose_.theta;
                    turn_start_time = ros::Time::now();
                    turn_started = true;
                    ROS_INFO("开始右转寻找池壁");
                }
                
                // 检查是否检测到池壁
                if (has_right_wall || has_front_wall) {
                    stop();
                    ROS_INFO("检测到池壁，停止旋转");
                    state = 2;  // 转到平行对齐
                    turn_started = false;
                    break;
                }
                
                // 旋转超时（转360度没找到）
                if ((ros::Time::now() - turn_start_time).toSec() > 12.0f) {
                    ROS_WARN("旋转360度未找到池壁");
                    state = 5;
                    success = false;
                    break;
                }
                
                // 执行右转
                {
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = 0.0f;
                    cmd.linear.y = pump_speed;
                    cmd.angular.z = -0.25f;  // 右转
                    cmd_vel_pub_.publish(cmd);
                }
                break;
                
            case 2:  // STATE_ALIGN_PARALLEL - 平行度对齐
                if (!has_right_wall) {
                    ROS_WARN("右侧池壁丢失，重新检测");
                    state = 0;
                    break;
                }
                
                if (fabs(parallel_error) < 0.015f) {
                    stop();
                    ROS_INFO("平行度对齐完成");
                    
                    // 检查距离是否需要调整
                    if (fabs(right_dist - target_distance) > 0.05f) {
                        target_gap = target_distance - right_dist;
                        ROS_INFO("需要调整距离: %.2fm", target_gap);
                        state = 3;  // 转到调整距离
                    } else {
                        state = 5;
                        success = true;
                    }
                    break;
                }
                
                // PID平行度调整
                angle_error = atan2(parallel_error, 0.14f);  // 传感器间距0.14m
                angular_speed = std::max(-0.2f, std::min(0.2f, angle_error * 2.0f));
                
                {
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = 0.0f;
                    cmd.linear.y = pump_speed;
                    cmd.angular.z = angular_speed;
                    cmd_vel_pub_.publish(cmd);
                    
                    ROS_INFO_THROTTLE(0.5, "平行调整: 误差=%.2fm -> 角速度=%.2f", 
                                      parallel_error, angular_speed);
                }
                break;
                
            case 3:  // STATE_ADJUST_DISTANCE - 调整到目标距离
                if (!has_right_wall) {
                    ROS_WARN("调整距离时右侧池壁丢失");
                    state = 0;
                    break;
                }
                
                // 先调整平行度（确保前进方向正确）
                if (fabs(parallel_error) > 0.02f) {
                    ROS_INFO("先调整平行度");
                    state = 2;
                    break;
                }
                
                current_dist = right_dist;
                error = target_distance - current_dist;
                
                if (fabs(error) < 0.05f) {
                    stop();
                    ROS_INFO("距离调整完成: %.2fm", current_dist);
                    state = 5;
                    success = true;
                    break;
                }
                
                // 前进或后退调整距离
                move_speed = 0.12f;
                if (error > 0) {
                    // 需要靠近池壁（前进）
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = move_speed;
                    cmd.linear.y = pump_speed;
                    cmd.angular.z = 0.0f;
                    cmd_vel_pub_.publish(cmd);
                    ROS_INFO_THROTTLE(0.5, "靠近: 当前=%.2fm, 目标=%.2fm", current_dist, target_distance);
                } else {
                    // 需要远离池壁（后退）
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = -move_speed;
                    cmd.linear.y = pump_speed;
                    cmd.angular.z = 0.0f;
                    cmd_vel_pub_.publish(cmd);
                    ROS_INFO_THROTTLE(0.5, "远离: 当前=%.2fm, 目标=%.2fm", current_dist, target_distance);
                }
                break;
                
            case 4:  // STATE_TURN_TO_WALL - 角落场景：左转90度
                if (!turn_started) {
                    turn_start_yaw = current_pose_.theta;
                    turn_start_time = ros::Time::now();
                    turn_started = true;
                    ROS_INFO("角落场景: 开始左转90度");
                }
                
                yaw_change = normalizeAngle(current_pose_.theta - turn_start_yaw);
                abs_yaw_change = fabs(yaw_change);
                
                if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
                    (ros::Time::now() - turn_start_time).toSec() > 8.0f) {
                    stop();
                    ROS_INFO("左转完成: 转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
                    state = 2;  // 转到平行对齐
                    turn_started = false;
                    break;
                }
                
                {
                    geometry_msgs::Twist cmd;
                    cmd.linear.x = 0.0f;
                    cmd.linear.y = pump_speed;
                    cmd.angular.z = 0.25f;  // 左转
                    cmd_vel_pub_.publish(cmd);
                }
                break;
                
            default:
                break;
        }
        
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
    
    // ========== 3. 清理 ==========
    stop();
    disabling_timer_ = old_disabling_state;
    
    if (success) {
        ROS_INFO("✅ 智能沿边对齐完成");
    } else {
        ROS_ERROR("❌ 智能沿边对齐失败");
    }
    
    return success;
}

// 获取日志目录
std::string RobotWalker::getWalkLogDirectory() {
    std::string home_dir = getenv("HOME");
    std::string base_dir = home_dir + "/sonar_logs/";
    std::string walk_dir = base_dir + "walk_logs/";
    
    struct stat st;
    if (stat(base_dir.c_str(), &st) != 0) {
        mkdir(base_dir.c_str(), 0755);
    }
    if (stat(walk_dir.c_str(), &st) != 0) {
        mkdir(walk_dir.c_str(), 0755);
    }
    
    return walk_dir;
}

// 初始化行走日志
void RobotWalker::initWalkLog(const std::string& log_type, float target_distance) {
    log_buffer_.clear();
    
    std::string dir = getWalkLogDirectory();
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::tm* tm_info = std::localtime(&now_time_t);
    char time_buf[64];
    strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", tm_info);
    
    current_log_filename_ = dir + log_type + "_" + std::string(time_buf) + 
                            "_dist" + std::to_string(target_distance) + ".csv";
    
    if (walk_log_file_.is_open()) {
        walk_log_file_.close();
    }
    walk_log_file_.open(current_log_filename_);
    
    if (walk_log_file_.is_open()) {
        // 写入CSV头
        walk_log_file_ << "elapsed_sec,right_front_m,right_rear_m,infra_left_m,infra_right_m,"
                       << "pose_x_m,pose_y_m,pose_theta_deg,target_distance_m,current_distance_m,status\n";
        walk_log_file_.flush();
        ROS_INFO("行走日志已创建: %s", current_log_filename_.c_str());
    } else {
        ROS_ERROR("无法创建行走日志文件: %s", current_log_filename_.c_str());
    }
    
    last_log_time_ = ros::Time::now();
    walk_start_time_ = ros::Time::now();
}

// 记录行走数据（每秒记录一次）
void RobotWalker::recordWalkData(float current_distance, float target_distance) {
    ros::Time now = ros::Time::now();
    
    if ((now - last_log_time_).toSec() < 1.0) {
        return;
    }
    
    WalkLogData data;
    data.timestamp = now;
    data.right_front = right_front_dist_;
    data.right_rear = right_rear_dist_;
    data.infra_left = infra_front_left_;
    data.infra_right = infra_front_right_;
    data.pose_x = current_pose_.x;
    data.pose_y = current_pose_.y;
    data.pose_theta = current_pose_.theta;
    data.target_distance = target_distance;  // 直接在这里设置
    data.current_distance = current_distance;
    
    log_buffer_.push_back(data);
    last_log_time_ = now;
}

// 保存行走日志到文件
void RobotWalker::saveWalkLog(bool success) {
    if (!walk_log_file_.is_open()) {
        ROS_WARN("行走日志文件未打开，无法保存");
        return;
    }
    
    for (size_t i = 0; i < log_buffer_.size(); i++) {
        const auto& data = log_buffer_[i];
        double elapsed = (data.timestamp - walk_start_time_).toSec();
        
        walk_log_file_ << std::fixed << std::setprecision(3)
                       << elapsed << ","
                       << data.right_front << ","
                       << data.right_rear << ","
                       << data.infra_left << ","
                       << data.infra_right << ","
                       << data.pose_x << ","
                       << data.pose_y << ","
                       << data.pose_theta * 180.0 / M_PI << ","
                       << data.target_distance << ","
                       << data.current_distance << ","
                       << (success ? "SUCCESS" : "FAILED") << "\n";
    }
    
    walk_log_file_.flush();
    walk_log_file_.close();
    
    ROS_INFO("行走日志已保存: %s (共%zu条记录)", 
             current_log_filename_.c_str(), log_buffer_.size());
}

// 移动平均滤波函数
float RobotWalker::movingAverageFilter(float new_value, std::deque<float>& buffer, int window_size) {
    buffer.push_back(new_value);
    if (buffer.size() > window_size) {
        buffer.pop_front();
    }
    
    float sum = 0.0f;
    for (float v : buffer) {
        sum += v;
    }
    return sum / buffer.size();
}

} // namespace sonar_slam

// ========== main函数 ==========
int main(int argc, char** argv) {
    // 设置locale
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    
    ros::init(argc, argv, "robot_walker");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("RobotWalker启动 (基于MoveController成功机制)");
    ROS_INFO("========================================");
    
    // 设置信号处理
    std::signal(SIGINT, sonar_slam::signalHandler);
    std::signal(SIGTERM, sonar_slam::signalHandler);
    
    // 创建行走控制器
    sonar_slam::RobotWalker walker(nh);
    sonar_slam::g_walker = &walker;
    
    // 检查是否自动开始测试
    bool auto_start = false;
    nh.param("auto_start", auto_start, false);

    
    
    // 选择行走模式
    bool use_advanced_walk = false;  // false: 普通模式, true: 高级模式（带避让）
    nh.param("use_advanced_walk", use_advanced_walk, false);
    
    float test_distance = 1.0f;
    float test_speed = 0.1f;
    float test_timeout = 20.0f;
    
    // 从参数服务器读取
    nh.getParam("test_distance", test_distance);
    nh.getParam("test_speed", test_speed);
    nh.getParam("test_timeout", test_timeout);
    
    ROS_INFO("参数配置:");
    ROS_INFO("  auto_start = %d", auto_start);
    ROS_INFO("  use_advanced_walk = %d %s", use_advanced_walk, use_advanced_walk ? "(带池壁避让)" : "(普通模式)");
    ROS_INFO("  distance = %.2f m", test_distance);
    ROS_INFO("  speed = %.2f m/s", test_speed);
    ROS_INFO("  timeout = %.1f s", test_timeout);
    
    if (auto_start) {
        ROS_INFO("开始自动行走测试");
        bool success;
        
        if (use_advanced_walk) {
            // 高级模式：强制关闭预旋转
            ROS_INFO("使用高级行走模式（带行走中池壁检测与避让）");
            ROS_INFO("注意：高级模式下预旋转已禁用");
            success = walker.walkStraightWithWallAvoidanceAdvanced(test_distance, test_speed, test_timeout);
        } else {
            // 普通模式：使用原有 pre_rotate_90_ 参数
            ROS_INFO("使用普通行走模式");
            success = walker.walkStraight(test_distance, test_speed, test_timeout);
        }
        
        if (success) {
            ROS_INFO("✅ 行走成功");
        } else {
            ROS_ERROR("❌ 行走失败");
        }
    } else {
        ROS_INFO("等待服务调用:");
        ROS_INFO("  普通模式: rosservice call /walker/walk_straight");
        ROS_INFO("  高级模式: 设置 auto_start:=true use_advanced_walk:=true");
    }
    
    // 主循环
    ROS_INFO("========================================");
    ROS_INFO("控制器准备就绪");
    ROS_INFO("  状态: %s", walker.getStatus().c_str());
    ROS_INFO("========================================");
    
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("机器人行走控制器关闭");
    return 0;
}