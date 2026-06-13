#include "sonar_slam/wall_follower.h"
#include <cmath>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <signal.h>
#include <csignal>
#include <locale>


sonar_slam::WallFollower* g_wall_follower = nullptr;

void signalHandler(int sig) {
    if (g_wall_follower) {
        g_wall_follower->stop();
    }
    ros::shutdown();
}

namespace sonar_slam {

// 常量定义
const float PI_M = 3.14159265358979323846f;
const float DEG_TO_RAD = PI_M / 180.0f;
const float RAD_TO_DEG = 180.0f / PI_M;

// 键盘模拟参数
const float LINEAR_INC = 0.01f;
const float ANGULAR_INC = 0.3f;
const float MAX_LINEAR = 0.35f;
const float MAX_ANGULAR = 0.5f;

WallFollower::WallFollower(ros::NodeHandle& nh)
    : nh_(nh)
    , state_(STATE_IDLE)
    , rotate_sub_state_(ROTATE_FULL_CIRCLE)
    , follow_sub_state_(FOLLOW_STRAIGHT)
    , odom_received_(false)
    , debug_counter_(0)
    , pump_speed_(0.0f)           
    , follow_start_yaw_(0.0f)      
    , follow_yaw_initialized_(false)  
    , last_decision_time_(ros::Time::now())
    , last_decision_("")
    , decision_stable_count_(0)
    , last_state_update_(ros::Time::now()) 
    , executing_action_(false)
    , turn_direction_(1)
    , turn_start_yaw_(0.0f)
    , turn_target_yaw_(0.0f)
    , accumulated_yaw_(0.0f)
    , turning_active_(false)
{
     // 设置locale
    std::setlocale(LC_ALL, "");
    std::setlocale(LC_NUMERIC, "C");
    // 初始化ROS接口
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub_ = nh_.subscribe("/wheel_odom", 100, &WallFollower::odomCallback, this);
    ultra_sub_ = nh_.subscribe("/ultra_data", 100, &WallFollower::ultraCallback, this);
    
    // 初始化传感器数据
    sensors_.ultra_right_front = 999.0f;
    sensors_.ultra_right_rear = 999.0f;
    sensors_.infra_front_left = 999.0f;
    sensors_.infra_front_right = 999.0f;
    
    // 初始化沿边控制参数
    wall_control_.kp_distance = 0.8f;
    wall_control_.kp_angle = 1.2f;
    wall_control_.target_distance = 0.3f;
    wall_control_.current_distance = 0.0f;
    wall_control_.angle_error = 0.0f;
    wall_control_.distance_error = 0.0f;
    
    // 初始化行走状态
    travel_.segment_start_x = 0.0f;
    travel_.segment_start_y = 0.0f;
    travel_.segment_traveled = 0.0f;
    travel_.total_traveled = 0.0f;
    travel_.start_yaw = 0.0f;
    travel_.start_pose.x = 0.0f;
    travel_.start_pose.y = 0.0f;
    travel_.start_pose.theta = 0.0f;
    
    // 初始化墙面搜索
    wall_search_.min_distance = 999.0f;
    wall_search_.min_distance_yaw = 0.0f;
    wall_search_.target_yaw = 0.0f;
    wall_search_.start_yaw = 0.0f;
    wall_search_.found_wall = false;
    wall_search_.need_random_walk = false; 
 
    
 
    
    ROS_INFO("========================================");
    ROS_INFO("WallFollower 初始化完成");
    ROS_INFO("  发布: /cmd_vel");
    ROS_INFO("  订阅: /wheel_odom, /ultra_data");
    ROS_INFO("========================================");
}

WallFollower::~WallFollower() {
    stop();
}

bool WallFollower::start(double wall_distance, double speed, 
                         double segment_distance, double stop_duration,
                         double turn_distance, double complete_distance,
                         double finish_distance, double pump_speed) {
    if (state_ != STATE_IDLE) {
        ROS_ERROR("沿边行走已在运行中，无法开始新的任务");
        return false;
    }
    
    // 保存参数
    params_.wall_distance = static_cast<float>(wall_distance);
    params_.speed = static_cast<float>(speed);
    params_.segment_distance = static_cast<float>(segment_distance);
    params_.stop_duration = static_cast<float>(stop_duration);
    params_.turn_distance = static_cast<float>(turn_distance);
    params_.complete_distance = static_cast<float>(complete_distance);
    params_.finish_distance = static_cast<float>(finish_distance);
    pump_speed_ = static_cast<float>(pump_speed);
    
    // 限制参数范围
 
    params_.segment_distance = std::max(0.5f, std::min(5.0f, params_.segment_distance));
    params_.stop_duration = std::max(3.0f, std::min(120.0f, params_.stop_duration));
    params_.turn_distance = std::max(0.3f, std::min(1.2f, params_.turn_distance));
    params_.complete_distance = std::max(1.0f, std::min(100.0f, params_.complete_distance));
    params_.speed = std::max(0.10f, std::min(0.40f, params_.speed));
    params_.finish_distance = std::max(0.3f, std::min(3.0f, params_.finish_distance));
    pump_speed_ = std::max(0.0f, std::min(1.0f, pump_speed_));
    
    wall_control_.target_distance = params_.wall_distance;

 
    twist_sub_ = nh_.subscribe("/wheelTwist_sta", 10, &WallFollower::twistCallback, this);
    
    ROS_INFO("========================================");
    ROS_INFO("沿边行走开始");
    ROS_INFO("  沿边距离: %.2f 米", params_.wall_distance);
    ROS_INFO("  行走速度: %.2f 米/秒", params_.speed);
    ROS_INFO("  每段距离: %.2f 米", params_.segment_distance);
    ROS_INFO("  停止时间: %.0f 秒", params_.stop_duration);
    ROS_INFO("  转弯距离: %.2f 米", params_.turn_distance);
    ROS_INFO("  完成距离: %.2f 米", params_.complete_distance);
    ROS_INFO("  水泵速度: %.0f %%", pump_speed_ * 100);
    ROS_INFO("========================================");
    front_align_step_ = 0;
    // 等待里程计数据
    if (!waitForOdom(5.0f)) {
        ROS_ERROR("等待里程计数据超时");
        return false;
    }
    
    // 记录起始位姿
    updateCurrentPose();
    travel_.start_pose = current_pose_;
    travel_.start_yaw = current_pose_.theta;
    
    // 重置 IMU 纠偏状态
    follow_start_yaw_ = 0.0f;
    follow_yaw_initialized_ = false;
    
    // 开始旋转寻找墙面
    state_ = STATE_ROTATING;
    rotate_sub_state_ = ROTATE_FULL_CIRCLE;
    wall_search_.min_distance = 999.0f;
    wall_search_.found_wall = false;
    wall_search_.start_yaw = current_pose_.theta;

  
    follow_yaw_initialized_ = false;
    alignment_completed_ = false;
    aligned_yaw_ = 0.0f;
    executing_action_ = false;
    accumulated_yaw_ = 0.0f;   
    ROS_INFO("开始旋转寻找墙面...");
    
    return true;
}

 
void WallFollower::stop() {
    stopMoving();
    state_ = STATE_IDLE;
    executing_action_ = false; // 强制清除锁
    turning_active_ = false;   
    ROS_INFO("沿边行走已停止");
}

void WallFollower::update() {
    if (state_ == STATE_IDLE) return;
    
    updateCurrentPose();
    
    // 状态机更新频率限制为5Hz（200ms间隔）
    if ((ros::Time::now() - last_state_update_).toSec() < 0.2) {
        return;
    }
    last_state_update_ = ros::Time::now();
    
    switch (state_) {
        case STATE_ROTATING:
            updateRotating();
            break;
        case STATE_ALIGNING:
            updateAligning();
            break;
        case STATE_FOLLOWING:
            updateFollowing();
            break;
        case STATE_STOPPING:
            updateStopping();
            break;
        case STATE_TURNING:
            updateTurning();
            break;
        case STATE_COMPLETED:
            ROS_INFO("沿边行走完成！");
            state_ = STATE_IDLE;
            break;
        case STATE_FRONT_WALL:
            updateFrontWall();
            break;
        case STATE_GAP_HANDLING:
            updateGapHandling();
            break;
        default:
            break;
    }
}

void WallFollower::updateRotating() {
    // 正在执行关键动作，不处理新决策
    if (executing_action_) {
        return;
    }
    
    // 每秒打印一次传感器数据
    static ros::Time last_log = ros::Time::now();
    if ((ros::Time::now() - last_log).toSec() >= 1.0) {
        last_log = ros::Time::now();
        ROS_INFO("传感器: 前L=%.2f 前R=%.2f 右前=%.2f 右后=%.2f",
                 sensors_.infra_front_left, sensors_.infra_front_right,
                 sensors_.ultra_right_front, sensors_.ultra_right_rear);
    }
    
    // ========== 前方红外检测（带连续稳定计数）==========
    float front_left = sensors_.infra_front_left;
    float front_right = sensors_.infra_front_right;
    bool front_detected = (front_left > 0.1f && front_left < 1.0f) ||
                          (front_right > 0.1f && front_right < 1.0f);
    
    static int front_valid_count = 0;
    if (front_detected) {
        front_valid_count++;
    } else {
        front_valid_count = 0;
    }
    bool front_valid = (front_valid_count >= 3);  // 连续3次有效
    
    // ========== 右侧超声波检测（带连续稳定计数）==========
    float right_dist = getRightUltrasoundMin();
    bool right_detected = (right_dist > 0.1f && right_dist < 5.0f);
    
    static int right_valid_count = 0;
    if (right_detected) {
        right_valid_count++;
    } else {
        right_valid_count = 0;
    }
    bool right_valid = (right_valid_count >= 3);  // 连续3次有效
    
    // ========== 决策（带稳定计数）==========
    std::string decision = "NONE";
    if (front_valid && right_valid) decision = "BOTH";
    else if (front_valid) decision = "FRONT";
    else if (right_valid) decision = "RIGHT";
    
    // 稳定计数：连续5次相同决策才执行
    if (decision == last_decision_) {
        decision_stable_count_++;
    } else {
        last_decision_ = decision;
        decision_stable_count_ = 0;
        return;
    }
    if (decision_stable_count_ < 5) return;
    
    // ========== 执行决策 ==========
    if (decision == "BOTH" || decision == "FRONT") {
        ROS_INFO("执行正向对齐 (前L=%.2f,前R=%.2f)", front_left, front_right);
        updateFrontAlign();
    } else if (decision == "RIGHT") {
        ROS_INFO("右侧有墙(%.2fm)，开始对齐", right_dist);
        state_ = STATE_ALIGNING;
        stopMoving();
    } else {
        // ========== 右转搜索 ==========
        static int search_state = 0;
        static float turn_start_yaw = 0.0f;
        static ros::Time walk_start_time;
        
        if (search_state == 0) {
            if (turn_start_yaw == 0.0f) {
                turn_start_yaw = current_pose_.theta;
                ROS_INFO("未找到墙，开始右转搜索");
            }
            float target_yaw = normalizeAngle(turn_start_yaw - PI_M / 2.0f);
            float err = normalizeAngle(target_yaw - current_pose_.theta);
            if (fabs(err) < 0.05f) {
                turn_start_yaw = 0.0f;
                float new_right = getRightUltrasoundMin();
                // 用稳定计数检查找到的墙
                bool new_right_valid = (new_right > 0.1f && new_right < 5.0f);
                static int new_right_count = 0;
                if (new_right_valid) {
                    new_right_count++;
                } else {
                    new_right_count = 0;
                }
                if (new_right_count >= 3) {
                    state_ = STATE_ALIGNING;
                    stopMoving();
                } else {
                    search_state = 1;
                    walk_start_time = ros::Time::now();
                    ROS_INFO("仍未找到墙，直行3秒");
                }
            } else {
                applyKeyboardCommand('d');
            }
        } else if (search_state == 1) {
            if ((ros::Time::now() - walk_start_time).toSec() >= 3.0f) {
                search_state = 0;
                ROS_INFO("直行结束，继续搜索");
            } else {
                applyKeyboardCommand('w');
            }
        }
    }
}

void WallFollower::updateFrontAlign() {
    // 设置执行标志
    ScopedAction lock(executing_action_);

    static int step = 0;
    
    if (step == 0) {
        step = 1;
        stopMoving();
        ROS_INFO("开始后退");
        return;
    }
    
    if (step == 1) {
        float front_left = sensors_.infra_front_left;
        float front_right = sensors_.infra_front_right;
        // 判断是否满足后退完成条件
        // 条件1：有效距离(0.1-1.0m) >= 0.35m
        // 条件2：无效(999) 表示距离 > 1m，也满足
        bool left_done = (front_left > 1.0f) || (front_left >= params_.wall_distance && front_left < 1.0f);
        bool right_done = (front_right > 1.0f) || (front_right >= params_.wall_distance && front_right < 1.0f);
        
        if (left_done || right_done) {
            step = 2;
            stopMoving();
            ROS_INFO("后退完成(前L=%.2f, 前R=%.2f)，开始左转90度", front_left, front_right);
        } else {
            applyKeyboardCommand('x');
            ROS_INFO_THROTTLE(0.5, "后退中: 前L=%.2f, 前R=%.2f", front_left, front_right);
        }
        return;
    }
    
    if (step == 2) {
        static float turn_target = 0.0f;
        if (turn_target == 0.0f) {
            turn_target = normalizeAngle(current_pose_.theta + PI_M / 2.0f);
            ROS_INFO("左转目标角度: %.1f°", turn_target * RAD_TO_DEG);
        }
        float err = normalizeAngle(turn_target - current_pose_.theta);
        if (fabs(err) < 0.05f) {
            stopMoving();
            step = 0;
            turn_target = 0.0f;
            state_ = STATE_ALIGNING;
            // 动作完成，清除标志
          
            ROS_INFO("左转完成，开始对齐");
        } else {
            applyKeyboardCommand('a');
            ROS_INFO_THROTTLE(0.5, "左转中: 误差=%.1f°", err * RAD_TO_DEG);
        }
        return;
    }
}

void WallFollower::updateAligning() {
    ScopedAction lock(executing_action_);
    
    float right_front = sensors_.ultra_right_front;
    float right_rear = sensors_.ultra_right_rear;
    float right_avg = (right_front + right_rear) / 2.0f;
    float parallel_error = right_rear - right_front;
    
    static int step = 0;
    static float start_total = 0.0f;
    static float gap = 0.0f;
    static bool turn_started = false;
    static float turn_start_yaw = 0.0f;
    static ros::Time turn_start_time;
    
    ROS_INFO_THROTTLE(0.5, "步骤%d: 前=%.2f 后=%.2f 平行=%.2f 平均=%.2f", 
                      step, right_front, right_rear, parallel_error, right_avg);
    
    // step0: 调平行度
    if (step == 0) {
        if (fabs(parallel_error) < 0.02f) {
            gap = params_.wall_distance - right_avg;
            ROS_INFO("平行达标，距离差距=%.2f", gap);
            
            if (fabs(gap) < 0.05f) {
                step = 0;
                state_ = STATE_FOLLOWING;
                travel_.segment_start_x = current_pose_.x;
                travel_.segment_start_y = current_pose_.y;
                travel_.segment_traveled = 0.0f;
                travel_.total_traveled = 0.0f;
                ROS_INFO("对齐完成");
                return;
            }
            step = 1;
            ROS_INFO("需要%s%.2f米", gap > 0 ? "前进" : "后退", fabs(gap));
            return;
        }
        
        if (parallel_error > 0) {
            // 前近后远 → 左转
            ROS_INFO("前近后远，左转");
            applyKeyboardCommand('a');
        } else {
            // 前远后近 → 右转
            ROS_INFO("前远后近，右转");
            applyKeyboardCommand('d');
        }
        ros::Duration(0.3f).sleep();
        stopMoving();
        return;
    }
    
    // step1: 左转90度
    if (step == 1) {
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_start_time = ros::Time::now();
            turn_started = true;
            applyKeyboardCommand('a');
            ROS_INFO("开始左转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
        }
        
        // 计算累计转角（处理360°跨越）
        float yaw_change = current_pose_.theta - turn_start_yaw;
        yaw_change = normalizeAngle(yaw_change);
        
        // 累计转角达到85°-95°之间就算完成
        ROS_INFO("左转: 累计转角=%.1f°", yaw_change * RAD_TO_DEG);
        
        if (yaw_change > 85.0f * DEG_TO_RAD || 
            (ros::Time::now() - turn_start_time).toSec() > 5.0f) {
            stopMoving();
            step = 2;
            start_total = travel_.total_traveled;
            turn_started = false;
            ROS_INFO("左转完成: 累计转角=%.1f°", yaw_change * RAD_TO_DEG);
        } else {
            setAngularSpeed(0.25f);
        }
        return;
    }
        
 
    // step2: 移动差距距离（使用速度×时间）
    if (step == 2) {
        static ros::Time move_start_time;
        static bool move_started = false;
        
        if (!move_started) {
            move_start_time = ros::Time::now();
            move_started = true;
            ROS_INFO("开始移动%.3f米", fabs(gap));
        }
        
        float elapsed = (ros::Time::now() - move_start_time).toSec();
        float traveled = params_.speed * elapsed;
        float target = fabs(gap);
        
        ROS_INFO("移动: 已走%.3f/%.3f米", traveled, target);
        
        if (traveled >= target) {
            stopMoving();
            step = 3;
            move_started = false;
            ROS_INFO("移动%.3f米完成", target);
        } else if (gap > 0) {
            applyKeyboardCommand('w');
        } else {
            applyKeyboardCommand('x');
        }
        return;
    }
    
    // step3: 右转90度
    if (step == 3) {
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_start_time = ros::Time::now();
            turn_started = true;
            applyKeyboardCommand('d');
            ROS_INFO("开始右转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
        }
        
        // 计算累计转角（负值，取绝对值）
        float yaw_change = current_pose_.theta - turn_start_yaw;
        yaw_change = normalizeAngle(yaw_change);
        // 右转时角度减小，累计转角为负
        float abs_yaw_change = fabs(yaw_change);
        
        ROS_INFO("右转: 累计转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
        
        if (abs_yaw_change > 85.0f * DEG_TO_RAD || 
            (ros::Time::now() - turn_start_time).toSec() > 5.0f) {
            stopMoving();
            step = 4;
            turn_started = false;
            ROS_INFO("右转完成: 累计转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
        } else {
            setAngularSpeed(-0.25f);
        }
        return;
    }
    
   // step4: 最终对齐（只检查平行度）
    if (step == 4) {
        if (fabs(parallel_error) < 0.02f) {
            step = 0;
            state_ = STATE_FOLLOWING;
            travel_.segment_start_x = current_pose_.x;
            travel_.segment_start_y = current_pose_.y;
            travel_.segment_traveled = 0.0f;
            travel_.total_traveled = 0.0f;
            ROS_INFO("最终对齐完成，开始沿边");
            return;
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
        stopMoving();
        return;
    }
}

void WallFollower::updateFollowing() {
    ScopedAction lock(executing_action_);
    
    // ========== 使用odom实际位移累加里程 ==========
    static double last_x = 0.0;
    static double last_y = 0.0;
    static bool has_last_pose = false;
    
    if (!has_last_pose) {
        last_x = current_pose_.x;
        last_y = current_pose_.y;
        has_last_pose = true;
    } else {
        double dx = current_pose_.x - last_x;
        double dy = current_pose_.y - last_y;
        double delta = sqrt(dx*dx + dy*dy);
        
        // 过滤异常跳变
        if (delta > 0.001 && delta < 0.5) {
            travel_.segment_traveled += delta;
            travel_.total_traveled += delta;
        }
        
        last_x = current_pose_.x;
        last_y = current_pose_.y;
    }
    
    // 每段完成后停止
    if (travel_.segment_traveled >= params_.segment_distance) {
        ROS_INFO("已走%.2fm，停止%.0f秒", travel_.segment_traveled, params_.stop_duration);
        state_ = STATE_STOPPING;
        travel_.stop_start_time = ros::Time::now();
        travel_.segment_traveled = 0.0f;
        has_last_pose = false;  // 重置里程累加状态
        stopMoving();
        return;
    }

     // ========== 需求6：检测凹凸入区域 ==========

    float right_front = sensors_.ultra_right_front;
    float right_rear = sensors_.ultra_right_rear;
    float right_avg = (right_front + right_rear) / 2.0f;
    
    static float last_right_avg = right_avg;
    static int gap_detect_count = 0;
    
    // 右侧距离突然变远或无效
    bool is_gap = (right_avg > 1.5f) || 
                  (right_avg > last_right_avg + 0.3f) ||
                  (right_front > 5.0f || right_rear > 5.0f);
    
    if (is_gap) {
        gap_detect_count++;
        if (gap_detect_count >= 3) {  // 连续3次确认
            ROS_WARN("检测到凹入区域，右侧平均距离=%.2fm，进入凹入处理", right_avg);
            gap_detect_count = 0;
            state_ = STATE_GAP_HANDLING;
            stopMoving();
            return;
        }
    } else {
        gap_detect_count = 0;
    }
    last_right_avg = right_avg;
    
    // ========== 需求5：检测前方墙壁 ==========
    if (checkFrontObstacle()) {
        ROS_WARN("检测到前方墙壁，进入前方墙壁处理");
        state_ = STATE_FRONT_WALL;
        stopMoving();
        return;
    }
        
    applyFollowControl();
}


bool WallFollower::checkFrontObstacleStable() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    float front_left = sensors_.infra_front_left;
    float front_right = sensors_.infra_front_right;
    
    bool left_obstacle = (front_left > 0.1f && front_left < params_.turn_distance);
    bool right_obstacle = (front_right > 0.1f && front_right < params_.turn_distance);
    bool has_obstacle = left_obstacle || right_obstacle;
    
    // 连续3次检测到才触发（参考代码用了10次）
    static int obstacle_stable_count = 0;
    if (has_obstacle) {
        obstacle_stable_count++;
    } else {
        obstacle_stable_count = 0;
    }
    
    return obstacle_stable_count >= 3;
}

void WallFollower::updateStopping() {
    ScopedAction lock(executing_action_);
    float elapsed = (ros::Time::now() - travel_.stop_start_time).toSec();

    if (elapsed >= params_.stop_duration) {
        alignment_completed_ = false;
        aligned_yaw_ = 0.0f;
        
        travel_.segment_start_x = current_pose_.x;
        travel_.segment_start_y = current_pose_.y;
        travel_.segment_traveled = 0.0f;
        
        // 检查右侧墙面距离
        float right_dist = getRightUltrasoundMin();
        if (right_dist > 0.2f && right_dist < 0.8f) {
            state_ = STATE_FOLLOWING;
            ROS_INFO("停止结束，继续沿边行走");
        } else {
            state_ = STATE_ALIGNING;
            ROS_INFO("停止结束，墙面距离=%.2fm，重新对齐", right_dist);
        }
    } else {
        if (travel_.total_traveled >= params_.complete_distance && checkCompletion()) {
            state_ = STATE_COMPLETED;
            stopMoving();
            return;
        }
        stopMoving();
    }
}

void WallFollower::updateTurning() {
    ScopedAction lock(executing_action_);
    
    if (!turning_active_) {
        turning_active_ = true;
        turn_start_yaw_ = current_pose_.theta;
        
        float turn_angle = calculateTurnAngle();
        if (turn_direction_ == 1) {
            turn_target_yaw_ = normalizeAngle(turn_start_yaw_ + turn_angle);
        } else {
            turn_target_yaw_ = normalizeAngle(turn_start_yaw_ - turn_angle);
        }
        
        ROS_INFO("开始转弯: 方向=%d, 起始=%.1f°, 目标=%.1f°", 
                 turn_direction_, 
                 turn_start_yaw_ * RAD_TO_DEG, 
                 turn_target_yaw_ * RAD_TO_DEG);
    }
    
    float err = normalizeAngle(turn_target_yaw_ - current_pose_.theta);
    
    if (fabs(err) < 3.0f * DEG_TO_RAD) {
        stopMoving();
        turning_active_ = false;
        state_ = STATE_ALIGNING;
        ROS_INFO("转弯完成");
    } else {
        float speed = (err > 0) ? 0.25f : -0.25f;
        setAngularSpeed(speed);
    }
}

void WallFollower::computeWallErrors() {
    // 使用右侧前后超声波计算距离和角度误差
    float dist_front = sensors_.ultra_right_front;
    float dist_rear = sensors_.ultra_right_rear;
    
    // 过滤无效数据
    if (dist_front > 5.0f || dist_front < 0.05f) dist_front = dist_rear;
    if (dist_rear > 5.0f || dist_rear < 0.05f) dist_rear = dist_front;
    
    // 传感器安装间距 (米)
    const float SENSOR_SPACING = 0.14f;
    
    // 角度误差: 利用前后传感器差值计算车身与墙的夹角
    float diff = dist_rear - dist_front;
    wall_control_.angle_error = atan2(diff, SENSOR_SPACING);
    wall_control_.angle_error = std::max(-0.3f, std::min(0.3f, wall_control_.angle_error));
    
    // 距离误差: 使用前传感器作为主参考
    wall_control_.current_distance = dist_front;
    wall_control_.distance_error = wall_control_.current_distance - params_.wall_distance;
    wall_control_.distance_error = std::max(-0.2f, std::min(0.2f, wall_control_.distance_error));
}
 
void WallFollower::applyFollowControl() {
    float right_dist = sensors_.ultra_right_front;
    float distance_error = right_dist - params_.wall_distance;
    
    // 超声波平行度纠偏（主要方式，最可靠）
    float diff = sensors_.ultra_right_rear - sensors_.ultra_right_front;
    float parallel_error = atan2(diff, 0.14f);
    
    static ros::Time last_correction = ros::Time::now();
    ros::Time now = ros::Time::now();
    
    if ((now - last_correction).toSec() >= 0.5) {
        last_correction = now;
        
        // 优先级1：超声波平行度纠偏
        // 平行纠偏
        if (fabs(parallel_error) > 0.02f) {
            if (parallel_error > 0) {
                applyKeyboardCommand('a');  // 前近后远 → 左转（修正）
                ROS_INFO("平行纠偏: 前近后远%.1f°, 左转", parallel_error * RAD_TO_DEG);
            } else {
                applyKeyboardCommand('d');  // 前远后近 → 右转
                ROS_INFO("平行纠偏: 前远后近%.1f°, 右转", -parallel_error * RAD_TO_DEG);
            }
            return;
        }
        
        // 优先级2：距离纠偏
        if (fabs(distance_error) > 0.10f) {
            if (distance_error > 0) {
                applyKeyboardCommand('d');
                ROS_INFO("距离纠偏: 太远%.2fm", distance_error);
            } else {
                applyKeyboardCommand('a');
                ROS_INFO("距离纠偏: 太近%.2fm", -distance_error);
            }
            return;
        }
    }
    
    applyKeyboardCommand('w');
}
/***
void WallFollower::updateFrontWall() {
    ScopedAction lock(executing_action_);
    
    static int step = 0;
    static float start_total = 0.0f;
    static float target_move_distance_ = 0.0f;
    static bool turn_started = false;
    static float turn_start_yaw = 0.0f;
    static ros::Time turn_start_time;
    static ros::Time move_start_time;
    static bool move_started = false;

        // ========== 记录后退开始时的传感器值 ==========
    static float initial_front_dist_when_backup = 0.0f;
    
    float front_left = sensors_.infra_front_left;
    float front_right = sensors_.infra_front_right;
    float front_dist = std::min(front_left, front_right);
    bool front_valid = (front_dist > 0.05f && front_dist < 1.0f);
    
    float right_front = sensors_.ultra_right_front;
    float right_rear = sensors_.ultra_right_rear;
    float parallel_error = right_rear - right_front;
    
    ROS_INFO("FrontWall步骤%d: 前方=%.2f, 平行=%.2f", step, front_dist, parallel_error);
    
    // step0: 检测到墙壁，计算需要后退的距离
    if (step == 0) {
        static int invalid_count = 0;
        
        // 1. 先检查数据是否异常（999）
        if (!front_valid) {
            if (front_dist > 5.0f) {  // 999 无效值
                invalid_count++;
                ROS_INFO("前方数据无效(%.2f)，连续%d次", front_dist, invalid_count);
                
                if (invalid_count >= 3) {
                    // 连续3次无效，直接左转90度
                    ROS_WARN("前方红外连续无效，直接左转90度");
                    step = 2;  // 进入左转90度
                    invalid_count = 0;
                } else {
                    // 等待数据稳定，不前进不后退
                    ros::Duration(0.3f).sleep();
                }
            } else {
                invalid_count = 0;
                applyKeyboardCommand('w');  // 继续前进
            }
            return;
        }
        
        // 2. 数据有效，重置计数
        invalid_count = 0;
        
        // 3. 计算需要移动的距离
        float move_needed = front_dist - params_.wall_distance;
        
        // 4. 距离已达标，直接转弯
        if (fabs(move_needed) < 0.05f) {
            ROS_INFO("距离已达标(%.2fm)，直接转弯", front_dist);
            step = 2;
            return;
        }
        
        // 5. 需要移动
        if (move_needed > 0) {
            // 需要后退
            target_move_distance_ = std::min(move_needed, 1.0f);
            ROS_INFO("需要后退%.2fm", target_move_distance_);
        } else {
            // 需要前进
            target_move_distance_ = std::min(-move_needed, 1.0f);
            ROS_INFO("需要前进%.2fm", target_move_distance_);
        }
        
        step = 1;
        move_start_time = ros::Time::now();
        move_started = true;
        return;
    }
    
    // step1: 后退
    if (step == 1) {
        // 后退过程中保持平行度（这部分不变）
        if (fabs(parallel_error) > 0.03f) {
            geometry_msgs::Twist cmd;
            cmd.linear.x = -params_.speed;
            cmd.linear.y = pump_speed_;
            if (parallel_error > 0) {
                cmd.angular.z = 0.10f;
            } else {
                cmd.angular.z = -0.10f;
            }
            cmd_vel_pub_.publish(cmd);
            ros::Duration(0.2f).sleep();
            stopMoving();
            ros::Duration(0.1f).sleep();
            return;
        }
        
        // ========== 以下才是需要修改的部分 ==========
        
        if (!move_started) {
            move_start_time = ros::Time::now();
            move_started = true;
        }
        
        // 实时更新前方距离
        front_left = sensors_.infra_front_left;
        front_right = sensors_.infra_front_right;
        front_dist = std::min(front_left, front_right);
        front_valid = (front_dist > 0.05f && front_dist < 1.0f);
        
        float elapsed = (ros::Time::now() - move_start_time).toSec();
        float max_time = target_move_distance_ / params_.speed * 1.5f;
        
        // 停止条件（三选一）
        bool should_stop = false;
        
        if (front_valid && front_dist <= params_.wall_distance + 0.03f) {
            should_stop = true;  // 传感器达标
            ROS_INFO("后退完成: 前方=%.2fm", front_dist);
        } 
        else if (elapsed >= max_time) {
            should_stop = true;  // 超时保护
            ROS_WARN("后退超时，强制停止");
        }
        else if (!front_valid && elapsed > 0.5f) {
            // 传感器失效：再退0.3秒
            static ros::Time lost_time;
            if (!move_started) {
                lost_time = ros::Time::now();
            }
            if ((ros::Time::now() - lost_time).toSec() >= 0.3f) {
                should_stop = true;
                ROS_WARN("传感器失效，停止后退");
            }
        }
        
        if (should_stop) {
            stopMoving();
            move_started = false;
            step = 2;  // 进入左转
            ROS_INFO("后退结束，开始左转");
            return;
        }
        
        applyKeyboardCommand('x');
    }
    
    
    // step2: 左转90度（参照 updateAligning）
    if (step == 2) {
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_start_time = ros::Time::now();
            turn_started = true;
            applyKeyboardCommand('a');
            ROS_INFO("开始左转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
        }
        
        // 计算累计转角（与 updateAligning 一致）
        float yaw_change = current_pose_.theta - turn_start_yaw;
        yaw_change = normalizeAngle(yaw_change);
        float elapsed = (ros::Time::now() - turn_start_time).toSec();
        
        ROS_INFO("左转: 累计转角=%.1f°", yaw_change * RAD_TO_DEG);
        
        if (yaw_change > 85.0f * DEG_TO_RAD || elapsed > 5.0f) {
            stopMoving();
            step = 3;
            turn_started = false;
            accumulated_yaw_ += PI_M / 2.0f;
            ROS_INFO("左转完成: 累计转角=%.1f°", yaw_change * RAD_TO_DEG);
        } else {
            setAngularSpeed(0.25f);
        }
        return;
    }
    
    // step3: 平行度检查
 
    // if (step == 3) {
    //     // 检查右侧是否有墙
    //     float right_front = sensors_.ultra_right_front;
    //     float right_rear = sensors_.ultra_right_rear;
    //     float right_dist = std::min(right_front, right_rear);
        
    //     if (right_dist > 5.0f) {
    //         ROS_WARN("右侧无墙，继续左转寻找");
    //         applyKeyboardCommand('a');
    //         return;
    //     }
        
    //     float parallel_error = right_rear - right_front;
        
    //     // 平行度调整
    //     if (fabs(parallel_error) < 0.03f) {
    //         // 平行度达标，直接进入沿边行走
    //         step = 0;
    //         state_ = STATE_FOLLOWING;  // 直接进入沿边，不经过 ALIGNING
    //         ROS_INFO("右侧对齐完成，开始沿边行走");
    //         applyKeyboardCommand('w');
    //     } else if (parallel_error > 0) {
    //         applyKeyboardCommand('a');  // 左转调平
    //         ROS_INFO_THROTTLE(0.5, "对齐: 前近后远，左转");
    //     } else {
    //         applyKeyboardCommand('d');  // 右转调平
    //         ROS_INFO_THROTTLE(0.5, "对齐: 前远后近，右转");
    //     }
    //     // 删除 sleep 和 stopMoving
    //     return;
    // }

    // step3: 右侧对齐墙壁 
    if (step == 3) {
        static int search_step = 0;  // 0=左转找, 1=右转找, 2=直行找
        static float search_start_yaw = 0.0f;
        static ros::Time search_start_time;
        static float search_angle = 0.0f;
        
        float right_front = sensors_.ultra_right_front;
        float right_rear = sensors_.ultra_right_rear;
        float right_dist = std::min(right_front, right_rear);
        float parallel_error = right_rear - right_front;
        
        // 检查右侧是否有墙（有效距离0.1-3米）
        bool has_right_wall = (right_dist > 0.1f && right_dist < 3.0f);
        
        if (has_right_wall) {
            // 找到墙壁，重置搜索状态，正常对齐
            search_step = 0;
            
            if (fabs(parallel_error) < 0.03f) {
                step = 0;
                state_ = STATE_FOLLOWING;
                ROS_INFO("找到右侧墙壁(%.2fm)，平行度达标，开始沿边", right_dist);
                applyKeyboardCommand('w');
            } else if (parallel_error > 0) {
                applyKeyboardCommand('a');
                ROS_INFO_THROTTLE(0.5, "对齐: 前近后远，左转");
            } else {
                applyKeyboardCommand('d');
                ROS_INFO_THROTTLE(0.5, "对齐: 前远后近，右转");
            }
            return;
        }
        
        // ========== 未找到墙壁，执行搜索 ==========
        switch (search_step) {
            case 0:  // 左转20度寻找
                ROS_WARN("右侧无墙，左转20度寻找");
                search_start_yaw = current_pose_.theta;
                search_start_time = ros::Time::now();
                search_angle = 20.0f * DEG_TO_RAD;
                search_step = 1;
                // 继续执行左转
                break;
                
            case 1:  // 正在左转
                {
                    float yaw_change = normalizeAngle(current_pose_.theta - search_start_yaw);
                    if (yaw_change >= search_angle) {
                        // 左转完成，还没找到，准备右转
                        stopMoving();
                        ROS_WARN("左转20度未找到墙壁，右转40度寻找");
                        search_start_yaw = current_pose_.theta;
                        search_start_time = ros::Time::now();
                        search_angle = 40.0f * DEG_TO_RAD;
                        search_step = 2;
                    } else {
                        applyKeyboardCommand('a');
                    }
                }
                return;
                
            case 2:  // 右转40度寻找
                {
                    float yaw_change = normalizeAngle(current_pose_.theta - search_start_yaw);
                    // 右转时角度减小，所以用负数判断
                    if (yaw_change <= -search_angle) {
                        // 右转完成，还没找到，准备直行
                        stopMoving();
                        ROS_WARN("右转40度未找到墙壁，前进0.5米寻找");
                        search_start_time = ros::Time::now();
                        search_step = 3;
                    } else {
                        applyKeyboardCommand('d');
                    }
                }
                return;
                
            case 3:  // 直行0.5米寻找
                {
                    float elapsed = (ros::Time::now() - search_start_time).toSec();
                    float traveled = params_.speed * elapsed;
                    
                    if (traveled >= 0.5f) {
                        // 直行完成还没找到，停止并报错
                        stopMoving();
                        ROS_ERROR("无法找到右侧墙壁！请检查传感器或环境");
                        state_ = STATE_IDLE;  // 停止任务
                        step = 0;
                        search_step = 0;
                    } else {
                        applyKeyboardCommand('w');
                    }
                }
                return;
        }
        
        // 执行左转（search_step == 0 时）
        applyKeyboardCommand('a');
        return;
    }
}
***/

void WallFollower::updateFrontWall() {
    ScopedAction lock(executing_action_);
    
    static int step = 0;
    static float backup_distance = 0.8f;   // 固定后退0.8米
    static bool turn_started = false;
    static float turn_start_yaw = 0.0f;
    static ros::Time move_start_time;
    
    float right_front = sensors_.ultra_right_front;
    float right_rear = sensors_.ultra_right_rear;
    float parallel_error = right_rear - right_front;
    
    ROS_INFO_THROTTLE(0.5, "FrontWall步骤%d: 平行=%.3f", step, parallel_error);
    
    // step0: 检测到池壁，先停止
    if (step == 0) {
        // 再次确认前方真的有墙
        float front_left = sensors_.infra_front_left;
        float front_right = sensors_.infra_front_right;
        float front_dist = std::min(front_left, front_right);
        bool front_valid = (front_dist > 0.1f && front_dist < 1.0f);
        
        if (!front_valid) {
            // 没有墙，不应该进入该状态
            ROS_WARN("前方无墙，退出前端处理");
            state_ = STATE_FOLLOWING;
            step = 0;
            return;
        }
        
        // 停止移动
        stopMoving();
        ros::Duration(0.2f).sleep();
        
        ROS_INFO("前方检测到池壁(%.2fm)，后退%.2f米", front_dist, backup_distance);
        move_start_time = ros::Time::now();
        step = 1;
        return;
    }
    
    // step1: 后退固定距离
    if (step == 1) {
        float elapsed = (ros::Time::now() - move_start_time).toSec();
        float traveled = params_.speed * elapsed;
        
        ROS_INFO_THROTTLE(0.3, "后退: 已走%.2f/%.2f米", traveled, backup_distance);
        
        if (traveled >= backup_distance) {
            stopMoving();
            step = 2;
            turn_started = false;
            ROS_INFO("后退完成，开始左转90度");
        } else {
            applyKeyboardCommand('x');  // 后退
        }
        return;
    }
    
    // step2: 左转90度
    if (step == 2) {
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_started = true;
            ROS_INFO("开始左转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
        }
        
        float yaw_change = normalizeAngle(current_pose_.theta - turn_start_yaw);
        
        if (yaw_change > 85.0f * DEG_TO_RAD) {
            stopMoving();
            step = 3;
            turn_started = false;
            accumulated_yaw_ += PI_M / 2.0f;
            ROS_INFO("左转完成: 实际转角=%.1f°", yaw_change * RAD_TO_DEG);
        } else {
            applyKeyboardCommand('a');  // 左转
        }
        return;
    }
    
    // step3: 右侧平行度对齐
    if (step == 3) {
        // 检查右侧是否有墙
        float right_dist = std::min(right_front, right_rear);
        
        if (right_dist > 5.0f) {
            ROS_WARN("右侧无墙，继续左转寻找");
            applyKeyboardCommand('a');
            return;
        }
        
        // 平行度调整（连续，不阻塞）
        if (fabs(parallel_error) < 0.05f) {
            step = 0;
            state_ = STATE_FOLLOWING;
            ROS_INFO("右侧对齐完成，继续沿边行走");
            applyKeyboardCommand('w');
        } else if (parallel_error > 0) {
            applyKeyboardCommand('a');
            ROS_INFO_THROTTLE(0.5, "对齐: 前近后远，左转");
        } else {
            applyKeyboardCommand('d');
            ROS_INFO_THROTTLE(0.5, "对齐: 前远后近，右转");
        }
        return;
    }
}

void WallFollower::updateGapHandling() {
    ScopedAction lock(executing_action_);
    
    static int step = 0;
    static float start_x = 0.0f;
    static bool turn_started = false;
    static float turn_start_yaw = 0.0f;
    static ros::Time turn_start_time;
    static ros::Time move_start_time;
    static bool move_started = false;
    
    float right_front = sensors_.ultra_right_front;
    float right_rear = sensors_.ultra_right_rear;
    
    ROS_INFO("GapHandling步骤%d: 前=%.2f, 后=%.2f", step, right_front, right_rear);
    
    // step0: 检测右前是否变长/丢失
 
    if (step == 0) {
        // 使用更敏感的阈值：大于目标值0.5倍就算变长
        if (right_front > params_.wall_distance * 1.5f || right_front > 5.0f) {
            ROS_INFO("右前变长(%.2fm)，等待右后也变长", right_front);
            step = 1;
            
        }
        return;
    }

  
    // step1 中增加超时，避免一直等待
    if (step == 1) {
        static ros::Time wait_start_time = ros::Time::now();
        static bool wait_started = false;
        
        if (!wait_started) {
            wait_start_time = ros::Time::now();
            wait_started = true;
        }
        
        float wait_elapsed = (ros::Time::now() - wait_start_time).toSec();
        
        if (right_rear > params_.wall_distance * 1.5f || right_rear > 5.0f) {
            // 条件满足
            step = 2;
            wait_started = false;
            // ...
        } else if (wait_elapsed > 5.0f) {
            // 超时5秒，强制进入下一步
            ROS_WARN("等待右后变长超时，强制前进");
            step = 2;
            wait_started = false;
            start_x = current_pose_.x;
        } else {
            applyKeyboardCommand('w');
        }
        return;
    }
 
    // step2: 前进 wall_distance 长度
    if (step == 2) {
        if (!move_started) {
            move_start_time = ros::Time::now();
            move_started = true;
        }
        
        float elapsed = (ros::Time::now() - move_start_time).toSec();
        float traveled = params_.speed * elapsed;
        
        // 累加里程
        travel_.segment_traveled += traveled;
        travel_.total_traveled += traveled;
        
        ROS_INFO("前进: 已走%.3f/%.3f米", traveled, params_.wall_distance);
        
        if (traveled >= params_.wall_distance) {
            stopMoving();
            step = 3;
            move_started = false;
            ROS_INFO("前进完成，开始右转");
        } else {
            applyKeyboardCommand('w');
        }
        return;
    }
    // step3: 右转90度
    if (step == 3) {
        if (!turn_started) {
            turn_start_yaw = current_pose_.theta;
            turn_start_time = ros::Time::now();
            turn_started = true;
           
            ROS_INFO("开始右转90度，起始角度=%.1f°", turn_start_yaw * RAD_TO_DEG);
        }
        
        applyKeyboardCommand('d');  // 右转
        // 计算累计转角（右转时角度应该减小）
        float yaw_change = current_pose_.theta - turn_start_yaw;
        // 处理360°跨越：如果差值大于π，说明转过了0°
        if (yaw_change > PI_M) {
            yaw_change = yaw_change - 2 * PI_M;
        }
        float abs_yaw_change = fabs(yaw_change);
        float elapsed = (ros::Time::now() - turn_start_time).toSec();
        
        ROS_INFO("右转: 累计转角=%.1f°", abs_yaw_change * RAD_TO_DEG);
        
        if (abs_yaw_change > 85.0f * DEG_TO_RAD || elapsed > 5.0f) {
            stopMoving();
            step = 0;           // 直接回到 step0，不经过 step4
            turn_started = false;
            state_ = STATE_ALIGNING;  // 直接进入对齐
            accumulated_yaw_ -= PI_M / 2.0f; 
            ROS_INFO("右转完成，进入对齐");
        }
        return;
    }
 
}

float WallFollower::getRightUltrasoundMin() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    float dist = std::min(sensors_.ultra_right_front, sensors_.ultra_right_rear);
    
    // 过滤异常值
    if (dist < 0.05f || dist > 5.0f) {
        return 999.0f;
    }
    return dist;
}

bool WallFollower::checkFrontObstacle() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    float front_left = sensors_.infra_front_left;
    float front_right = sensors_.infra_front_right;
    
    // 红外有效范围 0.1m-1.0m，只要有一个有效就算有墙
    bool left_valid = (front_left > 0.1f && front_left < 1.0f);
    bool right_valid = (front_right > 0.1f && front_right < 1.0f);
    
    // 任意一个有效就返回true
    return left_valid || right_valid;
}

float WallFollower::calculateTurnAngle() {
    // 默认90度直角转弯
    float default_angle = PI_M / 2.0f;
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 使用前方红外传感器（最远1米，所以只在小距离时有效）
    float front_left = sensors_.infra_front_left;
    float front_right = sensors_.infra_front_right;
    
    // 如果两个传感器都有效（距离在合理范围内）
    if (front_left < 0.9f && front_right < 0.9f && front_left > 0.1f && front_right > 0.1f) {
        const float SENSOR_WIDTH = 0.3f;  // 左右传感器间距
        float diff = front_right - front_left;
        float wall_angle = atan2(diff, SENSOR_WIDTH);
        
        // 根据墙面角度调整转弯角度
        if (fabs(wall_angle) > 0.1f) {
            float turn_angle = PI_M / 2.0f + wall_angle * 0.5f;
            turn_angle = std::max(0.7f, std::min(1.8f, turn_angle));
            ROS_INFO("检测到非直角墙面: 墙面角度=%.1f°, 转弯角度=%.1f°",
                    wall_angle * RAD_TO_DEG, turn_angle * RAD_TO_DEG);
            return turn_angle;
        }
    }
    
    return default_angle;
}

bool WallFollower::checkCompletion() {
    // 条件1：总里程达到设定值（确保走够了距离）
    if (travel_.total_traveled < params_.complete_distance) {
        return false;
    }
    
  
    // 条件2：累积转角接近360°（或 -360°）
    float total_turn = fabs(accumulated_yaw_);
    bool yaw_ok = (fabs(total_turn - 2 * PI_M) < 30.0f * DEG_TO_RAD);
    
    // 条件3：检查与起点的距离（位置回到起点附近）
    float dx = current_pose_.x - travel_.start_pose.x;
    float dy = current_pose_.y - travel_.start_pose.y;
    float distance_to_start = sqrt(dx*dx + dy*dy);
    bool distance_ok = (distance_to_start < params_.finish_distance);
    
    // 三重校验：里程足够 + 转角正确 + 位置接近
    if (yaw_ok && distance_ok) {
        ROS_INFO("沿边完成! 总里程=%.2fm, 总转角=%.1f°, 距离起点=%.2fm", 
                 travel_.total_traveled, 
                 total_turn * RAD_TO_DEG, 
                 distance_to_start);
        return true;
    }
    
    // 如果只有转角正确但距离不对，继续行走（可能还没完全回到起点）
    if (yaw_ok) {
        ROS_INFO_THROTTLE(5.0, "转角正确(%.1f°)，但距离起点=%.2fm(>%.2fm)，继续行走", 
                          total_turn * RAD_TO_DEG, 
                          distance_to_start, 
                          params_.finish_distance);
    }
    
    return false;
}

void WallFollower::applyKeyboardCommand(char key) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.angular.z = 0.0f;
    cmd.linear.y = pump_speed_;

    switch (key) {
        case 'w': cmd.linear.x = params_.speed; break;
        case 'x': cmd.linear.x = -params_.speed; break;
        case 'a': cmd.angular.z = 0.1f; break;  // 减小到0.1
        case 'd': cmd.angular.z = -0.1f; break; // 减小到0.1
        case 's': break;
        default: break;
    }
    cmd_vel_pub_.publish(cmd);
}

void WallFollower::stopMoving() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.angular.z = 0.0f;
    cmd.linear.y = 0.0f;  // 关闭水泵
    cmd_vel_pub_.publish(cmd);
}

void WallFollower::setLinearSpeed(float speed) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = std::max(-MAX_LINEAR, std::min(MAX_LINEAR, speed));
    cmd.angular.z = 0.0f;
    cmd.linear.y = pump_speed_;
    cmd_vel_pub_.publish(cmd);
}

void WallFollower::setAngularSpeed(float speed) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0f;
    cmd.angular.z = std::max(-MAX_ANGULAR, std::min(MAX_ANGULAR, speed));
    cmd.linear.y = pump_speed_; 
    cmd_vel_pub_.publish(cmd);
}

void WallFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_odom_ = *msg;
    odom_received_ = true;
}

void WallFollower::ultraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (msg->data.size() >= 4) {
        // 根据硬件代码映射:
        // data[0] = 前方右红外 (单位mm) -> infra_front_right
        // data[1] = 前方左红外 (单位mm) -> infra_front_left
        // data[2] = 右侧前超声波 (单位mm) -> ultra_right_front
        // data[3] = 右侧后超声波 (单位mm) -> ultra_right_rear
        sensors_.infra_front_right = msg->data[0] / 1000.0f;
        sensors_.infra_front_left = msg->data[1] / 1000.0f;
        // 修正映射
        sensors_.ultra_right_rear = msg->data[2] / 1000.0f;   // data[2] = 右后
        sensors_.ultra_right_front = msg->data[3] / 1000.0f;  // data[3] = 右前
        
        // 过滤红外异常值（红外最远1米，超过0.95米视为无效）
        if (sensors_.infra_front_right > 0.95f || sensors_.infra_front_right < 0.05f) {
            sensors_.infra_front_right = 999.0f;
        }
        if (sensors_.infra_front_left > 0.95f || sensors_.infra_front_left < 0.05f) {
            sensors_.infra_front_left = 999.0f;
        }
        
        // 过滤超声波异常值
        if (sensors_.ultra_right_front > 5.0f || sensors_.ultra_right_front < 0.05f) {
            sensors_.ultra_right_front = sensors_.ultra_right_rear;
        }
        if (sensors_.ultra_right_rear > 5.0f || sensors_.ultra_right_rear < 0.05f) {
            sensors_.ultra_right_rear = sensors_.ultra_right_front;
        }
            
        sensors_.last_update = ros::Time::now();
    }
}

void WallFollower::updateCurrentPose() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (odom_received_) {
        current_pose_.x = current_odom_.pose.pose.position.x;
        current_pose_.y = current_odom_.pose.pose.position.y;
        current_pose_.theta = calculateYawFromQuaternion(current_odom_.pose.pose.orientation);
    }
}

float WallFollower::calculateYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return static_cast<float>(yaw);
}

bool WallFollower::waitForOdom(float timeout) {
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
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

float WallFollower::normalizeAngle(float angle) {
    while (angle > PI_M) angle -= 2.0f * PI_M;
    while (angle < -PI_M) angle += 2.0f * PI_M;
    return angle;
}

void WallFollower::twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_twist_ = *msg;
}

} // namespace sonar_slam

// ========== main函数 ==========
int main(int argc, char** argv) {
    ros::init(argc, argv, "wall_follower_node");
    ros::NodeHandle nh("~");
    
    // 设置信号处理
 
    signal(SIGINT, ::signalHandler);    // 修改这里
    signal(SIGTERM, ::signalHandler);   // 修改这里
    
    ROS_INFO("========================================");
    ROS_INFO("WallFollower 节点启动 (基于按键模拟模式)");
    ROS_INFO("========================================");
    
    // 创建沿边行走控制器
    sonar_slam::WallFollower follower(nh);
    g_wall_follower = &follower;
    
    // 从参数服务器获取参数
    double wall_distance = 0.3;
    double speed = 0.15;
    double segment_distance = 2.0;
    double stop_duration = 45.0;
    double turn_distance = 0.8;
    double complete_distance = 3.0;
    double pump_speed = 0.5; 
    
    nh.param("wall_distance", wall_distance, 0.3);
    nh.param("speed", speed, 0.15);
    nh.param("segment_distance", segment_distance, 2.0);
    nh.param("stop_duration", stop_duration, 45.0);
    nh.param("turn_distance", turn_distance, 0.8);
    nh.param("complete_distance", complete_distance, 3.0);
    nh.param("pump_speed", pump_speed, 0.5); 
    double finish_distance = 1.0;
    nh.param("finish_distance", finish_distance, 1.0);

    ROS_INFO("参数配置:");
    ROS_INFO("  wall_distance = %.2f m", wall_distance);
    ROS_INFO("  speed = %.2f m/s", speed);
    ROS_INFO("  segment_distance = %.2f m", segment_distance);
    ROS_INFO("  stop_duration = %.0f s", stop_duration);
    ROS_INFO("  turn_distance = %.2f m", turn_distance);
    ROS_INFO("  complete_distance = %.2f m", complete_distance);
    
    // 是否自动开始
    bool auto_start = false;
    nh.param("auto_start", auto_start, false);
    
    if (auto_start) {
        ROS_INFO("开始自动沿边行走");
        follower.start(wall_distance, speed, segment_distance, 
                       stop_duration, turn_distance, complete_distance,finish_distance,pump_speed);
    } else {
        ROS_INFO("等待服务调用或设置 auto_start:=true");
        ROS_INFO("示例: rosrun sonar_slam wall_follower_node _auto_start:=true");
    }
    
    ROS_INFO("========================================");
    ROS_INFO("沿边行走控制器就绪");
    ROS_INFO("  状态: %d", follower.getState());
    ROS_INFO("========================================");
    
    ros::Rate rate(20);  // 20Hz
    while (ros::ok()) {
        follower.update();
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("沿边行走控制器关闭");
    return 0;
}