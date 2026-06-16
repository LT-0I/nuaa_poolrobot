/**
 * @file manual_point_collector.cpp
 * @brief 手动点位声呐数据采集节点 (manual_point_collector)
 *
 * 背景: 原版终端4(multi_position_sonar_mapper)自动沿边到 N 个点位停车采集; 自动运动差。
 * 本节点把"运动"交给操作员: 终端1 驱动 + mogo_teleop_keyboard.py 手动开车到点位停下,
 * 然后调用 ROS 服务 /point_collect/record 触发"采这个点位":
 *   - 扫描期间连续记录 odom 流(/wheel_odom);
 *   - 按原版逻辑驱动 Ping360 声呐(逐角发 0x0A29 命令, 解析 2300 帧), 扫 scan_duration 秒;
 *   - 落盘原始源数据(每帧 raw 强度采样) + 原版 per-angle 汇总 + 点云(.pcd) + meta。
 *
 * 数据层次: ~/sonar_logs/manual_collect/task_<时间戳>/point_NN/{odom_stream.csv,
 *           sonar_raw.csv, sonar_summary.csv, cloud.pcd, meta.txt}
 *   - task 夹: 每次启动本节点一个; point 夹: 每次 /point_collect/record 一个。
 *
 * 边界:
 *   - 本节点【不】发 /cmd_vel、不碰电机/水泵; 运动与泵由操作员的 teleop 负责。
 *   - 声呐与原版同样【会主动发射】: 必须【已入水】才可调用服务(空气中发射烧换能器)。
 *   - 单线程 ROS(ros::spin); CallbackAsyncSerial 自带 I/O 线程收串口。严禁 AsyncSpinner。
 *   - 终端打印一律 ASCII(机上无中文字库)。
 *
 * 声呐串口/协议/扫描/点云代码移植自 multi_position_sonar_mapper(活仓 main)。
 */

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>   // tf::getYaw

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "sonar_slam/AsyncSerial.h"

#include <boost/bind.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <memory>
#include <cmath>
#include <cstring>
#include <ctime>
#include <sys/stat.h>

using boost::placeholders::_1;
using boost::placeholders::_2;

namespace {
const float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
const float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);
const size_t RX_BUFFER_SIZE = 40960;
const uint16_t MAX_RAW_POINTS = 1200;
}

// ======================== 声呐 2300 消息结构(与硬件协议一致, 移植自 mapper 头) ========================
#pragma pack(push, 1)
struct MsgCmd2300 {
    uint8_t  mode;               // [0]
    uint8_t  gain_setting;       // [1]
    uint16_t angle;              // [2-3] 小端, 0-399
    uint16_t transmit_duration;  // [4-5] us
    uint16_t sample_period;      // [6-7] 25ns 为单位
    uint16_t transmit_frequency; // [8-9] kHz
    uint16_t number_of_samples;  // [10-11]
    uint16_t data_length;        // [12-13]
    uint8_t  data[1200];         // [14+] 强度(原始 0-255)
};
#pragma pack(pop)

// 一帧已收到的原始数据(源数据), data 仅保留有效采样
struct RawFrame {
    uint16_t angle;
    uint16_t sample_period;
    uint16_t number_of_samples;
    uint16_t data_length;
    std::vector<uint8_t> data;
    double   wall_t;
};

// 一帧 odom 采样
struct OdomSample {
    double stamp;
    double x, y, yaw;
    double vx, wz;
};

// ============================================================================
class ManualPointCollector {
public:
    ManualPointCollector();
    ~ManualPointCollector();

private:
    // ---- 串口接收/解析(移植自 mapper) ----
    bool initSerial(const std::string& port, unsigned int baudrate);
    void receiveDataCallback(const char* data, size_t len);
    void parseDataPacket();
    void processCompleteMessage();
    bool parseCmd2300Message(const std::vector<uint8_t>& data);
    void processScanFrame(const MsgCmd2300& msg);

    // ---- 控制命令(逐角驱动声呐, 移植自 mapper) ----
    void generateControlCommand(unsigned char* buffer, unsigned short angle);
    uint16_t samplesForLevel(const std::string& level) const;

    // ---- ROS ----
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    bool onRecord(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    // ---- 采集流程 ----
    void runCollect(const std::string& point_dir);
    void pumpOnce(std::ofstream& odom_ofs, ros::Time& next_odom, double odom_period);
    bool angleReceived(unsigned short angle);

    // ---- 落盘 ----
    void writeRawCsv(const std::string& dir, const std::vector<RawFrame>& frames);
    void writeSummaryCsv(const std::string& dir, const std::vector<RawFrame>& frames, int angle_base);
    void writeCloudPcd(const std::string& dir, const std::vector<RawFrame>& frames, int angle_base);
    void writeMeta(const std::string& dir, const ros::Time& t_start, const ros::Time& t_end, size_t n_frames);

    static bool ensureDir(const std::string& path);

    // ---- 成员 ----
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer record_srv_;
    std::unique_ptr<CallbackAsyncSerial> sonar_serial_;

    // 参数(默认对齐原版终端4命令)
    std::string serial_port_;
    int         baudrate_;
    double      scan_duration_;
    std::string range_level_;
    double      odom_rate_hz_;
    double      min_range_;

    float max_range_;   // 由 range_level_ 推得(samples*25ns*1500/2)

    // 串口接收缓冲 + 帧同步状态(仅 I/O 线程访问)
    unsigned char rx_buffer_[RX_BUFFER_SIZE];
    size_t rx_end_index_;
    size_t rx_used_index_;
    enum ParseState {
        STATE_IDLE, STATE_GOT_START1,
        STATE_READING_LENGTH_LOW, STATE_READING_LENGTH_HIGH,
        STATE_READING_COMMAND_LOW, STATE_READING_COMMAND_HIGH,
        STATE_READING_SOURCE, STATE_READING_DESTINATION,
        STATE_READING_DATA
    };
    ParseState parse_state_;
    struct CurrentMessage {
        uint16_t length;
        uint16_t command;
        uint8_t  source;
        uint8_t  destination;
        std::vector<uint8_t> data;
        void clear() { length = 0; command = 0; source = 0; destination = 0; data.clear(); }
    } current_message_;

    std::atomic<bool> is_scanning_;   // gate 串口回调(init 后恒真, 析构置假)
    std::atomic<bool> collecting_;    // 仅采集窗口内缓冲帧

    // 跨线程共享: I/O 线程写, 主线程读/清(frame_mutex_ 守)
    std::mutex frame_mutex_;
    std::vector<RawFrame> frames_;
    std::vector<char> angle_received_;  // size 400

    // odom(仅主线程 spinOnce 访问 → 与采集循环同线程, 无需锁)
    OdomSample latest_odom_;
    bool have_odom_;

    // 任务/点位目录
    std::string task_dir_;
    bool task_dir_ok_;
    int point_counter_;
};

// ======================== 构造 / 析构 ========================
ManualPointCollector::ManualPointCollector()
    : pnh_("~"),
      max_range_(4.0f),
      rx_end_index_(0), rx_used_index_(0),
      parse_state_(STATE_IDLE),
      is_scanning_(false), collecting_(false),
      angle_received_(400, 0),
      have_odom_(false),
      task_dir_ok_(false),
      point_counter_(0)
{
    current_message_.clear();
    latest_odom_ = OdomSample{0, 0, 0, 0, 0, 0};

    // 参数(默认对齐原版终端4命令: /dev/ttyACM0 @115200, scan_duration=95, range_level=s0)
    pnh_.param("serial_port", serial_port_, std::string("/dev/ttyACM0"));
    pnh_.param("baudrate",    baudrate_,    115200);
    pnh_.param("scan_duration", scan_duration_, 95.0);
    pnh_.param("range_level", range_level_, std::string("s0"));
    pnh_.param("odom_rate_hz", odom_rate_hz_, 20.0);
    pnh_.param("min_range",   min_range_,   0.1);
    if (odom_rate_hz_ < 1.0) odom_rate_hz_ = 1.0;
    if (scan_duration_ < 1.0) scan_duration_ = 1.0;
    {
        static const char* kLevels[] =
            {"s0","s1","s2","s3","s4","s5","s6","s7","s8","s9","s10","s11"};
        bool known = false;
        for (const char* lv : kLevels) { if (range_level_ == lv) { known = true; break; } }
        if (!known) {
            ROS_WARN("range_level '%s' unrecognized -> using default (s1/6m). valid: s0..s11",
                     range_level_.c_str());
        }
        uint16_t s = samplesForLevel(range_level_);
        max_range_ = s * 25e-6f * 1500.0f / 2.0f;
    }

    // task 目录: ~/sonar_logs/manual_collect/task_<时间戳>[_N]/
    std::string home = getenv("HOME") ? getenv("HOME") : ".";
    std::string base = home + "/sonar_logs/";
    std::string mc   = base + "manual_collect/";
    char ts[32];
    std::time_t now_t = std::time(nullptr);
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
    ensureDir(base); ensureDir(mc);
    // 同秒内重启不能复用旧 task 夹(ensureDir 命中既存即返回真)->追加 _N 后缀, 防覆盖既有数据
    {
        std::string cand = mc + "task_" + ts;
        std::string chosen = cand;
        struct stat stx;
        int suffix = 0;
        while (stat(chosen.c_str(), &stx) == 0) {
            chosen = cand + "_" + std::to_string(++suffix);
        }
        task_dir_ = chosen + "/";
    }
    task_dir_ok_ = ensureDir(task_dir_);
    if (!task_dir_ok_) {
        ROS_ERROR("FAILED to create task dir %s -- check HOME/permissions; records will be refused",
                  task_dir_.c_str());
    }

    ROS_INFO("manual_point_collector params:");
    ROS_INFO("  serial_port=%s baudrate=%d", serial_port_.c_str(), baudrate_);
    ROS_INFO("  scan_duration=%.1fs range_level=%s (max_range=%.2fm)",
             scan_duration_, range_level_.c_str(), max_range_);
    ROS_INFO("  odom_rate=%.1fHz min_range=%.2fm", odom_rate_hz_, min_range_);
    ROS_INFO("  task_dir=%s", task_dir_.c_str());

    if (!initSerial(serial_port_, static_cast<unsigned int>(baudrate_))) {
        ROS_ERROR("sonar serial init FAILED on %s -- no sonar data will be collected", serial_port_.c_str());
    } else {
        is_scanning_.store(true);
    }

    odom_sub_ = nh_.subscribe("/wheel_odom", 200, &ManualPointCollector::odomCallback, this);
    record_srv_ = nh_.advertiseService("/point_collect/record", &ManualPointCollector::onRecord, this);

    ROS_WARN("SAFETY: sonar transmits actively -- ensure robot is IN WATER before calling /point_collect/record");
    ROS_INFO("ready. drive manually, stop at a point, then: rosservice call /point_collect/record");
}

ManualPointCollector::~ManualPointCollector() {
    is_scanning_.store(false);
    collecting_.store(false);
    if (sonar_serial_) {
        try { sonar_serial_->close(); } catch (...) {}
    }
}

// ======================== 串口初始化(移植 mapper:310-329) ========================
bool ManualPointCollector::initSerial(const std::string& port, unsigned int baudrate) {
    try {
        sonar_serial_ = std::unique_ptr<CallbackAsyncSerial>(new CallbackAsyncSerial(port, baudrate));
        sonar_serial_->setCallback(
            boost::bind(&ManualPointCollector::receiveDataCallback, this, _1, _2));
        parse_state_ = STATE_IDLE;
        current_message_.clear();
        rx_end_index_ = 0;
        rx_used_index_ = 0;
        ROS_INFO("serial init success: %s @ %u baud", port.c_str(), baudrate);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("serial init failed %s: %s", port.c_str(), e.what());
        return false;
    }
}

// ======================== 串口回调(I/O 线程, 移植 mapper:333-352) ========================
void ManualPointCollector::receiveDataCallback(const char* data, size_t len) {
    if (len == 0 || !is_scanning_.load()) {
        return;
    }
    if (rx_end_index_ + len <= RX_BUFFER_SIZE) {
        memcpy(rx_buffer_ + rx_end_index_, data, len);
        rx_end_index_ += len;
    } else {
        ROS_WARN("RX buffer overflow, discarding");
        rx_end_index_ = 0;
        rx_used_index_ = 0;
    }
    parseDataPacket();
}

// ======================== 帧同步状态机(移植 mapper:355-479, 逐字节) ========================
void ManualPointCollector::parseDataPacket() {
    while (rx_used_index_ < rx_end_index_) {
        unsigned char current_byte = rx_buffer_[rx_used_index_];

        switch (parse_state_) {
            case STATE_IDLE:
                if (current_byte == 0x42) {  // 'B'
                    parse_state_ = STATE_GOT_START1;
                }
                rx_used_index_++;
                break;

            case STATE_GOT_START1:
                if (current_byte == 0x52) {  // 'R'
                    parse_state_ = STATE_READING_LENGTH_LOW;
                } else {
                    parse_state_ = STATE_IDLE;
                }
                rx_used_index_++;
                break;

            case STATE_READING_LENGTH_LOW:
                current_message_.length = current_byte;
                parse_state_ = STATE_READING_LENGTH_HIGH;
                rx_used_index_++;
                break;

            case STATE_READING_LENGTH_HIGH:
                current_message_.length |= (current_byte << 8);
                if (current_message_.length > 2000 || current_message_.length < 100) {
                    ROS_WARN_THROTTLE(5.0, "frame length abnormal: %u, resync", current_message_.length);
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                } else {
                    parse_state_ = STATE_READING_COMMAND_LOW;
                }
                rx_used_index_++;
                break;

            case STATE_READING_COMMAND_LOW:
                current_message_.command = current_byte;
                parse_state_ = STATE_READING_COMMAND_HIGH;
                rx_used_index_++;
                break;

            case STATE_READING_COMMAND_HIGH:
                current_message_.command |= (current_byte << 8);
                parse_state_ = STATE_READING_SOURCE;
                rx_used_index_++;
                break;

            case STATE_READING_SOURCE:
                current_message_.source = current_byte;
                parse_state_ = STATE_READING_DESTINATION;
                rx_used_index_++;
                break;

            case STATE_READING_DESTINATION:
                current_message_.destination = current_byte;
                if (current_message_.length > 0) {
                    current_message_.data.clear();
                    current_message_.data.reserve(current_message_.length);
                    parse_state_ = STATE_READING_DATA;
                } else {
                    processCompleteMessage();
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                }
                rx_used_index_++;
                break;

            case STATE_READING_DATA:
                current_message_.data.push_back(current_byte);
                if (current_message_.data.size() >= current_message_.length) {
                    processCompleteMessage();
                    parse_state_ = STATE_IDLE;
                    current_message_.clear();
                }
                rx_used_index_++;
                break;
        }
    }

    // 剩余缓冲中搜索下一个帧头 0x42 0x52
    if (parse_state_ == STATE_IDLE && (rx_end_index_ - rx_used_index_) >= 2) {
        for (size_t i = rx_used_index_; i < rx_end_index_ - 1; i++) {
            if (rx_buffer_[i] == 0x42 && rx_buffer_[i + 1] == 0x52) {
                size_t remaining = rx_end_index_ - i;
                memmove(rx_buffer_, rx_buffer_ + i, remaining);
                rx_end_index_ = remaining;
                rx_used_index_ = 0;
                break;
            }
        }
    }

    // 缓冲区压缩
    if (rx_used_index_ > RX_BUFFER_SIZE / 2) {
        size_t remaining = rx_end_index_ - rx_used_index_;
        if (remaining > 0) {
            memmove(rx_buffer_, rx_buffer_ + rx_used_index_, remaining);
        }
        rx_end_index_ = remaining;
        rx_used_index_ = 0;
    }
}

// ======================== 完整消息分发(移植 mapper:483-543, 去调试) ========================
void ManualPointCollector::processCompleteMessage() {
    // 跳过命令回显(data 以 0x42 0x52 开头)
    if (current_message_.data.size() >= 2 &&
        current_message_.data[0] == 0x42 &&
        current_message_.data[1] == 0x52) {
        return;
    }
    if (current_message_.command == 2300) {
        parseCmd2300Message(current_message_.data);
    }
}

// ======================== 解析 2300(移植 mapper:547-682, 去调试) ========================
bool ManualPointCollector::parseCmd2300Message(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(MsgCmd2300)) {
        return false;
    }
    MsgCmd2300 msg;
    memcpy(&msg, data.data(), sizeof(MsgCmd2300));
    if (msg.angle >= 400) {
        return false;
    }
    if (msg.number_of_samples > MAX_RAW_POINTS) {
        return false;
    }
    processScanFrame(msg);
    return true;
}

// ======================== 缓冲一帧源数据(I/O 线程; 仅采集窗口内) ========================
void ManualPointCollector::processScanFrame(const MsgCmd2300& msg) {
    if (!collecting_.load()) {
        return;
    }
    uint16_t actual_samples = std::min(msg.number_of_samples, msg.data_length);
    if (actual_samples > MAX_RAW_POINTS) actual_samples = MAX_RAW_POINTS;

    // 空帧(0 采样): 不落库(免污染 raw/汇总/帧数), 但仍记"本角已回包"以维持节拍
    if (actual_samples == 0) {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        if (msg.angle < 400) angle_received_[msg.angle] = 1;
        return;
    }

    RawFrame rf;
    rf.angle = msg.angle;
    rf.sample_period = msg.sample_period;
    rf.number_of_samples = msg.number_of_samples;
    rf.data_length = msg.data_length;
    rf.data.assign(msg.data, msg.data + actual_samples);
    rf.wall_t = ros::Time::now().toSec();

    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (frames_.size() < 200000) {   // 低 RAM 板防失控增长(正常一次采集~数百帧)
        frames_.push_back(std::move(rf));
    }
    if (msg.angle < 400) angle_received_[msg.angle] = 1;
}

// ======================== 控制命令(移植 mapper:1132-1241) ========================
uint16_t ManualPointCollector::samplesForLevel(const std::string& level) const {
    if (level == "s0")  return 0x00D6;  // 214 点, ~4.0m
    if (level == "s1")  return 0x0140;  // 320 点, 6.0m
    if (level == "s2")  return 0x01AB;  // 427 点, ~8.0m
    if (level == "s3")  return 0x0215;  // 533 点, ~10.0m
    if (level == "s4")  return 0x0280;  // 640 点, 12.0m
    if (level == "s5")  return 0x02EA;  // 746 点, ~14.0m
    if (level == "s6")  return 0x0320;  // 800 点, 15.0m
    if (level == "s7")  return 0x0355;  // 853 点, ~16.0m
    if (level == "s8")  return 0x03C0;  // 960 点, 18.0m
    if (level == "s9")  return 0x042B;  // 1067 点, ~20.0m
    if (level == "s10") return 0x0490;  // 1168 点, ~22.0m
    if (level == "s11") return 0x0500;  // 1280 点, 24.0m
    return 0x0140;                      // 默认 320 点, 6.0m
}

void ManualPointCollector::generateControlCommand(unsigned char* buffer, unsigned short angle) {
    memset(buffer, 0, 24);
    buffer[0] = 0x42;  // 'B'
    buffer[1] = 0x52;  // 'R'
    buffer[2] = 0x0E;  // 长度低 (14)
    buffer[3] = 0x00;
    buffer[4] = 0x29;  // 命令低 (0x0A29)
    buffer[5] = 0x0A;
    buffer[6] = 0x00;  // 源
    buffer[7] = 0x00;  // 目的
    buffer[8] = 0x01;  // 模式
    buffer[9] = 0x00;  // 增益
    buffer[10] = angle & 0xFF;          // 角度低
    buffer[11] = (angle >> 8) & 0xFF;   // 角度高
    buffer[12] = 0x6B;  // 发射持续时间 107us
    buffer[13] = 0x00;

    uint16_t samples = samplesForLevel(range_level_);
    buffer[14] = samples & 0xFF;
    buffer[15] = (samples >> 8) & 0xFF;
    // max_range_ 已在构造时按 range_level_ 算定; 此处不再改成员(避免与落盘线程的意外耦合)

    buffer[16] = 0xEE;  // 发射频率 750
    buffer[17] = 0x02;
    buffer[18] = 0xB0;  // 采样点数 1200
    buffer[19] = 0x04;
    buffer[20] = 0x01;  // 数据长度 1
    buffer[21] = 0x00;

    uint16_t checksum = 0;
    for (int i = 0; i < 22; i++) checksum += buffer[i];
    buffer[22] = checksum & 0xFF;
    buffer[23] = (checksum >> 8) & 0xFF;
}

// ======================== odom 回调(主线程 spinOnce) ========================
void ManualPointCollector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_.stamp = msg->header.stamp.toSec();
    latest_odom_.x = msg->pose.pose.position.x;
    latest_odom_.y = msg->pose.pose.position.y;
    latest_odom_.yaw = tf::getYaw(msg->pose.pose.orientation);
    latest_odom_.vx = msg->twist.twist.linear.x;
    latest_odom_.wz = msg->twist.twist.angular.z;
    have_odom_ = true;
}

// ======================== 服务: 触发采一个点位 ========================
bool ManualPointCollector::onRecord(std_srvs::Trigger::Request& /*req*/,
                                    std_srvs::Trigger::Response& res) {
    if (collecting_.load()) {
        res.success = false;
        res.message = "busy: a point is being collected";
        return true;
    }
    if (!sonar_serial_) {
        res.success = false;
        res.message = "sonar serial not initialized";
        return true;
    }
    if (!task_dir_ok_) {
        res.success = false;
        res.message = "task dir not writable: " + task_dir_;
        return true;
    }
    point_counter_++;
    std::ostringstream pd;
    pd << task_dir_ << "point_" << std::setw(2) << std::setfill('0') << point_counter_ << "/";
    std::string point_dir = pd.str();
    if (!ensureDir(point_dir)) {
        res.success = false;
        res.message = "failed to create point dir: " + point_dir;
        point_counter_--;
        return true;
    }
    ROS_INFO("=== collecting point %d -> %s (scan %.1fs) ===",
             point_counter_, point_dir.c_str(), scan_duration_);
    runCollect(point_dir);
    res.success = true;
    res.message = point_dir;
    ROS_INFO("=== point %d done -> %s ===", point_counter_, point_dir.c_str());
    return true;
}

bool ManualPointCollector::angleReceived(unsigned short angle) {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    return angle < 400 ? (angle_received_[angle] != 0) : false;
}

// 一次 spin + 按 odom_rate 采样写流 + 5ms 节拍
void ManualPointCollector::pumpOnce(std::ofstream& odom_ofs, ros::Time& next_odom, double odom_period) {
    ros::spinOnce();  // 主线程驱动 odom 回调
    ros::Time now = ros::Time::now();
    if (now >= next_odom) {
        if (have_odom_ && odom_ofs.is_open()) {
            if (now.toSec() - latest_odom_.stamp > 1.0) {
                ROS_WARN_THROTTLE(5.0, "odom stale (>1s) -- /wheel_odom may have stopped; poses frozen");
            }
            odom_ofs << std::fixed << std::setprecision(6)
                     << now.toSec() << ","
                     << latest_odom_.stamp << ","
                     << latest_odom_.x << "," << latest_odom_.y << "," << latest_odom_.yaw << ","
                     << latest_odom_.vx << "," << latest_odom_.wz << "\n";
        }
        next_odom = now + ros::Duration(odom_period);
    }
    ros::Duration(0.005).sleep();
}

// ======================== 采集流程(内联 mapper scanningThread 的逐角命令循环) ========================
void ManualPointCollector::runCollect(const std::string& point_dir) {
    std::ofstream odom_ofs(point_dir + "odom_stream.csv", std::ios::out | std::ios::trunc);
    if (odom_ofs.is_open()) {
        odom_ofs << "wall_t,odom_stamp,x,y,yaw_rad,twist_vx,twist_wz\n";
    } else {
        ROS_WARN("cannot open odom_stream.csv in %s", point_dir.c_str());
    }

    {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        frames_.clear();
        std::fill(angle_received_.begin(), angle_received_.end(), 0);
    }
    collecting_.store(true);

    const double odom_period = 1.0 / odom_rate_hz_;
    ros::Time scan_start = ros::Time::now();
    ros::Time next_odom = scan_start;
    unsigned short angle = 0;

    while ((ros::Time::now() - scan_start).toSec() < scan_duration_ && ros::ok()) {
        ROS_INFO_THROTTLE(10.0, "scanning... %.0f/%.0fs (angle=%u)",
                          (ros::Time::now() - scan_start).toSec(), scan_duration_, angle);
        // 1. 先清本角接收标志(必须在发命令前: 否则极快回包会被这次清零吞掉->空等 1.5s)
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            if (angle < 400) angle_received_[angle] = 0;
        }
        // 2. 发本角发射命令
        unsigned char cmd[24];
        generateControlCommand(cmd, angle);
        if (sonar_serial_) {
            try {
                sonar_serial_->write(reinterpret_cast<char*>(cmd), 24);
            } catch (const std::exception& e) {
                ROS_ERROR_THROTTLE(5.0, "serial write failed: %s", e.what());
            }
        }
        // 3. 等本角数据返回(<=1.5s), 期间持续 spin + 采 odom
        ros::Time wait_start = ros::Time::now();
        while (!angleReceived(angle) &&
               (ros::Time::now() - wait_start).toSec() < 1.5 &&
               (ros::Time::now() - scan_start).toSec() < scan_duration_ &&
               ros::ok()) {
            pumpOnce(odom_ofs, next_odom, odom_period);
        }
        // 4. 下一个角(满 400 回零)
        angle++;
        if (angle >= 400) {
            angle = 0;
            std::lock_guard<std::mutex> lk(frame_mutex_);
            std::fill(angle_received_.begin(), angle_received_.end(), 0);
        }
        // 5. 角间节拍 ~0.1s, 仍持续采 odom
        ros::Time pace_start = ros::Time::now();
        while ((ros::Time::now() - pace_start).toSec() < 0.1 &&
               (ros::Time::now() - scan_start).toSec() < scan_duration_ &&
               ros::ok()) {
            pumpOnce(odom_ofs, next_odom, odom_period);
        }
    }

    collecting_.store(false);
    if (odom_ofs.is_open()) { odom_ofs.flush(); odom_ofs.close(); }

    // 快照帧(脱离锁后落盘)
    std::vector<RawFrame> snap;
    {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        snap.swap(frames_);
    }
    ros::Time t_end = ros::Time::now();
    // 本循环恒从设备角 0 起逐角扫描, 故直接用设备原始角作绝对角轴(=减 0)。
    // 不取 snap.front().angle: 否则一旦首帧是乱序/迟到帧, 整片点云会被任意旋转(评审 HIGH)。
    int angle_base = 0;

    writeRawCsv(point_dir, snap);
    writeSummaryCsv(point_dir, snap, angle_base);
    writeCloudPcd(point_dir, snap, angle_base);
    writeMeta(point_dir, scan_start, t_end, snap.size());
    if (snap.empty()) {
        ROS_WARN("NO sonar frames captured -- check sonar serial / range_level / robot in-water");
    }
    ROS_INFO("point saved: %zu sonar frames", snap.size());
}

// ======================== 落盘: 原始源数据(一帧一行) ========================
void ManualPointCollector::writeRawCsv(const std::string& dir, const std::vector<RawFrame>& frames) {
    std::ofstream f(dir + "sonar_raw.csv", std::ios::out | std::ios::trunc);
    if (!f.is_open()) { ROS_WARN("cannot open sonar_raw.csv"); return; }
    f << "frame_idx,wall_t,angle_raw,sample_period,number_of_samples,data_length,intensities\n";
    for (size_t k = 0; k < frames.size(); k++) {
        const RawFrame& rf = frames[k];
        f << k << ","
          << std::fixed << std::setprecision(6) << rf.wall_t << ","
          << rf.angle << "," << rf.sample_period << ","
          << rf.number_of_samples << "," << rf.data_length << ",";
        for (size_t i = 0; i < rf.data.size(); i++) {
            if (i) f << ';';
            f << static_cast<int>(rf.data[i]);
        }
        f << "\n";
    }
    f.close();
}

// ======================== 落盘: per-angle 汇总(对齐 mapper logToCSV) ========================
void ManualPointCollector::writeSummaryCsv(const std::string& dir, const std::vector<RawFrame>& frames, int angle_base) {
    std::ofstream f(dir + "sonar_summary.csv", std::ios::out | std::ios::trunc);
    if (!f.is_open()) { ROS_WARN("cannot open sonar_summary.csv"); return; }
    f << "timestamp,angle_deg,num_points,min_range,max_range,min_intensity,max_intensity\n";
    for (const RawFrame& rf : frames) {
        int corrected = static_cast<int>(rf.angle) - angle_base;
        while (corrected < 0) corrected += 400;
        while (corrected >= 400) corrected -= 400;
        float angle_deg = static_cast<float>(corrected) * 0.9f;
        angle_deg = fmodf(angle_deg, 360.0f);
        if (angle_deg < 0.0f) angle_deg += 360.0f;

        float dpp_mm = static_cast<float>(rf.sample_period) * 25.0f / 1000.0f * 0.73f;
        float min_r = 0, max_r = 0, min_i = 0, max_i = 0;
        size_t n = 0;
        for (size_t i = 0; i < rf.data.size(); i++) {
            float dist_m = dpp_mm * (i + 1) * 0.001f;
            if (dist_m > max_range_ || dist_m < static_cast<float>(min_range_)) continue;
            float inten = static_cast<float>(rf.data[i]);
            if (n == 0) { min_r = max_r = dist_m; min_i = max_i = inten; }
            else {
                if (dist_m < min_r) min_r = dist_m; if (dist_m > max_r) max_r = dist_m;
                if (inten < min_i) min_i = inten;   if (inten > max_i) max_i = inten;
            }
            n++;
        }
        f << std::fixed << std::setprecision(6)
          << rf.wall_t << "," << angle_deg << "," << n << ","
          << min_r << "," << max_r << "," << min_i << "," << max_i << "\n";
    }
    f.close();
}

// ======================== 落盘: 点云(对齐 mapper convertToPointCloud 的 90 度旋转) ========================
void ManualPointCollector::writeCloudPcd(const std::string& dir, const std::vector<RawFrame>& frames, int angle_base) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    const float rot = 90.0f * DEG_TO_RAD;
    const float cos_t = cosf(rot), sin_t = sinf(rot);

    for (const RawFrame& rf : frames) {
        int corrected = static_cast<int>(rf.angle) - angle_base;
        while (corrected < 0) corrected += 400;
        while (corrected >= 400) corrected -= 400;
        float angle_deg = static_cast<float>(corrected) * 0.9f;
        angle_deg = fmodf(angle_deg, 360.0f);
        if (angle_deg < 0.0f) angle_deg += 360.0f;
        float angle_rad = angle_deg * DEG_TO_RAD;

        float dpp_mm = static_cast<float>(rf.sample_period) * 25.0f / 1000.0f * 0.73f;
        for (size_t i = 0; i < rf.data.size(); i++) {
            float dist_m = dpp_mm * (i + 1) * 0.001f;
            if (dist_m > max_range_ || dist_m < static_cast<float>(min_range_)) continue;
            float xx = dist_m * cosf(angle_rad);
            float yy = dist_m * sinf(angle_rad);
            pcl::PointXYZI p;
            p.x = cos_t * xx - sin_t * yy;
            p.y = sin_t * xx + cos_t * yy;
            p.z = 0.0f;
            p.intensity = static_cast<float>(rf.data[i]);
            cloud.points.push_back(p);
        }
    }

    if (cloud.points.empty()) {
        ROS_WARN("point cloud empty -> skip cloud.pcd");
        return;
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    try {
        pcl::io::savePCDFileBinary(dir + "cloud.pcd", cloud);
    } catch (const std::exception& e) {
        ROS_WARN("savePCDFile failed: %s", e.what());
    }
}

// ======================== 落盘: meta ========================
void ManualPointCollector::writeMeta(const std::string& dir, const ros::Time& t_start,
                                     const ros::Time& t_end, size_t n_frames) {
    std::ofstream f(dir + "meta.txt", std::ios::out | std::ios::trunc);
    if (!f.is_open()) { ROS_WARN("cannot open meta.txt"); return; }
    std::time_t st = static_cast<std::time_t>(t_start.toSec());
    char sbuf[32]; std::strftime(sbuf, sizeof(sbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&st));
    f << "range_level=" << range_level_ << "\n"
      << "max_range_m=" << std::fixed << std::setprecision(3) << max_range_ << "\n"
      << "min_range_m=" << min_range_ << "\n"
      << "scan_duration_cfg_s=" << scan_duration_ << "\n"
      << "scan_elapsed_s=" << (t_end - t_start).toSec() << "\n"
      << "start_wall=" << sbuf << " (" << std::setprecision(6) << t_start.toSec() << ")\n"
      << "end_unix=" << t_end.toSec() << "\n"
      << "serial_port=" << serial_port_ << "\n"
      << "baudrate=" << baudrate_ << "\n"
      << "odom_rate_hz=" << odom_rate_hz_ << "\n"
      << "sonar_frames=" << n_frames << "\n"
      << "note=/wheel_odom twist.linear.x is pinned 0 and pose.yaw is raw gyro integration (drifts); do NOT over-trust odom pose.\n"
      << "note=this node commands sonar transmits but NEVER /cmd_vel/motor/pump; motion is operator teleop.\n"
      << "note=summary/cloud angle axis = raw device angle (angle_base=0); each frame placed at its own reported angle.\n"
      << "note=RX frames are NOT checksum-validated (parity with original mapper); sonar_raw.csv keeps number_of_samples & data_length for offline validity checks.\n";
    f.close();
}

// ======================== 目录创建 ========================
bool ManualPointCollector::ensureDir(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) == 0) return true;
    return mkdir(path.c_str(), 0755) == 0;
}

// ============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "manual_point_collector");
    ManualPointCollector node;
    ros::spin();   // 单线程: odom 订阅 + Trigger 服务; CallbackAsyncSerial 自带 I/O 线程
    return 0;
}
