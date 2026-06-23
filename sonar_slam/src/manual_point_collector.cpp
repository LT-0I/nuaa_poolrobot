/**
 * @file manual_point_collector.cpp
 * @brief 手动点位声呐数据采集节点 (manual_point_collector)
 *
 * 背景: 原版终端4(multi_position_sonar_mapper)自动沿边到 N 个点位停车采集; 自动运动差。
 * 本节点把"运动"交给操作员: 终端1 驱动 + mogo_teleop_keyboard.py 手动开车到点位停下,
 * 然后调用 ROS 服务 /point_collect/record 触发"采这个点位":
 *   - 扫描期间连续记录 odom 流(/wheel_odom);
 *   - 按原版逻辑驱动 Ping360 声呐(逐角发 0x0A29 命令, 解析 2300 帧), 扫【整一圈 360deg=400 角】后即停
 *     (手控由操作员控运动, 不再给声呐计时; 量程 range_level 为唯一常调参数);
 *   - 给声呐方的交付三件套(照原版终端4 格式/命名; 目录并入本次任务夹·会话隔离防覆盖):
 *     ~/sonar_logs/manual_collect/task_<时间戳>/maps/phase_multi_submap_N_posN.{pcd(二进制)+pgm+yaml};
 *   - 我方诊断附加(留 point 夹): cloud.pcd(ASCII 副本) + sonar_raw/summary CSV + odom_stream + meta。
 *
 * 数据层次(全部并入每次任务夹 ~/sonar_logs/manual_collect/task_<时间戳>/, 一次任务=一自洽夹):
 *   交付(声呐方): task_<时间戳>/maps/phase_multi_submap_N_posN.{pcd,pgm,yaml}  (照原版终端4 命名/格式)
 *   附加(我方):   task_<时间戳>/point_NN/{odom_stream.csv, sonar_raw.csv,
 *                 sonar_summary.csv, cloud.pcd(ASCII), meta.txt} + task 级 odom_full.csv / points_index.csv
 *   - task 夹: 每次启动本节点一个; point 夹: 每次 /point_collect/record 一个。
 *   - 交付目录并入任务夹后跨任务不再共享 ~/sonar_logs/maps/(免操作员手动备份, 防 pos 同号互覆)。
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
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>   // tf::getYaw

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "sonar_slam/AsyncSerial.h"

#include <boost/bind.hpp>

#include <fstream>
#include <sstream>
#include <iomanip>
#include <locale>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <memory>
#include <cmath>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <cfloat>
#include <sys/stat.h>

using boost::placeholders::_1;
using boost::placeholders::_2;

namespace {
const float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;
const float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);
const size_t RX_BUFFER_SIZE = 40960;
const uint16_t MAX_RAW_POINTS = 1200;
}

#define COLLECTOR_VERSION "manual-collect-fullodom-assoc-taskmaps-20260622"

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
    pcl::PointCloud<pcl::PointXYZI> buildCloud(const std::vector<RawFrame>& frames, int angle_base);
    void writeAsciiPcd(const std::string& path, const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void writeBinaryPcd(const std::string& path, const pcl::PointCloud<pcl::PointXYZI>& cloud);
    // 给声呐方三件套(逐字移植 main mapper, 保证与终端4 输出同构)
    void createOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    bool saveMapAsPGM(const std::string& filename, const nav_msgs::OccupancyGrid& map);
    bool saveMapAsYAML(const std::string& filename, const std::string& pgm_file, const nav_msgs::OccupancyGrid& map);
    std::string mapsDir();
    void writeMeta(const std::string& dir, const ros::Time& t_start, const ros::Time& t_end,
                   size_t n_frames, bool aborted, size_t angles_with_echo,
                   double px, double py, double pyaw, long seq_a, long seq_b);

    // ---- 全程连续 odom 记录 + 声呐<->odom 关联 ----
    void writeOdomFullMarker(const std::string& event);

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
    double      scan_timeout_sec_;   // 单圈扫描的安全上限(整圈全无回波时防服务卡死); 非计时器, 正常按整圈结束
    double      angle_timeout_sec_;  // 单角等回波上限
    std::string range_level_;
    double      odom_rate_hz_;
    double      min_range_;

    float max_range_;   // 由 range_level_ 推得(samples*25ns*1500/2)

    // 占据栅格(逐字移植 main mapper, 供声呐方三件套 PGM/YAML; 默认对齐终端4)
    nav_msgs::OccupancyGrid occupancy_map_;
    float  map_resolution_;
    int    map_width_;
    int    map_height_;
    float  map_origin_x_;
    float  map_origin_y_;
    float  sound_speed_;

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

    // 全程连续 odom 记录 + 声呐<->odom 关联(仅 main 线程访问: odomCallback/runCollect, 无需锁)
    bool record_full_odom_;            // 开关, 默认 true
    std::ofstream odom_full_ofs_;      // task 夹一份, 连续全程(节点启动起记录)
    long odom_seq_;                    // odom_full 行号(单调), 初始 0
    int  active_point_id_;             // 0=开车途中; N=正扫第N点; 初始 0
    ros::Time last_full_flush_;        // 周期 flush 计时
    std::ofstream points_index_ofs_;   // task 夹一份, 每点一行的清单(声呐<->odom 索引)
    std::string task_stamp_;           // task 时间戳(供 meta task_id)
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
      point_counter_(0),
      record_full_odom_(true),
      odom_seq_(0),
      active_point_id_(0)
{
    current_message_.clear();
    latest_odom_ = OdomSample{0, 0, 0, 0, 0, 0};

    // 参数: 量程 range_level 为唯一常调参数; 每次 record 固定扫整一圈 400 角后停, 不计时。
    pnh_.param("serial_port", serial_port_, std::string("/dev/ttyACM0"));
    pnh_.param("baudrate",    baudrate_,    115200);
    pnh_.param("range_level", range_level_, std::string("s0"));
    pnh_.param("odom_rate_hz", odom_rate_hz_, 20.0);
    pnh_.param("min_range",   min_range_,   0.0);   // 照原版终端4(默认 0.0=不滤近场); 需要滤近场噪声可调高
    // angle_timeout_sec: 单角等回波上限(回波通常几十 ms 即回; 0.5s 余量充足)。
    //   单圈用 0.5s 而非原版 1.5s: 池壁掠射多无回波, 1.5s x 多角会拖长单圈并撞上安全上限被截断。
    pnh_.param("angle_timeout_sec", angle_timeout_sec_, 0.5);
    // scan_timeout_sec: 仅防卡死安全上限。默认 300 > 全无回波整圈 400x(0.5+0.1)=240s, 故正常整圈绝不被截断。
    pnh_.param("scan_timeout_sec", scan_timeout_sec_, 300.0);
    if (odom_rate_hz_ < 1.0) odom_rate_hz_ = 1.0;
    if (angle_timeout_sec_ < 0.05) angle_timeout_sec_ = 0.05;
    if (scan_timeout_sec_ < 5.0) scan_timeout_sec_ = 5.0;
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

    // 占据栅格参数(默认对齐原版终端4: 0.05m/格, 4000x4000=200m 范围, 原点 -100,-100, 声速 1500)。
    pnh_.param("map_resolution", map_resolution_, 0.05f);
    pnh_.param("map_width",      map_width_,      4000);
    pnh_.param("map_height",     map_height_,     4000);
    pnh_.param("map_origin_x",   map_origin_x_,   -100.0f);
    pnh_.param("map_origin_y",   map_origin_y_,   -100.0f);
    pnh_.param("sound_speed",    sound_speed_,    1500.0f);
    occupancy_map_.header.frame_id           = "map";
    occupancy_map_.info.resolution           = map_resolution_;
    occupancy_map_.info.width                = map_width_;
    occupancy_map_.info.height               = map_height_;
    occupancy_map_.info.origin.position.x    = map_origin_x_;
    occupancy_map_.info.origin.position.y    = map_origin_y_;
    occupancy_map_.info.origin.position.z    = 0.0;
    occupancy_map_.info.origin.orientation.w = 1.0;
    occupancy_map_.data.resize(static_cast<size_t>(map_width_) * map_height_, -1);

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

    // 全程连续 odom 记录开关 + 两个 task 级文件(odom_full.csv / points_index.csv)
    pnh_.param("record_full_odom", record_full_odom_, true);
    task_stamp_ = ts;   // ts 是上面 strftime 的时间戳缓冲, 仍在作用域
    if (task_dir_ok_ && record_full_odom_) {
        odom_full_ofs_.open(task_dir_ + "odom_full.csv", std::ios::out | std::ios::trunc);
        if (odom_full_ofs_.is_open()) {
            odom_full_ofs_ << "seq,wall_t,odom_stamp,x,y,yaw_rad,twist_vx,twist_wz,active_point_id,event\n";
        } else {
            ROS_WARN("cannot open odom_full.csv -- continuous odom track will be MISSING");
        }
        points_index_ofs_.open(task_dir_ + "points_index.csv", std::ios::out | std::ios::trunc);
        if (points_index_ofs_.is_open()) {
            points_index_ofs_ << "point_id,deliverable_basename,point_dir,scan_start_wall,scan_end_wall,"
                                 "x_at_scan,y_at_scan,yaw_at_scan,sonar_frames,sweep_complete,"
                                 "angles_with_echo,odom_full_seq_start,odom_full_seq_end\n";
        } else {
            ROS_WARN("cannot open points_index.csv -- sonar<->odom index will be MISSING");
        }
    }

    ROS_INFO("manual_point_collector COLLECTOR_VERSION=%s params:", COLLECTOR_VERSION);
    ROS_INFO("  serial_port=%s baudrate=%d", serial_port_.c_str(), baudrate_);
    ROS_INFO("  mode=single 360deg sweep (400 angles) per record; range_level=%s (max_range=%.2fm)",
             range_level_.c_str(), max_range_);
    ROS_INFO("  angle_timeout_sec=%.2f scan_timeout_sec=%.1f (SAFETY ceiling, NOT a timer)",
             angle_timeout_sec_, scan_timeout_sec_);
    ROS_INFO("  odom_rate=%.1fHz min_range=%.2fm", odom_rate_hz_, min_range_);
    ROS_INFO("  task_dir=%s", task_dir_.c_str());
    ROS_INFO("  full odom track -> %sodom_full.csv ; sonar<->odom index -> %spoints_index.csv (record_full_odom=%d)",
             task_dir_.c_str(), task_dir_.c_str(), (int)record_full_odom_);

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
    if (odom_full_ofs_.is_open())    { odom_full_ofs_.flush();    odom_full_ofs_.close(); }
    if (points_index_ofs_.is_open()) { points_index_ofs_.flush(); points_index_ofs_.close(); }
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

    // 全程连续记录(仅 main 线程): 每条 /wheel_odom 写一行到 task 级 odom_full.csv。
    // 普通行 event 字段留空; 扫描起止边界由 writeOdomFullMarker() 另写带 event 的行。
    if (odom_full_ofs_.is_open()) {
        ros::Time now = ros::Time::now();
        odom_full_ofs_ << (odom_seq_++) << ","
                       << std::fixed << std::setprecision(6)
                       << now.toSec() << ","
                       << latest_odom_.stamp << ","
                       << latest_odom_.x << "," << latest_odom_.y << "," << latest_odom_.yaw << ","
                       << latest_odom_.vx << "," << latest_odom_.wz << ","
                       << active_point_id_ << ",\n";
        if ((now - last_full_flush_).toSec() >= 1.0) {
            odom_full_ofs_.flush();
            last_full_flush_ = now;
        }
    }
}

// ======================== odom_full 边界标记行(扫描起止; 仅 main 线程) ========================
// 写一行(与普通行同 10 列), event 字段=传入字符串(如 POINT_3_SCAN_START); 写完立即 flush。
void ManualPointCollector::writeOdomFullMarker(const std::string& event) {
    if (!odom_full_ofs_.is_open()) return;
    ros::Time now = ros::Time::now();
    odom_full_ofs_ << (odom_seq_++) << ","
                   << std::fixed << std::setprecision(6)
                   << now.toSec() << ","
                   << latest_odom_.stamp << ","
                   << latest_odom_.x << "," << latest_odom_.y << "," << latest_odom_.yaw << ","
                   << latest_odom_.vx << "," << latest_odom_.wz << ","
                   << active_point_id_ << "," << event << "\n";
    odom_full_ofs_.flush();
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
    ROS_INFO("=== collecting point %d -> %s (single 360deg sweep) ===",
             point_counter_, point_dir.c_str());
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

    // 关联: 标活动点 id, 在 odom_full 写起始边界, 快照扫描起始 odom 位姿 + seq 起点。
    active_point_id_ = point_counter_;
    double px = latest_odom_.x, py = latest_odom_.y, pyaw = latest_odom_.yaw;
    long seq_start = odom_seq_;
    writeOdomFullMarker("POINT_" + std::to_string(point_counter_) + "_SCAN_START");

    const double odom_period = 1.0 / odom_rate_hz_;
    ros::Time scan_start = ros::Time::now();
    ros::Time next_odom = scan_start;
    bool aborted = false;

    // 固定扫整一圈: 设备角 0 -> 399 各发一次, 收齐(或单角超时)即转下一角; 走完一圈即停。
    // 不再按 scan_duration 连续多圈累积(那会把点云堆到几十万点并拖崩落盘)。
    for (unsigned short angle = 0; angle < 400 && ros::ok(); ++angle) {
        // 安全上限: 整圈几乎全无回波时防服务卡死(正常一圈远小于此, 非计时器)
        if ((ros::Time::now() - scan_start).toSec() >= scan_timeout_sec_) {
            ROS_WARN("scan_timeout_sec=%.1f reached at angle=%u -- aborting sweep early (sonar mostly silent?)",
                     scan_timeout_sec_, angle);
            aborted = true;
            break;
        }
        ROS_INFO_THROTTLE(5.0, "sweeping... angle=%u/400 (%.0fs elapsed)",
                          angle, (ros::Time::now() - scan_start).toSec());
        // 1. 先清本角接收标志(必须在发命令前: 否则极快回包会被这次清零吞掉->空等)
        {
            std::lock_guard<std::mutex> lk(frame_mutex_);
            angle_received_[angle] = 0;
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
        // 3. 等本角数据返回(<=angle_timeout_sec_), 期间持续 spin + 采 odom
        ros::Time wait_start = ros::Time::now();
        while (!angleReceived(angle) &&
               (ros::Time::now() - wait_start).toSec() < angle_timeout_sec_ &&
               (ros::Time::now() - scan_start).toSec() < scan_timeout_sec_ &&
               ros::ok()) {
            pumpOnce(odom_ofs, next_odom, odom_period);
        }
        // 4. 角间节拍 ~0.1s, 仍持续采 odom
        ros::Time pace_start = ros::Time::now();
        while ((ros::Time::now() - pace_start).toSec() < 0.1 &&
               (ros::Time::now() - scan_start).toSec() < scan_timeout_sec_ &&
               ros::ok()) {
            pumpOnce(odom_ofs, next_odom, odom_period);
        }
    }
    // 节点关停(Ctrl-C, ros::ok()=false)中途退出也算未扫满整圈 -> 标 partial(防把残圈当完整圈)
    if (!ros::ok()) aborted = true;
    if (aborted) ROS_WARN("sweep did not cover all 400 angles (safety timeout or shutdown); cloud/summary PARTIAL");

    collecting_.store(false);
    // 关联: 写结束边界, 记 seq 终点, 复位活动点 id(回到"开车途中"=0)。
    writeOdomFullMarker("POINT_" + std::to_string(point_counter_) + "_SCAN_END");
    long seq_end = odom_seq_;
    active_point_id_ = 0;
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

    // 统计本圈有回波(非空帧)的不同角度数, 连同是否被安全上限截断一起写入 meta 标注有效性。
    std::vector<char> echo_seen(400, 0);
    size_t angles_with_echo = 0;
    for (const RawFrame& rf : snap) {
        if (rf.angle < 400 && !echo_seen[rf.angle]) { echo_seen[rf.angle] = 1; angles_with_echo++; }
    }

    // 构建点云(90 度旋转 + 量程过滤), 供三件套与本地副本共用
    pcl::PointCloud<pcl::PointXYZI> cloud = buildCloud(snap, angle_base);

    // ==== 给声呐方的交付三件套: 照原版终端4 命名/格式 -> <task>/maps/phase_multi_submap_N_posN.{pcd(二进制)+pgm+yaml} ====
    const std::string deliverable_base = "phase_multi_submap_" + std::to_string(point_counter_)
                                       + "_pos" + std::to_string(point_counter_);
    bool deliverable_written = false;
    if (!cloud.points.empty()) {
        const std::string md = mapsDir();
        if (!md.empty()) {
            const std::string& base = deliverable_base;
            writeBinaryPcd(md + base + ".pcd", cloud);        // 二进制(同原版格式, ofstream 直写绕开 PCL mmap)
            createOccupancyGrid(cloud);                        // -> occupancy_map_
            saveMapAsPGM(md + base + ".pgm", occupancy_map_);
            saveMapAsYAML(md + base + ".yaml", base + ".pgm", occupancy_map_);
            ROS_INFO("sonar deliverable triple -> %s{.pcd,.pgm,.yaml}", (md + base).c_str());
            deliverable_written = true;
        } else {
            ROS_WARN("maps dir unavailable -> sonar triple SKIPPED (only local extras saved)");
        }
    }

    // ==== 我方诊断附加(留 point 夹): ASCII 点云副本 + 原始/汇总 CSV + meta ====
    writeAsciiPcd(point_dir + "cloud.pcd", cloud);
    writeRawCsv(point_dir, snap);
    writeSummaryCsv(point_dir, snap, angle_base);
    writeMeta(point_dir, scan_start, t_end, snap.size(), aborted, angles_with_echo,
              px, py, pyaw, seq_start, seq_end);

    // 声呐<->odom 索引: 每点一行追加到 task 级 points_index.csv(列与表头严格对齐)。
    if (points_index_ofs_.is_open()) {
        points_index_ofs_ << point_counter_ << ","
                          << (deliverable_written ? deliverable_base : std::string("(none)")) << ","
                          << point_dir << ","
                          << std::fixed << std::setprecision(6)
                          << scan_start.toSec() << "," << t_end.toSec() << ","
                          << px << "," << py << "," << pyaw << ","
                          << snap.size() << ","
                          << (aborted ? "false" : "true") << ","
                          << angles_with_echo << ","
                          << seq_start << "," << seq_end << "\n";
        points_index_ofs_.flush();
    }

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

// ======================== 由帧构建点云(对齐 mapper convertToPointCloud: 0.9deg/角, dpp*0.73, 90 度旋转) ========================
pcl::PointCloud<pcl::PointXYZI> ManualPointCollector::buildCloud(const std::vector<RawFrame>& frames, int angle_base) {
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
    cloud.width  = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    return cloud;
}

// ======================== 落盘: ASCII PCD(我方副本, CloudCompare 直开 + 校验二进制) ========================
void ManualPointCollector::writeAsciiPcd(const std::string& path, const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.points.empty()) { ROS_WARN("cloud empty -> skip ascii pcd: %s", path.c_str()); return; }
    std::ofstream pf(path, std::ios::out | std::ios::trunc);
    if (!pf.is_open()) { ROS_WARN("cannot open ascii pcd for write: %s", path.c_str()); return; }
    pf.imbue(std::locale::classic());   // force '.' decimal point regardless of any global locale
    const size_t npts = cloud.points.size();
    pf << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n" << "FIELDS x y z intensity\n" << "SIZE 4 4 4 4\n"
       << "TYPE F F F F\n" << "COUNT 1 1 1 1\n"
       << "WIDTH " << npts << "\n" << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << npts << "\n" << "DATA ascii\n";
    pf << std::fixed << std::setprecision(4);
    for (const pcl::PointXYZI& p : cloud.points) {
        pf << p.x << " " << p.y << " " << p.z << " " << p.intensity << "\n";
    }
    pf.flush();
    if (!pf.good()) ROS_WARN("ascii pcd stream error -- may be incomplete: %s", path.c_str());
    pf.close();
}

// ======================== 落盘: 二进制 PCD(同原版 savePCDFileBinary 格式, 但 ofstream 直写绕开崩掉的 PCL mmap) ========================
// 磁盘逐点 16 字节: float32 x,y,z,intensity(小端, ARM); 与 pcl::io::loadPCDFile 对
// FIELDS x y z intensity / SIZE 4 4 4 4 / TYPE F F F F / COUNT 1 1 1 1 / DATA binary 的解析一致。
// 注意: pcl::PointXYZI 内存 sizeof=32(intensity 在偏移16); PCL 二进制写盘同样按 FIELDS 紧凑成 16B/点,
//       故此手写格式与 PCL 产出在磁盘上一致, 标准读取器(PCL/CloudCompare)解析无差异。
void ManualPointCollector::writeBinaryPcd(const std::string& path, const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.points.empty()) { ROS_WARN("cloud empty -> skip binary pcd: %s", path.c_str()); return; }
    std::ofstream pf(path, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!pf.is_open()) { ROS_WARN("cannot open binary pcd for write: %s", path.c_str()); return; }
    pf.imbue(std::locale::classic());
    const size_t npts = cloud.points.size();
    pf << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n" << "FIELDS x y z intensity\n" << "SIZE 4 4 4 4\n"
       << "TYPE F F F F\n" << "COUNT 1 1 1 1\n"
       << "WIDTH " << npts << "\n" << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << npts << "\n" << "DATA binary\n";
    for (const pcl::PointXYZI& p : cloud.points) {
        const float buf[4] = { p.x, p.y, p.z, p.intensity };
        pf.write(reinterpret_cast<const char*>(buf), sizeof(buf));   // 16 bytes/point, little-endian
    }
    pf.flush();
    if (!pf.good()) {
        ROS_WARN("binary pcd stream error -- may be incomplete: %s", path.c_str());
    } else {
        ROS_INFO("binary pcd written: %zu points -> %s", npts, path.c_str());
    }
    pf.close();
}

// ======================== <task>/maps/ (交付三件套目录: 并入本次任务夹, 会话隔离防覆盖) ========================
// 文件名/格式仍照原版终端4(phase_multi_submap_N_posN.{pcd,pgm,yaml}); 仅父目录从共享
// ~/sonar_logs/maps/ 改为每次任务自己的 task_<时间戳>/maps/, 免操作员手动备份、防跨任务 pos 同号互覆。
// yaml 内嵌的 pointcloud: 绝对路径由本函数返回值经 saveMapAsYAML 派生, 自动跟随到本任务 maps/。
std::string ManualPointCollector::mapsDir() {
    if (!task_dir_ok_) { ROS_WARN("task dir not ready -> maps dir unavailable, deliverable skipped"); return std::string(); }
    std::string md = task_dir_ + "maps/";   // task_dir_ 以 '/' 结尾(见构造)
    if (!ensureDir(md)) { ROS_WARN("cannot create maps dir: %s", md.c_str()); return std::string(); }
    return md;
}

// ======================== 占据栅格(逐字移植 main mapper createOccupancyGrid, 入参改为传入点云) ========================
void ManualPointCollector::createOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.points.empty()) return;

    // 1. 动态计算当前点云的实际范围
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    for (const auto& p : cloud.points) {
        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
    }

    // 2. 扩展边界，留出余量
    float margin = 5.0f;
    min_x -= margin; max_x += margin;
    min_y -= margin; max_y += margin;

    // 3. 计算新的地图尺寸
    float new_width  = max_x - min_x;
    float new_height = max_y - min_y;
    int new_map_width  = static_cast<int>(new_width  / map_resolution_) + 100;
    int new_map_height = static_cast<int>(new_height / map_resolution_) + 100;

    // 4. 如果范围变化，重新初始化地图(与原版一致: 4m 单圈点云通常够不到 4000x4000 默认, 该分支不触发)
    bool need_resize = false;
    if (new_map_width > map_width_ || new_map_height > map_height_) need_resize = true;
    if (min_x < map_origin_x_ || min_y < map_origin_y_) need_resize = true;
    if (need_resize) {
        map_width_    = new_map_width;
        map_height_   = new_map_height;
        map_origin_x_ = min_x;
        map_origin_y_ = min_y;
        occupancy_map_.info.width             = map_width_;
        occupancy_map_.info.height            = map_height_;
        occupancy_map_.info.origin.position.x = map_origin_x_;
        occupancy_map_.info.origin.position.y = map_origin_y_;
        occupancy_map_.data.resize(static_cast<size_t>(map_width_) * map_height_, -1);
//        ROS_INFO("occupancy grid resized: %dx%d origin(%.1f,%.1f)",
//                 map_width_, map_height_, map_origin_x_, map_origin_y_);
    }

    // 5. 填充地图(只标占据格: occ=intensity*100/255 取最大; 其余保持 -1=未知, 不做空闲射线标记)
    std::fill(occupancy_map_.data.begin(), occupancy_map_.data.end(), -1);
    for (const auto& point : cloud.points) {
        int map_x = static_cast<int>((point.x - map_origin_x_) / map_resolution_ + 0.5f);
        int map_y = static_cast<int>((point.y - map_origin_y_) / map_resolution_ + 0.5f);
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
            int index = map_y * map_width_ + map_x;
            int occ = static_cast<int>(point.intensity * 100.0f / 255.0f);
            if (occ > occupancy_map_.data[index]) {
                occupancy_map_.data[index] = static_cast<int8_t>(occ);
            }
        }
    }
    occupancy_map_.header.stamp = ros::Time::now();
}

// ======================== 落盘: PGM(逐字移植 main mapper saveMapAsPGM; 日志改 ASCII) ========================
bool ManualPointCollector::saveMapAsPGM(const std::string& filename, const nav_msgs::OccupancyGrid& map) {
    std::ofstream pgm_file(filename);
    if (!pgm_file.is_open()) { ROS_ERROR("cannot open PGM file: %s", filename.c_str()); return false; }
    pgm_file.imbue(std::locale::classic());
    pgm_file << "P2\n";
    pgm_file << "# Sonar360 SLAM Map - " << filename << "\n";
    pgm_file << "# Generated: " << ros::Time::now() << "\n";
    pgm_file << map.info.width << " " << map.info.height << "\n";
    pgm_file << "255\n";
    // 地图数据(行倒序, 因为 PGM 左下角原点)
    for (int y = static_cast<int>(map.info.height) - 1; y >= 0; y--) {
        for (int x = 0; x < static_cast<int>(map.info.width); x++) {
            int value = map.data[y * static_cast<int>(map.info.width) + x];
            int pgm_value;
            if (value < 0)       pgm_value = 127;   // 未知=中性灰
            else if (value == 0) pgm_value = 255;   // 自由=白
            else                 pgm_value = 0;     // 占据=黑
            pgm_file << pgm_value;
            if (x < static_cast<int>(map.info.width) - 1) pgm_file << " ";
        }
        pgm_file << "\n";
    }
    pgm_file.flush();
    bool ok = pgm_file.good();
    pgm_file.close();
    if (!ok) { ROS_WARN("PGM stream error -- file may be incomplete (disk full?): %s", filename.c_str()); return false; }
    return true;
}

// ======================== 落盘: YAML(逐字移植 main mapper saveMapAsYAML; 日志改 ASCII) ========================
bool ManualPointCollector::saveMapAsYAML(const std::string& filename,
                                         const std::string& pgm_file,
                                         const nav_msgs::OccupancyGrid& map) {
    std::ofstream yaml_file(filename);
    if (!yaml_file.is_open()) { ROS_ERROR("cannot open YAML file: %s", filename.c_str()); return false; }
    yaml_file.imbue(std::locale::classic());   // force '.' decimals (resolution/origin/max_range) regardless of global locale

    double yaw = tf::getYaw(map.info.origin.orientation);
    int occupied = 0, free = 0, unknown = 0;
    for (int val : map.data) {
        if (val < 0) unknown++;
        else if (val == 0) free++;
        else if (val > 50) occupied++;
    }

    yaml_file << "# Sonar360 SLAM Map Configuration\n";
    yaml_file << "# Generated at: " << ros::Time::now() << "\n";
    yaml_file << "# ROS Time: " << ros::Time::now().toSec() << "\n\n";
    yaml_file << "image: " << pgm_file << "\n";
    yaml_file << "resolution: " << map.info.resolution << "\n";
    yaml_file << "origin: [" << map.info.origin.position.x << ", "
              << map.info.origin.position.y << ", " << yaw << "]\n";
    yaml_file << "negate: 0\n";
    yaml_file << "occupied_thresh: 0.65\n";
    yaml_file << "free_thresh: 0.196\n\n";
    yaml_file << "# ========== Sonar360 Metadata ==========\n";
    yaml_file << "sensor_type: \"mechanical_scanning_single_beam\"\n";
    yaml_file << "scan_resolution_deg: 1.0\n";
    yaml_file << "max_range: " << max_range_ << "\n";
    yaml_file << "min_range: " << min_range_ << "\n";
    yaml_file << "sound_speed: " << sound_speed_ << "\n";
    yaml_file << "frame_id: \"" << map.header.frame_id << "\"\n\n";
    std::string pcd_filename = filename.substr(0, filename.find_last_of('.')) + ".pcd";
    yaml_file << "# Point cloud reference (for ICP/NDT localization)\n";
    yaml_file << "pointcloud: \"" << pcd_filename << "\"\n\n";
    yaml_file << "# Metadata\n";
    yaml_file << "metadata:\n";
    yaml_file << "  width: " << map.info.width << "\n";
    yaml_file << "  height: " << map.info.height << "\n";
    yaml_file << "  timestamp: " << ros::Time::now().toSec() << "\n";
    yaml_file << "  frame_id: \"" << map.header.frame_id << "\"\n\n";
    yaml_file << "# Statistics\n";
    yaml_file << "statistics:\n";
    yaml_file << "  occupied_cells: " << occupied << "\n";
    yaml_file << "  free_cells: " << free << "\n";
    yaml_file << "  unknown_cells: " << unknown << "\n";
    yaml_file << "  total_cells: " << map.data.size() << "\n";
    yaml_file << "  occupied_percentage: " << (occupied * 100.0 / map.data.size()) << "%\n";
    yaml_file << "  free_percentage: " << (free * 100.0 / map.data.size()) << "%\n";
    yaml_file << "  unknown_percentage: " << (unknown * 100.0 / map.data.size()) << "%\n";
    yaml_file.flush();
    bool ok = yaml_file.good();
    yaml_file.close();
    if (!ok) { ROS_WARN("YAML stream error -- file may be incomplete: %s", filename.c_str()); return false; }
    return true;
}

// ======================== 落盘: meta ========================
void ManualPointCollector::writeMeta(const std::string& dir, const ros::Time& t_start,
                                     const ros::Time& t_end, size_t n_frames,
                                     bool aborted, size_t angles_with_echo,
                                     double px, double py, double pyaw,
                                     long seq_a, long seq_b) {
    std::ofstream f(dir + "meta.txt", std::ios::out | std::ios::trunc);
    if (!f.is_open()) { ROS_WARN("cannot open meta.txt"); return; }
    std::time_t st = static_cast<std::time_t>(t_start.toSec());
    char sbuf[32]; std::strftime(sbuf, sizeof(sbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&st));
    f << "range_level=" << range_level_ << "\n"
      << "max_range_m=" << std::fixed << std::setprecision(3) << max_range_ << "\n"
      << "min_range_m=" << min_range_ << "\n"
      << "scan_mode=single_360_sweep_400angles\n"
      << "sweep_complete=" << (aborted ? "false" : "true")
      << " (false = did not cover all 400 angles: safety timeout or shutdown; cloud/summary PARTIAL)\n"
      << "angles_with_echo=" << angles_with_echo << " of 400 (distinct angles with a non-empty return, not frame count)\n"
      << "angle_timeout_sec=" << angle_timeout_sec_ << "\n"
      << "scan_timeout_sec=" << scan_timeout_sec_ << " (safety ceiling only, not a timer)\n"
      << "deliverable=../maps/phase_multi_submap_" << point_counter_ << "_pos" << point_counter_
      << ".{pcd(binary)+pgm+yaml} (in this task's maps/; faithful to terminal-4 format/naming)\n"
      << "local_extra=cloud.pcd in this dir is an ASCII copy (CloudCompare-openable; verifies the binary)\n"
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

    // ---- sonar<->odom association block ----
    f << "task_id=" << task_stamp_ << "\n"
      << "point_id=" << point_counter_ << "\n"
      << "scan_start_wall_unix=" << std::fixed << std::setprecision(6) << t_start.toSec() << "\n"
      << "scan_end_wall_unix=" << t_end.toSec() << "\n"
      << "odom_pose_at_scan=[" << px << ", " << py << ", " << pyaw
      << "]  (x_m,y_m,yaw_rad from /wheel_odom at scan start)\n";
    if (record_full_odom_) {
        f << "odom_full=../odom_full.csv  (rows with active_point_id==point_id are this scan; "
             "plus POINT_" << point_counter_ << "_SCAN_START/END markers)\n"
          << "odom_full_seq_range=[" << seq_a << ", " << seq_b << "]\n";
    } else {
        f << "odom_full=disabled\n";
    }
    f << "assoc_note=join sonar->odom by wall-clock time (sonar_raw.csv wall_t vs scan_start/end_wall_unix) "
         "OR by point_id via ../points_index.csv\n";
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
