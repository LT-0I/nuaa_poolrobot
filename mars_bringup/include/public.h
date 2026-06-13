#ifndef __PUBLIC_H__
#define __PUBLIC_H__

#include <sys/ipc.h>
#include <sys/shm.h>
#include <vector>
#include <sys/select.h>

using namespace std;
#define MAXLINE 4096
#define PATH_NAME "./"
#define PROJ_ID 0X6666

#define PI_M 3.1415926535897932384626433832795029
// ------------------------- device heart beats ------------------------
// wheel
#define DEV_WHEEL_COM_ERROR 0X01  // 0000 0001
#define DEV_WHEEL_COM_NORMAL 0XFE // 1111 1110

// obstacle
#define DEV_OBSTACLE_COM_ERROR 0X02  // 0000 0010
#define DEV_OBSTACLE_COM_NORMAL 0XFD // 1111 1101

// wifi
#define DEV_WIFI_COM_ERROR 0X04  // 0000 0100
#define DEV_WIFI_COM_NORMAL 0XFB // 1111 1011

// jacking
#define DEV_JACKING_COM_ERROR 0X08  // 0000 1000
#define DEV_JACKING_COM_NORMAL 0XF7 // 1111 0111

// --------------------------- device on off ---------------------------
// wheel
#define DEV_DISABLE_WHEEL_CMD 0
#define DEV_ENABLE_WHEEL_CMD 1
#define DEV_WHEEL_DISABLE_STA 0
#define DEV_WHEEL_ENABLE_STA 1

// obstacle
#define DEV_DISABLE_OBSTACLE_CMD 0
#define DEV_ENABLE_OBSTACLE_CMD 1
#define DEV_OBSTACLE_DISABLE_STA 0
#define DEV_OBSTACLE_ENABLE_STA 1

// BaterryOnOffState
#define BATERRY_OFF_STA 0
#define BATERRY_ON_STA 1

// --------------------------- module on off ---------------------------
// mapping
#define MDU_EMPTY_MAP_CMD 0
#define MDU_DRAW_MAP_CMD 1
#define MDU_STOP_MAP_CMD 2
#define MDU_CANCEL_MAP_CMD 3

#define MDU_MAP_EMPTY_STA 0
#define MDU_MAP_DRAWING_STA 1
#define MDU_MAP_SAVING_STA 2

// location
#define MDU_STOP_LOCATION_CMD 0
#define MDU_START_LOCATION_CMD 1

#define MDU_LOCATION_STOP_STA 0
#define MDU_LOCATION_RUNNING_STA 1

// -------------------------- mars os command --------------------------
// mars_os
#define MARS_SHUTDOWN_OS_CMD 0
#define MARS_BOOT_OS_CMD 1
#define MARS_REBOOT_OS_CMD 2

// mars_ctlmode
#define MARS_GET_SYSTEM_CMD 0
#define MARS_GET_MANUAL_CMD 1
#define MARS_SYSTEM_MODE_STA 0
#define MARS_MANUAL_MODE_STA 1

// mars_log
#define MARS_DEFAUL_LOG_CMD 0
#define MARS_UPLOAD_LOG_CMD 1
#define MARS_CLEAR_LOG_CMD 2
#define MARS_LOG_DEFAUL_STA 0
#define MARS_LOG_UPLOADING_STA 1
#define MARS_LOG_CLEARING_STA 2

// mars_map
#define MARS_DEFAUL_MAP_CMD 0
#define MARS_UPLOAD_MAP_CMD 1
#define MARS_SYNC_MAP_CMD 2
#define MARS_STOP_UPLOAD_CMD 3
#define MARS_MAP_DEFAUL_STA 0
#define MARS_MAP_UPLOAD_STA 1
#define MARS_MAP_SYNC_STA 2

// control mode
#define MARS_REST_MARSOS_CMD 1

// emergency
#define MARS_CANCLE_EMERGENCY_CMD 0
#define MARS_TRIGGER_EMERGENCY_CMD 1
#define MARS_EMERGENCY_CANCLE_STA 0
#define MARS_EMERGENCY_TRIGER_STA 1

// save_map
#define MARS_MAP_SAVED 0
#define MARS_MAP_SAVEING 1

// task_state
#define STATE_NO_TASK 0
#define STATE_ARRIVED_DELIVERY_POINT_STATION 11
#define STATE_ERROR_TASK 12

// charge_state
#define NO_CHARGE 0
#define GO_CHARGE_STATION 1
#define ARRIVED_CHARGE_STATION 2
#define CHARGEING 3
#define DO_PATROL 4
#define GO_HOME 5
#define ARRIVED_HOME_STATION 6

typedef signed char int8_t;
typedef short int int16_t;
typedef int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

// #pragma pack(1)

typedef struct _Point
{

    double x;
    double y;
    _Point() : x(0), y(0) {}
    _Point(double _x, double _y) : x(_x), y(_y) {}
    void operator()(const double ax, const double ay)
    {
        x = ax;
        y = ay;
    }
    void operator=(const _Point b)
    {
        x = b.x;
        y = b.y;
    }
} Point;

enum Direction : uint8_t
{
    AUTO_DIR = 0,
    FORWARD = 1,
    BACKWARD = 2,
};

// 节点
typedef struct _NODE
{
    int id;
    Point point;
    _NODE() : id(), point() {}
    _NODE(int _id) : id(_id), point() {}
    _NODE(Point _point) : id(), point(_point) {}
    _NODE(int _id, Point _point) : id(_id), point(_point) {}
} NODE;

// 边线
typedef struct _EDGE
{
    enum PathType : uint8_t
    {
        PATH_ZERO = 0,
        PATH_LINE = 1,
        PATH_CIRCLE = 2,
        PATH_BEZIER = 3,
        PATH_ROTATE = 4,
    };
    PathType type; // 1:默认直线, 2:贝塞尔, 3:弧线

    int id;
    double cost; // 搜索时的代价参数 = length()

    NODE ps; // 起始点
    NODE pe; // 终端点
    NODE pc; // 控制点C
    NODE pd; // 控制点D
    float radius;

    int nodeStartType;
    int nodeEndType;

    _EDGE() : id(), cost(), ps(), pe(), pc(), pd() {}
    _EDGE(PathType _type, int _id, double _cost, NODE _ps, NODE _pe) : type(_type), id(_id), cost(_cost), ps(_ps), pe(_pe) {}
    _EDGE(int _id, NODE _ps, NODE _pe, NODE _pc, NODE _pd) : id(_id), ps(_ps), pe(_pe), pc(_pc), pd(_pd) {}
} EDGE;

typedef struct _PATH
{
    enum PathType : uint8_t
    {
        PATH_ZERO = 0,
        PATH_LINE = 1,
        PATH_CIRCLE = 2,
        PATH_BEZIER = 3,
        PATH_ROTATE = 4,
    };
    PathType type;

    Point ps;
    Point pe;
    Point pc;
    Point pd;

    Direction dir;
} PATH;

typedef struct
{
    double limit_speed;
    double limit_omega;
    double max_acce;
    double max_alpha;
} MotionParams;
typedef struct
{
    double wheel_spacing;  // 轮间距
    double wheel_diameter; // 轮子直径
    int wheel_reduction;   // 减速比
    int wheel_resolution;  // 编码器分辨率
} MachineParams;

typedef struct
{
    double Kp;
    double Ki;
    double Kd;
} PID;
typedef struct
{
    PID x;
    PID y;
    PID theta;
} ControlParams;
typedef struct LidarParams
{
    double scan_cull_n;
    vector<double> scan_cull_r1;
    vector<double> scan_cull_r2;
    vector<double> scan_range;       // ¼¤¹âÉ¨Ãè½Ç¶È
    vector<double> laser_vector3;    // ¼¤¹âÏà¶ÔÓÚÐ¡³µÖÐÐÄµãµÄÎ»ÖÃ
    vector<double> laser_quaternion; // ¼¤¹âËÄÔªËØ
    vector<double> obstacle_rect;    // ±ÜÕÏ·¶Î§
} LidarParams;
typedef struct AresParams
{
    MotionParams motion_params;
    MachineParams machine_params;
    ControlParams control_params;
    LidarParams lidar_params;
} AresParams;

typedef enum
{
    CM_CMD_STA_TYPE = 0,
    CM_DEV_CTL_TYPE = 1,
    CM_DEV_ONOFF_TYPE = 2,
} MSG_Type;

typedef enum
{
    CM_MOTOR_CTL_ID = 0,
    CM_BATTERY_DATA_ID = 1,
    CM_IMU_DATA_ID = 2,
    CM_SENSOR_DATA_ID = 3,
    CM_ULTRA_DATA_ID = 4,
} MSG_ID;

typedef enum
{
    RARID_MASTER_STATES = 0,
    RARID_DEVICE_STATES = 1, // 10  Hz
    RARID_OBSTCAL_DATA = 2,  // 50  Hz
    RARID_BATTERY_DATAS = 3,
    RARID_GOODS_DATA = 4, // 10  Hz
    ARID_MOTOR_DATA = 5,
} RAR_ID;

typedef enum
{
    RCRID_SET_MODE = 0,

    RCRID_MOTOR_COMMUN = 1,
    RCRID_ACTOR_COMMUN = 2,

    RCRID_ENABLE_MOTOR = 3,
    RCRID_ONFF_OBSTACLE = 4,
    RCRID_ONFF_BATERRY = 5,

    RCRID_ONFF_SINGNAL = 6,

    RCRID_ONFF_FILLLIGHT = 7,
    RCRID_ONFF_VISION = 8,

    RCRID_ONFF_GOODS = 9,

    RCRID_JACK_COMMUN = 10,
    RCRID_JACKRESET_COMMUN = 11,
} RCR_ID;

// typedef enum
// {
// MDU_DRAW_MAP_CMD = 0,
// MDU_STOP_MAP_CMD = 1,
// MDU_CANCEL_MAP_CMD = 2,
// } MapDrawCommand;

typedef struct
{
    float speed;
    float omega;
} WheelCommand;

typedef struct
{
    float speed;
    float omega;
} ActorCommand;

enum AvoidCmd : uint8_t
{
    NO_OBSTACLE = 0,
    SLOW_OBSTACLE = 1,
    MATCH_STOP = 2,
    SCAN_STOP = 3,
};
typedef struct
{
    int avoid_level;
    uint8_t aviod_scan_cmd;
    uint8_t aviod_match_cmd;
} AvoidCommand;

typedef struct
{
    int volume;
} AudioCommand;

typedef struct
{
    int brightness;
} SignalCommand;

typedef struct
{
    int brightness;
} LightingCommand;

typedef struct
{

} MotionCommand;

typedef struct
{

} LocationCommand;

typedef enum
{

} MapMergeCommand;
typedef enum
{

} MapSwitchCommand;
typedef struct
{
    // MapDrawCommand      map_draw_cmd;
    MapMergeCommand map_merge_cmd;
    MapSwitchCommand map_switch_cmd;
} MappingCommand;

typedef struct
{

} QrcodeCommand;

typedef struct
{

} AsrCommand;

enum MissionCommand : uint8_t
{
    CMD_NO_TASK = 0,
    CMD_PATROL = 1,
    CMD_FOLLOW = 2,
    CMD_CHARGE = 3,
};

#pragma pack(1)
typedef struct
{
    int task_id;
    MissionCommand mission;
    Point task_point;
} TaskCommand;
#pragma pack(0)

typedef struct
{
    // 外接　设备　开关
    uint8_t wheel_onff_cmd;
    uint8_t actor_onff_cmd;
    uint8_t avoid_onff_cmd;
    uint8_t charge_onff_cmd;
    uint8_t camera_onff_cmd;
    uint8_t audio_onff_cmd;
    uint8_t signal_onff_cmd;
    uint8_t lighting_onff_cmd;

    // 功能　模块　开关
    uint8_t motion_onff_cmd;
    uint8_t location_onff_cmd;
    uint8_t mapping_onff_cmd;
    uint8_t qrcode_onff_cmd;
    uint8_t asr_onff_cmd;

    // 外接　设备　行为
    WheelCommand wheel_action_cmd;
    ActorCommand actor_action_cmd;
    AvoidCommand avoid_action_cmd;
    AudioCommand audio_action_cmd;
    SignalCommand signal_action_cmd;
    LightingCommand lighting_action_cmd;

    // 功能　模块　行为
    MotionCommand motion_action_cmd;
    LocationCommand location_action_cmd;
    MappingCommand mapping_action_cmd;
    QrcodeCommand qrcode_action_cmd;
    AsrCommand asr_action_cmd;

    // 参数同步信号
    uint8_t motion_signal;
    uint8_t machine_signal;
    uint8_t control_signal;
    uint8_t lidar_signal;

    // mars 系统
    uint8_t mars_os_cmd;
    uint8_t mars_ctlmode_cmd;
    uint8_t mars_log_cmd;
    uint8_t mars_map_cmd;

    bool map_flag;
    bool connect_flag;

    TaskCommand task_command;

} AresCmd;

#endif