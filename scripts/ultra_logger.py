#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ultra_logger.py  —  4 路超声波 (/ultra_data) 实时落盘记录器（旁路 sidecar 节点）
#
# 用途：绕泳池贴边行走（终端4 multi_position_sonar_mapper + 终端2 move_controller）期间，
#       把驱动发布的 4 路超声波距离 /ultra_data 连同里程计 /wheel_odom、mapper 相位状态，
#       实时写成一份带时间戳的 CSV，供离线分析（沿边距墙、走/扫相位下的距离曲线）。
#
# 设计要点（为什么是独立节点，不改 C++ mapper）：
#   - 只“订阅+写文件”，绝不发 /cmd_vel、绝不命令电机 → 对正在工作的走边逻辑零影响、零风险；
#   - 纯 Python rospy，免编译，第5个终端随跑随停，随时可改字段；
#   - 只依赖 ROS 核心消息 (std_msgs/nav_msgs)，不依赖 mars_msgs，不需要重编译工作空间。
#
# 终端打印一律 ASCII（机器人终端无中文字库，中文会显示成 ???）；本注释不打印、可中文。
# 注意：CSV 文件里会写入 mapper 的相位串，这些串可能是中文——文件用 UTF-8 写盘，
#       中文进文件没问题（不经过终端字库），离线分析照常可读。
#
# /ultra_data 是 std_msgs/Int32MultiArray，data 含 4 个 int（单位 mm）。
# 通道含义（镜像 move_controller::ultraCallback 这个“消费者”的认知）：
#   data[0] = 前左 front-left   (fl)
#   data[1] = 前右 front-right  (fr)
#   data[2] = 右前 right-front  (rf)
#   data[3] = 右后 right-rear   (rr)   —— 右前/右后这一对用于右侧沿墙保持平行与距离。
# ⚠ 标签存在源码冲突：hardware_driver 的发送侧注释与 move_controller 的语义注释对 fl/fr 与
#   右侧前后这两对的对应**互相矛盾**（评审 D-? 复核发现）。本记录器列名沿用 move_controller（实际跑
#   沿边的那方）的解读；**列顺序严格等于 data[0..3]**，故即便物理对应有误，离线只需重命名列即可，
#   不丢数据。建议现场用记录器跑起来时做一次 30s 标定：按 fl→fr→rf→rr 顺序逐个在传感器前 ~30cm
#   伸手遮挡并记时，离线看哪列下跌即可永久定标。
# ⚠ 分析注意（驱动既有行为，非本记录器引入）：
#   - 驱动 UpdateUltra 把故障读数(65533/超量程)改写成整数 2000mm 再发布 → CSV 里**连续恰好 2000**
#     的段应当作故障哨兵/缺失数据，别当真实 2.0m 几何（尤其影响 3m 沿壁分析）。
#   - odom 的 twist(vx/wz) 可能滞后于 pose（驱动 D-29 已知缺陷：无新编码器分支时只刷 header.stamp
#     不刷 twist）→ odom_age_ms 看着新鲜，vx/wz 仍可能是旧值，勿用这两列做细时标速率分析。
#   - 首行 CSV 可能是订阅瞬间收到的 latched 旧 /ultra_data 帧、却盖了当前时间戳 → 分析丢弃/标记首样本。

import os
import csv
import math
import time
import threading

import rospy
from std_msgs.msg import Int32MultiArray, String
from nav_msgs.msg import Odometry

CODE_VERSION = "ultra-logger-v3-20260611"   # 版本闸门：grep 自判，改字段/逻辑必升版本号

CSV_HEADER = [
    "seq",            # 行号（本节点为第 N 个 /ultra_data 样本分配的序号；写盘失败会留空缺）
    "t_unix",         # 收到该 /ultra_data 的时间戳（秒，rospy 时钟，全精度）
    "t_iso",          # 同一时刻的本地可读时间（系统墙钟）
    "fl_mm", "fr_mm", "rf_mm", "rr_mm",   # 4 路原始距离（mm，直接来自驱动）
    "fl_m",  "fr_m",  "rf_m",  "rr_m",    # 同上换算成米（mm/1000，便于直读）
    "odom_x", "odom_y",      # 里程计位置（m，裸陀螺积分坐标系，可能漂移）
    "odom_yaw_deg",          # 里程计航向（deg，由 /wheel_odom 四元数还原）
    "odom_vx",               # 前向线速度（m/s，twist.linear.x）⚠ 可能滞后(驱动 D-29)，勿做细时标速率分析
    "odom_wz_dps",           # 角速率（deg/s，twist.angular.z 由 rad/s 换算）⚠ 同上 twist 滞后
    "odom_age_ms",           # 里程计新鲜度：距最近一帧 odom 的毫秒数（空=从未收到）
    "phase1",                # /phase1/status 最近值（mapper 相位串，可能中文）
    "phase_multi",           # /phase_multi/status 最近值（独立成列，避免两话题混淆）
]


def yaw_from_quaternion(q):
    """从四元数还原 yaw（rad）。通用公式，不依赖 tf。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def open_unique_csv(path):
    """独占创建 CSV，绝不截断已存在的文件（防重跑覆盖丢数据）。
    若 path 已存在，自动追加 _1/_2/... 后缀直到拿到一个新文件名。
    返回 (file_handle, actual_path)。UTF-8 写盘 + errors='replace' 兜底，
    保证任何相位串（含中文/异常字节）都不会让某一行写失败。"""
    base, ext = os.path.splitext(path)
    for i in range(0, 1000):
        cand = path if i == 0 else "%s_%d%s" % (base, i, ext)
        try:
            fh = open(cand, "x", newline="", encoding="utf-8", errors="replace")
            return fh, cand
        except FileExistsError:
            continue
    # 极端兜底（前 1000 个序号都被占用，几乎不可能）：pid + 递增计数循环独占创建，绝不抛出。
    k = 0
    while True:
        cand = "%s_pid%d_%d%s" % (base, os.getpid(), k, ext)
        try:
            return open(cand, "x", newline="", encoding="utf-8", errors="replace"), cand
        except FileExistsError:
            k += 1


class UltraLogger(object):
    def __init__(self):
        # ---- 输出文件路径：优先 ~out_file（完整路径），否则 ~out_dir + 带时间戳文件名 ----
        ts_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        default_dir = os.path.expanduser("~")
        out_file = rospy.get_param("~out_file", "")
        if out_file:
            req_path = os.path.expanduser(out_file)
        else:
            out_dir = os.path.expanduser(rospy.get_param("~out_dir", default_dir))
            req_path = os.path.join(out_dir, "ultra_log_%s.csv" % ts_name)

        out_parent = os.path.dirname(req_path) or "."
        try:
            os.makedirs(out_parent, exist_ok=True)
        except Exception as e:
            rospy.logerr("[ultra_logger] cannot create out dir %s: %s", out_parent, e)
            raise

        # ---- 共享状态（被多个订阅回调线程访问，加锁保护）----
        self.lock = threading.Lock()
        self.seq = 0
        self.rows_ok = 0             # 成功写盘的行数（健康指标，与 seq 区分）
        self.rows_err = 0            # 写盘失败的行数
        self.last_odom = None        # (x, y, yaw_deg, vx, wz_dps, stamp_sec)
        self.phase1 = ""             # 最近一次 /phase1/status
        self.phase_multi = ""        # 最近一次 /phase_multi/status
        self.last_fl = self.last_fr = self.last_rf = self.last_rr = None
        self.last_fsync = time.time()
        self.start_wall = time.time()

        # ---- 独占打开文件并写表头（UTF-8；每行 flush + 周期 fsync 抗断电丢数据）----
        self.fh, self.path = open_unique_csv(req_path)
        self.writer = csv.writer(self.fh)
        self.writer.writerow(CSV_HEADER)
        self.fh.flush()

        # ---- 订阅：/ultra_data 是主触发（每来一帧写一行）；其余只缓存最近值 ----
        # queue_size 给大一点，避免落盘偶发卡顿时丢消息。
        self.sub_ultra = rospy.Subscriber("/ultra_data", Int32MultiArray,
                                          self.cb_ultra, queue_size=200)
        self.sub_odom = rospy.Subscriber("/wheel_odom", Odometry,
                                         self.cb_odom, queue_size=50)
        # mapper 两个相位状态话题，各自独立成列（不再 last-write-wins 混淆来源）。
        self.sub_ph1 = rospy.Subscriber("/phase1/status", String,
                                        self.cb_phase1, queue_size=10)
        self.sub_phm = rospy.Subscriber("/phase_multi/status", String,
                                        self.cb_phase_multi, queue_size=10)

        # ---- 周期状态打印（ASCII），5s 一次，便于现场确认在记 ----
        self.status_timer = rospy.Timer(rospy.Duration(5.0), self.cb_status)
        rospy.on_shutdown(self.on_shutdown)

        rospy.logwarn("[ultra_logger] CODE_VERSION=%s", CODE_VERSION)
        rospy.logwarn("[ultra_logger] writing CSV -> %s", self.path)
        rospy.logwarn("[ultra_logger] cols: seq,t_unix,t_iso,"
                      "fl/fr/rf/rr(mm+m),odom_x/y/yaw/vx/wz,odom_age_ms,phase1,phase_multi")
        rospy.logwarn("[ultra_logger] ch map: fl=front-left fr=front-right "
                      "rf=right-front rr=right-rear (mm); right wall = rf,rr")
        rospy.logwarn("[ultra_logger] subscribing: /ultra_data /wheel_odom "
                      "/phase1/status /phase_multi/status. Ctrl-C to stop.")

    # ----------------- 订阅回调 -----------------
    def cb_odom(self, msg):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        with self.lock:
            self.last_odom = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                math.degrees(yaw),
                msg.twist.twist.linear.x,
                math.degrees(msg.twist.twist.angular.z),
                msg.header.stamp.to_sec(),
            )

    def cb_phase1(self, msg):
        with self.lock:
            self.phase1 = msg.data

    def cb_phase_multi(self, msg):
        with self.lock:
            self.phase_multi = msg.data

    def cb_ultra(self, msg):
        now = rospy.Time.now().to_sec()
        d = list(msg.data)
        if len(d) < 4:
            rospy.logwarn_throttle(
                5.0, "[ultra_logger] /ultra_data len=%d (<4), padding", len(d))
            d = d + [None] * (4 - len(d))
        fl, fr, rf, rr = d[0], d[1], d[2], d[3]

        def mm2m(v):
            return "" if v is None else "%.3f" % (v / 1000.0)

        with self.lock:
            self.seq += 1
            seq = self.seq
            self.last_fl, self.last_fr, self.last_rf, self.last_rr = fl, fr, rf, rr

            if self.last_odom is not None:
                ox, oy, oyaw, ovx, owz, ostamp = self.last_odom
                age_ms = "%.0f" % max(0.0, (now - ostamp) * 1000.0)
                ox_s, oy_s = "%.4f" % ox, "%.4f" % oy
                oyaw_s, ovx_s, owz_s = "%.3f" % oyaw, "%.4f" % ovx, "%.3f" % owz
            else:
                ox_s = oy_s = oyaw_s = ovx_s = owz_s = age_ms = ""

            row = [
                seq,
                "%.6f" % now,
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now)) +
                (".%03d" % int((now % 1.0) * 1000)),
                "" if fl is None else fl,
                "" if fr is None else fr,
                "" if rf is None else rf,
                "" if rr is None else rr,
                mm2m(fl), mm2m(fr), mm2m(rf), mm2m(rr),
                ox_s, oy_s, oyaw_s, ovx_s, owz_s, age_ms,
                self.phase1, self.phase_multi,
            ]
            try:
                self.writer.writerow(row)
                self.fh.flush()
                self.rows_ok += 1
            except Exception as e:
                self.rows_err += 1
                rospy.logerr_throttle(2.0, "[ultra_logger] write error: %s", e)
            # 周期性 fsync 强制落盘（SD 卡上每行 fsync 太慢，1s 一次足够抗断电）。
            # 独立 try：fsync 失败不计成写错(行已 flush 进 OS 缓冲)，且无论成败都推进计时防逐行重试。
            if (time.time() - self.last_fsync) >= 1.0:
                try:
                    os.fsync(self.fh.fileno())
                except Exception as e:
                    rospy.logwarn_throttle(5.0, "[ultra_logger] fsync warn: %s", e)
                self.last_fsync = time.time()

    # ----------------- 周期状态打印 -----------------
    def cb_status(self, _evt):
        with self.lock:
            ok, err = self.rows_ok, self.rows_err
            fl, fr, rf, rr = self.last_fl, self.last_fr, self.last_rf, self.last_rr
            have_odom = self.last_odom is not None
        elapsed = max(1e-3, time.time() - self.start_wall)
        rate = ok / elapsed
        rospy.logwarn(
            "[ultra_logger] rows_ok=%d rows_err=%d rate=%.1f Hz last(mm): "
            "fl=%s fr=%s rf=%s rr=%s odom=%s",
            ok, err, rate, str(fl), str(fr), str(rf), str(rr),
            "yes" if have_odom else "NONE")

    # ----------------- 退出清理 -----------------
    def on_shutdown(self):
        with self.lock:
            try:
                self.fh.flush()
                os.fsync(self.fh.fileno())
            except Exception:
                pass
            try:
                self.fh.close()
            except Exception:
                pass
            ok, err = self.rows_ok, self.rows_err
        rospy.logwarn("[ultra_logger] stopped. rows_ok=%d rows_err=%d -> %s",
                      ok, err, self.path)


if __name__ == "__main__":
    # disable_signals=False（默认）让 rospy 接管 Ctrl-C 并触发 on_shutdown 优雅关文件。
    rospy.init_node("ultra_logger")
    try:
        UltraLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
