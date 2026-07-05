#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_teleop_keyboard.py —— 键盘遥控：w/x/a/d 控前后左右 + t/g/y/h 单独微调左右轮
                            （基于 mogo_teleop_keyboard.py 改）

目的：主控用 w/x/a/d 控整车 前进/后退/左转/右转（差速）；t/g/y/h 保留"左轮/右轮
      独立微调"（老坦克式细调），供需要单独给某侧轮设定值时用（如手动补偿走偏）。

原理：机上驱动 mars_serial_server (hardware_driver.cpp::sendKeyboardVelCommand)
      订阅 /cmd_vel，再按差速公式反算出左右轮：
          right_speed = linear.x + (angular.z * OMEGA_GAIN) * wheel_spacing/2
          left_speed  = linear.x - (angular.z * OMEGA_GAIN) * wheel_spacing/2
          电机右 = right_speed * 230 ; 电机左 = -left_speed * 230
      本脚本让操作员直接给 (左轮 L, 右轮 R) 设定值，再【精确反解】出对应的
      linear.x / angular.z 发到 /cmd_vel，使驱动重建出 left_speed==L、right_speed==R：
          linear.x  = (L + R) / 2
          angular.z = (R - L) / (wheel_spacing * OMEGA_GAIN)
      代入回上面两式可验证：right_speed==R、left_speed==L（恒等成立）。
      因此【无需改动任何 C++】，纯发 /cmd_vel 即可独立控两轮。

与驱动一致的常量（默认取 main 分支 hardware_driver.cpp 的默认值）：
      OMEGA_GAIN    = 6.4   (sendKeyboardVelCommand 内硬编码)
      WHEEL_SPACING = 0.2   (ROS 参数 wheel_spacing 的默认值)
  ⚠ 若启动驱动时改了 wheel_spacing，请用本节点私有参数 _wheel_spacing / _omega_gain
    同步覆盖，否则反解出现交叉耦合（两轮设定值不再精确独立）。

安全：
  - L、R 各自限幅 ±WHEEL_MAX（默认 0.30，沿用原脚本验证过的范围）。
  - 's' 或 空格 立即清零两轮 + 水泵；退出(Ctrl-C/异常)连发多帧零并保 queue 排空。
  - 设定值持续保持（油门式），并以 ~8Hz 持续发布以喂看门狗（若驱动启用）。
  - 终端打印一律 ASCII（机上无中文字库）。
  - 水泵=吸底清洁泵、无推力；默认 0，只 '[' / ']' 调；运动只来自电机。
  - 唯一可靠停车=断电；本脚本"发零"仅停电机命令、不等于真停。
  - 'c' 一键扫描（方案 c，纯 python）：停车 + 泵置 0 + 静置 ~scan_settle_sec 秒
    → 后台 daemon 线程调 std_srvs/Trigger 服务 /point_collect/record（整圈扫描阻塞
    数分钟，主循环仍以 ~8Hz 发零帧、键盘不死）→ 扫完自动还原泵为按键时的保存值。
    SETTLE/SCANNING 期间锁运动/泵/步进键；STOP('s'/空格) 与 Ctrl-C 恒响应，但语义不同：
    STOP 只能在 SETTLE 期(声呐尚未触发)取消扫描并回 IDLE；一旦进入 SCANNING，声呐已在
    采集端触发，本脚本无法从遥控端撤销，只能等采集器自行扫完，或 Ctrl-C 直接退出脚本
    （采集器扫描仍会自行跑完，不受影响）。
"""

import sys
import time
import select
import termios
import threading
import tty

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


HELP = """\
========== wheel_teleop_keyboard : w/x/a/d drive + t/g/y/h wheel trim ==========
 Move (throttle-style, setpoint PERSISTS until changed or STOP):

                    w  = forward
        a = turn left    x = back    d = turn right

 Per-wheel trim (independent L / R, the old tank-style fine control):
        t / g : LEFT  wheel  +/- one step
        y / h : RIGHT wheel  +/- one step

 Keys:
   w / x : move FORWARD / BACK  (both wheels +/- one step)
   a / d : turn LEFT / RIGHT    (differential: a = L-,R+ ; d = L+,R-)
   t / g : LEFT  wheel +/- one step (independent trim)
   y / h : RIGHT wheel +/- one step (independent trim)
   s  or  SPACE : STOP  (zero both wheels and pump)
   - / = : step size   down / up   (0.005 .. 0.050)
   [ / ] : pump duty   down / up   (0.00 .. 1.00)  suction only, no thrust
   c     : SCAN a point (stop + pump->0 + settle, then record 360 sweep in
           background; pump auto-restored after; motion/pump keys locked while
           SETTLE/SCANNING. STOP cancels the scan ONLY during SETTLE (sonar not
           fired yet); once SCANNING, sonar has already fired at the collector
           and cannot be cancelled from here -- wait for it to finish sweeping,
           or Ctrl-C to quit this script (collector keeps sweeping on its own).
   r     : reset step & pump to defaults
   Ctrl-C: quit (sends zero cmd_vel on exit)

 Each wheel clamped to +/- %.2f.  Publishing /cmd_vel continuously (~8 Hz).
 w/x move both wheels equally (keep current L-R turn bias) ; a/d change the
 turn (L-R diff) ; t/g/y/h trim one wheel independently.
 NOTE: which physical direction (w = forward? a = turn-left?) must be
 confirmed on the real robot before driving fast.
================================================================================
"""


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def get_key(settings, timeout=0.12):
    """读单个按键；timeout 内无输入返回''。期间临时进 raw、读完恢复 cooked。
    若首字节是 ESC(方向键/Home/End 等多字节转义)，排空其余字节并返回''，
    避免尾字节(如 '[')误触发已绑定键。"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            while select.select([sys.stdin], [], [], 0.0)[0]:
                sys.stdin.read(1)
            key = ''
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def publish_stop(pub, times=5, gap=0.03):
    """连发多帧零 Twist，保证最后的停止命令真正排空到驱动再退出。
    用 time.sleep(非 rospy.sleep)，shutdown 期间也能跑；publish 包 try。"""
    stop = Twist()
    for _ in range(times):
        try:
            pub.publish(stop)
        except Exception:
            pass
        time.sleep(gap)


def _record_worker(record_call, result, lock, done):
    """daemon 线程: 同步调用 record 服务(整圈扫描阻塞数分钟)。
    结果写入 result(受 lock 保护)后置位 done; 绝不碰 /cmd_vel 发布。
    主循环只在 done 置位后、加锁读取 result, 故无并发写读竞态。

    [同步契约 · 关系到扫描期关泵正确性] 本设计"扫描全程零泵、扫完才还原 pump"
    的正确性完全依赖 /point_collect/record 是【同步】服务: record_call() 必须在整圈
    扫描真正结束后才返回, 届时才 done.set() -> 主循环才在 SCANNING 分支还原泵。
    collector 端 manual_point_collector.cpp 明写"单线程 ros::spin, 严禁 AsyncSpinner"
    正是此契约的另一半。若将来把 collector 改成 AsyncSpinner / 提前返回, record_call()
    会在扫描中途就返回 -> 主循环会【在声呐还在扫时就把泵抬起来】, 且无任何报警。
    动 collector 并发模型前, 必须先改这里的还原时序(scan_done 语义)。"""
    try:
        resp = record_call()
        with lock:
            result['ok'] = bool(resp.success)
            result['msg'] = str(resp.message)
            result['err'] = None
    except Exception as e:
        with lock:
            result['ok'] = False
            result['msg'] = ''
            result['err'] = str(e)
    finally:
        done.set()


def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('wheel_teleop_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 必须与驱动一致；默认 = main 分支 hardware_driver.cpp 默认
    omega_gain = float(rospy.get_param('~omega_gain', 6.4))
    wheel_spacing = float(rospy.get_param('~wheel_spacing', 0.2))
    wheel_max = float(rospy.get_param('~wheel_max', 0.30))

    # 'c' 一键扫描参数: settle 秒数 + record 服务名(std_srvs/Trigger)。
    scan_settle_sec = float(rospy.get_param('~scan_settle_sec', 3.0))
    record_srv = str(rospy.get_param('~record_srv', '/point_collect/record'))

    # 反解需要 denom=wheel_spacing*omega_gain>0；二者必为正物理量。
    # 若被误配成 <=0，反解失效(转向会被静默丢成直行)，直接拒绝启动。
    # scan_settle_sec 须 >=0.5：亚循环周期的极小值(如 0.0005)会让 record 线程在本迭代
    # 首帧零泵发布之前就触发(SETTLE 零泵静置形同虚设)；下限 0.5s 保证 ~4 帧零泵先刷出。
    if (wheel_spacing <= 0.0 or omega_gain <= 0.0 or wheel_max <= 0.0
            or scan_settle_sec < 0.5):
        sys.stdout.write(
            "[wheel_teleop] FATAL: need wheel_spacing>0, omega_gain>0, wheel_max>0, "
            "scan_settle_sec>=0.5; got ws=%.4f gain=%.4f max=%.4f settle=%.4f. exit.\n"
            % (wheel_spacing, omega_gain, wheel_max, scan_settle_sec))
        sys.stdout.flush()
        return

    DEFAULT_STEP = 0.01
    DEFAULT_PUMP = 0.0
    STEP_MIN, STEP_MAX = 0.005, 0.05
    PUMP_STEP = 0.05

    denom = wheel_spacing * omega_gain  # (R - L) -> angular.z 的分母

    left = 0.0
    right = 0.0
    step = DEFAULT_STEP
    pump = DEFAULT_PUMP

    # --- 'c' scan state machine (方案 c): IDLE | SETTLE | SCANNING ---
    scan_state = 'IDLE'
    settle_deadline = rospy.Time(0)
    saved_pump = 0.0            # 'c' 按下时的 pump 值, 扫完还原
    scan_lock = threading.Lock()
    scan_done = threading.Event()
    scan_result = {}           # 后台线程写(加锁), 主循环 done 后读(加锁)

    sys.stdout.write(HELP % wheel_max)
    sys.stdout.write(
        "[wheel_teleop] config: wheel_spacing=%.3f omega_gain=%.2f wheel_max=%.2f"
        "  (MUST match the running driver)\n"
        % (wheel_spacing, omega_gain, wheel_max))
    sys.stdout.flush()

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            # Ctrl-C 与 STOP 在任意状态(含 SETTLE/SCANNING)都响应;
            # 运动/泵/步进/扫描键仅在 IDLE 生效(扫描期一律锁死忽略)。
            if key == '\x03':            # Ctrl-C
                break
            elif key == 's' or key == ' ':       # STOP: always honored
                left = 0.0
                right = 0.0
                pump = 0.0
                if scan_state == 'SETTLE':
                    # Sonar has NOT fired yet -> safe to cancel outright.
                    scan_state = 'IDLE'
                    sys.stdout.write(
                        "\r\n[wheel_teleop] SCAN CANCELLED (settle aborted, "
                        "sonar NOT fired).\r\n")
                    sys.stdout.flush()
                # SCANNING: sonar already fired at the collector -> cannot cancel
                # here, leave scan_state/thread untouched (see HELP/docstring).
                saved_pump = 0.0   # explicit operator STOP must never be silently
                                    # overridden later by a scan restoring the pump.
            elif scan_state != 'IDLE':
                # SETTLE/SCANNING: motion/pump/step/scan keys locked -> ignore.
                pass
            elif key == 'c':                     # start one-key SCAN (IDLE only)
                saved_pump = pump
                left = 0.0
                right = 0.0
                pump = 0.0
                scan_state = 'SETTLE'
                settle_deadline = rospy.Time.now() + rospy.Duration(scan_settle_sec)
                sys.stdout.write(
                    "\r\n[wheel_teleop] SCAN: stop + pump->0, settle %.1fs then record...\r\n"
                    % scan_settle_sec)
                sys.stdout.flush()
            elif key == 'w':                     # FORWARD: both wheels +step
                left = clamp(left + step, -wheel_max, wheel_max)
                right = clamp(right + step, -wheel_max, wheel_max)
            elif key == 'x':                     # BACK: both wheels -step
                left = clamp(left - step, -wheel_max, wheel_max)
                right = clamp(right - step, -wheel_max, wheel_max)
            elif key == 'a':                     # turn LEFT (CCW): L-step, R+step
                left = clamp(left - step, -wheel_max, wheel_max)
                right = clamp(right + step, -wheel_max, wheel_max)
            elif key == 'd':                     # turn RIGHT (CW): L+step, R-step
                left = clamp(left + step, -wheel_max, wheel_max)
                right = clamp(right - step, -wheel_max, wheel_max)
            elif key == 't':                     # LEFT wheel trim +step (independent)
                left = clamp(left + step, -wheel_max, wheel_max)
            elif key == 'g':                     # LEFT wheel trim -step (independent)
                left = clamp(left - step, -wheel_max, wheel_max)
            elif key == 'y':                     # RIGHT wheel trim +step (independent)
                right = clamp(right + step, -wheel_max, wheel_max)
            elif key == 'h':                     # RIGHT wheel trim -step (independent)
                right = clamp(right - step, -wheel_max, wheel_max)
            elif key == '-':
                step = clamp(step - 0.005, STEP_MIN, STEP_MAX)
            elif key == '=' or key == '+':
                step = clamp(step + 0.005, STEP_MIN, STEP_MAX)
            elif key == '[':
                pump = clamp(pump - PUMP_STEP, 0.0, 1.0)
            elif key == ']':
                pump = clamp(pump + PUMP_STEP, 0.0, 1.0)
            elif key == 'r':
                step = DEFAULT_STEP
                pump = DEFAULT_PUMP
            # 其它键忽略；设定值保持不变

            # ---- scan 状态机(方案 c): SETTLE 到点 -> 后台调 record -> SCANNING ----
            # 只在主线程里推进; 服务的阻塞调用交给 daemon 线程, 绝不在此同步阻塞主循环。
            if scan_state == 'SETTLE' and rospy.Time.now() >= settle_deadline:
                try:
                    rospy.wait_for_service(record_srv, timeout=2.0)
                except Exception:
                    # Broadened beyond rospy.ROSException: a malformed ~record_srv
                    # (e.g. invalid resolved name) can raise ValueError/other errors
                    # here too; catch broadly so a bad param can't crash the node.
                    sys.stdout.write(
                        "\r\n[wheel_teleop] SCAN ABORT: service '%s' unreachable or "
                        "invalid (collector up? name right?). pump restored, back to IDLE.\r\n"
                        % record_srv)
                    sys.stdout.flush()
                    pump = saved_pump
                    scan_state = 'IDLE'
                else:
                    scan_done.clear()
                    scan_result = {}
                    record_call = rospy.ServiceProxy(record_srv, Trigger)
                    scan_thread = threading.Thread(
                        target=_record_worker,
                        args=(record_call, scan_result, scan_lock, scan_done))
                    scan_thread.daemon = True
                    scan_thread.start()
                    scan_state = 'SCANNING'
                    sys.stdout.write(
                        "\r\n[wheel_teleop] SCANNING: call '%s' (full 360 sweep, minutes)...\r\n"
                        % record_srv)
                    sys.stdout.flush()
            elif scan_state == 'SCANNING' and scan_done.is_set():
                with scan_lock:
                    ok = scan_result.get('ok', False)
                    msg = scan_result.get('msg', '')
                    err = scan_result.get('err', None)
                if ok:
                    sys.stdout.write(
                        "\r\n[wheel_teleop] SCAN OK -> point_dir=%s ; pump restored to %.2f.\r\n"
                        % (msg, saved_pump))
                else:
                    detail = err if err is not None else (msg if msg else "unknown")
                    sys.stdout.write(
                        "\r\n[wheel_teleop] SCAN FAIL: %s ; pump restored to %.2f.\r\n"
                        % (detail, saved_pump))
                sys.stdout.flush()
                pump = saved_pump
                scan_state = 'IDLE'

            # ---- 反解为 /cmd_vel（驱动会重建 left_speed==left, right_speed==right）----
            vx = (left + right) / 2.0
            wz = (right - left) / denom   # denom 已在启动时校验为正

            # SETTLE/SCANNING 期间恒发零帧(无视 left/right/pump 残值), 保证扫描期整车静止。
            if scan_state == 'IDLE':
                pub_vx, pub_wz, pub_pump = vx, wz, pump
            else:
                pub_vx, pub_wz, pub_pump = 0.0, 0.0, 0.0

            twist = Twist()
            twist.linear.x = pub_vx
            twist.linear.y = pub_pump   # 水泵占空比 0~1（吸底，无推力）
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = pub_wz
            pub.publish(twist)

            # ---- 状态行（ASCII，单行刷新）----
            if scan_state == 'SETTLE':
                rem = (settle_deadline - rospy.Time.now()).to_sec()
                if rem < 0.0:
                    rem = 0.0
                scan_str = "SETTLE(t-%.0fs)" % rem
            elif scan_state == 'SCANNING':
                scan_str = "SCANNING"
            else:
                scan_str = "IDLE"
            motor_l = -left * 230.0   # 驱动里电机左 = -left_speed*230（仅供对照）
            motor_r = right * 230.0   # 驱动里电机右 = right_speed*230
            status = ("\rL=%+.3f R=%+.3f | vx=%+.3f wz=%+.3f | "
                      "motorL=%+5d motorR=%+5d | pump=%.2f step=%.3f | "
                      "scan=%s | ws=%.3f gn=%.2f   "
                      % (left, right, vx, wz,
                         int(motor_l), int(motor_r), pump, step,
                         scan_str, wheel_spacing, omega_gain))
            sys.stdout.write(status)
            sys.stdout.flush()

    except Exception as e:
        sys.stdout.write("\r\n[wheel_teleop] exception: %s\r\n" % str(e))
    finally:
        # 退出/异常：连发多帧零，确保最后停止命令真正排空到驱动（main 驱动无看门狗，
        # 单帧零若被传输层丢弃则电机会保持上一条非零命令）。真停车仍以断电为准。
        if scan_state == 'SCANNING':
            sys.stdout.write(
                "\r\n[wheel_teleop] NOTE: exiting while SCANNING -- the collector "
                "may still be sweeping (sonar still firing) independently.\r\n")
        publish_stop(pub)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sys.stdout.write("\r\n[wheel_teleop] stopped, zero cmd_vel sent (x5).\r\n")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
