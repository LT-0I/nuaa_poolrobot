#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wheel_teleop_keyboard.py —— 键盘"左右轮独立"遥控（基于 mogo_teleop_keyboard.py 改）

目的：用键盘分别给左轮 / 右轮一个转速设定值（坦克式差速遥控）。

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
"""

import sys
import time
import select
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """\
============== wheel_teleop_keyboard : independent L/R wheel ==============
 Layout (left column -> LEFT wheel, right column -> RIGHT wheel):

       q          w          e
    L wheel +   both +    R wheel +
       a          s          d
    L wheel -   STOP     R wheel -
                  x
               both -

 Keys:
   q / a : LEFT  wheel  +/- one step
   e / d : RIGHT wheel  +/- one step
   w / x : BOTH  wheels +/- one step (intended straight fwd/back)
   s  or  SPACE : STOP  (zero both wheels and pump)
   - / = : step size   down / up   (0.005 .. 0.050)
   [ / ] : pump duty   down / up   (0.00 .. 1.00)  suction only, no thrust
   r     : reset step & pump to defaults
   Ctrl-C: quit (sends zero cmd_vel on exit)

 Each wheel clamped to +/- %.2f.  Publishing /cmd_vel continuously (~8 Hz).
 L == R  -> straight ;  L != R -> turn in place / curve.
 NOTE: which physical direction (L>0 = forward? CW vs CCW) must be
 confirmed on the real robot before driving fast.
==========================================================================
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


def main():
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('wheel_teleop_keyboard')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # 必须与驱动一致；默认 = main 分支 hardware_driver.cpp 默认
    omega_gain = float(rospy.get_param('~omega_gain', 6.4))
    wheel_spacing = float(rospy.get_param('~wheel_spacing', 0.2))
    wheel_max = float(rospy.get_param('~wheel_max', 0.30))

    # 反解需要 denom=wheel_spacing*omega_gain>0；二者必为正物理量。
    # 若被误配成 <=0，反解失效(转向会被静默丢成直行)，直接拒绝启动。
    if wheel_spacing <= 0.0 or omega_gain <= 0.0 or wheel_max <= 0.0:
        sys.stdout.write(
            "[wheel_teleop] FATAL: need wheel_spacing>0, omega_gain>0, wheel_max>0; "
            "got ws=%.4f gain=%.4f max=%.4f. exit.\n"
            % (wheel_spacing, omega_gain, wheel_max))
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

    sys.stdout.write(HELP % wheel_max)
    sys.stdout.write(
        "[wheel_teleop] config: wheel_spacing=%.3f omega_gain=%.2f wheel_max=%.2f"
        "  (MUST match the running driver)\n"
        % (wheel_spacing, omega_gain, wheel_max))
    sys.stdout.flush()

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == '\x03':            # Ctrl-C
                break
            elif key == 'q':
                left = clamp(left + step, -wheel_max, wheel_max)
            elif key == 'a':
                left = clamp(left - step, -wheel_max, wheel_max)
            elif key == 'e':
                right = clamp(right + step, -wheel_max, wheel_max)
            elif key == 'd':
                right = clamp(right - step, -wheel_max, wheel_max)
            elif key == 'w':
                left = clamp(left + step, -wheel_max, wheel_max)
                right = clamp(right + step, -wheel_max, wheel_max)
            elif key == 'x':
                left = clamp(left - step, -wheel_max, wheel_max)
                right = clamp(right - step, -wheel_max, wheel_max)
            elif key == 's' or key == ' ':
                left = 0.0
                right = 0.0
                pump = 0.0
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

            # ---- 反解为 /cmd_vel（驱动会重建 left_speed==left, right_speed==right）----
            vx = (left + right) / 2.0
            wz = (right - left) / denom   # denom 已在启动时校验为正

            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = pump   # 水泵占空比 0~1（吸底，无推力）
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = wz
            pub.publish(twist)

            # ---- 状态行（ASCII，单行刷新）----
            motor_l = -left * 230.0   # 驱动里电机左 = -left_speed*230（仅供对照）
            motor_r = right * 230.0   # 驱动里电机右 = right_speed*230
            status = ("\rL=%+.3f R=%+.3f | vx=%+.3f wz=%+.3f | "
                      "motorL=%+5d motorR=%+5d | pump=%.2f step=%.3f | "
                      "ws=%.3f gn=%.2f   "
                      % (left, right, vx, wz,
                         int(motor_l), int(motor_r), pump, step,
                         wheel_spacing, omega_gain))
            sys.stdout.write(status)
            sys.stdout.flush()

    except Exception as e:
        sys.stdout.write("\r\n[wheel_teleop] exception: %s\r\n" % str(e))
    finally:
        # 退出/异常：连发多帧零，确保最后停止命令真正排空到驱动（main 驱动无看门狗，
        # 单帧零若被传输层丢弃则电机会保持上一条非零命令）。真停车仍以断电为准。
        publish_stop(pub)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        sys.stdout.write("\r\n[wheel_teleop] stopped, zero cmd_vel sent (x5).\r\n")
        sys.stdout.flush()


if __name__ == '__main__':
    main()
