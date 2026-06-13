# 沿边行走修复 · move_controller

> 本文档对照本仓库源码说明**已实施的代码修改逻辑与效果**。
> 配套诊断见 [wall_following_diagnosis.md](wall_following_diagnosis.md)。
> 改动文件：消费侧 [`sonar_slam/src/move_controller.cpp`](../sonar_slam/src/move_controller.cpp)、[`sonar_slam/include/sonar_slam/move_controller.h`](../sonar_slam/include/sonar_slam/move_controller.h)（仅改活跃函数 `walkStraightWithWallAvoidanceAdvanced`、`ultraCallback`，死代码块与 `alignToRightWallOnly` 未动）；生产侧 [`mars_serial_server/src/hardware_driver.cpp`](../mars_serial_server/src/hardware_driver.cpp)（仅改 `rosNode` 超声定时器与 `sendUltraCommand` 调度，见 §5）。

> **未编译验证**：本机无 ROS 环境，需在机器人上 `catkin_make -j1`（内存小，禁多核）。控制律的符号/收敛/不撞墙已用独立离线仿真穷举验证（无 ROS，复现见仓外实验目录）。

---

## 修复总览（对应三大根因）

| 根因 | 修复 | 代码 |
|---|---|---|
| ① 无"距墙距离"控制环 | 直行段改成**距墙距离环 + 平行环**，复活 `config_wall_align_`（默认 2.0m） | [straight law @ L1222](../sonar_slam/src/move_controller.cpp#L1222) |
| ② 拐角用 0.83Hz 旧超声值在 20Hz 循环提前停 | 拐角改 **odom 粗转 + "新鲜帧"精修(stop-and-settle)**；新增超声新鲜度计数 | [corner @ L1025](../sonar_slam/src/move_controller.cpp#L1025) · [ultraCallback @ L1738](../sonar_slam/src/move_controller.cpp#L1738) |
| ③ 拐后错航向被 odom 锁成零基准 | **超声为主、odom 仅丢墙兜底**；超声有效时持续把 odom 参考归零 | [L1266](../sonar_slam/src/move_controller.cpp#L1266) / [L1286](../sonar_slam/src/move_controller.cpp#L1286) |

---

## 1. 传感器层：右侧超声"新鲜度"

每路右侧超声真实只 ~0.83Hz（1.2s 刷一次），话题却 20Hz（零阶保持，旧值复发）。新增：原始值变化即视为收到**新帧**，并记时间戳。

- 回调里自增序号 + 记时间戳：[`ultraCallback` L1738](../sonar_slam/src/move_controller.cpp#L1738)
- 新成员：[`ultra_right_seq_` / `ultra_right_stamp_` L201](../sonar_slam/include/sonar_slam/move_controller.h#L201)

`seq`（值变计数）供拐角"只在新帧动作"用；`stamp`（陈旧看门狗）供直行"超声死了就退回 odom"用。

## 2. 拐角左转：odom 粗转 + 新鲜帧精修

[corner block @ L1025](../sonar_slam/src/move_controller.cpp#L1025)

- **阶段A**（[L1054](../sonar_slam/src/move_controller.cpp#L1054)）：用 odom 开环快转到 `COARSE_DEG=80°`（odom 相对角短程可靠）。
- **阶段B**（[L1075](../sonar_slam/src/move_controller.cpp#L1075)）：**只在 `seq` 推进（收到新帧）时**评估平行；连续 `2` 个新鲜帧 `|rear−front|<0.025` 才算转完。未平行则朝误差方向发一个 `0.15s` 限时脉冲（单次约 5°）再回到"停—等新帧"。
  - 这样**杜绝了"同一旧快照在 20Hz 循环里复算 24 次→提前停"**（根因②）。
- **兜底**：8s 超时；95° 角度上限；**长时间(>2.5s)无新帧**（超声疑似无回波）→ 慢速开环补转，由 95° 收口退出（[L1084](../sonar_slam/src/move_controller.cpp#L1084)），不会空等到 8s、也不会卡死。

## 3. 直行段：距墙距离环 + 平行环（超声为主、odom 兜底）

[straight law @ L1222](../sonar_slam/src/move_controller.cpp#L1222)，替换原"odom 为主的四分支双传感器逻辑"。

- 超声有效且新鲜（[L1266](../sonar_slam/src/move_controller.cpp#L1266)）：
  ```
  d      = (右前 + 右后) / 2          // 距右墙
  e_par  = 右后 − 右前                 // >0: 机头朝墙
  w_cmd  = K_DIST·(target − d) + K_PAR·e_par   // 限幅 ±MAX_CORR
  ```
  这是标准墙跟随：距离项给**横向位置回正力**（消灭根因①——以前只有平行项，任何残差航向都让距墙单调漂移），平行项提供航向阻尼。
  同时把 odom 参考归零（`total_yaw_change=0`），使"丢墙兜底"保持的是**当前由超声建立的航向**，而非拐弯遗留的错航向（消灭根因③）。
- 仅一侧有效（[L1279](../sonar_slam/src/move_controller.cpp#L1279)）：按既有约定轻纠以重捕获，并刷新 odom 基准。
- 两侧无效/陈旧>看门狗（[L1286](../sonar_slam/src/move_controller.cpp#L1286)）：退回 odom 短时兜底保航向（odom 是裸陀螺、会漂，只作兜底）。陈旧看门狗现为 **ROS 参数 `wall_stale_sec`**（默认 **1.5s**、夹 [0.5,5.0]；[读取 L76](../sonar_slam/src/move_controller.cpp#L76)、[使用 L1248](../sonar_slam/src/move_controller.cpp#L1248)）：原 3.0 常量是按旧 0.83Hz(1.2s/路)定的；D-41 把右侧超声提到 ~3.3Hz(0.3s/路)后默认改 1.5s≈5×刷新周期，把丢回波时的盲走距离从 ~1m 压到 ~0.5m(0.35m/s)。**现场可 `rosparam set /wall_stale_sec X`（启动前设、节点启动时读一次）调，不必重编**。⚠ 须≥右侧刷新周期：若驱动 `ultra_poll_period` 调大或回退旧 1.2s/路，须同步上调本值。
- 读右侧超声统一加 `sensor_mutex_` 锁，顺带修掉原裸读竞态。

**增益**（[L1235](../sonar_slam/src/move_controller.cpp#L1235)）：`K_DIST=0.03, K_PAR=0.30, 限幅 0.08`。
单位为 cmd（驱动里再 ×6.4 成真实角速度）。这组值经离线仿真在 0.83Hz 超声的零阶保持下扫参确定为**稳定收敛、不振荡、不撞墙**；现场再按 `沿墙: d=.. e_par=.. -> w=..` 日志精调。

## 4. 距墙目标 2m

`config_wall_align_` 默认 1.0→**2.0**（[成员初始化](../sonar_slam/src/move_controller.cpp#L852) 与 `wall_align` 参数默认），可经 launch 参数覆盖。头注释已从"保留备用"更正为"目标距右墙距离"（[L167](../sonar_slam/include/sonar_slam/move_controller.h#L167)）。

## 5. 驱动层：超声请求加速 + 右侧优先调度（让数据更新鲜）

> §1–§4 只在消费侧"识别"旧帧；本节从**生产侧**根治：以前右侧每路真实只 ~0.83Hz，是因为驱动 300ms 才请求一次、四路平均轮询。

改 [`mars_serial_server/src/hardware_driver.cpp`](../mars_serial_server/src/hardware_driver.cpp)，两处：

- **请求周期 0.3s→0.1s，且可配**（[`rosNode` L198](../mars_serial_server/src/hardware_driver.cpp#L198)）：新增 ROS 参数 `ultra_poll_period`（默认 0.1s=10 次/秒，夹在 `[0.05,0.5]s` 防误设），匹配 MCU 上限"四路超声每秒 10 帧"（MCU 侧工程师 2026-06-13 告知）。启动时 `ROS_INFO` 打印实际每路刷新率，便于现场核对。
- **右侧优先 6 拍调度 `{3,4,1,3,4,2}`**（[`sendUltraCommand` L1937](../mars_serial_server/src/hardware_driver.cpp#L1937)）：右侧（命令 3/4）是沿墙距离环的连续控制输入，每 6 拍占 2 拍；前方（命令 1/2）只做前障检测，各占 1 拍。

效果（默认 0.1s 周期）：

| 通道 | 旧（300ms 均轮询） | 新（100ms 右优先） |
|---|---|---|
| 右前/右后（命令 3/4，距离环用） | 0.83 Hz | **~3.3 Hz**（每 3 拍/0.3s） |
| 前左/前右（命令 1/2，前障用） | 0.83 Hz | **~1.7 Hz**（每 6 拍/0.6s） |

**单调改善、无回退**：总请求带宽由 3.3 次/秒提到 10 次/秒（×3），因此**每一路的请求率都不低于旧的 0.83Hz**——右侧大涨、前方仍翻倍。无论 MCU 是"按请求即测"还是"内部缓存最新值"模型，实际刷新率 = min(请求率, MCU 内部率)，都不会低于旧基线（论证见仓外实验目录 `analysis/review/REVIEW_AUDIT.md`）。确切量级以现场实测 `ultra_right_seq_` 推进速率为准。

> 物理映射以**消费侧 `move_controller` 为准**（用户 2026-06-13 定）：命令 1→`data[0]`=前左、2→`data[1]`=前右、3→`data[2]`=右前、4→`data[3]`=右后。驱动原注释把 1/2 标"右侧"、3/4 标"前方红外"系笔误，已订正。
>
> ⚠ 改了 C++ → 必须在机器人上 `catkin_make -j1`（`mars_serial_server` 与 `sonar_slam` 两个包都要重编），详见仓外实验目录的编译单。

---

## 现场前置确认（控制律有效性的前提）

1. **遮挡定标**：以消费侧映射为准（命令 3/4 = 右前/右后 = `data[2]/[3]`，用户已定），现场遮挡定标只为**复核**无误，非再判定标号。
2. **3m 掠射回波存活性**：距墙 2m 时右侧超声须有有效回波，否则律退回 odom 漂移兜底（修复①失效）。建议沿墙实验前先静态采集确认。
3. **实测右侧刷新率**：跑起来后量 `ultra_right_seq_` 的推进速率，确认右前/右后是否到 ~3Hz（验证 §5 的右侧优先确实生效、MCU 不是缓存模型卡上限）。
4. 增益按首轮 CSV 日志精调。

*实施后按双对抗评审 + 对照活仓核对 + `catkin_make -j1` 编译下发；详见仓外实验目录的编译单/操作单。*
