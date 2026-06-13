# 沿边行走修复 · move_controller

> 本文档对照本仓库源码说明**已实施的代码修改逻辑与效果**。
> 配套诊断见 [wall_following_diagnosis.md](wall_following_diagnosis.md)。
> 改动文件：[`sonar_slam/src/move_controller.cpp`](../sonar_slam/src/move_controller.cpp)、[`sonar_slam/include/sonar_slam/move_controller.h`](../sonar_slam/include/sonar_slam/move_controller.h)。仅改活跃函数 `walkStraightWithWallAvoidanceAdvanced`、`ultraCallback`，死代码块与 `alignToRightWallOnly` 未动。

> **未编译验证**：本机无 ROS 环境，需在机器人上 `catkin_make -j1`（内存小，禁多核）。控制律的符号/收敛/不撞墙已用独立离线仿真穷举验证（无 ROS，复现见仓外实验目录）。

---

## 修复总览（对应三大根因）

| 根因 | 修复 | 代码 |
|---|---|---|
| ① 无"距墙距离"控制环 | 直行段改成**距墙距离环 + 平行环**，复活 `config_wall_align_`（默认 2.0m） | [straight law @ L1222](../sonar_slam/src/move_controller.cpp#L1222) |
| ② 拐角用 0.83Hz 旧超声值在 20Hz 循环提前停 | 拐角改 **odom 粗转 + "新鲜帧"精修(stop-and-settle)**；新增超声新鲜度计数 | [corner @ L1025](../sonar_slam/src/move_controller.cpp#L1025) · [ultraCallback @ L1738](../sonar_slam/src/move_controller.cpp#L1738) |
| ③ 拐后错航向被 odom 锁成零基准 | **超声为主、odom 仅丢墙兜底**；超声有效时持续把 odom 参考归零 | [L1257](../sonar_slam/src/move_controller.cpp#L1257) / [L1276](../sonar_slam/src/move_controller.cpp#L1276) |

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

- 超声有效且新鲜（[L1257](../sonar_slam/src/move_controller.cpp#L1257)）：
  ```
  d      = (右前 + 右后) / 2          // 距右墙
  e_par  = 右后 − 右前                 // >0: 机头朝墙
  w_cmd  = K_DIST·(target − d) + K_PAR·e_par   // 限幅 ±MAX_CORR
  ```
  这是标准墙跟随：距离项给**横向位置回正力**（消灭根因①——以前只有平行项，任何残差航向都让距墙单调漂移），平行项提供航向阻尼。
  同时把 odom 参考归零（`total_yaw_change=0`），使"丢墙兜底"保持的是**当前由超声建立的航向**，而非拐弯遗留的错航向（消灭根因③）。
- 仅一侧有效（[L1272](../sonar_slam/src/move_controller.cpp#L1272)）：按既有约定轻纠以重捕获，并刷新 odom 基准。
- 两侧无效/陈旧>3s（[L1276](../sonar_slam/src/move_controller.cpp#L1276)）：退回 odom 短时兜底保航向（odom 是裸陀螺、会漂，只作兜底）。
- 读右侧超声统一加 `sensor_mutex_` 锁，顺带修掉原裸读竞态。

**增益**（[L1235](../sonar_slam/src/move_controller.cpp#L1235)）：`K_DIST=0.03, K_PAR=0.30, 限幅 0.08`。
单位为 cmd（驱动里再 ×6.4 成真实角速度）。这组值经离线仿真在 0.83Hz 超声的零阶保持下扫参确定为**稳定收敛、不振荡、不撞墙**；现场再按 `沿墙: d=.. e_par=.. -> w=..` 日志精调。

## 4. 距墙目标 2m

`config_wall_align_` 默认 1.0→**2.0**（[成员初始化](../sonar_slam/src/move_controller.cpp#L852) 与 `wall_align` 参数默认），可经 launch 参数覆盖。头注释已从"保留备用"更正为"目标距右墙距离"（[L167](../sonar_slam/include/sonar_slam/move_controller.h#L167)）。

---

## 现场前置确认（控制律有效性的前提）

1. **遮挡定标**：确认 `data[2]/[3]` 确为右侧前/后超声（驱动请求侧通道标号存在历史冲突）。整套律依赖此映射。
2. **3m 掠射回波存活性**：距墙 2m 时右侧超声须有有效回波，否则律退回 odom 漂移兜底（修复①失效）。建议沿墙实验前先静态采集确认。
3. 增益按首轮 CSV 日志精调。

*实施后按双对抗评审 + 对照活仓核对 + `catkin_make -j1` 编译下发；详见仓外实验目录的编译单/操作单。*
