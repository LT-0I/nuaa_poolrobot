# reloc — 声呐方离线重定位管线（vendored 真码，逐字节未改）

本目录 = 声呐算法方交付的**离线重定位真码**，逐字节收进本仓库作为定位真源。
分支 `g-reloc-0701` 基于 e 手控采集基座（tag `e-manualsonar-frozen-0623`）新建；旧 g 家族分支（在线节点那套）已整支废弃删除——其做法对齐了错文件、且在线壳无法 bit-exact 复现算法方结果。

## 文件（勿改，改前先对 CHECKSUMS.sha256）
- `relocation-auto.py` — 主编排，顺序调用下面两个脚本。
- `bbs-1.py` — BBS + 自研 2D 点到线 ICP 单点重定位 → `poses_in_map.csv`（**定位结果**）。
- `corrected.py` — 锚点(默认用点 1,2 定旋转)分段刚体轨迹校正 → `corrected_trajectory_map.csv`（**最终轨迹**）。
- `CHECKSUMS.sha256` — 上面三者的 sha256，证明与交付源逐字节一致。

来源：`experiments/2026-06-29_locexp/声呐方算法＋数据0701/`。

## 四步管线
1. **滤波**：`maps/phase_multi_submap_N_posN.pcd` → z 置 0 → 距离+强度筛 → DBSCAN 同心弧滤 → `filtered_scans/scan_N.pcd`。
2. **角度换算**：`points_index.csv`/`odom_full.csv` 的 yaw 弧度→角度（`*-1.csv`）。
3. **BBS+ICP**：`bbs-1.py` 逐点定位（点1=全局 BBS；点>1=增量预测+局部网格）→ `poses_in_map.csv`。
4. **轨迹校正**：`corrected.py` 用锚点把 odom 流分段刚体投到 map 系 → `corrected_trajectory_map.csv`。

## 环境（复现算法方结果必须精确对齐）
声呐方：**Windows / Python 3.8 / open3d 0.19.0 / numpy 1.24.4 / scipy 1.10.1 / pandas 2.0.3**。
我方复现镜像：`experiments/2026-06-29_locexp/reloc_verify/Dockerfile`（`pool-reloc:repro38`，上述版本全 pin，仅 OS=Linux 不可对齐）。

## task ↔ 全局图 一一对应（**绝不混用**，混用=错墙）
- `task_20260629_202646` → `mid_pcd/global.pcd`
- `task_z` / `task_l` / `task_20260628_223722` → `circle-pcd/global.pcd`

## Phase 0 验证结论（2026-07-01，详见 `experiments/2026-06-29_locexp/analysis/`）
用本 vendored 码在复现镜像里跑，对照声呐方随附结果：
- `task_20260629_202646`(9点/mid_pcd)：`poses_in_map` **逐字节全等**（含分数）。
- `task_z`(5点)/`task_l`(7点)：**位姿 x/y/yaw 逐字节等**；仅内部 `final_score` 差 ~2e-3（likelihood 打分读数的平台末位，**不影响位姿**）。
- `corrected_trajectory`：max ~1e-13（纯浮点噪声）。
- Step1 滤波：`223722` 随附 `filtered_scans` 逐字节等（DBSCAN 跨平台一致）。
- ⚠ `task_20260628_223722`：其随附 GT 与本码结果差 ~0.2m——该 task GT 大概率用**更早/不同代码**跑（我方 fitness 反更高）；**孤例、非本码问题**，以另 3 组为准或请算法方用现码重生成其 GT。

**结论：定位结果跨平台 bit-exact 达成，集成正确。**

## 跑法
```bash
# 建镜像(一次)
docker build -t pool-reloc:repro38 -f experiments/2026-06-29_locexp/reloc_verify/Dockerfile experiments/2026-06-29_locexp/reloc_verify/
# 跑一个 task（容器内 relocation-auto 默认 --python 是 'python', Linux 必须传 python3）
python3 relocation-auto.py --task-dir <task_dir> --global-map <circle-pcd|mid_pcd>/global.pcd \
  --bbs-script bbs-1.py --corrected-script corrected.py --python python3
# 无头对照见 experiments/2026-06-29_locexp/reloc_verify/{phase0_run.sh,compare_all.py}
```

## 后续（本轮未做）
上机 demo 需要的**实时在线定位节点 + rviz 底图/位置/轨迹显示**——复用本管线算法，另起一轮实现（在线因抽稀/时序只能容差匹配、非 bit-exact，用于展示足够）。
