#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sonar_stitch.py - 声呐子图离线拼接 (ICP)

接入说明:
  拼接算法(滤波 / 2D point-to-line ICP / 合并, 即 section 1-8 与 main())
  逐字搬自 experiments/2026-06-20_demo-readiness/拼图/pinjie-auto1.py, 未改一行。
  本文件只在外层把四个路径常量参数化(指向本次沿边采集的会话目录),
  并加命令行入口 + 版本横幅。不动拼接方法本身。

用法:
  python3 sonar_stitch.py <session_dir>   # 拼指定会话目录(<session>/maps + <session>/sonar_odom_log.csv)
  python3 sonar_stitch.py --latest        # 自动拼 ~/sonar_logs/ 下时间戳最新的 scan_* 会话
  (不带参数 = 退回原脚本的默认相对路径行为)
输出: <session_dir>/stitching.pcd
"""

# 版本横幅(grep 自判): 改本脚本行为必改此串。拼接算法(ICP/合并)照搬未改;
# 逐帧滤波 MIN_INNER 对齐 lvbo=90(原60), 八个滤波参数可经 sonar_stitch_node 的 ROS 私有参数(~stitch_*)覆盖。
STITCH_VERSION = "sonar-stitch-lvbofilter-mininner90-param-20260625"

import open3d as o3d
import numpy as np
import copy
import struct
import os
import re
import pandas as pd
from collections import defaultdict

# ================= 0) 路径与参数配置 =================

# 文件夹路径配置 (默认相对路径仅离线手动跑用; 实际运行由 _resolve_paths(session_dir) 覆盖)
INPUT_DIR = "sonar_logs/maps"
FILTER_DIR = "sonar_logs/maps/filter"
ODOM_FILE_BASE = "sonar_logs/sonar_odom_log"
OUTPUT_PCD = "sonar_logs/stitching.pcd"


def _resolve_paths(session_dir):
    """把四个路径常量指向本次沿边采集会话目录 ~/sonar_logs/scan_<时间>/。
    布局(C++ multi_position_sonar_mapper 实测): <session>/maps/*.pcd + <session>/sonar_odom_log.csv。
    只改路径, 不碰下面任何拼接算法。"""
    global INPUT_DIR, FILTER_DIR, ODOM_FILE_BASE, OUTPUT_PCD
    session_dir = os.path.abspath(os.path.expanduser(session_dir))
    INPUT_DIR = os.path.join(session_dir, "maps")
    FILTER_DIR = os.path.join(session_dir, "maps", "filter")
    ODOM_FILE_BASE = os.path.join(session_dir, "sonar_odom_log")
    OUTPUT_PCD = os.path.join(session_dir, "stitching.pcd")
    return session_dir


def find_latest_session(root=None):
    """取 ~/sonar_logs/ 下时间戳最新的 scan_* 会话目录。
    名 scan_%Y%m%d_%H%M%S 字典序即时间序, 故 sorted()[-1] = 最新。"""
    if root is None:
        root = os.path.join(os.path.expanduser("~"), "sonar_logs")
    if not os.path.isdir(root):
        return None
    cands = sorted(d for d in os.listdir(root)
                   if d.startswith("scan_") and os.path.isdir(os.path.join(root, d)))
    return os.path.join(root, cands[-1]) if cands else None

# --- 滤波参数 (逐帧, 拼接前对每帧 maps/*.pcd 跑; 与离线 lvbo 滤波一致) ---
# 全部为模块级常量, 可经 sonar_stitch_node 读 ROS 私有参数 ~stitch_* 覆盖, 不必改本脚本。
MIN_DIST = 1.2
MAX_DIST = 12.0
MIN_INNER = 90.0          # 内圈(<=MAX_DIST)强度下限; 对齐 lvbo=90(原 60, 偏松易漏鬼影); 可经 ~stitch_min_inner 覆盖
MIN_OUTER = 120.0
MAX_LIMIT = 200.0
ARC_EPS = 0.3
ARC_MIN_POINTS = 10
ARC_MAX_RADIUS_STD = 0.25

# --- ICP 与拼接参数---
VOXEL_SIZE_ICP = 0.15  # ICP 体素大小
VOXEL_SIZE_FINAL = 0.02  # 最终保存地图体素大小
ICP_MAX_ITER = 40  # 2D 点到线 ICP 最大迭代次数
POINT_TO_LINE_MAX_CORR_DIST = 0.70  # 最大对应距离 (m)
TARGET_NORMAL_RADIUS = 0.45  # 地图法向量估计半径
TARGET_NORMAL_MIN_NN = 8  # 法向量估计最少邻居数
TARGET_LINE_MIN_LINEARITY = 0.25  # 线性度阈值
TRIM_RATIO = 0.80  # ICP 残差截断比例
USE_INTENSITY_FILTER_FOR_ICP = True
ICP_INTENSITY_PERCENTILE = 50  # 只保留强度在此百分位以上的点参与 ICP
LOCAL_MAP_MARGIN = 2.0  # 局部地图裁剪范围扩展距离 (m)

# ICP 质量门限
MIN_ICP_FITNESS = 0.12
MAX_ICP_RMSE = 0.35
MAX_CORRECTION_TRANS = 0.80  # 单次 ICP 允许修正的最大平移 (m)
MAX_CORRECTION_YAW_DEG = 8.0  # 单次 ICP 允许修正的最大角度 (deg)

FORCE_ODOM_ONLY_IDS = set()


# ================= 1) PCD 读写辅助函数 =================

def read_pcd_binary_xyzi(file_path):
    with open(file_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header 读取失败")
            header_lines.append(line.decode("utf-8", errors="ignore").strip())
            if header_lines[-1].startswith("DATA"):
                break

        header = {}
        for ln in header_lines:
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split()
            header[parts[0].upper()] = parts[1:]

        fields = [s.lower() for s in header.get("FIELDS", [])]
        sizes = list(map(int, header.get("SIZE", [])))
        counts = list(map(int, header.get("COUNT", []))) if "COUNT" in header else [1] * len(fields)
        points = int(
            header.get(
                "POINTS",
                [str(int(header.get("WIDTH", ["0"])[0]) * int(header.get("HEIGHT", ["1"])[0]))]
            )[0]
        )

        point_step = sum(s * c for s, c in zip(sizes, counts))
        blob = f.read(points * point_step)

        fmt = "<ffff"
        pts = np.empty((points, 3), dtype=np.float64)
        intens = np.empty((points,), dtype=np.float32)

        offset = 0
        for i in range(points):
            x, y, z, inten = struct.unpack_from(fmt, blob, offset)
            pts[i, 0], pts[i, 1], pts[i, 2], intens[i] = x, y, z, inten
            offset += point_step

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd, intens


def save_pcd_binary_xyzi(pcd, intensity, filename):
    pts = np.asarray(pcd.points)
    N = len(pts)
    header = (
        f"# .PCD v0.7 - Point Cloud Data file format\n"
        f"VERSION 0.7\n"
        f"FIELDS x y z intensity\n"
        f"SIZE 4 4 4 4\n"
        f"TYPE F F F F\n"
        f"COUNT 1 1 1 1\n"
        f"WIDTH {N}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {N}\n"
        f"DATA binary\n"
    )
    with open(filename, "wb") as f:
        f.write(header.encode("utf-8"))
        for i in range(N):
            f.write(
                struct.pack(
                    "<ffff",
                    np.float32(pts[i, 0]),
                    np.float32(pts[i, 1]),
                    np.float32(pts[i, 2]),
                    np.float32(intensity[i])
                )
            )


# ================= 2) 滤波算法函数 =================

def filter_base_distance_intensity(pcd, intensity, min_dist, max_dist, min_inner, min_outer, max_limit):
    pts = np.asarray(pcd.points)
    distances = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)

    mask_valid_distance = (distances > min_dist)
    mask_inner = (distances <= max_dist) & (intensity >= min_inner)
    mask_outer = (distances > max_dist) & (intensity >= min_outer)

    keep_mask = (mask_inner | mask_outer) & (intensity < max_limit) & mask_valid_distance

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(pts[keep_mask])
    return filtered_pcd, intensity[keep_mask]


def filter_concentric_arcs(pcd, intensity, eps, min_points, max_radius_std):
    pts = np.asarray(pcd.points)
    if len(pts) < min_points:
        return pcd, intensity

    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
    keep_mask = np.zeros(len(pts), dtype=bool)
    max_label = labels.max()

    for k in range(max_label + 1):
        idx = np.where(labels == k)[0]
        cluster_pts = pts[idx]

        if len(idx) > 250:
            keep_mask[idx] = True
            continue

        cluster_r = np.sqrt(cluster_pts[:, 0] ** 2 + cluster_pts[:, 1] ** 2)
        std_r = np.std(cluster_r)

        if std_r < max_radius_std:
            continue

        keep_mask[idx] = True

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(pts[keep_mask])
    return filtered_pcd, intensity[keep_mask]


def sor_filter_with_intensity(pcd, intensity, nb_neighbors=30, std_ratio=1.5):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pcd2 = pcd.select_by_index(ind)
    inten2 = intensity[ind] if intensity is not None and len(intensity) == len(pcd.points) else None
    return pcd2, inten2


def force_to_2d(pcd):
    pts = np.asarray(pcd.points).copy()
    pts[:, 2] = 0.0
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def voxel_downsample_with_intensity(pcd, intensity, voxel_size):
    pts = np.asarray(pcd.points)
    if len(pts) == 0:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        return pcd2, np.empty((0,), dtype=np.float32)

    keys = np.floor(pts / voxel_size).astype(np.int64)
    key_tuple = [tuple(k) for k in keys]

    bucket_pts = defaultdict(list)
    bucket_int = defaultdict(list)

    for i, k in enumerate(key_tuple):
        bucket_pts[k].append(pts[i])
        bucket_int[k].append(float(intensity[i]) if intensity is not None else 0.0)

    new_pts = np.empty((len(bucket_pts), 3), dtype=np.float64)
    new_int = np.empty((len(bucket_pts),), dtype=np.float32)

    for j, k in enumerate(bucket_pts.keys()):
        new_pts[j] = np.mean(np.asarray(bucket_pts[k], dtype=np.float64), axis=0)
        new_int[j] = np.float32(np.mean(np.asarray(bucket_int[k], dtype=np.float64)))

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(new_pts)
    return pcd2, new_int


def filter_by_intensity_percentile(pcd, intensity, percentile=60):
    if intensity is None or len(intensity) != len(pcd.points) or len(intensity) == 0:
        return pcd, intensity

    pts = np.asarray(pcd.points)
    th = np.percentile(intensity, percentile)
    mask = intensity >= th

    if np.sum(mask) < 20:
        return pcd, intensity

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pts[mask])
    return pcd2, intensity[mask]


# ================= 3) 矩阵变换与几何工具 =================

def get_transform_matrix_from_odom(x, y, theta_deg):
    theta = np.radians(theta_deg)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, x],
        [np.sin(theta), np.cos(theta), 0, y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float64)


def yaw_from_T(T):
    return np.arctan2(T[1, 0], T[0, 0])


def wrap_angle_rad(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def make_T_from_xyyaw(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
    T[0, 3], T[1, 3] = x, y
    return T


def params_from_T_2d(T):
    return T[0, 3], T[1, 3], yaw_from_T(T)


def transform_points_2d(pts_xy, x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    out = np.empty_like(pts_xy)
    out[:, 0] = c * pts_xy[:, 0] - s * pts_xy[:, 1] + x
    out[:, 1] = s * pts_xy[:, 0] + c * pts_xy[:, 1] + y
    return out


# ================= 4) 2D KDTree 与法向量估计 =================

class SimpleKDTree2D:
    def __init__(self, points_xy):
        self.points = np.asarray(points_xy, dtype=np.float64)
        try:
            from scipy.spatial import cKDTree
            self.tree = cKDTree(self.points)
            self.use_scipy = True
        except Exception:
            self.tree = None
            self.use_scipy = False
            print("警告：未检测到 scipy，将使用 numpy 暴力最近邻搜索，速度可能较慢。")

    def query(self, query_xy, k=1, distance_upper_bound=np.inf):
        query_xy = np.asarray(query_xy, dtype=np.float64)
        if self.use_scipy:
            return self.tree.query(query_xy, k=k, distance_upper_bound=distance_upper_bound)

        if k != 1:
            raise NotImplementedError("numpy fallback 只实现了 k=1 查询")
        dist = np.empty((len(query_xy),), dtype=np.float64)
        idx = np.empty((len(query_xy),), dtype=np.int64)
        for i, q in enumerate(query_xy):
            d2 = np.sum((self.points - q) ** 2, axis=1)
            j = int(np.argmin(d2))
            d = np.sqrt(d2[j])
            if d <= distance_upper_bound:
                dist[i], idx[i] = d, j
            else:
                dist[i], idx[i] = np.inf, len(self.points)
        return dist, idx

    def query_radius_indices(self, q, radius):
        q = np.asarray(q, dtype=np.float64)
        if self.use_scipy:
            return self.tree.query_ball_point(q, radius)
        d2 = np.sum((self.points - q) ** 2, axis=1)
        return np.where(d2 <= radius * radius)[0].tolist()


def estimate_2d_line_normals(points_xy, radius=0.45, min_nn=8, min_linearity=0.35):
    points_xy = np.asarray(points_xy, dtype=np.float64)
    if len(points_xy) == 0:
        return np.empty((0, 2)), np.empty((0, 2)), np.empty((0,))

    tree = SimpleKDTree2D(points_xy)
    valid_pts, normals, linearities = [], [], []

    for i, p in enumerate(points_xy):
        inds = tree.query_radius_indices(p, radius)
        if len(inds) < min_nn:
            continue
        neigh = points_xy[inds]
        mu = np.mean(neigh, axis=0)
        X = neigh - mu
        C = (X.T @ X) / max(len(neigh), 1)
        try:
            eigvals, eigvecs = np.linalg.eigh(C)
        except np.linalg.LinAlgError:
            continue

        l0, l1 = eigvals[0], eigvals[1]
        if l1 < 1e-12:
            continue
        linearity = (l1 - l0) / (l1 + 1e-12)
        if linearity < min_linearity:
            continue

        n = eigvecs[:, 0]
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-12:
            continue
        n = n / n_norm

        valid_pts.append(p)
        normals.append(n)
        linearities.append(linearity)

    if len(valid_pts) == 0:
        return np.empty((0, 2)), np.empty((0, 2)), np.empty((0,))
    return np.asarray(valid_pts), np.asarray(normals), np.asarray(linearities)


# ================= 5) 2D Point-to-Line ICP =================

def execute_2d_point_to_line_icp(source_pcd, target_map_pcd, initial_T, max_corr_dist=0.50,
                                 normal_radius=0.45, normal_min_nn=8, normal_min_linearity=0.35,
                                 max_iter=40, trim_ratio=0.80, verbose=False):
    src_pts_3d = np.asarray(source_pcd.points)
    tgt_pts_3d = np.asarray(target_map_pcd.points)

    if len(src_pts_3d) < 20 or len(tgt_pts_3d) < 50:
        return initial_T, {"success": False, "fitness": 0.0, "rmse": np.inf, "num_corr": 0, "reason": "点数不足"}

    src_xy = src_pts_3d[:, :2].astype(np.float64)
    tgt_xy_raw = tgt_pts_3d[:, :2].astype(np.float64)

    tgt_xy, tgt_normals, linearity = estimate_2d_line_normals(
        tgt_xy_raw, radius=normal_radius, min_nn=normal_min_nn, min_linearity=normal_min_linearity
    )

    if len(tgt_xy) < 50:
        return initial_T, {"success": False, "fitness": 0.0, "rmse": np.inf, "num_corr": 0,
                           "reason": "目标地图可用线特征太少"}

    target_tree = SimpleKDTree2D(tgt_xy)
    x, y, yaw = params_from_T_2d(initial_T)

    last_rmse = None
    final_num_corr = 0
    final_fitness = 0.0
    final_rmse = np.inf

    for it in range(max_iter):
        src_trans = transform_points_2d(src_xy, x, y, yaw)
        dist, idx = target_tree.query(src_trans, k=1, distance_upper_bound=max_corr_dist)
        idx, dist = np.asarray(idx), np.asarray(dist)

        valid = np.isfinite(dist) & (idx >= 0) & (idx < len(tgt_xy))
        if np.sum(valid) < 20:
            break

        src_valid = src_xy[valid]
        src_trans_valid = src_trans[valid]
        idx_valid = idx[valid]

        q = tgt_xy[idx_valid]
        n = tgt_normals[idx_valid]
        diff = src_trans_valid - q
        r = np.sum(n * diff, axis=1)
        abs_r = np.abs(r)

        if 0.0 < trim_ratio < 1.0 and len(abs_r) > 30:
            kth = np.percentile(abs_r, trim_ratio * 100.0)
            keep = abs_r <= kth
        else:
            keep = np.ones_like(abs_r, dtype=bool)

        if np.sum(keep) < 20:
            break

        src_k = src_valid[keep]
        n_k = n[keep]
        r_k = r[keep]

        px, py = src_k[:, 0], src_k[:, 1]
        c, s = np.cos(yaw), np.sin(yaw)
        dpx_dtheta = -s * px - c * py
        dpy_dtheta = c * px - s * py

        J = np.zeros((len(r_k), 3), dtype=np.float64)
        J[:, 0] = n_k[:, 0]
        J[:, 1] = n_k[:, 1]
        J[:, 2] = n_k[:, 0] * dpx_dtheta + n_k[:, 1] * dpy_dtheta

        H = J.T @ J
        b = J.T @ r_k
        damping = 1e-6 * np.eye(3)

        try:
            delta = -np.linalg.solve(H + damping, b)
        except np.linalg.LinAlgError:
            delta = -np.linalg.lstsq(H + damping, b, rcond=None)[0]

        max_step_xy = 0.25
        max_step_yaw = np.deg2rad(3.0)
        delta[0] = np.clip(delta[0], -max_step_xy, max_step_xy)
        delta[1] = np.clip(delta[1], -max_step_xy, max_step_xy)
        delta[2] = np.clip(delta[2], -max_step_yaw, max_step_yaw)

        x += delta[0]
        y += delta[1]
        yaw = wrap_angle_rad(yaw + delta[2])

        rmse = float(np.sqrt(np.mean(r_k ** 2)))
        fitness = float(np.sum(valid) / len(src_xy))
        final_num_corr = int(np.sum(keep))
        final_fitness = fitness
        final_rmse = rmse

        if verbose:
            print(f"      iter={it:02d}, corr={final_num_corr}, fitness={fitness:.3f}, rmse={rmse:.4f}")

        if last_rmse is not None:
            if abs(last_rmse - rmse) < 1e-5 and np.linalg.norm(delta[:2]) < 1e-4 and abs(delta[2]) < 1e-5:
                break
        last_rmse = rmse

    T_final = make_T_from_xyyaw(x, y, yaw)
    success = final_num_corr >= 20 and np.isfinite(final_rmse) and final_fitness > 0.0
    return T_final, {"success": success, "fitness": final_fitness, "rmse": final_rmse, "num_corr": final_num_corr,
                     "reason": "ok" if success else "ICP 未收敛"}


# ================= 6) 局部地图裁剪与结果检查 =================

def crop_map_by_predicted_source(map_pcd, map_intensity, source_pcd, T_pred, margin=2.0):
    map_pts = np.asarray(map_pcd.points)
    if len(map_pts) == 0:
        return map_pcd, map_intensity

    src_tmp = copy.deepcopy(source_pcd)
    src_tmp.transform(T_pred)
    src_pts = np.asarray(src_tmp.points)
    if len(src_pts) == 0:
        return map_pcd, map_intensity

    min_xy = np.min(src_pts[:, :2], axis=0) - margin
    max_xy = np.max(src_pts[:, :2], axis=0) + margin

    mask = (
            (map_pts[:, 0] >= min_xy[0]) & (map_pts[:, 0] <= max_xy[0]) &
            (map_pts[:, 1] >= min_xy[1]) & (map_pts[:, 1] <= max_xy[1])
    )

    if np.sum(mask) < 200:
        return map_pcd, map_intensity

    local_pcd = o3d.geometry.PointCloud()
    local_pcd.points = o3d.utility.Vector3dVector(map_pts[mask])
    local_intensity = map_intensity[mask] if map_intensity is not None else None
    return local_pcd, local_intensity


def check_icp_result(T_pred, T_refined, info):
    if not info["success"]:
        return False, f"ICP 失败: {info.get('reason', '')}"
    if info["fitness"] < MIN_ICP_FITNESS:
        return False, f"fitness 太低: {info['fitness']:.3f}"
    if info["rmse"] > MAX_ICP_RMSE:
        return False, f"rmse 太大: {info['rmse']:.3f}"

    T_delta = np.linalg.inv(T_pred) @ T_refined
    dtrans = np.sqrt(T_delta[0, 3] ** 2 + T_delta[1, 3] ** 2)
    dyaw_deg = np.degrees(abs(wrap_angle_rad(yaw_from_T(T_delta))))

    if dtrans > MAX_CORRECTION_TRANS:
        return False, f"ICP 修正平移过大: {dtrans:.3f} m"
    if dyaw_deg > MAX_CORRECTION_YAW_DEG:
        return False, f"ICP 修正角度过大: {dyaw_deg:.3f} deg"

    return True, "ICP 结果可信"


# ================= 7) 文件读取与匹配辅助函数 =================

def find_odom_file(base_path):
    """自动匹配表格文件的后缀名"""
    for ext in ["", ".csv", ".xlsx", ".xls"]:
        path = base_path + ext
        if os.path.exists(path):
            return path
    raise FileNotFoundError(f"未找到里程计文件: {base_path} (尝试了 .csv, .xlsx, .xls)")


def load_odom_data(file_path):
    """读取表格数据"""
    print(f"正在加载里程计数据: {file_path}")
    if file_path.endswith('.xlsx') or file_path.endswith('.xls'):
        df = pd.read_excel(file_path)
    else:
        try:
            df = pd.read_csv(file_path)
        except Exception:
            df = pd.read_csv(file_path, sep=None, engine='python')
    df.columns = [col.strip() for col in df.columns]
    return df


def find_pcd_file(input_dir, table_filename):

    # 1. 精确匹配
    exact_path = os.path.join(input_dir, table_filename)
    if os.path.exists(exact_path):
        return exact_path

    # 2. 模糊匹配
    if "_dist" in table_filename:
        short_name = table_filename.split("_dist")[0] + ".pcd"
        short_path = os.path.join(input_dir, short_name)
        if os.path.exists(short_path):
            return short_path

    # 3. 前缀匹配
    prefix = table_filename.split("_dist")[0] if "_dist" in table_filename else os.path.splitext(table_filename)[0]
    for f in os.listdir(input_dir):
        if f.lower().startswith(prefix.lower()) and f.lower().endswith('.pcd'):
            return os.path.join(input_dir, f)

    return None


# ================= 8) 主流程 =================

def main():
    # 1. 检查并创建文件夹
    if not os.path.exists(INPUT_DIR):
        print(f"错误: 找不到输入点云文件夹: {INPUT_DIR}")
        return
    if not os.path.exists(FILTER_DIR):
        os.makedirs(FILTER_DIR)
        print(f"已创建滤波输出文件夹: {FILTER_DIR}")

    # 2. 读取里程计表格
    try:
        odom_file_path = find_odom_file(ODOM_FILE_BASE)
        df_odom = load_odom_data(odom_file_path)
    except Exception as e:
        print(f"读取里程计表格失败: {e}")
        return

    # 检查必要列
    required_cols = ['position', 'pose_x', 'pose_y', 'pose_theta_deg', 'pointcloud_file']
    for col in required_cols:
        if col not in df_odom.columns:
            print(f"错误: 表格中缺少必要的列: {col}")
            print(f"当前表格列名: {list(df_odom.columns)}")
            return

    # 按 position 升序排序，确保拼接顺序正确
    df_odom = df_odom.sort_values(by='position').reset_index(drop=True)
    total_frames = len(df_odom)
    print(f"共找到 {total_frames} 帧里程计记录。开始处理...\n")

    # ================= 第一阶段：批量滤波 =================
    print("=== 阶段 1: 批量点云滤波 ===")
    valid_tasks = []

    for idx, row in df_odom.iterrows():
        pos_id = int(row['position'])
        table_file_name = row['pointcloud_file']

        # 寻找原始点云文件
        raw_pcd_path = find_pcd_file(INPUT_DIR, table_file_name)
        if not raw_pcd_path:
            print(f"[{idx + 1}/{total_frames}] 警告: 未找到对应的点云文件: {table_file_name}，跳过该帧。")
            continue

        filename = os.path.basename(raw_pcd_path)
        filtered_pcd_path = os.path.join(FILTER_DIR, filename)

        print(f"[{idx + 1}/{total_frames}] 正在滤波: {filename} -> {filtered_pcd_path}")

        try:
            # 读取点云
            pcd_ori, intensity_ori = read_pcd_binary_xyzi(raw_pcd_path)

            # 压平到 2D
            pts = np.asarray(pcd_ori.points)
            pts[:, 2] = 0.0
            pcd_ori.points = o3d.utility.Vector3dVector(pts)

            # 基础距离与强度滤波
            pcd_a, inten_a = filter_base_distance_intensity(
                pcd_ori, intensity_ori,
                min_dist=MIN_DIST, max_dist=MAX_DIST,
                min_inner=MIN_INNER, min_outer=MIN_OUTER, max_limit=MAX_LIMIT
            )

            # 同心圆弧鬼影滤除
            pcd_final, inten_final = filter_concentric_arcs(
                pcd_a, inten_a,
                eps=ARC_EPS, min_points=ARC_MIN_POINTS, max_radius_std=ARC_MAX_RADIUS_STD
            )

            # 保存滤波后的点云
            save_pcd_binary_xyzi(pcd_final, inten_final, filtered_pcd_path)

            # 记录有效任务信息
            valid_tasks.append({
                'id': pos_id,
                'x': float(row['pose_x']),
                'y': float(row['pose_y']),
                'theta': float(row['pose_theta_deg']),
                'pcd_path': filtered_pcd_path
            })

        except Exception as e:
            print(f"    处理文件 {filename} 时出错: {e}")

    if not valid_tasks:
        print("没有成功滤波任何点云，程序终止。")
        return

    # ================= 第二阶段：点云拼接 =================
    print("\n=== 阶段 2: 点云配准与拼接 ===")

    # 初始化第一帧
    first_task = valid_tasks[0]
    first_id = first_task['id']
    print(f"初始化第一帧 (ID: {first_id}): {os.path.basename(first_task['pcd_path'])}")

    first_pcd, first_intensity = read_pcd_binary_xyzi(first_task['pcd_path'])
    first_pcd, first_intensity = sor_filter_with_intensity(first_pcd, first_intensity)
    first_pcd = force_to_2d(first_pcd)

    # 全局拼接地图初始化
    merged_pcd = copy.deepcopy(first_pcd)
    merged_intensity = copy.deepcopy(first_intensity)

    # 状态变量
    prev_global_T = np.eye(4, dtype=np.float64)
    prev_odom = (first_task['x'], first_task['y'], first_task['theta'])

    for i in range(1, len(valid_tasks)):
        curr_task = valid_tasks[i]
        curr_id = curr_task['id']
        curr_pcd_path = curr_task['pcd_path']

        print(f"\n[{i + 1}/{len(valid_tasks)}] 正在拼接帧 ID: {curr_id} ({os.path.basename(curr_pcd_path)})")

        curr_pcd, curr_intensity = read_pcd_binary_xyzi(curr_pcd_path)
        curr_pcd, curr_intensity = sor_filter_with_intensity(curr_pcd, curr_intensity)
        curr_pcd = force_to_2d(curr_pcd)

        # 计算里程计相对增量
        x_prev, y_prev, theta_prev = prev_odom
        x_curr, y_curr, theta_curr = curr_task['x'], curr_task['y'], curr_task['theta']

        T_odom_prev = get_transform_matrix_from_odom(x_prev, y_prev, theta_prev)
        T_odom_curr = get_transform_matrix_from_odom(x_curr, y_curr, theta_curr)
        T_odom_rel = np.linalg.inv(T_odom_prev) @ T_odom_curr

        # 预测当前帧在地图坐标系下的位姿
        T_pred = prev_global_T @ T_odom_rel

        # 准备 ICP 数据 (Source)
        source_icp_pcd = copy.deepcopy(curr_pcd)
        source_icp_intensity = copy.deepcopy(curr_intensity)
        if USE_INTENSITY_FILTER_FOR_ICP:
            source_icp_pcd, source_icp_intensity = filter_by_intensity_percentile(
                source_icp_pcd, source_icp_intensity, percentile=ICP_INTENSITY_PERCENTILE
            )
        source_down, _ = voxel_downsample_with_intensity(source_icp_pcd, source_icp_intensity, VOXEL_SIZE_ICP)
        source_down = force_to_2d(source_down)

        # 准备 ICP 局部地图数据 (Target)
        local_map_pcd, local_map_intensity = crop_map_by_predicted_source(
            merged_pcd, merged_intensity, source_down, T_pred, margin=LOCAL_MAP_MARGIN
        )
        target_icp_pcd = copy.deepcopy(local_map_pcd)
        target_icp_intensity = copy.deepcopy(local_map_intensity)
        if USE_INTENSITY_FILTER_FOR_ICP:
            target_icp_pcd, target_icp_intensity = filter_by_intensity_percentile(
                target_icp_pcd, target_icp_intensity, percentile=ICP_INTENSITY_PERCENTILE
            )
        target_down, _ = voxel_downsample_with_intensity(target_icp_pcd, target_icp_intensity, VOXEL_SIZE_ICP)
        target_down = force_to_2d(target_down)

        # 执行 ICP 配准
        if curr_id in FORCE_ODOM_ONLY_IDS:
            print(f"   帧 {curr_id} 设置为强制只用 odom，跳过 ICP")
            T_final = T_pred.copy()
        else:
            T_icp, info = execute_2d_point_to_line_icp(
                source_pcd=source_down,
                target_map_pcd=target_down,
                initial_T=T_pred,
                max_corr_dist=POINT_TO_LINE_MAX_CORR_DIST,
                normal_radius=TARGET_NORMAL_RADIUS,
                normal_min_nn=TARGET_NORMAL_MIN_NN,
                normal_min_linearity=TARGET_LINE_MIN_LINEARITY,
                max_iter=ICP_MAX_ITER,
                trim_ratio=TRIM_RATIO,
                verbose=False
            )

            ok, reason = check_icp_result(T_pred, T_icp, info)
            if ok:
                print(f"   ICP 成功收敛并接受: {reason}")
                T_final = T_icp
            else:
                print(f"   ICP 被拒绝，退回使用 odom 预测: {reason}")
                T_final = T_pred.copy()

        # 将当前帧变换到全局坐标系并拼入地图
        curr_aligned = copy.deepcopy(curr_pcd)
        curr_aligned.transform(T_final)

        merged_pcd += curr_aligned
        merged_intensity = np.concatenate([merged_intensity, curr_intensity], axis=0)

        # 更新上一帧状态
        prev_global_T = T_final.copy()
        prev_odom = (x_curr, y_curr, theta_curr)

    # ================= 第三阶段：降采样与保存 =================
    print("\n=== 阶段 3: 全局地图后处理与保存 ===")
    print("全局地图体素内均值化降采样...")
    final_pcd, final_intensity = voxel_downsample_with_intensity(
        merged_pcd, merged_intensity, VOXEL_SIZE_FINAL
    )
    final_pcd = force_to_2d(final_pcd)

    print(f"正在保存最终拼接点云至: {OUTPUT_PCD} (总点数: {len(final_pcd.points)})")
    save_pcd_binary_xyzi(final_pcd, final_intensity, OUTPUT_PCD)

    print("\n 所有流程处理完毕！")


def _parse_and_run(argv):
    """命令行入口: <session_dir> | --latest | (空=默认相对路径=原脚本行为)。"""
    args = [a for a in argv[1:] if a]
    session_dir = None
    if args:
        if args[0] in ("--latest", "-l"):
            session_dir = find_latest_session()
            if session_dir is None:
                print("[sonar_stitch] no session under ~/sonar_logs/scan_*, abort")
                return 1
        else:
            session_dir = args[0]
    if session_dir is not None:
        session_dir = _resolve_paths(session_dir)
        print("[sonar_stitch %s] session_dir = %s" % (STITCH_VERSION, session_dir))
    main()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_parse_and_run(sys.argv))