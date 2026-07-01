import open3d as o3d
import numpy as np
import struct
import argparse
import copy
import heapq
import os
from collections import defaultdict


# ============================================================
# 0. 参数
# ============================================================

# 地图 likelihood field 栅格分辨率，建议 0.05~0.15 m
GRID_RESOLUTION = 0.05

# likelihood field 高斯 sigma，越大越宽容
LIKELIHOOD_SIGMA = 0.25

# 地图边界外扩，防止局部点云落在边界外
MAP_MARGIN = 3.0

# BBS 角度搜索分辨率，初版建议 2~5 deg
BBS_THETA_STEP_DEG = 3.0

# BBS 平移搜索最粗层级
# level=4 表示最粗平移步长 = GRID_RESOLUTION * 2^4
BBS_MAX_LEVEL = 4

# BBS 每个角度最多展开节点数，防止过慢
BBS_MAX_EXPANSIONS_PER_THETA = 4000

# BBS 最终保留多少个候选
BBS_TOP_K = 8

# BBS 参与匹配的局部点最大数量
BBS_MAX_SOURCE_POINTS = 800

# 是否用 intensity 过滤 BBS / ICP 点
USE_INTENSITY_FILTER = True

# 保留 intensity 百分位以上的点
INTENSITY_PERCENTILE = 50

# ICP 下采样体素
ICP_VOXEL_SIZE = 0.10

# Point-to-Line ICP 参数
ICP_MAX_ITER = 50
ICP_MAX_CORR_DIST = 0.50
ICP_TRIM_RATIO = 0.80

# 目标地图法向量估计
TARGET_NORMAL_RADIUS = 0.45
TARGET_NORMAL_MIN_NN = 8
TARGET_LINE_MIN_LINEARITY = 0.25

# ICP 质量筛选
MIN_ICP_FITNESS = 0.25 #最低匹配率
MAX_ICP_RMSE = 0.3

# 保存结果文件名
OUT_ALIGNED_LOCAL = "aligned_local_scan.pcd"
OUT_GLOBAL_PLUS_LOCAL = "global_plus_aligned_local.pcd"
OUT_RESULT_TXT = "relocalization_result.txt"


# ============================================================
# 1. PCD 读取与保存
# ============================================================

def read_pcd_binary_xyzi(file_path):
    """
    读取 binary PCD，默认字段为：
        x y z intensity
    且都是 float32。

    如果你的 PCD 是 ascii 或字段不一致，可以改用 read_pcd_general。
    """
    with open(file_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("PCD header 读取失败")
            decoded = line.decode("utf-8", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA"):
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

        data_type = header.get("DATA", [""])[0].lower()
        if data_type != "binary":
            raise ValueError("当前 read_pcd_binary_xyzi 只支持 binary PCD")

        if not all(k in fields for k in ["x", "y", "z"]):
            raise ValueError("PCD 中没有 x y z 字段")

        point_step = sum(s * c for s, c in zip(sizes, counts))
        blob = f.read(points * point_step)

        x_id = fields.index("x")
        y_id = fields.index("y")
        z_id = fields.index("z")
        inten_id = fields.index("intensity") if "intensity" in fields else None

        offsets = []
        off = 0
        for s, c in zip(sizes, counts):
            offsets.append(off)
            off += s * c

        pts = np.empty((points, 3), dtype=np.float64)
        intens = np.zeros((points,), dtype=np.float32)

        for i in range(points):
            base = i * point_step

            x = struct.unpack_from("<f", blob, base + offsets[x_id])[0]
            y = struct.unpack_from("<f", blob, base + offsets[y_id])[0]
            z = struct.unpack_from("<f", blob, base + offsets[z_id])[0]

            if inten_id is not None:
                inten = struct.unpack_from("<f", blob, base + offsets[inten_id])[0]
            else:
                inten = 1.0

            pts[i] = [x, y, z]
            intens[i] = inten

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        return pcd, intens


def read_pcd_general(file_path):
    """
    优先用 binary xyzi 方式读取。
    如果失败，则退回 Open3D 读取。
    """
    try:
        return read_pcd_binary_xyzi(file_path)
    except Exception as e:
        print(f"[WARN] binary xyzi 读取失败，尝试 Open3D 读取: {e}")
        pcd = o3d.io.read_point_cloud(file_path)
        pts = np.asarray(pcd.points)
        intensity = np.ones((len(pts),), dtype=np.float32)
        return pcd, intensity


def save_pcd_binary_xyzi(pcd, intensity, filename):
    pts = np.asarray(pcd.points)
    N = len(pts)

    if intensity is None or len(intensity) != N:
        intensity = np.ones((N,), dtype=np.float32)

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


# ============================================================
# 2. 点云预处理
# ============================================================

def force_to_2d(pcd):
    pts = np.asarray(pcd.points).copy()
    if len(pts) > 0:
        pts[:, 2] = 0.0
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def sor_filter_with_intensity(pcd, intensity, nb_neighbors=30, std_ratio=1.5):
    if len(pcd.points) < nb_neighbors:
        return pcd, intensity

    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )

    pcd2 = pcd.select_by_index(ind)

    if intensity is not None and len(intensity) == len(pcd.points):
        intensity2 = intensity[ind]
    else:
        intensity2 = None

    return pcd2, intensity2


def filter_by_intensity_percentile(pcd, intensity, percentile=50):
    if intensity is None or len(intensity) != len(pcd.points) or len(intensity) == 0:
        return pcd, intensity

    pts = np.asarray(pcd.points)
    th = np.percentile(intensity, percentile)
    mask = intensity >= th

    if np.sum(mask) < 30:
        return pcd, intensity

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pts[mask])

    return pcd2, intensity[mask]


def voxel_downsample_with_intensity(pcd, intensity, voxel_size):
    pts = np.asarray(pcd.points)

    if len(pts) == 0:
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
        return pcd2, np.empty((0,), dtype=np.float32)

    keys = np.floor(pts / voxel_size).astype(np.int64)

    bucket_pts = defaultdict(list)
    bucket_int = defaultdict(list)

    for i, k in enumerate(keys):
        kt = tuple(k)
        bucket_pts[kt].append(pts[i])
        if intensity is not None and len(intensity) == len(pts):
            bucket_int[kt].append(float(intensity[i]))
        else:
            bucket_int[kt].append(1.0)

    new_pts = np.empty((len(bucket_pts), 3), dtype=np.float64)
    new_int = np.empty((len(bucket_pts),), dtype=np.float32)

    for j, k in enumerate(bucket_pts.keys()):
        new_pts[j] = np.mean(np.asarray(bucket_pts[k]), axis=0)
        new_int[j] = np.float32(np.mean(bucket_int[k]))

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(new_pts)

    return pcd2, new_int


def random_sample_points_xy(points_xy, max_points):
    if len(points_xy) <= max_points:
        return points_xy

    idx = np.random.choice(len(points_xy), max_points, replace=False)
    return points_xy[idx]


def make_pcd_from_xy(points_xy):
    pts3 = np.zeros((len(points_xy), 3), dtype=np.float64)
    pts3[:, :2] = points_xy
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts3)
    return pcd


# ============================================================
# 3. 2D 位姿工具
# ============================================================

def wrap_angle_rad(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def yaw_from_T(T):
    return np.arctan2(T[1, 0], T[0, 0])


def make_T_from_xyyaw(x, y, yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)

    T = np.eye(4, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 3] = x
    T[1, 3] = y

    return T


def params_from_T_2d(T):
    return T[0, 3], T[1, 3], yaw_from_T(T)


def transform_points_2d(pts_xy, x, y, yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)

    out = np.empty_like(pts_xy)
    out[:, 0] = c * pts_xy[:, 0] - s * pts_xy[:, 1] + x
    out[:, 1] = s * pts_xy[:, 0] + c * pts_xy[:, 1] + y

    return out


# ============================================================
# 4. 构建 likelihood field
# ============================================================

def build_likelihood_field(global_pcd,
                           resolution=0.10,
                           sigma=0.25,
                           margin=3.0):
    """
    从全局地图点云构建 2D likelihood field。

    返回：
        likelihood: H x W
        origin_xy: 地图左下角坐标
        resolution
    """
    try:
        from scipy.ndimage import distance_transform_edt
    except Exception:
        raise ImportError(
            "需要 scipy.ndimage.distance_transform_edt。\n"
            "请先安装：pip install scipy"
        )

    pts = np.asarray(global_pcd.points)
    pts_xy = pts[:, :2]

    min_xy = np.min(pts_xy, axis=0) - margin
    max_xy = np.max(pts_xy, axis=0) + margin

    size_xy = max_xy - min_xy
    W = int(np.ceil(size_xy[0] / resolution)) + 1
    H = int(np.ceil(size_xy[1] / resolution)) + 1

    occ = np.zeros((H, W), dtype=bool)

    ij = np.floor((pts_xy - min_xy) / resolution).astype(np.int64)
    valid = (
        (ij[:, 0] >= 0) & (ij[:, 0] < W) &
        (ij[:, 1] >= 0) & (ij[:, 1] < H)
    )
    ij = ij[valid]

    # 注意 grid[y, x]
    occ[ij[:, 1], ij[:, 0]] = True

    # distance_transform_edt 输入为非障碍点 True 时，计算到 False 的距离
    # 这里 ~occ 表示空白区域，距离到 occ=True 的最近地图点
    dist_cells = distance_transform_edt(~occ)
    dist_m = dist_cells * resolution

    likelihood = np.exp(-(dist_m ** 2) / (2.0 * sigma * sigma))
    likelihood = likelihood.astype(np.float32)

    print("[Likelihood Field]")
    print(f"  grid size: W={W}, H={H}")
    print(f"  origin: x={min_xy[0]:.3f}, y={min_xy[1]:.3f}")
    print(f"  resolution: {resolution:.3f} m")
    print(f"  sigma: {sigma:.3f} m")

    return likelihood, min_xy, resolution


def max_pool_2x2(grid):
    H, W = grid.shape

    H2 = (H + 1) // 2
    W2 = (W + 1) // 2

    out = np.zeros((H2, W2), dtype=grid.dtype)

    for y in range(H2):
        for x in range(W2):
            block = grid[y * 2:min((y + 1) * 2, H),
                         x * 2:min((x + 1) * 2, W)]
            out[y, x] = np.max(block)

    return out


def build_likelihood_pyramid(likelihood, max_level):
    """
    pyramid[0] 是最细层。
    pyramid[level] 是 max-pooling 后的粗层。
    """
    pyramid = [likelihood]

    cur = likelihood
    for _ in range(max_level):
        cur = max_pool_2x2(cur)
        pyramid.append(cur)

    return pyramid


def score_points_on_grid(points_xy, grid, origin_xy, resolution):
    """
    在某一层 grid 上计算点的 likelihood 总分。
    """
    H, W = grid.shape

    ij = np.floor((points_xy - origin_xy) / resolution).astype(np.int64)

    valid = (
        (ij[:, 0] >= 0) & (ij[:, 0] < W) &
        (ij[:, 1] >= 0) & (ij[:, 1] < H)
    )

    if np.sum(valid) == 0:
        return 0.0, 0

    vals = grid[ij[valid, 1], ij[valid, 0]]

    return float(np.sum(vals)), int(np.sum(valid))


# ============================================================
# 5. BBS-like 全局粗搜索
# ============================================================

def bbs_search_single_theta(src_xy_rot,
                            pyramid,
                            origin_xy,
                            base_resolution,
                            theta,
                            max_level,
                            max_expansions,
                            top_k):
    """
    对固定 theta 做 BBS-like 平移搜索。

    说明：
        这是工程版 BBS-like 实现：
        - 用 likelihood pyramid 粗到细搜索；
        - 每个粗节点用粗层 likelihood 估计上界；
        - 优先展开高分节点；
        - 到最细层后得到具体 x,y。

    对你的第一版验证已经够用。
    """

    finest = pyramid[0]
    H0, W0 = finest.shape

    # 最粗层尺寸
    coarse = pyramid[max_level]
    Hc, Wc = coarse.shape

    # 节点定义：
    #   level: 当前层级，level 越大越粗
    #   ix, iy: 当前层格子索引
    #   score: 当前层估计分
    #
    # 该节点对应的平移：
    #   x = origin_x + ix * res_level
    #   y = origin_y + iy * res_level

    heap = []

    level = max_level
    res_level = base_resolution * (2 ** level)
    grid = pyramid[level]

    # 初始化最粗层所有平移节点
    for iy in range(Hc):
        for ix in range(Wc):
            tx = origin_xy[0] + ix * res_level
            ty = origin_xy[1] + iy * res_level

            pts_trans = src_xy_rot + np.array([tx, ty])
            score, valid = score_points_on_grid(
                pts_trans,
                grid,
                origin_xy,
                res_level
            )

            if valid < 5:
                continue

            # heapq 是最小堆，所以用负分数
            heapq.heappush(
                heap,
                (-score, level, ix, iy)
            )

    best_leaves = []
    expansions = 0

    while heap and expansions < max_expansions:
        neg_score, level, ix, iy = heapq.heappop(heap)
        score = -neg_score
        expansions += 1

        if level == 0:
            tx = origin_xy[0] + ix * base_resolution
            ty = origin_xy[1] + iy * base_resolution

            pts_trans = src_xy_rot + np.array([tx, ty])
            exact_score, valid = score_points_on_grid(
                pts_trans,
                pyramid[0],
                origin_xy,
                base_resolution
            )

            if valid < 5:
                continue

            mean_score = exact_score / max(valid, 1)

            best_leaves.append({
                "x": tx,
                "y": ty,
                "theta": theta,
                "score": exact_score,
                "mean_score": mean_score,
                "valid": valid
            })

            best_leaves.sort(key=lambda d: d["score"], reverse=True)
            if len(best_leaves) > top_k:
                best_leaves = best_leaves[:top_k]

            continue

        # 分裂到下一层
        child_level = level - 1
        child_grid = pyramid[child_level]
        child_res = base_resolution * (2 ** child_level)

        for dy in [0, 1]:
            for dx in [0, 1]:
                cix = ix * 2 + dx
                ciy = iy * 2 + dy

                H_child, W_child = child_grid.shape
                if cix < 0 or cix >= W_child or ciy < 0 or ciy >= H_child:
                    continue

                tx = origin_xy[0] + cix * child_res
                ty = origin_xy[1] + ciy * child_res

                pts_trans = src_xy_rot + np.array([tx, ty])
                child_score, valid = score_points_on_grid(
                    pts_trans,
                    child_grid,
                    origin_xy,
                    child_res
                )

                if valid < 5:
                    continue

                heapq.heappush(
                    heap,
                    (-child_score, child_level, cix, ciy)
                )

    return best_leaves


def bbs_global_search(src_xy,
                      pyramid,
                      origin_xy,
                      base_resolution,
                      theta_step_deg=3.0,
                      max_level=4,
                      max_expansions_per_theta=4000,
                      top_k=10):
    """
    全局搜索 x, y, theta。

    返回 Top-K 候选。
    """
    candidates = []

    theta_list_deg = np.arange(-180.0, 180.0, theta_step_deg)

    print("\n[BBS-like Global Search]")
    print(f"  theta step: {theta_step_deg:.2f} deg")
    print(f"  theta num: {len(theta_list_deg)}")
    print(f"  max_level: {max_level}")
    print(f"  top_k: {top_k}")

    for ti, theta_deg in enumerate(theta_list_deg):
        theta = np.deg2rad(theta_deg)

        c = np.cos(theta)
        s = np.sin(theta)

        src_rot = np.empty_like(src_xy)
        src_rot[:, 0] = c * src_xy[:, 0] - s * src_xy[:, 1]
        src_rot[:, 1] = s * src_xy[:, 0] + c * src_xy[:, 1]

        leaves = bbs_search_single_theta(
            src_xy_rot=src_rot,
            pyramid=pyramid,
            origin_xy=origin_xy,
            base_resolution=base_resolution,
            theta=theta,
            max_level=max_level,
            max_expansions=max_expansions_per_theta,
            top_k=top_k
        )

        candidates.extend(leaves)
        candidates.sort(key=lambda d: d["score"], reverse=True)
        if len(candidates) > top_k:
            candidates = candidates[:top_k]

        if (ti + 1) % 10 == 0 or ti == len(theta_list_deg) - 1:
            if candidates:
                best = candidates[0]
                print(
                    f"  theta {ti + 1:03d}/{len(theta_list_deg)} done, "
                    f"best score={best['score']:.2f}, "
                    f"x={best['x']:.2f}, y={best['y']:.2f}, "
                    f"theta={np.degrees(best['theta']):.1f}"
                )

    print("\n[BBS Top Candidates]")
    for i, candi in enumerate(candidates):
        print(
            f"  #{i + 1}: "
            f"score={candi['score']:.3f}, "
            f"mean={candi['mean_score']:.3f}, "
            f"valid={candi['valid']}, "
            f"x={candi['x']:.3f}, "
            f"y={candi['y']:.3f}, "
            f"theta={np.degrees(candi['theta']):.3f} deg"
        )

    return candidates


# ============================================================
# 6. 2D KDTree
# ============================================================

class SimpleKDTree2D:
    def __init__(self, points_xy):
        self.points = np.asarray(points_xy, dtype=np.float64)
        self.use_scipy = False
        self.tree = None

        try:
            from scipy.spatial import cKDTree
            self.tree = cKDTree(self.points)
            self.use_scipy = True
        except Exception:
            self.tree = None
            self.use_scipy = False
            print("[WARN] 未检测到 scipy.spatial.cKDTree，将使用 numpy 暴力搜索。")

    def query(self, query_xy, k=1, distance_upper_bound=np.inf):
        query_xy = np.asarray(query_xy, dtype=np.float64)

        if self.use_scipy:
            dist, idx = self.tree.query(
                query_xy,
                k=k,
                distance_upper_bound=distance_upper_bound
            )
            return dist, idx

        if k != 1:
            raise NotImplementedError("numpy fallback 只支持 k=1")

        dist = np.empty((len(query_xy),), dtype=np.float64)
        idx = np.empty((len(query_xy),), dtype=np.int64)

        for i, q in enumerate(query_xy):
            d2 = np.sum((self.points - q) ** 2, axis=1)
            j = int(np.argmin(d2))
            d = np.sqrt(d2[j])

            if d <= distance_upper_bound:
                dist[i] = d
                idx[i] = j
            else:
                dist[i] = np.inf
                idx[i] = len(self.points)

        return dist, idx

    def query_radius_indices(self, q, radius):
        q = np.asarray(q, dtype=np.float64)

        if self.use_scipy:
            return self.tree.query_ball_point(q, radius)

        d2 = np.sum((self.points - q) ** 2, axis=1)
        return np.where(d2 <= radius * radius)[0].tolist()


# ============================================================
# 7. 地图 2D 法向量估计
# ============================================================

def estimate_2d_line_normals(points_xy,
                             radius=0.45,
                             min_nn=8,
                             min_linearity=0.25):
    points_xy = np.asarray(points_xy, dtype=np.float64)

    if len(points_xy) == 0:
        return (
            np.empty((0, 2), dtype=np.float64),
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64)
        )

    tree = SimpleKDTree2D(points_xy)

    valid_pts = []
    normals = []
    linearities = []

    for p in points_xy:
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

        l0 = eigvals[0]
        l1 = eigvals[1]

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
        return (
            np.empty((0, 2), dtype=np.float64),
            np.empty((0, 2), dtype=np.float64),
            np.empty((0,), dtype=np.float64)
        )

    return (
        np.asarray(valid_pts, dtype=np.float64),
        np.asarray(normals, dtype=np.float64),
        np.asarray(linearities, dtype=np.float64)
    )


# ============================================================
# 8. 2D Point-to-Line ICP
# ============================================================

def execute_2d_point_to_line_icp(source_pcd,
                                 target_map_pcd,
                                 initial_T,
                                 max_corr_dist=0.60,
                                 normal_radius=0.45,
                                 normal_min_nn=8,
                                 normal_min_linearity=0.25,
                                 max_iter=50,
                                 trim_ratio=0.80,
                                 verbose=False):
    src_pts_3d = np.asarray(source_pcd.points)
    tgt_pts_3d = np.asarray(target_map_pcd.points)

    if len(src_pts_3d) < 20 or len(tgt_pts_3d) < 50:
        return initial_T, {
            "success": False,
            "fitness": 0.0,
            "rmse": np.inf,
            "num_corr": 0,
            "reason": "点数不足"
        }

    src_xy = src_pts_3d[:, :2].astype(np.float64)
    tgt_xy_raw = tgt_pts_3d[:, :2].astype(np.float64)

    tgt_xy, tgt_normals, linearity = estimate_2d_line_normals(
        tgt_xy_raw,
        radius=normal_radius,
        min_nn=normal_min_nn,
        min_linearity=normal_min_linearity
    )

    if len(tgt_xy) < 50:
        return initial_T, {
            "success": False,
            "fitness": 0.0,
            "rmse": np.inf,
            "num_corr": 0,
            "reason": "目标地图线特征太少"
        }

    target_tree = SimpleKDTree2D(tgt_xy)

    x, y, yaw = params_from_T_2d(initial_T)

    last_rmse = None
    final_num_corr = 0
    final_fitness = 0.0
    final_rmse = np.inf

    for it in range(max_iter):
        src_trans = transform_points_2d(src_xy, x, y, yaw)

        dist, idx = target_tree.query(
            src_trans,
            k=1,
            distance_upper_bound=max_corr_dist
        )

        idx = np.asarray(idx)
        dist = np.asarray(dist)

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

        px = src_k[:, 0]
        py = src_k[:, 1]

        c = np.cos(yaw)
        s = np.sin(yaw)

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
            print(
                f"      iter={it:02d}, "
                f"corr={final_num_corr}, "
                f"fitness={fitness:.3f}, "
                f"rmse={rmse:.4f}, "
                f"dx={delta[0]:+.4f}, "
                f"dy={delta[1]:+.4f}, "
                f"dyaw={np.degrees(delta[2]):+.3f} deg"
            )

        if last_rmse is not None:
            if (
                abs(last_rmse - rmse) < 1e-5 and
                np.linalg.norm(delta[:2]) < 1e-4 and
                abs(delta[2]) < 1e-5
            ):
                break

        last_rmse = rmse

    T_final = make_T_from_xyyaw(x, y, yaw)

    success = (
        final_num_corr >= 20 and
        np.isfinite(final_rmse) and
        final_fitness > 0.0
    )

    return T_final, {
        "success": success,
        "fitness": final_fitness,
        "rmse": final_rmse,
        "num_corr": final_num_corr,
        "reason": "ok" if success else "ICP 未收敛或对应点不足"
    }


# ============================================================
# 9. ICP 候选筛选
# ============================================================

def evaluate_icp_candidate(bbs_score, icp_info):
    """
    综合评分。
    分数越大越好。

    这里先用简单规则：
        score = BBS平均分 + fitness - rmse惩罚
    """
    if not icp_info["success"]:
        return -1e9

    fitness = icp_info["fitness"]
    rmse = icp_info["rmse"]

    if fitness < MIN_ICP_FITNESS:
        return -1e9

    if rmse > MAX_ICP_RMSE:
        return -1e9

    return 1.0 * bbs_score + 2.0 * fitness - 1.5 * rmse


# ============================================================
# 10. 可视化颜色
# ============================================================

def colorize_pcd(pcd, color):
    colors = np.tile(np.asarray(color, dtype=np.float64), (len(pcd.points), 1))
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# ============================================================
# 11. 多点重定位参数
# ============================================================

# scan_1 使用全局 BBS
# scan_2~scan_5 使用 odom 预测附近局部搜索

# 局部 BBS 搜索窗口，单位 m
LOCAL_SEARCH_XY_RANGE = 4

# 局部 BBS 平移步长，单位 m
LOCAL_SEARCH_XY_STEP = 0.2

# 局部 BBS yaw 搜索范围，单位 deg
LOCAL_SEARCH_YAW_RANGE_DEG = 50.0

# 局部 BBS yaw 搜索步长，单位 deg
LOCAL_SEARCH_YAW_STEP_DEG = 2.0

# odom 预测约束权重，越大越不容易偏离 odom
ODOM_PRIOR_XY_SIGMA = 5.0
ODOM_PRIOR_YAW_SIGMA_DEG = 20.0

# ============================================================
# odom 增量预测参数
# ============================================================

ODOM_YAW_DELTA_SIGN = 1.0

USE_INCREMENTAL_ODOM_PREDICTION = True

# 输出目录
OUT_DIR = "multi_relocalization_output1"

# 所有 pose 输出文件
OUT_ALL_POSES_CSV = "poses_in_map.csv"


# ============================================================
# 12. odom / csv / 位姿组合工具
# ============================================================

def inv_T_2d(T):
    """
    2D SE(2) 齐次矩阵求逆。
    """
    R = T[:2, :2]
    t = T[:2, 3]

    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:2, :2] = R.T
    T_inv[:2, 3] = -R.T @ t

    return T_inv


def compose_T(A, B):
    return A @ B


def relative_T(T_from, T_to):
    """
    返回从 T_from 到 T_to 的相对变换：
        T_rel = inv(T_from) @ T_to
    """
    return inv_T_2d(T_from) @ T_to


def parse_points_index_csv(csv_path):
    """
    读取 points_index.csv。

    兼容两种格式：

    1. 普通 CSV：
        point_id,deliverable_basename,...,x_at_scan,y_at_scan,yaw_at_scan,...

    2. 你发的 markdown 表格形式：
        | point_id | deliverable_basename | ... |
        | - | - | ... |
        | 1 | xxx | ... |

    返回：
        rows: list[dict]
    """

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"找不到 odom csv 文件: {csv_path}")

    with open(csv_path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    # 判断是否是 markdown pipe 表格
    is_markdown_table = any(ln.startswith("|") and ln.endswith("|") for ln in lines)

    rows = []

    if is_markdown_table:
        table_lines = [ln for ln in lines if ln.startswith("|") and ln.endswith("|")]

        if len(table_lines) < 3:
            raise ValueError("markdown 表格行数不足")

        header = [x.strip() for x in table_lines[0].strip("|").split("|")]

        for ln in table_lines[2:]:
            parts = [x.strip() for x in ln.strip("|").split("|")]

            if len(parts) != len(header):
                continue

            item = dict(zip(header, parts))

            # 跳过非法行
            if item.get("point_id", "").lower() in ["-", ""]:
                continue

            rows.append(item)

    else:
        import csv
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for item in reader:
                rows.append(item)

    parsed = []

    for item in rows:
        try:
            point_id = int(str(item["point_id"]).strip())
            x = float(str(item["x_at_scan"]).strip())
            y = float(str(item["y_at_scan"]).strip())
            yaw_deg = float(str(item["yaw_at_scan"]).strip())
        except Exception as e:
            print(f"[WARN] 跳过无法解析的 odom 行: {item}, error={e}")
            continue

        parsed.append({
            "point_id": point_id,
            "x": x,
            "y": y,
            "yaw_deg": yaw_deg,
            "yaw_rad": np.deg2rad(yaw_deg),
            "raw": item
        })

    parsed.sort(key=lambda d: d["point_id"])

    if len(parsed) == 0:
        raise ValueError("odom csv 中没有解析到有效 point 数据")

    print("\n[Load Odom CSV]")
    print(f"  file: {csv_path}")
    print(f"  valid points: {len(parsed)}")

    for r in parsed:
        print(
            f"  point {r['point_id']}: "
            f"x={r['x']:.3f}, y={r['y']:.3f}, yaw={r['yaw_deg']:.3f} deg"
        )

    return parsed


def get_scan_path(scan_dir, scan_pattern, point_id):
    """
    根据 point_id 获取 scan 文件路径。

    默认：
        scan_dir/scan_1.pcd
        scan_dir/scan_2.pcd
        ...
    """
    filename = scan_pattern.format(point_id=point_id)
    return os.path.join(scan_dir, filename)


def odom_row_to_T(row):
    """
    points_index.csv 中的 x_at_scan, y_at_scan, yaw_at_scan 转为 T_odom。
    注意 yaw_at_scan 默认单位为 deg。
    """
    return make_T_from_xyyaw(
        row["x"],
        row["y"],
        row["yaw_rad"]
    )


def predict_pose_from_odom(T_1_map, T_odom_1, T_odom_i):

    T_1_to_i_odom = relative_T(T_odom_1, T_odom_i)
    T_i_map_pred = T_1_map @ T_1_to_i_odom
    return T_i_map_pred


def predict_pose_from_odom_incremental(T_prev_map,
                                       T_odom_prev,
                                       T_odom_i,
                                       yaw_delta_sign=ODOM_YAW_DELTA_SIGN):


    x_prev_map, y_prev_map, yaw_prev_map = params_from_T_2d(T_prev_map)
    x_prev_odom, y_prev_odom, yaw_prev_odom = params_from_T_2d(T_odom_prev)
    x_i_odom, y_i_odom, yaw_i_odom = params_from_T_2d(T_odom_i)

    # ------------------------------------------------------------
    # 1. 计算相邻 odom 增量
    # ------------------------------------------------------------

    dx_odom = x_i_odom - x_prev_odom
    dy_odom = y_i_odom - y_prev_odom
    dyaw_odom = wrap_angle_rad(yaw_i_odom - yaw_prev_odom)

    # ------------------------------------------------------------
    # 2. yaw 增量叠加
    # ------------------------------------------------------------
    # 注意这里使用 yaw_delta_sign。
    # 对你当前数据，默认 yaw_delta_sign=-1.0。
    yaw_pred = wrap_angle_rad(
        yaw_prev_map + yaw_delta_sign * dyaw_odom
    )


    map_from_odom_yaw = wrap_angle_rad(
        yaw_prev_map - yaw_delta_sign * yaw_prev_odom
    )

    c = np.cos(map_from_odom_yaw)
    s = np.sin(map_from_odom_yaw)

    dx_map = c * dx_odom - s * dy_odom
    dy_map = s * dx_odom + c * dy_odom

    x_pred = x_prev_map + dx_map
    y_pred = y_prev_map + dy_map

    T_pred = make_T_from_xyyaw(
        x_pred,
        y_pred,
        yaw_pred
    )

    debug_info = {
        "x_prev_map": x_prev_map,
        "y_prev_map": y_prev_map,
        "yaw_prev_map": yaw_prev_map,

        "x_prev_odom": x_prev_odom,
        "y_prev_odom": y_prev_odom,
        "yaw_prev_odom": yaw_prev_odom,

        "x_i_odom": x_i_odom,
        "y_i_odom": y_i_odom,
        "yaw_i_odom": yaw_i_odom,

        "dx_odom": dx_odom,
        "dy_odom": dy_odom,
        "dyaw_odom": dyaw_odom,

        "map_from_odom_yaw": map_from_odom_yaw,

        "dx_map": dx_map,
        "dy_map": dy_map,

        "x_pred": x_pred,
        "y_pred": y_pred,
        "yaw_pred": yaw_pred,

        "yaw_delta_sign": yaw_delta_sign,
    }

    return T_pred, debug_info

# ============================================================
# 13. 单帧预处理封装
# ============================================================

def preprocess_global_map(global_map_path):
    """
    读取并预处理全局地图。
    返回：
        global_pcd
        global_intensity
        global_match_pcd
        global_match_intensity
        likelihood
        origin_xy
        resolution
        pyramid
        target_icp_pcd
    """

    print(f"\n[Load] global map: {global_map_path}")
    global_pcd, global_intensity = read_pcd_general(global_map_path)
    global_pcd = force_to_2d(global_pcd)

    print(f"  raw global points: {len(global_pcd.points)}")

    print("\n[Preprocess Global] SOR filter")
    global_pcd, global_intensity = sor_filter_with_intensity(
        global_pcd,
        global_intensity,
        nb_neighbors=30,
        std_ratio=1.8
    )
    global_pcd = force_to_2d(global_pcd)

    print(f"  global after SOR: {len(global_pcd.points)}")

    global_match_pcd = copy.deepcopy(global_pcd)
    global_match_intensity = copy.deepcopy(global_intensity)

    if USE_INTENSITY_FILTER:
        print("[Preprocess Global] intensity filter")
        global_match_pcd, global_match_intensity = filter_by_intensity_percentile(
            global_match_pcd,
            global_match_intensity,
            percentile=INTENSITY_PERCENTILE
        )

    print(f"  global match points: {len(global_match_pcd.points)}")

    likelihood, origin_xy, resolution = build_likelihood_field(
        global_match_pcd,
        resolution=GRID_RESOLUTION,
        sigma=LIKELIHOOD_SIGMA,
        margin=MAP_MARGIN
    )

    pyramid = build_likelihood_pyramid(
        likelihood,
        max_level=BBS_MAX_LEVEL
    )

    target_icp_pcd, target_icp_int = voxel_downsample_with_intensity(
        global_match_pcd,
        global_match_intensity,
        voxel_size=ICP_VOXEL_SIZE
    )
    target_icp_pcd = force_to_2d(target_icp_pcd)

    print(f"  ICP target points: {len(target_icp_pcd.points)}")

    return {
        "global_pcd": global_pcd,
        "global_intensity": global_intensity,
        "global_match_pcd": global_match_pcd,
        "global_match_intensity": global_match_intensity,
        "likelihood": likelihood,
        "origin_xy": origin_xy,
        "resolution": resolution,
        "pyramid": pyramid,
        "target_icp_pcd": target_icp_pcd,
    }


def preprocess_local_scan(scan_path):
    """
    读取并预处理单个 scan。
    返回：
        local_pcd
        local_intensity
        local_match_pcd
        local_match_intensity
        source_icp_pcd
        src_xy_bbs
    """

    print(f"\n[Load] local scan: {scan_path}")
    local_pcd, local_intensity = read_pcd_general(scan_path)
    local_pcd = force_to_2d(local_pcd)

    print(f"  raw local points: {len(local_pcd.points)}")

    local_pcd, local_intensity = sor_filter_with_intensity(
        local_pcd,
        local_intensity,
        nb_neighbors=30,
        std_ratio=1.8
    )
    local_pcd = force_to_2d(local_pcd)

    print(f"  local after SOR: {len(local_pcd.points)}")

    local_match_pcd = copy.deepcopy(local_pcd)
    local_match_intensity = copy.deepcopy(local_intensity)

    if USE_INTENSITY_FILTER:
        local_match_pcd, local_match_intensity = filter_by_intensity_percentile(
            local_match_pcd,
            local_match_intensity,
            percentile=INTENSITY_PERCENTILE
        )

    print(f"  local match points: {len(local_match_pcd.points)}")

    local_for_bbs_pcd, local_for_bbs_intensity = voxel_downsample_with_intensity(
        local_match_pcd,
        local_match_intensity,
        voxel_size=GRID_RESOLUTION
    )
    local_for_bbs_pcd = force_to_2d(local_for_bbs_pcd)

    src_xy = np.asarray(local_for_bbs_pcd.points)[:, :2]
    src_xy = random_sample_points_xy(src_xy, BBS_MAX_SOURCE_POINTS)

    source_icp_pcd, source_icp_int = voxel_downsample_with_intensity(
        local_match_pcd,
        local_match_intensity,
        voxel_size=ICP_VOXEL_SIZE
    )
    source_icp_pcd = force_to_2d(source_icp_pcd)

    print(f"  BBS source points: {len(src_xy)}")
    print(f"  ICP source points: {len(source_icp_pcd.points)}")

    return {
        "local_pcd": local_pcd,
        "local_intensity": local_intensity,
        "local_match_pcd": local_match_pcd,
        "local_match_intensity": local_match_intensity,
        "source_icp_pcd": source_icp_pcd,
        "src_xy_bbs": src_xy,
    }


# ============================================================
# 14. 局部 BBS 搜索
# ============================================================

def odom_prior_score(x, y, yaw, x_pred, y_pred, yaw_pred):
    """
    odom 先验分数。
    越接近 odom 预测位姿，分数越高。
    """

    dx = x - x_pred
    dy = y - y_pred
    dxy2 = dx * dx + dy * dy

    dyaw = wrap_angle_rad(yaw - yaw_pred)

    sigma_xy = ODOM_PRIOR_XY_SIGMA
    sigma_yaw = np.deg2rad(ODOM_PRIOR_YAW_SIGMA_DEG)

    score_xy = -0.5 * dxy2 / (sigma_xy * sigma_xy)
    score_yaw = -0.5 * dyaw * dyaw / (sigma_yaw * sigma_yaw)

    return score_xy + score_yaw


def bbs_local_search_around_pose(src_xy,
                                 likelihood,
                                 origin_xy,
                                 resolution,
                                 T_pred,
                                 xy_range=1.5,
                                 xy_step=0.10,
                                 yaw_range_deg=20.0,
                                 yaw_step_deg=2.0,
                                 top_k=10):
    """
    在 odom 预测位姿附近做局部 BBS 搜索。

    不再全地图搜索，而是在：
        x_pred ± xy_range
        y_pred ± xy_range
        yaw_pred ± yaw_range_deg
    内搜索。

    返回 Top-K 候选。
    """

    x_pred, y_pred, yaw_pred = params_from_T_2d(T_pred)

    x_list = np.arange(x_pred - xy_range, x_pred + xy_range + 1e-9, xy_step)
    y_list = np.arange(y_pred - xy_range, y_pred + xy_range + 1e-9, xy_step)
    yaw_offsets = np.deg2rad(
        np.arange(-yaw_range_deg, yaw_range_deg + 1e-9, yaw_step_deg)
    )

    candidates = []

    print("\n[Local BBS Search]")
    print(f"  pred: x={x_pred:.3f}, y={y_pred:.3f}, yaw={np.degrees(yaw_pred):.3f} deg")
    print(f"  x range: ±{xy_range:.2f} m, step={xy_step:.2f}")
    print(f"  y range: ±{xy_range:.2f} m, step={xy_step:.2f}")
    print(f"  yaw range: ±{yaw_range_deg:.2f} deg, step={yaw_step_deg:.2f} deg")
    print(f"  search size: {len(x_list)} x {len(y_list)} x {len(yaw_offsets)}")

    for dyaw in yaw_offsets:
        yaw = wrap_angle_rad(yaw_pred + dyaw)

        c = np.cos(yaw)
        s = np.sin(yaw)

        src_rot = np.empty_like(src_xy)
        src_rot[:, 0] = c * src_xy[:, 0] - s * src_xy[:, 1]
        src_rot[:, 1] = s * src_xy[:, 0] + c * src_xy[:, 1]

        for x in x_list:
            for y in y_list:
                pts_trans = src_rot + np.array([x, y])

                lf_score, valid = score_points_on_grid(
                    pts_trans,
                    likelihood,
                    origin_xy,
                    resolution
                )

                if valid < 5:
                    continue

                mean_score = lf_score / max(valid, 1)

                prior = odom_prior_score(
                    x, y, yaw,
                    x_pred, y_pred, yaw_pred
                )

                # 综合分数：
                # likelihood 平均分为主，odom prior 防止跑太远
                final_score = mean_score + 0.15 * prior

                cand = {
                    "x": x,
                    "y": y,
                    "theta": yaw,
                    "score": lf_score,
                    "mean_score": mean_score,
                    "prior_score": prior,
                    "local_score": final_score,
                    "valid": valid,
                    "pred_x": x_pred,
                    "pred_y": y_pred,
                    "pred_theta": yaw_pred
                }

                candidates.append(cand)

    candidates.sort(key=lambda d: d["local_score"], reverse=True)
    candidates = candidates[:top_k]

    print("\n[Local BBS Top Candidates]")
    for i, candi in enumerate(candidates):
        print(
            f"  #{i + 1}: "
            f"local_score={candi['local_score']:.4f}, "
            f"mean={candi['mean_score']:.4f}, "
            f"prior={candi['prior_score']:.4f}, "
            f"valid={candi['valid']}, "
            f"x={candi['x']:.3f}, y={candi['y']:.3f}, "
            f"yaw={np.degrees(candi['theta']):.3f} deg"
        )

    return candidates


# ============================================================
# 15. 候选 ICP 统一细配准
# ============================================================

def refine_candidates_by_icp(candidates,
                             source_icp_pcd,
                             target_icp_pcd,
                             use_odom_prior=False):
    """
    对 BBS 候选逐个执行 ICP，返回最佳结果。
    """

    print("\n[Top-K ICP Refinement]")

    best_result = None

    for i, cand in enumerate(candidates):
        T_init = make_T_from_xyyaw(
            cand["x"],
            cand["y"],
            cand["theta"]
        )

        print(
            f"\n  Candidate #{i + 1}: "
            f"BBS mean={cand['mean_score']:.4f}, "
            f"x={cand['x']:.3f}, "
            f"y={cand['y']:.3f}, "
            f"yaw={np.degrees(cand['theta']):.3f} deg"
        )

        T_icp, info = execute_2d_point_to_line_icp(
            source_pcd=source_icp_pcd,
            target_map_pcd=target_icp_pcd,
            initial_T=T_init,
            max_corr_dist=ICP_MAX_CORR_DIST,
            normal_radius=TARGET_NORMAL_RADIUS,
            normal_min_nn=TARGET_NORMAL_MIN_NN,
            normal_min_linearity=TARGET_LINE_MIN_LINEARITY,
            max_iter=ICP_MAX_ITER,
            trim_ratio=ICP_TRIM_RATIO,
            verbose=False
        )

        final_score = evaluate_icp_candidate(
            bbs_score=cand["mean_score"],
            icp_info=info
        )

        # 如果是局部重定位，可以把 odom prior 加入最终评分
        if use_odom_prior and "prior_score" in cand:
            final_score += 0.15 * cand["prior_score"]

        x, y, yaw = params_from_T_2d(T_icp)

        print(
            f"    ICP success={info['success']}, "
            f"fitness={info['fitness']:.3f}, "
            f"rmse={info['rmse']:.4f}, "
            f"corr={info['num_corr']}, "
            f"final_score={final_score:.4f}"
        )

        print(
            f"    refined pose: "
            f"x={x:.3f}, y={y:.3f}, "
            f"yaw={np.degrees(yaw):.3f} deg"
        )

        result = {
            "candidate_id": i + 1,
            "T": T_icp,
            "bbs": cand,
            "icp": info,
            "final_score": final_score
        }

        if best_result is None or final_score > best_result["final_score"]:
            best_result = result

    if best_result is None or best_result["final_score"] < -1e8:
        print("\n[WARN] 所有 ICP 候选均未通过质量门限，退回 BBS Top-1")
        cand = candidates[0]

        T_best = make_T_from_xyyaw(
            cand["x"],
            cand["y"],
            cand["theta"]
        )

        best_result = {
            "candidate_id": 1,
            "T": T_best,
            "bbs": cand,
            "icp": {
                "success": False,
                "fitness": 0.0,
                "rmse": np.inf,
                "num_corr": 0,
                "reason": "fallback to BBS"
            },
            "final_score": cand["mean_score"]
        }

    return best_result


# ============================================================
# 16. scan_1 全局重定位
# ============================================================

def relocalize_first_scan_global(scan_data, map_data):
    """
    scan_1:
        全局 BBS + ICP
    """

    print("\n================================================")
    print("  Relocalize scan_1 by Global BBS + ICP")
    print("================================================")

    src_xy = scan_data["src_xy_bbs"]

    bbs_candidates = bbs_global_search(
        src_xy=src_xy,
        pyramid=map_data["pyramid"],
        origin_xy=map_data["origin_xy"],
        base_resolution=map_data["resolution"],
        theta_step_deg=BBS_THETA_STEP_DEG,
        max_level=BBS_MAX_LEVEL,
        max_expansions_per_theta=BBS_MAX_EXPANSIONS_PER_THETA,
        top_k=BBS_TOP_K
    )

    if not bbs_candidates:
        raise RuntimeError("scan_1 全局 BBS 没有找到任何候选位姿")

    best_result = refine_candidates_by_icp(
        candidates=bbs_candidates,
        source_icp_pcd=scan_data["source_icp_pcd"],
        target_icp_pcd=map_data["target_icp_pcd"],
        use_odom_prior=False
    )

    return best_result


# ============================================================
# 17. scan_i 局部重定位
# ============================================================

def relocalize_scan_local(scan_data, map_data, T_pred):
    """
    scan_i:
        odom 预测位姿附近局部 BBS + ICP
    """

    print("\n================================================")
    print("  Relocalize scan_i by Odom Prediction + Local BBS + ICP")
    print("================================================")

    src_xy = scan_data["src_xy_bbs"]

    local_candidates = bbs_local_search_around_pose(
        src_xy=src_xy,
        likelihood=map_data["likelihood"],
        origin_xy=map_data["origin_xy"],
        resolution=map_data["resolution"],
        T_pred=T_pred,
        xy_range=LOCAL_SEARCH_XY_RANGE,
        xy_step=LOCAL_SEARCH_XY_STEP,
        yaw_range_deg=LOCAL_SEARCH_YAW_RANGE_DEG,
        yaw_step_deg=LOCAL_SEARCH_YAW_STEP_DEG,
        top_k=BBS_TOP_K
    )

    if not local_candidates:
        print("[WARN] 局部 BBS 没有候选，直接使用 odom 预测位姿")
        x, y, yaw = params_from_T_2d(T_pred)

        local_candidates = [{
            "x": x,
            "y": y,
            "theta": yaw,
            "score": 0.0,
            "mean_score": 0.0,
            "prior_score": 0.0,
            "local_score": 0.0,
            "valid": 0
        }]

    best_result = refine_candidates_by_icp(
        candidates=local_candidates,
        source_icp_pcd=scan_data["source_icp_pcd"],
        target_icp_pcd=map_data["target_icp_pcd"],
        use_odom_prior=True
    )

    return best_result


# ============================================================
# 18. 结果保存
# ============================================================

def save_one_aligned_scan(point_id,
                          scan_data,
                          T_map,
                          out_dir):
    """
    保存每个 scan 配准到 map 后的点云。
    """

    os.makedirs(out_dir, exist_ok=True)

    aligned = copy.deepcopy(scan_data["local_pcd"])
    aligned.transform(T_map)

    local_intensity = scan_data["local_intensity"]

    out_path = os.path.join(out_dir, f"aligned_scan_{point_id}.pcd")

    save_pcd_binary_xyzi(
        aligned,
        local_intensity,
        out_path
    )

    print(f"[Save] point {point_id} aligned scan: {out_path}")

    return aligned, local_intensity


def save_all_poses_csv(results, out_dir):
    """
    保存所有 pose_i_map。
    """

    os.makedirs(out_dir, exist_ok=True)

    out_csv = os.path.join(out_dir, OUT_ALL_POSES_CSV)

    with open(out_csv, "w", encoding="utf-8") as f:
        f.write(
            "point_id,"
            "x_map,y_map,yaw_map_deg,yaw_map_rad,"
            "icp_success,icp_fitness,icp_rmse,icp_corr,"
            "final_score,"
            "pred_x_map,pred_y_map,pred_yaw_map_deg\n"
        )

        for r in results:
            point_id = r["point_id"]
            T = r["T_map"]
            x, y, yaw = params_from_T_2d(T)

            icp = r["result"]["icp"]

            if r["T_pred"] is not None:
                px, py, pyaw = params_from_T_2d(r["T_pred"])
                pyaw_deg = np.degrees(pyaw)
            else:
                px, py, pyaw_deg = np.nan, np.nan, np.nan

            f.write(
                f"{point_id},"
                f"{x:.9f},{y:.9f},{np.degrees(yaw):.9f},{yaw:.9f},"
                f"{int(icp['success'])},{icp['fitness']:.9f},{icp['rmse']:.9f},{icp['num_corr']},"
                f"{r['result']['final_score']:.9f},"
                f"{px:.9f},{py:.9f},{pyaw_deg:.9f}\n"
            )

    print(f"[Save] all poses csv: {out_csv}")


def save_global_plus_all_scans(map_data, aligned_scans, out_dir):
    """
    保存 global map + 所有对齐后的 scan，方便可视化检查。
    """

    os.makedirs(out_dir, exist_ok=True)

    global_pcd = copy.deepcopy(map_data["global_pcd"])
    global_intensity = map_data["global_intensity"]

    merged = copy.deepcopy(global_pcd)

    intensity_list = []

    if global_intensity is not None and len(global_intensity) == len(global_pcd.points):
        intensity_list.append(global_intensity)
    else:
        intensity_list.append(
            np.ones((len(global_pcd.points),), dtype=np.float32)
        )

    for aligned, inten in aligned_scans:
        merged += aligned

        if inten is not None and len(inten) == len(aligned.points):
            intensity_list.append(inten)
        else:
            intensity_list.append(
                np.ones((len(aligned.points),), dtype=np.float32)
            )

    merged_intensity = np.concatenate(intensity_list)

    out_path = os.path.join(out_dir, "global_plus_all_aligned_scans.pcd")

    save_pcd_binary_xyzi(
        merged,
        merged_intensity,
        out_path
    )

    print(f"[Save] global + all aligned scans: {out_path}")


# ============================================================
# 19. 主流程：初始全局定位 + odom预测 + 局部约束修正
# ============================================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--global_map",
        default="global_map.pcd",
        help="全局地图 PCD 文件"
    )

    parser.add_argument(
        "--scan_dir",
        default="z-maps/xiugai",
        help="scan_1.pcd ~ scan_5.pcd 所在目录"
    )

    parser.add_argument(
        "--scan_pattern",
        default="scan_{point_id}.pcd",
        help="scan 文件命名格式，例如 scan_{point_id}.pcd"
    )

    parser.add_argument(
        "--odom_csv",
        default="points_index-1.csv",
        help="包含 point_id, x_at_scan, y_at_scan, yaw_at_scan 的 csv"
    )

    parser.add_argument(
        "--out_dir",
        default=OUT_DIR,
        help="输出目录"
    )

    parser.add_argument(
        "--no_vis",
        action="store_true",
        help="不显示 Open3D 可视化"
    )

    args = parser.parse_args()

    np.random.seed(0)

    os.makedirs(args.out_dir, exist_ok=True)

    print("\n====================================================")
    print("  Multi-Point Relocalization")
    print("  scan_1: Global BBS + ICP")
    print("  scan_i: Incremental Odom Prediction + Local BBS + ICP")
    print("====================================================")

    print("\n[Config]")
    print(f"  USE_INCREMENTAL_ODOM_PREDICTION = {USE_INCREMENTAL_ODOM_PREDICTION}")
    print(f"  ODOM_YAW_DELTA_SIGN             = {ODOM_YAW_DELTA_SIGN:+.1f}")
    print(f"  LOCAL_SEARCH_XY_RANGE           = {LOCAL_SEARCH_XY_RANGE:.3f} m")
    print(f"  LOCAL_SEARCH_XY_STEP            = {LOCAL_SEARCH_XY_STEP:.3f} m")
    print(f"  LOCAL_SEARCH_YAW_RANGE_DEG      = {LOCAL_SEARCH_YAW_RANGE_DEG:.3f} deg")
    print(f"  LOCAL_SEARCH_YAW_STEP_DEG       = {LOCAL_SEARCH_YAW_STEP_DEG:.3f} deg")

    # ------------------------------------------------------------
    # 1. 读取 odom csv
    # ------------------------------------------------------------

    odom_rows = parse_points_index_csv(args.odom_csv)

    # ------------------------------------------------------------
    # 2. 读取和预处理全局地图
    # ------------------------------------------------------------

    map_data = preprocess_global_map(args.global_map)

    # ------------------------------------------------------------
    # 3. 逐点重定位
    # ------------------------------------------------------------

    results = []
    aligned_scans = []

    # 第一帧相关
    T_1_map = None
    T_odom_1 = None

    # 相邻增量预测需要记录上一帧的配准结果和上一帧 odom
    T_prev_map = None
    T_odom_prev = None
    prev_point_id = None

    for idx, row in enumerate(odom_rows):
        point_id = row["point_id"]

        print("\n\n####################################################")
        print(f"  Processing point_id = {point_id}")
        print("####################################################")

        scan_path = get_scan_path(
            args.scan_dir,
            args.scan_pattern,
            point_id
        )

        if not os.path.exists(scan_path):
            print(f"[WARN] 找不到 scan 文件，跳过: {scan_path}")
            continue

        scan_data = preprocess_local_scan(scan_path)

        T_odom_i = odom_row_to_T(row)

        if idx == 0:
            # ----------------------------------------------------
            # 第一个点：全局 BBS + ICP
            # ----------------------------------------------------

            result = relocalize_first_scan_global(
                scan_data=scan_data,
                map_data=map_data
            )

            T_i_map = result["T"]

            # 记录第一帧，用于兼容旧逻辑
            T_1_map = T_i_map
            T_odom_1 = T_odom_i

            # 记录上一帧，用于相邻 odom 增量预测
            T_prev_map = T_i_map
            T_odom_prev = T_odom_i
            prev_point_id = point_id

            T_pred = None

        else:
            # ----------------------------------------------------
            # 后续点：由相邻 odom 增量预测，再局部 BBS + ICP 修正
            # ----------------------------------------------------

            if USE_INCREMENTAL_ODOM_PREDICTION:
                if T_prev_map is None or T_odom_prev is None:
                    raise RuntimeError("上一帧尚未成功定位，无法使用相邻 odom 增量预测当前点")

                T_pred, odom_debug = predict_pose_from_odom_incremental(
                    T_prev_map=T_prev_map,
                    T_odom_prev=T_odom_prev,
                    T_odom_i=T_odom_i,
                    yaw_delta_sign=ODOM_YAW_DELTA_SIGN
                )

                px, py, pyaw = params_from_T_2d(T_pred)

                print("\n[Incremental Odom Prediction]")
                print(f"  previous point_id: {prev_point_id}")
                print(f"  current  point_id: {point_id}")

                print("  odom previous:")
                print(f"    x      = {odom_debug['x_prev_odom']:.6f} m")
                print(f"    y      = {odom_debug['y_prev_odom']:.6f} m")
                print(f"    yaw    = {np.degrees(odom_debug['yaw_prev_odom']):.6f} deg")

                print("  odom current:")
                print(f"    x      = {odom_debug['x_i_odom']:.6f} m")
                print(f"    y      = {odom_debug['y_i_odom']:.6f} m")
                print(f"    yaw    = {np.degrees(odom_debug['yaw_i_odom']):.6f} deg")

                print("  odom delta:")
                print(f"    dx_odom    = {odom_debug['dx_odom']:.6f} m")
                print(f"    dy_odom    = {odom_debug['dy_odom']:.6f} m")
                print(f"    dyaw_odom  = {np.degrees(odom_debug['dyaw_odom']):.6f} deg")
                print(f"    yaw sign   = {odom_debug['yaw_delta_sign']:+.1f}")

                print("  previous map pose:")
                print(f"    x_prev_map   = {odom_debug['x_prev_map']:.6f} m")
                print(f"    y_prev_map   = {odom_debug['y_prev_map']:.6f} m")
                print(f"    yaw_prev_map = {np.degrees(odom_debug['yaw_prev_map']):.6f} deg")

                print("  converted map delta:")
                print(f"    dx_map     = {odom_debug['dx_map']:.6f} m")
                print(f"    dy_map     = {odom_debug['dy_map']:.6f} m")
                print(f"    yaw_offset = {np.degrees(odom_debug['map_from_odom_yaw']):.6f} deg")

                print("  predicted pose in map:")
                print(f"    x_pred     = {px:.6f} m")
                print(f"    y_pred     = {py:.6f} m")
                print(f"    yaw_pred   = {np.degrees(pyaw):.6f} deg")

            else:
                # ------------------------------------------------
                # 旧逻辑：由第一个点推算所有点
                # ------------------------------------------------
                if T_1_map is None or T_odom_1 is None:
                    raise RuntimeError("scan_1 尚未成功定位，无法使用 odom 预测后续点")

                T_pred = predict_pose_from_odom(
                    T_1_map=T_1_map,
                    T_odom_1=T_odom_1,
                    T_odom_i=T_odom_i
                )

                px, py, pyaw = params_from_T_2d(T_pred)

                print("\n[Odom Prediction From First Point - Old Logic]")
                print(
                    f"  point {point_id} predicted pose in map: "
                    f"x={px:.3f}, y={py:.3f}, yaw={np.degrees(pyaw):.3f} deg"
                )

            result = relocalize_scan_local(
                scan_data=scan_data,
                map_data=map_data,
                T_pred=T_pred
            )

            T_i_map = result["T"]

            # ----------------------------------------------------
            # 当前点配准完成后，更新上一帧状态。
            # 下一点将使用当前点的配准结果 + 当前点到下一点的 odom 增量。
            # ----------------------------------------------------
            T_prev_map = T_i_map
            T_odom_prev = T_odom_i
            prev_point_id = point_id

        # --------------------------------------------------------
        # 保存单点结果
        # --------------------------------------------------------

        x, y, yaw = params_from_T_2d(T_i_map)

        print("\n==============================")
        print(f"  Final Pose of Point {point_id}")
        print("==============================")
        print(f"  pose_{point_id}_map:")
        print(f"    x     = {x:.6f} m")
        print(f"    y     = {y:.6f} m")
        print(f"    yaw   = {np.degrees(yaw):.6f} deg")
        print(f"  ICP fitness = {result['icp']['fitness']:.6f}")
        print(f"  ICP rmse    = {result['icp']['rmse']:.6f}")
        print(f"  ICP corr    = {result['icp']['num_corr']}")
        print("==============================")

        aligned, aligned_intensity = save_one_aligned_scan(
            point_id=point_id,
            scan_data=scan_data,
            T_map=T_i_map,
            out_dir=args.out_dir
        )

        aligned_scans.append((aligned, aligned_intensity))

        results.append({
            "point_id": point_id,
            "T_map": T_i_map,
            "T_pred": T_pred,
            "T_odom": T_odom_i,
            "result": result,
            "scan_path": scan_path
        })

    # ------------------------------------------------------------
    # 4. 保存所有结果
    # ------------------------------------------------------------

    save_all_poses_csv(results, args.out_dir)

    save_global_plus_all_scans(
        map_data=map_data,
        aligned_scans=aligned_scans,
        out_dir=args.out_dir
    )

    # 同时保存一个 txt，方便直接查看
    out_txt = os.path.join(args.out_dir, "poses_in_map.txt")

    with open(out_txt, "w", encoding="utf-8") as f:
        f.write("# Multi-point relocalization result\n")
        f.write("# pose_i_map means scan_i pose in global map frame\n")
        f.write("# prediction uses incremental odom delta by default\n\n")

        for r in results:
            point_id = r["point_id"]
            T = r["T_map"]
            x, y, yaw = params_from_T_2d(T)

            f.write(f"pose_{point_id}_map:\n")
            f.write(f"  x_m: {x:.9f}\n")
            f.write(f"  y_m: {y:.9f}\n")
            f.write(f"  yaw_deg: {np.degrees(yaw):.9f}\n")
            f.write(f"  yaw_rad: {yaw:.9f}\n")
            f.write(f"  icp_success: {r['result']['icp']['success']}\n")
            f.write(f"  icp_fitness: {r['result']['icp']['fitness']:.9f}\n")
            f.write(f"  icp_rmse: {r['result']['icp']['rmse']:.9f}\n")
            f.write(f"  icp_corr: {r['result']['icp']['num_corr']}\n")

            if r["T_pred"] is not None:
                px, py, pyaw = params_from_T_2d(r["T_pred"])
                f.write(f"  pred_x_m: {px:.9f}\n")
                f.write(f"  pred_y_m: {py:.9f}\n")
                f.write(f"  pred_yaw_deg: {np.degrees(pyaw):.9f}\n")
            else:
                f.write(f"  pred_x_m: nan\n")
                f.write(f"  pred_y_m: nan\n")
                f.write(f"  pred_yaw_deg: nan\n")

            f.write("\n")

    print(f"[Save] poses txt: {out_txt}")

    # ------------------------------------------------------------
    # 5. 可视化
    # ------------------------------------------------------------

    if not args.no_vis:
        print("\n[Visualize] global map + all aligned scans")

        vis_list = []

        vis_global = copy.deepcopy(map_data["global_pcd"])
        colorize_pcd(vis_global, [0.2, 0.7, 1.0])
        vis_list.append(vis_global)

        colors = [
            [1.0, 0.1, 0.1],
            [1.0, 0.6, 0.1],
            [0.1, 1.0, 0.1],
            [0.8, 0.1, 1.0],
            [1.0, 1.0, 0.1],
            [0.1, 1.0, 1.0],
        ]

        for i, (aligned, _) in enumerate(aligned_scans):
            vis_scan = copy.deepcopy(aligned)
            colorize_pcd(vis_scan, colors[i % len(colors)])
            vis_list.append(vis_scan)

        o3d.visualization.draw_geometries(
            vis_list,
            window_name="Multi-Point Relocalization: Global + Aligned Scans",
            zoom=0.8,
            front=[0, 0, 1],
            lookat=[0, 0, 0],
            up=[0, 1, 0]
        )


if __name__ == "__main__":
    main()