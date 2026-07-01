import os
import csv
import argparse
import numpy as np


# ============================================================
# 1. 基础工具
# ============================================================

def to_float(v, default=None):
    try:
        if v is None:
            return default
        s = str(v).strip()
        if s == "":
            return default
        return float(s)
    except Exception:
        return default


def to_int(v, default=None):
    try:
        if v is None:
            return default
        s = str(v).strip()
        if s == "":
            return default
        return int(float(s))
    except Exception:
        return default


def normalize_key_dict(row):
    return {str(k).strip().lstrip("\ufeff"): v for k, v in row.items()}


def find_first_existing_key(row, candidates):
    for k in candidates:
        if k in row and str(row[k]).strip() != "":
            return k
    return None


def wrap_angle_rad(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def wrap_angle_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def detect_angle_unit(values, user_unit="auto"):

    if user_unit in ["rad", "deg"]:
        return user_unit

    valid = []
    for v in values:
        if v is None:
            continue
        try:
            fv = float(v)
            if np.isfinite(fv):
                valid.append(abs(fv))
        except Exception:
            pass

    if len(valid) == 0:
        return "rad"

    max_abs = max(valid)

    if max_abs <= 2.0 * np.pi + 0.5:
        return "rad"

    return "deg"


def angle_to_rad(v, unit):
    if unit == "deg":
        return np.deg2rad(v)
    return v


def angle_to_deg(v, unit):
    if unit == "deg":
        return v
    return np.rad2deg(v)


# ============================================================
# 2. SE(2) 位姿 / 矩阵工具
# ============================================================

def pose_to_T(x, y, yaw_rad):
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)

    T = np.eye(3, dtype=np.float64)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 2] = x
    T[1, 2] = y

    return T


def invert_T(T):
    R = T[:2, :2]
    t = T[:2, 2]

    T_inv = np.eye(3, dtype=np.float64)
    T_inv[:2, :2] = R.T
    T_inv[:2, 2] = -R.T @ t

    return T_inv


def T_to_pose(T):
    x = float(T[0, 2])
    y = float(T[1, 2])
    yaw_rad = float(np.arctan2(T[1, 0], T[0, 0]))
    yaw_rad = wrap_angle_rad(yaw_rad)
    return x, y, yaw_rad


def T_entries(T, prefix):
    return {
        f"{prefix}_00": float(T[0, 0]),
        f"{prefix}_01": float(T[0, 1]),
        f"{prefix}_02": float(T[0, 2]),
        f"{prefix}_10": float(T[1, 0]),
        f"{prefix}_11": float(T[1, 1]),
        f"{prefix}_12": float(T[1, 2]),
        f"{prefix}_20": float(T[2, 0]),
        f"{prefix}_21": float(T[2, 1]),
        f"{prefix}_22": float(T[2, 2]),
    }


# ============================================================
# 3. CSV / Markdown 表格读取
# ============================================================

def read_table_flexible(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"文件不存在: {path}")

    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]

    if len(lines) == 0:
        raise ValueError(f"空文件: {path}")

    is_markdown = any(ln.startswith("|") and ln.endswith("|") for ln in lines)

    rows = []

    if is_markdown:
        table_lines = [ln for ln in lines if ln.startswith("|") and ln.endswith("|")]

        if len(table_lines) < 3:
            raise ValueError(f"Markdown 表格行数不足: {path}")

        header = [x.strip().lstrip("\ufeff") for x in table_lines[0].strip("|").split("|")]

        for ln in table_lines[2:]:
            parts = [x.strip() for x in ln.strip("|").split("|")]

            if len(parts) != len(header):
                continue

            item = dict(zip(header, parts))

            if all(v.strip("- ") == "" for v in item.values()):
                continue

            rows.append(item)

    else:
        with open(path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for item in reader:
                rows.append(item)

    return rows


# ============================================================
# 4. 读取 odom_full
# ============================================================

def load_odom_full(path, odom_yaw_unit="auto", odom_yaw_sign=1.0):
    rows_raw = read_table_flexible(path)
    rows = []

    for item in rows_raw:
        item = normalize_key_dict(item)

        seq = to_int(item.get("seq"), None)
        wall_t = to_float(item.get("wall_t"), None)
        odom_stamp = to_float(item.get("odom_stamp"), None)

        x = to_float(item.get("x"), None)
        y = to_float(item.get("y"), None)

        yaw_key = find_first_existing_key(item, [
            "yaw",
            "theta",
            "heading",
            "yaw_raw",
            "yaw_rad",
            "yaw_deg"
        ])

        if yaw_key is None:
            continue

        yaw_raw = to_float(item.get(yaw_key), None)

        if x is None or y is None or yaw_raw is None:
            continue

        if odom_stamp is None:
            odom_stamp = wall_t

        rows.append({
            "seq": seq,
            "wall_t": wall_t,
            "odom_stamp": odom_stamp,
            "x": x,
            "y": y,
            "yaw_raw": yaw_raw,
            "yaw_key": yaw_key,
            "twist_vx": item.get("twist_vx", ""),
            "twist_wz": item.get("twist_wz", ""),
            "active_point_id": item.get("active_point_id", ""),
            "event": item.get("event", ""),
            "raw": item
        })

    if len(rows) == 0:
        raise ValueError("没有从 odom_full 中解析到有效 odom 数据")

    if all(r["seq"] is not None for r in rows):
        rows.sort(key=lambda d: d["seq"])
    else:
        rows.sort(key=lambda d: d["odom_stamp"])

    # 如果字段名明确是 yaw_rad / yaw_deg，则优先用字段名判断
    yaw_keys = set(r["yaw_key"] for r in rows)

    if odom_yaw_unit == "auto":
        if yaw_keys == {"yaw_rad"}:
            yaw_unit = "rad"
        elif yaw_keys == {"yaw_deg"}:
            yaw_unit = "deg"
        else:
            yaw_unit = detect_angle_unit([r["yaw_raw"] for r in rows], "auto")
    else:
        yaw_unit = odom_yaw_unit

    for r in rows:
        yaw_signed_raw = odom_yaw_sign * r["yaw_raw"]
        r["yaw_unit"] = yaw_unit
        r["yaw"] = r["yaw_raw"]
        r["yaw_rad"] = angle_to_rad(yaw_signed_raw, yaw_unit)
        r["yaw_deg"] = angle_to_deg(yaw_signed_raw, yaw_unit)

    print("\n[Load odom_full]")
    print(f"  file: {path}")
    print(f"  valid rows: {len(rows)}")
    print(f"  detected/used odom yaw unit: {yaw_unit}")
    print(f"  odom yaw sign: {odom_yaw_sign}")

    if rows[0]["seq"] is not None:
        print(f"  seq range: {rows[0]['seq']} -> {rows[-1]['seq']}")

    if rows[0]["odom_stamp"] is not None:
        print(f"  odom_stamp range: {rows[0]['odom_stamp']} -> {rows[-1]['odom_stamp']}")

    xs = np.array([r["x"] for r in rows])
    ys = np.array([r["y"] for r in rows])
    yaws_raw = np.array([r["yaw_raw"] for r in rows])
    yaws_deg = np.array([r["yaw_deg"] for r in rows])

    print(f"  x range: {xs.min():.9f} -> {xs.max():.9f}")
    print(f"  y range: {ys.min():.9f} -> {ys.max():.9f}")
    print(f"  yaw raw range: {yaws_raw.min():.9f} -> {yaws_raw.max():.9f}")
    print(f"  yaw deg range: {yaws_deg.min():.9f} -> {yaws_deg.max():.9f}")

    return rows


# ============================================================
# 5. 读取 points_index
# ============================================================

def load_points_index(path, anchor_seq_mode="start"):
    rows_raw = read_table_flexible(path)
    points = []

    for item in rows_raw:
        item = normalize_key_dict(item)

        point_id = to_int(item.get("point_id"), None)
        if point_id is None:
            point_id = to_int(item.get("id"), None)

        if point_id is None:
            continue

        seq_start = to_int(item.get("odom_full_seq_start"), None)
        seq_end = to_int(item.get("odom_full_seq_end"), None)

        if seq_start is None and seq_end is None:
            print(f"[WARN] point {point_id} 没有 odom_full_seq_start/end，跳过")
            continue

        if anchor_seq_mode == "start":
            anchor_seq = seq_start if seq_start is not None else seq_end
        elif anchor_seq_mode == "end":
            anchor_seq = seq_end if seq_end is not None else seq_start
        elif anchor_seq_mode == "mid":
            if seq_start is not None and seq_end is not None:
                anchor_seq = int(round(0.5 * (seq_start + seq_end)))
            else:
                anchor_seq = seq_start if seq_start is not None else seq_end
        else:
            raise ValueError(f"未知 anchor_seq_mode: {anchor_seq_mode}")

        x_at_scan = to_float(item.get("x_at_scan"), None)
        y_at_scan = to_float(item.get("y_at_scan"), None)
        yaw_at_scan = to_float(item.get("yaw_at_scan"), None)

        scan_start_wall = to_float(item.get("scan_start_wall"), None)
        scan_end_wall = to_float(item.get("scan_end_wall"), None)

        points.append({
            "point_id": point_id,
            "seq_start": seq_start,
            "seq_end": seq_end,
            "anchor_seq": anchor_seq,
            "x_at_scan": x_at_scan,
            "y_at_scan": y_at_scan,
            "yaw_at_scan": yaw_at_scan,
            "scan_start_wall": scan_start_wall,
            "scan_end_wall": scan_end_wall,
            "raw": item
        })

    points.sort(key=lambda d: d["point_id"])

    if len(points) == 0:
        raise ValueError("没有从 points_index 中解析到有效采样点")

    print("\n[Load points_index]")
    print(f"  file: {path}")
    print(f"  valid points: {len(points)}")
    print(f"  anchor_seq_mode: {anchor_seq_mode}")

    for p in points:
        print(
            f"  point {p['point_id']}: "
            f"seq_start={p['seq_start']}, "
            f"seq_end={p['seq_end']}, "
            f"anchor_seq={p['anchor_seq']}, "
            f"odom_at_scan=({p['x_at_scan']}, {p['y_at_scan']}, {p['yaw_at_scan']})"
        )

    return points


# ============================================================
# 6. 读取点云配准得到的 map 位姿
# ============================================================

def load_map_poses(path):
    rows_raw = read_table_flexible(path)
    poses = {}

    for item in rows_raw:
        item = normalize_key_dict(item)

        point_id = to_int(item.get("point_id"), None)
        if point_id is None:
            point_id = to_int(item.get("id"), None)
        if point_id is None:
            point_id = to_int(item.get("scan_id"), None)

        if point_id is None:
            continue

        x_key = find_first_existing_key(item, [
            "x_map",
            "map_x",
            "pose_x_map",
            "final_x",
            "icp_x",
            "opt_x",
            "x"
        ])

        y_key = find_first_existing_key(item, [
            "y_map",
            "map_y",
            "pose_y_map",
            "final_y",
            "icp_y",
            "opt_y",
            "y"
        ])

        yaw_deg_key = find_first_existing_key(item, [
            "yaw_map_deg",
            "map_yaw_deg",
            "pose_yaw_map_deg",
            "final_yaw_deg",
            "icp_yaw_deg",
            "opt_yaw_deg",
            "yaw_deg",
            "theta_deg"
        ])

        yaw_rad_key = find_first_existing_key(item, [
            "yaw_map_rad",
            "map_yaw_rad",
            "pose_yaw_map_rad",
            "final_yaw_rad",
            "icp_yaw_rad",
            "opt_yaw_rad",
            "yaw_rad",
            "theta_rad"
        ])

        yaw_plain_key = find_first_existing_key(item, [
            "yaw_map",
            "map_yaw",
            "pose_yaw_map",
            "final_yaw",
            "icp_yaw",
            "opt_yaw",
            "yaw",
            "theta"
        ])

        if x_key is None or y_key is None:
            print(f"[WARN] point {point_id} 没有找到 map x/y，跳过")
            continue

        x = to_float(item.get(x_key), None)
        y = to_float(item.get(y_key), None)

        yaw_deg = None

        if yaw_deg_key is not None:
            yaw_deg = to_float(item.get(yaw_deg_key), None)

        elif yaw_rad_key is not None:
            yaw_rad = to_float(item.get(yaw_rad_key), None)
            if yaw_rad is not None:
                yaw_deg = float(np.rad2deg(yaw_rad))

        elif yaw_plain_key is not None:
            yaw_val = to_float(item.get(yaw_plain_key), None)
            if yaw_val is not None:
                if abs(yaw_val) > 2.0 * np.pi + 0.5:
                    yaw_deg = yaw_val
                else:
                    yaw_deg = float(np.rad2deg(yaw_val))

        if x is None or y is None or yaw_deg is None:
            print(f"[WARN] point {point_id} map pose 不完整，跳过")
            continue

        poses[point_id] = {
            "point_id": point_id,
            "map_x": x,
            "map_y": y,
            "map_yaw_deg": yaw_deg,
            "map_yaw_rad": float(np.deg2rad(yaw_deg)),
            "raw": item
        }

    if len(poses) == 0:
        raise ValueError("没有从 map poses 文件中解析到有效位姿")

    print("\n[Load map poses]")
    print(f"  file: {path}")
    print(f"  valid poses: {len(poses)}")

    for pid in sorted(poses.keys()):
        p = poses[pid]
        print(
            f"  point {pid}: "
            f"map=({p['map_x']:.6f}, {p['map_y']:.6f}, {p['map_yaw_deg']:.6f} deg)"
        )

    return poses


# ============================================================
# 7. 根据 seq 找 odom 行
# ============================================================

def nearest_odom_index_by_seq(odom_rows, target_seq):
    if target_seq is None:
        raise ValueError("target_seq 为空，无法查找最近 odom 行")

    seqs = np.array([
        r["seq"] if r["seq"] is not None else i
        for i, r in enumerate(odom_rows)
    ], dtype=np.float64)

    idx = int(np.argmin(np.abs(seqs - target_seq)))
    return idx, odom_rows[idx]

def nearest_odom_index_by_wall_t(odom_rows, target_wall_t):
    """
    根据 wall_t 查找最近的 odom 行。

    points_index 里面的 scan_start_wall / scan_end_wall
    是 wall time，所以在线激活重定位结果时，建议用 wall_t 匹配。
    """
    if target_wall_t is None:
        raise ValueError("target_wall_t 为空，无法查找最近 odom 行")

    ts = np.array([
        r["wall_t"] if r["wall_t"] is not None else r["odom_stamp"]
        for r in odom_rows
    ], dtype=np.float64)

    idx = int(np.argmin(np.abs(ts - target_wall_t)))
    return idx, odom_rows[idx]


# ============================================================
# 8. 构造 anchor 与坐标变换矩阵
# ============================================================

def build_anchors(points_index, map_poses, odom_rows, activation_mode="scan_end"):
    anchors = []

    for p in points_index:
        pid = p["point_id"]

        if pid not in map_poses:
            print(f"[WARN] point {pid} 没有对应的 map 配准位姿，跳过")
            continue

        # --------------------------------------------------------
        # 1. 用 anchor_seq 找到该点的 odom 位姿
        # --------------------------------------------------------
        anchor_seq = p["anchor_seq"]
        odom_idx, odom_row = nearest_odom_index_by_seq(odom_rows, anchor_seq)

        seq_error = None
        if odom_row["seq"] is not None:
            seq_error = abs(odom_row["seq"] - anchor_seq)

        # --------------------------------------------------------
        # 2. 构造 T_odom_base_anchor
        # --------------------------------------------------------
        odom_T_anchor = pose_to_T(
            odom_row["x"],
            odom_row["y"],
            odom_row["yaw_rad"]
        )

        # --------------------------------------------------------
        # 3. 构造 T_map_base_anchor
        # --------------------------------------------------------
        map_T_anchor = pose_to_T(
            map_poses[pid]["map_x"],
            map_poses[pid]["map_y"],
            map_poses[pid]["map_yaw_rad"]
        )

        # --------------------------------------------------------
        # 4. 计算 T_map_odom
        # --------------------------------------------------------
        T_map_odom = map_T_anchor @ invert_T(odom_T_anchor)

        # --------------------------------------------------------
        # 5. 计算在线激活时刻
        # --------------------------------------------------------
        scan_start_wall = p.get("scan_start_wall", None)
        scan_end_wall = p.get("scan_end_wall", None)

        if activation_mode == "scan_end":
            if scan_end_wall is not None:
                activation_idx, activation_row = nearest_odom_index_by_wall_t(
                    odom_rows, scan_end_wall
                )
            elif p["seq_end"] is not None:
                activation_idx, activation_row = nearest_odom_index_by_seq(
                    odom_rows, p["seq_end"]
                )
            else:
                activation_idx, activation_row = odom_idx, odom_row

        elif activation_mode == "scan_start":
            if scan_start_wall is not None:
                activation_idx, activation_row = nearest_odom_index_by_wall_t(
                    odom_rows, scan_start_wall
                )
            elif p["seq_start"] is not None:
                activation_idx, activation_row = nearest_odom_index_by_seq(
                    odom_rows, p["seq_start"]
                )
            else:
                activation_idx, activation_row = odom_idx, odom_row

        elif activation_mode == "anchor":
            activation_idx, activation_row = odom_idx, odom_row

        else:
            raise ValueError(f"未知 activation_mode: {activation_mode}")

        anchors.append({
            "point_id": pid,

            # anchor 位姿对应的 odom 时刻
            "anchor_seq_from_points": anchor_seq,
            "odom_index": odom_idx,
            "odom_seq": odom_row["seq"],
            "seq_error": seq_error,

            "odom_stamp": odom_row["odom_stamp"],
            "wall_t": odom_row["wall_t"],

            # 在线启用该 anchor 的时刻
            "activation_mode": activation_mode,
            "activation_index": activation_idx,
            "activation_seq": activation_row["seq"],
            "activation_wall_t": activation_row["wall_t"],
            "activation_odom_stamp": activation_row["odom_stamp"],

            "scan_start_wall": scan_start_wall,
            "scan_end_wall": scan_end_wall,

            # anchor 处 odom 位姿
            "odom_x_anchor": odom_row["x"],
            "odom_y_anchor": odom_row["y"],
            "odom_yaw_anchor_raw": odom_row["yaw_raw"],
            "odom_yaw_anchor_rad": odom_row["yaw_rad"],
            "odom_yaw_anchor_deg": odom_row["yaw_deg"],

            # anchor 处 map 位姿
            "map_x": map_poses[pid]["map_x"],
            "map_y": map_poses[pid]["map_y"],
            "map_yaw_rad": map_poses[pid]["map_yaw_rad"],
            "map_yaw_deg": map_poses[pid]["map_yaw_deg"],

            "odom_T_anchor": odom_T_anchor,
            "map_T_anchor": map_T_anchor,
            "T_map_odom": T_map_odom
        })

    anchors.sort(key=lambda d: d["activation_index"])

    if len(anchors) == 0:
        raise ValueError("没有构造出有效 anchor")

    print("\n[Anchors and online activation]")
    for a in anchors:
        print(
            f"  point {a['point_id']}: "
            f"anchor_seq={a['anchor_seq_from_points']}, "
            f"anchor_odom_idx={a['odom_index']}, "
            f"anchor_odom_seq={a['odom_seq']}, "
            f"activation_idx={a['activation_index']}, "
            f"activation_seq={a['activation_seq']}, "
            f"activation_wall_t={a['activation_wall_t']}"
        )
        print(
            f"    odom_anchor=({a['odom_x_anchor']:.9f}, "
            f"{a['odom_y_anchor']:.9f}, "
            f"{a['odom_yaw_anchor_deg']:.9f} deg)"
        )
        print(
            f"    map_anchor =({a['map_x']:.9f}, "
            f"{a['map_y']:.9f}, "
            f"{a['map_yaw_deg']:.9f} deg)"
        )
        print("    T_map_odom:")
        print(a["T_map_odom"])

    return anchors

def parse_id_list(s):
    if s is None:
        return None

    s = str(s).strip()

    if s == "" or s.lower() == "all":
        return None

    ids = []

    for item in s.replace("，", ",").split(","):
        item = item.strip()
        if item:
            ids.append(int(item))

    return ids


def estimate_shared_R_from_anchor_positions(anchors, anchor_ids=None):
    odom_pts = []
    map_pts = []
    valid_ids = []

    id_set = None
    if anchor_ids is not None:
        id_set = set(int(x) for x in anchor_ids)

    for a in anchors:
        pid = int(a["point_id"])

        if id_set is not None and pid not in id_set:
            continue

        odom_pts.append([
            float(a["odom_x_anchor"]),
            float(a["odom_y_anchor"])
        ])

        map_pts.append([
            float(a["map_x"]),
            float(a["map_y"])
        ])

        valid_ids.append(pid)

    if len(valid_ids) < 2:
        raise ValueError(
            f"multi_xy 模式至少需要 2 个 anchor，当前有效 anchor={valid_ids}"
        )

    A = np.asarray(odom_pts, dtype=np.float64)
    B = np.asarray(map_pts, dtype=np.float64)

    mu_A = np.mean(A, axis=0)
    mu_B = np.mean(B, axis=0)

    A0 = A - mu_A
    B0 = B - mu_B

    H = A0.T @ B0

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T @ U.T

    # 防止反射
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T

    t_global = mu_B - R @ mu_A

    theta = float(np.arctan2(R[1, 0], R[0, 0]))

    pred_B = (R @ A.T).T + t_global
    errors = np.linalg.norm(pred_B - B, axis=1)

    print("\n[Multi-anchor shared rotation estimation]")
    print(f"  used anchor ids: {valid_ids}")
    print(f"  shared rotation: {np.rad2deg(theta):.9f} deg")
    print(f"  global translation: tx={t_global[0]:.9f}, ty={t_global[1]:.9f}")

    print("  fitting errors:")
    for pid, e, pa, pb, pp in zip(valid_ids, errors, A, B, pred_B):
        print(
            f"    P{pid}: err={e:.6f} m | "
            f"odom=({pa[0]:.6f}, {pa[1]:.6f}) -> "
            f"pred_map=({pp[0]:.6f}, {pp[1]:.6f}), "
            f"real_map=({pb[0]:.6f}, {pb[1]:.6f})"
        )

    print(f"  mean error: {np.mean(errors):.6f} m")
    print(f"  max  error: {np.max(errors):.6f} m")

    return R, t_global, theta, valid_ids, errors


def make_T_from_R_t(R, t):
    T = np.eye(3, dtype=np.float64)
    T[:2, :2] = R
    T[0, 2] = float(t[0])
    T[1, 2] = float(t[1])
    return T


def rewrite_anchor_T_map_odom_with_shared_rotation(
    anchors,
    R_shared,
    t_global=None,
    translation_mode="per_anchor"
):
    if translation_mode not in ["per_anchor", "global_fit"]:
        raise ValueError(f"未知 translation_mode: {translation_mode}")

    if translation_mode == "global_fit" and t_global is None:
        raise ValueError("global_fit 模式需要 t_global")

    theta = float(np.arctan2(R_shared[1, 0], R_shared[0, 0]))

    print("\n[Rewrite anchors T_map_odom with shared rotation]")
    print(f"  translation_mode: {translation_mode}")
    print(f"  shared yaw offset: {np.rad2deg(theta):.9f} deg")

    new_anchors = []

    for a in anchors:
        a = dict(a)

        odom_xy = np.array([
            float(a["odom_x_anchor"]),
            float(a["odom_y_anchor"])
        ], dtype=np.float64)

        map_xy = np.array([
            float(a["map_x"]),
            float(a["map_y"])
        ], dtype=np.float64)

        if translation_mode == "per_anchor":
            t_i = map_xy - R_shared @ odom_xy
        else:
            t_i = np.asarray(t_global, dtype=np.float64)

        T_new = make_T_from_R_t(R_shared, t_i)

        a["T_map_odom_original_from_pose_yaw"] = a["T_map_odom"]
        a["T_map_odom"] = T_new

        a["shared_rotation_yaw_rad"] = theta
        a["shared_rotation_yaw_deg"] = float(np.rad2deg(theta))
        a["shared_translation_mode"] = translation_mode

        new_anchors.append(a)

        print(
            f"  P{a['point_id']}: "
            f"t=({t_i[0]:.9f}, {t_i[1]:.9f}), "
            f"T_yaw={np.rad2deg(theta):.9f} deg"
        )

    return new_anchors


# ============================================================
# 9. 基于 SE(2) 坐标变换修正轨迹
# ============================================================

def correct_trajectory_by_transform(
    odom_rows,
    anchors,
    output_range="from_first_to_last",
    transform_mode="segment_anchor"
):
    if len(odom_rows) == 0:
        return []

    if len(anchors) == 0:
        raise ValueError("anchors 为空，无法修正轨迹")

    if transform_mode not in ["segment_anchor", "first_anchor"]:
        raise ValueError(f"未知 transform_mode: {transform_mode}")

    anchors = sorted(anchors, key=lambda d: d["odom_index"])
    anchor_by_index = {a["odom_index"]: a for a in anchors}

    ref_anchor = anchors[0]
    T_ref_map_odom = ref_anchor["T_map_odom"]

    first_anchor_idx = anchors[0]["odom_index"]
    last_anchor_idx = anchors[-1]["odom_index"]

    current_anchor = None
    corrected = []
    prev_out = None

    for i, row in enumerate(odom_rows):

        # --------------------------------------------------------
        # 输出范围控制
        # --------------------------------------------------------
        if output_range == "from_first_to_last":
            if i < first_anchor_idx or i > last_anchor_idx:
                continue
        elif output_range == "from_first":
            if i < first_anchor_idx:
                continue
        elif output_range == "all":
            pass
        else:
            raise ValueError(f"未知 output_range: {output_range}")

        # 当前 odom 位姿矩阵
        T_odom_i = pose_to_T(
            row["x"],
            row["y"],
            row["yaw_rad"]
        )

        # --------------------------------------------------------
        # raw_map：始终使用第 1 个 anchor 的 T_map_odom
        # --------------------------------------------------------
        T_raw_map_i = T_ref_map_odom @ T_odom_i
        raw_map_x, raw_map_y, raw_map_yaw_rad = T_to_pose(T_raw_map_i)
        raw_map_yaw_deg = float(np.rad2deg(raw_map_yaw_rad))

        # --------------------------------------------------------
        # 判断当前行是否是 anchor
        # --------------------------------------------------------
        is_anchor = 0
        reset_to_anchor = 0
        anchor_point_id = ""

        if i in anchor_by_index:
            current_anchor = anchor_by_index[i]
            is_anchor = 1
            anchor_point_id = current_anchor["point_id"]

        # --------------------------------------------------------
        # 选择本帧使用哪个 T_map_odom
        # --------------------------------------------------------
        if transform_mode == "first_anchor":
            active_anchor = ref_anchor
            T_active_map_odom = T_ref_map_odom

            if is_anchor and i == ref_anchor["odom_index"]:
                segment = "first_anchor_reset"
                reset_to_anchor = 1
            elif is_anchor:
                segment = "anchor_marker_no_reset_first_transform"
                reset_to_anchor = 0
            else:
                segment = "first_anchor_transform_integrate"
                reset_to_anchor = 0

        else:
            if current_anchor is None:
                active_anchor = ref_anchor
                T_active_map_odom = T_ref_map_odom
                segment = "before_first_anchor_by_first_transform"
                reset_to_anchor = 0
            else:
                active_anchor = current_anchor
                T_active_map_odom = current_anchor["T_map_odom"]

                if is_anchor:
                    segment = "anchor_reset_segment_transform"
                    reset_to_anchor = 1
                else:
                    segment = "segment_transform_integrate"
                    reset_to_anchor = 0

        # --------------------------------------------------------
        # opt_map：使用当前 active_anchor 的 T_map_odom
        # --------------------------------------------------------
        T_opt_map_i = T_active_map_odom @ T_odom_i
        opt_x, opt_y, opt_yaw_rad = T_to_pose(T_opt_map_i)
        opt_yaw_deg = float(np.rad2deg(opt_yaw_rad))

        opt_yaw_deg_wrapped = wrap_angle_deg(opt_yaw_deg)
        opt_yaw_rad_wrapped = np.deg2rad(opt_yaw_deg_wrapped)

        # --------------------------------------------------------
        # 相邻 odom 原始 delta
        # --------------------------------------------------------
        if i == 0:
            delta_odom_x = 0.0
            delta_odom_y = 0.0
            delta_odom_yaw_raw = 0.0
            delta_odom_yaw_rad = 0.0
            delta_odom_yaw_deg = 0.0
        else:
            prev = odom_rows[i - 1]
            delta_odom_x = row["x"] - prev["x"]
            delta_odom_y = row["y"] - prev["y"]
            delta_odom_yaw_raw = row["yaw_raw"] - prev["yaw_raw"]
            delta_odom_yaw_rad = row["yaw_rad"] - prev["yaw_rad"]
            delta_odom_yaw_deg = row["yaw_deg"] - prev["yaw_deg"]

        # --------------------------------------------------------
        # 修正后 map 坐标系下的相邻 delta
        # --------------------------------------------------------
        if prev_out is None:
            delta_opt_map_x = 0.0
            delta_opt_map_y = 0.0
            delta_opt_map_yaw_deg = 0.0
            delta_opt_map_yaw_deg_wrapped = 0.0
        else:
            delta_opt_map_x = opt_x - prev_out["opt_x"]
            delta_opt_map_y = opt_y - prev_out["opt_y"]
            delta_opt_map_yaw_deg = opt_yaw_deg - prev_out["opt_yaw_deg"]
            delta_opt_map_yaw_deg_wrapped = wrap_angle_deg(delta_opt_map_yaw_deg)

        # --------------------------------------------------------
        # 当前帧相对 active_anchor 的 odom 差值
        # 注意：这是 odom 坐标系内的差值，不直接作为 map 位移使用
        # --------------------------------------------------------
        rel_odom_from_anchor_x = row["x"] - active_anchor["odom_x_anchor"]
        rel_odom_from_anchor_y = row["y"] - active_anchor["odom_y_anchor"]
        rel_odom_from_anchor_yaw_raw = row["yaw_raw"] - active_anchor["odom_yaw_anchor_raw"]
        rel_odom_from_anchor_yaw_rad = row["yaw_rad"] - active_anchor["odom_yaw_anchor_rad"]
        rel_odom_from_anchor_yaw_deg = row["yaw_deg"] - active_anchor["odom_yaw_anchor_deg"]

        # --------------------------------------------------------
        # 当前帧相对 active_anchor 的 map 差值
        # 这是经过坐标变换后的真实 map 坐标系增量
        # --------------------------------------------------------
        rel_map_from_anchor_x = opt_x - active_anchor["map_x"]
        rel_map_from_anchor_y = opt_y - active_anchor["map_y"]
        rel_map_from_anchor_yaw_deg = wrap_angle_deg(opt_yaw_deg - active_anchor["map_yaw_deg"])

        out = {
            "seq": row["seq"],
            "wall_t": row["wall_t"],
            "odom_stamp": row["odom_stamp"],

            # 原始 odom
            "odom_x": row["x"],
            "odom_y": row["y"],
            "odom_yaw": row["yaw_raw"],
            "odom_yaw_raw": row["yaw_raw"],
            "odom_yaw_unit": row["yaw_unit"],
            "odom_yaw_rad": row["yaw_rad"],
            "odom_yaw_deg": row["yaw_deg"],

            # odom 坐标系下相邻帧直接 delta
            "delta_odom_x": delta_odom_x,
            "delta_odom_y": delta_odom_y,
            "delta_odom_yaw": delta_odom_yaw_raw,
            "delta_odom_yaw_raw": delta_odom_yaw_raw,
            "delta_odom_yaw_rad": delta_odom_yaw_rad,
            "delta_odom_yaw_deg": delta_odom_yaw_deg,

            # 使用第 1 个 anchor 变换后的纯 odom map 轨迹
            "raw_map_x": raw_map_x,
            "raw_map_y": raw_map_y,
            "raw_map_yaw_rad": raw_map_yaw_rad,
            "raw_map_yaw_deg": raw_map_yaw_deg,

            # 最终修正后的 map 轨迹
            "opt_x": opt_x,
            "opt_y": opt_y,
            "opt_yaw": opt_yaw_rad,
            "opt_yaw_rad": opt_yaw_rad,
            "opt_yaw_deg": opt_yaw_deg,
            "opt_yaw_deg_wrapped": opt_yaw_deg_wrapped,
            "opt_yaw_rad_wrapped": opt_yaw_rad_wrapped,

            # map 坐标系下相邻帧 delta
            "delta_opt_map_x": delta_opt_map_x,
            "delta_opt_map_y": delta_opt_map_y,
            "delta_opt_map_yaw_deg": delta_opt_map_yaw_deg,
            "delta_opt_map_yaw_deg_wrapped": delta_opt_map_yaw_deg_wrapped,

            # 当前帧相对 active_anchor 的 odom 差值
            "rel_odom_from_anchor_x": rel_odom_from_anchor_x,
            "rel_odom_from_anchor_y": rel_odom_from_anchor_y,
            "rel_odom_from_anchor_yaw_raw": rel_odom_from_anchor_yaw_raw,
            "rel_odom_from_anchor_yaw_rad": rel_odom_from_anchor_yaw_rad,
            "rel_odom_from_anchor_yaw_deg": rel_odom_from_anchor_yaw_deg,

            # 当前帧相对 active_anchor 的 map 差值
            "rel_map_from_anchor_x": rel_map_from_anchor_x,
            "rel_map_from_anchor_y": rel_map_from_anchor_y,
            "rel_map_from_anchor_yaw_deg": rel_map_from_anchor_yaw_deg,

            # 原始 odom 附加信息
            "twist_vx": row["twist_vx"],
            "twist_wz": row["twist_wz"],
            "active_point_id": row["active_point_id"],
            "event": row["event"],

            # 调试信息
            "transform_mode": transform_mode,
            "ref_anchor_point_id": ref_anchor["point_id"],
            "base_point_id": active_anchor["point_id"],
            "segment": segment,
            "is_anchor": is_anchor,
            "anchor_point_id": anchor_point_id,
            "reset_to_anchor": reset_to_anchor,

            "anchor_odom_x": active_anchor["odom_x_anchor"],
            "anchor_odom_y": active_anchor["odom_y_anchor"],
            "anchor_odom_yaw_raw": active_anchor["odom_yaw_anchor_raw"],
            "anchor_odom_yaw_rad": active_anchor["odom_yaw_anchor_rad"],
            "anchor_odom_yaw_deg": active_anchor["odom_yaw_anchor_deg"],

            "anchor_map_x": active_anchor["map_x"],
            "anchor_map_y": active_anchor["map_y"],
            "anchor_map_yaw_rad": active_anchor["map_yaw_rad"],
            "anchor_map_yaw_deg": active_anchor["map_yaw_deg"],
        }

        out.update(T_entries(T_active_map_odom, "T_map_odom"))

        corrected.append(out)
        prev_out = out

    return corrected


def correct_trajectory_online_dynamic(
    odom_rows,
    anchors,
    output_before_first_anchor=False,
    output_range="all"
):
    if len(odom_rows) == 0:
        return []

    if len(anchors) == 0:
        raise ValueError("anchors 为空，无法在线修正轨迹")

    anchors = sorted(anchors, key=lambda d: d["activation_index"])

    first_activation_idx = anchors[0]["activation_index"]
    last_activation_idx = anchors[-1]["activation_index"]

    corrected = []

    active_anchor = None
    active_T_map_odom = None
    next_anchor_ptr = 0

    prev_out = None

    for i, row in enumerate(odom_rows):

        # --------------------------------------------------------
        # 输出范围控制
        # --------------------------------------------------------
        if output_range == "from_first_activation":
            if i < first_activation_idx:
                continue
        elif output_range == "from_first_to_last_activation":
            if i < first_activation_idx or i > last_activation_idx:
                continue
        elif output_range == "all":
            pass
        else:
            raise ValueError(f"未知 output_range: {output_range}")

        # --------------------------------------------------------
        # 判断是否有新的重定位结果在当前时刻可用
        # --------------------------------------------------------
        relocation_update = 0
        activated_point_id = ""

        while next_anchor_ptr < len(anchors) and i >= anchors[next_anchor_ptr]["activation_index"]:
            active_anchor = anchors[next_anchor_ptr]
            active_T_map_odom = active_anchor["T_map_odom"]

            relocation_update = 1
            activated_point_id = active_anchor["point_id"]

            next_anchor_ptr += 1

        # --------------------------------------------------------
        # 如果还没有任何 map 定位结果
        # --------------------------------------------------------
        if active_anchor is None:
            if not output_before_first_anchor:
                continue

            # 还没有 map anchor 时，只能输出 odom 局部轨迹
            opt_x = row["x"]
            opt_y = row["y"]
            opt_yaw_rad = row["yaw_rad"]
            opt_yaw_deg = row["yaw_deg"]

            raw_map_x = opt_x
            raw_map_y = opt_y
            raw_map_yaw_rad = opt_yaw_rad
            raw_map_yaw_deg = opt_yaw_deg

            segment = "before_first_relocation_odom_only"
            base_point_id = ""
            reset_to_anchor = 0

            anchor_map_x = ""
            anchor_map_y = ""
            anchor_map_yaw_rad = ""
            anchor_map_yaw_deg = ""

            anchor_odom_x = ""
            anchor_odom_y = ""
            anchor_odom_yaw_raw = ""
            anchor_odom_yaw_rad = ""
            anchor_odom_yaw_deg = ""

            rel_odom_from_anchor_x = ""
            rel_odom_from_anchor_y = ""
            rel_odom_from_anchor_yaw_raw = ""
            rel_odom_from_anchor_yaw_rad = ""
            rel_odom_from_anchor_yaw_deg = ""

            rel_map_from_anchor_x = ""
            rel_map_from_anchor_y = ""
            rel_map_from_anchor_yaw_deg = ""

            T_used = np.eye(3, dtype=np.float64)

        else:
            # ----------------------------------------------------
            # 当前 odom 位姿
            # ----------------------------------------------------
            T_odom_i = pose_to_T(
                row["x"],
                row["y"],
                row["yaw_rad"]
            )

            # ----------------------------------------------------
            # 在线全局修正轨迹
            # ----------------------------------------------------
            T_opt_map_i = active_T_map_odom @ T_odom_i
            opt_x, opt_y, opt_yaw_rad = T_to_pose(T_opt_map_i)
            opt_yaw_deg = float(np.rad2deg(opt_yaw_rad))

            raw_map_x = opt_x
            raw_map_y = opt_y
            raw_map_yaw_rad = opt_yaw_rad
            raw_map_yaw_deg = opt_yaw_deg

            segment = "online_dynamic_transform"
            base_point_id = active_anchor["point_id"]
            reset_to_anchor = 1 if relocation_update else 0

            anchor_map_x = active_anchor["map_x"]
            anchor_map_y = active_anchor["map_y"]
            anchor_map_yaw_rad = active_anchor["map_yaw_rad"]
            anchor_map_yaw_deg = active_anchor["map_yaw_deg"]

            anchor_odom_x = active_anchor["odom_x_anchor"]
            anchor_odom_y = active_anchor["odom_y_anchor"]
            anchor_odom_yaw_raw = active_anchor["odom_yaw_anchor_raw"]
            anchor_odom_yaw_rad = active_anchor["odom_yaw_anchor_rad"]
            anchor_odom_yaw_deg = active_anchor["odom_yaw_anchor_deg"]

            rel_odom_from_anchor_x = row["x"] - active_anchor["odom_x_anchor"]
            rel_odom_from_anchor_y = row["y"] - active_anchor["odom_y_anchor"]
            rel_odom_from_anchor_yaw_raw = row["yaw_raw"] - active_anchor["odom_yaw_anchor_raw"]
            rel_odom_from_anchor_yaw_rad = wrap_angle_rad(row["yaw_rad"] - active_anchor["odom_yaw_anchor_rad"])
            rel_odom_from_anchor_yaw_deg = wrap_angle_deg(row["yaw_deg"] - active_anchor["odom_yaw_anchor_deg"])

            rel_map_from_anchor_x = opt_x - active_anchor["map_x"]
            rel_map_from_anchor_y = opt_y - active_anchor["map_y"]
            rel_map_from_anchor_yaw_deg = wrap_angle_deg(opt_yaw_deg - active_anchor["map_yaw_deg"])

            T_used = active_T_map_odom

        opt_yaw_deg_wrapped = wrap_angle_deg(opt_yaw_deg)
        opt_yaw_rad_wrapped = np.deg2rad(opt_yaw_deg_wrapped)

        # --------------------------------------------------------
        # odom 相邻帧增量
        # --------------------------------------------------------
        if i == 0:
            delta_odom_x = 0.0
            delta_odom_y = 0.0
            delta_odom_yaw_raw = 0.0
            delta_odom_yaw_rad = 0.0
            delta_odom_yaw_deg = 0.0
        else:
            prev = odom_rows[i - 1]
            delta_odom_x = row["x"] - prev["x"]
            delta_odom_y = row["y"] - prev["y"]
            delta_odom_yaw_raw = row["yaw_raw"] - prev["yaw_raw"]
            delta_odom_yaw_rad = wrap_angle_rad(row["yaw_rad"] - prev["yaw_rad"])
            delta_odom_yaw_deg = wrap_angle_deg(row["yaw_deg"] - prev["yaw_deg"])

        # --------------------------------------------------------
        # map 修正轨迹相邻帧增量
        # --------------------------------------------------------
        if prev_out is None:
            delta_opt_map_x = 0.0
            delta_opt_map_y = 0.0
            delta_opt_map_yaw_deg = 0.0
            delta_opt_map_yaw_deg_wrapped = 0.0
        else:
            delta_opt_map_x = opt_x - prev_out["opt_x"]
            delta_opt_map_y = opt_y - prev_out["opt_y"]
            delta_opt_map_yaw_deg = opt_yaw_deg - prev_out["opt_yaw_deg"]
            delta_opt_map_yaw_deg_wrapped = wrap_angle_deg(delta_opt_map_yaw_deg)

        out = {
            "seq": row["seq"],
            "wall_t": row["wall_t"],
            "odom_stamp": row["odom_stamp"],

            # 原始 odom
            "odom_x": row["x"],
            "odom_y": row["y"],
            "odom_yaw": row["yaw_raw"],
            "odom_yaw_raw": row["yaw_raw"],
            "odom_yaw_unit": row["yaw_unit"],
            "odom_yaw_rad": row["yaw_rad"],
            "odom_yaw_deg": row["yaw_deg"],

            # odom 增量
            "delta_odom_x": delta_odom_x,
            "delta_odom_y": delta_odom_y,
            "delta_odom_yaw": delta_odom_yaw_raw,
            "delta_odom_yaw_raw": delta_odom_yaw_raw,
            "delta_odom_yaw_rad": delta_odom_yaw_rad,
            "delta_odom_yaw_deg": delta_odom_yaw_deg,

            # map 输出
            "raw_map_x": raw_map_x,
            "raw_map_y": raw_map_y,
            "raw_map_yaw_rad": raw_map_yaw_rad,
            "raw_map_yaw_deg": raw_map_yaw_deg,

            "opt_x": opt_x,
            "opt_y": opt_y,
            "opt_yaw": opt_yaw_rad,
            "opt_yaw_rad": opt_yaw_rad,
            "opt_yaw_deg": opt_yaw_deg,
            "opt_yaw_deg_wrapped": opt_yaw_deg_wrapped,
            "opt_yaw_rad_wrapped": opt_yaw_rad_wrapped,

            # map 增量
            "delta_opt_map_x": delta_opt_map_x,
            "delta_opt_map_y": delta_opt_map_y,
            "delta_opt_map_yaw_deg": delta_opt_map_yaw_deg,
            "delta_opt_map_yaw_deg_wrapped": delta_opt_map_yaw_deg_wrapped,

            # 相对 anchor 的 odom 差值
            "rel_odom_from_anchor_x": rel_odom_from_anchor_x,
            "rel_odom_from_anchor_y": rel_odom_from_anchor_y,
            "rel_odom_from_anchor_yaw_raw": rel_odom_from_anchor_yaw_raw,
            "rel_odom_from_anchor_yaw_rad": rel_odom_from_anchor_yaw_rad,
            "rel_odom_from_anchor_yaw_deg": rel_odom_from_anchor_yaw_deg,

            # 相对 anchor 的 map 差值
            "rel_map_from_anchor_x": rel_map_from_anchor_x,
            "rel_map_from_anchor_y": rel_map_from_anchor_y,
            "rel_map_from_anchor_yaw_deg": rel_map_from_anchor_yaw_deg,

            # 原始附加信息
            "twist_vx": row["twist_vx"],
            "twist_wz": row["twist_wz"],
            "active_point_id": row["active_point_id"],
            "event": row["event"],

            # 在线调试信息
            "transform_mode": "online_dynamic",
            "ref_anchor_point_id": anchors[0]["point_id"],
            "base_point_id": base_point_id,
            "segment": segment,
            "is_anchor": 0,
            "anchor_point_id": activated_point_id,
            "reset_to_anchor": reset_to_anchor,
            "relocation_update": relocation_update,
            "activated_point_id": activated_point_id,

            # 当前使用的 anchor 信息
            "anchor_odom_x": anchor_odom_x,
            "anchor_odom_y": anchor_odom_y,
            "anchor_odom_yaw_raw": anchor_odom_yaw_raw,
            "anchor_odom_yaw_rad": anchor_odom_yaw_rad,
            "anchor_odom_yaw_deg": anchor_odom_yaw_deg,

            "anchor_map_x": anchor_map_x,
            "anchor_map_y": anchor_map_y,
            "anchor_map_yaw_rad": anchor_map_yaw_rad,
            "anchor_map_yaw_deg": anchor_map_yaw_deg,
        }

        out.update(T_entries(T_used, "T_map_odom"))

        corrected.append(out)
        prev_out = out

    return corrected


# ============================================================
# 10. 输出 CSV
# ============================================================

def save_corrected_csv(rows, out_path):
    if len(rows) == 0:
        raise ValueError("没有可保存的数据")

    fieldnames = [
        "seq",
        "wall_t",
        "odom_stamp",

        "odom_x",
        "odom_y",
        "odom_yaw",
        "odom_yaw_raw",
        "odom_yaw_unit",
        "odom_yaw_rad",
        "odom_yaw_deg",

        "delta_odom_x",
        "delta_odom_y",
        "delta_odom_yaw",
        "delta_odom_yaw_raw",
        "delta_odom_yaw_rad",
        "delta_odom_yaw_deg",

        "raw_map_x",
        "raw_map_y",
        "raw_map_yaw_rad",
        "raw_map_yaw_deg",

        "opt_x",
        "opt_y",
        "opt_yaw",
        "opt_yaw_rad",
        "opt_yaw_deg",
        "opt_yaw_deg_wrapped",
        "opt_yaw_rad_wrapped",

        "delta_opt_map_x",
        "delta_opt_map_y",
        "delta_opt_map_yaw_deg",
        "delta_opt_map_yaw_deg_wrapped",

        "rel_odom_from_anchor_x",
        "rel_odom_from_anchor_y",
        "rel_odom_from_anchor_yaw_raw",
        "rel_odom_from_anchor_yaw_rad",
        "rel_odom_from_anchor_yaw_deg",

        "rel_map_from_anchor_x",
        "rel_map_from_anchor_y",
        "rel_map_from_anchor_yaw_deg",

        "twist_vx",
        "twist_wz",
        "active_point_id",
        "event",

        "transform_mode",
        "ref_anchor_point_id",
        "base_point_id",
        "segment",
        "is_anchor",
        "anchor_point_id",
        "reset_to_anchor",
        "relocation_update",
        "activated_point_id",

        "anchor_odom_x",
        "anchor_odom_y",
        "anchor_odom_yaw_raw",
        "anchor_odom_yaw_rad",
        "anchor_odom_yaw_deg",

        "anchor_map_x",
        "anchor_map_y",
        "anchor_map_yaw_rad",
        "anchor_map_yaw_deg",

        "T_map_odom_00",
        "T_map_odom_01",
        "T_map_odom_02",
        "T_map_odom_10",
        "T_map_odom_11",
        "T_map_odom_12",
        "T_map_odom_20",
        "T_map_odom_21",
        "T_map_odom_22",
    ]

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for r in rows:
            writer.writerow({k: r.get(k, "") for k in fieldnames})

    print("\n[Save]")
    print(f"  corrected trajectory: {out_path}")
    print(f"  rows: {len(rows)}")


def save_anchor_check_csv(anchors, out_path):
    fieldnames = [
        "point_id",

        "anchor_seq_from_points",
        "odom_index",
        "odom_seq",
        "seq_error",
        "odom_stamp",
        "wall_t",

        "odom_x_anchor",
        "odom_y_anchor",
        "odom_yaw_anchor_raw",
        "odom_yaw_anchor_rad",
        "odom_yaw_anchor_deg",

        "map_x",
        "map_y",
        "map_yaw_rad",
        "map_yaw_deg",

        "T_map_odom_00",
        "T_map_odom_01",
        "T_map_odom_02",
        "T_map_odom_10",
        "T_map_odom_11",
        "T_map_odom_12",
        "T_map_odom_20",
        "T_map_odom_21",
        "T_map_odom_22",
    ]

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for a in anchors:
            row = {
                "point_id": a["point_id"],

                "anchor_seq_from_points": a["anchor_seq_from_points"],
                "odom_index": a["odom_index"],
                "odom_seq": a["odom_seq"],
                "seq_error": a["seq_error"],
                "odom_stamp": a["odom_stamp"],
                "wall_t": a["wall_t"],

                "odom_x_anchor": a["odom_x_anchor"],
                "odom_y_anchor": a["odom_y_anchor"],
                "odom_yaw_anchor_raw": a["odom_yaw_anchor_raw"],
                "odom_yaw_anchor_rad": a["odom_yaw_anchor_rad"],
                "odom_yaw_anchor_deg": a["odom_yaw_anchor_deg"],

                "map_x": a["map_x"],
                "map_y": a["map_y"],
                "map_yaw_rad": a["map_yaw_rad"],
                "map_yaw_deg": a["map_yaw_deg"],
            }

            row.update(T_entries(a["T_map_odom"], "T_map_odom"))

            writer.writerow(row)

    print(f"  anchor check: {out_path}")


# ============================================================
# 11. main
# ============================================================

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--odom",
        type=str,
        default="odom_full-1.csv",
        help="完整 odom 文件，例如 odom_full-1.csv"
    )

    parser.add_argument(
        "--points",
        type=str,
        default="points_index-1.csv",
        help="points_index.csv，包含采样点的 odom_full_seq_start/end"
    )

    parser.add_argument(
        "--map-poses",
        type=str,
        default="poses_in_map.csv",
        help="点云配准输出的采样点 map 位姿，例如 poses_in_map.csv"
    )

    parser.add_argument(
        "--out",
        type=str,
        default="corrected_trajectory_map.csv",
        help="输出连续修正轨迹 CSV"
    )

    parser.add_argument(
        "--anchor-check-out",
        type=str,
        default="corrected_anchors_check.csv",
        help="输出 anchor 与 T_map_odom 检查 CSV"
    )

    parser.add_argument(
        "--anchor-seq-mode",
        type=str,
        default="end",
        choices=["start", "mid", "end"],
        help=(
            "每个采样点使用 points_index 中哪个序号作为 anchor。"
            "start=odom_full_seq_start, "
            "end=odom_full_seq_end, "
            "mid=二者中点。默认 start。"
        )
    )

    parser.add_argument(
        "--activation-mode",
        type=str,
        default="scan_end",
        choices=["scan_start", "scan_end", "anchor"],
        help="在线模式下，重定位结果什么时候生效。真实在线推荐 scan_end"
    )

    parser.add_argument(
        "--trajectory-mode",
        type=str,
        default="online_dynamic",
        choices=["online_dynamic", "segment_anchor", "first_anchor"],
        help="轨迹修正模式"
    )

    parser.add_argument(
        "--output-range",
        type=str,
        default="all",
        choices=["all", "from_first_activation", "from_first_to_last_activation", "from_first_to_last", "from_first"],
        help="输出轨迹范围"
    )

    parser.add_argument(
        "--output-before-first-anchor",
        action="store_true",
        help="是否在第一个 map 定位结果可用前输出 odom 局部轨迹"
    )

    parser.add_argument(
        "--odom-yaw-unit",
        type=str,
        default="auto",
        choices=["auto", "rad", "deg"],
        help=(
            "odom yaw 单位。"
            "auto=自动判断；"
            "rad=弧度；"
            "deg=角度。"
            "如果 auto 误判，请手动指定。"
        )
    )

    parser.add_argument(
        "--odom-yaw-sign",
        type=float,
        default=1.0,
        help=(
            "odom yaw 符号，默认 1。"
            "如果发现转向方向与 map yaw 相反，可以尝试设为 -1。"
        )
    )

    parser.add_argument(
        "--anchor-transform-source",
        type=str,
        default="multi_xy",
        choices=["pose_yaw", "multi_xy"],
        help=(
            "anchor 的 T_map_odom 旋转来源。"
            "pose_yaw=使用每个点云配准位姿的 yaw_map - odom_yaw，旧逻辑；"
            "multi_xy=使用多个 anchor 的 x/y 位置关系拟合统一旋转，推荐。"
        )
    )

    parser.add_argument(
        "--multi-anchor-ids",
        type=str,
        default="1,2",
        help=(
            "multi_xy 模式下参与估计旋转的 anchor id，"
        )
    )

    parser.add_argument(
        "--multi-translation-mode",
        type=str,
        default="per_anchor",
        choices=["per_anchor", "global_fit"],
        help=(
            "multi_xy 模式下平移的处理方式。"
            "per_anchor=每段使用当前 anchor 的平移，保证经过重定位点，推荐；"
            "global_fit=所有轨迹使用统一最小二乘平移。"
        )
    )

    parser.add_argument(
        "--transform-mode",
        type=str,
        default="segment_anchor",
        choices=["segment_anchor", "first_anchor"],
        help=(
            "坐标变换模式。"
            "segment_anchor=每段使用当前 anchor 的 T_map_odom，推荐；"
            "first_anchor=全程只用第1个 anchor 的 T_map_odom。"
        )
    )

    args = parser.parse_args()

    odom_rows = load_odom_full(
        args.odom,
        odom_yaw_unit=args.odom_yaw_unit,
        odom_yaw_sign=args.odom_yaw_sign
    )

    points_index = load_points_index(
        args.points,
        anchor_seq_mode=args.anchor_seq_mode
    )

    map_poses = load_map_poses(args.map_poses)

    anchors = build_anchors(
        points_index,
        map_poses,
        odom_rows,
        activation_mode=args.activation_mode
    )


    if args.anchor_transform_source == "multi_xy":
        multi_anchor_ids = parse_id_list(args.multi_anchor_ids)

        R_shared, t_global, theta_shared, valid_ids, fit_errors = \
            estimate_shared_R_from_anchor_positions(
                anchors,
                anchor_ids=multi_anchor_ids
            )

        anchors = rewrite_anchor_T_map_odom_with_shared_rotation(
            anchors,
            R_shared=R_shared,
            t_global=t_global,
            translation_mode=args.multi_translation_mode
        )

    else:
        print("\n[Use original pose_yaw based T_map_odom]")
        print("  注意：该模式使用 map_yaw - odom_yaw 作为旋转，容易受单帧点云 yaw 误差影响。")

    save_anchor_check_csv(
        anchors,
        args.anchor_check_out
    )

    if args.trajectory_mode == "online_dynamic":
        corrected = correct_trajectory_online_dynamic(
            odom_rows,
            anchors,
            output_before_first_anchor=args.output_before_first_anchor,
            output_range=args.output_range
        )

    else:
        corrected = correct_trajectory_by_transform(
            odom_rows,
            anchors,
            output_range=args.output_range,
            transform_mode=args.trajectory_mode
        )

    save_corrected_csv(
        corrected,
        args.out
    )

if __name__ == "__main__":
    main()