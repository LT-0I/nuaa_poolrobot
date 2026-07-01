import os
import re
import csv
import math
import struct
import shutil
import argparse
import subprocess
import numpy as np
import open3d as o3d


# ============================================================
# 1. PCD 读取 / 保存
# ============================================================

def read_pcd_binary_xyzi(file_path):
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
            if len(parts) >= 2:
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

        offsets = []
        offset = 0

        for s, c in zip(sizes, counts):
            offsets.append(offset)
            offset += s * c

        x_id = fields.index("x")
        y_id = fields.index("y")
        z_id = fields.index("z")
        inten_id = fields.index("intensity") if "intensity" in fields else None

        pts = np.empty((points, 3), dtype=np.float64)
        intensity = np.ones((points,), dtype=np.float32)

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
            intensity[i] = inten

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)

        return pcd, intensity


def read_pcd_general(file_path):
    try:
        return read_pcd_binary_xyzi(file_path)
    except Exception as e:
        print(f"[WARN] binary xyzi 读取失败，尝试 Open3D 读取: {file_path}, error={e}")

        pcd = o3d.io.read_point_cloud(file_path)
        pts = np.asarray(pcd.points)
        intensity = np.ones((len(pts),), dtype=np.float32)

        return pcd, intensity


def save_pcd_binary_xyzi(pcd, intensity, filename):
    pts = np.asarray(pcd.points)
    n = len(pts)

    if intensity is None or len(intensity) != n:
        intensity = np.ones((n,), dtype=np.float32)

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )

    with open(filename, "wb") as f:
        f.write(header.encode("utf-8"))

        for i in range(n):
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
# 2. 点云滤波逻辑，来自 lvbo-1.py
# ============================================================

def force_to_2d(pcd):
    pts = np.asarray(pcd.points).copy()

    if len(pts) > 0:
        pts[:, 2] = 0.0

    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def filter_base_distance_intensity(
    pcd,
    intensity,
    min_dist=1.2,
    max_dist=12.0,
    min_inner=70.0,
    min_outer=120.0,
    max_limit=200.0
):
    pts = np.asarray(pcd.points)

    if len(pts) == 0:
        return pcd, intensity

    distances = np.sqrt(pts[:, 0] ** 2 + pts[:, 1] ** 2)

    mask_valid_distance = distances > min_dist
    mask_inner = (distances <= max_dist) & (intensity >= min_inner)
    mask_outer = (distances > max_dist) & (intensity >= min_outer)

    keep_mask = (mask_inner | mask_outer) & (intensity < max_limit) & mask_valid_distance

    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(pts[keep_mask])

    return filtered_pcd, intensity[keep_mask]


def filter_concentric_arcs(
    pcd,
    intensity,
    eps=0.3,
    min_points=10,
    max_radius_std=0.25
):
    pts = np.asarray(pcd.points)

    if len(pts) < min_points:
        return pcd, intensity

    labels = np.array(
        pcd.cluster_dbscan(
            eps=eps,
            min_points=min_points,
            print_progress=False
        )
    )

    keep_mask = np.zeros(len(pts), dtype=bool)
    max_label = labels.max()

    if max_label < 0:
        return pcd, intensity

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


def extract_pos_id(filename):
    """
    从 phase_multi_submap_1_pos1.pcd 这类文件名中提取 pos 编号。
    如果提取不到，则退回提取文件名中的最后一个数字。
    """
    basename = os.path.basename(filename)

    m = re.search(r"pos(\d+)", basename, flags=re.IGNORECASE)
    if m:
        return int(m.group(1))

    nums = re.findall(r"\d+", basename)
    if nums:
        return int(nums[-1])

    return 10 ** 9


def collect_raw_scan_files(maps_dir):
    if not os.path.isdir(maps_dir):
        raise FileNotFoundError(f"maps 文件夹不存在: {maps_dir}")

    files = []

    for name in os.listdir(maps_dir):
        if not name.lower().endswith(".pcd"):
            continue

        if name.lower().startswith("scan_"):
            continue

        path = os.path.join(maps_dir, name)
        files.append(path)

    files.sort(key=lambda p: (extract_pos_id(os.path.basename(p)), os.path.basename(p)))

    return files


def filter_all_scans(
    maps_dir,
    filtered_dir,
    min_dist=1.2,
    max_dist=12.0,
    min_inner=70.0,
    min_outer=120.0,
    max_limit=200.0,
    arc_eps=0.3,
    arc_min_points=10,
    arc_max_radius_std=0.25,
    clean_output=True
):
    raw_files = collect_raw_scan_files(maps_dir)

    if len(raw_files) == 0:
        raise ValueError(f"未在 maps 文件夹中找到待处理 PCD: {maps_dir}")

    if clean_output and os.path.isdir(filtered_dir):
        shutil.rmtree(filtered_dir)

    os.makedirs(filtered_dir, exist_ok=True)

    print("\n====================================================")
    print("  Step 1: Filter local sonar PCD scans")
    print("====================================================")
    print(f"  input maps dir : {maps_dir}")
    print(f"  output scan dir: {filtered_dir}")
    print(f"  raw pcd count  : {len(raw_files)}")

    scan_paths = []

    for idx, src_path in enumerate(raw_files, start=1):
        dst_name = f"scan_{idx}.pcd"
        dst_path = os.path.join(filtered_dir, dst_name)

        print(f"\n[{idx}/{len(raw_files)}] {os.path.basename(src_path)} -> {dst_name}")

        pcd, intensity = read_pcd_general(src_path)
        pcd = force_to_2d(pcd)

        print(f"  raw points: {len(pcd.points)}")

        pcd_a, intensity_a = filter_base_distance_intensity(
            pcd,
            intensity,
            min_dist=min_dist,
            max_dist=max_dist,
            min_inner=min_inner,
            min_outer=min_outer,
            max_limit=max_limit
        )

        pcd_b, intensity_b = filter_concentric_arcs(
            pcd_a,
            intensity_a,
            eps=arc_eps,
            min_points=arc_min_points,
            max_radius_std=arc_max_radius_std
        )

        pcd_b = force_to_2d(pcd_b)

        save_pcd_binary_xyzi(pcd_b, intensity_b, dst_path)

        print(f"  filtered points: {len(pcd_b.points)}")
        print(f"  saved: {dst_path}")

        scan_paths.append(dst_path)

    return scan_paths


# ============================================================
# 3. CSV 角度转换，来自 jiaodu1.py，但改成自动路径
# ============================================================

def safe_float(v):
    try:
        if v is None:
            return None

        s = str(v).strip()

        if s == "":
            return None

        return float(s)
    except Exception:
        return None


def convert_points_index_yaw_to_deg(input_csv, output_csv):
    if not os.path.exists(input_csv):
        raise FileNotFoundError(f"points_index 文件不存在: {input_csv}")

    with open(input_csv, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])

    if len(rows) == 0:
        raise ValueError(f"points_index 为空: {input_csv}")

    if "yaw_at_scan" not in fieldnames:
        raise ValueError("points_index.csv 中没有 yaw_at_scan 列")

    for row in rows:
        yaw = safe_float(row.get("yaw_at_scan"))

        if yaw is None:
            continue

        row["yaw_at_scan"] = f"{math.degrees(yaw):.12f}"

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    with open(output_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[Save] converted points_index: {output_csv}")


def convert_odom_full_yaw_to_deg(input_csv, output_csv):
    if not os.path.exists(input_csv):
        raise FileNotFoundError(f"odom_full 文件不存在: {input_csv}")

    with open(input_csv, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])

    if len(rows) == 0:
        raise ValueError(f"odom_full 为空: {input_csv}")

    if "yaw_rad" in fieldnames:
        for row in rows:
            yaw_rad = safe_float(row.get("yaw_rad"))

            if yaw_rad is None:
                row["yaw"] = row.get("yaw", "")
            else:
                row["yaw"] = f"{math.degrees(yaw_rad):.12f}"

        fieldnames = [x for x in fieldnames if x != "yaw_rad"]

        if "yaw" not in fieldnames:
            fieldnames.append("yaw")

        for row in rows:
            if "yaw_rad" in row:
                row.pop("yaw_rad", None)

    elif "yaw" in fieldnames:
        print("[WARN] odom_full.csv 中没有 yaw_rad，已有 yaw 列，将直接保留 yaw。")
    else:
        raise ValueError("odom_full.csv 中既没有 yaw_rad，也没有 yaw 列")

    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    with open(output_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[Save] converted odom_full: {output_csv}")


def convert_csv_angles(task_dir, output_dir):
    print("\n====================================================")
    print("  Step 2: Convert yaw rad to deg")
    print("====================================================")

    points_in = os.path.join(task_dir, "points_index.csv")
    odom_in = os.path.join(task_dir, "odom_full.csv")

    points_out = os.path.join(output_dir, "points_index-1.csv")
    odom_out = os.path.join(output_dir, "odom_full-1.csv")

    convert_points_index_yaw_to_deg(points_in, points_out)
    convert_odom_full_yaw_to_deg(odom_in, odom_out)

    return points_out, odom_out


# ============================================================
# 4. 外部脚本调用
# ============================================================

def run_command(cmd, cwd=None):
    print("\n[Run Command]")
    print("  " + " ".join(cmd))

    result = subprocess.run(
        cmd,
        cwd=cwd,
        text=True
    )

    if result.returncode != 0:
        raise RuntimeError(f"命令执行失败，returncode={result.returncode}: {' '.join(cmd)}")


def run_bbs_relocalization(
    python_exe,
    bbs_script,
    global_map,
    scan_dir,
    points_index_csv,
    out_dir,
    no_vis=True
):
    print("\n====================================================")
    print("  Step 3: Multi-point relocalization by bbs.py")
    print("====================================================")

    if not os.path.exists(bbs_script):
        raise FileNotFoundError(f"找不到 bbs.py: {bbs_script}")

    if not os.path.exists(global_map):
        raise FileNotFoundError(f"找不到全局地图: {global_map}")

    os.makedirs(out_dir, exist_ok=True)

    cmd = [
        python_exe,
        bbs_script,
        "--global_map",
        global_map,
        "--scan_dir",
        scan_dir,
        "--scan_pattern",
        "scan_{point_id}.pcd",
        "--odom_csv",
        points_index_csv,
        "--out_dir",
        out_dir,
    ]

    if no_vis:
        cmd.append("--no_vis")

    run_command(cmd)

    poses_csv = os.path.join(out_dir, "poses_in_map.csv")

    if not os.path.exists(poses_csv):
        raise FileNotFoundError(f"bbs-2.py 没有生成 poses_in_map.csv: {poses_csv}")

    return poses_csv


def run_trajectory_correction(
    python_exe,
    corrected_script,
    odom_csv,
    points_csv,
    poses_csv,
    out_csv,
    anchor_check_csv,
    multi_anchor_ids="1,2",
    anchor_seq_mode="end",
    activation_mode="scan_end",
    trajectory_mode="online_dynamic",
    output_range="all",
    multi_translation_mode="per_anchor",
    odom_yaw_unit="deg",
    odom_yaw_sign=1.0
):
    print("\n====================================================")
    print("  Step 4: Correct continuous trajectory by corrected.py")
    print("====================================================")

    if not os.path.exists(corrected_script):
        raise FileNotFoundError(f"找不到 corrected.py: {corrected_script}")

    cmd = [
        python_exe,
        corrected_script,
        "--odom",
        odom_csv,
        "--points",
        points_csv,
        "--map-poses",
        poses_csv,
        "--out",
        out_csv,
        "--anchor-check-out",
        anchor_check_csv,
        "--anchor-seq-mode",
        anchor_seq_mode,
        "--activation-mode",
        activation_mode,
        "--trajectory-mode",
        trajectory_mode,
        "--output-range",
        output_range,
        "--odom-yaw-unit",
        odom_yaw_unit,
        "--odom-yaw-sign",
        str(odom_yaw_sign),
        "--anchor-transform-source",
        "multi_xy",
        "--multi-anchor-ids",
        multi_anchor_ids,
        "--multi-translation-mode",
        multi_translation_mode,
    ]

    run_command(cmd)

    if not os.path.exists(out_csv):
        raise FileNotFoundError(f"corrected.py 没有生成修正轨迹: {out_csv}")

    if not os.path.exists(anchor_check_csv):
        raise FileNotFoundError(f"corrected.py 没有生成 anchor 检查表: {anchor_check_csv}")


# ============================================================
# 5. 主流程
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="全自动 sonar 多点重定位与轨迹修正流程"
    )

    parser.add_argument(
        "--task-dir",
        type=str,
        default="manual_collect/task_20260629_202646",
        help="任务文件夹，内部应包含 maps、odom_full.csv、points_index.csv"
    )

    parser.add_argument(
        "--global-map",
        type=str,
        default="global.pcd",
        help="全局点云地图 PCD"
    )

    parser.add_argument(
        "--bbs-script",
        type=str,
        default="bbs-1.py",
        help="bbs.py 路径"
    )

    parser.add_argument(
        "--corrected-script",
        type=str,
        default="corrected.py",
        help="corrected.py 路径"
    )

    parser.add_argument(
        "--work-dir-name",
        type=str,
        default="auto_relocalization_output",
        help="输出目录名，会创建在 task-dir 下"
    )

    parser.add_argument(
        "--filtered-scan-dir-name",
        type=str,
        default="filtered_scans",
        help="滤波后 scan_1.pcd、scan_2.pcd 的保存目录名"
    )

    parser.add_argument(
        "--multi-anchor-ids",
        type=str,
        default="1,2",
        help="用于拟合统一旋转的 anchor id。你的需求是只用前两个，所以默认 1,2"
    )

    parser.add_argument(
        "--anchor-seq-mode",
        type=str,
        default="end",
        choices=["start", "mid", "end"],
        help="采集点 anchor 使用 odom_full_seq_start / mid / end"
    )

    parser.add_argument(
        "--activation-mode",
        type=str,
        default="scan_end",
        choices=["scan_start", "scan_end", "anchor"],
        help="重定位结果生效时刻，在线显示推荐 scan_end"
    )

    parser.add_argument(
        "--output-range",
        type=str,
        default="all",
        choices=[
            "all",
            "from_first_activation",
            "from_first_to_last_activation",
            "from_first_to_last",
            "from_first"
        ],
        help="最终 corrected_trajectory_map.csv 输出范围"
    )

    parser.add_argument(
        "--multi-translation-mode",
        type=str,
        default="per_anchor",
        choices=["per_anchor", "global_fit"],
        help="multi_xy 下平移处理方式，per_anchor 可保证每段经过对应重定位点"
    )

    parser.add_argument(
        "--odom-yaw-sign",
        type=float,
        default=1.0,
        help="odom yaw 符号。如果发现旋转方向相反，可设为 -1"
    )

    parser.add_argument(
        "--keep-old-filtered",
        action="store_true",
        help="不删除旧的 filtered_scans 目录"
    )

    parser.add_argument(
        "--show-bbs-vis",
        action="store_true",
        help="允许 bbs-2.py 显示 Open3D 可视化"
    )

    parser.add_argument(
        "--python",
        type=str,
        default="python",
        help="Python 可执行程序，Windows 可用 python，Linux 可用 python3"
    )

    args = parser.parse_args()

    task_dir = os.path.abspath(args.task_dir)
    maps_dir = os.path.join(task_dir, "maps")
    work_dir = os.path.join(task_dir, args.work_dir_name)
    filtered_scan_dir = os.path.join(work_dir, args.filtered_scan_dir_name)

    os.makedirs(work_dir, exist_ok=True)

    print("\n====================================================")
    print("  Auto Sonar Relocalization Pipeline")
    print("====================================================")
    print(f"  task_dir       : {task_dir}")
    print(f"  maps_dir       : {maps_dir}")
    print(f"  work_dir       : {work_dir}")
    print(f"  global_map     : {os.path.abspath(args.global_map)}")
    print(f"  bbs_script     : {os.path.abspath(args.bbs_script)}")
    print(f"  corrected_script: {os.path.abspath(args.corrected_script)}")
    print(f"  multi_anchor_ids: {args.multi_anchor_ids}")

    # ------------------------------------------------------------
    # Step 1. 滤波点云并命名为 scan_1.pcd, scan_2.pcd ...
    # ------------------------------------------------------------

    filter_all_scans(
        maps_dir=maps_dir,
        filtered_dir=filtered_scan_dir,
        clean_output=not args.keep_old_filtered
    )

    # ------------------------------------------------------------
    # Step 2. 转换 CSV 中 yaw 弧度到角度
    # ------------------------------------------------------------

    converted_points_csv, converted_odom_csv = convert_csv_angles(
        task_dir=task_dir,
        output_dir=work_dir
    )

    # ------------------------------------------------------------
    # Step 3. BBS + ICP 多点重定位
    # ------------------------------------------------------------

    relocalization_out_dir = os.path.join(work_dir, "multi_relocalization_output")

    poses_csv = run_bbs_relocalization(
        python_exe=args.python,
        bbs_script=os.path.abspath(args.bbs_script),
        global_map=os.path.abspath(args.global_map),
        scan_dir=filtered_scan_dir,
        points_index_csv=converted_points_csv,
        out_dir=relocalization_out_dir,
        no_vis=not args.show_bbs_vis
    )

    # ------------------------------------------------------------
    # Step 4. 连续轨迹修正，只用前两个 anchor 拟合统一旋转
    # ------------------------------------------------------------

    corrected_trajectory_csv = os.path.join(work_dir, "corrected_trajectory_map.csv")
    corrected_anchor_check_csv = os.path.join(work_dir, "corrected_anchors_check.csv")

    run_trajectory_correction(
        python_exe=args.python,
        corrected_script=os.path.abspath(args.corrected_script),
        odom_csv=converted_odom_csv,
        points_csv=converted_points_csv,
        poses_csv=poses_csv,
        out_csv=corrected_trajectory_csv,
        anchor_check_csv=corrected_anchor_check_csv,
        multi_anchor_ids=args.multi_anchor_ids,
        anchor_seq_mode=args.anchor_seq_mode,
        activation_mode=args.activation_mode,
        trajectory_mode="online_dynamic",
        output_range=args.output_range,
        multi_translation_mode=args.multi_translation_mode,
        odom_yaw_unit="deg",
        odom_yaw_sign=args.odom_yaw_sign
    )

    print("\n====================================================")
    print("  Pipeline Finished")
    print("====================================================")
    print(f"  filtered scans              : {filtered_scan_dir}")
    print(f"  converted points_index      : {converted_points_csv}")
    print(f"  converted odom_full         : {converted_odom_csv}")
    print(f"  poses_in_map                : {poses_csv}")
    print(f"  corrected trajectory        : {corrected_trajectory_csv}")
    print(f"  corrected anchors check     : {corrected_anchor_check_csv}")
    print("====================================================")


if __name__ == "__main__":
    main()