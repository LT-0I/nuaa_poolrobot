#!/usr/bin/env python3
"""Copy and convert a saved sonar log dataset into return-home snapshots."""

import argparse
import csv
import math
import os
import re
import shutil
from pathlib import Path


def load_odom_log(dataset_root):
    odom_path = dataset_root / "sonar_odom_log.csv"
    by_position = {}
    by_submap_index = {}
    if not odom_path.is_file():
        return by_position, by_submap_index

    with odom_path.open("r", encoding="utf-8", newline="") as input_file:
        reader = csv.DictReader(input_file)
        for row in reader:
            try:
                position = int(row.get("position", ""))
                submap_index = int(row.get("submap_index", ""))
                pose = {
                    "odom_x": float(row.get("pose_x", "0") or 0.0),
                    "odom_y": float(row.get("pose_y", "0") or 0.0),
                    "odom_yaw": math.radians(float(row.get("pose_theta_deg", "0") or 0.0)),
                    "odom_source": "sonar_odom_log.csv",
                }
            except (TypeError, ValueError):
                continue
            by_position[position] = pose
            by_submap_index[submap_index] = pose
    return by_position, by_submap_index


def parse_snapshot_info(path, scan_id, odom_by_position, odom_by_submap_index):
    name = path.name
    index_match = re.search(r"submap_(\d+)", name)
    pos_match = re.search(r"_pos(\d+)", name)
    dist_match = re.search(r"_dist(-?[0-9]+(?:\.[0-9]+)?)", name)
    time_match = re.search(r"(\d{8}_\d{6})", name)
    submap_index = int(index_match.group(1)) if index_match else scan_id
    position = int(pos_match.group(1)) if pos_match else scan_id

    odom = odom_by_submap_index.get(submap_index) or odom_by_position.get(position)
    if odom:
        odom_x = odom["odom_x"]
        odom_y = odom["odom_y"]
        odom_yaw = odom["odom_yaw"]
        odom_source = odom["odom_source"]
    else:
        if dist_match:
            odom_x = float(dist_match.group(1))
        elif position <= 1:
            odom_x = 0.0
        else:
            odom_x = 4.0 * float(position - 1)
        odom_y = 0.0
        odom_yaw = 0.0
        odom_source = "filename_dist_fallback"
        if position <= 1:
            odom_x = 0.0

    return {
        "scan_id": scan_id,
        "position": position,
        "odom_x": odom_x,
        "odom_y": odom_y,
        "odom_yaw": odom_yaw,
        "odom_source": odom_source,
        "stamp": path.stat().st_mtime,
        "timestamp": time_match.group(1) if time_match else "",
    }


def discover_frame_pcdfiles(maps_dir):
    files = []
    for path in maps_dir.glob("*.pcd"):
        name = path.name
        if "pool_map" in name:
            continue
        if "submap" not in name:
            continue
        files.append(path)

    def sort_key(path):
        name = path.name
        time_match = re.search(r"(\d{8}_\d{6})", name)
        timestamp = time_match.group(1) if time_match else "99999999_999999"
        pos_match = re.search(r"_pos(\d+)", name)
        position = int(pos_match.group(1)) if pos_match else 999
        return (timestamp, position, name)

    files.sort(key=sort_key)
    return files


def write_metadata(path, pcd_name, info):
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as out:
        out.write("schema: sonar_return_home_v1\n")
        out.write(f"pointcloud: {pcd_name}\n")
        out.write(f"scan_id: {info['scan_id']}\n")
        out.write(f"position_number: {info['position']}\n")
        out.write(f"stamp: {info['stamp']:.3f}\n")
        out.write("full_scan: true\n")
        out.write("coverage_deg: 360.0\n")
        out.write("cloud_frame_id: sonar_frame\n")
        out.write("odom:\n")
        out.write("  valid: true\n")
        out.write(f"  source: {info['odom_source']}\n")
        out.write("  frame_id: odom\n")
        out.write("  child_frame_id: base_link\n")
        out.write("  epoch: 0\n")
        out.write(f"  x: {info['odom_x']:.6f}\n")
        out.write(f"  y: {info['odom_y']:.6f}\n")
        out.write(f"  yaw: {info['odom_yaw']:.9f}\n")
    tmp.replace(path)


def copy_dataset(source, output, force):
    if output.exists():
        if not force:
            raise RuntimeError(f"output exists: {output} (use --force)")
        shutil.rmtree(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, output, symlinks=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True,
                        help="Saved sonar log root. It should contain maps/ and optional sonar_odom_log.csv.")
    parser.add_argument(
        "--output",
        default="converted_sonar_logs",
        help="Output directory for copied logs and generated return_home sidecars.",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    source = Path(args.source).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if not source.is_dir():
        raise SystemExit(f"source directory not found: {source}")

    copy_dataset(source, output, args.force)

    maps_dir = output / "maps"
    frames = discover_frame_pcdfiles(maps_dir)
    odom_by_position, odom_by_submap_index = load_odom_log(output)
    manifest_path = output / "return_home_manifest.csv"
    with manifest_path.open("w", encoding="utf-8", newline="") as manifest_file:
        writer = csv.writer(manifest_file)
        writer.writerow([
            "scan_id",
            "position_number",
            "odom_x",
            "odom_y",
            "odom_yaw_rad",
            "odom_source",
            "pcd_file",
            "metadata_file",
            "timestamp",
        ])
        for scan_id, pcd_path in enumerate(frames, start=1):
            info = parse_snapshot_info(pcd_path, scan_id,
                                       odom_by_position, odom_by_submap_index)
            metadata_path = pcd_path.with_name(f"{pcd_path.stem}.return_home.yaml")
            write_metadata(metadata_path, pcd_path.name, info)
            writer.writerow([
                scan_id,
                info["position"],
                f"{info['odom_x']:.6f}",
                f"{info['odom_y']:.6f}",
                f"{info['odom_yaw']:.9f}",
                info["odom_source"],
                os.path.relpath(pcd_path, output),
                os.path.relpath(metadata_path, output),
                info["timestamp"],
            ])

    print(f"copied: {source} -> {output}")
    print(f"converted frames: {len(frames)}")
    print(f"manifest: {manifest_path}")


if __name__ == "__main__":
    main()
