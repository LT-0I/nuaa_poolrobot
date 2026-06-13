#!/usr/bin/env python3
"""Replay sonar_logs PCDs as live return-home file snapshots."""

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def parse_snapshot_info(path, scan_id):
    name = path.name
    pos_match = re.search(r"_pos(\d+)", name)
    dist_match = re.search(r"_dist(-?[0-9]+(?:\.[0-9]+)?)", name)
    time_match = re.search(r"(\d{8}_\d{6})", name)

    position = int(pos_match.group(1)) if pos_match else scan_id
    if dist_match:
        odom_x = float(dist_match.group(1))
    elif position <= 1:
        odom_x = 0.0
    else:
        odom_x = 4.0 * float(position - 1)

    if position <= 1:
        odom_x = 0.0

    return {
        "position": position,
        "odom_x": odom_x,
        "stamp": path.stat().st_mtime,
        "timestamp_text": time_match.group(1) if time_match else "",
    }


def discover_pcdfiles(source_dir, limit):
    files = []
    for path in sorted(Path(source_dir).glob("*.pcd")):
        name = path.name
        if "pool_map" in name:
            continue
        sidecar = path.with_name(f"{path.stem}.return_home.yaml")
        if "submap" not in name and not sidecar.is_file():
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
    if limit > 0:
        files = files[:limit]
    return files


def write_metadata(dest_dir, pcd_name, scan_id, info, source_metadata=None):
    base = Path(pcd_name).stem
    metadata_path = dest_dir / f"{base}.return_home.yaml"
    tmp_path = dest_dir / f"{base}.return_home.yaml.tmp"

    if source_metadata and source_metadata.is_file():
        with source_metadata.open("r", encoding="utf-8") as src, tmp_path.open("w", encoding="utf-8") as out:
            saw_pointcloud = False
            for line in src:
                if line.lstrip().startswith("pointcloud:"):
                    out.write(f"pointcloud: {pcd_name}\n")
                    saw_pointcloud = True
                else:
                    out.write(line)
            if not saw_pointcloud:
                out.write(f"pointcloud: {pcd_name}\n")
        tmp_path.replace(metadata_path)
        return metadata_path

    with tmp_path.open("w", encoding="utf-8") as out:
        out.write("schema: sonar_return_home_v1\n")
        out.write(f"pointcloud: {pcd_name}\n")
        out.write(f"scan_id: {scan_id}\n")
        out.write(f"position_number: {info['position']}\n")
        out.write(f"stamp: {info['stamp']:.3f}\n")
        out.write("full_scan: true\n")
        out.write("coverage_deg: 360.0\n")
        out.write("cloud_frame_id: sonar_frame\n")
        out.write("odom:\n")
        out.write("  valid: true\n")
        out.write("  source: filename_dist_fallback\n")
        out.write("  frame_id: odom\n")
        out.write("  child_frame_id: base_link\n")
        out.write("  epoch: 0\n")
        out.write(f"  x: {info['odom_x']:.6f}\n")
        out.write("  y: 0.000000\n")
        out.write("  yaw: 0.000000\n")
    tmp_path.replace(metadata_path)
    return metadata_path


def place_pcd(source, dest, copy_file):
    if dest.exists() or dest.is_symlink():
        dest.unlink()
    if copy_file:
        shutil.copy2(source, dest)
    else:
        link_target = os.path.relpath(source.resolve(), dest.parent.resolve())
        os.symlink(link_target, dest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True,
                        help="Directory containing saved submap PCD files.")
    parser.add_argument("--work-dir", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--interval-ms", type=int, default=500)
    parser.add_argument("--copy", action="store_true")
    parser.add_argument("--no-probe", action="store_true")
    parser.add_argument("--probe-cmd",
                        default="rosrun sonar_return_home_file_detector return_home_directory_probe")
    parser.add_argument("--csv", default="return_home_profile.csv")
    args = parser.parse_args()

    source_dir = Path(args.source)
    if not source_dir.is_dir():
        print(f"source directory not found: {source_dir}", file=sys.stderr)
        return 2

    files = discover_pcdfiles(source_dir, args.limit)
    if not files:
        print(f"no submap PCD files found in {source_dir}", file=sys.stderr)
        return 2

    if args.work_dir:
        work_dir = Path(args.work_dir).expanduser().resolve()
        if work_dir.exists():
            shutil.rmtree(work_dir)
        work_dir.mkdir(parents=True)
    else:
        work_dir = Path(tempfile.mkdtemp(prefix="return_home_replay_"))

    print(f"replay work dir: {work_dir}")
    print(f"snapshot count: {len(files)}")

    next_index = 1
    if not args.no_probe:
        first = files[0]
        first_dest = work_dir / first.name
        place_pcd(first, first_dest, args.copy)
        first_info = parse_snapshot_info(first, 1)
        first_source_metadata = first.with_name(f"{first.stem}.return_home.yaml")
        first_metadata = write_metadata(work_dir, first_dest.name, 1, first_info, first_source_metadata)
        print(f"[1/{len(files)}] {first_dest.name} -> {first_metadata.name}")
        next_index = 2

    probe = None
    if not args.no_probe:
        poll_ms = max(50, min(args.interval_ms // 2, 250))
        watch_iterations = max(20, len(files) * max(4, args.interval_ms // poll_ms + 3))
        csv_path = Path(args.csv)
        if csv_path.exists():
            csv_path.unlink()
        cmd = shlex.split(args.probe_cmd) + [
            str(work_dir),
            "--reset",
            "--watch", str(watch_iterations),
            "--poll-ms", str(poll_ms),
            "--expected-processed", str(len(files)),
            "--csv", args.csv,
        ]
        print("starting probe:", " ".join(shlex.quote(part) for part in cmd))
        probe = subprocess.Popen(cmd)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if probe.poll() is not None:
                break
            if csv_path.is_file():
                try:
                    lines = csv_path.read_text(encoding="utf-8").splitlines()
                    if len(lines) >= 2:
                        break
                except OSError:
                    pass
            time.sleep(0.05)

    for index, source in enumerate(files[next_index - 1:], start=next_index):
        dest = work_dir / source.name
        place_pcd(source, dest, args.copy)
        info = parse_snapshot_info(source, index)
        source_metadata = source.with_name(f"{source.stem}.return_home.yaml")
        metadata_path = write_metadata(work_dir, dest.name, index, info, source_metadata)
        print(f"[{index}/{len(files)}] {dest.name} -> {metadata_path.name}")
        time.sleep(max(0, args.interval_ms) / 1000.0)

    if probe:
        try:
            probe.wait(timeout=max(10, len(files) * 2))
        except subprocess.TimeoutExpired:
            probe.terminate()
            probe.wait(timeout=5)
            print("probe timed out and was terminated", file=sys.stderr)
            return 1
        print(f"profile csv: {args.csv}")
        return probe.returncode

    print("metadata replay generated without running probe")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
