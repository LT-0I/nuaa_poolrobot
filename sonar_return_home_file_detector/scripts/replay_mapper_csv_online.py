#!/usr/bin/env python3
"""Replay mapper CSV + maps output without modifying the source data."""

import argparse
import csv
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path


def parse_rows(csv_path, limit):
    with csv_path.open("r", encoding="utf-8", newline="") as input_file:
        rows = list(csv.DictReader(input_file))
    valid = []
    for row in rows:
        try:
            int(row["submap_index"])
            int(row["position"])
            float(row["pose_x"])
            float(row["pose_y"])
            float(row["pose_theta_deg"])
        except (KeyError, TypeError, ValueError):
            continue
        valid.append(row)
        if limit > 0 and len(valid) >= limit:
            break
    return valid


def resolve_pcd(source_root, row):
    maps_dir = source_root / "maps"
    pointcloud_file = row.get("pointcloud_file", "").strip()
    if pointcloud_file:
        candidate = maps_dir / pointcloud_file
        if candidate.is_file():
            return candidate

    submap_index = int(row["submap_index"])
    position = int(row["position"])
    expected = maps_dir / f"phase_multi_submap_{submap_index}_pos{position}.pcd"
    if expected.is_file():
        return expected

    prefix = f"phase_multi_submap_{submap_index}_pos{position}"
    matches = sorted(path for path in maps_dir.glob(f"{prefix}*.pcd")
                     if path.is_file())
    if matches:
        return matches[0]
    return None


def place_pcd(source, dest, copy_file):
    if dest.exists() or dest.is_symlink():
        dest.unlink()
    if copy_file:
        shutil.copy2(source, dest)
    else:
        link_target = os.path.relpath(source.resolve(), dest.parent.resolve())
        os.symlink(link_target, dest)


def count_processed_rows(profile_csv):
    if not profile_csv.is_file():
        return 0
    try:
        with profile_csv.open("r", encoding="utf-8", newline="") as input_file:
            reader = csv.DictReader(input_file)
            return sum(1 for row in reader
                       if row.get("processed_new_snapshot") == "1")
    except OSError:
        return 0


def wait_processed(profile_csv, expected, timeout_sec):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if count_processed_rows(profile_csv) >= expected:
            return True
        time.sleep(0.05)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True,
                        help="Read-only mapper log root containing sonar_odom_log.csv and maps/.")
    parser.add_argument("--work-dir", required=True,
                        help="Writable replay root. The script creates work-dir/maps and work-dir/sonar_odom_log.csv.")
    parser.add_argument("--csv", default="return_home_profile.csv",
                        help="Probe profile CSV output path.")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--poll-ms", type=int, default=100)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--copy", action="store_true",
                        help="Copy PCD files instead of symlinking them.")
    parser.add_argument("--no-probe", action="store_true")
    parser.add_argument("--probe-cmd",
                        default="rosrun sonar_return_home_file_detector return_home_directory_probe")
    args = parser.parse_args()

    source_root = Path(args.source).expanduser().resolve()
    source_csv = source_root / "sonar_odom_log.csv"
    source_maps = source_root / "maps"
    if not source_csv.is_file() or not source_maps.is_dir():
        print(f"source must contain sonar_odom_log.csv and maps/: {source_root}",
              file=sys.stderr)
        return 2

    work_dir = Path(args.work_dir).expanduser().resolve()
    try:
        work_dir.relative_to(source_root)
        print("refusing to write work-dir inside source directory", file=sys.stderr)
        return 2
    except ValueError:
        pass

    rows = parse_rows(source_csv, args.limit)
    frames = []
    for row in rows:
        pcd = resolve_pcd(source_root, row)
        if pcd is None:
            print(f"skip row without matching PCD: submap={row.get('submap_index')} "
                  f"position={row.get('position')} file={row.get('pointcloud_file')}",
                  file=sys.stderr)
            continue
        frames.append((row, pcd))

    if not frames:
        print(f"no valid frames found in {source_root}", file=sys.stderr)
        return 2

    if work_dir.exists():
        shutil.rmtree(work_dir)
    maps_work = work_dir / "maps"
    maps_work.mkdir(parents=True)

    profile_csv = Path(args.csv).expanduser()
    if profile_csv.exists():
        profile_csv.unlink()
    if profile_csv.parent != Path("."):
        profile_csv.parent.mkdir(parents=True, exist_ok=True)

    replay_csv = work_dir / "sonar_odom_log.csv"
    fieldnames = [
        "timestamp",
        "submap_index",
        "position",
        "pose_x",
        "pose_y",
        "pose_theta_deg",
        "pointcloud_file",
    ]

    print(f"source root: {source_root}")
    print(f"work root: {work_dir}")
    print(f"frames: {len(frames)}")

    probe = None
    with replay_csv.open("w", encoding="utf-8", newline="") as replay_file:
        writer = csv.DictWriter(replay_file, fieldnames=fieldnames)
        writer.writeheader()
        replay_file.flush()

        if not args.no_probe:
            watch_iterations = max(20, len(frames) * 30)
            cmd = shlex.split(args.probe_cmd) + [
                str(work_dir),
                "--reset",
                "--watch", str(watch_iterations),
                "--poll-ms", str(max(20, args.poll_ms)),
                "--expected-processed", str(len(frames)),
                "--csv", str(profile_csv),
            ]
            print("starting probe:", " ".join(shlex.quote(part) for part in cmd))
            probe = subprocess.Popen(cmd)

        for index, (row, pcd) in enumerate(frames, start=1):
            dest_pcd = maps_work / pcd.name
            place_pcd(pcd, dest_pcd, args.copy)

            writer.writerow({key: row.get(key, "") for key in fieldnames})
            replay_file.flush()
            os.fsync(replay_file.fileno())
            print(f"[{index}/{len(frames)}] {pcd.name} + csv row {row.get('submap_index')}")

            if probe:
                if probe.poll() is not None:
                    print("probe exited early", file=sys.stderr)
                    return probe.returncode or 1
                if not wait_processed(profile_csv, index, args.timeout_sec):
                    print(f"timeout waiting for processed frame {index}", file=sys.stderr)
                    return 1

    if probe:
        try:
            probe.wait(timeout=max(10, len(frames) * 2))
        except subprocess.TimeoutExpired:
            probe.terminate()
            probe.wait(timeout=5)
            print("probe timed out and was terminated", file=sys.stderr)
            return 1
        print(f"profile csv: {profile_csv}")
        return probe.returncode

    print("generated replay directory without running probe")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
