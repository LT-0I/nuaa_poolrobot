#!/usr/bin/env python3
"""Generate small return-home snapshot test data.

The generated directory mimics mapper output:
  sample_001_home.pcd
  sample_001_home.return_home.yaml
  ...
"""

import argparse
import math
from pathlib import Path


def make_points(scale=1.0, yaw=0.0):
    points = []
    for deg in range(0, 360, 4):
        angle = math.radians(deg)
        radius = scale * (3.0 + 0.35 * math.sin(3.0 * angle) + 0.2 * math.cos(5.0 * angle))
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        points.append((cy * x - sy * y, sy * x + cy * y, 0.0, 185.0))
    for x in [v * 0.2 for v in range(-20, 21)]:
        points.append((x, 4.2, 0.0, 175.0))
        points.append((x, -4.0, 0.0, 175.0))
    for y in [v * 0.2 for v in range(-20, 21)]:
        points.append((4.1, y, 0.0, 170.0))
        points.append((-3.9, y, 0.0, 170.0))
    return points


def write_pcd(path, points):
    with path.open("w", encoding="utf-8") as out:
        out.write("# .PCD v0.7 - Point Cloud Data file format\n")
        out.write("VERSION 0.7\n")
        out.write("FIELDS x y z intensity\n")
        out.write("SIZE 4 4 4 4\n")
        out.write("TYPE F F F F\n")
        out.write("COUNT 1 1 1 1\n")
        out.write(f"WIDTH {len(points)}\n")
        out.write("HEIGHT 1\n")
        out.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        out.write(f"POINTS {len(points)}\n")
        out.write("DATA ascii\n")
        for x, y, z, intensity in points:
            out.write(f"{x:.5f} {y:.5f} {z:.5f} {intensity:.1f}\n")


def write_meta(path, pcd_name, scan_id, position, stamp, x, y, yaw):
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as out:
        out.write("schema: sonar_return_home_v1\n")
        out.write(f"pointcloud: {pcd_name}\n")
        out.write(f"scan_id: {scan_id}\n")
        out.write(f"position_number: {position}\n")
        out.write(f"stamp: {stamp:.3f}\n")
        out.write("full_scan: true\n")
        out.write("coverage_deg: 360.0\n")
        out.write("cloud_frame_id: sonar_frame\n")
        out.write("odom:\n")
        out.write("  valid: true\n")
        out.write("  source: synthetic\n")
        out.write("  frame_id: odom\n")
        out.write("  child_frame_id: base_link\n")
        out.write("  epoch: 0\n")
        out.write(f"  x: {x:.4f}\n")
        out.write(f"  y: {y:.4f}\n")
        out.write(f"  yaw: {yaw:.6f}\n")
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", nargs="?", default="sample_data/generated")
    args = parser.parse_args()

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    samples = [
        ("sample_001_home", 1, 1, 1000.0, 0.0, 0.0, 0.0, make_points()),
        ("sample_002_away", 2, 2, 1001.0, 4.5, 0.0, 0.0, make_points(scale=0.96)),
        ("sample_003_return", 3, 3, 1002.0, 0.15, 0.03, 0.0, make_points()),
    ]

    for base, scan_id, position, stamp, x, y, yaw, points in samples:
        pcd_path = output_dir / f"{base}.pcd"
        meta_path = output_dir / f"{base}.return_home.yaml"
        write_pcd(pcd_path, points)
        write_meta(meta_path, pcd_path.name, scan_id, position, stamp, x, y, yaw)
        print(f"wrote {pcd_path}")
        print(f"wrote {meta_path}")


if __name__ == "__main__":
    main()
