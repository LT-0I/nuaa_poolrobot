#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS-free pytest for display_node.load_pcd_xyz (strict binary-PCD header parser).
rospy and message packages are stubbed so the real parser can be imported and exercised without ROS."""
import os
import sys
import types
import struct
import numpy as np
import pytest

# Stub ROS modules so `import display_node` succeeds on a host without ROS.
for _pkg, _names in [
    ("rospy", []),
    ("std_msgs", []), ("std_msgs.msg", ["Float64MultiArray", "String", "Header", "ColorRGBA"]),
    ("nav_msgs", []), ("nav_msgs.msg", ["Odometry", "Path"]),
    ("geometry_msgs", []), ("geometry_msgs.msg", ["PoseStamped", "Point", "Quaternion", "Vector3"]),
    ("visualization_msgs", []), ("visualization_msgs.msg", ["Marker", "MarkerArray"]),
    ("sensor_msgs", []), ("sensor_msgs.msg", ["PointCloud2", "PointField"]),
]:
    _m = types.ModuleType(_pkg)
    for _n in _names:
        setattr(_m, _n, type(_n, (object,), {}))
    sys.modules[_pkg] = _m

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import display_node as dn  # noqa: E402


def build_pcd(data_word="binary", fields="x y z intensity", sizes="4 4 4 4",
              types_="F F F F", counts="1 1 1 1", points=None, declared_points=None,
              include_data=True):
    if points is None:
        points = []
    npts = declared_points if declared_points is not None else len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS %s\nSIZE %s\nTYPE %s\nCOUNT %s\n"
        "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\n"
        % (fields, sizes, types_, counts, npts, npts)
    )
    if include_data:
        header += "DATA %s\n" % data_word
    body = b"".join(b"".join(struct.pack("<f", float(v)) for v in p) for p in points)
    return header.encode("ascii") + body


def write(tmp_path, data):
    p = tmp_path / "test.pcd"
    p.write_bytes(data)
    return str(p)


# ---------------- valid parses ----------------

def test_valid_binary_xyz_intensity(tmp_path):
    pts = [(1.0, 2.0, 3.0, 10.0), (4.0, 5.0, 6.0, 20.0)]
    path = write(tmp_path, build_pcd(points=pts))
    xyz = dn.load_pcd_xyz(path)
    assert xyz.shape == (2, 3) and xyz.dtype == np.float32
    assert np.allclose(xyz, [[1, 2, 3], [4, 5, 6]])


def test_valid_binary_xyz_only(tmp_path):
    pts = [(7.0, 8.0, 9.0)]
    path = write(tmp_path, build_pcd(fields="x y z", sizes="4 4 4", types_="F F F",
                                     counts="1 1 1", points=pts))
    xyz = dn.load_pcd_xyz(path)
    assert xyz.shape == (1, 3)
    assert np.allclose(xyz, [[7, 8, 9]])


def test_valid_empty_cloud(tmp_path):
    path = write(tmp_path, build_pcd(points=[], declared_points=0))
    xyz = dn.load_pcd_xyz(path)
    assert xyz.shape == (0, 3)


# ---------------- 5 required bad inputs (must raise, not crash) ----------------

def test_ascii_data_rejected(tmp_path):
    path = write(tmp_path, build_pcd(data_word="ascii",
                                     fields="x y z", sizes="4 4 4", types_="F F F", counts="1 1 1",
                                     points=[(1, 2, 3)]))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_binary_compressed_rejected(tmp_path):
    # whole-word check: 'binary_compressed' must be refused even though it starts with 'binary'
    path = write(tmp_path, build_pcd(data_word="binary_compressed", points=[(1, 2, 3, 4)]))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_fields_not_xyz_rejected(tmp_path):
    path = write(tmp_path, build_pcd(fields="a b c", sizes="4 4 4", types_="F F F",
                                     counts="1 1 1", points=[(1, 2, 3)]))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_points_mismatch_rejected(tmp_path):
    # declares 100 points but body only holds 1 -> body shorter than expected
    path = write(tmp_path, build_pcd(points=[(1, 2, 3, 4)], declared_points=100))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_empty_file_rejected(tmp_path):
    path = write(tmp_path, b"")
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


# ---------------- extra robustness ----------------

def test_no_data_line_rejected(tmp_path):
    path = write(tmp_path, build_pcd(include_data=False, points=[]))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_xyz_wrong_size_rejected(tmp_path):
    # x declared as 8-byte double -> not float32 -> refuse
    path = write(tmp_path, build_pcd(fields="x y z", sizes="8 4 4", types_="F F F",
                                     counts="1 1 1", declared_points=0))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


def test_length_mismatch_header_rejected(tmp_path):
    # SIZE has 3 entries but FIELDS has 4
    path = write(tmp_path, build_pcd(fields="x y z intensity", sizes="4 4 4",
                                     types_="F F F F", counts="1 1 1 1", declared_points=0))
    with pytest.raises(ValueError):
        dn.load_pcd_xyz(path)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
