#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer A - Online per-stop-point relocalization stepper (localization-kernel reuse shell).

WHAT THIS IS
------------
The vendored offline kernel bbs-1.py (sonar_slam/scripts/reloc/bbs-1.py) localizes a
whole task in one batch loop inside main(): it seeds numpy once, preprocesses the global
map once, then iterates over every stop point (scan_1..scan_N). Layer A refactors that
SAME loop body into a *stateful, one-point-at-a-time* stepper so it can run online
(robot arrives at a point -> we process exactly that one point, keeping incremental
state), while producing results that are BIT-EXACT to the batch pipeline / golden.

We never copy or re-implement any algorithm. Every algorithmic step is the vendored
function, imported verbatim via importlib (bbs-1.py has a hyphen, so it cannot be a
normal `import`). The vendored files are asserted against CHECKSUMS.sha256 at init, so
a modified vendored file aborts immediately.

WHY IT IS BIT-EXACT (RNG contract)
----------------------------------
The ONLY source of randomness in bbs-1.py is:
    * np.random.seed(0)                    once, in main()               (bbs-1.py:1954)
    * np.random.choice(...)                in random_sample_points_xy    (bbs-1.py:290)
random_sample_points_xy is called exactly once per stop point, from preprocess_local_scan
(bbs-1.py:1435). Nothing else in bbs-1.py or relocation-auto.py touches np.random
(verified by grep). open3d ops used here (SOR, DBSCAN, voxel downsample, custom
point-to-line ICP) are deterministic and do not draw from numpy's RNG.

main() consumes the RNG stream in this exact order:
    seed(0)
    parse_points_index_csv        # no RNG
    preprocess_global_map         # no RNG
    for each point (point_id asc):
        preprocess_local_scan     # <-- the single RNG draw for this point
        odom_row_to_T             # no RNG
        relocalize_* (BBS + ICP)  # no RNG

This stepper reproduces that order precisely:
    __init__:  np.random.seed(0); preprocess_global_map(...)
    step(...):  preprocess_local_scan(...); odom_row_to_T(...); relocalize_*(...)
Stop points are stepped in point_id-ascending order (== batch order, because
parse_points_index_csv sorts by point_id), and NOTHING draws from np.random between
two points. Therefore the RNG stream is consumed in the identical order, the seed is
consumed once, and the incremental (online) results equal the batch results, which
Phase 0 already proved equal to the vendor golden.

Global BBS runs ONLY for the first point (relocalize_first_scan_global). Subsequent
points use odom incremental prediction + local BBS + ICP (relocalize_scan_local). This
is exactly the "avoid re-running global BBS at every stop" online behaviour.

INPUT NOTE (critical)
---------------------
The odom rows MUST come from the rad->deg converted csv (relocation-auto Step 2 output,
points_index-1.csv). parse_points_index_csv treats yaw_at_scan as DEGREES and applies
np.deg2rad. Feeding the RAW radian points_index.csv would deg2rad an already-radian
value and shrink yaw ~57x. run_task() below performs the Step 2 conversion by reusing
relocation-auto.convert_points_index_yaw_to_deg, so callers pass the raw task csv.

REUSED VENDORED FUNCTIONS
-------------------------
From bbs-1.py: preprocess_global_map, preprocess_local_scan, parse_points_index_csv,
get_scan_path, odom_row_to_T, relocalize_first_scan_global,
predict_pose_from_odom_incremental, relocalize_scan_local, params_from_T_2d,
save_all_poses_csv, OUT_ALL_POSES_CSV, ODOM_YAW_DELTA_SIGN.
From relocation-auto.py: filter_all_scans (Step 1), convert_points_index_yaw_to_deg
(Step 2).

All terminal output is ASCII.
"""

import os
import sys
import time
import hashlib
import importlib.util

import numpy as np


THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# online_reloc/ and reloc/ are siblings under sonar_slam/scripts/
DEFAULT_RELOC_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "reloc"))


# ------------------------------------------------------------------
# Vendored-integrity guard + hyphenated-module loader
# ------------------------------------------------------------------

def _sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def assert_vendored_checksums(reloc_dir):
    """Abort unless every file in CHECKSUMS.sha256 matches byte-for-byte.

    This proves the vendored authoritative kernel has not been altered, which is a
    precondition for any bit-exact claim.
    """
    checks_path = os.path.join(reloc_dir, "CHECKSUMS.sha256")
    if not os.path.exists(checks_path):
        raise FileNotFoundError("CHECKSUMS.sha256 not found: %s" % checks_path)

    expected = {}
    with open(checks_path, "r") as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            parts = ln.split()
            if len(parts) != 2:
                raise ValueError("bad CHECKSUMS line: %r" % ln)
            digest, name = parts[0], parts[1]
            expected[name] = digest

    for name, digest in sorted(expected.items()):
        target = os.path.join(reloc_dir, name)
        if not os.path.exists(target):
            raise FileNotFoundError("vendored file missing: %s" % target)
        actual = _sha256_file(target)
        if actual != digest:
            raise RuntimeError(
                "VENDORED FILE MODIFIED (checksum mismatch): %s\n"
                "  expected: %s\n  actual:   %s" % (name, digest, actual)
            )
    return expected


def _load_hyphenated_module(mod_name, path):
    """Import a .py whose filename is not a valid identifier (e.g. bbs-1.py)."""
    spec = importlib.util.spec_from_file_location(mod_name, path)
    if spec is None or spec.loader is None:
        raise ImportError("cannot build import spec for %s" % path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


def load_vendored_bbs(reloc_dir=None, verify_checksums=True):
    reloc_dir = os.path.abspath(reloc_dir or DEFAULT_RELOC_DIR)
    if verify_checksums:
        assert_vendored_checksums(reloc_dir)
    return _load_hyphenated_module("bbs_reloc_vendored",
                                   os.path.join(reloc_dir, "bbs-1.py"))


def load_vendored_relocation_auto(reloc_dir=None, verify_checksums=True):
    reloc_dir = os.path.abspath(reloc_dir or DEFAULT_RELOC_DIR)
    if verify_checksums:
        assert_vendored_checksums(reloc_dir)
    return _load_hyphenated_module("relocation_auto_vendored",
                                   os.path.join(reloc_dir, "relocation-auto.py"))


# ------------------------------------------------------------------
# Stateful online stepper
# ------------------------------------------------------------------

class RelocStepper:
    """Online relocalizer: one stop point per step(), incremental state kept across calls.

    __init__ mirrors bbs-1.py main() lines 1954 + 1982:
        np.random.seed(0)                       # exactly once, never re-seeded
        map_data = preprocess_global_map(...)   # persistent, reused every step
    step() mirrors one iteration of main()'s per-point loop body.
    """

    def __init__(self, global_map_path, reloc_dir=None, verify_checksums=True):
        self.reloc_dir = os.path.abspath(reloc_dir or DEFAULT_RELOC_DIR)
        if verify_checksums:
            self.checksums = assert_vendored_checksums(self.reloc_dir)
        self.bbs = load_vendored_bbs(self.reloc_dir, verify_checksums=False)

        # --- RNG init: seed once, right before global-map preprocess, as in main() ---
        np.random.seed(0)
        t0 = time.perf_counter()
        self.map_data = self.bbs.preprocess_global_map(global_map_path)
        self.global_map_prep_sec = time.perf_counter() - t0

        # --- incremental state (mirrors main() locals) ---
        self.idx = 0
        self.T_prev_map = None
        self.T_odom_prev = None
        self.prev_point_id = None

        # batch-shaped records so we can reuse the vendored CSV writer verbatim
        self.results = []

    def step(self, scan_path, odom_row):
        """Process exactly one stop point; keep incremental state; return a fix dict.

        odom_row is a dict from bbs.parse_points_index_csv (keys: point_id, x, y,
        yaw_deg, yaw_rad, raw) read from the rad->deg CONVERTED csv.
        """
        bbs = self.bbs
        t0 = time.perf_counter()

        point_id = odom_row["point_id"]

        # (1) local scan preprocess -- the SINGLE np.random draw for this point
        scan_data = bbs.preprocess_local_scan(scan_path)

        # (2) odom pose from converted csv row (no RNG)
        T_odom_i = bbs.odom_row_to_T(odom_row)

        if self.idx == 0:
            # first point: global BBS + ICP
            result = bbs.relocalize_first_scan_global(
                scan_data=scan_data,
                map_data=self.map_data,
            )
            T_i_map = result["T"]
            T_pred = None
            is_first = True
        else:
            # subsequent points: incremental odom prediction + local BBS + ICP
            if self.T_prev_map is None or self.T_odom_prev is None:
                raise RuntimeError(
                    "previous point not localized; cannot run incremental prediction")
            T_pred, _odom_debug = bbs.predict_pose_from_odom_incremental(
                T_prev_map=self.T_prev_map,
                T_odom_prev=self.T_odom_prev,
                T_odom_i=T_odom_i,
                yaw_delta_sign=bbs.ODOM_YAW_DELTA_SIGN,  # vendored default (1.0), not overridden
            )
            result = bbs.relocalize_scan_local(
                scan_data=scan_data,
                map_data=self.map_data,
                T_pred=T_pred,
            )
            T_i_map = result["T"]
            is_first = False

        # update incremental state (mirrors main(): next point predicts from THIS point)
        self.T_prev_map = T_i_map
        self.T_odom_prev = T_odom_i
        self.prev_point_id = point_id
        self.idx += 1

        # accumulate a batch-shaped record for the vendored CSV writer
        self.results.append({
            "point_id": point_id,
            "T_map": T_i_map,
            "T_pred": T_pred,
            "T_odom": T_odom_i,
            "result": result,
            "scan_path": scan_path,
        })

        elapsed = time.perf_counter() - t0

        x, y, yaw = bbs.params_from_T_2d(T_i_map)
        icp = result["icp"]
        if T_pred is not None:
            px, py, pyaw = bbs.params_from_T_2d(T_pred)
            pred_x = float(px)
            pred_y = float(py)
            pred_yaw_deg = float(np.degrees(pyaw))
        else:
            pred_x = pred_y = pred_yaw_deg = float("nan")

        fix = {
            "point_id": point_id,
            "x_map": float(x),
            "y_map": float(y),
            "yaw_map_deg": float(np.degrees(yaw)),
            "yaw_map_rad": float(yaw),
            "icp_success": bool(icp["success"]),
            "icp_fitness": float(icp["fitness"]),
            "icp_rmse": float(icp["rmse"]),
            "icp_corr": int(icp["num_corr"]),
            "final_score": float(result["final_score"]),
            "pred_x_map": pred_x,
            "pred_y_map": pred_y,
            "pred_yaw_map_deg": pred_yaw_deg,
            "is_first": is_first,
            "elapsed_sec": elapsed,
            "T_map": T_i_map,
            "T_pred": T_pred,
        }
        return fix

    def save_poses_csv(self, out_dir):
        """Write poses_in_map.csv using the vendored writer -> byte-identical format."""
        self.bbs.save_all_poses_csv(self.results, out_dir)
        return os.path.join(out_dir, self.bbs.OUT_ALL_POSES_CSV)


# ------------------------------------------------------------------
# Convenience: run one whole task online (Step1 filter + Step2 convert + stepping)
# ------------------------------------------------------------------

def run_task(global_map_path, maps_dir, points_index_csv, out_dir, reloc_dir=None):
    """Online run of one full task, bit-exact to relocation-auto Step1..Step3 poses.

    Reuses relocation-auto.filter_all_scans (Step 1) and
    relocation-auto.convert_points_index_yaw_to_deg (Step 2) so the filtered scans and
    the converted csv are byte-identical to the batch pipeline. Then steps the bbs-1
    kernel one point at a time.

    Returns dict: poses_csv, fixes (list of fix dicts), filtered_dir, timing.
    """
    reloc_dir = os.path.abspath(reloc_dir or DEFAULT_RELOC_DIR)

    # assert all three vendored files untouched BEFORE using any of them
    assert_vendored_checksums(reloc_dir)
    reloc_auto = load_vendored_relocation_auto(reloc_dir, verify_checksums=False)

    os.makedirs(out_dir, exist_ok=True)
    filtered_dir = os.path.join(out_dir, "filtered_scans")

    # Step 1: filter raw sonar scans -> scan_1.pcd .. scan_N.pcd (no RNG)
    reloc_auto.filter_all_scans(
        maps_dir=maps_dir,
        filtered_dir=filtered_dir,
        clean_output=True,
    )

    # Step 2: rad -> deg for points_index -> points_index-1.csv (no RNG)
    points_out = os.path.join(out_dir, "points_index-1.csv")
    reloc_auto.convert_points_index_yaw_to_deg(points_index_csv, points_out)

    # Build the stepper: seeds(0) + preprocess_global_map (mirrors main() start)
    stepper = RelocStepper(global_map_path, reloc_dir=reloc_dir, verify_checksums=False)
    bbs = stepper.bbs

    # Parse converted csv (sorted by point_id == batch order; no RNG)
    odom_rows = bbs.parse_points_index_csv(points_out)

    fixes = []
    for row in odom_rows:
        point_id = row["point_id"]
        scan_path = bbs.get_scan_path(filtered_dir, "scan_{point_id}.pcd", point_id)
        if not os.path.exists(scan_path):
            print("[WARN] missing scan, skip point_id=%s: %s" % (point_id, scan_path))
            continue
        fix = stepper.step(scan_path, row)
        fixes.append(fix)
        tag = "GLOBAL-BBS(first)" if fix["is_first"] else "local"
        print("[STEP] point_id=%d  %-16s  x=%.6f y=%.6f yaw=%.6f deg  "
              "fitness=%.6f  final=%.6f  t=%.3fs"
              % (point_id, tag, fix["x_map"], fix["y_map"], fix["yaw_map_deg"],
                 fix["icp_fitness"], fix["final_score"], fix["elapsed_sec"]))

    poses_csv = stepper.save_poses_csv(out_dir)

    return {
        "poses_csv": poses_csv,
        "fixes": fixes,
        "filtered_dir": filtered_dir,
        "points_index_deg_csv": points_out,
        "global_map_prep_sec": stepper.global_map_prep_sec,
    }


# ------------------------------------------------------------------
# CLI for a single task (mainly for manual smoke tests)
# ------------------------------------------------------------------

def _main():
    import argparse
    ap = argparse.ArgumentParser(description="Online reloc stepper - single task")
    ap.add_argument("--global-map", required=True)
    ap.add_argument("--maps-dir", required=True)
    ap.add_argument("--points-index", required=True, help="RAW points_index.csv (yaw in rad)")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--reloc-dir", default=None)
    args = ap.parse_args()

    r = run_task(
        global_map_path=args.global_map,
        maps_dir=args.maps_dir,
        points_index_csv=args.points_index,
        out_dir=args.out_dir,
        reloc_dir=args.reloc_dir,
    )
    print("\n[DONE] poses_in_map.csv -> %s" % r["poses_csv"])
    print("[DONE] global-map preprocess: %.3fs" % r["global_map_prep_sec"])
    n = len(r["fixes"])
    if n:
        first = r["fixes"][0]["elapsed_sec"]
        rest = [f["elapsed_sec"] for f in r["fixes"][1:]]
        print("[TIME] first point (global BBS): %.3fs" % first)
        if rest:
            print("[TIME] subsequent points (local): mean=%.3fs min=%.3fs max=%.3fs"
                  % (sum(rest) / len(rest), min(rest), max(rest)))


if __name__ == "__main__":
    _main()
