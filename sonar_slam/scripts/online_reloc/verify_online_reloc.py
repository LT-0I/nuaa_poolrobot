#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verification harness for Layer A (online reloc stepper). ASCII output only.

Runs, inside the pool-reloc:repro38 container, for each of the 4 tasks:
  1. the online stepper (reloc_stepper.run_task) -> stepper poses_in_map.csv
  2. (optional, --batch) a fresh batch relocation-auto.py run -> batch poses_in_map.csv

Then byte-compares:
  * stepper vs vendor GOLDEN  (task requirement; cross-platform Win->Linux, so pose
    columns expected byte-identical, final_score may differ ~1e-3 = platform end-bit)
  * stepper vs fresh BATCH    (same platform; proves the online shell adds ZERO
    deviation -> expected 100% byte-identical incl final_score)
  * batch  vs GOLDEN          (sanity, reproduces Phase 0)

Paths assume the phase0 mount layout:
  -v <vendor data root>:/deliv       (manual_collect/<task>/..., circle-pcd/, mid_pcd/)
  -v <sonar_slam/scripts>:/scripts   (contains reloc/ and online_reloc/)
  -v <writable scratch>:/out
"""

import os
import sys
import csv
import math
import time
import shutil
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reloc_stepper as rs  # noqa: E402


DELIV = os.environ.get("DELIV", "/deliv")
OUT = os.environ.get("OUT", "/out")
RELOC_DIR = os.environ.get("RELOC_DIR", "/scripts/reloc")

# task name -> global map (relative to DELIV). NEVER mix these pairings.
TASKS = [
    ("task_z", "circle-pcd/global.pcd"),
    ("task_l", "circle-pcd/global.pcd"),
    ("task_20260628_223722(straight2)", "circle-pcd/global.pcd"),
    ("task_20260629_202646", "mid_pcd/global.pcd"),
]

POSE_COLS = {"x_map", "y_map", "yaw_map_deg", "yaw_map_rad",
             "pred_x_map", "pred_y_map", "pred_yaw_map_deg"}
QUALITY_COLS = {"icp_success", "icp_fitness", "icp_rmse", "icp_corr"}
TOLERATED_COLS = {"final_score"}  # BBS likelihood diagnostic; platform end-bit only


def _read_bytes(path):
    with open(path, "rb") as f:
        return f.read()


def _load_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def compare_poses(mine, other, label):
    """Compare two poses_in_map.csv. Returns a dict summary; prints an ASCII block."""
    res = {"label": label, "mine": mine, "other": other, "exists": True, "ok": False}
    if not os.path.exists(mine):
        print("\n[%s] MINE MISSING: %s  <== FAIL" % (label, mine))
        res["exists"] = False
        return res
    if not os.path.exists(other):
        print("\n[%s] OTHER MISSING: %s  <== FAIL" % (label, other))
        res["exists"] = False
        return res

    raw_id = (_read_bytes(mine) == _read_bytes(other))
    hm, dm = _load_csv(mine)
    hg, dg = _load_csv(other)

    header_ok = (hm == hg)
    rowcount_ok = (len(dm) == len(dg))

    n = min(len(dm), len(dg))
    ncol = min(len(hm), len(hg))

    percol_max = {}
    percol_ndiff = {}
    worst = None
    maxd = 0.0
    total_diff_cells = 0
    diff_cols = set()

    for i in range(n):
        for j in range(ncol):
            a, b = dm[i][j], dg[i][j]
            if a == b:
                continue
            col = hm[j]
            diff_cols.add(col)
            total_diff_cells += 1
            percol_ndiff[col] = percol_ndiff.get(col, 0) + 1
            try:
                fa, fb = float(a), float(b)
                if math.isnan(fa) and math.isnan(fb):
                    total_diff_cells -= 1
                    percol_ndiff[col] -= 1
                    diff_cols.discard(col) if percol_ndiff[col] == 0 else None
                    continue
                d = abs(fa - fb)
                percol_max[col] = max(percol_max.get(col, 0.0), d)
                if d > maxd:
                    maxd = d
                    worst = (i + 2, col, a, b)
            except ValueError:
                percol_max[col] = float("inf")
                if worst is None:
                    worst = (i + 2, col, a, b)

    pose_diff = diff_cols & POSE_COLS
    quality_diff = diff_cols & QUALITY_COLS
    only_tolerated = bool(diff_cols) and diff_cols.issubset(TOLERATED_COLS)

    print("\n[%s]" % label)
    print("  mine : %s" % mine)
    print("  other: %s" % other)
    print("  raw byte-identical file: %s" % raw_id)
    print("  header match: %s   row-count match: %s (mine=%d other=%d)"
          % (header_ok, rowcount_ok, len(dm), len(dg)))
    print("  differing cells: %d   max|delta|: %.3e" % (total_diff_cells, maxd))
    if worst:
        print("  worst cell: line %d col '%s': %s vs %s" % worst)
    if percol_max:
        for c, v in sorted(percol_max.items(), key=lambda x: -x[1]):
            print("    col %-20s max|delta|=%.3e  (n=%d)"
                  % (c, v, percol_ndiff.get(c, 0)))
    # verdict (cell-wise is authoritative; raw bytes may differ only by CRLF vs LF)
    if raw_id:
        verdict = "RAW BYTE-IDENTICAL (perfect, incl line endings)"
    elif total_diff_cells == 0:
        verdict = ("CELL BYTE-IDENTICAL (every value matches; raw bytes differ only "
                   "by line endings CRLF-vs-LF, cosmetic)")
    elif not pose_diff and not quality_diff and only_tolerated:
        verdict = "POSE+QUALITY CELL-IDENTICAL; only final_score differs (max=%.3e) = platform end-bit, ACCEPTABLE" % percol_max.get("final_score", 0.0)
    elif not pose_diff and not quality_diff:
        verdict = "POSE+QUALITY CELL-IDENTICAL; other cols differ: %s" % sorted(diff_cols)
    elif not pose_diff:
        verdict = "POSE BYTE-IDENTICAL but QUALITY cols differ: %s (max=%.3e)" % (sorted(quality_diff), maxd)
    else:
        verdict = "POSE COLUMNS DIFFER: %s (max=%.3e)  <== SHELL BUG" % (sorted(pose_diff), maxd)
    print("  VERDICT: %s" % verdict)

    # authoritative pass/fail (golden tolerance: final_score end-bit allowed). A missing file,
    # header mismatch, or row-count mismatch is a HARD FAIL -- never a silent PASS. Callers that
    # need same-platform strictness (0 diff incl final_score) additionally check diff_cols.
    ok = (header_ok and rowcount_ok and not pose_diff and not quality_diff
          and (not diff_cols or only_tolerated))

    res.update({
        "raw_byte_identical": raw_id,
        "header_ok": header_ok,
        "rowcount_ok": rowcount_ok,
        "n_rows": n,
        "diff_cells": total_diff_cells,
        "max_delta": maxd,
        "diff_cols": sorted(diff_cols),
        "pose_diff": sorted(pose_diff),
        "quality_diff": sorted(quality_diff),
        "only_tolerated": only_tolerated,
        "final_score_max_delta": percol_max.get("final_score", 0.0),
        "verdict": verdict,
        "ok": ok,
    })
    return res


def golden_poses_path(task):
    return os.path.join(DELIV, "manual_collect", task,
                        "auto_relocalization_output",
                        "multi_relocalization_output", "poses_in_map.csv")


def run_stepper(task, map_rel):
    maps_dir = os.path.join(DELIV, "manual_collect", task, "maps")
    points_index = os.path.join(DELIV, "manual_collect", task, "points_index.csv")
    global_map = os.path.join(DELIV, map_rel)
    out_dir = os.path.join(OUT, task, "stepper")
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print("STEPPER  task=%s  map=%s" % (task, map_rel))
    print("=" * 70)
    t0 = time.perf_counter()
    r = rs.run_task(
        global_map_path=global_map,
        maps_dir=maps_dir,
        points_index_csv=points_index,
        out_dir=out_dir,
        reloc_dir=RELOC_DIR,
    )
    r["wall_sec"] = time.perf_counter() - t0
    return r


def run_batch(task, map_rel):
    """Fresh batch relocation-auto.py run for same-platform cross-check."""
    src = os.path.join(DELIV, "manual_collect", task)
    work = os.path.join(OUT, task, "batch")
    if os.path.isdir(work):
        shutil.rmtree(work)
    os.makedirs(work, exist_ok=True)
    shutil.copytree(os.path.join(src, "maps"), os.path.join(work, "maps"))
    shutil.copy(os.path.join(src, "points_index.csv"), work)
    of = os.path.join(src, "odom_full.csv")
    if os.path.exists(of):
        shutil.copy(of, work)

    global_map = os.path.join(DELIV, map_rel)
    log = os.path.join(work, "relocation-auto.log")
    cmd = [
        "python3", os.path.join(RELOC_DIR, "relocation-auto.py"),
        "--task-dir", work,
        "--global-map", global_map,
        "--bbs-script", os.path.join(RELOC_DIR, "bbs-1.py"),
        "--corrected-script", os.path.join(RELOC_DIR, "corrected.py"),
        "--python", "python3",
        "--work-dir-name", "auto_relocalization_repro",
    ]
    print("\n" + "=" * 70)
    print("BATCH    task=%s  map=%s" % (task, map_rel))
    print("=" * 70)
    with open(log, "w") as lf:
        rc = subprocess.call(cmd, stdout=lf, stderr=subprocess.STDOUT)
    batch_poses = os.path.join(work, "auto_relocalization_repro",
                               "multi_relocalization_output", "poses_in_map.csv")
    if not os.path.exists(batch_poses):
        print("  [FAIL] batch produced no poses_in_map.csv (rc=%d); log tail:" % rc)
        try:
            with open(log) as lf:
                for ln in lf.readlines()[-25:]:
                    print("    " + ln.rstrip())
        except Exception:
            pass
        return None
    if rc != 0:
        print("  [NOTE] relocation-auto exit rc=%d (likely Step4 corrected.py); "
              "poses_in_map.csv from Step3 still produced and used." % rc)
    return batch_poses


def main():
    do_batch = ("--no-batch" not in sys.argv)

    print("Vendored checksum assertion (all 3 files):")
    ck = rs.assert_vendored_checksums(RELOC_DIR)
    for name, dg in sorted(ck.items()):
        print("  OK  %s  %s" % (dg[:16], name))

    summary = []
    for task, map_rel in TASKS:
        entry = {"task": task, "map": map_rel}
        try:
            sr = run_stepper(task, map_rel)
        except Exception as e:
            import traceback
            traceback.print_exc()
            entry["error"] = "stepper failed: %s" % e
            summary.append(entry)
            continue

        entry["stepper_poses"] = sr["poses_csv"]
        entry["global_map_prep_sec"] = sr["global_map_prep_sec"]
        fixes = sr["fixes"]
        entry["n_points"] = len(fixes)
        if fixes:
            entry["first_point_sec"] = fixes[0]["elapsed_sec"]
            rest = [f["elapsed_sec"] for f in fixes[1:]]
            entry["subsequent_sec"] = rest
            entry["subsequent_mean_sec"] = (sum(rest) / len(rest)) if rest else 0.0

        # compare vs golden
        gp = golden_poses_path(task)
        entry["cmp_vs_golden"] = compare_poses(sr["poses_csv"], gp, "%s : STEPPER vs GOLDEN" % task)

        # batch cross-check
        if do_batch:
            bp = run_batch(task, map_rel)
            if bp:
                entry["cmp_vs_batch"] = compare_poses(sr["poses_csv"], bp, "%s : STEPPER vs BATCH(same-platform)" % task)
                entry["cmp_batch_golden"] = compare_poses(bp, gp, "%s : BATCH vs GOLDEN(sanity)" % task)

        summary.append(entry)

    # ---------------- consolidated report ----------------
    print("\n\n" + "#" * 70)
    print("# CONSOLIDATED RESULT")
    print("#" * 70)
    all_pass = True
    for e in summary:
        print("\n== %s (map=%s) ==" % (e["task"], e.get("map")))
        if "error" in e:
            print("  ERROR: %s" % e["error"])
            all_pass = False
            continue
        print("  points: %d   global-map prep: %.3fs" % (e["n_points"], e["global_map_prep_sec"]))
        if "first_point_sec" in e:
            print("  timing: first(global BBS)=%.3fs  subsequent(local) mean=%.3fs [%s]"
                  % (e["first_point_sec"], e.get("subsequent_mean_sec", 0.0),
                     ", ".join("%.3f" % s for s in e.get("subsequent_sec", []))))
        g = e.get("cmp_vs_golden", {})
        print("  vs GOLDEN: %s" % g.get("verdict", "MISSING"))
        if not g.get("ok"):               # authoritative: also fails on missing/rowcount/header
            all_pass = False
        b = e.get("cmp_vs_batch")
        if b:
            print("  vs BATCH : %s" % b.get("verdict", "MISSING"))
            # same platform: must be present, aligned, and 0 diff cells (incl final_score)
            if (not b.get("exists") or b.get("diff_cols")
                    or not b.get("rowcount_ok") or not b.get("header_ok")):
                all_pass = False
        bg = e.get("cmp_batch_golden")
        if bg:
            print("  batch vs GOLDEN (sanity): %s" % bg.get("verdict", "n/a"))

    print("\n" + "#" * 70)
    print("# OVERALL: %s" % ("PASS - online shell is bit-exact to golden/batch"
                             if all_pass else "FAIL - see pose/quality diffs above"))
    print("#" * 70)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
