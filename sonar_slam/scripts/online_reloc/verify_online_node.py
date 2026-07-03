#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Verification harness for the ONLINE NODE CORE (online_reloc_core.OnlineRelocProcessor).

Unlike verify_online_reloc.py (which drives reloc_stepper.run_task = BATCH filter over the whole
maps/ dir), this drives the per-stop-point online path the ROS node uses:
  - read_odom_rows(task_dir)  (vendored rad->deg convert + parse, whole points_index)
  - for each ready point in ascending point_id: localize(raw_scan, odom_row)
      = filter THIS ONE raw scan in an isolated temp dir (vendored filter_all_scans) -> scan_1.pcd
      + RelocStepper.step(scan_1, odom_row)
  - save_poses_csv -> poses_in_map.csv

It proves the online-specific change (per-point isolated filtering + file-driven odom rows +
exactly-once ascending stepping) is byte-identical to the sonar-vendor GOLDEN (and, with
--with-stepper, to the same-platform batch stepper, isolating just the filter-path change).

Also sanity-checks the /reloc/fix payload (fix_to_reloc_list) is a well-formed 7-float vector
per point with point_id/map pose matching the saved poses.

Run inside pool-reloc:repro38 with the phase0 mount layout:
  -v <vendor data root>:/deliv  -v <sonar_slam/scripts>:/scripts  -v <scratch>:/out
  python3 -u /scripts/online_reloc/verify_online_node.py            # online vs golden
  python3 -u /scripts/online_reloc/verify_online_node.py --with-stepper   # + stepper cross-check
ASCII output only.
"""

import os
import sys
import time
import shutil
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reloc_stepper as rs                                            # noqa: E402
from online_reloc_core import (                                      # noqa: E402
    OnlineRelocProcessor, list_ready_points, scan_task, fix_to_reloc_list,
)
from verify_online_reloc import (                                    # noqa: E402  (reuse)
    compare_poses, golden_poses_path, TASKS, DELIV, OUT, RELOC_DIR,
)

PIDX_HEADER = ("point_id,deliverable_basename,point_dir,scan_start_wall,scan_end_wall,"
               "x_at_scan,y_at_scan,yaw_at_scan,sonar_frames,sweep_complete,angles_with_echo,"
               "odom_full_seq_start,odom_full_seq_end")


def test_gapped_selection():
    """Fast unit test (no BBS): a NON-PRISTINE session (empty '(none)' point + aborted
    sweep_complete=false point) must split into the right ready/nonpristine sets, so the node
    skips them AND flags the bit-exact break -- never silently. This locks the behavior the
    4 pristine golden tasks can never exercise (the HIGH from all 3 reviewers). Returns True/False.
    """
    td = tempfile.mkdtemp(prefix="gapped_task_")
    try:
        maps = os.path.join(td, "maps")
        os.makedirs(maps)
        rows = [                                             # (pid, basename, sweep_complete, make_pcd)
            (1, "phase_multi_submap_1_pos1", "true",  True),   # ready
            (2, "phase_multi_submap_2_pos2", "true",  True),   # ready
            (3, "(none)",                    "true",  False),  # empty     -> nonpristine
            (4, "phase_multi_submap_4_pos4", "true",  True),   # ready (pos4 with pos3 gone)
            (5, "phase_multi_submap_5_pos5", "false", True),   # aborted   -> nonpristine
            (6, "phase_multi_submap_6_pos6", "true",  False),  # real basename, pcd LOST, later pt exists -> lost_cloud
            (7, "phase_multi_submap_7_pos7", "true",  True),   # ready
            (8, "phase_multi_submap_8_pos8", "true",  False),  # real basename, pcd missing + NEWEST -> pending (silent)
        ]
        for pid, base, comp, make in rows:
            if make:
                open(os.path.join(maps, base + ".pcd"), "wb").close()
        with open(os.path.join(td, "points_index.csv"), "w", encoding="utf-8") as f:
            f.write(PIDX_HEADER + "\n")
            for pid, base, comp, _mk in rows:
                f.write("%d,%s,%s/point_%02d,0.0,1.0,0.0,0.0,0.0,10,%s,3,0,1\n"
                        % (pid, base, td, pid, comp))
        ready, nonp = scan_task(td)
        ready_ids = [p for p, _, _ in ready]
        nonp_set = set(nonp)
        nonp_pids = {p for p, _ in nonp}
        ok = (ready_ids == [1, 2, 4, 7]
              and (3, "empty") in nonp_set
              and (5, "aborted") in nonp_set
              and (6, "lost_cloud") in nonp_set        # mid-sequence pcd loss must be FLAGGED
              and 8 not in nonp_pids)                  # newest pcd-missing is silent pending (batch-consistent)
        print("\n[gapped-selection unit test]")
        print("  ready ids   : %s   (expect [1, 2, 4, 7])" % ready_ids)
        print("  nonpristine : %s" % sorted(nonp, key=lambda t: (t[0] or 0)))
        print("               (expect (3,empty) (5,aborted) (6,lost_cloud); pid 8 SILENT pending)")
        print("  RESULT: %s" % ("PASS" if ok else "FAIL"))
        return ok
    finally:
        shutil.rmtree(td, ignore_errors=True)


def run_online(task, map_rel):
    task_dir = os.path.join(DELIV, "manual_collect", task)
    global_map = os.path.join(DELIV, map_rel)
    out_dir = os.path.join(OUT, task, "online_node")
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print("\n" + "=" * 70)
    print("ONLINE-NODE CORE  task=%s  map=%s" % (task, map_rel))
    print("=" * 70)

    t0 = time.perf_counter()
    proc = OnlineRelocProcessor(global_map, reloc_dir=RELOC_DIR)     # seed(0)+preprocess+checksum
    odom_rows = proc.read_odom_rows(task_dir)
    ready = list_ready_points(task_dir)                             # ascending point_id
    print("  ready points: %s" % [pid for pid, _, _ in ready])

    fixes = []
    reloc_msgs = []
    for pid, raw, _row in ready:                                    # exactly once, ascending
        odom_row = odom_rows.get(pid)
        if odom_row is None:
            print("  [WARN] point %d in maps but missing from points_index parse; skipped" % pid)
            continue
        ts = time.perf_counter()
        fix = proc.localize(raw, odom_row)
        dt = time.perf_counter() - ts
        msg = fix_to_reloc_list(fix, odom_row)
        reloc_msgs.append(msg)
        fixes.append(fix)
        tag = "GLOBAL-BBS" if fix.get("is_first") else "local"
        print("  [STEP] point=%d %-11s x=%.6f y=%.6f yaw=%.6f rad  fit=%.4f  t=%.2fs"
              % (pid, tag, fix["x_map"], fix["y_map"], fix["yaw_map_rad"], fix["icp_fitness"], dt))

    poses_csv = proc.save_poses_csv(out_dir)

    # sanity-check the /reloc/fix payloads
    msg_ok = _check_reloc_msgs(reloc_msgs, fixes)

    return {
        "poses_csv": poses_csv, "fixes": fixes, "reloc_msgs": reloc_msgs,
        "msg_ok": msg_ok, "wall_sec": time.perf_counter() - t0,
        "prep_sec": proc.stepper.global_map_prep_sec,
    }


def _check_reloc_msgs(msgs, fixes):
    """Every /reloc/fix must be 7 finite floats; [0]=point_id, [1..3]=map pose == fix; carries
    the odom anchor in [4..6]."""
    import math
    ok = True
    for m, f in zip(msgs, fixes):
        if len(m) != 7:
            print("  [MSG-FAIL] point %s: len=%d != 7" % (f.get("point_id"), len(m)))
            ok = False
            continue
        if any((v != v) or math.isinf(v) for v in m):
            print("  [MSG-FAIL] point %s: non-finite in %s" % (f.get("point_id"), m))
            ok = False
        if int(round(m[0])) != int(f["point_id"]):
            print("  [MSG-FAIL] point %s: msg pid %s mismatch" % (f.get("point_id"), m[0]))
            ok = False
        for k, col in ((1, "x_map"), (2, "y_map"), (3, "yaw_map_rad")):
            if abs(m[k] - float(f[col])) > 0.0:
                print("  [MSG-FAIL] point %s: msg[%d]=%r != fix %s=%r"
                      % (f.get("point_id"), k, m[k], col, f[col]))
                ok = False
    print("  reloc-msg check: %s (%d msgs, all 7-float, pose matches fix)"
          % ("OK" if ok else "FAIL", len(msgs)))
    return ok


def main():
    with_stepper = ("--with-stepper" in sys.argv)

    print("Vendored checksum assertion (all 3 files):")
    ck = rs.assert_vendored_checksums(RELOC_DIR)
    for name, dg in sorted(ck.items()):
        print("  OK  %s  %s" % (dg[:16], name))

    unit_ok = test_gapped_selection()      # non-pristine selection/warn behavior (no BBS)

    summary = []
    for task, map_rel in TASKS:
        entry = {"task": task, "map": map_rel}
        try:
            r = run_online(task, map_rel)
        except Exception as e:
            import traceback
            traceback.print_exc()
            entry["error"] = "online core failed: %s" % e
            summary.append(entry)
            continue

        entry["n_points"] = len(r["fixes"])
        entry["msg_ok"] = r["msg_ok"]
        gp = golden_poses_path(task)
        entry["cmp_vs_golden"] = compare_poses(
            r["poses_csv"], gp, "%s : ONLINE-NODE vs GOLDEN" % task)

        if with_stepper:
            sr = rs.run_task(
                global_map_path=os.path.join(DELIV, map_rel),
                maps_dir=os.path.join(DELIV, "manual_collect", task, "maps"),
                points_index_csv=os.path.join(DELIV, "manual_collect", task, "points_index.csv"),
                out_dir=os.path.join(OUT, task, "stepper"),
                reloc_dir=RELOC_DIR,
            )
            entry["cmp_vs_stepper"] = compare_poses(
                r["poses_csv"], sr["poses_csv"],
                "%s : ONLINE-NODE vs STEPPER(batch-filter, same platform)" % task)

        summary.append(entry)

    # ---------------- consolidated ----------------
    print("\n\n" + "#" * 70)
    print("# CONSOLIDATED RESULT (online node core)")
    print("#" * 70)
    print("  gapped-selection unit test: %s" % ("PASS" if unit_ok else "FAIL"))
    all_pass = bool(unit_ok)
    for e in summary:
        print("\n== %s (map=%s) ==" % (e["task"], e.get("map")))
        if "error" in e:
            print("  ERROR: %s" % e["error"])
            all_pass = False
            continue
        print("  points: %d   reloc-msg well-formed: %s" % (e["n_points"], e["msg_ok"]))
        g = e.get("cmp_vs_golden", {})
        print("  vs GOLDEN : %s" % g.get("verdict", "MISSING"))
        if not g.get("ok"):               # authoritative: also fails on missing/rowcount/header
            all_pass = False
        if not e["msg_ok"]:
            all_pass = False
        s = e.get("cmp_vs_stepper")
        if s:
            print("  vs STEPPER: %s" % s.get("verdict", "MISSING"))
            # same platform -> must be 0 diff cells (incl final_score) AND present/aligned
            if (not s.get("exists") or s.get("diff_cols")
                    or not s.get("rowcount_ok") or not s.get("header_ok")):
                all_pass = False

    print("\n" + "#" * 70)
    print("# OVERALL: %s" % ("PASS - online node core is bit-exact to golden"
                             if all_pass else "FAIL - see diffs above"))
    print("#" * 70)
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
