#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer A ONLINE relocalization CORE (ROS-agnostic).

Wraps the bit-exact vendored kernel (via reloc_stepper.RelocStepper) so a live node can
localize ONE stop point at a time as manual_point_collector produces it, while staying
byte-identical to the offline batch / sonar-vendor golden.

WHY ROS-AGNOSTIC
----------------
The rospy node (online_reloc_node.py) is a thin wrapper over this class. Keeping the
bit-exact-critical logic here lets the verification harness prove byte-identity to golden
inside the pool-reloc:repro38 container (open3d 0.19.0, no ROS installed).

PER-STOP-POINT PIPELINE (mirrors relocation-auto Step1..Step3, fed one point at a time)
--------------------------------------------------------------------------------------
  read_odom_rows(task_dir): convert the live points_index.csv (yaw rad->deg via the vendored
    converter) and parse it -> {point_id: odom_row}. Reusing the vendored converter + parser
    reproduces the batch's rad->deg(.12f)->deg2rad round-trip exactly.
  localize(raw_scan_path, odom_row): filter THIS raw scan in an isolated temp dir with the
    vendored filter_all_scans (single file -> scan_1.pcd; per-file, no cross-scan state, no
    numpy RNG), then RelocStepper.step(scan_1.pcd, odom_row).

BIT-EXACT CONTRACT (see online_reloc/VERIFY.md + the reloc kernel):
  - one RelocStepper per session; seed(0) once in its __init__; never re-created mid-run.
  - localize() called exactly ONCE per point, in ascending point_id == arrival order.
  - nothing else in the process may draw from numpy's global RNG between steps (RNG isolation).
  - raw scan must be a binary PCD with a real intensity field.
Terminal output ASCII only.
"""

import os
import csv
import shutil
import tempfile

# reloc_stepper is a sibling module in online_reloc/
from reloc_stepper import (
    RelocStepper,
    load_vendored_relocation_auto,
    assert_vendored_checksums,
    DEFAULT_RELOC_DIR,
)

NONE_BASENAMES = ("", "(none)", "none")


def _maps_dir(task_dir):
    return os.path.join(task_dir, "maps")


def points_index_path(task_dir):
    return os.path.join(task_dir, "points_index.csv")


def raw_scan_path(task_dir, deliverable_basename):
    """Absolute path of one stop point's raw scan pcd (manual_point_collector output)."""
    return os.path.join(_maps_dir(task_dir), deliverable_basename + ".pcd")


def _parse_pid(row):
    """Parse point_id EXACTLY like the vendored parse_points_index_csv (int(str(...).strip())),
    so the online 'ready' set and the kernel's odom-row keys agree on which pid a row is (avoids
    a '3' vs '3.0' skew between the two parsers -> head-of-line stalls). Returns int or None."""
    try:
        return int(str(row["point_id"]).strip())
    except (KeyError, TypeError, ValueError):
        return None


def scan_task(task_dir):
    """Single pass over points_index.csv -> (ready, nonpristine).

      ready       = [(point_id:int, raw_scan_path:str, row:dict)] ascending by point_id, for points
                    that are fully scanned and whose raw pcd exists:
                      sweep_complete=='true' AND real deliverable_basename AND its pcd exists.

      nonpristine = [(point_id_or_None:int, reason:str)] for rows a BATCH/golden re-run would treat
                    DIFFERENTLY than this online node -- i.e. rows that make the online poses diverge
                    from an offline batch (BIT-EXACT-TO-GOLDEN NO LONGER HOLDS from that point on):
                      'aborted' : sweep_complete!='true' but a real pcd exists. The batch INCLUDES
                                  the row and localizes that scan via positional scan_{point_id};
                                  the online node SKIPS it. Divergence + RNG/T_prev desync onward.
                      'empty'   : deliverable_basename is (none). The batch keeps the row, so
                                  filter_all_scans' sequential scan_1..N renumbering MISPAIRS every
                                  later point (scan_{pid} lands on the wrong physical scan); the
                                  online node skips it. Divergence onward.
                      'lost_cloud': real basename but pcd missing AND a later point exists (write
                                  ordering violated -> cloud lost after the fact). Same batch
                                  mispairing as 'empty'; flagged, not skipped silently.
    A real-basename row whose pcd is merely the NEWEST-and-not-yet-present is NOT nonpristine: it is
    a silent NOT-YET-READY (excluded from both lists, retried next poll) because a batch would also
    skip a missing last scan -> no divergence.
    The online basename-pairing is the MORE correct behavior; nonpristine is surfaced so a skipped
    point is never silent -- the caller logs it loudly and the audit knows where equivalence broke.
    """
    pidx = points_index_path(task_dir)
    if not os.path.exists(pidx):
        return [], []
    with open(pidx, "r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    # newest point = highest parseable point_id (the collector appends rows in ascending order).
    pids = [p for p in (_parse_pid(r) for r in rows) if p is not None]
    last_pid = max(pids) if pids else None

    ready, nonpristine = [], []
    for row in rows:
        pid = _parse_pid(row)
        base = (row.get("deliverable_basename") or "").strip()
        complete = (row.get("sweep_complete") or "").strip().lower() == "true"
        is_none = base.lower() in NONE_BASENAMES
        rp = None if is_none else raw_scan_path(task_dir, base)
        pcd_ok = bool(rp) and os.path.exists(rp)

        if is_none:
            nonpristine.append((pid, "empty"))              # (none) basename -> batch mispairs later points
            continue
        if not pcd_ok:
            # real basename, pcd not on disk. Two very different cases:
            #  - NEWEST row (pid == last_pid): the collector may still be flushing it, or it is the
            #    last point and simply absent -> a batch would ALSO skip a missing last scan, so
            #    skipping is CONSISTENT. Silent NOT-YET-READY (wait); no false alarm, no divergence.
            #  - a LATER point exists (pid < last_pid): the collector's write ordering (pcd closed
            #    BEFORE its row is flushed, ascending) is VIOLATED -> the cloud was LOST after the
            #    fact (deleted / FS loss / hand-edited csv). A batch keeps this row and its sequential
            #    scan_1..N renumbering MISPAIRS every later point -> we MUST flag, never skip silently.
            if pid is not None and last_pid is not None and pid < last_pid:
                nonpristine.append((pid, "lost_cloud"))
            continue
        if not complete:
            nonpristine.append((pid, "aborted"))            # partial cloud -> batch includes, we skip
            continue
        if pid is None:
            nonpristine.append((None, "bad_point_id"))
            continue
        ready.append((pid, rp, row))
    ready.sort(key=lambda t: t[0])
    return ready, nonpristine


def list_ready_points(task_dir):
    """Ready points only (see scan_task). Kept for callers that don't need the nonpristine list."""
    return scan_task(task_dir)[0]


class OnlineRelocProcessor:
    """Bit-exact online relocalizer core. One instance per collection session."""

    def __init__(self, global_map_path, reloc_dir=None, verify_checksums=True):
        self.reloc_dir = os.path.abspath(reloc_dir or DEFAULT_RELOC_DIR)
        if verify_checksums:
            assert_vendored_checksums(self.reloc_dir)          # abort if vendored kernel altered
        # vendored Step1/Step2 helpers (filter_all_scans, convert_points_index_yaw_to_deg)
        self.reloc_auto = load_vendored_relocation_auto(self.reloc_dir, verify_checksums=False)
        # seeds(0) once + preprocess_global_map once (mirrors bbs-1 main() start)
        self.stepper = RelocStepper(global_map_path, reloc_dir=self.reloc_dir,
                                    verify_checksums=False)
        self.bbs = self.stepper.bbs                            # parse_points_index_csv, writer

    # ---- odom rows (reuse vendored rad->deg converter + parser => byte-identical to batch) ----
    def read_odom_rows(self, task_dir):
        """Return {point_id:int -> odom_row dict} from the live points_index.csv.
        Reuses convert_points_index_yaw_to_deg + parse_points_index_csv so yaw goes through
        the exact batch round-trip (rad -> deg .12f -> deg2rad). Processes the whole file each
        call (N is small); only the newly-ready point is stepped by the caller.
        """
        pidx = points_index_path(task_dir)
        if not os.path.exists(pidx):
            return {}
        tmpdir = tempfile.mkdtemp(prefix="online_reloc_idx_")
        try:
            deg_csv = os.path.join(tmpdir, "points_index-1.csv")
            self.reloc_auto.convert_points_index_yaw_to_deg(pidx, deg_csv)
            rows = self.bbs.parse_points_index_csv(deg_csv)
            return {int(r["point_id"]): r for r in rows}
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)

    # ---- localize exactly one stop point, split so the RNG draw is isolated ----
    # RNG SAFETY (see re-review HIGH): prepare_scan is RNG-FREE and idempotent -> safe to RETRY on
    # a transient IO error. step_prepared draws np.random EXACTLY ONCE (the point's only RNG
    # consumption) and advances incremental state -> it is the "point of no return" and MUST NOT be
    # retried; a retry would re-draw and silently desync the stream from the vendor golden. The ROS
    # node calls these two directly so it can retry only the prep and treat any step failure as a
    # (flagged) bit-exact break.
    def prepare_scan(self, raw_scan_pcd):
        """Filter ONE raw scan in an isolated temp dir -> (scan1_path, tmpdir). RNG-FREE; safe to
        retry. The caller MUST call cleanup(tmpdir) exactly once afterwards."""
        tmpdir = tempfile.mkdtemp(prefix="online_reloc_scan_")
        try:
            tmp_maps = os.path.join(tmpdir, "maps")
            os.makedirs(tmp_maps)
            # keep the original basename so collect_raw_scan_files() ranks the lone file cleanly
            shutil.copy2(raw_scan_pcd, os.path.join(tmp_maps, os.path.basename(raw_scan_pcd)))
            tmp_filtered = os.path.join(tmpdir, "filtered")
            self.reloc_auto.filter_all_scans(tmp_maps, tmp_filtered, clean_output=True)
            scan1 = os.path.join(tmp_filtered, "scan_1.pcd")
            if not os.path.exists(scan1):
                raise RuntimeError("filter produced no scan_1.pcd for %s" % raw_scan_pcd)
            return scan1, tmpdir
        except Exception:
            shutil.rmtree(tmpdir, ignore_errors=True)       # never leak a temp dir on prep failure
            raise

    def step_prepared(self, scan1_path, odom_row):
        """RelocStepper.step on an already-filtered scan. Draws np.random ONCE and advances state;
        NOT idempotent -- attempt exactly once per point, never retry. step() ignores the filename,
        so scan_1 naming is harmless."""
        return self.stepper.step(scan1_path, odom_row)

    @staticmethod
    def cleanup(tmpdir):
        shutil.rmtree(tmpdir, ignore_errors=True)

    def localize(self, raw_scan_pcd, odom_row):
        """Convenience prepare + step + cleanup, for OFFLINE verification / pristine batch use
        (no retry needed there). The ROS node does NOT use this -- it calls prepare_scan /
        step_prepared separately so a transient prep failure can be retried without re-drawing the
        RNG in step (see prepare_scan / step_prepared)."""
        scan1, tmpdir = self.prepare_scan(raw_scan_pcd)
        try:
            return self.step_prepared(scan1, odom_row)
        finally:
            self.cleanup(tmpdir)

    def save_poses_csv(self, out_dir):
        """Write poses_in_map.csv via the vendored writer (byte-identical format)."""
        return self.stepper.save_poses_csv(out_dir)


def fix_to_reloc_list(fix, odom_row):
    """Map (fix dict, odom_row) -> the 7-element /reloc/fix payload consumed by the Layer B
    display: [point_id, map_x, map_y, map_yaw_rad, anchor_odom_x, anchor_odom_y, anchor_odom_yaw].
    Anchor odom = the SAME odom pose the localization used for this point (points_index x/y/yaw),
    so it is consistent with how the map pose was derived and lives in the /wheel_odom frame.
    """
    return [
        float(fix["point_id"]),
        float(fix["x_map"]), float(fix["y_map"]), float(fix["yaw_map_rad"]),
        float(odom_row["x"]), float(odom_row["y"]), float(odom_row["yaw_rad"]),
    ]
