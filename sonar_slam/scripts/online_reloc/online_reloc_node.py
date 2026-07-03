#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer A ONLINE relocalization ROS node (manual-collect mode).

Watches a manual_point_collector task dir; as each stop point is fully scanned, localizes it
with the bit-exact vendored kernel (online_reloc_core / RelocStepper) and publishes /reloc/fix
(7-element Float64MultiArray) for the Layer B display (g-onlineviz-0702). The display also
subscribes the robot's live /wheel_odom, so it shows a low-latency live pose + a trajectory
that snaps onto each fix.

NON-INVASIVE: does not modify manual_point_collector or the frozen reloc/ kernel. It only READS
the collector's output files (maps/*.pcd + points_index.csv) and PUBLISHES /reloc/fix. It
subscribes to nothing, so nothing perturbs numpy's global RNG (a bit-exact precondition).

BIT-EXACT SCOPE (see online_reloc/VERIFY.md): the online poses are byte-identical to the sonar
vendor's offline GOLDEN *only for a PRISTINE session* -- every point sweep_complete AND delivered
a cloud, contiguous point_ids. On a NON-PRISTINE session (an aborted/timed-out sweep, or an
empty-cloud '(none)' point) the online node pairs each scan with its own point by basename (the
MORE correct behavior), whereas the offline batch positionally renumbers scans and would
include/mispair those rows -- so the two diverge from that point on. The node does NOT silently
diverge: it logs a loud ASCII "BIT-EXACT-TO-GOLDEN BROKEN from point N" so the offline audit knows
where equivalence ends. Points are localized exactly once, in ascending point_id (== arrival) order.

FIRST-FIX LATENCY: the first point runs a global BBS (~100 s on x86; several minutes on Orange
Pi 5 ARM). It BLOCKS by design; the robot is parked while scanning. Do NOT wrap this node in a
watchdog/respawn that would kill it, and do NOT change BBS params to speed it up (breaks bit-exact).

Terminal output ASCII only.
"""

import os
import time

import rospy
from std_msgs.msg import Float64MultiArray

from online_reloc_core import (
    OnlineRelocProcessor,
    scan_task,
    fix_to_reloc_list,
)

MAX_ODOM_RETRY = 8      # polls to wait for a ready point's odom row before treating it as dropped
MAX_PREP_RETRY = 1      # retries for the RNG-FREE scan prep (filter) on a transient IO error. The
                        # step() that follows draws np.random and is NEVER retried (a retry would
                        # re-draw and silently desync from golden) -> any step failure flags a break.


def _env_versions():
    """ASCII 'numpy=x, open3d=y' for the startup banner (bit-exact depends on these matching
    repro38: numpy 1.24.4 / open3d 0.19.0). Best-effort; never fatal."""
    out = []
    for mod in ("numpy", "open3d"):
        try:
            out.append("%s=%s" % (mod, __import__(mod).__version__))
        except Exception:
            out.append("%s=?" % mod)
    return ", ".join(out)


class OnlineRelocNode:
    def __init__(self):
        # explicit task dir wins; else auto-pick the ACTIVE session under watch_root (resolved
        # lazily so the node can be launched BEFORE the operator starts collecting).
        self.task_dir = (rospy.get_param("~task_dir", "") or "").strip()
        self.watch_root = os.path.expanduser(
            rospy.get_param("~watch_root", "~/sonar_logs/manual_collect"))
        global_map = (rospy.get_param("~global_map", "") or "").strip()
        reloc_dir = rospy.get_param("~reloc_dir", "") or None
        fix_topic = rospy.get_param("~fix_topic", "/reloc/fix")
        self.poll_hz = max(0.2, float(rospy.get_param("~poll_hz", 2.0)))
        # wall clock (compared against filesystem getmtime); NOT rospy time, which would be sim
        # time under use_sim_time and is not comparable to a file mtime.
        self.start_time = time.time()

        # fail FAST + CLEAR on a missing global map instead of an opaque open3d error deep in BBS.
        if not global_map:
            rospy.logfatal("online_reloc: ~global_map is REQUIRED (the reloc localization map "
                           "global.pcd) -- none given. Aborting.")
            raise SystemExit(2)
        if not os.path.isfile(global_map) or os.path.getsize(global_map) <= 0:
            rospy.logfatal("online_reloc: ~global_map not a readable non-empty file: %s", global_map)
            raise SystemExit(2)

        rospy.loginfo("online_reloc: env %s (bit-exact-to-golden needs numpy=1.24.4, open3d=0.19.0)",
                      _env_versions())
        rospy.loginfo("online_reloc: loading global map + seeding kernel (checksum-guarded): %s",
                      global_map)
        # seeds(0) once + preprocess_global_map once + asserts vendored checksums
        self.proc = OnlineRelocProcessor(global_map, reloc_dir=reloc_dir)
        rospy.loginfo("online_reloc: kernel ready (global-map prep %.2fs). First point will run "
                      "GLOBAL BBS (may take minutes on ARM) -- keep robot parked, do not kill.",
                      self.proc.stepper.global_map_prep_sec)

        self.pub_fix = rospy.Publisher(fix_topic, Float64MultiArray, queue_size=10)
        self.processed = set()          # point_ids already localized (exactly-once guard)
        self._odom_retry = {}           # pid -> polls waited for its odom row
        self._prep_retry = {}           # pid -> RNG-free scan-prep retries spent
        self._warned_np = set()         # nonpristine pids already logged (log once each)
        self._chain_broken = False      # set once any divergence-from-golden is logged
        self._waiting_logged = False
        if self.task_dir:
            rospy.loginfo("online_reloc: watching explicit task_dir=%s", self.task_dir)
        else:
            rospy.loginfo("online_reloc: no ~task_dir; will auto-pick the ACTIVE task_* under %s "
                          "(one whose points_index.csv is written after node start). Pass "
                          "~task_dir explicitly to avoid any ambiguity.", self.watch_root)

    # ---- task-dir resolution: never latch onto a STALE prior session ----
    def _newest_active_task_dir(self):
        """Newest task_* under watch_root whose points_index.csv was touched AFTER node start
        (i.e. the live session, not a leftover completed one). None while none is active yet."""
        root = self.watch_root
        if not root or not os.path.isdir(root):
            return None
        best, best_m = None, -1.0
        try:
            names = os.listdir(root)
        except OSError:
            return None
        for name in names:
            p = os.path.join(root, name)
            if not (name.startswith("task_") and os.path.isdir(p)):
                continue
            pidx = os.path.join(p, "points_index.csv")
            try:
                m = os.path.getmtime(pidx)
            except OSError:
                continue                                   # no index yet (or vanished) -> not active
            if m <= self.start_time:
                continue                                   # not touched since launch -> stale session
            if m > best_m:
                best, best_m = p, m
        return best

    def _resolve_task_dir(self):
        if self.task_dir:
            return self.task_dir
        td = self._newest_active_task_dir()
        if td:
            self.task_dir = td                             # latch the confirmed-active session
            rospy.logwarn("online_reloc: auto-picked ACTIVE task_dir=%s (points_index newer than "
                          "node start). If this is not the live session, relaunch with ~task_dir.",
                          td)
        elif not self._waiting_logged:
            rospy.loginfo("online_reloc: waiting for a collection session to start under %s ...",
                          self.watch_root)
            self._waiting_logged = True
        return self.task_dir

    def _warn_nonpristine(self, nonpristine):
        """Loudly (once per point) announce a row that breaks bit-exact-to-golden, so a skipped
        aborted/empty point is never silent and the offline audit knows where equivalence ends."""
        for pid, reason in nonpristine:
            key = (pid, reason)
            if key in self._warned_np:
                continue
            self._warned_np.add(key)
            self._chain_broken = True
            rospy.logerr("online_reloc: NON-PRISTINE point %s (%s) -> BIT-EXACT-TO-GOLDEN BROKEN "
                         "from here: an offline batch re-run would include/mispair this row; the "
                         "live poses are still self-consistent but NO LONGER byte-equal to golden.",
                         str(pid), reason)

    def poll(self):
        task_dir = self._resolve_task_dir()
        if not task_dir:
            return
        ready, nonpristine = scan_task(task_dir)
        if nonpristine:
            self._warn_nonpristine(nonpristine)
        odom_rows = None                                   # read lazily only if there is new work
        for pid, rp, _row in ready:                        # ascending point_id (scan_task sorts)
            if pid in self.processed:
                continue
            if odom_rows is None:
                odom_rows = self.proc.read_odom_rows(task_dir)   # {point_id -> odom_row}
            odom_row = odom_rows.get(pid)
            if odom_row is None:
                # keep STRICT ascending order: wait on THIS lowest new pid (do not skip ahead, which
                # would desync the RNG/T_prev chain). Bounded so a permanently-dropped row (batch's
                # parse would drop it too) can't wedge the session forever.
                n = self._odom_retry.get(pid, 0) + 1
                self._odom_retry[pid] = n
                if n < MAX_ODOM_RETRY:
                    return                                 # retry next poll
                self._chain_broken = True                  # defense-in-depth: skipping a row a batch
                rospy.logerr("online_reloc: point %d ready but has no parseable odom row after %d "
                             "polls -> skipping. A batch's parse drops the row too, but if its "
                             "pos_id != point_id later points could mispair -> flag BROKEN.", pid, n)
                self.processed.add(pid)
                continue
            if not self.processed:
                rospy.loginfo("online_reloc: point %d -> GLOBAL BBS (first fix; blocks, be patient)",
                              pid)
            # (1) prepare the scan = RNG-FREE filter -> retryable on a transient IO error.
            try:
                scan1, tmpdir = self.proc.prepare_scan(rp)
            except Exception as e:
                n = self._prep_retry.get(pid, 0) + 1
                self._prep_retry[pid] = n
                if n <= MAX_PREP_RETRY:
                    rospy.logwarn("online_reloc: scan prep error on point %d (%s): %s -- retry %d/%d "
                                  "next poll.", pid, os.path.basename(rp), e, n, MAX_PREP_RETRY)
                    return                                 # RNG-free -> safe to retry; order preserved
                self._chain_broken = True
                rospy.logerr("online_reloc: scan prep FAILED on point %d (%s): %s -> skipping. Point "
                             "not localized; poses now DIVERGE from a batch that could filter it -> "
                             "BROKEN from here.", pid, os.path.basename(rp), e)
                self.processed.add(pid)
                continue
            # (2) step ONCE = draws np.random + advances state = the point of no return. NEVER retry
            #     (a retry would re-draw and silently desync from golden). Mark processed the instant
            #     step() returns -- BEFORE any fallible publish -- so a publish error cannot trigger
            #     a re-step (double RNG draw / duplicate fix).
            try:
                fix = self.proc.step_prepared(scan1, odom_row)
            except Exception as e:
                self._chain_broken = True
                rospy.logerr("online_reloc: localize step FAILED on point %d (%s): %s -> skipping. "
                             "BIT-EXACT CHAIN BROKEN from here (RNG/T_prev may have advanced); "
                             "remaining poses NOT guaranteed vs golden.",
                             pid, os.path.basename(rp), e)
                self.processed.add(pid)
                continue
            finally:
                self.proc.cleanup(tmpdir)
            self.processed.add(pid)                        # step() succeeded -> point consumed
            try:
                self.pub_fix.publish(Float64MultiArray(data=fix_to_reloc_list(fix, odom_row)))
                rospy.loginfo("online_reloc: FIX point=%d %-12s map=(%.3f,%.3f,%.3f rad) fitness=%.3f "
                              "final=%.3f t=%.1fs -> /reloc/fix",
                              pid, "GLOBAL-BBS" if fix.get("is_first") else "local",
                              fix["x_map"], fix["y_map"], fix["yaw_map_rad"],
                              fix["icp_fitness"], fix["final_score"], fix["elapsed_sec"])
            except Exception as e:
                # point already consumed (RNG chain intact); we only fail to emit its /reloc/fix.
                rospy.logerr("online_reloc: point %d localized OK but publish/log failed: %s (point "
                             "consumed, its /reloc/fix dropped; RNG chain unaffected).", pid, e)

    def spin(self):
        # single-threaded manual loop (NO ros Timer thread) so nothing runs concurrently with
        # step() and perturbs numpy's global RNG between points. poll() is fully guarded so a
        # transient FS/CSV error logs and retries rather than killing the node (there is no
        # respawn by design -- a respawn would restart the multi-minute first-fix BBS).
        rate = rospy.Rate(self.poll_hz)
        while not rospy.is_shutdown():
            try:
                self.poll()
            except Exception as e:                         # keep the node alive on any transient
                rospy.logerr("online_reloc: poll() error (transient, will retry): %s", e)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


def main():
    rospy.init_node("online_reloc")
    OnlineRelocNode().spin()


if __name__ == "__main__":
    main()
