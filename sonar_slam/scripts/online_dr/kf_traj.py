#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer B trajectory CORRECTION filter (ROS-agnostic). This is lane 2 of the two-lane display.

Two-lane display (see display_node.py):
  LANE 1 (live, hot path, low latency): the current pose + growing blue tip are pure dr_core
    geometric DR (apply_T of the latest fix similarity). This module is NOT on that path.
  LANE 2 (correction, off hot path): on each (high-latency) localization fix, run a position-only
    linear Kalman forward pass + RTS (Rauch-Tung-Striebel) smoother over the distance-decimated node
    history so the drawn blue trajectory passes through every fix, binding each LATE fix to its PAST
    anchor node (the fix is produced seconds/minutes after the robot left that stop).

Why a plain LINEAR KF (not EKF/UKF): odom is pre-transformed into the map frame by the anchor-derived
similarity sR (rotation R + scale s come from dr_core; this module never estimates the frame). The
position state then evolves linearly and a fix is a linear position measurement (H = I). With ISOTROPIC
process/measurement noise the 2x2 covariance stays p*I2 forever, so the whole filter+smoother reduces to
SCALAR-variance recursions on 2-vectors -- no matrix inverse, closed-form testable.

Q is PER DISTANCE (not per time): the robot sits stationary ~54 s at every sonar scan; per-time Q would
inject variance into stationary intervals and the smoother would bend the line while the robot was parked.

FAITHFULNESS / SAFETY: does NOT modify dr_core or any vendored Layer A code. Reads R and s from the caller.
Position-only (the Path renders as a line strip -- node yaw is invisible in rviz; a DR yaw is carried only
for completeness). Every input is finite-guarded; every emitted coordinate is finite-checked. ASCII only.
"""
import math
import numpy as np


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def coarse_R_from_yaw(map_yaw, odom_yaw):
    """Provisional rigid rotation from a single fix's yaw pair (fix-1 startup, s=1)."""
    th = wrap_angle(float(map_yaw) - float(odom_yaw))
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


class KfTrajParams:
    """Plain container; every field mirrors a launch-exposed ROS param (defaults are data-fit)."""
    def __init__(self, q_xy_per_m=0.012, q_xy_floor=1e-8, sigma_fix_xy=0.02,
                 node_min_step=0.03, max_nodes=20000, init_var=1e6, bind_max_dist=0.5):
        self.q_xy_per_m = float(q_xy_per_m)     # process variance growth per metre travelled (m^2/m)
        self.q_xy_floor = float(q_xy_floor)     # tiny per-node floor so p stays > 0 when stationary
        self.sigma_fix_xy = float(sigma_fix_xy)  # fix (measurement) std; fixes are display-truth -> small
        self.node_min_step = float(node_min_step)  # distance decimation of the node history (m)
        self.max_nodes = int(max_nodes)         # hard safety cap on node count
        self.init_var = float(init_var)         # diffuse prior -> fix-1 backfill is a rigid translation
        self.bind_max_dist = float(bind_max_dist)  # reject a fix whose anchor odom is farther than this from every node (m)


WARN_MAX = 50  # bounded diagnostics ring


class KfTrajectory:
    """Distance-decimated node history + late-binding fixes -> KF forward + RTS smoothed map path."""

    def __init__(self, params=None):
        self.p = params if params is not None else KfTrajParams()
        self.nodes = []       # list of (ox, oy, oyaw) raw odom (map increments recomputed each refilter)
        self.fixes = []       # list of dict(pid, node_idx, mx, my), kept sorted by node_idx
        self._pids = set()
        self.R = None         # 2x2 similarity rotation (supplied by caller; never estimated here)
        self.s = 1.0          # map/odom scale (supplied by caller)
        self.theta_R = 0.0    # atan2(R10, R00), for the (invisible) DR node yaw
        self._smoothed = []   # list of (mx, my, myaw) produced by the last refilter()
        self.warnings = []    # bounded ASCII diagnostics (consumed by the status panel)

    # ---------------- ingestion ----------------
    def add_odom(self, ox, oy, oyaw):
        """Append a distance-decimated node. Returns True iff a node was appended.
        A single non-finite sample is rejected (never poisons the history)."""
        for v in (ox, oy, oyaw):
            if not math.isfinite(v):
                self._warn("DROP odom non-finite")
                return False
        if self.nodes:
            lx, ly, _ = self.nodes[-1]
            if math.hypot(ox - lx, oy - ly) < self.p.node_min_step:
                return False
        self.nodes.append((float(ox), float(oy), float(oyaw)))
        if len(self.nodes) > self.p.max_nodes:
            self._decimate_nodes()
        return True

    def add_fix(self, pid, mx, my, anchor_ox, anchor_oy):
        """Bind a fix to its PAST anchor node (nearest odom). Idempotent on pid. Returns True iff added."""
        for v in (pid, mx, my, anchor_ox, anchor_oy):
            if not math.isfinite(v):
                self._warn("DROP fix non-finite")
                return False
        ipid = int(pid)
        if ipid in self._pids:
            return False
        idx, bind_d = self._nearest_node(anchor_ox, anchor_oy)
        if idx is None:
            self._warn("DROP fix no node to bind")
            return False
        if bind_d > self.p.bind_max_dist:   # stale / re-sent / mis-topic fix far from any node -> do not corrupt the line
            self._warn("DROP fix pid=%d bind too far %.2fm > %.2fm" % (ipid, bind_d, self.p.bind_max_dist))
            return False
        self._pids.add(ipid)
        self.fixes.append(dict(pid=ipid, node_idx=idx, mx=float(mx), my=float(my)))
        self.fixes.sort(key=lambda f: f["node_idx"])
        return True

    def set_similarity(self, R, s):
        """Set the odom->map rotation and scale (from dr_core est.R/est.s, or the fix-1 coarse R)."""
        if R is None:
            return
        Rn = np.asarray(R, dtype=np.float64)
        if Rn.shape != (2, 2) or not np.all(np.isfinite(Rn)) or not math.isfinite(s):
            self._warn("DROP similarity non-finite")
            return
        self.R = Rn
        self.s = float(s)
        self.theta_R = math.atan2(Rn[1, 0], Rn[0, 0])

    # ---------------- helpers ----------------
    def _warn(self, msg):
        self.warnings.append(msg)
        if len(self.warnings) > WARN_MAX:
            del self.warnings[0:len(self.warnings) - WARN_MAX]

    def _nearest_node(self, ox, oy):
        """Return (index, distance_m) of the nearest node in odom space, preferring the LATEST node on a
        tie (correct bias for late binding when a track approaches itself). (None, inf) if no nodes."""
        if not self.nodes:
            return None, float("inf")
        best, bd = None, None
        for i, (nx, ny, _) in enumerate(self.nodes):
            d = (nx - ox) * (nx - ox) + (ny - oy) * (ny - oy)
            if bd is None or d <= bd:   # <= keeps the later (larger-index) node on an exact tie
                bd, best = d, i
        return best, math.sqrt(bd)

    def _decimate_nodes(self):
        """Safety cap only (never reached in practice). Keep node 0, newest, every fix-bound node;
        halve the rest. Rebind fix node indices to the surviving list."""
        keep = {0, len(self.nodes) - 1}
        for f in self.fixes:
            keep.add(f["node_idx"])
        new_nodes, remap = [], {}
        for i, nd in enumerate(self.nodes):
            if i in keep or (i % 2 == 0):
                remap[i] = len(new_nodes)
                new_nodes.append(nd)
        for f in self.fixes:
            if f["node_idx"] not in remap:
                remap[f["node_idx"]] = len(new_nodes)
                new_nodes.append(self.nodes[f["node_idx"]])
        self.nodes = new_nodes
        for f in self.fixes:
            f["node_idx"] = remap[f["node_idx"]]
        self.fixes.sort(key=lambda f: f["node_idx"])
        self._warn("DECIMATE nodes -> %d" % len(self.nodes))

    # ---------------- filter ----------------
    def refilter(self):
        """Full-history position-only linear KF forward pass + RTS backward smoother.
        Isotropic covariance -> scalar variance p (2x2 == p*I2). Rebuilds self._smoothed.
        Cost O(nodes); run off the hot path (once per fix)."""
        n = len(self.nodes)
        self._smoothed = []
        if n == 0 or self.R is None:
            return
        sR = self.s * self.R
        r = self.p.sigma_fix_xy * self.p.sigma_fix_xy
        fix_by_node = {}
        for f in self.fixes:
            fix_by_node.setdefault(f["node_idx"], []).append(f)

        xpred = [None] * n     # prior mean at node i (2-vector)
        xpost = [None] * n     # posterior mean at node i
        ppred = [0.0] * n      # prior scalar variance at node i
        ppost = [0.0] * n      # posterior scalar variance at node i

        prev = np.array(self.nodes[0][:2], dtype=np.float64)
        for i in range(n):
            cur = np.array(self.nodes[i][:2], dtype=np.float64)
            if i == 0:
                xpred[0] = np.zeros(2, dtype=np.float64)   # arbitrary; washed out by the diffuse prior
                ppred[0] = self.p.init_var
            else:
                dodo = cur - prev
                inc = sR @ dodo
                dist = math.hypot(float(dodo[0]), float(dodo[1]))
                xpred[i] = xpost[i - 1] + inc
                ppred[i] = ppost[i - 1] + self.p.q_xy_per_m * dist + self.p.q_xy_floor
            xi = xpred[i].copy()
            pi = ppred[i]
            for f in fix_by_node.get(i, []):
                z = np.array([f["mx"], f["my"]], dtype=np.float64)
                K = pi / (pi + r)
                xi = xi + K * (z - xi)
                pi = (1.0 - K) * pi
            xpost[i] = xi
            ppost[i] = pi
            prev = cur

        xs = [None] * n
        xs[n - 1] = xpost[n - 1]
        for i in range(n - 2, -1, -1):
            denom = ppred[i + 1]
            G = (ppost[i] / denom) if denom > 0.0 else 0.0
            xs[i] = xpost[i] + G * (xs[i + 1] - xpred[i + 1])

        out = []
        for i in range(n):
            mx, my = float(xs[i][0]), float(xs[i][1])
            if math.isfinite(mx) and math.isfinite(my):
                out.append((mx, my, wrap_angle(self.nodes[i][2] + self.theta_R)))
        self._smoothed = out

    def smoothed_path(self):
        """Return a copy of the last smoothed map path: list of (mx, my, myaw)."""
        return list(self._smoothed)

    @property
    def n_nodes(self):
        return len(self.nodes)

    @property
    def n_fixes(self):
        return len(self.fixes)
