#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer B display DR core (ROS-agnostic). Reuses vendored corrected.py math (import only, NOT modified).
Method (matches sonar-vendor corrected.py online-dynamic + a Layer-B scale enhancement):
  - fixes = truth anchors (from Layer A localization).
  - shared rotation R from the first 2 anchor POSITIONS (Kabsch, proper rotation, NO reflection) -- vendored estimate_shared_R.
  - scale s = map/odom from anchor chord ratios (Layer B enhancement; corrected!=truth so allowed). set use_scale=False for rigid.
  - per-anchor translation so the transform passes through each fix exactly.
  - online apply: map_pose = active_T_map_odom @ odom_pose (causal).
DISPLAY POLICY (user 2026-07-01): NO position until >= min_fixes (=2) fixes. Before that -> ACQUIRING.
Also: if the available anchors are not geometrically separated (degenerate baseline), stay ACQUIRING
(degenerate=True) until a separated fix arrives -- never fabricate a position from a bad transform.
This module is Layer B (iterable display). It must NOT be confused with the frozen Layer A localization core.
FAITHFULNESS: get_anchor_T (sR/t construction) and apply_T are byte-identical to the vendored transform;
rigid mode (use_scale=False) keeps s==1.0 exactly. Only the scale enhancement / gating logic is Layer B.
"""
import importlib.util
import io
import contextlib
import math
import numpy as np

MIN_BASELINE_M = 0.5   # anchor separation threshold: both odom & map chords must exceed this to trust geometry
SCALE_LO = 0.7         # scale physical-prior lower clamp (true prior ~0.90-0.97; loose bound)
SCALE_HI = 1.3         # scale physical-prior upper clamp


def load_corrected(path):
    spec = importlib.util.spec_from_file_location("corrected_vendored", path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class RelocDisplayEstimator:
    def __init__(self, corrected_path, min_fixes=2, use_scale=True):
        self.corr = load_corrected(corrected_path)
        self.min_fixes = max(2, int(min_fixes))   # vendored estimator requires >=2 anchors
        self.use_scale = bool(use_scale)
        self.anchors = []          # list of dicts: point_id, map_x/y, map_yaw_rad, odom_x/y_anchor, odom_yaw_anchor_rad
        self.R = None              # 2x2 shared rotation (fixed once from first 2 separated anchors)
        self.s = 1.0               # map/odom scale
        self.active_T = None       # 3x3 SE2-with-scale for latest anchor
        self.localized = False
        self.degenerate = False    # True when >=min_fixes anchors exist but none separated enough to build R
        self.warnings = []         # ASCII diagnostic strings (consumed by display status panel)

    def on_fix(self, pid, map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw):
        # M-1: reject any non-finite input (a single NaN/Inf would poison R / scale / transform forever).
        for v in (pid, map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw):
            try:
                fv = float(v)
            except (TypeError, ValueError):
                fv = float("nan")
            if not math.isfinite(fv):
                self.warnings.append("DROP fix pid=%s non-finite input" % str(pid))
                return
        ipid = int(pid)
        # J1: idempotent on point_id (display restart / fix_history replay re-sends the same fix).
        for a in self.anchors:
            if a["point_id"] == ipid:
                return
        self.anchors.append(dict(
            point_id=ipid,
            map_x=float(map_x), map_y=float(map_y), map_yaw_rad=float(map_yaw),
            odom_x_anchor=float(odom_x), odom_y_anchor=float(odom_y), odom_yaw_anchor_rad=float(odom_yaw),
        ))
        # H3 / O2: build R only from anchors separated by >= MIN_BASELINE_M (both chords). Degenerate
        # geometry -> stay ACQUIRING (no position) until a separated fix arrives. R once built never changes.
        if self.R is None and len(self.anchors) >= self.min_fixes:
            ids, best = self._separated_anchor_ids()
            if len(ids) >= 2:
                with contextlib.redirect_stdout(io.StringIO()):   # silence vendored prints
                    self.R = self.corr.estimate_shared_R_from_anchor_positions(self.anchors, anchor_ids=ids)[0]
                self.degenerate = False
            else:
                self.degenerate = True
                self.warnings.append(
                    "DEGENERATE baseline %.2fm < %.2fm, waiting next separated fix" % (best, MIN_BASELINE_M))
        if self.R is not None:
            self._update_scale()
            self.active_T = self.get_anchor_T(len(self.anchors) - 1)
            self.localized = True

    def _separated_anchor_ids(self):
        """Greedy from anchor 0: collect anchors whose successive odom AND map chords are both
        >= MIN_BASELINE_M. Returns (first two point_ids of that subset, best min-chord seen for diagnostics).
        Normal geometry (adjacent chords > 0.5m) yields [anchors[0], anchors[1]] == original behaviour."""
        if not self.anchors:
            return [], 0.0
        sep = [self.anchors[0]]
        best = 0.0
        for i in range(1, len(self.anchors)):
            prev, cur = sep[-1], self.anchors[i]
            do = math.hypot(cur["odom_x_anchor"] - prev["odom_x_anchor"],
                            cur["odom_y_anchor"] - prev["odom_y_anchor"])
            dm = math.hypot(cur["map_x"] - prev["map_x"], cur["map_y"] - prev["map_y"])
            best = max(best, min(do, dm))
            if do >= MIN_BASELINE_M and dm >= MIN_BASELINE_M:
                sep.append(cur)
                if len(sep) >= 2:
                    break
        ids = [a["point_id"] for a in sep[:2]]
        return ids, best

    def _update_scale(self):
        # Layer-B scale enhancement only. Rigid mode (use_scale=False) keeps s==1.0 exactly (no math change).
        if not self.use_scale:
            self.s = 1.0
            return
        rs = []
        for i in range(1, len(self.anchors)):
            a, b = self.anchors[i - 1], self.anchors[i]
            do = math.hypot(b["odom_x_anchor"] - a["odom_x_anchor"], b["odom_y_anchor"] - a["odom_y_anchor"])
            dm = math.hypot(b["map_x"] - a["map_x"], b["map_y"] - a["map_y"])
            if dm > MIN_BASELINE_M and do > MIN_BASELINE_M:   # H2: only trust ratio when both chords are long
                rs.append(do / dm)
        # H2/HIGH-2: guard empty / non-finite / out-of-prior; keep previous self.s (init 1.0) and warn. Never emit inf/nan.
        if not rs:
            self.warnings.append("SCALE no chord >= %.2fm, keep s=%.3f" % (MIN_BASELINE_M, self.s))
            return
        med = float(np.median(rs))
        if not math.isfinite(med) or med <= 0.0:
            self.warnings.append("SCALE median non-finite, keep s=%.3f" % self.s)
            return
        s_new = float(1.0 / med)   # core formula unchanged: s = 1.0 / median(do/dm)
        if not math.isfinite(s_new) or s_new < SCALE_LO or s_new > SCALE_HI:
            self.warnings.append(
                "SCALE %.3f out of [%.2f,%.2f], keep s=%.3f" % (s_new, SCALE_LO, SCALE_HI, self.s))
            return
        self.s = s_new

    def get_anchor_T(self, k):
        """scaled shared-R transform anchored at fix k (passes through fix k exactly). None if R unknown."""
        if self.R is None or k < 0 or k >= len(self.anchors):
            return None
        anc = self.anchors[k]
        sR = self.s * self.R
        oxy = np.array([anc["odom_x_anchor"], anc["odom_y_anchor"]], dtype=np.float64)
        mxy = np.array([anc["map_x"], anc["map_y"]], dtype=np.float64)
        t = mxy - sR @ oxy
        T = np.eye(3, dtype=np.float64)
        T[:2, :2] = sR
        T[0, 2] = t[0]
        T[1, 2] = t[1]
        return T

    @staticmethod
    def apply_T(T, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        Todom = np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]], dtype=np.float64)
        M = T @ Todom
        return float(M[0, 2]), float(M[1, 2]), math.atan2(M[1, 0], M[0, 0])

    def step(self, odom_x, odom_y, odom_yaw):
        """current robot map pose via latest anchor transform. None if not localized (ACQUIRING)."""
        if not self.localized or self.active_T is None:
            return None
        return self.apply_T(self.active_T, odom_x, odom_y, odom_yaw)

    @property
    def n_fixes(self):
        return len(self.anchors)

    @property
    def scale(self):
        return self.s
