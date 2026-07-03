#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS-free pytest for dr_core.RelocDisplayEstimator (Layer B display core).
Covers: rigid faithfulness (vs an independent Kabsch oracle), degenerate-geometry gating,
NaN/Inf rejection, point_id idempotency, and scale clamping. Does NOT import rospy."""
import os
import sys
import math
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dr_core import RelocDisplayEstimator, SCALE_LO, SCALE_HI  # noqa: E402

# Vendored corrected.py (frozen Layer A math the estimator reuses for R).
RELOC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                      "..", "reloc", "corrected.py"))


def est(min_fixes=2, use_scale=True):
    return RelocDisplayEstimator(RELOC, min_fixes=min_fixes, use_scale=use_scale)


# Well-separated, non-degenerate fix sequence: (pid, map_x, map_y, map_yaw, odom_x, odom_y, odom_yaw)
SEPARATED = [
    (1, 1.0, 2.0, 0.10, 0.0, 0.0, 0.00),
    (2, 5.0, 3.5, 0.20, 4.2, 0.7, 0.15),
    (3, 8.0, 9.0, -0.30, 7.9, 5.1, -0.25),
    (4, -2.0, 4.0, 1.10, 1.3, 6.6, 1.05),
]


def _kabsch_R(anchors_2):
    """Independent 2-point Kabsch oracle mirroring vendored estimate_shared_R_from_anchor_positions."""
    A = np.array([[a[4], a[5]] for a in anchors_2], dtype=np.float64)  # odom xy
    B = np.array([[a[1], a[2]] for a in anchors_2], dtype=np.float64)  # map xy
    muA, muB = A.mean(0), B.mean(0)
    H = (A - muA).T @ (B - muB)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    return R


def _oracle_pose(R, anchor, odom_xyyaw):
    """Rigid (s=1) per-anchor transform oracle: t = map - R@odom; pose = R@p + t, yaw += theta."""
    t = np.array([anchor[1], anchor[2]]) - R @ np.array([anchor[4], anchor[5]])
    x, y, yaw = odom_xyyaw
    px, py = R @ np.array([x, y]) + t
    theta = math.atan2(R[1, 0], R[0, 0])
    yaw_out = math.atan2(math.sin(yaw + theta), math.cos(yaw + theta))  # wrap, matches apply_T's atan2
    return float(px), float(py), yaw_out


def test_rigid_faithfulness_matches_kabsch_oracle():
    """use_scale=False output must equal the independent Kabsch + per-anchor rigid transform."""
    e = est(min_fixes=2, use_scale=False)
    R = _kabsch_R(SEPARATED[:2])   # R fixed from first two separated anchors
    probes = [(0.0, 0.0, 0.0), (1.5, 2.7, 0.3), (10.1, -3.3, -1.2), (4.4, 4.4, 3.0)]
    for k, fx in enumerate(SEPARATED):
        e.on_fix(*fx)
        if not e.localized:
            assert e.step(0.0, 0.0, 0.0) is None
            continue
        assert e.scale == 1.0          # rigid: scale is exactly 1.0
        last_anchor = SEPARATED[k]     # active_T anchored at latest fix
        for p in probes:
            got = e.step(*p)
            exp = _oracle_pose(R, last_anchor, p)
            assert got is not None
            for g, x in zip(got, exp):
                assert abs(g - x) < 1e-9, "rigid pose drift %.3e" % abs(g - x)


def test_rigid_passes_through_each_fix_exactly():
    """Per-anchor transform must pass through its own fix (odom_anchor -> map_anchor) to ~0."""
    e = est(min_fixes=2, use_scale=False)
    for fx in SEPARATED:
        e.on_fix(*fx)
    # After all fixes, active_T is anchored on the last fix: step(last odom) == last map xy.
    last = SEPARATED[-1]
    px, py, _ = e.step(last[4], last[5], last[6])
    assert abs(px - last[1]) < 1e-9 and abs(py - last[2]) < 1e-9


def test_R_is_proper_rotation_no_reflection():
    e = est(min_fixes=2, use_scale=False)
    e.on_fix(*SEPARATED[0])
    e.on_fix(*SEPARATED[1])
    assert e.R is not None
    assert abs(float(np.linalg.det(e.R)) - 1.0) < 1e-9          # proper rotation
    assert np.allclose(e.R @ e.R.T, np.eye(2), atol=1e-9)       # orthonormal


def test_localizes_at_two_separated_fixes():
    e = est(min_fixes=2, use_scale=False)
    e.on_fix(*SEPARATED[0])
    assert not e.localized and e.step(0, 0, 0) is None
    e.on_fix(*SEPARATED[1])
    assert e.localized and not e.degenerate and e.step(0, 0, 0) is not None


def test_degenerate_same_odom_defers_until_separated_fix():
    """Two fixes at the SAME odom position -> stay ACQUIRING (degenerate), no inf/nan; a separated
    third fix -> localize."""
    e = est(min_fixes=2, use_scale=False)
    e.on_fix(1, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    e.on_fix(2, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0)   # same odom -> do=0 < MIN_BASELINE_M
    assert not e.localized
    assert e.degenerate
    assert e.R is None
    assert e.step(0.0, 0.0, 0.0) is None
    assert math.isfinite(e.scale)               # no inf/nan produced
    assert any(w.startswith("DEGENERATE") for w in e.warnings)
    # A separated fix resolves the degeneracy.
    e.on_fix(3, 4.0, 1.0, 0.0, 3.0, 0.0, 0.0)   # do=3, dm=3 vs anchor 1
    assert e.localized and not e.degenerate
    res = e.step(3.0, 0.0, 0.0)
    assert res is not None and all(math.isfinite(v) for v in res)


def test_degenerate_map_only_separation_still_defers():
    """map chord long but odom chord ~0 (both must exceed) -> degenerate."""
    e = est(min_fixes=2, use_scale=False)
    e.on_fix(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    e.on_fix(2, 5.0, 5.0, 0.0, 0.01, 0.0, 0.0)  # dm big, do=0.01 < 0.5
    assert e.degenerate and not e.localized and e.R is None


def test_nan_inf_fix_is_dropped():
    e = est()
    e.on_fix(float("nan"), 0, 0, 0, 0, 0, 0)
    e.on_fix(1, float("nan"), 0, 0, 0, 0, 0)
    e.on_fix(2, 0, 0, 0, float("inf"), 0, 0)
    e.on_fix(3, 0, 0, float("-inf"), 0, 0, 0)
    assert e.n_fixes == 0
    assert len([w for w in e.warnings if w.startswith("DROP")]) == 4


def test_nan_does_not_break_later_valid_fixes():
    e = est(use_scale=False)
    e.on_fix(float("nan"), 1, 1, 0, 1, 1, 0)   # dropped
    e.on_fix(*SEPARATED[0])
    e.on_fix(*SEPARATED[1])
    assert e.n_fixes == 2 and e.localized
    assert all(math.isfinite(v) for v in e.step(1.0, 1.0, 0.0))


def test_duplicate_pid_is_idempotent():
    e = est(use_scale=False)
    e.on_fix(5, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0)
    assert e.n_fixes == 1
    e.on_fix(5, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0)  # same pid -> ignored
    assert e.n_fixes == 1
    assert e.anchors[0]["map_x"] == 1.0        # original kept, not overwritten


def test_scale_clamp_low_rejected_keeps_previous():
    """s = 1/median(do/dm) < SCALE_LO -> rejected, keep previous s (init 1.0), no inf/nan."""
    e = est(use_scale=True)
    e.on_fix(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    e.on_fix(2, 0.6, 0.0, 0.0, 2.0, 0.0, 0.0)  # do=2, dm=0.6 -> ratio 3.33 -> s=0.30 < 0.7
    assert e.localized
    assert e.scale == 1.0
    assert math.isfinite(e.scale)
    assert any(w.startswith("SCALE") and "out of" in w for w in e.warnings)


def test_scale_clamp_high_rejected_keeps_previous():
    e = est(use_scale=True)
    e.on_fix(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    e.on_fix(2, 2.0, 0.0, 0.0, 0.6, 0.0, 0.0)  # do=0.6, dm=2 -> ratio 0.3 -> s=3.33 > 1.3
    assert e.localized and e.scale == 1.0 and math.isfinite(e.scale)


def test_scale_valid_accepted_in_range():
    e = est(use_scale=True)
    e.on_fix(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    e.on_fix(2, 2.0, 0.0, 0.0, 2.2, 0.0, 0.0)  # ratio 1.1 -> s = 0.909 in [0.7,1.3]
    assert SCALE_LO <= e.scale <= SCALE_HI
    assert abs(e.scale - (1.0 / 1.1)) < 1e-9


def test_update_scale_empty_keeps_previous():
    """White-box: no chord >= MIN_BASELINE_M -> keep previous s, warn, no crash."""
    e = est(use_scale=True)
    e.anchors.append(dict(point_id=1, map_x=0.0, map_y=0.0, map_yaw_rad=0.0,
                          odom_x_anchor=0.0, odom_y_anchor=0.0, odom_yaw_anchor_rad=0.0))
    e._update_scale()
    assert e.scale == 1.0
    assert any(w.startswith("SCALE") for w in e.warnings)


def test_min_fixes_floor_is_two():
    assert est(min_fixes=1).min_fixes == 2
    assert est(min_fixes=0).min_fixes == 2
    assert est(min_fixes=5).min_fixes == 5


def test_scale_never_inf_or_nan_over_sequence():
    e = est(use_scale=True)
    for fx in SEPARATED:
        e.on_fix(*fx)
        assert math.isfinite(e.scale)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
