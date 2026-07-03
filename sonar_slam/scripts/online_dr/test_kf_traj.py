#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS-free pytest for kf_traj (Layer B lane-2 correction filter). Does NOT import rospy.
Covers: propagate/scale; scalar==full-matrix KF+RTS oracle; terminal-fix miss formula + closed-form
corrections; rigid diffuse-prior backfill; mid-history late binding; passes-through-all-fixes;
NaN/Inf guards; duplicate-pid idempotency; max_nodes decimation keeps fix nodes; wrap/coarse helpers."""
import os
import sys
import math
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kf_traj import KfTrajectory, KfTrajParams, wrap_angle, coarse_R_from_yaw  # noqa: E402


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def build(nodes, fixes, R, s, params=None):
    """nodes: list of (ox,oy,oyaw). fixes: list of (pid, mx, my, anchor_ox, anchor_oy)."""
    kf = KfTrajectory(params or KfTrajParams(node_min_step=0.0))
    kf.set_similarity(R, s)
    for nd in nodes:
        kf.add_odom(*nd)
    for fx in fixes:
        kf.add_fix(*fx)
    kf.refilter()
    return kf


def ref_kf_rts(kf):
    """Independent FULL 2x2-matrix linear KF + RTS oracle mirroring kf_traj.refilter's math."""
    n = len(kf.nodes)
    I = np.eye(2)
    sR = kf.s * kf.R
    r = kf.p.sigma_fix_xy ** 2
    fbn = {}
    for f in kf.fixes:
        fbn.setdefault(f["node_idx"], []).append(f)
    xpred = [None] * n; Ppred = [None] * n; xpost = [None] * n; Ppost = [None] * n
    prev = np.array(kf.nodes[0][:2], dtype=np.float64)
    for i in range(n):
        cur = np.array(kf.nodes[i][:2], dtype=np.float64)
        if i == 0:
            xpred[0] = np.zeros(2); Ppred[0] = kf.p.init_var * I
        else:
            d = cur - prev
            inc = sR @ d
            dist = math.hypot(float(d[0]), float(d[1]))
            xpred[i] = xpost[i - 1] + inc
            Ppred[i] = Ppost[i - 1] + (kf.p.q_xy_per_m * dist + kf.p.q_xy_floor) * I
        x = xpred[i].copy(); P = Ppred[i].copy()
        for f in fbn.get(i, []):
            z = np.array([f["mx"], f["my"]], dtype=np.float64)
            K = P @ np.linalg.inv(P + r * I)
            x = x + K @ (z - x); P = (I - K) @ P
        xpost[i] = x; Ppost[i] = P; prev = cur
    xs = [None] * n; xs[n - 1] = xpost[n - 1]
    for i in range(n - 2, -1, -1):
        G = Ppost[i] @ np.linalg.inv(Ppred[i + 1])
        xs[i] = xpost[i] + G @ (xs[i + 1] - xpred[i + 1])
    return [(float(v[0]), float(v[1])) for v in xs]


LINE = [(float(i) * 0.5, 0.0, 0.0) for i in range(11)]   # 11 nodes, 0.5 m apart along +x


def test_propagate_no_fix_is_scaled_rotated_odom():
    """No fix -> smoothed path is the sR-transformed odom shape (up to the arbitrary node-0 origin)."""
    R = rot(0.3); s = 0.9
    kf = build(LINE, [], R, s)
    sm = kf.smoothed_path()
    assert len(sm) == len(LINE)
    sR = s * R
    for i in range(1, len(LINE)):
        d_odom = np.array(LINE[i][:2]) - np.array(LINE[i - 1][:2])
        d_map_exp = sR @ d_odom
        d_map_got = np.array(sm[i][:2]) - np.array(sm[i - 1][:2])
        assert np.allclose(d_map_got, d_map_exp, atol=1e-9)


def test_scalar_equals_full_matrix_oracle():
    """The scalar-variance implementation must equal a full 2x2 KF+RTS to ~1e-9 (random walk + fixes)."""
    rng = np.random.default_rng(7)
    nodes = [(0.0, 0.0, 0.0)]
    x, y = 0.0, 0.0
    for _ in range(40):
        x += float(rng.normal(0.4, 0.1)); y += float(rng.normal(0.1, 0.1))
        nodes.append((x, y, 0.0))
    R = rot(-0.6); s = 0.95
    sR = s * R
    # place 3 fixes at nodes 8, 22, 38 with the true map pos + a small offset
    fixes = []
    for pid, j, off in [(1, 8, (0.2, -0.1)), (2, 22, (-0.15, 0.25)), (3, 38, (0.05, 0.05))]:
        true_map = sR @ np.array(nodes[j][:2])
        fixes.append((pid, float(true_map[0] + off[0]), float(true_map[1] + off[1]),
                      nodes[j][0], nodes[j][1]))
    kf = build(nodes, fixes, R, s)
    got = [(p[0], p[1]) for p in kf.smoothed_path()]
    exp = ref_kf_rts(kf)
    assert len(got) == len(exp)
    for g, e in zip(got, exp):
        assert abs(g[0] - e[0]) < 1e-8 and abs(g[1] - e[1]) < 1e-8


def test_terminal_fix_miss_formula():
    """Single fix at the last node: smoothed endpoint misses z by exactly r/(p_N+r)*innov."""
    R = np.eye(2); s = 1.0
    params = KfTrajParams(node_min_step=0.0, sigma_fix_xy=0.02, q_xy_per_m=0.01, init_var=1e6)
    j = len(LINE) - 1
    z = (LINE[j][0] + 0.5, LINE[j][1] - 0.3)   # innovation (0.5, -0.3) vs pure-DR prior at node j
    kf = build(LINE, [(1, z[0], z[1], LINE[j][0], LINE[j][1])], R, s, params)
    sm = kf.smoothed_path()
    end = np.array(sm[-1][:2])
    # prior at node j (no fix) is the DR position = node j odom (R=I,s=1) from origin 0
    prior = np.array(LINE[j][:2])
    innov = np.array(z) - prior
    # p_N = init_var + sum q over the line
    dist_total = sum(math.hypot(LINE[i][0] - LINE[i - 1][0], LINE[i][1] - LINE[i - 1][1])
                     for i in range(1, len(LINE)))
    pN = params.init_var + params.q_xy_per_m * dist_total + params.q_xy_floor * (len(LINE) - 1)
    miss_expected = (params.sigma_fix_xy ** 2) / (pN + params.sigma_fix_xy ** 2) * innov
    miss_got = np.array(z) - end
    assert np.allclose(miss_got, miss_expected, atol=1e-6)
    assert np.linalg.norm(miss_got) < 0.01   # invisible inside the 0.35 m red sphere


def test_rigid_backfill_under_diffuse_prior():
    """Single fix at the last node -> every node shifts by ~the SAME vector (rigid backfill)."""
    R = np.eye(2); s = 1.0
    params = KfTrajParams(node_min_step=0.0, init_var=1e9, q_xy_per_m=0.01)
    j = len(LINE) - 1
    z = (LINE[j][0] - 1.7, LINE[j][1] + 2.3)
    kf = build(LINE, [(1, z[0], z[1], LINE[j][0], LINE[j][1])], R, s, params)
    sm = kf.smoothed_path()
    corr = [np.array(sm[i][:2]) - np.array(LINE[i][:2]) for i in range(len(LINE))]
    spread = max(float(np.linalg.norm(c - corr[0])) for c in corr)
    assert spread < 1e-3, "backfill not rigid, spread=%.3e" % spread


def test_passes_through_all_fixes_within_tolerance():
    """After refilter the smoothed line passes through every fix's node within ~sigma-level tolerance."""
    rng = np.random.default_rng(3)
    nodes = [(0.0, 0.0, 0.0)]
    x, y = 0.0, 0.0
    for _ in range(30):
        x += float(rng.normal(0.5, 0.05)); y += float(rng.normal(0.0, 0.05))
        nodes.append((x, y, 0.0))
    R = rot(0.4); s = 0.9; sR = s * R
    fixes = []
    node_ids = [6, 15, 27]
    for k, j in enumerate(node_ids):
        tm = sR @ np.array(nodes[j][:2])
        fixes.append((k + 1, float(tm[0] + 0.1), float(tm[1] - 0.1), nodes[j][0], nodes[j][1]))
    kf = build(nodes, fixes, R, s)
    sm = kf.smoothed_path()
    for (pid, mx, my, aox, aoy), j in zip(fixes, node_ids):
        got = np.array(sm[j][:2])
        assert np.linalg.norm(got - np.array([mx, my])) < 0.02


def test_mid_history_late_binding():
    """A fix whose anchor odom is a PAST node binds to that node (not 'now'); the line hits it there."""
    R = np.eye(2); s = 1.0
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(R, s)
    for nd in LINE:
        kf.add_odom(*nd)
    # bind a fix to node 3 (a past node) while 11 nodes exist
    assert kf.add_fix(1, 1.5, 0.4, LINE[3][0], LINE[3][1]) is True
    assert kf.fixes[0]["node_idx"] == 3
    kf.refilter()
    sm = kf.smoothed_path()
    assert np.linalg.norm(np.array(sm[3][:2]) - np.array([1.5, 0.4])) < 0.02


def test_nan_inf_inputs_rejected():
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(np.eye(2), 1.0)
    assert kf.add_odom(float("nan"), 0, 0) is False
    assert kf.add_odom(0, float("inf"), 0) is False
    assert kf.n_nodes == 0
    kf.add_odom(0.0, 0.0, 0.0); kf.add_odom(1.0, 0.0, 0.0)
    assert kf.add_fix(1, float("nan"), 0, 0, 0) is False
    assert kf.add_fix(2, 0, 0, float("inf"), 0) is False
    assert kf.n_fixes == 0
    kf.set_similarity(np.array([[float("nan"), 0], [0, 1]]), 1.0)  # rejected, R stays identity
    assert np.allclose(kf.R, np.eye(2))
    kf.refilter()
    assert all(math.isfinite(v) for p in kf.smoothed_path() for v in p)


def test_duplicate_pid_idempotent():
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(np.eye(2), 1.0)
    for nd in LINE:
        kf.add_odom(*nd)
    assert kf.add_fix(5, 1.0, 1.0, LINE[2][0], LINE[2][1]) is True
    assert kf.add_fix(5, 9.0, 9.0, LINE[8][0], LINE[8][1]) is False   # same pid ignored
    assert kf.n_fixes == 1 and kf.fixes[0]["mx"] == 1.0


def test_max_nodes_decimation_keeps_fix_nodes():
    params = KfTrajParams(node_min_step=0.0, max_nodes=20)
    kf = KfTrajectory(params)
    kf.set_similarity(np.eye(2), 1.0)
    for i in range(15):
        kf.add_odom(float(i), 0.0, 0.0)
    kf.add_fix(1, 5.0, 0.0, 5.0, 0.0)   # bound to node 5
    bound_odom = kf.nodes[kf.fixes[0]["node_idx"]]
    for i in range(15, 40):             # force past max_nodes -> decimation
        kf.add_odom(float(i), 0.0, 0.0)
    assert kf.n_nodes <= params.max_nodes
    # the fix-bound node's odom coordinate survived
    survived = any(abs(n[0] - bound_odom[0]) < 1e-9 for n in kf.nodes)
    assert survived
    kf.refilter()
    assert all(math.isfinite(v) for p in kf.smoothed_path() for v in p)


def test_wrap_angle_and_coarse_R():
    assert abs(wrap_angle(3.0 * math.pi) - math.pi) < 1e-9 or abs(wrap_angle(3.0 * math.pi) + math.pi) < 1e-9
    assert abs(wrap_angle(0.5) - 0.5) < 1e-12
    R = coarse_R_from_yaw(0.7, 0.2)     # theta = 0.5
    assert abs(math.atan2(R[1, 0], R[0, 0]) - 0.5) < 1e-9
    assert abs(float(np.linalg.det(R)) - 1.0) < 1e-9


def test_refilter_before_similarity_is_empty():
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.add_odom(0.0, 0.0, 0.0); kf.add_odom(1.0, 0.0, 0.0)
    kf.refilter()                        # R is None -> no output, no crash
    assert kf.smoothed_path() == []


def test_bind_distance_gate_rejects_far_fix():
    """A fix whose anchor odom is far from every node (stale / re-sent / mis-topic) is dropped, not bound."""
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0, bind_max_dist=0.5))
    kf.set_similarity(np.eye(2), 1.0)
    for nd in LINE:
        kf.add_odom(*nd)                 # nodes span x in [0, 5]
    assert kf.add_fix(1, 100.0, 0.0, 46.0, 0.0) is False   # anchor 46 m away -> rejected
    assert kf.n_fixes == 0
    assert any("bind too far" in w for w in kf.warnings)
    assert kf.add_fix(2, 2.5, 0.0, LINE[5][0], LINE[5][1]) is True   # on a node -> accepted
    assert kf.n_fixes == 1


def test_nearest_node_prefers_latest_on_tie():
    """On a self-approach (two nodes at the same odom xy), a fix binds to the LATER node (late-binding bias)."""
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(np.eye(2), 1.0)
    for nd in [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0), (3.0, 0.0, 0.0), (2.0, 0.0, 0.0)]:
        kf.add_odom(*nd)                 # node 2 and node 4 are both at (2,0)
    assert kf.add_fix(1, 5.0, 5.0, 2.0, 0.0) is True
    assert kf.fixes[0]["node_idx"] == 4  # later index wins the tie


def test_coarse_then_kabsch_reprojects_whole_history():
    """Changing the similarity (coarse -> Kabsch) must reproject the WHOLE line (increments recomputed from
    raw odom each refilter -- no caching). Tail increments must equal the new sR @ odom-increment."""
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    for nd in LINE:
        kf.add_odom(*nd)
    kf.set_similarity(np.eye(2), 1.0)
    kf.add_fix(1, LINE[2][0], LINE[2][1], LINE[2][0], LINE[2][1])
    kf.refilter()
    R2 = rot(0.5); s2 = 0.9
    kf.set_similarity(R2, s2)
    kf.refilter()
    sm = kf.smoothed_path()
    sR2 = s2 * R2
    for i in range(7, len(LINE)):        # tail after the single fix = pure DR under the NEW sR
        d_odom = np.array(LINE[i][:2]) - np.array(LINE[i - 1][:2])
        d_got = np.array(sm[i][:2]) - np.array(sm[i - 1][:2])
        assert np.allclose(d_got, sR2 @ d_odom, atol=1e-9)


def test_out_of_order_fixes_bind_by_anchor():
    """Fixes arriving out of temporal order still bind to their own anchor node; line hits both."""
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(np.eye(2), 1.0)
    for nd in LINE:
        kf.add_odom(*nd)
    assert kf.add_fix(2, LINE[8][0], LINE[8][1] + 0.3, LINE[8][0], LINE[8][1]) is True   # later stop first
    assert kf.add_fix(1, LINE[3][0], LINE[3][1] - 0.2, LINE[3][0], LINE[3][1]) is True   # earlier stop after
    assert [f["node_idx"] for f in kf.fixes] == [3, 8]   # kept sorted by node
    kf.refilter()
    sm = kf.smoothed_path()
    assert np.linalg.norm(np.array(sm[3][:2]) - np.array([LINE[3][0], LINE[3][1] - 0.2])) < 0.02
    assert np.linalg.norm(np.array(sm[8][:2]) - np.array([LINE[8][0], LINE[8][1] + 0.3])) < 0.02


def test_two_fixes_same_node_stay_finite():
    """Two fixes bound to the same node -> sequential KF updates, finite, bounded by the two measurements."""
    kf = KfTrajectory(KfTrajParams(node_min_step=0.0))
    kf.set_similarity(np.eye(2), 1.0)
    for nd in LINE:
        kf.add_odom(*nd)
    kf.add_fix(1, LINE[5][0], 0.4, LINE[5][0], LINE[5][1])
    kf.add_fix(2, LINE[5][0], -0.2, LINE[5][0], LINE[5][1])   # same node, conflicting y
    assert kf.n_fixes == 2 and kf.fixes[0]["node_idx"] == kf.fixes[1]["node_idx"]
    kf.refilter()
    y = kf.smoothed_path()[5][1]
    assert math.isfinite(y) and -0.2 <= y <= 0.4   # between the two measurements, no blow-up


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
