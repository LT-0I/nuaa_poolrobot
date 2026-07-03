#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Layer B rviz display node (ROS/rospy). On-robot real-time localization DISPLAY.
Subscribes:
  ~odom_topic (default /wheel_odom, nav_msgs/Odometry)  -- dense odom from driver
  ~fix_topic  (default /reloc/fix, std_msgs/Float64MultiArray = [point_id, map_x, map_y, map_yaw_rad,
                anchor_odom_x, anchor_odom_y, anchor_odom_yaw_rad])  -- from Layer A localization
Publishes (ALL in frame 'map'; rviz FixedFrame=map, no TF chaining needed):
  /reloc/map         sensor_msgs/PointCloud2 (base map global_filter.pcd, latched)
  /reloc/robot_pose  geometry_msgs/PoseStamped (current position; only when LOCALIZED)
  /reloc/trajectory  nav_msgs/Path (map-frame trajectory; distance-decimated, published by 0.5Hz timer)
  /reloc/fixes       visualization_msgs/MarkerArray (fix truth points + labels)
  /reloc/status      visualization_msgs/Marker (TEXT: ACQUIRING / LOCALIZED / DEGENERATE) + std_msgs/String
DISPLAY POLICY (two-lane, use_filter=True default): ACQUIRING (nothing drawn) until the FIRST fix, since
with zero anchors odom cannot be placed in the map (never fabricate). At fix1 -> TRACKING-COARSE (line drawn
with a single-fix yaw rotation, s=1). At the 2nd SEPARATED fix -> LOCALIZED (dr_core Kabsch R + median scale,
one realignment). show_coarse=False restores the legacy "no position until 2 separated fixes" gate.
Legacy est-only path (use_filter=False) is preserved verbatim.
HARDENING (Layer B round 2): bounded odom/path buffers, distance-decimated Path on an independent timer,
  callbacks serialized by a lock, strict binary-PCD header parse with graceful degrade, status on a timer
  positioned from the base-map bbox (shows DEGENERATE baseline when geometry is too tight).
SAFETY: read-only display. Does NOT subscribe/publish /cmd_vel or /autoVel_cmd, does NOT touch motion/stop/driver.
        Does NOT trigger sonar. Terminal output ASCII only.
"""
import os
import math
import threading
import collections
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, String, Header, ColorRGBA
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField

from dr_core import RelocDisplayEstimator, MIN_BASELINE_M
from kf_traj import KfTrajectory, KfTrajParams, coarse_R_from_yaw

BUF_MAXLEN = 100000       # bounded odom backfill buffer (deque auto-drops oldest on overflow)
PATH_MIN_STEP = 0.03      # meters; skip Path points closer than this to the previous one
PATH_MAX = 5000           # hard cap on Path poses; decimate (halve) when exceeded


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def load_pcd_xyz(path):
    """Parse a binary PCD, returning an (N,3) float32 array of x,y,z.
    Strict: DATA must be the whole word 'binary' (rejects ascii / binary_compressed); FIELDS must
    start with x y z; stride and point count are derived from SIZE/TYPE/COUNT/POINTS and the body is
    sliced exactly. Raises ValueError with an ASCII message on any malformed header."""
    with open(path, "rb") as f:
        raw = f.read()
    fields = None
    sizes = None
    types_ = None
    counts = None
    npoints = None
    data_mode = None
    header_end = None
    pos = 0
    n = len(raw)
    while pos < n:
        nl = raw.find(b"\n", pos)
        if nl < 0:
            break
        line = raw[pos:nl].decode("ascii", "replace").strip()
        pos = nl + 1
        if line == "" or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            sizes = parts[1:]
        elif key == "TYPE":
            types_ = parts[1:]
        elif key == "COUNT":
            counts = parts[1:]
        elif key == "POINTS":
            npoints = int(parts[1]) if len(parts) > 1 else None
        elif key == "DATA":
            data_mode = parts[1] if len(parts) > 1 else ""
            header_end = pos
            break
    if header_end is None or data_mode is None:
        raise ValueError("PCD parse error: no DATA line found (empty or truncated header)")
    if data_mode != "binary":
        raise ValueError("PCD parse error: DATA '%s' unsupported (need whole-word 'binary'; "
                         "refuse ascii/binary_compressed)" % data_mode)
    if not fields or len(fields) < 3 or fields[0] != "x" or fields[1] != "y" or fields[2] != "z":
        raise ValueError("PCD parse error: FIELDS first three must be x y z, got %s" % (fields,))
    if npoints is None or npoints < 0:
        raise ValueError("PCD parse error: missing or invalid POINTS")
    nf = len(fields)
    if sizes is None or types_ is None:
        raise ValueError("PCD parse error: missing SIZE or TYPE")
    if counts is None:
        counts = ["1"] * nf
    if not (len(sizes) == len(types_) == len(counts) == nf):
        raise ValueError("PCD parse error: FIELDS/SIZE/TYPE/COUNT length mismatch")
    try:
        sizes_i = [int(s) for s in sizes]
        counts_i = [int(c) for c in counts]
    except ValueError:
        raise ValueError("PCD parse error: non-integer SIZE/COUNT")
    for i in range(3):
        if not (sizes_i[i] == 4 and types_[i].upper() == "F" and counts_i[i] == 1):
            raise ValueError("PCD parse error: x/y/z must be float32 (SIZE 4 TYPE F COUNT 1)")
    stride = sum(sz * ct for sz, ct in zip(sizes_i, counts_i))
    expected = npoints * stride
    body = raw[header_end:]
    if len(body) < expected:
        raise ValueError("PCD parse error: binary body %d bytes < expected %d (POINTS mismatch)"
                         % (len(body), expected))
    arr = np.frombuffer(body[:expected], dtype=np.uint8).reshape(npoints, stride)
    xyz = np.ascontiguousarray(arr[:, :12]).view(np.float32).reshape(npoints, 3)
    return xyz.astype(np.float32)


def make_cloud(xyz, frame):
    h = Header(stamp=rospy.Time.now(), frame_id=frame)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    buf = xyz.astype(np.float32).tobytes()
    return PointCloud2(header=h, height=1, width=xyz.shape[0], is_dense=True, is_bigendian=False,
                       fields=fields, point_step=12, row_step=12 * xyz.shape[0], data=buf)


class DisplayNode:
    def __init__(self):
        corrected_path = rospy.get_param("~corrected_path")
        pcd_path = rospy.get_param("~pcd_path")
        self.frame = rospy.get_param("~map_frame", "map")
        self.min_fixes = max(2, int(rospy.get_param("~min_fixes", 2)))
        use_scale = bool(rospy.get_param("~use_scale", True))
        odom_topic = rospy.get_param("~odom_topic", "/wheel_odom")
        fix_topic = rospy.get_param("~fix_topic", "/reloc/fix")
        self.pub_hz = max(0.1, float(rospy.get_param("~publish_hz", 10.0)))       # robot_pose rate
        self.path_hz = max(0.1, float(rospy.get_param("~path_publish_hz", 5.0)))  # Path publish rate (was 0.5; 5.0 kills the blue-line trail)
        self.status_hz = max(0.1, float(rospy.get_param("~status_publish_hz", 2.0)))

        self.est = RelocDisplayEstimator(corrected_path, min_fixes=self.min_fixes, use_scale=use_scale)

        # --- Layer B two-lane filter state (use_filter=True; else legacy est-only path below) ---
        self.use_filter = bool(rospy.get_param("~use_filter", True))
        self.show_coarse = bool(rospy.get_param("~show_coarse", True))
        kfp = KfTrajParams(
            q_xy_per_m=float(rospy.get_param("~q_xy_per_m", 0.012)),
            q_xy_floor=float(rospy.get_param("~q_xy_floor", 1e-8)),
            sigma_fix_xy=float(rospy.get_param("~sigma_fix_xy", 0.02)),
            node_min_step=float(rospy.get_param("~node_min_step", 0.03)),
            max_nodes=int(rospy.get_param("~max_nodes", 20000)),
            init_var=float(rospy.get_param("~init_var", 1e6)),
            bind_max_dist=float(rospy.get_param("~bind_max_dist", 0.5)))
        self.kf = KfTrajectory(kfp)
        self._live_T = None        # 3x3 current similarity for the live pose (est.active_T or fix-1 coarse)
        self._live_mode = "ACQUIRING"
        self._fix1 = None          # first accepted fix (for coarse yaw startup)
        self.path_min_step = float(rospy.get_param("~path_min_step", PATH_MIN_STEP))  # blue-line decimation (m)
        self.path_max = max(1, int(rospy.get_param("~path_max", PATH_MAX)))            # blue-line hard pose cap (>=1)

        self.lock = threading.Lock()          # serializes callbacks (only light math inside)
        self.buf = collections.deque(maxlen=BUF_MAXLEN)   # (x,y,yaw,n_fixes) until first localization (backfill)
        self._buf_warned = False
        self.latest = None                    # latest odom (x, y, yaw)
        self.path = Path()
        self.path.header.frame_id = self.frame
        self._path_dirty = False
        self._last_pub = 0.0
        self._text_pos = (0.0, 0.0)           # status text anchor, derived from base-map bbox
        self.map_loaded = False
        self.map_error = ""

        self.pub_map = rospy.Publisher("/reloc/map", PointCloud2, queue_size=1, latch=True)
        self.pub_pose = rospy.Publisher("/reloc/robot_pose", PoseStamped, queue_size=1)
        self.pub_path = rospy.Publisher("/reloc/trajectory", Path, queue_size=1, latch=True)  # latched: a late rviz sees the line
        self.pub_fixes = rospy.Publisher("/reloc/fixes", MarkerArray, queue_size=1, latch=True)
        self.pub_status_m = rospy.Publisher("/reloc/status", Marker, queue_size=1, latch=True)
        self.pub_status_s = rospy.Publisher("/reloc/status_str", String, queue_size=1, latch=True)

        if os.path.exists(pcd_path):
            try:
                xyz = load_pcd_xyz(pcd_path)
                self.pub_map.publish(make_cloud(xyz, self.frame))
                self._text_pos = self._bbox_text_pos(xyz)
                self.map_loaded = True
                rospy.loginfo("reloc_display: published base map %d pts from %s",
                              xyz.shape[0], os.path.basename(pcd_path))
            except Exception as e:
                self.map_error = str(e)
                rospy.logwarn("reloc_display: base map load FAILED (%s): %s",
                              os.path.basename(pcd_path), self.map_error)
        else:
            self.map_error = "pcd not found"
            rospy.logwarn("reloc_display: pcd not found: %s", pcd_path)

        self._publish_status()
        rospy.Subscriber(fix_topic, Float64MultiArray, self.on_fix, queue_size=50)
        rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size=200)
        rospy.Timer(rospy.Duration(1.0 / self.path_hz), self._on_path_timer)
        rospy.Timer(rospy.Duration(1.0 / self.status_hz), self._on_status_timer)
        if self.use_filter:
            rospy.loginfo("reloc_display: up. two-lane filter; ACQUIRING->COARSE@fix1->LOCALIZED@sep-fix2 "
                          "(show_coarse=%s). use_scale=%s", self.show_coarse, use_scale)
        else:
            rospy.loginfo("reloc_display: up. legacy; policy=show position only after %d separated fixes. use_scale=%s",
                          self.min_fixes, use_scale)

    def _bbox_text_pos(self, xyz):
        """Top-center-ish anchor for the status TEXT marker, derived from the base-map xy bbox."""
        if xyz is None or xyz.shape[0] == 0:
            return (0.0, 0.0)
        xmin = float(np.min(xyz[:, 0]))
        xmax = float(np.max(xyz[:, 0]))
        ymin = float(np.min(xyz[:, 1]))
        ymax = float(np.max(xyz[:, 1]))
        cx = 0.5 * (xmin + xmax)
        margin = max(0.5, 0.05 * (ymax - ymin))
        return (cx, ymax + margin)

    def on_fix(self, msg):
        d = list(msg.data)
        if len(d) < 7:
            rospy.logwarn("reloc_display: bad fix msg len=%d", len(d))
            return
        pid, mx, my, myaw, ox, oy, oyaw = d[0], d[1], d[2], d[3], d[4], d[5], d[6]
        with self.lock:
            was = self.est.localized
            self.est.on_fix(pid, mx, my, myaw, ox, oy, oyaw)
            nf = self.est.n_fixes
            loc = self.est.localized
            self._publish_fixes()   # red anchors = raw poses_in_map (passthrough, unchanged)
            if self.use_filter:
                self._on_fix_filter(pid, mx, my, myaw, ox, oy, oyaw)   # caller holds self.lock
            elif loc and not was:
                self._backfill_trajectory()   # caller holds self.lock; helper must NOT re-acquire
        pid_str = ("%d" % int(pid)) if math.isfinite(pid) else "nan"
        rospy.loginfo("reloc_display: FIX point=%s map=(%.2f,%.2f) fixes=%d mode=%s",
                      pid_str, mx, my, nf, self._live_mode if self.use_filter else ("LOC" if loc else "ACQ"))

    def on_odom(self, msg):
        # odom can arrive very fast (replay flood / driver ~100Hz+); keep only latest, do heavy work throttled.
        p = msg.pose.pose
        yaw = yaw_from_quat(p.orientation)
        with self.lock:
            self.latest = (p.position.x, p.position.y, yaw)
            if self.use_filter:
                self._on_odom_filter(p.position.x, p.position.y, yaw)   # caller holds self.lock
                return
            if not self.est.localized:
                # Before first localization only buffer once >=1 fix exists (the long BBS wait needs no backfill).
                if self.est.n_fixes >= 1:
                    self.buf.append((p.position.x, p.position.y, yaw, self.est.n_fixes))
                    if len(self.buf) >= self.buf.maxlen and not self._buf_warned:
                        rospy.logwarn("reloc_display: odom backfill buffer full (%d pts), dropping oldest",
                                      self.buf.maxlen)
                        self._buf_warned = True
                return
            now = rospy.Time.now().to_sec()
            if (now - self._last_pub) < (1.0 / self.pub_hz):
                return
            self._last_pub = now
            res = self.est.step(*self.latest)
            if res is None:
                return
            mx, my, myaw = res
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = self.frame
            ps.pose.position = Point(x=mx, y=my, z=0.0)
            ps.pose.orientation = quat_from_yaw(myaw)
            self.pub_pose.publish(ps)
            self._append_path(mx, my, ps)     # caller holds self.lock

    # ---------- Layer B two-lane filter (use_filter=True) ----------
    def _on_odom_filter(self, ox, oy, oyaw):
        # LANE 1 (live, low latency): feed the lane-2 node history, then publish the geometric-DR pose via
        # the current similarity (est.active_T when LOCALIZED, else the fix-1 coarse transform) and grow the
        # blue tip. No Kalman on this path: with no measurement between fixes the KF output == the DR pose.
        # Caller holds self.lock.
        self.kf.add_odom(ox, oy, oyaw)
        if self._live_T is None:
            return                                # ACQUIRING: no map anchor yet -> draw nothing (no fabrication)
        now = rospy.Time.now().to_sec()
        if (now - self._last_pub) < (1.0 / self.pub_hz):
            return
        self._last_pub = now
        mx, my, myaw = RelocDisplayEstimator.apply_T(self._live_T, ox, oy, oyaw)
        if not (math.isfinite(mx) and math.isfinite(my) and math.isfinite(myaw)):
            return
        ps = self._pose_stamped(mx, my, myaw)
        self.pub_pose.publish(ps)
        self._append_path(mx, my, ps)             # live tip; absorbed into the smoothed line at the next fix

    def _on_fix_filter(self, pid, mx, my, myaw, ox, oy, oyaw):
        # LANE 2 (correction, off hot path): choose the similarity (est Kabsch once >=2 separated fixes, else
        # the fix-1 coarse yaw-rotation), bind this LATE fix to its past anchor node, re-smooth the whole line
        # (KF forward + RTS) so it passes through every fix, and redraw the blue Path. Caller holds self.lock.
        if not all(math.isfinite(v) for v in (pid, mx, my, myaw, ox, oy, oyaw)):
            return                                # est.on_fix already dropped it; leave the filter untouched
        if self._fix1 is None:
            self._fix1 = dict(map_yaw=myaw, odom_yaw=oyaw, odom_x=ox, odom_y=oy, map_x=mx, map_y=my)
        if self.est.R is not None:
            self.kf.set_similarity(self.est.R, self.est.s)
            self._live_T = self.est.active_T
            self._live_mode = "LOCALIZED"
        elif self.show_coarse and self._fix1 is not None:
            Rc = coarse_R_from_yaw(self._fix1["map_yaw"], self._fix1["odom_yaw"])
            self.kf.set_similarity(Rc, 1.0)
            self._live_T = self._coarse_T(Rc, 1.0, self._fix1)
            self._live_mode = "COARSE"
        else:
            self._live_T = None
            self._live_mode = "ACQUIRING"
        if not self.kf.add_fix(pid, mx, my, ox, oy):
            rospy.logwarn("reloc_display: fix pid=%s not bound in KF (dropped); est tip already applied -> tip/line may briefly diverge", pid)
        self.kf.refilter()
        self._rebuild_path_from_smoothed()

    def _coarse_T(self, R, s, fix):
        # provisional similarity anchored exactly through the fix (same sR/t construction as dr_core.get_anchor_T).
        sR = float(s) * np.asarray(R, dtype=np.float64)
        oxy = np.array([fix["odom_x"], fix["odom_y"]], dtype=np.float64)
        mxy = np.array([fix["map_x"], fix["map_y"]], dtype=np.float64)
        t = mxy - sR @ oxy
        T = np.eye(3, dtype=np.float64)
        T[:2, :2] = sR
        T[0, 2] = t[0]
        T[1, 2] = t[1]
        return T

    def _pose_stamped(self, mx, my, myaw, stamp=None):
        ps = PoseStamped()
        ps.header.stamp = stamp if stamp is not None else rospy.Time.now()
        ps.header.frame_id = self.frame
        ps.pose.position = Point(x=mx, y=my, z=0.0)
        ps.pose.orientation = quat_from_yaw(myaw)
        return ps

    def _rebuild_path_from_smoothed(self):
        # Caller holds self.lock. Replace the whole blue Path with the freshly smoothed line and publish once.
        sm = self.kf.smoothed_path()
        n = len(sm)
        if n > self.path_max:   # bound the published Path (long missions); keep endpoints + every fix-bound node
            keep = set(range(0, n, (n // self.path_max) + 1))
            keep.add(n - 1)
            for f in self.kf.fixes:
                if 0 <= f["node_idx"] < n:
                    keep.add(f["node_idx"])
            sm = [sm[i] for i in sorted(keep)]
        stamp = rospy.Time.now()
        self.path.poses = [self._pose_stamped(x, y, yaw, stamp) for (x, y, yaw) in sm]
        self.path.header.stamp = stamp
        self.pub_path.publish(self.path)
        self._path_dirty = False
        rospy.loginfo("reloc_display: FILTER refilter -> %d nodes, %d fixes, mode=%s, scale=%.3f",
                      self.kf.n_nodes, self.kf.n_fixes, self._live_mode, self.est.scale)

    def _append_path(self, mx, my, ps):
        # distance-decimated append; the 0.5Hz timer publishes only when dirty (no per-odom full resend).
        if self.path.poses:
            last = self.path.poses[-1].pose.position
            if math.hypot(mx - last.x, my - last.y) < self.path_min_step:
                return
        self.path.poses.append(ps)
        self._path_dirty = True
        if len(self.path.poses) > self.path_max:
            self.path.poses = self.path.poses[::2]
            rospy.logwarn("reloc_display: path exceeded %d poses, decimated to %d",
                          self.path_max, len(self.path.poses))

    def _on_path_timer(self, event):
        with self.lock:
            if not self._path_dirty:
                return
            self.path.header.stamp = rospy.Time.now()
            self.pub_path.publish(self.path)
            self._path_dirty = False

    def _on_status_timer(self, event):
        with self.lock:
            self._publish_status()

    def _backfill_trajectory(self):
        # retroactively draw path from fix1 onward with the now-trustworthy transform (per-segment anchor).
        # Caller (on_fix) holds self.lock.
        items = list(self.buf)
        self.path.poses = []
        stride = max(1, len(items) // 600)   # downsample: keep Path light
        for idx in range(0, len(items), stride):
            (x, y, yaw, nf) = items[idx]
            if nf < 1:
                continue
            T = self.est.get_anchor_T(min(nf - 1, self.est.n_fixes - 1))
            if T is None:
                continue
            mx, my, myaw = RelocDisplayEstimator.apply_T(T, x, y, yaw)
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = self.frame
            ps.pose.position = Point(x=mx, y=my, z=0.0)
            ps.pose.orientation = quat_from_yaw(myaw)
            self.path.poses.append(ps)
        self.buf.clear()   # no longer needed after first localization
        self.path.header.stamp = rospy.Time.now()
        self.pub_path.publish(self.path)
        self._path_dirty = False
        rospy.loginfo("reloc_display: LOCALIZED -> backfilled trajectory (%d pts), scale=%.3f",
                      len(self.path.poses), self.est.scale)

    def _publish_fixes(self):
        # Caller holds self.lock.
        ma = MarkerArray()
        for i, a in enumerate(self.est.anchors):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = rospy.Time.now()
            m.ns = "fixes"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=a["map_x"], y=a["map_y"], z=0.0)
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=0.35, y=0.35, z=0.35)
            m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)
            ma.markers.append(m)
            t = Marker()
            t.header.frame_id = self.frame
            t.header.stamp = rospy.Time.now()
            t.ns = "fix_labels"
            t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(x=a["map_x"], y=a["map_y"] + 0.4, z=0.0)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.4
            t.color = ColorRGBA(r=0.6, g=0.0, b=0.0, a=1.0)
            t.text = str(a["point_id"])
            ma.markers.append(t)
        self.pub_fixes.publish(ma)

    def _last_degenerate(self):
        for w in reversed(self.est.warnings):
            if w.startswith("DEGENERATE"):
                return w
        return "DEGENERATE baseline < %.2fm" % MIN_BASELINE_M

    def _publish_status(self):
        # Caller holds self.lock (or single-threaded init). Reads est state; publishes status marker + string.
        if self.use_filter:
            if self._live_mode == "LOCALIZED":
                state = "LOCALIZED  fixes=%d  scale=%.3f" % (self.est.n_fixes, self.est.scale)
                col = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
            elif self._live_mode == "COARSE":
                state = "TRACKING-COARSE  fixes=%d  (yaw-aligned, waiting separated fix)" % self.est.n_fixes
                col = ColorRGBA(r=0.95, g=0.6, b=0.1, a=1.0)
            else:
                state = "ACQUIRING  fixes=%d/%d  (no map anchor yet)" % (self.est.n_fixes, self.min_fixes)
                col = ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0)
        elif self.est.localized:
            state = "LOCALIZED  fixes=%d  scale=%.3f" % (self.est.n_fixes, self.est.scale)
            col = ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0)
        elif self.est.degenerate:
            state = "ACQUIRING  fixes=%d/%d  %s" % (self.est.n_fixes, self.min_fixes, self._last_degenerate())
            col = ColorRGBA(r=0.95, g=0.6, b=0.1, a=1.0)
        else:
            state = "ACQUIRING  fixes=%d/%d  (no position: no trustworthy transform yet)" % (
                self.est.n_fixes, self.min_fixes)
            col = ColorRGBA(r=0.9, g=0.2, b=0.2, a=1.0)
        lines = [state]
        extra = None
        if not self.map_loaded:
            extra = "MAP LOAD FAILED - display without base map"
        elif self.use_filter and self.kf.warnings:
            extra = self.kf.warnings[-1]        # surface filter DROP/DECIMATE diagnostics at runtime
        elif self.est.warnings and self.est.warnings[-1] not in state:
            extra = self.est.warnings[-1]
        if extra:
            lines.append(extra)
        txt = "\n".join(lines)
        self.pub_status_s.publish(String(data=state))
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns = "status"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position = Point(x=self._text_pos[0], y=self._text_pos[1], z=0.0)
        m.pose.orientation.w = 1.0
        m.scale.z = 0.8
        m.color = col
        m.text = txt
        self.pub_status_m.publish(m)


if __name__ == "__main__":
    rospy.init_node("reloc_display")
    DisplayNode()
    rospy.spin()
