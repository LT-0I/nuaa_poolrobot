#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sonar_stitch_node.py - 声呐拼图自动触发 ROS 节点 (接入层, 不含拼接算法)

职责:
  订阅 /sonar_scan/session_done (std_msgs/String = 本次沿边采集会话目录绝对路径),
  采集完成 -> 后台线程调 sonar_stitch.main() 对该目录【离线拼图两遍, 仅 MIN_INNER 不同】, 出两张图:
    <session>/stitching_loc.pcd  (MIN_INNER 低=60 稠密: 点多, 供 g 单点声呐定位 ICP)
    <session>/stitching_view.pcd (MIN_INNER 高=90 干净: 去鬼影, 供 g rviz 显示底图)
  -> 发 /sonar_stitch/done(Bool) 与 (可选) /sonar_stitch/cloud(PointCloud2, latched, =view 干净图) 供 RViz。

设计要点:
  - 拼接算法在 sonar_stitch.py(逐字搬自原拼图程序), 本节点不碰算法, 只换 MIN_INNER/输出路径跑两遍。
  - 拼图重(open3d/scipy/pandas) 且耗时, 放后台线程跑, 不阻塞 ROS 回调; 每张图同会话只拼一次(幂等)。
  - 终端打印一律 ASCII(机上无中文字库)。
  - open3d/scipy/pandas 缺失时给清晰 ASCII 提示, 不让节点崩。
"""

import os
import sys
import threading
import contextlib

import rospy
from std_msgs.msg import String, Bool

# 让本节点能 import 同目录下的 sonar_stitch.py (源码树 / catkin 安装目录都成立)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 版本横幅(grep 自判): 改本节点编排行为必改此串。双图编排(loc=60稠密定位 / view=90干净显示)。
STITCH_NODE_VERSION = "sonar-stitch-node-twomap-loc60-view90-20260626"


class SonarStitchNode(object):
    def __init__(self):
        self.publish_cloud = rospy.get_param("~publish_cloud", True)
        self.frame_id = rospy.get_param("~frame_id", "map")

        # 共享逐帧滤波参数(两张图通用): 默认用 sonar_stitch 模块常量; 仅显式给 ~stitch_* 才覆盖,
        # 现场不改脚本即可调滤波, 拼接算法本身不动。MIN_INNER 不在此处(分 loc/view 两档, 见下)。
        self._filter_param_map = (
            ("MIN_DIST", "~stitch_min_dist", float),
            ("MAX_DIST", "~stitch_max_dist", float),
            ("MIN_OUTER", "~stitch_min_outer", float),
            ("MAX_LIMIT", "~stitch_max_limit", float),
            ("ARC_EPS", "~stitch_arc_eps", float),
            ("ARC_MIN_POINTS", "~stitch_arc_min_points", int),
            ("ARC_MAX_RADIUS_STD", "~stitch_arc_max_radius_std", float),
        )
        self._filter_overrides = {}
        for attr, pname, cast in self._filter_param_map:
            if rospy.has_param(pname):
                try:
                    self._filter_overrides[attr] = cast(rospy.get_param(pname))
                except (TypeError, ValueError):
                    rospy.logwarn("[sonar_stitch_node] bad %s, ignore override", pname)

        # 双图档位: 同一 main() 跑两遍, 仅 MIN_INNER 不同 -> 两张用途不同的图(同会话目录, 文件名固定)。
        #   loc  = 稠密(MIN_INNER 低, 默认60): 点多, 供 g 单点声呐定位 ICP。
        #   view = 干净(MIN_INNER 高, 默认90): 点少去鬼影, 供 g rviz 显示底图。
        min_inner_loc = float(rospy.get_param("~stitch_min_inner_loc", 60.0))
        min_inner_view = float(rospy.get_param("~stitch_min_inner_view", 90.0))
        self._map_specs = (
            ("loc",  min_inner_loc,  "stitching_loc.pcd"),
            ("view", min_inner_view, "stitching_view.pcd"),
        )
        # 旧单图参数 ~stitch_min_inner 在双图模式已弃用(拆成 loc/view 两档); 若仍被传入则明确告警, 不静默忽略。
        if rospy.has_param("~stitch_min_inner"):
            rospy.logwarn("[sonar_stitch_node] ~stitch_min_inner is DEPRECATED in two-map mode and IGNORED; "
                          "use ~stitch_min_inner_loc (default 60) / ~stitch_min_inner_view (default 90)")

        self._lock = threading.Lock()
        self._busy = False
        self._processed = set()

        self.done_pub = rospy.Publisher("/sonar_stitch/done", Bool, queue_size=1, latch=True)
        self.cloud_pub = None
        if self.publish_cloud:
            from sensor_msgs.msg import PointCloud2
            self.cloud_pub = rospy.Publisher("/sonar_stitch/cloud", PointCloud2, queue_size=1, latch=True)

        # latched 话题: 订阅即收到采集端最后一次发布的会话目录
        self.sub = rospy.Subscriber("/sonar_scan/session_done", String, self._on_session_done, queue_size=5)
        rospy.loginfo("[sonar_stitch_node] up (ver=%s). waiting /sonar_scan/session_done (publish_cloud=%s)",
                      STITCH_NODE_VERSION, str(self.publish_cloud))
        rospy.loginfo("[sonar_stitch_node] two maps per session: loc(MIN_INNER=%s)->%s | view(MIN_INNER=%s)->%s",
                      str(self._map_specs[0][1]), self._map_specs[0][2],
                      str(self._map_specs[1][1]), self._map_specs[1][2])

    def _on_session_done(self, msg):
        session_dir = (msg.data or "").strip()
        if not session_dir:
            rospy.logwarn("[sonar_stitch_node] empty session_dir, ignore")
            return
        with self._lock:
            if session_dir in self._processed:
                rospy.loginfo("[sonar_stitch_node] already stitched, skip: %s", session_dir)
                return
            if self._busy:
                rospy.logwarn("[sonar_stitch_node] busy stitching, ignore new: %s", session_dir)
                return
            self._busy = True
            self._processed.add(session_dir)
        t = threading.Thread(target=self._run_stitch, args=(session_dir,))
        t.daemon = True
        t.start()

    def _run_stitch(self, session_dir):
        ok = False
        try:
            if not os.path.isdir(session_dir):
                rospy.logerr("[sonar_stitch_node] session_dir not found: %s", session_dir)
                return
            try:
                import sonar_stitch
            except ImportError as e:
                rospy.logerr("[sonar_stitch_node] missing python deps for stitching: %s", str(e))
                rospy.logerr("[sonar_stitch_node] install on this machine: "
                             "pip3 install open3d scipy pandas numpy")
                return

            # 共享滤波覆盖(~stitch_*, 不含 MIN_INNER): 两张图通用, 拼前写进 sonar_stitch 模块常量一次。
            for attr, val in self._filter_overrides.items():
                old = getattr(sonar_stitch, attr, None)
                setattr(sonar_stitch, attr, val)
                rospy.loginfo("[sonar_stitch_node] filter override %s: %s -> %s",
                              attr, str(old), str(val))

            resolved = sonar_stitch._resolve_paths(session_dir)   # 设 INPUT/FILTER/ODOM; OUTPUT_PCD 逐图覆盖
            view_pcd_for_rviz = None
            all_ok = True
            # 同一 main() 跑两遍: 每遍只改 MIN_INNER(loc/view 档) + OUTPUT_PCD(本图路径), 拼接算法不动。
            for tag, min_inner, out_name in self._map_specs:
                out_pcd = os.path.join(resolved, out_name)
                tmp_pcd = out_pcd + ".tmp"          # 先写临时名, 整图写完才 os.replace 落定 -> 真名永远是完整图
                if tag == "view":
                    view_pcd_for_rviz = out_pcd
                setattr(sonar_stitch, "MIN_INNER", float(min_inner))   # 逐帧滤波/main() 运行期读此模块全局
                setattr(sonar_stitch, "OUTPUT_PCD", tmp_pcd)           # main() 写临时名; 真名只在成功 rename 后出现
                # 幂等只认【真名】: 该图已拼过(节点重启 / latched 重投)则跳过, 不白耗机上 CPU。
                # main() 中途被打断(OOM/掉电)只留 .tmp, 真名不存在 -> 重跑重生, 绝不把截断坏档当成品跳过。
                if os.path.isfile(out_pcd):
                    rospy.loginfo("[sonar_stitch_node] [%s] output exists, skip re-stitch: %s", tag, out_pcd)
                    continue
                log_path = os.path.join(resolved, "stitch_%s.log" % tag)
                rospy.loginfo("[sonar_stitch_node] [%s] stitch START (ver=%s, MIN_INNER=%s), "
                              "may take minutes; log -> %s", tag,
                              getattr(sonar_stitch, "STITCH_VERSION", "?"), str(min_inner), log_path)
                # 拼接算法(逐字搬自原拼图程序)内含中文 print; 重定向到日志文件, 保机上终端打印 ASCII 干净
                with open(log_path, "w") as lf:
                    with contextlib.redirect_stdout(lf):
                        sonar_stitch.main()
                if os.path.isfile(tmp_pcd):
                    os.replace(tmp_pcd, out_pcd)   # 同目录原子改名: 真名一旦出现即完整图(防截断坏档被当成品)
                    rospy.loginfo("[sonar_stitch_node] [%s] stitch DONE -> %s", tag, out_pcd)
                else:
                    all_ok = False
                    rospy.logerr("[sonar_stitch_node] [%s] stitch finished but no output: %s", tag, tmp_pcd)
            ok = all_ok
            # RViz 底图发干净的 view 图(view 缺则回退发任一已生成的图)
            if self.cloud_pub is not None:
                pub_pcd = view_pcd_for_rviz
                if pub_pcd is None or not os.path.isfile(pub_pcd):
                    pub_pcd = None
                    for _tag, _mi, out_name in self._map_specs:
                        cand = os.path.join(resolved, out_name)
                        if os.path.isfile(cand):
                            pub_pcd = cand
                            break
                if pub_pcd is not None:
                    self._publish_cloud(pub_pcd, sonar_stitch)
        except Exception as e:  # noqa: BLE001 - 拼图失败不能拖垮节点
            rospy.logerr("[sonar_stitch_node] stitch failed: %s", str(e))
        finally:
            try:
                self.done_pub.publish(Bool(data=ok))
            except Exception:
                pass
            with self._lock:
                self._busy = False

    def _publish_cloud(self, pcd_path, sonar_stitch):
        try:
            import numpy as np
            from sensor_msgs.msg import PointCloud2, PointField
            from sensor_msgs import point_cloud2 as pc2
            from std_msgs.msg import Header

            pcd, intensity = sonar_stitch.read_pcd_binary_xyzi(pcd_path)
            pts = np.asarray(pcd.points)
            if len(pts) == 0:
                rospy.logwarn("[sonar_stitch_node] stitched cloud empty, skip publish")
                return
            inten = np.asarray(intensity).reshape(-1) if intensity is not None else np.zeros(len(pts))
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("intensity", 12, PointField.FLOAT32, 1),
            ]
            data = [(float(p[0]), float(p[1]), float(p[2]), float(inten[i]))
                    for i, p in enumerate(pts)]
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.frame_id
            cloud = pc2.create_cloud(header, fields, data)
            self.cloud_pub.publish(cloud)
            rospy.loginfo("[sonar_stitch_node] published /sonar_stitch/cloud (%d pts, frame=%s)",
                          len(data), self.frame_id)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("[sonar_stitch_node] publish cloud failed (non-fatal): %s", str(e))


def main():
    rospy.init_node("sonar_stitch_node")
    SonarStitchNode()
    rospy.spin()


if __name__ == "__main__":
    main()
