#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sonar_stitch_node.py - 声呐拼图自动触发 ROS 节点 (接入层, 不含拼接算法)

职责:
  订阅 /sonar_scan/session_done (std_msgs/String = 本次沿边采集会话目录绝对路径),
  采集完成 -> 后台线程调 sonar_stitch.main() 对该目录离线拼图 -> 出 <session>/stitching.pcd
  -> 发 /sonar_stitch/done(Bool) 与 (可选) /sonar_stitch/cloud(PointCloud2, latched) 供 RViz。

设计要点:
  - 拼接算法在 sonar_stitch.py(逐字搬自原拼图程序), 本节点不碰算法。
  - 拼图重(open3d/scipy/pandas) 且耗时, 放后台线程跑, 不阻塞 ROS 回调; 同一目录只拼一次。
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


class SonarStitchNode(object):
    def __init__(self):
        self.publish_cloud = rospy.get_param("~publish_cloud", True)
        self.frame_id = rospy.get_param("~frame_id", "map")

        # 逐帧滤波参数(对齐 lvbo): 默认用 sonar_stitch 模块常量(MIN_INNER=90 等);
        # 仅当显式给了 ~stitch_* 私有参数才覆盖, 现场不改脚本即可调滤波, 拼接算法本身不动。
        self._filter_param_map = (
            ("MIN_DIST", "~stitch_min_dist", float),
            ("MAX_DIST", "~stitch_max_dist", float),
            ("MIN_INNER", "~stitch_min_inner", float),
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
        rospy.loginfo("[sonar_stitch_node] up. waiting /sonar_scan/session_done (publish_cloud=%s)",
                      str(self.publish_cloud))

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

            # 把现场给的 ~stitch_* 覆盖写进 sonar_stitch 模块常量(逐帧滤波/main() 运行期读这些全局)。
            for attr, val in self._filter_overrides.items():
                old = getattr(sonar_stitch, attr, None)
                setattr(sonar_stitch, attr, val)
                rospy.loginfo("[sonar_stitch_node] filter override %s: %s -> %s",
                              attr, str(old), str(val))

            resolved = sonar_stitch._resolve_paths(session_dir)
            out_pcd = sonar_stitch.OUTPUT_PCD
            # 幂等: 该会话已拼过(节点重启 / latched 话题重投)则跳过, 不白耗机上 CPU
            if os.path.isfile(out_pcd):
                ok = True
                rospy.loginfo("[sonar_stitch_node] output exists, skip re-stitch: %s", out_pcd)
                if self.cloud_pub is not None:
                    self._publish_cloud(out_pcd, sonar_stitch)
                return
            log_path = os.path.join(resolved, "stitch.log")
            rospy.loginfo("[sonar_stitch_node] stitch START (ver=%s), may take minutes; log -> %s",
                          getattr(sonar_stitch, "STITCH_VERSION", "?"), log_path)
            # 拼接算法(逐字搬自原拼图程序)内含中文 print; 重定向到日志文件, 保机上终端打印 ASCII 干净
            with open(log_path, "w") as lf:
                with contextlib.redirect_stdout(lf):
                    sonar_stitch.main()
            if os.path.isfile(out_pcd):
                ok = True
                rospy.loginfo("[sonar_stitch_node] stitch DONE -> %s", out_pcd)
                if self.cloud_pub is not None:
                    self._publish_cloud(out_pcd, sonar_stitch)
            else:
                rospy.logerr("[sonar_stitch_node] stitch finished but no output: %s", out_pcd)
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
