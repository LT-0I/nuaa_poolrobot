#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Replay driver (TEST TOOLING, not on-robot). Feeds display_node the recorded streams as if live:
  publishes /wheel_odom (nav_msgs/Odometry) time-paced from odom_full.csv (sped up by ~speed),
  publishes /reloc/fix (std_msgs/Float64MultiArray) when odom reaches each fix's activation seq
    (odom_full_seq_end from points_index), carrying [pid, map_x,map_y,map_yaw, anchor_odom_x,y,yaw]
    where anchor odom = odom_full row at that seq, map pose = poses_in_map (Layer A output / golden).
This stands in for the Layer A localization node during offline rviz review.
On-robot the real Layer A node publishes /reloc/fix at each scan; this replay is ONLY for the demo.
Terminal output ASCII only.
"""
import csv
import math
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def load_csv(p):
    with open(p) as f:
        return list(csv.DictReader(f))


def q_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main():
    rospy.init_node("reloc_replay")
    task = rospy.get_param("~task_dir")
    speed = float(rospy.get_param("~speed", 20.0))
    odom_frame = rospy.get_param("~odom_frame", "odom")
    # Model localization compute latency: publish each fix this many RECORDING seconds AFTER its scan end,
    # so the robot has already moved on (fix binds to a PAST node -> retroactive correction is visible). 0 = legacy.
    fix_delay = float(rospy.get_param("~fix_delay_sec", 0.0))

    odom = load_csv(task + "/odom_full.csv")
    pidx = load_csv(task + "/points_index.csv")
    poses = load_csv(task + "/auto_relocalization_output/multi_relocalization_output/poses_in_map.csv")
    pose_by_id = {int(float(r["point_id"])): r for r in poses}

    # odom row lookup by seq (for anchor odom)
    odom_by_seq = {int(float(r["seq"])): r for r in odom}
    fixes = []  # (activation_seq, pid, map_x,map_y,map_yaw, anch_ox,anch_oy,anch_oyaw, act_wall)
    for p in pidx:
        pid = int(float(p["point_id"]))
        if pid not in pose_by_id:
            continue
        act_seq = int(float(p["odom_full_seq_end"]))
        arow = odom_by_seq.get(act_seq)
        if arow is None:
            # nearest
            ks = min(odom_by_seq.keys(), key=lambda k: abs(k - act_seq))
            arow = odom_by_seq[ks]
        mp = pose_by_id[pid]
        fixes.append((act_seq, pid,
                      float(mp["x_map"]), float(mp["y_map"]), float(mp["yaw_map_rad"]),
                      float(arow["x"]), float(arow["y"]), float(arow["yaw_rad"]),
                      float(arow["wall_t"])))
    fixes.sort(key=lambda t: t[8] + fix_delay)   # emit in (scan_end + delay) order
    rospy.loginfo("reloc_replay: %d odom rows, %d fixes, speed=%.1fx, fix_delay=%.1fs",
                  len(odom), len(fixes), speed, fix_delay)

    pub_odom = rospy.Publisher("/wheel_odom", Odometry, queue_size=50)
    pub_fix = rospy.Publisher("/reloc/fix", Float64MultiArray, queue_size=10)
    rospy.sleep(1.5)  # let subscribers connect

    fi = 0
    t0_wall = float(odom[0]["wall_t"])
    t_start = rospy.Time.now().to_sec()
    for r in odom:
        if rospy.is_shutdown():
            break
        seq = int(float(r["seq"]))
        wall = float(r["wall_t"])
        # publish due fixes (when recording time reaches scan_end + fix_delay; anchor stays the PAST scan-end odom)
        while fi < len(fixes) and wall >= fixes[fi][8] + fix_delay:
            f = fixes[fi]
            pub_fix.publish(Float64MultiArray(data=[float(f[1]), f[2], f[3], f[4], f[5], f[6], f[7]]))
            rospy.loginfo("reloc_replay: publish FIX point=%d at seq=%d (scan_end_seq=%d, delayed %.1fs)",
                          f[1], seq, f[0], fix_delay)
            fi += 1
        # pace to wall clock / speed
        target = t_start + (float(r["wall_t"]) - t0_wall) / speed
        dt = target - rospy.Time.now().to_sec()
        if dt > 0:
            rospy.sleep(min(dt, 0.5))
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = odom_frame
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = float(r["x"])
        od.pose.pose.position.y = float(r["y"])
        od.pose.pose.orientation = q_yaw(float(r["yaw_rad"]))
        pub_odom.publish(od)
    # flush any remaining fixes
    while fi < len(fixes):
        f = fixes[fi]
        pub_fix.publish(Float64MultiArray(data=[float(f[1]), f[2], f[3], f[4], f[5], f[6], f[7]]))
        fi += 1
    rospy.loginfo("reloc_replay: done.")


if __name__ == "__main__":
    main()
