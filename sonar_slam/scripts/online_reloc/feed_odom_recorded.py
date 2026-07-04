#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ODOM feeder for the ON-ROBOT g localization+display SIM (recorded data, no live robot).
Publishes /wheel_odom paced from a recorded task's odom_full.csv, and NOTHING else -- so the
online_reloc node stays the sole publisher of /reloc/fix (that is what we are exercising).

WHY THE POST-EOF HOLD:
  The recorded odom is ENTIRELY in the past. The display's live (green) lane only renders AFTER
  the first localization fix arrives -- and on the Orange Pi 5 (ARM) that first fix runs a global
  BBS that takes MINUTES, i.e. long after the ~tens-of-seconds odom replay has finished. If the
  feeder simply stopped at EOF, no /wheel_odom sample would be left to render the live pose, so the
  green lane would NEVER appear (a healthy pipeline would look broken). To fix that, after replay we
  keep republishing the LAST pose at ~hold_hz until shutdown: once the first fix lands there is still
  a /wheel_odom sample to draw the live pose, and it then tracks each fix's refinement. Republishing
  the IDENTICAL last pose adds no KF trajectory node (its step is below node_min_step) so it cannot
  distort the trajectory. Set ~hold_hz:=0 to disable the hold (green lane then stays unrendered).
This is TEST TOOLING for the sim; it is NOT on-robot production code. ASCII output only.
"""
import csv
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def q_yaw(yaw):
    # yaw is used AS-IS (no negation): the odom y/yaw-vs-reality mirror was resolved on the robot
    # (2026-07-03), so recorded odom already matches the real heading.
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main():
    rospy.init_node("online_feed_odom")
    task = rospy.get_param("~task_dir")
    speed = float(rospy.get_param("~speed", 20.0))
    odom_frame = rospy.get_param("~odom_frame", "odom")
    hold_hz = float(rospy.get_param("~hold_hz", 5.0))    # after EOF, republish last pose at this rate; 0 = off
    start_delay = float(rospy.get_param("~start_delay", 1.5))   # wait before publishing so subscribers (display) are up; raise it if the feeder is launched together with the nodes (ARM startup is slow)
    if speed <= 0.0:
        rospy.logwarn("online_feed_odom: ~speed=%.3f invalid -> using 1.0", speed)
        speed = 1.0

    with open(task + "/odom_full.csv") as f:
        odom = list(csv.DictReader(f))
    if not odom:
        rospy.logfatal("online_feed_odom: %s/odom_full.csv has no rows -- nothing to replay.", task)
        return

    pub = rospy.Publisher("/wheel_odom", Odometry, queue_size=50)
    rospy.loginfo("online_feed_odom: %d odom rows, speed=%.1fx, hold_hz=%.1f (fix topic owned by "
                  "online node)", len(odom), speed, hold_hz)
    def make_od(r):
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = odom_frame
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = float(r["x"])
        od.pose.pose.position.y = float(r["y"])
        od.pose.pose.orientation = q_yaw(float(r["yaw_rad"]))
        return od

    # replay (Ctrl-C anywhere in here -> clean return: no traceback, no spurious non-zero exit)
    try:
        rospy.sleep(start_delay)
        t0 = float(odom[0]["wall_t"])
        ts = rospy.Time.now().to_sec()
        for r in odom:
            if rospy.is_shutdown():
                return
            target = ts + (float(r["wall_t"]) - t0) / speed
            dt = target - rospy.Time.now().to_sec()
            while dt > 0 and not rospy.is_shutdown():     # honor gaps > 0.5s too (loop, not a single clamp)
                rospy.sleep(min(dt, 0.5))
                dt = target - rospy.Time.now().to_sec()
            pub.publish(make_od(r))
    except rospy.ROSInterruptException:
        return
    rospy.loginfo("online_feed_odom: replay done (%d rows).", len(odom))

    # HOLD: keep the last pose alive so the live/green lane can render once the first fix arrives.
    if hold_hz > 0.0:
        last = odom[-1]
        rospy.loginfo("online_feed_odom: HOLD -- republishing the last pose at %.1f Hz until shutdown "
                      "(keeps the live/green lane renderable once the first fix arrives; Ctrl-C to stop).",
                      hold_hz)
        rate = rospy.Rate(hold_hz)
        while not rospy.is_shutdown():
            pub.publish(make_od(last))
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
    rospy.loginfo("online_feed_odom: done.")


if __name__ == "__main__":
    main()
