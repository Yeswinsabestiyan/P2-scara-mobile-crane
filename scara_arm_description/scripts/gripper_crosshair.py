#!/usr/bin/env python3
"""
gripper_crosshair.py  –  RViz visual aid markers for the SCARA crane.

Subscribes:
  /crane_telemetry  (std_msgs/String) – JSON state from scara_teleop

Publishes:
  /crane_markers    (visualization_msgs/MarkerArray)

Markers:
  • Downward LASER CROSSHAIR: bright-green vertical line + horizontal X
    dropped from gripper to the ground  (helps target cargo from above)
  • END-EFFECTOR TRAIL: a faint cyan line strip of the last 200 gripper
    positions, showing the path the arm has taken
  • REACHABILITY RING: a thin circle on the ground at the max arm reach
    radius from the current chassis position

Run in its own terminal:
    ros2 run scara_arm_description gripper_crosshair.py

Also add /crane_markers to an RViz MarkerArray display to see the overlays.
"""

import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

# ── constants ──────────────────────────────────────────────────────────────────
L1, L2 = 0.30, 0.25
MAX_REACH = L1 + L2            # 0.55 m
RING_SEGMENTS = 64             # polygon approximation for reach ring
TRAIL_LEN = 200                # how many past positions to keep
FRAME = 'world'

# Marker IDs
ID_CROSSHAIR_V  = 0    # vertical laser line
ID_CROSSHAIR_H1 = 1    # horizontal cross arm 1
ID_CROSSHAIR_H2 = 2    # horizontal cross arm 2
ID_TRAIL        = 3    # trajectory trail
ID_REACH_RING   = 4    # reachability circle


class CrosshairNode(Node):
    def __init__(self):
        super().__init__('gripper_crosshair')
        self.pub = self.create_publisher(MarkerArray, '/crane_markers', 10)
        self.create_subscription(String, '/crane_telemetry', self._on_tele, 10)

        self._trail = deque(maxlen=TRAIL_LEN)
        self._last  = {}

    def _on_tele(self, msg: String):
        try:
            t = json.loads(msg.data)
        except Exception:
            return
        self._last = t
        self._publish_markers(t)

    def _publish_markers(self, t: dict):
        x    = t.get('x', 0.0)
        y    = t.get('y', 0.0)
        lift = t.get('lift', 0.0)
        j1   = math.radians(t.get('j1_deg', 0.0))
        j2   = math.radians(t.get('j2_deg', 0.0))
        j3   = t.get('j3', 0.0)
        head = math.radians(t.get('heading_deg', 0.0))

        # Compute gripper world position (same formula as teleop)
        xc = L1 * math.cos(j1) + L2 * math.cos(j1 + j2)
        yc = L1 * math.sin(j1) + L2 * math.sin(j1 + j2)
        c, s = math.cos(head), math.sin(head)
        GRIPPER_Z_BASE = 0.41
        gx = x + xc * c - yc * s
        gy = y + xc * s + yc * c
        gz = GRIPPER_Z_BASE + lift + j3

        now = self.get_clock().now().to_msg()
        ma  = MarkerArray()

        # ── CROSSHAIR: vertical laser ─────────────────────────────────────────
        v_marker = Marker()
        v_marker.header.frame_id = FRAME
        v_marker.header.stamp    = now
        v_marker.ns   = 'crosshair'
        v_marker.id   = ID_CROSSHAIR_V
        v_marker.type = Marker.LINE_LIST
        v_marker.action = Marker.ADD
        v_marker.scale.x = 0.004   # line width
        v_marker.color.r = 0.0
        v_marker.color.g = 1.0
        v_marker.color.b = 0.3
        v_marker.color.a = 0.9
        # line from gripper down to ground
        p_top    = Point(x=gx, y=gy, z=gz)
        p_bottom = Point(x=gx, y=gy, z=0.001)
        v_marker.points = [p_top, p_bottom]
        ma.markers.append(v_marker)

        # ── CROSSHAIR: horizontal cross on the ground ─────────────────────────
        ARM = 0.06
        for mid, dx, dy in [(ID_CROSSHAIR_H1, ARM, 0), (ID_CROSSHAIR_H2, 0, ARM)]:
            h = Marker()
            h.header.frame_id = FRAME
            h.header.stamp    = now
            h.ns   = 'crosshair'
            h.id   = mid
            h.type = Marker.LINE_LIST
            h.action = Marker.ADD
            h.scale.x = 0.005
            h.color.r = 0.0
            h.color.g = 1.0
            h.color.b = 0.3
            h.color.a = 0.8
            h.points = [
                Point(x=gx - dx, y=gy - dy, z=0.003),
                Point(x=gx + dx, y=gy + dy, z=0.003),
            ]
            ma.markers.append(h)

        # ── TRAIL: end-effector path ──────────────────────────────────────────
        self._trail.append((gx, gy, gz))
        trail = Marker()
        trail.header.frame_id = FRAME
        trail.header.stamp    = now
        trail.ns   = 'trail'
        trail.id   = ID_TRAIL
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.scale.x = 0.006
        trail.color.r = 0.58
        trail.color.g = 0.89
        trail.color.b = 0.83
        trail.color.a = 0.6
        for (tx, ty, tz) in self._trail:
            trail.points.append(Point(x=tx, y=ty, z=tz))
        ma.markers.append(trail)

        # ── REACHABILITY RING ─────────────────────────────────────────────────
        ring = Marker()
        ring.header.frame_id = FRAME
        ring.header.stamp    = now
        ring.ns   = 'reach'
        ring.id   = ID_REACH_RING
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.005
        ring.color.r = 0.90
        ring.color.g = 0.55
        ring.color.b = 0.20
        ring.color.a = 0.45
        for i in range(RING_SEGMENTS + 1):
            ang = 2.0 * math.pi * i / RING_SEGMENTS
            ring.points.append(Point(
                x=x + MAX_REACH * math.cos(ang),
                y=y + MAX_REACH * math.sin(ang),
                z=0.005,
            ))
        ma.markers.append(ring)

        self.pub.publish(ma)


def main():
    rclpy.init()
    node = CrosshairNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
