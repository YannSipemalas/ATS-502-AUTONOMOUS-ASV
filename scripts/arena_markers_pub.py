#!/usr/bin/env python3
"""
Arena marker publisher.
Mengirim MarkerArray ke /arena_markers agar bola-bola dan kotak arena
terlihat di RViz2 tanpa harus bridge setiap model dari Gazebo.

Jalankan: python3 arena_markers_pub.py
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

# ===== Layout dibaca dari config/arena_layout.json (ditulis generate_arena.py) =====
import json
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT, "config", "arena_layout.json")) as f:
    LAYOUT = json.load(f)
BALL_R = LAYOUT["ball_r"]
GREEN = LAYOUT["green"]
RED = LAYOUT["red"]
BOXES = LAYOUT["boxes"]
TRACK = LAYOUT["track"]
ARENA_X, ARENA_Y = LAYOUT["arena"]

# Semua marker dipublish di frame "world" (Fixed Frame RViz).
# Koordinat langsung dari arena_layout.json (sudah dalam meter dunia).
FRAME = "world"
# ================================================================================


class ArenaMarkersNode(Node):
    def __init__(self):
        super().__init__("arena_markers_pub")
        # Pakai sim time agar stamp marker sinkron dengan RViz
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/arena_markers", qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info("Arena marker publisher started")

    def publish(self):
        ma = MarkerArray()
        uid = 0
        now = self.get_clock().now().to_msg()

        # Bola hijau
        for name, (x, y) in GREEN.items():
            m = self._sphere(uid, x, y, BALL_R, (0,1,0), f"ball_green_{name}")
            m.header.stamp = now; ma.markers.append(m); uid += 1

        # Bola merah
        for name, (x, y) in RED.items():
            m = self._sphere(uid, x, y, BALL_R, (1,0,0), f"ball_red_{name}")
            m.header.stamp = now; ma.markers.append(m); uid += 1

        # Kotak arena
        for name, (bx, by, bz, sx, sy, sz, rgb) in BOXES.items():
            m = self._box(uid, bx, by, bz, sx, sy, sz, rgb, name)
            m.header.stamp = now; ma.markers.append(m); uid += 1

        path_pts = TRACK
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = now
        m.ns = "track"
        m.id = uid; uid += 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.5
        m.pose.orientation.w = 1.0
        for px, py in path_pts:
            p = Point(); p.x = px; p.y = py; p.z = 0.003
            m.points.append(p)
        ma.markers.append(m)

        self.pub.publish(ma)
        self.get_logger().info(f"Published {len(ma.markers)} markers", once=True)

    def _sphere(self, uid, x, y, r, rgb, ns="balls"):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = ns
        m.id = uid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = r
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = r * 2
        m.color.r, m.color.g, m.color.b = (float(c) for c in rgb)
        m.color.a = 1.0
        return m

    def _box(self, uid, x, y, z, sx, sy, sz, rgb, ns="boxes"):
        m = Marker()
        m.header.frame_id = FRAME
        m.ns = ns
        m.id = uid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx; m.scale.y = sy; m.scale.z = sz
        m.color.r, m.color.g, m.color.b = (float(c) for c in rgb)
        m.color.a = 0.7
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ArenaMarkersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
