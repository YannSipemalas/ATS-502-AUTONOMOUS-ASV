#!/usr/bin/env python3
"""
auto_nav.py — Navigasi otonom ATS_SEMS5_PAK_YANN (ROS 2 Jazzy, rclpy).

Jalankan setelah run_sim.sh dan run_rviz.sh:
  python3 scripts/auto_nav.py --ros-args -p use_sim_time:=true

Verifikasi:
  ros2 topic echo /cmd_vel
  ros2 run tf2_ros tf2_echo world base_footprint
  ros2 topic hz /camera/image_raw
  find captured_images -type f
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Point, Twist
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf2_ros

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUT_PATH = os.path.join(ROOT, "config", "arena_layout.json")


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def dist_to_box(px, py, box):
    """Jarak titik ke tepi balok (axis-aligned). box = [cx, cy, cz, sx, sy, sz, rgb]."""
    cx, cy, _, sx, sy = box[0], box[1], box[2], box[3], box[4]
    dx = max(abs(px - cx) - sx / 2.0, 0.0)
    dy = max(abs(py - cy) - sy / 2.0, 0.0)
    return math.hypot(dx, dy)


# ──────────────────────────────────────────────────────────────
class WaypointLoader:
    GROUPS = {"right": 3, "top": 4, "left": 3}

    def __init__(self, path, logger, clearance=0.3):
        self.log = logger
        self.clearance = clearance
        with open(path) as f:
            self.layout = json.load(f)
        self.boxes = self.layout["boxes"]
        sp = self.layout["spawn"]
        self.start = (float(sp["x"]), float(sp["y"]))
        self.finish = self.start

    # ---- helper ----
    @staticmethod
    def _pair_geometry(xg, yg, xr, yr):
        """Kembalikan (mid_x, mid_y, normal_x, normal_y) untuk satu pasangan."""
        mx, my = (xg + xr) / 2.0, (yg + yr) / 2.0
        d = math.hypot(xg - xr, yg - yr)
        ax, ay = (xg - xr) / d, (yg - yr) / d  # sumbu hijau→merah
        return mx, my, -ay, ax                   # normal = tegak lurus sumbu

    def load(self):
        green, red = self.layout["green"], self.layout["red"]
        track = self.layout["track"]   # titik-titik lintasan arena
        wps = [
            ("start_straight", 3.900, 0.850),
            ("pre_right_3", 3.880, 1.050),
        ]

        gate_centers = []
        for group, count in self.GROUPS.items():
            for index in range(1, count + 1):
                key = f"{group}_{index}"
                mx, my, _, _ = self._pair_geometry(
                    *green[key], *red[key])
                gate_centers.append((key, mx, my))

        # Follow the arena's drawn track, inserting each gate midpoint on its
        # nearest track segment so the robot crosses the center of every gap.
        segment_gates = {}
        for name, gx, gy in gate_centers:
            best = None
            for segment_index, (start, end) in enumerate(
                    zip(track[1:-1], track[2:]), start=1):
                sx, sy = start
                dx, dy = end[0] - sx, end[1] - sy
                length_sq = dx * dx + dy * dy
                fraction = max(
                    0.0,
                    min(1.0, ((gx - sx) * dx + (gy - sy) * dy) / length_sq),
                )
                projected = (sx + fraction * dx, sy + fraction * dy)
                error_sq = (gx - projected[0]) ** 2 + (gy - projected[1]) ** 2
                candidate = (error_sq, segment_index, fraction)
                if best is None or candidate < best:
                    best = candidate
            _, segment_index, fraction = best
            segment_gates.setdefault(segment_index, []).append(
                (fraction, name, gx, gy))

        wps.append(("track_1", track[1][0], track[1][1]))
        for segment_index, track_point in enumerate(track[2:], start=1):
            for _, name, gx, gy in sorted(segment_gates.get(segment_index, [])):
                wps.append((name, gx, gy))
            wps.append((f"track_{segment_index + 1}", track_point[0], track_point[1]))

        wps.append(("finish", self.finish[0], self.finish[1]))
        self._report(wps)
        return wps

    def _report(self, wps):
        self.log.info("=== Daftar waypoint final ===")
        green, red = self.layout["green"], self.layout["red"]
        all_balls = []
        for k in list(green.keys()) + list(red.keys()):
            src = green if k in green else red
            all_balls.append(tuple(src[k]))

        prev = None
        for i, (name, x, y) in enumerate(wps):
            seg = "" if prev is None else f"  (+{math.hypot(x - prev[0], y - prev[1]):.3f} m)"
            # Jarak ke bola terdekat
            db = min(math.dist((x, y), b) for b in all_balls) if all_balls else 99
            warn_b = f"  [bola {db:.2f}m]" if db < 0.25 else ""
            self.log.info(f"{i:2d}. {name:22s} x={x:.3f} y={y:.3f}{seg}{warn_b}")
            prev = (x, y)
            for bname in ("box_capture_blue", "box_capture_green"):
                d = dist_to_box(x, y, self.boxes[bname])
                if d < self.clearance:
                    self.log.warn(f"    !! {name} hanya {d:.3f} m dari {bname} (< {self.clearance} m)")


# ──────────────────────────────────────────────────────────────
class PurePursuitController:
    def __init__(self, lookahead, v_max, w_max, goal_tol,
                 linear_accel, angular_accel, lateral_accel,
                 rotate_thresh=math.radians(150)):
        self.ld = lookahead
        self.v_max = v_max
        self.w_max = w_max
        self.goal_tol = goal_tol
        self.linear_accel = linear_accel
        self.angular_accel = angular_accel
        self.lateral_accel = lateral_accel
        self.rot_th = rotate_thresh

    def target_point(self, x, y, p0, p1):
        """Carrot at the lookahead-circle intersection with the active segment."""
        sx, sy = p1[0] - p0[0], p1[1] - p0[1]
        L = math.hypot(sx, sy)
        if L < 1e-6:
            return p1
        ux, uy = sx / L, sy / L
        projection = min(max((x - p0[0]) * ux + (y - p0[1]) * uy, 0.0), L)
        fx, fy = p0[0] - x, p0[1] - y
        b = 2.0 * (fx * ux + fy * uy)
        c = fx * fx + fy * fy - self.ld * self.ld
        discriminant = b * b - 4.0 * c
        if discriminant >= 0.0:
            root = math.sqrt(discriminant)
            candidates = [
                s for s in ((-b - root) / 2.0, (-b + root) / 2.0)
                if projection - 1e-6 <= s <= L
            ]
            if candidates:
                s = max(candidates)
                return p0[0] + ux * s, p0[1] + uy * s
        if math.hypot(p1[0] - x, p1[1] - y) <= self.ld:
            return p1
        s = min(projection + self.ld, L)
        return p0[0] + ux * s, p0[1] + uy * s

    def compute(self, x, y, yaw, target):
        dx, dy = target[0] - x, target[1] - y
        alpha = norm_angle(math.atan2(dy, dx) - yaw)
        if abs(alpha) > self.rot_th:
            return 0.0, math.copysign(self.w_max, alpha), 0.0
        ld = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * math.sin(alpha) / ld
        v = min(self.v_max, math.sqrt(self.lateral_accel / max(abs(curvature), 1e-6)))
        v *= max(0.25, math.cos(min(abs(alpha), math.pi / 2.0)))
        if abs(curvature) > 1e-6:
            v = min(v, self.w_max / abs(curvature))
        w = v * curvature
        return v, w, curvature


class SafetyController:
    """Check predicted motion against the arena's known static balls."""

    BODY_X_MIN = -0.04
    BODY_X_MAX = 0.17
    BODY_HALF_WIDTH = 0.106
    SAFETY_MARGIN = 0.015

    def __init__(self, layout):
        self.obstacles = [
            (float(x), float(y), float(layout["ball_r"]))
            for group in ("green", "red")
            for x, y in layout[group].values()
        ]

    def clearance(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        nearest = math.inf
        for ox, oy, radius in self.obstacles:
            dx, dy = ox - x, oy - y
            local_x, local_y = c * dx + s * dy, -s * dx + c * dy
            ex = max(self.BODY_X_MIN - local_x, 0.0, local_x - self.BODY_X_MAX)
            ey = max(abs(local_y) - self.BODY_HALF_WIDTH, 0.0)
            nearest = min(nearest, math.hypot(ex, ey) - radius)
        return nearest

    def motion_is_safe(self, x, y, yaw, v, w, horizon=0.35):
        current = self.clearance(x, y, yaw)
        predicted = []
        for step in range(1, 8):
            dt = horizon * step / 7.0
            if abs(w) < 1e-6:
                px, py = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
                pyaw = yaw
            else:
                pyaw = yaw + w * dt
                radius = v / w
                px = x + radius * (math.sin(pyaw) - math.sin(yaw))
                py = y - radius * (math.cos(pyaw) - math.cos(yaw))
            clearance = self.clearance(px, py, pyaw)
            predicted.append(clearance)
            if clearance < self.SAFETY_MARGIN and current >= self.SAFETY_MARGIN:
                return False
        if current < 0.0:
            return False
        if current < self.SAFETY_MARGIN:
            return min(predicted, default=current) >= current - 0.005
        return True


# ──────────────────────────────────────────────────────────────
class AutoNavNode(Node):
    def __init__(self):
        super().__init__("auto_nav")
        p = self.declare_parameter
        p("global_frame", "world")
        p("robot_frame", "base_footprint")
        p("lookahead", 0.28)
        p("v_max", 0.2)
        p("w_max", 1.0)
        p("goal_tol", 0.06)
        p("linear_accel_max", 0.35)
        p("angular_accel_max", 1.5)
        p("lateral_accel_max", 0.30)
        p("tf_timeout", 1.0)
        g = lambda n: self.get_parameter(n).value

        self.gframe, self.rframe = g("global_frame"), g("robot_frame")
        self.tf_timeout = float(g("tf_timeout"))

        loader = WaypointLoader(LAYOUT_PATH, self.get_logger())
        self.wps = loader.load()
        self.start = loader.start
        self.route = [self.start, *((wp[1], wp[2]) for wp in self.wps)]
        self.route_names = ["spawn", *(wp[0] for wp in self.wps)]
        self.route_lengths = [
            math.dist(start, end) for start, end in zip(self.route, self.route[1:])
        ]
        self.route_s = [0.0]
        for length in self.route_lengths:
            self.route_s.append(self.route_s[-1] + length)
        gate_names = {
            f"{group}_{index}"
            for group, count in loader.GROUPS.items()
            for index in range(1, count + 1)
        }
        self.gate_vertices = [
            index for index, name in enumerate(self.route_names) if name in gate_names
        ]
        self.gate_cursor = 0
        self.path_segment = 0
        self.path_progress = 0.0
        self.ctrl = PurePursuitController(
            g("lookahead"), g("v_max"), g("w_max"), g("goal_tol"),
            g("linear_accel_max"), g("angular_accel_max"), g("lateral_accel_max"))
        self.safety = SafetyController(loader.layout)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Marker, "/nav_path_strip", marker_qos)
        waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_waypoint_pub = self.create_publisher(
            String, "/nav/active_waypoint", waypoint_qos)
        self.active_waypoint_name = None
        self._publish_active_waypoint(self.route_names[1])
        self.publish_path()
        self.path_timer = self.create_timer(1.0, self.publish_path)

        self.done = False
        self.last_tf_ok = None
        self.tf_warned = False
        self.pose = None
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._safety_stopped = False
        self._last_control_time = self.get_clock().now()
        self._last_status_time = self._last_control_time

        self.get_logger().info(
            f"auto_nav siap: {len(self.wps)} waypoint, frame {self.gframe}->{self.rframe}, "
            f"lookahead={self.ctrl.ld:.2f} m, goal_tol={self.ctrl.goal_tol:.2f} m, "
            f"gate wajib dilewati={len(self.gate_vertices)}")
        self.create_timer(1.0 / 30.0, self.loop)

    def publish_path(self):
        marker = Marker()
        marker.header.frame_id = self.gframe
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = []
        for x, y in self.route:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.035
            marker.points.append(point)
        self.path_pub.publish(marker)

    def _publish_active_waypoint(self, name):
        if name == self.active_waypoint_name:
            return
        message = String()
        message.data = name
        self.active_waypoint_pub.publish(message)
        self.active_waypoint_name = name
        self.get_logger().info(f"Waypoint navigasi aktif: {name}")

    def _next_gate_vertex(self):
        if self.gate_cursor < len(self.gate_vertices):
            vertex = self.gate_vertices[self.gate_cursor]
            return vertex, self.route_names[vertex]
        return len(self.route) - 1, "finish"

    def _point_at(self, distance):
        distance = max(0.0, min(distance, self.route_s[-1]))
        for index, (start_s, end_s) in enumerate(zip(self.route_s, self.route_s[1:])):
            if distance <= end_s or index == len(self.route_lengths) - 1:
                segment_length = self.route_lengths[index]
                fraction = 0.0 if segment_length <= 1e-9 else (distance - start_s) / segment_length
                start, end = self.route[index], self.route[index + 1]
                return (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
        return self.route[-1]

    def _project_to_route(self, x, y, end_vertex):
        max_segment = end_vertex - 1
        if self.path_segment > max_segment:
            return
        best = None
        for index in range(self.path_segment, max_segment + 1):
            start, end = self.route[index], self.route[index + 1]
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_sq = dx * dx + dy * dy
            fraction = max(
                0.0,
                min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq),
            )
            px, py = start[0] + fraction * dx, start[1] + fraction * dy
            progress = self.route_s[index] + fraction * self.route_lengths[index]
            if progress < self.path_progress - 0.02:
                continue
            candidate = ((x - px) ** 2 + (y - py) ** 2, progress, index)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is not None:
            self.path_progress = max(self.path_progress, best[1])
            self.path_segment = max(self.path_segment, best[2])

    # ---- util ----
    def stop(self):
        """Emergency stop for lost TF, unsafe motion, shutdown, or finish."""
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self.cmd_pub.publish(Twist())

    def _publish_vel(self, v_target: float, w_target: float, dt: float):
        """Apply acceleration limits at the fixed control update rate."""
        dv = self.ctrl.linear_accel * dt
        dw = self.ctrl.angular_accel * dt
        self._v_cmd += max(-dv, min(dv, v_target - self._v_cmd))
        self._w_cmd += max(-dw, min(dw, w_target - self._w_cmd))
        cmd = Twist()
        cmd.linear.x = self._v_cmd
        cmd.angular.z = self._w_cmd
        self.cmd_pub.publish(cmd)

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.gframe, self.rframe, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    # ---- kontrol ----
    def loop(self):
        if self.done:
            return
        now = self.get_clock().now()
        dt = max(0.0, min(0.1, (now - self._last_control_time).nanoseconds * 1e-9))
        self._last_control_time = now

        pose = self.get_pose()
        if pose is None:
            if self.last_tf_ok is None or (now - self.last_tf_ok).nanoseconds * 1e-9 > self.tf_timeout:
                self.stop()
                if not self.tf_warned:
                    self.get_logger().warn(
                        f"TF {self.gframe}->{self.rframe} tidak tersedia > {self.tf_timeout}s, robot berhenti")
                    self.tf_warned = True
            return
        if self.tf_warned:
            self.get_logger().info("TF kembali tersedia, lanjut navigasi")
            self.tf_warned = False
        self.last_tf_ok = now
        self.pose = pose
        x, y, yaw = pose
        gate_vertex, name = self._next_gate_vertex()
        gx, gy = self.route[gate_vertex]
        gate_distance = math.hypot(gx - x, gy - y)
        if name == "finish" and gate_distance <= self.ctrl.goal_tol:
            self._publish_active_waypoint("finish")
            self.stop()
            self.done = True
            self.get_logger().info("FINISH tercapai. Robot berhenti.")
            return
        if name != "finish" and gate_distance <= self.ctrl.goal_tol:
            self.get_logger().info(
                f"Gate waypoint '{name}' dilewati aktual pada ({x:.3f}, {y:.3f}), "
                f"jarak={gate_distance:.3f} m")
            self.path_progress = max(self.path_progress, self.route_s[gate_vertex])
            self.gate_cursor += 1
            gate_vertex, name = self._next_gate_vertex()
            gx, gy = self.route[gate_vertex]
            gate_distance = math.hypot(gx - x, gy - y)

        self._project_to_route(x, y, gate_vertex)
        active_vertex = min(self.path_segment + 1, len(self.route_names) - 1)
        self._publish_active_waypoint(self.route_names[active_vertex])
        gate_s = self.route_s[gate_vertex]
        target_s = min(self.path_progress + self.ctrl.ld, gate_s)
        target = self._point_at(target_s)
        heading_error = norm_angle(math.atan2(target[1] - y, target[0] - x) - yaw)
        v, w, curvature = self.ctrl.compute(x, y, yaw, target)
        steering_mode = "putar-di-tempat" if abs(heading_error) > self.ctrl.rot_th else "ikuti-lintasan"
        remaining = max(0.0, gate_s - self.path_progress)
        if name == "finish":
            v = min(v, math.sqrt(2.0 * self.ctrl.linear_accel * gate_distance))
        else:
            pass_speed = min(0.08, self.ctrl.v_max)
            v = min(v, math.sqrt(
                pass_speed * pass_speed + 2.0 * self.ctrl.linear_accel * remaining))
        if not self.safety.motion_is_safe(x, y, yaw, v, w):
            self.stop()
            if not self._safety_stopped:
                self.get_logger().error(
                    f"SAFETY STOP: gerakan ke '{name}' berisiko kontak dengan bola "
                    f"pada posisi ({x:.3f}, {y:.3f}); tidak tersedia sensor "
                    "untuk mendeteksi rintangan tak dikenal/bergerak")
                self._safety_stopped = True
            return
        if self._safety_stopped:
            self.get_logger().info("Clearance aman kembali; navigasi dilanjutkan")
            self._safety_stopped = False

        self._publish_vel(v, w, dt)
        if (now - self._last_status_time).nanoseconds >= 1_000_000_000:
            self.get_logger().info(
                f"NAV aktif | pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f} deg) "
                f"| gate berikut='{name}' jarak={gate_distance:.3f} m "
                f"| progress={self.path_progress:.2f}/{self.route_s[-1]:.2f} m "
                f"| target=({target[0]:.2f},{target[1]:.2f}) "
                f"| error_arah={math.degrees(heading_error):.1f} deg "
                f"| mode={steering_mode} "
                f"| kurvatur={curvature:.2f} 1/m "
                f"| cmd=({self._v_cmd:.3f} m/s,{self._w_cmd:.3f} rad/s)")
            self._last_status_time = now


def main():
    # Handler sinyal rclpy dimatikan agar context masih hidup saat Ctrl+C,
    # sehingga Twist nol sempat dipublish sebelum shutdown.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
            node.get_logger().info("Dihentikan: Twist nol dipublish")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
