#!/usr/bin/env python3
"""Detect colored balls and capture boxes from the robot's Gazebo camera."""
import json
import math
import os
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent.parent
CAPTURE_ROOT = ROOT / "captured_images"


class BoxDetector:
    RANGES = {
        "green": ((35, 75, 55), (88, 255, 255)),
        "blue": ((95, 75, 55), (135, 255, 255)),
    }
    BALL_RANGES = {
        "green": (((35, 75, 55), (88, 255, 255)),),
        "red": (((0, 80, 55), (12, 255, 255)), ((168, 80, 55), (180, 255, 255))),
    }

    def __init__(self, min_area, min_ball_area=30.0):
        self.min_area = min_area
        self.min_ball_area = min_ball_area
        self.kernel = np.ones((5, 5), np.uint8)

    def color_mask(self, hsv, ranges):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(
                mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

    def detect(self, bgr, scale=1.0):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        found = {}
        for color, (lower, upper) in self.RANGES.items():
            mask = self.color_mask(hsv, ((lower, upper),))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                x, y, width, height = cv2.boundingRect(contour)
                if (area < self.min_area * scale * scale
                        or width < 20 * scale or height < 20 * scale):
                    continue
                image_height, image_width = bgr.shape[:2]
                border = max(1, round(scale))
                touched_edges = sum((
                    x <= border,
                    y <= border,
                    x + width >= image_width - border,
                    y + height >= image_height - border,
                ))
                if touched_edges > 1:
                    continue
                fill = area / float(width * height)
                aspect = width / float(height)
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0:
                    continue
                vertices = len(cv2.approxPolyDP(contour, 0.03 * perimeter, True))
                if fill < 0.45 or not 0.2 <= aspect <= 5.0 or not 4 <= vertices <= 6:
                    continue
                candidates.append((area, (x, y, width, height)))
            found[color] = sorted(candidates, reverse=True)
        return found

    def detect_balls(self, bgr, scale=1.0):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        found = {}
        image_height, image_width = bgr.shape[:2]
        for color, ranges in self.BALL_RANGES.items():
            mask = self.color_mask(hsv, ranges)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_ball_area * scale * scale:
                    continue
                x, y, width, height = cv2.boundingRect(contour)
                border = max(1, round(scale))
                if (x <= border or y <= border
                        or x + width >= image_width - border
                        or y + height >= image_height - border):
                    continue
                perimeter = cv2.arcLength(contour, True)
                if perimeter <= 0 or min(width, height) < 4:
                    continue
                circularity = 4.0 * math.pi * area / (perimeter * perimeter)
                aspect = width / float(height)
                vertices = len(cv2.approxPolyDP(contour, 0.03 * perimeter, True))
                if circularity < 0.62 or not 0.65 <= aspect <= 1.45 or vertices < 6:
                    continue
                candidates.append((area, (x, y, width, height)))
            found[color] = sorted(candidates, reverse=True)
        return found


class CameraCaptureNode(Node):
    PROCESS_SCALE = 0.5
    PROCESS_RATE = 15.0
    BOX_NAMES = {"green": "box_capture_green", "blue": "box_capture_blue"}
    CAPTURE_WAYPOINTS = {
        "blue": ("track_18", "track_19"),
        "green": ("track_20", "track_21"),
    }
    COLORS = {"green": (0, 255, 0), "blue": (255, 0, 0)}
    BALL_COLORS = {"green": (0, 255, 0), "red": (0, 0, 255)}

    def __init__(self):
        super().__init__("camera_box_capture")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("detection_image_topic", "/camera/detections/image_raw")
        self.declare_parameter("global_frame", "world")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("min_area", 1200.0)
        self.declare_parameter("min_ball_area", 30.0)
        self.declare_parameter("capture_dir", str(CAPTURE_ROOT))
        get = lambda name: self.get_parameter(name).value
        self.global_frame = str(get("global_frame"))
        self.robot_frame = str(get("robot_frame"))
        self.detector = BoxDetector(float(get("min_area")), float(get("min_ball_area")))
        self.bridge = CvBridge()
        self.capture_root = Path(str(get("capture_dir"))).expanduser()
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.processing_callback_group = MutuallyExclusiveCallbackGroup()

        self.output_dirs = {
            color: self.capture_root / f"{color}_box" for color in self.BOX_NAMES
        }
        try:
            for directory in self.output_dirs.values():
                directory.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().fatal(f"Gagal membuat folder hasil capture: {exc}")
            raise

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.detection_image_topic = str(get("detection_image_topic"))
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.detection_publisher = self.create_publisher(
            Image, self.detection_image_topic, image_qos)
        waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.active_waypoint = None
        self.waypoint_subscription = self.create_subscription(
            String,
            "/nav/active_waypoint",
            self.waypoint_callback,
            waypoint_qos,
        )
        self.subscription = self.create_subscription(
            Image,
            str(get("image_topic")),
            self.image_callback,
            image_qos,
            callback_group=self.image_callback_group,
        )
        self.timer = self.create_timer(5.0, self.report_status)
        self.processing_timer = self.create_timer(
            1.0 / self.PROCESS_RATE,
            self.process_latest_frame,
            callback_group=self.processing_callback_group,
        )
        self.latest_frame = None
        self.latest_frame_sequence = 0
        self.last_processed_sequence = 0
        self.last_image_time = None
        self.last_detection = {
            "boxes": {"green": 0, "blue": 0},
            "balls": {"green": 0, "red": 0},
        }
        self.captured_waypoints = {color: set() for color in self.BOX_NAMES}
        self.image_count = 0
        self.tf_warned = False
        self.get_logger().info(
            f"Kamera aktif: input={get('image_topic')}, "
            f"overlay={self.detection_image_topic}, "
            f"output={self.capture_root.resolve()}, "
            f"processing_rate={self.PROCESS_RATE:.0f} FPS, "
            "capture=waypoint+actual-detection, "
            "blue=track_18/track_19, green=track_20/track_21")

    def waypoint_callback(self, msg):
        if msg.data == self.active_waypoint:
            return
        self.active_waypoint = msg.data
        self.get_logger().info(f"Waypoint navigasi diterima: {self.active_waypoint}")

    def capture_count(self, color):
        return len(self.captured_waypoints[color])

    def capture_waypoint_active(self, color):
        return (
            self.active_waypoint in self.CAPTURE_WAYPOINTS[color]
            and self.active_waypoint not in self.captured_waypoints[color]
        )

    def current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            if not self.tf_warned:
                self.get_logger().warning(f"TF kamera capture belum tersedia: {exc}")
                self.tf_warned = True
            return None
        if self.tf_warned:
            self.get_logger().info("TF kamera capture tersedia kembali")
            self.tf_warned = False
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return float(t.x), float(t.y), yaw

    def image_callback(self, msg):
        self.last_image_time = self.get_clock().now()
        self.image_count += 1
        self.latest_frame_sequence += 1
        self.latest_frame = (msg, self.latest_frame_sequence)

    @staticmethod
    def scale_candidates(candidates, scale):
        inverse = 1.0 / scale
        return [
            (area * inverse * inverse,
             (round(x * inverse), round(y * inverse),
              round(width * inverse), round(height * inverse)))
            for area, (x, y, width, height) in candidates
        ]

    def process_latest_frame(self):
        frame = self.latest_frame
        if frame is None or frame[1] == self.last_processed_sequence:
            return
        msg, sequence = frame
        self.last_processed_sequence = sequence
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except (CvBridgeError, ValueError, TypeError, cv2.error) as exc:
            self.get_logger().error(f"Gagal mengonversi {msg.encoding} dari kamera: {exc}")
            return

        small = cv2.resize(
            bgr, None, fx=self.PROCESS_SCALE, fy=self.PROCESS_SCALE,
            interpolation=cv2.INTER_AREA)
        found = {
            color: self.scale_candidates(candidates, self.PROCESS_SCALE)
            for color, candidates in self.detector.detect(small, self.PROCESS_SCALE).items()
        }
        balls = {
            color: self.scale_candidates(candidates, self.PROCESS_SCALE)
            for color, candidates in self.detector.detect_balls(small, self.PROCESS_SCALE).items()
        }
        self.last_detection = {
            "boxes": {color: len(contours) for color, contours in found.items()},
            "balls": {color: len(contours) for color, contours in balls.items()},
        }
        annotated = bgr.copy()
        for color, candidates in found.items():
            for area, rect in candidates:
                self.draw_detection(annotated, color, "kotak", rect, area, self.COLORS[color])
        for color, candidates in balls.items():
            for area, rect in candidates:
                self.draw_detection(
                    annotated, color, "bola", rect, area, self.BALL_COLORS[color])
        status = (
            f"Bola hijau: {len(balls['green'])} | merah: {len(balls['red'])}   "
            f"Kotak hijau: {len(found['green'])} | biru: {len(found['blue'])}")
        cv2.putText(
            annotated, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(
            annotated, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (20, 20, 20), 1, cv2.LINE_AA)
        capture_status = (
            f"Waypoint {self.active_waypoint or 'menunggu'} | Foto hijau: "
            f"{self.capture_count('green')}/{len(self.CAPTURE_WAYPOINTS['green'])} | "
            f"biru: {self.capture_count('blue')}/{len(self.CAPTURE_WAYPOINTS['blue'])}")
        cv2.putText(
            annotated, capture_status, (10, 48), cv2.FONT_HERSHEY_SIMPLEX,
            0.48, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(
            annotated, capture_status, (10, 48), cv2.FONT_HERSHEY_SIMPLEX,
            0.48, (20, 20, 20), 1, cv2.LINE_AA)
        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            overlay_msg.header = msg.header
            self.detection_publisher.publish(overlay_msg)
        except (CvBridgeError, ValueError, TypeError, cv2.error) as exc:
            self.get_logger().error(f"Gagal menerbitkan overlay deteksi kamera: {exc}")

        pose = None
        for color, candidates in found.items():
            if not candidates or not self.capture_waypoint_active(color):
                continue
            if pose is None:
                pose = self.current_pose()
            area, rect = candidates[0]
            if self.save_capture(color, bgr, rect, area, pose, msg):
                self.captured_waypoints[color].add(self.active_waypoint)
                self.get_logger().info(
                    f"CAPTURE {color} {self.capture_count(color)}/"
                    f"{len(self.CAPTURE_WAYPOINTS[color])} pada waypoint "
                    f"{self.active_waypoint} "
                    f"({area:.0f} px); tersimpan di "
                    f"{self.output_dirs[color]}")

    @staticmethod
    def draw_detection(image, color, object_type, rect, area, draw_color):
        x, y, width, height = rect
        cv2.rectangle(image, (x, y), (x + width, y + height), draw_color, 2)
        label = f"{object_type} {color} ({area:.0f}px)"
        cv2.putText(
            image, label, (x, max(y - 7, 15)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 2, cv2.LINE_AA)

    def save_capture(self, color, bgr, rect, area, pose, msg):
        timestamp = datetime.now().astimezone()
        capture_number = self.capture_count(color) + 1
        stem = (
            f"{timestamp.strftime('%Y%m%d_%H%M%S_%f%z')}_"
            f"{color}_box_{capture_number:02d}")
        image_path = self.output_dirs[color] / f"{stem}.png"
        metadata_path = self.output_dirs[color] / f"{stem}.json"
        temporary_image = image_path.with_name(f".{stem}.tmp.png")
        temporary_metadata = metadata_path.with_name(f".{stem}.tmp")
        image_persisted = False
        x, y, width, height = rect
        annotated = bgr.copy()
        cv2.rectangle(annotated, (x, y), (x + width, y + height), self.COLORS[color], 2)
        try:
            if not cv2.imwrite(str(temporary_image), annotated):
                raise OSError(f"OpenCV menolak menulis {temporary_image}")
            sec = int(msg.header.stamp.sec)
            nanosec = int(msg.header.stamp.nanosec)
            record = {
                "object": self.BOX_NAMES[color],
                "color": color,
                "image": image_path.name,
                "captured_at_local": timestamp.isoformat(),
                "camera_topic": str(self.get_parameter("image_topic").value),
                "camera_stamp_sec": sec,
                "camera_stamp_nanosec": nanosec,
                "capture_waypoint": self.active_waypoint,
                "robot_frame": self.robot_frame,
                "global_frame": self.global_frame,
                "robot_pose": None if pose is None else {
                    "x": pose[0],
                    "y": pose[1],
                    "yaw_rad": pose[2],
                    "yaw_deg": math.degrees(pose[2]),
                },
                "detection": {
                    "bounding_box_xywh": [x, y, width, height],
                    "contour_area_px": float(area),
                },
            }
            with temporary_metadata.open("w", encoding="utf-8") as stream:
                json.dump(record, stream, indent=2)
                stream.write("\n")
            os.replace(temporary_image, image_path)
            image_persisted = True
            os.replace(temporary_metadata, metadata_path)
        except (OSError, cv2.error, TypeError, ValueError) as exc:
            for temporary in (temporary_image, temporary_metadata):
                try:
                    temporary.unlink(missing_ok=True)
                except OSError as cleanup_error:
                    self.get_logger().error(f"Gagal membersihkan file sementara: {cleanup_error}")
            if image_persisted:
                self.get_logger().error(
                    f"Gambar {image_path} tersimpan tetapi metadata gagal ditulis: {exc}")
                return True
            self.get_logger().error(f"Gagal menyimpan capture {color}: {exc}")
            return False
        return True

    def report_status(self):
        if self.last_image_time is None:
            self.get_logger().warning("Kamera belum mengirim frame pada /camera/image_raw")
            return
        age = (self.get_clock().now() - self.last_image_time).nanoseconds * 1e-9
        if self.active_waypoint is None:
            waypoint_status = "belum diterima"
        else:
            waypoint_status = self.active_waypoint
        self.get_logger().info(
            f"KAMERA aktif | frame={self.image_count}, umur={age:.2f}s, "
            f"deteksi={self.last_detection}, waypoint={waypoint_status}, "
            f"capture={{'green': {self.capture_count('green')}/"
            f"{len(self.CAPTURE_WAYPOINTS['green'])}, 'blue': "
            f"{self.capture_count('blue')}/{len(self.CAPTURE_WAYPOINTS['blue'])}}}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
