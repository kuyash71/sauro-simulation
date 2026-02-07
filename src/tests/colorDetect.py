#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_TOPIC = (
    "/world/iris_parkour/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
)


class ColorDetectNode(Node):
    def __init__(
        self,
        topic: str,
        show_window: bool,
        stats_period: float,
        display_scale: float,
        min_area: int,
        white_ignore_top_ratio: float,
        color_ignore_top_ratio: float,
    ):
        super().__init__("opencv_color_detect")
        self.bridge = CvBridge()
        self.show_window = show_window
        self.stats_period = stats_period
        self.display_scale = display_scale
        self.min_area = min_area
        self.white_ignore_top_ratio = white_ignore_top_ratio
        self.color_ignore_top_ratio = color_ignore_top_ratio
        self.prev_color_masks = {}

        self.open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.white_open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))
        self.white_close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9))

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.topic = topic

        self.frame_count = 0
        self.frames_since_last = 0
        self.start_time = time.time()
        self.last_stats_time = self.start_time
        self.no_frame_warned = False
        self.health_timer = self.create_timer(5.0, self.on_health_check)

        self.get_logger().info(f"Subscribed image topic: {topic}")
        self.get_logger().info("Press Ctrl+C to stop.")
        if self.show_window:
            self.get_logger().info("Showing 3-panel detection window (red | blue | white).")
        else:
            self.get_logger().info("Preview disabled. Use --display to force OpenCV windows.")

    def on_health_check(self):
        if self.no_frame_warned or self.frame_count > 0:
            return

        self.get_logger().warning(
            "No image frames received yet. Check ROS topic/bridge:\n"
            "  ros2 topic list | grep -E 'image|camera'\n"
            f"  ros2 topic hz {self.topic}\n"
            "  ros2 run ros_gz_bridge parameter_bridge "
            f"{self.topic}@sensor_msgs/msg/Image@gz.msgs.Image"
        )
        self.no_frame_warned = True

    def on_image(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as err:
            self.get_logger().error(f"cv_bridge conversion failed: {err}")
            return

        self.frame_count += 1
        self.frames_since_last += 1

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        red_mask = self._build_red_mask(frame_bgr, hsv)
        blue_mask = self._build_blue_mask(frame_bgr, hsv)
        white_mask = self._build_white_mask(frame_bgr, hsv)

        height, width = frame_bgr.shape[:2]
        color_ignore_top = int(height * self.color_ignore_top_ratio)
        if color_ignore_top > 0:
            red_mask[:color_ignore_top, :] = 0
            blue_mask[:color_ignore_top, :] = 0
        ignore_top = int(height * self.white_ignore_top_ratio)
        if ignore_top > 0:
            white_mask[:ignore_top, :] = 0

        red_mask = self._clean_mask(red_mask, "red")
        blue_mask = self._clean_mask(blue_mask, "blue")
        white_mask = self._clean_mask(white_mask, "white")
        red_mask = self._stabilize_color_mask("red", red_mask, threshold=48)
        blue_mask = self._stabilize_color_mask("blue", blue_mask, threshold=80)

        red_mask, red_boxes = self._filter_targets(red_mask, "red", width, height)
        blue_mask, blue_boxes = self._filter_targets(blue_mask, "blue", width, height)
        white_mask, white_boxes = self._filter_targets(white_mask, "white", width, height)

        red_panel = self._build_panel(frame_bgr, red_mask, red_boxes, "RED", (0, 0, 255))
        blue_panel = self._build_panel(frame_bgr, blue_mask, blue_boxes, "BLUE", (255, 0, 0))
        white_panel = self._build_panel(
            frame_bgr,
            white_mask,
            white_boxes,
            "WHITE",
            (255, 255, 255),
        )

        now = time.time()
        elapsed_total = max(now - self.start_time, 1e-6)
        elapsed_window = max(now - self.last_stats_time, 1e-6)
        if elapsed_window >= self.stats_period:
            fps_total = self.frame_count / elapsed_total
            fps_window = self.frames_since_last / elapsed_window
            self.get_logger().info(
                f"frames={self.frame_count} resolution={width}x{height} "
                f"fps(avg={fps_total:.2f}, instant={fps_window:.2f}) "
                f"targets(red={len(red_boxes)}, blue={len(blue_boxes)}, white={len(white_boxes)})"
            )
            self.last_stats_time = now
            self.frames_since_last = 0

        if self.show_window:
            combined = np.hstack((red_panel, blue_panel, white_panel))
            if self.display_scale != 1.0:
                combined = cv2.resize(
                    combined,
                    None,
                    fx=self.display_scale,
                    fy=self.display_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
            cv2.imshow("colordetect - red | blue | white", combined)
            cv2.waitKey(1)

    def _build_red_mask(self, frame_bgr, hsv):
        low_1 = cv2.inRange(hsv, (0, 30, 20), (24, 255, 255))
        low_2 = cv2.inRange(hsv, (150, 30, 20), (180, 255, 255))
        red_hsv = cv2.bitwise_or(low_1, low_2)

        b, g, r = cv2.split(frame_bgr)
        red_rgb = ((r >= 120) & (r >= g + 30) & (r >= b + 30)).astype(np.uint8) * 255
        red_rgb_strict = ((r >= 145) & (g <= 95) & (b <= 95)).astype(np.uint8) * 255
        return cv2.bitwise_or(cv2.bitwise_or(red_hsv, red_rgb), red_rgb_strict)

    def _build_blue_mask(self, frame_bgr, hsv):
        blue_hsv = cv2.inRange(hsv, (92, 90, 35), (132, 255, 255))

        b, g, r = cv2.split(frame_bgr)
        blue_rgb = ((b >= 75) & (b >= g + 24) & (b >= r + 24)).astype(np.uint8) * 255
        blue_rgb_strict = ((b >= 115) & (b >= g + 35) & (b >= r + 35)).astype(np.uint8) * 255
        blue_core = cv2.bitwise_and(blue_hsv, blue_rgb)
        return cv2.bitwise_or(blue_core, blue_rgb_strict)

    def _build_white_mask(self, frame_bgr, hsv):
        white_hsv = cv2.inRange(hsv, (0, 0, 120), (180, 110, 255))

        b, g, r = cv2.split(frame_bgr)
        max_ch = np.maximum(np.maximum(r, g), b)
        min_ch = np.minimum(np.minimum(r, g), b)
        white_rgb = ((max_ch > 115) & ((max_ch - min_ch) < 55)).astype(np.uint8) * 255

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        bright_gray = (gray > 120).astype(np.uint8) * 255

        combined = cv2.bitwise_or(white_hsv, white_rgb)
        combined = cv2.bitwise_and(combined, bright_gray)
        return combined

    def _clean_mask(self, mask, color_name: str):
        if color_name == "white":
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.white_open_kernel, iterations=1)
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, self.white_close_kernel, iterations=1)
            return cleaned

        if color_name == "red":
            # Preserve tiny red targets: avoid opening erosion, only light close.
            return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.open_kernel, iterations=1)

        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, self.close_kernel, iterations=1)
        return cleaned

    def _stabilize_color_mask(self, color_name: str, mask, threshold: int):
        prev = self.prev_color_masks.get(color_name)
        if prev is None:
            self.prev_color_masks[color_name] = mask
            return mask

        smooth = cv2.addWeighted(mask, 0.72, prev, 0.28, 0.0)
        _, smooth = cv2.threshold(smooth, threshold, 255, cv2.THRESH_BINARY)
        self.prev_color_masks[color_name] = smooth
        return smooth

    def _filter_targets(self, mask, color_name: str, frame_width: int, frame_height: int):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(mask)
        boxes = []
        frame_area = frame_width * frame_height

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            effective_area = max(area, float(w * h))
            min_area = self.min_area
            if color_name == "red":
                min_area = max(1, self.min_area // 4)
            elif color_name == "blue":
                min_area = max(4, self.min_area - 3)

            if effective_area < min_area:
                continue
            if effective_area > frame_area * 0.2:
                continue

            if color_name == "red":
                if w < 1 or h < 1:
                    continue
            elif w < 2 or h < 2:
                continue

            aspect = h / float(w)
            if color_name == "white":
                if h < 6:
                    continue
                if aspect < 1.2:
                    continue
                if w > max(5, int(frame_width * 0.08)):
                    continue
            else:
                if w > int(frame_width * 0.2) or h > int(frame_height * 0.2):
                    continue

            cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)
            boxes.append((x, y, w, h, effective_area))

        boxes.sort(key=lambda item: item[4], reverse=True)
        boxes = boxes[:20]
        return filtered_mask, boxes

    def _build_panel(self, frame_bgr, mask, boxes, label: str, color):
        panel = cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask)

        for x, y, w, h, _ in boxes:
            cv2.rectangle(panel, (x, y), (x + w, y + h), color, 2)

        cv2.rectangle(panel, (0, 0), (panel.shape[1], 28), (25, 25, 25), thickness=-1)
        cv2.putText(
            panel,
            f"{label}: {len(boxes)}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

        if not boxes:
            cv2.putText(
                panel,
                "No target",
                (10, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (200, 200, 200),
                1,
                cv2.LINE_AA,
            )
        return panel

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description="ROS 2 + OpenCV color detection with 3 separate panels"
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help="ROS 2 image topic name to subscribe",
    )
    parser.add_argument(
        "--bridge",
        action="store_true",
        help="Auto-start ros_gz_bridge parameter_bridge for --topic",
    )
    parser.add_argument(
        "--bridge-camera-info",
        action="store_true",
        help="Also bridge <topic>/camera_info (best-effort)",
    )
    display_group = parser.add_mutually_exclusive_group()
    display_group.add_argument(
        "--display",
        dest="display",
        action="store_true",
        help="Force-enable OpenCV windows",
    )
    display_group.add_argument(
        "--no-display",
        dest="display",
        action="store_false",
        help="Force-disable OpenCV windows",
    )
    parser.set_defaults(display=None)
    parser.add_argument(
        "--stats-period",
        type=float,
        default=1.0,
        help="Seconds between terminal stats logs",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        default=1.0,
        help="Scale factor for OpenCV panel window",
    )
    parser.add_argument(
        "--min-area",
        type=int,
        default=10,
        help="Minimum contour area in pixels",
    )
    parser.add_argument(
        "--white-ignore-top-ratio",
        type=float,
        default=0.30,
        help="Top image ratio ignored for white detection (0.0 - 1.0)",
    )
    parser.add_argument(
        "--color-ignore-top-ratio",
        type=float,
        default=0.40,
        help="Top image ratio ignored for red/blue detection (0.0 - 1.0)",
    )
    return parser.parse_args()


def start_bridge(topic: str, ros_type: str, gz_type: str):
    bridge_spec = f"{topic}@{ros_type}@{gz_type}"
    cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", bridge_spec]
    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("Failed to start bridge: 'ros2' command not found in PATH.", file=sys.stderr)
        return None
    except OSError as err:
        print(f"Failed to start bridge process: {err}", file=sys.stderr)
        return None

    print(f"Started bridge PID={proc.pid}: {' '.join(cmd)}", file=sys.stderr)
    return proc


def camera_info_topic_from_image_topic(topic: str) -> str:
    if topic.endswith("/image"):
        return topic[: -len("/image")] + "/camera_info"
    return topic.rstrip("/") + "/camera_info"


def stop_bridge(proc: subprocess.Popen | None):
    if proc is None or proc.poll() is not None:
        return

    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def main():
    args = parse_args()
    if args.stats_period <= 0:
        print("--stats-period must be greater than 0", file=sys.stderr)
        return 2
    if args.display_scale <= 0:
        print("--display-scale must be greater than 0", file=sys.stderr)
        return 2
    if args.min_area <= 0:
        print("--min-area must be greater than 0", file=sys.stderr)
        return 2
    if not (0.0 <= args.white_ignore_top_ratio <= 1.0):
        print("--white-ignore-top-ratio must be in range [0.0, 1.0]", file=sys.stderr)
        return 2
    if not (0.0 <= args.color_ignore_top_ratio <= 1.0):
        print("--color-ignore-top-ratio must be in range [0.0, 1.0]", file=sys.stderr)
        return 2

    rclpy.init()
    has_gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    show_window = args.display if args.display is not None else has_gui
    bridge_proc = None
    camera_info_bridge_proc = None

    if not show_window and args.display is None:
        print(
            "GUI environment not detected, running in terminal-only mode. "
            "Use --display to force window mode.",
            file=sys.stderr,
        )

    if args.bridge:
        bridge_proc = start_bridge(args.topic, "sensor_msgs/msg/Image", "gz.msgs.Image")
        if args.bridge_camera_info:
            camera_info_bridge_proc = start_bridge(
                camera_info_topic_from_image_topic(args.topic),
                "sensor_msgs/msg/CameraInfo",
                "gz.msgs.CameraInfo",
            )
        time.sleep(0.5)

    node = ColorDetectNode(
        topic=args.topic,
        show_window=show_window,
        stats_period=args.stats_period,
        display_scale=args.display_scale,
        min_area=args.min_area,
        white_ignore_top_ratio=args.white_ignore_top_ratio,
        color_ignore_top_ratio=args.color_ignore_top_ratio,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_bridge(camera_info_bridge_proc)
        stop_bridge(bridge_proc)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
