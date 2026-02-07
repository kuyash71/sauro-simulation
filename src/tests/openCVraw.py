#!/usr/bin/env python3

import argparse
import os
import signal
import subprocess
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_TOPIC = (
    "/world/iris_parkour/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image" 
)


class OpenCVTerminalTest(Node):
    def __init__(self, topic: str, show_window: bool, stats_period: float):
        super().__init__("opencv_terminal_test")
        self.bridge = CvBridge()
        self.show_window = show_window
        self.stats_period = stats_period

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_image,
            10,
        )
        self.topic = topic

        self.frame_count = 0
        self.frames_since_last = 0
        self.start_time = time.time()
        self.last_stats_time = self.start_time
        self.no_frame_warned = False
        self.health_timer = self.create_timer(5.0, self.on_health_check)

        self.get_logger().info(f"Subscribed image topic: {topic}")
        self.get_logger().info(
            "Press Ctrl+C to stop."
        )
        if self.show_window:
            self.get_logger().info("OpenCV preview windows enabled.")
        else:
            self.get_logger().info(
                "Preview windows disabled. Use --display to force-enable."
            )

    def on_health_check(self):
        if self.no_frame_warned:
            return
        if self.frame_count > 0:
            return

        self.get_logger().warning(
            "No image frames received yet. Check ROS topic/bridge:\n"
            "  ros2 topic list | grep -E 'image|camera'\n"
            "  ros2 topic hz "
            f"{self.topic}\n"
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

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        white_mask = cv2.inRange(frame_bgr, (170, 170, 170), (255, 255, 255))

        now = time.time()
        elapsed_total = max(now - self.start_time, 1e-6)
        elapsed_window = max(now - self.last_stats_time, 1e-6)

        if elapsed_window >= self.stats_period:
            h, w = frame_bgr.shape[:2]
            fps_total = self.frame_count / elapsed_total
            fps_window = self.frames_since_last / elapsed_window
            edge_ratio = float((edges > 0).sum()) / edges.size
            white_ratio = float((white_mask > 0).sum()) / white_mask.size
            mean_gray = float(gray.mean())

            self.get_logger().info(
                f"frames={self.frame_count} resolution={w}x{h} "
                f"fps(avg={fps_total:.2f}, instant={fps_window:.2f}) "
                f"mean_gray={mean_gray:.1f} edge_ratio={edge_ratio:.3f} "
                f"white_ratio={white_ratio:.3f}"
            )

            self.last_stats_time = now
            self.frames_since_last = 0

        if self.show_window:
            cv2.imshow("openCVraw - bgr", frame_bgr)
            cv2.imshow("openCVraw - edges", edges)
            cv2.imshow("openCVraw - white-mask", white_mask)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(
        description="ROS 2 + OpenCV basic terminal validation for Ubuntu 24.04 setup"
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
        help="Force-enable OpenCV windows for live preview",
    )
    display_group.add_argument(
        "--no-display",
        dest="display",
        action="store_false",
        help="Force-disable OpenCV windows (headless mode)",
    )
    parser.set_defaults(display=None)
    parser.add_argument(
        "--stats-period",
        type=float,
        default=1.0,
        help="Seconds between terminal stats logs",
    )
    return parser.parse_args()


def start_bridge(topic: str, ros_type: str, gz_type: str):
    bridge_spec = f"{topic}@{ros_type}@{gz_type}"
    cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge", bridge_spec]
    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print(
            "Failed to start bridge: 'ros2' command not found in PATH.",
            file=sys.stderr,
        )
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
    if proc is None:
        return
    if proc.poll() is not None:
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
        bridge_proc = start_bridge(
            args.topic, "sensor_msgs/msg/Image", "gz.msgs.Image"
        )
        if args.bridge_camera_info:
            camera_info_bridge_proc = start_bridge(
                camera_info_topic_from_image_topic(args.topic),
                "sensor_msgs/msg/CameraInfo",
                "gz.msgs.CameraInfo",
            )
        time.sleep(0.5)

    node = OpenCVTerminalTest(
        topic=args.topic,
        show_window=show_window,
        stats_period=args.stats_period,
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
