#!/usr/bin/env python3
"""
RoboDog Camera Detection Node

High-FPS camera capture for Raspberry Pi Camera Module v2.1.
Default backend uses continuous rpicam-vid MJPEG stream for better throughput.
"""

import subprocess
import threading
import time
import shutil
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraDetectionNode(Node):
    """Capture and publish camera frames to /robodog/camera/image_raw."""

    def __init__(self):
        super().__init__('camera_detection_node')

        # Parameters
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('frame_rate', 20)
        self.declare_parameter('capture_backend', 'rpicam_vid')  # rpicam_vid | rpicam_still
        self.declare_parameter('enable_detection', False)

        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        self.frame_rate = int(self.get_parameter('frame_rate').value)
        self.capture_backend = str(self.get_parameter('capture_backend').value)
        self.enable_detection = bool(self.get_parameter('enable_detection').value)

        self.image_pub = self.create_publisher(Image, '/robodog/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.capture_dir = Path('/tmp/robodog_camera')
        self.capture_dir.mkdir(exist_ok=True)

        self.stream_file = self.capture_dir / 'stream.mjpeg'
        self.rpicam_process = None
        self.capture_thread = None
        self.is_running = True

        self.frame_count = 0
        self.last_fps_log_time = time.time()
        self.last_fps_log_count = 0

        self.get_logger().info(
            'CameraDetectionNode started\n'
            f'  Backend: {self.capture_backend}\n'
            f'  Resolution: {self.width}x{self.height}\n'
            f'  Target FPS: {self.frame_rate}\n'
            f'  Detection: {self.enable_detection}'
        )

        if not self._init_capture():
            self.get_logger().error('Camera initialization failed; continuing without camera capture')
            self.capture_backend = 'disabled'
            self.capture_thread = None
        else:
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            self.get_logger().info('Camera capture thread started')

    def _init_capture(self) -> bool:
        """Initialize selected capture backend, with fallback when possible."""
        # Clean old frame files from previous runs
        for f in self.capture_dir.glob('frame_*.jpg'):
            try:
                f.unlink()
            except OSError:
                pass

        if self.capture_backend == 'rpicam_vid':
            for attempt in range(1, 6):
                if self._start_rpicam_vid():
                    return True
                self.get_logger().warn(f'rpicam_vid init failed (attempt {attempt}/5); retrying in 1s')
                time.sleep(1.0)

            self.get_logger().warn('rpicam_vid init failed after retries; camera capture disabled')
            if self._start_rpicam_still_probe():
                self.capture_backend = 'rpicam_still'
                return True
            return False

        if self.capture_backend == 'rpicam_still':
            return True

        self.get_logger().error(f'Unknown capture_backend: {self.capture_backend}')
        return False

    def _start_rpicam_vid(self) -> bool:
        """Start continuous MJPEG stream writer using rpicam-vid."""
        try:
            binary = shutil.which('rpicam-vid') or shutil.which('libcamera-vid') or '/usr/local/bin/rpicam-vid'
            if not binary:
                self.get_logger().error('rpicam-vid binary not found')
                return False

            if self.stream_file.exists():
                self.stream_file.unlink()

            cmd = [
                binary,
                '--nopreview',
                '--width', str(self.width),
                '--height', str(self.height),
                '--framerate', str(self.frame_rate),
                '--codec', 'mjpeg',
                '--timeout', '0',
                '--output', str(self.stream_file),
            ]

            self.get_logger().info(f'Starting rpicam-vid backend: {" ".join(cmd)}')
            self.rpicam_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )

            time.sleep(0.8)
            if self.rpicam_process.poll() is not None:
                err = self.rpicam_process.stderr.read() if self.rpicam_process.stderr else ''
                self.get_logger().error(f'rpicam-vid exited early: {err[-400:]}')
                return False

            timeout_s = 2.5
            t0 = time.time()
            while time.time() - t0 < timeout_s:
                if self.stream_file.exists() and self.stream_file.stat().st_size > 0:
                    self.get_logger().info(f'rpicam-vid stream ready: {self.stream_file}')
                    return True
                time.sleep(0.05)

            self.get_logger().error('rpicam-vid did not create stream output in time')
            return False

        except Exception as exc:
            self.get_logger().error(f'rpicam-vid init exception: {exc}')
            return False

    def _start_rpicam_still_probe(self) -> bool:
        """Probe whether rpicam-still is available as a fallback backend."""
        binary = shutil.which('rpicam-still') or shutil.which('libcamera-still') or '/usr/local/bin/rpicam-still'
        if not binary:
            self.get_logger().error('rpicam-still binary not found')
            return False
        self.rpicam_still_binary = binary
        return True

    def _publish_frame_bgr(self, frame_bgr):
        """Convert frame and publish ROS Image message."""
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        msg.header.frame_id = 'camera'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(msg)

        self.frame_count += 1

        now = time.time()
        if now - self.last_fps_log_time >= 2.0:
            delta_frames = self.frame_count - self.last_fps_log_count
            fps = delta_frames / (now - self.last_fps_log_time)
            self.get_logger().info(f'Published {self.frame_count} frames (actual ~{fps:.1f} FPS)')
            self.last_fps_log_time = now
            self.last_fps_log_count = self.frame_count

    def _capture_loop(self):
        if self.capture_backend == 'rpicam_vid':
            self._capture_loop_rpicam_vid()
        elif self.capture_backend == 'rpicam_still':
            self._capture_loop_rpicam_still()
        else:
            while self.is_running:
                time.sleep(1.0)

    def _capture_loop_rpicam_vid(self):
        """Read appended MJPEG bytes from file and decode frames continuously."""
        self.get_logger().info('Using high-FPS backend: rpicam_vid')
        read_offset = 0
        buf = bytearray()

        while self.is_running:
            try:
                if not self.stream_file.exists():
                    time.sleep(0.02)
                    continue

                with self.stream_file.open('rb') as f:
                    f.seek(read_offset)
                    chunk = f.read(1024 * 256)
                    read_offset = f.tell()

                if not chunk:
                    time.sleep(0.005)
                    continue

                buf.extend(chunk)

                # Parse all complete JPEG images currently in buffer
                while True:
                    soi = buf.find(b'\xff\xd8')
                    if soi < 0:
                        # keep tail only in case marker split
                        if len(buf) > 4:
                            del buf[:-4]
                        break

                    eoi = buf.find(b'\xff\xd9', soi + 2)
                    if eoi < 0:
                        # keep from SOI onward until full frame arrives
                        if soi > 0:
                            del buf[:soi]
                        break

                    jpeg = bytes(buf[soi:eoi + 2])
                    del buf[:eoi + 2]

                    frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    self._publish_frame_bgr(frame)

            except Exception as exc:
                self.get_logger().warn(f'rpicam_vid loop error: {exc}')
                time.sleep(0.02)

    def _capture_loop_rpicam_still(self):
        """Fallback backend: repeated still capture (lower FPS)."""
        self.get_logger().info('Using fallback backend: rpicam_still')
        frame_idx = 0
        interval = 1.0 / max(1, self.frame_rate)
        binary = getattr(self, 'rpicam_still_binary', None) or shutil.which('rpicam-still') or shutil.which('libcamera-still') or '/usr/local/bin/rpicam-still'

        while self.is_running:
            frame_path = self.capture_dir / f'frame_{frame_idx:06d}.jpg'
            cmd = [
                binary,
                '--nopreview',
                '--width', str(self.width),
                '--height', str(self.height),
                '--output', str(frame_path),
                '--timeout', '50',
            ]
            try:
                res = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True, timeout=2)
                if res.returncode == 0 and frame_path.exists():
                    frame = cv2.imread(str(frame_path))
                    if frame is not None:
                        self._publish_frame_bgr(frame)
                    try:
                        frame_path.unlink()
                    except OSError:
                        pass
                    frame_idx += 1
                else:
                    err = (res.stderr or '').strip().splitlines()
                    self.get_logger().warn(f'rpicam_still capture failed rc={res.returncode}: {err[-1] if err else "no stderr"}')
            except subprocess.TimeoutExpired:
                self.get_logger().warn('rpicam_still timeout')
            except Exception as exc:
                self.get_logger().warn(f'rpicam_still loop error: {exc}')

            time.sleep(interval)

    def destroy_node(self):
        self.get_logger().info('Shutting down camera detection node')
        self.is_running = False

        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)

        if self.rpicam_process:
            self.rpicam_process.terminate()
            try:
                self.rpicam_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.rpicam_process.kill()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if 'ExternalShutdownException' not in str(type(exc)):
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
