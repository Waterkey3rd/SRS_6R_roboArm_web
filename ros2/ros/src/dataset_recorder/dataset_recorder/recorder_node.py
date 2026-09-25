#!/usr/bin/env python3
"""Service-controlled rosbag2 recorder: one start/stop cycle per epoch."""

from __future__ import annotations

import datetime as dt
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DatasetRecorder(Node):
    """Manage one rosbag2 process; each start/stop cycle is one epoch."""

    def __init__(self) -> None:
        super().__init__("dataset_recorder")

        self.declare_parameter("output_dir", "/home/rm/comm_ws/datasets")
        self.declare_parameter(
            "topics",
            [
                "/camera/camera/color/image_raw",
                "/joint_states",
                "/gripper_state",
            ],
        )
        self.declare_parameter("storage_id", "sqlite3")
        self.declare_parameter("max_cache_size", 100 * 1024 * 1024)

        output_dir = str(self.get_parameter("output_dir").value)
        topics = [str(topic) for topic in self.get_parameter("topics").value]
        storage_id = str(self.get_parameter("storage_id").value)
        max_cache_size = int(self.get_parameter("max_cache_size").value)

        if not topics:
            raise ValueError("topics must contain at least one topic")
        if max_cache_size < 0:
            raise ValueError("max_cache_size must be non-negative")

        self._output_dir = Path(output_dir).expanduser()
        self._topics = topics
        self._storage_id = storage_id
        self._max_cache_size = max_cache_size

        self._process: Optional[subprocess.Popen[bytes]] = None
        self._epoch_dir: Optional[Path] = None
        self._epoch_started_at = 0.0
        self._recording = False
        self._last_process_error = ""

        self._state_pub = self.create_publisher(String, "~/state", 10)
        self.create_service(Trigger, "~/start", self._start_service)
        self.create_service(Trigger, "~/stop", self._stop_service)
        self.create_service(Trigger, "~/status", self._status_service)
        self._timer = self.create_timer(1.0, self._timer_callback)

        self._publish_state("idle")
        self.get_logger().info(
            f"Dataset recorder ready: one epoch per start/stop, "
            f"output={self._output_dir} topics={', '.join(self._topics)}"
        )

    @property
    def recording(self) -> bool:
        return self._recording and self._process is not None

    def _start_service(self, _request: Trigger.Request, response: Trigger.Response):
        if self.recording:
            response.success = False
            response.message = "already recording: " + str(self._epoch_dir)
            return response

        try:
            self._output_dir.mkdir(parents=True, exist_ok=True)
            available_topics = {
                name for name, _types in self.get_topic_names_and_types()
            }
            missing_topics = [
                topic for topic in self._topics if topic not in available_topics
            ]
            if missing_topics:
                raise RuntimeError(
                    "configured topics are not currently published: "
                    + ", ".join(missing_topics)
                )
            epoch_stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            self._epoch_dir = self._output_dir / f"epoch_{epoch_stamp}"
            self._last_process_error = ""
            self._start_epoch()
        except Exception as exc:  # service must report failures instead of crashing the node
            self._recording = False
            self._process = None
            self._epoch_dir = None
            response.success = False
            response.message = f"failed to start recorder: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = f"epoch started: {self._epoch_dir}"
        return response

    def _stop_service(self, _request: Trigger.Request, response: Trigger.Response):
        if not self.recording:
            response.success = False
            response.message = "not recording"
            return response

        epoch_dir = self._epoch_dir
        self._stop_process()
        self._recording = False
        self._publish_state("stopped")
        response.success = True
        response.message = f"epoch stopped: {epoch_dir}"
        return response

    def _status_service(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = self.recording
        if self.recording:
            elapsed = time.monotonic() - self._epoch_started_at
            response.message = (
                f"recording epoch={self._epoch_dir} "
                f"elapsed_sec={elapsed:.1f}"
            )
        elif self._last_process_error:
            response.message = f"idle; last recorder error: {self._last_process_error}"
        else:
            response.message = "idle"
        return response

    def _start_epoch(self) -> None:
        if self._epoch_dir is None:
            raise RuntimeError("epoch directory is not set")
        if self._epoch_dir.exists():
            raise FileExistsError(f"epoch directory already exists: {self._epoch_dir}")

        command = [
            "ros2",
            "bag",
            "record",
            "--storage",
            self._storage_id,
            "--max-cache-size",
            str(self._max_cache_size),
            "--output",
            str(self._epoch_dir),
            *self._topics,
        ]
        self.get_logger().info(f"Starting epoch: {' '.join(command)}")
        self._process = subprocess.Popen(
            command,
            cwd=str(self._output_dir),
            start_new_session=True,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._epoch_started_at = time.monotonic()
        self._recording = True
        self._publish_state("recording")

    def _stop_process(self) -> None:
        process = self._process
        self._process = None
        if process is None:
            return

        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error("ros2 bag did not stop after SIGINT; terminating it")
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

        if process.returncode not in (0, -signal.SIGINT, 130):
            self._last_process_error = f"ros2 bag exited with code {process.returncode}"
            self.get_logger().error(self._last_process_error)

    def _timer_callback(self) -> None:
        if not self.recording:
            return

        process = self._process
        if process is not None and process.poll() is not None:
            self._recording = False
            self._last_process_error = f"ros2 bag exited with code {process.returncode}"
            self._process = None
            self._publish_state("error")
            self.get_logger().error(self._last_process_error)

    def _publish_state(self, state: str) -> None:
        message = String()
        message.data = state
        self._state_pub.publish(message)

    def destroy_node(self):
        if self._process is not None:
            self._recording = False
            self._stop_process()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DatasetRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
