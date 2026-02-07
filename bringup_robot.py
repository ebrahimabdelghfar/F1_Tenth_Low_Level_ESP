#!/usr/bin/env python3
"""
Bringup script for F1/10 low-level ESP32 controller.

Detects an ESP32 on a serial port and launches the micro-ROS agent
inside a Docker container. Monitors the agent process and restarts it
automatically if it crashes.
"""

import glob
import os
import signal
import subprocess
import sys
from typing import List, Optional

import rclpy
import rclpy.executors
from rclpy.node import Node

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Known ESP32 USB-UART bridge identifiers (Silicon Labs CP2102)
ESP32_VENDOR_ID = "10c4"
ESP32_MODEL_ID = "CP2102_USB_to_UART_Bridge_Controller"

DEFAULT_BAUD_RATE = 115200
DEFAULT_DOCKER_IMAGE = "microros/micro-ros-agent:jazzy"
DEVICE_SCAN_PATTERNS = ["/dev/ttyUSB*", "/dev/ttyACM*"]

AGENT_HEALTH_CHECK_PERIOD_SEC = 3.0   # How often to check if agent is alive
DEVICE_RETRY_PERIOD_SEC = 5.0         # How often to retry device detection


# ---------------------------------------------------------------------------
# Device detection
# ---------------------------------------------------------------------------
class ESP32Detector:
    """Detect ESP32 devices connected via USB-UART bridges."""

    def __init__(
        self,
        vendor_id: str = ESP32_VENDOR_ID,
        model_id: str = ESP32_MODEL_ID,
    ) -> None:
        self._vendor_id = vendor_id
        self._model_id = model_id
        self._pyudev = self._try_import_pyudev()

    # ------------------------------------------------------------------
    @staticmethod
    def _try_import_pyudev():
        try:
            import pyudev
            return pyudev
        except ImportError:
            return None

    # ------------------------------------------------------------------
    def scan(self) -> List[str]:
        """Return a list of /dev paths that match the ESP32 bridge."""
        if self._pyudev is None:
            # Fallback: return raw device nodes without filtering
            devices: List[str] = []
            for pattern in DEVICE_SCAN_PATTERNS:
                devices.extend(sorted(glob.glob(pattern)))
            return devices

        ctx = self._pyudev.Context()
        found: List[str] = []
        for pattern in DEVICE_SCAN_PATTERNS:
            for dev_path in sorted(glob.glob(pattern)):
                try:
                    dev = self._pyudev.Devices.from_device_file(ctx, dev_path)
                    if (
                        dev.get("ID_VENDOR_ID") == self._vendor_id
                        and dev.get("ID_MODEL") == self._model_id
                    ):
                        found.append(dev_path)
                except self._pyudev.DeviceNotFoundError:
                    pass
        return found


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
class MicroRosAgentLauncher(Node):
    """
    ROS 2 node that manages a micro-ROS agent Docker container.

    Features
    --------
    * Auto-detects ESP32 device on startup (with periodic retry if absent).
    * Launches the Docker-based micro-ROS agent.
    * Monitors the agent process and restarts it if it crashes.
    * Cleans up **only its own** Docker container on shutdown.
    """

    def __init__(self) -> None:
        super().__init__("micro_ros_agent_launcher")

        # ---- Parameters ----
        self.declare_parameter("baud_rate", DEFAULT_BAUD_RATE)
        self.declare_parameter("docker_image", DEFAULT_DOCKER_IMAGE)
        self.declare_parameter("device", "")  # empty = auto-detect

        self._baud_rate: int = (
            self.get_parameter("baud_rate").get_parameter_value().integer_value
        )
        self._docker_image: str = (
            self.get_parameter("docker_image").get_parameter_value().string_value
        )
        self._explicit_device: str = (
            self.get_parameter("device").get_parameter_value().string_value
        )

        # ---- State ----
        self._detector = ESP32Detector()
        self._agent_process: Optional[subprocess.Popen] = None
        self._container_name: str = "micro_ros_agent_esp32"
        self._device_path: Optional[str] = None
        self._retry_timer = None
        self._health_timer = None

        # ---- Resolve device & start ----
        if self._explicit_device:
            if os.path.exists(self._explicit_device):
                self._device_path = self._explicit_device
                self.get_logger().info(
                    f"Using explicitly specified device: {self._device_path}"
                )
                self._start_agent()
            else:
                self.get_logger().error(
                    f"Specified device '{self._explicit_device}' does not exist"
                )
        else:
            self._try_detect_and_start()

    # ------------------------------------------------------------------
    # Device detection
    # ------------------------------------------------------------------
    def _try_detect_and_start(self) -> None:
        """Attempt to find an ESP32; schedule a retry timer if not found."""
        devices = self._detector.scan()
        if devices:
            self._device_path = devices[0]
            if len(devices) > 1:
                self.get_logger().warn(
                    f"Multiple ESP32 devices found: {devices}. "
                    f"Using {self._device_path}. "
                    "Set the 'device' parameter to override."
                )
            else:
                self.get_logger().info(f"ESP32 detected at {self._device_path}")
            self._start_agent()
        else:
            self.get_logger().warn(
                "No ESP32 devices found. "
                f"Retrying every {DEVICE_RETRY_PERIOD_SEC}s…"
            )
            if self._retry_timer is None:
                self._retry_timer = self.create_timer(
                    DEVICE_RETRY_PERIOD_SEC, self._device_retry_callback
                )

    def _device_retry_callback(self) -> None:
        devices = self._detector.scan()
        if devices:
            self._device_path = devices[0]
            self.get_logger().info(
                f"ESP32 detected at {self._device_path} (after retry)"
            )
            # Cancel retry timer
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            self._start_agent()

    # ------------------------------------------------------------------
    # Agent lifecycle
    # ------------------------------------------------------------------
    def _build_docker_command(self) -> List[str]:
        return [
            "sudo", "docker", "run", "--rm",
            f"--name={self._container_name}",
            "--net=host",
            f"--device={self._device_path}",
            self._docker_image,
            "serial",
            "--dev", self._device_path,
            "-b", str(self._baud_rate),
        ]

    def _start_agent(self) -> None:
        """Launch the micro-ROS agent container."""
        # Stop any lingering container with the same name
        self._stop_container(quiet=True)

        cmd = self._build_docker_command()
        self.get_logger().info(f"Starting micro-ROS agent: {' '.join(cmd)}")
        try:
            self._agent_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            self.get_logger().error(f"Failed to launch agent: {exc}")
            return

        # Start health-check timer
        if self._health_timer is None:
            self._health_timer = self.create_timer(
                AGENT_HEALTH_CHECK_PERIOD_SEC, self._health_check_callback
            )

    def _health_check_callback(self) -> None:
        """Restart the agent if the Docker process has exited unexpectedly."""
        if self._agent_process is None:
            return

        retcode = self._agent_process.poll()
        if retcode is not None:
            stderr_tail = ""
            if self._agent_process.stderr:
                try:
                    stderr_tail = self._agent_process.stderr.read().decode(
                        errors="replace"
                    )[-500:]
                except Exception:
                    pass
            self.get_logger().warn(
                f"micro-ROS agent exited (code {retcode}). Restarting…"
                + (f"\n  Last stderr: {stderr_tail}" if stderr_tail else "")
            )
            self._start_agent()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def _stop_container(self, quiet: bool = False) -> None:
        """Stop and remove **only** our named Docker container."""
        try:
            subprocess.run(
                ["sudo", "docker", "rm", "-f", self._container_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=10,
            )
            if not quiet:
                self.get_logger().info(
                    f"Removed Docker container '{self._container_name}'"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Timed out removing Docker container")
        except OSError as exc:
            if not quiet:
                self.get_logger().warn(f"Could not remove container: {exc}")

    def shutdown(self) -> None:
        """Gracefully shut down the agent and clean up resources."""
        self.get_logger().info("Shutting down micro-ROS agent launcher…")

        # Cancel timers
        for timer in (self._retry_timer, self._health_timer):
            if timer is not None:
                timer.cancel()

        # Kill the subprocess immediately — no long waits
        if self._agent_process is not None and self._agent_process.poll() is None:
            self._agent_process.kill()          # SIGKILL — instant
            self._agent_process.wait(timeout=2) # reap zombie

        # Remove the container in the background so we don't block exit
        try:
            subprocess.Popen(
                ["sudo", "docker", "rm", "-f", self._container_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)

    node: Optional[MicroRosAgentLauncher] = None
    _shutting_down = False

    def _signal_handler(sig, frame):
        nonlocal _shutting_down
        if _shutting_down:
            # Second Ctrl+C → force exit immediately
            sys.exit(1)
        _shutting_down = True
        rclpy.try_shutdown()   # unblocks rclpy.spin()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        node = MicroRosAgentLauncher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()