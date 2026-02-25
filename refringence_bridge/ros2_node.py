"""
ROS 2 bridge node — subscribes to ROS topics, forwards telemetry to
Refringence cloud relay, and publishes incoming commands to ROS topics.
"""

import asyncio
import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)

# ROS 2 imports are deferred to allow the package to be imported without rclpy
# (e.g., for standalone mode or testing).


def run_ros2_node(token: str, server_url: str, rate_hz: int = 30) -> None:
    """Entry point for the ROS 2 bridge node."""
    if rate_hz < 1 or rate_hz > 120:
        raise ValueError(f"rate_hz must be between 1 and 120, got {rate_hz}")
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    from refringence_bridge.connector import RefringenceConnector
    from refringence_bridge.topic_bridge import (
        joint_state_to_dict,
        tf_to_dict,
        clock_to_sim_time,
        point_cloud_to_dict,
        rosout_to_dict,
        image_to_dict,
        camera_info_to_dict,
    )
    from refringence_bridge.asset_sync import (
        discover_urdf_from_param,
        upload_discovered_meshes,
    )

    class RefringenceBridgeNode(Node):
        """ROS 2 node that bridges topics to Refringence cloud relay."""

        def __init__(self, connector: RefringenceConnector, rate: int):
            super().__init__("refringence_bridge")
            self._connector = connector
            self._rate_hz = rate
            self._loop: Optional[asyncio.AbstractEventLoop] = None

            # Latest data buffers (written by ROS callbacks, read by publish timer)
            # Protected by a lock since ROS callbacks and the publish timer may run
            # on different threads in MultiThreadedExecutor.
            self._data_lock = threading.Lock()
            self._latest_joint_state = None
            self._latest_tf = None
            self._latest_sim_time: Optional[float] = None
            self._latest_point_cloud = None
            self._point_cloud_tick = 0  # emit every 3rd tick (~10Hz at 30Hz timer)
            self._latest_rosout: list = []
            self._latest_camera_image = None
            self._latest_camera_info = None
            self._camera_tick = 0  # emit every 5th tick (~6fps at 30Hz timer)

            # Parameters
            self.declare_parameter("token", token)
            self.declare_parameter("server_url", server_url)
            self.declare_parameter("rate_hz", rate)

            # Subscribers
            self._joint_sub = self.create_subscription(
                JointState, "/joint_states", self._on_joint_states, 10
            )
            self._tf_sub = self.create_subscription(
                TFMessage, "/tf", self._on_tf, 10
            )

            # Try subscribing to /clock for simulation time
            try:
                from rosgraph_msgs.msg import Clock
                self._clock_sub = self.create_subscription(
                    Clock, "/clock", self._on_clock, 10
                )
            except ImportError:
                self._clock_sub = None
                self.get_logger().info("rosgraph_msgs not available, /clock not subscribed")

            # Try subscribing to PointCloud2
            try:
                from sensor_msgs.msg import PointCloud2
                self._pc_sub = self.create_subscription(
                    PointCloud2, "/camera/depth/points", self._on_point_cloud, 10
                )
            except ImportError:
                self._pc_sub = None
                self.get_logger().info("PointCloud2 not available, /camera/depth/points not subscribed")

            # Try subscribing to /rosout for log messages
            try:
                from rcl_interfaces.msg import Log
                self._rosout_sub = self.create_subscription(
                    Log, "/rosout", self._on_rosout, 10
                )
            except ImportError:
                self._rosout_sub = None
                self.get_logger().info("rcl_interfaces not available, /rosout not subscribed")

            # Try subscribing to camera image + camera info
            try:
                from sensor_msgs.msg import Image as ROSImage, CameraInfo as ROSCameraInfo
                self._cam_img_sub = self.create_subscription(
                    ROSImage, "/camera/color/image_raw", self._on_camera_image, 10
                )
                self._cam_info_sub = self.create_subscription(
                    ROSCameraInfo, "/camera/camera_info", self._on_camera_info, 10
                )
            except ImportError:
                self._cam_img_sub = None
                self._cam_info_sub = None
                self.get_logger().info("sensor_msgs/Image not available, camera topics not subscribed")

            # Timer for rate-limited telemetry emission
            self._publish_timer = self.create_timer(
                1.0 / self._rate_hz, self._publish_telemetry
            )

            # Register command handler
            self._connector.on_command("command:joint", self._handle_joint_command)
            self._connector.on_command("command:reset", self._handle_reset_command)

            self.get_logger().info(
                "Refringence bridge node initialized (rate=%d Hz)", self._rate_hz
            )

        def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
            self._loop = loop

        # ── ROS callbacks ─────────────────────────────────────────────────

        def _on_joint_states(self, msg: "JointState") -> None:
            with self._data_lock:
                self._latest_joint_state = msg

        def _on_tf(self, msg: "TFMessage") -> None:
            with self._data_lock:
                self._latest_tf = msg

        def _on_clock(self, msg: "Any") -> None:
            with self._data_lock:
                self._latest_sim_time = clock_to_sim_time(msg)

        def _on_point_cloud(self, msg: "Any") -> None:
            with self._data_lock:
                self._latest_point_cloud = msg

        def _on_rosout(self, msg: "Any") -> None:
            with self._data_lock:
                self._latest_rosout.append(msg)
                # Cap to avoid unbounded growth between publish ticks
                if len(self._latest_rosout) > 50:
                    self._latest_rosout = self._latest_rosout[-50:]

        def _on_camera_image(self, msg: "Any") -> None:
            with self._data_lock:
                self._latest_camera_image = msg

        def _on_camera_info(self, msg: "Any") -> None:
            with self._data_lock:
                self._latest_camera_info = msg

        # ── Rate-limited telemetry emission ───────────────────────────────

        def _publish_telemetry(self) -> None:
            """Timer callback: emit latest data to Refringence at configured rate."""
            if not self._connector.connected or self._loop is None:
                return

            with self._data_lock:
                js = self._latest_joint_state
                tf = self._latest_tf
                st = self._latest_sim_time
                # Point cloud emitted at reduced rate (every 3rd tick ≈ 10Hz at 30Hz)
                self._point_cloud_tick += 1
                pc = None
                if self._point_cloud_tick >= 3:
                    pc = self._latest_point_cloud
                    self._latest_point_cloud = None
                    self._point_cloud_tick = 0
                # Rosout: grab all accumulated log entries
                rosout_msgs = list(self._latest_rosout)
                self._latest_rosout.clear()

                # Camera: rate-limited (~6fps at 30Hz)
                self._camera_tick += 1
                cam_img = None
                cam_info = None
                if self._camera_tick >= 5:
                    cam_img = self._latest_camera_image
                    cam_info = self._latest_camera_info
                    self._latest_camera_image = None
                    self._latest_camera_info = None
                    self._camera_tick = 0

                # Clear after snapshot to avoid re-sending stale data
                self._latest_joint_state = None
                self._latest_tf = None
                self._latest_sim_time = None

            def _log_future_exception(fut, label):
                try:
                    exc = fut.exception()
                    if exc:
                        self.get_logger().error("Emit %s error: %s", label, exc)
                except Exception:
                    pass

            if js is not None:
                data = joint_state_to_dict(js)
                fut = asyncio.run_coroutine_threadsafe(
                    self._connector.emit_joint_states(
                        data["name"], data["position"],
                        data.get("velocity") or None,
                        data.get("effort") or None,
                    ),
                    self._loop,
                )
                fut.add_done_callback(lambda f: _log_future_exception(f, "joint_states"))

            if tf is not None:
                data = tf_to_dict(tf)
                fut = asyncio.run_coroutine_threadsafe(
                    self._connector.emit_tf(data["transforms"]),
                    self._loop,
                )
                fut.add_done_callback(lambda f: _log_future_exception(f, "tf"))

            if st is not None:
                fut = asyncio.run_coroutine_threadsafe(
                    self._connector.emit_sim_clock(st),
                    self._loop,
                )
                fut.add_done_callback(lambda f: _log_future_exception(f, "sim_clock"))

            if pc is not None:
                cloud_data = point_cloud_to_dict(pc)
                if cloud_data["point_count"] > 0:
                    fut = asyncio.run_coroutine_threadsafe(
                        self._connector.emit_point_cloud(cloud_data),
                        self._loop,
                    )
                    fut.add_done_callback(lambda f: _log_future_exception(f, "point_cloud"))

            for log_msg in rosout_msgs:
                log_data = rosout_to_dict(log_msg)
                fut = asyncio.run_coroutine_threadsafe(
                    self._connector.emit_rosout(log_data),
                    self._loop,
                )
                fut.add_done_callback(lambda f: _log_future_exception(f, "rosout"))

            if cam_img is not None:
                img_data = image_to_dict(cam_img)
                if not img_data.get("error"):
                    fut = asyncio.run_coroutine_threadsafe(
                        self._connector.emit_camera_image(img_data),
                        self._loop,
                    )
                    fut.add_done_callback(lambda f: _log_future_exception(f, "camera_image"))

            if cam_info is not None:
                info_data = camera_info_to_dict(cam_info)
                fut = asyncio.run_coroutine_threadsafe(
                    self._connector.emit_camera_info(info_data),
                    self._loop,
                )
                fut.add_done_callback(lambda f: _log_future_exception(f, "camera_info"))

        # ── Command handlers ──────────────────────────────────────────────

        def _handle_joint_command(self, data: dict) -> None:
            """Publish received joint command to ROS topic."""
            self.get_logger().debug("Joint command received: %s", data)
            # Could publish to /joint_trajectory_controller/command
            # For now, log it — actual publisher would depend on the robot's controller

        def _handle_reset_command(self, data: dict) -> None:
            """Handle simulation reset command."""
            self.get_logger().info("Reset command received")
            # Could call /reset_simulation service

    # ── Main loop ─────────────────────────────────────────────────────────

    import sys
    try:
        rclpy.init(args=sys.argv)
    except RuntimeError:
        logger.error("Failed to initialize rclpy. Is ROS 2 installed and sourced?")
        return

    connector = RefringenceConnector(token=token, server_url=server_url)
    node = RefringenceBridgeNode(connector, rate_hz)

    async def async_main():
        try:
            await connector.connect()

            # Upload robot description if available
            urdf = discover_urdf_from_param(node)
            if urdf:
                await connector.upload_robot_description(urdf)
                await upload_discovered_meshes(connector, urdf)
            else:
                node.get_logger().info("No /robot_description parameter found")

            # Keep connector alive until shutdown
            await connector.wait()
        except Exception as e:
            logger.error("Async main error: %s", e)
        finally:
            await connector.disconnect()

    loop = asyncio.new_event_loop()
    node.set_event_loop(loop)

    # Run asyncio event loop in a background thread
    async_thread = threading.Thread(target=loop.run_until_complete, args=(async_main(),), daemon=True)
    async_thread.start()

    # Run ROS 2 executor in the main thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Refringence bridge")
        # Signal connector disconnect and stop the event loop
        if loop.is_running():
            asyncio.run_coroutine_threadsafe(connector.disconnect(), loop)
            # Give disconnect a moment to complete before stopping the loop
            import time as _time
            _time.sleep(0.5)
            loop.call_soon_threadsafe(loop.stop)
        async_thread.join(timeout=5)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
