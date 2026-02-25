"""
Core connector class for bridging simulators to Refringence cloud relay.

Pure Python, no ROS 2 dependency — uses python-socketio[asyncio] + aiohttp.
"""

import asyncio
import base64
import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import socketio

logger = logging.getLogger(__name__)

# Client-side rate throttle: skip emissions above this rate to stay under
# server-side 100/s limit with comfortable headroom.
MAX_EMISSIONS_PER_SECOND = 90


class RefringenceConnector:
    """Connects a local simulator to the Refringence cloud relay via Socket.io."""

    def __init__(
        self,
        token: str,
        server_url: str = "https://api.refringence.com",
    ):
        self._token = token
        self._server_url = server_url.rstrip("/")

        # Socket.io async client
        self._sio = socketio.AsyncClient(
            reconnection=True,
            reconnection_attempts=10,
            reconnection_delay=1,
            reconnection_delay_max=10,
            logger=False,
        )

        self._connected = False
        self._command_handlers: Dict[str, Callable] = {}

        # Rate throttle counters (protected by lock for thread safety)
        self._rate_lock = threading.Lock()
        self._emit_count = 0
        self._emit_window_start = 0.0

        # Wire up internal event handlers
        self._sio.on("connect", self._on_connect, namespace="/robotics")
        self._sio.on("disconnect", self._on_disconnect, namespace="/robotics")
        self._sio.on("connect_error", self._on_connect_error, namespace="/robotics")
        self._sio.on("error:rate_limit", self._on_rate_limit, namespace="/robotics")
        self._sio.on("mesh:upload_ok", self._on_mesh_upload_ok, namespace="/robotics")
        self._sio.on("error:mesh_upload", self._on_mesh_upload_error, namespace="/robotics")

        # Forward command events to registered callbacks
        for event in ("command:joint", "command:spawn", "command:reset", "command:custom"):
            self._sio.on(event, self._make_command_handler(event), namespace="/robotics")

    # ── Connection lifecycle ──────────────────────────────────────────────

    async def connect(self) -> None:
        """Connect to the Refringence /robotics namespace."""
        logger.info("Connecting to %s/robotics ...", self._server_url)
        await self._sio.connect(
            self._server_url,
            namespaces=["/robotics"],
            auth={"role": "connector", "token": self._token},
            transports=["websocket", "polling"],
            wait_timeout=15,
        )

    async def disconnect(self) -> None:
        """Gracefully disconnect. Server auto-sets status to 'offline'."""
        if self._sio.connected:
            await self._sio.disconnect()
        self._connected = False

    async def wait(self) -> None:
        """Block until the socket disconnects (useful for standalone scripts)."""
        await self._sio.wait()

    @property
    def connected(self) -> bool:
        return self._connected

    # ── Telemetry emission ────────────────────────────────────────────────

    async def emit_joint_states(
        self,
        names: List[str],
        positions: List[float],
        velocities: Optional[List[float]] = None,
        efforts: Optional[List[float]] = None,
    ) -> None:
        """Emit joint states (ROS JointState format)."""
        if not self._rate_check():
            return
        data: Dict[str, Any] = {"name": names, "position": positions}
        if velocities is not None:
            data["velocity"] = velocities
        if efforts is not None:
            data["effort"] = efforts
        await self._emit("telemetry:joint_states", data)

    async def emit_tf(self, transforms: List[Dict[str, Any]]) -> None:
        """Emit TF transforms."""
        if not self._rate_check():
            return
        await self._emit("telemetry:tf", {"transforms": transforms})

    async def emit_sensor(self, sensor_data: Dict[str, Any]) -> None:
        """Emit generic sensor data."""
        if not self._rate_check():
            return
        await self._emit("telemetry:sensor", sensor_data)

    async def emit_sim_clock(self, sim_time: float, real_time_factor: float = 1.0) -> None:
        """Emit simulation clock."""
        if not self._rate_check():
            return
        await self._emit("telemetry:sim_clock", {
            "simTime": sim_time,
            "realTimeFactor": real_time_factor,
        })

    async def emit_point_cloud(self, cloud_data: Dict[str, Any]) -> None:
        """Emit point cloud data (binary positions + optional colors)."""
        if not self._rate_check():
            return
        await self._emit("telemetry:point_cloud", cloud_data)

    async def emit_rosout(self, log_data: Dict[str, Any]) -> None:
        """Emit a ROS log entry (rosout)."""
        if not self._rate_check():
            return
        await self._emit("telemetry:rosout", log_data)

    async def emit_camera_image(self, image_data: Dict[str, Any]) -> None:
        """Emit compressed camera image (base64 JPEG)."""
        if not self._rate_check():
            return
        await self._emit("telemetry:camera_image", image_data)

    async def emit_camera_info(self, info_data: Dict[str, Any]) -> None:
        """Emit camera calibration info."""
        if not self._rate_check():
            return
        await self._emit("telemetry:camera_info", info_data)

    # ── Robot description ─────────────────────────────────────────────────

    async def upload_robot_description(
        self,
        urdf_xml: str,
        mesh_file_names: Optional[List[str]] = None,
    ) -> None:
        """Upload URDF robot description to the server."""
        await self._emit("robot:description", {
            "urdfXml": urdf_xml,
            "meshFileNames": mesh_file_names or [],
        })
        logger.info("Robot description uploaded (%d bytes URDF)", len(urdf_xml))

    async def upload_mesh_file(self, file_path: str, max_size: int = 100 * 1024 * 1024) -> None:
        """Upload a mesh file (STL/OBJ) to the server via base64 encoding."""
        path = Path(file_path)
        if not path.exists():
            logger.error("Mesh file not found: %s", file_path)
            return
        file_size = path.stat().st_size
        if file_size > max_size:
            logger.error("Mesh file too large: %s (%d bytes > %d limit)", file_path, file_size, max_size)
            return
        data = path.read_bytes()
        encoded = base64.b64encode(data).decode("ascii")
        await self._emit("mesh:upload", {
            "fileName": path.name,
            "data": encoded,
        })
        logger.info("Mesh file uploaded: %s (%d bytes)", path.name, len(data))

    # ── Command callbacks ─────────────────────────────────────────────────

    def on_command(self, event: str, callback: Callable) -> None:
        """
        Register a callback for a command event from the browser viewer.

        Supported events: 'command:joint', 'command:spawn', 'command:reset', 'command:custom'
        """
        self._command_handlers[event] = callback

    # ── Internal helpers ──────────────────────────────────────────────────

    async def _emit(self, event: str, data: Any) -> None:
        if self._sio.connected:
            await self._sio.emit(event, data, namespace="/robotics")

    def _rate_check(self) -> bool:
        """Client-side rate limiter. Returns True if emission is allowed."""
        with self._rate_lock:
            now = time.monotonic()
            if now - self._emit_window_start >= 1.0:
                self._emit_count = 0
                self._emit_window_start = now
            self._emit_count += 1
            if self._emit_count > MAX_EMISSIONS_PER_SECOND:
                return False
            return True

    def _make_command_handler(self, event: str):
        async def handler(data):
            cb = self._command_handlers.get(event)
            if cb:
                if asyncio.iscoroutinefunction(cb):
                    await cb(data)
                else:
                    cb(data)
            else:
                logger.debug("Unhandled command: %s", event)
        return handler

    # ── Socket.io event handlers ──────────────────────────────────────────

    async def _on_connect(self) -> None:
        self._connected = True
        logger.info("Connected to Refringence relay (sid=%s)", self._sio.sid)

    async def _on_disconnect(self) -> None:
        self._connected = False
        logger.info("Disconnected from Refringence relay")

    async def _on_connect_error(self, data: Any) -> None:
        self._connected = False
        logger.error("Connection error: %s", data)

    async def _on_rate_limit(self, data: Any) -> None:
        msg = data.get("message", data) if isinstance(data, dict) else data
        logger.warning("Rate limited: %s", msg)

    async def _on_mesh_upload_ok(self, data: Any) -> None:
        name = data.get("fileName", "") if isinstance(data, dict) else data
        logger.info("Mesh upload confirmed: %s", name)

    async def _on_mesh_upload_error(self, data: Any) -> None:
        msg = data.get("message", "") if isinstance(data, dict) else data
        logger.error("Mesh upload error: %s", msg)
