"""
Standalone mode — connect to Refringence without ROS 2 installed.

Useful for testing the full pipeline: connect, upload URDF, send mock
joint states (demo sinusoidal oscillation or interactive).
"""

import asyncio
import logging
import math
import sys
from typing import List, Optional

from refringence_bridge.connector import RefringenceConnector
from refringence_bridge.asset_sync import (
    discover_urdf_from_file,
    extract_mesh_filenames,
    upload_discovered_meshes,
)

logger = logging.getLogger(__name__)


def run_standalone(
    token: str,
    server_url: str,
    urdf_path: Optional[str] = None,
    mesh_dir: Optional[str] = None,
    rate_hz: int = 30,
    demo: bool = False,
) -> None:
    """Run the connector in standalone mode (no ROS 2)."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    async def _main():
        connector = RefringenceConnector(token=token, server_url=server_url)
        logger.info("Connecting to %s ...", server_url)
        await connector.connect()

        if not connector.connected:
            logger.error("Failed to connect")
            return

        logger.info("Connected!")

        # Upload URDF if provided
        urdf_xml: Optional[str] = None
        joint_names: List[str] = []

        if urdf_path:
            urdf_xml = discover_urdf_from_file(urdf_path)
            if urdf_xml:
                # Extract joint names from URDF for demo mode
                joint_names = _extract_joint_names(urdf_xml)
                mesh_filenames = extract_mesh_filenames(urdf_xml)

                await connector.upload_robot_description(urdf_xml, mesh_filenames)

                # Upload mesh files if mesh_dir provided
                if mesh_dir:
                    await upload_discovered_meshes(connector, urdf_xml, [mesh_dir])
            else:
                logger.warning("Could not read URDF from %s", urdf_path)

        if demo:
            # Demo mode: sinusoidal joint oscillation
            if not joint_names:
                joint_names = [f"joint_{i}" for i in range(1, 7)]
                logger.info("No URDF joints found, using demo joints: %s", joint_names)

            logger.info("Starting demo mode (%d Hz, %d joints)", rate_hz, len(joint_names))
            await _demo_loop(connector, joint_names, rate_hz)
        else:
            # Interactive mode
            logger.info("Standalone mode ready. Type joint commands or 'quit' to exit.")
            logger.info("Format: joint_name=value (radians), e.g.: joint_1=0.5 joint_2=-1.0")
            await _interactive_loop(connector)

        await connector.disconnect()

    asyncio.run(_main())


async def _demo_loop(
    connector: RefringenceConnector,
    joint_names: List[str],
    rate_hz: int,
) -> None:
    """Send sinusoidal joint oscillation + demo TF, logs, camera at the given rate."""
    interval = 1.0 / rate_hz
    t = 0.0
    tick = 0

    try:
        while connector.connected:
            positions = [
                math.sin(t * (0.5 + 0.2 * i)) * 1.0
                for i in range(len(joint_names))
            ]
            await connector.emit_joint_states(joint_names, positions)
            await connector.emit_sim_clock(t, 1.0)

            # ── Demo TF: world → base_link (static) + base_link → sensor_link (rotating)
            tf_transforms = [
                {
                    "header": {"stamp": {"sec": int(t), "nanosec": int((t % 1) * 1e9)}, "frame_id": "world"},
                    "child_frame_id": "base_link",
                    "transform": {
                        "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                },
                {
                    "header": {"stamp": {"sec": int(t), "nanosec": int((t % 1) * 1e9)}, "frame_id": "base_link"},
                    "child_frame_id": "sensor_link",
                    "transform": {
                        "translation": {"x": 0.3, "y": 0.0, "z": 0.2},
                        "rotation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(t * 0.5) * 0.3,
                            "w": math.cos(t * 0.5) * 0.3 + 0.7,
                        },
                    },
                },
            ]
            await connector.emit_tf(tf_transforms)

            # ── Demo log messages: INFO heartbeat every 5s, WARN every 10s
            if tick % (rate_hz * 5) == 0:
                await connector.emit_rosout({
                    "timestamp": t,
                    "level": 20,  # INFO
                    "name": "/demo_node",
                    "msg": f"Heartbeat at t={t:.1f}s",
                    "file": "standalone.py",
                    "line": 0,
                })
            if tick % (rate_hz * 10) == 0 and tick > 0:
                await connector.emit_rosout({
                    "timestamp": t,
                    "level": 30,  # WARN
                    "name": "/demo_node",
                    "msg": f"Demo warning at t={t:.1f}s",
                    "file": "standalone.py",
                    "line": 0,
                })

            # ── Demo camera: synthetic JPEG every 5th tick (~6fps)
            if tick % 5 == 0:
                try:
                    cam_data = _generate_demo_camera_frame(t)
                    if cam_data:
                        await connector.emit_camera_image(cam_data)
                except Exception:
                    pass  # PIL not installed — skip camera demo

            # ── Demo diagnostics: every 10s
            if tick % (rate_hz * 10) == 0:
                await connector.emit_diagnostics({
                    "statuses": [
                        {
                            "level": 0,  # OK
                            "name": "Motor Controller",
                            "message": "All motors operational",
                            "hardware_id": "motor_controller",
                            "values": [
                                {"key": "voltage", "value": f"{12.0 + math.sin(t) * 0.3:.1f}V"},
                                {"key": "temperature", "value": f"{45.0 + math.sin(t * 0.2) * 5:.1f}C"},
                            ],
                        },
                        {
                            "level": 1 if int(t) % 20 < 10 else 0,  # alternating WARN/OK
                            "name": "Battery",
                            "message": "Low battery" if int(t) % 20 < 10 else "Battery OK",
                            "hardware_id": "power_system",
                            "values": [
                                {"key": "percentage", "value": f"{max(20, 80 - t * 0.1):.0f}%"},
                                {"key": "voltage", "value": f"{11.5 + math.cos(t * 0.05) * 0.5:.2f}V"},
                            ],
                        },
                    ],
                })

            # ── Demo GPS: circular path around San Francisco (~2Hz)
            if tick % max(1, rate_hz // 2) == 0:
                lat = 37.7749 + 0.001 * math.sin(t * 0.1)
                lon = -122.4194 + 0.001 * math.cos(t * 0.1)
                alt = 10.0 + math.sin(t * 0.3) * 2.0
                await connector.emit_navsatfix({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": alt,
                    "status": 0,  # fix
                    "timestamp": t,
                })

            tick += 1
            t += interval
            await asyncio.sleep(interval)
    except KeyboardInterrupt:
        logger.info("Demo stopped")


def _generate_demo_camera_frame(t: float) -> Optional[dict]:
    """Generate a synthetic 320x240 JPEG camera frame with moving circle."""
    import base64
    import io

    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return None

    w, h = 320, 240
    img = Image.new("RGB", (w, h), (30, 30, 50))
    draw = ImageDraw.Draw(img)

    # Moving circle
    cx = int(w / 2 + math.sin(t * 0.8) * (w / 3))
    cy = int(h / 2 + math.cos(t * 0.6) * (h / 3))
    r = 20
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(0, 200, 100))

    # Timestamp text
    try:
        draw.text((10, 10), f"t={t:.2f}s", fill=(255, 255, 255))
    except Exception:
        pass

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=50)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    return {
        "encoding": "jpeg",
        "width": w,
        "height": h,
        "data": b64,
        "frame_id": "camera_link",
        "timestamp": t,
    }


async def _interactive_loop(connector: RefringenceConnector) -> None:
    """Read joint commands from stdin."""
    loop = asyncio.get_running_loop()

    try:
        while connector.connected:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            line = line.strip()

            if not line or line.lower() == "quit":
                break

            # Parse "joint_1=0.5 joint_2=-1.0"
            names = []
            positions = []
            for pair in line.split():
                if "=" in pair:
                    name, val = pair.split("=", 1)
                    try:
                        names.append(name)
                        positions.append(float(val))
                    except ValueError:
                        logger.warning("Invalid value: %s", pair)

            if names:
                await connector.emit_joint_states(names, positions)
                logger.info("Sent: %s", dict(zip(names, positions)))

    except KeyboardInterrupt:
        pass


def _extract_joint_names(urdf_xml: str) -> List[str]:
    """Quick regex extraction of movable joint names from URDF."""
    import re

    joints = []
    # Find all <joint ...> tags and extract name + type attributes (any order)
    joint_tag_re = re.compile(r"<joint\s+([^>]+)>")
    name_re = re.compile(r'name=["\']([^"\']+)["\']')
    type_re = re.compile(r'type=["\']([^"\']+)["\']')
    for tag_match in joint_tag_re.finditer(urdf_xml):
        attrs = tag_match.group(1)
        name_match = name_re.search(attrs)
        type_match = type_re.search(attrs)
        if name_match and type_match:
            name = name_match.group(1)
            jtype = type_match.group(1)
            if jtype != "fixed":
                joints.append(name)
    return joints
