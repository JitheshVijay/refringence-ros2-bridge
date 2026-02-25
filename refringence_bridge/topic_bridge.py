"""
ROS 2 message ↔ JSON serialization bridge.

Converts ROS 2 messages to plain Python dicts for transmission
via Socket.io, and vice versa for command messages.
"""

import math
from typing import Any, Dict, List, Optional


def _sanitize_floats(values: list) -> list:
    """Replace NaN/Inf with 0.0 to prevent JSON serialization errors.
    Handles non-float types (ints, numpy scalars) safely."""
    out = []
    for v in values:
        try:
            fv = float(v)
            out.append(0.0 if (math.isnan(fv) or math.isinf(fv)) else fv)
        except (TypeError, ValueError):
            out.append(0.0)
    return out


def _has_values(seq: Any) -> bool:
    """Check if a ROS message field has values (safe for numpy arrays)."""
    if seq is None:
        return False
    try:
        return len(seq) > 0
    except TypeError:
        return False


# ─── ROS → JSON ──────────────────────────────────────────────────────────────


def joint_state_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert sensor_msgs/msg/JointState to dict."""
    return {
        "name": list(msg.name),
        "position": _sanitize_floats(list(msg.position)),
        "velocity": _sanitize_floats(list(msg.velocity)) if _has_values(msg.velocity) else [],
        "effort": _sanitize_floats(list(msg.effort)) if _has_values(msg.effort) else [],
    }


def tf_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert tf2_msgs/msg/TFMessage to dict."""
    transforms = []
    for t in msg.transforms:
        transforms.append({
            "header": {
                "stamp": {
                    "sec": t.header.stamp.sec,
                    "nanosec": t.header.stamp.nanosec,
                },
                "frame_id": t.header.frame_id,
            },
            "child_frame_id": t.child_frame_id,
            "transform": {
                "translation": {
                    "x": t.transform.translation.x,
                    "y": t.transform.translation.y,
                    "z": t.transform.translation.z,
                },
                "rotation": {
                    "x": t.transform.rotation.x,
                    "y": t.transform.rotation.y,
                    "z": t.transform.rotation.z,
                    "w": t.transform.rotation.w,
                },
            },
        })
    return {"transforms": transforms}


def _sanitize_float(v: Any) -> float:
    """Replace a single NaN/Inf with 0.0. Handles non-float types safely."""
    try:
        fv = float(v)
        return 0.0 if (math.isnan(fv) or math.isinf(fv)) else fv
    except (TypeError, ValueError):
        return 0.0


def imu_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert sensor_msgs/msg/Imu to dict."""
    sf = _sanitize_float
    return {
        "type": "imu",
        "orientation": {
            "x": sf(msg.orientation.x),
            "y": sf(msg.orientation.y),
            "z": sf(msg.orientation.z),
            "w": sf(msg.orientation.w),
        },
        "angular_velocity": {
            "x": sf(msg.angular_velocity.x),
            "y": sf(msg.angular_velocity.y),
            "z": sf(msg.angular_velocity.z),
        },
        "linear_acceleration": {
            "x": sf(msg.linear_acceleration.x),
            "y": sf(msg.linear_acceleration.y),
            "z": sf(msg.linear_acceleration.z),
        },
    }


def wrench_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert geometry_msgs/msg/WrenchStamped to dict."""
    sf = _sanitize_float
    return {
        "type": "wrench",
        "force": {
            "x": sf(msg.wrench.force.x),
            "y": sf(msg.wrench.force.y),
            "z": sf(msg.wrench.force.z),
        },
        "torque": {
            "x": sf(msg.wrench.torque.x),
            "y": sf(msg.wrench.torque.y),
            "z": sf(msg.wrench.torque.z),
        },
    }


def laser_scan_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert sensor_msgs/msg/LaserScan to dict (downsampled)."""
    ranges = list(msg.ranges)
    n = len(ranges)
    # Downsample if too many points (keep max 360 points)
    # Recompute angle_min/angle_max to maintain correct correspondence
    result: Dict[str, Any] = {
        "type": "laser_scan",
        "range_min": msg.range_min,
        "range_max": msg.range_max,
    }
    if n > 360:
        step = -(-n // 360)  # ceiling division guarantees <= 360 elements
        ranges = ranges[::step]
        # Adjust angular bounds to match downsampled indices
        orig_increment = getattr(msg, "angle_increment", 0.0)
        result["angle_min"] = msg.angle_min
        result["angle_increment"] = orig_increment * step if orig_increment else 0.0
        result["angle_max"] = msg.angle_min + orig_increment * step * (len(ranges) - 1) if orig_increment else msg.angle_max
    else:
        result["angle_min"] = msg.angle_min
        result["angle_max"] = msg.angle_max
        result["angle_increment"] = getattr(msg, "angle_increment", 0.0)
    result["ranges"] = _sanitize_floats(ranges)
    return result


def sensor_to_dict(msg: Any, msg_type: str) -> Dict[str, Any]:
    """Generic sensor message to dict dispatcher."""
    type_lower = msg_type.lower()
    if "imu" in type_lower:
        return imu_to_dict(msg)
    if "wrench" in type_lower:
        return wrench_to_dict(msg)
    if "laserscan" in type_lower or "laser_scan" in type_lower:
        return laser_scan_to_dict(msg)
    # Fallback: attempt basic attribute extraction
    result: Dict[str, Any] = {"type": msg_type}
    if hasattr(msg, "data"):
        result["data"] = msg.data
    return result


def point_cloud_to_dict(msg: Any, max_points: int = 50000) -> Dict[str, Any]:
    """
    Convert sensor_msgs/msg/PointCloud2 to dict with packed binary buffers.

    Parses the field descriptors from msg.fields to locate X/Y/Z and optional
    RGB/intensity data. Stride-downsamples if there are more than max_points.

    Returns dict with:
      - point_count: int
      - positions: bytes (float32 x,y,z packed = 12 bytes/point)
      - colors: bytes | None (uint8 r,g,b packed = 3 bytes/point)
      - has_color: bool
      - frame_id: str
    """
    import struct

    # Parse field offsets from msg.fields
    field_map: Dict[str, Dict[str, Any]] = {}
    for field in msg.fields:
        field_map[field.name.lower()] = {
            "offset": field.offset,
            "datatype": field.datatype,
        }

    # PointField datatype constants (from sensor_msgs/msg/PointField)
    # FLOAT32 = 7, UINT8 = 2, UINT32 = 6
    x_off = field_map.get("x", {}).get("offset")
    y_off = field_map.get("y", {}).get("offset")
    z_off = field_map.get("z", {}).get("offset")

    if x_off is None or y_off is None or z_off is None:
        return {"point_count": 0, "positions": b"", "colors": None, "has_color": False, "frame_id": ""}

    # Detect color fields
    has_rgb = "rgb" in field_map or "rgba" in field_map
    has_r = "r" in field_map and "g" in field_map and "b" in field_map
    has_intensity = "intensity" in field_map

    rgb_off = None
    r_off = g_off = b_off = None
    intensity_off = None

    if has_rgb:
        rgb_key = "rgba" if "rgba" in field_map else "rgb"
        rgb_off = field_map[rgb_key]["offset"]
    elif has_r:
        r_off = field_map["r"]["offset"]
        g_off = field_map["g"]["offset"]
        b_off = field_map["b"]["offset"]
    elif has_intensity:
        intensity_off = field_map["intensity"]["offset"]

    has_color = has_rgb or has_r or has_intensity

    data = bytes(msg.data)
    point_step = msg.point_step
    total_points = msg.width * msg.height

    if total_points == 0:
        return {"point_count": 0, "positions": b"", "colors": None, "has_color": False, "frame_id": ""}

    # Stride-downsample if exceeding max
    stride = 1
    if total_points > max_points:
        stride = -(-total_points // max_points)  # ceiling division
    n_out = -(-total_points // stride)  # ceiling division

    # Pack positions as float32 xyz
    pos_buf = bytearray(n_out * 12)
    col_buf = bytearray(n_out * 3) if has_color else None

    out_idx = 0
    for i in range(0, total_points, stride):
        if out_idx >= n_out:
            break
        base = i * point_step
        if base + point_step > len(data):
            break

        try:
            x = struct.unpack_from("<f", data, base + x_off)[0]
            y = struct.unpack_from("<f", data, base + y_off)[0]
            z = struct.unpack_from("<f", data, base + z_off)[0]
        except struct.error:
            break

        # Skip NaN points
        if x != x or y != y or z != z:  # NaN check
            continue

        struct.pack_into("<fff", pos_buf, out_idx * 12, x, y, z)

        if col_buf is not None:
            if rgb_off is not None:
                # RGB packed as uint32 (0x00RRGGBB)
                try:
                    rgb_val = struct.unpack_from("<I", data, base + rgb_off)[0]
                    col_buf[out_idx * 3] = (rgb_val >> 16) & 0xFF
                    col_buf[out_idx * 3 + 1] = (rgb_val >> 8) & 0xFF
                    col_buf[out_idx * 3 + 2] = rgb_val & 0xFF
                except struct.error:
                    pass
            elif r_off is not None:
                try:
                    col_buf[out_idx * 3] = data[base + r_off]
                    col_buf[out_idx * 3 + 1] = data[base + g_off]
                    col_buf[out_idx * 3 + 2] = data[base + b_off]
                except IndexError:
                    pass
            elif intensity_off is not None:
                try:
                    intensity = struct.unpack_from("<f", data, base + intensity_off)[0]
                    # Map intensity to grayscale byte [0..255]
                    gray = max(0, min(255, int(intensity * 255)))
                    col_buf[out_idx * 3] = gray
                    col_buf[out_idx * 3 + 1] = gray
                    col_buf[out_idx * 3 + 2] = gray
                except struct.error:
                    pass

        out_idx += 1

    # Trim to actual output count (NaN points were skipped)
    pos_bytes = bytes(pos_buf[:out_idx * 12])
    col_bytes = bytes(col_buf[:out_idx * 3]) if col_buf is not None else None

    return {
        "point_count": out_idx,
        "positions": pos_bytes,
        "colors": col_bytes,
        "has_color": has_color,
        "frame_id": getattr(msg.header, "frame_id", "") if hasattr(msg, "header") else "",
    }


def clock_to_sim_time(msg: Any) -> float:
    """Convert rosgraph_msgs/msg/Clock to simulation time in seconds."""
    return msg.clock.sec + msg.clock.nanosec * 1e-9


def rosout_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert rcl_interfaces/msg/Log to dict."""
    return {
        "timestamp": msg.stamp.sec + msg.stamp.nanosec * 1e-9 if hasattr(msg, "stamp") else 0.0,
        "level": getattr(msg, "level", 20),  # 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL
        "name": getattr(msg, "name", ""),
        "msg": getattr(msg, "msg", ""),
        "file": getattr(msg, "file", ""),
        "line": getattr(msg, "line", 0),
    }


def image_to_dict(msg: Any, quality: int = 50, max_width: int = 640) -> Dict[str, Any]:
    """
    Convert sensor_msgs/msg/Image to dict with base64-encoded JPEG.

    Handles rgb8, bgr8, mono8, and 16UC1 encodings.
    Downsamples if width exceeds max_width.
    """
    import base64
    import io

    try:
        from PIL import Image
    except ImportError:
        return {"error": "Pillow not installed", "encoding": "error",
                "width": 0, "height": 0, "data": "",
                "frame_id": "", "timestamp": 0.0}

    encoding = msg.encoding.lower() if hasattr(msg, "encoding") else "rgb8"
    width = msg.width
    height = msg.height
    raw_data = bytes(msg.data)
    frame_id = getattr(msg.header, "frame_id", "") if hasattr(msg, "header") else ""
    timestamp = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) if (
        hasattr(msg, "header") and hasattr(msg.header, "stamp")
    ) else 0.0

    try:
        if encoding in ("mono8", "8uc1"):
            img = Image.frombytes("L", (width, height), raw_data)
        elif encoding in ("16uc1",):
            import struct as _struct
            n_pixels = width * height
            pixels_16 = _struct.unpack(f"<{n_pixels}H", raw_data[:n_pixels * 2])
            max_val = max(pixels_16) if pixels_16 else 1
            scale = 255.0 / max_val if max_val > 0 else 1.0
            pixels_8 = bytes(min(255, int(p * scale)) for p in pixels_16)
            img = Image.frombytes("L", (width, height), pixels_8)
        elif encoding == "bgr8":
            img = Image.frombytes("RGB", (width, height), raw_data)
            r, g, b = img.split()
            img = Image.merge("RGB", (b, g, r))
        else:
            # Default: treat as rgb8
            img = Image.frombytes("RGB", (width, height), raw_data)

        # Downsample if needed
        if width > max_width:
            ratio = max_width / width
            new_height = int(height * ratio)
            img = img.resize((max_width, new_height), Image.LANCZOS)

        # Compress to JPEG
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")

        return {
            "encoding": "jpeg",
            "width": img.width,
            "height": img.height,
            "data": b64,
            "frame_id": frame_id,
            "timestamp": timestamp,
        }
    except Exception as e:
        return {"error": str(e), "encoding": "error",
                "width": 0, "height": 0, "data": "",
                "frame_id": frame_id, "timestamp": timestamp}


def camera_info_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert sensor_msgs/msg/CameraInfo to dict."""
    return {
        "width": msg.width,
        "height": msg.height,
        "distortion_model": getattr(msg, "distortion_model", ""),
        "D": list(msg.d) if hasattr(msg, "d") else [],
        "K": list(msg.k) if hasattr(msg, "k") else [],
        "frame_id": getattr(msg.header, "frame_id", "") if hasattr(msg, "header") else "",
    }


def diagnostic_array_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert diagnostic_msgs/msg/DiagnosticArray to dict."""
    statuses = []
    for s in msg.status:
        statuses.append({
            "level": int(s.level),
            "name": getattr(s, "name", ""),
            "message": getattr(s, "message", ""),
            "hardware_id": getattr(s, "hardware_id", ""),
            "values": [
                {"key": v.key, "value": v.value}
                for v in (s.values or [])
            ],
        })
    return {"statuses": statuses}


def navsatfix_to_dict(msg: Any) -> Dict[str, Any]:
    """Convert sensor_msgs/msg/NavSatFix to dict."""
    sf = _sanitize_float
    return {
        "latitude": sf(msg.latitude),
        "longitude": sf(msg.longitude),
        "altitude": sf(msg.altitude),
        "status": int(msg.status.status) if hasattr(msg.status, "status") else -1,
        "timestamp": (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if hasattr(msg, "header") else 0.0,
    }


def dict_to_twist(data: Dict[str, Any]) -> Dict[str, float]:
    """Convert {linear: {x,y,z}, angular: {x,y,z}} to flat dict for Twist construction."""
    linear = data.get("linear", {})
    angular = data.get("angular", {})
    return {
        "linear_x": float(linear.get("x", 0)),
        "linear_y": float(linear.get("y", 0)),
        "linear_z": float(linear.get("z", 0)),
        "angular_x": float(angular.get("x", 0)),
        "angular_y": float(angular.get("y", 0)),
        "angular_z": float(angular.get("z", 0)),
    }


# ─── JSON → ROS ──────────────────────────────────────────────────────────────


def dict_to_joint_trajectory_point(
    data: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """
    Convert { name, position } dict to data suitable for publishing a
    trajectory_msgs/msg/JointTrajectoryPoint.

    Returns dict with fields ready for ROS message construction.
    """
    names = data.get("name")
    positions = data.get("position")

    if names is None or positions is None:
        return None

    # Multi-joint: { name: ["j1", "j2"], position: [0.5, -1.0] }
    try:
        if isinstance(names, list) and isinstance(positions, list):
            if len(names) != len(positions):
                return None
            return {
                "names": names,
                "positions": [float(p) for p in positions],
            }

        # Single joint: { name: "joint_1", position: 0.5 }
        if isinstance(names, str):
            return {"names": [names], "positions": [float(positions)]}
    except (ValueError, TypeError):
        return None

    return None


def dict_to_joint_positions(data: Dict[str, Any]) -> Optional[Dict[str, float]]:
    """
    Convert command:joint data into a simple { joint_name: position } dict.
    Handles both single-joint and multi-joint formats.
    """
    name = data.get("name")
    position = data.get("position")

    try:
        if isinstance(name, str) and position is not None:
            return {name: float(position)}

        if isinstance(name, list) and isinstance(position, list):
            if len(name) != len(position):
                return None
            return {n: float(p) for n, p in zip(name, position)}
    except (ValueError, TypeError):
        return None

    return None
