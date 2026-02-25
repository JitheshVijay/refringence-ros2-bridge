# Refringence ROS 2 Bridge

Connect ROS 2 simulators (Gazebo, Isaac Sim, etc.) to the Refringence Robotics Simulator for real-time 3D visualization, telemetry plots, and remote control.

## Quick Start

### 1. Install

```bash
pip install git+https://github.com/JitheshVijay/refringence-ros2-bridge.git
```

### 2. Get a Token

Open the Robotics Simulator at [refringence.com/tools/robotics](https://refringence.com/tools/robotics), create a session, and click **Connect** to generate a token.

### 3. Run

```bash
# With a ROS 2 simulator running (Gazebo, Isaac Sim, etc.)
refringence-bridge --token <YOUR_TOKEN>

# Standalone demo (no ROS 2 needed)
refringence-bridge --token <YOUR_TOKEN> --standalone --demo
```

That's it. Joint states, TF frames, camera images, and logs will stream to your browser in real time.

---

## Usage Modes

### ROS 2 Mode (default)

Requires a ROS 2 environment (Humble/Iron/Jazzy). The bridge automatically subscribes to:

| Topic | Type | Rate |
|-------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 30 Hz |
| `/tf` | `tf2_msgs/TFMessage` | 30 Hz |
| `/rosout` | `rcl_interfaces/Log` | 30 Hz |
| `/camera/color/image_raw` | `sensor_msgs/Image` | ~6 fps |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ~6 fps |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | ~10 Hz |
| `/clock` | `rosgraph_msgs/Clock` | 10 Hz |

```bash
# Basic
refringence-bridge --token <TOKEN>

# Custom rate
refringence-bridge --token <TOKEN> --rate 60

# With URDF upload (also discovers meshes)
refringence-bridge --token <TOKEN> --urdf /path/to/robot.urdf --mesh-dir /path/to/meshes

# Using ROS 2 launch
ros2 launch refringence_bridge bridge.launch.py token:=<TOKEN>
```

### Standalone Mode

No ROS 2 required. Useful for testing or custom Python integrations.

```bash
# Demo: synthetic sinusoidal joint motion
refringence-bridge --token <TOKEN> --standalone --demo

# With a URDF file
refringence-bridge --token <TOKEN> --standalone --urdf robot.urdf --mesh-dir ./meshes --demo

# Interactive: type joint commands in the terminal
refringence-bridge --token <TOKEN> --standalone
# Then type: joint_1=0.5 joint_2=-1.0
```

---

## Simulator Guides

### Gazebo (Classic & Ignition)

Gazebo publishes standard ROS 2 topics out of the box. Just run the bridge alongside your simulation:

```bash
# Terminal 1: launch your Gazebo sim
ros2 launch my_robot_pkg gazebo.launch.py

# Terminal 2: start the bridge
refringence-bridge --token <TOKEN>
```

### NVIDIA Isaac Sim

Enable the ROS 2 Bridge extension in Isaac Sim (it's built in), then:

```bash
# Isaac Sim publishes to standard ROS 2 topics when the bridge extension is active
refringence-bridge --token <TOKEN>
```

If Isaac Sim uses custom topic names, remap them:

```bash
ros2 run refringence_bridge refringence-bridge --token <TOKEN> \
  --ros-args -r /isaac_joint_states:=/joint_states
```

### Webots

Webots has a built-in ROS 2 interface. Launch your Webots world with ROS 2 enabled:

```bash
ros2 launch webots_ros2_driver robot_launch.py
refringence-bridge --token <TOKEN>
```

### Custom Python (No Simulator)

Use the connector directly in your Python code:

```python
import asyncio
from refringence_bridge.connector import RefringenceConnector

async def main():
    conn = RefringenceConnector(token="<TOKEN>", server_url="https://api.refringence.com")
    await conn.connect()

    # Send joint states
    await conn.emit_joint_states(
        names=["joint_1", "joint_2"],
        positions=[0.5, -1.0],
    )

    # Upload a URDF
    with open("robot.urdf") as f:
        await conn.upload_robot_description(f.read())

    await conn.disconnect()

asyncio.run(main())
```

---

## CLI Reference

```
refringence-bridge [OPTIONS]

Required:
  --token TOKEN         Refringence connector token

Optional:
  --url URL             Server URL (default: https://api.refringence.com)
  --rate HZ             Telemetry rate, 1-120 (default: 30)
  --standalone          Run without ROS 2
  --urdf PATH           URDF file to upload
  --mesh-dir DIR        Directory with STL/OBJ mesh files
  --demo                Standalone demo with synthetic motion
```

## Requirements

- Python 3.8+
- For ROS 2 mode: ROS 2 Humble/Iron/Jazzy with `rclpy`, `sensor_msgs`, `tf2_msgs`
- For standalone mode: just Python (no ROS 2 needed)
- Optional: `Pillow` (for camera image demo in standalone mode)
