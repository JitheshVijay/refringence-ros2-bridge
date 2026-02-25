"""Unit tests for topic_bridge.py message serialization."""

import unittest
from types import SimpleNamespace

from refringence_bridge.topic_bridge import (
    joint_state_to_dict,
    tf_to_dict,
    imu_to_dict,
    wrench_to_dict,
    laser_scan_to_dict,
    sensor_to_dict,
    clock_to_sim_time,
    dict_to_joint_trajectory_point,
    dict_to_joint_positions,
    rosout_to_dict,
    image_to_dict,
    camera_info_to_dict,
)


def make_joint_state(names, positions, velocities=None, efforts=None):
    """Create a mock JointState message."""
    return SimpleNamespace(
        name=names,
        position=positions,
        velocity=velocities or [],
        effort=efforts or [],
    )


def make_tf_message(transforms):
    """Create a mock TFMessage."""
    tf_list = []
    for t in transforms:
        tf_list.append(SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=t["sec"], nanosec=t["nsec"]),
                frame_id=t["frame_id"],
            ),
            child_frame_id=t["child_frame_id"],
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=t["tx"], y=t["ty"], z=t["tz"]),
                rotation=SimpleNamespace(x=t["rx"], y=t["ry"], z=t["rz"], w=t["rw"]),
            ),
        ))
    return SimpleNamespace(transforms=tf_list)


class TestJointStateToDict(unittest.TestCase):
    def test_basic(self):
        msg = make_joint_state(["j1", "j2"], [0.5, -1.0])
        result = joint_state_to_dict(msg)
        self.assertEqual(result["name"], ["j1", "j2"])
        self.assertEqual(result["position"], [0.5, -1.0])
        self.assertEqual(result["velocity"], [])

    def test_with_velocity_effort(self):
        msg = make_joint_state(
            ["j1"], [0.5], velocities=[0.1], efforts=[2.0]
        )
        result = joint_state_to_dict(msg)
        self.assertEqual(result["velocity"], [0.1])
        self.assertEqual(result["effort"], [2.0])


class TestTfToDict(unittest.TestCase):
    def test_single_transform(self):
        msg = make_tf_message([{
            "sec": 10, "nsec": 500000000,
            "frame_id": "world", "child_frame_id": "base_link",
            "tx": 1.0, "ty": 2.0, "tz": 3.0,
            "rx": 0.0, "ry": 0.0, "rz": 0.0, "rw": 1.0,
        }])
        result = tf_to_dict(msg)
        self.assertEqual(len(result["transforms"]), 1)
        t = result["transforms"][0]
        self.assertEqual(t["header"]["frame_id"], "world")
        self.assertEqual(t["child_frame_id"], "base_link")
        self.assertAlmostEqual(t["transform"]["translation"]["x"], 1.0)
        self.assertAlmostEqual(t["transform"]["rotation"]["w"], 1.0)


class TestImuToDict(unittest.TestCase):
    def test_basic(self):
        msg = SimpleNamespace(
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
            angular_velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            linear_acceleration=SimpleNamespace(x=0, y=0, z=9.81),
        )
        result = imu_to_dict(msg)
        self.assertEqual(result["type"], "imu")
        self.assertAlmostEqual(result["linear_acceleration"]["z"], 9.81)


class TestWrenchToDict(unittest.TestCase):
    def test_basic(self):
        msg = SimpleNamespace(
            wrench=SimpleNamespace(
                force=SimpleNamespace(x=1, y=2, z=3),
                torque=SimpleNamespace(x=0.1, y=0.2, z=0.3),
            )
        )
        result = wrench_to_dict(msg)
        self.assertEqual(result["type"], "wrench")
        self.assertEqual(result["force"]["x"], 1)


class TestLaserScanToDict(unittest.TestCase):
    def test_basic(self):
        msg = SimpleNamespace(
            angle_min=-1.57, angle_max=1.57,
            range_min=0.1, range_max=10.0,
            ranges=[1.0, 2.0, 3.0],
        )
        result = laser_scan_to_dict(msg)
        self.assertEqual(result["type"], "laser_scan")
        self.assertEqual(len(result["ranges"]), 3)

    def test_downsampling(self):
        msg = SimpleNamespace(
            angle_min=-3.14, angle_max=3.14,
            range_min=0.1, range_max=30.0,
            ranges=list(range(720)),
        )
        result = laser_scan_to_dict(msg)
        self.assertLessEqual(len(result["ranges"]), 360)


class TestSensorToDict(unittest.TestCase):
    def test_imu_dispatch(self):
        msg = SimpleNamespace(
            orientation=SimpleNamespace(x=0, y=0, z=0, w=1),
            angular_velocity=SimpleNamespace(x=0, y=0, z=0),
            linear_acceleration=SimpleNamespace(x=0, y=0, z=9.81),
        )
        result = sensor_to_dict(msg, "sensor_msgs/msg/Imu")
        self.assertEqual(result["type"], "imu")

    def test_fallback(self):
        msg = SimpleNamespace(data=42)
        result = sensor_to_dict(msg, "custom/msg/SomeData")
        self.assertEqual(result["data"], 42)


class TestClockToSimTime(unittest.TestCase):
    def test_conversion(self):
        msg = SimpleNamespace(
            clock=SimpleNamespace(sec=10, nanosec=500000000)
        )
        t = clock_to_sim_time(msg)
        self.assertAlmostEqual(t, 10.5)


class TestDictToJointTrajectoryPoint(unittest.TestCase):
    def test_multi_joint(self):
        result = dict_to_joint_trajectory_point({
            "name": ["j1", "j2"], "position": [0.5, -1.0]
        })
        self.assertEqual(result["names"], ["j1", "j2"])
        self.assertEqual(result["positions"], [0.5, -1.0])

    def test_single_joint(self):
        result = dict_to_joint_trajectory_point({
            "name": "j1", "position": 0.5
        })
        self.assertEqual(result["names"], ["j1"])
        self.assertEqual(result["positions"], [0.5])

    def test_invalid(self):
        result = dict_to_joint_trajectory_point({"foo": "bar"})
        self.assertIsNone(result)


class TestDictToJointPositions(unittest.TestCase):
    def test_single(self):
        result = dict_to_joint_positions({"name": "j1", "position": 0.5})
        self.assertEqual(result, {"j1": 0.5})

    def test_multi(self):
        result = dict_to_joint_positions({
            "name": ["j1", "j2"], "position": [0.5, -1.0]
        })
        self.assertEqual(result, {"j1": 0.5, "j2": -1.0})

    def test_missing(self):
        result = dict_to_joint_positions({"action": "reset"})
        self.assertIsNone(result)


class TestRosoutToDict(unittest.TestCase):
    def test_basic(self):
        msg = SimpleNamespace(
            stamp=SimpleNamespace(sec=10, nanosec=500000000),
            level=20,
            name="/my_node",
            msg="Hello world",
            file="test.py",
            line=42,
        )
        result = rosout_to_dict(msg)
        self.assertAlmostEqual(result["timestamp"], 10.5)
        self.assertEqual(result["level"], 20)
        self.assertEqual(result["name"], "/my_node")
        self.assertEqual(result["msg"], "Hello world")

    def test_missing_optional_fields(self):
        msg = SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0),
            level=40,
            name="/node",
            msg="error",
        )
        result = rosout_to_dict(msg)
        self.assertEqual(result["level"], 40)
        self.assertEqual(result["file"], "")
        self.assertEqual(result["line"], 0)

    def test_all_severity_levels(self):
        for level in [10, 20, 30, 40, 50]:
            msg = SimpleNamespace(
                stamp=SimpleNamespace(sec=0, nanosec=0),
                level=level,
                name="/n",
                msg="m",
            )
            result = rosout_to_dict(msg)
            self.assertEqual(result["level"], level)


class TestImageToDict(unittest.TestCase):
    def _make_image(self, width, height, encoding="rgb8", data=None):
        if data is None:
            channels = 1 if encoding in ("mono8", "8uc1") else 3
            data = bytes([128] * (width * height * channels))
        return SimpleNamespace(
            width=width,
            height=height,
            encoding=encoding,
            data=data,
            header=SimpleNamespace(
                frame_id="camera",
                stamp=SimpleNamespace(sec=1, nanosec=0),
            ),
        )

    def test_rgb8_basic(self):
        msg = self._make_image(4, 4, "rgb8")
        result = image_to_dict(msg)
        self.assertEqual(result["encoding"], "jpeg")
        self.assertEqual(result["width"], 4)
        self.assertEqual(result["height"], 4)
        self.assertGreater(len(result["data"]), 0)  # base64 JPEG
        self.assertEqual(result["frame_id"], "camera")

    def test_bgr8_conversion(self):
        # Create a 2x2 BGR image: pixel is (B=0, G=128, R=255)
        data = bytes([0, 128, 255] * 4)
        msg = self._make_image(2, 2, "bgr8", data)
        result = image_to_dict(msg)
        self.assertEqual(result["encoding"], "jpeg")
        self.assertEqual(result["width"], 2)

    def test_mono8(self):
        msg = self._make_image(4, 4, "mono8")
        result = image_to_dict(msg)
        self.assertEqual(result["encoding"], "jpeg")

    def test_downsampling(self):
        msg = self._make_image(1280, 720, "rgb8")
        result = image_to_dict(msg, max_width=320)
        self.assertEqual(result["encoding"], "jpeg")
        self.assertLessEqual(result["width"], 320)


class TestCameraInfoToDict(unittest.TestCase):
    def test_basic(self):
        msg = SimpleNamespace(
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.0, 0.0, 0.0, 0.0, 0.0],
            k=[525.0, 0.0, 320.0, 0.0, 525.0, 240.0, 0.0, 0.0, 1.0],
            header=SimpleNamespace(frame_id="camera_optical"),
        )
        result = camera_info_to_dict(msg)
        self.assertEqual(result["width"], 640)
        self.assertEqual(result["height"], 480)
        self.assertEqual(result["distortion_model"], "plumb_bob")
        self.assertEqual(len(result["K"]), 9)
        self.assertEqual(result["frame_id"], "camera_optical")


if __name__ == "__main__":
    unittest.main()
