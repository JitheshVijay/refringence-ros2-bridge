"""Unit tests for PointCloud2 serialization in topic_bridge.py."""

import struct
import unittest
from types import SimpleNamespace

from refringence_bridge.topic_bridge import point_cloud_to_dict


def make_point_field(name, offset, datatype=7, count=1):
    """Create a mock PointField. datatype 7 = FLOAT32, 2 = UINT8, 6 = UINT32."""
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


def make_xyz_cloud(points, point_step=12):
    """Create a mock PointCloud2 with XYZ float32 fields."""
    fields = [
        make_point_field("x", 0),
        make_point_field("y", 4),
        make_point_field("z", 8),
    ]
    data = bytearray()
    for x, y, z in points:
        data.extend(struct.pack("<fff", x, y, z))
    return SimpleNamespace(
        fields=fields,
        point_step=point_step,
        width=len(points),
        height=1,
        data=bytes(data),
        header=SimpleNamespace(frame_id="camera_link"),
    )


def make_xyzrgb_cloud(points, point_step=16):
    """Create a mock PointCloud2 with XYZRGB fields (rgb as packed uint32)."""
    fields = [
        make_point_field("x", 0),
        make_point_field("y", 4),
        make_point_field("z", 8),
        make_point_field("rgb", 12, datatype=6),
    ]
    data = bytearray()
    for x, y, z, r, g, b in points:
        rgb_packed = (r << 16) | (g << 8) | b
        data.extend(struct.pack("<fffI", x, y, z, rgb_packed))
    return SimpleNamespace(
        fields=fields,
        point_step=point_step,
        width=len(points),
        height=1,
        data=bytes(data),
        header=SimpleNamespace(frame_id="camera_link"),
    )


class TestPointCloudToDict(unittest.TestCase):
    def test_basic_xyz(self):
        cloud = make_xyz_cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 2)
        self.assertFalse(result["has_color"])
        self.assertIsNone(result["colors"])
        self.assertEqual(result["frame_id"], "camera_link")
        # Verify positions
        positions = result["positions"]
        self.assertEqual(len(positions), 2 * 12)  # 2 points * 12 bytes
        x, y, z = struct.unpack_from("<fff", positions, 0)
        self.assertAlmostEqual(x, 1.0, places=5)
        self.assertAlmostEqual(y, 2.0, places=5)
        self.assertAlmostEqual(z, 3.0, places=5)

    def test_empty_cloud(self):
        cloud = make_xyz_cloud([])
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 0)

    def test_rgb_cloud(self):
        cloud = make_xyzrgb_cloud([
            (1.0, 2.0, 3.0, 255, 0, 0),
            (4.0, 5.0, 6.0, 0, 255, 0),
        ])
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 2)
        self.assertTrue(result["has_color"])
        self.assertIsNotNone(result["colors"])
        # Verify first point color is red
        colors = result["colors"]
        self.assertEqual(colors[0], 255)  # R
        self.assertEqual(colors[1], 0)    # G
        self.assertEqual(colors[2], 0)    # B
        # Second point is green
        self.assertEqual(colors[3], 0)    # R
        self.assertEqual(colors[4], 255)  # G
        self.assertEqual(colors[5], 0)    # B

    def test_nan_points_skipped(self):
        """NaN xyz values should be skipped."""
        nan = float("nan")
        cloud = make_xyz_cloud([
            (1.0, 2.0, 3.0),
            (nan, nan, nan),
            (4.0, 5.0, 6.0),
        ])
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 2)

    def test_downsampling(self):
        """More points than max_points should be stride-downsampled."""
        # Create 200 points, request max 50
        points = [(float(i), float(i), float(i)) for i in range(200)]
        cloud = make_xyz_cloud(points)
        result = point_cloud_to_dict(cloud, max_points=50)
        self.assertLessEqual(result["point_count"], 50)
        self.assertGreater(result["point_count"], 0)

    def test_missing_xyz_fields(self):
        """Cloud without x/y/z fields returns 0 points."""
        cloud = SimpleNamespace(
            fields=[make_point_field("intensity", 0)],
            point_step=4,
            width=10,
            height=1,
            data=b"\x00" * 40,
            header=SimpleNamespace(frame_id=""),
        )
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 0)

    def test_intensity_color(self):
        """Intensity field should be converted to grayscale."""
        fields = [
            make_point_field("x", 0),
            make_point_field("y", 4),
            make_point_field("z", 8),
            make_point_field("intensity", 12),
        ]
        data = bytearray()
        data.extend(struct.pack("<ffff", 1.0, 2.0, 3.0, 0.5))
        cloud = SimpleNamespace(
            fields=fields,
            point_step=16,
            width=1,
            height=1,
            data=bytes(data),
            header=SimpleNamespace(frame_id="lidar"),
        )
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 1)
        self.assertTrue(result["has_color"])
        # 0.5 intensity → ~127 gray
        gray = result["colors"][0]
        self.assertAlmostEqual(gray, 127, delta=1)

    def test_separate_rgb_fields(self):
        """R/G/B as separate uint8 fields."""
        fields = [
            make_point_field("x", 0),
            make_point_field("y", 4),
            make_point_field("z", 8),
            make_point_field("r", 12, datatype=2),
            make_point_field("g", 13, datatype=2),
            make_point_field("b", 14, datatype=2),
        ]
        data = bytearray()
        data.extend(struct.pack("<fff", 1.0, 2.0, 3.0))
        data.extend(bytes([128, 64, 32, 0]))  # r=128, g=64, b=32, pad=0
        cloud = SimpleNamespace(
            fields=fields,
            point_step=16,
            width=1,
            height=1,
            data=bytes(data),
            header=SimpleNamespace(frame_id="cam"),
        )
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 1)
        self.assertTrue(result["has_color"])
        self.assertEqual(result["colors"][0], 128)
        self.assertEqual(result["colors"][1], 64)
        self.assertEqual(result["colors"][2], 32)

    def test_large_cloud_performance(self):
        """1000 points should serialize without error."""
        points = [(float(i * 0.01), float(i * 0.02), float(i * 0.03)) for i in range(1000)]
        cloud = make_xyz_cloud(points)
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 1000)
        self.assertEqual(len(result["positions"]), 1000 * 12)

    def test_no_header(self):
        """Cloud without header attribute should not crash."""
        fields = [
            make_point_field("x", 0),
            make_point_field("y", 4),
            make_point_field("z", 8),
        ]
        data = struct.pack("<fff", 1.0, 2.0, 3.0)
        cloud = SimpleNamespace(
            fields=fields,
            point_step=12,
            width=1,
            height=1,
            data=data,
        )
        result = point_cloud_to_dict(cloud)
        self.assertEqual(result["point_count"], 1)
        self.assertEqual(result["frame_id"], "")


if __name__ == "__main__":
    unittest.main()
