"""Unit tests for asset_sync.py URDF/mesh discovery and upload."""

import os
import tempfile
import unittest

from refringence_bridge.asset_sync import (
    discover_urdf_from_file,
    extract_mesh_filenames,
    resolve_mesh_paths,
)


SAMPLE_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://my_robot/meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="link_1">
    <visual>
      <geometry>
        <mesh filename="package://my_robot/meshes/link1.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <geometry>
        <mesh filename="file:///opt/meshes/link2.obj"/>
      </geometry>
    </visual>
  </link>
  <link name="link_3">
    <visual>
      <geometry>
        <mesh filename="model://gazebo_models/mesh.stl"/>
      </geometry>
    </visual>
  </link>
  <link name="link_4">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
  </joint>
</robot>
"""


class TestDiscoverUrdfFromFile(unittest.TestCase):
    def test_existing_file(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
            f.write(SAMPLE_URDF)
            f.flush()
            result = discover_urdf_from_file(f.name)
        os.unlink(f.name)
        self.assertIsNotNone(result)
        self.assertIn("test_robot", result)

    def test_nonexistent_file(self):
        result = discover_urdf_from_file("/nonexistent/robot.urdf")
        self.assertIsNone(result)


class TestExtractMeshFilenames(unittest.TestCase):
    def test_package_prefix(self):
        filenames = extract_mesh_filenames(SAMPLE_URDF)
        self.assertIn("meshes/base.stl", filenames)
        self.assertIn("meshes/link1.stl", filenames)

    def test_file_prefix(self):
        filenames = extract_mesh_filenames(SAMPLE_URDF)
        self.assertIn("/opt/meshes/link2.obj", filenames)

    def test_model_prefix(self):
        filenames = extract_mesh_filenames(SAMPLE_URDF)
        self.assertIn("gazebo_models/mesh.stl", filenames)

    def test_no_duplicates(self):
        urdf = """
        <robot>
          <link name="a"><visual><geometry>
            <mesh filename="package://pkg/m.stl"/>
          </geometry></visual></link>
          <link name="b"><visual><geometry>
            <mesh filename="package://pkg/m.stl"/>
          </geometry></visual></link>
        </robot>
        """
        filenames = extract_mesh_filenames(urdf)
        self.assertEqual(filenames.count("m.stl"), 1)

    def test_no_meshes(self):
        urdf = '<robot><link name="a"><visual><geometry><box size="1 1 1"/></geometry></visual></link></robot>'
        filenames = extract_mesh_filenames(urdf)
        self.assertEqual(filenames, [])

    def test_total_count(self):
        filenames = extract_mesh_filenames(SAMPLE_URDF)
        self.assertEqual(len(filenames), 4)


class TestResolveMeshPaths(unittest.TestCase):
    def test_resolve_existing_file(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            # Create a fake mesh file
            mesh_path = os.path.join(tmpdir, "test.stl")
            with open(mesh_path, "w") as f:
                f.write("solid test")

            resolved = resolve_mesh_paths(["test.stl"], [tmpdir])
            self.assertEqual(len(resolved), 1)
            self.assertEqual(resolved[0][0], "test.stl")

    def test_resolve_nested_path(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            meshes_dir = os.path.join(tmpdir, "meshes")
            os.makedirs(meshes_dir)
            mesh_path = os.path.join(meshes_dir, "link.stl")
            with open(mesh_path, "w") as f:
                f.write("solid link")

            resolved = resolve_mesh_paths(["meshes/link.stl"], [tmpdir])
            self.assertEqual(len(resolved), 1)

    def test_missing_file_skipped(self):
        resolved = resolve_mesh_paths(["nonexistent.stl"], [])
        self.assertEqual(len(resolved), 0)


if __name__ == "__main__":
    unittest.main()
