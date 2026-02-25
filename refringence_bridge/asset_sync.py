"""
URDF + mesh file discovery and upload utilities.

Discovers URDF from ROS parameter or file, extracts mesh filenames,
and uploads mesh files to Refringence via the connector.
"""

import logging
import re
from pathlib import Path
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)


def discover_urdf_from_file(path: str) -> Optional[str]:
    """Read a URDF from a local file path."""
    p = Path(path)
    if not p.exists():
        logger.error("URDF file not found: %s", path)
        return None
    return p.read_text(encoding="utf-8")


def discover_urdf_from_param(node: "Any") -> Optional[str]:
    """
    Read the robot description from the /robot_description parameter.
    Requires a ROS 2 node.
    """
    try:
        node.declare_parameter("robot_description", "")
    except Exception:
        pass  # Already declared
    urdf = node.get_parameter("robot_description").get_parameter_value().string_value
    if urdf:
        return urdf
    return None


def extract_mesh_filenames(urdf_xml: str) -> List[str]:
    """
    Parse mesh filenames from a URDF XML string.

    Handles:
      - <mesh filename="package://pkg/meshes/link.stl"/>
      - <mesh filename="file:///path/to/mesh.stl"/>
      - <mesh filename="model://model_name/mesh.stl"/>
      - <mesh filename="meshes/link.stl"/>
    """
    # Match <mesh> tags with filename attribute in any position among other attributes
    pattern = r'<mesh\s+[^>]*?filename=["\']([^"\']+)["\']'
    raw_paths = re.findall(pattern, urdf_xml)

    filenames = []
    for raw in raw_paths:
        name = raw
        # Strip package://package_name/ prefix
        if name.startswith("package://"):
            name = name[len("package://"):]
            slash_idx = name.find("/")
            if slash_idx >= 0:
                name = name[slash_idx + 1:]
        # Strip file:// prefix (handle both Unix /path and Windows C:/path)
        # file:///opt/foo → /opt/foo (Unix absolute), file://C:/foo → C:/foo
        elif name.startswith("file:///"):
            name = name[len("file://"):]  # Keep leading / for Unix absolute paths
        elif name.startswith("file://"):
            name = name[len("file://"):]
        # Strip model:// prefix (keep full path after prefix, unlike package://)
        elif name.startswith("model://"):
            name = name[len("model://"):]

        if name and name not in filenames:
            filenames.append(name)

    return filenames


def resolve_mesh_paths(
    mesh_filenames: List[str],
    search_dirs: Optional[List[str]] = None,
) -> List[Tuple[str, str]]:
    """
    Resolve mesh filenames to absolute file paths.

    Returns list of (filename, absolute_path) for files that exist.
    Searches in search_dirs, current directory, and common ROS paths.
    """
    dirs = list(search_dirs or [])
    dirs.append(".")

    resolved: List[Tuple[str, str]] = []

    for filename in mesh_filenames:
        found = False
        # Sanitize: prevent path traversal by only using basename for search
        safe_basename = Path(filename).name
        if not safe_basename or safe_basename in ('.', '..'):
            logger.warning("Invalid mesh filename: %s", filename)
            continue

        def _is_within(resolved_path: Path, allowed_dirs: list) -> bool:
            """Check resolved path is within an allowed directory (symlink-safe)."""
            for d in allowed_dirs:
                try:
                    resolved_path.relative_to(Path(d).resolve())
                    return True
                except ValueError:
                    continue
            return False

        # Try filename directly (only absolute or within search dirs)
        direct = Path(filename)
        if direct.is_absolute() and direct.exists():
            resolved_direct = direct.resolve()
            if _is_within(resolved_direct, dirs):
                resolved.append((safe_basename, str(resolved_direct)))
                continue

        # Search in provided directories
        for d in dirs:
            candidate = Path(d) / filename
            if candidate.exists():
                resolved_candidate = candidate.resolve()
                if _is_within(resolved_candidate, dirs):
                    resolved.append((safe_basename, str(resolved_candidate)))
                    found = True
                    break
            # Also try just the basename
            candidate = Path(d) / safe_basename
            if candidate.exists():
                resolved_base = candidate.resolve()
                if _is_within(resolved_base, dirs):
                    resolved.append((safe_basename, str(resolved_base)))
                    found = True
                    break

        if not found:
            logger.warning("Mesh file not found: %s", filename)

    return resolved


async def upload_discovered_meshes(
    connector: "Any",
    urdf_xml: str,
    search_dirs: Optional[List[str]] = None,
) -> int:
    """
    Discover meshes from URDF and upload them via the connector.

    Returns the number of meshes successfully uploaded.
    """
    filenames = extract_mesh_filenames(urdf_xml)
    if not filenames:
        logger.info("No mesh files referenced in URDF")
        return 0

    logger.info("Found %d mesh references in URDF", len(filenames))
    resolved = resolve_mesh_paths(filenames, search_dirs)

    uploaded = 0
    for filename, abs_path in resolved:
        try:
            await connector.upload_mesh_file(abs_path)
            uploaded += 1
        except Exception as e:
            logger.error("Failed to upload mesh %s: %s", filename, e)

    logger.info("Uploaded %d/%d mesh files", uploaded, len(filenames))
    return uploaded
