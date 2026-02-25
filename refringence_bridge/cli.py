"""
CLI entry point for the Refringence bridge.

Supports both ROS 2 node mode and standalone mode.
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        prog="refringence-bridge",
        description="Bridge ROS 2 simulations to Refringence workspace",
    )
    parser.add_argument(
        "--token", required=True,
        help="Refringence connector token",
    )
    parser.add_argument(
        "--url", default="https://api.refringence.com",
        help="Refringence server URL (default: https://api.refringence.com)",
    )
    parser.add_argument(
        "--rate", type=int, default=30,
        help="Telemetry publishing rate in Hz (default: 30)",
    )
    parser.add_argument(
        "--standalone", action="store_true",
        help="Run in standalone mode (no ROS 2 required)",
    )
    parser.add_argument(
        "--urdf", default=None,
        help="Path to URDF file (standalone mode, optional)",
    )
    parser.add_argument(
        "--mesh-dir", default=None,
        help="Directory containing mesh files for URDF (standalone mode)",
    )
    parser.add_argument(
        "--demo", action="store_true",
        help="Run demo mode with sinusoidal joint oscillation (standalone)",
    )

    args = parser.parse_args()

    if args.rate < 1 or args.rate > 120:
        parser.error("--rate must be between 1 and 120")

    if not args.token.strip():
        parser.error("--token must not be empty")

    if args.standalone:
        from refringence_bridge.standalone import run_standalone
        run_standalone(
            token=args.token,
            server_url=args.url,
            urdf_path=args.urdf,
            mesh_dir=args.mesh_dir,
            rate_hz=args.rate,
            demo=args.demo,
        )
    else:
        # Try ROS 2 mode
        try:
            from refringence_bridge.ros2_node import run_ros2_node
            run_ros2_node(
                token=args.token,
                server_url=args.url,
                rate_hz=args.rate,
            )
        except ImportError:
            print(
                "ERROR: rclpy not found. Install ROS 2 or use --standalone mode.",
                file=sys.stderr,
            )
            sys.exit(1)


if __name__ == "__main__":
    main()
