#!/usr/bin/env python3
"""
TDCR Path Executor - Main entry point.
"""

import argparse
import sys
from pathlib import Path

from src.trajectory_loader import load_trajectory
from src.mujoco_sim import visualize_trajectory, SimulationConfig
from src.robot_executor import execute_trajectory
from src.config import DEFAULT_SCENE_PATH


def confirm_execution() -> bool:
    print("\n" + "=" * 60)
    print("Simulation complete!")
    print("=" * 60)
    response = input("Execute trajectory on real robot? [yes/no]: ")
    return response.lower() == "yes"


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Visualize and execute robot trajectories",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Visualize only (uses default scene)
  python execute_path.py trajectory.json --simulate-only

  # Full workflow with robot execution
  python execute_path.py trajectory.json --robot-ip 192.168.1.107

  # Custom playback speed and custom scene
  python execute_path.py trajectory.json --scene custom_scene.xml --fps 20 --no-loop
        """
    )

    parser.add_argument(
        "trajectory",
        type=str,
        help="Path to trajectory JSON file"
    )
    parser.add_argument(
        "--scene",
        type=str,
        default=DEFAULT_SCENE_PATH,
        help=f"Path to MuJoCo XML scene file (default: {DEFAULT_SCENE_PATH})"
    )
    parser.add_argument(
        "--robot-ip",
        type=str,
        default=None,
        help="IP address of Franka robot (required for execution)"
    )
    parser.add_argument(
        "--simulate-only",
        action="store_true",
        help="Only run simulation, don't execute on robot"
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=100,
        help="Simulation playback speed (default=10)"
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="Don't loop trajectory in simulation"
    )

    args = parser.parse_args()

    trajectory_path = Path(args.trajectory)
    scene_path = Path(args.scene)

    if not trajectory_path.exists():
        print(f"Error: Trajectory file not found: {trajectory_path}")
        sys.exit(1)

    if not scene_path.exists():
        print(f"Error: Scene file not found: {scene_path}")
        sys.exit(1)

    print("=" * 60)
    print("TDCR Path Executor")
    print("=" * 60)

    print(f"\nLoading trajectory from {trajectory_path}")
    try:
        trajectory, dt = load_trajectory(str(trajectory_path))
        print(f"\tLoaded {len(trajectory)} waypoints")
    except Exception as e:
        print(f"Error loading trajectory: {e}")
        sys.exit(1)

    print("\nLaunching MuJoCo visualization")
    sim_config = SimulationConfig(
        fps=args.fps,
        loop=not args.no_loop
    )

    try:
        visualize_trajectory(str(scene_path), trajectory, sim_config)
    except KeyboardInterrupt:
        print("\nVisualization interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error during visualization: {e}")
        sys.exit(1)

    print("\nSimulation complete")
    print("\nExecution decision")

    if args.simulate_only:
        print("\t--simulate-only flag set, skipping execution")
        return

    if args.robot_ip is None:
        print("\tNo --robot-ip provided, skipping execution")
        return

    if not confirm_execution():
        print("\tUser declined execution")
        return

    print("\nExecuting on robot")

    try:
        execute_trajectory(args.robot_ip, trajectory, dt)
        print("\nExecution complete")
    except KeyboardInterrupt:
        print("\nExecution interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error during execution: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
