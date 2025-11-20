"""Execute trajectories on real Franka robot using csc376_franky library."""

import time
import numpy as np
from dataclasses import dataclass
from typing import List, Dict

import csc376_bind_franky

from .trajectory_loader import TrajectoryData
from .config import CONTINUITY_THRESHOLD


def connect_robot(robot_ip: str):
    """Initialize connection to Franka robot.

    Args:
        robot_ip: IP address of the Franka robot

    Returns:
        FrankaJointTrajectoryController instance

    Raises:
        RuntimeError: If connection fails
    """
    print(f"Connecting to robot at {robot_ip}...")
    try:
        controller = csc376_bind_franky.FrankaJointTrajectoryController(robot_ip)
        controller.setupSignalHandler()
        print("Connected to robot")
        return controller
    except Exception as e:
        raise RuntimeError(f"Failed to connect to robot: {e}")


def execute_trajectory(
    robot_ip: str,
    trajectory: List[TrajectoryData],
    dt: float
):
    """Execute trajectory on real Franka robot.

    Args:
        robot_ip: IP address of the Franka robot
        trajectory: List of trajectory waypoints
        dt: Time step between waypoints in seconds

    Raises:
        RuntimeError: If execution fails
        ValueError: If trajectory validation fails
    """
    controller = connect_robot(robot_ip)

    # Check if we need to move to start position
    current_pos = np.array(controller.get_current_joint_positions())
    start_pos = trajectory[0].franka_qpos
    position_error = np.linalg.norm(current_pos - start_pos)

    print("\nStart position check:")
    print(f"  Current position: {np.array2string(current_pos, precision=4)}")
    print(f"  Trajectory start: {np.array2string(start_pos, precision=4)}")
    print(f"  Position error: {position_error:.4f} rad")

    if position_error > 0.01:
        print("\nMoving to start position...")
        _move_to_start(controller, current_pos, start_pos, dt)
        print("✓ Reached start position")

    q_traj = np.array([wp.franka_qpos for wp in trajectory])

    print("\nExecution info:")
    print(f"\t- Waypoints: {len(trajectory)}")
    print(f"\t- Timestep: {dt:.6f} seconds ({1.0/dt:.1f} Hz)")
    print(f"\t- Estimated duration: {len(trajectory) * dt:.1f} seconds")

    print("\nExecuting trajectory...")
    try:
        controller.run_joint_trajectory(q_traj, dt)
        print("Trajectory executed successfully")
    except Exception as e:
        raise RuntimeError(f"Trajectory execution failed: {e}")


def _move_to_start(controller, current_pos: np.ndarray, start_pos: np.ndarray, dt: float) -> None:
    """Move robot from current position to trajectory start position.

    Args:
        controller: FrankaJointTrajectoryController instance
        current_pos: Current joint positions (7-element array)
        start_pos: Target start joint positions (7-element array)
        dt: Time step between waypoints in seconds
    """
    delta = start_pos - current_pos
    max_delta = np.max(np.abs(delta))

    num_waypoints = int(np.ceil(max_delta / CONTINUITY_THRESHOLD))

    move_traj = np.linspace(current_pos, start_pos, num_waypoints)

    print(f"  - Move distance: {max_delta:.4f} rad")
    print(f"  - Move waypoints: {num_waypoints}")
    print(f"  - Estimated time: {num_waypoints * dt:.1f} seconds")

    controller.run_joint_trajectory(move_traj, dt)
