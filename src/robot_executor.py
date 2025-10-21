"""Execute trajectories on real Franka robot using csc376_franky library."""

import numpy as np
from dataclasses import dataclass
from typing import List

import csc376_franky

from .trajectory_loader import TrajectoryData
from .config import (DEFAULT_EXECUTION_SPEED, MAX_JOINT_VELOCITY,
                     BASE_EXECUTION_DT, CONTINUITY_THRESHOLD)


@dataclass
class ExecutionConfig:
    """Configuration for robot trajectory execution."""
    speed_scale: float = DEFAULT_EXECUTION_SPEED
    max_joint_velocity: float = MAX_JOINT_VELOCITY


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
        controller = csc376_franky.FrankaJointTrajectoryController(robot_ip)
        controller.setupSignalHandler()
        print("Connected to robot")
        return controller
    except Exception as e:
        raise RuntimeError(f"Failed to connect to robot: {e}")


def execute_trajectory(
    robot_ip: str,
    trajectory: List[TrajectoryData],
    config: ExecutionConfig = None
):
    """Execute trajectory on real Franka robot.

    Args:
        robot_ip: IP address of the Franka robot
        trajectory: List of trajectory waypoints
        config: Execution configuration (uses defaults if None)

    Raises:
        RuntimeError: If execution fails
        ValueError: If trajectory validation fails
    """
    if config is None:
        config = ExecutionConfig()

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
        _move_to_start(controller, current_pos, start_pos, config)
        print("✓ Reached start position")

    q_traj = np.array([wp.franka_qpos for wp in trajectory])
    dt = BASE_EXECUTION_DT / config.speed_scale if config.speed_scale > 0 else BASE_EXECUTION_DT

    max_velocity = _estimate_max_velocity(q_traj, dt)
    if max_velocity > config.max_joint_velocity:
        print(f"\nWarning: Trajectory max velocity ({max_velocity:.3f} rad/s) "
              f"exceeds safety limit ({config.max_joint_velocity:.3f} rad/s)")
        response = input("Continue anyway? [yes/no]: ")
        if response.lower() != "yes":
            print("Execution cancelled by user")
            return

    print("\nExecution info:")
    print(f"  - Waypoints: {len(trajectory)}")
    print(f"  - Speed scale: {config.speed_scale}")
    print(f"  - Timestep: {dt:.3f} seconds")
    print(f"  - Estimated duration: {len(trajectory) * dt:.1f} seconds")
    print(f"  - Max velocity: {max_velocity:.3f} rad/s")

    print("\nExecuting trajectory...")
    try:
        controller.run_trajectory(q_traj, dt)
        print("Trajectory executed successfully")
    except Exception as e:
        raise RuntimeError(f"Trajectory execution failed: {e}")


def _move_to_start(controller, current_pos: np.ndarray, start_pos: np.ndarray, config: ExecutionConfig) -> None:
    """Move robot from current position to trajectory start position.

    Generates a linear interpolation trajectory that respects joint difference constraints.

    Args:
        controller: FrankaJointTrajectoryController instance
        current_pos: Current joint positions (7-element array)
        start_pos: Target start joint positions (7-element array)
        config: Execution configuration
    """
    delta = start_pos - current_pos
    max_delta = np.max(np.abs(delta))

    num_waypoints = int(np.ceil(max_delta / CONTINUITY_THRESHOLD)) + 1
    dt = BASE_EXECUTION_DT / config.speed_scale if config.speed_scale > 0 else BASE_EXECUTION_DT

    move_traj = np.linspace(current_pos, start_pos, num_waypoints)

    print(f"  - Move distance: {max_delta:.4f} rad")
    print(f"  - Move waypoints: {num_waypoints}")
    print(f"  - Estimated time: {num_waypoints * dt:.1f} seconds")

    controller.run_trajectory(move_traj, dt)


def _estimate_max_velocity(q_traj: np.ndarray, dt: float) -> float:
    """Estimate maximum joint velocity from trajectory.

    Args:
        q_traj: Joint trajectory array (N x 7)
        dt: Time step between waypoints in seconds

    Returns:
        Maximum joint velocity estimate in rad/s
    """
    if len(q_traj) < 2:
        return 0.0

    deltas = np.diff(q_traj, axis=0)
    max_delta = np.max(np.abs(deltas))
    return max_delta / dt
