"""Execute trajectories on real Franka robot using csc376_franky library."""

import time
import numpy as np
from typing import List

import csc376_bind_franky
from lib.dynamixel.multi_tendon_sync_rw import MultiTendonSyncRW

from .trajectory_loader import TrajectoryData
from .config import (
    CONTINUITY_THRESHOLD,
    TDCR_SERVO_IDS,
    TDCR_SPOOL_RADII_MM,
    TDCR_SERVO_DIRECTIONS,
    TDCR_DEFAULT_SPEED_MM_PER_SEC,
)


def execute_trajectory(
    robot_ip: str,
    trajectory: List[TrajectoryData],
    dt: float,
    tdcr_device: str
):
    controller = csc376_bind_franky.FrankaJointTrajectoryController(robot_ip)
    controller.setupSignalHandler()
    print("Connected to Franka robot")

    tdcr_controller = MultiTendonSyncRW(
        servo_ids=TDCR_SERVO_IDS,
        spool_radii_mm=TDCR_SPOOL_RADII_MM,
        device_name=tdcr_device,
        servo_dir=TDCR_SERVO_DIRECTIONS
    )
    tdcr_controller.set_zero_offsets_to_current_position()
    tdcr_controller.set_tendons_speeds_mm_per_sec(
        np.ones(9) * TDCR_DEFAULT_SPEED_MM_PER_SEC
    )
    print("TDCR initialized")

    current_pos = np.array(controller.get_current_joint_positions())
    start_pos = trajectory[0].franka_qpos
    position_error = np.linalg.norm(current_pos - start_pos)

    print("\nFranka start position check:")
    print(f"  Current position: {np.array2string(current_pos, precision=4)}")
    print(f"  Trajectory start: {np.array2string(start_pos, precision=4)}")
    print(f"  Position error: {position_error:.4f} rad")

    if position_error > 0.01:
        print("\nMoving Franka to start position...")
        _move_to_start(controller, current_pos, start_pos, dt)
        print("Franka reached start position")

    initial_tdcr_lengths_m = trajectory[0].tdcr_tendon_lengths

    q_traj = np.array([wp.franka_qpos for wp in trajectory])

    print("\nExecution info:")
    print(f"  - Waypoints: {len(trajectory)}")
    print(f"  - Timestep: {dt:.6f} seconds ({1.0/dt:.1f} Hz)")
    print(f"  - Estimated duration: {len(trajectory) * dt:.1f} seconds")
    print(f"  - TDCR: {'enabled' if tdcr_controller else 'disabled'}")

    def trajectory_callback(index):
        relative_lengths_mm = _transform_tendon_lengths(
            trajectory[index].tdcr_tendon_lengths,
            initial_tdcr_lengths_m
        )
        tdcr_controller.async_set_tendons_mm(relative_lengths_mm)
    controller.set_trajectory_callback(trajectory_callback)

    print("\nExecuting trajectory...")
    controller.run_joint_trajectory(q_traj, dt)
    print("Trajectory executed successfully")

    _move_tdcr_to_zero(tdcr_controller)


def _move_to_start(controller, current_pos: np.ndarray, start_pos: np.ndarray, dt: float) -> None:
    delta = start_pos - current_pos
    max_delta = np.max(np.abs(delta))

    num_waypoints = int(np.ceil(max_delta / CONTINUITY_THRESHOLD)) * 10
    move_traj = np.linspace(current_pos, start_pos, num_waypoints)

    print(f"  - Max joint distance: {max_delta:.4f} rad")
    print(f"  - Move waypoints: {num_waypoints}")
    print(f"  - Estimated time: {num_waypoints * dt:.1f} seconds")

    controller.run_joint_trajectory(move_traj, dt)

def _transform_tendon_lengths(absolute_lengths_m: np.ndarray, initial_lengths_m: np.ndarray) -> np.ndarray:
    """Transform absolute tendon lengths (meters) to relative lengths (mm)."""
    return (absolute_lengths_m - initial_lengths_m) * 1000.0


def _move_tdcr_to_zero(tdcr_controller, wait_before: float = 10.0) -> None:
    """Move TDCR back to zero position after waiting."""
    print(f"\nWaiting {wait_before:.1f}s before returning TDCR to zero...")
    time.sleep(wait_before)
    print("Moving TDCR to zero position...")
    current_lengths = tdcr_controller.get_tendons_mm()
    max_delta = np.max(np.abs(current_lengths))
    tdcr_controller.async_set_tendons_mm(np.zeros(9))
    wait_time = max_delta / TDCR_DEFAULT_SPEED_MM_PER_SEC + 0.5
    time.sleep(wait_time)
    print("TDCR returned to zero position")
