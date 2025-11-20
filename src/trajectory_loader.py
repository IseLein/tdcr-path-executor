"""Load and validate robot trajectory JSON files."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np

from .config import (
    FRANKA_JOINT_LIMITS,
    TDCR_TENDON_COUNT,
    CONTINUITY_THRESHOLD
)


@dataclass
class TrajectoryData:
    """Represents a single waypoint in a trajectory."""
    franka_qpos: np.ndarray
    tdcr_tendon_lengths: np.ndarray
    step: int
    time: float

    def __post_init__(self):
        self.franka_qpos = np.asarray(self.franka_qpos, dtype=np.float64)
        self.tdcr_tendon_lengths = np.asarray(
            self.tdcr_tendon_lengths,
            dtype=np.float64
        )


def load_trajectory(json_path: str) -> tuple[List[TrajectoryData], float]:
    """Load trajectory from JSON file.

    Supports multiple JSON formats:
    - {"trajectory": [...]}
    - {"raw_joint_trajectory": [...]}

    Each waypoint should contain:
    - "franka_qpos": list of 7 joint angles (required)
    - "tendon_lengths": list of 9 values
    - "step": integer step index (optional)
    - "time": float timestamp in seconds (required)

    Args:
        json_path: Path to JSON trajectory file

    Returns:
        Tuple of (trajectory waypoints, dt between steps in seconds)

    Raises:
        FileNotFoundError: If file doesn't exist
        ValueError: If JSON format is invalid or dt is not constant
    """
    json_path = Path(json_path)
    if not json_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {json_path}")

    with open(json_path, 'r') as f:
        data = json.load(f)

    if "raw_joint_trajectory" in data:
        raw_trajectory = data["raw_joint_trajectory"]
    elif "trajectory" in data:
        raw_trajectory = data["trajectory"]
    else:
        raise ValueError(
            "JSON must contain 'trajectory' or 'raw_joint_trajectory' key. "
            f"Found keys: {list(data.keys())}"
        )

    trajectory = []
    for i, waypoint in enumerate(raw_trajectory):
        franka_qpos = waypoint["franka_qpos"]
        tdcr_tendon_lengths = waypoint["tendon_lengths"]
        step = waypoint["step"]
        time = waypoint["time"]

        trajectory.append(TrajectoryData(
            franka_qpos=franka_qpos,
            tdcr_tendon_lengths=tdcr_tendon_lengths,
            step=step,
            time=time
        ))

    # Calculate dt from time differences
    if len(trajectory) < 2:
        raise ValueError("Trajectory must have at least 2 waypoints to calculate dt")
    
    times = np.array([wp.time for wp in trajectory])
    dts = np.diff(times)
    dt = np.mean(dts)
    
    # Verify dt is constant (within tolerance)
    if np.std(dts) > 1e-6:
        raise ValueError(
            f"Time differences are not constant! "
            f"Mean dt: {dt:.6f}s, Std: {np.std(dts):.9f}s"
        )
    
    print(f"✓ Trajectory dt: {dt:.6f}s ({1.0/dt:.1f} Hz)")

    validate_trajectory(trajectory)
    return trajectory, dt


def validate_trajectory(trajectory: List[TrajectoryData]) -> None:
    """Validate trajectory for safety and correctness.

    Checks:
    - Franka joint positions are 7 elements
    - Joint angles are within limits
    - TDCR tendon lengths are 9 elements
    - Waypoint transitions are continuous

    Args:
        trajectory: List of trajectory waypoints

    Raises:
        ValueError: If validation fails
    """
    if len(trajectory) == 0:
        raise ValueError("Trajectory is empty")

    for i, waypoint in enumerate(trajectory):
        if len(waypoint.franka_qpos) != 7:
            raise ValueError(
                f"Waypoint {i}: franka_qpos must have 7 elements, "
                f"got {len(waypoint.franka_qpos)}"
            )

        for j, (q, (q_min, q_max)) in enumerate(
            zip(waypoint.franka_qpos, FRANKA_JOINT_LIMITS)
        ):
            if not (q_min <= q <= q_max):
                raise ValueError(
                    f"Waypoint {i}: Joint {j} value {q:.4f} rad "
                    f"outside limits [{q_min:.4f}, {q_max:.4f}]"
                )

        if len(waypoint.tdcr_tendon_lengths) != TDCR_TENDON_COUNT:
            raise ValueError(
                f"Waypoint {i}: tdcr_tendon_lengths must have "
                f"{TDCR_TENDON_COUNT} elements, "
                f"got {len(waypoint.tdcr_tendon_lengths)}"
            )

        if i > 0:
            prev_qpos = trajectory[i-1].franka_qpos
            curr_qpos = waypoint.franka_qpos
            joint_deltas = np.abs(curr_qpos - prev_qpos)
            max_delta = np.max(joint_deltas)

            if max_delta > CONTINUITY_THRESHOLD:
                max_joint_idx = np.argmax(joint_deltas)
                raise ValueError(
                    f"Waypoint {i}: Large discontinuity detected! "
                    f"Joint {max_joint_idx} jumped {max_delta:.4f} rad "
                    f"(threshold: {CONTINUITY_THRESHOLD:.4f} rad). "
                    f"This could cause unsafe robot motion."
                )

    print(f"✓ Trajectory validated: {len(trajectory)} waypoints")


def get_trajectory_summary(trajectory: List[TrajectoryData]) -> dict:
    """Get summary statistics about a trajectory.

    Args:
        trajectory: List of trajectory waypoints

    Returns:
        Dictionary with summary statistics
    """
    if len(trajectory) == 0:
        return {"num_waypoints": 0}

    all_qpos = np.array([wp.franka_qpos for wp in trajectory])
    joint_mins = np.min(all_qpos, axis=0)
    joint_maxs = np.max(all_qpos, axis=0)
    joint_ranges = joint_maxs - joint_mins

    if len(trajectory) > 1:
        deltas = np.diff(all_qpos, axis=0)
        max_velocities = np.max(np.abs(deltas), axis=0)
    else:
        max_velocities = np.zeros(7)

    return {
        "num_waypoints": len(trajectory),
        "joint_ranges": joint_ranges.tolist(),
        "max_joint_deltas": max_velocities.tolist(),
        "start_config": trajectory[0].franka_qpos.tolist(),
        "end_config": trajectory[-1].franka_qpos.tolist(),
    }
