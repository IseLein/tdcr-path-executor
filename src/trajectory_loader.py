"""Load and validate robot trajectory JSON files."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List
from scipy.interpolate import CubicSpline

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

    def __post_init__(self):
        self.franka_qpos = np.asarray(self.franka_qpos, dtype=np.float64)
        self.tdcr_tendon_lengths = np.asarray(
            self.tdcr_tendon_lengths,
            dtype=np.float64
        )


def interpolate_trajectory(trajectory: List[TrajectoryData]) -> List[TrajectoryData]:
    """Interpolate trajectory using cubic splines for smooth motion.

    Treats robot state as a single point in 16D configuration space
    (7 Franka joints + 9 TDCR tendons), ensuring franka positions and
    tendon lengths remain synchronized during interpolation. Creates
    C2-continuous (smooth position, velocity, acceleration) trajectory.

    Args:
        trajectory: Original trajectory waypoints

    Returns:
        Smoothly interpolated trajectory with additional waypoints
    """
    if len(trajectory) < 2:
        return trajectory

    combined_waypoints = np.array([
        np.concatenate([wp.franka_qpos, wp.tdcr_tendon_lengths])
        for wp in trajectory
    ])

    original_params = np.linspace(0, 1, len(trajectory))
    spline_waypoints = CubicSpline(
        original_params, combined_waypoints, bc_type='natural'
    )

    # Determine sampling density to meet continuity threshold
    max_delta = 0.0
    for i in range(1, len(trajectory)):
        delta = np.max(
            np.abs(trajectory[i].franka_qpos - trajectory[i-1].franka_qpos)
        )
        max_delta = max(max_delta, delta)

    samples_per_segment = max(
        1, int(np.ceil(max_delta / (0.8 * CONTINUITY_THRESHOLD)))
    )
    print(f"Max segment: {max_delta}")
    print(f"Samples: {samples_per_segment}")
    total_samples = len(trajectory) * samples_per_segment

    dense_params = np.linspace(0, 1, total_samples)
    dense_waypoints = spline_waypoints(dense_params)

    interpolated = []
    for i in range(len(dense_params)):
        interpolated.append(TrajectoryData(
            franka_qpos=dense_waypoints[i, :7],
            tdcr_tendon_lengths=dense_waypoints[i, 7:],
            step=(i + 1)
        ))
    return interpolated


def load_trajectory(json_path: str) -> tuple[List[TrajectoryData], float]:
    """Load trajectory from JSON file.

    Supports multiple JSON formats:
    - {"trajectory": [...]}
    - {"raw_joint_trajectory": [...]}

    Each waypoint should contain:
    - "franka_qpos": list of 7 joint angles (required)
    - "tendon_lengths": list of 9 values
    - "step": integer step index (optional)

    Args:
        json_path: Path to JSON trajectory file

    Returns:
        Tuple of (trajectory waypoints, waypoint_multiplier)
        - trajectory: List of TrajectoryData waypoints
        - waypoint_multiplier: Ratio of interpolated to original waypoints

    Raises:
        FileNotFoundError: If file doesn't exist
        ValueError: If JSON format is invalid
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

        trajectory.append(TrajectoryData(
            franka_qpos=franka_qpos,
            tdcr_tendon_lengths=tdcr_tendon_lengths,
            step=step
        ))

    # Interpolate trajectory to ensure continuity
    original_count = len(trajectory)
    trajectory = interpolate_trajectory(trajectory)
    waypoint_multiplier = len(trajectory) / original_count if original_count > 0 else 1.0

    if len(trajectory) > original_count:
        print(f"✓ Interpolated trajectory: {original_count} → {len(trajectory)} waypoints "
              f"({waypoint_multiplier:.2f}x for continuity)")

    validate_trajectory(trajectory)
    return trajectory, waypoint_multiplier


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
