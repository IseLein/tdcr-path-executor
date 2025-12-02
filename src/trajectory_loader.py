"""Load and validate robot trajectory JSON files."""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo

from .config import (
    FRANKA_JOINT_LIMITS,
    TDCR_TENDON_COUNT,
    CONTINUITY_THRESHOLD,
    DEFAULT_FRANKA_VEL_LIMITS,
    DEFAULT_FRANKA_ACC_LIMITS,
    DEFAULT_TENDON_VEL_LIMITS,
    DEFAULT_TENDON_ACC_LIMITS,
    DEFAULT_CONTROL_FREQ
)


@dataclass
class TrajectoryData:
    """Represents a single waypoint in a trajectory."""
    franka_qpos: np.ndarray
    tdcr_tendon_lengths: np.ndarray
    step: int
    time: float    # absolute time from start

    def __post_init__(self):
        self.franka_qpos = np.asarray(self.franka_qpos, dtype=np.float64)
        self.tdcr_tendon_lengths = np.asarray(
            self.tdcr_tendon_lengths,
            dtype=np.float64
        )


def _smooth_raw_trajectory(
    raw_waypoints: list,
    control_freq: float = DEFAULT_CONTROL_FREQ,
    franka_vel_limits: np.ndarray = DEFAULT_FRANKA_VEL_LIMITS,
    franka_acc_limits: np.ndarray = DEFAULT_FRANKA_ACC_LIMITS,
    tendon_vel_limits: np.ndarray = DEFAULT_TENDON_VEL_LIMITS,
    tendon_acc_limits: np.ndarray = DEFAULT_TENDON_ACC_LIMITS,
) -> tuple[List[TrajectoryData], float]:
    franka_positions = np.array([wp['franka_qpos'] for wp in raw_waypoints])
    tendon_positions = np.array([wp['tendon_lengths'] for wp in raw_waypoints])

    joint_positions = np.hstack([franka_positions, tendon_positions])
    n_joints = joint_positions.shape[1]

    print(f"Smoothing {len(raw_waypoints)} waypoints with {n_joints} DOF...")

    gridpoints = np.linspace(0, 1, len(joint_positions))
    path = ta.SplineInterpolator(gridpoints, joint_positions)

    vlim = np.hstack([franka_vel_limits, tendon_vel_limits])
    alim = np.hstack([franka_acc_limits, tendon_acc_limits])

    pc_vel = constraint.JointVelocityConstraint(vlim)
    pc_acc = constraint.JointAccelerationConstraint(alim)

    instance = algo.TOPPRA(
        [pc_vel, pc_acc],
        path,
        parametrizer="ParametrizeConstAccel"
    )

    jnt_traj = instance.compute_trajectory(0, 0)

    if jnt_traj is None:
        raise RuntimeError(
            "TOPPRA failed to compute trajectory. "
            "Try relaxing velocity/acceleration limits."
        )

    duration = jnt_traj.duration
    dt = 1.0 / control_freq

    n_samples = int(np.round(duration / dt)) + 1
    ts_sample = np.linspace(0, duration, n_samples)

    print(f"TOPPRA duration: {duration:.3f}s, {n_samples} samples at {control_freq} Hz")

    qs_sample = jnt_traj(ts_sample)

    trajectory = []
    for i, t in enumerate(ts_sample):
        franka_qpos = qs_sample[i, :7]
        tendon_lengths = qs_sample[i, 7:]

        trajectory.append(TrajectoryData(
            franka_qpos=franka_qpos,
            tdcr_tendon_lengths=tendon_lengths,
            step=i,
            time=float(t)
        ))

    return trajectory


def load_trajectory_no_toppra(
    json_path: str,
    dt: float = 0.02,
    franka_vel_limits: np.ndarray = DEFAULT_FRANKA_VEL_LIMITS /  6,
    tendon_vel_limits: np.ndarray = DEFAULT_TENDON_VEL_LIMITS,
) -> tuple[List[TrajectoryData], float]:
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

    if len(raw_trajectory) == 0:
        raise ValueError("Trajectory is empty")

    if len(raw_trajectory) < 2:
        raise ValueError("Trajectory must have at least 2 waypoints")

    max_franka_delta = franka_vel_limits * dt
    max_tendon_delta = tendon_vel_limits * dt
    max_deltas = np.hstack([max_franka_delta, max_tendon_delta])

    trajectory = []
    step_idx = 0

    for i in range(len(raw_trajectory) - 1):
        wp_a = raw_trajectory[i]
        wp_b = raw_trajectory[i + 1]

        pos_a = np.hstack([wp_a['franka_qpos'], wp_a['tendon_lengths']])
        pos_b = np.hstack([wp_b['franka_qpos'], wp_b['tendon_lengths']])

        deltas = np.abs(pos_b - pos_a)

        with np.errstate(divide='ignore', invalid='ignore'):
            steps_per_dof = np.ceil(deltas / max_deltas)
            steps_per_dof = np.nan_to_num(steps_per_dof, nan=1.0, posinf=1.0)

        n_steps = max(1, int(np.max(steps_per_dof)))

        for j in range(n_steps):
            alpha = j / n_steps
            pos = pos_a + alpha * (pos_b - pos_a)

            trajectory.append(TrajectoryData(
                franka_qpos=pos[:7],
                tdcr_tendon_lengths=pos[7:],
                step=step_idx,
                time=step_idx * dt
            ))
            step_idx += 1

    final_wp = raw_trajectory[-1]
    trajectory.append(TrajectoryData(
        franka_qpos=np.array(final_wp['franka_qpos']),
        tdcr_tendon_lengths=np.array(final_wp['tendon_lengths']),
        step=step_idx,
        time=step_idx * dt
    ))

    print(f"Generated {len(trajectory)} waypoints from {len(raw_trajectory)} raw waypoints")
    print(f"Total duration: {trajectory[-1].time:.3f}s")

    validate_trajectory(trajectory)
    return trajectory, dt


def load_trajectory(json_path: str) -> tuple[List[TrajectoryData], float]:
    json_path = Path(json_path)
    if not json_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {json_path}")

    with open(json_path, 'r') as f:
        data = json.load(f)

    if "raw_joint_trajectory" in data:
        raw_trajectory = data["raw_joint_trajectory"]
    elif "trajectory" in data:
        raw_trajectory = data["trajectory"]
    elif "smooth_trajectory" in data:
        raw_trajectory = data["smooth_trajectory"]
    else:
        raise ValueError(
            "JSON must contain 'trajectory', 'raw_joint_trajectory' or 'smooth_trajectory' key. "
            f"Found keys: {list(data.keys())}"
        )

    if len(raw_trajectory) == 0:
        raise ValueError("Trajectory is empty")

    is_raw = 'time' not in raw_trajectory[0]

    if is_raw:
        print(f"Detected raw trajectory, applying TOPPRA smoothing...")
        trajectory = _smooth_raw_trajectory(raw_waypoints=raw_trajectory)
    else:
        trajectory = []
        for i, waypoint in enumerate(raw_trajectory):
            franka_qpos = waypoint["franka_qpos"]
            tdcr_tendon_lengths = waypoint["tendon_lengths"]
            step = waypoint.get("step", i)
            time = waypoint["time"]

            trajectory.append(TrajectoryData(
                franka_qpos=franka_qpos,
                tdcr_tendon_lengths=tdcr_tendon_lengths,
                step=step,
                time=time
            ))

    if len(trajectory) < 2:
        raise ValueError("Trajectory must have at least 2 waypoints to calculate dt")

    times = np.array([wp.time for wp in trajectory])
    dts = np.diff(times)
    dt = np.mean(dts)

    if np.std(dts) > 1e-6:
        raise ValueError(
            f"Time differences are not constant! "
            f"Mean dt: {dt:.6f}s, Std: {np.std(dts):.9f}s"
        )

    print(f"Trajectory dt: {dt:.6f}s ({1.0/dt:.1f} Hz)")

    validate_trajectory(trajectory)
    return trajectory, dt


def validate_trajectory(trajectory: List[TrajectoryData]) -> None:
    """Validate trajectory for safety and correctness.

    Checks:
    - Franka joint positions are 7 elements
    - TDCR tendon lengths are 9 elements
    - Joint angles are within limits
    - Waypoint transitions are continuous
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
