"""MuJoCo simulation and visualization for robot trajectories."""

import time
import threading
from dataclasses import dataclass
from typing import List

import mujoco
import mujoco.viewer

from .trajectory_loader import TrajectoryData
from .config import DEFAULT_MUJOCO_FPS


@dataclass
class SimulationConfig:
    """Configuration for MuJoCo simulation playback."""
    fps: int = DEFAULT_MUJOCO_FPS
    loop: bool = True
    speed_scale: float = 1.0


def visualize_trajectory(
    model_path: str,
    trajectory: List[TrajectoryData],
    config: SimulationConfig = None
) -> None:
    """Launch interactive MuJoCo viewer with trajectory playback.

    Args:
        model_path: Path to MuJoCo XML scene file
        trajectory: List of trajectory waypoints
        config: Simulation configuration (uses defaults if None)
    """
    if config is None:
        config = SimulationConfig()

    print(f"Loading MuJoCo scene: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("\nTrajectory info:")
    print(f"\t- Total waypoints: {len(trajectory)}")
    print(f"\t- Playback speed: {config.fps} waypoints/sec")
    print(f"\t- Speed scale: {config.speed_scale}x")
    print(f"\t- Loop: {config.loop}")
    print("\nControls:")
    print("\t- Close window to quit")
    print("\t- Press SPACE in viewer to pause/play")

    running = [True]

    control_thread = threading.Thread(
        target=_control_loop,
        args=(data, trajectory, config, running),
        daemon=True
    )
    control_thread.start()

    try:
        mujoco.viewer.launch(model, data)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        running[0] = False
        control_thread.join(timeout=1.0)


def _control_loop(
    data,
    trajectory: List[TrajectoryData],
    config: SimulationConfig,
    running: list
) -> None:
    """Update simulation state each frame.

    This function runs in a separate thread and updates the MuJoCo data
    structure according to the trajectory waypoints.

    Args:
        data: MuJoCo data structure
        trajectory: List of trajectory waypoints
        config: Simulation configuration
        running: List with single boolean element for thread control
    """
    step_idx = 0
    effective_fps = config.fps * config.speed_scale

    while running[0]:
        if step_idx >= len(trajectory):
            if config.loop:
                step_idx = 0
                print("\n[Loop] Restarting trajectory...")
            else:
                print("\n[Done] Trajectory complete")
                break

        waypoint = trajectory[step_idx]

        for i in range(len(waypoint.franka_qpos)):
            data.ctrl[i] = waypoint.franka_qpos[i]

        for i, length in enumerate(waypoint.tdcr_tendon_lengths):
            data.ctrl[7 + i] = length

        print(f"Waypoint {step_idx + 1}/{len(trajectory)}", end='\r')

        if step_idx == 0:
            print("Waiting for system to get to start position")
            time.sleep(2.0)

        step_idx += 1
        time.sleep(1.0 / effective_fps)
