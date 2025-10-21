"""Configuration and constants for TDCR path executor."""

import os
import numpy as np

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SCENE_PATH = os.path.join(_PROJECT_ROOT, "data", "ftl_ftdcr_v2_franka_scene.xml")

# Franka Emika Panda joint limits (radians)
FRANKA_JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],
    [-1.7628, 1.7628],
    [-2.8973, 2.8973],
    [-3.0718, -0.0698],
    [-2.8973, 2.8973],
    [-0.0175, 3.7525],
    [-2.8973, 2.8973]
])

TDCR_TENDON_COUNT = 9
DEFAULT_MUJOCO_FPS = 10
BASE_EXECUTION_DT = 0.1  # seconds
DEFAULT_EXECUTION_SPEED = 1.0
CONTINUITY_THRESHOLD = 0.015  # (franky limit: 0.0174 rad)
