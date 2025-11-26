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
CONTINUITY_THRESHOLD = 0.015  # (franky limit: 0.0174 rad)

# Default velocity/acceleration limits for TOPPRA smoothing
DEFAULT_FRANKA_VEL_LIMITS = np.array([1.0] * 7)
DEFAULT_FRANKA_ACC_LIMITS = np.array([0.5] * 7)
DEFAULT_TENDON_VEL_LIMITS = np.array([0.05] * TDCR_TENDON_COUNT)
DEFAULT_TENDON_ACC_LIMITS = np.array([0.5] * TDCR_TENDON_COUNT)
DEFAULT_CONTROL_FREQ = 100