"""Configuration and constants for TDCR path executor."""

import os
import numpy as np

# Default scene file path (relative to project root)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SCENE_PATH = os.path.join(_PROJECT_ROOT, "data", "ftl_ftdcr_v2_franka_scene.xml")

# Franka Emika Panda joint limits (radians)
# Source: https://frankaemika.github.io/docs/control_parameters.html
FRANKA_JOINT_LIMITS = np.array([
    [-2.8973, 2.8973],   # Joint 1
    [-1.7628, 1.7628],   # Joint 2
    [-2.8973, 2.8973],   # Joint 3
    [-3.0718, -0.0698],  # Joint 4
    [-2.8973, 2.8973],   # Joint 5
    [-0.0175, 3.7525],   # Joint 6
    [-2.8973, 2.8973]    # Joint 7
])

TDCR_TENDON_COUNT = 9

DEFAULT_MUJOCO_FPS = 10

# Time between waypoints at normal speed (1.0)
BASE_EXECUTION_DT = 0.1  # seconds per waypoint

# Speed multiplier: 1.0 = normal, 0.5 = half speed (slower), 2.0 = double speed (faster)
DEFAULT_EXECUTION_SPEED = 1.0

MAX_JOINT_VELOCITY = 2.0  # rad/s - conservative limit for safety
CONTINUITY_THRESHOLD = 0.015  # rad - max jump between waypoints (franky limit: 0.0174 rad)
