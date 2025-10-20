# TDCR Path Executor

A modular system for visualizing and executing pre-computed motion plans on a Franka robot with TDCR end-effector. The system provides a safe workflow: simulate in MuJoCo, review the motion, then execute on hardware.

## Workflow

1. **Load** - Read trajectory from JSON file
2. **Simulate** - Visualize motion in MuJoCo
3. **Confirm** - User approves/rejects execution
4. **Execute** - Send commands to real Franka robot

## Project Structure

```
tdcr-path-executor/
├── plan.md                    # This file
├── execute_path.py            # Main entry point
├── src/
│   ├── __init__.py
│   ├── trajectory_loader.py  # Load and validate JSON trajectories
│   ├── mujoco_sim.py         # MuJoCo visualization
│   ├── robot_executor.py     # Real robot execution via franky
│   └── config.py             # Configuration and constants
├── lib/                       # External dependencies (see below)
│   └── csc376_franky/        # Symlink or copy of franky library
├── data/                      # Example trajectories
│   └── example_plan.json
└── requirements.txt
```

## Module Details

### `execute_path.py`
Main entry point script.

**Functions:**
- `main()` - Parse arguments, orchestrate workflow
- `confirm_execution()` - Prompt user for confirmation after simulation

**Flow:**
1. Parse command-line arguments (trajectory path, scene file, robot IP)
2. Load trajectory
3. Launch MuJoCo simulation
4. Prompt for confirmation
5. Execute on robot if approved

### `src/trajectory_loader.py`
Handles loading and validation of trajectory JSON files.

**Functions:**
- `load_trajectory(json_path)` - Load trajectory from file
- `validate_trajectory(trajectory)` - Check format, joint limits, continuity
- `TrajectoryData` - Dataclass representing a trajectory point

**Supports multiple JSON formats:**
- `{"trajectory": [...]}`
- `{"raw_joint_trajectory": [...]}`

Each waypoint contains:
- `franka_qpos`: 7-element list (Franka joint angles)
- `tdcr_tendon_lengths` or `tendon_lengths`: 9-element list

### `src/mujoco_sim.py`
MuJoCo simulation and visualization.

**Functions:**
- `visualize_trajectory(model_path, trajectory, config)` - Launch interactive viewer
- `_control_loop(data, trajectory, config)` - Update simulation state each frame
- `SimulationConfig` - Dataclass for simulation parameters (fps, loop, etc.)

**Features:**
- Real-time visualization
- Adjustable playback speed
- Loop/single-play modes
- User can pause and inspect motion

### `src/robot_executor.py`
Executes trajectory on real Franka robot using franky library.

**Functions:**
- `execute_trajectory(robot_ip, trajectory, config)` - Main execution function
- `connect_robot(robot_ip)` - Initialize franky robot connection
- `send_waypoint(robot, waypoint)` - Send single position command
- `ExecutionConfig` - Dataclass for execution parameters (speed, impedance)

**Safety features:**
- Joint limit checking
- Velocity limits
- Emergency stop on error
- Progress monitoring

### `src/config.py`
Shared configuration and constants.

**Contents:**
- `FRANKA_JOINT_LIMITS` - Min/max for each joint
- `DEFAULT_EXECUTION_SPEED` - Conservative speed for first execution
- `DEFAULT_MUJOCO_FPS` - Simulation playback rate
- `TDCR_TENDON_COUNT` - Number of TDCR tendons (9)

## Handling csc376-franky Library

### Git Submodule (Recommended)
Add the franky library as a git submodule:

```bash
git submodule add <repo-url> lib/csc376_franky
git submodule update --init --recursive
```

In Python, add to sys.path:
```python
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'lib'))
from csc376_franky import Robot  # or whatever the API is
```

## Dependencies

**requirements.txt:**
```
mujoco>=3.0.0
numpy
argparse
```

**External (in lib/):**
- csc376_franky - Franka robot control library

Note: TDCR control will be added later when that code is ready.

## Usage Example

```bash
# Visualize only (no robot execution)
python execute_path.py data/example_plan.json scene.xml --simulate-only

# Full workflow with robot
python execute_path.py data/example_plan.json scene.xml --robot-ip 172.16.0.2

# Custom playback speed
python execute_path.py data/example_plan.json scene.xml --fps 20 --no-loop
```

## Future Enhancements

1. **TDCR Execution** - Integrate real TDCR control when code is ready
2. **Trajectory Editing** - Interactive waypoint adjustment in simulation
3. **Recording** - Save executed trajectories for analysis
4. **Collision Checking** - Validate trajectory against environment model
5. **Multiple Robots** - Support for robot selection
