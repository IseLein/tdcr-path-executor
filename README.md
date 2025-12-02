# TDCR Path Executor

A system for visualizing and executing pre-computed motion plans on a Franka robot with TDCR end-effector.

## Workflow

1. Load trajectory from JSON file
2. Visualize in MuJoCo simulation
3. User confirms execution
4. Execute on real robot (automatically moves to start position if needed)

## Installation

```bash
# Clone and setup
git clone <repo-url>
cd tdcr-path-executor

# Create conda environment
conda create -n tdcr-executor python=3.12
conda activate tdcr-executor

# Install dependencies
pip install -r requirements.txt

# Install franky library
git submodule update --init --recursive
cd lib/csc376/csc376_franky
pip install .
cd ../../..
```

**If using Python venv instead:**
```bash
# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Then continue with pip install commands above
```

## Usage

```bash
# Visualize only (uses default scene)
python execute_path.py trajectory.json --simulate-only

# Full workflow with robot execution
python execute_path.py trajectory.json --robot-ip 192.168.1.107
```

### Example
Turn on Franka Robot with FCI.
```bash
conda activate tdcr-executor
python execute_path.py data/raw_traj3.json --robot-ip 192.168.1.105 [--no-toppra]
```
If segfault, double check tracjectory and FCI, reset enable switch

## Command-line Options

- `--scene PATH` - MuJoCo scene file (default: data/ftdcr_v4_sysid_franka_scene.xml)
- `--simulate-only` - Run visualization without executing on robot
- `--robot-ip IP` - IP address of Franka robot (required for execution)
- `--fps N` - Simulation playback speed (default: 10)
- `--no-loop` - Don't loop trajectory in simulation

## Project Structure

```
tdcr-path-executor/
├── execute_path.py           # Main entry point
├── src/
│   ├── trajectory_loader.py  # Load and validate trajectories
│   ├── mujoco_sim.py         # MuJoCo visualization
│   ├── robot_executor.py     # Robot execution via franky
│   └── config.py             # Configuration constants
├── lib/
│   └── csc376_franky/        # Franky control library
└── data/                     # Trajectory and scene files
```
