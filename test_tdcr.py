#!/usr/bin/env python3
"""Test TDCR communication by looping through trajectory points."""

import argparse
import time
import numpy as np

from lib.dynamixel.multi_tendon_sync_rw import MultiTendonSyncRW
from src.trajectory_loader import load_trajectory
from src.config import (
    TDCR_SERVO_IDS,
    TDCR_SPOOL_RADII_MM,
    TDCR_SERVO_DIRECTIONS,
    TDCR_DEFAULT_SPEED_MM_PER_SEC,
    DEFAULT_SERIAL_PORT,
    SIM_TO_REAL_RATIO,
)


def main():
    parser = argparse.ArgumentParser(description="Test TDCR by looping through trajectory")
    parser.add_argument("trajectory", type=str, help="Path to trajectory JSON file")
    parser.add_argument("--device", type=str, default=DEFAULT_SERIAL_PORT,
                        help=f"Serial port for TDCR (default: {DEFAULT_SERIAL_PORT})")
    parser.add_argument("--dt", type=float, default=0.1,
                        help="Time between waypoints in seconds (default: 0.1)")
    args = parser.parse_args()

    # Load trajectory
    print(f"Loading trajectory from {args.trajectory}")
    trajectory, _ = load_trajectory(args.trajectory)

    # Extract TDCR tendon lengths and convert from absolute (m) to relative (mm)
    initial_lengths_m = trajectory[0].tdcr_tendon_lengths
    tendon_trajectory = np.array([
        (wp.tdcr_tendon_lengths - initial_lengths_m) * 1000.0 * SIM_TO_REAL_RATIO for wp in trajectory
    ])
    print(f"Loaded {len(tendon_trajectory)} waypoints")
    print(f"Tendon range: {tendon_trajectory.min():.2f} to {tendon_trajectory.max():.2f} mm")

    # Initialize TDCR
    print(f"\nInitializing TDCR on {args.device}...")
    tdcr = MultiTendonSyncRW(
        servo_ids=TDCR_SERVO_IDS,
        spool_radii_mm=TDCR_SPOOL_RADII_MM,
        device_name=args.device,
        servo_dir=TDCR_SERVO_DIRECTIONS
    )
    tdcr.set_zero_offsets_to_current_position()
    tdcr.set_tendons_speeds_mm_per_sec(np.ones(9) * TDCR_DEFAULT_SPEED_MM_PER_SEC)
    print("TDCR initialized")

    # Read initial position
    print(f"\nCurrent tendon lengths: {tdcr.get_tendons_mm()}")

    print(f"\nStarting trajectory loop (dt={args.dt}s). Press Ctrl+C to stop.")
    print("=" * 60)

    try:
        loop_count = 0
        while True:
            loop_count += 1

            # Forward pass
            print(f"\nLoop {loop_count}: Forward pass...")
            for i, tendon_lengths in enumerate(tendon_trajectory):
                tdcr.async_set_tendons_mm(tendon_lengths)
                if i % 10 == 0:
                    print(f"  Waypoint {i}/{len(tendon_trajectory)}")
                time.sleep(args.dt)

            # Backward pass
            print(f"Loop {loop_count}: Backward pass...")
            for i, tendon_lengths in enumerate(reversed(tendon_trajectory)):
                tdcr.async_set_tendons_mm(tendon_lengths)
                if i % 10 == 0:
                    print(f"  Waypoint {i}/{len(tendon_trajectory)}")
                time.sleep(args.dt)

    except KeyboardInterrupt:
        print("\n\nStopped by user")
        print(f"Final tendon lengths: {tdcr.get_tendons_mm()}")


if __name__ == "__main__":
    main()
