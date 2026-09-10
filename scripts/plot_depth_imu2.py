#!/usr/bin/env python3
"""Plot depth and IMU orientation over time from estimator CSV logs."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def read_numeric_rows(path: Path):
    rows = []
    with path.open(newline="") as csv_file:
        for line_number, row in enumerate(csv.reader(csv_file), start=1):
            if not row or all(not value.strip() for value in row):
                continue
            try:
                rows.append([float(value) for value in row])
            except ValueError as error:
                raise ValueError(f"{path}:{line_number}: non-numeric CSV row") from error
    if not rows:
        raise ValueError(f"{path}: file contains no data")
    return rows


def relative_seconds(timestamps_ns, origin_ns=None):
    if origin_ns is None:
        origin_ns = timestamps_ns[0]
    return [(timestamp - origin_ns) / 1e9 for timestamp in timestamps_ns]


def quaternion_to_rpy_degrees(x, y, z, w):
    # Normalize first so small sensor/serialization errors do not distort angles.
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return math.nan, math.nan, math.nan
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


def main():
    parser = argparse.ArgumentParser(
        description="Graph state_estimator depth and IMU2 CSV recordings."
    )
    parser.add_argument("imu2_csv", type=Path)
    parser.add_argument("depth_csv", type=Path)
    parser.add_argument("--output", "-o", type=Path, help="Save the graph as an image")
    parser.add_argument("--no-show", action="store_true", help="Do not open a graph window")
    args = parser.parse_args()

    imu_rows = read_numeric_rows(args.imu2_csv)
    if any(len(row) != 8 for row in imu_rows):
        raise ValueError(
            "IMU2 rows must be: timestamp_ns,accel_x,accel_y,accel_z,quat_x,quat_y,quat_z,quat_w"
        )

    depth_rows = read_numeric_rows(args.depth_csv)
    depth_widths = {len(row) for row in depth_rows}
    if not depth_widths <= {1, 2} or len(depth_widths) != 1:
        raise ValueError("Depth rows must be depth_m or timestamp_ns,depth_m")

    imu_origin_ns = imu_rows[0][0]
    imu_time = relative_seconds([row[0] for row in imu_rows], imu_origin_ns)
    rpy = [quaternion_to_rpy_degrees(*row[4:8]) for row in imu_rows]

    if next(iter(depth_widths)) == 2:
        depth_time = relative_seconds([row[0] for row in depth_rows], imu_origin_ns)
        depth = [row[1] for row in depth_rows]
        depth_label = "Time from first IMU sample (s)"
    else:
        # Legacy logs have no depth timestamps. Spread samples across the IMU run
        # for visualization only; this is not valid for timing analysis.
        depth = [row[0] for row in depth_rows]
        duration = imu_time[-1] if len(depth) > 1 else 0.0
        depth_time = [index * duration / max(1, len(depth) - 1) for index in range(len(depth))]
        depth_label = "Approximate time (legacy depth log has no timestamps)"
        print("Warning: depth CSV has no timestamps; depth/IMU alignment is approximate.")

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 7))

    axes[0].plot(depth_time, depth, color="tab:blue")
    axes[0].set_ylabel("Depth (m)")
    axes[0].grid(True, alpha=0.3)

    for values, label in zip(zip(*rpy), ("Roll", "Pitch", "Yaw")):
        axes[1].plot(imu_time, values, label=label)
    axes[1].set_ylabel("Orientation (deg)")
    axes[1].set_xlabel(depth_label)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    figure.suptitle(f"{args.imu2_csv.name} and {args.depth_csv.name}")
    figure.tight_layout()

    if args.output:
        figure.savefig(args.output, dpi=160, bbox_inches="tight")
        print(f"Saved graph to {args.output}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
