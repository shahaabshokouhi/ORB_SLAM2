#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot camera trajectory + map points together:
 - Top-down (Z vs -X) overlay
 - 3D view

Inputs (expected in current dir):
  - CameraTrajectory.txt   (KITTI-style: each line has 12 floats -> 3x4 T_wc)
  - mappoint_descriptors.csv  (columns: mp_id, mp_x, mp_y, mp_z)

Outputs:
  - trajectory_map_topdown.png
  - trajectory_map_3d.png
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D)

EST_PATH = Path("CameraTrajectory.txt")
MAP_CSV  = Path("mappoint_descriptors.csv")
SAVE_DIR = Path("")

# ------------------------ I/O ------------------------

def load_kitti_poses_txt(path: Path):
    """
    Reads a KITTI-style pose file (each line: 12 numbers -> 3x4 matrix row-major).
    Returns:
        T (N, 3, 4), t (N, 3), R (N, 3, 3)
    Assumes T is T_wc (world-from-camera), i.e., last column is camera center in world.
    """
    mats = []
    with open(path, "r") as f:
        for line in f:
            vals = line.strip().split()
            if len(vals) < 12:
                continue
            m = np.array([float(x) for x in vals[:12]], dtype=np.float64).reshape(3, 4)
            mats.append(m)
    if not mats:
        raise ValueError(f"No valid lines found in {path}")
    T = np.stack(mats, axis=0)
    R = T[:, :, :3]
    t = T[:, :, 3]
    return T, t, R

def load_points(csv_path: Path) -> np.ndarray:
    """
    Load unique MapPoints (mp_id), aggregating duplicates by median.
    Returns array of shape (M, 3) with columns [x, y, z].
    """
    df = pd.read_csv(csv_path)
    required = {"mp_id", "mp_x", "mp_y", "mp_z"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Missing columns in CSV: {missing}")

    agg = (
        df[["mp_id","mp_x","mp_y","mp_z"]]
        .dropna(subset=["mp_x","mp_y","mp_z"])
        .groupby("mp_id", as_index=False)[["mp_x","mp_y","mp_z"]]
        .median()
    )
    return agg[["mp_x","mp_y","mp_z"]].to_numpy(dtype=float)

# --------------------- Plot helpers -------------------

def set_axes_equal_3d(ax):
    """
    Make 3D axes have equal scale so spheres/angles look right.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    x_mid = 0.5 * (x_limits[0] + x_limits[1])
    y_mid = 0.5 * (y_limits[0] + y_limits[1])
    z_mid = 0.5 * (z_limits[0] + z_limits[1])

    ax.set_xlim3d([x_mid - plot_radius, x_mid + plot_radius])
    ax.set_ylim3d([y_mid - plot_radius, y_mid + plot_radius])
    ax.set_zlim3d([z_mid - plot_radius, z_mid + plot_radius])

def topdown_proj_Z_negX(arr_xyz: np.ndarray):
    """
    Project 3D points/trajectory to top-down plane used in your map plot:
      X_plot <- Z
      Y_plot <- -X
    Returns (N, 2)
    """
    return np.column_stack((arr_xyz[:, 2], -arr_xyz[:, 0]))

# --------------------- Main plots ---------------------

def plot_topdown_overlay(map_pts_xyz: np.ndarray,
                         traj_xyz: np.ndarray,
                         limits=(-400, 400),
                         save_path: Path = SAVE_DIR / "trajectory_map_topdown.png"):
    """
    Top-down overlay (Z vs -X) with MapPoints as scatter and trajectory as a line.
    """
    mp_2d = topdown_proj_Z_negX(map_pts_xyz)
    tr_2d = topdown_proj_Z_negX(traj_xyz)

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.scatter(mp_2d[:, 0], mp_2d[:, 1], s=1, alpha=0.7, label="MapPoints")
    ax.plot(tr_2d[:, 0], tr_2d[:, 1], linewidth=1.5, label="Trajectory")

    ax.set_xlabel("Z (→)   [projected]")
    ax.set_ylabel("−X (↑)  [projected]")
    ax.set_title("Top-Down Overlay: Trajectory + Map")
    if limits is not None:
        lo, hi = limits
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    plt.tight_layout()
    plt.show()
    print(f"Saved top-down overlay: {save_path.resolve()}")

def plot_3d_overlay(map_pts_xyz: np.ndarray,
                    traj_xyz: np.ndarray,
                    save_path: Path = SAVE_DIR / "trajectory_map_3d.png"):
    """
    3D overlay with MapPoints (scatter) and trajectory (line).
    """
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(map_pts_xyz[:, 0], map_pts_xyz[:, 1], map_pts_xyz[:, 2], s=1, alpha=0.4, label="MapPoints")
    ax.plot(traj_xyz[:, 0], traj_xyz[:, 1], traj_xyz[:, 2], linewidth=1.5, label="Trajectory")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Overlay: Trajectory + Map")
    ax.legend(loc="upper left")
    set_axes_equal_3d(ax)
    plt.tight_layout()
    plt.show()
    print(f"Saved 3D overlay: {save_path.resolve()}")

# ------------------------ Run -------------------------

def main():
    # Load data
    T_est, t_est, _ = load_kitti_poses_txt(EST_PATH)
    traj_xyz = t_est.copy()  # (N,3) camera centers in world coords (KITTI T_wc)
    map_pts = load_points(MAP_CSV)
    print(f"Loaded trajectory frames: {len(traj_xyz)}")
    print(f"Loaded unique MapPoints: {map_pts.shape[0]}")

    # Plots
    plot_topdown_overlay(map_pts, traj_xyz, limits=(-400, 400))
    plot_3d_overlay(map_pts, traj_xyz)

if __name__ == "__main__":
    main()
