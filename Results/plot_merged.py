#!/usr/bin/env python3
# plot_merged_points.py
# Usage examples:
#   python plot_merged_points.py --csv ./Results/merged_mappoints.csv
#   python plot_merged_points.py --csv merged_mappoints.csv --downsample 5 --grid-size 0.2
#   python plot_merged_points.py --csv merged_mappoints.csv --ground-threshold 0.05
#   python plot_merged_points.py --csv merged_mappoints.csv --ground-axis raw_z   # (only if you truly mean raw z is height)
#   python plot_merged_points.py --csv merged_mappoints.csv --save fig_prefix

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt


def load_points(csv_path: str):
    """
    Reads CSV with headers including x,y,z and optional source.
    Returns:
      pts_raw: (N,3) float array in CSV coordinate columns [x,y,z]
      src: optional source array or None
    """
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding=None)
        cols = [c.lower() for c in data.dtype.names]
        if not ("x" in cols and "y" in cols and "z" in cols):
            raise RuntimeError("CSV must have x,y,z headers")

        # Access by exact field names as stored; handle case-insensitive by mapping
        name_map = {c.lower(): c for c in data.dtype.names}
        x = data[name_map["x"]].astype(np.float64)
        y = data[name_map["y"]].astype(np.float64)
        z = data[name_map["z"]].astype(np.float64)

        src = data[name_map["source"]] if "source" in cols else None
    except Exception as e:
        raise RuntimeError(f"Failed to read {csv_path}: {e}")

    pts_raw = np.stack([x, y, z], axis=1)
    return pts_raw, src


def transform_image_to_plot_coords(pts_raw: np.ndarray) -> np.ndarray:
    """
    User-provided mapping (image coords -> plot/real coords):
      z is actually x   => X = z_raw
      x is actually -y  => Y = -x_raw
      y is actually -z  => Z = -y_raw
    So:
      X = z
      Y = -x
      Z = -y
    """
    X = pts_raw[:, 2]
    Y = -pts_raw[:, 0]
    Z = -pts_raw[:, 1]
    return np.stack([X, Y, Z], axis=1)


def filter_ground(pts_raw: np.ndarray, pts_t: np.ndarray, ground_threshold: float, ground_axis: str):
    """
    ground_axis:
      - "transformed_z": remove points with transformed Z (height) < threshold  (DEFAULT; physically sensible)
      - "raw_z": remove points with raw CSV z < threshold (only if user insists)
    """
    if ground_axis == "raw_z":
        mask = pts_raw[:, 2] >= ground_threshold
    else:
        mask = pts_t[:, 2] >= ground_threshold
    return mask


def compute_2d_grid_occupancy_xy(pts_xy: np.ndarray, cell: float):
    """
    Build 2D occupancy grid on XY plane:
      occupied if ANY point falls in that cell (any height already removed before passing in if desired).
    Returns:
      grid (H,W) with 1=free (white), 0=occupied (black)
      origin (xmin, ymin)
      shape (W,H)
    """
    if pts_xy.shape[0] == 0:
        return None, None, None

    xmin, ymin = pts_xy.min(axis=0)
    xmax, ymax = pts_xy.max(axis=0)

    # Add a small padding so edge points don't sit exactly on border
    pad = cell * 2.0
    xmin -= pad
    ymin -= pad
    xmax += pad
    ymax += pad

    W = int(np.ceil((xmax - xmin) / cell))
    H = int(np.ceil((ymax - ymin) / cell))
    W = max(W, 1)
    H = max(H, 1)

    # Start as all free (white=1)
    grid = np.ones((H, W), dtype=np.uint8)

    ix = np.floor((pts_xy[:, 0] - xmin) / cell).astype(int)
    iy = np.floor((pts_xy[:, 1] - ymin) / cell).astype(int)

    valid = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix = ix[valid]
    iy = iy[valid]

    grid[iy, ix] = 0  # occupied => black
    return grid, (xmin, ymin), (W, H)


def compute_voxel_occupancy(pts_xyz: np.ndarray, cell: float):
    """
    Build sparse voxel occupancy indices (ix,iy,iz) using a cubic voxel size = cell.
    Returns:
      vox_idx: (M,3) int array of unique occupied voxels
      origin: (xmin,ymin,zmin)
      dims: (W,H,D) number of voxels along each axis
    """
    if pts_xyz.shape[0] == 0:
        return None, None, None

    mins = pts_xyz.min(axis=0)
    maxs = pts_xyz.max(axis=0)

    pad = cell * 2.0
    mins = mins - pad
    maxs = maxs + pad

    dims = np.ceil((maxs - mins) / cell).astype(int)
    dims = np.maximum(dims, 1)

    idx = np.floor((pts_xyz - mins) / cell).astype(int)

    # Clamp (numerical safety)
    idx[:, 0] = np.clip(idx[:, 0], 0, dims[0] - 1)
    idx[:, 1] = np.clip(idx[:, 1], 0, dims[1] - 1)
    idx[:, 2] = np.clip(idx[:, 2], 0, dims[2] - 1)

    # Unique occupied voxels
    vox = np.unique(idx, axis=0)
    return vox, mins, tuple(dims.tolist())


def voxel_centers(vox_idx: np.ndarray, origin_xyz: np.ndarray, cell: float) -> np.ndarray:
    """
    Convert voxel indices to voxel center coordinates.
    """
    return origin_xyz + (vox_idx.astype(np.float64) + 0.5) * cell


def set_axes_equal_3d(ax, pts_xyz: np.ndarray):
    """
    Make 3D axes have equal scaling based on points extents.
    """
    mins = pts_xyz.min(axis=0)
    maxs = pts_xyz.max(axis=0)
    centers = (mins + maxs) / 2.0
    ranges = (maxs - mins)
    max_range = float(np.max(ranges)) if np.all(np.isfinite(ranges)) else 1.0
    if max_range <= 0:
        max_range = 1.0

    half = max_range / 2.0
    ax.set_xlim(centers[0] - half, centers[0] + half)
    ax.set_ylim(centers[1] - half, centers[1] + half)
    ax.set_zlim(centers[2] - half, centers[2] + half)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="merged_mappoints.csv", help="Path to merged_mappoints.csv")
    ap.add_argument("--downsample", type=int, default=1, help="Use every Nth point (>=1)")
    ap.add_argument("--point-size", type=float, default=1.0, help="Matplotlib marker size")
    ap.add_argument("--alpha", type=float, default=0.9, help="Point transparency 0..1")
    ap.add_argument("--save", default="", help="If set, save figures using this prefix (no extension needed)")
    ap.add_argument("--grid-size", type=float, default=0.2, help="Grid/voxel cell size (meters) (default: 0.2)")
    ap.add_argument("--ground-threshold", type=float, default=0.05, help="Ground cutoff threshold (default: 0.05)")
    ap.add_argument(
        "--ground-axis",
        choices=["transformed_z", "raw_z"],
        default="transformed_z",
        help="Which axis to use for ground removal. Default transformed_z (recommended).",
    )
    args = ap.parse_args()

    csv_path = args.csv
    pts_raw, src = load_points(csv_path)

    if args.downsample > 1:
        pts_raw = pts_raw[:: args.downsample]
        if src is not None:
            src = src[:: args.downsample]

    pts_t = transform_image_to_plot_coords(pts_raw)

    mask = filter_ground(pts_raw, pts_t, args.ground_threshold, args.ground_axis)
    pts_raw_ng = pts_raw[mask]
    pts_t_ng = pts_t[mask]
    if src is not None:
        src_ng = src[mask]
    else:
        src_ng = None

    print(f"Loaded {pts_raw.shape[0]} points from {os.path.abspath(csv_path)}")
    print(f"After ground filter ({args.ground_axis}, threshold={args.ground_threshold}): {pts_t_ng.shape[0]} points remain")
    print(f"Transform used: X=z, Y=-x, Z=-y (applied to ALL plots)")

    # -------------------------
    # Plot 1: 3D scatter (kept)
    # -------------------------
    fig1 = plt.figure(figsize=(8, 7))
    ax1 = fig1.add_subplot(111, projection="3d")

    if src_ng is None:
        ax1.scatter(
            pts_t_ng[:, 0], pts_t_ng[:, 1], pts_t_ng[:, 2],
            s=args.point_size, alpha=args.alpha, depthshade=False
        )
    else:
        mask1 = (src_ng == 1)
        mask2 = (src_ng == 2)
        ax1.scatter(
            pts_t_ng[mask1, 0], pts_t_ng[mask1, 1], pts_t_ng[mask1, 2],
            s=args.point_size, alpha=args.alpha, depthshade=False, label="map1"
        )
        ax1.scatter(
            pts_t_ng[mask2, 0], pts_t_ng[mask2, 1], pts_t_ng[mask2, 2],
            s=args.point_size, alpha=args.alpha, depthshade=False, label="map2→map1"
        )
        ax1.legend(loc="upper left")

    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z (height)")
    ax1.set_title("Merged MapPoints (transformed coords, ground removed)")
    ax1.view_init(elev=20, azim=45)
    if pts_t_ng.shape[0] > 0:
        set_axes_equal_3d(ax1, pts_t_ng)

    plt.tight_layout()

    # --------------------------------
    # Plot 2: Top-down scatter (kept)
    # --------------------------------
    fig2, ax2 = plt.subplots(figsize=(7, 7))
    ax2.scatter(pts_t_ng[:, 0], pts_t_ng[:, 1], s=1, alpha=0.8)
    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_title("Top-down (X vs Y), transformed coords, ground removed")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True)

    if pts_t_ng.shape[0] > 0:
        mins = pts_t_ng[:, :2].min(axis=0)
        maxs = pts_t_ng[:, :2].max(axis=0)
        c = (mins + maxs) / 2.0
        r = np.max(maxs - mins)
        if r <= 0:
            r = 1.0
        pad = r * 0.05
        ax2.set_xlim(c[0] - r / 2 - pad, c[0] + r / 2 + pad)
        ax2.set_ylim(c[1] - r / 2 - pad, c[1] + r / 2 + pad)

    plt.tight_layout()

    # ---------------------------------------------
    # Plot 3: 2D occupancy grid map (NEW)
    # occupied cell => black, free => white
    # ---------------------------------------------
    pts_xy = pts_t_ng[:, :2]
    grid2d, origin2d, dims2d = compute_2d_grid_occupancy_xy(pts_xy, args.grid_size)

    if grid2d is not None:
        fig3, ax3 = plt.subplots(figsize=(7, 7))
        xmin, ymin = origin2d
        W, H = dims2d
        extent = [xmin, xmin + W * args.grid_size, ymin, ymin + H * args.grid_size]

        ax3.imshow(
            grid2d,
            origin="lower",
            extent=extent,
            interpolation="nearest",
            cmap="gray",   # 0=black occupied, 1=white free
            vmin=0, vmax=1
        )
        ax3.set_xlabel("X")
        ax3.set_ylabel("Y")
        ax3.set_title(f"2D Grid Occupancy (cell={args.grid_size}): black=occupied, white=free")
        ax3.set_aspect("equal", adjustable="box")
        ax3.grid(False)
        plt.tight_layout()

        occ_count = int(np.sum(grid2d == 0))
        print(f"2D grid: {H}x{W} cells, occupied={occ_count}, free={H*W - occ_count}")
    else:
        fig3 = None
        print("2D grid: no points available after filtering; skipped grid plot.")

    # ---------------------------------------------
    # Plot 4: 3D voxel occupancy (NEW)
    # ---------------------------------------------
    vox_idx, vox_origin, vox_dims = compute_voxel_occupancy(pts_t_ng, args.grid_size)
    if vox_idx is not None:
        centers = voxel_centers(vox_idx, vox_origin, args.grid_size)

        fig4 = plt.figure(figsize=(8, 7))
        ax4 = fig4.add_subplot(111, projection="3d")
        ax4.scatter(
            centers[:, 0], centers[:, 1], centers[:, 2],
            s=max(args.point_size * 2.0, 1.0), alpha=min(args.alpha, 0.9), depthshade=False
        )
        ax4.set_xlabel("X")
        ax4.set_ylabel("Y")
        ax4.set_zlabel("Z (height)")
        ax4.set_title(f"3D Voxel Occupancy (voxel={args.grid_size})")
        ax4.view_init(elev=20, azim=45)
        set_axes_equal_3d(ax4, centers)
        plt.tight_layout()

        print(f"Voxel grid dims (W,H,D)={vox_dims}, occupied voxels={vox_idx.shape[0]}")
    else:
        fig4 = None
        print("Voxel grid: no points available after filtering; skipped voxel plot.")

    # -------------
    # Save or show
    # -------------
    if args.save:
        prefix = args.save
        out1 = f"{prefix}_3d_points.png"
        out2 = f"{prefix}_topdown_scatter.png"
        plt.figure(fig1.number)
        plt.savefig(out1, dpi=200, bbox_inches="tight")
        plt.figure(fig2.number)
        plt.savefig(out2, dpi=200, bbox_inches="tight")
        print(f"Saved: {os.path.abspath(out1)}")
        print(f"Saved: {os.path.abspath(out2)}")

        if fig3 is not None:
            out3 = f"{prefix}_grid2d.png"
            plt.figure(fig3.number)
            plt.savefig(out3, dpi=200, bbox_inches="tight")
            print(f"Saved: {os.path.abspath(out3)}")

        if fig4 is not None:
            out4 = f"{prefix}_voxel3d.png"
            plt.figure(fig4.number)
            plt.savefig(out4, dpi=200, bbox_inches="tight")
            print(f"Saved: {os.path.abspath(out4)}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
