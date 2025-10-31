#!/usr/bin/env python3
# plot_merged_points.py
# Usage:
#   python plot_merged_points.py --csv ./Results/merged_mappoints.csv --downsample 1 --save fig.png

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def load_points(csv_path: str):
    # Try with header names x,y,z,(optional)source
    try:
        data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None, encoding=None)
        cols = [c.lower() for c in data.dtype.names]
        assert "x" in cols and "y" in cols and "z" in cols, "CSV must have x,y,z headers"
        x, y, z = data["x"], data["y"], data["z"]
        src = data["source"] if "source" in cols else None
    except Exception as e:
        raise RuntimeError(f"Failed to read {csv_path}: {e}")
    pts = np.stack([x, y, z], axis=1)
    return pts, src

def main():
    ap = argparse.ArgumentParser()
    # ap.add_argument("--csv", required=True, help="Path to merged_mappoints.csv")
    ap.add_argument("--downsample", type=int, default=1, help="Use every Nth point (>=1)")
    ap.add_argument("--point-size", type=float, default=1.0, help="Matplotlib marker size")
    ap.add_argument("--alpha", type=float, default=0.9, help="Point transparency 0..1")
    ap.add_argument("--save", default="", help="If set, save figure to this path instead of showing")
    args = ap.parse_args()

    csv = "merged_mappoints.csv"

    pts, src = load_points(csv)
    if args.downsample > 1:
        pts = pts[::args.downsample]
        if src is not None:
            src = src[::args.downsample]

    print(f"Loaded {pts.shape[0]} points from {os.path.abspath(csv)}")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    if src is None:
        ax.scatter(pts[:,0], pts[:,1], pts[:,2],
                   s=args.point_size, alpha=args.alpha, depthshade=False)
    else:
        # color by source (1 vs 2)
        mask1 = (src == 1)
        mask2 = (src == 2)
        ax.scatter(pts[mask1,0], pts[mask1,1], pts[mask1,2],
                   s=args.point_size, alpha=args.alpha, depthshade=False, label="map1")
        ax.scatter(pts[mask2,0], pts[mask2,1], pts[mask2,2],
                   s=args.point_size, alpha=args.alpha, depthshade=False, label="map2→map1")
        ax.legend(loc="upper left")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Merged MapPoints (map2 transformed into map1 frame)")
    ax.view_init(elev=20, azim=45)
    ax.set_box_aspect([1,1,1])  # equal-ish aspect

    # Tighten axes limits around the data
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = (mins + maxs) / 2.0
    extent = 5
    if extent <= 0: extent = 1.0
    ax.set_xlim(center[0]-extent/2, center[0]+extent/2)
    ax.set_ylim(center[1]-extent/2, center[1]+extent/2)
    ax.set_zlim(center[2]-extent/2, center[2]+extent/2)

    if args.save:
        plt.savefig(args.save, dpi=200, bbox_inches="tight")
        print(f"Saved figure to {os.path.abspath(args.save)}")
    else:
        plt.show()

    # Top-down mapping: X-axis <- Z, Y-axis <- -X
    x = pts[:, 2]
    y = -pts[:, 0]

    fig, ax = plt.subplots()
    ax.scatter(x, y, s=1, alpha=0.8)  # no explicit color/style
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Merged map")

    # Fixed limits [-200, 200] on both axes
    ax.set_xlim(center[0]-extent/2, center[0]+extent/2)
    ax.set_ylim(center[1]-extent/2, center[1]+extent/2)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
