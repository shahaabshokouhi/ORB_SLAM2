#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_points(csv_path: str = "mappoint_descriptors.csv") -> np.ndarray:
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

def plot_topdown_2d(pts: np.ndarray, title: str = "MapPoints (top-down: Z vs -X)"):
    if pts.size == 0:
        print("No points to plot.")
        return

    # Top-down mapping: X-axis <- Z, Y-axis <- -X
    x = pts[:, 2]
    y = -pts[:, 0]

    fig, ax = plt.subplots()
    ax.scatter(x, y, s=1, alpha=0.8)  # no explicit color/style
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(title)

    # Fixed limits [-200, 200] on both axes
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    pts = load_points("mappoint_descriptors.csv")
    print(f"Loaded {pts.shape[0]} unique MapPoints.")
    plot_topdown_2d(pts)

if __name__ == "__main__":
    main()
