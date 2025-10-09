# Plot only the estimated camera trajectory (top-down X–Z) from a KITTI-style 3x4 poses file.
# Input:
#   - Est: ./CameraTrajectory.txt   (each line: 12 numbers -> 3x4 matrix, row-major, assumed T_wc)
# Output:
#   - trajectory_topdown_est.png    (saved next to the script)

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

EST_PATH = Path("CameraTrajectory.txt")
SAVE_DIR = Path("")

def load_kitti_poses_txt(path: Path):
    """
    Reads a KITTI-style pose file (each line: 12 numbers -> 3x4 matrix row-major).
    Returns:
        T (N, 3, 4) array, positions as translation column (N, 3), rotations (N, 3, 3).
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

def camera_centers_from_matrix(T_3x4, assume_Twc=True):
    """
    Returns camera centers in world coordinates.
    If assume_Twc=True (KITTI-style world-from-camera), last column is camera center (t).
    Otherwise (assume world-to-camera), compute C = -R^T t.
    """
    R = T_3x4[:, :, :3]
    t = T_3x4[:, :, 3]
    if assume_Twc:
        return t.copy()
    # Treat T as world-to-camera; get camera center in world: C = -R^T t
    return -np.einsum("nij,ni->nj", np.transpose(R, (0, 2, 1)), t)

# Load estimated trajectory and extract camera centers
T_est, t_est, R_est = load_kitti_poses_txt(EST_PATH)
pos_est = camera_centers_from_matrix(T_est, assume_Twc=True)  # shape (N,3)

# Plot: Top-down trajectory (X-Z)
plt.figure(figsize=(8, 6))
plt.plot(pos_est[:, 0], pos_est[:, 2], linewidth=1.5, label="Estimated (X–Z)")
plt.axis("equal")
plt.title("Camera Trajectory (Top-Down)")
plt.xlabel("X (m)")
plt.ylabel("Z (m)")
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend()
plt.tight_layout()
out_path = SAVE_DIR / "trajectory_topdown_est.png"
plt.savefig(out_path, dpi=150)

print(f"Frames plotted: {len(pos_est)}")
print(f"Saved: {out_path.resolve()}")
