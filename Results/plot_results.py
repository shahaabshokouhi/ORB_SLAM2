# This script loads KITTI ground-truth poses and an ORB-SLAM2 estimated trajectory,
# overlays the trajectories (top-down X–Z), and plots per-frame and cumulative position error.
#
# Files used (already uploaded in this chat sandbox):
#   - GT:  /mnt/data/05.txt
#   - Est: /mnt/data/CameraTrajectory.txt
#
# The code is robust to differing lengths and will optionally align the estimated
# trajectory to the ground truth with a rigid SE(3) (no scale) Umeyama alignment.
#
# It will also save the plots next to the inputs.
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

GT_PATH = Path("00.txt")
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
    If assume_Twc=True (KITTI style), then the last column is the camera center (t).
    Otherwise, compute C = -R^T t.
    """
    R = T_3x4[:, :, :3]
    t = T_3x4[:, :, 3]
    if assume_Twc:
        return t.copy()
    # world-from-camera unknown: treat T as world-to-camera; get camera center in world
    # C = -R^T t for each frame
    CT = -np.einsum("nij,ni->nj", np.transpose(R, (0, 2, 1)), t)
    return CT

def umeyama_rigid_alignment(A, B):
    """
    Computes rigid alignment (R, t) that maps A -> B (no scale).
    A, B: (N,3). Returns R(3,3), t(3,).
    """
    if A.shape[0] != B.shape[0]:
        n = min(A.shape[0], B.shape[0])
        A = A[:n]
        B = B[:n]
    mu_A = A.mean(axis=0)
    mu_B = B.mean(axis=0)
    A0 = A - mu_A
    B0 = B - mu_B
    H = A0.T @ B0
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = mu_B - R @ mu_A
    return R, t

def compute_errors(gt_xyz, est_xyz, align=True):
    """
    Optionally align 'est' to 'gt' with a rigid transform; then compute per-frame error and cumulative error.
    Returns: est_aligned (N,3), err (N,), err_cum (N,), rmse (float)
    """
    n = min(len(gt_xyz), len(est_xyz))
    gt = gt_xyz[:n]
    est = est_xyz[:n]
    if align:
        R, t = umeyama_rigid_alignment(est, gt)
        est_aligned = (est @ R.T) + t
    else:
        est_aligned = est
    diffs = est_aligned - gt
    err = np.linalg.norm(diffs, axis=1)
    err_cum = np.cumsum(err)
    rmse = np.sqrt(np.mean(err**2))
    return est_aligned, err, err_cum, rmse

# Load files
T_gt, t_gt, R_gt = load_kitti_poses_txt(GT_PATH)
T_est, t_est, R_est = load_kitti_poses_txt(EST_PATH)

# For KITTI 'poses' the last column is already camera center in world coords.
pos_gt = t_gt  # (N,3)
pos_est = t_est

# Compute errors with alignment
est_aligned, err, err_cum, rmse = compute_errors(pos_gt, pos_est, align=True)

# --- Plot 1: Top-down trajectory (X-Z) overlay ---
plt.figure(figsize=(8, 6))
plt.plot(pos_gt[:, 0], pos_gt[:, 2], label="GT (X-Z)")
plt.plot(est_aligned[:, 0], est_aligned[:, 2], label="ORB-SLAM2 (aligned) (X-Z)")
plt.axis("equal")
plt.title("KITTI Trajectory (Top-Down)")
plt.xlabel("X (m)")
plt.ylabel("Z (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot 2: Per-frame position error ---
plt.figure(figsize=(8, 4))
plt.plot(err, label="Per-frame position error (m)")
plt.title(f"Per-frame Error (RMSE = {rmse:.3f} m)")
plt.xlabel("Frame index")
plt.ylabel("Euclidean position error (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- Plot 3: Cumulative position error ---
plt.figure(figsize=(8, 4))
plt.plot(err_cum, label="Cumulative position error (m)")
plt.title("Cumulative Position Error")
plt.xlabel("Frame index")
plt.ylabel("Accumulated error (m)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

print(f"Frames used: {min(len(pos_gt), len(pos_est))}")
print(f"ATE RMSE: {rmse:.3f} m")
print(f"Saved plots to: {SAVE_DIR}")

