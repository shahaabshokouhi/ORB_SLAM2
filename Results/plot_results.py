# compare_kitti_vs_orbslam2.py
# Reads:  /mnt/data/00.txt                 (KITTI GT; 3x4 per line)
#         /mnt/data/CameraTrajectory.txt   (KITTI-style; 3x4 per line)
#         /mnt/data/KeyFrameTrajectory.txt (TUM style; t tx ty tz qx qy qz qw)
# Outputs plots + metrics CSV.

import numpy as np
import pandas as pd
import math
from pathlib import Path
import matplotlib.pyplot as plt

DATA_DIR = Path("")
GT_PATH = DATA_DIR/"05.txt"
CAM_PATH = DATA_DIR/"CameraTrajectory.txt"
KF_PATH  = DATA_DIR/"KeyFrameTrajectory.txt"
METRICS_OUT = DATA_DIR/"traj_metrics.csv"

def load_kitti_poses_txt(path):
    mats = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: 
                continue
            parts = [p for p in line.replace('\t',' ').split(' ') if p!='']
            if len(parts) < 12:
                continue
            vals = list(map(float, parts[:12]))
            T = np.eye(4, dtype=float)
            T[:3,:4] = np.array(vals).reshape(3,4)
            mats.append(T)
    return np.stack(mats, axis=0) if mats else np.zeros((0,4,4))

def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)

def load_tum_trajectory(path):
    Ts, ts = [], []
    with open(path, 'r') as f:
        for line in f:
            line=line.strip()
            if not line or line.startswith("#"): continue
            parts = line.split()
            if len(parts) < 8: continue
            t, tx, ty, tz, qx, qy, qz, qw = map(float, parts[:8])
            T = np.eye(4); T[:3,:3]=quat_to_rot(qx,qy,qz,qw); T[:3,3]=[tx,ty,tz]
            Ts.append(T); ts.append(t)
    return (np.stack(Ts, axis=0), np.array(ts)) if Ts else (np.zeros((0,4,4)), [])

def positions(Ts):
    return Ts[:, :3, 3] if Ts.size else np.zeros((0,3))

def umeyama_sim3(X, Y):
    # Y ≈ s R X + t
    n = X.shape[0]
    muX, muY = X.mean(0), Y.mean(0)
    Xc, Yc = X-muX, Y-muY
    Sigma = (Yc.T @ Xc)/n
    U, D, Vt = np.linalg.svd(Sigma)
    S = np.eye(3); 
    if np.linalg.det(U @ Vt) < 0: S[2,2] = -1
    R = U @ S @ Vt
    varX = (Xc**2).sum()/n
    s = np.trace(np.diag(D) @ S)/varX
    t = muY - s*(R @ muX)
    return s, R, t

def apply_sim3(Ts, s, R, t):
    if Ts.size == 0: return Ts
    out = Ts.copy()
    for i in range(Ts.shape[0]):
        out[i,:3,:3] = R @ Ts[i,:3,:3]
        out[i,:3, 3] = s * (R @ Ts[i,:3,3]) + t
    return out

def align_and_metrics(gt_Ts, est_Ts):
    if gt_Ts.size == 0 or est_Ts.size == 0:
        return {"rmse": np.nan, "mean": np.nan, "median": np.nan, "n": 0}, est_Ts
    n = min(len(gt_Ts), len(est_Ts))
    gt, es = positions(gt_Ts[:n]), positions(est_Ts[:n])
    s, R, t = umeyama_sim3(es, gt)
    est_aligned = apply_sim3(est_Ts.copy(), s, R, t)
    esA = positions(est_aligned[:n])
    e = np.linalg.norm(esA - gt, axis=1)
    return {"rmse": float(np.sqrt((e**2).mean())),
            "mean": float(e.mean()),
            "median": float(np.median(e)),
            "n": int(n)}, est_aligned

# ---- Load ----
gt_Ts  = load_kitti_poses_txt(GT_PATH)
cam_Ts = load_kitti_poses_txt(CAM_PATH)
kf_Ts, _ = load_tum_trajectory(KF_PATH)

# For keyframes, compare against uniformly subsampled GT
if len(kf_Ts)>0 and len(gt_Ts)>0:
    idx = np.linspace(0, len(gt_Ts)-1, num=min(len(kf_Ts), len(gt_Ts)), dtype=int)
    gt_for_kf, kf_eval = gt_Ts[idx], kf_Ts[:len(idx)]
else:
    gt_for_kf = np.zeros((0,4,4)); kf_eval = np.zeros((0,4,4))

# ---- Align & metrics ----
m_cam, cam_aligned = align_and_metrics(gt_Ts, cam_Ts)
m_kf,  kf_aligned  = align_and_metrics(gt_for_kf, kf_eval)
pd.DataFrame([{"traj":"CameraTrajectory", **m_cam},
              {"traj":"KeyFrameTrajectory", **m_kf}]).to_csv(METRICS_OUT, index=False)

# ---- Plots ----
# 1) Top-down GT vs CameraTrajectory
plt.figure()
p_gt  = positions(gt_Ts)
p_cam = positions(cam_aligned)
plt.plot(p_gt[:,0], p_gt[:,2], label="GT (top-down)")
plt.plot(p_cam[:,0], p_cam[:,2], label="Cam aligned")
plt.axis('equal'); plt.legend(); plt.title("Top-down: GT vs CameraTrajectory (aligned)")

# 2) Top-down GT (subsampled) vs KeyFrameTrajectory
if len(kf_aligned)>0:
    plt.figure()
    p_gt_k = positions(gt_for_kf)
    p_kf   = positions(kf_aligned)
    plt.plot(p_gt_k[:,0], p_gt_k[:,2], label="GT (subsampled)")
    plt.plot(p_kf[:,0],   p_kf[:,2],   label="KeyFrames aligned")
    plt.axis('equal'); plt.legend(); plt.title("Top-down: GT vs KeyFrameTrajectory (aligned)")

# 3) Per-frame ATE: Cam
if gt_Ts.size and cam_aligned.size:
    n = min(len(gt_Ts), len(cam_aligned))
    e = np.linalg.norm(positions(cam_aligned[:n]) - positions(gt_Ts[:n]), axis=1)
    plt.figure()
    plt.plot(np.arange(n), e)
    plt.xlabel("Frame index"); plt.ylabel("Translational error (m)")
    plt.title(f"ATE (per-frame) CameraTrajectory | RMSE={m_cam['rmse']:.2f} m")

# 4) Per-keyframe ATE: KF
if gt_for_kf.size and kf_aligned.size:
    n2 = min(len(gt_for_kf), len(kf_aligned))
    e2 = np.linalg.norm(positions(kf_aligned[:n2]) - positions(gt_for_kf[:n2]), axis=1)
    plt.figure()
    plt.plot(np.arange(n2), e2)
    plt.xlabel("Keyframe index"); plt.ylabel("Translational error (m)")
    plt.title(f"ATE (per-keyframe) KeyFrameTrajectory | RMSE={m_kf['rmse']:.2f} m")

# 5) CDF of ATE
def cdf(data):
    xs = np.sort(data); ys = np.linspace(0,1,len(xs),endpoint=False)
    return xs, ys
plt.figure()
if gt_Ts.size and cam_aligned.size:
    n=min(len(gt_Ts),len(cam_aligned))
    xs,ys = cdf(np.linalg.norm(positions(cam_aligned[:n])-positions(gt_Ts[:n]),axis=1))
    plt.plot(xs, ys, label="Cam ATE CDF")
if gt_for_kf.size and kf_aligned.size:
    n2=min(len(gt_for_kf),len(kf_aligned))
    xs2,ys2 = cdf(np.linalg.norm(positions(kf_aligned[:n2])-positions(gt_for_kf[:n2]),axis=1))
    plt.plot(xs2, ys2, label="KF ATE CDF")
plt.xlabel("ATE (m)"); plt.ylabel("CDF"); plt.title("Translation error CDF"); plt.legend()

print("Saved metrics CSV at:", METRICS_OUT)
plt.show()

