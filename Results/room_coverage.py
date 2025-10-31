#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from math import tan, radians

# ------------ CONFIG ------------
room_length = 7.0
room_width  = 6.0
cam_height  = 3.8

hfov_deg    = 90.0
vfov_deg    = 50.0
max_range   = 7.0
auto_aim_center = True

grid_res_x = 120
grid_res_y = 90
slice_heights = [0.0, 1.0, 1.5, 2.0, 2.5, 3.0]  # six z-levels
coverage_threshold = 3  # binary split: >= thr vs < thr

# ------------ CAMERA LAYOUT (8 cams) ------------
L, W = room_length, room_width
z = cam_height
cx, cy = 0.0, 0.0

corners = [(+L/2, +W/2), (+L/2, -W/2), (-L/2, +W/2), (-L/2, -W/2)]
mids    = [(0.0, +W/2), (+L/2, 0.0), (0.0, -W/2), (-L/2, 0.0)]
cam_xy  = np.array(corners + mids, dtype=float)
cam_pos = np.column_stack([cam_xy, np.full(len(cam_xy), z)])  # (8,3)

def camera_axes(pos, center):
    if auto_aim_center:
        f = center - pos
        f = f / (np.linalg.norm(f) + 1e-12)
    else:
        f = np.array([0.0, 0.0, -1.0])
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(f, world_up)) > 0.99:
        world_up = np.array([0.0, 1.0, 0.0])
    r = np.cross(f, world_up); r = r / (np.linalg.norm(r)+1e-12)
    u = np.cross(r, f);        u = u / (np.linalg.norm(u)+1e-12)
    return f, r, u

room_center3 = np.array([cx, cy, 0.0])
F = np.zeros((8,3)); R = np.zeros((8,3)); U = np.zeros((8,3))
for i in range(8):
    F[i], R[i], U[i] = camera_axes(cam_pos[i], room_center3)

# ------------ GRID ------------
xs = np.linspace(-L/2, L/2, grid_res_x)
ys = np.linspace(-W/2, W/2, grid_res_y)
XX, YY = np.meshgrid(xs, ys, indexing='xy')

th = tan(radians(hfov_deg)*0.5)
tv = tan(radians(vfov_deg)*0.5)

# Precompute coverage for each slice, store to reuse in both figures
coverages = []

for zslice in slice_heights:
    P = np.stack([XX, YY, np.full_like(XX, zslice)], axis=-1)  # (Ny,Nx,3)
    coverage = np.zeros_like(XX, dtype=np.int16)

    for idx in range(8):
        C = cam_pos[idx]
        f, r, u = F[idx], R[idx], U[idx]
        V = P - C
        vz = np.tensordot(V, f, axes=([2],[0]))
        vx = np.tensordot(V, r, axes=([2],[0]))
        vy = np.tensordot(V, u, axes=([2],[0]))

        in_front = vz > 0
        in_range = vz <= max_range
        in_h = np.abs(vx) <= (th * vz)
        in_v = np.abs(vy) <= (tv * vz)
        vis = in_front & in_range & in_h & in_v
        coverage += vis.astype(np.int16)

    coverages.append(coverage)

# ------------ FIGURE 1: HEATMAPS ------------
fig1, axes1 = plt.subplots(2, 3, figsize=(12, 7), sharex=True, sharey=True)
axes1 = axes1.ravel()

vmax_global = max(c.max() for c in coverages)
for k, (zslice, cov) in enumerate(zip(slice_heights, coverages)):
    ax = axes1[k]
    im = ax.imshow(cov, origin='lower', extent=(-L/2, L/2, -W/2, W/2),
                   interpolation='nearest', vmin=0, vmax=vmax_global)
    ax.plot([-L/2, L/2, L/2, -L/2, -L/2], [-W/2, -W/2, W/2, W/2, -W/2], color='k', lw=1)
    ax.set_title(f"z = {zslice:.2f} m")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
fig1.colorbar(im, ax=axes1.tolist(), label="# Cameras")
fig1.suptitle("Camera Coverage Heatmaps at Different Heights", fontsize=14)
plt.tight_layout()

# ------------ FIGURE 2: BINARY (>= thr vs < thr) ------------
fig2, axes2 = plt.subplots(2, 3, figsize=(12, 7), sharex=True, sharey=True)
axes2 = axes2.ravel()

for k, (zslice, cov) in enumerate(zip(slice_heights, coverages)):
    ax = axes2[k]
    mask = (cov >= coverage_threshold).astype(np.int8)  # 1 = >= thr, 0 = < thr
    im2 = ax.imshow(mask, origin='lower', extent=(-L/2, L/2, -W/2, W/2),
                    interpolation='nearest', vmin=0, vmax=1)
    ax.plot([-L/2, L/2, L/2, -L/2, -L/2], [-W/2, -W/2, W/2, W/2, -W/2], color='k', lw=1)
    ax.set_title(f"z = {zslice:.2f} m (≥ {coverage_threshold})")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")

# Custom colorbar with 2 ticks: "< thr" and "≥ thr"
cbar = fig2.colorbar(im2, ax=axes2.tolist(), ticks=[0,1])
cbar.ax.set_yticklabels([f"< {coverage_threshold}", f"≥ {coverage_threshold}"])
fig2.suptitle(f"Binary Coverage Masks at Different Heights (threshold = {coverage_threshold})", fontsize=14)
plt.tight_layout()

plt.show()
