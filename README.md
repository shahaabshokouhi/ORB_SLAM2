# Collaborative Multi-Robot ORB-SLAM2

**High-quality map-point selection and HQ-BoW place recognition for robust multi-robot map merging.**

[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Built on ORB-SLAM2](https://img.shields.io/badge/based%20on-ORB--SLAM2-orange.svg)](https://github.com/raulmur/ORB_SLAM2)
[![C++11](https://img.shields.io/badge/C%2B%2B-11-blue.svg)]()
[![OpenCV 4.x](https://img.shields.io/badge/OpenCV-4.x-green.svg)]()

> This repository extends the open-source [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) system into a
> **collaborative multi-robot SLAM** framework. Multiple robots run visual SLAM independently and their maps
> are merged into a single, globally consistent map through a place-recognition and geometric-registration
> pipeline that operates on a compact set of **high-quality map points**.

---

## Table of Contents
- [Overview](#overview)
- [Key Contributions](#key-contributions)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Usage](#usage)
  - [1. Record per-robot data](#1-record-per-robot-data)
  - [2. Run single-robot SLAM](#2-run-single-robot-slam)
  - [3. Run multiple robots](#3-run-multiple-robots)
  - [4. Merge maps](#4-merge-maps)
  - [5. Evaluate BoW vs HQ-BoW](#5-evaluate-bow-vs-hq-bow)
- [Configuration](#configuration)
- [Repository Structure](#repository-structure)
- [Results](#results)
- [Citation](#citation)
- [Acknowledgements](#acknowledgements)
- [License](#license)

---

## Overview

Single-robot visual SLAM is limited by the area one platform can observe in bounded time. In multi-robot
missions (search-and-rescue, inspection, warehouse automation), several robots explore overlapping regions
and must fuse their partial maps into one coherent representation — **without** a shared clock, a common
starting frame, or a central tracker.

This framework addresses that problem in three parts:

1. Each robot runs a standard **ORB-SLAM2** RGB-D front-end/back-end and continuously identifies its most
   reliable landmarks (**high-quality map points**).
2. A **place-recognition** stage finds candidate keyframe correspondences *across* robots using a
   Bag-of-Words scheme restricted to high-quality landmarks (**HQ-BoW**).
3. A **geometric registration** stage recovers the rigid transform between maps from 3D–3D landmark
   correspondences and fuses them into a single global map.

The system has been **validated in real-world multi-robot experiments** and is additionally being
benchmarked in simulation (NVIDIA Isaac Sim) against GPU-accelerated stereo-VIO baselines.

---

## Key Contributions

### 1. High-Quality Map-Point Manager (`HighQualityManager`)
A dedicated background module (`include/HQmanager.h`) that continuously scores each map point and flags a
compact subset as **high quality** according to a selectable criterion:

- `observation` — landmarks with many keyframe observations (well-triangulated, persistent), or
- `ba` — landmarks that survive/benefit most from local bundle adjustment.

Only high-quality points are exported and exchanged between robots, which reduces communication and memory
overhead and makes cross-robot matching far more robust to outliers.

### 2. HQ-BoW Place Recognition
Standard DBoW2 builds Bag-of-Words vectors from *all* features. **HQ-BoW** builds them from high-quality
landmarks only, yielding more discriminative descriptors for **inter-robot loop detection** (finding where
two robots' maps overlap). The improvement is quantified against vanilla BoW in
`Results/eval_bow_vs_hqbow.py` (precision, recall, recall@k, and TP/FP/TN/FN confusion against ground truth).

### 3. Rigid Map Merging (`Examples/MapMerger/merg_maps.cc`)
Given two robots' exported maps, the merger:
1. computes (HQ-)BoW vectors per keyframe and proposes candidate cross-robot keyframe pairs,
2. establishes **3D–3D map-point correspondences** for each candidate pair, and
3. estimates the inter-map **rigid transform (SE(3))** with RANSAC (metric RGB-D maps → SE(3), no scale
   ambiguity), then fuses the maps into a common reference frame.

Default registration parameters: descriptor ratio `0.9`, max Hamming distance `60`, RANSAC inlier
threshold `0.07 m`, minimum `20` inliers, `1000` iterations.

---

## System Architecture

```
   Robot 0                         Robot 1                        Robot N
 ┌──────────┐                    ┌──────────┐                   ┌──────────┐
 │ RGB-D in │                    │ RGB-D in │        ...        │ RGB-D in │
 └────┬─────┘                    └────┬─────┘                   └────┬─────┘
      │                               │                              │
 ┌────▼───────────┐            ┌──────▼─────────┐            ┌───────▼────────┐
 │  ORB-SLAM2     │            │  ORB-SLAM2     │            │  ORB-SLAM2     │
 │  front + back  │            │  front + back  │            │  front + back  │
 └────┬───────────┘            └──────┬─────────┘            └───────┬────────┘
      │  HighQualityManager           │  HighQualityManager          │
      │  (scores + exports HQ pts)    │                              │
      └───────────────┬───────────────┴──────────────┬───────────────┘
                      │   per-robot HQ map points     │
                      ▼                               ▼
             ┌───────────────────────────────────────────────┐
             │           Map Merger (merg_maps)              │
             │  (HQ-)BoW candidate KF pairs                  │
             │        → 3D–3D correspondences                │
             │        → RANSAC SE(3) registration            │
             │        → fused global map                     │
             └───────────────────────────────────────────────┘
```

---

## Prerequisites

Tested on Ubuntu 20.04/22.04. This fork is compatible with **OpenCV 4.x**.

| Dependency | Notes |
|-----------|-------|
| C++11 compiler | uses `std::thread` / `std::chrono` |
| [Pangolin](https://github.com/stevenlovegrove/Pangolin) | visualization / GUI |
| [OpenCV](https://opencv.org) | **4.x** (image handling & features) |
| [Eigen3](http://eigen.tuxfamily.org) | ≥ 3.1.0 (required by g2o) |
| DBoW2, g2o | **bundled** in `Thirdparty/` (BSD) |
| [librealsense2](https://github.com/IntelRealSense/librealsense) | for live/recording with Intel RealSense (RGB-D examples) |
| ROS (optional) | for the ROS nodes under `Examples/ROS` |

---

## Building

```bash
git clone https://github.com/shahaabshokouhi/ORB_SLAM2.git
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

`build.sh` compiles the bundled Thirdparty libraries (DBoW2, g2o), uncompresses the ORB vocabulary, and
builds `libORB_SLAM2.so` plus all example executables (including `recorder_rgbd`, `rgbd_offline`,
`rgbd_live`, `multi_cam_rgbd`, and `merg_maps`).

To build the ROS nodes (optional):

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)/Examples/ROS
chmod +x build_ros.sh
./build_ros.sh
```

---

## Usage

The RGB-D examples use the Intel RealSense settings file `Examples/RGB-D/realsense.yaml`
(**adjust the intrinsics to your camera**).

### 1. Record per-robot data
Capture an RGB-D sequence for one robot/station:

```bash
./Examples/recorder_rgbd ./record_data/station_1
# helper: ./record_rgbd.sh
```

### 2. Run single-robot SLAM

**Offline** (on recorded data):

```bash
./Examples/rgbd_offline Vocabulary/ORBvoc.txt Examples/RGB-D/realsense.yaml ./record_data/agent_0
# helper: ./offline_rgbd.sh
```

**Live** (from a connected RealSense camera):

```bash
export REALSENSE_CONFIG=/absolute/path/to/Examples/RGB-D/realsense.yaml
./live_rgbd.sh
# runs: ./Examples/rgbd_live Vocabulary/ORBvoc.txt "$REALSENSE_CONFIG"
```

Each run's `HighQualityManager` exports the robot's high-quality map-point descriptors to a CSV
(e.g. `Results/results_agent_0/mappoint_descriptors.csv`) for later merging.

### 3. Run multiple robots

```bash
./Examples/multi_cam_rgbd Vocabulary/ORBvoc.txt Examples/RGB-D/realsense.yaml
# helper: ./multi_cam_rgbd.sh
```

### 4. Merge maps
With per-robot exports present (`Results/results_agent_0/mappoint_descriptors.csv`,
`Results/results_agent_1/mappoint_descriptors.csv`), run the merger:

```bash
./Examples/merg_maps
```

It loads the two robots' maps, computes (HQ-)BoW candidates, estimates the inter-map SE(3) transform via
RANSAC, and writes the fused map/trajectory for visualization (`Results/plot_merged.py`,
`Results/plot_map_trajectory.py`).

### 5. Evaluate BoW vs HQ-BoW

```bash
cd Results
python3 eval_bow_vs_hqbow.py
```

Reads the exported matches (`bow_vs_hqbow_matches.csv`) and ground truth (KITTI `NN.txt` or `poses.csv`) and
produces precision/recall, recall@k, and confusion metrics plus plots.

---

## Configuration

- **Camera settings:** `Examples/RGB-D/realsense.yaml` — intrinsics (`Camera.fx/fy/cx/cy`), resolution,
  fps, depth scale, and ORB extractor parameters. Update these to match your sensor.
- **`REALSENSE_CONFIG`** (env var): absolute path to the YAML used by the live example.
- **HQ criterion:** `observation` or `ba`, selected when constructing `HighQualityManager`.
- **Merger parameters:** ratio / max-Hamming / RANSAC threshold / inliers / iterations at the top of
  `Examples/MapMerger/merg_maps.cc`.

---

## Repository Structure

```
include/HQmanager.h            High-quality map-point manager (novel)
Examples/MapMerger/merg_maps.cc   Cross-robot map merger: (HQ-)BoW + 3D–3D RANSAC SE(3) (novel)
Examples/multi_cam_rgbd/       Multi-robot RGB-D pipeline (novel)
Examples/rgbd_live, rgbd_offline, recorder_rgbd   RGB-D capture/run tools
Results/eval_bow_vs_hqbow.py   BoW vs HQ-BoW evaluation (novel)
Results/plot_*.py, room_coverage.py   Analysis & plotting utilities
src/, include/                 Core ORB-SLAM2 (with modifications)
Thirdparty/                    DBoW2, g2o (bundled)
Vocabulary/                    ORB vocabulary
GT/                            KITTI ground-truth poses (00–10)
```

---

## Results

<!-- TODO: add quantitative tables and qualitative figures. Suggested content:
     - HQ-BoW vs BoW: precision/recall and recall@k table (from eval_bow_vs_hqbow.py)
     - Merged-map figure (Results/plot_merged.py) with per-robot trajectories overlaid
     - Real-world multi-robot experiment: setup photo + fused map
     - Runtime / memory: full map vs high-quality subset -->

Real-world multi-robot experiments confirm that exchanging only high-quality map points yields accurate
inter-robot registration while substantially reducing the data shared between robots. Quantitative tables
and figures are provided in `Results/`.

---

## Citation

If you use this framework in academic work, please cite the accompanying thesis/paper:

```bibtex
@phdthesis{shokouhi_multirobot_slam,
  title  = {Collaborative Multi-Robot Visual SLAM with High-Quality Map-Point Selection},
  author = {Shokouhi, Shahab},
  school = {TODO: University},
  year   = {TODO}
}
```

<!-- TODO: replace with your final thesis/paper citation. -->

As this work builds on ORB-SLAM2, please also cite the original papers listed in
[Acknowledgements](#acknowledgements).

---

## Acknowledgements

This project is built upon **ORB-SLAM2** by Raúl Mur-Artal, Juan D. Tardós, J. M. M. Montiel, and
Dorian Gálvez-López, and uses the **DBoW2** and **g2o** libraries.

- Mur-Artal, Montiel, Tardós. *ORB-SLAM: A Versatile and Accurate Monocular SLAM System.* IEEE T-RO, 2015.
- Mur-Artal, Tardós. *ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras.*
  IEEE T-RO, 2017.
- Gálvez-López, Tardós. *Bags of Binary Words for Fast Place Recognition in Image Sequences.* IEEE T-RO, 2012.

---

## License

This repository inherits the **GPLv3** license of ORB-SLAM2 (see [`LICENSE.txt`](LICENSE.txt) and
[`License-gpl.txt`](License-gpl.txt)). New contributions in this fork are released under the same license.
For commercial licensing of the original ORB-SLAM2 components, contact the original authors.
