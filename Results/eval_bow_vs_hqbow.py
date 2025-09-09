#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Evaluates BoW vs HQ-BoW using ground truth. Keeps your original plots,
# and (when KITTI GT is used) restricts GT to *only* the keyframes present
# in the matches CSV (plus their predicted IDs). Also writes per-query
# confusion flags (TP/FP/TN/FN) and a labeled copy of the matches CSV.
#
# Inputs (defaults):
#   bow_vs_hqbow_matches.csv   (columns: kf_frame_id, top_bow_frame_ids, top_hqbow_frame_ids)
#   poses.csv                  (fallback GT mode)
#   <SEQ>.txt (e.g., 00.txt)   (KITTI GT mode)
#
# Outputs:
#   metrics_summary.csv
#   metrics_at_k.csv
#   per_query_counts.csv
#   overall_metrics_bar.png
#   recall_vs_k.png
#   precision_vs_k.png
#   hit_vs_k.png
#   rr_improvement_hist.png
#   first_correct_rank_scatter.png
#   confusion_by_query.csv
#   confusion_summary.csv
#   bow_vs_hqbow_matches_labeled.csv
#
# Confusion definitions (per query keyframe q):
#   TP: q has ≥1 true loop partner AND any suggested keyframe is a true loop (within topK, gap, domain)
#   FN: q has ≥1 true loop partner BUT none of the suggestions is a true loop
#   TN: q has no true loop partner AND suggests nothing
#   FP: q has no true loop partner BUT suggests something
#
# True loop definition:
#   distance <= D_THRESH_M, |Δframe| >= MIN_FRAME_GAP, and (optional) yaw diff <= YAW_THRESH_D

import os, math, csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# ---------- CONFIG ----------
CSV_MATCHES     = "bow_vs_hqbow_matches.csv"   # your export
TOPK_USED       = 10
MIN_FRAME_GAP   = 50

# === Choose GT source ===
USE_KITTI_GT    = True                 # set False to use poses.csv / pairs.csv fallback
KITTI_SEQ_ID    = "01"                 # e.g., "00", "07"
KITTI_GT_PATH   = f"{KITTI_SEQ_ID}.txt"

# Fallback GT modes (only used if USE_KITTI_GT=False)
POSES_CSV       = "poses.csv"          # fallback A
USE_POSE_GT     = True                 # True: use poses.csv; False: use GT_PAIRS_CSV
GT_PAIRS_CSV    = None                 # fallback B: CSV with two columns i,j of true pairs

# GT thresholds
D_THRESH_M      = 10.0                  # meters
YAW_THRESH_D    = 15.0                 # degrees; set None to ignore yaw

KITTI_R_IS_WORLD_FROM_CAM = False  # True if file stores T_w_c; set False if it's T_c_w
# -----------------------------------------------

def yaw_from_R(R, assume_world_from_cam=True):
    """
    Return yaw (rad) as the heading of the camera +Z axis projected on the world XZ plane.
    If the file stores T_c_w, we invert R via transpose before using it.
    """
    R_wc = R if assume_world_from_cam else R.T
    fwd_world = R_wc[:, 2]                   # camera +Z in world coords
    # Project onto XZ plane and compute yaw about +Y
    return math.atan2(fwd_world[0], fwd_world[2])

def parse_list(s):
    """Robustly parse '12, 57, 103' → [12,57,103]; ignore empty/nan."""
    if s is None:
        return []
    if isinstance(s, float):
        if np.isnan(s):
            return []
        return [int(round(s))]
    if isinstance(s, (int, np.integer)):
        return [int(s)]
    s = str(s).strip().strip('"')
    if not s:
        return []
    out = []
    for tok in s.split(','):
        t = tok.strip()
        if not t or t.lower() == "nan":
            continue
        try:
            out.append(int(float(t)))
        except:
            pass
    return out

def load_matches(csv_path):
    df = pd.read_csv(csv_path)
    preds_bow = {}
    preds_hq  = {}
    queries   = []
    for _, row in df.iterrows():
        q = int(row['kf_frame_id'])
        queries.append(q)
        preds_bow[q] = parse_list(row.get('top_bow_frame_ids', ""))
        preds_hq[q]  = parse_list(row.get('top_hqbow_frame_ids', ""))
    return df, preds_bow, preds_hq, set(queries)

def load_poses(csv_path):
    df = pd.read_csv(csv_path)
    poses = {}
    for _, r in df.iterrows():
        fid = int(r['frame_id'])
        poses[fid] = (float(r['x']), float(r['y']), float(r['z']), float(r['yaw']))
    return poses

def load_kitti_poses_as_dict(txt_path):
    """Return {frame_id: (x,y,z,yaw)} from KITTI odometry poses file."""
    poses = {}
    if not os.path.exists(txt_path):
        return poses
    with open(txt_path, 'r') as f:
        for fid, line in enumerate(f):
            vals = line.strip().split()
            if len(vals) != 12:
                continue
            m = list(map(float, vals))
            T = np.array(m, dtype=float).reshape(3,4)
            R = T[:,:3]
            t = T[:,3]
            # forward = camera z in world; yaw about Y: atan2(x,z)
            fwd = R[:,2]
            _, pitch, _ = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
            poses[fid] = (float(t[0]), float(t[1]), float(t[2]), float(pitch))
    return poses

def ang_diff_deg(a, b):
    d = (a - b + math.pi) % (2*math.pi) - math.pi
    return abs(math.degrees(d))

def compute_gt_from_poses(poses, min_gap, d_thresh, yaw_thresh):
    ids = sorted(poses.keys())
    GT = {i:set() for i in ids}
    for i in ids:
        xi, yi, zi, yawi = poses[i]
        for j in ids:
            if j==i: continue
            if abs(i - j) < min_gap: continue
            xj, yj, zj, yawj = poses[j]
            d = math.sqrt((xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2)
            if d > d_thresh: continue
            if yaw_thresh is not None and ang_diff_deg(yawi, yawj) > yaw_thresh: continue
            GT[i].add(j)
    return GT

def build_kitti_gt_restricted(poses, allowed_ids, min_gap, d_thresh_m, yaw_thresh_deg):
    """GT only over ORB-SLAM keyframes (queries ∪ predicted ids)."""
    ids = sorted([i for i in allowed_ids if i in poses])
    GT = {i:set() for i in ids}
    for i in ids:
        xi, yi, zi, yawi = poses[i]
        for j in ids:
            if j==i: continue
            if abs(i - j) < min_gap: continue
            xj, yj, zj, yawj = poses[j]
            d = math.sqrt((xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2)
            if d > d_thresh_m: continue
            if yaw_thresh_deg is not None and ang_diff_deg(yawi, yawj) > yaw_thresh_deg: continue
            GT[i].add(j)
    return GT

def compute_gt_from_pairs(pairs_csv):
    GT = {}
    with open(pairs_csv, 'r') as f:
        for i, j in csv.reader(f):
            i, j = int(i), int(j)
            GT.setdefault(i, set()).add(j)
            GT.setdefault(j, set()).add(i)
    return GT

def metrics_for_prefix(pred_ranked, GT, K):
    # macro Precision@K, Recall@K, Hit@K, MRR@K (over queries with non-empty GT)
    precisions, recalls, hits, rr_list = [], [], [], []
    for q, full in pred_ranked.items():
        if q not in GT: 
            continue
        gt = GT[q]
        if len(gt)==0:
            continue
        pred = full[:K]
        pred_set = set(pred)
        tp = len(pred_set & gt)
        if len(pred) > 0:
            precisions.append(tp / len(pred))
        recalls.append(tp / len(gt))
        hits.append(1 if tp>0 else 0)
        rr = 0.0
        for rank, j in enumerate(pred, start=1):
            if j in gt:
                rr = 1.0 / rank
                break
        rr_list.append(rr)
    P = float(np.mean(precisions)) if len(precisions)>0 else float('nan')
    R = float(np.mean(recalls))    if len(recalls)>0  else float('nan')
    HR = float(np.mean(hits))      if len(hits)>0     else float('nan')
    MRR = float(np.mean(rr_list))  if len(rr_list)>0  else float('nan')
    return P, R, HR, MRR, rr_list

def overall_metrics(pred_ranked, GT):
    max_len = max((len(v) for v in pred_ranked.values()), default=0)
    return metrics_for_prefix(pred_ranked, GT, max_len)

def confusion_for_method(method_name, queries, preds, GT, topk, min_gap):
    """TP/FP/TN/FN per query (definitions above)."""
    rows = []
    gt_domain = set(GT.keys())
    for q in sorted(queries):
        if q not in GT:
            # no GT for that KF → skip
            continue
        gt_set = GT[q]
        has_loop = len(gt_set) > 0

        # enforce min gap and GT domain for fairness
        cand = preds.get(q, [])[:topk]
        filt = [j for j in cand if (j in gt_domain and abs(q-j) >= min_gap)]

        predicted_any = len(filt) > 0
        matched = any((j in gt_set) for j in filt)

        tp = int(has_loop and matched)
        fn = int(has_loop and not matched)
        tn = int((not has_loop) and (not predicted_any))
        fp = int((not has_loop) and predicted_any)

        rows.append({
            "method": method_name,
            "frame_id": q,
            "has_loop": int(has_loop),
            "predicted_any": int(predicted_any),
            "matched_any_true_loop": int(matched),
            "TP": tp, "FN": fn, "TN": tn, "FP": fp,
            "gt_partners_count": len(gt_set),
            "suggested_count": len(filt),
            "suggested_ids": ",".join(map(str, filt))
        })
    return rows

def summarize_confusion(df):
    out = []
    for m in sorted(df['method'].unique()):
        sub = df[df['method'] == m]
        TP = int(sub['TP'].sum())
        FP = int(sub['FP'].sum())
        TN = int(sub['TN'].sum())
        FN = int(sub['FN'].sum())
        total = TP + FP + TN + FN
        acc  = (TP+TN)/total if total>0 else float('nan')
        prec = TP/(TP+FP)    if (TP+FP)>0 else float('nan')
        rec  = TP/(TP+FN)    if (TP+FN)>0 else float('nan')
        f1   = (2*prec*rec)/(prec+rec) if (prec>0 and rec>0) else float('nan')
        out.append({"method": m, "TP":TP,"FP":FP,"TN":TN,"FN":FN,"Total":total,
                    "Accuracy":acc,"Precision":prec,"Recall":rec,"F1":f1})
    return pd.DataFrame(out)

def export_gt_loops_csv(GT, allowed_ids, out_path="gt_loops_by_keyframe.csv"):
    """
    Write one row per keyframe in allowed_ids with its GT loop partners
    (also restricted to allowed_ids for consistency with evaluation).
    """
    rows = []
    allowed_ids = set(allowed_ids)
    for i in sorted(allowed_ids):
        partners = sorted([j for j in GT.get(i, set()) if j in allowed_ids and j != i])
        rows.append({
            "frame_id": i,
            "gt_partners_count": len(partners),
            "gt_partner_ids": ",".join(map(str, partners))
        })
    pd.DataFrame(rows).to_csv(out_path, index=False)

# --- Main workflow ---
if not os.path.exists(CSV_MATCHES):
    print(f"[WARN] Matches file not found: {CSV_MATCHES}")
else:
    # Load matches
    df_matches, preds_bow, preds_hq, query_ids = load_matches(CSV_MATCHES)

    # Build the allowed KF universe = queries ∪ all predicted targets
    allowed_ids = set(query_ids)
    for v in preds_bow.values(): allowed_ids.update(v)
    for v in preds_hq.values():  allowed_ids.update(v)

    # Ground truth selection
    GT = None
    if USE_KITTI_GT:
        if not os.path.exists(KITTI_GT_PATH):
            print(f"[WARN] KITTI GT file not found: {KITTI_GT_PATH}")
        else:
            poses = load_kitti_poses_as_dict(KITTI_GT_PATH)
            if len(poses) == 0:
                print(f"[WARN] No poses parsed from KITTI file: {KITTI_GT_PATH}")
            else:
                # Restrict GT to ORB-SLAM keyframes
                GT = build_kitti_gt_restricted(poses, allowed_ids, MIN_FRAME_GAP, D_THRESH_M, YAW_THRESH_D)

    if GT is None:
        if USE_POSE_GT:
            if not os.path.exists(POSES_CSV):
                print(f"[WARN] Poses file not found: {POSES_CSV}. Cannot compute GT-based metrics.")
            else:
                poses = load_poses(POSES_CSV)
                GT = compute_gt_from_poses(poses, MIN_FRAME_GAP, D_THRESH_M, YAW_THRESH_D)
        else:
            if GT_PAIRS_CSV is None or not os.path.exists(GT_PAIRS_CSV):
                print("[WARN] GT_PAIRS_CSV not provided/found. Cannot compute metrics.")
                GT = None
            else:
                GT = compute_gt_from_pairs(GT_PAIRS_CSV)

    if GT is not None:

        export_gt_loops_csv(GT, allowed_ids, "gt_loops_by_keyframe.csv")
        
        # Compute metrics @ k = 1..TOPK_USED
        ks = list(range(1, TOPK_USED+1))
        rows = []
        rr_bow_dict = {}
        rr_hq_dict  = {}
        for k in ks:
            P_bow, R_bow, HR_bow, MRR_bow, RR_bow_list = metrics_for_prefix(preds_bow, GT, k)
            P_hq,  R_hq,  HR_hq,  MRR_hq,  RR_hq_list  = metrics_for_prefix(preds_hq,  GT, k)
            rows.append({
                "k": k,
                "precision_bow": P_bow, "recall_bow": R_bow, "hit_bow": HR_bow, "mrr_bow": MRR_bow,
                "precision_hq":  P_hq,  "recall_hq":  R_hq,  "hit_hq":  HR_hq,  "mrr_hq":  MRR_hq
            })
            if k == TOPK_USED:
                # per-query RR at final K (for plots)
                rr_bow_per_q, rr_hq_per_q = {}, {}
                for q, full in preds_bow.items():
                    if q not in GT or len(GT[q])==0: continue
                    pred = full[:k]
                    rr = 0.0
                    for rank, j in enumerate(pred, start=1):
                        if j in GT[q]: rr = 1.0 / rank; break
                    rr_bow_per_q[q] = rr
                for q, full in preds_hq.items():
                    if q not in GT or len(GT[q])==0: continue
                    pred = full[:k]
                    rr = 0.0
                    for rank, j in enumerate(pred, start=1):
                        if j in GT[q]: rr = 1.0 / rank; break
                    rr_hq_per_q[q] = rr
                rr_bow_dict = rr_bow_per_q
                rr_hq_dict  = rr_hq_per_q

        df_k = pd.DataFrame(rows)
        df_k.to_csv("metrics_at_k.csv", index=False)

        # Overall metrics using full list lengths
        P_bow_all, R_bow_all, HR_bow_all, MRR_bow_all, _ = overall_metrics(preds_bow, GT)
        P_hq_all,  R_hq_all,  HR_hq_all,  MRR_hq_all,  _ = overall_metrics(preds_hq,  GT)
        summary = pd.DataFrame([{
            "precision_bow": P_bow_all, "recall_bow": R_bow_all, "hit_bow": HR_bow_all, "mrr_bow": MRR_bow_all,
            "precision_hq":  P_hq_all,  "recall_hq":  R_hq_all,  "hit_hq":  HR_hq_all,  "mrr_hq":  MRR_hq_all
        }])
        summary.to_csv("metrics_summary.csv", index=False)

        # --- PLOTS ---  (matplotlib only; one chart per figure; no color styling set)

        # 1) Bar chart: Overall metrics (Precision, Recall, Hit, MRR)
        labels = ["Precision", "Recall", "Hit", "MRR"]
        bow_vals = [P_bow_all, R_bow_all, HR_bow_all, MRR_bow_all]
        hq_vals  = [P_hq_all,  R_hq_all,  HR_hq_all,  MRR_hq_all]
        x = np.arange(len(labels))
        width = 0.35

        plt.figure()
        plt.bar(x - width/2, bow_vals, width, label="BoW")
        plt.bar(x + width/2, hq_vals,  width, label="HQ-BoW")
        plt.xticks(x, labels)
        plt.ylabel("Score")
        ttl_seq = KITTI_SEQ_ID if USE_KITTI_GT else "poses.csv"
        plt.title(f"Overall Metrics (GT: {ttl_seq})")
        plt.legend()
        plt.tight_layout()
        plt.savefig("overall_metrics_bar.png")
        plt.close()

        # 2) Recall@k curve
        plt.figure()
        plt.plot(df_k["k"], df_k["recall_bow"], label="BoW")
        plt.plot(df_k["k"], df_k["recall_hq"],  label="HQ-BoW")
        plt.xlabel("k")
        plt.ylabel("Recall@k")
        plt.title("Recall vs k")
        plt.legend()
        plt.tight_layout()
        plt.savefig("recall_vs_k.png")
        plt.close()

        # 3) Precision@k curve
        plt.figure()
        plt.plot(df_k["k"], df_k["precision_bow"], label="BoW")
        plt.plot(df_k["k"], df_k["precision_hq"],  label="HQ-BoW")
        plt.xlabel("k")
        plt.ylabel("Precision@k")
        plt.title("Precision vs k")
        plt.legend()
        plt.tight_layout()
        plt.savefig("precision_vs_k.png")
        plt.close()

        # 4) Hit@k curve
        plt.figure()
        plt.plot(df_k["k"], df_k["hit_bow"], label="BoW")
        plt.plot(df_k["k"], df_k["hit_hq"],  label="HQ-BoW")
        plt.xlabel("k")
        plt.ylabel("Hit@k")
        plt.title("Hit rate vs k")
        plt.legend()
        plt.tight_layout()
        plt.savefig("hit_vs_k.png")
        plt.close()

        # 5) Histogram: RR improvement at k=TOPK_USED  (HQ - BoW)
        common_qs = sorted(set(rr_bow_dict.keys()) | set(rr_hq_dict.keys()))
        diff = []
        for q in common_qs:
            rb = rr_bow_dict.get(q, 0.0)
            rh = rr_hq_dict.get(q, 0.0)
            diff.append(rh - rb)
        if len(diff) > 0:
            plt.figure()
            plt.hist(diff, bins=30)
            plt.xlabel("RR(HQ) - RR(BoW) at k={}".format(TOPK_USED))
            plt.ylabel("Count")
            plt.title("Distribution of Reciprocal Rank Improvement")
            plt.tight_layout()
            plt.savefig("rr_improvement_hist.png")
            plt.close()

        # 6) Scatter: rank of first correct (BoW) vs (HQ) at k=TOPK_USED
        ranks_bow = []
        ranks_hq  = []
        for q in common_qs:
            pred_b = preds_bow.get(q, [])[:TOPK_USED]
            pred_h = preds_hq.get(q, [])[:TOPK_USED]
            gt = GT.get(q, set())

            rb = TOPK_USED + 1
            for rank, j in enumerate(pred_b, start=1):
                if j in gt:
                    rb = rank
                    break
            rh = TOPK_USED + 1
            for rank, j in enumerate(pred_h, start=1):
                if j in gt:
                    rh = rank
                    break
            if rb <= TOPK_USED or rh <= TOPK_USED:
                ranks_bow.append(rb)
                ranks_hq.append(rh)

        if len(ranks_bow) > 0:
            plt.figure()
            plt.scatter(ranks_bow, ranks_hq, alpha=0.6)
            plt.xlabel("First-correct rank (BoW)")
            plt.ylabel("First-correct rank (HQ-BoW)")
            plt.title("Rank of First Correct: BoW vs HQ (lower is better)")
            plt.tight_layout()
            plt.savefig("first_correct_rank_scatter.png")
            plt.close()

        # Per-query counts table (like before)
        rows = []
        all_queries = sorted(set(list(preds_bow.keys()) + list(preds_hq.keys())))
        for q in all_queries:
            gt = GT.get(q, set())
            pb = preds_bow.get(q, [])
            ph = preds_hq.get(q, [])
            tp_b = len(set(pb) & gt); tp_h = len(set(ph) & gt)
            rows.append({
                "frame_id": q,
                "gt_count": len(gt),
                "pred_bow": len(pb), "tp_bow": tp_b,
                "pred_hq": len(ph),  "tp_hq": tp_h
            })
        per_query_df = pd.DataFrame(rows)
        per_query_df.to_csv("per_query_counts.csv", index=False)

        # ---- Confusion tables (TP/FP/TN/FN per query) ----
        conf_bow = confusion_for_method("BoW", all_queries, preds_bow, GT, TOPK_USED, MIN_FRAME_GAP)
        conf_hq  = confusion_for_method("HQ-BoW", all_queries, preds_hq,  GT, TOPK_USED, MIN_FRAME_GAP)
        conf_df = pd.DataFrame(conf_bow + conf_hq)
        conf_df.to_csv("confusion_by_query.csv", index=False)

        conf_sum = summarize_confusion(conf_df)
        conf_sum.to_csv("confusion_summary.csv", index=False)

        # ---- Labeled copy of matches CSV with confusion flags ----
        # Merge BoW/HQ flags side-by-side per query
        bow_flags = conf_df[conf_df["method"]=="BoW"].drop(columns=["method"]).rename(
            columns={
                "predicted_any":"bow_predicted_any",
                "matched_any_true_loop":"bow_matched_any_true_loop",
                "TP":"bow_TP","FP":"bow_FP","TN":"bow_TN","FN":"bow_FN",
                "gt_partners_count":"bow_gt_partners_count",
                "suggested_count":"bow_suggested_count",
                "suggested_ids":"bow_suggested_ids"
            }
        )
        hq_flags = conf_df[conf_df["method"]=="HQ-BoW"].drop(columns=["method"]).rename(
            columns={
                "predicted_any":"hq_predicted_any",
                "matched_any_true_loop":"hq_matched_any_true_loop",
                "TP":"hq_TP","FP":"hq_FP","TN":"hq_TN","FN":"hq_FN",
                "gt_partners_count":"hq_gt_partners_count",
                "suggested_count":"hq_suggested_count",
                "suggested_ids":"hq_suggested_ids"
            }
        )
        labeled = df_matches.merge(bow_flags, how="left", left_on="kf_frame_id", right_on="frame_id")
        labeled = labeled.drop(columns=["frame_id"])
        labeled = labeled.merge(hq_flags, how="left", left_on="kf_frame_id", right_on="frame_id")
        labeled = labeled.drop(columns=["frame_id"])
        # unify has_loop (they're the same in both; keep BoW's)
        if "has_loop" in labeled.columns:
            labeled = labeled.rename(columns={"has_loop":"has_loop_bow"})
        if "has_loop_bow" in labeled.columns and "has_loop" in hq_flags.columns:
            labeled["has_loop_hq"] = labeled["has_loop_bow"]  # identical by construction
        labeled.to_csv("bow_vs_hqbow_matches_labeled.csv", index=False)

        print("Saved outputs:")
        for fn in [
            "metrics_summary.csv",
            "metrics_at_k.csv",
            "per_query_counts.csv",
            "confusion_by_query.csv",
            "confusion_summary.csv",
            "bow_vs_hqbow_matches_labeled.csv",
            "overall_metrics_bar.png",
            "recall_vs_k.png",
            "precision_vs_k.png",
            "hit_vs_k.png",
            "rr_improvement_hist.png",
            "first_correct_rank_scatter.png",
        ]:
            if os.path.exists(fn):
                print(" -", fn)

    else:
        print("Ground truth not available; cannot compute metrics or plots. "
              "Enable USE_KITTI_GT with a valid KITTI file (e.g., 00.txt), "
              "or provide poses.csv / pairs CSV.")
