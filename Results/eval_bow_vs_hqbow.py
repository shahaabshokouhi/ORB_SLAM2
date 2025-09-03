# This script extends your evaluation to also produce several plots and CSV summaries.
# It will look for the matches file in /mnt/data and poses.csv in /mnt/data.
# If poses.csv is missing and USE_POSE_GT=True, it will exit gracefully with a message.

import os, math, csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ---------- CONFIG (update if needed) ----------
CSV_MATCHES = "bow_vs_hqbow_matches.csv"   # your export
POSES_CSV   = "poses.csv"                  # dumped alongside
TOPK_USED   = 10                                     
MIN_FRAME_GAP = 50                                   
# Pose GT thresholds:
D_THRESH_M   = 5.0     
YAW_THRESH_D = 15.0    
# If you already have an external GT list, set USE_POSE_GT=False and fill GT_PAIRS_CSV
USE_POSE_GT  = True
GT_PAIRS_CSV = None
# ------------------------------------------------

def parse_list(s):
    if isinstance(s, float) and np.isnan(s): return []
    s = str(s).strip().strip('"')
    if not s: return []
    return [int(x) for x in s.split(',') if x.strip()!='']

def load_matches(csv_path):
    df = pd.read_csv(csv_path)
    preds_bow = {}
    preds_hq  = {}
    for _, row in df.iterrows():
        q = int(row['kf_frame_id'])
        preds_bow[q] = parse_list(row['top_bow_frame_ids'])
        preds_hq[q]  = parse_list(row['top_hqbow_frame_ids'])
    return preds_bow, preds_hq

def load_poses(csv_path):
    df = pd.read_csv(csv_path)
    poses = {}
    for _, r in df.iterrows():
        fid = int(r['frame_id'])
        poses[fid] = (float(r['x']), float(r['y']), float(r['z']), float(r['yaw']))
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
            if ang_diff_deg(yawi, yawj) > yaw_thresh: continue
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
    # Returns macro Precision@K, Recall@K, Hit@K, MRR@K and per-query RR@K
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
    # Uses full list (whatever length in CSV rows)
    max_len = max((len(v) for v in pred_ranked.values()), default=0)
    return metrics_for_prefix(pred_ranked, GT, max_len)

# --- Main workflow ---
if not os.path.exists(CSV_MATCHES):
    print(f"[WARN] Matches file not found: {CSV_MATCHES}")
else:
    preds_bow, preds_hq = load_matches(CSV_MATCHES)
    if USE_POSE_GT:
        if not os.path.exists(POSES_CSV):
            print(f"[WARN] Poses file not found: {POSES_CSV}. Cannot compute GT-based metrics.")
            GT = None
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
            # store per-query RR@K only for final K to analyze improvements
            if k == TOPK_USED:
                # Recompute to also capture mapping from query->RR
                # We'll compute RR per query dictionaries
                rr_bow_per_q = {}
                rr_hq_per_q  = {}
                for q, full in preds_bow.items():
                    if q not in GT or len(GT[q])==0: 
                        continue
                    pred = full[:k]
                    rr = 0.0
                    for rank, j in enumerate(pred, start=1):
                        if j in GT[q]:
                            rr = 1.0 / rank
                            break
                    rr_bow_per_q[q] = rr
                for q, full in preds_hq.items():
                    if q not in GT or len(GT[q])==0: 
                        continue
                    pred = full[:k]
                    rr = 0.0
                    for rank, j in enumerate(pred, start=1):
                        if j in GT[q]:
                            rr = 1.0 / rank
                            break
                    rr_hq_per_q[q] = rr
                rr_bow_dict = rr_bow_per_q
                rr_hq_dict  = rr_hq_per_q

        df_k = pd.DataFrame(rows)
        df_k.to_csv("metrics_at_k.csv", index=False)

        # Overall metrics using the full length of each list (may be <= TOPK_USED)
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
        # grouped bars
        plt.bar(x - width/2, bow_vals, width, label="BoW")
        plt.bar(x + width/2, hq_vals,  width, label="HQ-BoW")
        plt.xticks(x, labels)
        plt.ylabel("Score")
        plt.title("Overall Metrics (macro-avg over queries)")
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
        # Align keys
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
        # For queries without any correct in top-k, treat as rank = TOPK_USED + 1
        ranks_bow = []
        ranks_hq  = []
        for q in common_qs:
            # compute rank for bow
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
            # only include if at least one has a finite rank (<= TOPK_USED)
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

        # Save a per-query counts file (like before)
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

        # Display the summary tables to the user
        try:
            from caas_jupyter_tools import display_dataframe_to_user
            display_dataframe_to_user("Metrics summary", summary)
            display_dataframe_to_user("Metrics@k", df_k)
            display_dataframe_to_user("Per-query counts", per_query_df)
        except Exception as e:
            print("Could not display dataframes interactively:", e)

        print("Saved outputs:")
        print(" - metrics_summary.csv")
        print(" - metrics_at_k.csv")
        print(" - per_query_counts.csv")
        print(" - overall_metrics_bar.png")
        print(" - recall_vs_k.png")
        print(" - precision_vs_k.png")
        print(" - hit_vs_k.png")
        print(" - rr_improvement_hist.png")
        print(" - first_correct_rank_scatter.png")
    else:
        print("Ground truth not available; cannot compute metrics or plots. "
              "Please ensure Poses CSV or GT pairs are provided.")
