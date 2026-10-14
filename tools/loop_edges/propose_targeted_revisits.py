#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Targeted revisit loop-edge proposal for `rux graph discover --loop-edges`.
# Issue #221 / #364 follow-up: owner-directed approach.
#
# WHY THIS EXISTS
# ---------------
# The `export_loop_edges.py` spatial-proposal mode generates distributed
# intra-building loops using SEED POSE proximity.  That works well when drift
# is modest (< half a room diameter), but fails for frames that physically
# revisit the entrance/start area yet appear metres away in the drifted seed
# trajectory.  The owner confirmed that NewOffice frames ~1370, ~2816, ~3429,
# and ~3780 are genuine returns to the entrance/start region (seed frames 0–567)
# even though their seed poses are 1.4–20.9 m from the entrance centre due to
# accumulated drift.
#
# This script takes operator-provided (revisit_frame_index, target_region_frames)
# hints and proposes matched pairs by FRAME-INDEX proximity rather than by
# seed-pose proximity, bypassing the drift blindspot.  The resulting candidate
# pairs are passed to the same MASt3R/XFeat matching pipeline used by
# export_loop_edges.py.
#
# USAGE
# -----
#   ~/loop-edges-work/mast3r/run_mast3r.sh \
#       tools/loop_edges/propose_targeted_revisits.py \
#       /path/to/project.rux \
#       -o /path/to/targeted_edges.json \
#       --matcher mast3r --allow-noncommercial \
#       --revisit 1370 --revisit 2816 --revisit 3429 --revisit 3780 \
#       --target-start 0 --target-end 567 \
#       --window 100 --stride 12 \
#       --min-inliers 40 --device cuda
#
# The output schema is identical to export_loop_edges.py ("reusex.loop_edges.v1")
# so it can be passed directly to `rux graph discover --loop-edges`.

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from matchers import load_matcher  # noqa: E402
from export_loop_edges import (  # noqa: E402
    Frame,
    backproject,
    edge_sigmas,
    ransac_pose,
    read_frames,
    SCHEMA,
)

# --------------------------------------------------------------------------- #
# Targeted proposal                                                            #
# --------------------------------------------------------------------------- #


def propose_targeted_pairs(
    node_ids,
    revisit_frame_indices,
    target_start,
    target_end,
    window,
    stride,
    min_frame_gap,
    rng,
):
    """Generate candidate (i, j) index pairs for targeted revisit matching.

    For each revisit hint `r` ∈ revisit_frame_indices, we pair every frame in
    the window [r-window, r+window] (at `stride`) against every frame in the
    target entrance region [target_start, target_end] (at `stride`).
    Only pairs where the frame-index gap exceeds `min_frame_gap` are kept
    (same guard as the main exporter).

    Parameters
    ----------
    node_ids : list[int]
        Ordered node IDs for all frames loaded from the DB (same order as
        read_frames() returns them).
    revisit_frame_indices : list[int]
        Frame indices (0-based position in the node_ids list) of the revisit
        centres identified by the operator.
    target_start, target_end : int
        Frame-index range [start, end] (inclusive) of the known "entrance" region
        to pair against.
    window : int
        Half-width in frames around each revisit centre.
    stride : int
        Step size to subsample both the revisit window and the target region.
    min_frame_gap : int
        Minimum difference in frame index; guard against near-temporal pairs
        that are trivially close and therefore non-informative as loop edges.
    rng : numpy.random.Generator
        Used for reproducible tie-breaking only (not for pair selection, which
        is deterministic).

    Returns
    -------
    list[tuple[int, int]]
        Unique (i, j) pairs where i < j by frame index (target < revisit),
        with i in the target region and j in the revisit window.
    """
    n = len(node_ids)
    target_frames = list(range(target_start, min(target_end + 1, n), stride))

    all_pairs = set()
    revisit_summary = []
    for r in revisit_frame_indices:
        lo = max(0, r - window)
        hi = min(n - 1, r + window)
        revisit_frames = list(range(lo, hi + 1, stride))
        pairs_for_revisit = []
        for ti in target_frames:
            for ri in revisit_frames:
                # Ensure i < j (target comes first in time)
                i, j = (ti, ri) if ti < ri else (ri, ti)
                gap = abs(node_ids[j] - node_ids[i])  # gap by node_id (≈ time)
                if gap >= min_frame_gap and (i, j) not in all_pairs:
                    all_pairs.add((i, j))
                    pairs_for_revisit.append((i, j))
        revisit_summary.append(
            {
                "revisit_frame_idx": r,
                "revisit_node_id": node_ids[r] if r < n else None,
                "revisit_window": (lo, hi),
                "revisit_window_frames": len(revisit_frames),
                "target_frames": len(target_frames),
                "candidate_pairs": len(pairs_for_revisit),
            }
        )

    # Sort pairs for deterministic processing order
    pairs = sorted(all_pairs)
    return pairs, revisit_summary


# --------------------------------------------------------------------------- #
# Main                                                                         #
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(
        description="Targeted revisit loop-edge proposal for `rux graph "
        "discover --loop-edges` (issue #221 / #364).\n\n"
        "Pairs operator-identified revisit frames against a known target region "
        "by frame-index proximity, bypassing seed-pose drift blindspots."
    )
    ap.add_argument("project", help="path to the .rux project database")
    ap.add_argument("-o", "--output", required=True, help="output edge JSON")
    ap.add_argument(
        "--matcher",
        default="mast3r",
        choices=["orb", "xfeat", "lightglue", "mast3r", "mapanything"],
    )
    ap.add_argument(
        "--revisit",
        action="append",
        type=int,
        dest="revisit_frame_indices",
        metavar="FRAME_INDEX",
        help="frame index of an operator-identified revisit (repeat for multiple); "
        "frame indices are 0-based positions in the ordered frame list from the DB",
    )
    ap.add_argument(
        "--target-start",
        type=int,
        default=0,
        help="start of the target entrance region (frame index, inclusive). Default: 0",
    )
    ap.add_argument(
        "--target-end",
        type=int,
        default=567,
        help="end of the target entrance region (frame index, inclusive). Default: 567",
    )
    ap.add_argument(
        "--window",
        type=int,
        default=100,
        help="half-width in frames around each revisit centre. Default: 100",
    )
    ap.add_argument(
        "--stride",
        type=int,
        default=12,
        help="subsampling stride for both the revisit window and the target region. "
        "Default: 12  (~1 frame per 12 in each region; smaller = more pairs but "
        "slower). Budget guide: stride=12 with 4 revisits ≈ 3200 pairs ≈ 16 min "
        "MASt3R.",
    )
    ap.add_argument(
        "--min-frame-gap",
        type=int,
        default=300,
        help="minimum node_id difference between a revisit and a target frame. "
        "Guard against near-temporal pairs. Default: 300",
    )
    ap.add_argument("--min-inliers", type=int, default=40)
    ap.add_argument("--ransac-thresh", type=float, default=0.10, help="3D-3D (m)")
    ap.add_argument("--ransac-iters", type=int, default=500)
    ap.add_argument("--min-depth", type=float, default=0.3)
    ap.add_argument("--max-depth", type=float, default=5.0)
    ap.add_argument("--max-matches", type=int, default=4000)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda")
    ap.add_argument(
        "--allow-noncommercial",
        action="store_true",
        help="acknowledge that mast3r / mapanything-NC weights are research-only "
        "and their edges must not enter a commercial deliverable",
    )
    ap.add_argument("--weights", default=None)
    ap.add_argument("--variant", default=None)
    ap.add_argument(
        "--max-pairs",
        type=int,
        default=None,
        help="cap on the number of candidate pairs (random subsample). "
        "Use to bound runtime if the targeted pairs are unexpectedly large.",
    )
    args = ap.parse_args()

    if not args.revisit_frame_indices:
        ap.error(
            "at least one --revisit FRAME_INDEX is required\n"
            "example: --revisit 1370 --revisit 2816 --revisit 3429 --revisit 3780"
        )

    rng = np.random.default_rng(args.seed)
    matcher = load_matcher(args)

    t0 = time.time()
    frames = read_frames(args.project, stride=1)  # load all; proposal uses stride
    n = len(frames)
    print(f"[read] {n} frames from {args.project} in {time.time()-t0:.1f}s")
    if n < 2:
        sys.exit("need >= 2 frames")

    node_ids = [f.node_id for f in frames]

    # Build a frame-index lookup: list position -> Frame object (already ordered)
    pairs, revisit_summary = propose_targeted_pairs(
        node_ids=node_ids,
        revisit_frame_indices=args.revisit_frame_indices,
        target_start=args.target_start,
        target_end=args.target_end,
        window=args.window,
        stride=args.stride,
        min_frame_gap=args.min_frame_gap,
        rng=rng,
    )

    print("\n[revisit summary]")
    for s in revisit_summary:
        print(
            f"  revisit frame_idx={s['revisit_frame_idx']} "
            f"(node_id={s['revisit_node_id']}): "
            f"window [{s['revisit_window'][0]}, {s['revisit_window'][1]}] "
            f"({s['revisit_window_frames']} frames, stride={args.stride}), "
            f"target region [{args.target_start}, {args.target_end}] "
            f"({s['target_frames']} frames) -> "
            f"{s['candidate_pairs']} candidate pairs"
        )
    print(f"  TOTAL: {len(pairs)} unique candidate pairs")

    if args.max_pairs and len(pairs) > args.max_pairs:
        idx = rng.choice(len(pairs), args.max_pairs, replace=False)
        pairs = [pairs[k] for k in sorted(idx)]
        print(
            f"[cap] subsampled to {len(pairs)} pairs (--max-pairs={args.max_pairs})"
        )

    # Match all pairs
    edges = []
    t1 = time.time()
    per_revisit_stats = {r: {"pairs": 0, "edges": 0, "inliers": 0} for r in args.revisit_frame_indices}

    for k, (i, j) in enumerate(pairs):
        fi, fj = frames[i], frames[j]
        m = matcher.match(fi, fj, max_matches=args.max_matches)
        if m is None or len(m[0]) < 3:
            continue
        xy_i, xy_j = m
        p_i, vi = backproject(xy_i, fi.depth_m, fi.K, args.min_depth, args.max_depth)
        p_j, vj = backproject(xy_j, fj.depth_m, fj.K, args.min_depth, args.max_depth)
        both = vi & vj
        if int(both.sum()) < 3:
            continue
        # p_i ~= T_ij @ p_j  =>  src=p_j, dst=p_i
        T_ij, inl = ransac_pose(
            p_j[both], p_i[both], args.ransac_thresh, args.ransac_iters, rng
        )
        if T_ij is None:
            continue
        ninl = int(inl.sum())
        if ninl < args.min_inliers:
            continue
        st, sr = edge_sigmas(ninl, args.min_inliers, 0.10, 0.05, 0.04, 0.02)

        # Attribute this edge to the nearest revisit hint
        closest_revisit = min(
            args.revisit_frame_indices,
            key=lambda r: min(abs(i - r), abs(j - r)),
        )
        per_revisit_stats[closest_revisit]["pairs"] += 1
        per_revisit_stats[closest_revisit]["edges"] += 1
        per_revisit_stats[closest_revisit]["inliers"] += ninl

        edges.append(
            {
                "node_i": int(fi.node_id),
                "node_j": int(fj.node_id),
                "T_ij": [float(x) for x in T_ij.reshape(-1)],
                "sigma_rot": sr,
                "sigma_trans": st,
                "inliers": ninl,
                "revisit_hint": closest_revisit,
            }
        )
        if (k + 1) % 200 == 0:
            elapsed = time.time() - t1
            rate = (k + 1) / elapsed
            remaining = (len(pairs) - k - 1) / rate if rate > 0 else float("inf")
            print(
                f"[match] {k+1}/{len(pairs)} pairs, {len(edges)} edges, "
                f"{rate:.1f} pairs/s, ETA {remaining/60:.1f} min"
            )

    total_elapsed = time.time() - t0
    print(f"\n[match summary] {len(edges)} edges from {len(pairs)} pairs ({total_elapsed:.1f}s total)")
    print("\n[per-revisit match quality]")
    for r in args.revisit_frame_indices:
        s = per_revisit_stats[r]
        nid = node_ids[r] if r < n else "?"
        print(
            f"  revisit frame_idx={r} (node_id={nid}): "
            f"{s['edges']} edges accepted, "
            f"{s['inliers']} total inliers"
        )

    out = {
        "schema": SCHEMA,
        "producer": f"{args.matcher}_targeted_revisit",
        "project": str(args.project),
        "convention": "T_ij = pose(i)^-1 * pose(j), optical->world; node ids are "
        "sensor_frames.node_id",
        "params": {
            "matcher": args.matcher,
            "proposal": "targeted_revisit",
            "revisit_frame_indices": args.revisit_frame_indices,
            "target_start": args.target_start,
            "target_end": args.target_end,
            "window": args.window,
            "stride": args.stride,
            "min_frame_gap": args.min_frame_gap,
            "min_inliers": args.min_inliers,
            "ransac_thresh_m": args.ransac_thresh,
            "seed": args.seed,
        },
        "revisit_summary": revisit_summary,
        "per_revisit_match_quality": [
            {
                "revisit_frame_idx": r,
                "revisit_node_id": node_ids[r] if r < n else None,
                "edges_accepted": per_revisit_stats[r]["edges"],
                "total_inliers": per_revisit_stats[r]["inliers"],
            }
            for r in args.revisit_frame_indices
        ],
        "edges": edges,
    }
    Path(args.output).write_text(json.dumps(out, indent=1))
    print(
        f"[done] {len(edges)} edges -> {args.output} "
        f"({total_elapsed:.1f}s total, {len(pairs)} pairs)"
    )


if __name__ == "__main__":
    main()
