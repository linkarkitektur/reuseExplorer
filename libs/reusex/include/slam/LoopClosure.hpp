// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// P2 of issue #225: wide-baseline loop-closure edges for the plane-landmark
// pose graph.
//
// The plane-landmark back-end (PlaneGraphOptimizer) fixes *local* global
// consistency where frames co-observe the same walls, but it cannot constrain
// two temporally distant views of the same place that share no plane landmark
// chain — the "basin problem" from docs/research/registration-improvements.md.
// This module supplies the missing wide-baseline constraints: it proposes
// loop-candidate frame pairs, matches them with image features, lifts the
// matches to metric 3D using the stored per-frame depth, and estimates a robust
// metric relative pose (RANSAC 3D-3D). Each accepted pair becomes a
// gtsam::BetweenFactor<Pose3> loop edge fed into the SAME GncOptimizer graph,
// so a wrong edge is down-weighted rather than corrupting the solution.
//
// Front-end design (research doc §Q2, commercial-safe path): the matcher is
// abstracted behind FrameMatcher so the default OpenCV ORB detector+matcher
// (BSD, no model weights, verifiable offline) can be swapped for a learned
// matcher (EfficientLoFTR / LightGlue+ALIKED via the existing TensorRT backend)
// without touching the graph wiring. The bundled superpoint.pt is
// non-commercial and deliberately NOT used here.
//
// This header carries no GTSAM include (STANDARDS.md §2); the factor lives in
// PlaneGraphOptimizer.cpp.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <random>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::geometry {

/// A metric relative-pose constraint between two frames, ready to become a
/// gtsam::BetweenFactor<Pose3>. Indices are positions in the frame list the
/// optimizer works on (NOT database node ids). The measurement is the relative
/// pose T such that pose(j) = pose(i) * T, i.e. T = pose(i)^-1 * pose(j),
/// expressed in the same optical->world convention the graph uses.
struct LoopEdge {
  int i = -1; ///< first frame (index into the frames vector)
  int j = -1; ///< second frame (index into the frames vector)
  /// relative pose pose(i)^-1 * pose(j)
  Eigen::Matrix4d T_ij = Eigen::Matrix4d::Identity();
  double sigma_rot = 0.05;   ///< rotational std of the edge (rad)
  double sigma_trans = 0.05; ///< translational std of the edge (m)
  int inliers = 0;           ///< RANSAC inlier correspondences supporting it
};

/// How loop-candidate frame pairs are proposed before geometric verification.
enum class LoopProposal {
  /// Pick automatically (default): `exhaustive` when the temporally-distant
  /// pair count is within `exhaustive_budget` (small scans — the reliable
  /// choice, since the appearance shortlist can miss the true loop under indoor
  /// perceptual aliasing), otherwise `appearance`.
  automatic,
  /// Spatial shortlist from the SEED poses (camera-centre proximity + view
  /// agreement). Cheap, but structurally BLIND to loops the drift has pulled
  /// apart — a start/end revisit under trajectory drift is never proposed
  /// because the drifted poses place those frames far apart. Use only when the
  /// seed poses are already globally consistent.
  spatial,
  /// Appearance shortlist from a pose-INDEPENDENT bag-of-words over the frames'
  /// own ORB descriptors. Finds revisits by how the images LOOK, not where the
  /// (possibly drifted) poses say the camera was — this is what detects the
  /// start/end loop that fixes accumulated drift. What `automatic` falls back
  /// to once the exhaustive pair count exceeds `exhaustive_budget`.
  appearance,
  /// Every temporally distant pair (i, j) with j - i >= min_frame_gap. Pose-
  /// and appearance-independent; the ground-truth proposer for small scans
  /// where the O(N^2) matcher cost is affordable (a few hundred frames).
  exhaustive
};

/// Parameters for wide-baseline loop-edge detection.
struct LoopClosureOptions {
  /// Off by default here; on by default in `rux graph discover`.
  bool enable = false;

  /// Candidate-proposal strategy (see LoopProposal). Default `automatic`.
  LoopProposal proposal = LoopProposal::automatic;
  /// `automatic` uses exhaustive proposal when the temporally-distant pair
  /// count is at most this; above it, appearance (BoW) retrieval. ~80k pairs is
  /// a few minutes of matching.
  int exhaustive_budget = 80000;

  // --- Candidate proposal (common) -----------------------------------------
  /// Skip pairs closer than this in frame index (those are handled by the
  /// temporal odometry factors, not loop edges).
  int min_frame_gap = 50;
  /// Keep at most this many candidates per frame (nearest / most-similar
  /// first). Caps the O(N^2) proposal down to ~N*K matcher calls.
  int max_candidates_per_frame = 5;

  // --- Spatial proposal only -----------------------------------------------
  /// Propose a pair only if the seed camera centres are within this distance.
  float max_candidate_distance = 2.5f; ///< m
  /// …and their optical viewing directions agree within this angle.
  float max_view_angle = 45.0f; ///< deg

  // --- Appearance proposal only (bag-of-words over ORB) --------------------
  int vocab_size = 400;            ///< visual words (binary k-means clusters)
  int vocab_sample_per_frame = 80; ///< descriptors sampled per frame for vocab
  int vocab_iterations = 8;        ///< k-means (k-majority) refinement passes

  // --- Feature matching + robust relative pose -----------------------------
  // Verification defaults are tuned to actually surface wide-baseline loops on
  // iPad-LiDAR scans: 1500 features / 0.05 m were too strict and found nothing;
  // 3000 features + a 0.10 m 3D-3D threshold (depth is noisy at range) recover
  // strong loops (measured: a start/end pair with ~90 RANSAC inliers).
  int max_features = 3000;    ///< ORB features per frame
  float ratio_test = 0.85f;   ///< Lowe ratio threshold for descriptor match
  int min_match_inliers = 60; ///< reject a pair below this many RANSAC inliers
  float ransac_inlier_dist = 0.10f; ///< 3D-3D inlier threshold (m)
  int ransac_iterations = 300;      ///< RANSAC hypotheses
  float min_depth = 0.3f;           ///< ignore keypoints with depth outside
  float max_depth = 5.0f;           ///< [min_depth, max_depth] (m)
  /// Optional sanity gate: reject an edge whose relative translation disagrees
  /// with the seed poses by more than this (m). DISABLED by default (<= 0):
  /// the whole point of loop closure is to correct drift, so a large
  /// seed-disagreement is the signal, not the noise — hard-gating it throws
  /// away exactly the informative constraints (the #221 sweep learned this the
  /// hard way). Geometric RANSAC inlier count + GNC do the outlier rejection
  /// instead. Set > 0 only when the seed poses are trusted and you want a
  /// coarse guard against gross mismatches.
  float max_seed_disagreement = 0.0f;
  /// LOWER gate: keep a loop edge only if its relative translation disagrees
  /// with the seed poses by at least this (m). An edge that already agrees with
  /// the seed carries NO drift-correction information — it only re-imposes the
  /// noisier ORB+depth estimate on poses that are already right, which perturbs
  /// an already-good scan (measured: honka GT 0.795 -> 0.73 from redundant
  /// edges). This is the opposite of max_seed_disagreement and is safe: it
  /// drops only non-informative edges, never the large-drift corrections. On a
  /// well-aligned scan this makes loop closure a near no-op; on a drifted scan
  /// the true loop (large disagreement) is kept. Set 0 to keep every edge.
  ///
  /// This is the ABSOLUTE FLOOR of the gate (issue #339). The gate actually
  /// applied is `max(min_seed_disagreement_fraction * trajectory_extent,
  /// min_seed_disagreement)` — see seed_disagreement_gate(). The floor encodes
  /// a sensor property (the iPad-LiDAR depth-noise level below which a
  /// disagreement is matcher noise rather than drift) and so does NOT scale
  /// with the capture; the fraction encodes the drift budget, which does.
  float min_seed_disagreement = 0.10f;
  /// Scale-relative part of the lower gate: a fraction of the seed
  /// trajectory's extent (issue #339). Drift grows with how far the camera
  /// travelled, so a gate that separates "drift" from "matcher noise" cannot be
  /// a constant in metres — measured on a 16 m-drift scan it selects the wrong
  /// edges on a 2 m room scan (registration-improvements.md §9.4). Default
  /// 0.00555 reproduces today's 0.10 m on the office scan's 18.01 m extent
  /// exactly (0.00555 * 18.01 = 0.0999 m, below the 0.10 m floor), so this
  /// default only ever STRENGTHENS the gate — on captures longer than ~18 m.
  /// Set 0 to use the absolute floor alone.
  float min_seed_disagreement_fraction = 0.00555f;

  // --- Edge noise ----------------------------------------------------------
  /// Base translational/rotational std at min_match_inliers; both shrink as
  /// sqrt(min_match_inliers/inliers) so well-supported edges pull harder (down
  /// to the floors below). Deliberately loose: an ORB+depth relative pose has
  /// cm-scale error, so an over-confident edge injects that error broadly
  /// (measured: tight defaults dropped honka GT 0.795 -> 0.770). GNC
  /// down-weights gross outliers on top.
  float base_sigma_trans = 0.10f; ///< m
  float base_sigma_rot = 0.05f;   ///< rad
  float min_sigma_trans = 0.04f;  ///< floor on the translational std (m)
  float min_sigma_rot = 0.02f;    ///< floor on the rotational std (rad)

  // --- Pairwise Consistency Maximization (false-positive rejection) --------
  /// Keep only the largest MUTUALLY CONSISTENT set of loop edges (Mangelson et
  /// al., PCM). Two true loops form a near-identity cycle when chained through
  /// the seed odometry between their endpoints; a perceptual-aliasing false
  /// positive does not. This is what makes trusting loop edges safe on scans
  /// with repeated structure. Strongly recommended before loop_edges_trusted.
  bool pcm = true;
  float pcm_trans_threshold = 0.30f; ///< cycle translation tolerance (m)
  float pcm_rot_threshold = 0.15f; ///< cycle rotation tolerance (rad, ~8.6 deg)
  /// Cap the edge set fed to the O(M^2) consistency test (keep the highest-
  /// inlier edges). 0 = no cap.
  int pcm_max_edges = 3000;

  unsigned seed = 42; ///< RANSAC determinism

  /// Memberwise, covering every field above — see SurfelExtractionParams's
  /// operator== for why (integration review finding 4).
  bool operator==(const LoopClosureOptions &) const = default;
};

/// Spatial extent of a trajectory: the diagonal of the axis-aligned bounding
/// box of the camera centres, in metres.
///
/// This is the capture-scale denominator every scale-relative threshold in the
/// loop-closure front-end divides by (issue #339). It is deliberately the bbox
/// diagonal rather than the travelled path length: what a loop edge can be
/// asked to correct is how far apart two *places* are, not how long the
/// operator walked between them, and a scan that walks the same corridor twice
/// must not read as twice the scale. Deterministic, O(N), independent of frame
/// ordering (STANDARDS §6). Returns 0 for fewer than two poses.
///
/// Reference values (registration-improvements.md §9.5): office 18.01 m,
/// ARKitScenes 41069048/50/51 1.82 / 1.86 / 2.57 m.
double trajectory_extent(const std::vector<Eigen::Matrix4d> &poses);

/// Effective seed-disagreement gate in metres, combining a scale-relative term
/// with an absolute floor: `max(fraction * extent, floor)`, with negative
/// inputs treated as 0. A result of 0 means "no gate" (keep every edge).
///
/// Issue #339: expressing the gate purely in metres does not transfer across
/// capture scales. The 0.50 m external default was tuned on a scan with ~16 m
/// of drift over an 18 m extent; applied unchanged to a 1.8 m ARKitScenes room
/// scan it stops selecting *informative* edges and starts selecting the *most
/// disagreeing* — i.e. the wrong — ones, measured at F@50mm 0.29
/// (registration-improvements.md §9.4). The fraction term fixes that. The floor
/// term is kept because a matcher's own error is a sensor property that does
/// NOT shrink with the room: on a small scan the floor is what keeps loop
/// closure a no-op instead of injecting depth noise.
double seed_disagreement_gate(double extent, double fraction, double floor_m);

/// Summary statistics from loop-edge detection.
struct LoopClosureResult {
  int candidates = 0;    ///< spatially proposed pairs that were matched
  int edges = 0;         ///< accepted loop edges
  int total_inliers = 0; ///< summed RANSAC inliers across accepted edges
};

/// Detect wide-baseline loop edges among the given frames.
///
/// @param db          project database (read-only image/depth/intrinsics
/// access)
/// @param node_ids    database node id of each frame, in frame-index order (so
///                    LoopEdge::i/j index straight into the caller's vector)
/// @param seed_poses  optical->world pose of each frame (used to propose
/// spatial
///                    candidates and sanity-check edges)
/// @param options     detection parameters
/// @param out_result  optional statistics
/// @returns           accepted loop edges (possibly empty)
std::vector<LoopEdge>
detect_loop_edges(ProjectDB &db, const std::vector<int> &node_ids,
                  const std::vector<Eigen::Matrix4d> &seed_poses,
                  const LoopClosureOptions &options,
                  LoopClosureResult *out_result = nullptr);

/// Load externally-computed loop edges from a JSON file and map them onto the
/// optimizer's frame indices. This is the license-clean bridge for learned
/// matchers: any external producer — a commercial-safe matcher (XFeat /
/// EfficientLoFTR / LightGlue+ALIKED / MapAnything-apache) or an offline
/// research oracle (MASt3R / MapAnything-NC — non-commercial, run in Python
/// only for accuracy-ceiling probes) — writes a file of relative-pose
/// constraints, and the GPL C++ consumes it as DATA. No non-commercial or
/// non-GPL model ever links into the binary; see
/// docs/research/loop-closure-learned-matchers.md.
///
/// File schema "reusex.loop_edges.v1" (JSON):
///   {
///     "schema": "reusex.loop_edges.v1",
///     "producer": "<free-text tag, e.g. mast3r@<sha> or xfeat>",
///     "edges": [
///       { "node_i": <int>, "node_j": <int>,
///         "T_ij": [16 doubles, row-major 4x4 = pose(i)^-1 * pose(j) in the
///                  optical->world convention the graph uses],
///         "sigma_rot": <double, rad>, "sigma_trans": <double, m>,
///         "inliers": <int> }
///     ]
///   }
/// `node_i`/`node_j` are DB sensor_frame node ids; they are mapped to frame
/// indices via @p node_ids (node_ids[k] is the DB id of frame k). Edges that
/// reference a node id absent from @p node_ids, self-loops, or duplicate the
/// same (i,j) are skipped with a warning (never silently dropped —
/// docs/STANDARDS.md §5). Missing sigma/inliers fall back to the LoopEdge
/// defaults.
///
/// The numeric payload is validated, not merely shape-checked: an edge is
/// rejected as malformed when `T_ij` is not finite, its 3x3 block is not a
/// proper rotation (`R^T R != I` beyond 1e-3, or `det(R) < 0` — a reflection
/// would corrupt the solve), its bottom row is not `[0,0,0,1]` (beyond 1e-6),
/// or a supplied sigma is non-finite or <= 0. A zero sigma is specifically
/// dangerous: it becomes a gtsam Constrained (hard-equality) noise model, so
/// one bad producer line would nail two poses together.
///
/// This loader does NOT apply consistency filtering. PCM and the
/// seed-disagreement gate are applied by the caller over the UNION of the
/// external and internally-detected edges — see
/// `filter_consistent_loop_edges` and optimize_sensor_poses().
///
/// @param path       JSON file path
/// @param node_ids   DB node id of each frame, in frame-index order
/// @param out_result optional statistics (edges accepted, summed inliers)
/// @returns          accepted loop edges with i/j as frame indices
/// @throws std::runtime_error if the file cannot be opened or is not valid JSON
///         of the expected schema (a bad edge file must fail loudly, not run a
///         silently-unconstrained optimization).
std::vector<LoopEdge> load_loop_edges(const std::string &path,
                                      const std::vector<int> &node_ids,
                                      LoopClosureResult *out_result = nullptr);

/// Keep only the largest mutually consistent subset of @p edges (Pairwise
/// Consistency Maximization, Mangelson et al. 2018): two edges are consistent
/// when chaining them through the @p seed odometry between their endpoints
/// closes a near-identity cycle. True loops satisfy this even under heavy
/// drift; a perceptual-aliasing false positive does not.
///
/// Public because loop edges reach the graph from TWO sources — internal
/// detection (detect_loop_edges, which PCM-filters its own output) and the
/// external `--loop-edges` file — and consistency is a property of the UNION,
/// not of either source alone. Callers that union the two must run this over
/// the combined set, otherwise an aliasing false positive in the file has no
/// consistency filter at all (which is exactly the case `--loop-trust` makes
/// dangerous). Honour `options.pcm` (CLI `--loop-no-pcm`) at the call site so
/// one switch governs both sources.
///
/// @param edges   candidate edges; indices must address @p seed
/// @param seed    seed (pre-optimization) pose of every frame
/// @param options `pcm_trans_threshold` / `pcm_rot_threshold` / `pcm_max_edges`
/// @returns       the retained subset (sets of <= 2 edges are returned
///                unchanged — a pair cannot outvote itself)
std::vector<LoopEdge>
filter_consistent_loop_edges(std::vector<LoopEdge> edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &options);

/// Indices (into @p edges, ascending) of the largest mutually consistent
/// subset that contains every edge with fixed[k] == true — `rux graph
/// discover` passes the user-confirmed stored edges as fixed (spec §4.3).
/// A non-fixed edge inconsistent with any fixed edge is never kept, even
/// when it belongs to a larger clique or the set is too small for a clique
/// search; fixed edges are always kept, and the `pcm_max_edges` cap never
/// drops one. Greedy max-clique like pcm_filter, with deterministic (stable)
/// ordering. With no fixed edge, sets of <= 2 edges are kept whole (a pair
/// cannot outvote itself). @p fixed shorter than @p edges is padded with
/// false.
std::vector<size_t>
consistent_loop_edge_indices(const std::vector<LoopEdge> &edges,
                             const std::vector<Eigen::Matrix4d> &seed,
                             const LoopClosureOptions &options,
                             const std::vector<bool> &fixed);

/// Internals of detect_loop_edges, exposed ONLY so the geometry stages of the
/// pipeline can be regression-tested without a database and without RGB-D
/// fixtures (docs/STANDARDS.md §7; same rationale as
/// `plane_factor_consistent_residual()` in PlaneGraphOptimizer.cpp). Not part
/// of the stable API — do not call from application code.
namespace detail {

/// RANSAC 3D-3D rigid fit: estimate @p best_T with `dst ~= best_T * src` from
/// putative correspondences (both 3xN, column k of `src` matching column k of
/// `dst`). Uses `opt.ransac_iterations` hypotheses and `opt.ransac_inlier_dist`
/// as the inlier distance, refits on the full inlier set, and returns the
/// inlier column indices (empty when fewer than 3 correspondences are given or
/// no hypothesis was finite).
std::vector<int> ransac_rigid(const Eigen::Matrix3Xd &src,
                              const Eigen::Matrix3Xd &dst,
                              const LoopClosureOptions &opt, std::mt19937 &rng,
                              Eigen::Matrix4d &best_T);

/// Pairwise Consistency Maximization (Mangelson et al. 2018): return the
/// largest mutually consistent subset of @p edges, where two edges are
/// consistent when chaining them through the @p seed odometry closes a cycle
/// within `opt.pcm_trans_threshold` / `opt.pcm_rot_threshold`. Sets of two or
/// fewer edges are returned unchanged (a pair cannot outvote itself).
std::vector<LoopEdge> pcm_filter(std::vector<LoopEdge> edges,
                                 const std::vector<Eigen::Matrix4d> &seed,
                                 const LoopClosureOptions &opt);

} // namespace detail

} // namespace reusex::geometry
