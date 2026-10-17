// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Plane-landmark pose-graph back-end (P1 of issue #225).
//
// ReUseX's first owned global pose-optimization stage. Where Joint Pairwise
// Registration (JointPairwiseRegistration) polishes poses locally via
// frame-to-frame point-to-plane residuals — and provably saturates because
// pairwise energy cannot express global consistency — this optimizer changes
// the objective to *shared global plane landmarks*:
//
//   1. detect the dominant planes each frame sees (from its surfels),
//   2. associate those observations across frames into persistent
//      gtsam::OrientedPlane3 landmarks,
//   3. build a factor graph (per-frame Pose3 + odometry BetweenFactors from the
//      seed poses + one OrientedPlane3Factor per observation + a soft prior on
//      the first pose for gauge), and
//   4. solve it with GncOptimizer (Graduated Non-Convexity) wrapping
//      Levenberg-Marquardt, so a handful of bad plane associations are
//      down-weighted rather than corrupting the global solution.
//
// The surfel extraction and the pose write-back (removal of the constant local
// optical->sensor transform) mirror the JPR path exactly, so the two stages are
// interchangeable in the pipeline.
//
// This header is deliberately free of any GTSAM include (STANDARDS.md §2): the
// factor graph lives entirely in the .cpp.

#pragma once

#include "reusex/segmentation/Surfel.hpp"
#include "reusex/segmentation/surfel_extraction.hpp"
#include "reusex/slam/LoopClosure.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace reusex {
class ProjectDB;
}
namespace reusex::core {
struct PoseEdits;
}

namespace reusex::geometry {

/// How the measurement noise of an individual plane observation is chosen.
///
/// Every observation enters the graph as an `OrientedPlane3Factor` whose 3-DoF
/// residual is (two normal-tilt components [rad], one plane-offset component
/// [m]). This enum selects how much authority a *particular* observation gets
/// relative to the others; `plane_sigma_normal` / `plane_sigma_distance` always
/// remain the sigmas of a **median-quality** observation, so switching models
/// redistributes authority without changing the plane term's aggregate weight
/// against odometry.
enum class PlaneNoiseModel {
  /// Every observation gets `plane_sigma_*` unscaled. A plane fit from 40
  /// surfels on a cluttered shelf carries exactly as much authority as one fit
  /// from 4000 surfels on a bare wall — measured to be actively harmful once
  /// plane extraction gets dense (honka laser-GT F 0.795 -> 0.759).
  uniform,
  /// Legacy (pre-#225 round 4): `sigma *= clamp(sqrt(median_N / N), min, max)`,
  /// the SAME scalar on both the normal and the distance channel. Retained so
  /// the numbers recorded in #225 stay reproducible.
  inlier_count,
  /// Sigmas derived from the fit statistics of each detection — its inlier
  /// count, its point-to-plane residual RMS and its in-plane extent — see
  /// `plane_fit_sigmas`. Unlike `inlier_count` this weights the two channels
  /// *separately*, which matters because extent affects only the normal.
  fit_geometry,
};

/// How the per-edge measurement noise of the consecutive-frame odometry
/// `BetweenFactor`s is derived (#225 odometry-trust experiment).
enum class OdometryNoiseModel {
  /// Every edge gets `odometry_sigma_rot` / `odometry_sigma_trans` unscaled
  /// (modulo the observability guard). The shipped default.
  fixed,
  /// Sigma proportional to the edge's OWN seed motion, normalised by the
  /// median motion over the run: a frame pair the device barely moved between
  /// is a near-noiseless relative measurement, while one spanning a fast sweep
  /// integrates far more sensor error. This is the textbook "odometry error
  /// grows with distance travelled" model, and — like `fit_geometry` for the
  /// plane term — the median normalisation leaves the odometry chain's
  /// AGGREGATE authority against the plane factors unchanged, redistributing
  /// it rather than re-scaling it.
  motion,
};

/// Seed motion of one consecutive-frame odometry edge.
struct OdometryEdgeMotion {
  double translation = 0.0; ///< |t_i+1 - t_i| in the seed trajectory (m)
  double rotation = 0.0;    ///< relative rotation angle of the seed pair (rad)
};

/// Multiplicative scale applied to an odometry edge's base sigmas.
struct OdometrySigmaScale {
  double rotation = 1.0;
  double translation = 1.0;
};

/// Per-edge sigma scales for `OdometryNoiseModel::motion`.
///
/// Each channel is scaled by that edge's motion relative to the **median**
/// motion over the run, clamped to `[min_scale, max_scale]`:
///
///     scale_trans_i = clamp(|dt_i| / median_j(|dt_j|), min, max)
///     scale_rot_i   = clamp(|dr_i| / median_j(|dr_j|), min, max)
///
/// Properties this function guarantees, and which the unit tests pin:
///  - a **uniform-motion** run yields all-1.0 scales, i.e. it degenerates
///    exactly to `OdometryNoiseModel::fixed` (so the model is a strict
///    generalisation of the shipped one);
///  - a zero or non-finite median leaves every scale at 1.0 rather than
///    producing infinities (a stationary capture must not be a divide-by-zero);
///  - the mapping is monotone: a larger motion never earns a smaller sigma.
///
/// The two channels are normalised independently because a pure-rotation sweep
/// and a pure-translation dolly are different failure modes of the seed.
std::vector<OdometrySigmaScale>
odometry_motion_scales(const std::vector<OdometryEdgeMotion> &motion,
                       double min_scale, double max_scale);

/// Fit statistics of one per-frame plane detection: the inputs from which the
/// measurement noise of its `OrientedPlane3Factor` is derived.
struct PlaneFitQuality {
  int inliers = 0;           ///< supporting surfel count of the fit
  double residual_rms = 0.0; ///< RMS point-to-plane distance of the inliers (m)
  /// RMS in-plane extent along the **weaker** of the two principal in-plane
  /// axes (m). Deliberately the minor axis, not the mean footprint radius:
  /// gtsam expresses the plane residual in the landmark `Unit3`'s tangent
  /// basis, which is arbitrary with respect to the plane's principal axes, so a
  /// `Diagonal` noise model cannot represent true anisotropy. Taking the axis
  /// that constrains the normal *least* is the honest bound.
  double extent_minor = 0.0;
};

/// Statistical standard deviations of a plane fit, in the units of the
/// `OrientedPlane3Factor` residual.
struct PlaneFitSigmas {
  double normal = 0.0;   ///< normal-tilt std (rad)
  double distance = 0.0; ///< plane-offset std (m)
};

/// Standard first-order uncertainty of a least-squares plane fit through @p q
/// .inliers points with point noise `sigma = q.residual_rms` and in-plane RMS
/// extent `r = q.extent_minor`:
///
///     sigma_distance = sigma / sqrt(N)
///     sigma_normal   = sigma / (r * sqrt(N))
///
/// The offset is an average over the inliers, so it tightens as `1/sqrt(N)`.
/// The tilt is a *lever-arm* estimate: the same point noise at the edge of a
/// 3 m wall subtends a far smaller angle than at the edge of a 20 cm patch, so
/// the normal additionally tightens as `1/r`. This factor of `r` is the whole
/// reason the two channels must not share one scalar — two detections with
/// identical `N` and `sigma` but 15x different extent are equally certain in
/// offset and 15x apart in tilt.
///
/// The returned values are *relative* quality indicators, not absolute
/// calibrated sigmas: the dominant real error is model error (surface
/// roughness, depth bias, slight mis-association), not the fit's own
/// statistical error. The optimizer therefore normalises them by their median
/// across the run and applies the result as a scale on `plane_sigma_normal` /
/// `plane_sigma_distance`.
///
/// Degenerate inputs are clamped rather than propagated, so the result is
/// always finite and strictly positive.
PlaneFitSigmas plane_fit_sigmas(const PlaneFitQuality &q);

/// Which world axis is "up" for the Manhattan prior.
enum class ManhattanUp {
  /// The world axis along which the camera positions spread LEAST: a capture
  /// is walked at roughly constant height, so this is the vertical unless the
  /// trajectory climbs more than it travels. (The frames' mean image-up is
  /// NOT usable: a phone held in portrait over a landscape sensor puts image
  /// up horizontal — house1 would get -y.) A heuristic; pass the axis when
  /// the importer's convention is known.
  automatic,
  x,
  y,
  z,
};

/// Parameters for plane-landmark pose-graph optimization. All defaults live
/// here (STANDARDS.md §4); CLI flags mirror them.
struct PlaneGraphOptions {
  // --- Per-frame plane detection (sequential RANSAC on frame surfels) -------
  // "Mid-density" defaults: measured to beat the old conservative values
  // (4 / 200 / obs 5) on both office flatness and honka laser GT once
  // per-observation plane weighting (plane_noise_model) is on.
  int max_planes_per_frame = 6;     ///< keep at most this many planes per frame
  int min_plane_inliers = 120;      ///< reject planes with fewer inliers
  float ransac_distance = 0.02f;    ///< inlier point-to-plane distance (m)
  float ransac_normal_angle = 20.f; ///< inlier normal agreement threshold (deg)
  int ransac_iterations = 200;      ///< RANSAC hypotheses per plane

  // --- Cross-frame association into landmarks -------------------------------
  float assoc_normal_angle = 10.f; ///< max normal angle to merge (deg)
  float assoc_distance = 0.10f;    ///< max plane-offset difference to merge (m)
  int min_landmark_observations =
      4; ///< landmark must be seen by >= this many frames
  /// Overlap gate: two observations may only merge into the same landmark if
  /// their in-plane footprints overlap. The in-plane gap between their inlier
  /// centroids must be below (sum of the two footprint radii + this margin).
  /// This is what stops two distinct offset/disjoint parallel walls that happen
  /// to agree in normal+offset (aliasing) from collapsing into one landmark.
  /// <= 0 disables the overlap check (pure normal+offset gating, legacy).
  float assoc_overlap_margin = 0.30f;
  /// Reject landmarks whose observation inlier-centroids are near-collinear: if
  /// the second singular value of the centroid scatter is below this fraction
  /// of the first, the observations pin only a line, leaving a rotational DoF
  /// about that line unconstrained. Such landmarks are dropped from the graph.
  /// <= 0 disables the degeneracy check.
  float min_landmark_spread_ratio = 0.05f;

  // --- Alternating association/optimization rounds --------------------------
  /// Number of (associate at current poses -> optimize -> refit) rounds. One
  /// round reproduces the original one-shot behaviour; 2-3 rounds let landmarks
  /// re-form on the improved poses (EM-style), which recovers drift the greedy
  /// one-shot association aliases away. Rounds stop early once the pose update
  /// of a round falls below assoc_round_tol. Two rounds is the tuned default
  /// (best office flatness at cm-level shift without regressing MuSHRoom GT).
  int assoc_rounds = 2;
  float assoc_round_tol = 0.02f; ///< stop rounds when max pose shift < this (m)

  // --- Factor graph noise (std-devs) ----------------------------------------
  // The seed trajectory (RTABMap SLAM) is already good, so odometry is trusted
  // tightly and the plane factors act as a gentle global regularizer: this is
  // what keeps corrections at cm scale and beats the no-op baseline instead of
  // degrading it (the #225 P1 defaults over-trusted the fragile plane factors).
  float odometry_sigma_rot = 0.005f;  ///< odometry rotation std (rad)
  float odometry_sigma_trans = 0.01f; ///< odometry translation std (m)
  /// Observability guard: a frame whose landmark observations span fewer than
  /// two independent normal directions is under-constrained by planes along the
  /// missing directions. Its two odometry factors (to prev/next frame) get
  /// their sigmas multiplied by this factor (< 1 tightens them) so the trusted
  /// odometry holds the trajectory where planes cannot. 1.0 disables the guard.
  float underconstrained_odom_scale = 0.25f;
  /// Which per-edge noise model weights the odometry factors (#225).
  ///
  /// The default is `fixed`, and the **measurement is why** — the odometry
  /// sweep on the three ARKitScenes GT scans falsified the standing hypothesis
  /// that `rux optimize` regresses drifting captures because it over-trusts the
  /// drifted seed. Loosening odometry uniformly makes absolute GT accuracy
  /// monotonically WORSE on every scan (41069048 F@50mm 0.8512 at the default,
  /// 0.4565 at 10x looser, 0.2346 at 100x), while tightening it 10x moves the
  /// result back TOWARD the no-pose-stage baseline (0.8802 vs 0.8917). The
  /// odometry chain is the accurate part of this graph; the plane term is what
  /// costs absolute accuracy on these captures.
  OdometryNoiseModel odometry_noise_model = OdometryNoiseModel::fixed;
  float odometry_weight_min = 0.5f; ///< min per-edge sigma scale (`motion`)
  float odometry_weight_max = 3.0f; ///< max per-edge sigma scale (`motion`)
  /// Let GNC down-weight individual odometry edges instead of registering every
  /// one as a known inlier.
  ///
  /// Default `false`, again because it was measured rather than assumed: the
  /// premise ("a grossly-wrong seed edge should be demotable") is sound in
  /// principle, but on captures whose drift accumulates smoothly there is no
  /// single wrong edge to demote — the error is spread evenly over thousands of
  /// individually-good relative measurements. Turning it on therefore just
  /// removes constraints, which the sigma sweep already showed to be harmful.
  bool odometry_robust = false;
  /// GNC-TLS inlier threshold for odometry factors when `odometry_robust` is
  /// set. Odometry is a 6-DoF measurement, so the analogue of the plane term's
  /// chi-square 99% / 2 is chi2(6, 0.99) / 2 = 16.81 / 2.
  float odometry_gnc_inlier_cost = 8.41f;
  float plane_sigma_normal = 0.24f;   ///< plane-normal measurement std (rad)
  float plane_sigma_distance = 0.19f; ///< plane-distance measurement std (m)
  /// Which per-observation noise model weights the plane factors.
  ///
  /// **The default is deliberately `inlier_count`, not the newer, physically
  /// better-motivated `fit_geometry`** — because the measurement says so, and
  /// the measurement says something more interesting than "one model wins":
  ///
  ///  - On scans whose seed poses genuinely DRIFT (ARKitScenes 41069048 /
  ///    41069050 / 41069051, absolute `3dod_mesh` GT), `fit_geometry` beats
  ///    `inlier_count` on every metric of every scan — GT F +0.8% / +1.9% /
  ///    +5.2%, median accuracy error -5.6% / -12.7% / -7.8%.
  ///  - On a scan that does NOT drift (MuSHRoom honka, Faro laser GT), it is
  ///    worse, and worse than running no pose stage at all (F 0.7572 baseline,
  ///    0.7595 `inlier_count`, 0.7523 `fit_geometry`).
  ///
  /// That is the same lesson the loop-closure front-end learned (#225, PR
  /// #237):
  /// **machinery that redistributes authority toward strong geometric evidence
  /// pays off exactly when the seed trajectory is wrong, and costs when it is
  /// already right.** So `fit_geometry` ships as an opt-in lever for drifting
  /// captures rather than a new default, and the default remains bit-identical
  /// to what shipped in #228 — zero regression on the guard scan.
  PlaneNoiseModel plane_noise_model = PlaneNoiseModel::inlier_count;
  /// Clamp on the per-observation sigma scale, applied to whichever model is
  /// selected. Both models are normalised by their median over the run, so a
  /// scale of 1.0 is a median-quality observation, `plane_weight_min` is the
  /// most authority any single observation may earn and `plane_weight_max` the
  /// least. The clamp is what stops one exceptionally clean (or exceptionally
  /// bad) detection from dominating the solve.
  ///
  /// **The right range depends on the model**, because the two have very
  /// different dynamic range, and the defaults below belong to the DEFAULT
  /// model (`inlier_count`). Its scale varies only as `sqrt(N)`, spans roughly
  /// one order of magnitude, and `[0.5, 3.0]` barely binds.
  ///
  /// `fit_geometry` additionally divides by the in-plane extent, which enters
  /// *linearly* and varies far more across detections, so the same range is far
  /// too narrow for it: measured, 62% of office-scan observations saturated
  /// `[0.5, 3.0]`, degrading the model into a near-binary weighting that scored
  /// WORSE than the legacy one (office flatness 13.03 mm vs 11.72 mm). Office
  /// flatness improves monotonically as the range widens to about
  /// `[0.10, 15]` (11.66 mm), turns back over past that, and the clamp stops
  /// binding entirely near `[0.05, 30]`.
  ///
  /// So `--plane-noise fit` should be paired with `--plane-weight-min 0.10
  /// --plane-weight-max 15`. Forgetting to is not silent: the optimizer warns
  /// at run time, with the numbers, when the clamp rather than the fit quality
  /// is setting most weights (STANDARDS §5).
  float plane_weight_min = 0.5f; ///< min sigma scale (strongest planes)
  float plane_weight_max = 3.0f; ///< max sigma scale (weakest planes)
  /// Global multiplier on BOTH plane sigmas, i.e. the plane term's authority
  /// relative to the odometry chain and the gauge prior. The plane term's
  /// contribution to the objective scales as `1 / plane_sigma_scale^2`, so
  /// values > 1 weaken it and values < 1 strengthen it. `1.0` is the shipped
  /// calibration and is bit-identical to not passing the knob.
  ///
  /// It exists as one dimension rather than as "remember to scale both
  /// `plane_sigma_normal` and `plane_sigma_distance` by the same factor",
  /// because the question it answers — *is there a plane-term weight that is
  /// optimal against absolute GT?* — is one-dimensional (#225 §9). Note it is
  /// NOT exactly equivalent to inversely scaling the odometry sigmas: the GNC
  /// TLS threshold `gnc_inlier_cost` is applied to the *whitened* plane
  /// residual, so changing the plane sigmas also changes which observations
  /// GNC classifies as outliers, whereas changing odometry sigmas does not.
  float plane_sigma_scale = 1.0f;
  /// Build the graph WITHOUT any plane factors — the `plane_sigma_scale ->
  /// infinity` limit, taken exactly. Plane detection, association and landmark
  /// hygiene still run (so the reported statistics stay comparable), but no
  /// `OrientedPlane3Factor` and no landmark variable enters the graph, leaving
  /// odometry + the frame-0 gauge prior. The solve is then a no-op on the seed
  /// trajectory, which is exactly what makes it useful: it is the measurable
  /// "plane term off" endpoint of a weight sweep, and a check that the harness
  /// reproduces the no-pose-stage baseline through the full pipeline.
  bool use_plane_factors = true;
  float prior_sigma_rot = 0.001f; ///< first-pose gauge prior rotation std (rad)
  float prior_sigma_trans =
      0.001f; ///< first-pose gauge prior translation std (m)

  // --- Solver ---------------------------------------------------------------
  bool use_gnc = true; ///< wrap LM in Graduated Non-Convexity
  /// GNC TLS inlier threshold on the factor error (0.5 * whitened squared
  /// residual). gtsam's default of 1.0 rejects exactly the informative
  /// (drifted) plane observations this stage exists to exploit; the default
  /// here is the chi-square 99% quantile for a 3-DoF measurement / 2
  /// (11.345 / 2), so only genuinely wrong associations are down-weighted.
  float gnc_inlier_cost = 5.67f;
  int max_iterations = 100; ///< LM (inner) iteration cap
  unsigned seed = 42;       ///< RANSAC seed (STANDARDS.md §6)

  /// Surfel extraction settings. `sampling_factor` is overridden to 6 (denser
  /// than the shared SurfelExtractionParams default of 8, which `rux register`
  /// relies on): the "mid-density" surfels this stage was tuned on. Overriding
  /// it here — rather than in SurfelExtractionParams or in apps/rux — keeps a
  /// single source of truth for the plane-graph default (STANDARDS.md §4)
  /// without changing what the registration stage extracts.
  SurfelExtractionParams surfel{.sampling_factor = 6};

  /// How loop-edge factors are treated by the solver:
  ///   false (default, SAFE): loop edges are ordinary GNC candidates under the
  ///     shared `gnc_inlier_cost` threshold — a wrong edge is down-weighted.
  ///     But GNC also zeros a genuine LARGE-drift edge (its residual at the
  ///     drifted seed looks like an outlier), so this rarely applies big
  ///     corrections. Good when loops are near the seed estimate.
  ///   true (AGGRESSIVE): loop edges keep their own, far more generous GNC
  ///     inlier threshold (`loop_trust_inlier_cost`) instead of the shared
  ///     `gnc_inlier_cost`, so a genuine large-drift correction stays an inlier
  ///     and actually flows — while an edge whose residual exceeds even that
  ///     generous threshold is still truncated by GNC-TLS, bounding a
  ///     grossly-wrong edge's influence. Under `use_gnc == false` (plain LM)
  ///     there is no GNC machinery, so trusted loop edges instead get a Huber
  ///     kernel for the same bounded-influence effect. Still needs a
  ///     discriminative matcher / PCM to be safe, and looser odometry
  ///     (`odometry_sigma_trans`) so the drift can redistribute.
  bool loop_edges_trusted = false;
  /// GNC-TLS inlier threshold applied ONLY to loop-edge factors when
  /// `loop_edges_trusted` is set (same units as `gnc_inlier_cost`: 0.5 *
  /// whitened squared residual). Deliberately generous — at the default
  /// LoopClosureOptions sigmas (0.10 m / 0.05 rad) this admits roughly 2 m of
  /// translational drift correction as an inlier — but FINITE, which is the
  /// point: a 10x-worse edge lands far above it and is truncated to zero
  /// weight. Note that GTSAM's GncOptimizer strips `noiseModel::Robust`
  /// wrappers from every factor it is given, so a per-factor threshold is the
  /// only way to bound a loop edge under GNC.
  float loop_trust_inlier_cost = 200.0f;

  /// Noise of a loop edge that carries no information matrix (a stored edge
  /// with NULL `information`, spec §3.1). Mirrors the LoopClosureOptions
  /// base sigmas.
  float loop_default_sigma_rot = 0.05f;   ///< rad
  float loop_default_sigma_trans = 0.10f; ///< m

  // --- Manhattan-world prior (reusex-rqn.1) ---------------------------------
  /// Constrain structural plane LANDMARKS to a Manhattan grid: every vertical
  /// landmark's normal to one of the two dominant horizontal axes, every
  /// horizontal landmark's normal to the up axis. The two horizontal axes are
  /// one scalar variable (their azimuth) estimated jointly with the poses, so
  /// the grid follows the gauge frame instead of a pre-fitted direction.
  ///
  /// What it buys: rotational drift becomes observable WITHOUT loop closure.
  /// The plane term only relates frames that share a landmark, so yaw drift
  /// between rooms the capture never revisits is invisible to it; measured on
  /// house1 / house2, frame wall-azimuth residuals trend by 7-15 deg over the
  /// capture (correlation with capture time -0.75 / -0.94). Default off: on a
  /// capture with little rotational drift (NewOffice, +-1.4 deg) it has
  /// nothing to correct, and a building that is not Manhattan gets nothing
  /// from it.
  bool manhattan = false;
  /// Std of a Manhattan factor's 2-DoF normal residual (deg).
  float manhattan_sigma_deg = 1.0f;
  /// Gate (deg): a vertical landmark more than this off its nearest axis, or a
  /// landmark tilted more than this from both vertical and horizontal, gets NO
  /// Manhattan factor (a genuinely off-grid wall, a sloped ceiling). The same
  /// angle is the factor's GNC-TLS inlier threshold (Huber under --no-gnc), so
  /// a wall that only drifts out of the gate during the solve is released.
  /// It must exceed the rotational drift to be corrected, and stay below the
  /// smallest real off-grid angle the building has.
  float manhattan_max_angle_deg = 15.0f;
  /// Gate (deg) for the LEVEL factors: a landmark within this of horizontal
  /// is tied to up. Much tighter than the wall gate, because tilt drift is
  /// small (house2's storeys: ~1 deg) while real ceilings slope: house1's
  /// mono-pitch ceiling (~7 deg) forced level tilted the floors 1 deg.
  float manhattan_level_max_angle_deg = 4.0f;
  /// Std (deg) of the per-frame Manhattan HEADING factor (reusex-0oi); <= 0
  /// switches that factor off and leaves only the landmark factors above.
  ///
  /// The landmark factors pull a wall LANDMARK onto the grid, and a frame
  /// follows only through its plane factors (`plane_sigma_normal`, 14 deg).
  /// A frame whose walls are seen only through such loose factors, or whose
  /// detections never form a kept landmark, is barely turned: on house2 the
  /// upper-storey N room and the blurred stair tail (frames 342-389) kept a
  /// 3-4 deg heading error under --manhattan. This factor ties the POSE
  /// directly: each of the frame's own vertical plane detections, rotated to
  /// world by the frame's pose, gets a 1-DoF residual to the nearest grid
  /// axis (theta + k * 90 deg, the same jointly estimated azimuth). It uses
  /// the wall gate (`manhattan_max_angle_deg`) both to pick the detections
  /// and as its GNC-TLS inlier threshold (Huber under --no-gnc), so off-grid
  /// walls and door leaves drop out per detection. Only active under
  /// `manhattan`.
  ///
  /// Two things keep it from bending the map (both found on the ARKitScenes
  /// drift bench, reusex-xka cells): it acts on HEADING only (its pose
  /// Jacobian is rotation about up; tilt stays with the plane and level
  /// factors), and with it the grid's azimuth starts from the walls the
  /// first frames see, next to the gauge, instead of from all landmarks of
  /// the drifted seed. The latter limits how much accumulated drift one run
  /// reaches: a wall drifted past `manhattan_max_angle_deg` from the start
  /// frames' grid is only caught once the frames before it have turned (two
  /// association rounds carried a synthetic 30 deg; 36 deg was not).
  ///
  /// The default is measured (reusex-0oi). A single detection's heading
  /// residual has a robust std of 2.0-2.1 deg (house2, at every sigma
  /// tried), but a frame's 2-4 detections share its depth and tilt errors,
  /// so they are not independent: at 2 deg house2 came closest to its DRAFT
  /// pose reference (ATE 0.131 m without the factor -> 0.075 m) while house1
  /// lost local consistency (plane flatness 11.5 -> 14.4 mm, worse than no
  /// optimisation). 4 deg keeps most of the house2 correction (ATE 0.091 m,
  /// tail -4.0 -> -0.9 deg, mesh QA mean 24.5 -> 19.1 cm) with house1 at
  /// 11.4 mm; 8 deg gives back half of it (0.103 m, tail -1.7 deg).
  float manhattan_frame_sigma_deg = 4.0f;
  /// The up axis the prior levels to. z is the pipeline's convention
  /// (storeys, plan renders, RTABMap / MuSHRoom imports).
  ManhattanUp manhattan_up = ManhattanUp::z;

  // --- Pose edits (reusex-rcv.1.3) ------------------------------------------
  /// GNC-TLS inlier threshold of the pose-edit factors (PosePrior,
  /// SurfaceConstraint), in the units of `gnc_inlier_cost` (0.5 * whitened
  /// squared residual). Edits are NOT known inliers: a person or an agent said
  /// so, but a contradictory edit must be reported rather than obeyed, so they
  /// get this generous but finite threshold, like `loop_trust_inlier_cost`.
  /// At an edit's own sigma, 200 admits a residual of 20 sigma. Under
  /// --no-gnc the edits are plain Gaussian factors and an edit above the
  /// threshold is still reported as not honoured.
  float edit_inlier_cost = 200.0f;

  /// Memberwise, covering every field above AND every field of `surfel`
  /// (its own defaulted operator==).
  /// Relied on by CliCommandRoundTrip_OptimizeFlagDefaults_MatchTheStageBase
  /// (tests/unit/rux_app/test_cli_command_roundtrip.cpp) to compare the whole
  /// struct through one comparison, so a field added here without a matching
  /// CLI default is caught instead of silently skipped by a hand-maintained
  /// field list (integration review finding 4).
  bool operator==(const PlaneGraphOptions &) const = default;
};

/// One edge of the (stored) pose graph handed to the solver, already in the
/// optimizer's OPTICAL convention (slam/pose_graph_conventions.hpp). Indices
/// are positions in the frames vector.
struct GraphEdge {
  enum class Kind { odometry, loop };
  int i = -1;
  int j = -1;
  /// pose(i)^-1 * pose(j), optical convention.
  Eigen::Matrix4d T_ij = Eigen::Matrix4d::Identity();
  /// 6x6, GTSAM order (rx, ry, rz, tx, ty, tz). nullopt: odometry uses
  /// odometry_sigma_* (x observability guard x motion model), loop uses
  /// loop_default_sigma_*.
  std::optional<Eigen::Matrix<double, 6, 6>> information;
  Kind kind = Kind::odometry;
  /// GNC may never demote this factor (a user-confirmed edge).
  bool known_inlier = false;
  /// Caller's tag, echoed in PoseGraphEdgeRecord::tag; -1 = none.
  int tag = -1;
};

/// An absolute prior on one frame's pose (a pose edit: `anchor`, or one node
/// of a `move`). Optical convention, like FrameSurfels::world_pose.
///
/// Its residual is [roll, pitch, yaw, x, y, z] (GTSAM order: rotation
/// first), all about / along `axes` (world -> residual axes; the edit
/// document's frame): the position error of the camera centre, and the
/// yaw / pitch / roll (Z-Y-X Euler) differences of the camera's LEVELLED
/// frame, i.e. its rotation times the fixed body permutation that puts the
/// target's most vertical body axis on z (prior_levelling(); this keeps a
/// portrait or a downward-looking camera clear of gimbal lock). Euler
/// components make a DoF mask exact: with only x, y and yaw selected, the
/// target's z, roll and pitch exert no force at all. Masked-out entries of
/// `information` are zero; the solver adds 1e-9 to the diagonal.
struct PosePrior {
  int i = -1;                                         ///< index into frames[]
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(); ///< target world_pose
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix3d axes = Eigen::Matrix3d::Identity(); ///< world -> residual
  std::string tag; ///< the edit's id; residuals are reported per tag
};

/// The body permutation (a proper rotation) a PosePrior levels with: its z
/// column is the body axis of @p R_target most aligned with the z of
/// @p axes (sign included).
Eigen::Matrix3d prior_levelling(const Eigen::Matrix3d &R_target,
                                const Eigen::Matrix3d &axes);
/// [roll, pitch, yaw] (rad) of world rotation @p R in a PosePrior's
/// convention (levelling @p C, residual @p axes).
Eigen::Vector3d prior_rpy(const Eigen::Matrix3d &R, const Eigen::Matrix3d &C,
                          const Eigen::Matrix3d &axes);
/// The world rotation with [roll, pitch, yaw] @p rpy: the inverse of
/// prior_rpy().
Eigen::Matrix3d prior_rotation(const Eigen::Vector3d &rpy,
                               const Eigen::Matrix3d &C,
                               const Eigen::Matrix3d &axes);

/// A relative constraint between two frames whose information may be rank
/// deficient (a pose edit: `same_surface`, `relative`): a BetweenFactor with
/// measurement T_ij = pose(i)^-1 * pose(j) (optical convention) and an error
/// in j's local frame, GTSAM order. A same-surface constraint carries
/// information only along the surface normal and about the two in-surface
/// axes; the solver adds 1e-9 to the diagonal.
struct SurfaceConstraint {
  int i = -1;
  int j = -1;
  Eigen::Matrix4d T_ij = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> information =
      Eigen::Matrix<double, 6, 6>::Identity();
  std::string tag;
};

/// How well the solution honours one pose edit (all factors sharing a tag).
struct EditResidual {
  std::string tag;
  /// Largest translation residual over the edit's factors (m), counting only
  /// the directions the edit constrains (the range of its information).
  double trans_m = 0.0;
  double rot_deg = 0.0; ///< same, rotation (deg)
  /// Root mean square of the same over the edit's factors (reusex-rcv.1.28):
  /// for a move, the block's own residual -- what its sigma speaks for, as
  /// its n priors at sigma * sqrt(n) together carry one sigma.
  double rms_trans_m = 0.0;
  double rms_rot_deg = 0.0;
  double chi2 = 0.0; ///< sum of whitened squared residuals
  /// GNC kept every factor of the edit (weight >= 0.5); without GNC, every
  /// factor's error is within `edit_inlier_cost`.
  bool gnc_inlier = true;
  int factors = 0; ///< factors the edit lowered to
};

/// One pose-graph edge produced by the optimizer, indexed by position in the
/// frames[] vector (NOT by database node_id — translate at the write site so
/// GTSAM types never escape into the public API).
struct PoseGraphEdgeRecord {
  int i = 0;             ///< from-frame index in frames[]
  int j = 0;             ///< to-frame index in frames[]
  std::string type;      ///< "odometry" | "loop_closure"
  double residual = 0.0; ///< 0.5 × whitened squared residual (GTSAM convention)
  double weight = 1.0;   ///< 1 / sigma_trans² at build time
  int tag = -1;          ///< GraphEdge::tag of the edge
};

/// Summary statistics from a plane-graph optimization run.
struct PlaneGraphResult {
  bool converged = false;      ///< optimizer produced a solution
  int frames = 0;              ///< frames that participated
  int planes_detected = 0;     ///< total per-frame plane detections
  int landmarks = 0;           ///< persistent landmarks entering the graph
  int plane_factors = 0;       ///< OrientedPlane3 observation factors
  int iterations = 0;          ///< optimizer iterations performed
  double initial_error = 0.0;  ///< factor-graph error before optimization
  double final_error = 0.0;    ///< factor-graph error after optimization
  double max_pose_shift = 0.0; ///< largest per-frame translation change (m)
  int rounds = 0; ///< association/optimization rounds actually performed
  int landmarks_rejected_overlap = 0; ///< merges blocked by the overlap gate
  int landmarks_rejected_degenerate =
      0;                           ///< landmarks dropped as near-collinear
  int underconstrained_frames = 0; ///< frames whose odometry was tightened
  /// Per-edge residuals from the FINAL solve round, frame-index based.
  /// Populated only when the optimizer converged; empty otherwise.
  std::vector<PoseGraphEdgeRecord> edges;
  int loop_edges = 0; ///< wide-baseline loop BetweenFactors added
  /// Of `loop_edges`, how many came from the panorama front-end (#236).
  /// Reported separately so a run's panorama contribution is auditable.
  int panorama_loop_edges = 0;
  /// Observations whose noise scale hit `plane_weight_min` (trusted as hard as
  /// the model allows) and `plane_weight_max` (distrusted as hard as it
  /// allows). Both saturating for most detections means the clamp — not the
  /// fit statistics — is deciding the weights, i.e. the range is too narrow.
  int plane_noise_clamped_low = 0;
  int plane_noise_clamped_high = 0;
  /// Median fit sigmas across all detections, the values the scales are
  /// normalised by (0 under PlaneNoiseModel::uniform). Reported because they
  /// are the run's own estimate of what a typical plane observation is worth.
  double median_fit_sigma_normal = 0.0;   ///< rad
  double median_fit_sigma_distance = 0.0; ///< m
  /// Stored-graph bookkeeping (optimize_sensor_poses).
  int graph_edges = 0;  ///< stored edges that entered the solve
  int bridge_edges = 0; ///< connectivity bridges from the initial poses
  bool graph_initialised = false; ///< the run created the odometry edges
  int edges_with_metrics = 0;     ///< stored rows whose metrics were written
  /// Manhattan prior (zero unless PlaneGraphOptions::manhattan), final round.
  int manhattan_vertical = 0;   ///< wall landmarks tied to a horizontal axis
  int manhattan_horizontal = 0; ///< floor/ceiling landmarks tied to up
  int manhattan_skipped = 0;    ///< landmarks outside the gate (no factor)
  /// Per-frame heading factors (reusex-0oi), final round: vertical plane
  /// detections tied to the grid, detections outside the wall gate (no
  /// factor), and factors GNC down-weighted below 0.5 at the solution.
  int manhattan_frame_factors = 0;
  int manhattan_frame_skipped = 0;
  int manhattan_frame_outliers = 0;
  /// Solved azimuth of the first horizontal axis, wrapped to [-45, 45) deg and
  /// measured from the world axis after `manhattan_up_axis` (x for z-up).
  double manhattan_axis_deg = 0.0;
  int manhattan_up_axis = -1; ///< 0 / 1 / 2 = world x / y / z; -1 = off
  /// Pose edits (reusex-rcv.1.3), final round, in the order of their first
  /// factor. Empty without edits.
  std::vector<EditResidual> edit_residuals;
  int edit_factors = 0; ///< PosePrior + SurfaceConstraint factors added
  /// optimize_sensor_poses with edits: ids of edits read but not lowered
  /// (on_plan_wall, for now) and stored edges left out by reject_edge.
  std::vector<std::string> edits_skipped;
  int edges_rejected_by_edits = 0;
};

/// Plane-landmark pose-graph optimizer operating purely in memory.
class PlaneGraphOptimizer {
    public:
  explicit PlaneGraphOptimizer(PlaneGraphOptions options);

  /// Globally optimize the world poses of the given frames in place (each
  /// FrameSurfels::world_pose is updated). Does not touch any database. When
  /// too few plane landmarks are found the poses are left unchanged.
  ///
  /// @param frames      per-frame surfels + seed poses (mutated in place)
  /// @param loop_edges  optional wide-baseline relative-pose constraints (P2);
  ///                    added as robust BetweenFactor<Pose3> edges the GNC
  ///                    solver can down-weight. Indices must refer to @p
  ///                    frames.
  PlaneGraphResult optimize(std::vector<FrameSurfels> &frames,
                            const std::vector<LoopEdge> &loop_edges = {}) const;

  /// Solve with an explicit edge list instead of consecutive-seed odometry:
  /// odometry AND loop factors all come from @p edges (out-of-range or
  /// self-loop edges are skipped). The gauge prior still pins frame 0 at its
  /// world_pose. optimize() == optimize_graph(frames,
  /// legacy_graph_edges(frames, loop_edges)).
  ///
  /// @p priors and @p surfaces are pose edits (slam/pose_edit_lowering.hpp):
  /// extra factors with the `edit_inlier_cost` GNC threshold, reported per
  /// tag in PlaneGraphResult::edit_residuals. Without them the result is
  /// bit-identical to a call that does not pass them. With them the graph is
  /// solved even when no plane landmark survives.
  PlaneGraphResult
  optimize_graph(std::vector<FrameSurfels> &frames,
                 const std::vector<GraphEdge> &edges,
                 std::span<const PosePrior> priors = {},
                 std::span<const SurfaceConstraint> surfaces = {}) const;

    private:
  PlaneGraphOptions options_;
};

/// The graph optimize() has always built: odometry between consecutive frames
/// measured from their world_pose (no information: default sigmas), then each
/// loop edge with information_from_sigmas(sigma_rot, sigma_trans).
std::vector<GraphEdge>
legacy_graph_edges(const std::vector<FrameSurfels> &frames,
                   const std::vector<LoopEdge> &loop_edges);

/// High-level entry point: solve the STORED pose graph of @p db
/// (pose_graph_edges, schema v28). Seeds every frame from
/// sensor_frames.initial_transform (falling back to `transform` with a
/// warning), adds one factor per active stored edge (auto + confirmed, with a
/// measurement; confirmed loop edges are GNC known inliers), bridges a
/// disconnected graph from the initial poses, and — unless @p dry_run —
/// writes the optimized world poses to `transform` (never to
/// initial_transform) and residual / deviation / seed-disagreement back to
/// every stored edge. The edge set is never replaced. When the graph has no
/// usable odometry, slam::init_pose_graph() runs first (in memory only under
/// @p dry_run).
///
/// @param db       Project database (read/write).
/// @param options  Optimization parameters.
/// @param dry_run  When true, compute and report statistics without writing.
/// @param edits    Optional pose edits (docs/design/pose-edits.md), lowered
///                 by slam::lower_pose_edits() over the solved frames; their
///                 residuals land in PlaneGraphResult::edit_residuals. The
///                 caller checks the edits' project_guid.
/// @returns        Optimization statistics.
PlaneGraphResult optimize_sensor_poses(ProjectDB &db,
                                       const PlaneGraphOptions &options,
                                       bool dry_run = false,
                                       const core::PoseEdits *edits = nullptr);

/// The stored pose graph of a project, loaded once so it can be solved many
/// times (`rux optimize --watch`, reusex-rcv.1.5): the per-frame surfels are
/// the expensive part and do not depend on the poses or the edits.
/// optimize_sensor_poses() is load_stored_pose_graph() +
/// solve_stored_pose_graph() + the write.
struct StoredPoseGraph {
  /// Frames with surfels, node order; world_pose = the INITIAL pose (the
  /// seed every solve starts from), optical convention.
  std::vector<FrameSurfels> frames;
  std::vector<Eigen::Matrix4d> local;       ///< per frame: localTf
  std::vector<Eigen::Matrix4d> seed_stored; ///< per frame: initial, worldTf
  /// One per stored edge row (plus the in-memory odometry of a graph without
  /// any), in ProjectDB::list_pose_graph_edges() order.
  struct Edge {
    int from_node_id = 0, to_node_id = 0;
    std::string edge_type;
    /// The measurement, worldTf convention; nullopt = none stored.
    std::optional<Eigen::Matrix4d> measurement;
    /// The factor it becomes (tag = its index here); nullopt when it does
    /// not enter a solve (rejected, no measurement, frame not solved).
    std::optional<GraphEdge> solver;
    bool panorama = false;
  };
  std::vector<Edge> edges;
  /// The project has no usable odometry: it was derived from the initial
  /// poses in memory (nothing written).
  bool odometry_in_memory = false;
  std::size_t frames_total = 0; ///< sensor frames in the project
};

/// Load the stored pose graph of @p db for repeated solves. Read-only: a
/// graph without odometry gets it in memory (slam::odometry_init_edges).
/// Warns (STANDARDS §5) about skipped edges and frames, as the solve does.
/// @throws std::runtime_error with fewer than 2 frames that yield surfels.
StoredPoseGraph load_stored_pose_graph(const ProjectDB &db,
                                       const PlaneGraphOptions &options);

/// One solve of a loaded graph.
struct StoredGraphSolve {
  PlaneGraphResult result;
  /// Solved poses, sensor_frames.transform convention, parallel to
  /// StoredPoseGraph::frames. Equal to the seeds when nothing constrained
  /// the solve (no landmark, loop edge or edit).
  std::vector<Eigen::Matrix4d> poses;
};

/// Solve @p graph from its initial poses with @p edits (lowered over the
/// graph's frames against @p db). Never writes; deterministic: the same
/// graph, options and edits give bit-identical poses (STANDARDS §6).
/// @throws std::runtime_error naming the edit when an edit cannot be lowered.
StoredGraphSolve
solve_stored_pose_graph(const ProjectDB &db, const StoredPoseGraph &graph,
                        const PlaneGraphOptions &options,
                        const core::PoseEdits *edits = nullptr);

} // namespace reusex::geometry
