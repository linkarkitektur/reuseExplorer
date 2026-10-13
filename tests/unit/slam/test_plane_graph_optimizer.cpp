// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the plane-landmark pose-graph optimizer (issue #225, P1).
//
// Strategy: build several sensor frames that all observe the *same* three
// mutually-perpendicular planes (a corner). Three perpendicular planes make all
// six pose DoF observable from plane factors alone. Placing the frames at known
// world poses and then corrupting the seed poses with drift, the optimizer must
// pull the drifted poses back toward the (shared, globally-consistent) truth.
// Fixed seeds keep everything deterministic (docs/STANDARDS.md §6).

#include <reusex/segmentation/Surfel.hpp>
#include <reusex/slam/PlaneGraphOptimizer.hpp>
#include <reusex/types.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <vector>

using namespace reusex;
using namespace reusex::geometry;
using Catch::Matchers::WithinAbs;

// Implemented in PlaneGraphOptimizer.cpp (gtsam lives only in the .cpp).
// Returns the OrientedPlane3Factor error norm for a perfectly consistent
// off-origin observation under the optimizer's Hessian encoding.
namespace reusex::geometry {
double plane_factor_consistent_residual();
} // namespace reusex::geometry

// Regression guard for the sign-convention bug: gtsam's OrientedPlane3 stores a
// plane as n.x + d = 0 (distance = d, NOT -d). The optimizer once negated d on
// both the landmark and the measurement, which does NOT cancel (landmark is in
// world, measurement in the optical frame; the factor transforms the landmark
// into the pose frame). A perfectly consistent observation must have ~0
// residual; the buggy encoding produced ~2.1 for the same geometry. This uses
// an off-origin plane (d != 0) because origin-planes hide the sign entirely.
TEST_CASE("PlaneFactorConsistentResidual_OffOriginPlane_IsZero",
          "[plane_graph][optimize]") {
  const double residual = reusex::geometry::plane_factor_consistent_residual();
  REQUIRE_THAT(residual, WithinAbs(0.0, 1e-9));
}

namespace {

// A dense corner: three perpendicular planes sampled as optical-frame surfels,
// with plenty of inliers per plane so RANSAC recovers all three above the
// default --min-plane-inliers threshold. The corner is deliberately placed OFF
// the origin (offset o) so each plane has a non-zero Hessian offset d: a plane
// through the origin has d=0, which hides the OrientedPlane3 sign convention
// and let a real bug pass the earlier version of this test.
FrameSurfels make_corner_frame(int node_id) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const float step = 0.03f;
  const float extent = 1.2f;
  const float o = 1.5f; // corner offset from origin -> non-zero plane offsets
  auto add = [&](float x, float y, float z, float nx, float ny, float nz) {
    PointT p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = p.g = p.b = 200;
    pts->push_back(p);
    NormalT n;
    n.normal_x = nx;
    n.normal_y = ny;
    n.normal_z = nz;
    n.curvature = 0.0f;
    nrm->push_back(n);
  };
  for (float a = 0.0f; a <= extent; a += step) {
    for (float b = 0.0f; b <= extent; b += step) {
      add(o + a, o + b, o, 0.0f, 0.0f, 1.0f); // z=o plane, normal +z
      add(o, o + a, o + b, 1.0f, 0.0f, 0.0f); // x=o plane, normal +x
      add(o + a, o, o + b, 0.0f, 1.0f, 0.0f); // y=o plane, normal +y
    }
  }
  FrameSurfels f;
  f.node_id = node_id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = Eigen::Affine3d::Identity();
  return f;
}

// A single dense square planar patch with normal +z, centred in world at
// (cx, cy, cz), sampled as optical-frame surfels of an identity-pose frame (so
// optical == world here). `half` is the half-side of the patch. Enough points
// to clear the default --min-plane-inliers threshold.
FrameSurfels make_patch_frame(int node_id, float cx, float cy, float cz,
                              float half) {
  auto pts = std::make_shared<Cloud>();
  auto nrm = std::make_shared<CloudN>();
  const float step = 0.03f;
  for (float a = -half; a <= half; a += step) {
    for (float b = -half; b <= half; b += step) {
      PointT p;
      p.x = cx + a;
      p.y = cy + b;
      p.z = cz;
      p.r = p.g = p.b = 200;
      pts->push_back(p);
      NormalT n;
      n.normal_x = 0.0f;
      n.normal_y = 0.0f;
      n.normal_z = 1.0f;
      n.curvature = 0.0f;
      nrm->push_back(n);
    }
  }
  FrameSurfels f;
  f.node_id = node_id;
  f.points = pts;
  f.normals = nrm;
  f.world_pose = Eigen::Affine3d::Identity();
  return f;
}

// Small rigid drift used to corrupt a seed pose.
Eigen::Affine3d drift(double angle, const Eigen::Vector3d &axis,
                      const Eigen::Vector3d &t) {
  Eigen::Affine3d d = Eigen::Affine3d::Identity();
  d.rotate(Eigen::AngleAxisd(angle, axis.normalized()));
  d.translation() = t;
  return d;
}

PlaneGraphOptions test_options() {
  PlaneGraphOptions o;
  o.max_planes_per_frame = 4;
  o.min_plane_inliers = 200;
  o.ransac_distance = 0.02f;
  o.ransac_normal_angle = 20.0f;
  o.ransac_iterations = 200;
  o.assoc_normal_angle = 15.0f;
  o.assoc_distance = 0.20f; // roomy: drift shifts world offsets a little
  o.min_landmark_observations = 2;
  // These synthetic tests probe the plane-landmark objective directly (a shared
  // corner), so they pin the noise knobs the library ships weaker-by-default
  // for real scans: TIGHT plane factors (the objective under test) and LOOSE
  // odometry (which here comes from the *drifted* seed poses and is wrong), so
  // the equilibrium is dominated by the landmarks rather than the seed chain.
  o.plane_sigma_normal = 0.05f;
  o.plane_sigma_distance = 0.03f;
  o.odometry_sigma_rot = 0.25f;
  o.odometry_sigma_trans = 0.5f;
  // Corner geometry: three big perpendicular planes shared by all frames. The
  // overlap/degeneracy hygiene is exercised by dedicated tests below; here it
  // is set permissive so it never interferes with the recovery objective.
  o.assoc_overlap_margin = 2.0f;
  o.min_landmark_spread_ratio = 0.0f;
  o.assoc_rounds = 1;
  o.underconstrained_odom_scale = 1.0f;
  o.max_iterations = 100;
  o.seed = 42;
  return o;
}

// Translational distance between two poses.
double pose_trans_error(const Eigen::Affine3d &a, const Eigen::Affine3d &b) {
  return (a.translation() - b.translation()).norm();
}

// Frobenius rotation error between two poses.
double pose_rot_error(const Eigen::Affine3d &a, const Eigen::Affine3d &b) {
  return (a.rotation() - b.rotation()).norm();
}

} // namespace

TEST_CASE("PlaneGraphOptimizer_DriftedPosesSharedCorner_RecoverTowardTruth",
          "[plane_graph][optimize]") {
  // All frames sit at the same true pose (identity): each sees the exact same
  // corner. This is the cleanest global-consistency signal.
  const Eigen::Affine3d truth = Eigen::Affine3d::Identity();

  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 4; ++i)
    frames.push_back(make_corner_frame(i));

  // Frame 0 is the gauge anchor: seed it at truth so the recovered solution is
  // expressed in the truth frame and errors are directly comparable.
  frames[0].world_pose = truth;
  // Frames 1..3 get distinct drifts (rotation + translation).
  frames[1].world_pose = drift(0.04, {1, 1, 1}, {0.03, -0.02, 0.015}) * truth;
  frames[2].world_pose = drift(0.05, {0, 1, 0}, {-0.025, 0.03, -0.01}) * truth;
  frames[3].world_pose = drift(0.03, {1, 0, 1}, {0.02, 0.02, 0.02}) * truth;

  // Record the drifted seed error for comparison.
  std::vector<double> seed_terr, seed_rerr;
  for (int i = 1; i < 4; ++i) {
    seed_terr.push_back(pose_trans_error(frames[i].world_pose, truth));
    seed_rerr.push_back(pose_rot_error(frames[i].world_pose, truth));
  }

  PlaneGraphOptimizer optimizer(test_options());
  PlaneGraphResult res = optimizer.optimize(frames);

  // The optimizer must have found the shared planes and formed landmarks.
  REQUIRE(res.landmarks >= 3);     // three perpendicular planes
  REQUIRE(res.plane_factors >= 6); // seen by multiple frames
  REQUIRE(res.converged);          // the solve itself must not have failed
  REQUIRE(res.final_error <= res.initial_error);

  // Frame 0 (gauge) must stay put.
  REQUIRE_THAT(pose_trans_error(frames[0].world_pose, truth),
               WithinAbs(0.0, 1e-3));

  // Each drifted frame must end up measurably closer to truth.
  for (int i = 1; i < 4; ++i) {
    const double terr = pose_trans_error(frames[i].world_pose, truth);
    const double rerr = pose_rot_error(frames[i].world_pose, truth);
    INFO("frame " << i << " trans " << seed_terr[i - 1] << " -> " << terr
                  << ", rot " << seed_rerr[i - 1] << " -> " << rerr);
    REQUIRE(terr < seed_terr[i - 1]);
    REQUIRE(rerr < seed_rerr[i - 1]);
    // And close to truth in absolute terms.
    REQUIRE(terr < 0.01);
    REQUIRE(rerr < 0.03);
  }
}

TEST_CASE("PlaneGraphOptimizer_AlreadyConsistentPoses_IsNearNoOp",
          "[plane_graph][optimize]") {
  // All frames already at the true (identity) pose: no drift to correct.
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 3; ++i)
    frames.push_back(make_corner_frame(i));

  std::vector<Eigen::Affine3d> before;
  for (auto &f : frames)
    before.push_back(f.world_pose);

  PlaneGraphOptimizer optimizer(test_options());
  PlaneGraphResult res = optimizer.optimize(frames);

  REQUIRE(res.landmarks >= 3);
  // Poses must not move meaningfully.
  REQUIRE(res.max_pose_shift < 5e-3);
  for (size_t i = 0; i < frames.size(); ++i) {
    REQUIRE_THAT(pose_trans_error(frames[i].world_pose, before[i]),
                 WithinAbs(0.0, 5e-3));
    REQUIRE(pose_rot_error(frames[i].world_pose, before[i]) < 5e-3);
  }
}

// ---------------------------------------------------------------------------
// #225 follow-up: landmark hygiene.
// ---------------------------------------------------------------------------

TEST_CASE("PlaneGraphOptimizer_OffsetParallelWalls_DoesNotAliasLandmarks",
          "[plane_graph][optimize][hygiene]") {
  // Two frames each see a co-normal (normal +z, offset d=0) patch, but the two
  // patches are far apart IN-PLANE (centres 10 m apart along x). They agree in
  // normal AND offset — the old greedy front-end would merge them into a single
  // landmark, aliasing two distinct surfaces. The overlap gate must keep them
  // separate: with min_observations=1 that means TWO kept landmarks, not one.
  std::vector<FrameSurfels> frames;
  frames.push_back(make_patch_frame(0, 0.0f, 0.0f, 0.0f, 1.0f));
  frames.push_back(make_patch_frame(1, 10.0f, 0.0f, 0.0f, 1.0f));

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 1;    // each patch seen by exactly one frame
  o.assoc_normal_angle = 15.0f;       // normals are identical anyway
  o.assoc_distance = 0.5f;            // offsets are identical (both d=0)
  o.assoc_overlap_margin = 0.30f;     // patches are 10 m apart -> no overlap
  o.min_landmark_spread_ratio = 0.0f; // disable degeneracy check here
  o.assoc_rounds = 1;

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames);

  INFO("landmarks=" << res.landmarks
                    << " rejected_overlap=" << res.landmarks_rejected_overlap);
  REQUIRE(res.landmarks == 2);                  // NOT merged
  REQUIRE(res.landmarks_rejected_overlap >= 1); // the overlap gate fired

  // Sanity: with the overlap gate OFF the SAME geometry collapses to one
  // landmark (proves the gate, not the geometry, is doing the separating).
  o.assoc_overlap_margin = 0.0f; // disable overlap gate
  std::vector<FrameSurfels> frames2;
  frames2.push_back(make_patch_frame(0, 0.0f, 0.0f, 0.0f, 1.0f));
  frames2.push_back(make_patch_frame(1, 10.0f, 0.0f, 0.0f, 1.0f));
  PlaneGraphOptimizer optimizer2(o);
  PlaneGraphResult res2 = optimizer2.optimize(frames2);
  REQUIRE(res2.landmarks == 1); // aliased when the gate is off
}

TEST_CASE("PlaneGraphOptimizer_NearCollinearCentroids_"
          "RejectsDegenerateLandmark",
          "[plane_graph][optimize][hygiene]") {
  // Four frames see adjacent strips of the SAME z=0 plane whose centroids lie
  // on a straight line (along x). Merged into one landmark, those centroids are
  // near-collinear: a rotation about that line is unobservable, so the landmark
  // must be rejected (kept landmarks == 0). The generous overlap margin lets
  // the adjacent strips merge so the degeneracy test is what does the work.
  std::vector<FrameSurfels> collinear;
  for (int i = 0; i < 4; ++i)
    collinear.push_back(
        make_patch_frame(i, static_cast<float>(i) * 0.8f, 0.0f, 0.0f, 0.6f));

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 3;
  o.assoc_normal_angle = 15.0f;
  o.assoc_distance = 0.5f;
  o.assoc_overlap_margin = 1.0f; // adjacent strips overlap -> one landmark
  o.min_landmark_spread_ratio = 0.15f; // require a genuine 2D patch
  o.assoc_rounds = 1;

  PlaneGraphOptimizer opt_c(o);
  PlaneGraphResult res_c = opt_c.optimize(collinear);
  INFO("collinear landmarks=" << res_c.landmarks << " degenerate_rejected="
                              << res_c.landmarks_rejected_degenerate);
  REQUIRE(res_c.landmarks == 0);
  REQUIRE(res_c.landmarks_rejected_degenerate >= 1);

  // Control: the same strips arranged in a 2x2 GRID (centroids span a plane)
  // form a well-conditioned landmark that is kept.
  std::vector<FrameSurfels> grid;
  int id = 0;
  for (int gx = 0; gx < 2; ++gx)
    for (int gy = 0; gy < 2; ++gy)
      grid.push_back(make_patch_frame(id++, static_cast<float>(gx) * 0.8f,
                                      static_cast<float>(gy) * 0.8f, 0.0f,
                                      0.6f));
  PlaneGraphOptimizer opt_g(o);
  PlaneGraphResult res_g = opt_g.optimize(grid);
  INFO("grid landmarks=" << res_g.landmarks);
  REQUIRE(res_g.landmarks >= 1);
}

TEST_CASE("PlaneGraphOptimizer_HeavyDriftAlternatingRounds_"
          "ConvergesCloserThanSingleRound",
          "[plane_graph][optimize][hygiene]") {
  // Guards that additional association+optimization rounds produce a strictly
  // better solution than a single round from the same heavy starting drift.
  //
  // The mechanism (see #383): each round builds a fresh factor graph
  // re-linearised at the poses the previous round produced. A single round
  // capped at max_iterations cannot fully converge from a ~2× larger drift;
  // three rounds of fresh re-linearisation compound to a substantially smaller
  // residual. All three plane associations are correct from round 1 onwards
  // (drift < assoc_distance), so the improvement is entirely EM-style
  // re-linearisation rather than association recovery.
  const Eigen::Affine3d truth = Eigen::Affine3d::Identity();

  auto build = [&]() {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < 4; ++i)
      frames.push_back(make_corner_frame(i));
    frames[0].world_pose = truth; // gauge anchor at truth
    // ~2× heavier than the basic DriftedPosesSharedCorner fixture so a single
    // capped round cannot reach the global minimum.
    frames[1].world_pose = drift(0.18, {1, 1, 1}, {0.16, -0.13, 0.11}) * truth;
    frames[2].world_pose = drift(0.20, {0, 1, 0}, {-0.13, 0.17, -0.11}) * truth;
    frames[3].world_pose = drift(0.16, {1, 0, 1}, {0.12, 0.12, 0.15}) * truth;
    return frames;
  };

  auto total_error = [&](const std::vector<FrameSurfels> &frames) {
    double e = 0.0;
    for (int i = 1; i < 4; ++i)
      e += pose_trans_error(frames[i].world_pose, truth) +
           pose_rot_error(frames[i].world_pose, truth);
    return e;
  };

  PlaneGraphOptions base = test_options();
  base.min_landmark_observations = 2;
  base.assoc_overlap_margin = 2.0f;      // corner planes are large & shared
  base.min_landmark_spread_ratio = 0.0f; // corner geometry, not under test here
  base.underconstrained_odom_scale = 1.0f; // isolate the rounds effect
  // Cap per-round LM iterations so a single round cannot fully converge from
  // the heavy drift above. Three rounds of 5 iterations (with fresh
  // re-linearisation between them) achieve a substantially better solution.
  base.max_iterations = 5;

  PlaneGraphOptions one = base;
  one.assoc_rounds = 1;
  std::vector<FrameSurfels> f1 = build();
  PlaneGraphResult r1 = PlaneGraphOptimizer(one).optimize(f1);
  const double e1 = total_error(f1);

  PlaneGraphOptions many = base;
  many.assoc_rounds = 3;
  many.assoc_round_tol = 1e-5f; // do not stop early
  std::vector<FrameSurfels> f3 = build();
  PlaneGraphResult r3 = PlaneGraphOptimizer(many).optimize(f3);
  const double e3 = total_error(f3);

  INFO("1-round total err " << e1 << " vs 3-round " << e3
                            << " (rounds=" << r3.rounds << ")");
  REQUIRE(r1.rounds == 1);
  REQUIRE(r3.rounds >= 2);
  // The 3-round run must end strictly closer to truth than the 1-round run.
  REQUIRE(e3 < e1);
}

TEST_CASE("PlaneGraphOptimizer_NoSharedLandmark_LeavesPosesUnchanged",
          "[plane_graph][optimize]") {
  // Two frames but require more observations than any landmark can get: the
  // optimizer must refuse to move the poses (no silent guessing).
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 2; ++i)
    frames.push_back(make_corner_frame(i));
  frames[1].world_pose = drift(0.05, {1, 0, 0}, {0.05, 0.0, 0.0});

  std::vector<Eigen::Affine3d> before;
  for (auto &f : frames)
    before.push_back(f.world_pose);

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 10; // impossible with 2 frames

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames);

  REQUIRE(res.landmarks == 0);
  for (size_t i = 0; i < frames.size(); ++i)
    REQUIRE((frames[i].world_pose.matrix() - before[i].matrix()).norm() == 0.0);
}

TEST_CASE("PlaneGraphOptimizer_CorrectLoopEdgeNoPlaneFactors_"
          "PullsDriftedFrameTowardTruth",
          "[plane_graph][optimize][loop_closure]") {
  // Isolate the P2 loop-edge machinery from the plane objective: require more
  // observations than any landmark can get (so NO plane factors form), leaving
  // only the gauge prior, the (wrong, drift-encoding) odometry, and one correct
  // wide-baseline loop edge. Both frames' truth pose is identity, so the
  // correct relative pose X_0^-1 X_1 is the identity — a loop edge that must
  // pull the drifted frame 1 back toward its true pose.
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 2; ++i)
    frames.push_back(make_corner_frame(i));
  const Eigen::Affine3d truth = Eigen::Affine3d::Identity();
  frames[1].world_pose = drift(0.05, {1, 0, 0}, {0.05, 0.0, 0.0});
  const double seed_err = pose_trans_error(frames[1].world_pose, truth);

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 10; // impossible with 2 frames -> 0 landmarks
  // Plain LM: this test probes the loop-edge factor mechanics directly. GNC
  // would see the single edge's drift-sized initial residual as an outlier and
  // down-weight it (the same informative-residual effect gnc_inlier_cost tunes
  // for plane factors); robustness of loop edges under GNC is a system concern,
  // not what this unit isolates.
  o.use_gnc = false;

  // A correct, tightly-trusted loop edge: pose(1) = pose(0) * I.
  LoopEdge e;
  e.i = 0;
  e.j = 1;
  e.T_ij = Eigen::Matrix4d::Identity();
  e.sigma_rot = 0.01;
  e.sigma_trans = 0.01;
  e.inliers = 100;

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames, {e});

  REQUIRE(res.landmarks == 0);  // no plane factors: the loop edge did the work
  REQUIRE(res.loop_edges == 1); // the edge entered the graph
  REQUIRE(res.converged);
  // Frame 1 must land much closer to truth than the drifted seed.
  const double after_err = pose_trans_error(frames[1].world_pose, truth);
  REQUIRE(after_err < 0.5 * seed_err);
}

TEST_CASE(
    "PlaneGraphOptimizer_TrustedLoopEdge_BoundsCorrectionInsteadOfBlindTrust",
    "[plane_graph][optimize][loop_closure]") {
  // Regression guard for the --loop-trust semantics (PR #228 review).
  //
  // The original implementation marked a trusted loop edge as a GNC KNOWN
  // INLIER and wrapped it in a Huber kernel. GTSAM's GncOptimizer constructor
  // strips noiseModel::Robust from every factor it is handed, so the Huber
  // never existed: a trusted edge was an unbounded Gaussian known-inlier, and
  // the documented outlier bound was fiction. The fix keeps trusted loop edges
  // as GNC candidates and instead raises THEIR TLS inlier threshold to
  // loop_trust_inlier_cost — generous enough that a genuine correction still
  // applies, finite enough that a grossly-wrong edge is truncated.
  //
  // Setup: six frames whose true pose is identity, chained by TIGHT odometry.
  // A loop edge between the first and last frame is the only thing that can
  // disagree with that chain, so what GNC does to it is directly observable in
  // max_pose_shift.
  constexpr int kFrames = 6;

  auto build = [&](const Eigen::Affine3d &last_pose) {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < kFrames; ++i)
      frames.push_back(make_corner_frame(i));
    frames[kFrames - 1].world_pose = last_pose;
    return frames;
  };

  PlaneGraphOptions base = test_options();
  base.min_landmark_observations = 10; // no plane factors: isolate the edge
  base.use_gnc = true;
  base.loop_edges_trusted = true;
  // Tight odometry: the seed chain is what a bogus loop edge has to fight, and
  // without it any edge trivially "wins" whatever GNC decides.
  base.odometry_sigma_rot = 0.005f;
  base.odometry_sigma_trans = 0.005f;

  SECTION("a grossly-wrong trusted edge is truncated, not applied") {
    // 20 m of nonsense. Its residual stays far above loop_trust_inlier_cost at
    // every iterate (the odometry chain refuses to stretch that far), so
    // GNC-TLS must drive its weight to ~0 and leave the trajectory on the seed.
    LoopEdge gross;
    gross.i = 0;
    gross.j = kFrames - 1;
    gross.T_ij = Eigen::Matrix4d::Identity();
    gross.T_ij(0, 3) = 20.0;
    gross.sigma_rot = 0.05;
    gross.sigma_trans = 0.05;
    gross.inliers = 100;

    std::vector<FrameSurfels> frames = build(Eigen::Affine3d::Identity());
    PlaneGraphResult res = PlaneGraphOptimizer(base).optimize(frames, {gross});
    REQUIRE(res.converged);
    REQUIRE(res.loop_edges == 1);
    INFO("bounded max_pose_shift = " << res.max_pose_shift << " m");
    REQUIRE(res.max_pose_shift < 0.05);

    // Control: with an effectively INFINITE threshold — which is what the old
    // known-inlier-plus-stripped-Huber path amounted to — the very same edge
    // drags the trajectory by (tens of) centimetres. This is the behaviour the
    // finite threshold is there to prevent, so it must be reproducible.
    PlaneGraphOptions unbounded = base;
    unbounded.loop_trust_inlier_cost = 1e12f;
    std::vector<FrameSurfels> frames_u = build(Eigen::Affine3d::Identity());
    PlaneGraphResult res_u =
        PlaneGraphOptimizer(unbounded).optimize(frames_u, {gross});
    REQUIRE(res_u.converged);
    INFO("unbounded max_pose_shift = " << res_u.max_pose_shift << " m");
    REQUIRE(res_u.max_pose_shift > 10.0 * res.max_pose_shift);
    REQUIRE(res_u.max_pose_shift > 0.1);
  }

  SECTION("a correct trusted edge still applies its drift correction") {
    // The point of --loop-trust: a genuine loop edge whose measurement matches
    // the TRUTH (identity) must pull the drifted last frame back, not be
    // discarded as an outlier.
    const Eigen::Affine3d truth = Eigen::Affine3d::Identity();
    const Eigen::Affine3d drifted = drift(0.05, {1, 0, 0}, {0.30, 0.0, 0.0});
    const double seed_err = pose_trans_error(drifted, truth);

    LoopEdge good;
    good.i = 0;
    good.j = kFrames - 1;
    good.T_ij = Eigen::Matrix4d::Identity();
    good.sigma_rot = 0.02;
    good.sigma_trans = 0.02;
    good.inliers = 100;

    PlaneGraphOptions o = base;
    // Loosen odometry so the drift can redistribute (exactly what the CLI help
    // tells the user to do alongside --loop-trust).
    o.odometry_sigma_rot = 0.05f;
    o.odometry_sigma_trans = 0.05f;

    std::vector<FrameSurfels> frames = build(drifted);
    PlaneGraphResult res = PlaneGraphOptimizer(o).optimize(frames, {good});
    REQUIRE(res.converged);
    REQUIRE(res.loop_edges == 1);
    REQUIRE(res.landmarks == 0); // the loop edge did all the work

    const double after =
        pose_trans_error(frames[kFrames - 1].world_pose, truth);
    INFO("last frame " << seed_err << " m -> " << after << " m from truth");
    REQUIRE(after < 0.3 * seed_err);
  }
}

TEST_CASE("PlaneGraphOptimizer_OutOfRangeLoopEdge_IsSkipped",
          "[plane_graph][optimize][loop_closure]") {
  // A loop edge whose endpoints do not exist must be skipped, not crash, and
  // leave poses untouched when there is nothing else to constrain them.
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 2; ++i)
    frames.push_back(make_corner_frame(i));
  frames[1].world_pose = drift(0.05, {1, 0, 0}, {0.05, 0.0, 0.0});
  std::vector<Eigen::Affine3d> before;
  for (auto &f : frames)
    before.push_back(f.world_pose);

  PlaneGraphOptions o = test_options();
  o.min_landmark_observations = 10; // no landmarks

  LoopEdge bad;
  bad.i = 0;
  bad.j = 7; // out of range (only 2 frames)
  bad.T_ij = Eigen::Matrix4d::Identity();

  PlaneGraphOptimizer optimizer(o);
  PlaneGraphResult res = optimizer.optimize(frames, {bad});

  REQUIRE(res.loop_edges == 0); // the invalid edge was skipped
  for (size_t i = 0; i < frames.size(); ++i)
    REQUIRE((frames[i].world_pose.matrix() - before[i].matrix()).norm() == 0.0);
}

// --- Per-observation plane noise models (#225) ------------------------------
//
// plane_fit_sigmas itself is covered exhaustively in test_plane_fit_sigmas.cpp.
// What these cases guard is the WIRING: that the scales computed from the fit
// statistics reach the factors without breaking the optimizer, and that all
// three models remain interchangeable on the same fixture. A NaN or zero scale
// would not fail to compile — it would silently produce a graph whose plane
// factors are infinitely trusted or entirely ignored, which is exactly the
// class of bug that cost this issue two rounds of bad numbers.

TEST_CASE("PlaneGraphOptimizer_EveryPlaneNoiseModel_RecoversDrift",
          "[plane_graph][optimize][noise]") {
  const Eigen::Affine3d truth = Eigen::Affine3d::Identity();

  auto build = [&]() {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < 4; ++i)
      frames.push_back(make_corner_frame(i));
    frames[0].world_pose = truth; // gauge anchor
    frames[1].world_pose = drift(0.04, {1, 1, 1}, {0.03, -0.02, 0.015}) * truth;
    frames[2].world_pose =
        drift(0.05, {0, 1, 0}, {-0.025, 0.03, -0.01}) * truth;
    frames[3].world_pose = drift(0.03, {1, 0, 1}, {0.02, 0.02, 0.02}) * truth;
    return frames;
  };

  // Seed error, identical for every model since the fixture is rebuilt.
  std::vector<double> seed_terr;
  {
    auto f = build();
    for (int i = 1; i < 4; ++i)
      seed_terr.push_back(pose_trans_error(f[i].world_pose, truth));
  }

  auto run = [&](PlaneNoiseModel model) {
    auto frames = build();
    PlaneGraphOptions o = test_options();
    o.plane_noise_model = model;
    PlaneGraphOptimizer optimizer(o);
    PlaneGraphResult res = optimizer.optimize(frames);

    REQUIRE(res.converged);
    REQUIRE(res.landmarks >= 3);
    REQUIRE(res.final_error <= res.initial_error);
    // Every drifted frame moves toward truth, whichever model weighted it.
    for (int i = 1; i < 4; ++i) {
      const double terr = pose_trans_error(frames[i].world_pose, truth);
      INFO("frame " << i << " trans " << seed_terr[i - 1] << " -> " << terr);
      REQUIRE(terr < seed_terr[i - 1]);
      REQUIRE(terr < 0.01);
    }
    return res;
  };

  SECTION("uniform") {
    const auto res = run(PlaneNoiseModel::uniform);
    // No weighting means nothing to clamp and no fit statistics to report.
    REQUIRE(res.plane_noise_clamped_low == 0);
    REQUIRE(res.plane_noise_clamped_high == 0);
    REQUIRE(res.median_fit_sigma_normal == 0.0);
  }

  SECTION("inlier_count (legacy)") { run(PlaneNoiseModel::inlier_count); }

  SECTION("fit_geometry (default)") {
    const auto res = run(PlaneNoiseModel::fit_geometry);
    // The run must have produced usable, strictly positive fit statistics —
    // a zero here would mean the detections carried no residual/extent and the
    // scales silently collapsed to the clamp.
    REQUIRE(res.median_fit_sigma_normal > 0.0);
    REQUIRE(res.median_fit_sigma_distance > 0.0);
    REQUIRE(std::isfinite(res.median_fit_sigma_normal));
    REQUIRE(std::isfinite(res.median_fit_sigma_distance));
  }
}

TEST_CASE("PlaneGraphOptimizer_FitGeometryNoiseModel_IsDeterministic",
          "[plane_graph][optimize][noise]") {
  // STANDARDS §6: the noise scales are a pure function of the (seeded) RANSAC
  // fits, so two identical runs must agree bit for bit — otherwise none of the
  // before/after measurements in #225 mean anything.
  auto build = [&]() {
    std::vector<FrameSurfels> frames;
    for (int i = 0; i < 3; ++i)
      frames.push_back(make_corner_frame(i));
    frames[1].world_pose = drift(0.04f, {1, 1, 1}, {0.03f, -0.02f, 0.015f});
    frames[2].world_pose = drift(0.05f, {0, 1, 0}, {-0.025f, 0.03f, -0.01f});
    return frames;
  };

  PlaneGraphOptions o = test_options();
  o.plane_noise_model = PlaneNoiseModel::fit_geometry;

  auto a = build();
  auto b = build();
  PlaneGraphResult ra = PlaneGraphOptimizer(o).optimize(a);
  PlaneGraphResult rb = PlaneGraphOptimizer(o).optimize(b);

  REQUIRE(ra.median_fit_sigma_normal == rb.median_fit_sigma_normal);
  REQUIRE(ra.median_fit_sigma_distance == rb.median_fit_sigma_distance);
  REQUIRE(ra.plane_noise_clamped_low == rb.plane_noise_clamped_low);
  REQUIRE(ra.plane_noise_clamped_high == rb.plane_noise_clamped_high);
  REQUIRE(ra.final_error == rb.final_error);
  for (size_t i = 0; i < a.size(); ++i)
    REQUIRE((a[i].world_pose.matrix() - b[i].world_pose.matrix()).norm() ==
            0.0f);
}

// ── Plane-term authority knob (#225 §9) ─────────────────────────────────────
// `plane_sigma_scale` multiplies both plane sigmas, so the plane term's weight
// in the objective goes as 1/scale^2; `use_plane_factors == false` is that
// knob's infinite limit taken exactly. Both must be *strictly* opt-in: the
// shipped default path has to stay bit-identical, or every before/after number
// recorded in #225 becomes uninterpretable.

namespace {

// Three frames seeing the same corner, frames 1..2 drifted off the gauge.
std::vector<FrameSurfels> drifted_corner_frames() {
  std::vector<FrameSurfels> frames;
  for (int i = 0; i < 3; ++i)
    frames.push_back(make_corner_frame(i));
  frames[1].world_pose = drift(0.04, {1, 1, 1}, {0.03, -0.02, 0.015});
  frames[2].world_pose = drift(0.05, {0, 1, 0}, {-0.025, 0.03, -0.01});
  return frames;
}

} // namespace

TEST_CASE("PlaneGraph plane_sigma_scale 1.0 is the shipped default exactly",
          "[plane_graph][optimize][plane_weight]") {
  // The default-preservation guarantee. 1.0 must not merely be "close to" the
  // unscaled path — a float multiply by 1.0f is exact, and this pins that.
  PlaneGraphOptions def = test_options();
  PlaneGraphOptions explicit_one = test_options();
  explicit_one.plane_sigma_scale = 1.0f;

  auto a = drifted_corner_frames();
  auto b = drifted_corner_frames();
  PlaneGraphResult ra = PlaneGraphOptimizer(def).optimize(a);
  PlaneGraphResult rb = PlaneGraphOptimizer(explicit_one).optimize(b);

  REQUIRE(ra.plane_factors == rb.plane_factors);
  REQUIRE(ra.final_error == rb.final_error);
  for (size_t i = 0; i < a.size(); ++i)
    REQUIRE((a[i].world_pose.matrix() - b[i].world_pose.matrix()).norm() ==
            0.0f);
}

TEST_CASE("PlaneGraph plane_sigma_scale weakens the correction monotonically",
          "[plane_graph][optimize][plane_weight]") {
  // The knob has to be a real authority dial, not a no-op: a larger sigma
  // scale is a weaker plane term, so less of the seed drift gets corrected and
  // the residual error against truth must not decrease. This is what makes a
  // sweep over the knob a meaningful experiment.
  const Eigen::Affine3d truth = Eigen::Affine3d::Identity();
  double prev_err = -1.0;
  for (const float scale : {1.0f, 10.0f, 100.0f, 1000.0f}) {
    PlaneGraphOptions o = test_options();
    o.plane_sigma_scale = scale;
    auto frames = drifted_corner_frames();
    const double seed_err = pose_trans_error(frames[1].world_pose, truth);
    PlaneGraphResult res = PlaneGraphOptimizer(o).optimize(frames);
    const double err = pose_trans_error(frames[1].world_pose, truth);
    INFO("scale " << scale << ": seed " << seed_err << " -> " << err);
    REQUIRE(res.plane_factors > 0); // the factors still exist, just weaker
    if (prev_err >= 0.0)
      REQUIRE(err >= prev_err - 1e-6);
    prev_err = err;
  }
  // ... and the weakest setting must have given up most of the correction.
  auto frames = drifted_corner_frames();
  const double seed_err = pose_trans_error(frames[1].world_pose, truth);
  REQUIRE(prev_err > 0.5 * seed_err);
}

TEST_CASE("PlaneGraph --no-plane-factors leaves the seed trajectory alone",
          "[plane_graph][optimize][plane_weight]") {
  // The "plane term off" endpoint: detection and association still run and are
  // still reported (so a sweep row stays comparable), but the graph is
  // odometry + gauge prior only, which is exactly satisfied by the seed poses.
  auto frames = drifted_corner_frames();
  std::vector<Eigen::Matrix4d> seed;
  for (const auto &f : frames)
    seed.push_back(f.world_pose.matrix());

  PlaneGraphOptions o = test_options();
  o.use_plane_factors = false;
  PlaneGraphResult res = PlaneGraphOptimizer(o).optimize(frames);

  REQUIRE(res.plane_factors == 0);
  REQUIRE(res.planes_detected > 0); // still measured and reported
  REQUIRE(res.landmarks >= 3);      // association still runs
  REQUIRE(res.converged);
  for (size_t i = 0; i < frames.size(); ++i) {
    INFO("frame " << i);
    REQUIRE((frames[i].world_pose.matrix() - seed[i]).norm() < 1e-4);
  }
}

TEST_CASE("PlaneGraph --no-plane-factors ignores the plane sigmas entirely",
          "[plane_graph][optimize][plane_weight]") {
  // Off means off: with no plane factors in the graph, the plane sigmas (and
  // hence the sigma scale) cannot influence the result at all. Guards against
  // a future refactor that keeps some plane influence alive on this path.
  auto a = drifted_corner_frames();
  auto b = drifted_corner_frames();

  PlaneGraphOptions o1 = test_options();
  o1.use_plane_factors = false;
  PlaneGraphOptions o2 = o1;
  o2.plane_sigma_scale = 1000.0f;
  o2.plane_noise_model = PlaneNoiseModel::fit_geometry;

  PlaneGraphResult ra = PlaneGraphOptimizer(o1).optimize(a);
  PlaneGraphResult rb = PlaneGraphOptimizer(o2).optimize(b);

  REQUIRE(ra.final_error == rb.final_error);
  for (size_t i = 0; i < a.size(); ++i)
    REQUIRE((a[i].world_pose.matrix() - b[i].world_pose.matrix()).norm() ==
            0.0f);
}

// Integration review finding 4: the CLI/library default parity test
// (CliCommandRoundTrip_OptimizeFlagDefaults_MatchTheStageBase,
// tests/unit/rux_app/test_cli_command_roundtrip.cpp) now relies on
// PlaneGraphOptions::operator== (and its nested option structs') to compare
// every field at once, rather than a hand-maintained list that had already
// fallen behind several LoopClosureOptions fields. This pins that the
// defaulted comparison really is memberwise and really does recurse into
// the nested structs, on a field each of those three structs' blind spot.
TEST_CASE("PlaneGraphOptions operator== is memberwise through nested structs",
          "[plane_graph][parity]") {
  const PlaneGraphOptions a;
  PlaneGraphOptions b = a;
  CHECK(a == b);

  SECTION("a top-level field") {
    b.max_planes_per_frame = a.max_planes_per_frame + 1;
    CHECK_FALSE(a == b);
  }
  SECTION("a surfel field the old hand list did cover") {
    b.surfel.sampling_factor = a.surfel.sampling_factor + 1;
    CHECK_FALSE(a == b);
  }
}
