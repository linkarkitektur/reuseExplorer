// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Walls with a measured thickness and a bounded wall graph (solid-model
/// WP3, reusex-z1v.5). Sub-steps as free, unit-testable functions:
///
///  1. axis_families(): the directions walls run in, per storey (W3.1).
///  2. wall_faces(): faces (one-sided sheets of wall points) and their runs
///     along the axis, from the offset profile of each family (W3.2).
///
/// Everything is in the level frame of one storey (core::StoreyRecord::
/// level_transform): z up, the storey's floor at its z_floor. Inputs are
/// records (core::PlaneClassRecord, core::StoreyRecord) and clouds, never
/// segmentation types: reconstruction must not link segmentation (STANDARDS
/// §1); the stage that composes them is pipeline's.
///
/// Deterministic (STANDARDS §6): no randomness, inputs visited in index
/// order, every iteration bounded.

#include "reusex/core/solid_model.hpp"
#include "reusex/types/point_types.hpp"

#include <Eigen/Core>

#include <map>
#include <utility>
#include <vector>

namespace reusex::geometry {

/// Every tunable of the wall stage (STANDARDS §4).
struct WallOptions {
  // --- W3.1 axis families -------------------------------------------------
  /// Extra families allowed beyond the dominant orthogonal pair (decision D6:
  /// buildings are mostly orthogonal, but not always).
  int max_families = 2;
  /// A vertical-plane normal farther than this from both axes of the
  /// dominant pair is a candidate for an extra family; candidates whose
  /// angles are farther apart than this start a new cluster [deg]. Also the
  /// inlier window when the dominant pair is refined.
  double family_angle_deg = 5.0;
  /// An extra family needs at least this share of the total vertical-plane
  /// inlier mass.
  double min_family_share = 0.05;
  /// Use only planes whose role is "wall" for the families (falling back to
  /// every vertical plane when the storey has none).
  bool wall_role_only = true;
  /// Refinements of the dominant pair over its own inliers (bounded). The
  /// inlier window starts at family_angle_deg and halves each round down to
  /// family_refine_deg [deg].
  int family_refine_iterations = 4;
  double family_refine_deg = 1.0;

  // --- W3.2 wall faces ---------------------------------------------------
  /// A wall point belongs to the family whose axis its plane normal is
  /// nearest to, if within this angle [deg]; otherwise it is no evidence.
  double assign_angle_deg = 10.0;
  /// Square a wall plane within this of its family axis onto the axis,
  /// about the plane's centroid (W3.4's squaring, done on the evidence so a
  /// slightly drifted wall still gives one sharp profile peak) [deg].
  /// Points with no plane id are never squared.
  double square_deg = 8.0;
  /// Use the sign of the plane normal (it faces the sensor that saw it,
  /// CONTRACTS.md `clouds`): points seen from +d and from -d get separate
  /// offset profiles, so the two faces of a thin wall never merge. Off for
  /// legacy clouds whose normals faced the world origin.
  bool use_normal_sign = true;
  /// Offset-profile bin [m]; bins are anchored at the level-frame origin.
  double bin_m = 0.01;
  /// A profile peak (after the [1,2,3,2,1]/9 smoothing) is at least this
  /// many points per bin...
  double min_peak_points = 10.0;
  /// ...stands out from the deeper of its two bases by at least this share
  /// of its height (topographic prominence)...
  double min_peak_prominence_share = 0.3;
  /// ...and is at least this far from a higher accepted peak [m].
  double min_peak_separation_m = 0.03;
  /// Half-width of the slab of points a face takes around its peak [m]
  /// (narrowed to half the distance to a neighbouring peak).
  double face_window_m = 0.03;
  /// A cell_m wide column of a face is evidence ("solid") when its points
  /// cover at least this share of the storey's cell rows: a wall shows over
  /// much of its height, a row of furniture ends at one offset does not.
  double min_column_share = 0.3;
  /// A face splits into runs along the axis where no solid column is seen
  /// for longer than this [m]...
  double run_gap_m = 0.6;
  /// ...and runs shorter than this [m] or with fewer points are dropped.
  double min_run_m = 0.5;
  int min_run_points = 30;
  /// Side of the (s, z) occupancy cells [m], anchored at the frame origin.
  double cell_m = 0.25;
  /// wall_points(): points this far below the storey's floor / above its
  /// ceiling still belong to it [m].
  double storey_margin_m = 0.1;
};

/// One direction family: walls whose normal is (anti)parallel to @ref d.
struct AxisFamily {
  int index = 0;
  /// Unit horizontal wall normal in the level frame; angle in [0, 180) deg.
  Eigen::Vector2d d = Eigen::Vector2d::UnitX();
  double angle_deg = 0.0; ///< atan2(d.y, d.x) in degrees, [0, 180)
  double weight = 0.0;    ///< inlier mass of the normals assigned to it
  bool dominant = false;  ///< one of the orthogonal pair (index 0 and 1)
};

/// A horizontal plane normal with a weight (inlier count).
struct WeightedNormal {
  Eigen::Vector2d n = Eigen::Vector2d::UnitX(); ///< need not be unit length
  double weight = 1.0;
};

/// The axis families of a set of horizontal wall normals (W3.1).
///
/// The dominant orthogonal pair comes from the weighted circular mean of 4θ
/// (θ = the normal's angle), so normals at θ, θ+90 and θ+180 all vote for
/// the same pair. It is seeded at the weighted mode (the angle mod 90 with
/// the most mass within family_angle_deg) and refined as the 4θ mean of the
/// normals within a window that narrows from family_angle_deg to
/// WallOptions::family_refine_deg. Normals farther from the pair are
/// sorted by angle (mod 180) and clustered where neighbours are at most
/// family_angle_deg apart; a cluster becomes an extra family when its weight
/// clears WallOptions::min_family_share of the total, heaviest first, up to
/// WallOptions::max_families. Family 0 is the dominant axis with the smaller
/// angle in [0, 90), family 1 is it + 90; extra families follow in order of
/// weight. Each family's weight is the mass of the normals nearest to it.
///
/// Returns an empty vector when there is no normal with positive weight.
std::vector<AxisFamily>
axis_families(const std::vector<WeightedNormal> &normals,
              const WallOptions &opt = {});

/// The axis families of one storey from the stored plane classes: the
/// vertical planes of @p storey (PlaneClassRecord::storey == storey.idx),
/// their normals (@p plane_normals, index = plane label - 1) rotated into the
/// storey's level frame, weighted by PlaneClassRecord::point_count. With
/// WallOptions::wall_role_only, only role "wall" planes vote unless the
/// storey has none.
///
/// @throws std::invalid_argument when a record's plane label has no normal,
///         or the storey's level_transform is not finite.
std::vector<AxisFamily>
axis_families(const std::vector<core::PlaneClassRecord> &planes,
              const CloudN &plane_normals, const core::StoreyRecord &storey,
              const WallOptions &opt = {});

/// The evidence of one storey, in its level frame (see wall_points()).
struct WallPoints {
  /// Points of wall-role planes: the wall evidence...
  std::vector<Eigen::Vector3d> points;
  /// ...the horizontal part of their plane's normal (index-aligned; need not
  /// be unit length; it faces the sensor that saw the plane)...
  std::vector<Eigen::Vector2d> normals;
  /// ...and their plane label (index-aligned, or empty: no plane ids). Points
  /// of one plane are squared together (WallOptions::square_deg); -1 = none.
  std::vector<int> plane;
  /// Every other point of the storey (floors, furniture, unsegmented): what
  /// the valley and external tests (W3.3) look at.
  std::vector<Eigen::Vector3d> others;
  double z_floor = 0.0, z_ceiling = 0.0; ///< the storey's height range
};

/// One point of a face in its family's coordinates: s = p . t along the axis
/// (t = d rotated +90 deg), z, and the offset o = p . d.
struct FaceSample {
  double s = 0.0, z = 0.0, o = 0.0;
};

/// A cell of the (s, z) occupancy grid: (floor(s / cell_m), floor(z / cell_m))
/// -- anchored at the level-frame origin, so a re-run lands on the same grid.
using FaceCell = std::pair<int, int>;

/// One face: a sheet of wall points seen from one side, at one offset, over
/// one contiguous run along the axis.
struct WallFace {
  int family = 0; ///< AxisFamily::index
  /// +1: seen from +d (its outward normal points along d), -1: from -d,
  /// 0: unknown (WallOptions::use_normal_sign off).
  int side = 0;
  double offset = 0.0;       ///< median offset of its points [m]
  double s0 = 0.0, s1 = 0.0; ///< run along the axis [m]
  double z0 = 0.0, z1 = 0.0; ///< height range of its points [m]
  int point_count = 0;
  /// Smoothed profile height at its peak [points per bin].
  double peak_height = 0.0;
  std::map<FaceCell, int> cells;   ///< points per occupied (s, z) cell
  std::vector<FaceSample> samples; ///< its points, sorted by (s, z, o)
};

/// The faces of a storey (W3.2). Per family, and per side when the normal
/// sign is used, a 1 cm histogram of the wall points' offsets o = p . d
/// (bins anchored at the level-frame origin), smoothed [1,2,3,2,1]/9; each
/// peak with enough height, prominence and separation is a face taking the
/// points within WallOptions::face_window_m of it; a face splits into runs
/// where no solid column (WallOptions::min_column_share of the storey's
/// height seen) occurs for longer than WallOptions::run_gap_m, and runs
/// shorter than WallOptions::min_run_m are dropped. A wall point goes to the
/// family its plane normal is nearest to (within assign_angle_deg); a plane
/// within square_deg of its family is squared onto it about its centroid.
///
/// Sorted by (family, offset, s0, side). Deterministic: the result does not
/// depend on the order of the points.
///
/// @throws std::invalid_argument when @p wp's normals (or non-empty plane
///         ids) are not index-aligned with its points.
std::vector<WallFace> wall_faces(const WallPoints &wp,
                                 const std::vector<AxisFamily> &families,
                                 const WallOptions &opt = {});

/// The evidence of one storey from the clouds: every finite point of
/// @p cloud whose level-frame height lies within the storey (with
/// WallOptions::storey_margin_m), taken into @p storey's level frame; a point
/// whose plane (@p planes, label = plane_label) has role "wall" in
/// @p plane_classes is wall evidence with its plane's normal
/// (@p plane_normals, index = label - 1), any other point goes to
/// WallPoints::others.
///
/// @throws std::invalid_argument when @p planes is not index-aligned with
///         @p cloud, a wall plane has no normal, or the storey's
///         level_transform is not finite.
WallPoints wall_points(const Cloud &cloud, const CloudL &planes,
                       const CloudN &plane_normals,
                       const std::vector<core::PlaneClassRecord> &plane_classes,
                       const core::StoreyRecord &storey,
                       const WallOptions &opt = {});

} // namespace reusex::geometry
