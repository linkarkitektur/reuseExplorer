// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// Walls with a measured thickness and a bounded wall graph (solid-model
/// WP3, reusex-z1v.5). Sub-steps as free, unit-testable functions:
///
///  1. axis_families(): the directions walls run in, per storey (W3.1).
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

} // namespace reusex::geometry
