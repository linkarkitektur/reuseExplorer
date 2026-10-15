// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file
/// The level frame of a storey (solid-model workstream, WP1): the rigid
/// world -> level transform stored in `core::StoreyRecord::level_transform`.
///
/// In the level frame z is the measured vertical and the floor of storey 0 is
/// at z = 0 (its floor point is the origin). Raw data is never rewritten:
/// stages keep world coordinates in the clouds and go through this frame.
///
/// Layering note: header-only, like `component_persistence.hpp`. It includes a
/// core header (`core/solid_model.hpp`, plain PODs), so it is deliberately not
/// compiled into `reusex_geometry_common`, which must not link core
/// (STANDARDS §1). Its consumers already link both.

#include "reusex/core/solid_model.hpp"
#include "reusex/geometry/transform_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace reusex::geometry {

/// World <-> level frame of one storey.
struct LevelFrame {
  Eigen::Affine3d world_to_level = Eigen::Affine3d::Identity();

  /// The frame a stored storey row carries (identity for a default record).
  static LevelFrame from_record(const core::StoreyRecord &s) {
    LevelFrame f;
    f.world_to_level = to_affine(s.level_transform);
    return f;
  }

  [[nodiscard]] Eigen::Vector3d to_level(const Eigen::Vector3d &p) const {
    return world_to_level * p;
  }

  [[nodiscard]] Eigen::Vector3d to_world(const Eigen::Vector3d &p) const {
    return world_to_level.inverse(Eigen::Isometry) * p;
  }

  /// Row-major 4x4, the `StoreyRecord::level_transform` layout.
  [[nodiscard]] std::array<double, 16> to_array() const {
    return to_array16(world_to_level);
  }

  /// World z where the level frame's z = z_level plane crosses the frame
  /// origin's vertical (the level z axis). Used to pick MIP section planes
  /// (W1.4) and mesh-sectioning heights.
  [[nodiscard]] double world_z_at(double z_level) const {
    return to_world(Eigen::Vector3d(0.0, 0.0, z_level)).z();
  }
};

/// The level frame whose +z is @p vertical and whose origin is
/// @p floor_point: the minimal rotation taking @p vertical to +z, then a
/// translation putting @p floor_point at (0, 0, 0). @p vertical need not be
/// normalised but must be non-zero.
inline LevelFrame make_level_frame(const Eigen::Vector3d &vertical,
                                   const Eigen::Vector3d &floor_point) {
  const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
      vertical.normalized(), Eigen::Vector3d::UnitZ());
  LevelFrame f;
  f.world_to_level = Eigen::Affine3d::Identity();
  f.world_to_level.linear() = q.toRotationMatrix();
  f.world_to_level.translation() = -(q * floor_point);
  return f;
}

} // namespace reusex::geometry
