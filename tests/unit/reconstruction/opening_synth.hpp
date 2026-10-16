// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Synthetic wall faces for the WP5 opening tests (reusex-z1v.7): a straight
// wall along +x in the level frame, its faces sampled every 5 cm with holes
// cut out of them, plus floor, ceiling and furniture points.

#include <reusex/core/solid_model.hpp>

#include <Eigen/Core>

#include <functional>
#include <vector>

namespace reusex::test_support::openings {

/// An axis-aligned (s, z) rectangle on the face.
struct Hole {
  double s0, s1, z0, z1;
  [[nodiscard]] bool contains(double s, double z) const {
    return s >= s0 && s < s1 && z >= z0 && z < z1;
  }
};

/// A wall from (0, 0) to (length, 0), storey 0 .. height, thickness 0.1:
/// faces at y = +-0.05.
inline core::WallRecord wall(double length = 5.0, double height = 2.5,
                             double thickness = 0.1) {
  core::WallRecord w;
  w.guid = "w-test";
  w.x0 = 0.0;
  w.y0 = 0.0;
  w.x1 = length;
  w.y1 = 0.0;
  w.z_base = 0.0;
  w.height = height;
  w.thickness = thickness;
  return w;
}

/// Sample the face at y = @p t over [0, length) x [0, height), every
/// @p step, leaving out the points inside any of @p holes.
inline void add_face(std::vector<Eigen::Vector3d> &pts, double t, double length,
                     double height, const std::vector<Hole> &holes,
                     double step = 0.05) {
  for (double s = 0.5 * step; s < length; s += step)
    for (double z = 0.5 * step; z < height; z += step) {
      bool cut = false;
      for (const auto &h : holes)
        cut = cut || h.contains(s, z);
      if (!cut)
        pts.emplace_back(s, t, z);
    }
}

/// Floor (z = 0.01) and ceiling (z = height - 0.01) points over
/// [0, length] x [y0, y1].
inline void add_floor_ceiling(std::vector<Eigen::Vector3d> &pts, double length,
                              double y0, double y1, double height,
                              double step = 0.05) {
  for (double x = 0.5 * step; x < length; x += step)
    for (double y = y0 + 0.5 * step; y < y1; y += step) {
      pts.emplace_back(x, y, 0.01);
      pts.emplace_back(x, y, height - 0.01);
    }
}

/// A box (furniture) with its front face at y = @p y_front, over
/// s in [s0, s1), z in [0, z1): its front and top surfaces.
inline void add_box(std::vector<Eigen::Vector3d> &pts, double s0, double s1,
                    double z1, double y_back, double y_front,
                    double step = 0.05) {
  for (double s = s0 + 0.5 * step; s < s1; s += step) {
    for (double z = 0.5 * step; z < z1; z += step)
      pts.emplace_back(s, y_front, z);
    for (double y = std::min(y_back, y_front) + 0.5 * step;
         y < std::max(y_back, y_front); y += step)
      pts.emplace_back(s, y, z1);
  }
}

} // namespace reusex::test_support::openings
