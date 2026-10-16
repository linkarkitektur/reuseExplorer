// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Synthetic wall evidence for the W3 wall tests (reusex-z1v.5): vertical
// faces sampled on a regular (s, z) grid with seeded offset noise, in the
// level frame, plus floor points standing in for "a room is here".

#include <reusex/reconstruction/walls.hpp>

#include <Eigen/Core>

#include <cmath>
#include <random>

namespace reusex::test_support::walls {

/// Sample a vertical face from (x0, y0) to (x1, y1), z in [z0, z1), every
/// @p step, displaced along its normal by N(0, noise). @p normal is the
/// direction the face is seen from (the plane normal, sensor-facing).
inline void add_face(geometry::WallPoints &wp, Eigen::Vector2d a,
                     Eigen::Vector2d b, Eigen::Vector2d normal, double z0,
                     double z1, double noise = 0.0, unsigned seed = 1,
                     double step = 0.05, double lean_per_m = 0.0) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> nd(0.0, noise > 0.0 ? noise : 1.0);
  const Eigen::Vector2d t = (b - a).normalized();
  const Eigen::Vector2d n = normal.normalized();
  const double len = (b - a).norm();
  for (double s = 0.5 * step; s < len; s += step)
    for (double z = z0 + 0.5 * step; z < z1; z += step) {
      const double e = noise > 0.0 ? nd(rng) : 0.0;
      const Eigen::Vector2d p = a + s * t + (e + lean_per_m * z) * n;
      wp.points.emplace_back(p.x(), p.y(), z);
      wp.normals.push_back(n);
    }
}

/// Floor points over the rectangle [x0, x1] x [y0, y1] (z = 0.01): the
/// non-wall evidence of a room.
inline void add_floor(geometry::WallPoints &wp, double x0, double y0, double x1,
                      double y1, double step = 0.05) {
  for (double x = x0 + 0.5 * step; x < x1; x += step)
    for (double y = y0 + 0.5 * step; y < y1; y += step)
      wp.others.emplace_back(x, y, 0.01);
}

/// A storey 0..2.5 m.
inline geometry::WallPoints storey_points() {
  geometry::WallPoints wp;
  wp.z_floor = 0.0;
  wp.z_ceiling = 2.5;
  return wp;
}

} // namespace reusex::test_support::walls
