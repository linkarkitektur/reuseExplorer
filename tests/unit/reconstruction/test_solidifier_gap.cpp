// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The Solidifier's MIP must be solved to (near) optimality, not to a loose
// relative gap (reusex-uur). On house2 a 2.7 m^2 stair hall is worth about
// 0.03 objective units net (volume term minus boundary term) in a storey
// section whose objective is about -372: cuOpt's former 1 % default gap
// (3.7 units) let it return a 'feasible' labelling without the hall, and
// which one it returned changed from run to run. HiGHS (default 1e-4)
// proved the labelling with the hall optimal.

#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/Solidifier.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <utility>
#include <vector>

using namespace reusex::geometry;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

// n . p + d = 0 for the plane through c along axis (0 = x, 1 = y, 2 = z),
// its normal along +axis (sign = 1) or -axis (sign = -1).
Eigen::Vector4d axis_plane(int axis, double c, double sign) {
  Eigen::Vector4d p = Eigen::Vector4d::Zero();
  p[axis] = sign;
  p[3] = -sign * c;
  return p;
}

} // namespace

TEST_CASE("SolidifierOptions_DefaultMipGap_IsTightEnoughForASmallRoom",
          "[geometry][solidifier]") {
  // A room's value in the objective is its volume times its room
  // probability margin, a few tenths for a closet or a hall, while a
  // storey section's objective runs to several hundred (NewOffice: 1065).
  // The default relative gap must be far below their ratio, and it must be
  // explicit so cuOpt and HiGHS stop at the same tolerance (cuOpt's own
  // default was 1e-2, HiGHS' 1e-4).
  const SolidifierOptions opt;
  CHECK(opt.mip_gap > 0.0);
  CHECK(opt.mip_gap <= 1e-5);
}

TEST_CASE("Solidifier_MarginalSmallRoom_IsMeshed", "[geometry][solidifier]") {
  // A 10 x 10 m room A and, behind a 0.3 m wall, a 1 x 1 m room B whose
  // cells read room B only just more often than outside (0.52 vs 0.48):
  // B is worth 2.5 m^3 * 0.04 = 0.1 against an objective of about -225
  // (0.04 %), well inside a 1 % gap. Every face is fully supported, so no
  // wall costs anything: the optimum has B as a room.
  PlaneVec planes;
  std::vector<size_t> verticals, horizontals;
  std::vector<std::pair<size_t, size_t>> pairs;
  auto add = [&](const Eigen::Vector4d &p, bool vertical) {
    planes.push_back(p);
    (vertical ? verticals : horizontals).push_back(planes.size() - 1);
    return planes.size() - 1;
  };
  // A wall from c0 to c1 along an axis: its two faces' normals point away
  // from the slab between them, as make_pairs orients a scanned wall.
  auto wall = [&](int axis, double c0, double c1) {
    pairs.emplace_back(add(axis_plane(axis, c0, -1.0), true),
                       add(axis_plane(axis, c1, 1.0), true));
  };
  wall(0, -0.2, 0.0);
  wall(0, 10.0, 10.3); // between A and B
  wall(0, 11.3, 11.5);
  wall(1, -0.2, 0.0);
  wall(1, 1.0, 1.2); // B's far wall (runs on through A)
  wall(1, 10.0, 10.2);
  add(axis_plane(2, 0.0, 1.0), false);
  add(axis_plane(2, 2.5, -1.0), false);

  auto cc = std::make_shared<CellComplex>(
      planes, verticals, horizontals, pairs, std::array<double, 2>{-1.0, -1.0},
      std::array<double, 2>{12.5, 11.2}, std::nullopt, 5000);

  // Rooms: 1 = A, 2 = B.
  cc->n_rooms = 2;
  auto c_rp = cc->add_property_map<CellComplex::Vertex, std::vector<double>>(
                    "c:room_probabilities")
                  .first;
  auto inside = [](const Eigen::Vector3d &p, double x0, double x1, double y0,
                   double y1) {
    return p.x() > x0 && p.x() < x1 && p.y() > y0 && p.y() < y1 &&
           p.z() > 0.0 && p.z() < 2.5;
  };
  std::vector<CellComplex::Vertex> b_cells;
  for (auto c = cc->cells_begin(); c != cc->cells_end(); ++c) {
    const auto &p = (*cc)[*c].pos;
    if (inside(p, 0.0, 10.0, 0.0, 10.0)) {
      c_rp[*c] = {0.1, 0.9, 0.0};
    } else if (inside(p, 10.3, 11.3, 0.0, 1.0)) {
      c_rp[*c] = {0.48, 0.0, 0.52};
      b_cells.push_back(*c);
    } else {
      c_rp[*c] = {1.0, 0.0, 0.0};
    }
  }
  REQUIRE_FALSE(b_cells.empty());
  auto f_sp =
      cc->add_property_map<CellComplex::Vertex, double>("f:support_probability")
          .first;
  for (auto f = cc->faces_begin(); f != cc->faces_end(); ++f)
    f_sp[*f] = 1.0;

  Solidifier solidifier(cc);
  const auto result = solidifier.solve();
  REQUIRE(result.has_value());
  const auto &room_label = result->first;
  for (auto c : b_cells) {
    INFO("room B cell at " << (*cc)[c].pos.transpose());
    CHECK(room_label.at(c) == 2);
  }
}
