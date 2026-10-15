// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/storey_identity.hpp"
#include "core/guid.hpp"

#include <Eigen/Geometry>
#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>

namespace reusex::core {
namespace {

Eigen::Matrix4d to_matrix(const std::array<double, 16> &a) {
  Eigen::Matrix4d m;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m(r, c) = a[static_cast<size_t>(r * 4 + c)];
  return m;
}

/// Height @p z of @p s's level frame, expressed in @p to.
double height_in_frame(const StoreyRecord &s, double z,
                       const std::array<double, 16> &to) {
  const Eigen::Vector4d world =
      to_matrix(s.level_transform).inverse() * Eigen::Vector4d(0, 0, z, 1);
  return (to_matrix(to) * world).z();
}

} // namespace

double floor_in_frame(const StoreyRecord &s,
                      const std::array<double, 16> &world_to_level) {
  return height_in_frame(s, s.z_floor, world_to_level);
}

StoreyIdentityResult assign_storey_guids(
    std::vector<StoreyRecord> &fresh, const std::vector<StoreyRecord> &stored,
    std::string_view project_guid, const StoreyIdentityOptions &opt) {
  StoreyIdentityResult result;
  // (distance, fresh index, stored index): greedy by distance, ties by the
  // fresh then the stored storey's position (both idx-ordered).
  std::vector<std::tuple<double, size_t, size_t>> pairs;
  for (size_t i = 0; i < fresh.size(); ++i)
    for (size_t j = 0; j < stored.size(); ++j) {
      const double d =
          std::abs(floor_in_frame(stored[j], fresh[i].level_transform) -
                   fresh[i].z_floor);
      if (d <= opt.match_m)
        pairs.emplace_back(d, i, j);
    }
  std::sort(pairs.begin(), pairs.end());

  std::vector<bool> fresh_done(fresh.size(), false);
  std::vector<bool> stored_used(stored.size(), false);
  std::set<std::string> taken;
  for (const auto &s : stored)
    taken.insert(s.guid);
  for (const auto &[d, i, j] : pairs) {
    (void)d;
    if (fresh_done[i] || stored_used[j])
      continue;
    fresh[i].guid = stored[j].guid;
    fresh_done[i] = stored_used[j] = true;
    ++result.matched;
  }

  for (size_t i = 0; i < fresh.size(); ++i) {
    if (fresh_done[i])
      continue;
    const std::string base =
        fmt::format("storeys/z{}", std::llround(fresh[i].z_floor * 100.0));
    std::string guid = deterministic_guid(project_guid, base);
    for (int k = 2; taken.count(guid); ++k)
      guid = deterministic_guid(project_guid, fmt::format("{}#{}", base, k));
    fresh[i].guid = guid;
    taken.insert(guid);
    ++result.minted;
  }
  return result;
}

std::vector<LockedStoreyConflict>
locked_storey_conflicts(const std::vector<StoreyRecord> &stored,
                        const std::vector<StoreyRecord> &fresh,
                        double angle_tol_deg, double shift_tol_m) {
  std::vector<LockedStoreyConflict> out;
  if (fresh.empty())
    return out;
  const auto &frame = fresh.front().level_transform;
  const Eigen::Matrix4d to = to_matrix(frame);
  for (const auto &s : stored) {
    if (!s.locked)
      continue;
    LockedStoreyConflict c;
    c.guid = s.guid;
    c.idx = s.idx;
    const Eigen::Matrix4d from = to_matrix(s.level_transform);
    const Eigen::Matrix3d dr =
        to.topLeftCorner<3, 3>() * from.topLeftCorner<3, 3>().transpose();
    c.frame_angle_deg =
        Eigen::AngleAxisd(dr).angle() * 180.0 / 3.14159265358979323846;
    c.frame_shift_m =
        (to.topRightCorner<3, 1>() - dr * from.topRightCorner<3, 1>()).norm();
    c.frame_differs =
        c.frame_angle_deg > angle_tol_deg || c.frame_shift_m > shift_tol_m;

    const double lo = height_in_frame(s, s.z_floor, frame);
    const double hi = height_in_frame(s, s.z_ceiling, frame);
    for (const auto &f : fresh) {
      if (f.guid == s.guid)
        continue;
      const double overlap =
          std::min(hi, f.z_ceiling) - std::max(lo, f.z_floor);
      if (overlap > 1e-3)
        c.overlapping_idx.push_back(f.idx);
    }
    if (c.frame_differs || !c.overlapping_idx.empty())
      out.push_back(std::move(c));
  }
  return out;
}

} // namespace reusex::core
