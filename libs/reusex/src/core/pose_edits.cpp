// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/pose_edits.hpp"

#include <algorithm>
#include <cmath>

namespace reusex::core {
namespace {

constexpr std::array<std::pair<PoseEditKind, std::string_view>, 6> kKinds{{
    {PoseEditKind::anchor, "anchor"},
    {PoseEditKind::move, "move"},
    {PoseEditKind::relative, "relative"},
    {PoseEditKind::same_surface, "same_surface"},
    {PoseEditKind::on_plan_wall, "on_plan_wall"},
    {PoseEditKind::reject_edge, "reject_edge"},
}};

constexpr std::array<std::pair<std::uint8_t, std::string_view>, 6> kDofs{{
    {Dof::x, "x"},
    {Dof::y, "y"},
    {Dof::z, "z"},
    {Dof::roll, "roll"},
    {Dof::pitch, "pitch"},
    {Dof::yaw, "yaw"},
}};

} // namespace

std::string_view to_string(PoseEditKind kind) {
  for (const auto &[k, name] : kKinds)
    if (k == kind)
      return name;
  return "unknown";
}

std::optional<PoseEditKind> pose_edit_kind_from_string(std::string_view name) {
  for (const auto &[k, n] : kKinds)
    if (n == name)
      return k;
  return std::nullopt;
}

std::vector<std::string> dof_names(std::uint8_t dofs) {
  std::vector<std::string> out;
  for (const auto &[bit, name] : kDofs)
    if (dofs & bit)
      out.emplace_back(name);
  return out;
}

std::uint8_t dof_from_string(std::string_view name) {
  for (const auto &[bit, n] : kDofs)
    if (n == name)
      return bit;
  return 0;
}

bool NodeRange::contains(std::int64_t id) const {
  if (is_list())
    return std::find(list.begin(), list.end(), id) != list.end();
  return id >= first && id <= last;
}

std::vector<std::int64_t>
NodeRange::select(const std::vector<std::int64_t> &available) const {
  std::vector<std::int64_t> out;
  for (const auto id : available)
    if (contains(id))
      out.push_back(id);
  return out;
}

Eigen::Matrix3d rotation_from_ypr_deg(double yaw_deg, double pitch_deg,
                                      double roll_deg) {
  constexpr double kDeg = M_PI / 180.0;
  return (Eigen::AngleAxisd(yaw_deg * kDeg, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch_deg * kDeg, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll_deg * kDeg, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Matrix4d
RigidTransform::to_matrix(const Eigen::Vector3d &default_pivot) const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (matrix) {
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        T(r, c) = (*matrix)[static_cast<size_t>(4 * r + c)];
    return T;
  }
  const Eigen::Matrix3d R = rotation_from_ypr_deg(yaw_deg, pitch_deg, roll_deg);
  const Eigen::Vector3d c =
      pivot ? Eigen::Vector3d((*pivot)[0], (*pivot)[1], (*pivot)[2])
            : default_pivot;
  T.topLeftCorner<3, 3>() = R;
  T.topRightCorner<3, 1>() =
      c + Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - R * c;
  return T;
}

} // namespace reusex::core
