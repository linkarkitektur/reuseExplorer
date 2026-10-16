// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/io/trajectory.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <Eigen/Geometry>

#include <fmt/format.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::io {

namespace {

constexpr int kPrecision = std::numeric_limits<double>::max_digits10;

Eigen::Quaterniond quaternion_from_pose(const Eigen::Matrix4d &pose) {
  return Eigen::Quaterniond(pose.topLeftCorner<3, 3>());
}

std::vector<std::string> split_fields(const std::string &line) {
  std::istringstream ss(line);
  std::vector<std::string> fields;
  std::string tok;
  while (ss >> tok)
    fields.push_back(tok);
  return fields;
}

/// Parse `tok` as a double with `strtod`, which (unlike chained
/// `istream::operator>>`, whose "nan"/"inf" support is implementation
/// defined) always recognises "nan" / "inf" / "-inf", so a non-finite value
/// in a TUM file reaches the `std::isfinite` check below instead of being
/// misreported as "not a number".
std::optional<double> parse_double(const std::string &tok) {
  if (tok.empty())
    return std::nullopt;
  char *end = nullptr;
  const double v = std::strtod(tok.c_str(), &end);
  if (end != tok.c_str() + tok.size())
    return std::nullopt; // trailing garbage, e.g. "1.0x"
  return v;
}

} // namespace

void write_tum(const core::Trajectory &trajectory,
               const std::filesystem::path &path) {
  std::ofstream out(path);
  if (!out)
    throw std::runtime_error(
        fmt::format("write_tum: cannot open '{}' for writing", path.string()));

  out.precision(kPrecision);
  out << "# timestamp tx ty tz qx qy qz qw\n";
  for (const auto &sp : trajectory) {
    const Eigen::Vector3d t = sp.pose.topRightCorner<3, 1>();
    const Eigen::Quaterniond q = quaternion_from_pose(sp.pose).normalized();
    out << sp.timestamp << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
        << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  if (!out)
    throw std::runtime_error(
        fmt::format("write_tum: write failed for '{}'", path.string()));
}

core::Trajectory read_tum(const std::filesystem::path &path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error(
        fmt::format("read_tum: cannot open '{}'", path.string()));

  core::Trajectory trajectory;
  std::string line;
  std::int64_t line_no = 0;
  std::int64_t row = 0;
  while (std::getline(in, line)) {
    ++line_no;
    // Trim trailing CR (files written on another platform) and whitespace.
    while (!line.empty() &&
           (line.back() == '\r' ||
            std::isspace(static_cast<unsigned char>(line.back()))))
      line.pop_back();
    if (line.empty() || line.front() == '#')
      continue;

    const auto fields = split_fields(line);
    if (fields.size() != 8)
      throw std::runtime_error(
          fmt::format("read_tum: '{}' line {}: expected 8 numbers "
                      "(timestamp tx ty tz qx qy qz qw), got {}: '{}'",
                      path.string(), line_no, fields.size(), line));

    std::array<double, 8> values{};
    for (std::size_t k = 0; k < 8; ++k) {
      const auto v = parse_double(fields[k]);
      if (!v)
        throw std::runtime_error(
            fmt::format("read_tum: '{}' line {}: field {} ('{}') is not a "
                        "number",
                        path.string(), line_no, k + 1, fields[k]));
      values[k] = *v;
    }
    const double ts = values[0], tx = values[1], ty = values[2], tz = values[3],
                 qx = values[4], qy = values[5], qz = values[6], qw = values[7];

    for (double v : values)
      if (!std::isfinite(v))
        throw std::runtime_error(
            fmt::format("read_tum: '{}' line {}: non-finite value",
                        path.string(), line_no));

    const Eigen::Quaterniond q(qw, qx, qy, qz);
    const double norm = q.norm();
    if (!(norm > 1e-6))
      throw std::runtime_error(
          fmt::format("read_tum: '{}' line {}: quaternion has ~zero norm",
                      path.string(), line_no));

    core::StampedPose sp;
    sp.node_id = row;
    sp.timestamp = ts;
    sp.pose = Eigen::Matrix4d::Identity();
    sp.pose.topLeftCorner<3, 3>() = q.normalized().toRotationMatrix();
    sp.pose.topRightCorner<3, 1>() = Eigen::Vector3d(tx, ty, tz);
    trajectory.push_back(sp);
    ++row;
  }
  return trajectory;
}

core::Trajectory trajectory_from_project(const ProjectDB &db,
                                         bool use_initial) {
  auto ids = db.sensor_frame_ids();
  if (ids.empty())
    throw std::runtime_error(
        "trajectory_from_project: project has no sensor frames");

  core::Trajectory trajectory;
  trajectory.reserve(ids.size());
  std::size_t skipped = 0;
  for (int id : ids) {
    std::optional<std::array<double, 16>> raw;
    if (use_initial) {
      if (db.has_initial_pose(id))
        raw = db.initial_pose(id);
    } else if (db.has_sensor_frame_pose(id)) {
      raw = db.sensor_frame_pose(id);
    }
    if (!raw) {
      ++skipped;
      continue;
    }

    core::StampedPose sp;
    sp.node_id = id;
    const double ts = db.sensor_frame_timestamp(id);
    sp.timestamp = ts >= 0.0 ? ts : static_cast<double>(id);
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        m(r, c) = (*raw)[static_cast<std::size_t>(r * 4 + c)];
    sp.pose = m;
    trajectory.push_back(sp);
  }

  if (trajectory.empty())
    throw std::runtime_error(fmt::format(
        "trajectory_from_project: none of {} sensor frames carry a usable "
        "{} pose",
        ids.size(), use_initial ? "initial" : "stored"));
  if (skipped > 0)
    core::warn(
        "trajectory_from_project: {} of {} sensor frames have no usable {} "
        "pose and were skipped",
        skipped, ids.size(), use_initial ? "initial" : "stored");

  return trajectory;
}

} // namespace reusex::io
