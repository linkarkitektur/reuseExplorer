// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/pair_alignment.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rux::qt {
namespace {
constexpr double kDeg = std::numbers::pi / 180.0;
} // namespace

Mat4 mat4_identity() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

Mat4 mat4_mul(const Mat4 &a, const Mat4 &b) {
  Mat4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  return r;
}

Mat4 rigid_inverse(const Mat4 &m) {
  Mat4 r = mat4_identity();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i * 4 + j] = m[j * 4 + i];
  for (int i = 0; i < 3; ++i)
    r[i * 4 + 3] =
        -(r[i * 4 + 0] * m[3] + r[i * 4 + 1] * m[7] + r[i * 4 + 2] * m[11]);
  return r;
}

Vec3 transform_point(const Mat4 &m, const Vec3 &p) {
  return {m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
          m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
          m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11]};
}

bool ManualOffset::is_zero() const {
  constexpr double e = 1e-9;
  return std::abs(x) < e && std::abs(y) < e && std::abs(z) < e &&
         std::abs(yaw_deg) < e && std::abs(pitch_deg) < e &&
         std::abs(roll_deg) < e;
}

Mat4 offset_matrix(const ManualOffset &o, const Vec3 &pivot) {
  const double cz = std::cos(o.yaw_deg * kDeg), sz = std::sin(o.yaw_deg * kDeg);
  const double cp = std::cos(o.pitch_deg * kDeg),
               sp = std::sin(o.pitch_deg * kDeg);
  const double cr = std::cos(o.roll_deg * kDeg),
               sr = std::sin(o.roll_deg * kDeg);
  Mat4 m = mat4_identity();
  m[0] = cz * cp;
  m[1] = cz * sp * sr - sz * cr;
  m[2] = cz * sp * cr + sz * sr;
  m[4] = sz * cp;
  m[5] = sz * sp * sr + cz * cr;
  m[6] = sz * sp * cr - cz * sr;
  m[8] = -sp;
  m[9] = cp * sr;
  m[10] = cp * cr;
  // p -> R (p - pivot) + pivot + t
  const Vec3 rp = transform_point(m, pivot); // R · pivot (translation still 0)
  m[3] = pivot[0] - rp[0] + o.x;
  m[7] = pivot[1] - rp[1] + o.y;
  m[11] = pivot[2] - rp[2] + o.z;
  return m;
}

ManualOffset offset_from_matrix(const Mat4 &m, const Vec3 &pivot) {
  ManualOffset o;
  o.yaw_deg = std::atan2(m[4], m[0]) / kDeg;
  o.pitch_deg = std::asin(std::clamp(-m[8], -1.0, 1.0)) / kDeg;
  o.roll_deg = std::atan2(m[9], m[10]) / kDeg;
  Mat4 r = m;
  r[3] = r[7] = r[11] = 0.0;
  const Vec3 rp = transform_point(r, pivot);
  o.x = m[3] - (pivot[0] - rp[0]);
  o.y = m[7] - (pivot[1] - rp[1]);
  o.z = m[11] - (pivot[2] - rp[2]);
  return o;
}

ManualOffset nudge(ManualOffset o, NudgeKey key, bool coarse) {
  const double f = coarse ? kNudgeCoarse : 1.0;
  switch (key) {
  case NudgeKey::x_minus:
    o.x -= kNudgeM * f;
    break;
  case NudgeKey::x_plus:
    o.x += kNudgeM * f;
    break;
  case NudgeKey::y_minus:
    o.y -= kNudgeM * f;
    break;
  case NudgeKey::y_plus:
    o.y += kNudgeM * f;
    break;
  case NudgeKey::z_minus:
    o.z -= kNudgeM * f;
    break;
  case NudgeKey::z_plus:
    o.z += kNudgeM * f;
    break;
  case NudgeKey::yaw_minus:
    o.yaw_deg -= kNudgeDeg * f;
    break;
  case NudgeKey::yaw_plus:
    o.yaw_deg += kNudgeDeg * f;
    break;
  }
  return o;
}

Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to) {
  return mat4_mul(mat4_mul(rigid_inverse(pose_from), world_delta_on_to),
                  pose_to);
}

CloudPoints thin_points(const CloudPoints &points, std::size_t max_count) {
  if (max_count == 0)
    return {};
  if (points.size() <= max_count)
    return points;
  const std::size_t stride = (points.size() + max_count - 1) / max_count;
  CloudPoints out;
  out.reserve(points.size() / stride + 1);
  for (std::size_t i = 0; i < points.size(); i += stride)
    out.push_back(points[i]);
  return out;
}

Vec3 centroid(const CloudPoints &points) {
  if (points.empty())
    return {0.0, 0.0, 0.0};
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (const auto &p : points) {
    sx += p[0];
    sy += p[1];
    sz += p[2];
  }
  const double n = static_cast<double>(points.size());
  return {sx / n, sy / n, sz / n};
}

} // namespace rux::qt
