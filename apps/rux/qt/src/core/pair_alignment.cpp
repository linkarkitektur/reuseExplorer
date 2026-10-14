// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/pair_alignment.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

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
  return normalized(o);
}

Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to) {
  return mat4_mul(mat4_mul(rigid_inverse(pose_from), world_delta_on_to),
                  pose_to);
}

namespace {
/// @p deg wrapped into (-180, 180].
double wrap_deg(double deg) {
  double w = std::fmod(deg, 360.0); // (-360, 360)
  if (w > 180.0)
    w -= 360.0;
  else if (w <= -180.0)
    w += 360.0;
  return w;
}
} // namespace

ManualOffset normalized(ManualOffset o) {
  for (double *t : {&o.x, &o.y, &o.z})
    *t = std::clamp(*t, -kOffsetMaxM, kOffsetMaxM);
  for (double *a : {&o.yaw_deg, &o.pitch_deg, &o.roll_deg})
    *a = wrap_deg(*a);
  return o;
}

double rotation_angle_deg(const Mat4 &m) {
  const double trace = m[0] + m[5] + m[10];
  const double c = std::clamp((trace - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / std::numbers::pi;
}

namespace {
/// Every k-th element, k = ceil(n / max_count): thin_points' and
/// thin_colours' one stride.
template <class T>
std::vector<T> thin_every(const std::vector<T> &in, std::size_t max_count) {
  if (max_count == 0)
    return {};
  if (in.size() <= max_count)
    return in;
  const std::size_t stride = (in.size() + max_count - 1) / max_count;
  std::vector<T> out;
  out.reserve(in.size() / stride + 1);
  for (std::size_t i = 0; i < in.size(); i += stride)
    out.push_back(in[i]);
  return out;
}
} // namespace

CloudPoints thin_points(const CloudPoints &points, std::size_t max_count) {
  return thin_every(points, max_count);
}

CloudColours thin_colours(const CloudColours &colours, std::size_t max_count) {
  return thin_every(colours, max_count);
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

bool icp_seed_unchanged(const std::optional<Mat4> &launched,
                        const std::optional<Mat4> &current) {
  return launched == current;
}

IcpFitState icp_fit_state(bool ok, bool stale, bool converged,
                          const std::optional<Mat4> &applied,
                          const std::optional<Mat4> &now) {
  if (!ok)
    return IcpFitState::failed;
  if (stale)
    return IcpFitState::dropped;
  if (!icp_seed_unchanged(applied, now))
    return IcpFitState::moved_since;
  if (!converged)
    return IcpFitState::not_converged;
  return applied ? IcpFitState::aligned : IcpFitState::not_placed;
}

bool icp_fit_usable(IcpFitState state) {
  return state == IcpFitState::aligned || state == IcpFitState::not_placed;
}

EdgeRecord pair_edge_to_add(int a, int b, const std::string &type,
                            const Mat4 &pose_a, const Mat4 &pose_b,
                            const std::optional<Mat4> &delta,
                            const std::optional<IcpEdgeMeasurement> &icp) {
  EdgeRecord e;
  e.status = "confirmed";
  if (icp) {
    e.key = {b, a, type}; // ICP ran B onto A
    e.transform = icp->edge_transform;
    e.information = icp->information;
    e.source = "icp"; // weight: the store derives it from the information
  } else {
    e.key = {a, b, type};
    e.transform =
        edge_measurement(pose_a, pose_b, delta.value_or(mat4_identity()));
    e.source = "manual"; // the optimizer's per-type default information
  }
  return e;
}

Info6 reverse_edge_information(const Info6 &info_ij, const Mat4 &T_ij) {
  const Mat4 t = rigid_inverse(T_ij); // T_ji
  // Ad(T_ji) = [[R, 0], [t^ R, R]] (rotation block first).
  const double tx = t[3], ty = t[7], tz = t[11];
  const double sk[9] = {0, -tz, ty, tz, 0, -tx, -ty, tx, 0};
  std::array<double, 36> ad{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const double r = t[i * 4 + j];
      ad[i * 6 + j] = r;             // top-left R
      ad[(i + 3) * 6 + (j + 3)] = r; // bottom-right R
      double s = 0.0;                // bottom-left t^ R
      for (int k = 0; k < 3; ++k)
        s += sk[i * 3 + k] * t[k * 4 + j];
      ad[(i + 3) * 6 + j] = s;
    }
  // out = Ad^T · L · Ad
  std::array<double, 36> la{};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k)
        s += info_ij[i * 6 + k] * ad[k * 6 + j];
      la[i * 6 + j] = s;
    }
  Info6 out{};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      double s = 0.0;
      for (int k = 0; k < 6; ++k)
        s += ad[k * 6 + i] * la[k * 6 + j];
      out[i * 6 + j] = s;
    }
  for (int i = 0; i < 6; ++i)
    for (int j = i + 1; j < 6; ++j) {
      const double m = 0.5 * (out[i * 6 + j] + out[j * 6 + i]);
      out[i * 6 + j] = out[j * 6 + i] = m;
    }
  return out;
}

EdgeRecord canonical_edge(EdgeRecord e) {
  if (e.key.from <= e.key.to)
    return e;
  std::swap(e.key.from, e.key.to);
  if (e.transform) {
    if (e.information) {
      e.information = reverse_edge_information(*e.information, *e.transform);
      e.weight = std::numeric_limits<double>::quiet_NaN();
    }
    e.transform = rigid_inverse(*e.transform);
  }
  return e;
}

std::string_view overlay_colouring_key(OverlayColouring c) {
  return c == OverlayColouring::rgb ? "rgb" : "tint";
}

OverlayColouring parse_overlay_colouring(std::string_view key) {
  return key == "rgb" ? OverlayColouring::rgb : OverlayColouring::tint;
}

} // namespace rux::qt
