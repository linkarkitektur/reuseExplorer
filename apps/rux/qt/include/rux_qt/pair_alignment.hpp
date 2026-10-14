// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pair panel's math, Qt-free (spec §5.2). Row-major 4x4 rigid
// transforms. B's manual offset ΔT is a WORLD-space transform applied to B's
// cloud, rotating about @p pivot (B's centroid) so the cloud turns in place;
// R = Rz(yaw) · Ry(pitch) · Rx(roll). An edge A -> B measured with ΔT is
// T_A^-1 · ΔT · T_B (the store's convention); ICP from B onto A seeded with
// ΔT returns a world_delta that replaces ΔT.
//
// ICP information is slam::icp_edge_information's, and edge direction is
// canonicalised by the library's upsert — neither lives here.

#include <array>
#include <cstddef>
#include <vector>

namespace rux::qt {

using Mat4 = std::array<double, 16>;
using Vec3 = std::array<double, 3>;

Mat4 mat4_identity();
Mat4 mat4_mul(const Mat4 &a, const Mat4 &b);
/// Inverse of a rigid transform (R^T, -R^T t).
Mat4 rigid_inverse(const Mat4 &m);
Vec3 transform_point(const Mat4 &m, const Vec3 &p);

struct ManualOffset {
  double x = 0.0, y = 0.0, z = 0.0;                      ///< metres, world axes
  double yaw_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0; ///< about the pivot
  bool is_zero() const;
};

Mat4 offset_matrix(const ManualOffset &o, const Vec3 &pivot);
/// Inverse of offset_matrix(); ambiguous at pitch = +/-90 deg (gimbal lock:
/// yaw and roll trade off and only their sum/difference is recoverable).
ManualOffset offset_from_matrix(const Mat4 &m, const Vec3 &pivot);

enum class NudgeKey {
  x_minus,
  x_plus,
  y_minus,
  y_plus,
  z_minus,
  z_plus,
  yaw_minus,
  yaw_plus
};
inline constexpr double kNudgeM = 0.01;
inline constexpr double kNudgeDeg = 0.5;
inline constexpr double kNudgeCoarse = 10.0;
/// The step, then normalized() (so a nudge past ±180° wraps).
ManualOffset nudge(ManualOffset o, NudgeKey key, bool coarse);

/// The overlay's offset fields span ±this many metres.
inline constexpr double kOffsetMaxM = 50.0;
/// Angles wrapped into (-180, 180], translation clamped to ±kOffsetMaxM:
/// what the six fields can show, so they always equal the real ΔT.
ManualOffset normalized(ManualOffset o);
/// The rotation angle of @p m's rotation part, degrees in [0, 180]:
/// acos((tr R - 1) / 2).
double rotation_angle_deg(const Mat4 &m);

/// T_from^-1 · @p world_delta_on_to · T_to.
Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to);

/// A back-projected depth cloud (slam::frame_world_points' element type).
using CloudPoints = std::vector<std::array<float, 3>>;
/// The overlay draws at most this many points per frame, so dragging the
/// box widget stays smooth on a dense depth sensor.
inline constexpr std::size_t kOverlayMaxPoints = 50000;
/// Every k-th point (k = ceil(n / max_count)), spread over the whole cloud;
/// @p points unchanged when it already fits.
CloudPoints thin_points(const CloudPoints &points, std::size_t max_count);
/// Mean of @p points; the origin for none.
Vec3 centroid(const CloudPoints &points);

} // namespace rux::qt
