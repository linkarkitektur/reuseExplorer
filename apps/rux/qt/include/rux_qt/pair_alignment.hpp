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
// ICP information is slam::icp_edge_information's. Edge direction is
// canonicalised here exactly as the library's upsert does it
// (canonical_edge mirrors ProjectDB::upsert_pose_graph_edge and
// core::pose_graph::reverse_edge_information, which need Eigen), so the
// editor holds the row the file will hold; a test pins the two together.

#include <rux_qt/database_logic.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
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

/// ΔT's translation is clamped to ±this many metres.
inline constexpr double kOffsetMaxM = 50.0;
/// Angles wrapped into (-180, 180], translation clamped to ±kOffsetMaxM:
/// the ΔT the overlay holds, seeds ICP with and reads out.
ManualOffset normalized(ManualOffset o);
/// The rotation angle of @p m's rotation part, degrees in [0, 180]:
/// acos((tr R - 1) / 2).
double rotation_angle_deg(const Mat4 &m);

/// T_from^-1 · @p world_delta_on_to · T_to.
Mat4 edge_measurement(const Mat4 &pose_from, const Mat4 &pose_to,
                      const Mat4 &world_delta_on_to);

/// Whether an ICP run launched with seed @p launched may still move B: the
/// seed the pair panel would give now, @p current, is the same (nullopt = no
/// manual offset). Exact: ΔT only changes through an edit, and a moved,
/// reset or re-nudged B makes the result stale.
bool icp_seed_unchanged(const std::optional<Mat4> &launched,
                        const std::optional<Mat4> &current);

/// What a finished ICP run of the pair still means, given B's placement.
enum class IcpFitState {
  failed,        ///< ICP threw
  dropped,       ///< B was moved while ICP ran; never applied
  not_converged, ///< B stayed where it was
  aligned,       ///< converged, B placed on the result and still there
  not_placed,    ///< converged, but B could not be placed (no overlay)
  moved_since,   ///< B was moved after ICP: the fit no longer describes it
};
/// @p applied is ΔT right after the result was handled (B's placement on
/// the result, or the seed it stayed at; nullopt = no offset), @p now is ΔT
/// at present.
IcpFitState icp_fit_state(bool ok, bool stale, bool converged,
                          const std::optional<Mat4> &applied,
                          const std::optional<Mat4> &now);
/// Whether a new edge may use the fit's measurement and information.
bool icp_fit_usable(IcpFitState state);

/// What a usable ICP fit of the pair measured, B -> A.
struct IcpEdgeMeasurement {
  /// FramePairIcpResult::edge_transform: the edge B -> A.
  Mat4 edge_transform{};
  /// slam::icp_edge_information of the fit.
  std::array<double, 36> information{};
};
/// The edge "Tilføj kant" stages between A and B, status "confirmed": ICP's
/// B -> A measurement and information while its fit is usable (@p icp;
/// source "icp"), else T_A^-1 · ΔT · T_B as A -> B with the optimizer's
/// per-type default information (source "manual"). The weight is NaN either
/// way: the store derives it from the information. @p delta is B's manual
/// offset (nullopt = none: the stored relative pose). Direction is
/// canonicalised when it is staged (PendingEdgeEdits::add).
EdgeRecord pair_edge_to_add(int a, int b, const std::string &type,
                            const Mat4 &pose_a, const Mat4 &pose_b,
                            const std::optional<Mat4> &delta,
                            const std::optional<IcpEdgeMeasurement> &icp);

using Info6 = std::array<double, 36>;
/// The information of the reversed edge j -> i, given the information of
/// i -> j and its measurement T_ij: Ad(T_ji)^T · L · Ad(T_ji), symmetrised,
/// with Ad in GTSAM's (rotation, translation) order. Mirrors
/// reusex::core::pose_graph::reverse_edge_information.
Info6 reverse_edge_information(const Info6 &info_ij, const Mat4 &T_ij);
/// @p e with from < to: when from > to the key is swapped, the transform
/// inverted and the information reversed; the weight is then cleared (NaN),
/// since the store derives it from the information. Unchanged otherwise.
EdgeRecord canonical_edge(EdgeRecord e);

/// A back-projected depth cloud (slam::frame_world_points' element type).
using CloudPoints = std::vector<std::array<float, 3>>;
/// The overlay draws at most this many points per frame, so dragging the
/// box widget stays smooth on a dense depth sensor.
inline constexpr std::size_t kOverlayMaxPoints = 50000;
/// Every k-th point (k = ceil(n / max_count)), spread over the whole cloud;
/// @p points unchanged when it already fits.
CloudPoints thin_points(const CloudPoints &points, std::size_t max_count);
/// RGB per point, index-aligned with a CloudPoints
/// (slam::frame_world_colored_points' colours).
using CloudColours = std::vector<std::array<std::uint8_t, 3>>;
/// thin_points() for colours: the same stride for the same size and cap,
/// so thinned colours stay aligned with the thinned points.
CloudColours thin_colours(const CloudColours &colours, std::size_t max_count);
/// Mean of @p points; the origin for none.
Vec3 centroid(const CloudPoints &points);

/// How the overlay colours A's and B's clouds (reusex-xlg.10): a flat tint
/// each (A --label-1, B --label-0) or the frames' own RGB.
enum class OverlayColouring { tint, rgb };
/// The QSettings value (rediger/overlay_colour).
std::string_view overlay_colouring_key(OverlayColouring c);
/// tint for anything that is not a known key.
OverlayColouring parse_overlay_colouring(std::string_view key);

} // namespace rux::qt
