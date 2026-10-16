// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Pose edits (reusex.pose_edits.v1, reusex-rcv.1.2): the records of a hand-
// or agent-authored file of corrections that `rux optimize --edits` lowers
// into extra factors of the stored-graph solve. The pose analogue of the
// solid-model plan document (docs/design/pose-edits.md).
//
// Plain records owned by core (the solid-model split, STANDARDS §1): the
// reader/writer lives in io (io/pose_edits.hpp), the lowering into factors in
// slam (slam/pose_edit_lowering.hpp), and slam and io never link each other.
//
// Conventions:
//  - Node ids are `sensor_frames.node_id`s.
//  - Poses are in the stored `sensor_frames.transform` convention (sensor
//    frame -> world), expressed in the document's `frame` (world, or a
//    storey's level frame).
//  - Lengths in metres, angles in degrees. A rotation given as yaw / pitch /
//    roll is R = Rz(yaw) * Ry(pitch) * Rx(roll) about the frame's axes.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

/// What an edit says.
enum class PoseEditKind {
  anchor,       ///< absolute pose prior on one node, with a DoF mask
  move,         ///< rigid delta for a node range (a pass, a segment)
  relative,     ///< pose of node b in node a's frame (a manual edge)
  same_surface, ///< two node ranges see one surface (rank-1..3 constraint)
  on_plan_wall, ///< a node range's points lie on a plan wall
  reject_edge,  ///< leave a stored pose-graph edge out of the solve
};

/// The spelling of @p kind in the document ("same_surface", ...).
std::string_view to_string(PoseEditKind kind);
/// The kind spelled @p name; nullopt when there is none.
std::optional<PoseEditKind> pose_edit_kind_from_string(std::string_view name);

/// Degree-of-freedom bits of a pose prior, about / along the edit frame's
/// axes. `all` = every DoF.
struct Dof {
  static constexpr std::uint8_t x = 1, y = 2, z = 4, roll = 8, pitch = 16,
                                yaw = 32, all = 63;
};

/// The names of the bits set in @p dofs, in the order x y z roll pitch yaw.
std::vector<std::string> dof_names(std::uint8_t dofs);
/// The bit named @p name ("x" ... "yaw"); 0 when there is none.
std::uint8_t dof_from_string(std::string_view name);

/// A node set: the inclusive range [first, last], or an explicit `list`
/// (when `list` is non-empty, first/last are unused).
struct NodeRange {
  std::int64_t first = -1;
  std::int64_t last = -1;
  std::vector<std::int64_t> list;

  [[nodiscard]] bool is_list() const { return !list.empty(); }
  [[nodiscard]] bool contains(std::int64_t id) const;
  /// The ids of @p available (sorted ascending) inside this set, in order.
  [[nodiscard]] std::vector<std::int64_t>
  select(const std::vector<std::int64_t> &available) const;
  bool operator==(const NodeRange &) const = default;
};

/// A rigid transform as written in the document: a translation `xyz` plus
/// yaw / pitch / roll (deg) about `pivot`, or a full 4x4 `matrix`
/// (row-major), which then wins.
struct RigidTransform {
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
  /// Centre of rotation of a delta. nullopt: the centroid of the edit's
  /// nodes (a move), or the frame origin (an absolute pose).
  std::optional<std::array<double, 3>> pivot;
  std::optional<std::array<double, 16>> matrix;

  /// The 4x4: [R, c + xyz - R c] for pivot c (@p default_pivot when `pivot`
  /// is unset), or `matrix` when set.
  [[nodiscard]] Eigen::Matrix4d to_matrix(
      const Eigen::Vector3d &default_pivot = Eigen::Vector3d::Zero()) const;
  bool operator==(const RigidTransform &) const = default;
};

/// The pose an anchor / relative edit asks for.
struct PoseSpec {
  enum class Source {
    current, ///< the node's `transform` when the edits are lowered
    initial, ///< the node's `initial_transform`
    given,   ///< `value`
  };
  Source source = Source::current;
  RigidTransform value;
  bool operator==(const PoseSpec &) const = default;
};

/// One edit. Which fields are meaningful depends on `kind` (see
/// docs/design/pose-edits.md); the rest keep their defaults.
struct PoseEdit {
  std::string id; ///< unique within the document; names the edit in errors
  PoseEditKind kind = PoseEditKind::anchor;
  NodeRange nodes;      ///< anchor: one node; move / on_plan_wall: the range;
                        ///< relative / reject_edge: node a; same_surface: a
  NodeRange nodes_b;    ///< relative / reject_edge: node b; same_surface: b
  PoseSpec pose;        ///< anchor: world pose; relative: pose of b in a
  RigidTransform delta; ///< move: rigid delta (world, about pivot)
  std::uint8_t dofs = Dof::all; ///< anchor / move / relative: DoF mask
  double sigma_trans_m = 0.02;  ///< anchor / move / relative
  double sigma_rot_deg = 0.5;   ///< anchor / move / relative
  double sigma_m = 0.01;        ///< same_surface / on_plan_wall
  std::optional<std::array<double, 4>> region;   ///< [x0, y0, x1, y1]
  std::optional<std::array<double, 4>> region_b; ///< same_surface: b's
  std::string plan_path;                         ///< on_plan_wall
  std::string wall_guid;                         ///< on_plan_wall
  std::string edge_type;                         ///< reject_edge
  std::string note; ///< free text: provenance (which render showed what)
  bool operator==(const PoseEdit &) const = default;
};

/// A pose-edits document.
struct PoseEdits {
  int version = 1;
  /// The project the edits were written for (ProjectDB::project_guid());
  /// empty = not checked.
  std::string project_guid;
  /// "world", or "storey:<guid>" (that storey's level frame).
  std::string frame = "world";
  std::vector<PoseEdit> edits;
  bool operator==(const PoseEdits &) const = default;
};

/// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in degrees.
Eigen::Matrix3d rotation_from_ypr_deg(double yaw_deg, double pitch_deg,
                                      double roll_deg);

} // namespace reusex::core
