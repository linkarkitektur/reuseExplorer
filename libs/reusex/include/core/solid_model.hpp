// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Records of the typed building model (solid-model workstream, reusex-z1v).
//
// Plain PODs, owned by core (the #227 pattern): a stage writes them, ProjectDB
// stores them (one table each, added by WP1..WP6), io/solid_model_json.hpp
// reads and writes them as one JSON document, and the benchmark
// (reconstruction/solid_metrics.hpp) compares two of them. Core needs no
// geometry types for any of this.
//
// Conventions shared by every record (docs/design/solid-model-json.md):
//  - Lengths in metres. Coordinates of walls, spaces and openings are in the
//    level frame of their storey: z up, floor of storey 0 at z = 0. A storey's
//    `level_transform` maps world -> level frame.
//  - Records refer to each other by stable keys, never by table row id:
//    a storey by its `idx`, a wall / space by its `guid`. `id` is the
//    ProjectDB row id (-1 when the record has not been stored).
//  - `source` is "detected" | "manual" | "imported"; `locked` = a re-run must
//    not overwrite the row; `confidence` 0..1, -1 unknown; `metadata` opaque
//    JSON text for evidence that has no field yet ("" = none).

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace reusex::core {

/// A 2D point in a storey's level frame [m].
using PlanPoint = std::array<double, 2>;

/// One storey: a floor level and the frame its elements are expressed in.
struct StoreyRecord {
  int64_t id = -1;
  std::string guid;
  int idx = 0;          ///< 0 = lowest storey; the key other records use
  std::string name;     ///< display name, e.g. "Stueetage"
  double z_floor = 0.0; ///< floor height in the level frame [m]
  double z_ceiling = 0.0;
  /// World -> level frame, 4x4 row-major. Identity when not measured.
  std::array<double, 16> level_transform{1, 0, 0, 0, 0, 1, 0, 0,
                                         0, 0, 1, 0, 0, 0, 0, 1};
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// One wall, as its centre line (axis) plus a measured thickness.
struct WallRecord {
  int64_t id = -1;
  std::string guid;
  int storey = 0; ///< StoreyRecord::idx of the storey it stands on
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0; ///< axis endpoints
  double z_base = 0.0;
  double height = 0.0;
  double thickness = -1.0; ///< [m]; -1 = unknown (e.g. a single face)
  double thickness_ci_lo = -1.0, thickness_ci_hi = -1.0;
  double lean_deg = 0.0;
  bool is_external = false;
  bool assumed = false;     ///< one-sided: thickness defaulted, not measured
  double seen_share = -1.0; ///< fraction of the face area with evidence; -1 n/a
  int face_a_points = 0, face_b_points = 0;
  double co_visibility = -1.0;
  int axis_family = -1;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// How two walls meet (IfcRelConnectsPathElements).
struct WallJoinRecord {
  std::string wall_a; ///< WallRecord::guid
  std::string wall_b; ///< WallRecord::guid
  std::string kind;   ///< corner | tee | collinear
  std::string at_a;   ///< start | end | path
  std::string at_b;   ///< start | end | path
};

/// A space's footprint: one outer ring and optional holes, in the level frame.
/// Rings are open (the first point is not repeated) and have >= 3 points.
struct SpacePolygon {
  std::vector<PlanPoint> outer;
  std::vector<std::vector<PlanPoint>> holes;
};

/// One space (room, corridor, open plan).
struct SpaceRecord {
  int64_t id = -1;
  std::string guid;
  int storey = 0; ///< StoreyRecord::idx
  std::string name;
  std::string kind = "unknown"; ///< room | corridor | open_plan | unknown
  SpacePolygon polygon;
  double z_floor = 0.0, z_ceiling = 0.0;
  /// Derived quantities; -1 = not computed.
  double area = -1.0, perimeter = -1.0, volume = -1.0, compactness = -1.0;
  int room_label = -1; ///< label in the "rooms" cloud, -1 if none
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// The stretch of a wall that bounds a space (IfcRelSpaceBoundary).
struct SpaceBoundaryRecord {
  std::string space;                 ///< SpaceRecord::guid
  std::string wall;                  ///< WallRecord::guid
  std::string side;                  ///< a | b
  double s_start = 0.0, s_end = 0.0; ///< along the wall axis from (x0,y0) [m]
};

/// A door, window or plain opening in a wall.
struct OpeningRecord {
  int64_t id = -1;
  std::string guid;
  std::string wall;             ///< host WallRecord::guid
  std::string kind = "opening"; ///< door | window | opening
  /// Extent along the host wall's axis, measured from (x0,y0) [m].
  double s_start = 0.0, s_end = 0.0;
  /// Bottom and top in the level frame [m].
  double z_sill = 0.0, z_head = 0.0;
  std::string state = "unknown"; ///< open | closed | unknown (doors)
  bool is_external = false;
  std::string source_instance_guid;
  std::string source = "detected";
  bool locked = false;
  double confidence = -1.0;
  std::string metadata;
};

/// The role one plane of the "planes" cloud plays (one row per plane label).
struct PlaneClassRecord {
  int plane_label = 0;
  std::string orientation; ///< horizontal | vertical | slanted
  int semantic_class = -1; ///< majority API class over the inliers, -1 none
  double class_confidence = -1.0;
  std::string role = "unknown"; ///< wall|floor|ceiling|slab|furniture|...
  int storey = -1;              ///< StoreyRecord::idx, -1 none
  int point_count = 0;
  double area = -1.0, z_min = 0.0, z_max = 0.0;
};

/// A typed relation between two elements, by guid (#138).
struct SceneRelationRecord {
  std::string from_guid, to_guid;
  std::string relation_type; ///< contains | connects_to | adjacent_to | ...
  double weight = 1.0;
  std::string metadata;
};

/// Which element kinds a model actually carries.
///
/// An element kind with no rows is ambiguous: "this model has no walls" vs
/// "this model does not say anything about walls" (a reference that only
/// authors spaces, or a project whose walls table does not exist yet). The
/// benchmark scores the first as a miss and reports the second as n/a. A
/// non-empty vector always counts as provided; set a flag here to say "provided
/// and empty".
struct SolidModelProvided {
  bool storeys = false;
  bool walls = false;
  bool spaces = false;
  bool openings = false;
};

/// The whole typed building model of one project (or one authored reference).
struct SolidModel {
  std::string name;        ///< free text, e.g. "NewOffice reference"
  std::string description; ///< free text: who authored it, how, from what
  std::vector<StoreyRecord> storeys;
  std::vector<WallRecord> walls;
  std::vector<WallJoinRecord> wall_joins;
  std::vector<SpaceRecord> spaces;
  std::vector<SpaceBoundaryRecord> space_boundaries;
  std::vector<OpeningRecord> openings;
  std::vector<PlaneClassRecord> plane_classes;
  std::vector<SceneRelationRecord> scene_relations;
  SolidModelProvided provided;

  [[nodiscard]] bool has_storeys() const {
    return provided.storeys || !storeys.empty();
  }
  [[nodiscard]] bool has_walls() const {
    return provided.walls || !walls.empty();
  }
  [[nodiscard]] bool has_spaces() const {
    return provided.spaces || !spaces.empty();
  }
  [[nodiscard]] bool has_openings() const {
    return provided.openings || !openings.empty();
  }
};

} // namespace reusex::core
