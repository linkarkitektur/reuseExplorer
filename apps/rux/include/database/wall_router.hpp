// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for the walls table (schema v33, `rux create walls`)
 *
 * Handles paths like:
 * - walls               → every wall (JSON array, storey ascending)
 * - walls.<id>          → the wall with that row id
 * - walls.<guid>        → the wall with that guid
 * - walls[i]            → the i-th wall
 * - walls.<id>.<prop>   → one field: id, guid, storey, x0, y0, x1, y1,
 *                         length, z_base, height, thickness,
 *                         thickness_ci_lo, thickness_ci_hi, lean_deg,
 *                         is_external, assumed, seen_share, face_a_points,
 *                         face_b_points, co_visibility, axis_family, source,
 *                         locked, confidence, metadata, joins
 *
 * The walls of one storey: `rux get storeys.<idx>.walls`. Coordinates are in
 * that storey's level frame. `rux del walls` deletes every wall and join;
 * when some walls are locked (edited by hand) it needs --force. The rows are
 * written by `rux create walls`, not by `rux set`.
 */
class WallRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::optional<std::string> del_requires_force(
      const std::vector<PathComponent> &components) const override;
  std::vector<std::string> list() const override;
};

/**
 * @brief Router for the wall_joins table (schema v33), read-only
 *
 * - wall_joins     → every join: {wall_a, wall_b, kind, at_a, at_b}, walls
 *                    by guid
 * - wall_joins[i]  → the i-th join
 *
 * Joins go with their walls (`rux del walls`).
 */
class WallJoinRouter : public ResourceRouter {
    public:
  using ResourceRouter::ResourceRouter;

  DataPayload get(const std::vector<PathComponent> &components) override;
  void set(const std::vector<PathComponent> &components,
           const DataPayload &data) override;
  void del(const std::vector<PathComponent> &components) override;
  std::vector<std::string> list() const override;
};

/// One wall as the JSON `rux get walls` prints; @p joins (every stored
/// join) fills its "joins" array, from this wall's side.
nlohmann::json
wall_to_json(const reusex::core::WallRecord &w,
             const std::vector<reusex::core::WallJoinRecord> &joins);

/// The walls of the storey with idx @p storey (every wall when < 0).
nlohmann::json walls_json(const reusex::ProjectDB &db, int storey = -1);

} // namespace rux::database
