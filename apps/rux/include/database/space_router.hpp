// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for the spaces table (schema v35, `rux create spaces`)
 *
 * Handles paths like:
 * - spaces               → every space (JSON array, storey ascending)
 * - spaces.<id>          → the space with that row id
 * - spaces.<guid>        → the space with that guid
 * - spaces[i]            → the i-th space
 * - spaces.<id>.<prop>   → one field: id, guid, storey, name, kind, polygon
 *                          ({outer, holes}, rings of [x, y]), z_floor,
 *                          z_ceiling, area, perimeter, volume, compactness,
 *                          room_label, source, locked, confidence, metadata,
 *                          boundaries (the space_boundaries rows: wall,
 *                          side, s_start, s_end)
 *
 * The spaces of one storey: `rux get storeys.<idx>.spaces`. Coordinates are
 * in that storey's level frame. `rux del spaces` deletes every space and
 * boundary; when some spaces are locked (edited by hand) it needs --force.
 * The rows are written by `rux create spaces`, not by `rux set`.
 */
class SpaceRouter : public ResourceRouter {
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

/// One space as the JSON `rux get spaces` prints, with its boundaries.
nlohmann::json
space_to_json(const reusex::core::SpaceRecord &s,
              const std::vector<reusex::core::SpaceBoundaryRecord> &bounds);

/// The spaces of storey @p storey (every space when nullopt).
nlohmann::json spaces_json(const reusex::ProjectDB &db,
                           std::optional<int> storey = std::nullopt);

} // namespace rux::database
