// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for the openings table (schema v34, `rux create openings`)
 *
 * Handles paths like:
 * - openings               → every opening (JSON array, by host wall, s)
 * - openings.<id>          → the opening with that row id
 * - openings.<guid>        → the opening with that guid
 * - openings[i]            → the i-th opening
 * - openings.<id>.<prop>   → one field: id, guid, wall, kind, s_start, s_end,
 *                            width, z_sill, z_head, height, state,
 *                            is_external, source_instance_guid, source,
 *                            locked, confidence, metadata
 *
 * The openings of one wall: `rux get walls.<id|guid>.openings`. s runs along
 * the host wall's axis from its (x0, y0); z is in its storey's level frame.
 * `rux del openings` deletes every opening; when some are locked (edited by
 * hand) it needs --force. The rows are written by `rux create openings`, not
 * by `rux set`.
 */
class OpeningRouter : public ResourceRouter {
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

/// One opening as the JSON `rux get openings` prints.
nlohmann::json opening_to_json(const reusex::core::OpeningRecord &o);

/// The openings in the wall with guid @p wall (every opening when empty).
nlohmann::json openings_json(const reusex::ProjectDB &db,
                             const std::string &wall = {});

} // namespace rux::database
