// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for the storeys table (schema v30, `rux create storeys`)
 *
 * Handles paths like:
 * - storeys               → every storey (JSON array, idx ascending)
 * - storeys.<idx>         → the storey with that idx (also storeys/<idx>)
 * - storeys.<guid>        → the storey with that guid
 * - storeys[i]            → the i-th storey
 * - storeys.<idx>.<prop>  → one field: guid, idx, name, z_floor, z_ceiling,
 *                           level_transform, source, locked, confidence,
 *                           metadata
 * - storeys.<idx>.walls   → the walls standing on that storey (walls table,
 *                           `rux create walls`)
 *
 * `rux del storeys` deletes every storey; when some are locked (hand-made
 * or hand-fixed) it needs --force. The rows are written by
 * `rux create storeys`, not by `rux set`.
 */
class StoreyRouter : public ResourceRouter {
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

/// One storey as the JSON `rux get storeys` prints.
nlohmann::json storey_to_json(const reusex::core::StoreyRecord &s);

} // namespace rux::database
