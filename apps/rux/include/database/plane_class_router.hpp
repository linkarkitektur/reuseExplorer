// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "resource_router.hpp"

namespace rux::database {

/**
 * @brief Router for the plane_classes table (schema v31, `rux create
 * classify`), for the planes of the "planes" cloud
 *
 * Handles paths like:
 * - plane_classes                 → every plane (JSON array, label ascending)
 * - plane_classes.<label>         → the plane with that plane label
 * - plane_classes.<guid>          → the plane with that guid
 * - plane_classes[i]              → the i-th row
 * - plane_classes.<label>.<prop>  → one field: plane_label, guid,
 *                                   orientation, semantic_class,
 *                                   class_confidence, role, role_source,
 *                                   storey, point_count, area, z_min, z_max,
 *                                   source, locked, metadata
 *
 * `rux del plane_classes` deletes every row; when some are locked (edited by
 * hand) it needs --force. The rows are written by `rux create classify`, not
 * by `rux set`. Another label cloud's classes: `rux get clouds.<name>.classes`.
 */
class PlaneClassRouter : public ResourceRouter {
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

/// One plane class as the JSON `rux get plane_classes` prints.
nlohmann::json plane_class_to_json(const reusex::core::PlaneClassRecord &p);

/// Every plane class of the label cloud @p cloud, as a JSON array.
nlohmann::json plane_classes_json(const reusex::ProjectDB &db,
                                  std::string_view cloud);

} // namespace rux::database
