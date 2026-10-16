// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "slam/optimize_parameters.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace reusex::geometry {

void apply_optimize_parameters(PlaneGraphOptions &options, bool &dry_run,
                               std::string_view parameters) {
  nlohmann::json params = nlohmann::json::object();
  if (!parameters.empty()) {
    params = nlohmann::json::parse(parameters, nullptr,
                                   /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("optimize parameters must be a JSON object");
  }
  auto get = [&](const char *key, auto fallback) {
    auto it = params.find(key);
    if (it == params.end() || it->is_null())
      return fallback;
    try {
      return it->template get<decltype(fallback)>();
    } catch (const nlohmann::json::exception &e) {
      throw std::invalid_argument(std::string("optimize parameter '") + key +
                                  "': " + e.what());
    }
  };
  options.min_landmark_observations =
      get("min_observations", options.min_landmark_observations);
  options.assoc_rounds = get("assoc_rounds", options.assoc_rounds);
  if (get("no_gnc", false))
    options.use_gnc = false;
  options.manhattan = get("manhattan", options.manhattan);
  options.manhattan_sigma_deg =
      get("manhattan_sigma_deg", options.manhattan_sigma_deg);
  options.manhattan_max_angle_deg =
      get("manhattan_max_angle_deg", options.manhattan_max_angle_deg);
  options.manhattan_level_max_angle_deg = get(
      "manhattan_level_max_angle_deg", options.manhattan_level_max_angle_deg);
  dry_run = get("dry_run", dry_run);
}

} // namespace reusex::geometry
