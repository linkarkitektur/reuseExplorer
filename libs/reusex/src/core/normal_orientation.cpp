// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/normal_orientation.hpp"

#include <nlohmann/json.hpp>

namespace reusex::core {

std::optional<std::string>
normal_orientation_of(std::string_view parameters_json) noexcept {
  try {
    const auto j = nlohmann::json::parse(parameters_json, nullptr,
                                         /*allow_exceptions=*/false);
    if (!j.is_object())
      return std::nullopt;
    const auto it = j.find(kNormalOrientationKey);
    if (it == j.end() || !it->is_string())
      return std::nullopt;
    return it->get<std::string>();
  } catch (...) {
    return std::nullopt;
  }
}

} // namespace reusex::core
