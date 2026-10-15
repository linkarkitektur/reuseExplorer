// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file normal_orientation.hpp
/// @brief Which way a stored cloud's normals face (reusex-0se).
///
/// `rux create clouds` orients every normal toward the sensor that saw the
/// point (reusex-0pi; CONTRACTS.md, `clouds`): a floor's normal points up, a
/// ceiling's down. Before that, normals faced the world origin, and nothing in
/// the `.rux` told the two apart. The clouds stage now stamps
/// `"normal_orientation": "sensor"` into the parameters of `cloud` and
/// `normals`, and the planes stage carries it over to `plane_normals`. A cloud
/// without the key is legacy (or imported, e.g. E57): its normal signs carry
/// no meaning a stage may rely on.

#include <optional>
#include <string>
#include <string_view>

namespace reusex::core {

/// Key in a cloud's parameters JSON (point_clouds.parameters).
inline constexpr const char *kNormalOrientationKey = "normal_orientation";

/// Value of kNormalOrientationKey: each normal faces the sensor that saw it.
inline constexpr const char *kNormalOrientationSensor = "sensor";

/// The normal_orientation recorded in a cloud's parameters JSON text, or
/// std::nullopt when the key is absent, not a string, or the text is not a
/// JSON object (never throws: legacy provenance may be anything).
std::optional<std::string>
normal_orientation_of(std::string_view parameters_json) noexcept;

} // namespace reusex::core
