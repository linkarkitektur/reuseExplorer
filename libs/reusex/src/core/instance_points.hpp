// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Private to reusex_core: the one instance-cloud loader behind both
// core::instance_points (frame_projection.hpp) and core::instance_probes
// (instance_evidence.hpp), so their checks and errors cannot drift apart.

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core::detail {

/// core::instance_points, with @p caller prefixing the invalid_argument and
/// runtime_error messages (e.g. "instance_probes").
std::map<std::uint32_t, std::vector<Eigen::Vector3f>>
load_instance_points(const ProjectDB &db, std::string_view label_cloud,
                     std::string_view positions_cloud, std::string_view caller);

} // namespace reusex::core::detail
