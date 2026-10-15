// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/annotate_class_map.hpp"

#include "core/ProjectDB.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"

#include <nlohmann/json.hpp>

#include <string_view>

namespace reusex::core {

namespace {
constexpr std::string_view kClassMapStage = "annotate_class_map";
} // namespace

std::map<int, std::string> annotated_label_definitions(const ProjectDB &db) {
  // Pick the latest entry by id explicitly rather than trusting the order
  // pipeline_log() happens to return.
  const ProjectDB::PipelineLogEntry *latest = nullptr;
  const auto log = db.pipeline_log();
  for (const auto &entry : log)
    if (entry.stage == kClassMapStage && !entry.parameters.empty() &&
        (latest == nullptr || entry.id > latest->id))
      latest = &entry;
  if (latest == nullptr)
    return {};

  std::map<int, std::string> definitions;
  try {
    const auto j = nlohmann::json::parse(latest->parameters);
    for (const auto &[key, val] : j.items())
      definitions[static_cast<int>(api_to_point_label(std::stoi(key)))] =
          val.get<std::string>();
  } catch (const std::exception &e) {
    warn("Failed to parse the annotate class map of pipeline_log entry {}: {} "
         "— the label cloud gets no class names",
         latest->id, e.what());
    return {};
  }
  return definitions;
}

} // namespace reusex::core
