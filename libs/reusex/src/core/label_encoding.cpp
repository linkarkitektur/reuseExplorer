// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/label_encoding.hpp"

#include "core/ProjectDB.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"

#include <nlohmann/json.hpp>

namespace reusex::core {

namespace {
constexpr int kEncodingSchemaVersion = 29;

bool is_structural_stage(std::string_view stage) {
  return stage == "segment_planes" || stage == "segment_rooms" ||
         stage == "segment_instances";
}
} // namespace

bool is_semantic_label_cloud(std::string_view name, std::string_view stage) {
  if (is_structural_stage(stage))
    return false;
  return name == "labels" || stage == "reconstruct" ||
         stage == "project_labels";
}

bool has_api_plus_one_encoding(std::string_view params_json) {
  if (params_json.empty())
    return false;
  try {
    const auto j = nlohmann::json::parse(params_json);
    if (!j.is_object())
      return false;
    const auto it = j.find(kLabelEncodingKey);
    return it != j.end() && it->is_string() &&
           it->get<std::string>() == kLabelEncodingApiPlusOne;
  } catch (const nlohmann::json::exception &) {
    return false;
  }
}

std::string stamp_label_encoding(std::string_view params_json) {
  nlohmann::json j = nlohmann::json::object();
  if (!params_json.empty()) {
    try {
      auto parsed = nlohmann::json::parse(params_json);
      if (parsed.is_object())
        j = std::move(parsed);
    } catch (const nlohmann::json::exception &) {
    }
  }
  j[kLabelEncodingKey] = kLabelEncodingApiPlusOne;
  return j.dump();
}

bool is_unstamped_semantic_cloud(const ProjectDB &db, std::string_view cloud) {
  if (db.schema_version() < kEncodingSchemaVersion ||
      !db.has_point_cloud(cloud) || db.point_cloud_type(cloud) != "Label")
    return false;
  const auto [stage, params] = db.point_cloud_provenance(cloud);
  // The caller reads it as semantic; only a structural writer overrules that.
  return !is_structural_stage(stage) && !has_api_plus_one_encoding(params);
}

bool warn_if_unstamped_semantic_cloud(const ProjectDB &db,
                                      std::string_view cloud) {
  if (!is_unstamped_semantic_cloud(db, cloud))
    return false;
  warn("semantic label cloud '{}' (stage '{}') has no \"{}\": \"{}\" in its "
       "parameters although the project is at schema {}. It was probably "
       "written by a pre-v29 binary, so its class ids may be one too low "
       "(API class 0 lost as unlabeled); re-run `rux create clouds` or `rux "
       "create project` with this version",
       cloud, db.point_cloud_provenance(cloud).first, kLabelEncodingKey,
       kLabelEncodingApiPlusOne, db.schema_version());
  return true;
}

} // namespace reusex::core
