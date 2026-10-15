// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "segmentation/sync_downsample.hpp"

#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "segmentation/downsample.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::geometry {

namespace {

// `params_json` plus the label-encoding marker when the cloud being replaced
// carried it, so a downsampled semantic cloud is not re-encoded by a later
// migration (STANDARDS §3.1).
std::string with_label_encoding(ProjectDB &db, const std::string &name,
                                const std::string &params_json) {
  nlohmann::json old;
  try {
    old = nlohmann::json::parse(db.point_cloud_provenance(name).second);
  } catch (const std::exception &) {
    return params_json;
  }
  if (!old.is_object() || !old.contains(core::kLabelEncodingKey))
    return params_json;
  auto out = nlohmann::json::parse(params_json);
  out[core::kLabelEncodingKey] = old[core::kLabelEncodingKey];
  return out.dump();
}

// A sibling cloud discovered by point-count match.
struct Sibling {
  std::string name;
  std::string type; // "PointXYZRGB", "Normal", "Label"
};

// Enumerate every point cloud in `db` (other than `primary`) whose point count
// equals `primary_count`. These are the index-aligned parallel clouds that
// must move together with the primary (docs/STANDARDS.md §3.2).
std::vector<Sibling> discover_siblings(ProjectDB &db,
                                       const std::string &primary,
                                       size_t primary_count) {
  std::vector<Sibling> siblings;
  const auto summary = db.project_summary();
  for (const auto &c : summary.clouds) {
    if (c.name == primary)
      continue;
    if (c.point_count != primary_count)
      continue;
    siblings.push_back({c.name, c.type});
  }
  // Deterministic ordering for reproducible logs/tests.
  std::sort(siblings.begin(), siblings.end(),
            [](const Sibling &a, const Sibling &b) { return a.name < b.name; });
  return siblings;
}

} // namespace

SyncDownsampleResult sync_downsample(ProjectDB &db, const std::string &primary,
                                     const SyncDownsampleOptions &opts) {
  if (!(opts.leaf_size > 0.0f))
    throw std::invalid_argument("Leaf size must be positive");
  if (!db.has_point_cloud(primary))
    throw std::invalid_argument("Primary cloud '" + primary +
                                "' not found in project");

  const std::string primary_type = db.point_cloud_type(primary);
  if (primary_type != "PointXYZRGB")
    throw std::invalid_argument("Primary cloud '" + primary + "' has type " +
                                primary_type + ", expected PointXYZRGB");

  auto cloud = db.point_cloud_xyzrgb(primary);
  if (!cloud || cloud->empty())
    throw std::invalid_argument("Primary cloud '" + primary + "' is empty");

  const size_t primary_count = cloud->size();
  const std::string primary_output =
      opts.primary_output.empty() ? primary : opts.primary_output;

  auto siblings = discover_siblings(db, primary, primary_count);

  // --- Desync guard (docs/STANDARDS.md §5, loud failure) -----------------
  if (opts.only_primary) {
    if (!siblings.empty() && !opts.force_desync) {
      std::vector<std::string> names;
      names.reserve(siblings.size());
      for (const auto &s : siblings)
        names.push_back(s.name);
      throw std::runtime_error(fmt::format(
          "Refusing to downsample only '{}': it would silently desynchronize "
          "{} index-aligned sibling cloud(s) [{}] which share its point count "
          "({}). Re-run with force_desync to break the alignment on purpose, "
          "or drop only_primary to downsample all of them together.",
          primary, siblings.size(), fmt::join(names, ", "), primary_count));
    }
  }

  const std::string params_json =
      fmt::format(R"({{"primary":"{}","resolution":{},"source_points":{}}})",
                  primary, opts.leaf_size, primary_count);

  const int log_id = db.log_pipeline_start("sync_downsample", params_json);

  SyncDownsampleResult result;
  try {
    core::info(
        "sync_downsample: building voxel assignment for '{}' (leaf={} m)",
        primary, opts.leaf_size);
    auto assignment = voxel_assignment(*cloud, opts.leaf_size);

    auto down = downsample(*cloud, assignment);
    core::info("sync_downsample: primary '{}' -> '{}' ({} -> {} points)",
               primary, primary_output, primary_count, down->size());
    db.save_point_cloud(primary_output, *down, "sync_downsample", params_json);
    result.output_points = down->size();
    result.written_clouds.push_back(primary_output);

    // Release the primary input early; the assignment is all we still need.
    cloud.reset();
    down.reset();

    if (opts.only_primary) {
      for (const auto &s : siblings)
        result.desynced_clouds.push_back(s.name);
      if (!siblings.empty())
        core::warn("sync_downsample: only_primary/force_desync set — {} "
                   "sibling cloud(s) are now stale: [{}]",
                   siblings.size(), fmt::join(result.desynced_clouds, ", "));
    } else {
      for (const auto &s : siblings) {
        if (s.type == "PointXYZRGB") {
          auto par = db.point_cloud_xyzrgb(s.name);
          if (!par)
            throw std::runtime_error("Sibling cloud '" + s.name +
                                     "' could not be loaded");
          if (par->size() != assignment.point_to_bucket.size())
            throw std::runtime_error(
                fmt::format("Sibling '{}' has {} rows, expected {}", s.name,
                            par->size(), assignment.point_to_bucket.size()));
          auto ds = downsample(*par, assignment);
          core::info("sync_downsample: sibling '{}' (PointXYZRGB) -> {} points",
                     s.name, ds->size());
          db.save_point_cloud(s.name, *ds, "sync_downsample", params_json);
        } else if (s.type == "Normal") {
          auto par = db.point_cloud_normal(s.name);
          if (!par)
            throw std::runtime_error("Sibling cloud '" + s.name +
                                     "' could not be loaded");
          if (par->size() != assignment.point_to_bucket.size())
            throw std::runtime_error(
                fmt::format("Sibling '{}' has {} rows, expected {}", s.name,
                            par->size(), assignment.point_to_bucket.size()));
          auto ds = downsample(*par, assignment);
          core::info("sync_downsample: sibling '{}' (Normal) -> {} entries",
                     s.name, ds->size());
          db.save_point_cloud(s.name, *ds, "sync_downsample", params_json);
        } else if (s.type == "Label") {
          auto par = db.point_cloud_label(s.name);
          if (!par)
            throw std::runtime_error("Sibling cloud '" + s.name +
                                     "' could not be loaded");
          if (par->size() != assignment.point_to_bucket.size())
            throw std::runtime_error(
                fmt::format("Sibling '{}' has {} rows, expected {}", s.name,
                            par->size(), assignment.point_to_bucket.size()));
          auto ds = downsample(*par, assignment);
          core::info("sync_downsample: sibling '{}' (Label) -> {} entries "
                     "(majority vote)",
                     s.name, ds->size());
          // Preserve semantic label definitions and the label encoding
          // marker (STANDARDS §3.1) across the operation.
          auto defs = db.label_definitions(s.name);
          db.save_point_cloud(s.name, *ds, "sync_downsample",
                              with_label_encoding(db, s.name, params_json));
          if (!defs.empty())
            db.save_label_definitions(s.name, defs);
        } else {
          throw std::runtime_error(fmt::format(
              "Sibling cloud '{}' has unsupported type '{}'; refusing to leave "
              "it desynchronized. Supported: PointXYZRGB, Normal, Label.",
              s.name, s.type));
        }
        result.written_clouds.push_back(s.name);
      }
    }

    db.log_pipeline_end(log_id, true);
  } catch (...) {
    db.log_pipeline_end(log_id, false, "sync_downsample failed");
    throw;
  }

  return result;
}

} // namespace reusex::geometry
