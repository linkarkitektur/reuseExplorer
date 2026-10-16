// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "info.hpp"
#include "exit_status.hpp"
#include <nlohmann/json.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_fingerprint.hpp>
#include <spdlog/spdlog.h>

#include <fmt/color.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <range/v3/all.hpp>

#include <filesystem>
#include <iostream>
#include <locale>

using json = nlohmann::json;

namespace {

// Format terminal output with detailed information
void format_terminal_output(const reusex::ProjectDB::ProjectSummary &summary) {
  // Enable thousand separators for numbers
  std::locale loc("");

  constexpr std::string_view indent = "  ";

  fmt::print("Project: {}\n", summary.path.string());
  fmt::print(
      "Schema Version: {}\n\n",
      fmt::styled(summary.schema_version, fmt::fg(fmt::terminal_color::red)));

  // Projects section
  if (!summary.projects.empty()) {
    fmt::print("{}",
               fmt::format(loc, "Projects: {:L}\n", summary.projects.size()));

    for (const auto &project : summary.projects) {
      // Build main project line with available fields
      std::vector<std::string> parts;

      if (!project.name.empty()) {
        parts.push_back(fmt::format("\"{}\"", project.name));
      }

      if (!project.building_address.empty()) {
        parts.push_back(project.building_address);
      }

      if (project.year_of_construction > 0) {
        parts.push_back(fmt::format("Built: {}", project.year_of_construction));
      }

      if (!project.survey_date.empty()) {
        parts.push_back(fmt::format("Survey: {}", project.survey_date));
      }

      if (!project.survey_organisation.empty()) {
        parts.push_back(fmt::format("Org: {}", project.survey_organisation));
      }

      // Display ID and main info on one line
      fmt::print("{}{}  {}\n", indent,
                 fmt::styled(project.id, fmt::fg(fmt::terminal_color::cyan)),
                 fmt::join(parts, " | "));

      // Display notes on indented line if present
      if (!project.notes.empty()) {
        fmt::print("{}  Note: {}\n", indent, project.notes);
      }
    }

    fmt::print("\n");
  }

  // Sensor frames section
  auto sensor_frame_shape =
      summary.sensor_frames.total_count > 0
          ? fmt::format(loc, "({}x{})", summary.sensor_frames.width,
                        summary.sensor_frames.height)
          : "";
  fmt::print("{}", fmt::format(loc, "Sensor Frames: {:L} frames {}\n",
                               summary.sensor_frames.total_count,
                               sensor_frame_shape));

  // Per-scan breakdown (only when more than one import session is present)
  if (summary.sensor_frames.scans.size() > 1) {
    for (const auto &scan : summary.sensor_frames.scans) {
      std::string basename =
          scan.source_path.empty()
              ? "(legacy)"
              : std::filesystem::path(scan.source_path).filename().string();
      fmt::print("{}{} scan {}  {} frames  source: {}\n", indent, indent,
                 fmt::styled(scan.scan_id, fmt::fg(fmt::terminal_color::cyan)),
                 fmt::format(loc, "{:L}", scan.frame_count), basename);
    }
  }

  // Panoramic images section
  if (summary.panoramic_images.total_count > 0) {
    fmt::print("{}", fmt::format(loc, "Panoramic Images: {:L} ({:L} matched)\n",
                                 summary.panoramic_images.total_count,
                                 summary.panoramic_images.matched_count));
  }

  fmt::print("\n");

  // Point clouds section
  fmt::print("{}",
             fmt::format(loc, "Point Clouds: {:L}\n", summary.clouds.size()));

  int clouds_max_length =
      ranges::max_element(summary.clouds, std::less<>{}, [](auto &item) {
        return item.name.size();
      })->name.size();

  for (const auto &cloud : summary.clouds) {
    std::string org = cloud.organized ? "O" : "U";
    auto org_color = cloud.organized ? fmt::terminal_color::green
                                     : fmt::terminal_color::yellow;

    auto dims = fmt::format(
        loc, "[{:>8L}{}{:L}]{}", cloud.width,
        fmt::styled("x", fmt::fg(fmt::terminal_color::bright_black)),
        cloud.height, fmt::styled(org, fmt::fg(org_color)));

    auto labels =
        cloud.type == "Label" && !cloud.labels.empty()
            ? fmt::format(loc, "({:L} labels) {} {}", cloud.labels.size(),
                          fmt::join(cloud.labels | ranges::views::take(5) |
                                        ranges::views::values,
                                    " "),
                          cloud.labels.size() > 5 ? "..." : "")
            : "";
    auto type = fmt::format(
        "{:<13}",
        fmt::styled(cloud.type, fmt::fg(fmt::terminal_color::bright_blue)));

    fmt::print("{}", fmt::format(loc, "{}{:<{}} {} {} {}\n", indent, cloud.name,
                                 clouds_max_length, type, dims, labels));
  }
  fmt::print("\n");

  fmt::print("{}", fmt::format(loc, "Meshes: {:L}\n", summary.meshes.size()));
  int mesh_max_length =
      ranges::max_element(summary.meshes, std::less<>{}, [](auto &item) {
        return item.name.size();
      })->name.size();

  for (const auto &mesh : summary.meshes) {
    fmt::print("{}", fmt::format(loc, "{}{:<{}}  V:{:<6L}  F:{:L}\n", indent,
                                 mesh.name, mesh_max_length, mesh.vertex_count,
                                 mesh.face_count));
  }
  fmt::print("\n");

  // Material passports section
  fmt::print("{}",
             fmt::format(loc, "Materials: {:L}\n", summary.materials.size()));

  int material_max_length = 0;
  if (!summary.materials.empty()) {
    material_max_length =
        ranges::max_element(summary.materials, std::less<>{}, [](auto &item) {
          return item.id.size();
        })->id.size();
  }

  for (const auto &material : summary.materials) {
    fmt::print(
        "{}",
        fmt::format(loc, "{}{:<{}} {:>8} - Ver:{:<8} Props:{:<4L}\n", indent,
                    fmt::styled(material.id,
                                fmt::fg(fmt::terminal_color::bright_black)),
                    material_max_length, material.guid, material.version_number,
                    material.property_count));
  }
}

// Format JSON output
void format_json_output(const reusex::ProjectDB::ProjectSummary &summary) {
  json j;

  j["project_path"] = summary.path.string();
  j["schema_version"] = summary.schema_version;

  // Projects
  j["projects"] = json::array();
  for (const auto &project : summary.projects) {
    json project_json = {{"id", project.id},
                         {"name", project.name},
                         {"building_address", project.building_address},
                         {"survey_date", project.survey_date},
                         {"survey_organisation", project.survey_organisation},
                         {"notes", project.notes}};

    // Use null for unset year (JSON best practice)
    if (project.year_of_construction > 0) {
      project_json["year_of_construction"] = project.year_of_construction;
    } else {
      project_json["year_of_construction"] = nullptr;
    }

    j["projects"].push_back(project_json);
  }

  // Sensor frames
  j["sensor_frames"] = {{"total", summary.sensor_frames.total_count},
                        {"width", summary.sensor_frames.width},
                        {"height", summary.sensor_frames.height},
                        {"segmented", summary.sensor_frames.segmented_count}};

  j["sensor_frames"]["scans"] = json::array();
  for (const auto &scan : summary.sensor_frames.scans) {
    j["sensor_frames"]["scans"].push_back({{"scan_id", scan.scan_id},
                                           {"source_path", scan.source_path},
                                           {"imported_at", scan.imported_at},
                                           {"frame_count", scan.frame_count}});
  }

  // Panoramic images
  j["panoramic_images"] = {{"total", summary.panoramic_images.total_count},
                           {"matched", summary.panoramic_images.matched_count}};

  // Point clouds
  j["point_clouds"] = json::array();
  for (const auto &cloud : summary.clouds) {
    json cloud_json = {{"name", cloud.name},
                       {"type", cloud.type},
                       {"point_count", cloud.point_count},
                       {"width", cloud.width},
                       {"height", cloud.height},
                       {"organized", cloud.organized}};

    // Add labels for Label clouds
    if (cloud.type == "Label" && !cloud.labels.empty()) {
      json labels_json;
      for (const auto &[id, name] : cloud.labels) {
        labels_json[std::to_string(id)] = name;
      }
      cloud_json["labels"] = labels_json;
    }

    j["point_clouds"].push_back(cloud_json);
  }

  // Meshes
  j["meshes"] = json::array();
  for (const auto &mesh : summary.meshes) {
    j["meshes"].push_back({{"name", mesh.name},
                           {"vertex_count", mesh.vertex_count},
                           {"face_count", mesh.face_count}});
  }

  // Material passports
  j["material_passports"] = json::array();
  for (const auto &material : summary.materials) {
    j["material_passports"].push_back(
        {{"id", material.id},
         {"guid", material.guid},
         {"property_count", material.property_count},
         {"created_at", material.created_at},
         {"version_number", material.version_number}});
  }

  // Output pretty-printed JSON to stdout
  fmt::print("{}\n", j.dump(2));
}

} // anonymous namespace

void setup_subcommand_info(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandInfoOptions>();

  auto *sub = app.add_subcommand("info", "Display project database summary");

  sub->footer(R"(
DESCRIPTION:
  Displays comprehensive summary information about a ReUseX project database
  including sensor frames, point clouds, meshes, building components, and
  material passports. Provides both human-readable terminal output and
  machine-readable JSON for scripting.

EXAMPLES:
  rux info                             # Terminal formatted output
  rux info --json                      # JSON output for scripting
  rux -p scan.rux info                 # Custom project path
  rux info --json | jq '.point_clouds'  # Query with jq

OUTPUT INCLUDES:
  - Project path and schema version
  - Sensor frame count and dimensions
  - Point clouds: name, type, size, organization
  - Meshes: name, vertex count, face count
  - Material passport count
  - Label definitions for segmented clouds

NOTES:
  - Default output: human-readable terminal format with thousand separators
  - JSON mode: structured output suitable for parsing and automation
  - Use 'rux get' for detailed queries of specific resources
  - Fast read-only operation (does not modify database)
)");

  sub->add_flag("-j,--json", opt->json_output, "Output in JSON format")
      ->default_val(false);

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running info subcommand");
    rux::finish(run_subcommand_info(*opt, *global_opt));
  });
}

int run_subcommand_info(SubcommandInfoOptions const &opt,
                        const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    // Open project database in read-only mode
    reusex::ProjectDB db(project_path, /* readOnly */ true);

    // Get project summary
    auto summary = db.project_summary();

    // A stale cloud (built from poses `rux optimize`/`rux register` has since
    // superseded) is not visible in any of the counts above — warn
    // separately (reusex-rcv.1.15). Logged, not printed, so it never corrupts
    // `rux info --json` for a script piping it to jq.
    reusex::core::warn_if_cloud_poses_stale(db, "cloud");

    // Format output
    if (opt.json_output) {
      format_json_output(summary);
    } else {
      format_terminal_output(summary);
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to read project info: {}", e.what());
    return RuxError::IO;
  }
}
