// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "analyze/solid.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <pcl/PolygonMesh.h>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string read_text(const std::string &path) {
  std::ifstream in(path);
  if (!in)
    throw std::runtime_error(fmt::format("cannot read '{}'", path));
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string show(const std::optional<double> &v, double scale = 1.0,
                 const char *unit = "") {
  return v ? fmt::format("{:.4g}{}", *v * scale, unit) : std::string("n/a");
}

void print_summary(const reusex::geometry::SolidModelReport &r) {
  std::cout << fmt::format(
      "room count error      {}\n"
      "room IoU (mean)       {}\n"
      "wall recall           {}  ({} of {} reference walls)\n"
      "wall precision        {}  ({} detected walls)\n"
      "wall position RMS     {}\n"
      "wall thickness error  mean {} / P95 {}\n"
      "opening recall        {}  ({} of {} reference openings)\n"
      "opening precision     {}  ({} detected openings)\n"
      "fit mean              {}  ({} points)\n"
      "fit within            {}\n"
      "fit source            {}\n"
      "seen share (mean)     {}\n"
      "assumed share         {}\n",
      !r.room_count_error        ? std::string("n/a")
      : *r.room_count_error == 0 ? std::string("0")
                                 : fmt::format("{:+d}", *r.room_count_error),
      show(r.room_iou_mean), show(r.wall_recall), r.matched_walls,
      r.reference_walls, show(r.wall_precision), r.detected_walls,
      show(r.wall_position_rms_m, 1000.0, " mm"),
      show(r.wall_thickness_err_mean_m, 1000.0, " mm"),
      show(r.wall_thickness_err_p95_m, 1000.0, " mm"), show(r.opening_recall),
      r.matched_openings, r.reference_openings, show(r.opening_precision),
      r.detected_openings, show(r.fit_mean_m, 1000.0, " mm"), r.fit_points,
      show(r.fit_within_share),
      r.fit_source.empty() ? std::string("n/a") : r.fit_source,
      show(r.seen_share_mean), show(r.assumed_share));
}

/// Say why a block of metrics is n/a (STANDARDS §5: no silent n/a).
void explain_missing(const reusex::core::SolidModel &det,
                     const reusex::core::SolidModel &ref,
                     const std::string &det_name) {
  auto note = [&](const char *kind, bool d, bool r) {
    if (d && r)
      return;
    spdlog::warn("{} metrics are n/a: {} provides no {}s{}", kind,
                 !d ? det_name : std::string("the reference"), kind,
                 !d && det_name == "the project"
                     ? " (its solid-model stage has not run or does not "
                       "exist yet)"
                     : "");
  };
  note("wall", det.has_walls(), ref.has_walls());
  note("space", det.has_spaces(), ref.has_spaces());
  note("opening", det.has_openings(), ref.has_openings());
}

} // namespace

void setup_subcommand_analyze_solid(CLI::App &app,
                                    std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandAnalyzeSolidOptions>();
  auto *sub = app.add_subcommand(
      "solid", "Score the typed building model against a reference plan");

  sub->footer(R"(
DESCRIPTION:
  Compares the project's typed building model (storeys, walls, spaces,
  openings) with a reference plan in the solid-model JSON format
  (docs/design/solid-model-json.md), and measures how well the scan cloud
  fits the solid mesh:

  - room count error, room IoU (2D polygons, one-to-one matched)
  - wall recall / precision, wall position RMS, wall thickness error
  - opening recall / precision (IoU in the wall face)
  - point-to-solid fit: mean distance, share within --fit-within
  - seen share / assumed share of the detected walls

  A metric is n/a (JSON null) when either side does not provide the element
  kind it needs, e.g. before the walls stage exists. The reference is a
  benchmark target, not ground truth (decision D5).

EXAMPLES:
  rux -p scan.rux analyze solid --reference plan.json
  rux -p scan.rux analyze solid --reference plan.json --json -o report.json
  rux analyze solid --reference plan.json --model baseline.json --json

NOTES:
  - Metres, z up. The models may use different level frames: each storey's
    level_transform takes its elements to world before matching, and when
    both list storeys, storeys are paired by floor height in world z.
  - --model scores a model file (e.g. a baseline converted from another
    tool) instead of the project's. The fit metrics are then n/a unless
    --cloud or --mesh is passed explicitly (they would measure the
    project's mesh, not the model file). "fit_source" in the report always
    names the project, cloud and mesh the fit came from.
  - Detected elements on storeys the reference does not cover are not
    scored (counts.detected_outside_reference).
)");

  sub->add_option("--reference", opt->reference_path,
                  "Reference plan (solid-model JSON)")
      ->required()
      ->check(CLI::ExistingFile);
  sub->add_option("--model", opt->model_path,
                  "Score this solid-model JSON instead of the project's model")
      ->check(CLI::ExistingFile);
  sub->add_option("--cloud", opt->cloud_name, "Scan cloud for the fit metrics")
      ->default_val(opt->cloud_name);
  sub->add_option("--mesh", opt->mesh_name, "Solid mesh for the fit metrics")
      ->default_val(opt->mesh_name);
  sub->add_flag("--json", opt->json, "Print the JSON report to stdout");
  sub->add_option("-o,--output", opt->output_path,
                  "Also write the JSON report to this file");

  auto &m = opt->metrics;
  sub->add_option("--wall-overlap", m.wall_match_min_overlap,
                  "Min overlap (1D IoU along the reference axis) of matched "
                  "walls")
      ->default_val(m.wall_match_min_overlap)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--wall-angle", m.wall_match_angle_deg,
                  "Max angle between matched wall axes [deg]")
      ->default_val(m.wall_match_angle_deg)
      ->check(CLI::PositiveNumber);
  sub->add_option("--wall-dist", m.wall_match_dist_m,
                  "Max distance of a matched wall from the reference axis [m]")
      ->default_val(m.wall_match_dist_m)
      ->check(CLI::PositiveNumber);
  sub->add_option("--opening-iou", m.opening_match_iou,
                  "Min face IoU for an opening match")
      ->default_val(m.opening_match_iou)
      ->check(CLI::Range(0.0, 1.0));
  sub->add_option("--fit-within", m.fit_within_m,
                  "Distance a fitting point may be from the solid [m]")
      ->default_val(m.fit_within_m)
      ->check(CLI::PositiveNumber);

  sub->callback([opt, global_opt, sub]() {
    spdlog::trace("calling analyze solid subcommand");
    opt->fit_requested = sub->count("--cloud") + sub->count("--mesh") > 0;
    rux::finish(run_subcommand_analyze_solid(*opt, *global_opt));
  });
}

int run_subcommand_analyze_solid(SubcommandAnalyzeSolidOptions const &opt,
                                 const RuxOptions &global_opt) {
  using namespace reusex;
  core::SolidModel reference;
  try {
    reference = io::solid_model_from_json(read_text(opt.reference_path));
  } catch (const std::exception &e) {
    spdlog::error("Reference plan '{}': {}", opt.reference_path, e.what());
    return RuxError::INVALID_ARGUMENT;
  }

  try {
    const bool from_file = !opt.model_path.empty();
    core::SolidModel detected;
    if (from_file) {
      try {
        detected = io::solid_model_from_json(read_text(opt.model_path));
      } catch (const std::exception &e) {
        spdlog::error("Model '{}': {}", opt.model_path, e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    std::unique_ptr<ProjectDB> db;
    if (!from_file || fs::exists(global_opt.project_db)) {
      if (!fs::exists(global_opt.project_db)) {
        spdlog::error("Project '{}' not found (pass -p, or --model)",
                      global_opt.project_db.string());
        return RuxError::INVALID_ARGUMENT;
      }
      db = std::make_unique<ProjectDB>(global_opt.project_db,
                                       /*readOnly=*/true);
      if (!from_file)
        detected = io::load_solid_model(*db);
    }

    explain_missing(detected, reference,
                    from_file ? fmt::format("'{}'", opt.model_path)
                              : std::string("the project"));
    auto report =
        geometry::compare_solid_models(detected, reference, opt.metrics);

    if (from_file && !opt.fit_requested) {
      spdlog::warn("fit metrics are n/a: --model scores a model file; pass "
                   "--mesh/--cloud to fit the project's mesh as well");
    } else if (!db || !db->has_point_cloud(opt.cloud_name) ||
               !db->has_mesh(opt.mesh_name)) {
      spdlog::warn("fit metrics are n/a: {}",
                   !db ? std::string("no project to read a cloud and mesh from")
                       : fmt::format("the project lacks cloud '{}' or mesh "
                                     "'{}'",
                                     opt.cloud_name, opt.mesh_name));
    } else {
      const auto cloud = db->point_cloud_xyzrgb(opt.cloud_name);
      const auto mesh = db->mesh(opt.mesh_name);
      try {
        geometry::fit_report(*cloud, *mesh, opt.metrics, report);
        report.fit_source = fmt::format("project '{}': cloud '{}' vs mesh '{}'",
                                        global_opt.project_db.string(),
                                        opt.cloud_name, opt.mesh_name);
      } catch (const std::invalid_argument &e) {
        // The model metrics stand on their own (STANDARDS §5: say why).
        spdlog::warn("fit metrics are n/a: {} (cloud '{}', mesh '{}')",
                     e.what(), opt.cloud_name, opt.mesh_name);
      }
    }

    const std::string json = report.to_json();
    if (opt.json)
      std::cout << json << '\n';
    else
      print_summary(report);
    if (!opt.output_path.empty()) {
      std::ofstream out(opt.output_path);
      if (!out) {
        spdlog::error("Cannot write to '{}'", opt.output_path);
        return RuxError::IO;
      }
      out << json << '\n';
      spdlog::info("Report written to {}", opt.output_path);
    }
    return RuxError::SUCCESS;
  } catch (const std::invalid_argument &e) {
    spdlog::error("Solid-model analysis: invalid input: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  } catch (const std::exception &e) {
    spdlog::error("Solid-model analysis failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
