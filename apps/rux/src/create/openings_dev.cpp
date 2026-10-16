// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Hidden dev driver for the opening sub-steps (reusex-z1v.7): read-only, so
// it can be pointed at a copy of a project between beads, before the
// openings stage and its table exist (W5.3).

#include "create/openings_dev.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/geometry/transform_utils.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <map>
#include <vector>

namespace {

/// The raster as an elevation image (seen from +t, s to the right, z up):
/// solid grey, open white, occluded orange, the kept rectangles in red.
void write_raster_png(const std::filesystem::path &path,
                      const reusex::geometry::FaceRaster &r,
                      const std::vector<reusex::geometry::OpeningRect> &rects,
                      double z_floor) {
  constexpr int px = 6;
  cv::Mat img(r.nz * px, r.ns * px, CV_8UC3);
  for (int iz = 0; iz < r.nz; ++iz)
    for (int is = 0; is < r.ns; ++is) {
      cv::Scalar c;
      switch (r.at(is, iz)) {
      case reusex::geometry::FaceCellState::solid:
        c = cv::Scalar(110, 110, 110);
        break;
      case reusex::geometry::FaceCellState::open:
        c = cv::Scalar(255, 255, 255);
        break;
      case reusex::geometry::FaceCellState::occluded:
        c = cv::Scalar(60, 160, 255);
        break;
      }
      cv::rectangle(img,
                    cv::Rect(is * px, (r.nz - 1 - iz) * px, px - 1, px - 1), c,
                    cv::FILLED);
    }
  for (const auto &o : rects) {
    const auto row = [&](double z) {
      return static_cast<int>(std::lround(
          (r.nz * r.cell - (std::max(z, r.z0) - r.z0)) / r.cell * px));
    };
    (void)z_floor;
    cv::rectangle(
        img,
        cv::Point(static_cast<int>(o.s_start / r.cell * px), row(o.z_head)),
        cv::Point(static_cast<int>(o.s_end / r.cell * px), row(o.z_sill)),
        cv::Scalar(0, 0, 230), 2);
  }
  cv::imwrite(path.string(), img);
}

} // namespace

void setup_subcommand_create_openings_dev(
    CLI::App &app, std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateOpeningsDevOptions>();
  auto &o = opt->openings;
  auto *sub = app.add_subcommand(
      "openings-dev",
      "Dev driver: run the opening sub-steps, write solid-model JSON");
  sub->group(""); // hidden: replaced by `rux create openings` (W5.3)
  sub->add_option("-o,--output", opt->output,
                  "Solid-model JSON with the storeys, walls and openings");
  sub->add_option("--raster-dir", opt->raster_dir,
                  "Write each wall's face raster as <wall id>.png here");
  sub->add_option("--cell", o.cell_m)->default_val(o.cell_m);
  sub->add_option("--slab-margin", o.slab_margin_m)
      ->default_val(o.slab_margin_m);
  sub->add_option("--front", o.front_m)->default_val(o.front_m);
  sub->add_option("--min-wall-points", o.min_wall_points)
      ->default_val(o.min_wall_points);
  sub->add_option("--min-occluder-points", o.min_occluder_points)
      ->default_val(o.min_occluder_points);
  sub->add_option("--morph-radius", o.morph_radius)
      ->default_val(o.morph_radius);
  sub->add_option("--min-width", o.min_width_m)->default_val(o.min_width_m);
  sub->add_option("--min-height", o.min_height_m)->default_val(o.min_height_m);
  sub->add_option("--min-fill", o.min_fill)->default_val(o.min_fill);
  sub->add_option("--min-jamb-share", o.min_jamb_share)
      ->default_val(o.min_jamb_share);
  sub->add_flag("!--keep-top-bands", o.drop_top_bands,
                "Keep rectangles that reach only the top of the face");
  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_create_openings_dev(*opt, *global_opt));
  });
}

int run_subcommand_create_openings_dev(
    SubcommandCreateOpeningsDevOptions const &opt,
    const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db, /*readOnly=*/true);
    if (!db.has_point_cloud("cloud") || db.storeys().empty() ||
        !db.has_walls()) {
      spdlog::error("openings-dev: needs the cloud and the storeys and walls "
                    "tables (rux create storeys, classify, walls)");
      return RuxError::INVALID_ARGUMENT;
    }
    auto model = reusex::io::load_solid_model(db);
    model.name = "openings-dev";
    const auto cloud = db.point_cloud_xyzrgb("cloud");
    if (!opt.raster_dir.empty())
      std::filesystem::create_directories(opt.raster_dir);

    for (const auto &st : model.storeys) {
      const Eigen::Affine3d T = reusex::geometry::to_affine(st.level_transform);
      std::vector<Eigen::Vector3d> pts;
      for (const auto &p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        const Eigen::Vector3d q = T * Eigen::Vector3d(p.x, p.y, p.z);
        if (q.z() >= st.z_floor - 0.1 && q.z() <= st.z_ceiling + 0.1)
          pts.push_back(q);
      }
      int n = 0;
      for (const auto &w : model.walls) {
        if (w.storey != st.idx)
          continue;
        const auto raster = reusex::geometry::face_raster(
            pts, w, st.z_floor, st.z_ceiling, opt.openings);
        const auto rects = reusex::geometry::fit_openings(
            raster, st.z_floor, st.z_ceiling, opt.openings);
        if (!opt.raster_dir.empty())
          write_raster_png(opt.raster_dir / fmt::format("{}.png", w.id), raster,
                           rects, st.z_floor);
        for (const auto &r : rects) {
          reusex::core::OpeningRecord o;
          o.guid = fmt::format("o-{}-{}", w.id, n++);
          o.wall = w.guid;
          o.kind = "opening";
          o.s_start = r.s_start;
          o.s_end = r.s_end;
          o.z_sill = r.z_sill;
          o.z_head = r.z_head;
          o.confidence = r.fill;
          o.metadata = fmt::format(
              R"({{"fill":{:.3f},"jamb_start":{:.3f},"jamb_end":{:.3f}}})",
              r.fill, r.jamb_start, r.jamb_end);
          model.openings.push_back(o);
        }
      }
      spdlog::info("openings-dev: storey {}: {} points, {} opening(s)", st.idx,
                   pts.size(), n);
    }
    model.provided.openings = true;
    if (!opt.output.empty()) {
      reusex::io::solid_model_to_file(model, opt.output);
      spdlog::info("openings-dev: wrote {} ({} walls, {} openings)",
                   opt.output.string(), model.walls.size(),
                   model.openings.size());
    }
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("openings-dev failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
