// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Hidden dev driver for the wall sub-steps (reusex-z1v.5): read-only, so it
// can be pointed at a copy of a project between beads, before the walls
// stage and its tables exist (W3.5).

#include "create/walls_dev.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/normal_orientation.hpp>
#include <reusex/geometry/transform_utils.hpp>
#include <reusex/io/solid_model_json.hpp>

#include <Eigen/Geometry>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace {

/// Undirected angle between two lines [deg], 0..90.
double line_gap_deg(double a, double b) {
  double d = std::fmod(std::abs(a - b), 180.0);
  return std::min(d, 180.0 - d);
}

/// How far each reference wall of @p storey runs from its nearest family
/// (directions taken through world into the detected level frame).
void report_reference_angles(
    const reusex::core::SolidModel &ref, const reusex::core::StoreyRecord &st,
    const std::vector<reusex::geometry::AxisFamily> &families) {
  using reusex::geometry::to_affine;
  const reusex::core::StoreyRecord *rs = nullptr;
  for (const auto &s : ref.storeys)
    if (s.idx == st.idx)
      rs = &s;
  Eigen::Matrix3d ref_to_world = Eigen::Matrix3d::Identity();
  if (rs)
    ref_to_world = to_affine(rs->level_transform).linear().transpose();
  const Eigen::Matrix3d ref_to_det =
      to_affine(st.level_transform).linear() * ref_to_world;
  std::vector<double> dev;
  for (const auto &w : ref.walls) {
    if (w.storey != st.idx)
      continue;
    const Eigen::Vector3d dir =
        ref_to_det * Eigen::Vector3d(w.x1 - w.x0, w.y1 - w.y0, 0.0);
    // The wall's normal angle, the families' convention.
    const double a =
        std::atan2(dir.y(), dir.x()) * 180.0 / std::numbers::pi + 90.0;
    double best = 90.0;
    for (const auto &f : families)
      best = std::min(best, line_gap_deg(a, f.angle_deg));
    dev.push_back(best);
  }
  if (dev.empty())
    return;
  std::sort(dev.begin(), dev.end());
  const auto within = [&](double t) {
    return static_cast<double>(std::count_if(
               dev.begin(), dev.end(), [&](double d) { return d <= t; })) /
           static_cast<double>(dev.size());
  };
  spdlog::info("  reference walls vs families: n={} median {:.2f} deg, max "
               "{:.2f} deg, within 2 deg {:.0f} %, within 5 deg {:.0f} %",
               dev.size(), dev[dev.size() / 2], dev.back(), 100.0 * within(2.0),
               100.0 * within(5.0));
}

/// A face as a wall record on its own line (thickness unknown): what W3.2
/// alone can say.
reusex::core::WallRecord face_record(const reusex::geometry::WallFace &f,
                                     const reusex::geometry::AxisFamily &fam,
                                     const reusex::core::StoreyRecord &st,
                                     int n) {
  const Eigen::Vector2d t(-fam.d.y(), fam.d.x());
  const Eigen::Vector2d a = f.offset * fam.d + f.s0 * t;
  const Eigen::Vector2d b = f.offset * fam.d + f.s1 * t;
  reusex::core::WallRecord w;
  w.guid = fmt::format("face-{}-{}", st.idx, n);
  w.storey = st.idx;
  w.x0 = a.x();
  w.y0 = a.y();
  w.x1 = b.x();
  w.y1 = b.y();
  w.z_base = st.z_floor;
  w.height = st.z_ceiling - st.z_floor;
  w.axis_family = f.family;
  (f.side < 0 ? w.face_a_points : w.face_b_points) = f.point_count;
  return w;
}

} // namespace

void setup_subcommand_create_walls_dev(CLI::App &app,
                                       std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateWallsDevOptions>();
  auto &w = opt->walls;
  auto *sub = app.add_subcommand(
      "walls-dev",
      "Dev driver: run the wall sub-steps, write solid-model JSON");
  sub->group(""); // hidden: replaced by `rux create walls` (W3.5)

  sub->add_option("-o,--output", opt->output,
                  "Solid-model JSON with the storeys and detected walls");
  sub->add_option("--reference", opt->reference,
                  "Reference plan: report how its walls sit on the families");
  sub->add_option("--max-families", w.max_families)
      ->default_val(w.max_families);
  sub->add_option("--family-angle", w.family_angle_deg)
      ->default_val(w.family_angle_deg);
  sub->add_option("--min-family-share", w.min_family_share)
      ->default_val(w.min_family_share);
  sub->add_option("--family-refine", w.family_refine_deg)
      ->default_val(w.family_refine_deg);
  sub->add_option("--square", w.square_deg)->default_val(w.square_deg);
  sub->add_option("--face-window", w.face_window_m)
      ->default_val(w.face_window_m);
  sub->add_option("--min-peak-points", w.min_peak_points)
      ->default_val(w.min_peak_points);
  sub->add_option("--min-column-share", w.min_column_share)
      ->default_val(w.min_column_share);

  sub->callback([opt, global_opt]() {
    rux::finish(run_subcommand_create_walls_dev(*opt, *global_opt));
  });
}

int run_subcommand_create_walls_dev(SubcommandCreateWallsDevOptions const &opt,
                                    const RuxOptions &global_opt) {
  try {
    reusex::ProjectDB db(global_opt.project_db, /*readOnly=*/true);
    const auto storeys = db.storeys();
    if (!db.has_point_cloud("cloud") || !db.has_point_cloud("planes") ||
        !db.has_point_cloud("plane_normals")) {
      spdlog::error("walls-dev: needs the cloud, planes and plane_normals "
                    "clouds");
      return RuxError::INVALID_ARGUMENT;
    }
    const auto planes = db.plane_classes();
    if (storeys.empty() || planes.empty()) {
      spdlog::error("walls-dev: needs the storeys and plane_classes tables "
                    "(rux create storeys, rux create classify)");
      return RuxError::INVALID_ARGUMENT;
    }
    const auto plane_normals = db.point_cloud_normal("plane_normals");
    const auto cloud = db.point_cloud_xyzrgb("cloud");
    const auto plane_labels = db.point_cloud_label("planes");
    reusex::geometry::WallOptions wopt = opt.walls;
    const auto orientation = reusex::core::normal_orientation_of(
        db.point_cloud_provenance("plane_normals").second);
    wopt.use_normal_sign =
        orientation && *orientation == reusex::core::kNormalOrientationSensor;
    spdlog::info("walls-dev: plane normals {} the sensor",
                 wopt.use_normal_sign ? "face" : "do not (provably) face");

    reusex::core::SolidModel ref;
    if (!opt.reference.empty())
      ref = reusex::io::solid_model_from_file(opt.reference);

    reusex::core::SolidModel model;
    model.name = "walls-dev";
    model.storeys = storeys;
    model.provided.walls = true;
    for (const auto &st : storeys) {
      const auto families =
          reusex::geometry::axis_families(planes, *plane_normals, st, wopt);
      spdlog::info("storey {}: {} axis families", st.idx, families.size());
      for (const auto &f : families)
        spdlog::info("  family {}: {:.2f} deg, weight {:.0f}{}", f.index,
                     f.angle_deg, f.weight, f.dominant ? " (dominant)" : "");
      if (!opt.reference.empty())
        report_reference_angles(ref, st, families);

      const auto wp = reusex::geometry::wall_points(
          *cloud, *plane_labels, *plane_normals, planes, st, wopt);
      const auto faces = reusex::geometry::wall_faces(wp, families, wopt);
      spdlog::info("  {} wall points, {} other points, {} faces",
                   wp.points.size(), wp.others.size(), faces.size());
      for (size_t k = 0; k < faces.size(); ++k) {
        const auto &f = faces[k];
        const auto fam =
            std::find_if(families.begin(), families.end(),
                         [&](const auto &x) { return x.index == f.family; });
        model.walls.push_back(
            face_record(f, *fam, st, static_cast<int>(model.walls.size())));
      }
    }

    if (!opt.output.empty()) {
      reusex::io::solid_model_to_file(model, opt.output);
      spdlog::info("walls-dev: wrote {}", opt.output.string());
    }
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("walls-dev failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
