// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/qa_sheet.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"
#include "types/point_types.hpp"

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <tuple>

namespace reusex::visualize {

namespace {

constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;

/// A caption in a dark strip across the top of @p image.
void caption(cv::Mat &image, const std::string &text) {
  const double s = std::clamp(image.cols / 960.0, 0.5, 2.0);
  const int h = static_cast<int>(30 * s);
  cv::Mat strip = image(cv::Rect(0, 0, image.cols, std::min(h, image.rows)));
  strip *= 0.3;
  cv::putText(image, text, {static_cast<int>(8 * s), static_cast<int>(21 * s)},
              kFont, 0.6 * s, cv::Scalar(240, 240, 240),
              std::max(1, static_cast<int>(std::lround(s))), cv::LINE_AA);
}

std::string pct(double share) { return fmt::format("{:.0f}%", 100.0 * share); }

std::string cm(double m) { return fmt::format("{:.1f} cm", 100.0 * m); }

std::string summary_line(const char *what, const DeviationStats &d,
                         const DeviationOptions &dv) {
  return fmt::format("{}: mean {}, P95 {}, within {:g} cm {}, uncovered "
                     "(>{:g} cm) {}",
                     what, cm(d.mean_m), cm(d.p95_m), 100.0 * dv.within_m,
                     pct(d.within_share), 100.0 * dv.uncovered_m,
                     pct(d.uncovered_share));
}

} // namespace

std::vector<SectionSpec>
default_qa_sections(std::vector<double> &u, std::vector<double> &v,
                    const std::array<double, 3> &box_min,
                    const std::array<double, 3> &box_max, double yaw_deg,
                    double thickness) {
  const double yaw = yaw_deg * std::numbers::pi / 180.0;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const auto world = [&](double a, double b) {
    const auto cm = [](double m) { return std::round(m * 100.0) / 100.0; };
    return std::array<double, 2>{cm(c * a + s * b), cm(-s * a + c * b)};
  };
  const auto quantile = [](std::vector<double> &x, double f, double fallback) {
    if (x.empty())
      return fallback;
    const auto i = static_cast<std::size_t>(
        std::floor(f * static_cast<double>(x.size() - 1)));
    std::nth_element(x.begin(), x.begin() + static_cast<long>(i), x.end());
    return x[i];
  };
  const double pad = thickness;
  std::vector<SectionSpec> out;
  const auto seg = [&](std::array<double, 2> a, std::array<double, 2> b) {
    SectionSpec spec;
    spec.kind = SectionSpec::Kind::segment;
    spec.p0 = a;
    spec.p1 = b;
    spec.thickness = thickness;
    out.push_back(spec);
  };
  const double um = quantile(u, 0.5, 0.5 * (box_min[0] + box_max[0]));
  seg(world(um, box_min[1] - pad), world(um, box_max[1] + pad));
  for (const double f : {1.0 / 3.0, 2.0 / 3.0}) {
    const double vq =
        quantile(v, f, box_min[1] + f * (box_max[1] - box_min[1]));
    seg(world(box_min[0] - pad, vq), world(box_max[0] + pad, vq));
  }
  return out;
}

QaSheet render_qa_sheet(const ProjectDB &db, const QaSheetOptions &opts) {
  if (opts.panel_width <= 0 || opts.panel_height <= 0 || opts.columns <= 0)
    throw std::runtime_error(
        fmt::format("render: QA sheet needs a positive panel size and column "
                    "count (got {}x{}, {} columns)",
                    opts.panel_width, opts.panel_height, opts.columns));
  const stopwatch timer;
  QaSheet out;
  const std::string &mesh = opts.base.mesh_name;

  RenderOptions base = opts.base;
  base.width = opts.panel_width;
  base.height = opts.panel_height;
  base.cut = false;
  base.section.reset();
  base.camera.reset();
  base.legend = true;
  base.frame_quantile = opts.frame_quantile;

  const auto panel = [&](const std::string &name, const std::string &title,
                         RenderOptions o, RenderInfo *info = nullptr) {
    core::info("render: QA panel '{}'", name);
    QaPanel p{name, title, render_view(db, o, info)};
    caption(p.image, title);
    out.panels.push_back(std::move(p));
  };

  // 1. Plan cut: cloud + translucent mesh + the mesh's section line.
  RenderInfo plan_info;
  {
    RenderOptions o = base;
    o.layers = {Layer::cloud, Layer::mesh};
    o.mesh_style = MeshStyle::overlay;
    o.view = ViewPreset::plan;
    const std::string cut =
        o.cut_height ? fmt::format("{:.2f} m", *o.cut_height) : "default";
    panel("plan",
          fmt::format("Plan cut ({}) - cloud + mesh '{}' (section line)", cut,
                      mesh),
          o, &plan_info);
  }

  // 2. The four elevations.
  for (const auto &[view, name, title] :
       {std::tuple{ViewPreset::front, "elev_front",
                   "front (from the plan's bottom edge)"},
        std::tuple{ViewPreset::back, "elev_back",
                   "back (from the plan's top edge)"},
        std::tuple{ViewPreset::left, "elev_left",
                   "left (from the plan's left edge)"},
        std::tuple{ViewPreset::right, "elev_right",
                   "right (from the plan's right edge)"}}) {
    RenderOptions o = base;
    o.layers = {Layer::cloud, Layer::mesh};
    o.mesh_style = MeshStyle::overlay;
    o.view = view;
    panel(name, fmt::format("Elevation {} - cloud + mesh", title), o);
  }

  // 3. Vertical sections.
  std::vector<SectionSpec> sections = opts.sections;
  if (sections.empty()) {
    const CloudPtr cloud = db.point_cloud_xyzrgb(opts.base.cloud_name);
    const double yaw = plan_info.yaw_deg * std::numbers::pi / 180.0;
    const double c = std::cos(yaw), sn = std::sin(yaw);
    std::vector<double> u, v;
    if (cloud)
      for (const auto &p : *cloud)
        if (std::isfinite(p.x) && std::isfinite(p.y)) {
          u.push_back(c * p.x - sn * p.y);
          v.push_back(sn * p.x + c * p.y);
        }
    sections =
        default_qa_sections(u, v, plan_info.display_min, plan_info.display_max,
                            plan_info.yaw_deg, SectionSpec{}.thickness);
  }
  for (std::size_t i = 0; i < sections.size(); ++i) {
    RenderOptions o = base;
    o.layers = {Layer::cloud, Layer::mesh};
    o.mesh_style = MeshStyle::overlay;
    o.view = ViewPreset::section;
    o.section = sections[i];
    panel(fmt::format("section_{}", i + 1),
          fmt::format("Section {} {} (slab {:g} cm) - mesh section line", i + 1,
                      to_string(sections[i]), 100.0 * sections[i].thickness),
          o);
  }

  // 4. Deviation plan.
  {
    RenderOptions o = base;
    o.layers = {Layer::deviation, Layer::mesh};
    o.mesh_style = MeshStyle::edges;
    o.view = ViewPreset::plan;
    RenderInfo info;
    panel("deviation",
          fmt::format("Deviation plan - cloud distance to mesh '{}'", mesh), o,
          &info);
    out.deviation = info.deviation;
    out.deviation_planar = info.deviation_planar;
  }

  // ── Compose ──────────────────────────────────────────────────────────────
  const int cols =
      std::min<int>(opts.columns, static_cast<int>(out.panels.size()));
  const int rows =
      (static_cast<int>(out.panels.size()) + cols - 1) / std::max(cols, 1);
  const double s = std::clamp(opts.panel_width / 960.0, 0.5, 2.0);
  const int header = static_cast<int>(70 * s);
  const int gap = 4;
  out.sheet = cv::Mat(header + rows * (opts.panel_height + gap),
                      cols * (opts.panel_width + gap) - gap, CV_8UC3,
                      cv::Scalar(20, 18, 16));
  for (std::size_t i = 0; i < out.panels.size(); ++i) {
    const int r = static_cast<int>(i) / cols;
    const int c = static_cast<int>(i) % cols;
    out.panels[i].image.copyTo(out.sheet(cv::Rect(
        c * (opts.panel_width + gap), header + r * (opts.panel_height + gap),
        opts.panel_width, opts.panel_height)));
  }
  const cv::Scalar white(240, 240, 240);
  const int th = std::max(1, static_cast<int>(std::lround(s)));
  cv::putText(out.sheet,
              fmt::format("Mesh QA - {} - mesh '{}' vs cloud '{}' - view yaw "
                          "{:g} deg",
                          db.path().filename().string(), mesh,
                          opts.base.cloud_name, plan_info.yaw_deg),
              {static_cast<int>(10 * s), static_cast<int>(28 * s)}, kFont,
              0.8 * s, white, th, cv::LINE_AA);
  if (out.deviation) {
    std::string line =
        summary_line("all points", *out.deviation, opts.base.deviation);
    if (out.deviation_planar)
      line += "   |   " + summary_line("planar points", *out.deviation_planar,
                                       opts.base.deviation);
    cv::putText(out.sheet, line,
                {static_cast<int>(10 * s), static_cast<int>(58 * s)}, kFont,
                0.55 * s, white, th, cv::LINE_AA);
  }
  core::info("render: QA sheet of {} panels ({}x{}) in {:.1f}s",
             out.panels.size(), out.sheet.cols, out.sheet.rows,
             timer.elapsed());
  return out;
}

} // namespace reusex::visualize
