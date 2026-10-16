// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/render_view.hpp"

#include "core/ProjectDB.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/logging.hpp"
#include "core/pose_graph.hpp"
#include "geometry/transform_utils.hpp"
#include "utils/parse_number.hpp"
#include "visualize/offscreen_gl.hpp"
#include "visualize/scene.hpp"

#include <fmt/format.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

// The scene itself — loading the layers, building the actors, the cut plane
// and the preset cameras — lives in visualize/scene.cpp (populate_scene), so
// the Qt client's interactive 3D view draws exactly what this renders. What
// stays here is the headless part: option validation, the offscreen render
// window, the GL probe and the read-back.

namespace reusex::visualize {

namespace {
// ── Offscreen OpenGL availability (#313) ─────────────────────────────────────

/// Refuse to render, with a diagnosis, when there is no offscreen GL here.
///
/// Called after every option and data check and immediately before the first
/// Render(): VTK's EGL render window crashes rather than failing when it has
/// no device, so this is the last point at which the condition can still be
/// reported. Placing it here (rather than at the top of render_view()) also
/// keeps the cheaper, more common diagnostics — a missing cloud, a stale label
/// layer — as the message a caller sees, on a GPU-less machine as much as
/// anywhere else.
void require_offscreen_gl() {
  // One probe per process. The answer cannot change while it runs, and an
  // orbit sweep would otherwise pay for it once per frame.
  static const OffscreenGlProbe probe = probe_offscreen_gl();

  if (probe.status == OffscreenGlStatus::usable) {
    core::debug("render: offscreen GL available — {}", probe.detail);
    return;
  }
  if (probe.status == OffscreenGlStatus::unknown) {
    // Not a verdict. VTK may be using OSMesa or a windowed GL path that never
    // goes through EGL; SupportsOpenGL() after the render still guards those.
    core::debug("render: offscreen GL probe inconclusive — {}", probe.detail);
    return;
  }

  if (display_configured()) {
    // VTK tries the windowed path first here and only falls back to EGL if it
    // cannot reach the display. An EGL that will not initialise says nothing
    // about GLX on a working X server, so this must not be fatal.
    core::warn("render: no usable EGL device ({}), but DISPLAY/WAYLAND_DISPLAY "
               "is set — continuing on VTK's windowed GL path",
               probe.detail);
    return;
  }

  throw OffscreenGlUnavailable(
      "render: this machine cannot create an offscreen OpenGL context, so "
      "there is nothing to render into — " +
      probe.detail +
      ". Headless rendering needs a GPU reachable through EGL (a DRM render "
      "node under /dev/dri, or a vendor driver), a software rasteriser (Mesa's "
      "llvmpipe EGL driver), or an X/Wayland display to fall back on. Set "
      "REUSEX_SKIP_EGL_PROBE=1 to bypass this check and let VTK try anyway.");
}

// ── Image readback ───────────────────────────────────────────────────────────

/// Copy the rendered frame into a BGR cv::Mat, flipping VTK's bottom-up rows.
cv::Mat to_bgr_image(vtkImageData *image, int width, int height) {
  int dims[3] = {0, 0, 0};
  image->GetDimensions(dims);
  if (dims[0] <= 0 || dims[1] <= 0) {
    throw std::runtime_error(
        "render: the offscreen framebuffer came back empty (" +
        std::to_string(dims[0]) + "x" + std::to_string(dims[1]) + ")");
  }

  cv::Mat out(dims[1], dims[0], CV_8UC3);
  const auto *src =
      static_cast<const unsigned char *>(image->GetScalarPointer());
  const std::size_t stride = static_cast<std::size_t>(dims[0]) * 3;
  for (int y = 0; y < dims[1]; ++y) {
    const unsigned char *row = src + static_cast<std::size_t>(y) * stride;
    auto *dst =
        out.ptr<cv::Vec3b>(dims[1] - 1 - y); // VTK origin is bottom-left
    for (int x = 0; x < dims[0]; ++x) {
      dst[x] = cv::Vec3b(row[3 * x + 2], row[3 * x + 1], row[3 * x + 0]);
    }
  }

  if (dims[0] != width || dims[1] != height) {
    // render_view() documents an exact output size; silently handing back a
    // different one would corrupt anything that lays images out (STANDARDS §5).
    throw std::runtime_error(
        "render: the offscreen framebuffer came back " +
        std::to_string(dims[0]) + "x" + std::to_string(dims[1]) + " but " +
        std::to_string(width) + "x" + std::to_string(height) +
        " was requested — the GPU may be clamping the offscreen buffer size");
  }
  return out;
}

// ── Deviation legend ─────────────────────────────────────────────────────────

/// "12.3 cm" / "1.20 m".
std::string length_text(double m) {
  return m < 1.0 ? fmt::format("{:.1f} cm", 100.0 * m)
                 : fmt::format("{:.2f} m", m);
}

/// Draw the deviation colour ramp and its summary into the lower-left corner
/// of @p image (BGR). Hershey text: ASCII only, deterministic.
void draw_deviation_legend(cv::Mat &image, const DeviationStats &all,
                           const std::optional<DeviationStats> &planar,
                           const DeviationOptions &dv,
                           const std::string &mesh_name) {
  const double s = std::clamp(image.cols / 1600.0, 0.5, 2.0);
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const double fs = 0.55 * s;
  const int th = std::max(1, static_cast<int>(std::lround(s)));
  const int pad = static_cast<int>(10 * s);
  const int line = static_cast<int>(22 * s);
  const int bar_w = static_cast<int>(360 * s);
  const int bar_h = static_cast<int>(14 * s);

  std::vector<std::string> lines;
  const auto row = [&](const char *what, const DeviationStats &d) {
    return fmt::format("{}: n={}  mean {}  P95 {}  <={}: {:.0f}%  >{}: {:.0f}%",
                       what, d.points, length_text(d.mean_m),
                       length_text(d.p95_m), length_text(dv.within_m),
                       100.0 * d.within_share, length_text(dv.uncovered_m),
                       100.0 * d.uncovered_share);
  };
  lines.push_back("cloud -> mesh '" + mesh_name + "' distance");
  lines.push_back(row("all points", all));
  if (planar)
    lines.push_back(row("planar pts", *planar));

  int text_w = bar_w;
  for (const auto &l : lines) {
    int base = 0;
    text_w = std::max(text_w, cv::getTextSize(l, font, fs, th, &base).width);
  }
  const int box_w = text_w + 2 * pad;
  const int box_h =
      pad + bar_h + line + static_cast<int>(lines.size()) * line + pad;
  const int x0 = pad;
  const int y0 = image.rows - box_h - pad;
  if (y0 < 0 || box_w + x0 > image.cols)
    return; // too small an image for a legend
  cv::Mat roi = image(cv::Rect(x0, y0, box_w, box_h));
  roi *= 0.25; // darken behind the text
  const cv::Scalar white(240, 240, 240);

  // The ramp: 0 .. uncovered over 85 % of the bar, "beyond" after.
  const int bx = x0 + pad, by = y0 + pad;
  const int ramp_w = static_cast<int>(0.85 * bar_w);
  for (int i = 0; i < bar_w; ++i) {
    const double d = i < ramp_w ? dv.uncovered_m * i / std::max(ramp_w - 1, 1)
                                : 2.0 * dv.uncovered_m;
    const auto c = deviation_color(d, dv);
    cv::line(image, {bx + i, by}, {bx + i, by + bar_h},
             cv::Scalar(c[2], c[1], c[0]), 1);
  }
  const auto tick = [&](double d, const std::string &label, int px) {
    (void)d;
    cv::line(image, {px, by + bar_h}, {px, by + bar_h + line / 4}, white, th);
    cv::putText(image, label, {px - static_cast<int>(8 * s), by + bar_h + line},
                font, fs * 0.9, white, th, cv::LINE_AA);
  };
  tick(0.0, "0", bx);
  tick(dv.within_m, length_text(dv.within_m),
       bx + static_cast<int>(ramp_w * dv.within_m / dv.uncovered_m));
  tick(dv.uncovered_m, length_text(dv.uncovered_m), bx + ramp_w);
  cv::putText(image, "uncovered",
              {bx + ramp_w + static_cast<int>(6 * s),
               by + bar_h - static_cast<int>(2 * s)},
              font, fs * 0.8, white, th, cv::LINE_AA);
  int y = by + bar_h + 2 * line;
  for (const auto &l : lines) {
    cv::putText(image, l, {bx, y}, font, fs, white, th, cv::LINE_AA);
    y += line;
  }
}

void validate(const RenderOptions &opts) {
  if (opts.layers.empty())
    throw std::runtime_error(
        "render: no layers selected — pass at least one of "
        "cloud, labels, planes, rooms, instances, mesh, "
        "components, frustums, panoramas, deviation");
  if (opts.width <= 0 || opts.height <= 0)
    throw std::runtime_error("render: image size must be positive (got " +
                             std::to_string(opts.width) + "x" +
                             std::to_string(opts.height) + ")");
  if (opts.view == ViewPreset::orbit) {
    if (opts.orbit_count <= 0)
      throw std::runtime_error("render: orbit count must be >= 1 (got " +
                               std::to_string(opts.orbit_count) + ")");
    if (opts.orbit_index < 0 || opts.orbit_index >= opts.orbit_count)
      throw std::runtime_error(
          "render: orbit index " + std::to_string(opts.orbit_index) +
          " is out of range for a ring of " + std::to_string(opts.orbit_count));
  }
  if (opts.view == ViewPreset::section && !opts.section)
    throw std::runtime_error(
        "render: view is 'section' but no section was supplied");
  if (opts.view == ViewPreset::explicit_camera && !opts.camera)
    throw std::runtime_error(
        "render: view is 'explicit_camera' but no camera was supplied");
  if (opts.margin <= 0.0)
    throw std::runtime_error("render: margin must be positive (got " +
                             std::to_string(opts.margin) + ")");
  if (opts.point_size <= 0.0)
    throw std::runtime_error("render: point size must be positive (got " +
                             std::to_string(opts.point_size) + ")");
  if (opts.cut_height && *opts.cut_height <= 0.0)
    throw std::runtime_error(
        "render: cut height must be positive — it is measured upward from the "
        "floor (got " +
        std::to_string(*opts.cut_height) + ")");
  // At +/-90 degrees the orbit view direction is parallel to the (0,0,1) up
  // vector and the camera basis collapses.
  if (opts.view == ViewPreset::orbit &&
      std::abs(opts.orbit_elevation_deg) > 89.0)
    throw std::runtime_error(
        "render: orbit elevation must be within +/-89 degrees (got " +
        std::to_string(opts.orbit_elevation_deg) + ")");
}

} // namespace

std::optional<Layer> layer_from_string(std::string_view name) {
  for (const Layer layer : all_layers()) {
    if (to_string(layer) == name)
      return layer;
  }
  return std::nullopt;
}

std::string_view to_string(Layer layer) {
  switch (layer) {
  case Layer::cloud:
    return "cloud";
  case Layer::labels:
    return "labels";
  case Layer::planes:
    return "planes";
  case Layer::rooms:
    return "rooms";
  case Layer::instances:
    return "instances";
  case Layer::mesh:
    return "mesh";
  case Layer::components:
    return "components";
  case Layer::frustums:
    return "frustums";
  case Layer::panoramas:
    return "panoramas";
  case Layer::deviation:
    return "deviation";
  case Layer::walls:
    return "walls";
  case Layer::openings:
    return "openings";
  }
  return "unknown";
}

std::string_view to_string(ViewPreset view) {
  switch (view) {
  case ViewPreset::top:
    return "top";
  case ViewPreset::plan:
    return "plan";
  case ViewPreset::front:
    return "front";
  case ViewPreset::back:
    return "back";
  case ViewPreset::left:
    return "left";
  case ViewPreset::right:
    return "right";
  case ViewPreset::section:
    return "section";
  case ViewPreset::orbit:
    return "orbit";
  case ViewPreset::explicit_camera:
    return "frame";
  }
  return "unknown";
}

std::optional<ViewPreset> view_preset_from_string(std::string_view name) {
  for (const ViewPreset view :
       {ViewPreset::top, ViewPreset::plan, ViewPreset::front, ViewPreset::back,
        ViewPreset::left, ViewPreset::right, ViewPreset::section,
        ViewPreset::orbit, ViewPreset::explicit_camera}) {
    if (to_string(view) == name)
      return view;
  }
  return std::nullopt;
}

const std::vector<Layer> &all_layers() {
  static const std::vector<Layer> layers{
      Layer::cloud,     Layer::labels,    Layer::planes,     Layer::rooms,
      Layer::instances, Layer::mesh,      Layer::components, Layer::frustums,
      Layer::panoramas, Layer::deviation, Layer::walls,      Layer::openings};
  return layers;
}

std::string_view to_string(MeshStyle style) {
  switch (style) {
  case MeshStyle::automatic:
    return "auto";
  case MeshStyle::solid:
    return "solid";
  case MeshStyle::overlay:
    return "overlay";
  case MeshStyle::edges:
    return "edges";
  }
  return "unknown";
}

std::optional<MeshStyle> mesh_style_from_string(std::string_view name) {
  for (const MeshStyle s : {MeshStyle::automatic, MeshStyle::solid,
                            MeshStyle::overlay, MeshStyle::edges})
    if (to_string(s) == name)
      return s;
  return std::nullopt;
}

SectionSpec parse_section(std::string_view text) {
  const auto number = [&](std::string_view sv) {
    std::size_t used = 0;
    const auto v = utils::parse_number<double>(sv, &used);
    if (!v || used != sv.size() || !std::isfinite(*v))
      throw std::invalid_argument("section: '" + std::string(sv) +
                                  "' is not a number in '" + std::string(text) +
                                  "'");
    return *v;
  };
  SectionSpec spec;
  if (text.size() > 2 && (text[0] == 'x' || text[0] == 'y') && text[1] == '=') {
    spec.kind = text[0] == 'x' ? SectionSpec::Kind::x : SectionSpec::Kind::y;
    spec.at = number(text.substr(2));
    return spec;
  }
  const auto colon = text.find(':');
  const auto pair = [&](std::string_view sv) -> std::array<double, 2> {
    const auto comma = sv.find(',');
    if (comma == std::string_view::npos)
      throw std::invalid_argument("section: expected <x>,<y> (got '" +
                                  std::string(sv) + "')");
    return {number(sv.substr(0, comma)), number(sv.substr(comma + 1))};
  };
  if (colon == std::string_view::npos)
    throw std::invalid_argument(
        "section: expected x=<m>, y=<m> or <x0>,<y0>:<x1>,<y1> (got '" +
        std::string(text) + "')");
  spec.kind = SectionSpec::Kind::segment;
  spec.p0 = pair(text.substr(0, colon));
  spec.p1 = pair(text.substr(colon + 1));
  if (spec.p0 == spec.p1)
    throw std::invalid_argument("section: the segment '" + std::string(text) +
                                "' has zero length");
  return spec;
}

std::string to_string(const SectionSpec &section) {
  switch (section.kind) {
  case SectionSpec::Kind::x:
    return fmt::format("x={:g}", section.at);
  case SectionSpec::Kind::y:
    return fmt::format("y={:g}", section.at);
  case SectionSpec::Kind::segment:
    break;
  }
  return fmt::format("{:g},{:g}:{:g},{:g}", section.p0[0], section.p0[1],
                     section.p1[0], section.p1[1]);
}

cv::Mat render_view(const ProjectDB &db, const RenderOptions &opts,
                    RenderInfo *out_info) {
  validate(opts);
  if (!db.is_open())
    throw std::runtime_error("render: project database is not open");

  install_vtk_log_bridge();
  const stopwatch timer;

  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(opts.background[0], opts.background[1],
                          opts.background[2]);

  double yaw = 0.0;
  if (opts.yaw_deg) {
    yaw = *opts.yaw_deg;
  } else if (const auto text =
                 db.setting(core::pose_graph::kSettingViewYawDeg)) {
    yaw = utils::parse_number<double>(*text).value_or(0.0);
  }

  SceneOptions scene;
  scene.layers = opts.layers;
  if (opts.view != ViewPreset::explicit_camera)
    scene.frame_yaw_deg = yaw;
  scene.cloud_name = opts.cloud_name;
  scene.mesh_name = opts.mesh_name;
  scene.point_size = opts.point_size;
  scene.highlight = opts.highlight;
  scene.plan = opts.plan;
  scene.plan_labels = opts.plan_labels;
  scene.mesh_style = opts.mesh_style;
  scene.deviation = opts.deviation;
  scene.frame_quantile = opts.frame_quantile;
  scene.plan_placement =
      opts.view == ViewPreset::top || opts.view == ViewPreset::plan
          ? PlanPlacement::above_scene
          : PlanPlacement::level;
  const SceneInfo info = populate_scene(renderer, db, scene);
  const SceneBounds &bounds = info.bounds;
  const SceneBounds &frame = info.frame_bounds;

  // A plan view is a top view plus the cut; an explicit `cut` cuts any view.
  if (opts.cut || opts.view == ViewPreset::plan) {
    const CutPlane cut = resolve_cut_plane(db, bounds, opts.cut_height);
    apply_cut_plane(renderer, cut.z);
    core::info("render: cut plane at z={:.3f} m — {:.2f} m above the {} floor "
               "at z={:.3f}",
               cut.z, cut.height, cut.from_planes ? "detected" : "bounding-box",
               cut.floor_z);
    if (opts.mesh_section_line)
      add_mesh_section_line(renderer, info, {0.0, 0.0, cut.z}, {0.0, 0.0, 1.0},
                            {0.0, 0.0, 0.0}, scene.mesh_edge_rgb);
  }

  // A vertical section: clip to the slab, and draw the mesh's cut through
  // the slab's centre plane in front of the slab's points.
  std::optional<ResolvedSection> section;
  if (opts.view == ViewPreset::section) {
    try {
      section = resolve_section(*opts.section, frame);
    } catch (const std::invalid_argument &e) {
      throw std::runtime_error(e.what());
    }
    apply_section(renderer, *section);
    if (opts.mesh_section_line) {
      const double toward = 0.5 * section->thickness + 0.01;
      add_mesh_section_line(
          renderer, info, {section->p0[0], section->p0[1], 0.0},
          {section->view[0], section->view[1], 0.0},
          {-section->view[0] * toward, -section->view[1] * toward, 0.0},
          scene.mesh_edge_rgb);
    }
    core::info("render: section {} ({:.3f},{:.3f})->({:.3f},{:.3f}), slab "
               "{:.2f} m",
               to_string(*opts.section), section->p0[0], section->p0[1],
               section->p1[0], section->p1[1], section->thickness);
  }

  vtkNew<vtkRenderWindow> window;
  // The whole point of this function: never touch a window manager. With the
  // flake's EGL-enabled VTK this succeeds with no DISPLAY set (#294).
  window->SetOffScreenRendering(1);
  window->SetMultiSamples(0); // removes MSAA sample-order variance
  window->AddRenderer(renderer);
  window->SetSize(opts.width, opts.height);

  if (opts.view == ViewPreset::explicit_camera) {
    place_explicit_camera(renderer, *opts.camera, opts.width, opts.height,
                          frame);
  } else {
    CameraFraming framing;
    framing.view = opts.view;
    framing.orbit_count = opts.orbit_count;
    framing.orbit_index = opts.orbit_index;
    framing.orbit_elevation_deg = opts.orbit_elevation_deg;
    framing.margin = opts.margin;
    framing.aspect =
        static_cast<double>(opts.width) / static_cast<double>(opts.height);
    framing.yaw_deg = yaw;
    if (info.display_bounds && info.display_yaw_deg == yaw)
      framing.display_bounds = info.display_bounds;
    framing.section = section;
    place_preset_camera(renderer, framing, frame);
  }

  // Everything above this line is checked without touching the GPU, so a
  // GPU-less machine still gets the more useful diagnosis when the request
  // itself is wrong. From here on VTK owns the process: its EGL render window
  // crashes rather than fails when it has no device, so the question has to be
  // asked before the first Render() (#313).
  require_offscreen_gl();

  window->Render();
  core::debug("render: window class {}", window->GetClassName());

  // A render window with no usable OpenGL implementation does not fail — it
  // hands back a correctly sized frame of pure background. Nothing downstream
  // can tell that apart from a legitimately empty scene, so check it here
  // rather than let a black PNG be reported as success (STANDARDS §5). The
  // probe above cannot replace this: it answers for EGL, and VTK may have
  // taken a windowed or OSMesa path instead.
  if (window->SupportsOpenGL() == 0) {
    throw OffscreenGlUnavailable(
        "render: the render window has no usable OpenGL implementation "
        "(window class " +
        std::string(window->GetClassName()) +
        "); with no display this needs an EGL-capable VTK and a rendering "
        "device");
  }

  vtkNew<vtkWindowToImageFilter> capture;
  capture->SetInput(window);
  capture->SetInputBufferTypeToRGB();
  capture->ReadFrontBufferOff();
  // Update() re-renders by default; the frame is already drawn, and an orbit
  // sweep would otherwise pay for 2 renders per view.
  capture->ShouldRerenderOff();
  capture->Update();

  cv::Mat image = to_bgr_image(capture->GetOutput(), opts.width, opts.height);

  // Loud, but not fatal: a uniform frame with geometry in the scene means
  // nothing reached the framebuffer. It is legitimate only for an explicit
  // camera the caller aimed away from the geometry.
  double lo = 0.0;
  double hi = 0.0;
  cv::minMaxLoc(image.reshape(1), &lo, &hi);
  if (lo == hi && (info.drawn_points + info.drawn_faces) > 0) {
    core::warn("render: the frame is a uniform colour despite {} points and {} "
               "faces in the scene — the camera may be pointed away from the "
               "geometry, or the GL context produced nothing",
               info.drawn_points, info.drawn_faces);
  }

  if (info.deviation && opts.legend)
    draw_deviation_legend(image, *info.deviation, info.deviation_planar,
                          opts.deviation, opts.mesh_name);
  if (out_info != nullptr) {
    out_info->drawn_points = info.drawn_points;
    out_info->drawn_faces = info.drawn_faces;
    out_info->bounds_min = frame.min;
    out_info->bounds_max = frame.max;
    out_info->yaw_deg = yaw;
    out_info->display_min =
        info.display_bounds ? info.display_bounds->min : frame.min;
    out_info->display_max =
        info.display_bounds ? info.display_bounds->max : frame.max;
    out_info->deviation = info.deviation;
    out_info->deviation_planar = info.deviation_planar;
  }

  core::info("render: {} points, {} faces -> {}x{} in {:.2f}s",
             info.drawn_points, info.drawn_faces, image.cols, image.rows,
             timer.elapsed());

  // vtkNew would finalize on destruction anyway; doing it here releases the
  // EGL surface before the (potentially large) image is copied out.
  window->Finalize();

  return image;
}

CameraSpec camera_from_sensor_frame(const ProjectDB &db, int node_id, int width,
                                    int height) {
  if (width <= 0 || height <= 0)
    throw std::runtime_error(
        "render: camera_from_sensor_frame needs a positive output size (got " +
        std::to_string(width) + "x" + std::to_string(height) + ")");

  const core::SensorIntrinsics intr = db.sensor_frame_intrinsics(node_id);
  if (intr.fx <= 0.0 || intr.fy <= 0.0 || intr.width <= 0 || intr.height <= 0) {
    throw std::runtime_error(
        "render: sensor frame " + std::to_string(node_id) +
        " has no usable intrinsics (fx=" + std::to_string(intr.fx) +
        ", fy=" + std::to_string(intr.fy) + ", " + std::to_string(intr.width) +
        "x" + std::to_string(intr.height) + ")");
  }

  // Unlike the pipeline stages, there is nothing to skip here: this function
  // returns exactly one camera. Falling through to `sensor_frame_pose()`'s
  // identity fallback would render the scene from the world origin and hand
  // back a PNG that looks like a real answer (#336), so refuse the same way
  // the unusable-intrinsics check above does.
  if (!db.has_sensor_frame_pose(node_id)) {
    throw std::runtime_error(
        "render: sensor frame " + std::to_string(node_id) +
        " has no usable stored pose (missing, non-finite, or degenerate "
        "transform) — pick another frame, or run 'rux optimize' to give the "
        "scan poses");
  }

  // The stored pose is body-to-world; the camera sits at pose * local_transform
  // — the same composition segmentation/reconstruct.cpp uses to back-project
  // depth, which is what makes a render line up with the captured frame.
  const Eigen::Affine3f c2w =
      (geometry::to_affine(db.sensor_frame_pose(node_id)) *
       geometry::to_affine(intr.local_transform))
          .cast<float>();

  // Intrinsics describe the captured frame; rescale them to the output size.
  const double sx = static_cast<double>(width) / intr.width;
  const double sy = static_cast<double>(height) / intr.height;

  CameraSpec spec;
  spec.pose = geometry::to_array16(c2w);
  spec.fx = intr.fx * sx;
  spec.fy = intr.fy * sy;
  spec.cx = intr.cx * sx;
  spec.cy = intr.cy * sy;
  return spec;
}

} // namespace reusex::visualize
