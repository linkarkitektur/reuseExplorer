// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "render.hpp"
#include "exit_status.hpp"
#include "render_watch.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/pose_edits.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/slam/drift_diagnostics.hpp>
#include <reusex/visualize/qa_sheet.hpp>
#include <reusex/visualize/render_view.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <charconv>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace viz = reusex::visualize;

namespace {

/// Split a comma-separated list, trimming surrounding whitespace.
std::vector<std::string> split_csv(const std::string &text) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::size_t end = comma == std::string::npos ? text.size() : comma;
    std::string part = text.substr(start, end - start);
    const auto first = part.find_first_not_of(" \t");
    const auto last = part.find_last_not_of(" \t");
    if (first != std::string::npos)
      parts.push_back(part.substr(first, last - first + 1));
    if (comma == std::string::npos)
      break;
    start = comma + 1;
  }
  return parts;
}

/// The layer vocabulary, for help text and error messages.
std::string layer_vocabulary() {
  std::vector<std::string> names;
  for (const auto layer : viz::all_layers())
    names.emplace_back(viz::to_string(layer));
  return fmt::format("{}", fmt::join(names, ", "));
}

std::string default_layer_list(const viz::RenderOptions &defaults) {
  std::vector<std::string> names;
  for (const auto layer : defaults.layers)
    names.emplace_back(viz::to_string(layer));
  return fmt::format("{}", fmt::join(names, ","));
}

/// Parse `--layers cloud,planes` into the library enum.
std::vector<viz::Layer> parse_layers(const std::string &text) {
  const auto names = split_csv(text);
  if (names.empty())
    throw std::runtime_error("--layers is empty; expected one or more of: " +
                             layer_vocabulary());

  std::vector<viz::Layer> layers;
  for (const auto &name : names) {
    const auto layer = viz::layer_from_string(name);
    if (!layer)
      throw std::runtime_error("unknown layer '" + name +
                               "'; expected one of: " + layer_vocabulary());
    layers.push_back(*layer);
  }
  return layers;
}

/// Parse `--size 1600x1200`.
void parse_size(const std::string &text, int &width, int &height) {
  const std::size_t x = text.find_first_of("xX");
  if (x == std::string::npos)
    throw std::runtime_error("--size must look like WxH (got '" + text + "')");

  const auto to_int = [&text](std::string_view sv) {
    int value = 0;
    const auto *end = sv.data() + sv.size();
    const auto result = std::from_chars(sv.data(), end, value);
    if (result.ec != std::errc{} || result.ptr != end || value <= 0)
      throw std::runtime_error(
          "--size must be two positive integers WxH (got '" + text + "')");
    return value;
  };

  width = to_int(std::string_view(text).substr(0, x));
  height = to_int(std::string_view(text).substr(x + 1));
}

/// Parse a `name:value` suffix, e.g. the N in `orbit:N`.
std::optional<int> parse_suffix(const std::string &text,
                                std::string_view name) {
  if (text.size() <= name.size() + 1 || !text.starts_with(name) ||
      text[name.size()] != ':')
    return std::nullopt;
  int value = 0;
  const std::string_view digits =
      std::string_view(text).substr(name.size() + 1);
  const auto *end = digits.data() + digits.size();
  const auto result = std::from_chars(digits.data(), end, value);
  if (result.ec != std::errc{} || result.ptr != end)
    throw std::runtime_error("expected an integer after '" + std::string(name) +
                             ":' (got '" + text + "')");
  return value;
}

/// Parse a `name:value` suffix whose value is a length in metres, e.g. the
/// 1.4 in `plan:1.4`. Separate from parse_suffix() because a cut height is a
/// measurement, not a count, and `plan:1` must not silently mean 1 m when the
/// user meant the first of something.
std::optional<double> parse_metre_suffix(const std::string &text,
                                         std::string_view name) {
  if (text.size() <= name.size() + 1 || !text.starts_with(name) ||
      text[name.size()] != ':')
    return std::nullopt;

  const std::string digits = text.substr(name.size() + 1);
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(digits, &consumed);
  } catch (const std::exception &) {
    consumed = 0;
  }
  if (consumed != digits.size())
    throw std::runtime_error("expected a height in metres after '" +
                             std::string(name) + ":' (got '" + text + "')");
  if (!(value > 0.0))
    throw std::runtime_error("--view " + std::string(name) +
                             ":<height> needs a positive height in metres "
                             "above the floor (got '" +
                             text + "')");
  return value;
}

/// `/tmp/view.png` + index 3 -> `/tmp/view_003.png`.
fs::path numbered_path(const fs::path &base, int index) {
  fs::path out = base;
  out.replace_filename(fmt::format("{}_{:03d}{}", base.stem().string(), index,
                                   base.extension().string()));
  return out;
}

void write_image(const cv::Mat &image, const fs::path &path) {
  if (path.has_parent_path() && !path.parent_path().empty())
    fs::create_directories(path.parent_path());
  if (!cv::imwrite(path.string(), image))
    throw std::runtime_error("failed to write image to " + path.string());
  spdlog::info("Wrote {} ({}x{})", path.string(), image.cols, image.rows);
}

} // namespace

void setup_subcommand_render(CLI::App &app,
                             std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandRenderOptions>();

  // Defaults come from the library options struct — never redefined here.
  const viz::RenderOptions defaults;
  opt->view = std::string(viz::to_string(defaults.view));
  opt->layers = default_layer_list(defaults);
  opt->size = fmt::format("{}x{}", defaults.width, defaults.height);
  opt->cloud_name = defaults.cloud_name;
  opt->mesh_name = defaults.mesh_name;
  opt->point_size = defaults.point_size;
  opt->orbit_elevation_deg = defaults.orbit_elevation_deg;
  opt->section_thickness = viz::SectionSpec{}.thickness;
  opt->mesh_style = std::string(viz::to_string(defaults.mesh_style));
  opt->deviation_within_m = defaults.deviation.within_m;
  opt->deviation_uncovered_m = defaults.deviation.uncovered_m;
  opt->drift_good_m = defaults.drift.good_m;
  opt->drift_bad_m = defaults.drift.bad_m;
  opt->pass_segments = defaults.passes.segments;
  opt->pass_depth_stride = defaults.passes.depth_stride;
  opt->pass_frame_step = defaults.passes.frame_step;
  opt->pass_voxel_m = defaults.passes.voxel_m;
  opt->graph_label_every = defaults.graph.label_every;

  auto *sub = app.add_subcommand(
      "render", "Render the project to an image file without a display");

  sub->footer(R"(
DESCRIPTION:
  Renders a .rux project off-screen and writes a PNG. Needs no X or Wayland
  session: the VTK backend falls back to EGL, so this works over SSH, inside
  CI, and from an agent's worktree.

EXAMPLES:
  # True floor plan: cut 1.2 m above the detected floor, looking down
  rux -p scan.rux render -o plan.png --view plan --layers cloud

  # The same, cut at 1.6 m to clear a high sill
  rux -p scan.rux render -o plan.png --view plan:1.6 --layers cloud

  # Eight views around the scene, coloured by plane segment
  rux -p scan.rux render -o orbit.png --view orbit:8 --layers planes

  # Mesh plus room colours, at a specific size
  rux -p scan.rux render -o rooms.png --layers mesh,rooms --size 1920x1080

  # Reproduce the viewpoint of a captured frame
  rux -p scan.rux render -o frame.png --view frame:1995

  # Check an authored solid-model plan against the cut cloud
  rux -p scan.rux render -o check.png --view plan:1.2 --plan ref.plan.json

  # Mesh over the cloud, seen from the west
  rux -p scan.rux render -o west.png --view left --layers cloud,mesh

  # A 10 cm vertical slab at x = 4.5 m, with the mesh's section line
  rux -p scan.rux render -o sx.png --section x=4.5 --layers cloud,mesh

  # Cloud coloured by distance to the mesh, from above, cut
  rux -p scan.rux render -o dev.png --view plan --layers deviation,mesh

  # The whole mesh-QA contact sheet (plus one PNG per panel next to it)
  rux -p scan.rux render --qa -o qa.png

  # Drift loop: passes in colour, the pose graph, the drift heatmap
  rux -p scan.rux render -o passes.png --view plan --layers passes
  rux -p scan.rux render -o graph.png --view plan --layers cloud,graph
  rux -p scan.rux analyze drift -o drift.json
  rux -p scan.rux render -o heat.png --view plan --layers cloud,drift \
      --drift-report drift.json

VIEWS:
  top          orthographic view looking down -Z (default). In a closed
               interior this shows the CEILING -- use `plan` for a floor plan.
  plan[:h]     `top` plus a horizontal cut: everything above h metres over the
               floor is clipped away, leaving the floor and the walls in
               section. Default h is 1.2 m above the floor plane detected by
               `rux create planes`, or 45 % of the scene height when the
               project has no planes yet.
  front        orthographic elevation from the south, looking along +Y
  back         ... from the north, looking along -Y
  left         ... from the west, looking along +X
  right        ... from the east, looking along -X
               (all four turned by --yaw / the project's view.yaw_deg, so
               "front" is seen from the bottom edge of the plan view)
  orbit[:N]    N perspective views on a ring; writes N numbered files
               (out_000.png, out_001.png, ...)
  frame:<id>   the pose and intrinsics of stored sensor frame <id>
  --section S  a vertical section: S is x=<m>, y=<m> or <x0>,<y0>:<x1>,<y1>;
               a slab of --section-thickness around that vertical plane,
               seen perpendicular to it with the line pointing right. More
               than one --section writes numbered files.

MESH QA (reusex-8vx):
  - With cloud and mesh layers together the mesh is drawn translucent with
    its feature edges (--mesh-style auto -> overlay), and plan cuts and
    sections add the mesh's own section line in vermillion.
  - The `deviation` layer colours each cloud point by its distance to the
    mesh: green on it, yellow at --deviation-within, vermillion up to
    --deviation-uncovered, reddish purple beyond (cloud the mesh does not
    cover). A legend with mean, P95 and the two shares is drawn in.
  - --qa writes a contact sheet: plan cut, four elevations, three sections
    (or the given --section ones) and a deviation plan.

NOTES:
  - Camera presets are derived from the geometry's bounding box, so the same
    project always frames the same shot. The cut changes what is visible, not
    how the shot is framed.
  - --cut-height applies the same cut to any view (e.g. a cut axonometric).
  - Layers draw back to front in the order given.
  - A missing layer is an error naming the stage that produces it.
  - --plan draws a solid-model plan, JSON or YAML (by extension: .yaml/.yml
    is YAML, anything else JSON; docs/design/solid-model-json.md) over the
    layers: walls as footprints with their axis, spaces as outlines,
    openings as bars with end ticks; labelled by space name and wall guid.
    In top/plan views it lies on top of everything and is not cut.

DRIFT LOOP (reusex-rcv.1.4):
  - `passes` back-projects the depth frames (every --pass-depth-stride-th
    pixel of every --pass-frame-step-th frame) and colours each point by the
    capture segment of its frame (--pass-segments equal frame counts, the
    segments `rux analyze drift --segments` uses): a wall the poses doubled
    shows in two colours. The legend lists each segment's node range.
  - `graph` draws the camera path (coloured by the same segments), the
    stored pose-graph edges coloured by their solve residual on the fixed
    0.5 chi2 scale of the Rediger graph panel (green -> yellow -> vermillion,
    purple beyond, grey never solved), node ids every --graph-label-every
    nodes, and the pose edits of --edits as pins. A plan cut keeps the path
    within a slab around the cut, so a storey's plan shows its own cameras.
  - `drift` draws the pass-to-pass residual cells of a drift report
    (--drift-report, else computed here with the defaults -- slow on a big
    capture) as a heatmap, one per storey; a plan cut shows the storey it
    cuts, --drift-storey picks one.

WATCH (ground-truth authoring loop, reusex-yf4.2):
  rux -p scan.rux render --plan gt.yaml --watch --qa -o /tmp/gt/qa.png
  rux -p scan.rux render --plan gt.yaml --watch --view plan:1.2 -o plan.png

  Renders once, then again every time the --plan file is saved (and, with
  --watch-project, when the project changes), until Ctrl-C or SIGTERM. Each
  pass prints a block to stdout: the plan's element counts, how well its
  walls follow the cloud (support share per wall, face offset; weak walls
  named), and the `rux analyze solid` summary of the project's model
  against the plan. A file that does not parse keeps the last good plan on
  screen and prints the reader's error, which names the element.
)");

  sub->add_option("-o,--output", opt->output, "Output image path")
      ->default_val(opt->output.string());
  sub->add_option(
         "--view", opt->view,
         "Camera: top, plan[:height], front, back, left, right, orbit[:N] "
         "or frame:<node_id>")
      ->default_val(opt->view);
  sub->add_option("--layers", opt->layers,
                  "Comma-separated layers (" + layer_vocabulary() + ")")
      ->default_val(opt->layers);
  sub->add_option("--size", opt->size, "Image size as WxH")
      ->default_val(opt->size);
  sub->add_option("--cloud", opt->cloud_name, "Named geometry cloud to draw")
      ->default_val(opt->cloud_name);
  sub->add_option("--mesh", opt->mesh_name, "Named mesh for the mesh layer")
      ->default_val(opt->mesh_name);
  sub->add_option("--point-size", opt->point_size, "Point size in pixels")
      ->default_val(opt->point_size);
  sub->add_option("--elevation", opt->orbit_elevation_deg,
                  "Orbit elevation above the scene, in degrees")
      ->default_val(opt->orbit_elevation_deg);
  sub->add_option("--cut-height", opt->cut_height,
                  fmt::format("Cut everything above this height in metres "
                              "above the floor; applies to any view "
                              "(default with --view plan: {:g} m)",
                              viz::kDefaultCutHeightM));
  sub->add_option("--plan", opt->plan,
                  "Solid-model plan (JSON or YAML) to draw over the layers "
                  "(walls, spaces, openings)")
      ->check(CLI::ExistingFile);
  sub->add_flag("--no-plan-labels", opt->no_plan_labels,
                "Do not label the --plan overlay's spaces and walls");
  sub->add_option("--yaw", opt->yaw_deg,
                  "View yaw in degrees, counter-clockwise seen from above "
                  "(default: the project's view.yaw_deg setting)");
  sub->add_option("--section", opt->sections,
                  "Vertical section: x=<m>, y=<m> or <x0>,<y0>:<x1>,<y1> "
                  "(repeatable)");
  sub->add_option("--section-thickness", opt->section_thickness,
                  "Section slab thickness in metres")
      ->default_val(opt->section_thickness);
  sub->add_option("--mesh-style", opt->mesh_style,
                  "Mesh layer: auto, solid, overlay or edges")
      ->default_val(opt->mesh_style);
  sub->add_option("--deviation-within", opt->deviation_within_m,
                  "Deviation layer: a point this close to the mesh fits it "
                  "[m]")
      ->default_val(opt->deviation_within_m);
  sub->add_option("--deviation-uncovered", opt->deviation_uncovered_m,
                  "Deviation layer: a point farther than this from the mesh "
                  "is not covered by it [m]")
      ->default_val(opt->deviation_uncovered_m);
  sub->add_flag("--no-legend", opt->no_legend,
                "Leave the deviation layer's legend out of the image");
  sub->add_option("--frame-quantile", opt->frame_quantile,
                  fmt::format("Frame on the [q, 1-q] per-axis quantiles of the "
                              "cloud so stray points do not shrink the view "
                              "(default {:g}; with --qa {:g})",
                              defaults.frame_quantile,
                              viz::QaSheetOptions{}.frame_quantile));
  sub->add_option("--drift-report", opt->drift_report,
                  "Drift layer: the JSON of `rux analyze drift -o` (default: "
                  "run the diagnostics here)")
      ->check(CLI::ExistingFile);
  sub->add_option("--drift-storey", opt->drift_storey,
                  "Drift layer: draw only this storey's cells (default: the "
                  "storey of the plan cut, else all)");
  sub->add_option("--drift-good", opt->drift_good_m,
                  "Drift layer: a cell this good is green [m]")
      ->default_val(opt->drift_good_m);
  sub->add_option("--drift-bad", opt->drift_bad_m,
                  "Drift layer: a cell this bad is vermillion [m]")
      ->default_val(opt->drift_bad_m);
  sub->add_option("--pass-segments", opt->pass_segments,
                  "Passes/graph layers: capture segments (one colour each)")
      ->default_val(opt->pass_segments)
      ->check(CLI::PositiveNumber);
  sub->add_option("--pass-depth-stride", opt->pass_depth_stride,
                  "Passes layer: sample every Nth depth pixel")
      ->default_val(opt->pass_depth_stride)
      ->check(CLI::PositiveNumber);
  sub->add_option("--pass-frame-step", opt->pass_frame_step,
                  "Passes layer: use every Nth frame")
      ->default_val(opt->pass_frame_step)
      ->check(CLI::PositiveNumber);
  sub->add_option("--pass-voxel", opt->pass_voxel_m,
                  "Passes layer: one point per voxel and segment [m] (0 = all)")
      ->default_val(opt->pass_voxel_m)
      ->check(CLI::NonNegativeNumber);
  sub->add_option("--graph-label-every", opt->graph_label_every,
                  "Graph layer: label every Nth node with its id (0 = none)")
      ->default_val(opt->graph_label_every)
      ->check(CLI::NonNegativeNumber);
  sub->add_flag("--graph-odometry", opt->graph_odometry,
                "Graph layer: draw odometry edges too");
  sub->add_flag("--graph-rejected", opt->graph_rejected,
                "Graph layer: draw rejected edges too");
  sub->add_option("--edits", opt->edits,
                  "Graph layer: pose-edits file (YAML/JSON) drawn as pins")
      ->check(CLI::ExistingFile);
  sub->add_flag("--qa", opt->qa,
                "Write the mesh-QA contact sheet to --output (and each panel "
                "next to it)");
  auto *watch = sub->add_flag(
      "--watch", opt->watch,
      "Keep running: re-render and re-score whenever the --plan file changes "
      "(Ctrl-C to stop)");
  sub->add_flag("--watch-project", opt->watch_project,
                "With --watch: also re-render when the project file changes")
      ->needs(watch);
  sub->add_option("--watch-interval", opt->watch_interval_ms,
                  "With --watch: poll interval in milliseconds")
      ->default_val(opt->watch_interval_ms)
      ->check(CLI::Range(10, 60000))
      ->needs(watch);
  // A test / scripting hook: stop after N re-renders (0 = the first pass
  // only). Hidden: a person stops the watch with Ctrl-C.
  sub->add_option("--watch-cycles", opt->watch_cycles,
                  "With --watch: stop after this many re-renders")
      ->group("")
      ->needs(watch);

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running render subcommand");
    rux::finish(run_subcommand_render(*opt, *global_opt));
  });
}

std::vector<fs::path>
render_to_files(SubcommandRenderOptions const &opt,
                const RuxOptions &global_opt,
                const std::optional<reusex::core::SolidModel> &plan) {
  std::vector<fs::path> written;
  const auto write = [&written](const cv::Mat &image, const fs::path &path) {
    write_image(image, path);
    written.push_back(path);
  };
  {
    viz::RenderOptions render_opts;
    render_opts.layers = parse_layers(opt.layers);
    parse_size(opt.size, render_opts.width, render_opts.height);
    render_opts.cloud_name = opt.cloud_name;
    render_opts.mesh_name = opt.mesh_name;
    render_opts.point_size = opt.point_size;
    render_opts.orbit_elevation_deg = opt.orbit_elevation_deg;
    render_opts.plan_labels = !opt.no_plan_labels;
    render_opts.yaw_deg = opt.yaw_deg;
    render_opts.legend = !opt.no_legend;
    if (opt.frame_quantile >= 0.0)
      render_opts.frame_quantile = opt.frame_quantile;
    render_opts.deviation.within_m = opt.deviation_within_m;
    render_opts.deviation.uncovered_m = opt.deviation_uncovered_m;
    if (const auto style = viz::mesh_style_from_string(opt.mesh_style))
      render_opts.mesh_style = *style;
    else
      throw std::runtime_error("unknown --mesh-style '" + opt.mesh_style +
                               "'; expected auto, solid, overlay or edges");
    std::vector<viz::SectionSpec> sections;
    for (const auto &text : opt.sections) {
      try {
        viz::SectionSpec spec = viz::parse_section(text);
        spec.thickness = opt.section_thickness;
        sections.push_back(spec);
      } catch (const std::invalid_argument &e) {
        throw std::runtime_error(std::string("--section: ") + e.what());
      }
    }
    if (!(opt.section_thickness > 0.0))
      throw std::runtime_error("--section-thickness must be positive");
    render_opts.plan = plan;
    render_opts.drift.good_m = opt.drift_good_m;
    render_opts.drift.bad_m = opt.drift_bad_m;
    render_opts.drift.storey = opt.drift_storey;
    render_opts.passes.segments = opt.pass_segments;
    render_opts.passes.depth_stride = opt.pass_depth_stride;
    render_opts.passes.frame_step = opt.pass_frame_step;
    render_opts.passes.voxel_m = opt.pass_voxel_m;
    render_opts.graph.label_every = opt.graph_label_every;
    render_opts.graph.odometry_edges = opt.graph_odometry;
    render_opts.graph.rejected_edges = opt.graph_rejected;
    if (!opt.edits.empty())
      render_opts.edits = reusex::io::read_pose_edits(opt.edits);

    reusex::ProjectDB db(global_opt.project_db, /* readOnly */ true);

    // ── Drift heatmap: a report file, or the diagnostics run here ─────────
    const bool wants_drift =
        std::find(render_opts.layers.begin(), render_opts.layers.end(),
                  viz::Layer::drift) != render_opts.layers.end();
    if (wants_drift) {
      std::string json;
      if (!opt.drift_report.empty()) {
        std::ifstream in(opt.drift_report);
        std::stringstream ss;
        ss << in.rdbuf();
        json = ss.str();
      } else {
        spdlog::info("No --drift-report: running the drift diagnostics with "
                     "their defaults (save them with `rux analyze drift -o` "
                     "to render again quickly)");
        json = reusex::slam::drift_report_to_json(
            reusex::slam::diagnose_drift(db, {}, plan ? &*plan : nullptr));
      }
      try {
        render_opts.drift_heatmap = viz::drift_heatmap_from_json(json);
      } catch (const std::invalid_argument &e) {
        throw std::runtime_error("--drift-report '" +
                                 opt.drift_report.string() + "': " + e.what());
      }
    }

    // ── Camera selection ────────────────────────────────────────────────
    // An explicit --cut-height cuts whatever view was asked for; `--view
    // plan[:h]` is the shorthand that also picks the top-down camera.
    if (opt.cut_height < 0.0)
      throw std::runtime_error(
          "--cut-height is measured upward from the floor and must be "
          "positive (got " +
          std::to_string(opt.cut_height) + ")");
    if (opt.cut_height > 0.0) {
      render_opts.cut = true;
      render_opts.cut_height = opt.cut_height;
    }

    // ── Mesh-QA contact sheet ─────────────────────────────────────────────
    if (opt.qa) {
      viz::QaSheetOptions qa;
      qa.base = render_opts;
      if (const auto height = parse_metre_suffix(opt.view, "plan"))
        qa.base.cut_height = *height;
      qa.sections = sections;
      if (opt.frame_quantile >= 0.0)
        qa.frame_quantile = opt.frame_quantile;
      const viz::QaSheet sheet = viz::render_qa_sheet(db, qa);
      write(sheet.sheet, opt.output);
      for (const auto &p : sheet.panels) {
        fs::path out = opt.output;
        out.replace_filename(fmt::format("{}_{}{}", opt.output.stem().string(),
                                         p.name,
                                         opt.output.extension().string()));
        write(p.image, out);
      }
      const auto report = [&](const char *what, const viz::DeviationStats &d) {
        spdlog::info("Deviation ({}): {} points, mean {:.3f} m, median {:.3f} "
                     "m, P95 {:.3f} m, {:.1f}% within {:.2f} m, {:.1f}% "
                     "beyond {:.2f} m (uncovered)",
                     what, d.points, d.mean_m, d.median_m, d.p95_m,
                     100.0 * d.within_share, qa.base.deviation.within_m,
                     100.0 * d.uncovered_share, qa.base.deviation.uncovered_m);
      };
      if (sheet.deviation)
        report("all points", *sheet.deviation);
      if (sheet.deviation_planar)
        report("planar points", *sheet.deviation_planar);
      return written;
    }

    std::optional<int> frame_id;
    if (!sections.empty()) {
      render_opts.view = viz::ViewPreset::section;
    } else if (opt.view == "top") {
      render_opts.view = viz::ViewPreset::top;
    } else if (opt.view == "plan") {
      render_opts.view = viz::ViewPreset::plan;
    } else if (const auto height = parse_metre_suffix(opt.view, "plan")) {
      render_opts.view = viz::ViewPreset::plan;
      render_opts.cut_height = *height;
    } else if (opt.view == "front") {
      render_opts.view = viz::ViewPreset::front;
    } else if (opt.view == "back") {
      render_opts.view = viz::ViewPreset::back;
    } else if (opt.view == "left") {
      render_opts.view = viz::ViewPreset::left;
    } else if (opt.view == "right") {
      render_opts.view = viz::ViewPreset::right;
    } else if (opt.view == "orbit") {
      render_opts.view = viz::ViewPreset::orbit;
    } else if (const auto count = parse_suffix(opt.view, "orbit")) {
      // Guard the count here: the render loop below would otherwise spin zero
      // times and report success without writing a file, never reaching the
      // library's own validation (STANDARDS §5).
      if (*count < 1)
        throw std::runtime_error("--view orbit:N needs N >= 1 (got '" +
                                 opt.view + "')");
      render_opts.view = viz::ViewPreset::orbit;
      render_opts.orbit_count = *count;
    } else if (const auto node = parse_suffix(opt.view, "frame")) {
      render_opts.view = viz::ViewPreset::explicit_camera;
      frame_id = *node;
      render_opts.camera = viz::camera_from_sensor_frame(
          db, *node, render_opts.width, render_opts.height);
    } else {
      throw std::runtime_error(
          "unknown --view '" + opt.view +
          "'; expected top, plan[:height], front, back, left, right, "
          "orbit[:N] or frame:<node_id>");
    }

    // ── Render ──────────────────────────────────────────────────────────
    if (render_opts.view == viz::ViewPreset::section) {
      for (std::size_t i = 0; i < sections.size(); ++i) {
        render_opts.section = sections[i];
        write(viz::render_view(db, render_opts),
              sections.size() == 1
                  ? opt.output
                  : numbered_path(opt.output, static_cast<int>(i)));
      }
    } else if (render_opts.view == viz::ViewPreset::orbit) {
      for (int i = 0; i < render_opts.orbit_count; ++i) {
        render_opts.orbit_index = i;
        write(viz::render_view(db, render_opts), numbered_path(opt.output, i));
      }
      spdlog::info("Rendered {} orbit views around {}", render_opts.orbit_count,
                   global_opt.project_db.string());
    } else {
      write(viz::render_view(db, render_opts), opt.output);
      if (frame_id)
        spdlog::info("Rendered from the viewpoint of sensor frame {}",
                     *frame_id);
    }

    return written;
  }
}

int run_subcommand_render(SubcommandRenderOptions const &opt,
                          const RuxOptions &global_opt) {
  if (opt.watch)
    return run_render_watch(opt, global_opt);
  try {
    std::optional<reusex::core::SolidModel> plan;
    if (!opt.plan.empty()) {
      try {
        plan = reusex::io::solid_model_from_file(opt.plan);
      } catch (const std::invalid_argument &e) {
        spdlog::error("Render failed: invalid plan '{}': {}", opt.plan.string(),
                      e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }
    render_to_files(opt, global_opt, plan);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("Render failed: {}", e.what());
    return RuxError::IO;
  }
}
