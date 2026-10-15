// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "render.hpp"
#include "exit_status.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/io/solid_model_json.hpp>
#include <reusex/visualize/render_view.hpp>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/spdlog.h>

#include <charconv>
#include <optional>
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

VIEWS:
  top          orthographic view looking down -Z (default). In a closed
               interior this shows the CEILING -- use `plan` for a floor plan.
  plan[:h]     `top` plus a horizontal cut: everything above h metres over the
               floor is clipped away, leaving the floor and the walls in
               section. Default h is 1.2 m above the floor plane detected by
               `rux create planes`, or 45 % of the scene height when the
               project has no planes yet.
  front        orthographic elevation, looking along +Y
  orbit[:N]    N perspective views on a ring; writes N numbered files
               (out_000.png, out_001.png, ...)
  frame:<id>   the pose and intrinsics of stored sensor frame <id>

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
)");

  sub->add_option("-o,--output", opt->output, "Output image path")
      ->default_val(opt->output.string());
  sub->add_option(
         "--view", opt->view,
         "Camera: top, plan[:height], front, orbit[:N] or frame:<node_id>")
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

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running render subcommand");
    rux::finish(run_subcommand_render(*opt, *global_opt));
  });
}

int run_subcommand_render(SubcommandRenderOptions const &opt,
                          const RuxOptions &global_opt) {
  try {
    viz::RenderOptions render_opts;
    render_opts.layers = parse_layers(opt.layers);
    parse_size(opt.size, render_opts.width, render_opts.height);
    render_opts.cloud_name = opt.cloud_name;
    render_opts.mesh_name = opt.mesh_name;
    render_opts.point_size = opt.point_size;
    render_opts.orbit_elevation_deg = opt.orbit_elevation_deg;
    render_opts.plan_labels = !opt.no_plan_labels;
    if (!opt.plan.empty()) {
      try {
        render_opts.plan = reusex::io::solid_model_from_file(opt.plan);
      } catch (const std::invalid_argument &e) {
        spdlog::error("Render failed: invalid plan '{}': {}", opt.plan.string(),
                      e.what());
        return RuxError::INVALID_ARGUMENT;
      }
    }

    reusex::ProjectDB db(global_opt.project_db, /* readOnly */ true);

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

    std::optional<int> frame_id;
    if (opt.view == "top") {
      render_opts.view = viz::ViewPreset::top;
    } else if (opt.view == "plan") {
      render_opts.view = viz::ViewPreset::plan;
    } else if (const auto height = parse_metre_suffix(opt.view, "plan")) {
      render_opts.view = viz::ViewPreset::plan;
      render_opts.cut_height = *height;
    } else if (opt.view == "front") {
      render_opts.view = viz::ViewPreset::front;
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
          "'; expected top, plan[:height], front, orbit[:N] or "
          "frame:<node_id>");
    }

    // ── Render ──────────────────────────────────────────────────────────
    if (render_opts.view == viz::ViewPreset::orbit) {
      for (int i = 0; i < render_opts.orbit_count; ++i) {
        render_opts.orbit_index = i;
        write_image(viz::render_view(db, render_opts),
                    numbered_path(opt.output, i));
      }
      spdlog::info("Rendered {} orbit views around {}", render_opts.orbit_count,
                   global_opt.project_db.string());
    } else {
      write_image(viz::render_view(db, render_opts), opt.output);
      if (frame_id)
        spdlog::info("Rendered from the viewpoint of sensor frame {}",
                     *frame_id);
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Render failed: {}", e.what());
    return RuxError::IO;
  }
}
