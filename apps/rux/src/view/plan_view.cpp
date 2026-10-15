// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux view --plan <gt> [--watch]` (reusex-yf4.2): the PCL-viewer glue around
// view::PlanWorkbench. The main thread watches the plan file and parses it;
// everything VTK runs on the viewer thread through enqueued tasks.

#include "exit_status.hpp"
#include "plan_watch.hpp"
#include "processing_observer.hpp"
#include "view.hpp"
#include "view/panorama_handler.hpp" // viewport_renderer()
#include "view/plan_workbench.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/utils/file_watcher.hpp>
#include <reusex/utils/parse_number.hpp>
#include <reusex/visualize/scene.hpp>

#include <fmt/format.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace viz = reusex::visualize;
using rux::view::PlanWorkbench;

namespace {

constexpr const char *kStatusId = "reusex.plan_status";
constexpr int kStatusFontSize = 13;

std::vector<viz::Layer> parse_view_layers(const std::string &text) {
  std::vector<viz::Layer> layers;
  std::stringstream ss(text);
  std::string name;
  while (std::getline(ss, name, ',')) {
    const auto first = name.find_first_not_of(" \t");
    if (first == std::string::npos)
      continue;
    name = name.substr(first, name.find_last_not_of(" \t") - first + 1);
    const auto layer = viz::layer_from_string(name);
    if (!layer)
      throw std::runtime_error("unknown layer '" + name + "' in --layers");
    layers.push_back(*layer);
  }
  if (layers.empty())
    throw std::runtime_error("--layers is empty");
  return layers;
}

/// The workbench, owned by the viewer thread (created by its first task).
struct Shared {
  std::unique_ptr<PlanWorkbench> bench;
};

void show_status(const rux::VizualizationObserver::ViewerPtr &viewer,
                 const PlanWorkbench &bench) {
  const std::string line = bench.status_line();
  const bool error = !bench.error().empty();
  const double r = error ? 0.84 : 0.15, g = error ? 0.12 : 0.15,
               b = error ? 0.0 : 0.15;
  if (!viewer->updateText(line, 10, 10, kStatusFontSize, r, g, b, kStatusId))
    viewer->addText(line, 10, 10, kStatusFontSize, r, g, b, kStatusId);
}

} // namespace

int run_view_plan(SubcommandViewOptions const &opt,
                  const RuxOptions &global_opt) {
  std::vector<viz::Layer> layers;
  try {
    layers = parse_view_layers(opt.layers);
  } catch (const std::exception &e) {
    spdlog::error("view --plan: {}", e.what());
    return RuxError::INVALID_ARGUMENT;
  }

  rux::PlanFile plan_file(opt.plan);
  if (!plan_file.reload()) {
    spdlog::error("Plan '{}': {}", opt.plan.string(), plan_file.error());
    if (!opt.watch)
      return RuxError::INVALID_ARGUMENT;
    spdlog::warn("Watching it anyway: fix the file and save");
  }

  rux::start_viewer();
  rux::VizualizationObserver &observer = rux::get_processing_observer();
  auto shared = std::make_shared<Shared>();
  const fs::path project = global_opt.project_db;
  const std::string source = opt.plan.filename().string();

  // ── Build the scene and the workbench (viewer thread) ────────────────────
  observer.viewer_enqueue_task(
      [shared, project, source, layers, opt, first_plan = plan_file.model(),
       first_gen = plan_file.generation(), first_error = plan_file.error()](
          const rux::VizualizationObserver::ViewerPtr &viewer,
          const std::vector<int> &viewports) {
        vtkRenderer *renderer =
            rux::view::viewport_renderer(viewer, viewports[0]);
        try {
          reusex::ProjectDB db(project, /*readOnly=*/true);
          viz::SceneOptions so;
          so.layers = layers;
          so.cloud_name = opt.cloud_name;
          so.max_points = opt.max_points;
          double yaw = 0.0;
          if (const auto text =
                  db.setting(reusex::core::pose_graph::kSettingViewYawDeg))
            yaw = reusex::utils::parse_number<double>(*text).value_or(0.0);
          so.frame_yaw_deg = yaw;
          const viz::SceneInfo info = viz::populate_scene(renderer, db, so);
          const viz::CutPlane cut = viz::resolve_cut_plane(
              db, info.bounds,
              opt.cut_height > 0.0 ? std::optional<double>(opt.cut_height)
                                   : std::nullopt);
          shared->bench = std::make_unique<PlanWorkbench>(
              renderer, info, so, cut.floor_z, cut.height, source);
          if (first_plan)
            shared->bench->show_plan(*first_plan, first_gen);
          if (!first_error.empty())
            shared->bench->show_error(first_error);

          viz::CameraFraming framing;
          framing.view = viz::ViewPreset::top;
          framing.yaw_deg = yaw;
          framing.display_bounds = info.display_bounds;
          int *size = viewer->getRenderWindow()->GetSize();
          if (size != nullptr && size[1] > 0)
            framing.aspect = static_cast<double>(size[0]) / size[1];
          viz::place_preset_camera(renderer, framing, info.frame_bounds);
          show_status(viewer, *shared->bench);
          spdlog::info("Workbench: {} point(s) drawn; {}", info.drawn_points,
                       shared->bench->status_line());
        } catch (const std::exception &e) {
          spdlog::error("view --plan: cannot build the scene: {}", e.what());
          viewer->addText(fmt::format("cannot build the scene: {}", e.what()),
                          10, 10, kStatusFontSize, 0.84, 0.12, 0.0, kStatusId);
          return;
        }

        viewer->registerKeyboardCallback(
            [shared, viewer_weak = std::weak_ptr(viewer)](
                const pcl::visualization::KeyboardEvent &event) {
              if (!event.keyDown() || !shared->bench)
                return;
              PlanWorkbench &b = *shared->bench;
              const std::string key = event.getKeySym();
              bool handled = true;
              if (key == "n")
                b.cycle_storey(+1);
              else if (key == "b")
                b.cycle_storey(-1);
              else if (key == "bracketright")
                b.move_cut(+0.1);
              else if (key == "bracketleft")
                b.move_cut(-0.1);
              else if (key == "braceright")
                b.move_cut(+1.0);
              else if (key == "braceleft")
                b.move_cut(-1.0);
              else if (key == "v")
                b.toggle_cut();
              else if (key == "0")
                b.toggle_plan();
              else if (key.size() == 1 && key[0] >= '1' && key[0] <= '9')
                handled =
                    b.toggle_layer(static_cast<std::size_t>(key[0] - '1'));
              else if (key == "h") {
                spdlog::info("Workbench keys:\n{}", PlanWorkbench::key_help());
                handled = false;
              } else
                handled = false;
              if (!handled)
                return;
              if (auto v = viewer_weak.lock()) {
                show_status(v, b);
                v->getRenderWindow()->Render();
              }
              spdlog::info("{}", b.status_line());
            });
      });

  spdlog::info("Workbench keys: n/b storey, ]/[ cut +-10 cm, }/{ +-1 m, v "
               "cut on/off, 1-9 layers, 0 plan, h help");

  // ── Watch the plan (main thread) ─────────────────────────────────────────
  if (opt.watch) {
    reusex::utils::FileWatcher watcher({opt.plan});
    std::atomic<bool> never{false};
    const auto interval = std::chrono::milliseconds(opt.watch_interval_ms);
    spdlog::info("Watching {} -- save it to reload; close the window to stop",
                 opt.plan.string());
    while (observer.viewer_is_active()) {
      // Bounded, so a closed window is noticed within a second.
      if (watcher.wait(interval, never, std::chrono::milliseconds(1000))
              .empty())
        continue;
      const bool ok = plan_file.reload();
      if (ok)
        spdlog::info("Plan reloaded (load {}): {}", plan_file.generation(),
                     rux::plan_counts(*plan_file.model()));
      else
        spdlog::error("Plan '{}' did not parse; keeping the last good one: {}",
                      opt.plan.string(), plan_file.error());
      observer.viewer_enqueue_task(
          [shared, ok, model = plan_file.model(), gen = plan_file.generation(),
           error = plan_file.error()](
              const rux::VizualizationObserver::ViewerPtr &viewer,
              const std::vector<int> &) {
            if (!shared->bench)
              return;
            if (ok && model)
              shared->bench->show_plan(*model, gen);
            else
              shared->bench->show_error(error);
            show_status(viewer, *shared->bench);
          });
    }
  }

  observer.viewer_wait_for_user();
  return RuxError::SUCCESS;
}
