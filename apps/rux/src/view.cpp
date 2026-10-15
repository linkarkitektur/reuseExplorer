// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include "view.hpp"
#include "processing_observer.hpp"

#include "exit_status.hpp"
#include "view/component_renderer.hpp"
#include "view/label_renderer.hpp"
#include "view/panorama_handler.hpp"
#include "view/project_loader.hpp"
#include "view/turntable.hpp"
#include "view/viewer_types.hpp"

#include <fmt/format.h>
#include <fmt/std.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

#include <vtkCamera.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

using namespace rux::view;

// ============================================================================
// SUBCOMMAND SETUP
// ============================================================================

void setup_subcommand_view(CLI::App &app,
                           std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandViewOptions>();
  auto *sub = app.add_subcommand("view", "Visualize point clouds and meshes");

  sub->add_option(
         "-f, --filter", opt->filter_expr,
         "Filter expression to limit visualization to specific labeled "
         "points.\n"
         "Syntax: <cloud_name> <op> <value(s)>\n"
         "Examples:\n"
         "  -f 'planes in [1, 2, 5]'        # Show only labels 1, 2, 5 from "
         "planes cloud\n"
         "  -f 'rooms == 3'                 # Show only room 3\n"
         "  -f 'planes in [1,2] || rooms == 5'  # Combine multiple clouds\n"
         "  -f 'planes >= 10 && planes <= 20'   # Range filter")
      ->default_val("");

  auto *plan = sub->add_option("--plan", opt->plan,
                               "Ground-truth workbench: draw this solid-model "
                               "plan (JSON or YAML) over the scene")
                   ->check(CLI::ExistingFile);
  sub->add_flag("--watch", opt->watch,
                "With --plan: reload the plan whenever the file is saved")
      ->needs(plan);
  sub->add_option("--layers", opt->layers,
                  "With --plan: comma-separated scene layers (as in "
                  "`rux render`)")
      ->default_val(opt->layers)
      ->needs(plan);
  sub->add_option("--cloud", opt->cloud_name,
                  "With --plan: named geometry cloud")
      ->default_val(opt->cloud_name)
      ->needs(plan);
  sub->add_option("--cut-height", opt->cut_height,
                  "With --plan: initial plan cut above the floor in metres "
                  "(default: as `rux render --view plan`)")
      ->needs(plan);
  sub->add_option("--watch-interval", opt->watch_interval_ms,
                  "With --watch: poll interval in milliseconds")
      ->default_val(opt->watch_interval_ms)
      ->check(CLI::Range(10, 60000))
      ->needs(plan);
  sub->add_option("--max-points", opt->max_points,
                  "With --plan: draw at most this many points per point layer "
                  "(0 = all)")
      ->default_val(opt->max_points)
      ->needs(plan);

  sub->footer(R"(
DESCRIPTION:
  Interactive 3D visualization of point clouds, meshes, and segmentation
  labels stored in ProjectDB. Loads all available geometry and provides
  keyboard controls for toggling visibility, switching label overlays,
  and exploring multi-modal scan data.

EXAMPLES:
  rux view                             # View project at ./project.rux
  rux -p scan.rux view                 # View custom project
  rux view -f 'planes == 3'            # View only plane 3
  rux view -f 'rooms in [1,2]'         # View only rooms 1 and 2
  rux -vv view                         # Debug mode with verbose output

KEYBOARD CONTROLS:
  c           Toggle all point clouds on/off
  m           Toggle all meshes on/off
  w           Toggle building components (windows, doors, etc.)
  1-9         Toggle label overlay (planes, rooms, etc.)
  l           Toggle label color legend
  Shift+Click Enter 360 panorama (click on orange sphere)
  Escape      Exit panorama mode
  [/]         Previous/Next panorama (in panorama mode)
  k           Toggle turntable orbit mode on/off
  Space       Pause/resume turntable spinning
  ,/.         Decrease/increase turntable speed
  h           Show help message
  q           Quit viewer

GROUND-TRUTH WORKBENCH (--plan, reusex-yf4.2):
  rux -p scan.rux view --plan gt.yaml --watch
  rux -p scan.rux view --plan gt.yaml --watch --layers cloud,planes

  Shows the scene `rux render` draws (visualize::populate_scene: --layers,
  default the cloud) cut at a plan height, with the plan on top, and with
  --watch reloads the plan every time the file is saved. A save that does not
  parse keeps the last good plan and shows the reader's error (it names the
  element) on the status line at the bottom. Keys in this mode:
    n / b     next / previous storey (all, then each storey of the plan)
    ] / [     raise / lower the plan cut by 10 cm
    } / {     raise / lower the plan cut by 1 m
    v         plan cut on / off
    1-9       toggle scene layer 1-9 (--layers order)
    0         toggle the plan overlay
    h         print these keys to the terminal
  The headless twin for agents is `rux render --plan gt.yaml --watch`.

WORKFLOW:
  1. rux import rtabmap scan.db        # Import sensor data
  2. rux create clouds                 # Reconstruct geometry
  3. rux create planes                 # Add segmentation
  4. rux view                          # Visualize results

NOTES:
  - Loads all point clouds, meshes, and labels from ProjectDB automatically
  - Filter applies uniformly to all XYZRGB clouds with corresponding label filtering
  - Meshes and building components are not filtered (shown in full)
  - Label clouds mapped to first XYZRGB cloud for geometry
  - Supports multiple viewports for simultaneous views
  - PCL viewer based on VTK - standard camera controls apply
  - Use global -p/--project flag to specify database path
)");

  // View command loads data exclusively from the project database
  // Use global --project flag to specify database path

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling viewer subcommand");
    rux::finish(run_subcommand_view(*opt, *global_opt));
  });
}

// ============================================================================
// MAIN SUBCOMMAND FUNCTION
// ============================================================================

int run_subcommand_view([[maybe_unused]] SubcommandViewOptions const &opt,
                        [[maybe_unused]] const RuxOptions &global_opt) {
  if (!opt.plan.empty())
    return run_view_plan(opt, global_opt);
  spdlog::trace("Visualization thread started");

  // Start the viewer thread
  rux::start_viewer();
  rux::VizualizationObserver &observer = rux::get_processing_observer();

  // === Phase 1: Load Data ===
  std::vector<LoadedCloud> clouds;
  std::vector<LoadedMesh> meshes;
  std::vector<LabelCloudInfo> labels;

  // Load data from project database
  fs::path project_path = global_opt.project_db;
  spdlog::debug("Loading visualization data from project database: {}",
                project_path.string());

  auto result = load_from_project_db(project_path, observer, opt.filter_expr);
  clouds = std::move(result.clouds);
  meshes = std::move(result.meshes);
  labels = std::move(result.labels);
  auto components = std::move(result.components);

  // Setup panorama state
  auto pano_state = std::make_shared<PanoramaState>();
  pano_state->infos = std::move(result.panoramas);

  // Setup turntable state
  auto turntable_state = std::make_shared<TurntableState>();

  // === Phase 2: Register Keyboard Callbacks ===
  register_turntable_callbacks(turntable_state, observer);

  register_individual_toggles(clouds, observer);
  register_toggle_all_callback(clouds, observer);

  register_individual_toggles(meshes, observer);
  register_toggle_all_callback(meshes, observer);

  if (!labels.empty()) {
    auto state = std::make_shared<ViewerState>();
    register_label_toggles(labels, state, observer);
    register_legend_toggle(labels, state, observer);
  }

  // Render building components as line loops and register 'w' toggle
  if (!components.empty()) {
    add_component_lines(components, observer);
    auto components_visible = std::make_shared<bool>(true);
    register_component_toggle(components, components_visible, observer);
  }

  // Render panorama position spheres and register callbacks
  bool has_panoramas = false;
  if (!pano_state->infos.empty()) {
    // Add orange spheres at panorama positions
    for (size_t i = 0; i < pano_state->infos.size(); ++i) {
      const auto &pi = pano_state->infos[i];
      if (!pi.pose_valid)
        continue;
      has_panoramas = true;
      auto sphere_name = fmt::format("pano_sphere_{}", i);
      float px = static_cast<float>(pi.px);
      float py = static_cast<float>(pi.py);
      float pz = static_cast<float>(pi.pz);
      observer.viewer_enqueue_task(
          [px, py, pz,
           sphere_name](const rux::VizualizationObserver::ViewerPtr &viewer,
                        const std::vector<int> &viewports) {
            viewer->addSphere(pcl::PointXYZ(px, py, pz), 0.15, 1.0, 0.5, 0.0,
                              sphere_name, viewports[0]);
          });
    }

    // Register Shift+Click point picking callback for panorama entry
    observer.viewer_enqueue_task(
        [pano_state, project_path,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerPointPickingCallback(
              [pano_state, project_path,
               &observer](const pcl::visualization::PointPickingEvent &event) {
                if (pano_state->immersive)
                  return;
                if (event.getPointIndex() == -1)
                  return;

                float x, y, z;
                event.getPoint(x, y, z);

                // Find nearest panorama sphere
                int best_idx = -1;
                double best_dist = 0.5; // threshold in meters
                for (size_t i = 0; i < pano_state->infos.size(); ++i) {
                  const auto &pi = pano_state->infos[i];
                  if (!pi.pose_valid)
                    continue;
                  double dx = x - pi.px;
                  double dy = y - pi.py;
                  double dz = z - pi.pz;
                  double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                  if (d < best_dist) {
                    best_dist = d;
                    best_idx = static_cast<int>(i);
                  }
                }

                if (best_idx >= 0) {
                  observer.viewer_enqueue_task(
                      [pano_state, best_idx, project_path](
                          const rux::VizualizationObserver::ViewerPtr &v,
                          const std::vector<int> &vp) {
                        enter_panorama_mode(pano_state, best_idx, project_path,
                                            v, vp);
                      });
                }
              });
        });

    // Register Escape key to exit panorama mode
    observer.viewer_enqueue_task(
        [pano_state,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerKeyboardCallback(
              [pano_state,
               &observer](const pcl::visualization::KeyboardEvent &event) {
                if (event.getKeySym() == "Escape" && event.keyDown() &&
                    pano_state->immersive) {
                  observer.viewer_enqueue_task(
                      [pano_state](
                          const rux::VizualizationObserver::ViewerPtr &v,
                          const std::vector<int> &vp) {
                        exit_panorama_mode(pano_state, v, vp);
                      });
                }
              });
        });

    // Register [ / ] keys for cycling panoramas in immersive mode
    observer.viewer_enqueue_task(
        [pano_state, project_path, &clouds, &meshes,
         &observer](const rux::VizualizationObserver::ViewerPtr &viewer,
                    const std::vector<int> &) {
          viewer->registerKeyboardCallback(
              [pano_state, project_path, &clouds, &meshes,
               &observer](const pcl::visualization::KeyboardEvent &event) {
                if (!pano_state->immersive || !event.keyDown())
                  return;

                int delta = 0;
                if (event.getKeySym() == "bracketright")
                  delta = 1;
                else if (event.getKeySym() == "bracketleft")
                  delta = -1;
                else
                  return;

                // Find next panorama with a valid pose
                int count = static_cast<int>(pano_state->infos.size());
                int cur = pano_state->active_index;
                for (int step = 1; step < count; ++step) {
                  int idx = ((cur + delta * step) % count + count) % count;
                  if (pano_state->infos[static_cast<size_t>(idx)].pose_valid) {
                    observer.viewer_enqueue_task(
                        [pano_state, idx, project_path, &clouds, &meshes](
                            const rux::VizualizationObserver::ViewerPtr &v,
                            const std::vector<int> &vp) {
                          deactivate_panorama_skybox(pano_state->skybox, v,
                                                     vp[0]);
                          const auto &info =
                              pano_state->infos[static_cast<size_t>(idx)];
                          activate_panorama_skybox(
                              project_path, info, pano_state->skybox, v, vp[0]);
                          // Update camera to new panorama position
                          if (info.pose_valid) {
                            auto *renderer = viewport_renderer(v, vp[0]);
                            auto *cam = renderer->GetActiveCamera();
                            cam->SetPosition(info.px, info.py, info.pz);
                            cam->SetFocalPoint(info.px + info.pose[2],
                                               info.py + info.pose[6],
                                               info.pz + info.pose[10]);
                            cam->SetViewUp(-info.pose[1], -info.pose[5],
                                           -info.pose[9]);
                            cam->SetClippingRange(0.01, 100.0);
                          }
                          pano_state->active_index = idx;
                          spdlog::info("Panorama: {} [{}]", info.filename,
                                       idx + 1);
                        });
                    break;
                  }
                }
              });
        });
  }

  register_help_callback(clouds, meshes, labels, !components.empty(),
                         has_panoramas, observer);

  // === Phase 3: Display Summary ===
  spdlog::info("Press 'h' for keyboard controls");
  int pano_count = 0;
  for (const auto &pi : pano_state->infos)
    if (pi.pose_valid)
      ++pano_count;
  spdlog::info(
      "Loaded: {} cloud(s), {} mesh(es), {} label(s), {} component(s), {} "
      "panorama(s)",
      clouds.size(), meshes.size(), labels.size(), components.size(),
      pano_count);

  // We need to wait here otherwise the clouds go out of scope and get destroyed
  // and the viewer segfaults when it tries to access them.
  observer.viewer_wait_for_user();

  return 0;
}
