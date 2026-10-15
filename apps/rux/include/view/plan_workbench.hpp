// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `rux view --plan <gt> [--watch]` (reusex-yf4.2): the interactive half of
// the ground-truth authoring loop. The scene (cloud and the other --layers)
// is built once by the shared visualize::populate_scene(); the plan overlay
// on top of it is swapped whenever the plan file is saved.
//
// PlanWorkbench is the viewer-independent state behind the keys: which
// storey is shown, where the plan cut sits, which layers are on, and the
// status line. It needs a vtkRenderer and nothing else -- no window, no PCL
// visualizer -- so the reload logic is tested headlessly
// (tests/unit/rux_app/test_plan_workbench.cpp). Like every VTK object, use it
// from the viewer thread only.

#include <reusex/core/solid_model.hpp>
#include <reusex/visualize/scene.hpp>

#include <vtkSmartPointer.h>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

class vtkRenderer;

namespace rux::view {

/// @p plan restricted to storey @p idx: its storey record, the walls and
/// spaces standing on it, and the openings in those walls.
reusex::core::SolidModel plan_for_storey(const reusex::core::SolidModel &plan,
                                         int idx);

/// The storey indices @p plan uses (records, walls, spaces), ascending.
std::vector<int> plan_storeys(const reusex::core::SolidModel &plan);

/// World z of storey @p idx's floor (its z_floor taken out of the level
/// frame), or nullopt when the plan has no record for it.
std::optional<double> storey_floor_world_z(const reusex::core::SolidModel &plan,
                                           int idx);

class PlanWorkbench {
    public:
  /// @param scene    what populate_scene() added to @p renderer
  /// @param opts     the options it was called with (overlay style)
  /// @param floor_z  world z the cut is measured from with no storey chosen
  /// @param cut_height  initial cut height above the floor [m]
  /// @param source   the plan's name for the status line (its file name)
  PlanWorkbench(vtkRenderer *renderer, reusex::visualize::SceneInfo scene,
                reusex::visualize::SceneOptions opts, double floor_z,
                double cut_height, std::string source);
  ~PlanWorkbench();
  PlanWorkbench(const PlanWorkbench &) = delete;
  PlanWorkbench &operator=(const PlanWorkbench &) = delete;

  /// A plan file was accepted (load number @p generation): draw it.
  void show_plan(const reusex::core::SolidModel &plan, int generation);
  /// A save did not parse: keep the plan on screen, show @p error.
  void show_error(std::string error);

  /// Step through "all storeys" and each storey of the plan.
  void cycle_storey(int step);
  /// Move the cut by @p delta_m (kept above 5 cm).
  void move_cut(double delta_m);
  void toggle_cut();
  /// Toggle layer @p index (0-based, --layers order).
  /// @return false when there is no such layer.
  bool toggle_layer(std::size_t index);
  void toggle_plan();

  /// One line for the on-screen status: the last reload or error, storey,
  /// cut and layers.
  [[nodiscard]] std::string status_line() const;
  /// The key help, one key per line.
  [[nodiscard]] static std::string key_help();

  [[nodiscard]] std::optional<int> storey() const { return storey_; }
  [[nodiscard]] bool cut_enabled() const { return cut_; }
  [[nodiscard]] double cut_height() const { return cut_height_; }
  /// World z of the cut plane (floor of the chosen storey + cut height).
  [[nodiscard]] double cut_z() const;
  [[nodiscard]] bool plan_visible() const { return plan_visible_; }
  [[nodiscard]] bool layer_visible(std::size_t index) const;
  [[nodiscard]] const reusex::visualize::PlanOverlayInfo &overlay() const {
    return overlay_;
  }
  [[nodiscard]] const std::string &error() const { return error_; }

    private:
  void rebuild_overlay();
  void apply_cut();

  /// Held, not borrowed: the viewer may drop its renderer before the
  /// workbench is destroyed.
  vtkSmartPointer<vtkRenderer> renderer_;
  reusex::visualize::SceneInfo scene_;
  reusex::visualize::SceneOptions opts_;
  double floor_z_;
  double cut_height_;
  std::string source_;

  std::optional<reusex::core::SolidModel> plan_;
  int generation_ = 0;
  std::string error_;
  std::optional<int> storey_; ///< nullopt = all storeys
  bool cut_ = true;
  bool plan_visible_ = true;
  std::vector<bool> layer_visible_;
  reusex::visualize::PlanOverlayInfo overlay_;
};

} // namespace rux::view
