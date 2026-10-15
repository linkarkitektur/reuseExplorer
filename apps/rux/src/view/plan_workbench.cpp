// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/plan_workbench.hpp"
#include "plan_watch.hpp"

#include <reusex/geometry/transform_utils.hpp>

#include <fmt/format.h>

#include <vtkActor.h>
#include <vtkRenderer.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <set>
#include <unordered_set>
#include <utility>

namespace viz = reusex::visualize;
using reusex::core::SolidModel;

namespace rux::view {

namespace {

/// Shortest cut the keys can set [m].
constexpr double kMinCutM = 0.05;
/// How far under the cut plane the plan sheet is drawn [m].
constexpr double kSheetBelowCutM = 0.01;
/// Longest error text the status line shows; the full text is logged.
constexpr std::size_t kStatusErrorChars = 160;

void set_visible(const std::vector<vtkActor *> &actors, bool on) {
  for (auto *a : actors)
    if (a != nullptr)
      a->SetVisibility(on ? 1 : 0);
}

} // namespace

SolidModel plan_for_storey(const SolidModel &plan, int idx) {
  SolidModel out;
  out.name = plan.name;
  out.description = plan.description;
  out.provided = plan.provided;
  for (const auto &s : plan.storeys)
    if (s.idx == idx)
      out.storeys.push_back(s);
  std::unordered_set<std::string> walls;
  for (const auto &w : plan.walls)
    if (w.storey == idx) {
      out.walls.push_back(w);
      walls.insert(w.guid);
    }
  for (const auto &s : plan.spaces)
    if (s.storey == idx)
      out.spaces.push_back(s);
  for (const auto &o : plan.openings)
    if (walls.count(o.wall) > 0)
      out.openings.push_back(o);
  return out;
}

std::vector<int> plan_storeys(const SolidModel &plan) {
  std::set<int> idx;
  for (const auto &s : plan.storeys)
    idx.insert(s.idx);
  for (const auto &w : plan.walls)
    idx.insert(w.storey);
  for (const auto &s : plan.spaces)
    idx.insert(s.storey);
  return {idx.begin(), idx.end()};
}

std::optional<double> storey_floor_world_z(const SolidModel &plan, int idx) {
  for (const auto &s : plan.storeys)
    if (s.idx == idx) {
      const Eigen::Affine3d to_world =
          reusex::geometry::to_affine(s.level_transform).inverse();
      return (to_world * Eigen::Vector3d(0.0, 0.0, s.z_floor)).z();
    }
  return std::nullopt;
}

PlanWorkbench::PlanWorkbench(vtkRenderer *renderer, viz::SceneInfo scene,
                             viz::SceneOptions opts, double floor_z,
                             double cut_height, std::string source)
    : renderer_(renderer), scene_(std::move(scene)), opts_(std::move(opts)),
      floor_z_(floor_z), cut_height_(std::max(cut_height, kMinCutM)),
      source_(std::move(source)), layer_visible_(scene_.layers.size(), true) {
  opts_.plan.reset(); // the overlay is ours to manage
  apply_cut();
}

PlanWorkbench::~PlanWorkbench() {
  viz::remove_plan_overlay(renderer_, overlay_);
}

void PlanWorkbench::show_plan(const SolidModel &plan, int generation) {
  plan_ = plan;
  generation_ = generation;
  error_.clear();
  // A storey the edited plan no longer has falls back to all storeys.
  if (storey_) {
    const auto ids = plan_storeys(*plan_);
    if (std::find(ids.begin(), ids.end(), *storey_) == ids.end())
      storey_.reset();
  }
  rebuild_overlay();
  apply_cut();
}

void PlanWorkbench::show_error(std::string error) { error_ = std::move(error); }

void PlanWorkbench::cycle_storey(int step) {
  if (!plan_)
    return;
  const auto ids = plan_storeys(*plan_);
  // Positions: 0 = all, 1..n = ids.
  const int n = static_cast<int>(ids.size()) + 1;
  int at = 0;
  if (storey_) {
    const auto it = std::find(ids.begin(), ids.end(), *storey_);
    at = it == ids.end() ? 0 : static_cast<int>(it - ids.begin()) + 1;
  }
  at = ((at + step) % n + n) % n;
  storey_ = at == 0 ? std::nullopt
                    : std::optional<int>(ids[static_cast<std::size_t>(at - 1)]);
  rebuild_overlay();
  apply_cut();
}

void PlanWorkbench::move_cut(double delta_m) {
  cut_height_ = std::max(kMinCutM, cut_height_ + delta_m);
  rebuild_overlay(); // the sheet lies in the cut
  apply_cut();
}

void PlanWorkbench::toggle_cut() {
  cut_ = !cut_;
  rebuild_overlay();
  apply_cut();
}

bool PlanWorkbench::toggle_layer(std::size_t index) {
  if (index >= scene_.layers.size())
    return false;
  layer_visible_[index] = !layer_visible_[index];
  set_visible(scene_.layers[index].actors, layer_visible_[index]);
  return true;
}

void PlanWorkbench::toggle_plan() {
  plan_visible_ = !plan_visible_;
  set_visible(overlay_.actors, plan_visible_);
  set_visible(overlay_.labels, plan_visible_);
}

bool PlanWorkbench::layer_visible(std::size_t index) const {
  return index < layer_visible_.size() && layer_visible_[index];
}

double PlanWorkbench::cut_z() const {
  double floor = floor_z_;
  if (plan_ && storey_)
    floor = storey_floor_world_z(*plan_, *storey_).value_or(floor_z_);
  return floor + cut_height_;
}

void PlanWorkbench::rebuild_overlay() {
  viz::remove_plan_overlay(renderer_, overlay_);
  if (!plan_)
    return;
  // Drawn as a sheet in the cut plane, just under it, so a view from above
  // shows the plan on the section instead of hidden among the floor points;
  // with the cut off, on its storey's floor in 3D.
  if (cut_) {
    opts_.plan_placement = viz::PlanPlacement::above_scene;
    opts_.plan_sheet_z = cut_z() - kSheetBelowCutM;
  } else {
    opts_.plan_placement = viz::PlanPlacement::level;
    opts_.plan_sheet_z.reset();
  }
  overlay_ = viz::add_plan_overlay(
      renderer_, storey_ ? plan_for_storey(*plan_, *storey_) : *plan_, opts_,
      scene_.bounds);
  if (!plan_visible_) {
    set_visible(overlay_.actors, false);
    set_visible(overlay_.labels, false);
  }
}

void PlanWorkbench::apply_cut() {
  // apply_cut_plane() adds a plane; clear first so a moved cut replaces the
  // old one instead of stacking on it.
  viz::clear_cut_planes(renderer_);
  if (cut_)
    viz::apply_cut_plane(renderer_, cut_z());
}

std::string PlanWorkbench::status_line() const {
  std::string plan;
  if (!error_.empty()) {
    std::string e = error_;
    if (e.size() > kStatusErrorChars)
      e = e.substr(0, kStatusErrorChars) + "...";
    plan = plan_ ? fmt::format("{}: ERROR {} -- showing load {}", source_, e,
                               generation_)
                 : fmt::format("{}: ERROR {} -- no good plan yet", source_, e);
  } else if (plan_) {
    plan = fmt::format(
        "{}: load {} ok ({})", source_, generation_,
        plan_counts(storey_ ? plan_for_storey(*plan_, *storey_) : *plan_));
  } else {
    plan = fmt::format("{}: not loaded", source_);
  }
  std::string layers;
  for (std::size_t i = 0; i < scene_.layers.size(); ++i)
    layers += fmt::format("{}{} {} {}", i == 0 ? "" : ", ", i + 1,
                          viz::to_string(scene_.layers[i].layer),
                          layer_visible_[i] ? "on" : "off");
  return fmt::format(
      "{}\nstorey {}  |  cut {}  |  layers: {}  |  plan {}", plan,
      storey_ ? std::to_string(*storey_) : std::string("all"),
      cut_ ? fmt::format("{:.2f} m (z {:.2f})", cut_height_, cut_z())
           : std::string("off"),
      layers, plan_visible_ ? "on" : "off");
}

std::string PlanWorkbench::key_help() {
  return "n / b     next / previous storey (all, then each storey)\n"
         "] / [     raise / lower the plan cut by 10 cm\n"
         "} / {     raise / lower the plan cut by 1 m\n"
         "v         plan cut on / off\n"
         "1-9       toggle layer 1-9 (--layers order)\n"
         "0         toggle the plan overlay\n"
         "r         reset the camera (PCL)\n"
         "q         quit\n";
}

} // namespace rux::view
