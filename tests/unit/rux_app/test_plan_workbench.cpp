// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux view --plan --watch`'s reload logic (reusex-yf4.2), headless: the
// workbench only needs a vtkRenderer, so no window or GL context is opened.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <view/plan_workbench.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/visualize/scene.hpp>

#include "../../support/temp_path.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPlaneCollection.h>
#include <vtkRenderer.h>

#include <cstring>
#include <string>

using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using reusex::core::SolidModel;
using rux::view::PlanWorkbench;
namespace viz = reusex::visualize;

namespace {

reusex::core::WallRecord wall(const std::string &guid, int storey, double y) {
  reusex::core::WallRecord w;
  w.guid = guid;
  w.storey = storey;
  w.x0 = 0.0;
  w.y0 = y;
  w.x1 = 4.0;
  w.y1 = y;
  w.height = 2.5;
  w.thickness = 0.2;
  return w;
}

/// Two storeys: 0 with its floor at world z 0, 1 with its floor at world z 3
/// (level frame = world shifted down by 3 m).
SolidModel two_storey_plan() {
  SolidModel plan;
  reusex::core::StoreyRecord s0, s1;
  s0.idx = 0;
  s1.idx = 1;
  s1.level_transform[11] = -3.0;
  plan.storeys = {s0, s1};
  plan.walls = {wall("ground", 0, 0.0), wall("upper", 1, 1.0)};
  return plan;
}

std::size_t overlay_actors_in(vtkRenderer *renderer) {
  std::size_t n = 0;
  vtkActorCollection *actors = renderer->GetActors();
  actors->InitTraversal();
  while (vtkActor *a = actors->GetNextActor())
    if (a->GetObjectName() == viz::kPlanOverlayActorName)
      ++n;
  return n;
}

int clipping_planes(vtkActor *actor) {
  auto *planes = actor->GetMapper()->GetClippingPlanes();
  return planes == nullptr ? 0 : planes->GetNumberOfItems();
}

struct Fixture {
  TempPath project{"plan_workbench"};
  vtkNew<vtkRenderer> renderer;
  viz::SceneInfo scene;
  viz::SceneOptions opts;

  Fixture() {
    reusex::Cloud cloud;
    for (int i = 0; i < 50; ++i) {
      pcl::PointXYZRGB p;
      p.x = 0.1F * static_cast<float>(i);
      p.y = 0.0F;
      p.z = 0.12F * static_cast<float>(i % 50);
      cloud.push_back(p);
    }
    reusex::ProjectDB db(project.path);
    db.save_point_cloud("cloud", cloud);
    opts.layers = {viz::Layer::cloud};
    scene = viz::populate_scene(renderer, db, opts);
  }

  PlanWorkbench bench(double cut = 1.2) {
    return PlanWorkbench(renderer, scene, opts, /*floor_z=*/0.0, cut,
                         "gt.yaml");
  }
};

} // namespace

TEST_CASE("PlanWorkbench swaps the overlay on reload, never stacks it",
          "[rux_app][view][watch]") {
  Fixture f;
  PlanWorkbench b(f.renderer, f.scene, f.opts, 0.0, 1.2, "gt.yaml");
  CHECK(overlay_actors_in(f.renderer) == 0);
  CHECK_THAT(b.status_line(), ContainsSubstring("gt.yaml: not loaded"));

  SolidModel plan = two_storey_plan();
  b.show_plan(plan, 1);
  CHECK(b.overlay().walls == 2);
  const std::size_t drawn = overlay_actors_in(f.renderer);
  CHECK(drawn == b.overlay().actors.size() + b.overlay().labels.size());
  CHECK_THAT(b.status_line(),
             ContainsSubstring("load 1 ok (2 storeys, 2 walls"));

  plan.walls.pop_back();
  plan.walls[0].y1 = 2.0; // the saved edit
  b.show_plan(plan, 2);
  CHECK(b.overlay().walls == 1);
  CHECK(overlay_actors_in(f.renderer) ==
        b.overlay().actors.size() + b.overlay().labels.size());
  CHECK_THAT(b.status_line(), ContainsSubstring("load 2 ok"));
}

TEST_CASE("PlanWorkbench keeps the last good overlay on a bad save",
          "[rux_app][view][watch]") {
  Fixture f;
  auto b = f.bench();
  b.show_plan(two_storey_plan(), 1);
  const std::size_t drawn = overlay_actors_in(f.renderer);
  b.show_error("walls[1] (guid 'upper'): axis has zero length");
  CHECK(overlay_actors_in(f.renderer) == drawn);
  CHECK(b.overlay().walls == 2);
  CHECK_THAT(b.status_line(),
             ContainsSubstring("ERROR walls[1] (guid 'upper')"));
  CHECK_THAT(b.status_line(), ContainsSubstring("showing load 1"));

  b.show_plan(two_storey_plan(), 2); // fixed
  CHECK(b.error().empty());
  CHECK_THAT(b.status_line(), ContainsSubstring("load 2 ok"));
}

TEST_CASE("PlanWorkbench steps through storeys and cuts at each one's floor",
          "[rux_app][view][watch]") {
  Fixture f;
  auto b = f.bench(1.2);
  b.show_plan(two_storey_plan(), 1);
  CHECK_FALSE(b.storey());
  CHECK_THAT(b.cut_z(), WithinAbs(1.2, 1e-9));

  b.cycle_storey(+1);
  REQUIRE(b.storey());
  CHECK(*b.storey() == 0);
  CHECK(b.overlay().walls == 1);
  CHECK_THAT(b.cut_z(), WithinAbs(1.2, 1e-9));

  b.cycle_storey(+1);
  CHECK(*b.storey() == 1);
  CHECK(b.overlay().walls == 1);
  CHECK_THAT(b.cut_z(), WithinAbs(4.2, 1e-9)); // floor at world z 3
  CHECK_THAT(b.status_line(), ContainsSubstring("storey 1"));

  b.cycle_storey(+1); // wraps to all
  CHECK_FALSE(b.storey());
  b.cycle_storey(-1); // and back to the last
  CHECK(*b.storey() == 1);

  // A reload without that storey falls back to all storeys.
  SolidModel one = two_storey_plan();
  one.storeys.pop_back();
  one.walls.pop_back();
  b.show_plan(one, 2);
  CHECK_FALSE(b.storey());
}

TEST_CASE("PlanWorkbench moves the cut without stacking clipping planes",
          "[rux_app][view][watch]") {
  Fixture f;
  auto b = f.bench(1.2);
  b.show_plan(two_storey_plan(), 1);
  vtkActor *cloud = f.scene.layers.at(0).actors.at(0);
  CHECK(clipping_planes(cloud) == 1);
  b.move_cut(0.1);
  b.move_cut(0.1);
  CHECK(clipping_planes(cloud) == 1);
  CHECK_THAT(b.cut_height(), WithinAbs(1.4, 1e-9));
  b.move_cut(-10.0);
  CHECK_THAT(b.cut_height(), WithinAbs(0.05, 1e-9)); // floored
  // The overlay is a drawing over the section: never cut, and drawn as a
  // sheet just under the cut plane, following it.
  for (vtkActor *a : b.overlay().actors) {
    CHECK(clipping_planes(a) == 0);
    CHECK_THAT(a->GetBounds()[4], WithinAbs(b.cut_z() - 0.01, 1e-6));
  }
  b.toggle_cut();
  CHECK_FALSE(b.cut_enabled());
  CHECK(clipping_planes(cloud) == 0);
  CHECK_THAT(b.status_line(), ContainsSubstring("cut off"));
}

TEST_CASE("PlanWorkbench toggles layers and the overlay",
          "[rux_app][view][watch]") {
  Fixture f;
  auto b = f.bench();
  b.show_plan(two_storey_plan(), 1);
  vtkActor *cloud = f.scene.layers.at(0).actors.at(0);
  REQUIRE(b.toggle_layer(0));
  CHECK(cloud->GetVisibility() == 0);
  CHECK_THAT(b.status_line(), ContainsSubstring("1 cloud off"));
  CHECK_FALSE(b.toggle_layer(5));

  b.toggle_plan();
  for (vtkActor *a : b.overlay().actors)
    CHECK(a->GetVisibility() == 0);
  b.show_plan(two_storey_plan(), 2); // a hidden overlay stays hidden
  for (vtkActor *a : b.overlay().actors)
    CHECK(a->GetVisibility() == 0);
  CHECK_THAT(b.status_line(), ContainsSubstring("plan off"));
}

TEST_CASE("plan_for_storey keeps the storey's elements and their openings",
          "[rux_app][view][watch]") {
  SolidModel plan = two_storey_plan();
  reusex::core::OpeningRecord o;
  o.guid = "d";
  o.wall = "upper";
  o.s_end = 1.0;
  plan.openings = {o};
  const auto up = rux::view::plan_for_storey(plan, 1);
  REQUIRE(up.walls.size() == 1);
  CHECK(up.walls[0].guid == "upper");
  CHECK(up.openings.size() == 1);
  CHECK(up.storeys.size() == 1);
  CHECK(rux::view::plan_for_storey(plan, 0).openings.empty());
  CHECK(rux::view::plan_storeys(plan) == std::vector<int>{0, 1});
}
