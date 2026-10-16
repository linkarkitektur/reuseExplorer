// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// populate_scene() — the scene builder shared by `rux render` and the Qt
// client's 3D workspace (Stream Q, Q3) — and its label palette and LOD.
//
// Nothing here renders: populate_scene() only fills a vtkRenderer, so the
// tests inspect the actors' poly-data directly and need no GL context.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/visualize/scene.hpp>
#include <rux_qt/tokens.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
namespace viz = reusex::visualize;

namespace {

/// A 10-point cloud on a line, coloured by index, with a label cloud whose
/// labels cycle 0..9 (0 unlabeled).
void seed_cloud(ProjectDB &db, std::size_t n, std::string_view order = "") {
  Cloud cloud;
  CloudL labels;
  for (std::size_t i = 0; i < n; ++i) {
    PointT p;
    p.x = static_cast<float>(i);
    p.y = 0.5F;
    p.z = 0.25F * static_cast<float>(i % 3);
    p.r = static_cast<std::uint8_t>(i);
    p.g = 10;
    p.b = 20;
    cloud.push_back(p);
    LabelT l;
    l.label = static_cast<std::uint32_t>(i % 10);
    labels.push_back(l);
  }
  const std::string params =
      order.empty() ? std::string()
                    : "{\"storage_order\":\"" + std::string(order) + "\"}";
  db.save_point_cloud("cloud", cloud, "test", params);
  db.save_point_cloud("planes", labels, "test", params);
}

vtkUnsignedCharArray *colours(vtkActor *actor) {
  auto *poly = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  REQUIRE(poly != nullptr);
  return vtkUnsignedCharArray::SafeDownCast(poly->GetPointData()->GetScalars());
}

vtkIdType point_count(vtkActor *actor) {
  auto *poly = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  REQUIRE(poly != nullptr);
  return poly->GetNumberOfPoints();
}

std::string read_file(const std::string &path) {
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

} // namespace

TEST_CASE("LabelPalette_Default_MatchesTheDesignTokens",
          "[visualize][scene][palette]") {
  // The library cannot read tokens.css at run time; this pins its copy of the
  // --label-* scale to the file, so a design sync that changes the scale
  // fails here until the table follows (and render, web and Qt keep agreeing).
  const std::string css = read_file(std::string(REUSEX_SOURCE_DIR) +
                                    "/apps/rux/qt/theme/tokens.css");
  REQUIRE_FALSE(css.empty());
  for (const auto mode :
       {rux::qt::ThemeMode::light, rux::qt::ThemeMode::dark}) {
    const auto tokens = rux::qt::parse_tokens_css(css, mode);
    const auto &palette = viz::default_label_palette();
    REQUIRE(tokens.count("--label-count") == 1);
    REQUIRE(std::stoul(tokens.at("--label-count")) == palette.colors.size());
    for (std::size_t i = 0; i < palette.colors.size(); ++i) {
      const auto c =
          rux::qt::parse_color(tokens.at("--label-" + std::to_string(i)));
      REQUIRE(c);
      INFO("--label-" << i);
      CHECK(c->r == palette.colors[i][0]);
      CHECK(c->g == palette.colors[i][1]);
      CHECK(c->b == palette.colors[i][2]);
    }
    const auto inv = rux::qt::parse_color(tokens.at("--label-invalid"));
    REQUIRE(inv);
    // Unlike the rest of the scale, --label-invalid is PROVISIONAL and now
    // varies per theme (final review finding 4): dim in dark, muted in
    // light. default_label_palette() is theme-unaware (headless `rux
    // render` has no theme concept) and mirrors the dark value, since that
    // is the one that actually sits near the near-black canvas both themes
    // share — so only the dark iteration pins it here.
    if (mode == rux::qt::ThemeMode::dark) {
      CHECK(inv->r == palette.invalid[0]);
      CHECK(inv->g == palette.invalid[1]);
      CHECK(inv->b == palette.invalid[2]);
    }
    // Not a class colour in either theme: it must not be confused with one.
    for (const auto &c : palette.colors)
      CHECK_FALSE((c[0] == inv->r && c[1] == inv->g && c[2] == inv->b));
    const auto u = rux::qt::parse_color(tokens.at("--label-unlabeled"));
    REQUIRE(u);
    CHECK(u->r == palette.unlabeled[0]);
    CHECK(u->g == palette.unlabeled[1]);
    CHECK(u->b == palette.unlabeled[2]);
  }
}

TEST_CASE("LabelPalette_Slot_IsLabelMinusOneModuloSize",
          "[visualize][scene][palette]") {
  // The web viewport's labelColorIndex(): label 0 is no slot, 1 is slot 0.
  CHECK(viz::label_palette_slot(0, 8) == -1);
  CHECK(viz::label_palette_slot(1, 8) == 0);
  CHECK(viz::label_palette_slot(8, 8) == 7);
  CHECK(viz::label_palette_slot(9, 8) == 0);
  CHECK(viz::label_palette_slot(3, 0) == -1);
  // A -1 that wrapped in a uint32 cloud (STANDARDS §3.1) is not slot 6.
  CHECK(viz::label_palette_slot(0xFFFFFFFFu, 8) == viz::kInvalidSlot);
  CHECK(viz::label_palette_slot(0x80000000u, 8) == viz::kInvalidSlot);
  CHECK(viz::label_palette_slot(0x7FFFFFFFu, 8) >= 0);
  const auto &p = viz::default_label_palette();
  CHECK(viz::label_palette_color(p, 0) == p.unlabeled);
  CHECK(viz::label_palette_color(p, 10) == p.colors[1]);
  CHECK(viz::label_palette_color(p, 0xFFFFFFFFu) == p.invalid);
}

TEST_CASE("LodIndices_PicksAPrefixOnlyForBitReversedMorton",
          "[visualize][scene][lod]") {
  viz::LodMethod m = viz::LodMethod::all;
  CHECK(viz::lod_indices(100, 0, "", &m).empty());
  CHECK(m == viz::LodMethod::all);
  CHECK(viz::lod_indices(100, 100, "", &m).empty());
  CHECK(m == viz::LodMethod::all);

  // Bit-reversed Morton: any prefix is stratified over the whole scene.
  const auto prefix = viz::lod_indices(100, 10, "morton_10bit_bitrev", &m);
  CHECK(m == viz::LodMethod::morton_prefix);
  REQUIRE(prefix.size() == 10);
  CHECK(prefix.front() == 0);
  CHECK(prefix.back() == 9);

  // Plain Morton's prefix is one octant; it gets the stride like any other
  // order, spread over the whole storage range.
  const auto stride = viz::lod_indices(100, 10, "morton_10bit", &m);
  CHECK(m == viz::LodMethod::stride);
  REQUIRE(stride.size() == 10);
  CHECK(stride.front() == 0);
  CHECK(stride[1] == 10);
  CHECK(stride.back() == 90);
}

TEST_CASE("PopulateScene_CloudAndLabels_UseStoredRgbAndTheTokenPalette",
          "[visualize][scene]") {
  TempPath tmp("test_scene_layers");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::planes};
  const viz::SceneInfo info = viz::populate_scene(renderer, db, o);

  REQUIRE(info.layers.size() == 2);
  CHECK(info.drawn_points == 20);
  CHECK(info.source_points == 10);
  CHECK(info.lod == viz::LodMethod::all);
  CHECK(info.indices.empty());
  CHECK(info.bounds.valid);
  CHECK(info.bounds.min[0] == 0.0);
  CHECK(info.bounds.max[0] == 9.0);
  CHECK(renderer->GetActors()->GetNumberOfItems() == 2);

  vtkUnsignedCharArray *rgb = colours(info.layers[0].actors.front());
  CHECK(rgb->GetTypedComponent(7, 0) == 7);
  CHECK(rgb->GetTypedComponent(7, 2) == 20);

  // Label 0 is --label-unlabeled; label 3 is slot 2 (--label-2).
  const auto &pal = viz::default_label_palette();
  vtkUnsignedCharArray *lab = colours(info.layers[1].actors.front());
  CHECK(lab->GetTypedComponent(0, 0) == pal.unlabeled[0]);
  CHECK(lab->GetTypedComponent(3, 0) == pal.colors[2][0]);
  CHECK(lab->GetTypedComponent(3, 1) == pal.colors[2][1]);
  CHECK(lab->GetTypedComponent(3, 2) == pal.colors[2][2]);
}

TEST_CASE("PopulateScene_WrappedMinusOne_IsDrawnInTheInvalidColour",
          "[visualize][scene][palette]") {
  TempPath tmp("test_scene_invalid");
  ProjectDB db(tmp.path);
  seed_cloud(db, 4);
  CloudL labels;
  for (const std::uint32_t l : {0u, 0xFFFFFFFFu, 7u, 0xFFFFFFFFu}) {
    LabelT x;
    x.label = l;
    labels.push_back(x);
  }
  db.save_point_cloud("rooms", labels, "test");
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::rooms};
  const auto info = viz::populate_scene(renderer, db, o);
  const auto &pal = viz::default_label_palette();
  vtkUnsignedCharArray *c = colours(info.layers[0].actors.front());
  for (int k = 0; k < 3; ++k) {
    CHECK(c->GetTypedComponent(1, k) ==
          pal.invalid[static_cast<std::size_t>(k)]);
    CHECK(c->GetTypedComponent(3, k) ==
          pal.invalid[static_cast<std::size_t>(k)]);
  }
  // ... and label 7 keeps its class colour, slot 6.
  CHECK(c->GetTypedComponent(2, 0) == pal.colors[6][0]);
}

TEST_CASE("PopulateScene_CustomPalette_IsUsed", "[visualize][scene]") {
  TempPath tmp("test_scene_palette");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::planes};
  o.palette = viz::LabelPalette{{{1, 2, 3}, {4, 5, 6}}, {9, 9, 9}};
  const auto info = viz::populate_scene(renderer, db, o);
  vtkUnsignedCharArray *lab = colours(info.layers[0].actors.front());
  CHECK(lab->GetTypedComponent(0, 0) == 9);
  CHECK(lab->GetTypedComponent(1, 0) == 1);
  CHECK(lab->GetTypedComponent(2, 0) == 4);
  CHECK(lab->GetTypedComponent(3, 0) == 1);
}

TEST_CASE("PopulateScene_Budget_DrawsAStrideAndAHiddenCoarseTwin",
          "[visualize][scene][lod]") {
  TempPath tmp("test_scene_lod");
  ProjectDB db(tmp.path);
  seed_cloud(db, 1000);

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::planes};
  o.max_points = 500;
  o.coarse_points = 100;
  const auto info = viz::populate_scene(renderer, db, o);

  CHECK(info.lod == viz::LodMethod::stride);
  REQUIRE(info.indices.size() == 500);
  CHECK(info.indices[1] == 2);
  for (const auto &layer : info.layers) {
    CHECK(layer.points == 500);
    REQUIRE(layer.coarse != nullptr);
    CHECK_FALSE(layer.coarse->GetVisibility());
    CHECK(point_count(layer.actors.front()) == 500);
    CHECK(point_count(layer.coarse) == 100);
  }
  // A drawn point keeps its stored colour: vtk id 3 is cloud index 6.
  CHECK(colours(info.layers[0].actors.front())->GetTypedComponent(3, 0) == 6);
  // The label layer is sampled at the same points (index-aligned).
  const auto &pal = viz::default_label_palette();
  CHECK(colours(info.layers[1].actors.front())->GetTypedComponent(3, 0) ==
        viz::label_palette_color(pal, 6)[0]);
  // The bounds still cover the whole cloud, not just the drawn points.
  CHECK(info.bounds.max[0] == 999.0);
}

TEST_CASE("PopulateScene_BitReversedMortonCloud_DrawsAPrefix",
          "[visualize][scene][lod]") {
  TempPath tmp("test_scene_morton");
  ProjectDB db(tmp.path);
  seed_cloud(db, 1000, "morton_10bit_bitrev");
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.max_points = 250;
  const auto info = viz::populate_scene(renderer, db, o);
  CHECK(info.lod == viz::LodMethod::morton_prefix);
  REQUIRE(info.indices.size() == 250);
  CHECK(info.indices.back() == 249);
  CHECK(info.layers[0].coarse == nullptr);
}

TEST_CASE("PopulateScene_Frustums_OnePerPosedFrameWithinTheBudget",
          "[visualize][scene]") {
  TempPath tmp("test_scene_frustums");
  ProjectDB db(tmp.path);
  for (int id = 1; id <= 6; ++id)
    db.save_sensor_frame(id, make_color(32, 24), cv::Mat(), cv::Mat(),
                         translation_pose(id, 0.0, 1.0),
                         make_intrinsics(16.0, 16.0, 16.0, 12.0, 32, 24));
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::frustums};
  o.max_frustums = 3;
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 1);
  CHECK(info.layers[0].items == 3);
  // Eye + 4 corners per frustum.
  CHECK(point_count(info.layers[0].actors.front()) == 15);
  CHECK(info.bounds.valid);
}

TEST_CASE("PopulateScene_MissingLayerData_NamesTheStageToRun",
          "[visualize][scene]") {
  TempPath tmp("test_scene_missing");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;

  o.layers = {viz::Layer::frustums};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("posed sensor frames"));
  o.layers = {viz::Layer::panoramas};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux import 360"));
  o.layers = {viz::Layer::rooms};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux create rooms"));
}

TEST_CASE("PlacePresetCamera_Yaw_TurnsTheViewAboutZ",
          "[visualize][scene][camera]") {
  using namespace reusex::visualize;
  SceneBounds b;
  b.add(0, 0, 0);
  b.add(10, 2, 3); // long in x
  auto up_for = [&](ViewPreset view, double yaw) {
    vtkNew<vtkRenderer> r;
    CameraFraming f;
    f.view = view;
    f.yaw_deg = yaw;
    place_preset_camera(r, f, b);
    double up[3];
    r->GetActiveCamera()->GetViewUp(up);
    return std::array<double, 3>{up[0], up[1], up[2]};
  };
  const auto up0 = up_for(ViewPreset::top, 0.0);
  CHECK(up0[1] == Catch::Approx(1.0));
  // +90°: the model turns counter-clockwise, so world +x points up the page.
  const auto up90 = up_for(ViewPreset::top, 90.0);
  CHECK(up90[0] == Catch::Approx(1.0));
  CHECK(up90[1] == Catch::Approx(0.0).margin(1e-9));

  vtkNew<vtkRenderer> r;
  CameraFraming f;
  f.view = ViewPreset::top;
  f.aspect = 1.0;
  const double s0 = [&] {
    place_preset_camera(r, f, b);
    return r->GetActiveCamera()->GetParallelScale();
  }();
  f.yaw_deg = 90.0;
  place_preset_camera(r, f, b);
  // The 10 m side now runs up the page: a square viewport frames the same
  // extent, so the scale is unchanged; with a wide aspect it would grow.
  CHECK(r->GetActiveCamera()->GetParallelScale() == Catch::Approx(s0));
}

// ── Plan overlay (reusex-z1v.2.6): a solid-model plan drawn over the scene ───

namespace {

/// One storey, one 4 m wall along +x (0.2 m thick) with a door in it, and one
/// 4 x 3 m space north of it.
core::SolidModel overlay_plan() {
  core::SolidModel m;
  core::StoreyRecord s;
  s.idx = 0;
  s.guid = "storey-0";
  m.storeys.push_back(s);
  core::WallRecord w;
  w.guid = "w1";
  w.x0 = 0.0, w.y0 = 0.0, w.x1 = 4.0, w.y1 = 0.0;
  w.height = 2.5;
  w.thickness = 0.2;
  m.walls.push_back(w);
  core::SpaceRecord sp;
  sp.guid = "s1";
  sp.name = "Office";
  sp.polygon.outer = {{0.1, 0.1}, {4.0, 0.1}, {4.0, 3.0}, {0.1, 3.0}};
  m.spaces.push_back(sp);
  core::OpeningRecord o;
  o.guid = "o1";
  o.wall = "w1";
  o.kind = "door";
  o.s_start = 1.0, o.s_end = 2.0, o.z_sill = 0.0, o.z_head = 2.1;
  m.openings.push_back(o);
  return m;
}

/// Every point of every overlay actor, as {x, y, z}.
std::vector<std::array<double, 3>> overlay_points(const viz::SceneInfo &info) {
  std::vector<std::array<double, 3>> out;
  for (vtkActor *a : info.plan.actors) {
    auto *poly = vtkPolyData::SafeDownCast(a->GetMapper()->GetInput());
    REQUIRE(poly != nullptr);
    for (vtkIdType i = 0; i < poly->GetNumberOfPoints(); ++i) {
      double p[3];
      poly->GetPoint(i, p);
      out.push_back({p[0], p[1], p[2]});
    }
  }
  return out;
}

} // namespace

TEST_CASE("PopulateScene_PlanOverlay_DrawsWallsSpacesAndOpenings",
          "[visualize][scene][plan]") {
  TempPath tmp("test_scene_plan");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud};
  o.plan = overlay_plan();
  const viz::SceneInfo info = viz::populate_scene(renderer, db, o);

  CHECK(info.plan.walls == 1);
  CHECK(info.plan.spaces == 1);
  CHECK(info.plan.openings == 1);
  REQUIRE_FALSE(info.plan.actors.empty());
  // The space's name and the wall's guid are labelled.
  CHECK(info.plan.labels.size() == 2);

  // The wall's footprint is its axis +/- half the thickness; the door's end
  // ticks reach 0.1 m past each face.
  double ymin = 1e9, ymax = -1e9;
  bool face = false;
  for (const auto &p : overlay_points(info)) {
    ymin = std::min(ymin, p[1]);
    ymax = std::max(ymax, p[1]);
    if (std::abs(p[1] + 0.1) < 1e-9 && std::abs(p[0] - 4.0) < 1e-9)
      face = true; // footprint corner (4, -0.1)
  }
  CHECK(face);
  CHECK(ymin == Catch::Approx(-0.2));
  CHECK(ymax == Catch::Approx(3.0));
  // The overlay widens the framing: the cloud spans y = 0.5 only.
  CHECK(info.bounds.max[1] == Catch::Approx(3.0));
  CHECK(info.bounds.min[1] == Catch::Approx(-0.2));

  // Overlay colours: walls on --label-2.
  const auto &pal = viz::default_label_palette();
  bool wall_colour = false;
  for (vtkActor *a : info.plan.actors) {
    double rgb[3];
    a->GetProperty()->GetColor(rgb);
    if (std::abs(rgb[0] * 255.0 - pal.colors[2][0]) < 0.5 &&
        std::abs(rgb[1] * 255.0 - pal.colors[2][1]) < 0.5 &&
        std::abs(rgb[2] * 255.0 - pal.colors[2][2]) < 0.5)
      wall_colour = true;
  }
  CHECK(wall_colour);
}

TEST_CASE("PopulateScene_PlanOverlay_AppliesTheInverseLevelTransform",
          "[visualize][scene][plan]") {
  TempPath tmp("test_scene_plan_level");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);

  core::SolidModel plan = overlay_plan();
  // world -> level: shift by (-10, -20, +1.5), i.e. the level origin sits at
  // world (10, 20, -1.5).
  plan.storeys[0].level_transform = {1, 0, 0, -10, 0, 1, 0, -20,
                                     0, 0, 1, 1.5, 0, 0, 0, 1};
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud};
  o.plan = plan;
  o.plan_placement = viz::PlanPlacement::level;
  const auto info = viz::populate_scene(renderer, db, o);

  double xmin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
  for (const auto &p : overlay_points(info)) {
    xmin = std::min(xmin, p[0]);
    ymax = std::max(ymax, p[1]);
    zmin = std::min(zmin, p[2]);
    zmax = std::max(zmax, p[2]);
  }
  CHECK(xmin == Catch::Approx(10.0).margin(0.06));
  CHECK(ymax == Catch::Approx(23.0));
  // Level placement draws on the storey floor (world z = -1.5), lifted a
  // hair so it does not z-fight the floor points.
  CHECK(zmin == Catch::Approx(-1.5).margin(0.05));
  CHECK(zmax == Catch::Approx(-1.5).margin(0.05));
}

TEST_CASE("PopulateScene_PlanOverlay_AboveScene_SitsOnTopAndSurvivesTheCut",
          "[visualize][scene][plan]") {
  TempPath tmp("test_scene_plan_top");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10); // z in [0, 0.5]

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud};
  o.plan = overlay_plan();
  o.plan_placement = viz::PlanPlacement::above_scene;
  const auto info = viz::populate_scene(renderer, db, o);

  for (const auto &p : overlay_points(info))
    CHECK(p[2] >= info.bounds.max[2]);
  // The overlay never moves the vertical extent the cut is measured in.
  CHECK(info.bounds.max[2] == Catch::Approx(0.5));

  viz::apply_cut_plane(renderer, 0.2);
  for (vtkActor *a : info.plan.actors)
    CHECK(a->GetMapper()->GetNumberOfClippingPlanes() == 0);
  for (const auto &layer : info.layers)
    for (vtkActor *a : layer.actors)
      CHECK(a->GetMapper()->GetNumberOfClippingPlanes() == 1);
}

TEST_CASE("PopulateScene_PlanOverlay_OpeningOnAMissingWall_IsSkippedLoudly",
          "[visualize][scene][plan]") {
  TempPath tmp("test_scene_plan_orphan");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  core::SolidModel plan = overlay_plan();
  plan.openings[0].wall = "nope";
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.plan = plan;
  const auto info = viz::populate_scene(renderer, db, o);
  CHECK(info.plan.openings == 0);
  CHECK(info.plan.walls == 1);
}

// ── Walls layer (reusex-z1v.5.6) ─────────────────────────────────────────────

namespace {

/// Two walls on storey 0, whose level origin sits at world (10, 20, -1.5):
/// a measured one (thickness 0.2, seen 0.9) and an assumed one.
void seed_walls(ProjectDB &db) {
  core::StoreyRecord s;
  s.guid = "storey-0";
  s.z_ceiling = 2.5;
  s.level_transform = {1, 0, 0, -10, 0, 1, 0, -20, 0, 0, 1, 1.5, 0, 0, 0, 1};
  db.save_storeys({s});
  core::WallRecord a;
  a.guid = "wall-a";
  a.x0 = 0.0, a.y0 = 0.0, a.x1 = 4.0, a.y1 = 0.0;
  a.height = 2.5;
  a.thickness = 0.2;
  a.seen_share = 0.9;
  core::WallRecord b = a;
  b.guid = "wall-b";
  b.x0 = 4.0, b.y0 = 0.0, b.x1 = 4.0, b.y1 = 3.0;
  b.thickness = 0.2;
  b.assumed = true;
  db.save_walls({a, b}, {{"wall-a", "wall-b", "corner", "end", "start"}});
}

std::vector<std::array<double, 3>> layer_points(const viz::SceneLayer &l) {
  std::vector<std::array<double, 3>> out;
  for (vtkActor *a : l.actors) {
    auto *poly = vtkPolyData::SafeDownCast(a->GetMapper()->GetInput());
    REQUIRE(poly != nullptr);
    for (vtkIdType i = 0; i < poly->GetNumberOfPoints(); ++i) {
      double p[3];
      poly->GetPoint(i, p);
      out.push_back({p[0], p[1], p[2]});
    }
  }
  return out;
}

} // namespace

TEST_CASE("PopulateScene_Walls_NoWalls_NamesTheStageToRun",
          "[visualize][scene][walls]") {
  TempPath tmp("test_scene_walls_missing");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::walls};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux create walls"));
  CHECK(viz::layer_from_string("walls") == viz::Layer::walls);
  CHECK(viz::to_string(viz::Layer::walls) == "walls");
}

TEST_CASE("PopulateScene_Walls_Level_AreBoxesInWorld",
          "[visualize][scene][walls]") {
  TempPath tmp("test_scene_walls_level");
  ProjectDB db(tmp.path);
  seed_walls(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::walls};
  o.plan_placement = viz::PlanPlacement::level;
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 1);
  const auto &l = info.layers[0];
  CHECK(l.layer == viz::Layer::walls);
  CHECK(l.items == 2);
  CHECK(l.dim == 1); // the assumed one
  REQUIRE_FALSE(l.actors.empty());

  double xmin = 1e9, xmax = -1e9, ymin = 1e9, zmin = 1e9, zmax = -1e9;
  for (const auto &p : layer_points(l)) {
    xmin = std::min(xmin, p[0]);
    xmax = std::max(xmax, p[0]);
    ymin = std::min(ymin, p[1]);
    zmin = std::min(zmin, p[2]);
    zmax = std::max(zmax, p[2]);
  }
  // Level -> world through the storey frame; footprint = axis +/- 0.1 m.
  CHECK(xmin == Catch::Approx(10.0).margin(1e-9));
  CHECK(xmax == Catch::Approx(14.1).margin(1e-9));
  CHECK(ymin == Catch::Approx(19.9).margin(1e-9));
  // From the storey floor (world z = -1.5, lifted a hair) to its height.
  CHECK(zmin == Catch::Approx(-1.5).margin(0.05));
  CHECK(zmax == Catch::Approx(1.0).margin(0.05));

  // Measured walls in walls_rgb, the dim one in a paler shade of it.
  bool full = false, paler = false;
  for (vtkActor *a : l.actors) {
    CHECK(std::string(a->GetObjectName()) == viz::kWallsActorName);
    double rgb[3];
    a->GetProperty()->GetColor(rgb);
    if (std::abs(rgb[2] * 255.0 - o.walls_rgb[2]) < 0.5)
      full = true;
    else
      paler = true;
  }
  CHECK(full);
  CHECK(paler);

  // In 3D the walls are cut with the scene.
  viz::apply_cut_plane(renderer, 0.0);
  for (vtkActor *a : l.actors)
    CHECK(a->GetMapper()->GetNumberOfClippingPlanes() == 1);
}

TEST_CASE("PopulateScene_Walls_AboveScene_AreFootprintsOverTheCut",
          "[visualize][scene][walls]") {
  TempPath tmp("test_scene_walls_plan");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10); // z in [0, 0.5]
  seed_walls(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  // walls listed first: still drawn last, on a sheet over the cloud
  o.layers = {viz::Layer::walls, viz::Layer::cloud};
  o.plan_placement = viz::PlanPlacement::above_scene;
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 2);
  const auto &l = info.layers[0];
  CHECK(l.layer == viz::Layer::walls);
  CHECK(l.items == 2);
  const auto pts = layer_points(l);
  REQUIRE_FALSE(pts.empty());
  for (const auto &p : pts)
    CHECK(p[2] >= 0.5);
  viz::apply_cut_plane(renderer, 0.2);
  for (vtkActor *a : l.actors)
    CHECK(a->GetMapper()->GetNumberOfClippingPlanes() == 0);
}

// ── Openings layer (reusex-z1v.7.3) ──────────────────────────────────────────

namespace {

/// The two walls of seed_walls() plus a door in wall-a (s 1 .. 2, floor to
/// 2.1 m) and a window in wall-b (s 1 .. 2, 0.9 .. 2.0 m).
void seed_openings(ProjectDB &db) {
  seed_walls(db);
  core::OpeningRecord door;
  door.guid = "door-1";
  door.wall = "wall-a";
  door.kind = "door";
  door.s_start = 1.0;
  door.s_end = 2.0;
  door.z_sill = 0.0;
  door.z_head = 2.1;
  core::OpeningRecord win = door;
  win.guid = "win-1";
  win.wall = "wall-b";
  win.kind = "window";
  win.z_sill = 0.9;
  win.z_head = 2.0;
  db.save_openings({door, win});
}

} // namespace

TEST_CASE("PopulateScene_Openings_NoOpenings_NamesTheStageToRun",
          "[visualize][scene][openings]") {
  TempPath tmp("test_scene_openings_missing");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::openings};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux create openings"));
  CHECK(viz::layer_from_string("openings") == viz::Layer::openings);
  CHECK(viz::to_string(viz::Layer::openings) == "openings");
}

TEST_CASE("PopulateScene_Openings_Level_AreRectanglesInTheWallFaces",
          "[visualize][scene][openings]") {
  TempPath tmp("test_scene_openings_level");
  ProjectDB db(tmp.path);
  seed_openings(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::openings};
  o.plan_placement = viz::PlanPlacement::level;
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 1);
  const auto &l = info.layers[0];
  CHECK(l.layer == viz::Layer::openings);
  CHECK(l.items == 2);
  REQUIRE_FALSE(l.actors.empty());
  // The door: in wall-a (world y = 20 +/- 0.11), x 11 .. 12, z -1.5 .. 0.6.
  bool door = false, window = false;
  for (const auto &p : layer_points(l)) {
    if (std::abs(p[1] - 20.0) < 0.2) {
      door = true;
      CHECK(std::abs(std::abs(p[1] - 20.0) - 0.11) < 1e-6);
      CHECK(p[0] >= 11.0 - 1e-9);
      CHECK(p[0] <= 12.0 + 1e-9);
      CHECK(p[2] >= -1.5 - 1e-9);
      CHECK(p[2] <= 0.6 + 0.05);
    } else {
      window = true; // wall-b: x = 14 +/- 0.11, y 21 .. 22, z -0.6 .. 0.5
      CHECK(std::abs(std::abs(p[0] - 14.0) - 0.11) < 1e-6);
      CHECK(p[1] >= 21.0 - 1e-9);
      CHECK(p[1] <= 22.0 + 1e-9);
      CHECK(p[2] >= -0.6 - 1e-9);
      CHECK(p[2] <= 0.5 + 0.05);
    }
  }
  CHECK(door);
  CHECK(window);
  // Coloured by kind.
  bool door_rgb = false, window_rgb = false;
  for (vtkActor *a : l.actors) {
    CHECK(std::string(a->GetObjectName()) == viz::kOpeningsActorName);
    double rgb[3];
    a->GetProperty()->GetColor(rgb);
    door_rgb |= std::abs(rgb[0] * 255.0 - o.plan_door_rgb[0]) < 0.5 &&
                std::abs(rgb[1] * 255.0 - o.plan_door_rgb[1]) < 0.5;
    window_rgb |= std::abs(rgb[2] * 255.0 - o.plan_window_rgb[2]) < 0.5 &&
                  std::abs(rgb[0] * 255.0 - o.plan_window_rgb[0]) < 0.5;
  }
  CHECK(door_rgb);
  CHECK(window_rgb);
}

TEST_CASE("PopulateScene_Openings_AboveScene_AreBarsOverTheCut",
          "[visualize][scene][openings]") {
  TempPath tmp("test_scene_openings_plan");
  ProjectDB db(tmp.path);
  seed_cloud(db, 10);
  seed_openings(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::openings, viz::Layer::cloud, viz::Layer::walls};
  o.plan_placement = viz::PlanPlacement::above_scene;
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 3);
  const auto &l = info.layers[0];
  CHECK(l.layer == viz::Layer::openings);
  CHECK(l.items == 2);
  for (const auto &p : layer_points(l))
    CHECK(p[2] >= 0.5);
  viz::apply_cut_plane(renderer, 0.2);
  for (vtkActor *a : l.actors)
    CHECK(a->GetMapper()->GetNumberOfClippingPlanes() == 0);
}
