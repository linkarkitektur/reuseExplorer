// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Mesh-vs-cloud QA in the shared scene builder (reusex-8vx): the mesh
// overlay style, vertical sections, the elevation cameras, the mesh section
// line and the deviation layer. Nothing here renders (see test_scene.cpp).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/visualize/qa_sheet.hpp>
#include <reusex/visualize/scene.hpp>

#include "../../support/temp_path.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
namespace viz = reusex::visualize;

namespace {

/// An axis-aligned 4 x 3 x 2.5 m closed box (floor at z = 0), 12 triangles,
/// welded vertices.
pcl::PolygonMesh box_mesh() {
  pcl::PointCloud<pcl::PointXYZ> v;
  for (const float z : {0.0F, 2.5F})
    for (const auto &[x, y] : {std::pair{0.0F, 0.0F}, std::pair{4.0F, 0.0F},
                               std::pair{4.0F, 3.0F}, std::pair{0.0F, 3.0F}})
      v.push_back({x, y, z});
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(v, mesh.cloud);
  const std::uint32_t quads[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                     {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
  for (const auto &q : quads) {
    pcl::Vertices a, b;
    a.vertices = {q[0], q[1], q[2]};
    b.vertices = {q[0], q[2], q[3]};
    mesh.polygons.push_back(a);
    mesh.polygons.push_back(b);
  }
  return mesh;
}

PointT at(float x, float y, float z) {
  PointT p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.r = p.g = p.b = 128;
  return p;
}

/// Ten points at known distances from the box's floor (z = 0, well inside
/// its footprint): five at 1 cm, three at 10 cm and two 50 cm below it. The
/// first eight lie on a plane (planes label 1), the last two are clutter.
void seed_qa_project(ProjectDB &db) {
  Cloud cloud;
  CloudL planes;
  const float dz[10] = {0.01F, 0.01F, 0.01F, 0.01F,  0.01F,
                        0.10F, 0.10F, 0.10F, -0.50F, -0.50F};
  for (int i = 0; i < 10; ++i) {
    // x in [1.6, 2.5], y = 1.5: farther from every wall than from the floor
    // (and from the ceiling, 2.4 m up).
    cloud.push_back(at(1.6F + 0.1F * static_cast<float>(i), 1.5F, dz[i]));
    LabelT l;
    l.label = i < 8 ? 1U : 0U;
    planes.push_back(l);
  }
  db.save_point_cloud("cloud", cloud, "test", "");
  db.save_point_cloud("planes", planes, "test", "");
  db.save_mesh("mesh", box_mesh(), "test", "");
}

vtkUnsignedCharArray *colours(vtkActor *actor) {
  auto *poly = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  REQUIRE(poly != nullptr);
  return vtkUnsignedCharArray::SafeDownCast(poly->GetPointData()->GetScalars());
}

/// True when @p p survives every clipping plane of @p actor.
bool kept(vtkActor *actor, const std::array<double, 3> &p) {
  vtkPlaneCollection *planes = actor->GetMapper()->GetClippingPlanes();
  if (planes == nullptr)
    return true;
  for (int i = 0; i < planes->GetNumberOfItems(); ++i) {
    double x[3] = {p[0], p[1], p[2]};
    if (planes->GetItem(i)->EvaluateFunction(x) < 0.0)
      return false;
  }
  return true;
}

} // namespace

TEST_CASE("PopulateScene_MeshWithCloud_AutoStyleIsATranslucentOverlay",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_overlay");
  ProjectDB db(tmp.path);
  seed_qa_project(db);

  SECTION("mesh alone stays a solid surface") {
    vtkNew<vtkRenderer> renderer;
    viz::SceneOptions o;
    o.layers = {viz::Layer::mesh};
    const auto info = viz::populate_scene(renderer, db, o);
    CHECK(info.mesh_style == viz::MeshStyle::solid);
    REQUIRE(info.layers.size() == 1);
    REQUIRE(info.layers[0].actors.size() == 1);
    CHECK(info.layers[0].actors[0]->GetProperty()->GetOpacity() == 1.0);
    CHECK(info.layers[0].mesh_surface != nullptr);
  }
  SECTION("cloud + mesh: translucent surface plus feature edges") {
    vtkNew<vtkRenderer> renderer;
    viz::SceneOptions o;
    o.layers = {viz::Layer::cloud, viz::Layer::mesh};
    const auto info = viz::populate_scene(renderer, db, o);
    CHECK(info.mesh_style == viz::MeshStyle::overlay);
    REQUIRE(info.layers.size() == 2);
    const auto &mesh = info.layers[1];
    REQUIRE(mesh.actors.size() == 2);
    CHECK(mesh.actors[0]->GetProperty()->GetOpacity() < 1.0);
    // The box's 12 edges are creases (90 deg); the triangulation diagonals
    // (coplanar) are not feature edges.
    auto *edges =
        vtkPolyData::SafeDownCast(mesh.actors[1]->GetMapper()->GetInput());
    REQUIRE(edges != nullptr);
    CHECK(edges->GetNumberOfLines() == 12);
  }
  SECTION("edges style hides the surface but keeps it as section source") {
    vtkNew<vtkRenderer> renderer;
    viz::SceneOptions o;
    o.layers = {viz::Layer::mesh};
    o.mesh_style = viz::MeshStyle::edges;
    const auto info = viz::populate_scene(renderer, db, o);
    REQUIRE(info.layers[0].actors.size() == 1);
    CHECK(info.layers[0].mesh_surface != nullptr);
    CHECK(renderer->GetActors()->GetNumberOfItems() == 2); // + hidden surface
  }
}

TEST_CASE("PopulateScene_Deviation_ColoursByDistanceAndSummarises",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_deviation");
  ProjectDB db(tmp.path);
  seed_qa_project(db);

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::deviation};
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.deviation.has_value());
  const auto &d = *info.deviation;
  CHECK(d.points == 10);
  CHECK(d.mean_m ==
        Catch::Approx((5 * 0.01 + 3 * 0.10 + 2 * 0.50) / 10.0).epsilon(1e-5));
  CHECK(d.p95_m == Catch::Approx(0.50).epsilon(1e-5)); // nearest rank 10 of 10
  CHECK(d.within_share == Catch::Approx(0.5));         // the five 1 cm points
  CHECK(d.uncovered_share == Catch::Approx(0.2));      // the two 50 cm points

  // Only the eight planar points: the clutter drops out.
  REQUIRE(info.deviation_planar.has_value());
  CHECK(info.deviation_planar->points == 8);
  CHECK(info.deviation_planar->uncovered_share == Catch::Approx(0.0));

  // Colours follow deviation_color().
  auto *rgb = colours(info.layers[0].actors[0]);
  REQUIRE(rgb != nullptr);
  const auto expect = [&](vtkIdType i, double dist) {
    const auto c = viz::deviation_color(dist, o.deviation);
    CHECK(rgb->GetTypedComponent(i, 0) == c[0]);
    CHECK(rgb->GetTypedComponent(i, 1) == c[1]);
    CHECK(rgb->GetTypedComponent(i, 2) == c[2]);
  };
  expect(0, 0.01);
  expect(9, 0.50);
  // Beyond `uncovered` is the --label-6 reddish purple.
  CHECK(viz::deviation_color(0.5, o.deviation) ==
        std::array<std::uint8_t, 3>{0xcc, 0x79, 0xa7});
  CHECK(viz::deviation_color(0.0, o.deviation) ==
        std::array<std::uint8_t, 3>{0x00, 0x9e, 0x73});
}

TEST_CASE("PopulateScene_Deviation_WithoutMesh_NamesTheStage",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_nomesh");
  ProjectDB db(tmp.path);
  Cloud cloud;
  cloud.push_back(at(0, 0, 0));
  db.save_point_cloud("cloud", cloud, "test", "");
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::deviation};
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux create mesh"));
}

TEST_CASE("ParseSection_AcceptsXYAndSegments", "[visualize][scene][meshqa]") {
  const auto x = viz::parse_section("x=4.5");
  CHECK(x.kind == viz::SectionSpec::Kind::x);
  CHECK(x.at == 4.5);
  const auto y = viz::parse_section("y=-1");
  CHECK(y.kind == viz::SectionSpec::Kind::y);
  CHECK(y.at == -1.0);
  const auto s = viz::parse_section("0,1:3.5,1");
  CHECK(s.kind == viz::SectionSpec::Kind::segment);
  CHECK(s.p0 == std::array<double, 2>{0.0, 1.0});
  CHECK(s.p1 == std::array<double, 2>{3.5, 1.0});
  CHECK(viz::to_string(s) == "0,1:3.5,1");
  CHECK(viz::to_string(x) == "x=4.5");
  CHECK_THROWS_AS(viz::parse_section("z=1"), std::invalid_argument);
  CHECK_THROWS_AS(viz::parse_section("x=abc"), std::invalid_argument);
  CHECK_THROWS_AS(viz::parse_section("1,1:1,1"), std::invalid_argument);
}

TEST_CASE("ApplySection_KeepsOnlyTheSlabBetweenTheEnds",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_section");
  ProjectDB db(tmp.path);
  seed_qa_project(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::mesh};
  const auto info = viz::populate_scene(renderer, db, o);

  viz::SectionSpec spec = viz::parse_section("x=2");
  spec.thickness = 0.2;
  const auto sec = viz::resolve_section(spec, info.bounds);
  // x = 2 runs +Y across the whole scene; the camera looks along -X.
  CHECK(sec.dir[1] == Catch::Approx(1.0));
  CHECK(sec.view[0] == Catch::Approx(-1.0));
  CHECK(sec.p0[1] < info.bounds.min[1]);
  CHECK(sec.p1[1] > info.bounds.max[1]);

  viz::apply_section(renderer, sec);
  vtkActor *cloud = info.layers[0].actors[0];
  CHECK(cloud->GetMapper()->GetNumberOfClippingPlanes() == 4);
  CHECK(kept(cloud, {2.0, 1.5, 0.0}));
  CHECK(kept(cloud, {2.09, 1.5, 2.0}));
  CHECK_FALSE(kept(cloud, {2.11, 1.5, 0.0}));
  CHECK_FALSE(kept(cloud, {1.85, 1.5, 0.0}));

  // A segment also ends at its end points.
  viz::clear_cut_planes(renderer);
  const auto seg =
      viz::resolve_section(viz::parse_section("0,1:2,1"), info.bounds);
  viz::apply_section(renderer, seg);
  CHECK(kept(cloud, {1.0, 1.0, 0.0}));
  CHECK_FALSE(kept(cloud, {3.0, 1.0, 0.0}));
  CHECK_FALSE(kept(cloud, {-0.5, 1.0, 0.0}));

  spec.at = 99.0;
  CHECK_THROWS_AS(viz::resolve_section(spec, info.bounds),
                  std::invalid_argument);
}

TEST_CASE("AddMeshSectionLine_TracesTheMeshAndIsNeverClipped",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_line");
  ProjectDB db(tmp.path);
  seed_qa_project(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::cloud, viz::Layer::mesh};
  const auto info = viz::populate_scene(renderer, db, o);

  // A horizontal cut at 1.2 m crosses the four walls: a closed rectangle.
  vtkActor *line = viz::add_mesh_section_line(
      renderer, info, {0, 0, 1.2}, {0, 0, 1}, {0, 0, 0}, {255, 0, 0});
  REQUIRE(line != nullptr);
  CHECK(std::string(line->GetObjectName()) == viz::kSectionLineActorName);
  auto *poly = vtkPolyData::SafeDownCast(line->GetMapper()->GetInput());
  REQUIRE(poly != nullptr);
  CHECK(poly->GetNumberOfLines() >= 4);
  for (vtkIdType i = 0; i < poly->GetNumberOfPoints(); ++i) {
    double p[3];
    poly->GetPoint(i, p);
    CHECK(p[2] == Catch::Approx(1.2));
  }
  viz::apply_cut_plane(renderer, 1.2);
  CHECK(line->GetMapper()->GetNumberOfClippingPlanes() == 0);
  CHECK(info.layers[0].actors[0]->GetMapper()->GetNumberOfClippingPlanes() ==
        1);

  // A plane above the mesh misses it.
  CHECK(viz::add_mesh_section_line(renderer, info, {0, 0, 9}, {0, 0, 1},
                                   {0, 0, 0}, {255, 0, 0}) == nullptr);
}

TEST_CASE("PlacePresetCamera_Elevations_LookFromEachSide",
          "[visualize][scene][camera][meshqa]") {
  using viz::ViewPreset;
  viz::SceneBounds b;
  b.add(0, 0, 0);
  b.add(10, 2, 3);
  const auto look = [&](ViewPreset view) {
    vtkNew<vtkRenderer> r;
    viz::CameraFraming f;
    f.view = view;
    f.aspect = 1.0;
    viz::place_preset_camera(r, f, b);
    double dop[3];
    r->GetActiveCamera()->GetDirectionOfProjection(dop);
    return std::array<double, 4>{dop[0], dop[1], dop[2],
                                 r->GetActiveCamera()->GetParallelScale()};
  };
  const auto front = look(ViewPreset::front);
  CHECK(front[1] == Catch::Approx(1.0));
  const auto back = look(ViewPreset::back);
  CHECK(back[1] == Catch::Approx(-1.0));
  const auto left = look(ViewPreset::left);
  CHECK(left[0] == Catch::Approx(1.0));
  const auto right = look(ViewPreset::right);
  CHECK(right[0] == Catch::Approx(-1.0));
  // Front/back frame the 10 m side, left/right the 3 m height (the 2 m depth
  // is narrower): a square viewport, so the larger extent decides.
  CHECK(front[3] == Catch::Approx(back[3]));
  CHECK(left[3] == Catch::Approx(right[3]));
  CHECK(front[3] > left[3]);

  // The section view looks along the section's view direction.
  vtkNew<vtkRenderer> r;
  viz::CameraFraming f;
  f.view = ViewPreset::section;
  f.section = viz::resolve_section(viz::parse_section("x=5"), b);
  viz::place_preset_camera(r, f, b);
  double dop[3];
  r->GetActiveCamera()->GetDirectionOfProjection(dop);
  CHECK(dop[0] == Catch::Approx(-1.0));
}

TEST_CASE("DefaultQaSections_CrossEveryWing", "[visualize][scene][meshqa]") {
  // An L: a long west wing (u in [0, 3], v in [0, 30]) holding most points
  // and a short north wing (u in [3, 9], v in [27, 30]). The box midpoint
  // u = 4.5 would miss the long wing; the point median does not.
  std::vector<double> u, v;
  for (int i = 0; i < 300; ++i) {
    u.push_back(1.5);
    v.push_back(0.1 * i);
  }
  for (int i = 0; i < 60; ++i) {
    u.push_back(3.0 + 0.1 * i);
    v.push_back(28.5);
  }
  const auto s =
      viz::default_qa_sections(u, v, {0, 0, 0}, {9, 30, 3}, 0.0, 0.1);
  REQUIRE(s.size() == 3);
  CHECK(s[0].kind == viz::SectionSpec::Kind::segment);
  CHECK(s[0].p0 == std::array<double, 2>{1.5, -0.1});
  CHECK(s[0].p1 == std::array<double, 2>{1.5, 30.1});
  CHECK(s[1].p0[0] == Catch::Approx(-0.1));
  CHECK(s[1].p1[0] == Catch::Approx(9.1));
  CHECK(s[1].p0[1] < s[2].p0[1]);
  CHECK(s[2].thickness == 0.1);

  // Yaw 90: (u, v) = (-y, x), so a +v line runs world +x at y = -u.
  std::vector<double> u2{1.0}, v2{2.0};
  const auto r =
      viz::default_qa_sections(u2, v2, {0, 0, 0}, {2, 4, 3}, 90.0, 0.1);
  CHECK(r[0].p0[1] == Catch::Approx(-1.0));
  CHECK(r[0].p1[1] == Catch::Approx(-1.0));
  CHECK(r[0].p1[0] > r[0].p0[0]);
}

TEST_CASE("PopulateScene_FrameQuantile_IgnoresStrayPoints",
          "[visualize][scene][meshqa]") {
  TempPath tmp("test_scene_meshqa_quantile");
  ProjectDB db(tmp.path);
  Cloud cloud;
  for (int i = 0; i < 1000; ++i)
    cloud.push_back(at(0.001F * static_cast<float>(i), 0.5F, 0.5F));
  cloud.push_back(at(100.0F, 0.5F, 0.5F)); // one stray point
  db.save_point_cloud("cloud", cloud, "test", "");

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  auto info = viz::populate_scene(renderer, db, o);
  CHECK(info.frame_bounds.max[0] == Catch::Approx(100.0)); // default: all

  vtkNew<vtkRenderer> renderer2;
  o.frame_quantile = 0.002;
  info = viz::populate_scene(renderer2, db, o);
  CHECK(info.bounds.max[0] == Catch::Approx(100.0));
  CHECK(info.frame_bounds.max[0] < 1.0);
  CHECK(info.frame_bounds.min[0] >= 0.0);

  // A yaw measures the box in the turned frame: at 90 deg, u = -y, v = x.
  vtkNew<vtkRenderer> renderer3;
  o.frame_yaw_deg = 90.0;
  info = viz::populate_scene(renderer3, db, o);
  REQUIRE(info.display_bounds.has_value());
  CHECK(info.display_bounds->min[1] >= 0.0);
  CHECK(info.display_bounds->max[1] < 1.0);
  CHECK(info.display_bounds->min[0] == Catch::Approx(-0.5).margin(1e-6));
}
