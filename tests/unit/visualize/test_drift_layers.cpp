// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The drift-loop render layers (reusex-rcv.1.4): passes, graph, drift.
//
// Scene-level tests inspect the actors populate_scene() adds (no GL); the
// one pixel test renders a doubled surface offscreen and is skipped on a
// machine without offscreen OpenGL.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/frame_depth_cache.hpp>
#include <reusex/core/pose_edits.hpp>
#include <reusex/slam/drift_diagnostics.hpp>
#include <reusex/visualize/drift_layers.hpp>
#include <reusex/visualize/render_view.hpp>
#include <reusex/visualize/scene.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCellData.h>
#include <vtkMapper.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <span>
#include <string>
#include <vector>

using namespace reusex;
using namespace reusex::test_support;
using Catch::Matchers::ContainsSubstring;
namespace viz = reusex::visualize;

namespace {

using Rgb = std::array<std::uint8_t, 3>;

/// Six frames seeing a flat surface 2 m ahead (a constant depth image):
/// frames 1-3 from the origin, frames 4-6 (the second pass) from
/// (0.3, 0, 0.05) -- the same surface, doubled.
void seed_two_passes(ProjectDB &db) {
  for (int id = 1; id <= 6; ++id) {
    const bool second = id >= 4;
    db.save_sensor_frame(id, make_color(32, 24), make_depth(32, 24, 2.0),
                         cv::Mat(),
                         second ? translation_pose(0.3, 0.0, 0.05)
                                : translation_pose(0.0, 0.0, 0.0),
                         make_intrinsics(16.0, 16.0, 16.0, 12.0, 32, 24),
                         static_cast<double>(id));
  }
}

viz::PreviewCloudOptions two_segments() {
  viz::PreviewCloudOptions o;
  o.segments = 2;
  o.depth_stride = 2;
  return o;
}

vtkPolyData *poly_of(vtkActor *actor) {
  auto *poly = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  REQUIRE(poly != nullptr);
  return poly;
}

/// Count of each colour among an actor's point (or cell) scalars.
std::map<Rgb, int> colour_counts(vtkActor *actor, bool cells = false) {
  vtkPolyData *poly = poly_of(actor);
  auto *rgb = vtkUnsignedCharArray::SafeDownCast(
      cells ? poly->GetCellData()->GetScalars()
            : poly->GetPointData()->GetScalars());
  REQUIRE(rgb != nullptr);
  std::map<Rgb, int> out;
  for (vtkIdType i = 0; i < rgb->GetNumberOfTuples(); ++i)
    ++out[{rgb->GetTypedComponent(i, 0), rgb->GetTypedComponent(i, 1),
           rgb->GetTypedComponent(i, 2)}];
  return out;
}

viz::DriftHeatmap two_storey_heatmap() {
  viz::DriftHeatmap h;
  h.cell_m = 0.5;
  h.noise_floor_m = 0.007;
  for (int i = 0; i < 2; ++i) {
    viz::DriftHeatStorey s;
    s.idx = i;
    s.name = i == 0 ? "Stue" : "1. sal";
    s.z_floor = i == 0 ? 0.0 : 2.8;
    s.z_ceiling = i == 0 ? 2.5 : 5.0;
    s.level_transform[3] = 10.0; // level = world + 10 m in x
    h.storeys.push_back(s);
  }
  h.cells.push_back({0, 10.25, 0.25, 0.0, 0.005, 100});
  h.cells.push_back({0, 10.75, 0.25, 0.0, 0.050, 100});
  h.cells.push_back({1, 10.25, 0.25, 2.8, 0.020, 100});
  return h;
}

} // namespace

TEST_CASE("DriftLayers_Vocabulary_RoundTrips", "[visualize][drift]") {
  for (const auto *name : {"passes", "graph", "drift"}) {
    const auto layer = viz::layer_from_string(name);
    REQUIRE(layer);
    CHECK(viz::to_string(*layer) == name);
    CHECK(std::find(viz::all_layers().begin(), viz::all_layers().end(),
                    *layer) != viz::all_layers().end());
  }
}

TEST_CASE("DriftLayers_PassSegment_MatchesTheDriftReportRule",
          "[visualize][drift]") {
  // slam::diagnose_drift: segment = k * segments / n.
  CHECK(viz::pass_segment(0, 6, 2) == 0);
  CHECK(viz::pass_segment(2, 6, 2) == 0);
  CHECK(viz::pass_segment(3, 6, 2) == 1);
  CHECK(viz::pass_segment(5, 6, 2) == 1);
  CHECK(viz::pass_segment(383, 384, 12) == 11);
  const auto &pal = viz::default_label_palette();
  CHECK(viz::pass_color(pal, 1) == pal.colors[1]);
  CHECK(viz::pass_color(pal, static_cast<int>(pal.colors.size())) ==
        pal.colors[0]);
}

TEST_CASE("DriftLayers_PreviewCloud_KeepsEachPointsFrame",
          "[visualize][drift]") {
  TempPath tmp("test_drift_preview");
  ProjectDB db(tmp.path);
  seed_two_passes(db);
  const auto pc = viz::preview_cloud(db, two_segments());
  REQUIRE(pc.node_ids.size() == 6);
  CHECK(pc.frame_segment == std::vector<int>{0, 0, 0, 1, 1, 1});
  REQUIRE(pc.segments.size() == 2);
  CHECK(pc.segments[1].first_node == 4);
  CHECK(pc.segments[1].last_node == 6);
  // 16 x 12 samples per frame; frames of one pass coincide and thin to one
  // point per voxel, the two passes do not merge.
  CHECK(pc.points.size() == 2 * 16 * 12);
  CHECK(pc.segments[0].points == 16 * 12);
  REQUIRE(pc.frame.size() == pc.points.size());
  for (std::size_t i = 0; i < pc.points.size(); ++i)
    CHECK(std::abs(pc.points[i].z() -
                   (pc.frame_segment[pc.frame[i]] == 0 ? 2.0F : 2.05F)) <
          1e-3F);

  // A candidate solve replaces the stored poses.
  std::vector<Eigen::Matrix4d> poses(6, Eigen::Matrix4d::Identity());
  const auto moved = viz::preview_cloud(db, two_segments(), poses);
  for (const auto &p : moved.points)
    CHECK(std::abs(p.z() - 2.0F) < 1e-3F);
  poses.pop_back();
  CHECK_THROWS_AS(viz::preview_cloud(db, two_segments(), poses),
                  std::invalid_argument);
}

TEST_CASE("DriftLayers_PreviewCloud_DepthCacheIsBitIdentical",
          "[visualize][drift][cache]") {
  // reusex-rcv.1.17: a ramp of depths, with holes and returns on and past
  // both ends of the range, seen with a tilted local transform.
  TempPath tmp("test_drift_preview_cache");
  ProjectDB db(tmp.path);
  auto k = make_intrinsics(30.0, 28.0, 16.5, 11.5, 32, 24);
  k.local_transform = {0.0, 0.0,  1.0, 0.03, -1.0, 0.0, 0.0, -0.01,
                       0.0, -1.0, 0.0, 0.12, 0.0,  0.0, 0.0, 1.0};
  for (int id = 1; id <= 7; ++id) {
    cv::Mat depth(24, 32, CV_16UC1);
    for (int v = 0; v < depth.rows; ++v)
      for (int u = 0; u < depth.cols; ++u)
        depth.at<std::uint16_t>(v, u) = static_cast<std::uint16_t>(
            (u + v + id) % 9 == 0 ? 0 : 150 + 37 * u + 101 * v + 13 * id);
    depth.at<std::uint16_t>(0, 0) = 200;  // exactly min
    depth.at<std::uint16_t>(2, 2) = 4000; // exactly max
    db.save_sensor_frame(id, make_color(32, 24), depth, cv::Mat(),
                         translation_pose(0.1 * id, 0.0, 0.0), k,
                         static_cast<double>(id));
  }
  std::vector<Eigen::Matrix4d> poses;
  for (int id = 1; id <= 7; ++id) {
    Eigen::Matrix4d p = Eigen::Matrix4d::Identity();
    p.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(0.07 * id, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    p.topRightCorner<3, 1>() = Eigen::Vector3d(12.3 + 0.2 * id, -4.1, 0.7);
    poses.push_back(p);
  }
  for (double voxel : {0.0, 0.02}) {
    for (int step : {1, 2}) {
      viz::PreviewCloudOptions o;
      o.segments = 3;
      o.frame_step = step;
      o.depth_stride = 3;
      o.voxel_m = voxel;
      const auto cache =
          core::cache_frame_depth(db, viz::preview_depth_sampling(o));
      INFO("voxel " << voxel << " frame step " << step);
      for (const bool candidate : {false, true}) {
        const std::span<const Eigen::Matrix4d> ps =
            candidate ? std::span<const Eigen::Matrix4d>(poses)
                      : std::span<const Eigen::Matrix4d>();
        const auto a = viz::preview_cloud(db, o, ps);
        const auto b = viz::preview_cloud(db, o, ps, &cache);
        REQUIRE(!a.points.empty());
        REQUIRE(a.points.size() == b.points.size());
        CHECK(std::memcmp(a.points.data(), b.points.data(),
                          a.points.size() * sizeof(a.points[0])) == 0);
        CHECK(a.frame == b.frame);
        CHECK(a.node_ids == b.node_ids);
        CHECK(a.frame_segment == b.frame_segment);
        REQUIRE(a.segments.size() == b.segments.size());
        for (std::size_t i = 0; i < a.segments.size(); ++i)
          CHECK(a.segments[i].points == b.segments[i].points);
      }
    }
  }
  viz::PreviewCloudOptions o;
  const auto cache =
      core::cache_frame_depth(db, viz::preview_depth_sampling(o));
  o.depth_stride = 4;
  CHECK_THROWS_AS(viz::preview_cloud(db, o, {}, &cache), std::invalid_argument);
}

TEST_CASE("DriftLayers_Passes_ColoursEachPassApart", "[visualize][drift]") {
  TempPath tmp("test_drift_passes");
  ProjectDB db(tmp.path);
  seed_two_passes(db);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::passes};
  o.passes = two_segments();
  const auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 1);
  REQUIRE(info.layers[0].actors.size() == 1);
  const auto counts = colour_counts(info.layers[0].actors[0]);
  const auto &pal = viz::default_label_palette();
  CHECK(counts.size() == 2);
  CHECK(counts.at(pal.colors[0]) == 16 * 12);
  CHECK(counts.at(pal.colors[1]) == 16 * 12);
  // The legend names each pass's nodes in its colour.
  const auto &lg = info.drift_legend;
  CHECK(lg.passes_drawn);
  REQUIRE(lg.passes.size() == 2);
  CHECK(lg.passes[1].first_node == 4);
  CHECK(lg.passes[1].rgb == pal.colors[1]);
}

TEST_CASE("DriftLayers_Graph_TrajectoryEdgesByResidualAndPins",
          "[visualize][drift]") {
  TempPath tmp("test_drift_graph");
  ProjectDB db(tmp.path);
  for (int id = 1; id <= 6; ++id)
    db.save_sensor_frame(id, make_color(32, 24), cv::Mat(), cv::Mat(),
                         translation_pose(id, 0.0, 1.4),
                         make_intrinsics(16.0, 16.0, 16.0, 12.0, 32, 24));
  const auto edge = [&](int a, int b, const char *type, double residual,
                        const char *status = "auto") {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = type;
    e.residual = residual;
    e.status = status;
    db.upsert_pose_graph_edge(e, /*overwrite_rejected=*/true);
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  edge(1, 6, "loop_closure", 1.0);             // good
  edge(2, 5, "loop_closure", 100.0);           // bad
  edge(3, 4, "loop_closure", nan);             // never solved
  edge(1, 2, "odometry", 1.0);                 // along the path: not drawn
  edge(2, 4, "loop_closure", 1.0, "rejected"); // not drawn

  core::PoseEdits edits;
  core::PoseEdit anchor;
  anchor.id = "a";
  anchor.kind = core::PoseEditKind::anchor;
  anchor.nodes.first = anchor.nodes.last = 3;
  core::PoseEdit move;
  move.id = "m";
  move.kind = core::PoseEditKind::move;
  move.nodes.first = 4;
  move.nodes.last = 6;
  edits.edits = {anchor, move};

  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::graph};
  o.passes.segments = 2;
  o.graph.label_every = 2;
  o.edits = edits;
  const auto info = viz::populate_scene(renderer, db, o);
  const auto &lg = info.drift_legend;
  CHECK(lg.graph_drawn);
  CHECK(lg.graph_nodes == 6);
  CHECK(lg.graph_edges == 3);
  CHECK(lg.graph_edges_bad == 1);
  CHECK(lg.graph_edges_unsolved == 1);
  CHECK(lg.graph_pins == 3); // the anchor, the move's first and last node
  REQUIRE(info.layers.size() == 1);
  const auto &layer = info.layers[0];
  // Every actor carries the graph name (the cut keeps it in a slab).
  for (vtkActor *a : layer.actors)
    CHECK(std::string(a->GetObjectName()) == viz::kGraphActorName);

  // The trajectory (6 vertices, coloured by segment) and the edges (one
  // cell each, coloured by residual).
  const auto &pal = viz::default_label_palette();
  bool trajectory = false, edges = false;
  for (vtkActor *a : layer.actors) {
    vtkPolyData *p = poly_of(a);
    if (p->GetNumberOfLines() == 5 && p->GetPointData()->GetScalars()) {
      const auto c = colour_counts(a);
      trajectory = c.count(pal.colors[0]) == 1 && c.count(pal.colors[1]) == 1;
    }
    if (p->GetNumberOfLines() == 3 && p->GetCellData()->GetScalars()) {
      const auto c = colour_counts(a, /*cells=*/true);
      edges = c.count(viz::residual_color(1.0, o.graph)) == 1 &&
              c.count(viz::residual_color(100.0, o.graph)) == 1 &&
              c.count(o.graph.unsolved_rgb) == 1;
    }
  }
  CHECK(trajectory);
  CHECK(edges);
  // Good is green, bad beyond the scale is not.
  CHECK(viz::residual_color(0.0, o.graph) !=
        viz::residual_color(100.0, o.graph));
}

TEST_CASE("DriftLayers_Drift_HeatCellsPerStorey", "[visualize][drift]") {
  TempPath tmp("test_drift_heat");
  ProjectDB db(tmp.path);
  vtkNew<vtkRenderer> renderer;
  viz::SceneOptions o;
  o.layers = {viz::Layer::drift};

  // No report: the error names the command that makes one.
  CHECK_THROWS_WITH(viz::populate_scene(renderer, db, o),
                    ContainsSubstring("rux analyze drift"));

  o.drift_heatmap = two_storey_heatmap();
  auto info = viz::populate_scene(renderer, db, o);
  REQUIRE(info.layers.size() == 1);
  const auto &layer = info.layers[0];
  REQUIRE(layer.actors.size() == 2);
  CHECK(layer.actor_storeys == std::vector<int>{0, 1});
  // Storey 0's two cells, mapped back from the level frame (x - 10) and
  // drawn just above the floor, coloured by residual.
  vtkPolyData *p0 = poly_of(layer.actors[0]);
  CHECK(p0->GetNumberOfPolys() == 2);
  double b[6];
  p0->GetBounds(b);
  CHECK(std::abs(b[0] - 0.0) < 1e-9);
  CHECK(std::abs(b[1] - 1.0) < 1e-9);
  CHECK(std::abs(b[4] - o.drift.lift_m) < 1e-9);
  const auto c0 = colour_counts(layer.actors[0], /*cells=*/true);
  CHECK(c0.count(viz::drift_color(0.005, o.drift)) == 1);
  CHECK(c0.count(viz::drift_color(0.050, o.drift)) == 1);
  vtkPolyData *p1 = poly_of(layer.actors[1]);
  p1->GetBounds(b);
  CHECK(std::abs(b[4] - (2.8 + o.drift.lift_m)) < 1e-9);
  CHECK(info.drift_legend.drift_drawn);
  CHECK(info.drift_legend.drift_cells == 3);
  CHECK(std::abs(info.drift_legend.drift_max_m - 0.05) < 1e-12);

  // A cut at z = 3.5 shows storey 1 only.
  CHECK(viz::show_drift_storey_at(info, 3.5, o) == 1);
  CHECK(!layer.actors[0]->GetVisibility());
  CHECK(layer.actors[1]->GetVisibility());
  CHECK(info.drift_legend.drift_storey == 1);
  CHECK(info.drift_legend.drift_storey_name == "1. sal");
  // ... and the legend counts what is shown.
  CHECK(info.drift_legend.drift_cells == 1);
  CHECK(std::abs(info.drift_legend.drift_max_m - 0.02) < 1e-12);

  // An explicit storey draws only that one.
  vtkNew<vtkRenderer> r2;
  o.drift.storey = 0;
  const auto one = viz::populate_scene(r2, db, o);
  CHECK(one.layers[0].actors.size() == 1);
  CHECK(one.layers[0].actor_storeys == std::vector<int>{0});
}

TEST_CASE("DriftLayers_HeatmapJson_ReadsTheDriftReport", "[visualize][drift]") {
  slam::DriftReport r;
  r.cell_m = 0.25;
  r.noise_floor_m = 0.007;
  slam::DriftStorey s;
  s.idx = 1;
  s.name = "Loft";
  s.z_floor = 2.8;
  s.z_ceiling = 5.0;
  s.level_transform[3] = 4.0;
  r.storeys.push_back(s);
  slam::DriftCell c;
  c.storey = 1;
  c.x = 1.125;
  c.y = -0.375;
  c.z_m = 2.9;
  c.residual_m = 0.031;
  c.points = 77;
  r.cells.push_back(c);
  const auto h = viz::drift_heatmap_from_json(slam::drift_report_to_json(r));
  CHECK(h.cell_m == 0.25);
  REQUIRE(h.noise_floor_m);
  CHECK(*h.noise_floor_m == 0.007);
  REQUIRE(h.storeys.size() == 1);
  CHECK(h.storeys[0].name == "Loft");
  CHECK(h.storeys[0].level_transform[3] == 4.0);
  REQUIRE(h.cells.size() == 1);
  CHECK(h.cells[0].storey == 1);
  CHECK(h.cells[0].x == 1.125);
  CHECK(h.cells[0].residual_m == 0.031);
  CHECK(h.cells[0].points == 77);

  // A pre-storey report (no "storeys", no "cell_m") reads as world cells.
  const auto old = viz::drift_heatmap_from_json(
      R"({"cells":[{"x":0.25,"y":0.25,"residual_m":0.02,"points":60}]})");
  CHECK(old.storeys.empty());
  CHECK(old.cell_m == 0.5);
  REQUIRE(old.cells.size() == 1);
  CHECK(old.cells[0].storey == -1);

  CHECK_THROWS_AS(viz::drift_heatmap_from_json("not json"),
                  std::invalid_argument);
  CHECK_THROWS_AS(viz::drift_heatmap_from_json(R"({"frames": []})"),
                  std::invalid_argument);
}

TEST_CASE("DriftLayers_RenderPasses_DoubledSurfaceShowsTwoColours",
          "[visualize][drift][render]") {
  TempPath tmp("test_drift_render");
  ProjectDB db(tmp.path);
  seed_two_passes(db);
  viz::RenderOptions o;
  o.layers = {viz::Layer::passes};
  o.passes = two_segments();
  o.view = viz::ViewPreset::top;
  o.width = 320;
  o.height = 240;
  o.point_size = 6.0;
  o.legend = false;
  cv::Mat image;
  try {
    image = viz::render_view(db, o);
  } catch (const viz::OffscreenGlUnavailable &e) {
    SKIP("no offscreen OpenGL: " << e.what());
  }
  const auto &pal = viz::default_label_palette();
  std::map<Rgb, int> seen;
  for (int y = 0; y < image.rows; ++y)
    for (int x = 0; x < image.cols; ++x) {
      const auto &v = image.at<cv::Vec3b>(y, x);
      ++seen[{v[2], v[1], v[0]}];
    }
  INFO("distinct colours " << seen.size());
  CHECK(seen[pal.colors[0]] > 50);
  CHECK(seen[pal.colors[1]] > 50);

  // The legend draws into the image when asked.
  o.legend = true;
  const cv::Mat with_legend = viz::render_view(db, o);
  CHECK(cv::norm(with_legend, image, cv::NORM_L1) > 0.0);
}
