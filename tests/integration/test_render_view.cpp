// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Headless rendering integration test (#294).
//
// What this asserts, and deliberately does not:
//
//   - It DOES assert that render_view() returns an image of the requested size
//     in which a substantial share of the pixels are not the background. That
//     is the regression this test exists to catch: a render pipeline that
//     "succeeds" and hands back a black frame — a missing OpenGL factory, an
//     off-screen context that never got a surface, a camera pointed at nothing.
//     Every one of those failures is invisible to an exit code and obvious in
//     one pixel statistic.
//   - It DOES assert that two identical calls produce byte-identical images
//     (STANDARDS §6), which is what makes rendered artifacts usable for
//     before/after comparison.
//   - It does NOT compare against a golden image. Point rasterization differs
//     across GPUs and driver versions, so a golden file would fail for reasons
//     unrelated to the code under test.
//
// Rendering needs a rendering device, and a GitHub-hosted CI runner or a nix
// build sandbox has none. These tests used to guess at that themselves, by
// looking for a DRM render node or an X/Wayland session, because render_view()
// answered the question with a segfault. It no longer does (#313): it probes
// EGL and throws OffscreenGlUnavailable, so the guess is gone and the tests
// simply render and skip on *that* exception. Two things follow — the skip
// condition is now the library's own verdict rather than a second, divergent
// heuristic, and the diagnosis itself is under test, since a machine that can
// render must not produce that exception.
//
// "Renders with no display" is still asserted in full: on any machine with a
// GPU these tests run with DISPLAY unset and exercise the EGL fallback, which
// is the case the feature exists for.
//
// The error-path test renders nothing successfully: every case it covers is
// rejected before any GL work happens, so it keeps its value on a device-less
// runner.

#include "../support/source_dir.hpp"
#include "../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/visualize/render_view.hpp>

#include <opencv2/core.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace fs = std::filesystem;
namespace viz = reusex::visualize;

namespace {

/// Render, or skip the test when the library reports this machine has no
/// offscreen OpenGL.
///
/// Every other exception propagates: a missing cloud or a stale label layer is
/// a failure, not a reason to stop looking.
cv::Mat render_or_skip(const reusex::ProjectDB &db,
                       const viz::RenderOptions &opts) {
  try {
    return viz::render_view(db, opts);
  } catch (const viz::OffscreenGlUnavailable &e) {
    SKIP("No offscreen OpenGL on this machine: " << e.what());
  }
  return {}; // unreachable; SKIP throws
}

/// tests/fixtures/scans/office_corridor.rux, located via the repo root.
fs::path fixture_path() {
  return reusex::test_support::source_dir() / "tests" / "fixtures" / "scans" /
         "office_corridor.rux";
}

/// True when `p` is an unexpanded git-lfs pointer rather than the real blob.
bool is_lfs_pointer(const fs::path &p) {
  std::error_code ec;
  const auto size = fs::file_size(p, ec);
  if (ec || size > 1024) {
    return false;
  }
  std::ifstream in(p, std::ios::binary);
  std::string head(64, '\0');
  in.read(head.data(), static_cast<std::streamsize>(head.size()));
  head.resize(static_cast<std::size_t>(in.gcount()));
  return head.rfind("version https://git-lfs", 0) == 0;
}

/// Fraction of pixels that differ noticeably from the render background.
///
/// Anti-aliased point edges blend toward the background, so "not background"
/// is a tolerance, not equality.
double foreground_fraction(const cv::Mat &image,
                           const std::array<double, 3> &background) {
  REQUIRE(image.type() == CV_8UC3);
  const cv::Vec3b bg(static_cast<unsigned char>(background[2] * 255.0 + 0.5),
                     static_cast<unsigned char>(background[1] * 255.0 + 0.5),
                     static_cast<unsigned char>(background[0] * 255.0 + 0.5));

  std::size_t foreground = 0;
  for (int y = 0; y < image.rows; ++y) {
    const auto *row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image.cols; ++x) {
      const int db = std::abs(int(row[x][0]) - int(bg[0]));
      const int dg = std::abs(int(row[x][1]) - int(bg[1]));
      const int dr = std::abs(int(row[x][2]) - int(bg[2]));
      if (db + dg + dr > 12) {
        ++foreground;
      }
    }
  }
  return static_cast<double>(foreground) /
         static_cast<double>(image.rows * image.cols);
}

/// An exactly-this-colour pixel count.
///
/// Point sprites are drawn with lighting off, direct scalars and no
/// multisampling, so a point's colour reaches the framebuffer unmodified and
/// an exact match is the right test — which is what lets the plan-view test
/// ask "is any of the ceiling still visible" and get a yes/no answer.
std::size_t count_color(const cv::Mat &image, int r, int g, int b) {
  const cv::Vec3b want(static_cast<unsigned char>(b),
                       static_cast<unsigned char>(g),
                       static_cast<unsigned char>(r));
  std::size_t hits = 0;
  for (int y = 0; y < image.rows; ++y) {
    const auto *row = image.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image.cols; ++x) {
      if (row[x] == want) {
        ++hits;
      }
    }
  }
  return hits;
}

/// Surface colours of coloured_room(), as RGB.
constexpr int kFloorRgb[3] = {40, 90, 230};
constexpr int kCeilingRgb[3] = {230, 60, 40};
constexpr int kWallRgb[3] = {60, 200, 90};

/// A closed room whose floor, ceiling and walls are each a different colour.
///
/// The point of a floor plan is *which* surfaces it shows, and the hollow box
/// below cannot answer that — every face of it is the same colour. This one
/// can: count the ceiling pixels.
reusex::CloudPtr coloured_room(float height = 2.7F, int per_side = 60) {
  auto cloud = std::make_shared<reusex::Cloud>();
  const auto add = [&cloud](float x, float y, float z, const int rgb[3]) {
    reusex::PointT p;
    p.x = x, p.y = y, p.z = z;
    p.r = static_cast<std::uint8_t>(rgb[0]);
    p.g = static_cast<std::uint8_t>(rgb[1]);
    p.b = static_cast<std::uint8_t>(rgb[2]);
    cloud->push_back(p);
  };

  const int last = per_side - 1;
  const float span = 4.0F; // the room is 4 x 4 m, centred on the origin
  for (int i = 0; i < per_side; ++i) {
    const float u = -0.5F * span + span * static_cast<float>(i) / last;
    for (int j = 0; j < per_side; ++j) {
      const float v = -0.5F * span + span * static_cast<float>(j) / last;
      add(u, v, 0.0F, kFloorRgb);
      add(u, v, height, kCeilingRgb);
    }
    for (int k = 0; k < per_side; ++k) {
      const float z = height * static_cast<float>(k) / last;
      add(u, -0.5F * span, z, kWallRgb);
      add(u, 0.5F * span, z, kWallRgb);
      add(-0.5F * span, u, z, kWallRgb);
      add(0.5F * span, u, z, kWallRgb);
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  return cloud;
}

/// A hollow box of points, so a render has something with extent in all axes.
reusex::CloudPtr synthetic_room(int per_side = 60) {
  auto cloud = std::make_shared<reusex::Cloud>();
  const float step = 2.0F / static_cast<float>(per_side - 1);
  for (int i = 0; i < per_side; ++i) {
    for (int j = 0; j < per_side; ++j) {
      const float u = -1.0F + step * static_cast<float>(i);
      const float v = -1.0F + step * static_cast<float>(j);
      for (const float w : {-1.0F, 1.0F}) {
        reusex::PointT p;
        p.x = u, p.y = v, p.z = w;
        p.r = 220, p.g = 180, p.b = 90;
        cloud->push_back(p);
        p.x = u, p.y = w, p.z = v;
        cloud->push_back(p);
      }
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  return cloud;
}

} // namespace

TEST_CASE("RenderView_HeadlessSyntheticProject_DrawsNonBlankDeterministicImage",
          "[integration][render]") {
  const reusex::test_support::TempDir work("reusex_render_synthetic");
  const fs::path project = work.path / "synthetic.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  const auto cloud = synthetic_room();
  REQUIRE(cloud->size() > 1000);
  db.save_point_cloud("cloud", *cloud);

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 480;
  opts.height = 360;

  // ── The image has the requested shape and is not blank ──────────────────
  const cv::Mat top = render_or_skip(db, opts);
  CHECK(top.cols == opts.width);
  CHECK(top.rows == opts.height);
  CHECK(top.type() == CV_8UC3);

  const double covered = foreground_fraction(top, opts.background);
  INFO("foreground fraction (top): " << covered);
  CHECK(covered > 0.01);

  // ── Repeating the call reproduces the frame exactly (STANDARDS §6) ──────
  const cv::Mat again = render_or_skip(db, opts);
  cv::Mat difference;
  cv::absdiff(top, again, difference);
  CHECK(cv::countNonZero(difference.reshape(1)) == 0);

  // ── An orbit view of the same box is also non-blank ─────────────────────
  opts.view = viz::ViewPreset::orbit;
  opts.orbit_count = 4;
  opts.orbit_index = 1;
  const cv::Mat orbit = render_or_skip(db, opts);
  INFO("foreground fraction (orbit): " << foreground_fraction(orbit,
                                                              opts.background));
  CHECK(foreground_fraction(orbit, opts.background) > 0.01);

  // ── Label colouring actually recolours the geometry ─────────────────────
  //
  // Rendering the same points through the `planes` layer must not produce the
  // same pixels as their stored RGB, or the label layer is a no-op.
  auto labels = std::make_shared<reusex::CloudL>();
  labels->resize(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    labels->points[i].label = static_cast<std::uint32_t>(1 + (i % 3));
  }
  labels->width = labels->size();
  labels->height = 1;
  db.save_point_cloud("planes", *labels);

  viz::RenderOptions label_opts;
  label_opts.layers = {viz::Layer::planes};
  label_opts.width = opts.width;
  label_opts.height = opts.height;
  const cv::Mat by_plane = render_or_skip(db, label_opts);
  CHECK(foreground_fraction(by_plane, label_opts.background) > 0.01);

  cv::absdiff(top, by_plane, difference);
  CHECK(cv::countNonZero(difference.reshape(1)) > 0);
}

TEST_CASE("RenderView_PlanView_CutsAwayTheCeilingAndShowsTheFloor",
          "[integration][render]") {
  // The acceptance criterion of #306, stated in pixels: `top` on a closed
  // interior shows the ceiling and nothing else, `plan` shows the floor and
  // the walls in section and none of the ceiling.
  const reusex::test_support::TempDir work("reusex_render_plan");
  const fs::path project = work.path / "plan.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());
  db.save_point_cloud("cloud", *coloured_room());

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 480;
  opts.height = 360;
  const auto pixels = static_cast<double>(opts.width * opts.height);

  const auto ceiling = [](const cv::Mat &m) {
    return count_color(m, kCeilingRgb[0], kCeilingRgb[1], kCeilingRgb[2]);
  };
  const auto floor = [](const cv::Mat &m) {
    return count_color(m, kFloorRgb[0], kFloorRgb[1], kFloorRgb[2]);
  };
  const auto wall = [](const cv::Mat &m) {
    return count_color(m, kWallRgb[0], kWallRgb[1], kWallRgb[2]);
  };

  // ── `top` is the problem: it is a picture of the ceiling ─────────────────
  const cv::Mat top = render_or_skip(db, opts);
  INFO("top: ceiling " << ceiling(top) << ", floor " << floor(top));
  CHECK(static_cast<double>(ceiling(top)) > 0.05 * pixels);
  // The ceiling sits directly over the floor on the same sample grid, so the
  // floor is not merely rare in a top view — it is completely hidden.
  CHECK(static_cast<double>(floor(top)) < 0.001 * pixels);

  // ── `plan` is the fix ────────────────────────────────────────────────────
  //
  // This project has no plane segmentation, so the cut is placed by the
  // bounding-box fallback: 45 % of 2.7 m ≈ 1.2 m above the floor.
  opts.view = viz::ViewPreset::plan;
  const cv::Mat plan = render_or_skip(db, opts);
  INFO("plan: ceiling " << ceiling(plan) << ", floor " << floor(plan)
                        << ", wall " << wall(plan));
  CHECK(ceiling(plan) == 0); // exact: clipping is per-vertex, not a heuristic
  CHECK(static_cast<double>(floor(plan)) > 0.05 * pixels);
  CHECK(wall(plan) > 0);

  // ── An explicit height moves the cut, and still removes the ceiling ──────
  opts.cut_height = 0.5;
  const cv::Mat low = render_or_skip(db, opts);
  CHECK(ceiling(low) == 0);
  CHECK(static_cast<double>(floor(low)) > 0.05 * pixels);

  // A cut above everything is a plain top view again — and says so in the log
  // rather than pretending to be a plan (STANDARDS §5).
  opts.cut_height = 10.0;
  const cv::Mat above_everything = render_or_skip(db, opts);
  CHECK(static_cast<double>(ceiling(above_everything)) > 0.05 * pixels);

  // ── The cut is a property of the scene, not of the plan camera ───────────
  viz::RenderOptions orbit_opts;
  orbit_opts.layers = {viz::Layer::cloud};
  orbit_opts.width = opts.width;
  orbit_opts.height = opts.height;
  orbit_opts.view = viz::ViewPreset::orbit;
  orbit_opts.orbit_count = 4;
  orbit_opts.orbit_index = 1;
  const cv::Mat orbit = render_or_skip(db, orbit_opts);
  CHECK(static_cast<double>(ceiling(orbit)) > 0.0);

  orbit_opts.cut = true;
  const cv::Mat cut_orbit = render_or_skip(db, orbit_opts);
  CHECK(ceiling(cut_orbit) == 0);
  CHECK(wall(cut_orbit) > 0);

  // ── A cut height is measured upward, so zero and below are nonsense ──────
  viz::RenderOptions bad_cut;
  bad_cut.view = viz::ViewPreset::plan;
  bad_cut.cut_height = -1.0;
  CHECK_THROWS_AS(viz::render_view(db, bad_cut), std::runtime_error);
}

TEST_CASE("RenderView_PlanViewWithSegmentedPlanes_CutsAboveTheDetectedFloor",
          "[integration][render]") {
  // With `rux create planes` run, the cut is measured from the floor plane
  // rather than the bounding box — which matters exactly when the two differ,
  // so this project has a stray point 5 m below the floor to pull the bounding
  // box down. A cut placed from the box would land below the real floor and
  // render an empty frame.
  const reusex::test_support::TempDir work("reusex_render_plan_planes");
  const fs::path project = work.path / "plan_planes.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  auto cloud = coloured_room();
  reusex::PointT stray;
  stray.x = 0.0F, stray.y = 0.0F, stray.z = -5.0F;
  stray.r = 255, stray.g = 255, stray.b = 255;
  cloud->push_back(stray);
  cloud->width = cloud->size();
  db.save_point_cloud("cloud", *cloud);

  // The per-plane clouds `rux create planes` would write: a floor at z = 0 and
  // a ceiling at z = 2.7, both horizontal.
  auto centroids = std::make_shared<reusex::CloudLoc>();
  centroids->push_back(reusex::LocT(0.0F, 0.0F, 0.0F));
  centroids->push_back(reusex::LocT(0.0F, 0.0F, 2.7F));
  centroids->width = centroids->size();
  centroids->height = 1;

  auto normals = std::make_shared<reusex::CloudN>();
  normals->resize(2);
  for (auto &n : normals->points) {
    n.normal_x = 0.0F, n.normal_y = 0.0F, n.normal_z = 1.0F;
  }
  normals->width = normals->size();
  normals->height = 1;

  db.save_point_cloud("plane_centroids", *centroids);
  db.save_point_cloud("plane_normals", *normals);

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 480;
  opts.height = 360;
  opts.view = viz::ViewPreset::plan;

  const cv::Mat plan = render_or_skip(db, opts);
  const std::size_t ceiling =
      count_color(plan, kCeilingRgb[0], kCeilingRgb[1], kCeilingRgb[2]);
  const std::size_t floor =
      count_color(plan, kFloorRgb[0], kFloorRgb[1], kFloorRgb[2]);
  INFO("ceiling " << ceiling << ", floor " << floor);

  // 1.2 m above the *detected* floor (z = 0), not above the box floor
  // (z = -5), which would have put the cut at -2.8 and cut everything away.
  CHECK(ceiling == 0);
  CHECK(static_cast<double>(floor) >
        0.05 * static_cast<double>(opts.width * opts.height));
}

TEST_CASE("RenderView_MissingOrInvalidInputs_ThrowsWithDiagnosticMessage",
          "[integration][render]") {
  const reusex::test_support::TempDir work("reusex_render_errors");
  const fs::path project = work.path / "errors.rux";

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  // No cloud stored yet: the message must name the stage to run (STANDARDS §5).
  viz::RenderOptions opts;
  try {
    viz::render_view(db, opts);
    FAIL("rendering an empty project should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("cloud") != std::string::npos);
    CHECK(message.find("rux create clouds") != std::string::npos);
  }

  db.save_point_cloud("cloud", *synthetic_room(8));

  // A label layer whose cloud is absent names its own producing stage.
  viz::RenderOptions rooms_opts;
  rooms_opts.layers = {viz::Layer::rooms};
  try {
    viz::render_view(db, rooms_opts);
    FAIL("rendering a missing label layer should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("rux create rooms") != std::string::npos);
  }

  // Option validation happens before any GL work.
  viz::RenderOptions empty_layers;
  empty_layers.layers.clear();
  CHECK_THROWS_AS(viz::render_view(db, empty_layers), std::runtime_error);

  viz::RenderOptions bad_size;
  bad_size.width = 0;
  CHECK_THROWS_AS(viz::render_view(db, bad_size), std::runtime_error);

  viz::RenderOptions bad_orbit;
  bad_orbit.view = viz::ViewPreset::orbit;
  bad_orbit.orbit_count = 4;
  bad_orbit.orbit_index = 4; // one past the end of the ring
  CHECK_THROWS_AS(viz::render_view(db, bad_orbit), std::runtime_error);

  // Index-aligned contract: a stale label cloud is a hard error, not a
  // silently truncated render (STANDARDS §3.2).
  auto short_labels = std::make_shared<reusex::CloudL>();
  short_labels->resize(3);
  short_labels->width = 3;
  short_labels->height = 1;
  db.save_point_cloud("rooms", *short_labels);
  try {
    viz::render_view(db, rooms_opts);
    FAIL("a mismatched label cloud should have thrown");
  } catch (const std::runtime_error &e) {
    const std::string message = e.what();
    INFO("message: " << message);
    CHECK(message.find("out of sync") != std::string::npos);
  }
}

TEST_CASE("RenderView_RealScanFixture_ProducesNonBlankImage",
          "[integration][fixture][render]") {
  const auto fixture = fixture_path();
  if (!fs::exists(fixture)) {
    SKIP("Fixture missing: " << fixture
                             << " -- run `git lfs pull` to fetch it.");
  }
  if (is_lfs_pointer(fixture)) {
    SKIP("Fixture is an unexpanded git-lfs pointer: "
         << fixture << " -- run `git lfs pull` to fetch it.");
  }

  const reusex::test_support::TempDir work("reusex_render_fixture");
  const fs::path project = work.path / "office_corridor.rux";
  fs::copy_file(fixture, project);

  reusex::ProjectDB db(project);
  REQUIRE(db.is_open());

  // Same pinned parameters as tests/integration/test_real_scan_fixture.cpp, so
  // both tests describe the same reconstruction.
  reusex::geometry::ReconstructionParams params;
  params.resolution = 0.02F;
  params.min_distance = 0.0F;
  params.max_distance = 4.0F;
  params.sampling_factor = 1;
  params.confidence_threshold = 2;
  reusex::geometry::reconstruct_point_clouds(db, params);

  const auto cloud = db.point_cloud_xyzrgb("cloud");
  REQUIRE(cloud != nullptr);
  REQUIRE_FALSE(cloud->empty());

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud};
  opts.width = 640;
  opts.height = 480;

  const cv::Mat plan = render_or_skip(db, opts);
  REQUIRE(plan.cols == 640);
  REQUIRE(plan.rows == 480);

  // The corridor fills a good share of an orthographic plan view. 1 % is a
  // floor, not a target: the reference run covers far more, and anything at or
  // below this threshold means the scene did not reach the framebuffer.
  const double covered = foreground_fraction(plan, opts.background);
  INFO("foreground fraction: " << covered);
  CHECK(covered > 0.01);

  opts.view = viz::ViewPreset::orbit;
  opts.orbit_count = 8;
  opts.orbit_index = 1;
  const cv::Mat orbit = render_or_skip(db, opts);
  const double orbit_covered = foreground_fraction(orbit, opts.background);
  INFO("orbit foreground fraction: " << orbit_covered);
  CHECK(orbit_covered > 0.01);
}

// ── Mesh-vs-cloud QA views (reusex-8vx) ──────────────────────────────────────

namespace {

/// The coloured_room() box as a closed mesh, 2 triangles per face.
pcl::PolygonMesh room_mesh(float height = 2.7F) {
  pcl::PointCloud<pcl::PointXYZ> v;
  for (const float z : {0.0F, height})
    for (const auto &[x, y] : {std::pair{-2.0F, -2.0F}, std::pair{2.0F, -2.0F},
                               std::pair{2.0F, 2.0F}, std::pair{-2.0F, 2.0F}})
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

} // namespace

TEST_CASE("RenderView_MeshQaViews_RenderNonBlankAndDeterministic",
          "[integration][render][meshqa]") {
  const reusex::test_support::TempDir work("reusex_render_meshqa");
  reusex::ProjectDB db(work.path / "qa.rux");
  db.save_point_cloud("cloud", *coloured_room());
  db.save_mesh("mesh", room_mesh());

  viz::RenderOptions opts;
  opts.layers = {viz::Layer::cloud, viz::Layer::mesh};
  opts.width = 400;
  opts.height = 300;

  for (const auto view : {viz::ViewPreset::front, viz::ViewPreset::back,
                          viz::ViewPreset::left, viz::ViewPreset::right}) {
    opts.view = view;
    const cv::Mat image = render_or_skip(db, opts);
    INFO("view " << viz::to_string(view));
    CHECK(foreground_fraction(image, opts.background) > 0.01);
  }

  // A section through the middle of the room: walls in section, the mesh's
  // section line drawn in its edge colour (vermillion, unlit).
  opts.view = viz::ViewPreset::section;
  opts.section = viz::parse_section("x=0");
  const cv::Mat section = render_or_skip(db, opts);
  CHECK(foreground_fraction(section, opts.background) > 0.005);
  CHECK(count_color(section, 0xd5, 0x5e, 0x00) > 0);

  // The deviation layer: the cloud lies on the mesh, so every point is
  // within 5 cm, and two renders are identical (legend included).
  viz::RenderOptions dev;
  dev.layers = {viz::Layer::deviation};
  dev.view = viz::ViewPreset::plan;
  dev.width = 640;
  dev.height = 480;
  viz::RenderInfo info;
  const cv::Mat a = [&] {
    try {
      return viz::render_view(db, dev, &info);
    } catch (const viz::OffscreenGlUnavailable &e) {
      SKIP("No offscreen OpenGL on this machine: " << e.what());
    }
    return cv::Mat{};
  }();
  REQUIRE(info.deviation.has_value());
  CHECK(info.deviation->within_share == 1.0);
  CHECK(info.deviation->uncovered_share == 0.0);
  const cv::Mat b = render_or_skip(db, dev);
  cv::Mat difference;
  cv::absdiff(a, b, difference);
  CHECK(cv::countNonZero(difference.reshape(1)) == 0);
}
