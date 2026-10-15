// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Synthetic tests for slam::refine_frame_pair_icp (#465): a real ProjectDB
// with synthetic sensor frames whose geometry and relative pose are known, so
// the recovered transform can be checked within tolerance. The web GUI's
// IcpRefineFn (ruxd's own src/icp.cpp and test_icp.cpp, in its own repo) and
// the Qt client's pair strip both call this function.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <reusex/slam/frame_pair_icp.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>

#include <opencv2/core.hpp>

#include <sqlite3.h>

#include "../../support/temp_path.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

using reusex::test_support::TempPath;

namespace {

constexpr int kW = 160;
constexpr int kH = 120;

reusex::core::SensorIntrinsics make_intr() {
  reusex::core::SensorIntrinsics k;
  k.fx = k.fy = kW / 2.0;
  k.cx = kW / 2.0;
  k.cy = kH / 2.0;
  k.width = kW;
  k.height = kH;
  // local_transform stays identity (default)
  return k;
}

std::array<double, 16> identity_pose() {
  return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// Pose with a translation of tx in x (row-major 4×4).
std::array<double, 16> translate_pose(double tx) {
  return {1, 0, 0, tx, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
}

/// Depth image of a flat scene at z_mm millimetres everywhere.
cv::Mat flat_depth(uint16_t z_mm) {
  return cv::Mat(kH, kW, CV_16UC1, cv::Scalar(z_mm));
}

cv::Mat dummy_color() {
  return cv::Mat(kH, kW, CV_8UC3, cv::Scalar(128, 128, 128));
}

cv::Mat full_confidence() { return cv::Mat(kH, kW, CV_8UC1, cv::Scalar(255)); }

} // namespace

// ---------------------------------------------------------------------------

TEST_CASE("FramePairIcp_EmptyDepth_ThrowsOnRefine", "[slam][icp]") {
  // A frame with no depth stored → IcpRefineFn should throw (caller catches
  // and maps to 422).
  const TempPath project("gui_icp_nodepth");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    const auto intr = make_intr();
    // Save a frame with NO depth (empty Mat).
    db.save_sensor_frame(1, dummy_color(), cv::Mat{}, full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }

  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fn = [](const reusex::ProjectDB &d, int a, int b) {
    return reusex::slam::refine_frame_pair_icp(d, a, b);
  };

  CHECK_THROWS_AS(fn(db, 1, 2), std::runtime_error);
}

TEST_CASE("FramePairIcp_FlatPlane_ConvergesAndRecoversTranslation",
          "[slam][icp]") {
  // Frame A at origin, frame B translated 0.15 m in x. Both frames see a flat
  // plane at z = 1 m (1000 mm). The back-projected world-space clouds share
  // the same point positions in the overlap region, so ICP should converge to
  // T_delta ≈ identity and yield T_rel ≈ T_B^{-1} = translate(−0.15 m).
  const TempPath project("gui_icp_flatplane");
  const auto intr = make_intr();

  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }

  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto fn = [](const reusex::ProjectDB &d, int a, int b) {
    return reusex::slam::refine_frame_pair_icp(d, a, b);
  };
  const auto result = fn(db, 1, 2);

  // ICP must converge on this simple geometry.
  CHECK(result.converged);

  // RMS error should be very small (< 5 cm) when clouds perfectly overlap.
  CHECK(result.fitness < 0.05);

  // Most source points should match the target within 5 cm.
  CHECK(result.inlier_fraction > 0.5);

  // Relative pose: element [0][3] (= relative_pose[3]) is the x-translation
  // in the "to" frame's coordinate system. Frame A is 0.15 m behind B in x.
  CHECK(std::abs(result.relative_pose[3] - (-0.15)) < 0.03);

  // Diagonal should be close to 1 (rotation near identity).
  CHECK(std::abs(result.relative_pose[0] - 1.0) < 0.05);
  CHECK(std::abs(result.relative_pose[5] - 1.0) < 0.05);
  CHECK(std::abs(result.relative_pose[10] - 1.0) < 0.05);
}

TEST_CASE("FramePairIcp_SameFrame_IsRejected", "[slam][icp]") {
  const TempPath project("frame_pair_icp_same");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), make_intr(), 0.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  CHECK_THROWS_AS(reusex::slam::refine_frame_pair_icp(db, 1, 1),
                  std::invalid_argument);
}

TEST_CASE("FramePairIcp_ReportsPointCountsAndIdentityWorldDelta",
          "[slam][icp]") {
  const TempPath project("frame_pair_icp_counts");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), make_intr(), 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), make_intr(), 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = reusex::slam::refine_frame_pair_icp(db, 1, 2);
  // 160x120 at a 4 px step = 40 x 30 samples, all at 1 m.
  CHECK(r.source_points == 1200);
  CHECK(r.target_points == 1200);
  // The stored poses already agree on the plane: the world correction is
  // (near) identity in translation along the plane normal.
  CHECK(std::abs(r.world_delta[11]) < 0.01);
  // Nothing to correct along the normal, and no rotation.
  CHECK(r.rotation_deg < 1.0);
  CHECK(r.converged);
}

TEST_CASE("FramePairIcp_ShiftedFrame_ReportsTheCameraCentreShift",
          "[slam][icp]") {
  // The same 1 m plane, but frame 2's stored pose claims it is 5 cm further
  // from the plane than it is (its depth still reads 1 m): ICP must pull its
  // cloud back 5 cm along z — a pure translation of the camera centre, with
  // the frames far from the world origin so a lever arm would show.
  const TempPath project("frame_pair_icp_shift");
  auto far = [](double tx, double tz) {
    return std::array<double, 16>{1, 0, 0, 7.0 + tx, 0, 1, 0, 3.0,
                                  0, 0, 1, tz,       0, 0, 0, 1};
  };
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         far(0.0, 0.0), make_intr(), 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         far(0.0, -0.05), make_intr(), 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = reusex::slam::refine_frame_pair_icp(db, 2, 1);
  CHECK(r.converged);
  CHECK(r.rotation_deg < 1.0);
  CHECK(r.source_center_shift_m == Catch::Approx(0.05).margin(0.01));
  CHECK(r.world_delta[11] == Catch::Approx(0.05).margin(0.01));
}

TEST_CASE("FramePairIcp_InitialGuess_IsTheStartingPoint", "[slam][icp]") {
  // Both frames at the identity pose; "from" sees a plane at 1.0 m, "to" at
  // 1.1 m, so the correct world correction of "from" is +0.1 m in z. With a
  // 2 cm correspondence window ICP cannot find it from identity, but starts
  // right on it from the guess.
  const TempPath project("frame_pair_icp_guess");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1100), full_confidence(),
                         identity_pose(), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);

  reusex::slam::FramePairIcpOptions o;
  o.max_correspondence_m = 0.02;
  o.max_iterations = 5;
  const auto no_guess = reusex::slam::refine_frame_pair_icp(db, 1, 2, o);
  CHECK(std::abs(no_guess.world_delta[11]) < 0.01);

  o.initial_guess =
      std::array<double, 16>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.1, 0, 0, 0, 1};
  const auto guessed = reusex::slam::refine_frame_pair_icp(db, 1, 2, o);
  CHECK(guessed.converged);
  CHECK(std::abs(guessed.world_delta[11] - 0.1) < 0.01);
  CHECK(guessed.fitness < 0.03);
  // edge (1 -> 2) = (delta * T_1)^-1 * T_2 = translate(0, 0, -0.1)
  CHECK(std::abs(guessed.edge_transform[11] + 0.1) < 0.01);
}

TEST_CASE("FramePairIcp_EdgeTransform_IsInverseOfRelativePose", "[slam][icp]") {
  const TempPath project("frame_pair_icp_edge_transform");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = reusex::slam::refine_frame_pair_icp(db, 1, 2);
  Eigen::Matrix4d rel, edge;
  for (int i = 0; i < 16; ++i) {
    rel(i / 4, i % 4) = r.relative_pose[static_cast<size_t>(i)];
    edge(i / 4, i % 4) = r.edge_transform[static_cast<size_t>(i)];
  }
  CHECK((edge - rel.inverse()).norm() < 1e-9);

  const auto info = reusex::slam::icp_edge_information(r);
  for (int k = 0; k < 6; ++k)
    CHECK(info[static_cast<size_t>(k * 6 + k)] > 0.0);
  CHECK(info[1] == 0.0); // diagonal
  // translation precision = 1 / max(fitness, 0.01)^2
  const double st = std::max(r.fitness, 0.01);
  CHECK(info[21] == Catch::Approx(1.0 / (st * st)));
}

TEST_CASE("FrameWorldPoints_IsTheBackProjectedDepthInWorld", "[slam][icp]") {
  const TempPath project("frame_world_points");
  const auto intr = make_intr();
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), intr, 0.0);
    db.save_sensor_frame(2, dummy_color(), flat_depth(1000), full_confidence(),
                         translate_pose(0.15), intr, 1.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto a = reusex::slam::frame_world_points(db, 1);
  const auto b = reusex::slam::frame_world_points(db, 2);
  // pixel_step 4 over 160x120: 40 x 30 samples, all at the 1 m plane.
  REQUIRE(a.size() == 1200);
  REQUIRE(b.size() == a.size());
  for (std::size_t i = 0; i < a.size(); i += 97) {
    CHECK(a[i][2] == Catch::Approx(1.0f));
    CHECK(b[i][0] - a[i][0] == Catch::Approx(0.15f).margin(1e-5));
  }
}

TEST_CASE("FrameWorldPoints_NoDepth_Throws", "[slam][icp]") {
  const TempPath project("frame_world_points_nodepth");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), cv::Mat{}, full_confidence(),
                         identity_pose(), make_intr(), 0.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  CHECK_THROWS_AS(reusex::slam::frame_world_points(db, 1), std::runtime_error);
}

// ---------------------------------------------------------------------------
// frame_world_colored_points (reusex-xlg.10): the overlay's RGB mode.

namespace {

/// A smooth colour gradient (JPEG keeps it close; the tests compare with the
/// decoded stored image anyway): B follows u, G follows v, R is constant.
cv::Mat gradient_color(int w, int h) {
  cv::Mat img(h, w, CV_8UC3);
  for (int v = 0; v < h; ++v)
    for (int u = 0; u < w; ++u)
      img.at<cv::Vec3b>(v, u) =
          cv::Vec3b(static_cast<uint8_t>(u * 255 / (w - 1)),
                    static_cast<uint8_t>(v * 255 / (h - 1)), 200);
  return img;
}

void exec_raw(const std::string &db_path, const std::string &sql) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(db_path.c_str(), &raw) == SQLITE_OK);
  char *err = nullptr;
  const int rc = sqlite3_exec(raw, sql.c_str(), nullptr, nullptr, &err);
  const std::string message = err ? err : "";
  sqlite3_free(err);
  sqlite3_close(raw);
  INFO(message);
  REQUIRE(rc == SQLITE_OK);
}

/// The RGB the stored (decoded, BGR) image holds at (u, v).
std::array<uint8_t, 3> stored_rgb(const cv::Mat &bgr, int u, int v) {
  const cv::Vec3b p = bgr.at<cv::Vec3b>(v, u);
  return {p[2], p[1], p[0]};
}

} // namespace

TEST_CASE("FrameWorldColoredPoints_ColourIsTheSourcePixel", "[slam][icp]") {
  const TempPath project("frame_world_colored_points");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, gradient_color(kW, kH), flat_depth(1000),
                         full_confidence(), identity_pose(), make_intr(), 0.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto cloud = reusex::slam::frame_world_colored_points(db, 1);
  const auto plain = reusex::slam::frame_world_points(db, 1);
  // Same sampling as ICP: the points are frame_world_points', 1:1.
  REQUIRE(cloud.points == plain);
  REQUIRE(cloud.colors.size() == cloud.points.size());

  const cv::Mat stored = db.sensor_frame_image(1);
  REQUIRE(stored.cols == kW);
  // pixel_step 4 over 160x120: raster order, 40 samples a row.
  for (std::size_t i = 0; i < cloud.points.size(); i += 37) {
    const int u = static_cast<int>(i % 40) * 4;
    const int v = static_cast<int>(i / 40) * 4;
    INFO("i=" << i << " u=" << u << " v=" << v);
    CHECK(cloud.colors[i] == stored_rgb(stored, u, v));
  }
  // The gradient really varies, so a wrong pixel would show.
  CHECK(cloud.colors.front() != cloud.colors.back());
}

TEST_CASE("FrameWorldColoredPoints_HalfResolutionDepth_ScalesToColour",
          "[slam][icp]") {
  // iPad-style capture: depth at half the colour resolution; intrinsics at
  // the colour resolution. Depth pixel (u, v) is colour pixel (2u, 2v).
  const TempPath project("frame_world_colored_points_halfres");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, gradient_color(kW, kH),
                         cv::Mat(kH / 2, kW / 2, CV_16UC1, cv::Scalar(1000)),
                         cv::Mat(kH / 2, kW / 2, CV_8UC1, cv::Scalar(255)),
                         identity_pose(), make_intr(), 0.0);
  }
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto cloud = reusex::slam::frame_world_colored_points(db, 1);
  // 80x60 depth, step 4: 20 x 15 samples.
  REQUIRE(cloud.points.size() == 300);
  REQUIRE(cloud.colors.size() == 300);
  const cv::Mat stored = db.sensor_frame_image(1);
  for (std::size_t i = 0; i < cloud.points.size(); i += 7) {
    const int u = static_cast<int>(i % 20) * 4;
    const int v = static_cast<int>(i / 20) * 4;
    INFO("i=" << i << " depth u=" << u << " v=" << v);
    CHECK(cloud.colors[i] == stored_rgb(stored, 2 * u, 2 * v));
  }
  // The far corner samples the far half of the colour image, not its
  // top-left quarter.
  CHECK(cloud.colors.back() == stored_rgb(stored, 2 * 76, 2 * 56));
}

TEST_CASE("FrameWorldColoredPoints_NoColourImage_KeepsPointsWithoutColours",
          "[slam][icp]") {
  const TempPath project("frame_world_colored_points_nocolor");
  {
    reusex::ProjectDB db(project.path, /*readOnly=*/false);
    db.save_sensor_frame(1, dummy_color(), flat_depth(1000), full_confidence(),
                         identity_pose(), make_intr(), 0.0);
  }
  exec_raw(project.path.string(),
           "UPDATE sensor_frames SET color = NULL WHERE node_id = 1;");
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto cloud = reusex::slam::frame_world_colored_points(db, 1);
  CHECK(cloud.points.size() == 1200);
  CHECK(cloud.colors.empty()); // the caller falls back to a tint
}

// ---------------------------------------------------------------------------
// Low-structure pairs (reusex-xlg.12): a wall with one small box on it, seen
// by two cameras whose stored poses are CORRECT but whose views only partly
// overlap. Point-to-point ICP with the 0.5 m window paired B's points past
// the overlap with A's border points and dragged B along the wall (NewOffice
// 2240/2241: 22 cm / 5 deg off a 1.6 cm match fit). ICP must stay put here.

namespace {

enum class Feature { none, small_box, large_box };

/// Depth (mm) a camera at world x = @p cam_x (looking down +z, identity
/// rotation) sees of the plane z = 2 m with an axis-aligned box standing on
/// it: small x in [0.45, 0.9], y in [-0.3, 0.3], z in [1.8, 2.0] (~2% of
/// the view, its side faces nearly edge-on), or large x in [0.5, 1.3],
/// y in [0.3, 1.0], z in [1.2, 2.0], off-axis so that both cameras see its
/// x = 0.5 and y = 0.3 faces.
cv::Mat wall_with_box_depth(double cam_x, Feature feature) {
  cv::Mat d(kH, kW, CV_16UC1);
  const double f = kW / 2.0, cx = kW / 2.0, cy = kH / 2.0;
  const bool with_box = feature != Feature::none;
  const bool large = feature == Feature::large_box;
  const Eigen::Vector3d lo =
      large ? Eigen::Vector3d(0.5, 0.3, 1.2) : Eigen::Vector3d(0.45, -0.3, 1.8);
  const Eigen::Vector3d hi =
      large ? Eigen::Vector3d(1.3, 1.0, 2.0) : Eigen::Vector3d(0.9, 0.3, 2.0);
  for (int v = 0; v < kH; ++v)
    for (int u = 0; u < kW; ++u) {
      const Eigen::Vector3d o(cam_x, 0, 0);
      const Eigen::Vector3d dir((u - cx) / f, (v - cy) / f, 1.0);
      double z = 2.0; // the wall: dir.z == 1, so t == depth
      if (with_box) {
        double t0 = 0, t1 = 1e9;
        bool hit = true;
        for (int k = 0; k < 3 && hit; ++k) {
          if (std::abs(dir[k]) < 1e-12) {
            hit = o[k] >= lo[k] && o[k] <= hi[k];
            continue;
          }
          double a = (lo[k] - o[k]) / dir[k], b = (hi[k] - o[k]) / dir[k];
          if (a > b)
            std::swap(a, b);
          t0 = std::max(t0, a);
          t1 = std::min(t1, b);
          hit = t0 <= t1;
        }
        if (hit)
          z = std::min(z, t0);
      }
      d.at<uint16_t>(v, u) = static_cast<uint16_t>(std::lround(z * 1000.0));
    }
  return d;
}

/// Frames 1 (camera at x = 0) and 2 (camera at x = @p b_x), poses stored
/// exactly; frame 2's stored pose may be off by @p b_err in x.
void save_wall_pair(const TempPath &project, double b_x, double b_err,
                    Feature feature) {
  reusex::ProjectDB db(project.path, /*readOnly=*/false);
  db.save_sensor_frame(1, dummy_color(), wall_with_box_depth(0.0, feature),
                       full_confidence(), identity_pose(), make_intr(), 0.0);
  db.save_sensor_frame(2, dummy_color(), wall_with_box_depth(b_x, feature),
                       full_confidence(), translate_pose(b_x + b_err),
                       make_intr(), 1.0);
}

} // namespace

TEST_CASE("FramePairIcp_PartialOverlapWallWithBox_DoesNotSlide",
          "[slam][icp]") {
  const TempPath project("frame_pair_icp_wall_box");
  save_wall_pair(project, 1.0, 0.0, Feature::small_box);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);

  // Unseeded and seeded with the (correct) identity: the same answer.
  for (const bool seeded : {false, true}) {
    INFO("seeded=" << seeded);
    reusex::slam::FramePairIcpOptions o;
    if (seeded)
      o.initial_guess = identity_pose();
    const auto r = reusex::slam::refine_frame_pair_icp(db, 2, 1, o);
    CHECK(r.converged);
    CHECK(r.source_center_shift_m < 0.01);
    CHECK(r.rotation_deg < 0.5);
    CHECK(r.seed_shift_m < 0.01);
    CHECK_FALSE(r.worse_than_seed);
    // The seed was already right: ICP does not make the overlap worse.
    CHECK(r.inlier_fraction >= r.seed_inlier_fraction - 1e-9);
    CHECK(r.inlier_fraction > 0.6);
  }
}

TEST_CASE("FramePairIcp_PureWall_LeavesTheUnobservableDirectionsAlone",
          "[slam][icp]") {
  // A bare wall constrains only its normal and the two tilts: B's stored
  // pose is 8 cm off ALONG the wall, which no depth can see. ICP must not
  // invent a slide (nor blow up on the singular normal equations), and it
  // reports the three directions it could not constrain.
  const TempPath project("frame_pair_icp_pure_wall");
  save_wall_pair(project, 0.5, 0.08, Feature::none);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  const auto r = reusex::slam::refine_frame_pair_icp(db, 2, 1);
  for (double v : r.world_delta)
    REQUIRE(std::isfinite(v));
  CHECK(r.converged);
  CHECK(r.source_center_shift_m < 0.01);
  CHECK(r.rotation_deg < 0.5);
  CHECK(r.unconstrained_dofs == 3);
  // B's view reaches 0.5 m past A's: the rest lies on the wall A sees.
  CHECK(r.inlier_fraction > 0.8);
  CHECK(r.inlier_rms_m < 0.005);
}

TEST_CASE("FramePairIcp_WallWithBox_RecoversAnInPlaneOffset", "[slam][icp]") {
  // A box that sticks out far enough makes the in-plane directions
  // observable again: B's stored pose is 6 cm off along the wall and ICP
  // finds it (world_delta pulls B's cloud back by -6 cm in x). Leaving the
  // unconstrained directions alone must not freeze the observed ones.
  const TempPath project("frame_pair_icp_box_offset");
  save_wall_pair(project, 0.3, 0.06, Feature::large_box);
  reusex::ProjectDB db(project.path, /*readOnly=*/true);
  reusex::slam::FramePairIcpOptions o;
  o.pixel_step = 1; // the box's side faces are seen obliquely: sample them
  o.max_correspondence_m = 0.1;
  const auto r = reusex::slam::refine_frame_pair_icp(db, 2, 1, o);
  CHECK(r.converged);
  // The slide along the wall is constrained now (the turn about the wall's
  // normal, pinned only by the box's edges, may still be dropped).
  CHECK(r.unconstrained_dofs < 3);
  CHECK(r.world_delta[3] == Catch::Approx(-0.06).margin(0.015));
  CHECK(r.rotation_deg < 1.0);
  CHECK(r.inlier_fraction > r.seed_inlier_fraction);
}

TEST_CASE("FramePairIcp_WorseThanSeed_FlagsAFarMoveThatLosesOverlap",
          "[slam][icp]") {
  using reusex::slam::FramePairIcpOptions;
  using reusex::slam::FramePairIcpResult;
  using reusex::slam::icp_worse_than_seed;
  const FramePairIcpOptions o; // inlier gate 5 cm
  FramePairIcpResult r;
  r.seed_inlier_fraction = 0.60;
  r.seed_inlier_rms_m = 0.017;
  r.inlier_fraction = 0.59;
  r.inlier_rms_m = 0.024;

  // Moved far (22 cm, 5 deg) and fit the overlap worse: flagged.
  r.seed_shift_m = 0.22;
  r.seed_rotation_deg = 5.0;
  CHECK(icp_worse_than_seed(r, o));
  // A far move that IMPROVES the overlap is a correction, not a drift.
  r.inlier_fraction = 0.70;
  r.inlier_rms_m = 0.015;
  CHECK_FALSE(icp_worse_than_seed(r, o));
  // A small move is never flagged, however the numbers wobble.
  r.inlier_fraction = 0.59;
  r.inlier_rms_m = 0.024;
  r.seed_shift_m = 0.01;
  r.seed_rotation_deg = 0.3;
  CHECK_FALSE(icp_worse_than_seed(r, o));
  // A rotation alone can be far.
  r.seed_rotation_deg = 4.0;
  CHECK(icp_worse_than_seed(r, o));
}
