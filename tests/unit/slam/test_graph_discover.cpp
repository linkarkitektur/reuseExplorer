// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph discover` (spec 2026-10-13 §4.3), driven through the external
// edge-file source so no image matching is needed.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/pose_graph.hpp>
#include <reusex/slam/graph_discover.hpp>
#include <reusex/slam/pose_graph.hpp>
#include <reusex/slam/pose_graph_conventions.hpp>

#include "../../support/pose_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::test_support;
namespace pg = reusex::core::pose_graph;
namespace slam = reusex::slam;

namespace {

Eigen::Matrix4d truth(int k) {
  return pg::matrix_from_row_major(translation_pose(0.5 * k, 0, 0));
}
Eigen::Matrix4d rel(int a, int b) { return truth(a).inverse() * truth(b); }

void make_project(ProjectDB &db, int n = 8) {
  for (int k = 1; k <= n; ++k)
    db.save_sensor_frame(k, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
                         pg::row_major_from_matrix(truth(k)),
                         make_intrinsics(8, 8, 4, 4, 8, 8), k);
}

void write_edges(const std::filesystem::path &path,
                 const std::vector<std::tuple<int, int, Eigen::Matrix4d>> &es) {
  nlohmann::json j;
  j["schema"] = "reusex.loop_edges.v1";
  j["producer"] = "test";
  j["edges"] = nlohmann::json::array();
  for (const auto &[a, b, T] : es) {
    nlohmann::json e;
    e["node_i"] = a;
    e["node_j"] = b;
    std::vector<double> t;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        t.push_back(T(r, c));
    e["T_ij"] = t;
    e["sigma_rot"] = 0.05;
    e["sigma_trans"] = 0.1;
    e["inliers"] = 100;
    j["edges"].push_back(e);
  }
  std::ofstream(path) << j.dump();
}

slam::GraphDiscoverOptions file_only(const std::filesystem::path &file) {
  slam::GraphDiscoverOptions o;
  o.candidates.loop_closure.enable = false;
  o.candidates.loop_edges_file = file.string();
  o.candidates.loop_edges_min_seed_disagreement = 0.0;
  o.candidates.loop_edges_min_seed_disagreement_fraction = 0.0;
  return o;
}

void store_loop(ProjectDB &db, int a, int b, const Eigen::Matrix4d &T,
                const char *status, const char *type = "loop_closure") {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = a;
  e.to_node_id = b;
  e.edge_type = type;
  e.transform = pg::row_major_from_matrix(T);
  e.source = "manual";
  e.status = "auto";
  REQUIRE(db.upsert_pose_graph_edge(e));
  if (std::string(status) != "auto")
    REQUIRE(db.set_pose_graph_edge_status(a, b, type, status));
}

} // namespace

TEST_CASE("GraphDiscover_FileEdges_SkipStoredAndTombstonedPairs",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_skip");
  TempPath file("test_discover_skip", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  store_loop(db, 2, 7, rel(2, 7), "auto");
  store_loop(db, 1, 7, rel(1, 7), "rejected");
  write_edges(file.path, {{1, 7, rel(1, 7)},
                          {7, 2, rel(7, 2)},
                          {1, 8, rel(1, 8)},
                          {2, 8, rel(2, 8)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.skipped_rejected == 1);
  CHECK(r.skipped_existing == 1);
  CHECK(r.pcm_rejected == 0);
  CHECK(r.added == 2);
  CHECK(r.added_file == 2);
  CHECK(db.pose_graph_edge(1, 7, "loop_closure")->status == "rejected");
  for (auto [a, b] : {std::pair{1, 8}, std::pair{2, 8}}) {
    const auto e = db.pose_graph_edge(a, b, "loop_closure");
    REQUIRE(e.has_value());
    CHECK(e->source == "file");
    CHECK(e->status == "auto");
    CHECK(e->information.has_value());
    CHECK(*e->seed_disagreement_m < 1e-9);
    CHECK((pg::matrix_from_row_major(*e->transform) - rel(a, b)).norm() < 1e-9);
  }
}

TEST_CASE("GraphDiscover_ConfirmedStoredEdge_IsAFixedPcmInlier",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_fixed");
  TempPath file("test_discover_fixed", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0; // three mutually consistent aliasing edges
  write_edges(file.path, {{1, 4, rel(1, 4) * off},
                          {2, 5, rel(2, 5) * off},
                          {3, 6, rel(3, 6) * off}});

  SECTION("with a confirmed true edge they are all rejected") {
    store_loop(db, 1, 5, rel(1, 5), "confirmed");
    const auto r = slam::discover_loop_edges(db, file_only(file.path));
    CHECK(r.pcm_rejected == 3);
    CHECK(r.added == 0);
    CHECK(db.pose_graph_edge(1, 5, "loop_closure")->status == "confirmed");
  }
  SECTION("without it they agree with each other and are added") {
    const auto r = slam::discover_loop_edges(db, file_only(file.path));
    CHECK(r.pcm_rejected == 0);
    CHECK(r.added == 3);
  }
}

TEST_CASE("GraphDiscover_DryRun_WritesNothing", "[slam][graph_discover]") {
  TempPath tmp("test_discover_dry");
  TempPath file("test_discover_dry", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  write_edges(file.path, {{1, 8, rel(1, 8)}, {2, 8, rel(2, 8)}});
  auto o = file_only(file.path);
  o.dry_run = true;
  const auto r = slam::discover_loop_edges(db, o);
  CHECK(r.candidates == 2);
  CHECK(r.added == 0);
  CHECK_FALSE(r.initialised_graph);
  CHECK(db.list_pose_graph_edges().empty());
}

TEST_CASE("GraphDiscover_NoGraph_InitialisesOdometryFirst",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_init");
  TempPath file("test_discover_init", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  write_edges(file.path, {{1, 8, rel(1, 8)}});
  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.initialised_graph);
  CHECK(r.odometry_edges_written == 7);
  CHECK(slam::count_usable_odometry_edges(db.list_pose_graph_edges()) == 7);
  CHECK(r.added == 1);
}

TEST_CASE("GraphDiscover_NoSource_IsInvalid", "[slam][graph_discover]") {
  TempPath tmp("test_discover_nosource");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::GraphDiscoverOptions o;
  o.candidates.loop_closure.enable = false;
  CHECK_THROWS_AS(slam::discover_loop_edges(db, o), std::invalid_argument);
}

TEST_CASE("GraphDiscover_SkipIsKeyedByEdgeType", "[slam][graph_discover]") {
  // Ruling R10 / spec §4.3 "skips any key that already exists": the key is
  // (pair, edge_type), so a stored panorama edge — even a tombstoned one —
  // does not block a loop_closure on the same pair.
  TempPath tmp("test_discover_type_key");
  TempPath file("test_discover_type_key", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  store_loop(db, 1, 8, rel(1, 8), "auto", "panorama");
  store_loop(db, 2, 8, rel(2, 8), "rejected", "panorama");
  write_edges(file.path, {{1, 8, rel(1, 8)}, {2, 8, rel(2, 8)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.skipped_existing == 0);
  CHECK(r.skipped_rejected == 0);
  CHECK(r.added == 2);
  CHECK(db.pose_graph_edge(1, 8, "loop_closure").has_value());
  CHECK(db.pose_graph_edge(2, 8, "loop_closure").has_value());
  CHECK(db.pose_graph_edge(2, 8, "panorama")->status == "rejected");
}

TEST_CASE("GraphDiscover_SingleCandidateContradictingAConfirmedEdge_"
          "IsRejected",
          "[slam][graph_discover]") {
  // Ruling R9: two edges in the PCM set (one confirmed, one candidate) still
  // run PCM, because the confirmed edge is a fixed inlier.
  TempPath tmp("test_discover_fixed_small");
  TempPath file("test_discover_fixed_small", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  store_loop(db, 1, 5, rel(1, 5), "confirmed");
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  write_edges(file.path, {{2, 6, rel(2, 6) * off}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.pcm_rejected == 1);
  CHECK(r.added == 0);
  CHECK_FALSE(db.pose_graph_edge(2, 6, "loop_closure").has_value());
}

TEST_CASE("GraphDiscover_InconsistentStoredAutoEdge_IsOnlyReported",
          "[slam][graph_discover]") {
  TempPath tmp("test_discover_stored_bad");
  TempPath file("test_discover_stored_bad", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  store_loop(db, 1, 6, rel(1, 6) * off, "auto"); // aliasing, stored
  write_edges(file.path,
              {{1, 5, rel(1, 5)}, {2, 6, rel(2, 6)}, {3, 7, rel(3, 7)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.stored_inconsistent == 1);
  CHECK(r.added == 3);
  const auto kept = db.pose_graph_edge(1, 6, "loop_closure");
  REQUIRE(kept.has_value());
  CHECK(kept->status == "auto");
  CHECK((pg::matrix_from_row_major(*kept->transform) - rel(1, 6) * off).norm() <
        1e-9);
}

TEST_CASE("GraphDiscover_PcmCapCountsNewCandidatesOnly",
          "[slam][graph_discover]") {
  // Three stored (true) loop edges exceed pcm_max_edges = 2. They must all
  // take part in PCM, none may be reported inconsistent, and the cap may only
  // cut NEW candidates — and those are counted as capped, not rejected.
  TempPath tmp("test_discover_cap");
  TempPath file("test_discover_cap", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  store_loop(db, 1, 5, rel(1, 5), "auto");
  store_loop(db, 2, 6, rel(2, 6), "auto");
  store_loop(db, 3, 7, rel(3, 7), "auto");
  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 2.0;
  write_edges(file.path, {{4, 8, rel(4, 8) * off}, // aliasing: tested, out
                          {1, 8, rel(1, 8)},       // true: tested, kept
                          {2, 8, rel(2, 8)}});     // over the cap
  auto o = file_only(file.path);
  o.candidates.loop_closure.pcm_max_edges = 2;

  const auto r = slam::discover_loop_edges(db, o);
  CHECK(r.stored_inconsistent == 0);
  CHECK(r.pcm_capped == 1);
  CHECK(r.pcm_rejected == 1);
  CHECK(r.added == 1);
  CHECK(db.pose_graph_edge(1, 8, "loop_closure").has_value());
  CHECK_FALSE(db.pose_graph_edge(4, 8, "loop_closure").has_value());
  CHECK_FALSE(db.pose_graph_edge(2, 8, "loop_closure").has_value());
}

TEST_CASE("GraphDiscover_NonIdentityLocalTransforms_RoundTrip",
          "[slam][graph_discover]") {
  // Every frame gets a DIFFERENT local (body -> optical) transform, so a
  // swapped local_i / local_j in the stored <-> optical conversions breaks
  // both the confirmed stored edge's PCM role and the written measurement.
  TempPath tmp("test_discover_local");
  TempPath file("test_discover_local", ".json");
  ProjectDB db(tmp.path);
  auto local = [](int k) {
    Eigen::Matrix4d L = Eigen::Matrix4d::Identity();
    L.block<3, 3>(0, 0) =
        (Eigen::AngleAxisd(0.3 * k, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    L.block<3, 1>(0, 3) = Eigen::Vector3d(0.05 * k, -0.02, 0.1);
    return L;
  };
  for (int k = 1; k <= 8; ++k)
    db.save_sensor_frame(
        k, make_color(8, 8), make_depth(8, 8, 1.0), cv::Mat(),
        pg::row_major_from_matrix(truth(k)),
        make_intrinsics(8, 8, 4, 4, 8, 8, pg::row_major_from_matrix(local(k))),
        k);
  slam::init_pose_graph(db);
  store_loop(db, 1, 5, rel(1, 5), "confirmed");
  auto optical = [&](int a, int b) {
    return Eigen::Matrix4d((truth(a) * local(a)).inverse() *
                           (truth(b) * local(b)));
  };
  write_edges(file.path, {{2, 6, optical(2, 6)}, {3, 7, optical(3, 7)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.pcm_rejected == 0);
  CHECK(r.added == 2);
  for (auto [a, b] : {std::pair{2, 6}, std::pair{3, 7}}) {
    const auto e = db.pose_graph_edge(a, b, "loop_closure");
    REQUIRE(e.has_value());
    CHECK((pg::matrix_from_row_major(*e->transform) - rel(a, b)).norm() < 1e-9);
    CHECK(*e->seed_disagreement_m < 1e-9);
    const auto expected = slam::information_optical_to_stored(
        pg::information_from_sigmas(0.05, 0.1), local(b));
    REQUIRE(e->information.has_value());
    double diff = 0.0;
    for (int r6 = 0; r6 < 6; ++r6)
      for (int c6 = 0; c6 < 6; ++c6)
        diff = std::max(
            diff, std::abs((*e->information)[r6 * 6 + c6] - expected(r6, c6)) /
                      (1.0 + std::abs(expected(r6, c6))));
    CHECK(diff < 1e-9);
  }
}

TEST_CASE("GraphDiscover_ActiveRowWithoutMeasurement_IsFilled",
          "[slam][graph_discover]") {
  // Final review I2: a legacy (pre-v27) or GUI-added loop row has no
  // transform. A candidate on its key fills the measurement and the row
  // keeps its status; a tombstone still blocks.
  TempPath tmp("test_discover_fill");
  TempPath file("test_discover_fill", ".json");
  ProjectDB db(tmp.path);
  make_project(db);
  slam::init_pose_graph(db);
  auto bare = [&](int a, int b, const char *source, const char *status) {
    ProjectDB::PoseGraphEdge e;
    e.from_node_id = a;
    e.to_node_id = b;
    e.edge_type = "loop_closure";
    e.source = source;
    e.status = "auto";
    REQUIRE(db.upsert_pose_graph_edge(e));
    if (std::string(status) != "auto")
      REQUIRE(db.set_pose_graph_edge_status(a, b, "loop_closure", status));
  };
  bare(1, 7, "legacy", "auto");
  bare(2, 7, "manual", "confirmed");
  bare(3, 8, "legacy", "rejected");
  write_edges(file.path, {{1, 7, rel(1, 7)},
                          {7, 2, rel(7, 2)},
                          {3, 8, rel(3, 8)},
                          {1, 8, rel(1, 8)}});

  const auto r = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(r.filled == 2);
  CHECK(r.added == 1);
  CHECK(r.skipped_existing == 0);
  CHECK(r.skipped_rejected == 1);
  for (auto [a, b, status] :
       {std::tuple{1, 7, "auto"}, std::tuple{2, 7, "confirmed"}}) {
    const auto e = db.pose_graph_edge(a, b, "loop_closure");
    REQUIRE(e.has_value());
    CHECK(e->status == status);
    CHECK(e->source == "file");
    REQUIRE(e->transform.has_value());
    CHECK((pg::matrix_from_row_major(*e->transform) - rel(a, b)).norm() < 1e-9);
    CHECK(e->information.has_value());
  }
  const auto tomb = db.pose_graph_edge(3, 8, "loop_closure");
  CHECK(tomb->status == "rejected");
  CHECK_FALSE(tomb->transform.has_value());

  // A second run finds every key measured: nothing more is filled.
  const auto again = slam::discover_loop_edges(db, file_only(file.path));
  CHECK(again.filled == 0);
  CHECK(again.added == 0);
  CHECK(again.skipped_existing == 3);
}
