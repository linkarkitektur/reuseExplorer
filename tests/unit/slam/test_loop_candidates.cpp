// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// slam::collect_loop_candidates — the ORB / panorama / file front-ends moved
// out of optimize_sensor_poses (spec §4.3). The file source runs without
// images, so it carries the unit test; ORB and panorama keep their own tests.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/loop_candidates.hpp>

#include "../../support/temp_path.hpp"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <fstream>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace slam = reusex::slam;

namespace {

std::vector<Eigen::Matrix4d> seeds_along_x(int n) {
  std::vector<Eigen::Matrix4d> s;
  for (int k = 0; k < n; ++k) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = 0.1 * k;
    s.push_back(T);
  }
  return s;
}

nlohmann::json edge_json(int ni, int nj, const Eigen::Matrix4d &T) {
  nlohmann::json e;
  e["node_i"] = ni;
  e["node_j"] = nj;
  std::vector<double> t;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      t.push_back(T(r, c));
  e["T_ij"] = t;
  e["sigma_rot"] = 0.05;
  e["sigma_trans"] = 0.1;
  e["inliers"] = 100;
  return e;
}

} // namespace

TEST_CASE("LoopCandidates_FileSource_GatesAndTagsEdges",
          "[slam][loop_candidates]") {
  TempPath project("test_loop_candidates");
  TempPath file("test_loop_candidates", ".json");
  ProjectDB db(project.path);
  const std::vector<int> ids = {10, 11, 12, 13, 14, 15, 16, 17};
  const auto seeds = seeds_along_x(8);

  Eigen::Matrix4d off = Eigen::Matrix4d::Identity();
  off(0, 3) = 0.5;
  nlohmann::json j;
  j["schema"] = "reusex.loop_edges.v1";
  j["producer"] = "test";
  j["edges"] = {
      edge_json(10, 17, seeds[0].inverse() * seeds[7]),        // agrees
      edge_json(11, 16, seeds[1].inverse() * seeds[6] * off)}; // 0.5 m off
  std::ofstream(file.path) << j.dump();

  slam::LoopCandidateOptions o;
  o.loop_closure.enable = false;
  o.loop_edges_file = file.path.string();
  o.loop_edges_min_seed_disagreement = 0.2;
  o.loop_edges_min_seed_disagreement_fraction = 0.0;

  slam::LoopCandidateReport rep;
  const auto c = slam::collect_loop_candidates(db, ids, seeds, o, &rep);
  REQUIRE(c.size() == 1);
  CHECK(c[0].source == "file");
  CHECK(c[0].edge.i == 1);
  CHECK(c[0].edge.j == 6);
  CHECK(rep.file_loaded == 2);
  CHECK(rep.dropped_gate == 1);
  CHECK(rep.file == 1);
}

TEST_CASE("LoopCandidates_NoSourceEnabled_ReturnsNothing",
          "[slam][loop_candidates]") {
  TempPath project("test_loop_candidates_none");
  ProjectDB db(project.path);
  slam::LoopCandidateOptions o; // loop_closure.enable defaults to false here
  CHECK(slam::collect_loop_candidates(db, {1, 2}, seeds_along_x(2), o).empty());
}

TEST_CASE(
    "LoopCandidateOptions_OperatorEquals_IsMemberwiseThroughNestedStructs",
    "[slam][loop_candidates][parity]") {
  const slam::LoopCandidateOptions a;
  slam::LoopCandidateOptions b = a;
  CHECK(a == b);
  SECTION("a loop_closure field") {
    b.loop_closure.vocab_size = a.loop_closure.vocab_size + 1;
    CHECK_FALSE(a == b);
  }
  SECTION("a panorama_loops field") {
    b.panorama_loops.refine_iterations = a.panorama_loops.refine_iterations + 1;
    CHECK_FALSE(a == b);
  }
  SECTION("the file gate") {
    b.loop_edges_min_seed_disagreement_fraction = 0.5;
    CHECK_FALSE(a == b);
  }
}
