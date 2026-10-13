// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux graph prune` (spec §4.3).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/slam/graph_prune.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
#include <vector>

using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace slam = reusex::slam;

namespace {

void add(ProjectDB &db, int a, int b, const char *type, const char *source,
         const char *status, double residual, double dev) {
  ProjectDB::PoseGraphEdge e;
  e.from_node_id = a;
  e.to_node_id = b;
  e.edge_type = type;
  e.source = source;
  e.status = status;
  REQUIRE(db.upsert_pose_graph_edge(e));
  const std::vector<ProjectDB::PoseGraphEdgeMetrics> m = {
      {a, b, type, residual, dev, dev * 10.0, std::nullopt, std::nullopt}};
  REQUIRE(db.update_pose_graph_edge_metrics(m) == 1);
}

void make_graph(ProjectDB &db) {
  add(db, 1, 2, "odometry", "odometry_init", "auto", 0.1, 0.01);
  add(db, 3, 7, "loop_closure", "orb", "auto", 50.0, 0.8);
  add(db, 2, 6, "loop_closure", "file", "auto", 60.0, 0.1);
  add(db, 4, 8, "loop_closure", "manual", "confirmed", 99.0, 2.0);
}

} // namespace

TEST_CASE("GraphPrune_CriteriaAndTogether_AndSpareConfirmed",
          "[slam][graph_prune]") {
  TempPath tmp("test_prune_and");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.residual_above = 10.0;
  f.deviation_above_m = 0.5;
  const auto r = slam::prune_pose_graph(db, f);
  REQUIRE(r.matched.size() == 1);
  CHECK(r.matched[0].from_node_id == 3);
  CHECK(r.rejected == 1);
  CHECK(db.pose_graph_edge(3, 7, "loop_closure")->status == "rejected");
  CHECK(db.pose_graph_edge(2, 6, "loop_closure")->status == "auto");
  CHECK(db.pose_graph_edge(4, 8, "loop_closure")->status == "confirmed");
}

TEST_CASE("GraphPrune_IncludeConfirmed_RejectsThemToo", "[slam][graph_prune]") {
  TempPath tmp("test_prune_confirmed");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.deviation_above_m = 0.5;
  f.include_confirmed = true;
  CHECK(slam::prune_pose_graph(db, f).rejected == 2);
  CHECK(db.pose_graph_edge(4, 8, "loop_closure")->status == "rejected");
}

TEST_CASE("GraphPrune_DryRun_WritesNothing", "[slam][graph_prune]") {
  TempPath tmp("test_prune_dry");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.edge_type = "loop_closure";
  f.dry_run = true;
  const auto r = slam::prune_pose_graph(db, f);
  CHECK(r.matched.size() == 2);
  CHECK(r.rejected == 0);
  CHECK(db.pose_graph_edge(3, 7, "loop_closure")->status == "auto");
}

TEST_CASE("GraphPrune_SourceAndPairFilters", "[slam][graph_prune]") {
  TempPath tmp("test_prune_pair");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter by_pair;
  by_pair.pair = std::pair{7, 3}; // either direction
  CHECK(slam::prune_pose_graph(db, by_pair).rejected == 1);
  slam::PruneFilter by_source;
  by_source.source = "file";
  CHECK(slam::prune_pose_graph(db, by_source).rejected == 1);
  // Already rejected rows never match again.
  CHECK(slam::prune_pose_graph(db, by_source).matched.empty());
}

TEST_CASE("GraphPrune_NoCriterion_IsInvalid", "[slam][graph_prune]") {
  TempPath tmp("test_prune_none");
  ProjectDB db(tmp.path);
  make_graph(db);
  slam::PruneFilter f;
  f.include_confirmed = true;
  CHECK_THROWS_AS(slam::prune_pose_graph(db, f), std::invalid_argument);
}
