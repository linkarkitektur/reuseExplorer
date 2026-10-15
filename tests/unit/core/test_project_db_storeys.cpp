// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The storeys table (schema v30, reusex-z1v.3.2): core::StoreyRecord rows with
// deterministic GUIDs, source/locked semantics, and the project GUID they are
// derived from.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::ProjectDB;
using reusex::core::StoreyRecord;
using reusex::test_support::TempPath;

namespace {

StoreyRecord storey(const ProjectDB &db, int idx, double z_floor,
                    double z_ceiling) {
  StoreyRecord s;
  s.idx = idx;
  s.guid = reusex::core::deterministic_guid(db.project_guid(),
                                            "storeys/" + std::to_string(idx));
  s.name = "Etage " + std::to_string(idx);
  s.z_floor = z_floor;
  s.z_ceiling = z_ceiling;
  s.level_transform = {1, 0, 0, 0.5, 0, 1, 0, -0.25, 0, 0, 1, -1.5, 0, 0, 0, 1};
  s.confidence = 0.75;
  s.metadata = R"({"floor_inliers":1234})";
  return s;
}

} // namespace

TEST_CASE("ProjectGuid_IsMintedOnceAndStable", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_project_guid");
  std::string first;
  {
    ProjectDB db(tmp.path);
    first = db.project_guid();
    CHECK(first.size() == 36);
  }
  ProjectDB db(tmp.path);
  CHECK(db.project_guid() == first);
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK(ro.project_guid() == first);
}

TEST_CASE("Storeys_FreshProject_IsEmpty", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_empty");
  ProjectDB db(tmp.path);
  CHECK_FALSE(db.has_storeys());
  CHECK(db.storeys().empty());
  CHECK_FALSE(db.storey_by_guid("nope").has_value());
}

TEST_CASE("Storeys_SaveAndLoad_RoundTripsEveryField", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_roundtrip");
  ProjectDB db(tmp.path);
  const int run = db.log_pipeline_start("storeys");
  // Saved out of order: storeys() returns idx ascending.
  const std::vector<StoreyRecord> in{storey(db, 1, 2.9, 5.6),
                                     storey(db, 0, 0.0, 2.6)};
  CHECK(db.save_storeys(in, run) == 2);
  REQUIRE(db.has_storeys());

  const auto out = db.storeys();
  REQUIRE(out.size() == 2);
  CHECK(out[0].idx == 0);
  CHECK(out[1].idx == 1);
  const auto &s = out[1];
  CHECK(s.id > 0);
  CHECK(s.guid == in[0].guid);
  CHECK(s.name == "Etage 1");
  CHECK_THAT(s.z_floor, WithinAbs(2.9, 1e-12));
  CHECK_THAT(s.z_ceiling, WithinAbs(5.6, 1e-12));
  CHECK(s.level_transform == in[0].level_transform);
  CHECK(s.source == "detected");
  CHECK_FALSE(s.locked);
  CHECK_THAT(s.confidence, WithinAbs(0.75, 1e-12));
  CHECK(s.metadata == R"({"floor_inliers":1234})");

  const auto by_guid = db.storey_by_guid(in[1].guid);
  REQUIRE(by_guid.has_value());
  CHECK(by_guid->idx == 0);
}

TEST_CASE("Storeys_TwoSaves_KeepTheDeterministicGuid", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_guid_stable");
  ProjectDB db(tmp.path);
  db.save_storeys({storey(db, 0, 0.0, 2.6)});
  const auto first = db.storeys().at(0).guid;
  db.save_storeys({storey(db, 0, 0.01, 2.61)});
  const auto again = db.storeys();
  REQUIRE(again.size() == 1);
  CHECK(again[0].guid == first);
  CHECK_THAT(again[0].z_floor, WithinAbs(0.01, 1e-12));
}

TEST_CASE("Storeys_Save_ReplacesUnlockedRowsOnly", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_locked");
  ProjectDB db(tmp.path);

  auto locked0 = storey(db, 0, 0.0, 2.6);
  locked0.locked = true;
  locked0.source = "manual";
  locked0.name = "Stueetage";
  auto stale2 = storey(db, 2, 6.0, 8.6); // unlocked, not in the next run
  auto locked7 = storey(db, 7, 20.0, 22.0);
  locked7.guid = reusex::core::generate_guid(); // a hand-made row, no match
  locked7.locked = true;
  locked7.source = "manual";
  db.save_storeys({locked0, stale2, locked7});
  REQUIRE(db.storeys().size() == 3);

  // A re-run detects storeys 0 and 1. Storey 0 matches the locked row by guid
  // and is not written; storey 1 is new; stale storey 2 is replaced away;
  // the unmatched locked row 7 is kept.
  const auto written =
      db.save_storeys({storey(db, 0, 0.1, 2.7), storey(db, 1, 3.0, 5.6)});
  CHECK(written == 1);
  const auto rows = db.storeys();
  REQUIRE(rows.size() == 3);
  CHECK(rows[0].idx == 0);
  CHECK(rows[0].name == "Stueetage");
  CHECK(rows[0].locked);
  CHECK(rows[0].source == "manual");
  CHECK_THAT(rows[0].z_floor, WithinAbs(0.0, 1e-12));
  CHECK(rows[1].idx == 1);
  CHECK(rows[2].idx == 7);
  CHECK(rows[2].guid == locked7.guid);
}

TEST_CASE("Storeys_Save_SkipsADetectedRowWhoseIdxALockedRowHolds",
          "[projectdb][storeys]") {
  TempPath tmp("test_storeys_idx_clash");
  ProjectDB db(tmp.path);
  auto manual = storey(db, 1, 3.0, 5.5);
  manual.guid = reusex::core::generate_guid();
  manual.locked = true;
  manual.source = "manual";
  db.save_storeys({manual});

  CHECK(db.save_storeys({storey(db, 0, 0.0, 2.6), storey(db, 1, 2.9, 5.6)}) ==
        1);
  const auto rows = db.storeys();
  REQUIRE(rows.size() == 2);
  CHECK(rows[1].guid == manual.guid);
}

TEST_CASE("Storeys_Save_RejectsInvalidRecords", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_invalid");
  ProjectDB db(tmp.path);
  auto no_guid = storey(db, 0, 0.0, 2.6);
  no_guid.guid.clear();
  CHECK_THROWS_AS(db.save_storeys({no_guid}), std::invalid_argument);
  auto bad_source = storey(db, 0, 0.0, 2.6);
  bad_source.source = "guessed";
  CHECK_THROWS_AS(db.save_storeys({bad_source}), std::invalid_argument);
  CHECK_THROWS_AS(
      db.save_storeys({storey(db, 0, 0.0, 2.6), storey(db, 0, 0.0, 2.6)}),
      std::invalid_argument);
  CHECK_FALSE(db.has_storeys()); // nothing half-written
}

TEST_CASE("Storeys_Delete_RemovesLockedRowsToo", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_delete");
  ProjectDB db(tmp.path);
  auto s = storey(db, 0, 0.0, 2.6);
  s.locked = true;
  db.save_storeys({s, storey(db, 1, 3.0, 5.6)});
  db.delete_storeys();
  CHECK_FALSE(db.has_storeys());
}

TEST_CASE("Storeys_ReadOnlyProject_RefusesWrites", "[projectdb][storeys]") {
  TempPath tmp("test_storeys_ro");
  {
    ProjectDB db(tmp.path);
  }
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.save_storeys({storey(ro, 0, 0.0, 2.6)}),
                  std::runtime_error);
  CHECK_THROWS_AS(ro.delete_storeys(), std::runtime_error);
}

TEST_CASE("LoadSolidModel_ReadsStoredStoreys", "[projectdb][storeys][io]") {
  TempPath tmp("test_storeys_load_solid_model");
  ProjectDB db(tmp.path);
  db.save_storeys({storey(db, 1, 2.9, 5.6), storey(db, 0, 0.0, 2.6)});
  const auto m = reusex::io::load_solid_model(db);
  CHECK(m.provided.storeys);
  REQUIRE(m.storeys.size() == 2);
  CHECK(m.storeys[0].idx == 0);
  CHECK(m.storeys[1].name == "Etage 1");
  CHECK_FALSE(m.has_walls());
}
