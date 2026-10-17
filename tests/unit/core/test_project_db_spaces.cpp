// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The spaces and space_boundaries tables (schema v35, reusex-z1v.6.3): the
// polygon blob codec, save / load / locked rows / validation, and
// io::load_solid_model filling spaces.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/space_polygon.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
using reusex::core::SpaceBoundaryRecord;
using reusex::core::SpacePolygon;
using reusex::core::SpaceRecord;
using reusex::core::WallRecord;
using reusex::test_support::TempPath;

namespace {

SpacePolygon square(double x0, double y0, double side) {
  SpacePolygon p;
  p.outer = {
      {x0, y0}, {x0 + side, y0}, {x0 + side, y0 + side}, {x0, y0 + side}};
  return p;
}

SpaceRecord space(const ProjectDB &db, const std::string &key, double x0) {
  SpaceRecord s;
  s.guid = reusex::core::deterministic_guid(db.project_guid(), key);
  s.storey = 0;
  s.name = "Space " + key;
  s.kind = "room";
  s.polygon = square(x0, 0.0, 4.0);
  s.z_floor = 0.0;
  s.z_ceiling = 2.6;
  s.area = 16.0;
  s.perimeter = 16.0;
  s.volume = 41.6;
  s.compactness = 0.785;
  s.room_label = 3;
  s.confidence = 0.7;
  s.metadata = R"({"arrangement_share":0.9})";
  return s;
}

WallRecord wall(const ProjectDB &db) {
  WallRecord w;
  w.guid = reusex::core::deterministic_guid(db.project_guid(), "walls/0/a");
  w.x1 = 8.0;
  w.height = 2.6;
  w.thickness = 0.12;
  return w;
}

} // namespace

TEST_CASE("SpacePolygon_Codec_RoundTripsRingsAndHoles",
          "[projectdb][spaces][codec]") {
  SpacePolygon p = square(1.0, 2.0, 5.0);
  p.holes.push_back({{2.0, 3.0}, {3.0, 3.0}, {2.5, 4.0}});
  p.holes.push_back({{4.0, 4.0}, {4.5, 4.0}, {4.5, 4.5}, {4.0, 4.5}});
  const auto blob = reusex::core::encode_space_polygon(p);
  // ring count + per ring a count and n x (x, y) doubles
  CHECK(blob.size() == 4 + (4 + 4 * 16) + (4 + 3 * 16) + (4 + 4 * 16));
  const auto q = reusex::core::decode_space_polygon(blob);
  CHECK(q.outer == p.outer);
  CHECK(q.holes == p.holes);
}

TEST_CASE("SpacePolygon_Codec_RejectsBadInput", "[projectdb][spaces][codec]") {
  SpacePolygon p;
  p.outer = {{0, 0}, {1, 0}};
  CHECK_THROWS_AS(reusex::core::encode_space_polygon(p), std::invalid_argument);
  p = square(0, 0, 1);
  p.holes.push_back({{0.1, 0.1}});
  CHECK_THROWS_AS(reusex::core::encode_space_polygon(p), std::invalid_argument);
  p = square(0, 0, 1);
  p.outer[1][0] = std::numeric_limits<double>::quiet_NaN();
  CHECK_THROWS_AS(reusex::core::encode_space_polygon(p), std::invalid_argument);

  auto blob = reusex::core::encode_space_polygon(square(0, 0, 1));
  blob.pop_back(); // truncated
  CHECK_THROWS_AS(reusex::core::decode_space_polygon(blob), std::runtime_error);
  CHECK_THROWS_AS(reusex::core::decode_space_polygon({}), std::runtime_error);
}

TEST_CASE("SpacePolygon_Metrics_AreaPerimeterCentroid",
          "[projectdb][spaces][codec]") {
  SpacePolygon p = square(0.0, 0.0, 4.0);
  p.holes.push_back({{1, 1}, {1, 2}, {2, 2}, {2, 1}}); // either orientation
  CHECK(reusex::core::space_polygon_area(p) == 15.0);
  const auto c = reusex::core::space_polygon_centroid(p);
  // (16 * (2, 2) - 1 * (1.5, 1.5)) / 15
  CHECK(std::abs(c[0] - (32.0 - 1.5) / 15.0) < 1e-12);
  CHECK(std::abs(c[1] - (32.0 - 1.5) / 15.0) < 1e-12);
}

TEST_CASE("Spaces_FreshProject_IsEmpty", "[projectdb][spaces]") {
  TempPath tmp("test_spaces_empty");
  ProjectDB db(tmp.path);
  CHECK(db.spaces().empty());
  CHECK(db.space_boundaries().empty());
  CHECK_FALSE(db.has_spaces());
  CHECK_FALSE(db.space_by_guid("nope").has_value());
}

TEST_CASE("Spaces_SaveAndLoad_RoundTripsEveryField", "[projectdb][spaces]") {
  TempPath tmp("test_spaces_roundtrip");
  ProjectDB db(tmp.path);
  const auto w = wall(db);
  auto a = space(db, "a", 0.0);
  a.polygon.holes.push_back({{1, 1}, {1.5, 1}, {1.5, 1.5}});
  auto b = space(db, "b", 4.0);
  b.storey = 1;
  b.kind = "corridor";
  b.room_label = -1;
  b.locked = true;
  b.source = "manual";
  const std::vector<SpaceBoundaryRecord> bounds = {
      {a.guid, w.guid, "a", 0.0, 4.0}, {b.guid, w.guid, "a", 4.0, 8.0}};
  CHECK(db.save_spaces({b, a}, bounds, 9) == 2);
  CHECK(db.has_spaces());
  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  // storey, then row id
  const auto &r = rows[0];
  CHECK(r.id > 0);
  CHECK(r.guid == a.guid);
  CHECK(r.storey == 0);
  CHECK(r.name == a.name);
  CHECK(r.kind == "room");
  CHECK(r.polygon.outer == a.polygon.outer);
  CHECK(r.polygon.holes == a.polygon.holes);
  CHECK(r.z_floor == 0.0);
  CHECK(r.z_ceiling == 2.6);
  CHECK(r.area == 16.0);
  CHECK(r.perimeter == 16.0);
  CHECK(r.volume == 41.6);
  CHECK(r.compactness == 0.785);
  CHECK(r.room_label == 3);
  CHECK(r.source == "detected");
  CHECK_FALSE(r.locked);
  CHECK(r.confidence == 0.7);
  CHECK(r.metadata == a.metadata);
  CHECK(rows[1].guid == b.guid);
  CHECK(rows[1].room_label == -1);
  CHECK(rows[1].locked);
  CHECK(rows[1].kind == "corridor");
  CHECK(db.space_by_guid(b.guid)->storey == 1);

  const auto sb = db.space_boundaries();
  REQUIRE(sb.size() == 2);
  CHECK(sb[0].space == a.guid);
  CHECK(sb[0].wall == w.guid);
  CHECK(sb[0].side == "a");
  CHECK(sb[0].s_start == 0.0);
  CHECK(sb[0].s_end == 4.0);
  CHECK(sb[1].space == b.guid);
}

TEST_CASE("Spaces_Save_ReplacesUnlockedRowsOnly", "[projectdb][spaces]") {
  TempPath tmp("test_spaces_locked");
  ProjectDB db(tmp.path);
  const auto w = wall(db);
  auto keep = space(db, "k", 0.0);
  keep.locked = true;
  keep.source = "manual";
  keep.name = "Kitchen";
  const auto gone = space(db, "g", 4.0);
  db.save_spaces({keep, gone}, {{keep.guid, w.guid, "a", 0.0, 4.0},
                                {gone.guid, w.guid, "a", 4.0, 8.0}});

  // A re-run: the locked one comes back renamed (skipped, the stored row
  // wins, its boundaries too), the unlocked one is not found again, a new
  // one is.
  auto moved = keep;
  moved.locked = false;
  moved.source = "detected";
  moved.name = "Space 1";
  const auto fresh = space(db, "n", 8.0);
  CHECK(db.save_spaces({moved, fresh}, {{moved.guid, w.guid, "b", 0.0, 1.0},
                                        {fresh.guid, w.guid, "a", 8.0, 9.0}}) ==
        1);
  const auto rows = db.spaces();
  REQUIRE(rows.size() == 2);
  CHECK(rows[0].guid == keep.guid);
  CHECK(rows[0].name == "Kitchen");
  CHECK(rows[0].locked);
  CHECK(rows[1].guid == fresh.guid);
  const auto sb = db.space_boundaries();
  REQUIRE(sb.size() == 2);
  CHECK(sb[0].space == keep.guid);
  CHECK(sb[0].side == "a"); // the stored boundary of the locked space
  CHECK(sb[1].space == fresh.guid);
}

TEST_CASE("Spaces_Save_RejectsInvalidRecords", "[projectdb][spaces]") {
  TempPath tmp("test_spaces_invalid");
  ProjectDB db(tmp.path);
  const auto w = wall(db);
  const auto ok = space(db, "a", 0.0);
  auto bad = ok;
  bad.guid.clear();
  CHECK_THROWS_AS(db.save_spaces({bad}, {}), std::invalid_argument);
  bad = ok;
  bad.kind = "attic";
  CHECK_THROWS_AS(db.save_spaces({bad}, {}), std::invalid_argument);
  bad = ok;
  bad.source = "guessed";
  CHECK_THROWS_AS(db.save_spaces({bad}, {}), std::invalid_argument);
  bad = ok;
  bad.polygon.outer.resize(2);
  CHECK_THROWS_AS(db.save_spaces({bad}, {}), std::invalid_argument);
  CHECK_THROWS_AS(db.save_spaces({ok, ok}, {}), std::invalid_argument);
  // Boundaries: unknown space, bad side, empty wall, reversed range.
  CHECK_THROWS_AS(db.save_spaces({ok}, {{"other", w.guid, "a", 0.0, 1.0}}),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.save_spaces({ok}, {{ok.guid, w.guid, "c", 0.0, 1.0}}),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.save_spaces({ok}, {{ok.guid, "", "a", 0.0, 1.0}}),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.save_spaces({ok}, {{ok.guid, w.guid, "a", 2.0, 1.0}}),
                  std::invalid_argument);
  // A refused call leaves the tables as they were.
  db.save_spaces({ok}, {{ok.guid, w.guid, "a", 0.0, 1.0}});
  CHECK_THROWS_AS(db.save_spaces({bad}, {}), std::invalid_argument);
  CHECK(db.spaces().size() == 1);
  CHECK(db.space_boundaries().size() == 1);
}

TEST_CASE("Spaces_Delete_AndReadOnly", "[projectdb][spaces]") {
  TempPath tmp("test_spaces_delete");
  {
    ProjectDB db(tmp.path);
    auto s = space(db, "a", 0.0);
    s.locked = true;
    db.save_spaces({s}, {{s.guid, wall(db).guid, "a", 0.0, 1.0}});
  }
  {
    ProjectDB ro(tmp.path, /*readOnly=*/true);
    CHECK(ro.spaces().size() == 1);
    CHECK(ro.space_boundaries().size() == 1);
    CHECK_THROWS_AS(ro.save_spaces({}, {}), std::runtime_error);
    CHECK_THROWS_AS(ro.delete_spaces(), std::runtime_error);
  }
  ProjectDB db(tmp.path);
  db.delete_spaces();
  CHECK_FALSE(db.has_spaces());
  CHECK(db.space_boundaries().empty());
}

TEST_CASE("LoadSolidModel_ReadsSpaces_SkipsOrphanBoundaries",
          "[projectdb][spaces][io]") {
  TempPath tmp("test_spaces_load_model");
  ProjectDB db(tmp.path);
  reusex::core::StoreyRecord st;
  st.guid = reusex::core::deterministic_guid(db.project_guid(), "storeys/z0");
  st.z_ceiling = 2.6;
  db.save_storeys({st});
  const auto w = wall(db);
  db.save_walls({w}, {});
  {
    const auto m = reusex::io::load_solid_model(db);
    CHECK(m.spaces.empty());
    CHECK_FALSE(m.has_spaces());
  }
  const auto s = space(db, "a", 0.0);
  db.save_spaces({s}, {{s.guid, w.guid, "a", 0.0, 4.0},
                       {s.guid, "a-wall-that-is-gone", "b", 0.0, 4.0}});
  const auto m = reusex::io::load_solid_model(db);
  REQUIRE(m.spaces.size() == 1);
  CHECK(m.spaces[0].guid == s.guid);
  CHECK(m.has_spaces());
  REQUIRE(m.space_boundaries.size() == 1);
  CHECK(m.space_boundaries[0].wall == w.guid);
}
