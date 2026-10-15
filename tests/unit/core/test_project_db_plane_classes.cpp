// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The plane_classes table (schema v31, reusex-z1v.4.2): one
// core::PlaneClassRecord per plane label of a label cloud, with
// deterministic GUIDs and source/locked semantics.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/solid_model.hpp>
#include <reusex/io/solid_model_json.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>
#include <string>
#include <vector>

using Catch::Matchers::WithinAbs;
using reusex::ProjectDB;
using reusex::core::PlaneClassRecord;
using reusex::test_support::TempPath;

namespace {

/// A tiny "planes" label cloud so the rows have a cloud to belong to.
void seed_planes_cloud(ProjectDB &db, const char *name = "planes") {
  reusex::CloudL labels;
  for (uint32_t l : {1u, 1u, 2u, 3u}) {
    reusex::LabelT p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud(name, labels, "segment_planes", "{}");
}

PlaneClassRecord plane(const ProjectDB &db, int label,
                       const char *role = "wall") {
  PlaneClassRecord r;
  r.plane_label = label;
  r.guid = reusex::core::deterministic_guid(
      db.project_guid(), "plane_classes/planes/" + std::to_string(label));
  r.orientation = "vertical";
  r.semantic_class = 2;
  r.class_confidence = 0.8;
  r.role = role;
  r.role_source = "semantic";
  r.storey = 0;
  r.point_count = 1200 + label;
  r.area = 4.5;
  r.z_min = 0.02;
  r.z_max = 2.55;
  r.metadata = R"({"labelled_share":0.9})";
  return r;
}

} // namespace

TEST_CASE("PlaneClasses_FreshProject_IsEmpty", "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_fresh");
  ProjectDB db(tmp.path);
  CHECK(db.plane_classes().empty());
  CHECK_FALSE(db.has_plane_classes());
}

TEST_CASE("PlaneClasses_SaveAndLoad_RoundTripsEveryField",
          "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_roundtrip");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);

  auto none = plane(db, 3, "unknown");
  none.semantic_class = -1;     // stored as NULL
  none.class_confidence = -1.0; // stored as NULL
  none.role_source = "rules";
  none.orientation = "slanted";
  none.storey = -1;
  none.metadata.clear();
  CHECK(db.save_plane_classes("planes", {none, plane(db, 1)}, 42) == 2);
  CHECK(db.has_plane_classes());

  const auto rows = db.plane_classes();
  REQUIRE(rows.size() == 2);
  const auto &a = rows[0]; // plane_label ascending
  const auto want = plane(db, 1);
  CHECK(a.plane_label == 1);
  CHECK(a.guid == want.guid);
  CHECK(a.orientation == "vertical");
  CHECK(a.semantic_class == 2);
  CHECK_THAT(a.class_confidence, WithinAbs(0.8, 1e-12));
  CHECK(a.role == "wall");
  CHECK(a.role_source == "semantic");
  CHECK(a.storey == 0);
  CHECK(a.point_count == 1201);
  CHECK_THAT(a.area, WithinAbs(4.5, 1e-12));
  CHECK_THAT(a.z_min, WithinAbs(0.02, 1e-12));
  CHECK_THAT(a.z_max, WithinAbs(2.55, 1e-12));
  CHECK(a.source == "detected");
  CHECK_FALSE(a.locked);
  CHECK(a.metadata == R"({"labelled_share":0.9})");

  const auto &b = rows[1];
  CHECK(b.plane_label == 3);
  CHECK(b.semantic_class == -1);
  CHECK(b.class_confidence == -1.0);
  CHECK(b.storey == -1);
  CHECK(b.role_source == "rules");
  CHECK(b.metadata.empty());

  // Rows are per cloud.
  CHECK(db.plane_classes("rooms").empty());
}

TEST_CASE("PlaneClasses_Save_ReplacesUnlockedRowsOnly",
          "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_locked");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);

  auto hand = plane(db, 1, "slab");
  hand.locked = true;
  hand.source = "manual";
  hand.role_source = "manual";
  db.save_plane_classes("planes", {hand, plane(db, 2), plane(db, 3)});

  // A re-run classifies planes 1 and 2 only: plane 1 is locked and kept,
  // plane 2 is replaced, plane 3 (unlocked, not in the run) goes.
  auto rerun2 = plane(db, 2, "clutter");
  CHECK(db.save_plane_classes("planes", {plane(db, 1, "wall"), rerun2}) == 1);
  const auto rows = db.plane_classes();
  REQUIRE(rows.size() == 2);
  CHECK(rows[0].role == "slab");
  CHECK(rows[0].locked);
  CHECK(rows[0].source == "manual");
  CHECK(rows[1].role == "clutter");
}

TEST_CASE("PlaneClasses_Save_RejectsInvalidRecords",
          "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_invalid");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);

  auto no_guid = plane(db, 1);
  no_guid.guid.clear();
  CHECK_THROWS_AS(db.save_plane_classes("planes", {no_guid}),
                  std::invalid_argument);
  auto label0 = plane(db, 1);
  label0.plane_label = 0;
  CHECK_THROWS_AS(db.save_plane_classes("planes", {label0}),
                  std::invalid_argument);
  auto bad_role = plane(db, 1, "door");
  CHECK_THROWS_AS(db.save_plane_classes("planes", {bad_role}),
                  std::invalid_argument);
  auto bad_orient = plane(db, 1);
  bad_orient.orientation = "diagonal";
  CHECK_THROWS_AS(db.save_plane_classes("planes", {bad_orient}),
                  std::invalid_argument);
  auto bad_rs = plane(db, 1);
  bad_rs.role_source = "guess";
  CHECK_THROWS_AS(db.save_plane_classes("planes", {bad_rs}),
                  std::invalid_argument);
  auto bad_source = plane(db, 1);
  bad_source.source = "guessed";
  CHECK_THROWS_AS(db.save_plane_classes("planes", {bad_source}),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.save_plane_classes("planes", {plane(db, 1), plane(db, 1)}),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.save_plane_classes("no_such_cloud", {plane(db, 1)}),
                  std::runtime_error);
  CHECK_FALSE(db.has_plane_classes()); // nothing half-written
}

TEST_CASE("PlaneClasses_GoWithTheirCloud", "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_cascade");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);
  auto locked = plane(db, 1);
  locked.locked = true;
  db.save_plane_classes("planes", {locked, plane(db, 2)});

  // Re-saving the cloud (a planes re-run) keeps the cloud row and the rows.
  seed_planes_cloud(db);
  CHECK(db.plane_classes().size() == 2);

  db.delete_point_cloud("planes");
  CHECK(db.plane_classes().empty());
}

TEST_CASE("PlaneClasses_Delete_RemovesLockedRowsToo",
          "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_delete");
  ProjectDB db(tmp.path);
  seed_planes_cloud(db);
  seed_planes_cloud(db, "other_planes");
  auto locked = plane(db, 1);
  locked.locked = true;
  db.save_plane_classes("planes", {locked, plane(db, 2)});
  auto other = plane(db, 1);
  other.guid = reusex::core::generate_guid();
  db.save_plane_classes("other_planes", {other});

  db.delete_plane_classes("planes");
  CHECK_FALSE(db.has_plane_classes());
  CHECK(db.has_plane_classes("other_planes"));
}

TEST_CASE("PlaneClasses_ReadOnlyProject_RefusesWrites",
          "[projectdb][plane_classes]") {
  TempPath tmp("test_plane_classes_ro");
  {
    ProjectDB db(tmp.path);
    seed_planes_cloud(db);
  }
  ProjectDB ro(tmp.path, /*readOnly=*/true);
  CHECK_THROWS_AS(ro.save_plane_classes("planes", {plane(ro, 1)}),
                  std::runtime_error);
  CHECK_THROWS_AS(ro.delete_plane_classes(), std::runtime_error);
}

TEST_CASE("LoadSolidModel_ReadsStoredPlaneClasses",
          "[projectdb][plane_classes][io]") {
  TempPath tmp("test_plane_classes_load_solid_model");
  ProjectDB db(tmp.path);
  {
    const auto m = reusex::io::load_solid_model(db);
    CHECK_FALSE(m.provided.plane_classes);
    CHECK_FALSE(m.has_plane_classes());
  }
  seed_planes_cloud(db);
  db.save_plane_classes("planes", {plane(db, 2), plane(db, 1)});
  const auto m = reusex::io::load_solid_model(db);
  CHECK(m.provided.plane_classes);
  REQUIRE(m.plane_classes.size() == 2);
  CHECK(m.plane_classes[0].plane_label == 1);
  CHECK(m.plane_classes[1].guid == plane(db, 2).guid);
}

TEST_CASE("SolidModelJson_PlaneClasses_RoundTripTheNewFields",
          "[plane_classes][io]") {
  TempPath tmp("test_plane_classes_json");
  ProjectDB db(tmp.path);
  reusex::core::SolidModel m;
  auto p = plane(db, 3, "slab");
  p.role_source = "manual";
  p.source = "manual";
  p.locked = true;
  m.plane_classes.push_back(p);
  m.plane_classes.push_back(PlaneClassRecord{}); // the old minimal shape
  m.plane_classes.back().plane_label = 4;
  m.plane_classes.back().orientation = "vertical";

  const auto back =
      reusex::io::solid_model_from_json(reusex::io::solid_model_to_json(m));
  REQUIRE(back.plane_classes.size() == 2);
  const auto &a = back.plane_classes[0];
  CHECK(a.guid == p.guid);
  CHECK(a.role == "slab");
  CHECK(a.role_source == "manual");
  CHECK(a.source == "manual");
  CHECK(a.locked);
  CHECK(a.metadata == R"({"labelled_share":0.9})");
  CHECK(back.plane_classes[1].guid.empty());
  CHECK(back.plane_classes[1].role_source == "none");
  CHECK(back.plane_classes[1].source == "detected");

  // A document written before these fields existed still reads.
  const auto old = reusex::io::solid_model_from_json(
      R"({"model_version":1,"plane_classes":[{"plane_label":1,)"
      R"("orientation":"horizontal",)"
      R"("role":"floor"}]})");
  REQUIRE(old.plane_classes.size() == 1);
  CHECK(old.plane_classes[0].role_source == "none");
  CHECK_FALSE(old.plane_classes[0].locked);
}
