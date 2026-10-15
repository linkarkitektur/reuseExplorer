// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux get plane_classes` / `rux del plane_classes` / `rux get
// clouds.planes.classes` (reusex-z1v.4.2): the plane-class router.

#include <catch2/catch_test_macros.hpp>

#include <database/path_parser.hpp>
#include <database/plane_class_router.hpp>
#include <database/resource_router.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

using namespace rux::database;
using json = nlohmann::json;

namespace {

std::shared_ptr<reusex::ProjectDB>
seeded(const reusex::test_support::TempPath &tmp) {
  auto db = std::make_shared<reusex::ProjectDB>(tmp.path);
  reusex::CloudL labels;
  for (uint32_t l : {1u, 2u}) {
    reusex::LabelT p;
    p.label = l;
    labels.push_back(p);
  }
  db->save_point_cloud("planes", labels, "segment_planes", "{}");
  std::vector<reusex::core::PlaneClassRecord> rows;
  for (int label : {1, 2}) {
    reusex::core::PlaneClassRecord p;
    p.plane_label = label;
    p.guid = reusex::core::deterministic_guid(
        db->project_guid(), "plane_classes/planes/" + std::to_string(label));
    p.orientation = label == 1 ? "horizontal" : "vertical";
    p.role = label == 1 ? "floor" : "wall";
    p.role_source = label == 1 ? "rules" : "semantic";
    p.semantic_class = label == 1 ? -1 : 0;
    p.class_confidence = label == 1 ? -1.0 : 0.9;
    p.storey = 0;
    p.point_count = 100 * label;
    p.area = 3.0;
    p.metadata = R"({"labelled_share":0.5})";
    rows.push_back(p);
  }
  db->save_plane_classes("planes", rows);
  return db;
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("PlaneClassRouter_Get_ListsAndDrillsDown",
          "[rux_app][router][plane_classes]") {
  reusex::test_support::TempPath tmp("test_plane_class_router");
  auto db = seeded(tmp);
  RouterRegistry registry(db);

  const json all = get(registry, "plane_classes");
  REQUIRE(all.is_array());
  REQUIRE(all.size() == 2);
  CHECK(all[0]["role"] == "floor");
  CHECK(all[0]["semantic_class"].is_null());
  CHECK(all[0]["class_confidence"].is_null());
  CHECK(all[1]["semantic_class"] == 0);
  CHECK(all[1]["role_source"] == "semantic");
  CHECK(all[1]["metadata"]["labelled_share"] == 0.5);

  CHECK(get(registry, "plane_classes.2.role") == "wall");
  CHECK(get(registry, "plane_classes[0]")["plane_label"] == 1);
  const std::string guid = all[1]["guid"];
  CHECK(get(registry, "plane_classes." + guid)["plane_label"] == 2);
  CHECK_THROWS(get(registry, "plane_classes.9"));

  // The same rows hang off the cloud.
  CHECK(get(registry, "clouds.planes.classes") == all);
}

TEST_CASE("PlaneClassRouter_Del_RemovesAllAndSetIsRefused",
          "[rux_app][router][plane_classes]") {
  reusex::test_support::TempPath tmp("test_plane_class_router_del");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("plane_classes");
  CHECK_THROWS(router.set({}, json::object()));
  CHECK_THROWS(router.del(parse_path("plane_classes.1")));
  router.del({});
  CHECK_FALSE(db->has_plane_classes());
}

TEST_CASE("PlaneClassRouter_Del_LockedRowsNeedForce",
          "[rux_app][router][plane_classes]") {
  reusex::test_support::TempPath tmp("test_plane_class_router_locked");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("plane_classes");
  CHECK_FALSE(router.del_requires_force({}).has_value());

  auto rows = db->plane_classes();
  rows[0].locked = true;
  rows[0].source = "manual";
  db->save_plane_classes("planes", rows);
  const auto reason = router.del_requires_force({});
  REQUIRE(reason.has_value());
  CHECK(reason->find("1 of 2") != std::string::npos);
}
