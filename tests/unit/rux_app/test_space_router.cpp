// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux get spaces` / `rux get storeys.<idx>.spaces` / `rux del spaces`
// (reusex-z1v.6.3): the spaces router.

#include <catch2/catch_test_macros.hpp>

#include <database/path_parser.hpp>
#include <database/resource_router.hpp>
#include <database/space_router.hpp>

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
  reusex::core::WallRecord w;
  w.guid = reusex::core::deterministic_guid(db->project_guid(), "walls/0");
  w.x1 = 8.0;
  w.height = 2.6;
  db->save_walls({w}, {});
  std::vector<reusex::core::SpaceRecord> spaces(3);
  std::vector<reusex::core::SpaceBoundaryRecord> bounds;
  for (int i = 0; i < 3; ++i) {
    auto &s = spaces[i];
    s.guid = reusex::core::deterministic_guid(db->project_guid(),
                                              "spaces/" + std::to_string(i));
    s.storey = i < 2 ? 0 : 1;
    s.name = "Space " + std::to_string(i + 1);
    s.kind = i == 1 ? "corridor" : "room";
    const double x0 = 4.0 * i;
    s.polygon.outer = {{x0, 0}, {x0 + 4, 0}, {x0 + 4, 3}, {x0, 3}};
    if (i == 0)
      s.polygon.holes.push_back({{1, 1}, {1.5, 1}, {1.5, 1.5}});
    s.area = 12.0;
    s.room_label = i == 2 ? -1 : i + 1;
    s.metadata = i == 0 ? R"({"arrangement_share":0.9})" : "";
    bounds.push_back({s.guid, w.guid, "a", x0, x0 + 4});
  }
  db->save_spaces(spaces, bounds);
  return db;
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("SpaceRouter_Get_ListsAndDrillsDown", "[rux_app][router][spaces]") {
  reusex::test_support::TempPath tmp("test_space_router");
  auto db = seeded(tmp);
  RouterRegistry registry(db);

  const json all = get(registry, "spaces");
  REQUIRE(all.is_array());
  REQUIRE(all.size() == 3);
  const json &first = all[0];
  CHECK(first["name"] == "Space 1");
  CHECK(first["polygon"]["outer"].size() == 4);
  CHECK(first["polygon"]["outer"][1] == json::array({4.0, 0.0}));
  CHECK(first["polygon"]["holes"].size() == 1);
  CHECK(first["metadata"]["arrangement_share"] == 0.9);
  CHECK(first["room_label"] == 1);
  CHECK(all[2]["room_label"].is_null());
  REQUIRE(first["boundaries"].size() == 1);
  CHECK(first["boundaries"][0]["side"] == "a");
  CHECK(first["boundaries"][0]["wall"] == db->walls()[0].guid);

  const auto id = all[1]["id"].get<int64_t>();
  CHECK(get(registry, "spaces." + std::to_string(id) + ".kind") == "corridor");
  const std::string guid = all[2]["guid"];
  CHECK(get(registry, "spaces." + guid)["storey"] == 1);
  CHECK(get(registry, "spaces[0]")["guid"] == first["guid"]);
  CHECK(get(registry, "spaces[0].boundaries").size() == 1);
  CHECK_THROWS(get(registry, "spaces.999999"));
  CHECK_THROWS(get(registry, "spaces[0].nope"));

  // Per storey.
  reusex::core::StoreyRecord s0, s1;
  s0.guid = reusex::core::deterministic_guid(db->project_guid(), "storeys/0");
  s1.guid = reusex::core::deterministic_guid(db->project_guid(), "storeys/1");
  s1.idx = 1;
  s1.z_floor = 3.0;
  db->save_storeys({s0, s1});
  CHECK(get(registry, "storeys.0.spaces").size() == 2);
  CHECK(get(registry, "storeys.1.spaces").size() == 1);
}

TEST_CASE("SpaceRouter_Del_AllAtOnce_LockedNeedForce",
          "[rux_app][router][spaces]") {
  reusex::test_support::TempPath tmp("test_space_router_del");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("spaces");
  CHECK_THROWS(router.set({}, json::object()));
  CHECK_THROWS(router.del(parse_path("spaces.1")));
  CHECK_FALSE(router.del_requires_force({}).has_value());
  auto rows = db->spaces();
  rows[0].locked = true;
  rows[0].source = "manual";
  db->save_spaces(rows, db->space_boundaries());
  const auto reason = router.del_requires_force({});
  REQUIRE(reason.has_value());
  CHECK(reason->find("1 of 3") != std::string::npos);
  router.del({});
  CHECK_FALSE(db->has_spaces());
  CHECK(db->space_boundaries().empty());
}
