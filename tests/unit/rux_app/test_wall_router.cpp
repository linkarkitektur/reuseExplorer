// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux get walls` / `rux get wall_joins` / `rux get storeys.<idx>.walls` /
// `rux del walls` (reusex-z1v.5.5): the wall routers.

#include <catch2/catch_test_macros.hpp>

#include <database/path_parser.hpp>
#include <database/resource_router.hpp>
#include <database/wall_router.hpp>

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
  std::vector<reusex::core::StoreyRecord> storeys(2);
  for (int i = 0; i < 2; ++i) {
    storeys[i].idx = i;
    storeys[i].guid = reusex::core::deterministic_guid(
        db->project_guid(), "storeys/" + std::to_string(i));
    storeys[i].z_floor = 2.8 * i;
    storeys[i].z_ceiling = 2.8 * i + 2.6;
  }
  db->save_storeys(storeys);
  std::vector<reusex::core::WallRecord> walls(3);
  const double xy[3][4] = {{0, 0, 4, 0}, {4, 0, 4, 3}, {0, 0, 5, 0}};
  for (int i = 0; i < 3; ++i) {
    auto &w = walls[i];
    w.guid = reusex::core::deterministic_guid(db->project_guid(),
                                              "walls/" + std::to_string(i));
    w.storey = i < 2 ? 0 : 1;
    w.x0 = xy[i][0];
    w.y0 = xy[i][1];
    w.x1 = xy[i][2];
    w.y1 = xy[i][3];
    w.height = 2.6;
    w.thickness = 0.1 * (i + 1);
    w.assumed = i == 1;
    w.metadata = i == 1 ? R"({"low_seen_share":true})" : "";
  }
  db->save_walls(walls,
                 {{walls[0].guid, walls[1].guid, "corner", "end", "start"}});
  return db;
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("WallRouter_Get_ListsAndDrillsDown", "[rux_app][router][walls]") {
  reusex::test_support::TempPath tmp("test_wall_router");
  auto db = seeded(tmp);
  RouterRegistry registry(db);

  const json all = get(registry, "walls");
  REQUIRE(all.is_array());
  REQUIRE(all.size() == 3);
  CHECK(all[0]["storey"] == 0);
  CHECK(all[0]["length"] == 4.0);
  CHECK(all[1]["assumed"] == true);
  CHECK(all[1]["metadata"]["low_seen_share"] == true);
  CHECK(all[0]["metadata"].is_null());
  REQUIRE(all[0]["joins"].size() == 1);
  CHECK(all[0]["joins"][0]["wall"] == all[1]["guid"]);
  CHECK(all[0]["joins"][0]["at"] == "end");
  CHECK(all[1]["joins"][0]["at"] == "start");
  CHECK(all[2]["joins"].empty());

  const auto id = all[1]["id"].get<int64_t>();
  CHECK(get(registry, "walls." + std::to_string(id) + ".thickness") ==
        all[1]["thickness"]);
  const std::string guid = all[2]["guid"];
  CHECK(get(registry, "walls." + guid)["storey"] == 1);
  CHECK(get(registry, "walls[0]")["guid"] == all[0]["guid"]);
  CHECK_THROWS(get(registry, "walls.999999"));
  CHECK_THROWS(get(registry, "walls[0].nope"));

  // The storey filter.
  CHECK(get(registry, "storeys.0.walls").size() == 2);
  CHECK(get(registry, "storeys.1.walls").size() == 1);

  const json joins = get(registry, "wall_joins");
  REQUIRE(joins.size() == 1);
  CHECK(joins[0]["kind"] == "corner");
  CHECK(get(registry, "wall_joins[0].wall_b") == all[1]["guid"]);
}

TEST_CASE("WallRouter_Del_RemovesAllAndSetIsRefused",
          "[rux_app][router][walls]") {
  reusex::test_support::TempPath tmp("test_wall_router_del");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("walls");
  CHECK_THROWS(router.set({}, json::object()));
  CHECK_THROWS(router.del(parse_path("walls.1")));
  CHECK_THROWS(registry.get_router("wall_joins").del({}));
  router.del({});
  CHECK_FALSE(db->has_walls());
  CHECK(db->wall_joins().empty());
}

TEST_CASE("WallRouter_Del_LockedRowsNeedForce", "[rux_app][router][walls]") {
  reusex::test_support::TempPath tmp("test_wall_router_locked");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("walls");
  CHECK_FALSE(router.del_requires_force({}).has_value());

  auto rows = db->walls();
  rows[0].locked = true;
  rows[0].source = "manual";
  db->save_walls(rows, {});
  const auto reason = router.del_requires_force({});
  REQUIRE(reason.has_value());
  CHECK(reason->find("1 of 3") != std::string::npos);
}
