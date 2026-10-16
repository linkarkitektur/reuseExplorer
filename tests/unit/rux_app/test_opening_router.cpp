// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux get openings` / `rux get walls.<id>.openings` / `rux del openings`
// (reusex-z1v.7.3): the openings router.

#include <catch2/catch_test_macros.hpp>

#include <database/opening_router.hpp>
#include <database/path_parser.hpp>
#include <database/resource_router.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/guid.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace rux::database;
using json = nlohmann::json;

namespace {

std::shared_ptr<reusex::ProjectDB>
seeded(const reusex::test_support::TempPath &tmp) {
  auto db = std::make_shared<reusex::ProjectDB>(tmp.path);
  std::vector<reusex::core::WallRecord> walls(2);
  for (int i = 0; i < 2; ++i) {
    walls[i].guid = reusex::core::deterministic_guid(
        db->project_guid(), "walls/" + std::to_string(i));
    walls[i].x1 = 5.0;
    walls[i].y0 = walls[i].y1 = 3.0 * i;
    walls[i].height = 2.6;
  }
  db->save_walls(walls, {});
  std::vector<reusex::core::OpeningRecord> ops(3);
  for (int i = 0; i < 3; ++i) {
    auto &o = ops[i];
    o.guid = reusex::core::deterministic_guid(db->project_guid(),
                                              "openings/" + std::to_string(i));
    o.wall = walls[i < 2 ? 0 : 1].guid;
    o.kind = i == 0 ? "door" : "window";
    o.s_start = 1.0 + 2.0 * i;
    o.s_end = o.s_start + 0.9;
    o.z_sill = i == 0 ? 0.0 : 0.9;
    o.z_head = 2.1;
    o.metadata = i == 0 ? R"({"crossings":2})" : "";
  }
  db->save_openings(ops);
  return db;
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("OpeningRouter_Get_ListsAndDrillsDown",
          "[rux_app][router][openings]") {
  reusex::test_support::TempPath tmp("test_opening_router");
  auto db = seeded(tmp);
  RouterRegistry registry(db);

  const json all = get(registry, "openings");
  REQUIRE(all.is_array());
  REQUIRE(all.size() == 3);
  // Ordered by host wall guid, then s: find them by kind.
  json door, win;
  for (const auto &o : all)
    (o["kind"] == "door" ? door : win) = o;
  CHECK(door["s_start"] == 1.0);
  CHECK(std::abs(door["width"].get<double>() - 0.9) < 1e-9);
  CHECK(door["metadata"]["crossings"] == 2);
  CHECK(win["metadata"].is_null());
  CHECK(door["source_instance_guid"].is_null());
  const auto id = win["id"].get<int64_t>();
  CHECK(get(registry, "openings." + std::to_string(id) + ".z_sill") == 0.9);
  const std::string guid = door["guid"];
  CHECK(get(registry, "openings." + guid)["kind"] == "door");
  CHECK(get(registry, "openings[0]")["guid"] == all[0]["guid"]);
  CHECK_THROWS(get(registry, "openings.999999"));
  CHECK_THROWS(get(registry, "openings[0].nope"));

  // The per-wall filter.
  const auto walls = db->walls();
  CHECK(get(registry, "walls." + walls[0].guid + ".openings").size() == 2);
  CHECK(get(registry, "walls." + walls[1].guid + ".openings").size() == 1);
}

TEST_CASE("OpeningRouter_Del_AllAtOnce_LockedNeedForce",
          "[rux_app][router][openings]") {
  reusex::test_support::TempPath tmp("test_opening_router_del");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("openings");
  CHECK_THROWS(router.set({}, json::object()));
  CHECK_THROWS(router.del(parse_path("openings.1")));
  CHECK_FALSE(router.del_requires_force({}).has_value());
  auto rows = db->openings();
  rows[0].locked = true;
  rows[0].source = "manual";
  db->save_openings(rows);
  const auto reason = router.del_requires_force({});
  REQUIRE(reason.has_value());
  CHECK(reason->find("1 of 3") != std::string::npos);
  router.del({});
  CHECK_FALSE(db->has_openings());
}
