// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux get storeys` / `rux del storeys` (reusex-z1v.3.3): the storeys router.

#include <catch2/catch_test_macros.hpp>

#include <database/path_parser.hpp>
#include <database/resource_router.hpp>
#include <database/storey_router.hpp>

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
  std::vector<reusex::core::StoreyRecord> rows;
  for (int idx : {0, 1}) {
    reusex::core::StoreyRecord s;
    s.idx = idx;
    s.guid = reusex::core::deterministic_guid(db->project_guid(),
                                              "storeys/" + std::to_string(idx));
    s.name = "Etage " + std::to_string(idx);
    s.z_floor = 3.0 * idx;
    s.z_ceiling = 3.0 * idx + 2.6;
    s.metadata = R"({"levels":3})";
    rows.push_back(s);
  }
  db->save_storeys(rows);
  return db;
}

json get(RouterRegistry &registry, const std::string &path) {
  auto components = parse_path(path);
  std::vector<PathComponent> rel(components.begin() + 1, components.end());
  return std::get<json>(registry.get_router(components[0].value).get(rel));
}

} // namespace

TEST_CASE("StoreyRouter_Get_ListsAndDrillsDown", "[rux_app][router][storeys]") {
  reusex::test_support::TempPath tmp("test_storey_router");
  auto db = seeded(tmp);
  RouterRegistry registry(db);

  const json all = get(registry, "storeys");
  REQUIRE(all.is_array());
  REQUIRE(all.size() == 2);
  CHECK(all[1]["idx"] == 1);
  CHECK(std::abs(all[1]["height"].get<double>() - 2.6) < 1e-12);
  CHECK(all[1]["metadata"]["levels"] == 3);
  CHECK(all[0]["level_transform"].size() == 16);

  CHECK(get(registry, "storeys/1")["name"] == "Etage 1");
  CHECK(get(registry, "storeys.1.z_floor") == 3.0);
  CHECK(get(registry, "storeys[0]")["idx"] == 0);
  const std::string guid = all[0]["guid"];
  CHECK(get(registry, "storeys." + guid)["idx"] == 0);
  CHECK_THROWS(get(registry, "storeys.7"));
}

TEST_CASE("StoreyRouter_Del_RemovesAllAndSetIsRefused",
          "[rux_app][router][storeys]") {
  reusex::test_support::TempPath tmp("test_storey_router_del");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("storeys");
  CHECK_THROWS(router.set({}, json::object()));
  CHECK_THROWS(router.del(parse_path("storeys.1")));
  router.del({});
  CHECK_FALSE(db->has_storeys());
}

TEST_CASE("StoreyRouter_Del_LockedRowsNeedForce",
          "[rux_app][router][storeys]") {
  reusex::test_support::TempPath tmp("test_storey_router_locked");
  auto db = seeded(tmp);
  RouterRegistry registry(db);
  auto &router = registry.get_router("storeys");
  CHECK_FALSE(router.del_requires_force({}).has_value());

  auto rows = db->storeys();
  rows[0].locked = true;
  rows[0].source = "manual";
  db->save_storeys(rows);
  const auto reason = router.del_requires_force({});
  REQUIRE(reason.has_value());
  CHECK(reason->find("1 of 2") != std::string::npos);
}
