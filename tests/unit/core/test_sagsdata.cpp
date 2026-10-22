// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core/sagsdata.hpp: the BEK 496 Sagsdata key scheme shared by ruxd and the
// PDF report.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <core/ProjectDB.hpp>
#include <core/sagsdata.hpp>

#include "../../support/temp_path.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>

using Catch::Matchers::ContainsSubstring;
using reusex::ProjectDB;
using reusex::test_support::TempPath;
namespace core = reusex::core;
using nlohmann::json;

TEST_CASE("Sagsdata_SettingKey_IsTheRuxdScheme", "[sagsdata]") {
  CHECK(core::sagsdata_setting_key("p1", "bbr_number") ==
        "sagsdata/p1/bbr_number");
  CHECK(core::sagsdata_fields().size() == 9);
  CHECK(core::is_sagsdata_field("screening_approved_at"));
  CHECK_FALSE(core::is_sagsdata_field("name"));
}

TEST_CASE("Sagsdata_Json_ReadsStoredValuesAndNulls", "[sagsdata]") {
  TempPath tmp("test_sagsdata");
  ProjectDB db(tmp.path);
  db.set_setting(core::sagsdata_setting_key("p1", "client"), "Region Syd");
  db.set_setting(core::sagsdata_setting_key("p1", "demolished_floor_area_m2"),
                 "1240.5");
  db.set_setting(core::sagsdata_setting_key("p1", "cadastral"), "");
  const auto j = core::sagsdata_json(db, "p1");
  CHECK(j.at("client") == "Region Syd");
  CHECK(j.at("demolished_floor_area_m2") == 1240.5);
  CHECK(j.at("cadastral").is_null()); // "" is unset
  CHECK(j.at("bbr_number").is_null());
  CHECK(j.size() == core::sagsdata_fields().size());
  CHECK_FALSE(core::sagsdata_value(db, "p1", "cadastral").has_value());
  CHECK(core::sagsdata_value(db, "other", "client") == std::nullopt);
}

TEST_CASE("Sagsdata_StoredValue_ValidatesEachField", "[sagsdata]") {
  CHECK(core::sagsdata_stored_value("client", json(nullptr)).empty());
  CHECK(core::sagsdata_stored_value("client", json("X")) == "X");
  CHECK(core::sagsdata_stored_value("demolished_floor_area_m2", json(12.5)) ==
        "12.5");
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("demolished_floor_area_m2", json(-1)),
      std::invalid_argument);
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("demolished_floor_area_m2", json("12")),
      std::invalid_argument);
  CHECK_THROWS_WITH(
      core::sagsdata_stored_value("demolition_start", json("2026-02-29")),
      ContainsSubstring("demolition_start"));
  CHECK(core::sagsdata_stored_value("demolition_start", json("2028-02-29")) ==
        "2028-02-29");
  CHECK(core::sagsdata_stored_value("screening_approved_at",
                                    json("2026-10-01T09:30:00Z")) ==
        "2026-10-01T09:30:00Z");
  CHECK_THROWS_AS(
      core::sagsdata_stored_value("screening_approved_at", json("1. oktober")),
      std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("client", json(3)),
                  std::invalid_argument);
  CHECK_THROWS_AS(core::sagsdata_stored_value("name", json("x")),
                  std::invalid_argument);
}

TEST_CASE("Sagsdata_DanishDate_FormatsIsoDatesOnly", "[sagsdata]") {
  CHECK(core::danish_date("2026-09-21") == "21.09.2026");
  CHECK(core::danish_date("2026-09-21T10:00:00Z") == "21.09.2026");
  CHECK(core::danish_date("i morgen") == "i morgen");
}
