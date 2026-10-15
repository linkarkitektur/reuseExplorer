// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22 samples: codes, stage/result storage, link replacement,
// per-type lookup, and cascade on type deletion; schema v31 part links.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace core = reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_samples") {}
};
int64_t add_type(ProjectDB &db, const char *name) {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  return db.add_survey_type(t).id;
}
} // namespace

TEST_CASE("Samples_Add_AssignsSequentialCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a =
      db.add_sample("PCB i fugemasse", "Fugemasse omkring vinduespartier");
  const auto b = db.add_sample("Bly i maling", "Malede indervægge");
  CHECK(a.code == "P-01");
  CHECK(b.code == "P-02");
  CHECK(a.stage == core::SampleStage::planlagt);
  CHECK(a.result == core::SampleResult::none);
  CHECK(db.samples().size() == 2);
}

TEST_CASE("Samples_Delete_DoesNotReuseCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_sample("a", "");
  const auto b = db.add_sample("b", "");
  CHECK(db.delete_sample(b.id));
  CHECK_FALSE(db.delete_sample(b.id));
  // A lab report quoting P-02 must never come to mean a different sample.
  CHECK(db.add_sample("c", "").code == "P-03");
}

TEST_CASE("Samples_UpdateStageAndResult", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("Asbest i linoleumslim", "Gulvlim, Office Zone");
  ProjectDB::SamplePatch p;
  p.stage = core::SampleStage::svar;
  p.result = core::SampleResult::forurenet;
  const auto u = db.update_sample(s.id, p);
  CHECK(u.stage == core::SampleStage::svar);
  CHECK(u.result == core::SampleResult::forurenet);
  CHECK(u.title == "Asbest i linoleumslim");
  CHECK_THROWS_AS(db.update_sample(s.id + 50, p), std::out_of_range);
}

TEST_CASE("Samples_Links_ReplaceSet_PerTypeLookup_AtomicOnUnknownType",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto walls = add_type(db, "Indvendige murvægge, malet");
  const auto windows = add_type(db, "Vinduespartier");
  const auto s = db.add_sample("Bly i maling", "");
  db.set_sample_links(s.id, {windows, walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls, windows});
  db.set_sample_links(s.id, {walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls});
  CHECK(db.samples_for_type(walls).size() == 1);
  CHECK(db.samples_for_type(windows).empty());
  CHECK_THROWS_AS(db.set_sample_links(s.id, {walls, 9999}), std::out_of_range);
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls}); // unchanged
}

TEST_CASE("Samples_Add_BadTypeOrStage_ThrowsAndWritesNothing",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(db.add_sample("x", "", {9999}), std::out_of_range);
  CHECK_THROWS_AS(db.add_sample("x", "", {}, core::SampleStage::svar),
                  std::invalid_argument);
  CHECK_THROWS_AS(db.add_sample("x", "", {}, core::SampleStage::sendt),
                  std::invalid_argument);
  CHECK(db.samples().empty());
  CHECK(db.add_sample("y", "").code == "P-01");
}

TEST_CASE("Samples_Add_FailureMidTransaction_RollsBack",
          "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Fuger");
  auto exec_raw = [&](const char *sql) {
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  };
  // The sample row is inserted, then the link insert fails: the row must go.
  exec_raw("CREATE TRIGGER fail_link BEFORE INSERT ON sample_links "
           "BEGIN SELECT RAISE(ABORT,'forced'); END;");
  CHECK_THROWS_AS(db.add_sample("x", "", {t}), std::runtime_error);
  CHECK(db.samples().empty());
  exec_raw("DROP TRIGGER fail_link;");
  // The rolled-back insert consumed no P-## code.
  CHECK(db.add_sample("y", "", {t}).code == "P-01");
}

TEST_CASE("Samples_Add_TypesAndStage_NoPartCode", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduespartier, aluminium");
  const auto u = add_type(db, "Fuger");
  const auto s =
      db.add_sample("Asbest", "", {u, t, u}, core::SampleStage::udtaget);
  CHECK(s.type_ids == std::vector<int64_t>{t, u});
  CHECK(s.stage == core::SampleStage::udtaget);
  CHECK(db.samples_for_type(t).size() == 1);
}

// --- Schema v31: samples linked to specific survey parts -------------------

namespace {
int64_t count_part_links(const reusex::test_support::TempPath &tmp) {
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
  sqlite3_stmt *s = nullptr;
  REQUIRE(sqlite3_prepare_v2(raw, "SELECT COUNT(*) FROM sample_part_links;", -1,
                             &s, nullptr) == SQLITE_OK);
  REQUIRE(sqlite3_step(s) == SQLITE_ROW);
  const int64_t n = sqlite3_column_int64(s, 0);
  sqlite3_finalize(s);
  sqlite3_close(raw);
  return n;
}
} // namespace

TEST_CASE("Samples_PartLinks_AddSetReplaceAndClear",
          "[ProjectDB][samples][sample_part_links]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduespartier");
  reusex::test_support::make_part(db, "RX-002", t);
  reusex::test_support::make_part(db, "RX-001", t);
  const auto s = db.add_sample("PCB", "", {}, core::SampleStage::planlagt,
                               {"RX-002", "RX-001", "RX-002"});
  // Collapsed and sorted; no type link was asked for.
  CHECK(s.part_codes == std::vector<std::string>{"RX-001", "RX-002"});
  CHECK(s.type_ids.empty());

  db.set_sample_part_links(s.id, {"RX-002"});
  CHECK(db.sample(s.id)->part_codes == std::vector<std::string>{"RX-002"});

  // set_sample_links without part codes leaves them alone ...
  db.set_sample_links(s.id, {t});
  CHECK(db.sample(s.id)->part_codes == std::vector<std::string>{"RX-002"});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{t});
  // ... with them it replaces both sets; set_sample_part_links keeps types.
  db.set_sample_links(s.id, {}, std::vector<std::string>{"RX-001"});
  CHECK(db.sample(s.id)->type_ids.empty());
  CHECK(db.sample(s.id)->part_codes == std::vector<std::string>{"RX-001"});
  db.set_sample_links(s.id, {t});
  db.set_sample_part_links(s.id, {});
  CHECK(db.sample(s.id)->part_codes.empty());
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{t});
  CHECK(db.samples().front().type_ids == std::vector<int64_t>{t});
}

TEST_CASE("Samples_PartLinks_UnknownCode_ThrowsAndWritesNothing",
          "[ProjectDB][samples][sample_part_links]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Fuger");
  const auto u = add_type(db, "Vægge");
  reusex::test_support::make_part(db, "RX-001", t);
  CHECK_THROWS_AS(db.add_sample("x", "", {t}, core::SampleStage::planlagt,
                                {"RX-001", "RX-999"}),
                  std::out_of_range);
  CHECK(db.samples().empty());
  // The rejected request consumed no P-## code.
  const auto s =
      db.add_sample("y", "", {t}, core::SampleStage::planlagt, {"RX-001"});
  CHECK(s.code == "P-01");

  CHECK_THROWS_AS(db.set_sample_part_links(s.id, {"RX-999"}),
                  std::out_of_range);
  // A bad part code leaves the type links untouched too: one transaction.
  CHECK_THROWS_AS(
      db.set_sample_links(s.id, {u}, std::vector<std::string>{"nope"}),
      std::out_of_range);
  CHECK_THROWS_AS(db.set_sample_part_links(s.id + 50, {"RX-001"}),
                  std::out_of_range);
  const auto after = db.sample(s.id);
  CHECK(after->type_ids == std::vector<int64_t>{t});
  CHECK(after->part_codes == std::vector<std::string>{"RX-001"});
  try {
    db.set_sample_part_links(s.id, {"RX-999"});
  } catch (const std::out_of_range &e) {
    CHECK(std::string(e.what()) == "no survey part 'RX-999'");
  }
}

TEST_CASE("Samples_PartLinks_CascadeOnPartTypeAndSampleDelete",
          "[ProjectDB][samples][sample_part_links]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Fuger");
  const auto u = add_type(db, "Vægge");
  reusex::test_support::make_part(db, "RX-001", t);
  reusex::test_support::make_part(db, "RX-002", t);
  reusex::test_support::make_part(db, "RX-003", u);
  const auto a = db.add_sample("a", "", {}, core::SampleStage::planlagt,
                               {"RX-001", "RX-002", "RX-003"});
  const auto b =
      db.add_sample("b", "", {}, core::SampleStage::planlagt, {"RX-003"});
  CHECK(count_part_links(tmp) == 4);

  // Deleting a part drops its links, not the sample.
  db.delete_survey_part("RX-001");
  CHECK(db.sample(a.id)->part_codes ==
        std::vector<std::string>{"RX-002", "RX-003"});
  // Deleting a type cascades to its parts and on to their links.
  db.delete_survey_type(t);
  CHECK(db.sample(a.id)->part_codes == std::vector<std::string>{"RX-003"});
  // Deleting a sample drops its links; the part stays.
  CHECK(db.delete_sample(a.id));
  CHECK(count_part_links(tmp) == 1);
  CHECK(db.sample(b.id)->part_codes == std::vector<std::string>{"RX-003"});
  CHECK(db.survey_part("RX-003").has_value());
}
