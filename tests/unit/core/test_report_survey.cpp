// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The PDF report's survey section (GUI Phase 5, R8): which types it lists,
// their summed quantities, the Danish labels, the blocking count stored with a
// version and — where typst is installed — that the template compiles.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>
#include <core/report_generator.hpp>
#include <core/resource_templates.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using reusex::ProjectDB;
namespace core = reusex::core;
using Catch::Approx;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_report_survey") {}
};

int64_t add_type(ProjectDB &db, const char *name, core::Treatment tr,
                 std::optional<double> mass, core::ReviewStatus st) {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  t.treatment = tr;
  t.mass_t = mass;
  t.review_status = st;
  t.eak_code = "17.01.01";
  t.bim7aa_code = "211 Ydervægge";
  t.unit = "m²";
  return db.add_survey_type(t).id;
}

void add_part(ProjectDB &db, const char *code, int64_t type, double qty) {
  db.add_survey_part({code,
                      type,
                      std::nullopt,
                      std::nullopt,
                      std::nullopt,
                      "Facade",
                      qty,
                      false,
                      "",
                      {},
                      {}});
}

/// Link a sample that has not been answered yet: the type reads `afventer`.
void await_sample(ProjectDB &db, int64_t type) {
  const auto s = db.add_sample("PCB i fugemasse", "");
  db.set_sample_links(s.id, {type});
}

} // namespace

TEST_CASE("ReportSurveyRows_ApprovedOnly_QuantitySummed", "[report][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto facade =
      add_type(db, "Facadeelementer, sandwich", core::Treatment::genanvendelse,
               190, core::ReviewStatus::approved);
  add_type(db, "Betonsøjler, bærende", core::Treatment::genbrug, 58,
           core::ReviewStatus::queue);
  add_type(db, "Fejldetektion", core::Treatment::genbrug, 1,
           core::ReviewStatus::rejected);
  add_part(db, "RX-005", facade, 340);
  add_part(db, "RX-006", facade, 280);

  const auto rows = core::report_survey_rows(db);
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].name == "Facadeelementer, sandwich");
  CHECK(rows[0].bim7aa_code == "211 Ydervægge");
  CHECK(rows[0].quantity == Approx(620));
  CHECK(rows[0].unit == "m²");
  REQUIRE(rows[0].mass_t.has_value());
  CHECK(*rows[0].mass_t == Approx(190));
  CHECK(rows[0].treatment == core::Treatment::genanvendelse);
  CHECK(rows[0].environment == core::EnvironmentStatus::ren_screening);
}

TEST_CASE("ReportSurveyRows_WithholdApprovedTypeAwaitingSample",
          "[report][survey]") {
  // F2: an approved type whose sample is still out is not reportable — its
  // answer can make it contaminated — so neither the table nor the "approved"
  // circularity totals may include it.
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "Facadeelementer, sandwich", core::Treatment::genanvendelse, 190,
           core::ReviewStatus::approved);
  const auto pending =
      add_type(db, "Fuger, PCB-mistanke", core::Treatment::genbrug, 12,
               core::ReviewStatus::approved);
  await_sample(db, pending);
  REQUIRE(core::environment_status_of(db, pending) ==
          core::EnvironmentStatus::afventer);

  const auto rows = core::report_survey_rows(db);
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].name == "Facadeelementer, sandwich");

  // The circularity line is built from exactly these rows.
  std::vector<core::TypeTotals> approved;
  for (const auto &r : rows)
    approved.push_back({r.treatment, core::ReviewStatus::approved, r.mass_t,
                        r.eak_code, r.environment});
  const auto breakdown = core::circularity_breakdown(approved);
  CHECK(breakdown[static_cast<std::size_t>(core::Treatment::genbrug)] == 0.0);
  CHECK(breakdown[static_cast<std::size_t>(core::Treatment::genanvendelse)] ==
        Approx(190));

  // ...and it still counts against completeness, as a sample blocker.
  CHECK(reusex::report_blocking_types(db) == 1);
}

TEST_CASE("ReportSurveyRows_AgreeWithFractionsOnMissingTonnes",
          "[report][survey]") {
  // An approved non-bevaring type without tonnes blocks the waste report
  // (reason `mass`), so it is withheld from the rows too. Bevaring is not
  // waste and never blocks; without tonnes it is listed with no mass.
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "Isolering, ukendt mængde", core::Treatment::bortskaffelse,
           std::nullopt, core::ReviewStatus::approved);
  add_type(db, "Fundamenter", core::Treatment::bevaring, std::nullopt,
           core::ReviewStatus::approved);

  const auto rows = core::report_survey_rows(db);
  REQUIRE(rows.size() == 1);
  CHECK(rows[0].name == "Fundamenter");
  CHECK_FALSE(rows[0].mass_t.has_value());
  CHECK(reusex::report_blocking_types(db) == 1);
}

TEST_CASE("ReportBlockingTypes_CountsUnapprovedNonRejected",
          "[report][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::genanvendelse, 1,
           core::ReviewStatus::approved);
  add_type(db, "b", core::Treatment::genbrug, 1, core::ReviewStatus::queue);
  add_type(db, "c", core::Treatment::genbrug, 1, core::ReviewStatus::rejected);
  CHECK(reusex::report_blocking_types(db) == 1);
}

TEST_CASE("DanishLabels_MatchTheFrontendVocabulary", "[report][survey]") {
  // Mirrors the rux-frontend repo's src/kortlaegning/vocab.ts.
  CHECK(core::treatment_label_da(core::Treatment::bevaring) == "Bevaring");
  CHECK(core::treatment_label_da(core::Treatment::nyttiggoerelse) ==
        "Nyttiggørelse");
  CHECK(core::environment_label_da(core::EnvironmentStatus::afventer) ==
        "Afventer prøve");
  CHECK(core::environment_label_da(core::EnvironmentStatus::ren_proevesvar) ==
        "Ren (prøvesvar)");
}

TEST_CASE("ReportPdf_WithSurveySection_Compiles", "[report][typst]") {
  if (std::system("command -v typst > /dev/null 2>&1") != 0)
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto facade =
      add_type(db, "Facadeelementer, sandwich", core::Treatment::genanvendelse,
               190, core::ReviewStatus::approved);
  add_part(db, "RX-005", facade, 340);
  add_type(db, "Betonsøjler, bærende", core::Treatment::genbrug, 58,
           core::ReviewStatus::queue);
  // A name that is Typst markup and code. It reaches the template as data,
  // so it prints literally; were it evaluated, #panic would fail the compile.
  add_type(db, "#panic(\"x\") *y*", core::Treatment::genanvendelse, 4,
           core::ReviewStatus::approved);
  REQUIRE(core::report_survey_rows(db).size() == 2);

  const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db);
  REQUIRE(pdf.size() > 4);
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}

TEST_CASE("ReportPdf_UnknownResourceTemplate_ThrowsBeforeTypst",
          "[report][resources]") {
  // Checked before any typst work, so it holds on machines without typst.
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK_THROWS_AS(reusex::generate_ressourcekortlaegning_pdf(db, 999),
                  std::out_of_range);
}

TEST_CASE("ReportPdf_WithResourceTable_Compiles", "[report][typst]") {
  if (std::system("command -v typst > /dev/null 2>&1") != 0)
    SKIP("typst is not on PATH (the nix devshell provides it)");
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t =
      add_type(db, "#panic(\"x\") *y*", core::Treatment::genanvendelse, 4,
               core::ReviewStatus::approved);
  add_part(db, "RX-001", t, 3);
  add_part(db, "RX-002", t, 5);
  int64_t screening = 0;
  for (const auto &rec : db.resource_templates())
    if (rec.seed == std::optional<std::string>("screening"))
      screening = rec.id;
  REQUIRE(screening > 0);
  // 11 screening keys: Betegnelse + 10 others -> two stacked tables.
  const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db, screening);
  REQUIRE(pdf.size() > 4);
  CHECK(std::string(pdf.begin(), pdf.begin() + 4) == "%PDF");
}
