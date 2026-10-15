// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the cuOpt size guard added for reusex-81y: cuOpt SIGSEGVs
// deep inside branch_and_bound_t::set_final_solution on MIP sections around
// ~3.3M variables (NewOffice, 2334 Leiden rooms). Solidifier::solve() used to
// log "Using MIP solver: cuOpt" unconditionally (a compile-time message) and
// hand every section to the compiled-in primary backend regardless of size
// or --solver. Both the backend-selection decision (select_mip_backend) and
// the "Using MIP solver: ..." log text (describe_mip_solver) are pure,
// backend-independent functions declared in Solidifier.hpp, so they are
// testable here without linking cuOpt/HiGHS or building a CellComplex, and
// without reproducing the segfault itself.

#include <reusex/reconstruction/Solidifier.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <string>

using namespace reusex::geometry;
using Catch::Matchers::ContainsSubstring;

namespace {
constexpr size_t kBudget = 2'000'000;
} // namespace

// ── select_mip_backend ──────────────────────────────────────────────────

TEST_CASE("SelectMipBackend_SolverHighs_AlwaysUsesHighsRegardlessOfSize",
          "[geometry][solidifier][solver-select]") {
  for (bool primary_is_cuopt : {false, true}) {
    for (size_t vars : {size_t{1}, kBudget, kBudget * 10}) {
      auto d = select_mip_backend(SolverChoice::highs, primary_is_cuopt, vars,
                                  kBudget);
      REQUIRE(d.use_highs);
      REQUIRE_FALSE(d.size_guard_tripped);
    }
  }
}

TEST_CASE("SelectMipBackend_SolverAuto_PrimaryHighsBuild_AlwaysHighsNoGuard",
          "[geometry][solidifier][solver-select]") {
  // No cuOpt compiled in: auto's compiled-in primary is already HiGHS, so
  // there is nothing to guard against — this is a build limitation, not a
  // size-triggered downgrade.
  auto under = select_mip_backend(SolverChoice::automatic,
                                  /*primary_is_cuopt=*/false, 100, kBudget);
  REQUIRE(under.use_highs);
  REQUIRE_FALSE(under.size_guard_tripped);

  auto over =
      select_mip_backend(SolverChoice::automatic,
                         /*primary_is_cuopt=*/false, kBudget * 10, kBudget);
  REQUIRE(over.use_highs);
  REQUIRE_FALSE(over.size_guard_tripped);
}

TEST_CASE("SelectMipBackend_SolverAuto_PrimaryCuopt_UnderBudgetUsesCuopt",
          "[geometry][solidifier][solver-select]") {
  auto d = select_mip_backend(SolverChoice::automatic,
                              /*primary_is_cuopt=*/true, kBudget - 1, kBudget);
  REQUIRE_FALSE(d.use_highs);
  REQUIRE_FALSE(d.size_guard_tripped);
}

TEST_CASE("SelectMipBackend_SolverAuto_PrimaryCuopt_AtBudgetUsesCuopt",
          "[geometry][solidifier][solver-select]") {
  // Boundary: exactly at the budget is NOT over it.
  auto d = select_mip_backend(SolverChoice::automatic,
                              /*primary_is_cuopt=*/true, kBudget, kBudget);
  REQUIRE_FALSE(d.use_highs);
  REQUIRE_FALSE(d.size_guard_tripped);
}

TEST_CASE("SelectMipBackend_SolverAuto_PrimaryCuopt_OverBudgetTripsSizeGuard",
          "[geometry][solidifier][solver-select]") {
  // This is the reusex-81y case: a 1402-cell x 2334-room section estimates
  // ~3.27M variables, well past a 2M-variable budget — auto must downgrade
  // to HiGHS pre-emptively instead of handing it to cuOpt.
  const size_t estimated_variables = 1402 * (2334 + 1);
  REQUIRE(estimated_variables > kBudget);

  auto d = select_mip_backend(SolverChoice::automatic,
                              /*primary_is_cuopt=*/true, estimated_variables,
                              kBudget);
  REQUIRE(d.use_highs);
  REQUIRE(d.size_guard_tripped);
}

TEST_CASE("SelectMipBackend_SolverCuopt_OverBudget_RespectsExplicitChoice",
          "[geometry][solidifier][solver-select]") {
  // An explicit --solver cuopt is always honored, even past the guard — the
  // caller is responsible for warning loudly about the risk (issue text:
  // "warn loudly with the sizes but respect the user's choice").
  auto d = select_mip_backend(SolverChoice::cuopt, /*primary_is_cuopt=*/true,
                              kBudget * 10, kBudget);
  REQUIRE_FALSE(d.use_highs);
  REQUIRE_FALSE(d.size_guard_tripped);
}

TEST_CASE(
    "SelectMipBackend_SolverCuopt_BuildWithoutCuopt_FallsBackToCompiledPrimary",
    "[geometry][solidifier][solver-select]") {
  // --solver cuopt on a build with no cuOpt compiled in: falls back to the
  // compiled primary (HiGHS), but that's a build limitation, not the size
  // guard, regardless of problem size.
  auto d = select_mip_backend(SolverChoice::cuopt, /*primary_is_cuopt=*/false,
                              100, kBudget);
  REQUIRE(d.use_highs);
  REQUIRE_FALSE(d.size_guard_tripped);
}

// ── describe_mip_solver (feeds the "Using MIP solver: ..." log line) ─────

TEST_CASE("DescribeMipSolver_SolverHighs_NamesForcedChoice",
          "[geometry][solidifier][solver-select]") {
  for (bool build_has_cuopt : {false, true}) {
    const auto desc =
        describe_mip_solver(SolverChoice::highs, build_has_cuopt, kBudget);
    REQUIRE_THAT(desc, ContainsSubstring("HiGHS"));
    REQUIRE_THAT(desc, ContainsSubstring("--solver highs"));
  }
}

TEST_CASE("DescribeMipSolver_SolverCuopt_BuildHasCuopt_NamesCuopt",
          "[geometry][solidifier][solver-select]") {
  const auto desc = describe_mip_solver(SolverChoice::cuopt,
                                        /*build_has_cuopt=*/true, kBudget);
  REQUIRE_THAT(desc, ContainsSubstring("cuOpt"));
  REQUIRE_THAT(desc, ContainsSubstring("--solver cuopt"));
}

TEST_CASE("DescribeMipSolver_SolverCuopt_BuildWithoutCuopt_NamesFallback",
          "[geometry][solidifier][solver-select]") {
  // This is the bug report: the old log line said "cuOpt" unconditionally
  // regardless of --solver or whether cuOpt is even compiled in.
  const auto desc = describe_mip_solver(SolverChoice::cuopt,
                                        /*build_has_cuopt=*/false, kBudget);
  REQUIRE_THAT(desc, ContainsSubstring("HiGHS"));
  REQUIRE_THAT(desc, ContainsSubstring("no cuOpt"));
}

TEST_CASE("DescribeMipSolver_SolverAuto_BuildWithoutCuopt_NamesHighs",
          "[geometry][solidifier][solver-select]") {
  const auto desc = describe_mip_solver(SolverChoice::automatic,
                                        /*build_has_cuopt=*/false, kBudget);
  REQUIRE_THAT(desc, ContainsSubstring("HiGHS"));
}

TEST_CASE("DescribeMipSolver_SolverAuto_BuildHasCuopt_NamesCuoptAndSizeGuard",
          "[geometry][solidifier][solver-select]") {
  // auto's description must name the actually-compiled primary (cuOpt) and
  // the size-guard budget that can downgrade a given section to HiGHS, not
  // a static "Using MIP solver: cuOpt" regardless of what will run.
  const auto desc = describe_mip_solver(SolverChoice::automatic,
                                        /*build_has_cuopt=*/true, kBudget);
  REQUIRE_THAT(desc, ContainsSubstring("cuOpt"));
  REQUIRE_THAT(desc, ContainsSubstring("auto"));
  REQUIRE_THAT(desc, ContainsSubstring(std::to_string(kBudget)));
}
