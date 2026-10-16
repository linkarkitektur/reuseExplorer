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

#include <stdexcept>
#include <string>

using namespace reusex::geometry;
using Catch::Matchers::ContainsSubstring;

namespace {
constexpr size_t kBudget = 2'000'000;

MipSubResult make_result(SolveStatus status, bool solved = true) {
  MipSubResult r;
  r.status = status;
  r.solved = solved;
  r.variables = 10;
  r.constraints = 5;
  return r;
}
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

// ── solve_mip_subset_with_fallback (reusex-8r0) ───────────────────────────
//
// reusex-8r0: a stub/broken CUDA driver makes cuOpt's RMM allocator throw
// std::bad_alloc (wrapping "cudaErrorStubLibrary") instead of returning a
// clean SolveStatus::error, and that exception used to escape
// solve_subset_with_fallback entirely — "auto" never got a chance to retry
// on HiGHS, so the whole mesh stage failed. These tests exercise the
// extracted, backend-independent fallback policy with injected callables
// that throw, so the bug is reproduced (and fixed) without linking cuOpt or
// building a real CellComplex.

TEST_CASE("MipFallback_SolverHighs_NeverCallsPrimary",
          "[geometry][solidifier][solver-fallback]") {
  bool primary_called = false;
  bool used_fallback = true; // must be reset to false
  bool size_guard = true;    // must be reset to false

  auto result = solve_mip_subset_with_fallback(
      SolverChoice::highs, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      /*estimated_variables=*/100, kBudget,
      [&]() -> MipSubResult {
        primary_called = true;
        throw std::bad_alloc();
      },
      [&] { return make_result(SolveStatus::optimal); }, used_fallback,
      size_guard);

  REQUIRE_FALSE(primary_called);
  REQUIRE(result.status == SolveStatus::optimal);
  REQUIRE_FALSE(used_fallback);
  REQUIRE_FALSE(size_guard);
}

TEST_CASE("MipFallback_Auto_PrimaryThrowsBadAlloc_FallsBackToHighs",
          "[geometry][solidifier][solver-fallback]") {
  // Reproduces the exact reusex-8r0 crash: cuOpt's RMM allocator throws
  // std::bad_alloc on a stub CUDA driver (cudaErrorStubLibrary).
  bool highs_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  auto result = solve_mip_subset_with_fallback(
      SolverChoice::automatic, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      /*estimated_variables=*/100, kBudget,
      [&]() -> MipSubResult { throw std::bad_alloc(); },
      [&] {
        highs_called = true;
        return make_result(SolveStatus::optimal);
      },
      used_fallback, size_guard);

  REQUIRE(highs_called);
  REQUIRE(used_fallback);
  REQUIRE_FALSE(size_guard);
  REQUIRE(result.status == SolveStatus::optimal);
  REQUIRE(result.solved);
}

TEST_CASE("MipFallback_Auto_PrimaryThrowsRuntimeError_FallsBackToHighs",
          "[geometry][solidifier][solver-fallback]") {
  // A different exception type (e.g. a CUDA runtime error surfaced as
  // std::runtime_error rather than std::bad_alloc) must also trigger the
  // fallback under auto — the catch must not be narrowly typed to bad_alloc.
  bool highs_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  auto result = solve_mip_subset_with_fallback(
      SolverChoice::automatic, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      /*estimated_variables=*/100, kBudget,
      [&]() -> MipSubResult {
        throw std::runtime_error("CUDA error: cudaErrorStubLibrary");
      },
      [&] {
        highs_called = true;
        return make_result(SolveStatus::optimal);
      },
      used_fallback, size_guard);

  REQUIRE(highs_called);
  REQUIRE(used_fallback);
  REQUIRE(result.status == SolveStatus::optimal);
}

TEST_CASE("MipFallback_Cuopt_PrimaryThrows_ReportsErrorInsteadOfFallingBack",
          "[geometry][solidifier][solver-fallback]") {
  // An explicit --solver cuopt must still report the error (not silently
  // fall back to HiGHS behind the user's back).
  bool highs_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  REQUIRE_THROWS_WITH(solve_mip_subset_with_fallback(
                          SolverChoice::cuopt, /*primary_is_cuopt=*/true,
                          /*subset_cells=*/10, /*estimated_variables=*/100,
                          kBudget,
                          [&]() -> MipSubResult { throw std::bad_alloc(); },
                          [&] {
                            highs_called = true;
                            return make_result(SolveStatus::optimal);
                          },
                          used_fallback, size_guard),
                      ContainsSubstring("alloc"));

  REQUIRE_FALSE(highs_called);
  REQUIRE_FALSE(used_fallback);
}

TEST_CASE("MipFallback_Auto_PrimaryReturnsErrorStatus_FallsBackToHighs",
          "[geometry][solidifier][solver-fallback]") {
  // Non-throwing hard-error path (pre-existing behavior, issue #226): a
  // SolveStatus::error return (no exception) must still trigger the retry
  // under auto.
  bool highs_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  auto result = solve_mip_subset_with_fallback(
      SolverChoice::automatic, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      /*estimated_variables=*/100, kBudget,
      [&] { return make_result(SolveStatus::error, /*solved=*/false); },
      [&] {
        highs_called = true;
        return make_result(SolveStatus::optimal);
      },
      used_fallback, size_guard);

  REQUIRE(highs_called);
  REQUIRE(used_fallback);
  REQUIRE(result.status == SolveStatus::optimal);
}

TEST_CASE("MipFallback_Cuopt_PrimaryReturnsErrorStatus_DoesNotFallBack",
          "[geometry][solidifier][solver-fallback]") {
  // Explicit --solver cuopt with a clean error status (no exception) must
  // not silently retry on HiGHS either (pre-existing behavior, issue #226).
  bool highs_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  auto result = solve_mip_subset_with_fallback(
      SolverChoice::cuopt, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      /*estimated_variables=*/100, kBudget,
      [&] { return make_result(SolveStatus::error, /*solved=*/false); },
      [&] {
        highs_called = true;
        return make_result(SolveStatus::optimal);
      },
      used_fallback, size_guard);

  REQUIRE_FALSE(highs_called);
  REQUIRE_FALSE(used_fallback);
  REQUIRE(result.status == SolveStatus::error);
  REQUIRE_FALSE(result.solved);
}

TEST_CASE("MipFallback_Auto_OverSizeGuard_NeverCallsPrimary",
          "[geometry][solidifier][solver-fallback]") {
  // The pre-emptive reusex-81y size guard must still short-circuit before
  // ever invoking the primary callable (no chance for it to throw).
  bool primary_called = false;
  bool used_fallback = false;
  bool size_guard = false;

  const size_t over_budget_vars = kBudget * 10;
  auto result = solve_mip_subset_with_fallback(
      SolverChoice::automatic, /*primary_is_cuopt=*/true, /*subset_cells=*/10,
      over_budget_vars, kBudget,
      [&]() -> MipSubResult {
        primary_called = true;
        throw std::bad_alloc();
      },
      [&] { return make_result(SolveStatus::optimal); }, used_fallback,
      size_guard);

  REQUIRE_FALSE(primary_called);
  REQUIRE(size_guard);
  REQUIRE_FALSE(used_fallback); // size guard, not an error-triggered retry
  REQUIRE(result.status == SolveStatus::optimal);
}
