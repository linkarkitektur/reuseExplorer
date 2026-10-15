// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the Solidifier / CellComplex robustness guards added for
// issues #212 (MIP timeout + diagnostics) and #213 (cell-complex guards).

#include "core/logging.hpp"
#include <reusex/reconstruction/CellComplex.hpp>
#include <reusex/reconstruction/Solidifier.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

// Pull in whichever MIP backend the library was built with so the timeout
// plumbing can be exercised against the active solver (issue #212). cuOpt
// requires a GPU at runtime, so its test carries the [gpu] tag.
#if defined(USE_CUOPT)
#include <CGAL/cuOpt_mixed_integer_program_traits.h>
#define TIMEOUT_SOLVER_TAG "[geometry][solidifier][timeout][gpu]"
using TimeoutSolver = CGAL::cuOpt_mixed_integer_program_traits<double>;
#elif defined(USE_HIGHS)
#include <CGAL/HiGHS_mixed_integer_program_traits.h>
#define TIMEOUT_SOLVER_TAG "[geometry][solidifier][timeout]"
using TimeoutSolver = CGAL::HiGHS_mixed_integer_program_traits<double>;
#endif

#include <Eigen/Dense>

#include <chrono>

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace reusex::geometry;
using Catch::Matchers::ContainsSubstring;

namespace {

using PlaneVec =
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

// Build the plane set for an axis-aligned box spanning [0,w] x [0,d] x [0,h].
// Returns planes plus the vertical/horizontal index lists and wall pairs.
struct BoxPlanes {
  PlaneVec planes;
  std::vector<size_t> verticals;
  std::vector<size_t> horizontals;
  std::vector<std::pair<size_t, size_t>> pairs;
};

BoxPlanes make_box(double w = 4.0, double d = 3.0, double h = 2.5) {
  BoxPlanes b;
  // Vertical walls (normal in xy-plane), as (a,b,c,d) with a*x+b*y+c*z+d=0.
  b.planes.emplace_back(1, 0, 0, 0);  // x = 0     (id 0)
  b.planes.emplace_back(1, 0, 0, -w); // x = w     (id 1)
  b.planes.emplace_back(0, 1, 0, 0);  // y = 0     (id 2)
  b.planes.emplace_back(0, 1, 0, -d); // y = d     (id 3)
  b.verticals = {0, 1, 2, 3};
  // Horizontal floor/ceiling (normal in z).
  b.planes.emplace_back(0, 0, 1, 0);  // z = 0     (id 4)
  b.planes.emplace_back(0, 0, 1, -h); // z = h     (id 5)
  b.horizontals = {4, 5};
  // Opposing wall pairs.
  b.pairs = {{0, 1}, {2, 3}};
  return b;
}

// Construct a CellComplex for a single box. min/max add a 1m margin like the
// mesh() pipeline does.
std::shared_ptr<CellComplex> make_box_complex(size_t max_cells = 500) {
  auto b = make_box();
  return std::make_shared<CellComplex>(
      b.planes, b.verticals, b.horizontals, b.pairs,
      std::array<double, 2>{-1.0, -1.0}, std::array<double, 2>{5.0, 4.0},
      std::nullopt, max_cells);
}

} // namespace

TEST_CASE("Solidifier_MissingPropertyMaps_ThrowsDescriptiveError",
          "[geometry][solidifier][guards]") {
  // A freshly-constructed cell complex has "f:area"/"c:volume" (written by the
  // constructor) but NOT "f:support_probability" or "c:room_probabilities"
  // (those come from compute_face_coverage / compute_room_probabilities).
  // Solidifier::solve() must fail fast with a descriptive error naming the
  // missing map rather than building a malformed MIP.
  auto cc = make_box_complex();

  Solidifier solidifier(cc);
  REQUIRE_THROWS_WITH(solidifier.solve(),
                      ContainsSubstring("property map") &&
                          ContainsSubstring("c:room_probabilities"));
}

TEST_CASE("CellComplex_ZeroBoundedFaces_ThrowsDescriptiveErrorNamingCause",
          "[geometry][cellcomplex][guards]") {
  // Two parallel vertical planes with no perpendicular wall cannot enclose
  // any area: the 2D arrangement has 0 bounded faces, so the projected cell
  // count is 0. Construction must fail fast here, naming the real cause
  // (no bounded faces, the plane counts) and pointing at `create planes` —
  // not continue on to build a 0-cell complex and let Solidifier::solve()
  // run a vacuous MIP that reports the misleading "increase --time-limit /
  // use --solver highs / stronger plane merging" hint (reusex-qw0). Because
  // construction throws, Solidifier is never constructed and no solve is
  // ever attempted for this case.
  PlaneVec planes;
  planes.emplace_back(1, 0, 0, 0);  // x = 0
  planes.emplace_back(1, 0, 0, -4); // x = 4 (parallel: no perpendicular pair)
  std::vector<size_t> verticals = {0, 1};

  planes.emplace_back(0, 0, 1, 0);    // z = 0
  planes.emplace_back(0, 0, 1, -2.5); // z = 2.5
  std::vector<size_t> horizontals = {2, 3};

  std::vector<std::pair<size_t, size_t>> pairs{}; // no enclosing wall pairs

  REQUIRE_THROWS_WITH(
      std::make_shared<CellComplex>(planes, verticals, horizontals, pairs,
                                    std::array<double, 2>{-1.0, -1.0},
                                    std::array<double, 2>{5.0, 3.5},
                                    std::nullopt, /*max_cells=*/500),
      ContainsSubstring("0 bounded faces") && ContainsSubstring("2 vertical") &&
          ContainsSubstring("2 horizontal") &&
          ContainsSubstring("create planes"));
}

TEST_CASE("CellComplex_MaxCellsExceededByFragmentation_ThrowsDescriptiveError",
          "[geometry][cellcomplex][guards]") {
  // A tiny max_cells makes even the single-box arrangement exceed the limit,
  // so construction must throw with an actionable message.
  auto b = make_box();
  REQUIRE_THROWS_WITH(
      std::make_shared<CellComplex>(b.planes, b.verticals, b.horizontals,
                                    b.pairs, std::array<double, 2>{-1.0, -1.0},
                                    std::array<double, 2>{5.0, 4.0},
                                    std::nullopt, /*max_cells=*/0),
      ContainsSubstring("max_cells") && ContainsSubstring("merging"));
}

TEST_CASE("SolveStatus_ToString_StringifiesDistinctFailureModes",
          "[geometry][solidifier]") {
  // The status enum must round-trip through to_string so log lines can name
  // the failure mode (timeout vs infeasible) distinctly.
  REQUIRE(std::string(to_string(SolveStatus::time_limit)) == "time_limit");
  REQUIRE(std::string(to_string(SolveStatus::infeasible)) == "infeasible");
  REQUIRE(std::string(to_string(SolveStatus::optimal)) == "optimal");
  REQUIRE(std::string(to_string(SolveStatus::not_solved)) == "not_solved");
}

#if defined(USE_CUOPT) || defined(USE_HIGHS)
TEST_CASE("MipTraits_TinyTimeLimit_ReturnsClassifiedStatusWithoutHanging",
          TIMEOUT_SOLVER_TAG) {
  // Build a nontrivial 0/1 knapsack-style MIP and give the solver a tiny time
  // limit. The important property (issue #212) is that solve() *returns*
  // promptly with a classified status instead of running unbounded. On this
  // small instance the solver may still finish optimally; either way the call
  // must return quickly and report a status, never hang.
  using Solver = TimeoutSolver;
  using Variable = Solver::Variable;
  using Linear_objective = Solver::Linear_objective;
  using Linear_constraint = Solver::Linear_constraint;

  Solver solver;
  solver.set_time_limit(0.01); // 10 ms

  const int n = 60;
  std::vector<Variable *> vars;
  vars.reserve(n);
  Linear_objective *obj = solver.create_objective(Linear_objective::MAXIMIZE);
  Linear_constraint *cap = solver.create_constraint(
      -Linear_constraint::infinity(), static_cast<double>(n) / 2.0, "capacity");
  for (int i = 0; i < n; ++i) {
    Variable *v =
        solver.create_variable(Variable::BINARY, 0, 1, "x" + std::to_string(i));
    vars.push_back(v);
    obj->add_coefficient(v, 1.0 + 0.01 * (i % 7));  // slightly varied values
    cap->add_coefficient(v, 1.0 + 0.001 * (i % 5)); // slightly varied weights
  }

  const auto t0 = std::chrono::steady_clock::now();
  bool ok = solver.solve();
  const auto elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();

  // Must not hang: with a 10 ms limit the solve must return well within a
  // few seconds (generous bound to avoid CI flakiness).
  REQUIRE(elapsed < 30.0);

  const auto status = solver.solve_status();
  reusex::core::info("MIP timeout test: ok={}, elapsed={:.3f}s, status={}", ok,
                     elapsed, CGAL::to_string(status));

  // The solve reached a definite classified outcome (not the pre-solve
  // sentinel), proving the time-limit plumbing is wired through.
  REQUIRE(status != CGAL::MIP_solve_status::not_solved);
}
#endif // USE_CUOPT || USE_HIGHS
