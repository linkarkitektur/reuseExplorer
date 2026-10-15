// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/reconstruction/CellComplex.hpp"

#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace reusex::geometry {

/**
 * @brief Backend-agnostic classification of the last MIP solve outcome.
 *
 * Lets callers distinguish *why* a solve failed (infeasible vs. time-limit
 * vs. error) without depending on the underlying solver's status enum.
 */
enum class SolveStatus {
  not_solved,
  optimal,
  feasible,
  infeasible,
  unbounded,
  time_limit,
  iteration_limit,
  numerical_error,
  error
};

/// Human-readable name for a SolveStatus (for logging).
const char *to_string(SolveStatus status);

/// Which MIP backend the Solidifier should use for each (sub)problem.
///
/// - auto:  use the compiled-in primary backend (cuOpt if built with GPU,
///          else HiGHS) and, on a solver *error/OOM*, retry the same problem
///          with HiGHS on the CPU before giving up (issue #226).
/// - cuopt: force the GPU backend (falls back to the compiled primary if the
///          build has no cuOpt).
/// - highs: force the CPU HiGHS backend.
enum class SolverChoice { automatic, cuopt, highs };

/// Parse a --solver CLI string ("auto"/"cuopt"/"highs") into a SolverChoice.
/// Throws std::runtime_error on an unrecognized value.
SolverChoice parse_solver_choice(const std::string &s);

/// Outcome of select_mip_backend(): which backend to run a (sub)problem on,
/// and whether that was an automatic size-based downgrade.
struct BackendDecision {
  /// true => run the HiGHS (CPU) backend for this (sub)problem; false => run
  /// the compiled-in primary backend (cuOpt when the build has it).
  bool use_highs = true;
  /// true => `solver == automatic` picked HiGHS pre-emptively because the
  /// (sub)problem's estimated variable count exceeds the cuOpt size guard,
  /// not because of a solver error. Always false for `highs`/`cuopt`.
  bool size_guard_tripped = false;
};

/// Decide which MIP backend to run a (sub)problem on, given the requested
/// --solver choice, whether this build's compiled-in primary backend is
/// cuOpt, the (sub)problem's variable count, and the configured size guard.
///
/// cuOpt is known to SIGSEGV deep inside
/// `branch_and_bound_t::set_final_solution` on sections around ~3.3M
/// variables (reusex-81y); `automatic` pre-emptively downgrades a section
/// over @p cuopt_max_variables to HiGHS instead of handing it to cuOpt.
/// `cuopt` is an explicit user choice and is always respected (the caller
/// should still warn loudly when the size guard would otherwise have
/// tripped). Pure and backend-independent, so it is unit-testable without
/// linking cuOpt or HiGHS.
BackendDecision select_mip_backend(SolverChoice solver, bool primary_is_cuopt,
                                   size_t estimated_variables,
                                   size_t cuopt_max_variables);

/// Human-readable description of the MIP solver `solve()` will actually use,
/// for the "Using MIP solver: ..." log line. Reflects the --solver choice at
/// run time (not just which backend is compiled in), and names the size
/// guard for `automatic`. Pure (no CGAL/cuOpt dependency), so it is
/// unit-testable on its own.
std::string describe_mip_solver(SolverChoice solver, bool build_has_cuopt,
                                size_t cuopt_max_variables);

struct SolidifierOptions {
  /// Wall-clock time limit for the MIP solve, in seconds. Guards against the
  /// solver hanging indefinitely on complex buildings (issue #212). When
  /// solving per section, this limit applies to *each* section's solve.
  double time_limit_seconds = 120.0;

  /// Weight on the wall/complexity term of the objective (was hardcoded).
  double alpha = 0.04;

  /// Relative MIP gap tolerance. Negative => use the solver default.
  double mip_gap = -1.0;

  /// Solve the MIP per horizontal section (storey) instead of monolithically,
  /// when the arrangement has more than one section and enough cells
  /// (issue #226). Each section is an independent subproblem (~500 cells on
  /// the measured multi-room scan) whose labels are merged into one global
  /// solution; shared horizontal faces at section joints are emitted once by
  /// toMesh() because they separate an interior cell from an exterior one.
  bool sectioned = true;

  /// Only take the sectioned path when the monolithic problem would have at
  /// least this many cells. Below it the one-shot solve is fast and the
  /// original (well-tested) path is used unchanged (issue #226).
  size_t sectioned_threshold = 2000;

  /// Solver backend selection + fallback policy (issue #226).
  SolverChoice solver = SolverChoice::automatic;

  /// Size guard for `solver == automatic` (reusex-81y): a (sub)problem whose
  /// *exact* variable count (cells in the subset times n_rooms+1+n_walls)
  /// exceeds this is solved with HiGHS instead of cuOpt, pre-emptively —
  /// cuOpt SIGSEGVs inside `branch_and_bound_t::set_final_solution` on
  /// sections around ~3.3M variables, well past the point where it would
  /// have returned a clean error status. Ignored for `highs`; with an
  /// explicit `cuopt` the choice is respected (just logged loudly).
  size_t cuopt_max_variables = 2'000'000;
};

/// Per-section solve diagnostics (issue #226). One entry per section actually
/// solved; logged at info level and queryable for tests/tools.
struct SectionSolveStats {
  int section = -1;       ///< Section (storey) index.
  size_t cells = 0;       ///< Cells in this section's subproblem.
  size_t variables = 0;   ///< MIP variables created.
  size_t constraints = 0; ///< MIP constraints created.
  double seconds = 0.0;   ///< Wall-clock solve time.
  SolveStatus status = SolveStatus::not_solved; ///< Outcome.
  bool used_highs_fallback = false; ///< True if HiGHS retried after primary.
  /// True if `solver == automatic` pre-emptively ran this section on HiGHS
  /// because its size exceeded `cuopt_max_variables` (reusex-81y) — distinct
  /// from `used_highs_fallback`, which means cuOpt was tried and errored.
  bool size_guard_tripped = false;
};

/**
 * @brief Solidifier solves room segmentation using Mixed Integer Programming
 *
 * Uses PIMPL idiom to hide CGAL MIP solver implementation details.
 * This significantly reduces compile times by not exposing CGAL headers.
 */
class Solidifier {
    public:
  using Fd = CellComplex::Vertex;
  using Cd = CellComplex::Vertex;

  Solidifier() = delete;
  explicit Solidifier(std::shared_ptr<const CellComplex> cc,
                      SolidifierOptions options = SolidifierOptions{});
  ~Solidifier();

  // Delete copy (contains unique_ptr to incomplete type)
  Solidifier(const Solidifier &) = delete;
  Solidifier &operator=(const Solidifier &) = delete;

  // Delete move (simpler, could be implemented if needed)
  Solidifier(Solidifier &&) = delete;
  Solidifier &operator=(Solidifier &&) = delete;

  /**
   * @brief Solve the MIP problem for room segmentation
   * @return Optional pair of room labels and wall labels, or nullopt if solving
   * fails
   */
  std::optional<std::pair<std::unordered_map<Cd, int>,
                          std::unordered_map<Cd, std::set<int>>>>
  solve();

  /**
   * @brief Status of the most recent solve() call.
   *
   * Distinguishes timeout from infeasibility from a modelling error so the
   * caller can report an actionable reason on failure (issue #212).
   */
  SolveStatus last_solve_status() const;

  /**
   * @brief Per-section solve diagnostics from the most recent solve().
   *
   * Empty when the monolithic path was taken; otherwise one entry per section
   * that was attempted (issue #226).
   */
  const std::vector<SectionSolveStats> &section_stats() const;

  /**
   * @brief Convert solved cell complex to mesh
   * @param filter Function to filter which cells to include in mesh
   * @return Pair of vertex matrix and face matrix (Eigen format)
   */
  std::pair<Eigen::MatrixXd, Eigen::MatrixXi>
  toMesh(std::function<bool(const Cd)> filter);

    protected:
  // Internal accessor for implementations in separate compilation units
  std::shared_ptr<const CellComplex> get_cell_complex() const;

    private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace reusex::geometry
