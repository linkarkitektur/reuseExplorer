// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reconstruction/Solidifier.hpp"
#include "core/logging.hpp"

// Both traits headers are always available (both are vendored in
// extern/include/CGAL and HiGHS is always linked). The HiGHS traits class body
// is guarded by USE_HIGHS, so force-enable it here even in a cuOpt build to
// make the CPU fallback backend usable (mirrors test_solver_comparison.cpp).
// Including both lets the Solidifier use cuOpt as the primary GPU backend and
// fall back to the CPU HiGHS backend on a solver error/OOM (issue #226).
#ifndef USE_HIGHS
#define USE_HIGHS
#endif
#include <CGAL/HiGHS_mixed_integer_program_traits.h>
#ifdef USE_CUOPT
#include <CGAL/cuOpt_mixed_integer_program_traits.h>
using PrimarySolver = CGAL::cuOpt_mixed_integer_program_traits<double>;
#else
using PrimarySolver = CGAL::HiGHS_mixed_integer_program_traits<double>;
#endif
using HighsSolver = CGAL::HiGHS_mixed_integer_program_traits<double>;

#include <algorithm>
#include <cmath>
#include <exception>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#define SolverDebug 0

namespace reusex::geometry {

namespace {
/// Map the solver-backend status enum to the public Solidifier SolveStatus.
SolveStatus map_status(CGAL::MIP_solve_status s) {
  switch (s) {
  case CGAL::MIP_solve_status::not_solved:
    return SolveStatus::not_solved;
  case CGAL::MIP_solve_status::optimal:
    return SolveStatus::optimal;
  case CGAL::MIP_solve_status::feasible:
    return SolveStatus::feasible;
  case CGAL::MIP_solve_status::infeasible:
    return SolveStatus::infeasible;
  case CGAL::MIP_solve_status::unbounded:
    return SolveStatus::unbounded;
  case CGAL::MIP_solve_status::time_limit:
    return SolveStatus::time_limit;
  case CGAL::MIP_solve_status::iteration_limit:
    return SolveStatus::iteration_limit;
  case CGAL::MIP_solve_status::numerical_error:
    return SolveStatus::numerical_error;
  case CGAL::MIP_solve_status::error:
    return SolveStatus::error;
  }
  return SolveStatus::error;
}

// Set operators used by the wall-consistency constraints.
template <typename T>
std::set<T> operator-(const std::set<T> &a, const std::set<T> &b) {
  std::set<T> result;
  std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
                      std::inserter(result, result.begin()));
  return result;
}
template <typename T>
std::set<T> operator+(const std::set<T> &a, const std::set<T> &b) {
  std::set<T> result;
  std::set_union(a.begin(), a.end(), b.begin(), b.end(),
                 std::inserter(result, result.begin()));
  return result;
}
} // namespace

const char *to_string(SolveStatus status) {
  switch (status) {
  case SolveStatus::not_solved:
    return "not_solved";
  case SolveStatus::optimal:
    return "optimal";
  case SolveStatus::feasible:
    return "feasible";
  case SolveStatus::infeasible:
    return "infeasible";
  case SolveStatus::unbounded:
    return "unbounded";
  case SolveStatus::time_limit:
    return "time_limit";
  case SolveStatus::iteration_limit:
    return "iteration_limit";
  case SolveStatus::numerical_error:
    return "numerical_error";
  case SolveStatus::error:
    return "error";
  }
  return "unknown";
}

SolverChoice parse_solver_choice(const std::string &s) {
  if (s == "auto")
    return SolverChoice::automatic;
  if (s == "cuopt")
    return SolverChoice::cuopt;
  if (s == "highs")
    return SolverChoice::highs;
  throw std::runtime_error(fmt::format(
      "invalid --solver '{}': expected one of auto, cuopt, highs", s));
}

BackendDecision select_mip_backend(SolverChoice solver, bool primary_is_cuopt,
                                   size_t estimated_variables,
                                   size_t cuopt_max_variables) {
  BackendDecision d;
  switch (solver) {
  case SolverChoice::highs:
    d.use_highs = true;
    return d;
  case SolverChoice::cuopt:
    // Explicit choice: always respected. If this build has no cuOpt
    // compiled in as the primary backend there is nothing to force onto —
    // HiGHS runs because it's the compiled primary, not because of a size
    // guard, so size_guard_tripped stays false either way.
    d.use_highs = !primary_is_cuopt;
    return d;
  case SolverChoice::automatic:
  default:
    if (!primary_is_cuopt) {
      d.use_highs = true; // the compiled-in primary is already HiGHS
      return d;
    }
    if (estimated_variables > cuopt_max_variables) {
      d.use_highs = true;
      d.size_guard_tripped = true;
      return d;
    }
    d.use_highs = false;
    return d;
  }
}

std::string describe_mip_solver(SolverChoice solver, bool build_has_cuopt,
                                size_t cuopt_max_variables) {
  switch (solver) {
  case SolverChoice::highs:
    return "HiGHS (forced by --solver highs)";
  case SolverChoice::cuopt:
    return build_has_cuopt
               ? "cuOpt (forced by --solver cuopt)"
               : "HiGHS (--solver cuopt requested, but this build has no "
                 "cuOpt; using the compiled primary backend)";
  case SolverChoice::automatic:
  default:
    if (!build_has_cuopt)
      return "HiGHS (auto; this build has no cuOpt)";
    return fmt::format(
        "cuOpt (auto; HiGHS fallback on a solver error, or pre-emptively "
        "for a (sub)problem over {} MIP variables, reusex-81y)",
        cuopt_max_variables);
  }
}

MipSubResult solve_mip_subset_with_fallback(
    SolverChoice solver, bool primary_is_cuopt, size_t subset_cells,
    size_t estimated_variables, size_t cuopt_max_variables,
    const std::function<MipSubResult()> &run_primary,
    const std::function<MipSubResult()> &run_highs, bool &used_highs_fallback,
    bool &size_guard_tripped) {
  used_highs_fallback = false;
  size_guard_tripped = false;

  if (solver == SolverChoice::highs)
    return run_highs();

  if (primary_is_cuopt && estimated_variables > cuopt_max_variables) {
    if (solver == SolverChoice::cuopt) {
      reusex::warn(
          "--solver cuopt forced on a {}-cell (sub)problem estimated at {} "
          "MIP variables (over the {}-variable cuOpt size guard, "
          "reusex-81y: cuOpt SIGSEGVs inside "
          "branch_and_bound_t::set_final_solution on sections around 3.3M "
          "variables) — respecting the explicit choice, but if this crashes, "
          "retry with --solver highs, 'create rooms --min-room-area', or "
          "stronger structural filtering",
          subset_cells, estimated_variables, cuopt_max_variables);
    } else {
      reusex::warn(
          "(Sub)problem has {} cells, estimated {} MIP variables — over the "
          "{}-variable cuOpt size guard (reusex-81y: cuOpt SIGSEGVs inside "
          "branch_and_bound_t::set_final_solution on sections around 3.3M "
          "variables); using HiGHS (CPU) for this (sub)problem instead of "
          "cuOpt. Consider 'create rooms --min-room-area' or stronger "
          "structural filtering to shrink future sections",
          subset_cells, estimated_variables, cuopt_max_variables);
    }
  }

  const BackendDecision decision = select_mip_backend(
      solver, primary_is_cuopt, estimated_variables, cuopt_max_variables);
  size_guard_tripped = decision.size_guard_tripped;

  if (decision.use_highs)
    return run_highs();

  // auto or cuopt, under budget (or this build's primary is already cuOpt
  // and within size): try the compiled-in primary backend first. A
  // stub/broken CUDA driver surfaces a cuOpt failure as std::bad_alloc from
  // RMM's allocator (reusex-8r0), not a clean MIP_solve_status::error, so an
  // exception escaping run_primary() must be caught here or it would escape
  // the whole fallback policy — and, under `auto`, the whole mesh stage.
  MipSubResult res;
  bool threw = false;
  std::string error_message;
  std::exception_ptr caught;
  try {
    res = run_primary();
  } catch (const std::exception &e) {
    threw = true;
    error_message = e.what();
    caught = std::current_exception();
  } catch (...) {
    threw = true;
    error_message = "unknown exception (not derived from std::exception)";
    caught = std::current_exception();
  }

  if (threw) {
    if (solver == SolverChoice::automatic) {
      reusex::warn(
          "Primary MIP backend threw solving a {}-cell (sub)problem ({} MIP "
          "variables): {}; retrying with HiGHS (CPU) as fallback (issue "
          "#226, reusex-8r0)",
          subset_cells, estimated_variables, error_message);
      used_highs_fallback = true;
      return run_highs();
    }
    reusex::error(
        "Primary MIP backend (forced by --solver cuopt) failed solving a "
        "{}-cell (sub)problem ({} MIP variables): {}",
        subset_cells, estimated_variables, error_message);
    std::rethrow_exception(caught);
  }

  const bool hard_error = res.status == SolveStatus::error ||
                          res.status == SolveStatus::numerical_error;
  // Only auto retries on the CPU; an explicit --solver cuopt fails loudly.
  if (hard_error && solver == SolverChoice::automatic) {
    reusex::warn("Primary MIP backend returned '{}'; retrying this problem "
                 "with HiGHS (CPU) as fallback (issue #226)",
                 to_string(res.status));
    used_highs_fallback = true;
    return run_highs();
  }
  return res;
}

namespace {

using Cd = CellComplex::Vertex;
using Fd = CellComplex::Vertex;

/// Outcome of solving one (sub)problem on a single solver instance. Alias for
/// the publicly-declared MipSubResult (Solidifier.hpp) so the
/// cuOpt/HiGHS-specific solve_subset() below can keep using the short name.
using SubResult = MipSubResult;

std::pair<std::set<int>, std::set<int>> getWallIds(const CellComplex &cc,
                                                   Cd cit_a, Cd cit_b) {
  auto W_a = std::get<CellData>(cc[cit_a].data).wall_ids;
  auto W_b = std::get<CellData>(cc[cit_b].data).wall_ids;
  return {W_a, W_b};
}

/// Build and solve the MIP for the subset of cells selected by @p in_subset,
/// restricted to inter-cell faces whose *both* adjacent cells are in the
/// subset. Templated on the solver type so the same formulation runs on cuOpt
/// (primary) or HiGHS (fallback). Returns the room/wall labels for the subset
/// cells and the resulting status (issue #226).
///
/// This is the original monolithic formulation (7 constraints) with two
/// changes: (1) cells and faces are filtered by @p in_subset; (2) the model is
/// self-contained in a local solver so it can be retried on another backend.
template <typename Solver>
SubResult solve_subset(const CellComplex &cc, const SolidifierOptions &options,
                       const std::function<bool(Cd)> &in_subset) {
  using Variable = typename Solver::Variable;
  using Linear_objective = typename Solver::Linear_objective;
  using Linear_constraint = typename Solver::Linear_constraint;

  Solver solver;
  solver.set_time_limit(options.time_limit_seconds);
  solver.set_mip_gap(options.mip_gap);

  const double alpha = options.alpha;

  auto area = cc.property_map<Fd, double>("f:area");
  auto volume = cc.property_map<Cd, double>("c:volume");
  auto f_sp = cc.property_map<Fd, double>("f:support_probability");
  auto c_rp = cc.property_map<Cd, std::vector<double>>("c:room_probabilities");

  std::unordered_map<Cd, std::vector<Variable *>> room_variables;
  std::unordered_map<Cd, std::vector<Variable *>> wall_variables;

  // ── Variables ──────────────────────────────────────────────────────────
  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!in_subset(*cit))
      continue;
    room_variables[*cit] = std::vector<Variable *>(cc.n_rooms + 1);
    wall_variables[*cit] = std::vector<Variable *>(cc.n_walls);
    const int c_id = cc[*cit].id;
    for (size_t l = 0; l < cc.n_rooms + 1; ++l)
      room_variables[*cit][l] = solver.create_variable(
          Variable::BINARY, 0, 1, fmt::format("x_{},r{}", c_id, l));
    for (size_t w = 0; w < cc.n_walls; ++w)
      wall_variables[*cit][w] = solver.create_variable(
          Variable::BINARY, 0, 1, fmt::format("x_{},w{}", c_id, w));
  }

  // A face participates in the subproblem only if both its cells are inside.
  auto face_in_subset = [&](Fd f) {
    return in_subset(cc.get_a(f)) && in_subset(cc.get_b(f));
  };

  // ── Objective ──────────────────────────────────────────────────────────
  Linear_objective *obj = solver.create_objective(Linear_objective::MINIMIZE);
  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!in_subset(*cit))
      continue;
    for (size_t l = 0; l < cc.n_rooms + 1; ++l) {
      const double weight = c_rp[*cit][l] * volume[*cit];
      obj->add_coefficient(room_variables[*cit][l], -weight);
    }
  }
  for (auto fit = cc.faces_between_cells_begin();
       fit != cc.faces_between_cells_end(); ++fit) {
    if (!face_in_subset(*fit))
      continue;
    auto cit_a = cc.get_a(*fit);
    auto cit_b = cc.get_b(*fit);
    auto [Wa, Wb] = getWallIds(cc, cit_a, cit_b);
    const double weight = (1 - f_sp[*fit]) * area[*fit];
    for (auto id : Wb - Wa)
      obj->add_coefficient(wall_variables[cit_b][id], alpha * weight);
    for (auto id : Wa + Wb) {
      obj->add_coefficient(wall_variables[cit_b][id], alpha * weight);
      obj->add_coefficient(wall_variables[cit_a][id], -alpha * weight);
    }
  }

  // ── Constraints ────────────────────────────────────────────────────────
  // c1: each cell gets exactly one room label.
  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!in_subset(*cit))
      continue;
    Linear_constraint *c1 = solver.create_constraint(1, 1, "c1");
    for (size_t l = 0; l < cc.n_rooms + 1; ++l)
      c1->add_coefficient(room_variables[*cit][l], 1);
  }
  // c3: wall labels only on outside cells.
  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!in_subset(*cit))
      continue;
    for (size_t w = 0; w < cc.n_walls; ++w) {
      Linear_constraint *c3 =
          solver.create_constraint(-Linear_constraint::infinity(), 0, "c3");
      c3->add_coefficient(wall_variables[*cit][w], 1);
      c3->add_coefficient(room_variables[*cit][0], -1);
    }
  }
  // Face-based constraints c2, c4, c5, c6, c7 over internal faces only.
  for (auto fit = cc.faces_between_cells_begin();
       fit != cc.faces_between_cells_end(); ++fit) {
    if (!face_in_subset(*fit))
      continue;
    auto cit_a = cc.get_a(*fit);
    auto cit_b = cc.get_b(*fit);
    auto [W_a, W_b] = getWallIds(cc, cit_a, cit_b);

    // c2: room label may only occur on the positive side.
    for (size_t r = 1; r < cc.n_rooms + 1; ++r) {
      Linear_constraint *c2 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c2");
      c2->add_coefficient(room_variables[cit_a][r], 1);
      c2->add_coefficient(room_variables[cit_b][r], -1);
    }

    // c4: room boundary faces must be active-wall boundary faces.
    {
      Linear_constraint *c4 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c4");
      for (auto id : W_b - W_a)
        c4->add_coefficient(wall_variables[cit_b][id], 1);
      c4->add_coefficient(room_variables[cit_b][0], -1);
      c4->add_coefficient(room_variables[cit_a][0], 1);
    }

    // c5: wall label on the negative side at inner faces.
    for (auto id : W_a + W_b) {
      Linear_constraint *c5 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c5");
      c5->add_coefficient(wall_variables[cit_b][id], 1);
      c5->add_coefficient(wall_variables[cit_a][id], -1);
    }

    // c6: a wall may end at an inner face only if another wall is active there.
    for (auto id : W_a + W_b) {
      Linear_constraint *c6 =
          solver.create_constraint(0, Linear_constraint::infinity(), "c6");
      c6->add_coefficient(wall_variables[cit_b][id], -1);
      c6->add_coefficient(wall_variables[cit_a][id], 1);
      for (auto id2 : W_b - W_a)
        c6->add_coefficient(wall_variables[cit_b][id2], 1);
    }

    // c7: outside label only on the negative side.
    {
      Linear_constraint *c7 =
          solver.create_constraint(-Linear_constraint::infinity(), 0, "c7");
      c7->add_coefficient(room_variables[cit_a][0], 1);
      c7->add_coefficient(room_variables[cit_b][0], -1);
    }
  }

  SubResult res;
  res.variables = solver.number_of_variables();
  res.constraints = solver.number_of_constraints();

  const bool ok = solver.solve();
  res.status = map_status(solver.solve_status());
  res.solved = ok;
  if (!ok)
    return res;

  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!in_subset(*cit))
      continue;
    for (size_t i = 0; i < room_variables[*cit].size(); ++i)
      if (room_variables[*cit][i]->solution_value() > 0.5)
        res.room_label[*cit] = static_cast<int>(i);
    res.wall_label[*cit] = std::set<int>{};
    for (size_t i = 0; i < wall_variables[*cit].size(); ++i)
      if (wall_variables[*cit][i]->solution_value() > 0.5)
        res.wall_label[*cit].insert(static_cast<int>(i));
  }
  return res;
}

/// Solve one subproblem honoring the SolverChoice + fallback policy. Builds
/// the two solver-specific callables (primary / HiGHS) and hands the actual
/// decide-retry-or-report policy to the backend-independent, unit-tested
/// solve_mip_subset_with_fallback() (Solidifier.hpp, reusex-8r0) — see its
/// doc comment for the full policy (size guard, hard-error retry under
/// `auto`, exception handling).
/// @p used_highs_fallback reports whether the error-triggered retry fired;
/// @p size_guard_tripped reports whether the pre-emptive size guard fired.
SubResult solve_subset_with_fallback(const CellComplex &cc,
                                     const SolidifierOptions &options,
                                     const std::function<bool(Cd)> &in_subset,
                                     size_t subset_cells,
                                     bool &used_highs_fallback,
                                     bool &size_guard_tripped) {
  constexpr bool primary_is_cuopt = !std::is_same_v<PrimarySolver, HighsSolver>;
  const size_t estimated_variables =
      subset_cells * (cc.n_rooms + 1 + cc.n_walls);

  return solve_mip_subset_with_fallback(
      options.solver, primary_is_cuopt, subset_cells, estimated_variables,
      options.cuopt_max_variables,
      [&] { return solve_subset<PrimarySolver>(cc, options, in_subset); },
      [&] { return solve_subset<HighsSolver>(cc, options, in_subset); },
      used_highs_fallback, size_guard_tripped);
}

} // namespace

// ── PIMPL ────────────────────────────────────────────────────────────────
class Solidifier::Impl {
    public:
  std::shared_ptr<const CellComplex> _cc;
  SolidifierOptions _options;
  SolveStatus _last_status = SolveStatus::not_solved;
  std::vector<SectionSolveStats> _section_stats;

  Impl(std::shared_ptr<const CellComplex> cc, SolidifierOptions options)
      : _cc(cc), _options(options) {
    reusex::trace("Solidifier created (alpha={}, time_limit={}s, sectioned={})",
                  options.alpha, options.time_limit_seconds, options.sectioned);
  }

  void _validatePropertyMaps() const;
};

Solidifier::Solidifier(std::shared_ptr<const CellComplex> cc,
                       SolidifierOptions options)
    : pimpl_(std::make_unique<Impl>(cc, options)) {}

Solidifier::~Solidifier() = default;

SolveStatus Solidifier::last_solve_status() const {
  return pimpl_->_last_status;
}

const std::vector<SectionSolveStats> &Solidifier::section_stats() const {
  return pimpl_->_section_stats;
}

void Solidifier::Impl::_validatePropertyMaps() const {
  std::vector<std::string> missing;
  auto probe_double = [&](const char *name) {
    try {
      (void)_cc->property_map<Fd, double>(name);
    } catch (const std::exception &) {
      missing.emplace_back(name);
    }
  };
  probe_double("f:area");
  probe_double("f:support_probability");
  probe_double("c:volume");
  try {
    (void)_cc->property_map<Cd, std::vector<double>>("c:room_probabilities");
  } catch (const std::exception &) {
    missing.emplace_back("c:room_probabilities");
  }
  if (!missing.empty()) {
    throw std::runtime_error(fmt::format(
        "Solidifier: required cell-complex property map(s) missing before "
        "building the MIP: [{}]. These are populated by "
        "compute_face_coverage() and compute_room_probabilities(); ensure "
        "both run before Solidifier::solve().",
        fmt::join(missing, ", ")));
  }

  auto volume = _cc->property_map<Cd, double>("c:volume");
  auto area = _cc->property_map<Fd, double>("f:area");
  auto f_sp = _cc->property_map<Fd, double>("f:support_probability");
  auto c_rp =
      _cc->property_map<Cd, std::vector<double>>("c:room_probabilities");
  const size_t expected = _cc->n_rooms + 1;
  for (auto cit = _cc->cells_begin(); cit != _cc->cells_end(); ++cit) {
    const auto &probs = c_rp[*cit];
    if (probs.size() != expected)
      throw std::runtime_error(fmt::format(
          "Solidifier: 'c:room_probabilities' for cell {} has size {} but "
          "expected {} (= n_rooms+1). The cell complex and room-probability "
          "computation are out of sync.",
          (*_cc)[*cit].id, probs.size(), expected));
    if (!std::isfinite(volume[*cit]))
      throw std::runtime_error(
          fmt::format("Solidifier: 'c:volume' for cell {} is not finite.",
                      (*_cc)[*cit].id));
  }
  for (auto fit = _cc->faces_between_cells_begin();
       fit != _cc->faces_between_cells_end(); ++fit) {
    if (!std::isfinite(area[*fit]))
      throw std::runtime_error(fmt::format(
          "Solidifier: 'f:area' for face {} is not finite.", (*_cc)[*fit].id));
    if (!std::isfinite(f_sp[*fit]))
      throw std::runtime_error(
          fmt::format("Solidifier: 'f:support_probability' for face {} is not "
                      "finite.",
                      (*_cc)[*fit].id));
  }
}

std::optional<std::pair<std::unordered_map<Solidifier::Cd, int>,
                        std::unordered_map<Solidifier::Cd, std::set<int>>>>
Solidifier::solve() {
  const auto &cc = *pimpl_->_cc;
  const auto &opt = pimpl_->_options;
  pimpl_->_section_stats.clear();

  pimpl_->_validatePropertyMaps();

  const auto n_cells =
      static_cast<size_t>(std::distance(cc.cells_begin(), cc.cells_end()));

  constexpr bool build_has_cuopt =
#ifdef USE_CUOPT
      true;
#else
      false;
#endif
  reusex::info("Using MIP solver: {}",
               describe_mip_solver(opt.solver, build_has_cuopt,
                                   opt.cuopt_max_variables));

  // Decide monolithic vs sectioned (issue #226): only section when enabled,
  // there is more than one storey, and the problem is large enough to warrant
  // it. Below the threshold the original one-shot path is used unchanged.
  const bool do_sectioned =
      opt.sectioned && cc.n_sections > 1 && n_cells >= opt.sectioned_threshold;

  reusex::info("Solidifier: {} cells, {} rooms, {} walls, {} sections => "
               "{} solve (threshold {} cells)",
               n_cells, cc.n_rooms, cc.n_walls, cc.n_sections,
               do_sectioned ? "SECTIONED" : "monolithic",
               opt.sectioned_threshold);

  auto sw = reusex::core::stopwatch{};

  if (!do_sectioned) {
    // Monolithic: single subproblem over all cells.
    bool used_fallback = false;
    bool size_guard_tripped = false;
    auto all = [](Cd) { return true; };
    SubResult res = solve_subset_with_fallback(
        cc, opt, all, n_cells, used_fallback, size_guard_tripped);
    pimpl_->_last_status = res.status;
    if (!res.solved) {
      reusex::error("MIP solver failed to find a solution [status={}, "
                    "{} cells]: increase --time-limit, use --solver highs, or "
                    "reduce complexity via stronger plane merging / --filter",
                    to_string(res.status), n_cells);
      return {};
    }
    reusex::info("Solved monolithic MIP in {:>.3f} seconds (status={}{}{})", sw,
                 to_string(res.status), used_fallback ? ", HiGHS fallback" : "",
                 size_guard_tripped ? ", HiGHS size guard (reusex-81y)" : "");
    return std::make_pair(std::move(res.room_label), std::move(res.wall_label));
  }

  // ── Sectioned solve ──────────────────────────────────────────────────────
  // Partition cells by their "c:section" property. Each section is solved
  // independently; a failing section is reported and skipped, but the others
  // still contribute to the mesh (docs/STANDARDS.md §5). Shared horizontal
  // faces between sections are excluded from every section's constraints and
  // are emitted at most once by toMesh() because they separate interior from
  // exterior cells.
  auto section_of = cc.property_map<Cd, int>("c:section");

  std::unordered_map<Cd, int> room_label;
  std::unordered_map<Cd, std::set<int>> wall_label;

  size_t solved_sections = 0;
  size_t failed_sections = 0;
  size_t interior_cells = 0;

  for (int s = 0; s < static_cast<int>(cc.n_sections); ++s) {
    auto in_section = [&, s](Cd c) { return section_of[c] == s; };

    const size_t section_cells = static_cast<size_t>(
        std::count_if(cc.cells_begin(), cc.cells_end(),
                      [&](Cd c) { return section_of[c] == s; }));
    if (section_cells == 0)
      continue;

    auto sec_sw = reusex::core::stopwatch{};
    bool used_fallback = false;
    bool size_guard_tripped = false;
    SubResult res = solve_subset_with_fallback(
        cc, opt, in_section, section_cells, used_fallback, size_guard_tripped);

    SectionSolveStats st;
    st.section = s;
    st.cells = section_cells;
    st.variables = res.variables;
    st.constraints = res.constraints;
    st.seconds = sec_sw.elapsed();
    st.status = res.status;
    st.used_highs_fallback = used_fallback;
    st.size_guard_tripped = size_guard_tripped;
    pimpl_->_section_stats.push_back(st);

    if (!res.solved) {
      ++failed_sections;
      reusex::error(
          "Section {} FAILED to solve [status={}, {} cells, {} vars, {} "
          "constraints, {:.3f}s]{}{} — skipping this storey; other sections "
          "still meshed",
          s, to_string(res.status), section_cells, res.variables,
          res.constraints, st.seconds,
          used_fallback ? " (after HiGHS fallback)" : "",
          size_guard_tripped ? " (HiGHS size guard, reusex-81y)" : "");
      continue;
    }

    ++solved_sections;
    size_t sec_interior = 0;
    for (auto &[c, r] : res.room_label) {
      room_label[c] = r;
      if (r > 0)
        ++sec_interior;
    }
    for (auto &[c, w] : res.wall_label)
      wall_label[c] = std::move(w);
    interior_cells += sec_interior;

    reusex::info("Section {} solved [status={}, {} cells, {} vars, {} "
                 "constraints, {:.3f}s, {} interior cells]{}{}",
                 s, to_string(res.status), section_cells, res.variables,
                 res.constraints, st.seconds, sec_interior,
                 used_fallback ? " (HiGHS fallback)" : "",
                 size_guard_tripped ? " (HiGHS size guard, reusex-81y)" : "");
  }

  // Any cell without a label (unsolved section) defaults to outside so the
  // mesh filter (room_label > 0) simply drops it.
  for (auto cit = cc.cells_begin(); cit != cc.cells_end(); ++cit) {
    if (!room_label.contains(*cit))
      room_label[*cit] = 0;
    if (!wall_label.contains(*cit))
      wall_label[*cit] = std::set<int>{};
  }

  pimpl_->_last_status =
      failed_sections == 0 ? SolveStatus::optimal : SolveStatus::feasible;

  reusex::info("Sectioned solve complete in {:>.3f}s: {}/{} sections solved "
               "({} failed), {} interior cells total",
               sw, solved_sections, solved_sections + failed_sections,
               failed_sections, interior_cells);

  if (solved_sections == 0) {
    reusex::error("All {} sections failed to solve; no mesh can be produced. "
                  "Try --solver highs, a larger --time-limit, or stronger "
                  "plane merging.",
                  failed_sections);
    pimpl_->_last_status = SolveStatus::error;
    return {};
  }

  return std::make_pair(std::move(room_label), std::move(wall_label));
}

std::shared_ptr<const CellComplex> Solidifier::get_cell_complex() const {
  return pimpl_->_cc;
}

} // namespace reusex::geometry
