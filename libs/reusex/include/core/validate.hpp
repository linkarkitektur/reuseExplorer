// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "reusex/core/stage_contract.hpp"

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// Severity of a single validation finding.
enum class ValidationSeverity { warning, error };

/// One problem discovered by a validation check.
struct ValidationIssue {
  std::string check; ///< Machine-readable check id (e.g. "orphaned_passport").
  std::string message; ///< Human-readable description.
  ValidationSeverity severity = ValidationSeverity::error;
  /// What the user should run to fix it, or empty when there is no mechanical
  /// resolution. For stage-input issues this is DERIVED from the stage
  /// contract table (#246) — the producing stage of every missing artifact,
  /// walked backwards to the first prerequisite that is actually satisfied —
  /// so the "run these commands in order" guidance cannot drift out of step
  /// with the checks themselves.
  std::string hint;
  /// The pipeline artifact this issue is about, when it is about one; empty
  /// otherwise. Surfaced separately so a front end can say "inputs missing:
  /// cloud, normals" without regex-ing `message`, which is prose and is
  /// allowed to change wording (#305).
  std::string artifact;
  /// The same resolution as `hint`, as an ordered list of commands rather than
  /// prose. `hint` is formatted for a terminal (it is multi-line when several
  /// commands are needed); a GUI needs the steps, not the layout, and parsing
  /// them back out of the prose would be a second source of truth (#305).
  std::vector<std::string> commands;
};

/// Aggregate result of running all validation checks over a project.
struct ValidationReport {
  std::vector<ValidationIssue> issues;

  /// True when no error-severity issue was found.
  bool ok() const;
  size_t error_count() const;
  size_t warning_count() const;
};

// ── Individual checks (each appends to `out`) ──────────────────────────────
// These are read-only. Split out so they can be unit-tested in isolation.

/// Material passports not referenced by any instance_materials link.
/// Reported as warnings (an unused passport is not necessarily a bug).
void check_orphaned_passports(const ProjectDB &db,
                              std::vector<ValidationIssue> &out);

/// instance_materials rows whose instance row or passport is missing.
void check_dangling_instance_materials(const ProjectDB &db,
                                       std::vector<ValidationIssue> &out);

/// instances rows with no corresponding label_definitions entry.
void check_instances_without_label_defs(const ProjectDB &db,
                                        std::vector<ValidationIssue> &out);

/// Parallel/sibling clouds (cloud, labels, normals, planes, rooms, instances)
/// whose point counts disagree — they are supposed to be index-aligned.
void check_sibling_cloud_sizes(const ProjectDB &db,
                               std::vector<ValidationIssue> &out);

/// Semantic label clouds (core::is_semantic_label_cloud(), not instance
/// clouds) without `parameters.label_encoding = "api_plus_one"` in a project
/// at schema >= 29 — written by a pre-v29 binary, so their class ids are
/// probably one too low (STANDARDS §3.1). Reported as warnings.
void check_unstamped_semantic_clouds(const ProjectDB &db,
                                     std::vector<ValidationIssue> &out);

/// Run every check and aggregate the findings.
ValidationReport validate_project(const ProjectDB &db);

// ── Stage input contracts (#222, consolidated in #246) ─────────────────────
// Each pipeline stage consumes a known set of named clouds/tables and produces
// another. That table lives in ONE place — core/stage_contract.hpp, mirrored in
// prose by docs/CONTRACTS.md — and the functions below are generic interpreters
// of it, not a second copy. `rux validate --stage <name>` and the refusal path
// of `pipeline::run_stage()` are the same code, so a missing/misaligned
// prerequisite fails identically whichever door the user came through.
//
// `PipelineStage`, `parse_pipeline_stage()`, `to_string()` and
// `pipeline_stage_names()` come from <reusex/core/stage_contract.hpp>, included
// above.

/// Runtime substitutions for artifact names, for the flags that let a stage be
/// pointed at a differently-named cloud (`rux create instances
/// --semantic-cloud foo`). Maps the contract's declared name to the name to
/// actually look for; when any member of an `any_of` input is overridden, the
/// input collapses to the override so the check is exact rather than lenient.
using ArtifactOverrides = std::map<std::string, std::string, std::less<>>;

/// Assert that @p stage's input clouds/tables exist in @p db and are
/// index-aligned, per the contract in core/stage_contract.hpp. Appends an issue
/// per missing input or size mismatch, each carrying a derived resolution hint.
void check_stage_inputs(const ProjectDB &db, PipelineStage stage,
                        std::vector<ValidationIssue> &out,
                        const ArtifactOverrides &overrides = {});

/// Run check_stage_inputs for a single stage and aggregate the findings.
ValidationReport validate_stage(const ProjectDB &db, PipelineStage stage,
                                const ArtifactOverrides &overrides = {});

} // namespace reusex::core
