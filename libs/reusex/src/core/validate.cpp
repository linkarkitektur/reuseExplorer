// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/validate.hpp"
#include "core/ProjectDB.hpp"
#include "core/label_encoding.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <opencv2/core/mat.hpp>

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace reusex::core {

bool ValidationReport::ok() const {
  return std::none_of(issues.begin(), issues.end(),
                      [](const ValidationIssue &i) {
                        return i.severity == ValidationSeverity::error;
                      });
}

size_t ValidationReport::error_count() const {
  return static_cast<size_t>(
      std::count_if(issues.begin(), issues.end(), [](const ValidationIssue &i) {
        return i.severity == ValidationSeverity::error;
      }));
}

size_t ValidationReport::warning_count() const {
  return static_cast<size_t>(
      std::count_if(issues.begin(), issues.end(), [](const ValidationIssue &i) {
        return i.severity == ValidationSeverity::warning;
      }));
}

namespace {

// Names of instance-label clouds — those that may have instance_materials
// links. We consider any cloud that has instances rows or material links.
std::vector<std::string> instance_cloud_names(const ProjectDB &db) {
  std::vector<std::string> result;
  for (const auto &name : db.list_point_clouds()) {
    bool has = false;
    try {
      has = !db.instances(name).empty() || !db.instance_materials(name).empty();
    } catch (const std::exception &) {
      has = false;
    }
    if (has)
      result.push_back(name);
  }
  return result;
}

} // namespace

void check_orphaned_passports(const ProjectDB &db,
                              std::vector<ValidationIssue> &out) {
  // Gather every material_guid referenced by any link.
  std::set<std::string> referenced;
  for (const auto &name : instance_cloud_names(db)) {
    for (const auto &[id, guid] : db.instance_materials(name))
      referenced.insert(guid);
  }

  for (const auto &guid : db.list_passport_guids()) {
    if (referenced.find(guid) == referenced.end()) {
      out.push_back({"orphaned_passport",
                     fmt::format("material passport {} is not linked to any "
                                 "instance",
                                 guid),
                     ValidationSeverity::warning,
                     /*hint=*/{}, /*artifact=*/{}, /*commands=*/{}});
    }
  }
}

void check_dangling_instance_materials(const ProjectDB &db,
                                       std::vector<ValidationIssue> &out) {
  std::set<std::string> passports;
  for (const auto &g : db.list_passport_guids())
    passports.insert(g);

  for (const auto &name : instance_cloud_names(db)) {
    // Set of valid instance ids for this cloud.
    std::set<uint32_t> instance_ids;
    for (const auto &r : db.instances(name))
      instance_ids.insert(r.instance_id);

    for (const auto &[iid, guid] : db.instance_materials(name)) {
      if (instance_ids.find(static_cast<uint32_t>(iid)) == instance_ids.end())
        out.push_back(
            {"dangling_instance_material",
             fmt::format("cloud '{}': link for instance {} has no instances "
                         "row (guid {})",
                         name, iid, guid),
             ValidationSeverity::error,
             /*hint=*/{}, /*artifact=*/{}, /*commands=*/{}});
      if (passports.find(guid) == passports.end())
        out.push_back(
            {"dangling_instance_material",
             fmt::format("cloud '{}': instance {} links to missing passport {}",
                         name, iid, guid),
             ValidationSeverity::error,
             /*hint=*/{}, /*artifact=*/{}, /*commands=*/{}});
    }
  }
}

void check_instances_without_label_defs(const ProjectDB &db,
                                        std::vector<ValidationIssue> &out) {
  for (const auto &name : instance_cloud_names(db)) {
    auto rows = db.instances(name);
    if (rows.empty())
      continue;
    std::map<int, std::string> defs;
    try {
      defs = db.label_definitions(name);
    } catch (const std::exception &) {
      // Treated as "no definitions": every instance is missing one.
    }
    for (const auto &r : rows) {
      if (defs.find(static_cast<int>(r.instance_id)) == defs.end())
        out.push_back({"instance_without_label_def",
                       fmt::format("cloud '{}': instance {} (guid {}) has no "
                                   "label_definitions entry",
                                   name, r.instance_id, r.guid),
                       ValidationSeverity::error,
                       /*hint=*/{}, /*artifact=*/{}, /*commands=*/{}});
    }
  }
}

void check_sibling_cloud_sizes(const ProjectDB &db,
                               std::vector<ValidationIssue> &out) {
  // The canonical parallel clouds for a scan; they are index-aligned.
  static const std::vector<std::string> siblings = {
      "cloud", "labels", "normals", "planes", "rooms", "instances"};

  auto summary = db.project_summary();
  std::map<std::string, size_t> counts;
  for (const auto &c : summary.clouds)
    counts[c.name] = c.point_count;

  // Reference size = "cloud" if present, else the first sibling present.
  std::string ref_name;
  size_t ref_size = 0;
  for (const auto &s : siblings) {
    auto it = counts.find(s);
    if (it != counts.end()) {
      ref_name = s;
      ref_size = it->second;
      break;
    }
  }
  if (ref_name.empty())
    return; // No sibling clouds present.

  for (const auto &s : siblings) {
    if (s == ref_name)
      continue;
    auto it = counts.find(s);
    if (it == counts.end())
      continue; // Missing sibling is not an error here.
    if (it->second != ref_size)
      out.push_back(
          {"sibling_size_mismatch",
           fmt::format("cloud '{}' has {} points but sibling '{}' has {} — "
                       "parallel clouds must be index-aligned",
                       s, it->second, ref_name, ref_size),
           ValidationSeverity::error,
           /*hint=*/{}, /*artifact=*/{}, /*commands=*/{}});
  }
}

void check_unstamped_semantic_clouds(const ProjectDB &db,
                                     std::vector<ValidationIssue> &out) {
  const auto instance_clouds = instance_cloud_names(db);
  for (const auto &name : db.list_point_clouds()) {
    if (std::find(instance_clouds.begin(), instance_clouds.end(), name) !=
        instance_clouds.end())
      continue;
    if (!is_unstamped_semantic_cloud(db, name))
      continue;
    const auto stage = db.point_cloud_provenance(name).first;
    if (!is_semantic_label_cloud(name, stage))
      continue;
    out.push_back(
        {"semantic_cloud_unstamped",
         fmt::format("semantic label cloud '{}' (stage '{}') has no "
                     "\"label_encoding\": \"api_plus_one\" at schema {} — "
                     "probably written by a pre-v29 binary, so its class ids "
                     "may be one too low",
                     name, stage, db.schema_version()),
         ValidationSeverity::warning,
         /*hint=*/"re-run `rux create clouds` or `rux create project`",
         /*artifact=*/name,
         /*commands=*/{"rux create clouds", "rux create project"}});
  }
}

ValidationReport validate_project(const ProjectDB &db) {
  ValidationReport report;
  check_orphaned_passports(db, report.issues);
  check_dangling_instance_materials(db, report.issues);
  check_instances_without_label_defs(db, report.issues);
  check_sibling_cloud_sizes(db, report.issues);
  check_unstamped_semantic_clouds(db, report.issues);
  return report;
}

// ── Stage input contracts (#222, consolidated in #246) ─────────────────────
//
// This section used to be a hand-written `switch` with one case per stage,
// duplicated again in apps/rux/src/validation.cpp. It is now a generic
// interpreter of the table in core/stage_contract.hpp: adding a stage or
// changing a prerequisite is a table edit, and every consumer — `rux validate
// --stage`, every `rux create` subcommand, and pipeline::run_stage()'s refusal
// path — moves with it.
//
// parse_pipeline_stage() / to_string() / pipeline_stage_names() live in
// core/stage_contract.cpp, next to the table they read.

namespace {

// What the project currently contains, gathered in one project_summary() call.
struct ProjectState {
  std::map<std::string, size_t, std::less<>> cloud_points;
  std::set<std::string, std::less<>> meshes;
  int sensor_frames = 0;
  int segmentation_images = 0;
  int building_components = 0;
  int material_passports = 0;
  int storeys = 0;
};

ProjectState read_state(const ProjectDB &db) {
  ProjectState state;
  auto summary = db.project_summary();
  for (const auto &cloud : summary.clouds)
    state.cloud_points[cloud.name] = cloud.point_count;
  for (const auto &mesh : summary.meshes)
    state.meshes.insert(mesh.name);
  state.sensor_frames = summary.sensor_frames.total_count;
  state.segmentation_images = summary.sensor_frames.segmented_count;
  state.building_components = summary.components.total_count;
  state.material_passports = static_cast<int>(summary.materials.size());
  state.storeys = static_cast<int>(db.storeys().size());
  return state;
}

// Row count of a table artifact. This is the one place that has to know how a
// given table is counted in ProjectDB — it is an accessor mapping, not a
// second copy of the contract; which tables a stage needs still comes from the
// table.
int table_rows(const ProjectState &state, std::string_view name) {
  if (name == "sensor_frames")
    return state.sensor_frames;
  if (name == "segmentation_images")
    return state.segmentation_images;
  if (name == "building_components")
    return state.building_components;
  // `instance_materials` (the instance→material links) is produced by
  // `rux create materials` alongside the material passports; a project has the
  // links exactly when it has passports, so the passport count is the presence
  // signal. `material_annotations` is only ever an OUTPUT of `attributes`, so
  // it is never checked as an input and needs no counter.
  if (name == "instance_materials")
    return state.material_passports;
  if (name == "storeys")
    return state.storeys;
  return 0;
}

// Number of stored frames that carry a depth image, counted up to `wanted` so
// a satisfied project never pays for decoding more than it must.
int count_depth_frames(const ProjectDB &db, int wanted) {
  int found = 0;
  for (int id : db.sensor_frame_ids()) {
    if (!db.sensor_frame_depth(id).empty() && ++found >= wanted)
      break;
  }
  return found;
}

/// One input after `overrides` has been applied: what the contract declared
/// (which fixes the artifact's kind and alignment class) and what to actually
/// look for in the project.
struct ResolvedName {
  std::string_view declared;
  std::string effective;
};

std::vector<ResolvedName> resolve(const StageInput &input,
                                  const ArtifactOverrides &overrides) {
  std::vector<ResolvedName> resolved;
  std::vector<ResolvedName> substituted;
  for (const auto &declared : input.any_of) {
    auto it = overrides.find(declared);
    if (it != overrides.end())
      substituted.push_back({declared, it->second});
    resolved.push_back({declared, std::string(declared)});
  }
  // An explicit override is an instruction, not a preference: `rux create
  // instances --semantic-cloud foo` must fail when `foo` is missing, even if
  // the contract's fallback (`planes`) happens to be present.
  return substituted.empty() ? resolved : substituted;
}

bool present(const ProjectDB &db, const ProjectState &state,
             const Artifact &artifact, const std::string &name, int min_rows,
             int min_depth_frames) {
  switch (artifact.kind) {
  case ArtifactKind::point_cloud:
    return state.cloud_points.find(name) != state.cloud_points.end();
  case ArtifactKind::mesh:
    return state.meshes.find(name) != state.meshes.end();
  case ArtifactKind::gaussian_splat:
    // Queried straight from the database rather than pre-collected into
    // ProjectState: a project holds a handful of splats and nothing consumes
    // one today, so caching the list would cost every validation run a query
    // to answer a question nobody asks.
    return db.has_gaussian_splat(name);
  case ArtifactKind::table:
    if (table_rows(state, name) < std::max(min_rows, 1))
      return false;
    return min_depth_frames <= 0 ||
           count_depth_frames(db, min_depth_frames) >= min_depth_frames;
  }
  return false;
}

/// Collect the indices (into stage_contracts()) of every stage that has to run
/// before @p artifact can exist, transitively.
///
/// Terminates because a stage's inputs are only ever produced by strictly
/// earlier stages — an invariant of the table asserted by
/// tests/unit/core/test_stage_contract.cpp.
void collect_producers(const ProjectDB &db, const ProjectState &state,
                       std::string_view artifact, std::set<size_t> &stages) {
  auto producer = producing_stage(artifact);
  if (!producer)
    return;

  const auto &contracts = stage_contracts();
  for (size_t i = 0; i < contracts.size(); ++i) {
    if (contracts[i].stage != *producer)
      continue;
    if (!stages.insert(i).second)
      return; // already collected, and so are its prerequisites
    for (const auto &input : contracts[i].inputs) {
      const auto names = resolve(input, {});
      const bool satisfied =
          std::any_of(names.begin(), names.end(), [&](const ResolvedName &n) {
            const Artifact *art = find_artifact(n.declared);
            return art && present(db, state, *art, n.effective, input.min_rows,
                                  input.min_depth_frames);
          });
      if (!satisfied && !names.empty())
        collect_producers(db, state, names.front().declared, stages);
    }
    return;
  }
}

/// The progressive "run these commands in order" guidance the CLI used to
/// hard-code per stage, derived from the table instead (#246).
std::vector<std::string> resolution_commands(const ProjectDB &db,
                                             const ProjectState &state,
                                             std::string_view artifact) {
  std::set<size_t> stages;
  collect_producers(db, state, artifact, stages);

  const auto &contracts = stage_contracts();
  std::vector<std::string> commands;
  for (size_t index : stages) // std::set iterates in pipeline order
    commands.emplace_back(contracts[index].command);
  return commands;
}

std::string resolution_hint(const std::vector<std::string> &commands,
                            std::string_view artifact) {
  if (commands.empty())
    return {};
  if (commands.size() == 1)
    return fmt::format("Run '{}' to produce '{}'", commands.front(), artifact);
  return fmt::format("Run the following commands in order:\n    {}",
                     fmt::join(commands, "\n    "));
}

/// One issue about a named artifact, carrying both forms of the resolution.
ValidationIssue artifact_issue(const ProjectDB &db, const ProjectState &state,
                               std::string check, std::string message,
                               std::string_view artifact) {
  auto commands = resolution_commands(db, state, artifact);
  return ValidationIssue{
      std::move(check),          std::move(message),
      ValidationSeverity::error, resolution_hint(commands, artifact),
      std::string(artifact),     std::move(commands)};
}

/// Message for an unsatisfied input, preserving the wording the pre-#246
/// checks emitted so `rux validate --stage <name> --json` keeps its output.
std::string missing_message(std::string_view stage, const Artifact &artifact,
                            const std::vector<ResolvedName> &names,
                            const StageInput &input, const ProjectState &state,
                            const ProjectDB &db) {
  if (artifact.kind == ArtifactKind::table) {
    if (input.min_depth_frames > 0 &&
        table_rows(state, names.front().effective) >=
            std::max(input.min_rows, 1))
      return fmt::format(
          "stage '{}' requires at least {} sensor frame(s) with depth data, "
          "found {}",
          stage, input.min_depth_frames,
          count_depth_frames(db, input.min_depth_frames));
    if (names.front().effective == "sensor_frames") {
      if (input.min_rows > 1)
        return fmt::format(
            "stage '{}' requires at least {} stored sensor frames, found {}",
            stage, input.min_rows, state.sensor_frames);
      return fmt::format("stage '{}' requires stored sensor frames (run 'rux "
                         "import' first)",
                         stage);
    }
    return fmt::format(
        "stage '{}' requires '{}' ({}), which is empty or not present", stage,
        names.front().effective, artifact.description);
  }

  const char *kind = artifact.kind == ArtifactKind::mesh ? "mesh"
                     : artifact.kind == ArtifactKind::gaussian_splat
                         ? "Gaussian splat"
                         : "cloud";
  if (names.size() == 1)
    return fmt::format("stage '{}' requires {} '{}' which is not present in "
                       "the project",
                       stage, kind, names.front().effective);

  std::vector<std::string> options;
  for (const auto &name : names)
    options.push_back(fmt::format("'{}'", name.effective));
  return fmt::format("stage '{}' requires one of the {}s {}, none of which is "
                     "present in the project",
                     stage, kind, fmt::join(options, " or "));
}

} // namespace

void check_stage_inputs(const ProjectDB &db, PipelineStage stage,
                        std::vector<ValidationIssue> &out,
                        const ArtifactOverrides &overrides) {
  const auto &contract = stage_contract(stage);
  const std::string_view name = contract.name;
  const ProjectState state = read_state(db);

  // Inputs that are present, grouped by the index-alignment class of the
  // artifact the contract declared. Alignment is a property of the artifact,
  // so a stage cannot accidentally require the per-plane clouds to match the
  // per-point ones (STANDARDS §3.2).
  std::vector<std::pair<Alignment, std::vector<std::pair<std::string, size_t>>>>
      groups;
  auto group_for = [&groups](Alignment alignment)
      -> std::vector<std::pair<std::string, size_t>> & {
    for (auto &entry : groups)
      if (entry.first == alignment)
        return entry.second;
    groups.push_back({alignment, {}});
    return groups.back().second;
  };

  for (const auto &input : contract.inputs) {
    const auto names = resolve(input, overrides);
    if (names.empty())
      continue;

    const Artifact *artifact = find_artifact(names.front().declared);
    if (!artifact) {
      // A contract naming an unregistered artifact is a programming error the
      // table-integrity test catches at build time; refuse loudly rather than
      // silently skipping a prerequisite.
      out.push_back({"missing_stage_input",
                     fmt::format("stage '{}' declares unknown artifact '{}'",
                                 name, names.front().declared),
                     ValidationSeverity::error,
                     /*hint=*/{}, std::string(names.front().declared),
                     /*commands=*/{}});
      continue;
    }

    bool satisfied = false;
    for (const auto &candidate : names) {
      if (!present(db, state, *artifact, candidate.effective, input.min_rows,
                   input.min_depth_frames))
        continue;
      satisfied = true;
      if (artifact->alignment != Alignment::none) {
        auto it = state.cloud_points.find(candidate.effective);
        if (it != state.cloud_points.end())
          group_for(artifact->alignment)
              .push_back({candidate.effective, it->second});
      }
      break;
    }

    if (!satisfied)
      out.push_back(artifact_issue(
          db, state, "missing_stage_input",
          missing_message(name, *artifact, names, input, state, db),
          names.front().declared));
  }

  for (const auto &[alignment, members] : groups) {
    (void)alignment;
    if (members.size() < 2)
      continue;
    const auto &[ref_name, ref_size] = members.front();
    for (size_t i = 1; i < members.size(); ++i) {
      const auto &[member_name, member_size] = members[i];
      if (member_size == ref_size)
        continue;
      out.push_back(artifact_issue(
          db, state, "stage_input_size_mismatch",
          fmt::format("stage '{}': cloud '{}' has {} points but '{}' has {} — "
                      "index-aligned inputs must match",
                      name, member_name, member_size, ref_name, ref_size),
          member_name));
    }
  }
}

ValidationReport validate_stage(const ProjectDB &db, PipelineStage stage,
                                const ArtifactOverrides &overrides) {
  ValidationReport report;
  check_stage_inputs(db, stage, report.issues, overrides);
  return report;
}

} // namespace reusex::core
