// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/cli_command.hpp>

#include <array>
#include <charconv>
#include <cmath>

namespace rux::qt {
namespace {

/// How one key reaches the CLI. For a boolean, `on`/`off` are the flags for
/// true/false; an empty one means "that value is the CLI default: emit
/// nothing".
struct FlagSpec {
  std::string_view key;
  std::string_view flag; ///< value-taking option (non-boolean)
  std::string_view on;
  std::string_view off;
};

struct StageSpec {
  std::string_view stage;
  std::array<std::string_view, 2> subcommand; ///< {"create","planes"}
  std::vector<FlagSpec> flags;
};

/// Mirrors apps/rux/src/create/*.cpp, optimize.cpp and graph/discover.cpp.
/// Keys absent here have no flag (rooms.propagate_k) and are reported as
/// unmapped.
const std::vector<StageSpec> &stage_specs() {
  static const std::vector<StageSpec> specs = {
      {"clouds",
       {"create", "clouds"},
       {{"resolution", "--grid", {}, {}},
        {"min_distance", "--min-distance", {}, {}},
        {"max_distance", "--max-distance", {}, {}},
        {"sampling_factor", "--sampling-factor", {}, {}},
        {"confidence_threshold", "--confidence", {}, {}},
        {"glass_filter", {}, "--glass-filter", {}},
        {"glass_threshold", "--glass-threshold", {}, {}}}},
      {"planes",
       {"create", "planes"},
       {{"angle_threshold", "--angle-threshold", {}, {}},
        {"plane_dist_threshold", "--plane-dist-threshold", {}, {}},
        {"min_inliers", "--min-cluster-size", {}, {}},
        {"radius", "--radius", {}, {}}, // token-lint: allow (a CLI flag)
        {"interval_0", "--interval-0", {}, {}},
        {"interval_factor", "--interval-factor", {}, {}},
        {"adaptive", {}, {}, "--no-adaptive"},
        {"noise_seed", "--noise-seed", {}, {}},
        {"filter", "--filter", {}, {}}}},
      {"rooms",
       {"create", "rooms"},
       {{"grid_size", "--grid-size", {}, {}},
        {"resolution", "--resolution", {}, {}},
        {"beta", "--beta", {}, {}},
        {"max_iter", "--max-iter", {}, {}},
        {"propagate_max_radius", "--propagate-radius", {}, {}},
        {"min_room_area", "--min-room-area", {}, {}},
        {"merge_affinity", "--merge-affinity", {}, {}},
        {"filter", "--filter", {}, {}}}},
      {"instances",
       {"create", "instances"},
       {{"semantic_cloud", "--semantic", {}, {}},
        {"output_cloud", "--output", {}, {}},
        {"cluster_tolerance", "--tolerance", {}, {}},
        {"min_cluster_size", "--min-size", {}, {}},
        {"max_cluster_size", "--max-size", {}, {}},
        {"labels", "--labels", {}, {}}}},
      {"mesh",
       {"create", "mesh"},
       {{"solver", "--solver", {}, {}},
        {"cuopt_max_variables", "--cuopt-max-variables", {}, {}},
        {"time_limit_seconds", "--time-limit", {}, {}},
        {"output_name", "--output-name", {}, {}},
        {"search_threshold", "--threshold", {}, {}},
        {"new_plane_offset", "--offset", {}, {}},
        {"alpha", "--alpha", {}, {}},
        {"max_cells", "--max-cells", {}, {}},
        {"sectioned", {}, {}, "--no-sectioned"},
        {"sectioned_threshold", "--sectioned-threshold", {}, {}},
        {"structural_filter", {}, {}, "--no-structural-filter"},
        {"structural_min_wall_area", "--structural-min-area", {}, {}},
        {"structural_collinear_offset", "--structural-collinear", {}, {}},
        {"structural_storey_clearance", "--structural-clearance", {}, {}},
        {"structural_footprint_overlap", "--structural-overlap", {}, {}},
        {"plane_classes", {}, "--plane-classes", {}},
        {"filter", "--filter", {}, {}}}},
      {"optimize",
       {"optimize", {}},
       {{"min_observations", "--min-observations", {}, {}},
        {"assoc_rounds", "--assoc-rounds", {}, {}},
        {"no_gnc", {}, "--no-gnc", {}},
        {"manhattan", {}, "--manhattan", {}},
        {"manhattan_sigma_deg", "--manhattan-sigma", {}, {}},
        {"manhattan_max_angle_deg", "--manhattan-max-angle", {}, {}},
        {"manhattan_level_max_angle_deg",
         "--manhattan-level-max-angle",
         {},
         {}},
        {"dry_run", {}, "--dry-run", {}}}},
      {"graph_discover",
       {"graph", "discover"},
       {{"loop_closure", {}, {}, "--no-loop-closure"},
        {"use_panoramas", {}, "--use-panoramas", {}},
        {"loop_edges_file", "--loop-edges", {}, {}},
        {"loop_proposal", "--loop-proposal", {}, {}},
        {"loop_min_frame_gap", "--loop-min-frame-gap", {}, {}},
        {"loop_min_inliers", "--loop-min-inliers", {}, {}},
        {"pcm", {}, {}, "--loop-no-pcm"},
        {"dry_run", {}, "--dry-run", {}}}},
      {"storeys",
       {"create", "storeys"},
       {{"horizontal_cos", "--horizontal-cos", {}, {}},
        {"surface_max_tilt_deg", "--surface-max-tilt", {}, {}},
        {"inlier_m", "--inlier", {}, {}},
        {"max_fit_iterations", "--max-fit-iterations", {}, {}},
        {"hist_bin_m", "--hist-bin", {}, {}},
        {"peak_min_share", "--peak-min-share", {}, {}},
        {"slab_merge_m", "--slab-merge", {}, {}},
        {"min_storey_height_m", "--min-storey-height", {}, {}},
        {"default_height_m", "--default-height", {}, {}},
        {"use_normal_sign", {}, {}, "--no-normal-sign"},
        {"per_storey_frame", {}, {}, "--shared-frame"},
        {"storey_frame_tol_deg", "--storey-frame-tol", {}, {}},
        {"guid_match_m", "--guid-match", {}, {}}}},
      {"classify",
       {"create", "classify"},
       {{"horizontal_cos", "--horizontal-cos", {}, {}},
        {"vertical_cos", "--vertical-cos", {}, {}},
        {"min_large_area_m2", "--min-large-area", {}, {}},
        {"band_m", "--band", {}, {}},
        {"min_wall_height_m", "--min-wall-height", {}, {}},
        {"min_wall_height_share", "--min-wall-height-share", {}, {}},
        {"max_furniture_area_m2", "--max-furniture-area", {}, {}},
        {"table_min_m", "--table-min", {}, {}},
        {"table_max_m", "--table-max", {}, {}},
        {"min_storey_height_m", "--min-storey-height", {}, {}},
        {"area_cell_m", "--area-cell", {}, {}},
        {"extent_quantile", "--extent-quantile", {}, {}},
        {"min_labelled_share", "--min-labelled-share", {}, {}},
        {"min_class_share", "--min-class-share", {}, {}},
        {"role_map", "--role-map", {}, {}}}},
      {"walls",
       {"create", "walls"},
       {{"max_families", "--max-families", {}, {}},
        {"family_angle_deg", "--family-angle", {}, {}},
        {"min_family_share", "--min-family-share", {}, {}},
        {"wall_role_only", {}, {}, "--all-vertical-planes"},
        {"family_refine_iterations", "--family-refine-iterations", {}, {}},
        {"family_refine_deg", "--family-refine", {}, {}},
        {"assign_angle_deg", "--assign-angle", {}, {}},
        {"square_deg", "--square", {}, {}},
        {"bin_m", "--bin", {}, {}},
        {"min_peak_points", "--min-peak-points", {}, {}},
        {"min_peak_prominence_share", "--min-peak-prominence", {}, {}},
        {"min_peak_separation_m", "--min-peak-separation", {}, {}},
        {"face_window_m", "--face-window", {}, {}},
        {"min_column_share", "--min-column-share", {}, {}},
        {"run_gap_m", "--run-gap", {}, {}},
        {"min_run_m", "--min-run", {}, {}},
        {"min_run_points", "--min-run-points", {}, {}},
        {"cell_m", "--cell", {}, {}},
        {"storey_margin_m", "--storey-margin", {}, {}},
        {"min_gap_m", "--min-gap", {}, {}},
        {"max_gap_m", "--max-gap", {}, {}},
        {"min_shared_m", "--min-shared", {}, {}},
        {"max_valley_share", "--max-valley-share", {}, {}},
        {"min_co_visibility", "--min-co-visibility", {}, {}},
        {"max_crossed_gap_m", "--max-crossed-gap", {}, {}},
        {"min_cell_points", "--min-cell-points", {}, {}},
        {"outlier_sigma_k", "--outlier-sigma", {}, {}},
        {"min_robust_sigma_m", "--min-robust-sigma", {}, {}},
        {"bootstrap_samples", "--bootstrap-samples", {}, {}},
        {"seed", "--seed", {}, {}},
        {"ci_level", "--ci-level", {}, {}},
        {"default_thickness_m", "--default-thickness", {}, {}},
        {"external_strip_near_m", "--external-strip-near", {}, {}},
        {"external_strip_far_m", "--external-strip-far", {}, {}},
        {"external_max_z_m", "--external-max-z", {}, {}},
        {"min_room_density", "--min-room-density", {}, {}},
        {"snap_wide_m", "--snap-wide", {}, {}},
        {"snap_narrow_m", "--snap-narrow", {}, {}},
        {"bridge_gap_m", "--bridge-gap", {}, {}},
        {"bridge_offset_m", "--bridge-offset", {}, {}},
        {"bridge_thickness_m", "--bridge-thickness", {}, {}},
        {"recess_max_length_m", "--recess-max-length", {}, {}},
        {"corner_m", "--corner", {}, {}},
        {"tee_m", "--tee", {}, {}},
        {"tee_on_wall_m", "--tee-on-wall", {}, {}},
        {"min_seen_share", "--min-seen-share", {}, {}},
        {"guid_match_m", "--guid-match", {}, {}},
        {"guid_match_angle_deg", "--guid-match-angle", {}, {}},
        {"guid_min_overlap", "--guid-min-overlap", {}, {}}}},
  };
  return specs;
}

const StageSpec *find_stage(std::string_view stage) {
  for (const auto &s : stage_specs())
    if (s.stage == stage)
      return &s;
  return nullptr;
}

const FlagSpec *find_flag(const StageSpec &s, std::string_view key) {
  for (const auto &f : s.flags)
    if (f.key == key)
      return &f;
  return nullptr;
}

bool safe_char(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') ||
         std::string_view("_-./:=,+@%").find(c) != std::string_view::npos;
}

} // namespace

std::string format_number(double value) {
  if (!std::isfinite(value))
    return "0";
  std::array<char, 64> buf{};
  const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  return std::string(buf.data(), r.ptr);
}

std::string format_number(float value) {
  if (!std::isfinite(value))
    return "0";
  std::array<char, 64> buf{};
  const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  return std::string(buf.data(), r.ptr);
}

std::string shell_quote(std::string_view word) {
  if (word.empty())
    return "''";
  bool safe = true;
  for (char c : word)
    safe = safe && safe_char(c);
  if (safe)
    return std::string(word);
  std::string out = "'";
  for (char c : word) {
    if (c == '\'')
      out += "'\\''";
    else
      out += c;
  }
  out += '\'';
  return out;
}

std::vector<std::string> split_shell_words(std::string_view line) {
  std::vector<std::string> out;
  std::string cur;
  bool in_word = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'') {
      in_word = true;
      for (++i; i < line.size() && line[i] != '\''; ++i)
        cur += line[i];
    } else if (c == '"') {
      in_word = true;
      for (++i; i < line.size() && line[i] != '"'; ++i) {
        if (line[i] == '\\' && i + 1 < line.size() &&
            std::string_view("\"\\$`").find(line[i + 1]) !=
                std::string_view::npos)
          ++i;
        cur += line[i];
      }
    } else if (c == '\\' && i + 1 < line.size()) {
      in_word = true;
      cur += line[++i];
    } else if (c == ' ' || c == '\t' || c == '\n') {
      if (in_word)
        out.push_back(std::move(cur));
      cur.clear();
      in_word = false;
    } else {
      in_word = true;
      cur += c;
    }
  }
  if (in_word)
    out.push_back(std::move(cur));
  return out;
}

std::string wrap_shell_command(std::string_view line, std::size_t columns) {
  // Words as written (quotes kept), split at unquoted blanks.
  std::vector<std::string> words;
  std::string cur;
  char quote = 0;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quote) {
      cur += c;
      if (c == quote)
        quote = 0;
    } else if (c == '\'' || c == '"') {
      quote = c;
      cur += c;
    } else if (c == '\\' && i + 1 < line.size()) {
      cur += c;
      cur += line[++i];
    } else if (c == ' ' || c == '\t' || c == '\n') {
      if (!cur.empty())
        words.push_back(std::move(cur));
      cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty())
    words.push_back(std::move(cur));

  // A flag and the value after it travel together.
  std::vector<std::string> groups;
  for (std::size_t i = 0; i < words.size(); ++i) {
    std::string g = words[i];
    if (g.size() > 1 && g[0] == '-' && i + 1 < words.size() &&
        words[i + 1][0] != '-')
      g += " " + words[++i];
    groups.push_back(std::move(g));
  }

  std::string out;
  std::size_t col = 0;
  const std::size_t cont = 2; // " \\" at a line end
  // A group never breaks mid-word: a bare backslash-newline inside a path
  // reads back as a different path (`office_corridor.rux` -> a literal
  // "offi\ce_corridor.rux" if cut between 'i' and 'c'), so a group that does
  // not fit even on a line of its own is placed on its own line and allowed
  // to overflow `columns` rather than being cut.
  for (std::size_t i = 0; i < groups.size(); ++i) {
    const std::string &g = groups[i];
    if (col != 0 && col + 1 + g.size() + cont > columns) {
      out += " \\\n  ";
      col = 2;
    } else if (col != 0) {
      out += ' ';
      col += 1;
    }
    out += g;
    col += g.size();
  }
  return out;
}

std::string cli_flag_for(std::string_view stage, std::string_view key) {
  const StageSpec *s = find_stage(stage);
  if (!s)
    return {};
  const FlagSpec *f = find_flag(*s, key);
  if (!f)
    return {};
  if (!f->flag.empty())
    return std::string(f->flag);
  return std::string(!f->off.empty() ? f->off : f->on);
}

CliCommand build_cli_command(std::string_view stage, std::string_view project,
                             const std::vector<CliParam> &params) {
  CliCommand cmd;
  const StageSpec *s = find_stage(stage);
  if (!s)
    return cmd;
  cmd.supported = true;
  if (!project.empty()) {
    cmd.args.emplace_back("-p");
    cmd.args.emplace_back(project);
  }
  for (const auto &word : s->subcommand)
    if (!word.empty())
      cmd.args.emplace_back(word);

  for (const CliParam &p : params) {
    if (p.key == "job_id")
      continue; // the runner's bookkeeping, not a parameter
    const FlagSpec *f = find_flag(*s, p.key);
    if (!f) {
      cmd.unmapped.push_back(p.key);
      continue;
    }
    if (p.kind == CliValueKind::boolean || f->flag.empty()) {
      const bool on = p.value == "true" || p.value == "1";
      const std::string_view flag = on ? f->on : f->off;
      if (!flag.empty())
        cmd.args.emplace_back(flag);
      continue;
    }
    // An empty filter / list is "not set" on the CLI too.
    if (p.value.empty() &&
        (p.kind == CliValueKind::integer_list || p.key == "filter"))
      continue;
    cmd.args.emplace_back(f->flag);
    cmd.args.push_back(p.value);
  }

  cmd.text = "rux";
  for (const auto &a : cmd.args) {
    cmd.text += ' ';
    cmd.text += shell_quote(a);
  }
  return cmd;
}

} // namespace rux::qt
