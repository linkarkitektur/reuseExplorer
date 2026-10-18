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
        {"filter", "--filter", {}, {}},
        {"method", "--method", {}, {}},
        {"band_bottom", "--band-bottom", {}, {}},
        {"band_top", "--band-top", {}, {}},
        {"band_cell_size", "--band-cell", {}, {}},
        {"band_core_radius", "--core-radius", {}, {}},
        {"band_min_room_area", "--band-min-room-area", {}, {}},
        {"band_wall_min_points", "--band-wall-points", {}, {}},
        {"band_footprint_close", "--footprint-close", {}, {}},
        {"band_use_walls", {}, {}, "--no-wall-graph"},
        {"band_cut_at_doors", {}, {}, "--no-door-cuts"},
        {"band_wall_min_seen_share", "--wall-min-seen", {}, {}}}},
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
        {"plane_classes", {}, {}, "--no-plane-classes"},
        {"storey_sections", {}, {}, "--no-storey-sections"},
        {"storey_local", {}, {}, "--no-storey-local"},
        {"space_prior", {}, {}, "--no-space-prior"},
        // A CLI flag, not a --space- design token (token-lint).
        // clang-format off
        {"space_prior_weight", "--space-prior-weight", {}, {}}, // token-lint: allow
        // clang-format on
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
        {"manhattan_frame_sigma_deg", "--manhattan-frame-sigma", {}, {}},
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
        {"min_floating_wall_run_m", "--min-floating-wall-run", {}, {}},
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
        {"door_max_width_m", "--door-max-width", {}, {}},
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
        {"door_bridge_gap_m", "--door-bridge-gap", {}, {}},
        {"lintel_min_z_m", "--lintel-min-z", {}, {}},
        {"lintel_min_share", "--lintel-min-share", {}, {}},
        {"crossing_max_step_m", "--crossing-max-step", {}, {}},
        {"bridge_thickness_m", "--bridge-thickness", {}, {}},
        {"recess_max_length_m", "--recess-max-length", {}, {}},
        {"duplicate_min_overlap", "--duplicate-min-overlap", {}, {}},
        {"duplicate_min_overlap_m", "--duplicate-min-overlap-length", {}, {}},
        {"duplicate_max_offset_m", "--duplicate-max-offset", {}, {}},
        {"corner_m", "--corner", {}, {}},
        {"tee_m", "--tee", {}, {}},
        {"tee_on_wall_m", "--tee-on-wall", {}, {}},
        {"min_seen_share", "--min-seen-share", {}, {}},
        {"reveal_tol_m", "--reveal-tol", {}, {}},
        {"free_max_extent_m", "--free-max-extent", {}, {}},
        {"free_min_top_m", "--free-min-top", {}, {}},
        {"free_top_quantile", "--free-top-quantile", {}, {}},
        {"guid_match_m", "--guid-match", {}, {}},
        {"guid_match_angle_deg", "--guid-match-angle", {}, {}},
        {"guid_min_overlap", "--guid-min-overlap", {}, {}}}},
      {"openings",
       {"create", "openings"},
       {{"cell_m", "--cell", {}, {}},
        {"slab_margin_m", "--slab-margin", {}, {}},
        {"unknown_half_thickness_m", "--unknown-half-thickness", {}, {}},
        {"front_m", "--front", {}, {}},
        {"floor_clear_m", "--floor-clear", {}, {}},
        {"ceiling_clear_m", "--ceiling-clear", {}, {}},
        {"local_levels", {}, {}, "--storey-levels"},
        {"level_search_m", "--level-search", {}, {}},
        {"min_level_points", "--min-level-points", {}, {}},
        {"sill_snap_m", "--sill-snap", {}, {}},
        {"min_wall_points", "--min-wall-points", {}, {}},
        {"min_occluder_points", "--min-occluder-points", {}, {}},
        {"morph_radius", "--morph-radius", {}, {}},
        {"min_width_m", "--min-width", {}, {}},
        {"min_height_m", "--min-height", {}, {}},
        {"min_fill", "--min-fill", {}, {}},
        {"min_open_share", "--min-open-share", {}, {}},
        {"min_jamb_share", "--min-jamb-share", {}, {}},
        {"drop_top_bands", {}, {}, "--keep-top-bands"},
        {"top_band_max_m", "--top-band-max", {}, {}},
        {"min_floor_height_m", "--min-floor-height", {}, {}},
        {"door_max_sill_m", "--door-max-sill", {}, {}},
        {"door_min_height_m", "--door-min-height", {}, {}},
        {"door_max_height_m", "--door-max-height", {}, {}},
        {"full_height_tol_m", "--full-height-tol", {}, {}},
        {"mullion_max_gap_m", "--mullion-max-gap", {}, {}},
        {"mullion_max_dz_m", "--mullion-max-dz", {}, {}},
        {"evidence_band_m", "--evidence-band", {}, {}},
        {"min_class_share", "--min-class-share", {}, {}},
        {"min_instance_points", "--min-instance-points", {}, {}},
        {"min_glass_share", "--min-glass-share", {}, {}},
        {"crossing_max_step_m", "--crossing-max-step", {}, {}},
        {"crossing_tol_m", "--crossing-tol", {}, {}},
        {"crossing_door_max_sill_m", "--crossing-door-max-sill", {}, {}},
        {"crossing_storey_below_m", "--crossing-storey-below", {}, {}},
        {"crossing_storey_above_m", "--crossing-storey-above", {}, {}},
        {"leaf_margin_m", "--leaf-margin", {}, {}},
        {"min_leaf_points", "--min-leaf-points", {}, {}},
        {"leaf_bin_deg", "--leaf-bin", {}, {}},
        {"leaf_min_radius_share", "--leaf-min-radius", {}, {}},
        {"guid_match_m", "--guid-match", {}, {}},
        {"guid_min_overlap", "--guid-min-overlap", {}, {}}}},
      {"spaces",
       {"create", "spaces"},
       {{"use_walls", {}, {}, "--no-walls"},
        {"axes_only", {}, "--axes-only", {}},
        {"wall_min_seen_share", "--wall-min-seen", {}, {}},
        {"join_extension_m", "--join-extension", {}, {}},
        {"bridge_max_offset_m", "--bridge-max-offset", {}, {}},
        {"bridge_max_gap_m", "--bridge-max-gap", {}, {}},
        {"bridge_max_angle_deg", "--bridge-max-angle", {}, {}},
        {"exterior_depth_m", "--exterior-depth", {}, {}},
        {"exterior_min_ratio", "--exterior-min-ratio", {}, {}},
        {"close_at_interior", {}, {}, "--no-interior-outline"},
        {"interior_simplify_m", "--interior-simplify", {}, {}},
        {"pure_min_share", "--pure-min-share", {}, {}},
        {"guest_min_share", "--guest-min-share", {}, {}},
        {"guest_grow_m", "--guest-grow", {}, {}},
        {"empty_max_share", "--empty-max-share", {}, {}},
        {"small_face_area", "--small-face-area", {}, {}},
        {"min_piece_area", "--min-piece-area", {}, {}},
        {"min_hole_area", "--min-hole-area", {}, {}},
        {"boundary_tolerance_m", "--boundary-tolerance", {}, {}},
        {"boundary_max_angle_deg", "--boundary-max-angle", {}, {}},
        {"boundary_min_length_m", "--boundary-min-length", {}, {}},
        {"corridor_min_aspect", "--corridor-min-aspect", {}, {}},
        {"open_plan_min_area", "--open-plan-min-area", {}, {}},
        {"open_plan_min_holes", "--open-plan-min-holes", {}, {}},
        {"open_plan_min_hole_area", "--open-plan-min-hole-area", {}, {}},
        {"measure_heights", {}, {}, "--storey-heights"},
        {"height_floor_band_m", "--height-floor-band", {}, {}},
        {"height_ceiling_min_m", "--height-ceiling-min", {}, {}},
        {"height_min_cells", "--height-min-cells", {}, {}},
        {"height_min_share", "--height-min-share", {}, {}},
        {"guid_match_m", "--guid-match", {}, {}},
        {"guid_min_area_ratio", "--guid-min-area-ratio", {}, {}}}},
      {"solids",
       {"create", "solids"},
       {{"unknown_wall_thickness_m", "--unknown-thickness", {}, {}},
        {"extend_at_joins", {}, {}, "--no-join-extension"},
        {"opening_overcut_m", "--opening-overcut", {}, {}},
        {"slab_thickness_m", "--slab-thickness", {}, {}},
        {"max_slab_thickness_m", "--max-slab-thickness", {}, {}},
        {"slab_min_area_m2", "--slab-min-area", {}, {}},
        {"slab_min_hole_area_m2", "--slab-min-hole-area", {}, {}},
        {"roof", {}, {}, "--no-roof"}}},
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
