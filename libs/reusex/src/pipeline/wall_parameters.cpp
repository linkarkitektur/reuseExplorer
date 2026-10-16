// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/wall_parameters.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

// An accessor into WallStageOptions, decayed to a plain function pointer so
// it fits WallParameter::Field.
#define WALL(f)                                                                \
  +[](WallStageOptions &o) -> decltype(auto) { return (o.walls.f); }
#define IDENT(f)                                                               \
  +[](WallStageOptions &o) -> decltype(auto) { return (o.identity.f); }

constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

const std::vector<WallParameter> &wall_parameter_table() {
  // Order = the WallOptions field order (reconstruction/walls.hpp, where each
  // field is documented in full), then identity.
  static const std::vector<WallParameter> table = {
      // --- W3.1 axis families
      {"max_families", "--max-families", "Extra axis families",
       "Families allowed beyond the dominant orthogonal pair.",
       WALL(max_families), 0.0, 8.0},
      {"family_angle_deg", "--family-angle", "Family angle [deg]",
       "Normals this far from both dominant axes are candidates for an extra "
       "family; also the clustering and inlier window.",
       WALL(family_angle_deg), 0.5, 45.0},
      {"min_family_share", "--min-family-share", "Minimum family share",
       "An extra family holds at least this share of the vertical-plane "
       "inlier mass.",
       WALL(min_family_share), 0.0, 1.0},
      {"wall_role_only", "--all-vertical-planes", "Wall planes only",
       "Only planes classified as walls vote for the families (else every "
       "vertical plane, which is also the fallback when a storey has none).",
       WALL(wall_role_only), std::nullopt, std::nullopt},
      {"family_refine_iterations", "--family-refine-iterations",
       "Family refinements", "Refinements of the dominant pair (bounded).",
       WALL(family_refine_iterations), 0.0, 20.0},
      {"family_refine_deg", "--family-refine", "Family refine window [deg]",
       "Narrowest inlier window of the dominant-pair refinement.",
       WALL(family_refine_deg), 0.05, 45.0},
      // --- W3.2 wall faces
      {"assign_angle_deg", "--assign-angle", "Assign angle [deg]",
       "A wall point joins the family nearest its plane normal within this.",
       WALL(assign_angle_deg), 0.5, 45.0},
      {"square_deg", "--square", "Square within [deg]",
       "A wall plane this close to its family axis is squared onto it.",
       WALL(square_deg), 0.0, 45.0},
      {"bin_m", "--bin", "Offset bin [m]",
       "Bin of the offset profile; anchored at the level-frame origin.",
       WALL(bin_m), 0.001, 0.2},
      {"min_peak_points", "--min-peak-points", "Minimum peak height",
       "A face's smoothed profile peak holds at least this many points per "
       "bin.",
       WALL(min_peak_points), 0.0, kInf},
      {"min_peak_prominence_share", "--min-peak-prominence",
       "Minimum peak prominence",
       "A peak stands out from its deeper base by this share of its height.",
       WALL(min_peak_prominence_share), 0.0, 1.0},
      {"min_peak_separation_m", "--min-peak-separation",
       "Minimum peak separation [m]",
       "A peak is at least this far from a higher accepted one.",
       WALL(min_peak_separation_m), 0.0, 1.0},
      {"face_window_m", "--face-window", "Face window [m]",
       "Half-width of the slab of points a face takes around its peak.",
       WALL(face_window_m), 0.001, 0.5},
      {"min_column_share", "--min-column-share", "Minimum column share",
       "A column of a face is evidence when its points cover this share of "
       "the storey's height cells.",
       WALL(min_column_share), 0.0, 1.0},
      {"run_gap_m", "--run-gap", "Run gap [m]",
       "A face splits where no solid column is seen for longer than this.",
       WALL(run_gap_m), 0.0, 20.0},
      {"min_run_m", "--min-run", "Minimum run [m]",
       "Runs shorter than this are dropped.", WALL(min_run_m), 0.0, 20.0},
      {"min_run_points", "--min-run-points", "Minimum run points",
       "Runs with fewer points are dropped.", WALL(min_run_points), 0.0, kInf},
      {"cell_m", "--cell", "Occupancy cell [m]",
       "Side of the (s, z) occupancy cells, anchored at the frame origin.",
       WALL(cell_m), 0.01, 2.0},
      {"storey_margin_m", "--storey-margin", "Storey margin [m]",
       "Points this far below the floor / above the ceiling still belong to "
       "the storey.",
       WALL(storey_margin_m), 0.0, 2.0},
      // --- W3.3 pairing, thickness, external test
      {"min_gap_m", "--min-gap", "Thinnest wall [m]",
       "Two faces at least this far apart can be the sides of one wall ...",
       WALL(min_gap_m), 0.0, 2.0},
      {"max_gap_m", "--max-gap", "Thickest wall [m]",
       "... and at most this far apart.", WALL(max_gap_m), 0.0, 2.0},
      {"min_shared_m", "--min-shared", "Minimum shared length [m]",
       "Paired faces share at least this length along the axis.",
       WALL(min_shared_m), 0.0, 20.0},
      {"max_valley_share", "--max-valley-share", "Maximum valley share",
       "The emptiest bin between paired faces holds at most this share of the "
       "weaker face's peak.",
       WALL(max_valley_share), 0.0, 1.0},
      {"min_co_visibility", "--min-co-visibility", "Minimum co-visibility",
       "This share of the cells seen on either face over the shared stretch "
       "is seen on both.",
       WALL(min_co_visibility), 0.0, 1.0},
      {"max_crossed_gap_m", "--max-crossed-gap", "Crossed pair gap [m]",
       "Faces looking at each other closer than this are one wall seen with "
       "drift.",
       WALL(max_crossed_gap_m), 0.0, 2.0},
      {"min_cell_points", "--min-cell-points", "Minimum cell points",
       "A cell enters the thickness fit with at least this many points.",
       WALL(min_cell_points), 1.0, kInf},
      {"outlier_sigma_k", "--outlier-sigma", "Outlier sigmas",
       "Theil-Sen residuals beyond this many robust sigmas drop a cell.",
       WALL(outlier_sigma_k), 0.5, 20.0},
      {"min_robust_sigma_m", "--min-robust-sigma", "Minimum robust sigma [m]",
       "Floor of the robust sigma (1.4826 x MAD).", WALL(min_robust_sigma_m),
       0.0, 1.0},
      {"bootstrap_samples", "--bootstrap-samples", "Bootstrap samples",
       "Resamples (over cells) for the thickness CI.", WALL(bootstrap_samples),
       0.0, 100000.0},
      {"seed", "--seed", "Bootstrap seed",
       "Seed of the bootstrap (deterministic CIs).", WALL(seed), 0.0,
       4294967295.0},
      {"ci_level", "--ci-level", "CI level",
       "Confidence level of the thickness interval.", WALL(ci_level), 0.5,
       0.999},
      {"default_thickness_m", "--default-thickness", "Assumed thickness [m]",
       "Thickness given to a wall seen from one side only.",
       WALL(default_thickness_m), 0.0, 2.0},
      {"external_strip_near_m", "--external-strip-near",
       "External strip from [m]",
       "The room-evidence strip beyond each face starts this far out ...",
       WALL(external_strip_near_m), 0.0, 5.0},
      {"external_strip_far_m", "--external-strip-far", "External strip to [m]",
       "... and ends this far out.", WALL(external_strip_far_m), 0.0, 10.0},
      {"external_max_z_m", "--external-max-z", "External strip height [m]",
       "Only points below this height above the floor count as room "
       "evidence.",
       WALL(external_max_z_m), 0.0, 10.0},
      {"min_room_density", "--min-room-density",
       "Minimum room density [pts/m^2]",
       "Fewer non-wall points per m^2 than this on one side: no room there, "
       "the wall is external.",
       WALL(min_room_density), 0.0, kInf},
      // --- W3.4 wall graph
      {"snap_wide_m", "--snap-wide", "Snap (wide) [m]",
       "First snapping pass of a seen face to the median of the raw points.",
       WALL(snap_wide_m), 0.0, 1.0},
      {"snap_narrow_m", "--snap-narrow", "Snap (narrow) [m]",
       "Second, narrower snapping pass.", WALL(snap_narrow_m), 0.0, 1.0},
      {"bridge_gap_m", "--bridge-gap", "Bridge gap [m]",
       "Collinear walls with at most this between them merge (a doorway, a "
       "window, an occlusion).",
       WALL(bridge_gap_m), 0.0, 20.0},
      {"bridge_offset_m", "--bridge-offset", "Bridge offset [m]",
       "Collinear = faces or axes within this of each other.",
       WALL(bridge_offset_m), 0.0, 1.0},
      {"bridge_thickness_m", "--bridge-thickness", "Bridge thickness [m]",
       "Two measured walls whose thickness differs by more meet end to end "
       "instead of merging.",
       WALL(bridge_thickness_m), 0.0, 1.0},
      {"recess_max_length_m", "--recess-max-length", "Longest recess [m]",
       "A short one-sided wall up to this long behind or before a neighbour "
       "is a recess of it.",
       WALL(recess_max_length_m), 0.0, 20.0},
      {"corner_m", "--corner", "Corner reach [m]",
       "Two walls whose nearer ends lie within this of their crossing meet "
       "there.",
       WALL(corner_m), 0.0, 20.0},
      {"tee_m", "--tee", "Tee reach [m]",
       "A wall whose end lies within this of a crossing on another wall ends "
       "there.",
       WALL(tee_m), 0.0, 20.0},
      {"tee_on_wall_m", "--tee-on-wall", "Tee tolerance [m]",
       "The crossing lies on the other wall within this of its extent.",
       WALL(tee_on_wall_m), 0.0, 5.0},
      {"min_seen_share", "--min-seen-share", "Minimum seen share",
       "A wall whose face area is seen less than this is flagged.",
       WALL(min_seen_share), 0.0, 1.0},
      // --- identity (core::WallIdentityOptions)
      {"guid_match_m", "--guid-match", "Same wall within [m]",
       "A re-detected wall keeps the GUID of the stored wall whose axis lies "
       "within this ...",
       IDENT(match_m), 0.0, 5.0},
      {"guid_match_angle_deg", "--guid-match-angle", "Same wall angle [deg]",
       "... is parallel within this ...", IDENT(match_angle_deg), 0.0, 90.0},
      {"guid_min_overlap", "--guid-min-overlap", "Same wall overlap",
       "... and overlaps it by this share of the shorter one.",
       IDENT(min_overlap), 0.0, 1.0},
  };
  return table;
}

#undef WALL
#undef IDENT

const std::vector<ParameterDescriptor> &wall_stage_parameters() {
  static const std::vector<ParameterDescriptor> descriptors = [] {
    WallStageOptions d{};
    std::vector<ParameterDescriptor> out;
    for (const auto &p : wall_parameter_table()) {
      ParameterDescriptor desc;
      desc.key = std::string(p.key);
      desc.label = std::string(p.label);
      desc.description = std::string(p.description);
      if (p.minimum)
        desc.minimum = p.minimum;
      if (p.maximum && std::isfinite(*p.maximum))
        desc.maximum = p.maximum;
      std::visit(
          [&](auto get) {
            using T = std::remove_reference_t<decltype(get(d))>;
            if constexpr (std::is_same_v<T, double>) {
              desc.type = ParameterType::number;
              desc.default_value = get(d);
            } else if constexpr (std::is_same_v<T, bool>) {
              desc.type = ParameterType::boolean;
              desc.default_value = get(d);
              desc.minimum.reset();
              desc.maximum.reset();
            } else {
              desc.type = ParameterType::integer;
              desc.default_value = static_cast<long long>(get(d));
            }
          },
          p.field);
      out.push_back(std::move(desc));
    }
    return out;
  }();
  return descriptors;
}

WallStageOptions wall_options_from_parameters(std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("walls parameters must be a JSON object");
  }
  WallStageOptions o{};
  for (const auto &p : wall_parameter_table()) {
    const auto it = params.find(std::string(p.key));
    if (it == params.end() || it->is_null())
      continue;
    const auto bad = [&](std::string_view why) {
      return std::invalid_argument(
          fmt::format("walls parameter '{}': {}", p.key, why));
    };
    const auto check_range = [&](double v) {
      if (!std::isfinite(v) || (p.minimum && v < *p.minimum) ||
          (p.maximum && v > *p.maximum))
        throw bad(fmt::format("{} is outside [{}, {}]", v,
                              p.minimum.value_or(-kInf),
                              p.maximum.value_or(kInf)));
    };
    std::visit(
        [&](auto get) {
          using T = std::remove_reference_t<decltype(get(o))>;
          if constexpr (std::is_same_v<T, bool>) {
            if (!it->is_boolean())
              throw bad("expected true or false");
            get(o) = it->template get<bool>();
          } else if constexpr (std::is_same_v<T, double>) {
            if (!it->is_number())
              throw bad("expected a number");
            const double v = it->template get<double>();
            check_range(v);
            get(o) = v;
          } else {
            if (!it->is_number_integer() && !it->is_number_unsigned())
              throw bad("expected an integer");
            const double v = it->template get<double>();
            check_range(v);
            get(o) = static_cast<T>(it->template get<long long>());
          }
        },
        p.field);
  }
  return o;
}

std::string wall_parameters_json(const WallStageOptions &options) {
  WallStageOptions o = options;
  json out = json::object();
  for (const auto &p : wall_parameter_table())
    std::visit(
        [&](auto get) {
          using T = std::remove_reference_t<decltype(get(o))>;
          if constexpr (std::is_same_v<T, double>)
            // shortest round-trip decimal, as rux::StageParams writes it
            out[std::string(p.key)] = json::parse(fmt::format("{}", get(o)));
          else
            out[std::string(p.key)] = get(o);
        },
        p.field);
  return out.dump();
}

} // namespace reusex::pipeline
