// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/opening_parameters.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

// An accessor into OpeningStageOptions, decayed to a plain function pointer
// so it fits OpeningParameter::Field.
#define OPEN(f)                                                                \
  +[](OpeningStageOptions &o) -> decltype(auto) { return (o.openings.f); }
#define IDENT(f)                                                               \
  +[](OpeningStageOptions &o) -> decltype(auto) { return (o.identity.f); }

constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

const std::vector<OpeningParameter> &opening_parameter_table() {
  // Order = the OpeningOptions field order (reconstruction/openings.hpp,
  // where each field is documented in full), then identity.
  static const std::vector<OpeningParameter> table = {
      // --- W5.1 face raster
      {"cell_m", "--cell", "Raster cell [m]",
       "Side of the (s, z) face cells, anchored at the wall's start and the "
       "storey floor.",
       OPEN(cell_m), 0.01, 1.0},
      {"slab_margin_m", "--slab-margin", "Slab margin [m]",
       "Wall points lie within half the thickness plus this of the axis.",
       OPEN(slab_margin_m), 0.0, 1.0},
      {"unknown_half_thickness_m", "--unknown-half-thickness",
       "Unknown half-thickness [m]",
       "Half-thickness of a wall whose thickness is unknown.",
       OPEN(unknown_half_thickness_m), 0.0, 1.0},
      {"front_m", "--front", "Occluder reach [m]",
       "Anything this close in front of a face hides it: the cell is "
       "occluded, not open.",
       OPEN(front_m), 0.0, 10.0},
      {"floor_clear_m", "--floor-clear", "Floor clearance [m]",
       "Points this close to the floor are neither wall nor occluder.",
       OPEN(floor_clear_m), 0.0, 1.0},
      {"ceiling_clear_m", "--ceiling-clear", "Ceiling clearance [m]",
       "Points this close to the ceiling are neither wall nor occluder.",
       OPEN(ceiling_clear_m), 0.0, 1.0},
      {"local_levels", "--storey-levels", "Measure floor and ceiling locally",
       "Use the floor and ceiling measured in front of each wall, not the "
       "storey's (vertical drift between rooms).",
       OPEN(local_levels), std::nullopt, std::nullopt},
      {"level_search_m", "--level-search", "Level search [m]",
       "Look this far above the storey floor / below its ceiling for the "
       "local level.",
       OPEN(level_search_m), 0.0, 5.0},
      {"min_level_points", "--min-level-points", "Minimum level points",
       "A local floor or ceiling layer holds at least this many points.",
       OPEN(min_level_points), 1.0, kInf},
      {"sill_snap_m", "--sill-snap", "Sill snap [m]",
       "A sill at most this above the floor stands on it.", OPEN(sill_snap_m),
       0.0, 1.0},
      {"min_wall_points", "--min-wall-points", "Minimum wall points",
       "A cell is solid with at least this many wall points.",
       OPEN(min_wall_points), 1.0, kInf},
      {"min_occluder_points", "--min-occluder-points",
       "Minimum occluder points",
       "A cell is occluded with at least this many points in front of it.",
       OPEN(min_occluder_points), 1.0, kInf},
      // --- W5.1 rectangle fit
      {"morph_radius", "--morph-radius", "Morphology radius [cells]",
       "Radius of the morphological opening of the open cells.",
       OPEN(morph_radius), 0.0, 10.0},
      {"min_width_m", "--min-width", "Narrowest opening [m]",
       "Rectangles narrower than this are dropped.", OPEN(min_width_m), 0.0,
       20.0},
      {"min_height_m", "--min-height", "Lowest opening [m]",
       "Rectangles lower than this are dropped.", OPEN(min_height_m), 0.0,
       10.0},
      {"min_fill", "--min-fill", "Minimum fill",
       "At least this share of a rectangle's cells is open.", OPEN(min_fill),
       0.0, 1.0},
      {"min_open_share", "--min-open-share", "Minimum open share",
       "At least this share of a rectangle's cells is seen through (not only "
       "hidden).",
       OPEN(min_open_share), 0.0, 1.0},
      {"min_jamb_share", "--min-jamb-share", "Minimum jamb share",
       "The cells just beyond each side are solid over this share of the "
       "rows (occluded counts half).",
       OPEN(min_jamb_share), 0.0, 1.0},
      {"drop_top_bands", "--keep-top-bands", "Drop unseen top bands",
       "A rectangle reaching the top of the face but not the floor is unseen "
       "face, not an opening.",
       OPEN(drop_top_bands), std::nullopt, std::nullopt},
      {"top_band_max_m", "--top-band-max", "Top band below [m]",
       "A rectangle at the top of the face lower than this is unseen face.",
       OPEN(top_band_max_m), 0.0, 10.0},
      {"min_floor_height_m", "--min-floor-height", "Lowest floor opening [m]",
       "A rectangle on the floor lower than this is the gap under furniture.",
       OPEN(min_floor_height_m), 0.0, 10.0},
      // --- W5.2 classification
      {"door_max_sill_m", "--door-max-sill", "Highest door sill [m]",
       "A door's sill is below this above the floor.", OPEN(door_max_sill_m),
       0.0, 5.0},
      {"door_min_height_m", "--door-min-height", "Lowest door [m]",
       "A door is at least this tall ...", OPEN(door_min_height_m), 0.0, 10.0},
      {"door_max_height_m", "--door-max-height", "Tallest door [m]",
       "... and at most this tall.", OPEN(door_max_height_m), 0.0, 10.0},
      {"full_height_tol_m", "--full-height-tol",
       "Floor-to-ceiling tolerance [m]",
       "A low-silled opening whose head is this close to the ceiling is a "
       "floor-to-ceiling opening.",
       OPEN(full_height_tol_m), 0.0, 2.0},
      {"mullion_max_gap_m", "--mullion-max-gap", "Widest mullion [m]",
       "Rectangles this close along the wall with matching sill and head are "
       "one opening.",
       OPEN(mullion_max_gap_m), 0.0, 5.0},
      {"mullion_max_dz_m", "--mullion-max-dz", "Mullion height match [m]",
       "Sills and heads agree within this.", OPEN(mullion_max_dz_m), 0.0, 5.0},
      // --- W5.2 evidence
      {"evidence_band_m", "--evidence-band", "Evidence band [m]",
       "Points this far beyond the slab count as on the face for labels.",
       OPEN(evidence_band_m), 0.0, 5.0},
      {"min_class_share", "--min-class-share", "Minimum class share",
       "A semantic class supports an opening on this share of its points.",
       OPEN(min_class_share), 0.0, 1.0},
      {"min_instance_points", "--min-instance-points",
       "Minimum instance points",
       "An instance supports an opening with this many points on it.",
       OPEN(min_instance_points), 1.0, kInf},
      {"min_glass_share", "--min-glass-share", "Minimum glass share",
       "Glass images support a window over this share of it.",
       OPEN(min_glass_share), 0.0, 1.0},
      {"crossing_max_step_m", "--crossing-max-step", "Longest camera step [m]",
       "Longer steps of the camera path are jumps, not walks.",
       OPEN(crossing_max_step_m), 0.0, 20.0},
      {"crossing_tol_m", "--crossing-tol", "Crossing tolerance [m]",
       "A camera path crossing this close to an opening supports it.",
       OPEN(crossing_tol_m), 0.0, 5.0},
      {"crossing_door_max_sill_m", "--crossing-door-max-sill",
       "Walked-through sill [m]",
       "A rectangle the camera path crosses with its sill at most this high "
       "reaches the floor.",
       OPEN(crossing_door_max_sill_m), 0.0, 5.0},
      {"crossing_storey_below_m", "--crossing-storey-below",
       "Camera below the floor [m]",
       "A camera this far below the floor still belongs to the storey.",
       OPEN(crossing_storey_below_m), 0.0, 5.0},
      {"crossing_storey_above_m", "--crossing-storey-above",
       "Camera above the ceiling [m]",
       "A camera this far above the ceiling still belongs to the storey.",
       OPEN(crossing_storey_above_m), 0.0, 5.0},
      // --- W5.2 door state
      {"leaf_margin_m", "--leaf-margin", "Leaf margin [m]",
       "Leaf points lie this far above the sill and below the head.",
       OPEN(leaf_margin_m), 0.0, 2.0},
      {"min_leaf_points", "--min-leaf-points", "Minimum leaf points",
       "An open leaf has at least this many points in one angular bin.",
       OPEN(min_leaf_points), 1.0, kInf},
      {"leaf_bin_deg", "--leaf-bin", "Leaf angle bin [deg]",
       "Angular bin of the leaf search.", OPEN(leaf_bin_deg), 1.0, 45.0},
      {"leaf_min_radius_share", "--leaf-min-radius", "Leaf inner radius",
       "Leaf points lie at least this share of the door's width from the "
       "hinge.",
       OPEN(leaf_min_radius_share), 0.0, 1.0},
      // --- identity (core::OpeningIdentityOptions)
      {"guid_match_m", "--guid-match", "Same opening within [m]",
       "A re-detected opening keeps the GUID of the stored one in the same "
       "wall whose start lies within this ...",
       IDENT(match_m), 0.0, 5.0},
      {"guid_min_overlap", "--guid-min-overlap", "Same opening overlap",
       "... and that overlaps it by this share of the narrower one.",
       IDENT(min_overlap), 0.0, 1.0},
  };
  return table;
}

#undef OPEN
#undef IDENT

const std::vector<ParameterDescriptor> &opening_stage_parameters() {
  static const std::vector<ParameterDescriptor> descriptors = [] {
    OpeningStageOptions d{};
    std::vector<ParameterDescriptor> out;
    for (const auto &p : opening_parameter_table()) {
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

OpeningStageOptions
opening_options_from_parameters(std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("openings parameters must be a JSON object");
  }
  OpeningStageOptions o{};
  for (const auto &p : opening_parameter_table()) {
    const auto it = params.find(std::string(p.key));
    if (it == params.end() || it->is_null())
      continue;
    const auto bad = [&](std::string_view why) {
      return std::invalid_argument(
          fmt::format("openings parameter '{}': {}", p.key, why));
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
  // Cross-field: a door's height range must not be empty.
  if (o.openings.door_min_height_m > o.openings.door_max_height_m)
    throw std::invalid_argument(fmt::format(
        "openings parameters: door_min_height_m {} exceeds door_max_height_m "
        "{}",
        o.openings.door_min_height_m, o.openings.door_max_height_m));
  return o;
}

std::string opening_parameters_json(const OpeningStageOptions &options) {
  OpeningStageOptions o = options;
  json out = json::object();
  for (const auto &p : opening_parameter_table())
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
