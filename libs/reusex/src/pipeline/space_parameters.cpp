// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/space_parameters.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

// An accessor into SpaceStageOptions, decayed to a plain function pointer so
// it fits SpaceParameter::Field.
#define SPACE(f)                                                               \
  +[](SpaceStageOptions &o) -> decltype(auto) { return (o.spaces.f); }
#define IDENT(f)                                                               \
  +[](SpaceStageOptions &o) -> decltype(auto) { return (o.identity.f); }

constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

const std::vector<SpaceParameter> &space_parameter_table() {
  // Order = the SpacesOptions field order (segmentation/spaces.hpp, where
  // each field is documented in full), then identity.
  static const std::vector<SpaceParameter> table = {
      // --- arrangement
      {"use_walls", "--no-walls", "Use the walls table",
       "Build the arrangement from the walls; off, every space is its "
       "raster outline.",
       SPACE(use_walls), std::nullopt, std::nullopt},
      {"axes_only", "--axes-only", "Wall axes only",
       "Insert the wall axes instead of their two faces.", SPACE(axes_only),
       std::nullopt, std::nullopt},
      {"wall_min_seen_share", "--wall-min-seen", "Least seen wall share",
       "Walls seen on less of their face than this share are left out.",
       SPACE(wall_min_seen_share), 0.0, 1.0},
      {"join_extension_m", "--join-extension", "Wall extension [m]",
       "Wall faces run this far past the wall's ends, so corners and tees "
       "with a gap still close.",
       SPACE(join_extension_m), 0.0, 100.0},
      {"bridge_max_offset_m", "--bridge-max-offset", "Bridge offset [m]",
       "Two parallel wall runs at most this far apart whose ends meet get a "
       "bridge across them, so an offset wall still closes; 0 disables ...",
       SPACE(bridge_max_offset_m), 0.0, 5.0},
      {"bridge_max_gap_m", "--bridge-max-gap", "Bridge gap [m]",
       "... when their facing ends are at most this far apart (or overlap "
       "by at most this) ...",
       SPACE(bridge_max_gap_m), 0.0, 100.0},
      {"bridge_max_angle_deg", "--bridge-max-angle", "Bridge angle [deg]",
       "... and they are at most this far from parallel.",
       SPACE(bridge_max_angle_deg), 0.0, 90.0},
      {"exterior_depth_m", "--exterior-depth", "Exterior depth [m]",
       "Room cells up to this far beyond an external wall's outer face are "
       "outside the building (a footprint leak through a window); 0 "
       "disables ...",
       SPACE(exterior_depth_m), 0.0, 100.0},
      {"exterior_min_ratio", "--exterior-min-ratio", "Exterior side ratio",
       "... when the wall's inside holds at least this many times the room "
       "cells of its outside.",
       SPACE(exterior_min_ratio), 1.0, 1000.0},
      {"close_at_interior", "--no-interior-outline",
       "Close at the interior outline",
       "Insert the storey's interior outline, so rooms the walls leave open "
       "are closed by it.",
       SPACE(close_at_interior), std::nullopt, std::nullopt},
      {"interior_simplify_m", "--interior-simplify",
       "Interior outline simplification [m]",
       "The interior outline is simplified to this tolerance.",
       SPACE(interior_simplify_m), 0.0, 5.0},
      // --- labelling
      {"pure_min_share", "--pure-min-share", "Pure face share",
       "A face is one room's when the room covers this share of it.",
       SPACE(pure_min_share), 0.0, 1.0},
      {"guest_min_share", "--guest-min-share", "Guest room share",
       "Another room with this share of its cells in a pure face keeps its "
       "own cells there.",
       SPACE(guest_min_share), 0.0, 1.0},
      {"guest_grow_m", "--guest-grow", "Guest growth [m]",
       "A guest room's cells grow this far to reach the walls.",
       SPACE(guest_grow_m), 0.0, 5.0},
      {"empty_max_share", "--empty-max-share", "Empty face share",
       "A face rooms cover at most this share of is empty.",
       SPACE(empty_max_share), 0.0, 1.0},
      {"small_face_area", "--small-face-area", "Small face [m2]",
       "Faces smaller than this take their longest neighbour's class.",
       SPACE(small_face_area), 0.0, 100.0},
      // --- polygons
      {"min_piece_area", "--min-piece-area", "Warn on dropped piece [m2]",
       "Pieces of a space other than its largest are dropped; larger ones "
       "than this are named in a warning.",
       SPACE(min_piece_area), 0.0, 1000.0},
      {"min_hole_area", "--min-hole-area", "Smallest hole [m2]",
       "Holes smaller than this are filled.", SPACE(min_hole_area), 0.0,
       1000.0},
      // --- boundaries
      {"boundary_tolerance_m", "--boundary-tolerance", "Boundary tolerance [m]",
       "A polygon edge lies on a wall face when both ends are this close to "
       "it ...",
       SPACE(boundary_tolerance_m), 0.0, 5.0},
      {"boundary_max_angle_deg", "--boundary-max-angle", "Boundary angle [deg]",
       "... it is at most this far from parallel ...",
       SPACE(boundary_max_angle_deg), 0.0, 90.0},
      {"boundary_min_length_m", "--boundary-min-length", "Boundary overlap [m]",
       "... and it overlaps the wall by this much.",
       SPACE(boundary_min_length_m), 0.0, 100.0},
      // --- kind
      {"corridor_min_aspect", "--corridor-min-aspect", "Corridor aspect",
       "A space whose minimum bounding rectangle is longer than this ratio "
       "is a corridor.",
       SPACE(corridor_min_aspect), 1.0, 1000.0},
      {"open_plan_min_area", "--open-plan-min-area", "Open plan area [m2]",
       "An open plan is at least this big ...", SPACE(open_plan_min_area), 0.0,
       kInf},
      {"open_plan_min_holes", "--open-plan-min-holes",
       "Open plan furniture holes",
       "... and encloses at least this many furniture pockets ...",
       SPACE(open_plan_min_holes), 0.0, kInf},
      {"open_plan_min_hole_area", "--open-plan-min-hole-area",
       "Furniture hole [m2]", "... of at least this area.",
       SPACE(open_plan_min_hole_area), 0.0, 1000.0},
      // --- heights (measure_space_heights)
      {"measure_heights", "--storey-heights", "Measure each space's heights",
       "Measure each space's own floor and ceiling from the points over it "
       "(split levels, sloped ceilings); off, every space takes its "
       "storey's.",
       SPACE(measure_heights), std::nullopt, std::nullopt},
      {"height_floor_band_m", "--height-floor-band", "Floor sample band [m]",
       "A horizontal point this close to its cell's floor is a floor "
       "sample ...",
       SPACE(height_floor_band_m), 0.0, 1.0},
      {"height_ceiling_min_m", "--height-ceiling-min",
       "Ceiling sample height [m]",
       "... and one at least this far above it a ceiling sample.",
       SPACE(height_ceiling_min_m), 0.5, 10.0},
      {"height_min_cells", "--height-min-cells", "Sampled cells",
       "A space needs samples in this many grid cells to take a measured "
       "height ...",
       SPACE(height_min_cells), 1.0, kInf},
      {"height_min_share", "--height-min-share", "Sampled share",
       "... and in this share of its area.", SPACE(height_min_share), 0.0, 1.0},
      // --- identity (core::SpaceIdentityOptions)
      {"guid_match_m", "--guid-match", "Same space within [m]",
       "A re-detected space keeps the GUID of the stored one of its storey "
       "whose centroid lies within this ...",
       IDENT(match_m), 0.0, 100.0},
      {"guid_min_area_ratio", "--guid-min-area-ratio", "Same space area ratio",
       "... and whose area is within this ratio (smaller / larger).",
       IDENT(min_area_ratio), 0.0, 1.0},
  };
  return table;
}

#undef SPACE
#undef IDENT

const std::vector<ParameterDescriptor> &space_stage_parameters() {
  static const std::vector<ParameterDescriptor> descriptors = [] {
    SpaceStageOptions d{};
    std::vector<ParameterDescriptor> out;
    for (const auto &p : space_parameter_table()) {
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

SpaceStageOptions space_options_from_parameters(std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("spaces parameters must be a JSON object");
  }
  SpaceStageOptions o{};
  for (const auto &p : space_parameter_table()) {
    const auto it = params.find(std::string(p.key));
    if (it == params.end() || it->is_null())
      continue;
    const auto bad = [&](std::string_view why) {
      return std::invalid_argument(
          fmt::format("spaces parameter '{}': {}", p.key, why));
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

std::string space_parameters_json(const SpaceStageOptions &options) {
  SpaceStageOptions o = options;
  json out = json::object();
  for (const auto &p : space_parameter_table())
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
