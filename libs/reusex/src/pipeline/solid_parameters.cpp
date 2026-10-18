// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/pipeline/solid_parameters.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace reusex::pipeline {
namespace {

using json = nlohmann::json;

// An accessor into SolidStageOptions, decayed to a plain function pointer so
// it fits SolidParameter::Field.
#define SOLID(f)                                                               \
  +[](SolidStageOptions &o) -> decltype(auto) { return (o.solids.f); }

constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

const std::vector<SolidParameter> &solid_parameter_table() {
  // Order = the SolidsOptions field order (reconstruction/solids.hpp, where
  // each field is documented in full).
  static const std::vector<SolidParameter> table = {
      {"unknown_wall_thickness_m", "--unknown-thickness",
       "Unknown wall thickness [m]",
       "A wall seen from one side only gets this thickness, away from the "
       "space it faces.",
       SOLID(unknown_wall_thickness_m), 0.01, 2.0},
      {"extend_at_joins", "--no-join-extension", "Extend walls at joins",
       "A wall end at a corner or tee runs on to the far face of the wall it "
       "meets.",
       SOLID(extend_at_joins), std::nullopt, std::nullopt},
      {"opening_overcut_m", "--opening-overcut", "Opening overcut [m]",
       "An opening box reaches this far past the wall faces it cuts.",
       SOLID(opening_overcut_m), 0.0, 1.0},
      {"slab_thickness_m", "--slab-thickness", "Default slab thickness [m]",
       "Slab thickness with no storey below, and of the roof.",
       SOLID(slab_thickness_m), 0.01, 2.0},
      {"max_slab_thickness_m", "--max-slab-thickness",
       "Thickest slab between storeys [m]",
       "A wider gap to the storey below is not a slab; the default is used.",
       SOLID(max_slab_thickness_m), 0.01, 10.0},
      {"slab_min_area_m2", "--slab-min-area", "Smallest slab piece [m2]",
       "Pieces of a storey's floor polygon smaller than this are dropped.",
       SOLID(slab_min_area_m2), 0.0, 1000.0},
      {"slab_min_hole_area_m2", "--slab-min-hole-area",
       "Smallest slab hole [m2]",
       "Holes in a floor polygon smaller than this are filled.",
       SOLID(slab_min_hole_area_m2), 0.0, 1000.0},
      {"roof", "--no-roof", "Roof slab",
       "Also build a roof slab over the top storey's ceiling.", SOLID(roof),
       std::nullopt, std::nullopt},
  };
  return table;
}

#undef SOLID

const std::vector<ParameterDescriptor> &solid_stage_parameters() {
  static const std::vector<ParameterDescriptor> descriptors = [] {
    SolidStageOptions d{};
    std::vector<ParameterDescriptor> out;
    for (const auto &p : solid_parameter_table()) {
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

SolidStageOptions solid_options_from_parameters(std::string_view parameters) {
  json params = json::object();
  if (!parameters.empty()) {
    params = json::parse(parameters, nullptr, /*allow_exceptions=*/false);
    if (params.is_discarded() || !params.is_object())
      throw std::invalid_argument("solids parameters must be a JSON object");
  }
  SolidStageOptions o{};
  for (const auto &p : solid_parameter_table()) {
    const auto it = params.find(std::string(p.key));
    if (it == params.end() || it->is_null())
      continue;
    const auto bad = [&](std::string_view why) {
      return std::invalid_argument(
          fmt::format("solids parameter '{}': {}", p.key, why));
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

std::string solid_parameters_json(const SolidStageOptions &options) {
  SolidStageOptions o = options;
  json out = json::object();
  for (const auto &p : solid_parameter_table())
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
