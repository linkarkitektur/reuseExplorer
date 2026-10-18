// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The solids stage's parameters, in ONE table (W6.1, reusex-z1v.8.1), the
// pattern of pipeline/space_parameters.hpp: the stage parser, the
// stage_parameters() descriptors, the `rux create solids` flags and the Qt
// client's "Kopiér som rux-kommando" spec all come from
// solid_parameter_table(): JSON key (= the SolidsOptions field name), CLI
// flag, label, description, bounds and an accessor into SolidStageOptions.

#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/reconstruction/solids.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// Names of the meshes the solids stage writes (one per element type).
inline constexpr std::string_view kSolidWallsMesh = "solid_walls";
inline constexpr std::string_view kSolidSlabsMesh = "solid_slabs";
inline constexpr std::string_view kSolidSpacesMesh = "solid_spaces";

/// Everything the solids stage is tuned by.
struct SolidStageOptions {
  geometry::SolidsOptions solids;
};

/// One parameter of the solids stage.
struct SolidParameter {
  using Field = std::variant<double &(*)(SolidStageOptions &),
                             int &(*)(SolidStageOptions &),
                             bool &(*)(SolidStageOptions &)>;
  std::string_view key;  ///< JSON key: the option field's name
  std::string_view flag; ///< `rux create solids` flag; for a boolean, the
                         ///< flag that sets it to the opposite of its default
  std::string_view label;
  std::string_view description;
  Field field;
  std::optional<double> minimum, maximum; ///< inclusive bounds
};

/// Every parameter, in form-display order (the SolidsOptions field order).
const std::vector<SolidParameter> &solid_parameter_table();

/// stage_parameters(JobStage::solids): the table as descriptors, defaults
/// from a default-constructed SolidStageOptions.
const std::vector<ParameterDescriptor> &solid_stage_parameters();

/// Read the solids stage's JSON parameters ("" = defaults). Unknown keys are
/// ignored.
/// @throws std::invalid_argument for a non-object JSON, a value of the wrong
///         type, or one outside its bounds.
SolidStageOptions solid_options_from_parameters(std::string_view parameters);

/// The JSON parameters (every key of the table) for @p options.
std::string solid_parameters_json(const SolidStageOptions &options);

} // namespace reusex::pipeline
