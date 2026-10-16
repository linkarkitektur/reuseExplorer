// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The openings stage's parameters, in ONE table (reusex-z1v.7.3), the
// pattern of pipeline/wall_parameters.hpp: the stage parser, the
// stage_parameters() descriptors, the `rux create openings` flags and the Qt
// client's "Kopiér som rux-kommando" spec all come from
// opening_parameter_table(): JSON key (= the OpeningOptions /
// OpeningIdentityOptions field name), CLI flag, label, description, bounds
// and an accessor into OpeningStageOptions.

#include <reusex/core/opening_identity.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/reconstruction/openings.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// Everything the openings stage is tuned by.
struct OpeningStageOptions {
  geometry::OpeningOptions openings;
  /// GUID matching against the stored openings (keys guid_match_m,
  /// guid_min_overlap).
  core::OpeningIdentityOptions identity;
};

/// One parameter of the openings stage.
struct OpeningParameter {
  using Field = std::variant<double &(*)(OpeningStageOptions &),
                             int &(*)(OpeningStageOptions &),
                             bool &(*)(OpeningStageOptions &)>;
  std::string_view key;  ///< JSON key: the option field's name
  std::string_view flag; ///< `rux create openings` flag; for a boolean that
                         ///< defaults to true, the flag that turns it off
  std::string_view label;
  std::string_view description;
  Field field;
  std::optional<double> minimum, maximum; ///< inclusive bounds
};

/// Every parameter, in form-display order (the OpeningOptions field order,
/// then identity).
const std::vector<OpeningParameter> &opening_parameter_table();

/// stage_parameters(JobStage::openings): the table as descriptors, defaults
/// from a default-constructed OpeningStageOptions.
const std::vector<ParameterDescriptor> &opening_stage_parameters();

/// Read the openings stage's JSON parameters ("" = defaults). Unknown keys
/// are ignored.
/// @throws std::invalid_argument for a non-object JSON, a value of the wrong
///         type, or one outside its bounds.
OpeningStageOptions
opening_options_from_parameters(std::string_view parameters);

/// The JSON parameters (every key of the table) for @p options.
std::string opening_parameters_json(const OpeningStageOptions &options);

} // namespace reusex::pipeline
