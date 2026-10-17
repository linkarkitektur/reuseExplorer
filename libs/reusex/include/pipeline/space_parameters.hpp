// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The spaces stage's parameters, in ONE table (reusex-z1v.6.3), the pattern
// of pipeline/opening_parameters.hpp: the stage parser, the
// stage_parameters() descriptors, the `rux create spaces` flags and the Qt
// client's "Kopiér som rux-kommando" spec all come from
// space_parameter_table(): JSON key (= the SpacesOptions field name, or
// guid_* for SpaceIdentityOptions), CLI flag, label, description, bounds and
// an accessor into SpaceStageOptions.
//
// The band-rooms grid the spaces are polygonised from is not tuned here: the
// stage rebuilds it with the parameters the `rooms` cloud was made with, so
// its room ids are the cloud's labels.

#include <reusex/core/space_identity.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/segmentation/spaces.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// Everything the spaces stage is tuned by.
struct SpaceStageOptions {
  geometry::SpacesOptions spaces;
  /// GUID matching against the stored spaces (keys guid_match_m,
  /// guid_min_area_ratio).
  core::SpaceIdentityOptions identity;
};

/// One parameter of the spaces stage.
struct SpaceParameter {
  using Field = std::variant<double &(*)(SpaceStageOptions &),
                             int &(*)(SpaceStageOptions &),
                             bool &(*)(SpaceStageOptions &)>;
  std::string_view key;  ///< JSON key: the option field's name
  std::string_view flag; ///< `rux create spaces` flag; for a boolean, the
                         ///< flag that sets it to the opposite of its default
  std::string_view label;
  std::string_view description;
  Field field;
  std::optional<double> minimum, maximum; ///< inclusive bounds
};

/// Every parameter, in form-display order (the SpacesOptions field order,
/// then identity).
const std::vector<SpaceParameter> &space_parameter_table();

/// stage_parameters(JobStage::spaces): the table as descriptors, defaults
/// from a default-constructed SpaceStageOptions.
const std::vector<ParameterDescriptor> &space_stage_parameters();

/// Read the spaces stage's JSON parameters ("" = defaults). Unknown keys are
/// ignored.
/// @throws std::invalid_argument for a non-object JSON, a value of the wrong
///         type, or one outside its bounds.
SpaceStageOptions space_options_from_parameters(std::string_view parameters);

/// The JSON parameters (every key of the table) for @p options.
std::string space_parameters_json(const SpaceStageOptions &options);

} // namespace reusex::pipeline
