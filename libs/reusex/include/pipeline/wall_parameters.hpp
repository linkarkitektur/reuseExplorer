// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The walls stage's parameters, in ONE table (reusex-z1v.5.5).
//
// WallOptions has some fifty fields. Rather than repeat each one in the stage
// parser, the stage_parameters() descriptors, the `rux create walls` flags and
// the Qt client's "Kopiér som rux-kommando" spec -- four lists that would
// drift -- every one of them is generated from wall_parameter_table(): JSON
// key (= the WallOptions / WallIdentityOptions field name), CLI flag, label,
// description, bounds and an accessor into WallStageOptions.

#include <reusex/core/wall_identity.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/reconstruction/walls.hpp>

#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// Everything the walls stage is tuned by.
struct WallStageOptions {
  geometry::WallOptions walls;
  /// GUID matching against the stored walls (keys guid_match_m,
  /// guid_match_angle_deg, guid_min_overlap).
  core::WallIdentityOptions identity;
};

/// One parameter of the walls stage.
struct WallParameter {
  using Field = std::variant<
      double &(*)(WallStageOptions &), int &(*)(WallStageOptions &),
      unsigned &(*)(WallStageOptions &), bool &(*)(WallStageOptions &)>;
  std::string_view key;  ///< JSON key: the option field's name
  std::string_view flag; ///< `rux create walls` flag; for a boolean that
                         ///< defaults to true, the flag that turns it off
  std::string_view label;
  std::string_view description;
  Field field;
  std::optional<double> minimum, maximum; ///< inclusive bounds
};

/// Every parameter, in form-display order (the WallOptions field order, then
/// identity). `use_normal_sign` is not one: the stage reads it from the
/// clouds' normal_orientation marker (reusex-0se), as classify does.
const std::vector<WallParameter> &wall_parameter_table();

/// stage_parameters(JobStage::walls): the table as descriptors, defaults
/// from a default-constructed WallStageOptions.
const std::vector<ParameterDescriptor> &wall_stage_parameters();

/// Read the walls stage's JSON parameters ("" = defaults). Unknown keys are
/// ignored.
/// @throws std::invalid_argument for a non-object JSON, a value of the wrong
///         type, or one outside its bounds.
WallStageOptions wall_options_from_parameters(std::string_view parameters);

/// The JSON parameters (every key of the table) for @p options.
std::string wall_parameters_json(const WallStageOptions &options);

} // namespace reusex::pipeline
