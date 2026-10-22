// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file survey_mass.hpp
/// @brief Survey tonnes from build-ups and quantities, and where each value
/// came from (reusex-uprf).
///
/// A part's mass is its type's build-up (core/buildups.hpp, main layer) over
/// the part's quantity in the type's unit:
///
///  - m², layer with kg_m2: quantity × kg_m2;
///  - m², layer with t × rho: quantity × t × rho — except an element-backed
///    wall with a *measured* thickness (walls.assumed = 0, thickness > 0)
///    under a one-layer build-up, which uses quantity × measured thickness
///    × rho: the scan measured the whole wall, and a one-layer build-up
///    says the whole wall is that material. Under a multi-layer build-up
///    the layers' thicknesses are kept (the scan cannot tell them apart),
///    and a measured thickness more than 25 % off their sum is a warning;
///  - m³: quantity × rho for a massive layer (t >= 1 or no t), else
///    quantity × rho × t / layered thickness (the main layer's share of the
///    measured volume);
///  - stk, another unit, no build-up, or an unknown build-up id: no mass.
///
/// A type's calculated mass is the sum over its counted (non-dismissed)
/// parts; none when the type has no counted part or no part could be
/// massed. survey_types.mass_t is the user's override: the effective mass is
/// the override when set, else the calculated mass.

#include "reusex/core/ProjectDB.hpp"
#include "reusex/core/buildups.hpp"
#include "reusex/core/survey.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

struct PartMass {
  std::string code;
  std::optional<double> mass_t;
  /// "areal × tykkelse × densitet" | "areal × målt tykkelse × densitet" |
  /// "areal × kg/m²" | "volumen × densitet" | "volumen × lagandel ×
  /// densitet" | "" (none).
  std::string method;
  bool dismissed = false;
};

struct TypeMass {
  int64_t type_id = 0;
  std::optional<double> effective_t, calc_t, override_t;
  /// "override" | "calculated" | "none".
  std::string source = "none";
  /// How calc_t was computed (the parts' method; "blandet" when they
  /// differ), "" without a calculation.
  std::string method;
  std::optional<std::string> buildup_id;
  std::size_t parts_counted = 0;
  std::vector<PartMass> parts; ///< every part of the type, by code
  /// Why there is no calculation, or what to check (unknown build-up,
  /// a thickness that disagrees with the build-up, ...).
  std::vector<std::string> warnings;
};

/// One TypeMass per survey type, by type id.
std::map<int64_t, TypeMass> survey_masses(const ProjectDB &db);
/// @throws std::out_of_range for an unknown type.
TypeMass type_mass(const ProjectDB &db, int64_t type_id);

/// The stored provenance rows of a type, with its `mass_t` row derived
/// from @p mass: `calculated` (source "buildup:<id>") when no override is
/// set, the stored row (else `entered`, source "user") when one is, none
/// without a mass.
std::vector<ProjectDB::ValueProvenanceRecord>
type_provenance(const ProjectDB &db, const TypeMass &mass);
/// The stored provenance rows of a part.
std::vector<ProjectDB::ValueProvenanceRecord>
part_provenance(const ProjectDB &db, std::string_view code);

/// User edits that record `entered` provenance (schema v37; a no-op on
/// provenance below it). Every engaged field of the patch is marked
/// entered with @p source; clearing mass_t removes its row (the calculation
/// takes over), clearing buildup_id is recorded as entered (so
/// measure_survey does not suggest one again). Runs in one transaction.
ProjectDB::SurveyTypeRecord
update_survey_type_entered(ProjectDB &db, int64_t id,
                           const ProjectDB::SurveyTypePatch &patch,
                           std::string_view source = "user");
ProjectDB::SurveyPartRecord
update_survey_part_entered(ProjectDB &db, std::string_view code,
                           const ProjectDB::SurveyPartPatch &patch,
                           std::string_view source = "user");
/// The provenance half of the two above, for a caller that already applied
/// the patch inside its own transaction (core::patch_resource).
void record_entered(ProjectDB &db, int64_t type_id,
                    const ProjectDB::SurveyTypePatch &patch,
                    std::string_view source = "user");
void record_entered(ProjectDB &db, std::string_view code,
                    const ProjectDB::SurveyPartPatch &patch,
                    std::string_view source = "user");

} // namespace reusex::core
