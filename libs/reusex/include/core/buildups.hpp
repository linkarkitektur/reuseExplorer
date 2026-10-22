// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file buildups.hpp
/// @brief The Opbygning (build-up) library: typical Danish wall, slab and
/// roof build-ups that turn a survey quantity into tonnes (reusex-uprf).
///
/// The default library is `core/buildups.json` (embedded at build time,
/// moved from rux-frontend's `src/bygning/buildups.ts`); a project overrides
/// or adds entries by id in its `buildups` table (schema v37,
/// ProjectDB::project_buildups). A survey type names its build-up by id
/// (survey_types.buildup_id).
///
/// Mass rule (one layer, as the frontend computed it): the *main* layer gives
/// a type's tonnes — `quantity × kg_m2` when the layer has a mass per area,
/// `quantity × rho` when the quantity is a volume (m³), else
/// `quantity × t × rho` (an area, m²). The other layers do not add to the
/// type: each can become a type of its own ("Opret som egen type"), and
/// counting them here too would count them twice.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

/// What a build-up is for; also a survey type's element category
/// (survey_types.element_category, schema v37).
enum class BuildupCategory { ydervaeg, indervaeg, daek, tag };
std::string_view to_string(BuildupCategory c) noexcept;
std::optional<BuildupCategory> buildup_category_from_string(std::string_view);

/// Survey units, as the GUI writes them.
inline constexpr std::string_view kUnitCount = "stk";
inline constexpr std::string_view kUnitArea = "m²";
inline constexpr std::string_view kUnitVolume = "m³";

struct BuildupLayer {
  std::string material;
  /// Thickness [m]; >= 1 reads as "massiv" (a slab counted in m³).
  std::optional<double> t;
  /// Density [kg/m³].
  std::optional<double> rho;
  /// Mass per area [kg/m²] (instead of t × rho).
  std::optional<double> kg_m2;
  std::string eak;
  /// The type name a split-off layer gets ("" for the main layer).
  std::string name;
};

struct Buildup {
  std::string id;
  BuildupCategory category = BuildupCategory::ydervaeg;
  std::string name;
  BuildupLayer main;
  std::vector<BuildupLayer> others;
  std::string note;
  /// "library" (core/buildups.json) or "project" (the buildups table).
  std::string origin = "library";
};

/// Parse one build-up object (the format of an entry of buildups.json).
/// @throws std::invalid_argument on malformed JSON, a missing id / name /
///         category / main layer, an unknown category, a layer with neither
///         kg_m2 nor rho, or a negative number.
Buildup parse_buildup(std::string_view json_text);
/// Parse a whole library file ({"version": 1, "buildups": [...]}).
/// @throws std::invalid_argument as parse_buildup, plus a duplicate id or a
///         version other than 1.
std::vector<Buildup> parse_buildup_library(std::string_view json_text);
/// One build-up as the JSON object parse_buildup reads (origin left out).
std::string to_json(const Buildup &b);

/// The embedded default library (core/buildups.json), in file order.
const std::vector<Buildup> &default_buildups();
/// The project's library: the defaults with the project's `buildups` rows
/// replacing an entry of the same id, then the project-only ids ascending.
/// A project row that fails to parse is skipped (logged at warn).
std::vector<Buildup> buildup_library(const ProjectDB &db);
/// By id in @p library; nullopt when absent.
std::optional<Buildup> find_buildup(const std::vector<Buildup> &library,
                                    std::string_view id);
/// By exact name (blanks trimmed) in @p library; the frontend's
/// "Opbygning" user column stored names.
std::optional<Buildup> find_buildup_by_name(const std::vector<Buildup> &library,
                                            std::string_view name);

/// Tonnes of one layer for @p quantity in @p unit (m² or m³; anything else
/// is read as m²). 0 when the layer lacks the numbers.
double layer_tonnes(const BuildupLayer &layer, double quantity,
                    std::string_view unit);
/// Σ t of the layers measured by thickness (t < 1, no kg_m2) [m]; 0 when
/// none is.
double layered_thickness(const Buildup &b);

/// The category of a type by its name, then its BIM7AA group (21 ydervægge,
/// 22 indervægge, 23 dæk, 27 tage) — the frontend's categoryOf().
std::optional<BuildupCategory> category_of(std::string_view type_name,
                                           std::string_view bim7aa_code);

} // namespace reusex::core
