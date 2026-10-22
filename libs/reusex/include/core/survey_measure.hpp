// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// @file survey_measure.hpp
/// @brief Ground survey parts in the typed building model (reusex-uprf).
///
/// measure_survey() gives each survey part the geometry it stands for
/// (survey_parts.element_guid / geom_quantity / geom_unit / geom_source,
/// schema v37):
///
///  - an instance-backed part whose instance an opening was detected from
///    (openings.source_instance_guid) is that opening: its area
///    (s_end - s_start) × (z_head - z_sill), clipped to the wall's length
///    [opening_area, m²];
///  - every wall of the `walls` table and every slab piece of the
///    `solid_slabs` index (the `solids` stage) is an element-backed part,
///    created on the first run in a type per element category (Ydervæg /
///    Indervæg from walls.is_external, Etagedæk, Tag): a wall's net area is
///    length × height − its openings (height from its solid, which follows
///    the spaces beside it, else walls.height) [wall_area, m²]; a floor
///    slab's volume is its solid's volume [solid_volume, m³]; a roof
///    slab's area is its volume / thickness [slab_area, m²], because roof
///    build-ups are per m²;
///  - any other instance-backed part is one counted item [count, stk].
///
/// A part's quantity is seeded from its measured quantity when the units
/// agree and no user entered it (value_provenance kind `entered`), and
/// marked `measured`. An element-backed type without a build-up gets one
/// suggested from its category (kind `suggested`): for walls, the candidate
/// whose layered thickness is closest to the median *measured* thickness
/// of the type's walls (walls.assumed = 0), else the category's first
/// build-up; an empty EAK takes the build-up's main layer code.
///
/// Idempotent: a re-run refreshes the geometry, never duplicates a part,
/// never re-creates a deleted element part (its guid is tombstoned like an
/// instance's, ProjectDB::dismiss_instance) and never touches an entered
/// quantity or a chosen build-up. Needs schema v37.

#include "reusex/core/ProjectDB.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace reusex::core {

struct SurveyMeasureOptions {
  /// Create element-backed parts for walls and slab pieces without one.
  bool seed_elements = true;
  /// Suggest a build-up (and EAK) for element types that have none.
  bool suggest_buildups = true;
  /// Meshes holding the solids index (the `solids` stage's output).
  std::string walls_mesh = "solid_walls";
  std::string slabs_mesh = "solid_slabs";
};

struct SurveyMeasureReport {
  std::size_t parts_seen = 0;
  /// Parts with a measured quantity after the run, by source.
  std::size_t opening_parts = 0, wall_parts = 0, slab_parts = 0, roof_parts = 0,
              count_parts = 0;
  std::size_t types_created = 0;
  std::size_t parts_created = 0;
  /// Quantities set from the measurement (not entered by a user).
  std::size_t quantities_seeded = 0;
  /// Element-backed parts whose element is gone (walls / solids re-run
  /// without it): geometry cleared, part kept.
  std::size_t elements_missing = 0;
  std::size_t buildups_suggested = 0;
  /// Walls whose thickness was measured (not assumed).
  std::size_t walls_with_thickness = 0;
  /// Instance-backed parts with no geometry beyond the count, manual parts.
  std::size_t parts_without_geometry = 0;
};

/// See the file comment. Runs in one transaction.
/// @throws std::runtime_error below schema v37 or on a read-only project.
SurveyMeasureReport measure_survey(ProjectDB &db,
                                   const SurveyMeasureOptions &opts = {});

/// Display name, BIM7AA group and unit of the type an element category
/// seeds: Ydervæg 21 m², Indervæg 22 m², Etagedæk 23 m³, Tag 27 m².
struct ElementTypeSeed {
  std::string name, bim7aa_code, unit;
};
ElementTypeSeed element_type_seed(std::string_view category);

} // namespace reusex::core
