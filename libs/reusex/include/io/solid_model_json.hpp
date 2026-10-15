// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The solid-model JSON document (docs/design/solid-model-json.md, schema
// docs/design/solid-model-reference.schema.json).
//
// One format for both sides of the benchmark: a model detected by the
// pipeline (`load_solid_model`) and a reference plan authored by a person or
// an agent are the same document, so `rux analyze solid` compares like with
// like.

#include <reusex/core/solid_model.hpp>

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
} // namespace reusex

namespace reusex::io {

/// The `model_version` this build reads and writes.
inline constexpr int kSolidModelJsonVersion = 1;

/// Serialise a model. Keys come out in a fixed order and every element array
/// the model provides (`SolidModel::has_*`) is written, so the output is
/// deterministic and a "provided but empty" kind survives a round trip.
///
/// @throws std::invalid_argument when a record's `metadata` is not JSON text
///         (the reader always stores JSON text there).
std::string solid_model_to_json(const core::SolidModel &model, int indent = 2);

/// Parse a document.
///
/// Strict: an unknown key, a wrong type, a non-finite number, an integer
/// outside `int`, a dangling reference (an opening whose `wall` names no
/// wall), a duplicate guid / storey idx, a zero-length wall axis, and a space
/// polygon that is not a valid polygon with holes (a ring with < 3 points,
/// self-intersecting or of zero area; a hole not strictly inside the outer
/// ring, or overlapping another hole) all throw. A ring whose last point
/// repeats its first is accepted and stored open (the point dropped).
///
/// @throws std::invalid_argument naming the offending element, e.g.
///         "walls[2] (guid 'w3'): axis has zero length".
core::SolidModel solid_model_from_json(std::string_view text);

/// Read and parse a document file (solid_model_from_json).
///
/// @throws std::runtime_error naming the path when the file cannot be read.
/// @throws std::invalid_argument when its content is not a valid document.
core::SolidModel solid_model_from_file(const std::filesystem::path &path);

/// Write @p model to @p path (solid_model_to_json, plus a final newline),
/// creating parent directories.
///
/// @throws std::runtime_error naming the path when it cannot be written.
void solid_model_to_file(const core::SolidModel &model,
                         const std::filesystem::path &path, int indent = 2);

/// Mark every storey, wall, space and opening as `source` "imported",
/// `locked` — what an authored reference becomes when it is imported into a
/// project, so no re-run of a detecting stage overwrites it.
void mark_imported(core::SolidModel &model);

/// What store_solid_model() wrote.
struct SolidModelStoreResult {
  std::size_t storeys = 0, walls = 0, spaces = 0, openings = 0; ///< rows
  /// Element kinds the model provides (SolidModel::has_*) that were not
  /// stored because the project has no table for them yet, in the order
  /// storeys, walls, spaces, openings.
  std::vector<std::string> not_stored;

  [[nodiscard]] std::size_t stored() const {
    return storeys + walls + spaces + openings;
  }
};

/// Write @p model's elements into the project's solid-model tables, the
/// inverse of load_solid_model() (`rux edit plan --import`).
///
/// Each kind is written where its table exists; at WP0 none does, so every
/// provided kind lands in SolidModelStoreResult::not_stored and nothing is
/// written. WP1 (storeys), WP3 (walls), WP4 (spaces) and WP5 (openings) each
/// add their part here, next to their part of load_solid_model(), so that
/// `JSON -> tables -> JSON` stays lossless.
SolidModelStoreResult store_solid_model(ProjectDB &db,
                                        const core::SolidModel &model);

/// The typed building model stored in a project.
///
/// Fills each element kind from its ProjectDB table as the table exists. At
/// WP0 no solid-model table exists yet, so this returns an empty model with
/// nothing provided (every benchmark metric reports n/a); WP1 (storeys), WP3
/// (walls), WP4 (spaces) and WP5 (openings) each add their part here.
core::SolidModel load_solid_model(const ProjectDB &db);

} // namespace reusex::io
