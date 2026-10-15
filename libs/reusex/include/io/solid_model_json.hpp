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

#include <string>
#include <string_view>

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

/// The typed building model stored in a project.
///
/// Fills each element kind from its ProjectDB table as the table exists. At
/// WP0 no solid-model table exists yet, so this returns an empty model with
/// nothing provided (every benchmark metric reports n/a); WP1 (storeys), WP3
/// (walls), WP4 (spaces) and WP5 (openings) each add their part here.
core::SolidModel load_solid_model(const ProjectDB &db);

} // namespace reusex::io
