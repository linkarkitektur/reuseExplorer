// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// YAML <-> nlohmann::json, shared by the io module's hand-authored document
// front ends (the solid-model plan, reusex-yf4.1; the pose edits,
// reusex-rcv.1.2). PRIVATE to reusex_io: yaml-cpp is a PRIVATE dependency,
// so nothing here may leak into a public header.
//
// A YAML document is converted to a `nlohmann::json` value and then read by
// the same strict validator as the JSON text, so each format has exactly one
// reader. Typing of a *plain* (unquoted) scalar is deliberately narrower than
// full YAML 1.1: the null spellings yaml-cpp resolves (`null`, `~`, empty)
// are null, exactly `true` / `false` are booleans, a scalar that parses in
// full as a decimal integer or a floating-point number (`from_chars`, so
// `nan` / `inf` too) is a number, and anything else is a string. A quoted
// scalar is always a string.

#include <nlohmann/json.hpp>

#include <string>
#include <string_view>

namespace reusex::io::detail {

/// Parse YAML @p text into a JSON value (comments are dropped).
/// @throws std::invalid_argument "not valid YAML (...)" or "unsupported YAML
///         node (...)"; callers prefix it with the element they read.
nlohmann::json yaml_text_to_json(std::string_view text);

/// Block-style YAML of @p doc in its key order, doubles written with 17
/// significant digits, ending in a newline. A string that would read back as
/// another type unquoted ("123", "true", "") is written double-quoted.
std::string json_to_yaml_text(const nlohmann::ordered_json &doc);

} // namespace reusex::io::detail
