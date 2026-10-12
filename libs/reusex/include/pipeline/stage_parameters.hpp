// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Machine-readable description of the parameters each runnable stage accepts
// (#305).
//
// `run_stage()` reads a job's `parameters` JSON object with `param_or(params,
// "<key>", <option-struct field>)`, so the *authority* on what a knob is called
// and what it defaults to has always been the library option struct. Until now
// that authority was unreadable from outside: a front end wanting to render a
// parameter form had no choice but to hard-code the keys and re-type the
// defaults, which is exactly the duplication STANDARDS §4 forbids.
//
// This table closes that hole. It is built by *default-constructing the option
// struct and reading its fields* — never by copying literals — so a default
// that moves in `segmentation/` moves here, and in every UI, without anyone
// remembering to. `tests/unit/pipeline/test_stage_parameters.cpp` asserts that
// correspondence field by field.
//
// Layering: this lives in `pipeline` (Layer 4) because that is where the
// option structs are already assembled from JSON; `core` could not reach
// `segmentation` to read the defaults.

#include <reusex/pipeline/stages.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace reusex::pipeline {

/// Wire type of a stage parameter, i.e. what JSON a caller should send.
enum class ParameterType {
  number,       ///< JSON number, fractional (float in the option struct)
  integer,      ///< JSON number, integral
  boolean,      ///< JSON true/false
  string,       ///< JSON string
  integer_list, ///< JSON array of non-negative integers
  string_list,  ///< JSON array of strings
};

/// Canonical lower-case token for @p type, as it appears on the wire.
std::string_view to_string(ParameterType type);

/// The default a parameter takes when the key is absent from the job's
/// `parameters` object. `monostate` means "no scalar default" — the parameter
/// is absent-by-default and has no neutral value to pre-fill a form with
/// (`planes.filter`, `instances.labels`).
using ParameterDefault =
    std::variant<std::monostate, double, long long, bool, std::string>;

/// One knob of one stage.
struct ParameterDescriptor {
  /// The JSON key `run_stage()` reads. This is the contract; the CLI flag
  /// spelling may differ (`clouds` reads `resolution`, the CLI calls it
  /// `-g/--grid`).
  std::string key;
  ParameterType type = ParameterType::number;
  /// Short human label for a form field.
  std::string label;
  /// One-line explanation, mirroring the CLI help for the equivalent flag.
  std::string description;
  ParameterDefault default_value;
  /// Inclusive bounds, mirroring the CLI's `CLI::Range` checks where one
  /// exists. Absent means the algorithm imposes no documented bound.
  std::optional<double> minimum;
  std::optional<double> maximum;
  /// True when the mere PRESENCE of the key changes behaviour, independently
  /// of its value (#214): sending `plane_dist_threshold` at all pins that
  /// threshold and switches off adaptive derivation *for it*. A form must
  /// therefore omit an untouched key rather than helpfully echoing its
  /// default back — that would silently disable adaptivity.
  bool presence_sensitive = false;
};

/// Every parameter @p stage recognises, in form-display order.
const std::vector<ParameterDescriptor> &stage_parameters(JobStage stage);

/// Name of the semantic-label cloud `instances` reads when the job does not
/// say otherwise. Shared with `run_instances()` so the descriptor's default
/// and the runner's fallback cannot drift apart.
inline constexpr std::string_view kDefaultSemanticCloud = "labels";
/// Name of the cloud `instances` writes when the job does not say otherwise.
inline constexpr std::string_view kDefaultInstanceCloud = "instances";

} // namespace reusex::pipeline
