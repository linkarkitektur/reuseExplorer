// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>
#include <string_view>

namespace reusex::core {

/// Generate a fresh UUID-v4-like identifier (36-char, hyphenated hex).
///
/// Used for stable entity identity (material passports, building components,
/// instances). Non-deterministic by design: each call yields a new value from
/// a random_device-seeded generator. Do not use where reproducibility across
/// runs is required.
std::string generate_guid();

/// A name-based identifier: RFC 4122 version 5 (SHA-1) of @p name in the
/// namespace @p ns_guid, lowercase and hyphenated.
///
/// The stable GUID of a solid-model row (storeys, walls, ...): the same
/// project namespace (`ProjectDB::project_guid()`) and the same name (e.g.
/// "storeys/0") give the same GUID on every run, so references and manual
/// edits survive a re-run (reusex-z1v, determinism, STANDARDS §6).
///
/// @throws std::invalid_argument when @p ns_guid is not a UUID.
std::string deterministic_guid(std::string_view ns_guid, std::string_view name);

} // namespace reusex::core
