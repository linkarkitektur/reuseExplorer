// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// core::deterministic_guid: RFC 4122 name-based (v5, SHA-1) identifiers, the
// stable GUIDs of the solid-model rows (reusex-z1v.3.2).

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/guid.hpp>

#include <stdexcept>
#include <string>

using reusex::core::deterministic_guid;
using reusex::core::generate_guid;

TEST_CASE("DeterministicGuid_MatchesRfc4122Version5", "[core][guid]") {
  // Python: uuid.uuid5(uuid.NAMESPACE_DNS, "python.org").
  CHECK(deterministic_guid("6ba7b810-9dad-11d1-80b4-00c04fd430c8",
                           "python.org") ==
        "886313e1-3b8a-5372-9b90-0c9aee199e5d");
  // uuid.uuid5(UUID("12345678-1234-5678-1234-567812345678"), "storeys/0").
  CHECK(
      deterministic_guid("12345678-1234-5678-1234-567812345678", "storeys/0") ==
      "3d7ee2a1-b570-5023-9f82-49e6d0465e6c");
}

TEST_CASE("DeterministicGuid_IsStableAndNameSensitive", "[core][guid]") {
  const std::string ns = generate_guid();
  const auto a = deterministic_guid(ns, "storeys/0");
  CHECK(a == deterministic_guid(ns, "storeys/0"));
  CHECK(a != deterministic_guid(ns, "storeys/1"));
  CHECK(a != deterministic_guid(generate_guid(), "storeys/0"));
  REQUIRE(a.size() == 36);
  CHECK(a[14] == '5'); // version nibble
}

TEST_CASE("DeterministicGuid_AcceptsUpperCaseNamespace", "[core][guid]") {
  CHECK(deterministic_guid("6BA7B810-9DAD-11D1-80B4-00C04FD430C8",
                           "python.org") ==
        "886313e1-3b8a-5372-9b90-0c9aee199e5d");
}

TEST_CASE("DeterministicGuid_RejectsMalformedNamespace", "[core][guid]") {
  CHECK_THROWS_AS(deterministic_guid("", "storeys/0"), std::invalid_argument);
  CHECK_THROWS_AS(deterministic_guid("not-a-guid", "storeys/0"),
                  std::invalid_argument);
}
