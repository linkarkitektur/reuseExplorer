// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A small JSON-Schema (2020-12) validator for the keywords the repo's
// document schemas use (docs/design/*.schema.json): type, const, enum,
// required, properties, additionalProperties (false or a schema), items,
// minItems, maxItems, uniqueItems, minLength, pattern, minimum,
// exclusiveMinimum, oneOf, anyOf and local $ref ("#/$defs/<name>"). No
// validator ships in the dev shell; an unsupported keyword fails loudly
// instead of being ignored, so a schema cannot outgrow this silently.

#include <nlohmann/json.hpp>

#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::test_support {

class JsonSchemaLite {
    public:
  explicit JsonSchemaLite(nlohmann::json root) : root_(std::move(root)) {}

  /// Empty when @p doc is valid; otherwise the first error, with its path.
  std::string validate(const nlohmann::json &doc) const {
    std::string err;
    check(root_, doc, "$", err);
    return err;
  }

    private:
  using json = nlohmann::json;

  const json &resolve(const std::string &ref) const {
    const std::string prefix = "#/$defs/";
    if (ref.rfind(prefix, 0) != 0)
      throw std::logic_error("JsonSchemaLite: unsupported $ref " + ref);
    return root_.at("$defs").at(ref.substr(prefix.size()));
  }

  static bool is_type(const json &v, const std::string &t) {
    if (t == "object")
      return v.is_object();
    if (t == "array")
      return v.is_array();
    if (t == "string")
      return v.is_string();
    if (t == "integer")
      return v.is_number_integer();
    if (t == "number")
      return v.is_number();
    if (t == "boolean")
      return v.is_boolean();
    if (t == "null")
      return v.is_null();
    throw std::logic_error("JsonSchemaLite: unknown type " + t);
  }

  bool check(const json &s, const json &v, const std::string &path,
             std::string &err) const {
    static const std::set<std::string> known = {
        "$schema",   "$id",      "title",      "description",
        "$defs",     "$ref",     "type",       "const",
        "enum",      "required", "properties", "additionalProperties",
        "items",     "minItems", "maxItems",   "uniqueItems",
        "minLength", "pattern",  "minimum",    "exclusiveMinimum",
        "oneOf",     "anyOf"};
    for (const auto &[k, _] : s.items())
      if (!known.count(k))
        throw std::logic_error("JsonSchemaLite: unsupported keyword " + k);
    auto fail = [&](const std::string &what) {
      if (err.empty())
        err = path + ": " + what;
      return false;
    };
    if (s.contains("$ref") && !check(resolve(s.at("$ref")), v, path, err))
      return false;
    if (s.contains("type") && !is_type(v, s.at("type")))
      return fail("not of type " + s.at("type").get<std::string>());
    if (s.contains("const") && v != s.at("const"))
      return fail("not " + s.at("const").dump());
    if (s.contains("enum")) {
      bool any = false;
      for (const auto &e : s.at("enum"))
        any = any || e == v;
      if (!any)
        return fail("not one of " + s.at("enum").dump());
    }
    if (s.contains("oneOf") || s.contains("anyOf")) {
      const bool one = s.contains("oneOf");
      int matches = 0;
      for (const auto &sub : s.at(one ? "oneOf" : "anyOf")) {
        std::string sub_err;
        if (check(sub, v, path, sub_err))
          ++matches;
      }
      if (one ? matches != 1 : matches == 0)
        return fail(std::to_string(matches) + " of the " +
                    (one ? "oneOf" : "anyOf") + " alternatives match");
    }
    if (v.is_object()) {
      if (s.contains("required"))
        for (const auto &r : s.at("required"))
          if (!v.contains(r.get<std::string>()))
            return fail("missing " + r.get<std::string>());
      const json props = s.value("properties", json::object());
      for (const auto &[k, val] : v.items()) {
        if (props.contains(k)) {
          if (!check(props.at(k), val, path + "." + k, err))
            return false;
        } else if (s.contains("additionalProperties")) {
          const json &ap = s.at("additionalProperties");
          if (ap.is_boolean() && !ap.get<bool>())
            return fail("unknown key " + k);
          if (ap.is_object() && !check(ap, val, path + "." + k, err))
            return false;
        }
      }
    }
    if (v.is_array()) {
      if (s.contains("minItems") && v.size() < s.at("minItems").get<size_t>())
        return fail("too few items");
      if (s.contains("maxItems") && v.size() > s.at("maxItems").get<size_t>())
        return fail("too many items");
      if (s.value("uniqueItems", false)) {
        std::set<std::string> seen;
        for (const auto &e : v)
          if (!seen.insert(e.dump()).second)
            return fail("duplicate item " + e.dump());
      }
      if (s.contains("items"))
        for (size_t i = 0; i < v.size(); ++i)
          if (!check(s.at("items"), v[i], path + "[" + std::to_string(i) + "]",
                     err))
            return false;
    }
    if (v.is_string()) {
      const auto str = v.get<std::string>();
      if (s.contains("minLength") &&
          str.size() < s.at("minLength").get<size_t>())
        return fail("too short");
      if (s.contains("pattern") &&
          !std::regex_search(str,
                             std::regex(s.at("pattern").get<std::string>())))
        return fail("does not match " + s.at("pattern").get<std::string>());
    }
    if (v.is_number()) {
      const double d = v.get<double>();
      if (s.contains("minimum") && d < s.at("minimum").get<double>())
        return fail("below minimum");
      if (s.contains("exclusiveMinimum") &&
          d <= s.at("exclusiveMinimum").get<double>())
        return fail("not above exclusiveMinimum");
    }
    return true;
  }

  json root_;
};

} // namespace reusex::test_support
