// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "yaml_json.hpp"

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <charconv>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace reusex::io::detail {
namespace {

using json = nlohmann::json;
using ojson = nlohmann::ordered_json;

// ── YAML front-end (reusex-yf4.1) ───────────────────────────────────────────
//
// `solid_model_from_yaml` converts a parsed YAML tree into a `nlohmann::json`
// value and hands it to the same strict validator `solid_model_from_json`
// uses, so there is exactly one reader. `solid_model_to_yaml` walks the same
// ordered document `solid_model_to_json` builds and emits it block-style, so
// both front ends agree on key order.
//
// Typing a *plain* (unquoted) YAML scalar: yaml-cpp itself already resolves
// the null spellings it recognises (`null`, `Null`, `NULL`, `~`, empty) to a
// null node before we ever see the scalar text. Everything else is ours to
// decide, deliberately narrower than full YAML 1.1: exactly `true` / `false`
// are booleans, a scalar that parses in full as a decimal integer or
// floating-point number is a number, anything else (including `yes`, `on`,
// `True`) is a string. A scalar written with quotes in the source
// (`n.Tag() == "!"`) is always a string, whatever it looks like.
//
// `plain_scalar_would_retype` is the same decision run in reverse: it tells
// the writer whether a JSON *string* value would come back as something else
// if written unquoted (e.g. a guid that happens to read "123", or a name
// that happens to read "true"), so those get forced double-quoted on write
// and everything else is left to yaml-cpp's own quoting heuristics.

/// A full-string decimal integer (optional leading '+' or '-'); anything
/// left over (a trailing '.', an exponent, non-digits) is not an integer.
std::optional<std::int64_t> parse_plain_int(const std::string &s) {
  std::string_view v = s;
  if (!v.empty() && v.front() == '+')
    v.remove_prefix(1);
  if (v.empty())
    return std::nullopt;
  std::int64_t val = 0;
  const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), val);
  if (ec != std::errc() || ptr != v.data() + v.size())
    return std::nullopt;
  return val;
}

/// A full-string floating-point number (optional leading '+'); accepts
/// anything `std::from_chars` does (decimal, exponent, `inf`, `nan`), as long
/// as the whole scalar is consumed.
std::optional<double> parse_plain_float(const std::string &s) {
  std::string_view v = s;
  if (!v.empty() && v.front() == '+')
    v.remove_prefix(1);
  if (v.empty())
    return std::nullopt;
  double val = 0;
  const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), val);
  if (ec != std::errc() || ptr != v.data() + v.size())
    return std::nullopt;
  return val;
}

/// Would an unquoted plain scalar holding exactly @p s be read back as
/// something other than the string @p s? Mirrors the reader's resolution
/// (plus the null spellings yaml-cpp's own scanner resolves before we see the
/// scalar at all), so this is the only place either direction can disagree.
bool plain_scalar_would_retype(const std::string &s) {
  if (s.empty() || s == "~" || s == "null" || s == "Null" || s == "NULL")
    return true;
  if (s == "true" || s == "false")
    return true;
  return parse_plain_int(s).has_value() || parse_plain_float(s).has_value();
}

json yaml_scalar_to_json(const YAML::Node &n) {
  if (n.IsNull())
    return nullptr;
  const std::string &s = n.Scalar();
  if (n.Tag() != "!") { // a plain (unquoted) scalar: resolve its type
    if (s == "true")
      return true;
    if (s == "false")
      return false;
    if (const auto i = parse_plain_int(s))
      return *i;
    if (const auto d = parse_plain_float(s))
      return *d;
  }
  return s;
}

json yaml_node_to_json(const YAML::Node &n, const std::string &where) {
  if (!n.IsDefined() || n.IsNull())
    return nullptr;
  if (n.IsScalar())
    return yaml_scalar_to_json(n);
  if (n.IsSequence()) {
    json arr = json::array();
    for (const auto &e : n)
      arr.push_back(yaml_node_to_json(e, where));
    return arr;
  }
  if (n.IsMap()) {
    json obj = json::object();
    for (const auto &kv : n)
      obj[kv.first.as<std::string>()] = yaml_node_to_json(kv.second, where);
    return obj;
  }
  throw std::invalid_argument(
      "unsupported YAML node (expected a scalar, sequence or map)");
}

void emit_yaml(const ojson &j, YAML::Emitter &out) {
  if (j.is_object()) {
    out << YAML::BeginMap;
    for (auto it = j.begin(); it != j.end(); ++it) {
      out << YAML::Key << it.key() << YAML::Value;
      emit_yaml(it.value(), out);
    }
    out << YAML::EndMap;
  } else if (j.is_array()) {
    out << YAML::BeginSeq;
    for (const auto &e : j)
      emit_yaml(e, out);
    out << YAML::EndSeq;
  } else if (j.is_null()) {
    out << YAML::Null;
  } else if (j.is_string()) {
    const std::string &s = j.get_ref<const std::string &>();
    if (plain_scalar_would_retype(s))
      out << YAML::DoubleQuoted;
    out << s;
  } else {
    // bool / integer / float: nlohmann's own dump() of a primitive is its
    // literal text ("true", "123", "2.6", ...), which is exactly the plain
    // scalar we want; none of these ever needs quoting.
    out << j.dump();
  }
}

} // namespace

json yaml_text_to_json(std::string_view text) {
  YAML::Node root;
  try {
    root = YAML::Load(std::string(text));
    return yaml_node_to_json(root, "document");
  } catch (const YAML::Exception &e) {
    throw std::invalid_argument(fmt::format("not valid YAML ({})", e.what()));
  }
}

std::string json_to_yaml_text(const ojson &doc) {
  YAML::Emitter out;
  out.SetDoublePrecision(17);
  emit_yaml(doc, out);
  return std::string(out.c_str()) + "\n";
}

} // namespace reusex::io::detail
