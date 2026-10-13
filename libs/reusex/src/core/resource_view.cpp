// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_view.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace reusex::core {

std::string_view to_string(ViewKind kind) {
  return kind == ViewKind::panorama ? "panorama" : "frame";
}

std::optional<ViewKind> view_kind_from_string(std::string_view s) {
  if (s == "frame")
    return ViewKind::frame;
  if (s == "panorama")
    return ViewKind::panorama;
  return std::nullopt;
}

void validate_view_shape(const ResourceView &v) {
  if (v.id < 0)
    throw std::invalid_argument(
        fmt::format("view id must be >= 0, got {}", v.id));
  if (!v.region)
    return;
  const auto &r = *v.region;
  for (double c : r)
    if (!std::isfinite(c))
      throw std::invalid_argument("view region must hold finite numbers");
  const double x = r[0], y = r[1], w = r[2], h = r[3];
  if (!(w > 0.0) || !(h > 0.0))
    throw std::invalid_argument(
        fmt::format("view region needs w, h > 0, got w={} h={}", w, h));
  if (x < 0.0 || y < 0.0)
    throw std::invalid_argument(
        fmt::format("view region needs x, y >= 0, got x={} y={}", x, y));
  if (x >= 1.0)
    throw std::invalid_argument(
        fmt::format("view region needs x < 1, got {}", x));
  if (w > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(
        fmt::format("view region needs w <= 1, got {}", w));
  if (y + h > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(
        fmt::format("view region needs y + h <= 1, got {}", y + h));
  if (v.kind == ViewKind::frame && x + w > 1.0 + kViewRegionEpsilon)
    throw std::invalid_argument(fmt::format(
        "a frame view region needs x + w <= 1, got {} (only a panorama "
        "wraps across its seam)",
        x + w));
}

std::string view_to_json(const ResourceView &v) {
  nlohmann::ordered_json j;
  j["kind"] = std::string(to_string(v.kind));
  j["id"] = v.id;
  j["region"] = v.region ? nlohmann::ordered_json(*v.region)
                         : nlohmann::ordered_json(nullptr);
  return j.dump();
}

ResourceView parse_view(std::string_view text) {
  const auto j = nlohmann::json::parse(text.begin(), text.end(), nullptr,
                                       /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object())
    throw std::invalid_argument(
        "a view must be a JSON object {\"kind\", \"id\", \"region\"}");
  for (const auto &item : j.items())
    if (item.key() != "kind" && item.key() != "id" && item.key() != "region")
      throw std::invalid_argument("unknown view field '" + item.key() + "'");
  const auto kind = j.find("kind");
  const auto parsed_kind = kind != j.end() && kind->is_string()
                               ? view_kind_from_string(kind->get<std::string>())
                               : std::nullopt;
  if (!parsed_kind)
    throw std::invalid_argument(
        "view 'kind' must be \"frame\" or \"panorama\"");
  const auto id = j.find("id");
  if (id == j.end() || !id->is_number_integer())
    throw std::invalid_argument("view 'id' must be an integer");
  const bool too_big =
      id->is_number_unsigned()
          ? id->get<std::uint64_t>() >
                static_cast<std::uint64_t>(std::numeric_limits<int>::max())
          : id->get<std::int64_t>() > std::numeric_limits<int>::max() ||
                id->get<std::int64_t>() < std::numeric_limits<int>::min();
  if (too_big)
    throw std::invalid_argument("view 'id' is out of range");
  ResourceView v{*parsed_kind, static_cast<int>(id->get<std::int64_t>()),
                 std::nullopt};
  if (const auto r = j.find("region"); r != j.end() && !r->is_null()) {
    if (!r->is_array() || r->size() != 4)
      throw std::invalid_argument("view 'region' must be null or [x, y, w, h]");
    ViewRegion region{};
    for (std::size_t i = 0; i < 4; ++i) {
      if (!(*r)[i].is_number())
        throw std::invalid_argument("view 'region' must hold four numbers");
      region[i] = (*r)[i].get<double>();
    }
    v.region = region;
  }
  validate_view_shape(v);
  return v;
}

std::string view_display(const ResourceView &v) {
  return v.kind == ViewKind::frame ? fmt::format("Foto #{}", v.id)
                                   : fmt::format("360° #{}", v.id);
}

ViewRegion pad_region(const ViewRegion &r, double padding) {
  const double dx = r[2] * padding;
  const double dy = r[3] * padding;
  const double x0 = std::max(0.0, r[0] - dx);
  const double y0 = std::max(0.0, r[1] - dy);
  const double x1 = std::min(1.0, r[0] + r[2] + dx);
  const double y1 = std::min(1.0, r[1] + r[3] + dy);
  return {x0, y0, x1 - x0, y1 - y0};
}

} // namespace reusex::core
