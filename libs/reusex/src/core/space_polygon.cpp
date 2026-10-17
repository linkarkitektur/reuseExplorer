// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/space_polygon.hpp"

#include <bit>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace reusex::core {
namespace {

static_assert(std::endian::native == std::endian::little,
              "space polygon blobs are little-endian");

void put_u32(std::vector<uint8_t> &out, uint32_t v) {
  const auto *p = reinterpret_cast<const uint8_t *>(&v);
  out.insert(out.end(), p, p + sizeof v);
}

void put_f64(std::vector<uint8_t> &out, double v) {
  const auto *p = reinterpret_cast<const uint8_t *>(&v);
  out.insert(out.end(), p, p + sizeof v);
}

void check_ring(const std::vector<PlanPoint> &ring, const char *what) {
  if (ring.size() < 3)
    throw std::invalid_argument(std::string("space polygon: ") + what +
                                " has " + std::to_string(ring.size()) +
                                " point(s), needs at least 3");
  for (const auto &p : ring)
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]))
      throw std::invalid_argument(std::string("space polygon: ") + what +
                                  " has a coordinate that is not finite");
}

/// Twice the signed area and the first moments of one ring (shoelace).
struct RingMoments {
  double a2 = 0.0, cx6 = 0.0, cy6 = 0.0;
};

RingMoments moments(const std::vector<PlanPoint> &ring) {
  RingMoments m;
  for (size_t i = 0; i < ring.size(); ++i) {
    const auto &p = ring[i];
    const auto &q = ring[(i + 1) % ring.size()];
    const double c = p[0] * q[1] - q[0] * p[1];
    m.a2 += c;
    m.cx6 += (p[0] + q[0]) * c;
    m.cy6 += (p[1] + q[1]) * c;
  }
  // Normalise to counter-clockwise so outer and holes combine by sign.
  if (m.a2 < 0.0) {
    m.a2 = -m.a2;
    m.cx6 = -m.cx6;
    m.cy6 = -m.cy6;
  }
  return m;
}

} // namespace

std::vector<uint8_t> encode_space_polygon(const SpacePolygon &polygon) {
  check_ring(polygon.outer, "the outer ring");
  for (const auto &h : polygon.holes)
    check_ring(h, "a hole");
  std::vector<uint8_t> out;
  put_u32(out, static_cast<uint32_t>(1 + polygon.holes.size()));
  const auto ring = [&](const std::vector<PlanPoint> &r) {
    put_u32(out, static_cast<uint32_t>(r.size()));
    for (const auto &p : r) {
      put_f64(out, p[0]);
      put_f64(out, p[1]);
    }
  };
  ring(polygon.outer);
  for (const auto &h : polygon.holes)
    ring(h);
  return out;
}

SpacePolygon decode_space_polygon(const std::vector<uint8_t> &blob) {
  size_t at = 0;
  const auto need = [&](size_t n) {
    if (blob.size() - at < n)
      throw std::runtime_error("space polygon blob is truncated (" +
                               std::to_string(blob.size()) + " bytes)");
  };
  const auto u32 = [&] {
    need(4);
    uint32_t v;
    std::memcpy(&v, blob.data() + at, 4);
    at += 4;
    return v;
  };
  const auto f64 = [&] {
    need(8);
    double v;
    std::memcpy(&v, blob.data() + at, 8);
    at += 8;
    return v;
  };
  const uint32_t rings = u32();
  if (rings < 1)
    throw std::runtime_error("space polygon blob has no ring");
  SpacePolygon poly;
  for (uint32_t r = 0; r < rings; ++r) {
    const uint32_t n = u32();
    if (n < 3)
      throw std::runtime_error("space polygon blob has a ring of " +
                               std::to_string(n) + " point(s)");
    need(static_cast<size_t>(n) * 16);
    std::vector<PlanPoint> ring(n);
    for (auto &p : ring) {
      p[0] = f64();
      p[1] = f64();
    }
    if (r == 0)
      poly.outer = std::move(ring);
    else
      poly.holes.push_back(std::move(ring));
  }
  if (at != blob.size())
    throw std::runtime_error("space polygon blob has " +
                             std::to_string(blob.size() - at) +
                             " trailing byte(s)");
  return poly;
}

double space_polygon_area(const SpacePolygon &polygon) {
  double a2 = moments(polygon.outer).a2;
  for (const auto &h : polygon.holes)
    a2 -= moments(h).a2;
  return 0.5 * a2;
}

PlanPoint space_polygon_centroid(const SpacePolygon &polygon) {
  RingMoments m = moments(polygon.outer);
  for (const auto &h : polygon.holes) {
    const auto mh = moments(h);
    m.a2 -= mh.a2;
    m.cx6 -= mh.cx6;
    m.cy6 -= mh.cy6;
  }
  if (std::abs(m.a2) > 1e-12)
    return {m.cx6 / (3.0 * m.a2), m.cy6 / (3.0 * m.a2)};
  PlanPoint mean{0.0, 0.0};
  for (const auto &p : polygon.outer) {
    mean[0] += p[0];
    mean[1] += p[1];
  }
  if (!polygon.outer.empty()) {
    mean[0] /= static_cast<double>(polygon.outer.size());
    mean[1] /= static_cast<double>(polygon.outer.size());
  }
  return mean;
}

} // namespace reusex::core
