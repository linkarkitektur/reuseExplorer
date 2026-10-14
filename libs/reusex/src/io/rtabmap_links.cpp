// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/rtabmap_links.hpp"

#include "core/pose_graph.hpp"

#include <string>

namespace reusex::io {

namespace pg = core::pose_graph;

std::array<double, 16> rtabmap_transform_to_array(const float *d) {
  std::array<double, 16> a{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      a[static_cast<size_t>(r * 4 + c)] = static_cast<double>(d[r * 4 + c]);
  a[15] = 1.0;
  return a;
}

std::array<double, 36>
rtabmap_information_to_gtsam(const std::array<double, 36> &in) {
  // GTSAM index k reads RTABMap index p[k].
  constexpr int p[6] = {3, 4, 5, 0, 1, 2};
  std::array<double, 36> out{};
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      out[static_cast<size_t>(r * 6 + c)] =
          in[static_cast<size_t>(p[r] * 6 + p[c])];
  return out;
}

RtabmapLinkConversion
convert_rtabmap_link(const RtabmapLink &link, int id_offset,
                     const std::map<int, std::array<double, 16>> &imported) {
  RtabmapLinkConversion out;
  std::string type;
  switch (link.type) {
  case rtabmap_link_type::neighbor:
  case rtabmap_link_type::neighbor_merged:
    type = std::string(pg::kTypeOdometry);
    break;
  case rtabmap_link_type::global_closure:
  case rtabmap_link_type::local_space_closure:
  case rtabmap_link_type::local_time_closure:
  case rtabmap_link_type::user_closure:
    type = std::string(pg::kTypeLoopClosure);
    break;
  default:
    out.skip = RtabmapLinkSkip::unsupported_type;
    return out;
  }
  if (link.from == link.to) {
    out.skip = RtabmapLinkSkip::self_link;
    return out;
  }
  const auto a = imported.find(link.from);
  const auto b = imported.find(link.to);
  if (a == imported.end() || b == imported.end()) {
    out.skip = RtabmapLinkSkip::endpoint_not_imported;
    return out;
  }
  if (!pg::is_rigid_transform(link.transform)) {
    out.skip = RtabmapLinkSkip::invalid_transform;
    return out;
  }

  ProjectDB::PoseGraphEdge e;
  e.from_node_id = link.from + id_offset;
  e.to_node_id = link.to + id_offset;
  e.edge_type = type;
  e.transform = link.transform;
  const auto info = rtabmap_information_to_gtsam(link.information);
  if (pg::is_valid_information(info))
    e.information = info; // else NULL: the per-type default sigma
  e.source = std::string(pg::kSourceRtabmap);
  e.status = std::string(pg::kStatusAuto);
  const auto err =
      pg::transform_error(pg::matrix_from_row_major(link.transform),
                          pg::matrix_from_row_major(a->second),
                          pg::matrix_from_row_major(b->second));
  e.seed_disagreement_m = err.translation_m;
  e.seed_disagreement_deg = err.rotation_deg;
  out.edge = std::move(e);
  return out;
}

} // namespace reusex::io
