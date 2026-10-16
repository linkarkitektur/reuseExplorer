// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/pose_fingerprint.hpp"

#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>

namespace reusex::core {

std::string compute_pose_fingerprint(const ProjectDB &db) {
  // FNV-1a 64-bit. Not a cryptographic hash — just cheap, deterministic
  // change detection over data this process already has in memory, not
  // anything persisted across builds or machines.
  constexpr uint64_t kOffsetBasis = 1469598103934665603ull;
  constexpr uint64_t kPrime = 1099511628211ull;
  uint64_t h = kOffsetBasis;
  auto mix = [&h](const void *data, std::size_t len) {
    const auto *bytes = static_cast<const unsigned char *>(data);
    for (std::size_t i = 0; i < len; ++i) {
      h ^= bytes[i];
      h *= kPrime;
    }
  };

  // Sorted so the fingerprint does not depend on sensor_frame_ids()'s
  // (unspecified) order.
  auto ids = db.sensor_frame_ids();
  std::sort(ids.begin(), ids.end());

  for (int id : ids) {
    // A frame with no usable pose contributes nothing to reconstruction
    // (segmentation/reconstruct.cpp skips it the same way), so it must not
    // contribute to the fingerprint either.
    if (!db.has_sensor_frame_pose(id))
      continue;
    mix(&id, sizeof(id));
    const auto pose = db.sensor_frame_pose(id);
    mix(pose.data(), pose.size() * sizeof(double));
  }
  return fmt::format("{:016x}", h);
}

PoseFreshness cloud_pose_freshness(const ProjectDB &db,
                                   std::string_view cloud_name) {
  if (!db.has_point_cloud(cloud_name))
    return PoseFreshness::unknown;

  const auto params = db.point_cloud_provenance(cloud_name).second;
  if (params.empty())
    return PoseFreshness::unknown;

  try {
    const auto j = nlohmann::json::parse(params);
    if (!j.is_object())
      return PoseFreshness::unknown;
    const auto it = j.find(kPoseFingerprintKey);
    if (it == j.end() || !it->is_string())
      return PoseFreshness::unknown;
    return it->get<std::string>() == compute_pose_fingerprint(db)
               ? PoseFreshness::fresh
               : PoseFreshness::stale;
  } catch (const nlohmann::json::exception &) {
    return PoseFreshness::unknown;
  }
}

PoseFreshness warn_if_cloud_poses_stale(const ProjectDB &db,
                                        std::string_view cloud_name) {
  if (!db.has_point_cloud(cloud_name))
    return PoseFreshness::unknown;

  const auto freshness = cloud_pose_freshness(db, cloud_name);
  switch (freshness) {
  case PoseFreshness::stale:
    warn("cloud '{}' was reconstructed from poses this project no longer has "
         "— its stamped pose fingerprint does not match the current "
         "sensor_frames transforms (a pose refinement such as `rux optimize` "
         "or `rux register` likely ran since); re-run `rux create clouds` to "
         "rebuild it from the current poses",
         cloud_name);
    break;
  case PoseFreshness::unknown:
    // Not a warning: a cloud built before this check existed is not evidence
    // of anything wrong, just unknowable without re-reading depth data.
    info("cloud '{}' has no recorded pose fingerprint (built by an older "
         "version of `rux create clouds`); whether it still matches the "
         "current poses is unknown",
         cloud_name);
    break;
  case PoseFreshness::fresh:
    break;
  }
  return freshness;
}

} // namespace reusex::core
