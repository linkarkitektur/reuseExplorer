// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// The PDF report's cover image (Forside), chosen per project.
///
/// Stored in project_settings under `report/cover_image` as one of
///   {"kind":"frame","id":N,"region":[x,y,w,h]|null}     (a sensor frame)
///   {"kind":"panorama","id":N,"region":[x,y,w,h]|null}  (a 360° panorama)
///   {"kind":"upload"}                                  (report_assets "cover")
/// The frame/panorama form is the resource view record's shape
/// (core/resource_view.hpp). Unset ("" or no row): the report falls back to
/// a render of the model, which the caller supplies
/// (ReportOptions::cover_fallback) — core cannot link `visualize`.

#include <reusex/core/resource_view.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace reusex::core {

inline constexpr std::string_view kReportCoverSetting = "report/cover_image";
/// report_assets name of an uploaded cover.
inline constexpr std::string_view kReportCoverAsset = "cover";
/// Longest side of a cover image the report embeds; larger crops are
/// scaled down (≈ 250 dpi across the A4 text block).
inline constexpr int kReportCoverMaxSide = 1800;

struct ReportCover {
  enum class Source { frame, panorama, upload };
  Source source = Source::frame;
  /// Set for frame and panorama.
  std::optional<ResourceView> view;
};

/// "frame" | "panorama" | "upload".
std::string_view to_string(ReportCover::Source source);

/// The stored choice; nullopt when unset. A stored value that no longer
/// parses (hand-edited) reads as unset, with a warning.
std::optional<ReportCover> report_cover(const ProjectDB &db);

/// Wire form: {"source": "frame"|"panorama"|"upload"|null, "id": N|null,
/// "region": [x,y,w,h]|null, "mime_type": "image/jpeg"|null (upload)}.
nlohmann::json report_cover_json(const ProjectDB &db);

/// Choose a sensor frame or a panorama (+ region). Replaces any upload
/// (its asset is deleted).
/// @throws std::invalid_argument for a malformed view (validate_view_shape);
///         std::out_of_range when the frame / panorama does not exist.
void set_report_cover_view(ProjectDB &db, const ResourceView &view);

/// Store an uploaded image as the cover. Only JPEG and PNG are accepted
/// (sniffed from the bytes, which must also decode).
/// @returns the MIME type stored.
/// @throws std::invalid_argument for anything else.
std::string set_report_cover_upload(ProjectDB &db,
                                    const std::vector<std::uint8_t> &bytes);

/// Back to the fallback render: clears the setting and drops an upload.
void clear_report_cover(ProjectDB &db);

struct EncodedImage {
  std::vector<std::uint8_t> bytes;
  std::string mime_type; ///< "image/jpeg" | "image/png"
};

/// The cover's bytes as the report embeds them: a frame or panorama is
/// cropped to its region (a panorama region may wrap across the seam),
/// scaled to at most kReportCoverMaxSide and JPEG-encoded; an upload is
/// returned as stored. nullopt when unset, or when the chosen image is gone
/// (warned) — the caller then uses its fallback.
std::optional<EncodedImage> report_cover_image(const ProjectDB &db);

} // namespace reusex::core
