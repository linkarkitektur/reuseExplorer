// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/report_cover.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace reusex::core {

namespace {

std::optional<std::string> sniff_mime(const std::vector<std::uint8_t> &b) {
  if (b.size() >= 3 && b[0] == 0xFF && b[1] == 0xD8 && b[2] == 0xFF)
    return "image/jpeg";
  if (b.size() >= 8 && b[0] == 0x89 && b[1] == 'P' && b[2] == 'N' &&
      b[3] == 'G' && b[4] == 0x0D && b[5] == 0x0A && b[6] == 0x1A &&
      b[7] == 0x0A)
    return "image/png";
  return std::nullopt;
}

/// The pixel span [first, first + count) of a normalised [x, x + w).
std::pair<int, int> span(double x, double w, int size) {
  const int first = std::clamp(static_cast<int>(std::floor(x * size)), 0,
                               std::max(0, size - 1));
  const int last = std::clamp(static_cast<int>(std::ceil((x + w) * size)),
                              first + 1, size * 2);
  return {first, last - first};
}

/// @p image cut to @p region; horizontally wrapped when @p wrap (an
/// equirect panorama whose region crosses the seam).
cv::Mat crop(const cv::Mat &image, const std::optional<ViewRegion> &region,
             bool wrap) {
  if (!region)
    return image;
  const auto &r = *region;
  const auto [y0, h] = span(r[1], r[3], image.rows);
  const int rows = std::min(h, image.rows - y0);
  auto [x0, w] = span(r[0], r[2], image.cols);
  if (!wrap)
    w = std::min(w, image.cols - x0);
  w = std::min(w, image.cols);
  if (x0 + w <= image.cols)
    return image(cv::Rect(x0, y0, w, rows)).clone();
  // Across the seam: the right end, then the left start.
  cv::Mat out;
  cv::hconcat(image(cv::Rect(x0, y0, image.cols - x0, rows)),
              image(cv::Rect(0, y0, x0 + w - image.cols, rows)), out);
  return out;
}

cv::Mat to_bgr8(const cv::Mat &in) {
  cv::Mat m = in;
  if (m.depth() != CV_8U) {
    double lo = 0, hi = 0;
    cv::minMaxLoc(m.reshape(1), &lo, &hi);
    m.convertTo(m, CV_8U, hi > 255.0 ? 255.0 / 65535.0 : 1.0);
  }
  if (m.channels() == 1)
    cv::cvtColor(m, m, cv::COLOR_GRAY2BGR);
  else if (m.channels() == 4)
    cv::cvtColor(m, m, cv::COLOR_BGRA2BGR);
  return m;
}

} // namespace

std::string_view to_string(ReportCover::Source source) {
  switch (source) {
  case ReportCover::Source::frame:
    return "frame";
  case ReportCover::Source::panorama:
    return "panorama";
  case ReportCover::Source::upload:
    return "upload";
  }
  return "frame";
}

std::optional<ReportCover> report_cover(const ProjectDB &db) {
  const auto stored = db.setting(kReportCoverSetting);
  if (!stored || stored->empty())
    return std::nullopt;
  try {
    const auto j = nlohmann::json::parse(*stored);
    if (j.is_object() && j.value("kind", std::string()) == "upload") {
      ReportCover c;
      c.source = ReportCover::Source::upload;
      return c;
    }
    ReportCover c;
    c.view = parse_view(*stored);
    c.source = c.view->kind == ViewKind::panorama
                   ? ReportCover::Source::panorama
                   : ReportCover::Source::frame;
    return c;
  } catch (const std::exception &e) {
    reusex::warn("report cover: ignoring unparsable {} value '{}': {}",
                 kReportCoverSetting, *stored, e.what());
    return std::nullopt;
  }
}

nlohmann::json report_cover_json(const ProjectDB &db) {
  nlohmann::json out{{"source", nullptr},
                     {"id", nullptr},
                     {"region", nullptr},
                     {"mime_type", nullptr}};
  const auto cover = report_cover(db);
  if (!cover)
    return out;
  out["source"] = std::string(to_string(cover->source));
  if (cover->view) {
    out["id"] = cover->view->id;
    if (cover->view->region)
      out["region"] = *cover->view->region;
  } else if (const auto asset = db.report_asset(kReportCoverAsset)) {
    out["mime_type"] = asset->second;
  }
  return out;
}

void set_report_cover_view(ProjectDB &db, const ResourceView &view) {
  validate_view_shape(view);
  if (view.kind == ViewKind::frame ? !db.has_sensor_frame(view.id)
                                   : !db.has_panoramic_image(view.id))
    throw std::out_of_range(
        std::string("no ") +
        (view.kind == ViewKind::frame ? "sensor frame " : "panorama ") +
        std::to_string(view.id));
  ProjectDB::Transaction tx(db);
  db.set_setting(kReportCoverSetting, view_to_json(view));
  db.delete_report_asset(kReportCoverAsset);
  tx.commit();
}

std::string set_report_cover_upload(ProjectDB &db,
                                    const std::vector<std::uint8_t> &bytes) {
  const auto mime = sniff_mime(bytes);
  if (!mime)
    throw std::invalid_argument("cover image must be a JPEG or a PNG file");
  const cv::Mat probe =
      cv::imdecode(cv::Mat(1, static_cast<int>(bytes.size()), CV_8UC1,
                           const_cast<std::uint8_t *>(bytes.data())),
                   cv::IMREAD_UNCHANGED);
  if (probe.empty())
    throw std::invalid_argument("cover image does not decode as " + *mime);
  ProjectDB::Transaction tx(db);
  db.set_report_asset(kReportCoverAsset, bytes, *mime);
  db.set_setting(kReportCoverSetting, R"({"kind":"upload"})");
  tx.commit();
  return *mime;
}

void clear_report_cover(ProjectDB &db) {
  ProjectDB::Transaction tx(db);
  db.set_setting(kReportCoverSetting, "");
  db.delete_report_asset(kReportCoverAsset);
  tx.commit();
}

std::optional<EncodedImage> report_cover_image(const ProjectDB &db) {
  const auto cover = report_cover(db);
  if (!cover)
    return std::nullopt;
  if (cover->source == ReportCover::Source::upload) {
    auto asset = db.report_asset(kReportCoverAsset);
    if (!asset || asset->first.empty()) {
      reusex::warn("report cover: {} names an upload but report_assets has "
                   "no '{}'; using the fallback",
                   kReportCoverSetting, kReportCoverAsset);
      return std::nullopt;
    }
    return EncodedImage{std::move(asset->first), std::move(asset->second)};
  }
  const auto &view = *cover->view;
  const bool pano = view.kind == ViewKind::panorama;
  const cv::Mat image =
      pano ? db.panoramic_image(view.id) : db.sensor_frame_image(view.id);
  if (image.empty()) {
    reusex::warn("report cover: {} #{} has no image; using the fallback",
                 pano ? "panorama" : "sensor frame", view.id);
    return std::nullopt;
  }
  cv::Mat out = to_bgr8(crop(image, view.region, pano));
  const int side = std::max(out.cols, out.rows);
  if (side > kReportCoverMaxSide) {
    const double s = static_cast<double>(kReportCoverMaxSide) / side;
    cv::resize(out, out, cv::Size(), s, s, cv::INTER_AREA);
  }
  EncodedImage enc;
  enc.mime_type = "image/jpeg";
  if (!cv::imencode(".jpg", out, enc.bytes, {cv::IMWRITE_JPEG_QUALITY, 88}) ||
      enc.bytes.empty()) {
    reusex::warn("report cover: JPEG encode failed for a {}x{} crop", out.cols,
                 out.rows);
    return std::nullopt;
  }
  return enc;
}

} // namespace reusex::core
