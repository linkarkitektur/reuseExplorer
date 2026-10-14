// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's match editor, Qt-free (spec 2026-10-14 §4.4): the working set
// of correspondences between A and B (automatic and manual, excluded or
// not, coloured by the last fit), a half-made manual match, the Danish
// read-outs, and the canvas geometry — image <-> canvas mapping under a
// per-image zoom/pan, zoom about the cursor, line hit-testing, a dragged
// region. Nothing here is stored: a different pair clears it all (C4).

#include <rux_qt/pair_alignment.hpp>

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rux::qt {

enum class MatchSide { a, b };
/// Vælg (toggle a line, drag pans), Tilføj (click A then B), Område (drag
/// a rectangle that limits the next Find match).
enum class MatchMode { select, add, region };
enum class MatchFit { neutral, inlier, outlier };
enum class LineTone { neutral, inlier, outlier, excluded };
enum class MatchFitFailure { too_few_pairs, no_consensus, degenerate };

struct MatchPoint {
  float u = 0, v = 0;      ///< colour-image pixel
  std::optional<Vec3> xyz; ///< stored-pose-frame point; none = no depth
};

struct MatchLine {
  int a = -1, b = -1; ///< keypoint indices on A's / B's side
  float distance = 0;
  bool manual = false;
  bool excluded = false;
  MatchFit fit = MatchFit::neutral; ///< from the last Placér B fra match
};

/// A colour-image rectangle; empty when w or h <= 0.
struct PixelBox {
  int x = 0, y = 0, w = 0, h = 0;
  bool empty() const { return w <= 0 || h <= 0; }
};

struct MatchCounts {
  int total = 0;   ///< lines, automatic and manual
  int inliers = 0; ///< fitted inliers not excluded since
  int manual = 0;
  int excluded = 0;
  int usable = 0;        ///< enabled lines with depth at both ends
  bool fitted = false;   ///< a fit coloured the lines
  bool searched = false; ///< Find match ran for this pair
};

/// What a fit consumes: one (A xyz, B xyz) per usable line, and the line
/// each pair came from.
struct FitInput {
  std::vector<std::pair<Vec3, Vec3>> pairs;
  std::vector<int> lines;
};

class MatchSet {
    public:
  /// A different pair (a swap included) clears everything.
  void set_pair(int a, int b);
  int a() const { return a_; }
  int b() const { return b_; }
  void clear();
  /// Find match's result: replaces the automatic lines, keeps the manual
  /// ones (and whether they are excluded), resets every fit colour.
  void set_detected(std::vector<MatchPoint> keypoints_a,
                    std::vector<MatchPoint> keypoints_b,
                    std::vector<MatchLine> lines);
  /// A manual line between two lifted points; neutral until the next fit.
  void add_manual(const MatchPoint &a, const MatchPoint &b);
  /// Vælg: a line in or out; an excluded line loses its fit colour.
  void toggle_excluded(int line);
  FitInput fit_input() const;
  /// @p inliers index into @p in.pairs (RigidFit::inliers): those lines
  /// become inliers, the rest of @p in.lines outliers, others neutral.
  void apply_fit(const FitInput &in, const std::vector<int> &inliers);
  const std::vector<MatchPoint> &keypoints(MatchSide side) const;
  const std::vector<MatchLine> &lines() const { return lines_; }
  MatchCounts counts() const;
  void set_region(MatchSide side, std::optional<PixelBox> box);
  const std::optional<PixelBox> &region(MatchSide side) const;
  bool has_region() const;
  void clear_regions();

    private:
  int a_ = -1, b_ = -1;
  std::vector<MatchPoint> kp_a_, kp_b_;
  std::vector<MatchLine> lines_;
  std::optional<PixelBox> region_a_, region_b_;
  bool fitted_ = false, searched_ = false;
};

/// Tilføj mode's half-made match: the first click's lifted point, waiting
/// for one on the other side.
class ManualPick {
    public:
  /// The completed (A, B) pair when the other side was picked before; a
  /// pick on the same side replaces the waiting point.
  std::optional<std::pair<MatchPoint, MatchPoint>> pick(MatchSide side,
                                                        const MatchPoint &p);
  void cancel() { side_.reset(); }
  bool pending() const { return side_.has_value(); }
  std::optional<MatchSide> side() const { return side_; }
  const MatchPoint *point() const { return side_ ? &point_ : nullptr; }

    private:
  std::optional<MatchSide> side_;
  MatchPoint point_;
};

LineTone line_tone(const MatchLine &l);
/// "38 matches · 24 inliers · 3 manuelle · 2 udeladt" (inliers only after a
/// fit, manual/excluded only when > 0); "Ingen match endnu" before Find
/// match, "Ingen match fundet" after one that found none.
std::string match_readout(const MatchCounts &c);
/// "Fra matches: 24 inliers · RMS 2,1 cm".
std::string match_fit_text(int inliers, double rms_m);
std::string match_fit_failure_text(MatchFitFailure f, int pairs, int inliers,
                                   double inlier_dist_m);

// ---------------------------------------------------------- canvas geometry

struct ViewRect {
  double x = 0, y = 0, w = 0, h = 0;
  bool contains(double px, double py) const {
    return px >= x && px <= x + w && py >= y && py <= y + h;
  }
};

/// One image's zoom (1 = fitted) and pan (canvas px), independent per side.
struct ImageView {
  double zoom = 1.0, pan_x = 0.0, pan_y = 0.0;
};
inline constexpr double kMaxImageZoom = 16.0;

/// A's and B's slots side by side, @p gap apart (the match lines cross it).
std::array<ViewRect, 2> pair_slots(double width, double height, double gap);
/// Where an @p img_w x @p img_h image is drawn: fitted into @p slot
/// (aspect kept, centred), scaled by the zoom about the slot centre, panned.
ViewRect image_rect(const ViewRect &slot, int img_w, int img_h,
                    const ImageView &view);
/// Pixel (u, v)'s CENTRE on the canvas (pixel u spans [u, u+1) * scale).
std::array<double, 2> image_to_canvas(const ViewRect &drawn, int img_w,
                                      int img_h, double u, double v);
std::array<double, 2> canvas_to_image(const ViewRect &drawn, int img_w,
                                      int img_h, double x, double y);
/// The view after zooming by @p factor about canvas point (x, y), keeping
/// the image point under it fixed; clamped to [1, kMaxImageZoom], and at 1
/// back to the fitted, unpanned view.
ImageView zoom_about(const ViewRect &slot, int img_w, int img_h, ImageView view,
                     double x, double y, double factor);

struct Segment {
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
};
/// The segment nearest (x, y) within @p max_dist; -1 when none. Ties go to
/// the lowest index.
int nearest_segment(const std::vector<Segment> &segs, double x, double y,
                    double max_dist);
/// Whether image point (u, v) lies on an @p img_w x @p img_h image, by the
/// pixel-centre convention: pixel u spans [u - 0.5, u + 0.5), so the image
/// is [-0.5, w - 0.5) x [-0.5, h - 0.5).
bool in_image(double u, double v, int img_w, int img_h);
/// A dragged rectangle in image px, normalised and clamped to the image;
/// empty when it is smaller than @p min_px either way (a click, not a drag)
/// or lies outside the image.
PixelBox box_from_drag(double u0, double v0, double u1, double v1, int img_w,
                       int img_h, int min_px = 4);

} // namespace rux::qt
