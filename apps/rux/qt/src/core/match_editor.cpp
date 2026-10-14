// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/database_logic.hpp>
#include <rux_qt/match_editor.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rux::qt {

void MatchSet::set_pair(int a, int b) {
  if (a == a_ && b == b_)
    return;
  clear();
  a_ = a;
  b_ = b;
}

void MatchSet::clear() {
  kp_a_.clear();
  kp_b_.clear();
  lines_.clear();
  region_a_.reset();
  region_b_.reset();
  fitted_ = searched_ = false;
}

void MatchSet::set_detected(std::vector<MatchPoint> keypoints_a,
                            std::vector<MatchPoint> keypoints_b,
                            std::vector<MatchLine> lines) {
  std::vector<std::pair<std::pair<MatchPoint, MatchPoint>, bool>> manual;
  for (const MatchLine &l : lines_)
    if (l.manual)
      manual.push_back({{kp_a_[static_cast<std::size_t>(l.a)],
                         kp_b_[static_cast<std::size_t>(l.b)]},
                        l.excluded});
  kp_a_ = std::move(keypoints_a);
  kp_b_ = std::move(keypoints_b);
  lines_ = std::move(lines);
  for (MatchLine &l : lines_) {
    l.manual = l.excluded = false;
    l.fit = MatchFit::neutral;
  }
  for (const auto &[pts, excluded] : manual) {
    add_manual(pts.first, pts.second);
    lines_.back().excluded = excluded;
  }
  fitted_ = false;
  searched_ = true;
}

void MatchSet::add_manual(const MatchPoint &a, const MatchPoint &b) {
  kp_a_.push_back(a);
  kp_b_.push_back(b);
  MatchLine l;
  l.a = static_cast<int>(kp_a_.size()) - 1;
  l.b = static_cast<int>(kp_b_.size()) - 1;
  l.manual = true;
  lines_.push_back(l);
}

void MatchSet::toggle_excluded(int line) {
  if (line < 0 || line >= static_cast<int>(lines_.size()))
    return;
  MatchLine &l = lines_[static_cast<std::size_t>(line)];
  l.excluded = !l.excluded;
  if (l.excluded)
    l.fit = MatchFit::neutral;
}

FitInput MatchSet::fit_input() const {
  FitInput in;
  for (int i = 0; i < static_cast<int>(lines_.size()); ++i) {
    const MatchLine &l = lines_[static_cast<std::size_t>(i)];
    const MatchPoint &pa = kp_a_[static_cast<std::size_t>(l.a)];
    const MatchPoint &pb = kp_b_[static_cast<std::size_t>(l.b)];
    if (l.excluded || !pa.xyz || !pb.xyz)
      continue;
    in.pairs.emplace_back(*pa.xyz, *pb.xyz);
    in.lines.push_back(i);
  }
  return in;
}

void MatchSet::apply_fit(const FitInput &in, const std::vector<int> &inliers) {
  for (MatchLine &l : lines_)
    l.fit = MatchFit::neutral;
  for (int line : in.lines)
    lines_[static_cast<std::size_t>(line)].fit = MatchFit::outlier;
  for (int k : inliers)
    if (k >= 0 && k < static_cast<int>(in.lines.size()))
      lines_[static_cast<std::size_t>(in.lines[static_cast<std::size_t>(k)])]
          .fit = MatchFit::inlier;
  fitted_ = true;
}

const std::vector<MatchPoint> &MatchSet::keypoints(MatchSide side) const {
  return side == MatchSide::a ? kp_a_ : kp_b_;
}

MatchCounts MatchSet::counts() const {
  MatchCounts c;
  c.total = static_cast<int>(lines_.size());
  for (const MatchLine &l : lines_) {
    c.manual += l.manual ? 1 : 0;
    c.excluded += l.excluded ? 1 : 0;
    c.inliers += !l.excluded && l.fit == MatchFit::inlier ? 1 : 0;
  }
  c.usable = static_cast<int>(fit_input().lines.size());
  c.fitted = fitted_;
  c.searched = searched_;
  return c;
}

void MatchSet::set_region(MatchSide side, std::optional<PixelBox> box) {
  if (box && box->empty())
    box.reset();
  (side == MatchSide::a ? region_a_ : region_b_) = box;
}

const std::optional<PixelBox> &MatchSet::region(MatchSide side) const {
  return side == MatchSide::a ? region_a_ : region_b_;
}

bool MatchSet::has_region() const { return region_a_ || region_b_; }

void MatchSet::clear_regions() {
  region_a_.reset();
  region_b_.reset();
}

std::optional<std::pair<MatchPoint, MatchPoint>>
ManualPick::pick(MatchSide side, const MatchPoint &p) {
  if (!side_ || *side_ == side) {
    side_ = side;
    point_ = p;
    return std::nullopt;
  }
  const MatchPoint first = point_;
  side_.reset();
  return side == MatchSide::b ? std::pair{first, p} : std::pair{p, first};
}

LineTone line_tone(const MatchLine &l) {
  if (l.excluded)
    return LineTone::excluded;
  switch (l.fit) {
  case MatchFit::inlier:
    return LineTone::inlier;
  case MatchFit::outlier:
    return LineTone::outlier;
  case MatchFit::neutral:
    break;
  }
  return LineTone::neutral;
}

std::string match_readout(const MatchCounts &c) {
  if (c.total == 0)
    return c.searched ? "Ingen match fundet" : "Ingen match endnu";
  std::string s =
      std::to_string(c.total) + (c.total == 1 ? " match" : " matches");
  if (c.fitted)
    s += " · " + std::to_string(c.inliers) +
         (c.inliers == 1 ? " inlier" : " inliers");
  if (c.manual > 0)
    s += " · " + std::to_string(c.manual) +
         (c.manual == 1 ? " manuel" : " manuelle");
  if (c.excluded > 0)
    s += " · " + std::to_string(c.excluded) + " udeladt";
  return s;
}

std::string match_fit_text(int inliers, double rms_m) {
  return "Fra matches: " + std::to_string(inliers) +
         (inliers == 1 ? " inlier" : " inliers") + " · RMS " +
         format_decimal_da(rms_m * 100.0, 1) + " cm";
}

std::string match_fit_failure_text(MatchFitFailure f, int pairs, int inliers,
                                   double inlier_dist_m) {
  switch (f) {
  case MatchFitFailure::too_few_pairs:
    return "Placér B kræver mindst 3 match med dybde i begge billeder — der "
           "er " +
           std::to_string(pairs) + ".";
  case MatchFitFailure::no_consensus:
    return "Ingen sammenhængende placering: kun " + std::to_string(inliers) +
           " af " + std::to_string(pairs) + " match passer sammen inden for " +
           format_decimal_da(inlier_dist_m * 100.0, 0) + " cm.";
  case MatchFitFailure::degenerate:
    break;
  }
  return "Matchene ligger næsten på en linje — tilføj match et andet sted i "
         "billedet.";
}

std::string frame_problem_text(int frame_id, FrameProblem p) {
  const std::string head = "billede " + std::to_string(frame_id) + " har ";
  switch (p) {
  case FrameProblem::no_colour:
    return head + "intet farvebillede";
  case FrameProblem::no_depth:
    return head + "ingen dybde";
  case FrameProblem::no_pose:
    break;
  }
  return head + "ingen pose";
}

std::array<ViewRect, 2> pair_slots(double width, double height, double gap) {
  const double w = std::max(0.0, (width - gap) / 2.0);
  return {ViewRect{0.0, 0.0, w, height}, ViewRect{w + gap, 0.0, w, height}};
}

ViewRect image_rect(const ViewRect &slot, int img_w, int img_h,
                    const ImageView &view) {
  if (img_w <= 0 || img_h <= 0 || slot.w <= 0 || slot.h <= 0)
    return {};
  const double s = std::min(slot.w / img_w, slot.h / img_h) * view.zoom;
  const double w = img_w * s, h = img_h * s;
  return {slot.x + (slot.w - w) / 2.0 + view.pan_x,
          slot.y + (slot.h - h) / 2.0 + view.pan_y, w, h};
}

std::array<double, 2> image_to_canvas(const ViewRect &d, int img_w, int img_h,
                                      double u, double v) {
  return {d.x + (u + 0.5) * d.w / img_w, d.y + (v + 0.5) * d.h / img_h};
}

std::array<double, 2> canvas_to_image(const ViewRect &d, int img_w, int img_h,
                                      double x, double y) {
  return {(x - d.x) * img_w / d.w - 0.5, (y - d.y) * img_h / d.h - 0.5};
}

ImageView zoom_about(const ViewRect &slot, int img_w, int img_h, ImageView view,
                     double x, double y, double factor) {
  const double zoom = std::clamp(view.zoom * factor, 1.0, kMaxImageZoom);
  if (zoom == 1.0)
    return {};
  const auto under =
      canvas_to_image(image_rect(slot, img_w, img_h, view), img_w, img_h, x, y);
  ImageView next{zoom, view.pan_x, view.pan_y};
  const auto now = image_to_canvas(image_rect(slot, img_w, img_h, next), img_w,
                                   img_h, under[0], under[1]);
  next.pan_x += x - now[0];
  next.pan_y += y - now[1];
  return next;
}

int nearest_segment(const std::vector<Segment> &segs, double x, double y,
                    double max_dist) {
  int best = -1;
  double best_d = max_dist;
  for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
    const Segment &s = segs[static_cast<std::size_t>(i)];
    const double dx = s.x1 - s.x0, dy = s.y1 - s.y0;
    const double len2 = dx * dx + dy * dy;
    const double t =
        len2 > 0
            ? std::clamp(((x - s.x0) * dx + (y - s.y0) * dy) / len2, 0.0, 1.0)
            : 0.0;
    const double d = std::hypot(x - (s.x0 + t * dx), y - (s.y0 + t * dy));
    if (d < best_d || (d == best_d && best < 0)) {
      best = i;
      best_d = d;
    }
  }
  return best;
}

bool in_image(double u, double v, int img_w, int img_h) {
  return u >= -0.5 && v >= -0.5 && u < img_w - 0.5 && v < img_h - 0.5;
}

PixelBox box_from_drag(double u0, double v0, double u1, double v1, int img_w,
                       int img_h, int min_px) {
  if (std::abs(u1 - u0) < min_px || std::abs(v1 - v0) < min_px)
    return {};
  const int x0 =
      std::clamp(static_cast<int>(std::floor(std::min(u0, u1))), 0, img_w);
  const int y0 =
      std::clamp(static_cast<int>(std::floor(std::min(v0, v1))), 0, img_h);
  const int x1 =
      std::clamp(static_cast<int>(std::ceil(std::max(u0, u1))), 0, img_w);
  const int y1 =
      std::clamp(static_cast<int>(std::ceil(std::max(v0, v1))), 0, img_h);
  return {x0, y0, x1 - x0, y1 - y0};
}

} // namespace rux::qt
