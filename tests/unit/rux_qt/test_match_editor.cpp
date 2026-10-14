// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Rediger's match editor, the Qt-free half (spec 2026-10-14 §4.4).

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/match_editor.hpp>

#include <cmath>

using namespace rux::qt;

namespace {

MatchPoint pt(float u, float v, bool depth = true) {
  MatchPoint p;
  p.u = u;
  p.v = v;
  if (depth)
    p.xyz = Vec3{u / 100.0, v / 100.0, 1.0};
  return p;
}

/// Three keypoints a side, matched 0-0, 1-1, 2-2; keypoint 2 of B has no depth.
void detect3(MatchSet &s) {
  s.set_detected({pt(1, 1), pt(2, 2), pt(3, 3)},
                 {pt(1, 1), pt(2, 2), pt(3, 3, false)},
                 {{0, 0, 10.0f}, {1, 1, 11.0f}, {2, 2, 12.0f}});
}

} // namespace

TEST_CASE("MatchSet_SwappedPairIsANewPair_AndClears", "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(7, 8);
  detect3(s);
  s.set_region(MatchSide::a, PixelBox{0, 0, 10, 10});
  s.set_pair(7, 8); // same pair: kept
  CHECK(s.lines().size() == 3);
  s.set_pair(8, 7); // Byt A og B: a different pair
  CHECK(s.lines().empty());
  CHECK(s.keypoints(MatchSide::a).empty());
  CHECK_FALSE(s.has_region());
  CHECK_FALSE(s.counts().searched);
}

TEST_CASE("MatchSet_FitInput_TakesEnabledLinesWithDepthAtBothEnds",
          "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  detect3(s);
  s.toggle_excluded(0);
  const FitInput in = s.fit_input();
  REQUIRE(in.lines == std::vector<int>{1}); // 0 excluded, 2 lacks depth in B
  CHECK(in.pairs.size() == 1);
  CHECK(s.counts().usable == 1);
}

TEST_CASE("MatchSet_ApplyFit_ColoursFittedLinesOnly", "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  s.set_detected({pt(1, 1), pt(2, 2), pt(3, 3), pt(4, 4)},
                 {pt(1, 1), pt(2, 2), pt(3, 3), pt(4, 4, false)},
                 {{0, 0, 1}, {1, 1, 1}, {2, 2, 1}, {3, 3, 1}});
  const FitInput in = s.fit_input(); // lines 0, 1, 2
  s.apply_fit(in, {0, 2});           // pairs 0 and 2 are inliers
  CHECK(s.lines()[0].fit == MatchFit::inlier);
  CHECK(s.lines()[1].fit == MatchFit::outlier);
  CHECK(s.lines()[2].fit == MatchFit::inlier);
  CHECK(s.lines()[3].fit == MatchFit::neutral); // never fitted: no depth
  const MatchCounts c = s.counts();
  CHECK(c.fitted);
  CHECK(c.inliers == 2);
  s.toggle_excluded(0); // excluding an inlier: faint, no longer counted
  CHECK(line_tone(s.lines()[0]) == LineTone::excluded);
  CHECK(s.counts().inliers == 1);
}

TEST_CASE("MatchSet_RedetectKeepsManualMatches_AndResetsTheFit",
          "[rux_qt][matches]") {
  MatchSet s;
  s.set_pair(1, 2);
  detect3(s);
  s.add_manual(pt(50, 50), pt(60, 60));
  s.apply_fit(s.fit_input(), {0, 1, 2});
  detect3(s); // Find match again (e.g. with a region)
  const MatchCounts c = s.counts();
  CHECK(c.total == 4);
  CHECK(c.manual == 1);
  CHECK_FALSE(c.fitted);
  const MatchLine &m = s.lines().back();
  CHECK(m.manual);
  CHECK(s.keypoints(MatchSide::a)[static_cast<std::size_t>(m.a)].u == 50.0f);
  CHECK(s.keypoints(MatchSide::b)[static_cast<std::size_t>(m.b)].u == 60.0f);
}

TEST_CASE("ManualPick_CompletesAcrossSides_AndReplacesOnTheSameSide",
          "[rux_qt][matches]") {
  ManualPick p;
  CHECK_FALSE(p.pick(MatchSide::a, pt(1, 1)));
  CHECK_FALSE(p.pick(MatchSide::a, pt(2, 2))); // re-aim A
  REQUIRE(p.pending());
  CHECK(p.point()->u == 2.0f);
  const auto done = p.pick(MatchSide::b, pt(9, 9));
  REQUIRE(done);
  CHECK(done->first.u == 2.0f);
  CHECK(done->second.u == 9.0f);
  CHECK_FALSE(p.pending());
  CHECK_FALSE(p.pick(MatchSide::b, pt(3, 3))); // B first works too
  const auto rev = p.pick(MatchSide::a, pt(4, 4));
  REQUIRE(rev);
  CHECK(rev->first.u == 4.0f); // always (A, B)
  p.pick(MatchSide::a, pt(5, 5));
  p.cancel(); // Esc
  CHECK_FALSE(p.pending());
}

TEST_CASE("MatchReadout_NamesWhatThereIs", "[rux_qt][matches]") {
  CHECK(match_readout({}) == "Ingen match endnu");
  MatchCounts none;
  none.searched = true;
  CHECK(match_readout(none) == "Ingen match fundet");
  MatchCounts c;
  c.searched = true;
  c.total = 38;
  c.inliers = 24;
  c.manual = 3;
  c.excluded = 2;
  c.fitted = true;
  CHECK(match_readout(c) == "38 matches · 24 inliers · 3 manuelle · 2 udeladt");
  MatchCounts one;
  one.searched = true;
  one.total = 1;
  one.manual = 1;
  CHECK(match_readout(one) == "1 match · 1 manuel");
}

TEST_CASE("MatchFitTexts_AreDanish", "[rux_qt][matches]") {
  CHECK(match_fit_text(24, 0.021) == "Fra matches: 24 inliers · RMS 2,1 cm");
  CHECK(
      match_fit_failure_text(MatchFitFailure::too_few_pairs, 2, 0, 0.05) ==
      "Placér B kræver mindst 3 match med dybde i begge billeder — der er 2.");
  CHECK(match_fit_failure_text(MatchFitFailure::no_consensus, 9, 2, 0.05) ==
        "Ingen sammenhængende placering: kun 2 af 9 match passer sammen inden "
        "for 5 cm.");
  CHECK(match_fit_failure_text(MatchFitFailure::degenerate, 5, 5, 0.05) ==
        "Matchene ligger næsten på en linje — tilføj match et andet sted i "
        "billedet.");
}

TEST_CASE("FrameProblemText_IsDanish_AndNamesTheFrame", "[rux_qt][matches]") {
  CHECK(frame_problem_text(2241, FrameProblem::no_depth) ==
        "billede 2241 har ingen dybde");
  CHECK(frame_problem_text(7, FrameProblem::no_colour) ==
        "billede 7 har intet farvebillede");
  CHECK(frame_problem_text(12, FrameProblem::no_pose) ==
        "billede 12 har ingen pose");
}

TEST_CASE("ImageMapping_RoundTrips_AndCentresPixels", "[rux_qt][matches]") {
  const ViewRect slot{10, 20, 200, 100};
  const ViewRect r = image_rect(slot, 400, 100, {}); // fits the width
  CHECK(r.x == 10.0);
  CHECK(r.w == 200.0);
  CHECK(r.h == 50.0);
  CHECK(r.y == 45.0);
  const auto c = image_to_canvas(r, 400, 100, 0.0, 0.0); // pixel 0's centre
  CHECK(c[0] == 10.25);
  const auto back = canvas_to_image(r, 400, 100, 123.0, 67.0);
  const auto again = image_to_canvas(r, 400, 100, back[0], back[1]);
  CHECK(std::abs(again[0] - 123.0) < 1e-9);
  CHECK(std::abs(again[1] - 67.0) < 1e-9);
}

TEST_CASE("ZoomAbout_KeepsThePointUnderTheCursor_AndClamps",
          "[rux_qt][matches]") {
  const ViewRect slot{0, 0, 300, 200};
  const ImageView v0{};
  const ViewRect r0 = image_rect(slot, 600, 400, v0);
  const auto under = canvas_to_image(r0, 600, 400, 80.0, 50.0);
  const ImageView v1 = zoom_about(slot, 600, 400, v0, 80.0, 50.0, 2.0);
  CHECK(v1.zoom == 2.0);
  const auto at = image_to_canvas(image_rect(slot, 600, 400, v1), 600, 400,
                                  under[0], under[1]);
  CHECK(std::abs(at[0] - 80.0) < 1e-9);
  CHECK(std::abs(at[1] - 50.0) < 1e-9);
  CHECK(zoom_about(slot, 600, 400, v1, 0, 0, 1000.0).zoom == kMaxImageZoom);
  const ImageView out = zoom_about(slot, 600, 400, v1, 10, 10, 0.01);
  CHECK(out.zoom == 1.0); // back to fitted, centred
  CHECK(out.pan_x == 0.0);
  CHECK(out.pan_y == 0.0);
}

TEST_CASE("PairSlots_SplitWithAGap", "[rux_qt][matches]") {
  const auto s = pair_slots(410, 100, 10);
  CHECK(s[0].x == 0.0);
  CHECK(s[0].w == 200.0);
  CHECK(s[1].x == 210.0);
  CHECK(s[1].w == 200.0);
}

TEST_CASE("NearestSegment_HitsWithinTheTolerance", "[rux_qt][matches]") {
  const std::vector<Segment> segs{{0, 0, 100, 0}, {0, 10, 100, 10}};
  CHECK(nearest_segment(segs, 50, 3, 4) == 0);
  CHECK(nearest_segment(segs, 50, 8, 4) == 1);
  CHECK(nearest_segment(segs, 50, 30, 4) == -1);
  CHECK(nearest_segment(segs, 105, 0, 4) == -1); // past the end: 5 px
  CHECK(nearest_segment(segs, 50, 5, 6) == 0);   // tie: lowest index
}

TEST_CASE("BoxFromDrag_ClampsToTheImage_AndIgnoresAClick",
          "[rux_qt][matches]") {
  const PixelBox b = box_from_drag(250, 90, -20.5, 10.2, 200, 100);
  CHECK(b.x == 0);
  CHECK(b.y == 10);
  CHECK(b.w == 200);
  CHECK(b.h == 80);
  CHECK(box_from_drag(10, 10, 12, 11, 200, 100).empty());
  CHECK(box_from_drag(300, 300, 400, 400, 200, 100).empty()); // off-image
}

TEST_CASE("InImage_UsesThePixelCentreRange", "[rux_qt][matches]") {
  // Pixel u's centre is u, so the image spans [-0.5, w - 0.5).
  CHECK(in_image(-0.5, -0.5, 4, 3));
  CHECK(in_image(3.49, 2.49, 4, 3));
  CHECK(in_image(0, 0, 4, 3));
  CHECK_FALSE(in_image(-0.51, 0, 4, 3));
  CHECK_FALSE(in_image(3.5, 0, 4, 3));
  CHECK_FALSE(in_image(0, 2.5, 4, 3));
  CHECK_FALSE(in_image(0, 0, 0, 3));
}
