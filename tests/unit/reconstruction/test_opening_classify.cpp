// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// W5.2 (reusex-z1v.7.2): opening classification, mullion merge, instance /
// label / glass / camera-path evidence, door state and swing, on synthetic
// wall faces.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <reusex/reconstruction/openings.hpp>

#include "opening_synth.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace reusex;
using namespace reusex::geometry;
namespace os = reusex::test_support::openings;

namespace {

constexpr double kLen = 6.0, kH = 2.5;

core::StoreyRecord storey() {
  core::StoreyRecord s;
  s.idx = 0;
  s.z_floor = 0.0;
  s.z_ceiling = kH;
  return s;
}

OpeningRect rect(double s0, double s1, double z0, double z1) {
  OpeningRect r;
  r.s_start = s0;
  r.s_end = s1;
  r.z_sill = z0;
  r.z_head = z1;
  r.fill = 1.0;
  r.jamb_start = r.jamb_end = 1.0;
  r.at_floor = z0 <= 0.0;
  r.at_ceiling = z1 >= kH;
  return r;
}

/// A two-sided wall along x with the given holes cut through both faces,
/// floor and ceiling on both sides.
OpeningEvidence wall_with(const std::vector<os::Hole> &holes) {
  OpeningEvidence ev;
  os::add_face(ev.points, 0.05, kLen, kH, holes);
  os::add_face(ev.points, -0.05, kLen, kH, holes);
  os::add_floor_ceiling(ev.points, kLen, -3.0, 3.0, kH);
  return ev;
}

std::vector<DetectedOpening> run(const OpeningEvidence &ev) {
  auto w = os::wall(kLen, kH);
  w.storey = 0;
  return detect_openings(ev, {w}, storey());
}

} // namespace

TEST_CASE("ClassifyOpening_ByItsSillAndHead", "[reconstruction][openings]") {
  // Door: floor to 2.1 m.
  CHECK(classify_opening(rect(1, 2, 0.0, 2.1), 0.0, kH) == "door");
  // Low sill (a threshold) still a door.
  CHECK(classify_opening(rect(1, 2, 0.2, 2.2), 0.0, kH) == "door");
  // Window: a sill above 0.3 m.
  CHECK(classify_opening(rect(1, 2, 0.9, 2.1), 0.0, kH) == "window");
  // Floor to ceiling: an opening (a passage, a glazed front).
  CHECK(classify_opening(rect(1, 3, 0.0, kH), 0.0, kH) == "opening");
  // From the floor but only 1.2 m tall: neither a door nor full height.
  CHECK(classify_opening(rect(1, 2, 0.0, 1.2), 0.0, kH) == "window");
  // Relative to the storey's floor, not to z = 0.
  CHECK(classify_opening(rect(1, 2, 3.0, 5.1), 3.0, 5.5) == "door");
}

TEST_CASE("MergeMullions_JoinsASplitWindow", "[reconstruction][openings]") {
  // Two panes 10 cm apart with the same sill and head: one window...
  auto m = merge_mullions({rect(1.0, 1.8, 0.9, 2.1), rect(1.9, 2.7, 0.9, 2.1)});
  REQUIRE(m.size() == 1);
  CHECK_THAT(m[0].s_start, WithinAbs(1.0, 1e-12));
  CHECK_THAT(m[0].s_end, WithinAbs(2.7, 1e-12));
  CHECK(m[0].fill < 1.0);
  // ...but not two windows a pier apart, nor a door next to a window.
  CHECK(merge_mullions({rect(1.0, 1.8, 0.9, 2.1), rect(2.5, 3.3, 0.9, 2.1)})
            .size() == 2);
  CHECK(merge_mullions({rect(1.0, 1.8, 0.0, 2.1), rect(1.9, 2.7, 0.9, 2.1)})
            .size() == 2);
}

TEST_CASE("DetectOpenings_EachClass_GeometryAlone",
          "[reconstruction][openings]") {
  const auto found = run(wall_with({{0.6, 1.5, 0.0, 2.1},   // door
                                    {2.2, 3.4, 0.9, 2.1},   // window
                                    {4.2, 5.4, 0.0, kH}})); // full height
  REQUIRE(found.size() == 3);
  CHECK(found[0].record.kind == "door");
  CHECK(found[1].record.kind == "window");
  CHECK(found[2].record.kind == "opening");
  for (const auto &d : found) {
    INFO(d.record.kind);
    CHECK(d.record.wall == "w-test");
    CHECK(d.record.source == "detected");
    CHECK(d.record.source_instance_guid.empty());
    // No instance, label, glass or camera path: confidence from the
    // geometry alone, capped.
    CHECK_THAT(d.record.confidence, WithinAbs(d.geometry_confidence, 1e-12));
    CHECK(d.record.confidence <= 0.7);
    CHECK(d.record.confidence > 0.5);
    CHECK(d.glass_share < 0.0);
    CHECK(d.crossings == 0);
    const auto meta = nlohmann::json::parse(d.record.metadata);
    CHECK(meta.contains("fill"));
    CHECK(meta.at("crossings") == 0);
  }
  // The door with nothing in its swing has no visible leaf: state unknown.
  CHECK(found[0].record.state == "unknown");
  CHECK(found[1].record.state == "unknown");
  // Natural keys from the host and the rounded start.
  CHECK(found[0].record.guid == "openings/w-test/0.60");
}

TEST_CASE("DetectOpenings_InstanceAndLabels_RaiseConfidence",
          "[reconstruction][openings]") {
  const os::Hole win{2.2, 3.4, 0.9, 2.1};
  auto ev = wall_with({win});
  // The window's frame labelled 'window', as SAM3 + project leave it: the
  // face points within 10 cm around the hole.
  ev.classes.assign(ev.points.size(), OpeningClass::none);
  ev.instance.assign(ev.points.size(), -1);
  ev.instances.push_back({"inst-window-1", OpeningClass::window});
  for (std::size_t i = 0; i < ev.points.size(); ++i) {
    const auto &p = ev.points[i];
    if (p.x() > win.s0 - 0.1 && p.x() < win.s1 + 0.1 && p.z() > win.z0 - 0.1 &&
        p.z() < win.z1 + 0.1 && std::abs(p.y()) < 0.1) {
      ev.classes[i] = OpeningClass::window;
      ev.instance[i] = 0;
    }
  }
  const auto found = run(ev);
  REQUIRE(found.size() == 1);
  const auto &d = found[0];
  CHECK(d.record.kind == "window");
  CHECK(d.record.source_instance_guid == "inst-window-1");
  CHECK(d.window_share > 0.2);
  CHECK(d.record.confidence > d.geometry_confidence + 0.2);
}

TEST_CASE("DetectOpenings_Glass_RaisesWindowConfidence",
          "[reconstruction][openings]") {
  auto ev = wall_with({{2.2, 3.4, 0.9, 2.1}});
  ev.glass = [](const core::WallRecord &, const OpeningRect &) { return 0.8; };
  const auto found = run(ev);
  REQUIRE(found.size() == 1);
  CHECK_THAT(found[0].glass_share, WithinAbs(0.8, 1e-12));
  CHECK_THAT(found[0].record.confidence,
             WithinAbs(found[0].geometry_confidence + 0.1, 1e-12));
  CHECK(nlohmann::json::parse(found[0].record.metadata).at("glass_share") ==
        0.8);
}

TEST_CASE("DetectOpenings_CameraPathThroughADoor_RaisesConfidence",
          "[reconstruction][openings]") {
  auto ev = wall_with({{0.6, 1.5, 0.0, 2.1}, {2.2, 3.4, 0.9, 2.1}});
  // Walk through the door (s ~ 1.0), then along the wall, then a jump
  // through the window that is longer than a step.
  ev.trajectory = {{1.0, 1.0, 1.4},  {1.0, 0.4, 1.4},  {1.05, -0.3, 1.4},
                   {1.1, -1.0, 1.4}, {2.8, -1.0, 1.4}, {2.8, 2.0, 1.4}};
  const auto found = run(ev);
  REQUIRE(found.size() == 2);
  CHECK(found[0].record.kind == "door");
  CHECK(found[0].crossings == 1);
  CHECK_THAT(found[0].record.confidence,
             WithinAbs(found[0].geometry_confidence + 0.1, 1e-12));
  CHECK(found[1].crossings == 0);

  const auto s =
      trajectory_crossings(ev.trajectory, os::wall(kLen, kH), 0.0, kH);
  REQUIRE(s.size() == 1);
  CHECK_THAT(s[0], WithinAbs(1.0 + 0.05 * 0.4 / 0.7, 1e-9));
}

TEST_CASE("DetectOpenings_OpenDoorLeaf_GivesStateAndSwing",
          "[reconstruction][openings]") {
  // A 0.9 m door hinged at s = 0.6, its leaf standing open at 90 deg on the
  // +y side (side a): a sheet from (0.6, 0.05) to (0.6, 0.95).
  auto ev = wall_with({{0.6, 1.5, 0.0, 2.1}});
  for (double v = 0.1; v < 0.9; v += 0.05)
    for (double z = 0.025; z < 2.05; z += 0.05)
      ev.points.emplace_back(0.62, 0.05 + v, z);
  const auto found = run(ev);
  REQUIRE(found.size() == 1);
  const auto &d = found[0];
  CHECK(d.record.kind == "door");
  CHECK(d.record.state == "open");
  REQUIRE(d.leaf.found);
  CHECK(d.leaf.hinge_at_start);
  CHECK(d.leaf.side == 1);
  CHECK(d.leaf.angle_deg > 70.0);
  const auto meta = nlohmann::json::parse(d.record.metadata);
  CHECK(meta.at("leaf").at("side") == "a");
  CHECK(meta.at("leaf").at("hinge") == "start");
  // Seen from side a, a leaf hinged at s_start has its hinge on the right.
  CHECK(meta.at("swing") == "right");
}

TEST_CASE("DoorLeaf_AWallRunningAwayFromTheJamb_IsNotALeaf",
          "[reconstruction][openings]") {
  // A perpendicular wall meets the face at the door's jamb and runs on 3 m:
  // its points line up like a leaf at 90 deg but continue past the width.
  std::vector<Eigen::Vector3d> pts;
  for (double v = 0.1; v < 3.0; v += 0.05)
    for (double z = 0.025; z < 2.05; z += 0.05)
      pts.emplace_back(0.6, 0.05 + v, z);
  const auto leaf =
      door_leaf(pts, os::wall(kLen, kH), rect(0.6, 1.5, 0.0, 2.1));
  CHECK_FALSE(leaf.found);
}

TEST_CASE("DetectOpenings_ExternalFlag_FollowsTheHostWall",
          "[reconstruction][openings]") {
  auto w = os::wall(kLen, kH);
  w.storey = 0;
  w.is_external = true;
  const auto found =
      detect_openings(wall_with({{2.2, 3.4, 0.9, 2.1}}), {w}, storey());
  REQUIRE(found.size() == 1);
  CHECK(found[0].record.is_external);
}

TEST_CASE("DetectOpenings_MisalignedEvidence_IsRejected",
          "[reconstruction][openings]") {
  auto ev = wall_with({});
  ev.classes.assign(3, OpeningClass::door);
  CHECK_THROWS_AS(run(ev), std::invalid_argument);
}

TEST_CASE("DetectOpenings_RoomFloorOffTheStoreyLevel_UsesTheLocalFloor",
          "[reconstruction][openings]") {
  // The room in front of this wall drifted 0.6 m up from the storey's
  // level frame: its floor at z = 0.6, ceiling at 3.1 (storey: 0 .. 2.5 +
  // margin). The door still stands on the floor and is a door.
  OpeningEvidence ev;
  std::vector<Eigen::Vector3d> pts;
  os::add_face(pts, 0.05, kLen, kH, {{2.0, 2.9, 0.0, 2.1}});
  os::add_face(pts, -0.05, kLen, kH, {{2.0, 2.9, 0.0, 2.1}});
  os::add_floor_ceiling(pts, kLen, -3.0, 3.0, kH);
  for (auto &p : pts)
    ev.points.emplace_back(p.x(), p.y(), p.z() + 0.6);
  auto w = os::wall(kLen, kH);
  auto st = storey();
  st.z_ceiling = 3.2;
  const auto lv = face_levels(ev.points, w, st.z_floor, st.z_ceiling);
  CHECK(lv.floor_measured);
  CHECK_THAT(lv.z_floor, WithinAbs(0.61, 0.02));
  CHECK_THAT(lv.z_ceiling, WithinAbs(3.09, 0.02));
  const auto found = detect_openings(ev, {w}, st);
  REQUIRE(found.size() == 1);
  CHECK(found[0].record.kind == "door");
  CHECK_THAT(found[0].record.z_sill, WithinAbs(0.61, 0.02));
  CHECK_THAT(found[0].record.z_head, WithinAbs(2.7, 0.11));
}

TEST_CASE("DetectOpenings_CameraWalksThroughAWindow_ItIsADoor",
          "[reconstruction][openings]") {
  // A doorway whose lower 0.6 m is hidden on the face (a chair standing in
  // it, both sides seen): geometry alone says a window with a 0.6 m sill...
  auto ev = wall_with({{2.0, 2.9, 0.6, 2.1}});
  {
    const auto found = run(ev);
    REQUIRE(found.size() == 1);
    CHECK(found[0].record.kind == "window");
  }
  // ...but the camera walked through it: it reaches the floor, a door.
  ev.trajectory = {{2.4, 1.0, 1.4}, {2.45, 0.2, 1.4}, {2.5, -0.6, 1.4}};
  const auto found = run(ev);
  REQUIRE(found.size() == 1);
  CHECK(found[0].crossings == 1);
  CHECK(found[0].record.kind == "door");
  CHECK_THAT(found[0].record.z_sill, WithinAbs(0.01, 0.02));
  CHECK(nlohmann::json::parse(found[0].record.metadata)
            .at("sill_from_camera_path") == true);
}
