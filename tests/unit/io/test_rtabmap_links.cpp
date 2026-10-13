// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// RTABMap link -> pose_graph_edges conversion (spec §4.2). Pure: no rtabmap
// headers, so it is unit-testable without an RTABMap database.

#include <catch2/catch_test_macros.hpp>

#include <reusex/core/pose_graph.hpp>
#include <reusex/io/rtabmap_links.hpp>

#include <Eigen/Geometry>

#include <map>

namespace io = reusex::io;
namespace pg = reusex::core::pose_graph;

namespace {

Eigen::Matrix4d make_pose(double yaw, double x, double y) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T(0, 3) = x;
  T(1, 3) = y;
  return T;
}

std::array<double, 36> rtabmap_diag(double tx, double ty, double tz, double rr,
                                    double rp, double ry) {
  std::array<double, 36> a{};
  a[0] = tx;
  a[7] = ty;
  a[14] = tz;
  a[21] = rr;
  a[28] = rp;
  a[35] = ry;
  return a;
}

io::RtabmapLink link(int from, int to, int type, const Eigen::Matrix4d &T) {
  io::RtabmapLink l;
  l.from = from;
  l.to = to;
  l.type = type;
  l.transform = pg::row_major_from_matrix(T);
  l.information = rtabmap_diag(100, 100, 100, 400, 400, 400);
  return l;
}

} // namespace

TEST_CASE("RtabmapTransformToArray_3x4Floats_RowMajorWithBottomRow",
          "[io][rtabmap]") {
  const float d[12] = {1, 0, 0, 1.5f, 0, 1, 0, -2.0f, 0, 0, 1, 0.25f};
  const auto a = io::rtabmap_transform_to_array(d);
  CHECK(a[3] == 1.5);
  CHECK(a[7] == -2.0);
  CHECK(a[11] == 0.25);
  CHECK(a[12] == 0.0);
  CHECK(a[15] == 1.0);
}

TEST_CASE("RtabmapInformation_PermutesToGtsamOrder", "[io][rtabmap]") {
  auto in = rtabmap_diag(1, 2, 3, 4, 5, 6);
  in[0 * 6 + 3] = in[3 * 6 + 0] = 0.5; // (x, roll) coupling
  const auto out = io::rtabmap_information_to_gtsam(in);
  CHECK(out[0] == 4.0);         // rx <- roll
  CHECK(out[14] == 6.0);        // rz <- yaw
  CHECK(out[21] == 1.0);        // tx <- x
  CHECK(out[35] == 3.0);        // tz <- z
  CHECK(out[0 * 6 + 3] == 0.5); // (rx, tx) <- (roll, x)
  CHECK(out[3 * 6 + 0] == 0.5);
}

TEST_CASE("RtabmapLink_Neighbor_BecomesOdometryInTheStoredConvention",
          "[io][rtabmap]") {
  const Eigen::Matrix4d P1 = make_pose(0.3, 1.0, 2.0);
  const Eigen::Matrix4d P2 = make_pose(0.5, 1.4, 2.1);
  const std::map<int, std::array<double, 16>> imported = {
      {1, pg::row_major_from_matrix(P1)}, {2, pg::row_major_from_matrix(P2)}};
  const auto c = io::convert_rtabmap_link(
      link(1, 2, io::rtabmap_link_type::neighbor, P1.inverse() * P2),
      /*id_offset=*/100, imported);
  REQUIRE(c.edge.has_value());
  CHECK(c.skip == io::RtabmapLinkSkip::none);
  CHECK(c.edge->from_node_id == 101);
  CHECK(c.edge->to_node_id == 102);
  CHECK(c.edge->edge_type == "odometry");
  CHECK(c.edge->source == "rtabmap");
  CHECK(c.edge->status == "auto");
  CHECK(*c.edge->seed_disagreement_m < 1e-9); // stored == imported poses
  REQUIRE(c.edge->information.has_value());
  CHECK((*c.edge->information)[0] == 400.0); // rotation first
}

TEST_CASE("RtabmapLink_LoopClosureTypes_BecomeLoopClosure", "[io][rtabmap]") {
  const std::map<int, std::array<double, 16>> imported = {
      {3, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())},
      {9, pg::row_major_from_matrix(make_pose(0, 3, 0))}};
  for (int type : {io::rtabmap_link_type::global_closure,
                   io::rtabmap_link_type::local_space_closure,
                   io::rtabmap_link_type::local_time_closure,
                   io::rtabmap_link_type::user_closure}) {
    INFO(type);
    // RTABMap stores loop closures newest -> oldest; direction is kept here
    // and canonicalised by ProjectDB::upsert_pose_graph_edge.
    const auto c = io::convert_rtabmap_link(
        link(9, 3, type, make_pose(0, -3, 0)), 0, imported);
    REQUIRE(c.edge.has_value());
    CHECK(c.edge->edge_type == "loop_closure");
    CHECK(c.edge->from_node_id == 9);
  }
}

TEST_CASE("RtabmapLink_UnsupportedOrDangling_IsSkippedWithAReason",
          "[io][rtabmap]") {
  const std::map<int, std::array<double, 16>> imported = {
      {1, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())},
      {2, pg::row_major_from_matrix(Eigen::Matrix4d::Identity())}};
  const auto I = Eigen::Matrix4d::Identity();
  CHECK(io::convert_rtabmap_link(
            link(1, 2, io::rtabmap_link_type::virtual_closure, I), 0, imported)
            .skip == io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(
            link(1, -4, io::rtabmap_link_type::landmark, I), 0, imported)
            .skip == io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(link(1, 1, io::rtabmap_link_type::gravity, I),
                                 0, imported)
            .skip == io::RtabmapLinkSkip::unsupported_type);
  CHECK(io::convert_rtabmap_link(link(2, 2, io::rtabmap_link_type::neighbor, I),
                                 0, imported)
            .skip == io::RtabmapLinkSkip::self_link);
  CHECK(io::convert_rtabmap_link(link(1, 7, io::rtabmap_link_type::neighbor, I),
                                 0, imported)
            .skip == io::RtabmapLinkSkip::endpoint_not_imported);
  auto broken = link(1, 2, io::rtabmap_link_type::neighbor, I);
  broken.transform = {};
  CHECK(io::convert_rtabmap_link(broken, 0, imported).skip ==
        io::RtabmapLinkSkip::invalid_transform);
}
