// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The graph_discover stage JSON -> GraphDiscoverOptions, and the descriptor
// table's hard-coded defaults pinned against the struct (STANDARDS §4).

#include <catch2/catch_test_macros.hpp>

#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/slam/discover_parameters.hpp>
#include <reusex/slam/graph_discover.hpp>

#include <stdexcept>
#include <string>
#include <variant>

namespace slam = reusex::slam;
namespace pl = reusex::pipeline;

namespace {
const pl::ParameterDescriptor &descriptor(const std::string &key) {
  for (const auto &d : pl::stage_parameters(pl::JobStage::graph_discover))
    if (d.key == key)
      return d;
  throw std::logic_error("no descriptor " + key);
}
} // namespace

TEST_CASE("DiscoverParameters_Empty_KeepsTheDefaults",
          "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  slam::apply_discover_parameters(o, "");
  CHECK(o == slam::GraphDiscoverOptions{});
}

TEST_CASE("DiscoverParameters_Keys_Applied", "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  slam::apply_discover_parameters(o,
                                  R"({"loop_closure":false,"use_panoramas":true,
             "loop_edges_file":"e.json","loop_proposal":"spatial",
             "loop_min_frame_gap":7,"loop_min_inliers":30,"pcm":false,
             "dry_run":true})");
  CHECK_FALSE(o.candidates.loop_closure.enable);
  CHECK(o.candidates.panorama_loops.enable);
  CHECK(o.candidates.loop_edges_file == "e.json");
  CHECK(o.candidates.loop_closure.proposal ==
        reusex::geometry::LoopProposal::spatial);
  CHECK(o.candidates.loop_closure.min_frame_gap == 7);
  CHECK(o.candidates.loop_closure.min_match_inliers == 30);
  CHECK_FALSE(o.candidates.loop_closure.pcm);
  CHECK(o.dry_run);
}

TEST_CASE("DiscoverParameters_BadInput_InvalidArgument",
          "[slam][discover_parameters]") {
  slam::GraphDiscoverOptions o;
  for (const char *bad : {"[1]", "{oops", R"({"loop_min_inliers":"many"})",
                          R"({"loop_proposal":"psychic"})"}) {
    INFO(bad);
    CHECK_THROWS_AS(slam::apply_discover_parameters(o, bad),
                    std::invalid_argument);
  }
}

TEST_CASE("DiscoverParameters_DescriptorDefaults_MirrorTheStruct",
          "[slam][discover_parameters]") {
  const slam::GraphDiscoverOptions d;
  const auto &lc = d.candidates.loop_closure;
  CHECK(std::get<bool>(descriptor("loop_closure").default_value) == lc.enable);
  CHECK(std::get<bool>(descriptor("use_panoramas").default_value) ==
        d.candidates.panorama_loops.enable);
  CHECK(std::get<std::string>(descriptor("loop_edges_file").default_value) ==
        d.candidates.loop_edges_file);
  CHECK(std::get<std::string>(descriptor("loop_proposal").default_value) ==
        std::string(slam::to_string(lc.proposal)));
  CHECK(std::get<long long>(descriptor("loop_min_frame_gap").default_value) ==
        lc.min_frame_gap);
  CHECK(std::get<long long>(descriptor("loop_min_inliers").default_value) ==
        lc.min_match_inliers);
  CHECK(std::get<bool>(descriptor("pcm").default_value) == lc.pcm);
  CHECK(std::get<bool>(descriptor("dry_run").default_value) == d.dry_run);
}
