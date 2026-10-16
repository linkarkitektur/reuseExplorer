// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The parameter descriptor table (#305) exists so a front end can build a
// stage form without re-typing the library defaults. That is only true while
// the table and the option structs agree, and nothing about the C++ forces
// them to: `stage_parameters.cpp` reads the struct fields, but a future knob
// could be added to a struct and forgotten here, or a descriptor could be
// left pointing at a field that was renamed.
//
// So the correspondence is asserted here, field by field, against
// default-constructed option structs — the same values `run_stage()` falls
// back to when a key is absent.

#include <catch2/catch_test_macros.hpp>

#include <reusex/pipeline/mesh_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/pipeline/stages.hpp>
#include <reusex/reconstruction/mesh.hpp>
#include <reusex/segmentation/band_rooms.hpp>
#include <reusex/segmentation/reconstruct.hpp>
#include <reusex/segmentation/segment_instances.hpp>
#include <reusex/segmentation/segment_planes.hpp>
#include <reusex/segmentation/segment_rooms.hpp>

#include <set>
#include <string>
#include <string_view>
#include <variant>

using namespace reusex::pipeline;

namespace {

/// Returns a pointer, never null: the descriptor lives in the table
/// `stage_parameters()` hands back a reference to, so it outlives the caller.
///
/// A pointer rather than a reference on purpose. Returning `const T&` from a
/// function whose arguments include a temporary makes GCC's
/// `-Wdangling-reference` assume the result may alias that temporary, which it
/// cannot here. Handing back a pointer states the non-owning relationship
/// outright instead of arguing with the heuristic.
const ParameterDescriptor *parameter_of(JobStage stage, std::string_view key) {
  for (const auto &parameter : stage_parameters(stage))
    if (parameter.key == key)
      return &parameter;
  FAIL("no descriptor for parameter '" << key << "'");
  throw std::logic_error("unreachable");
}

double number_default(JobStage stage, const std::string &key) {
  const ParameterDescriptor &parameter = *parameter_of(stage, key);
  REQUIRE(std::holds_alternative<double>(parameter.default_value));
  return std::get<double>(parameter.default_value);
}

/// The wire value narrowed back to the `float` the option struct holds.
///
/// The table publishes the shortest decimal that round-trips the float, so the
/// wire carries 0.05 rather than 0.05000000074505806. Comparing that double
/// against `static_cast<double>(0.05F)` would fail for the right reason and
/// prove nothing; narrowing back is the property that actually matters — a
/// caller echoing the published default reproduces the struct's value exactly.
float float_default(JobStage stage, const std::string &key) {
  return static_cast<float>(number_default(stage, key));
}

long long integer_default(JobStage stage, const std::string &key) {
  const ParameterDescriptor &parameter = *parameter_of(stage, key);
  REQUIRE(std::holds_alternative<long long>(parameter.default_value));
  return std::get<long long>(parameter.default_value);
}

} // namespace

TEST_CASE("StageParameters_EveryRunnableStage_DescribesWellFormedParameters",
          "[pipeline][params]") {
  for (const auto &name : job_stage_names()) {
    const auto stage = parse_job_stage(name);
    REQUIRE(stage.has_value());
    INFO("stage: " << name);

    const auto &table = stage_parameters(*stage);
    CHECK_FALSE(table.empty());

    std::set<std::string> keys;
    for (const auto &parameter : table) {
      INFO("parameter: " << parameter.key);
      CHECK_FALSE(parameter.key.empty());
      CHECK_FALSE(parameter.label.empty());
      CHECK_FALSE(parameter.description.empty());
      // A duplicate key would make the form ambiguous and the last writer win.
      CHECK(keys.insert(parameter.key).second);

      // A bound that excludes the default would make the form reject its own
      // starting state.
      if (const auto *value = std::get_if<double>(&parameter.default_value)) {
        if (parameter.minimum)
          CHECK(*value >= *parameter.minimum);
        if (parameter.maximum)
          CHECK(*value <= *parameter.maximum);
      }
      if (const auto *value =
              std::get_if<long long>(&parameter.default_value)) {
        if (parameter.minimum)
          CHECK(static_cast<double>(*value) >= *parameter.minimum);
        if (parameter.maximum)
          CHECK(static_cast<double>(*value) <= *parameter.maximum);
      }

      // The declared wire type and the stored default must agree, or a client
      // trusting `type` will send something the runner cannot parse.
      switch (parameter.type) {
      case ParameterType::number:
        CHECK(std::holds_alternative<double>(parameter.default_value));
        break;
      case ParameterType::integer:
        CHECK(std::holds_alternative<long long>(parameter.default_value));
        break;
      case ParameterType::boolean:
        CHECK(std::holds_alternative<bool>(parameter.default_value));
        break;
      case ParameterType::string:
        CHECK(
            (std::holds_alternative<std::string>(parameter.default_value) ||
             std::holds_alternative<std::monostate>(parameter.default_value)));
        break;
      case ParameterType::integer_list:
      case ParameterType::string_list:
        // A list has no scalar default; absent means "no restriction".
        CHECK(std::holds_alternative<std::monostate>(parameter.default_value));
        break;
      }
    }
  }
}

TEST_CASE("StageParameters_Clouds_MirrorReconstructionParamsDefaults",
          "[pipeline][params]") {
  const reusex::geometry::ReconstructionParams d{};
  CHECK(float_default(JobStage::clouds, "resolution") == d.resolution);
  CHECK(float_default(JobStage::clouds, "min_distance") == d.min_distance);
  CHECK(float_default(JobStage::clouds, "max_distance") == d.max_distance);
  CHECK(integer_default(JobStage::clouds, "sampling_factor") ==
        d.sampling_factor);
  CHECK(integer_default(JobStage::clouds, "confidence_threshold") ==
        d.confidence_threshold);
}

TEST_CASE("StageParameters_Planes_MirrorSegmentPlanesOptionsDefaults",
          "[pipeline][params]") {
  const reusex::geometry::SegmentPlanesOptions d{};
  CHECK(float_default(JobStage::planes, "angle_threshold") ==
        d.angle_threshold);
  CHECK(float_default(JobStage::planes, "plane_dist_threshold") ==
        d.plane_dist_threshold);
  CHECK(integer_default(JobStage::planes, "min_inliers") == d.min_inliers);
  CHECK(float_default(JobStage::planes, "radius") == d.radius);
  CHECK(float_default(JobStage::planes, "interval_0") == d.interval_0);
  CHECK(float_default(JobStage::planes, "interval_factor") ==
        d.interval_factor);
  CHECK(integer_default(JobStage::planes, "noise_seed") ==
        static_cast<long long>(d.noise_seed));

  const ParameterDescriptor &adaptive =
      *parameter_of(JobStage::planes, "adaptive");
  REQUIRE(std::holds_alternative<bool>(adaptive.default_value));
  CHECK(std::get<bool>(adaptive.default_value) == d.adaptive);

  // #214: presence pins the parameter. A form echoing an untouched default
  // back would silently switch adaptivity off for it, so the flag has to be
  // on the wire — and only on these two.
  for (const auto &parameter : stage_parameters(JobStage::planes)) {
    const bool expected = parameter.key == "plane_dist_threshold" ||
                          parameter.key == "min_inliers";
    INFO("parameter: " << parameter.key);
    CHECK(parameter.presence_sensitive == expected);
  }
}

TEST_CASE("StageParameters_Rooms_MirrorSegmentRoomsOptionsDefaults",
          "[pipeline][params]") {
  const reusex::geometry::SegmentRoomsOptions d{};
  CHECK(float_default(JobStage::rooms, "grid_size") == d.grid_size);
  CHECK(float_default(JobStage::rooms, "resolution") == d.resolution);
  CHECK(float_default(JobStage::rooms, "beta") == d.beta);
  CHECK(integer_default(JobStage::rooms, "max_iter") == d.max_iter);
  CHECK(integer_default(JobStage::rooms, "propagate_k") == d.propagate_k);
  CHECK(float_default(JobStage::rooms, "propagate_max_radius") ==
        d.propagate_max_radius);
  CHECK(float_default(JobStage::rooms, "min_room_area") == d.min_room_area);
  CHECK(float_default(JobStage::rooms, "merge_affinity") == d.merge_affinity);
}

TEST_CASE("StageParameters_Rooms_MirrorBandRoomsOptionsDefaults",
          "[pipeline][stage_parameters]") {
  const reusex::geometry::BandRoomsOptions b{};
  const auto &table = stage_parameters(JobStage::rooms);
  auto find = [&](std::string_view key) -> const ParameterDescriptor & {
    for (const auto &p : table)
      if (p.key == key)
        return p;
    FAIL("rooms has no parameter " << key);
    return table.front();
  };
  CHECK(std::get<std::string>(find("method").default_value) ==
        reusex::geometry::kDefaultRoomsMethod);
  CHECK(std::get<double>(find("band_bottom").default_value) == b.band_bottom);
  CHECK(std::get<double>(find("band_top").default_value) == b.band_top);
  CHECK(std::get<double>(find("band_cell_size").default_value) == b.cell_size);
  CHECK(std::get<double>(find("band_core_radius").default_value) ==
        b.core_radius);
  CHECK(std::get<double>(find("band_min_room_area").default_value) ==
        b.min_room_area);
  CHECK(std::get<long long>(find("band_wall_min_points").default_value) ==
        b.wall_min_points);
  CHECK(std::get<double>(find("band_footprint_close").default_value) ==
        b.footprint_close);
}

TEST_CASE("StageParameters_Instances_MirrorSegmentInstancesRequestDefaults",
          "[pipeline][params]") {
  const reusex::geometry::SegmentInstancesRequest d{};
  CHECK(float_default(JobStage::instances, "cluster_tolerance") ==
        d.cluster_tolerance);
  CHECK(integer_default(JobStage::instances, "min_cluster_size") ==
        d.min_cluster_size);
  CHECK(integer_default(JobStage::instances, "max_cluster_size") ==
        d.max_cluster_size);

  // These two have no option-struct home — they name clouds, not algorithm
  // knobs — so the constants are shared with run_instances() instead.
  const ParameterDescriptor &semantic =
      *parameter_of(JobStage::instances, "semantic_cloud");
  REQUIRE(std::holds_alternative<std::string>(semantic.default_value));
  CHECK(std::get<std::string>(semantic.default_value) ==
        std::string(kDefaultSemanticCloud));

  const ParameterDescriptor &output =
      *parameter_of(JobStage::instances, "output_cloud");
  REQUIRE(std::holds_alternative<std::string>(output.default_value));
  CHECK(std::get<std::string>(output.default_value) ==
        std::string(kDefaultInstanceCloud));
}

TEST_CASE("StageParameters_Mesh_MirrorMeshOptionsDefaults",
          "[pipeline][params]") {
  const reusex::geometry::MeshOptions d{};
  CHECK(float_default(JobStage::mesh, "search_threshold") ==
        d.search_threshold);
  CHECK(float_default(JobStage::mesh, "new_plane_offset") ==
        d.new_plane_offset);
  CHECK(number_default(JobStage::mesh, "time_limit_seconds") ==
        d.time_limit_seconds);
  CHECK(number_default(JobStage::mesh, "alpha") == d.alpha);
  CHECK(integer_default(JobStage::mesh, "max_cells") ==
        static_cast<long long>(d.max_cells));
  CHECK(integer_default(JobStage::mesh, "sectioned_threshold") ==
        static_cast<long long>(d.sectioned_threshold));
  // cuOpt size guard (reusex-81y).
  CHECK(integer_default(JobStage::mesh, "cuopt_max_variables") ==
        static_cast<long long>(d.cuopt_max_variables));
  // Structural plane selection (reusex-8l3).
  CHECK(float_default(JobStage::mesh, "structural_min_wall_area") ==
        static_cast<float>(d.structural.min_wall_area_m2));
  CHECK(float_default(JobStage::mesh, "structural_collinear_offset") ==
        static_cast<float>(d.structural.collinear_offset_m));
  CHECK(float_default(JobStage::mesh, "structural_storey_clearance") ==
        static_cast<float>(d.structural.min_storey_clearance_m));
  CHECK(float_default(JobStage::mesh, "structural_footprint_overlap") ==
        static_cast<float>(d.structural.min_footprint_overlap));
  const ParameterDescriptor &structural =
      *parameter_of(JobStage::mesh, "structural_filter");
  REQUIRE(std::holds_alternative<bool>(structural.default_value));
  CHECK(std::get<bool>(structural.default_value) == d.structural.enabled);

  const ParameterDescriptor &sectioned =
      *parameter_of(JobStage::mesh, "sectioned");
  REQUIRE(std::holds_alternative<bool>(sectioned.default_value));
  CHECK(std::get<bool>(sectioned.default_value) == d.sectioned);

  // Storey sections (reusex-z1v.3.4) and storey-local walls (reusex-sba).
  const ParameterDescriptor &storey_sections =
      *parameter_of(JobStage::mesh, "storey_sections");
  REQUIRE(std::holds_alternative<bool>(storey_sections.default_value));
  CHECK(std::get<bool>(storey_sections.default_value) ==
        reusex::pipeline::MeshStageOptions{}.use_storey_sections);
  const ParameterDescriptor &storey_local =
      *parameter_of(JobStage::mesh, "storey_local");
  REQUIRE(std::holds_alternative<bool>(storey_local.default_value));
  CHECK(std::get<bool>(storey_local.default_value) == d.storey_local);

  const ParameterDescriptor &solver = *parameter_of(JobStage::mesh, "solver");
  REQUIRE(std::holds_alternative<std::string>(solver.default_value));
  CHECK(std::get<std::string>(solver.default_value) == "auto");

  const ParameterDescriptor &output_name =
      *parameter_of(JobStage::mesh, "output_name");
  REQUIRE(std::holds_alternative<std::string>(output_name.default_value));
  CHECK(std::get<std::string>(output_name.default_value) == "mesh");
}

TEST_CASE("StageParameters_Annotate_DescribesWireContract",
          "[pipeline][params]") {
  // No option-struct home for this one (annotate_parameters() is hard-coded,
  // like optimize_parameters()), so this asserts the literal wire contract
  // from docs/design/gpu-worker.md §3 (ruxd repo) instead of a struct.
  const ParameterDescriptor &prompts =
      *parameter_of(JobStage::annotate, "prompts");
  CHECK(prompts.type == ParameterType::string_list);
  CHECK(std::holds_alternative<std::monostate>(prompts.default_value));
  // ruxd refuses a job without prompts (or with none): there is no default
  // class set, and the descriptor says so (gpu-worker re-review N8).
  CHECK(prompts.required);

  const ParameterDescriptor &frames =
      *parameter_of(JobStage::annotate, "frames");
  CHECK(frames.type == ParameterType::integer_list);
  CHECK(std::holds_alternative<std::monostate>(frames.default_value));
  CHECK_FALSE(frames.required);

  CHECK(number_default(JobStage::annotate, "confidence") == 0.5);
  const ParameterDescriptor &confidence =
      *parameter_of(JobStage::annotate, "confidence");
  REQUIRE(confidence.minimum);
  REQUIRE(confidence.maximum);
  CHECK(*confidence.minimum == 0.0);
  CHECK(*confidence.maximum == 1.0);

  const ParameterDescriptor &skip_annotated =
      *parameter_of(JobStage::annotate, "skip_annotated");
  REQUIRE(std::holds_alternative<bool>(skip_annotated.default_value));
  CHECK_FALSE(std::get<bool>(skip_annotated.default_value));

  const ParameterDescriptor &placement =
      *parameter_of(JobStage::annotate, "placement");
  REQUIRE(std::holds_alternative<std::string>(placement.default_value));
  CHECK(std::get<std::string>(placement.default_value) == "auto");
}

TEST_CASE("StageParameters_FloatDefault_RoundTripsShortestDecimal",
          "[pipeline][params]") {
  // static_cast<double>(0.05F) is 0.05000000074505806, which is what a JSON
  // encoder would faithfully print into a form field. The table goes through
  // the shortest round-trip decimal of the float instead.
  const auto resolution = number_default(JobStage::clouds, "resolution");
  CHECK(resolution == 0.05);
  CHECK(number_default(JobStage::rooms, "beta") == 0.01);
  CHECK(number_default(JobStage::planes, "plane_dist_threshold") == 0.07);
}

TEST_CASE("StageParameters_OnlyAnnotatePromptsAreRequired",
          "[pipeline][params]") {
  // `required` means "no job without it": so far only annotate's prompts.
  for (const auto &name : job_stage_names()) {
    const auto stage = parse_job_stage(name);
    REQUIRE(stage);
    for (const auto &d : stage_parameters(*stage))
      CHECK(d.required == (*stage == JobStage::annotate && d.key == "prompts"));
  }
}
