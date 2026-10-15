// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/segmentation/depth_filters.hpp>

#include <opencv2/core.hpp>

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

namespace py = pybind11;

// ── Per-frame reconstruction helper ──────────────────────────────────────────
// Must be called with the GIL already released (or before it is acquired).

namespace {

struct FrameCloud {
  int fid{};
  std::vector<std::array<float, 3>> pts;
  std::vector<std::array<uint8_t, 3>> cols;
};

void reconstruct_single(const reusex::ProjectDB &db, int fid, int step,
                        float min_dist, float max_dist, int conf_thresh,
                        FrameCloud &out) {
  out.fid = fid;

  cv::Mat color = db.sensor_frame_image(fid);
  cv::Mat depth16 = db.sensor_frame_depth(fid);
  cv::Mat confidence = db.sensor_frame_confidence(fid);
  if (color.empty() || depth16.empty())
    return;

  auto intr = db.sensor_frame_intrinsics(fid);
  if (intr.width <= 0 || intr.height <= 0)
    return;

  cv::Mat depth_f;
  depth16.convertTo(depth_f, CV_32FC1, 1.0 / 1000.0);
  reusex::geometry::apply_depth_discontinuity_filter(depth_f, confidence, 0.5f);
  reusex::geometry::apply_ray_consistency_filter(depth_f, confidence, 0.2f);

  const float fx = static_cast<float>(
      intr.fx * static_cast<double>(depth_f.cols) / std::max(1, intr.width));
  const float fy = static_cast<float>(
      intr.fy * static_cast<double>(depth_f.rows) / std::max(1, intr.height));
  const float cx = static_cast<float>(
      intr.cx * static_cast<double>(depth_f.cols) / std::max(1, intr.width));
  const float cy = static_cast<float>(
      intr.cy * static_cast<double>(depth_f.rows) / std::max(1, intr.height));

  out.pts.reserve(
      static_cast<size_t>((depth_f.rows / step) * (depth_f.cols / step)));
  const bool has_conf = !confidence.empty();

  for (int v = 0; v < depth_f.rows; v += step) {
    for (int u = 0; u < depth_f.cols; u += step) {
      const float z = depth_f.at<float>(v, u);
      if (z <= 0.0f || z < min_dist || z > max_dist)
        continue;
      if (has_conf &&
          confidence.at<uint8_t>(v, u) < static_cast<uint8_t>(conf_thresh))
        continue;

      const float x = (static_cast<float>(u) - cx) * z / fx;
      const float y = (static_cast<float>(v) - cy) * z / fy;

      const int cu =
          std::clamp(u * color.cols / depth_f.cols, 0, color.cols - 1);
      const int cv_ =
          std::clamp(v * color.rows / depth_f.rows, 0, color.rows - 1);
      const auto &px = color.at<cv::Vec3b>(cv_, cu);

      out.pts.push_back({x, y, z});
      out.cols.push_back({px[2], px[1], px[0]}); // BGR → RGB
    }
  }
}

// Build a pybind11 dict from a FrameCloud (must hold GIL).
py::dict frame_cloud_to_dict(const FrameCloud &fc) {
  std::vector<ssize_t> empty_shape{ssize_t{0}, ssize_t{3}};
  const ssize_t n = static_cast<ssize_t>(fc.pts.size());

  auto positions = n > 0 ? py::array_t<float>({n, ssize_t{3}})
                         : py::array_t<float>(empty_shape);
  auto colors_out = n > 0 ? py::array_t<uint8_t>({n, ssize_t{3}})
                          : py::array_t<uint8_t>(empty_shape);

  if (n > 0) {
    auto pos = positions.mutable_unchecked<2>();
    auto col = colors_out.mutable_unchecked<2>();
    for (ssize_t i = 0; i < n; ++i) {
      pos(i, 0) = fc.pts[static_cast<size_t>(i)][0];
      pos(i, 1) = fc.pts[static_cast<size_t>(i)][1];
      pos(i, 2) = fc.pts[static_cast<size_t>(i)][2];
      col(i, 0) = fc.cols[static_cast<size_t>(i)][0];
      col(i, 1) = fc.cols[static_cast<size_t>(i)][1];
      col(i, 2) = fc.cols[static_cast<size_t>(i)][2];
    }
  }

  py::dict result;
  result["positions"] = positions;
  result["colors"] = colors_out;
  return result;
}

} // namespace

// ─────────────────────────────────────────────────────────────────────────────

PYBIND11_MODULE(_reusex, m) {
  m.doc() =
      "ReUseX Python bindings — read-only access to .rux project databases";

  using PDB = reusex::ProjectDB;
  using Summary = PDB::ProjectSummary;

  // --- ProjectSummary nested structs (read-only) ---

  py::class_<Summary::ProjectInfo>(m, "ProjectInfo")
      .def_readonly("id", &Summary::ProjectInfo::id)
      .def_readonly("name", &Summary::ProjectInfo::name)
      .def_readonly("building_address", &Summary::ProjectInfo::building_address)
      .def_readonly("year_of_construction",
                    &Summary::ProjectInfo::year_of_construction)
      .def_readonly("survey_date", &Summary::ProjectInfo::survey_date)
      .def_readonly("survey_organisation",
                    &Summary::ProjectInfo::survey_organisation)
      .def_readonly("notes", &Summary::ProjectInfo::notes)
      .def("__repr__", [](const Summary::ProjectInfo &p) {
        return "<ProjectInfo name='" + p.name + "'>";
      });

  py::class_<Summary::CloudInfo>(m, "CloudInfo")
      .def_readonly("name", &Summary::CloudInfo::name)
      .def_readonly("type", &Summary::CloudInfo::type)
      .def_readonly("point_count", &Summary::CloudInfo::point_count)
      .def_readonly("width", &Summary::CloudInfo::width)
      .def_readonly("height", &Summary::CloudInfo::height)
      .def_readonly("organized", &Summary::CloudInfo::organized)
      .def_readonly("labels", &Summary::CloudInfo::labels)
      .def("__repr__", [](const Summary::CloudInfo &c) {
        return "<CloudInfo name='" + c.name + "' type='" + c.type +
               "' points=" + std::to_string(c.point_count) + ">";
      });

  py::class_<Summary::MeshInfo>(m, "MeshInfo")
      .def_readonly("name", &Summary::MeshInfo::name)
      .def_readonly("vertex_count", &Summary::MeshInfo::vertex_count)
      .def_readonly("face_count", &Summary::MeshInfo::face_count)
      .def("__repr__", [](const Summary::MeshInfo &mi) {
        return "<MeshInfo name='" + mi.name +
               "' vertices=" + std::to_string(mi.vertex_count) +
               " faces=" + std::to_string(mi.face_count) + ">";
      });

  py::class_<Summary::SensorFrameInfo>(m, "SensorFrameInfo")
      .def_readonly("total_count", &Summary::SensorFrameInfo::total_count)
      .def_readonly("width", &Summary::SensorFrameInfo::width)
      .def_readonly("height", &Summary::SensorFrameInfo::height)
      .def_readonly("segmented_count",
                    &Summary::SensorFrameInfo::segmented_count)
      .def("__repr__", [](const Summary::SensorFrameInfo &sf) {
        return "<SensorFrameInfo count=" + std::to_string(sf.total_count) +
               " segmented=" + std::to_string(sf.segmented_count) + ">";
      });

  py::class_<Summary::PanoramicInfo>(m, "PanoramicInfo")
      .def_readonly("total_count", &Summary::PanoramicInfo::total_count)
      .def_readonly("matched_count", &Summary::PanoramicInfo::matched_count)
      .def("__repr__", [](const Summary::PanoramicInfo &pi) {
        return "<PanoramicInfo count=" + std::to_string(pi.total_count) +
               " matched=" + std::to_string(pi.matched_count) + ">";
      });

  py::class_<Summary::ComponentInfo>(m, "ComponentInfo")
      .def_readonly("total_count", &Summary::ComponentInfo::total_count)
      .def_readonly("count_by_type", &Summary::ComponentInfo::count_by_type)
      .def("__repr__", [](const Summary::ComponentInfo &ci) {
        return "<ComponentInfo count=" + std::to_string(ci.total_count) + ">";
      });

  py::class_<Summary::MaterialInfo>(m, "MaterialInfo")
      .def_readonly("id", &Summary::MaterialInfo::id)
      .def_readonly("guid", &Summary::MaterialInfo::guid)
      .def_readonly("property_count", &Summary::MaterialInfo::property_count)
      .def_readonly("created_at", &Summary::MaterialInfo::created_at)
      .def_readonly("version_number", &Summary::MaterialInfo::version_number)
      .def("__repr__", [](const Summary::MaterialInfo &mi) {
        return "<MaterialInfo id='" + mi.id + "' guid='" + mi.guid + "'>";
      });

  // --- ProjectSummary ---

  py::class_<Summary>(m, "ProjectSummary")
      .def_readonly("path", &Summary::path)
      .def_readonly("schema_version", &Summary::schema_version)
      .def_readonly("projects", &Summary::projects)
      .def_readonly("clouds", &Summary::clouds)
      .def_readonly("meshes", &Summary::meshes)
      .def_readonly("sensor_frames", &Summary::sensor_frames)
      .def_readonly("panoramic_images", &Summary::panoramic_images)
      .def_readonly("components", &Summary::components)
      .def_readonly("materials", &Summary::materials)
      .def("__repr__", [](const Summary &s) {
        return "<ProjectSummary schema_v" + std::to_string(s.schema_version) +
               " clouds=" + std::to_string(s.clouds.size()) +
               " meshes=" + std::to_string(s.meshes.size()) + ">";
      });

  // --- PipelineLogEntry ---

  py::class_<PDB::PipelineLogEntry>(m, "PipelineLogEntry")
      .def_readonly("id", &PDB::PipelineLogEntry::id)
      .def_readonly("stage", &PDB::PipelineLogEntry::stage)
      .def_readonly("started_at", &PDB::PipelineLogEntry::started_at)
      .def_readonly("finished_at", &PDB::PipelineLogEntry::finished_at)
      .def_readonly("parameters", &PDB::PipelineLogEntry::parameters)
      .def_readonly("status", &PDB::PipelineLogEntry::status)
      .def_readonly("error_msg", &PDB::PipelineLogEntry::error_msg)
      .def("__repr__", [](const PDB::PipelineLogEntry &e) {
        return "<PipelineLogEntry id=" + std::to_string(e.id) + " stage='" +
               e.stage + "' status='" + e.status + "'>";
      });

  // --- ProjectDB ---

  py::class_<PDB>(m, "ProjectDB")
      .def(py::init<std::filesystem::path, bool>(), py::arg("path"),
           py::arg("read_only") = true,
           "Open a ReUseX project database.\n\n"
           "Args:\n"
           "    path: Path to the .rux database file\n"
           "    read_only: Open in read-only mode (default: True)")
      .def("is_open", &PDB::is_open, "Check if the database is open")
      .def("path", &PDB::path, "Get the database file path")
      .def("schema_version", &PDB::schema_version,
           "Get the database schema version")
      .def("project_summary", &PDB::project_summary,
           "Get a summary of all project data")
      .def("list_point_clouds", &PDB::list_point_clouds,
           "List names of all stored point clouds")
      .def("list_meshes", &PDB::list_meshes, "List names of all stored meshes")
      .def("list_building_components",
           py::overload_cast<>(&PDB::list_building_components, py::const_),
           "List names of all building components")
      .def("building_component_count", &PDB::building_component_count,
           "Get the total number of building components")
      .def("pipeline_log", &PDB::pipeline_log, py::arg("limit") = 0,
           "Get pipeline execution log entries.\n\n"
           "Args:\n"
           "    limit: Max entries to return (0 = all)")

      // --- Point cloud geometry loading (returns numpy arrays) ---

      .def(
          "point_cloud_xyzrgb",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_xyzrgb(name);
            const ssize_t n = cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto positions = py::array_t<float>({n, ssize_t{3}});
            auto colors = py::array_t<uint8_t>({n, ssize_t{3}});

            if (n > 0) {
              auto pos = positions.mutable_unchecked<2>();
              auto col = colors.mutable_unchecked<2>();
              for (ssize_t i = 0; i < n; ++i) {
                const auto &pt = (*cloud)[static_cast<size_t>(i)];
                pos(i, 0) = pt.x;
                pos(i, 1) = pt.y;
                pos(i, 2) = pt.z;
                col(i, 0) = pt.r;
                col(i, 1) = pt.g;
                col(i, 2) = pt.b;
              }
            }

            py::dict result;
            result["positions"] = positions;
            result["colors"] = colors;
            return result;
          },
          py::arg("name"),
          "Load a PointXYZRGB cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'positions': (N, 3) float32\n"
          "    'colors':    (N, 3) uint8 (RGB)")

      .def(
          "point_cloud_xyz",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_xyz(name);
            const ssize_t n = cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto positions = py::array_t<float>({n, ssize_t{3}});

            if (n > 0) {
              auto pos = positions.mutable_unchecked<2>();
              for (ssize_t i = 0; i < n; ++i) {
                const auto &pt = (*cloud)[static_cast<size_t>(i)];
                pos(i, 0) = pt.x;
                pos(i, 1) = pt.y;
                pos(i, 2) = pt.z;
              }
            }

            py::dict result;
            result["positions"] = positions;
            return result;
          },
          py::arg("name"),
          "Load a PointXYZ cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'positions': (N, 3) float32")

      .def(
          "point_cloud_label",
          [](const PDB &db, const std::string &name) {
            auto cloud = db.point_cloud_label(name);
            const ssize_t n = cloud ? static_cast<ssize_t>(cloud->size()) : 0;

            auto labels = py::array_t<int32_t>(n);

            if (n > 0) {
              auto lbl = labels.mutable_unchecked<1>();
              for (ssize_t i = 0; i < n; ++i)
                lbl(i) = static_cast<int32_t>(
                    (*cloud)[static_cast<size_t>(i)].label);
            }

            py::dict result;
            result["labels"] = labels;
            return result;
          },
          py::arg("name"),
          "Load a Label cloud as numpy arrays.\n\n"
          "Returns dict with:\n"
          "    'labels': (N,) int32 (point label per point; 0 = unlabeled;\n"
          "    for a semantic cloud, point label = API class + 1)")

      .def("label_definitions", &PDB::label_definitions, py::arg("name"),
           "Get label ID → name definitions for a cloud.\n\n"
           "Returns dict mapping int label_id to str label_name; label_id is\n"
           "the point label (1-based), as in point_cloud_label().")

      .def("sensor_frame_ids", &PDB::sensor_frame_ids,
           "Return list of all stored sensor frame IDs.")

      // --- Per-frame sensor data ---

      .def(
          "sensor_frame_intrinsics",
          [](const PDB &db, int nodeId) {
            auto intr = db.sensor_frame_intrinsics(nodeId);
            auto lt = py::array_t<double>({ssize_t{4}, ssize_t{4}});
            auto m = lt.mutable_unchecked<2>();
            for (int r = 0; r < 4; ++r)
              for (int c = 0; c < 4; ++c)
                m(r, c) = intr.local_transform[static_cast<size_t>(r * 4 + c)];
            py::dict result;
            result["fx"] = intr.fx;
            result["fy"] = intr.fy;
            result["cx"] = intr.cx;
            result["cy"] = intr.cy;
            result["width"] = intr.width;
            result["height"] = intr.height;
            result["local_transform"] = lt;
            return result;
          },
          py::arg("node_id"),
          "Return intrinsics for a sensor frame.\n\n"
          "Returns dict with fx, fy, cx, cy, width, height and\n"
          "    'local_transform': (4,4) float64 camera→sensor SE(3).")

      .def(
          "sensor_frame_pose",
          [](const PDB &db, int nodeId) {
            auto arr = db.sensor_frame_pose(nodeId);
            auto mat = py::array_t<double>({ssize_t{4}, ssize_t{4}});
            auto m = mat.mutable_unchecked<2>();
            for (int r = 0; r < 4; ++r)
              for (int c = 0; c < 4; ++c)
                m(r, c) = arr[static_cast<size_t>(r * 4 + c)];
            return mat;
          },
          py::arg("node_id"),
          "Return the (4,4) float64 SE(3) world pose for a frame.\n\n"
          "Verbatim: a frame with no stored pose reads back as identity, so\n"
          "call has_sensor_frame_pose() first when the difference matters.")

      .def("has_sensor_frame_pose", &PDB::has_sensor_frame_pose,
           py::arg("node_id"),
           "True when the frame carries a usable stored world pose.\n\n"
           "sensor_frame_pose() cannot answer this: it returns identity for a\n"
           "frame with no row, a NULL transform, or a wrong-sized blob, and\n"
           "hands back an all-zero or NaN transform verbatim. This checks the\n"
           "row exists and the blob is 16 finite doubles with bottom row\n"
           "[0,0,0,1] and a non-degenerate rotation (#336).")

      // --- Single-frame reconstruction ---

      .def(
          "reconstruct_frame",
          [](const PDB &db, int nodeId, int step, float min_dist,
             float max_dist, int conf_thresh) {
            FrameCloud fc;
            [&]() {
              py::gil_scoped_release release;
              reconstruct_single(db, nodeId, step, min_dist, max_dist,
                                 conf_thresh, fc);
            }();
            return frame_cloud_to_dict(fc);
          },
          py::arg("node_id"), py::arg("step") = 4,
          py::arg("min_distance") = 0.0f, py::arg("max_distance") = 4.0f,
          py::arg("confidence_threshold") = 2,
          "Reconstruct a single frame's point cloud in optical (camera) "
          "frame.\n\n"
          "Returns dict with 'positions' (N,3) float32 and 'colors' (N,3) "
          "uint8.\n"
          "Points are in optical frame — apply localTransform then worldPose\n"
          "to place them in world space (or use Blender's parent hierarchy).")

      // --- Parallel multi-frame reconstruction
      // -------------------------------- GIL is released exactly once for the
      // entire C++ parallel section. Each worker thread opens its own DB
      // connection (SQLite is not thread-safe with shared connections).  Python
      // only needs one background thread to call this; C++ handles all
      // parallelism internally.

      .def(
          "reconstruct_frames_parallel",
          [](const PDB &db, const std::vector<int> &nodeIds, int step,
             float min_dist, float max_dist, int conf_thresh) {
            const size_t n_frames = nodeIds.size();
            std::vector<FrameCloud> results(n_frames);

            {
              py::gil_scoped_release release; // released for the whole batch

              const auto db_path = db.path();
              const unsigned int hw = std::thread::hardware_concurrency();
              const int n_threads =
                  static_cast<int>(std::max(1u, std::min(hw, 8u)));

              std::atomic<size_t> counter{0};
              std::vector<std::thread> workers;
              workers.reserve(static_cast<size_t>(n_threads));

              for (int t = 0; t < n_threads; ++t) {
                workers.emplace_back([&]() {
                  PDB thread_db(db_path, /*read_only=*/true);
                  while (true) {
                    const size_t idx =
                        counter.fetch_add(1, std::memory_order_relaxed);
                    if (idx >= n_frames)
                      return;
                    reconstruct_single(thread_db, nodeIds[idx], step, min_dist,
                                       max_dist, conf_thresh, results[idx]);
                  }
                });
              }

              for (auto &w : workers)
                w.join();
            } // GIL re-acquired here

            // Build Python output with GIL held
            py::list out;
            out.attr("__len__"); // ensure list type is active (no-op touch)
            for (auto &fc : results)
              out.append(py::make_tuple(fc.fid, frame_cloud_to_dict(fc)));
            return out;
          },
          py::arg("node_ids"), py::arg("step") = 4,
          py::arg("min_distance") = 0.0f, py::arg("max_distance") = 4.0f,
          py::arg("confidence_threshold") = 2,
          "Reconstruct multiple frames in parallel using C++ threads.\n\n"
          "The GIL is released for the entire parallel C++ section; each "
          "thread\n"
          "opens its own DB connection.  Returns list of (fid, dict) tuples\n"
          "where dict has 'positions' (N,3) float32 and 'colors' (N,3) uint8.")

      .def("__repr__", [](const PDB &db) {
        return "<ProjectDB path='" + db.path().string() +
               "' open=" + (db.is_open() ? "True" : "False") + ">";
      });
}
