// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "reusex/core/label_semantics.hpp"
#include "reusex/core/logging.hpp"
#include "reusex/core/processing_observer.hpp"

#include <Eigen/Core>
#include <embree4/rtcore.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <pcl/filters/uniform_sampling.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// Spherical Fibonacci
static std::vector<Eigen::Vector3d> sampleSphericalFibonacci(size_t N) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(N);
  if (N == 0)
    return out;
  constexpr double golden_angle =
      M_PI * (3.0 - std::sqrt(5.0)); // ~2.3999632297

  for (size_t i = 0; i < N; ++i) {
    double t = (double)i;
    double z = 1.0 - 2.0 * t / (double)(N - 1); // maps to [1, -1] if N>1
    double phi = golden_angle * t;
    double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    double x = std::cos(phi) * r;
    double y = std::sin(phi) * r;
    out.emplace_back(x, y, z);
  }
  return out;
}

namespace reusex::geometry {
template <typename PointT, typename PointN, typename PointL>
auto CellComplex::compute_room_probabilities(
    pcl::PointCloud<PointT>::ConstPtr cloud_,
    pcl::PointCloud<PointN>::ConstPtr normals_,
    pcl::PointCloud<PointL>::ConstPtr labels_, const double grid_size) -> void {

  struct RTCVertex {
    float x, y, z, r; // x, y, z coordinates and radius
  };

  struct RTCNormal {
    float x, y, z; // x, y, z components of the normal vector
  };

  struct RTCData {
    size_t label_index;
  };

  reusex::trace("calling compute_room_probabilities");

  assert(cloud_->size() == labels_->size() &&
         "Point cloud and label cloud must have the same size");
  assert(cloud_->size() == normals_->size() &&
         "Point cloud and normal cloud must have the same size");

  const double radius_ = grid_size * M_SQRT2;

  reusex::debug(
      "Using ray tracing for {} points with grid size {:.3} and radius {:.3}",
      cloud_->size(), grid_size, radius_);

  auto c_rp = this->add_property_map<Vertex, std::vector<double>>(
                      "c:room_probabilities")
                  .first;

  // Get unique labels
  std::vector<unsigned int> labels;
  labels.reserve(labels_->points.size());

  // Rooms follow the point-label convention: 0 = unlabeled, valid rooms 1..N.
  for (const auto &p : labels_->points)
    if (reusex::core::is_valid_label(p.label))
      labels.push_back(p.label);

  std::sort(labels.begin(), labels.end());
  labels.erase(std::unique(labels.begin(), labels.end()), labels.end());
  reusex::debug("Room labels: {}", fmt::join(labels, ", "));

  for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit)
    c_rp[*cit] = std::vector<double>(labels.size() + 1, 0.0);
  this->n_rooms = labels.size();
  reusex::debug("Number of rooms (labels): {}", labels.size());

  unsigned int old_mxcsr = _mm_getcsr(); // save current flagsq
  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  reusex::trace("Creating device and scene for ray tracing");
  RTCDevice device_ = rtcNewDevice("verbose=0"); // 0-3
  RTCScene scene_ = rtcNewScene(device_);
  rtcSetSceneFlags(scene_, RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(scene_, RTC_BUILD_QUALITY_HIGH);
  assert(device_ != nullptr && "Error creating Embree device");
  assert(scene_ != nullptr && "Error creating Embree scene");
  rtcSetDeviceErrorFunction(
      device_,
      [](void *, RTCError err, const char *str) {
        reusex::error("Embree error {}:", str);
        switch (err) {
        case RTC_ERROR_NONE:
          break;
        case RTC_ERROR_UNKNOWN:
          throw std::runtime_error("Embree: An unknown error has occurred.");
          break;
        case RTC_ERROR_INVALID_ARGUMENT:
          throw std::runtime_error(
              "Embree: An invalid argument was specified.");
          break;
        case RTC_ERROR_INVALID_OPERATION:
          throw std::runtime_error(
              "Embree: The operation is not allowed for the specified object.");
          break;
        case RTC_ERROR_OUT_OF_MEMORY:
          throw std::runtime_error("Embree: There is not enough memory left to "
                                   "complete the operation.");
          break;
        case RTC_ERROR_UNSUPPORTED_CPU:
          throw std::runtime_error(
              "Embree: The CPU is not supported as it does not support SSE2.");
          break;
        case RTC_ERROR_CANCELLED:
          throw std::runtime_error(
              "Embree: The operation got cancelled by an Memory Monitor "
              "Callback or Progress Monitor Callback function.");
          break;
        default:
          throw std::runtime_error("Embree: An invalid error has occurred.");
          break;
        }
      },
      nullptr);

  // Intersect with ground plane to get line/ INFO: Create Embree scene
  reusex::trace("Creating scene geometry for ray tracing");

  pcl::UniformSampling<PointT> us;
  us.setInputCloud(cloud_);
  us.setRadiusSearch(grid_size);

  using RTCDataPtr = std::shared_ptr<RTCData>;
  std::vector<RTCDataPtr> rtc_data_vec{}; // To keep data alive
  for (size_t i = 0; i < labels.size(); ++i) {

    pcl::IndicesPtr indices(new pcl::Indices);
    for (size_t j = 0; j < cloud_->size(); ++j)
      if (labels_->points[j].label == labels[i])
        indices->push_back(static_cast<int>(j));

    reusex::trace("Room label {}: {} points", labels[i], indices->size());

    // size_t n_points_before = indices->size();
    us.setIndices(indices);
    us.filter(*indices);
    // reusex::trace(
    //     "Label {}: Reduced from {} to {} points after uniform sampling",
    //     labels[i], n_points_before, indices->size());

    RTCGeometry geometry_ =
        rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT);

    auto *vb = static_cast<RTCVertex *>(rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4,
        sizeof(RTCVertex), indices->size()));

    auto *nb = static_cast<RTCNormal *>(rtcSetNewGeometryBuffer(
        geometry_, RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3,
        sizeof(RTCNormal), indices->size()));

    for (size_t k = 0; k < indices->size(); ++k) {
      const auto &p = cloud_->points[indices->at(k)];
      const auto &n = normals_->points[indices->at(k)];

      vb[k].x = static_cast<float>(p.x);
      vb[k].y = static_cast<float>(p.y);
      vb[k].z = static_cast<float>(p.z);
      vb[k].r = static_cast<float>(radius_) * 1.05f;

      nb[k].x = static_cast<float>(n.normal_x);
      nb[k].y = static_cast<float>(n.normal_y);
      nb[k].z = static_cast<float>(n.normal_z);
    }

    rtc_data_vec.emplace_back(new RTCData{i});
    rtcSetGeometryUserData(geometry_, rtc_data_vec.back().get());

    rtcCommitGeometry(geometry_);
    rtcAttachGeometry(scene_, geometry_);
    rtcReleaseGeometry(geometry_);
  }

  reusex::trace("Committing scene");
  rtcCommitScene(scene_);

  {
    const auto dirs = sampleSphericalFibonacci(100);

    // A hit closer than this to its origin means the origin sits on (or
    // inside) a scanned surface: the disc radius the points are splatted
    // with.
    const float near_distance = static_cast<float>(radius_) * 1.05f;

    auto observer = reusex::core::ProgressObserver(
        reusex::core::Stage::computing_room_probabilities, this->num_cells());

#pragma omp parallel for schedule(dynamic)
    for (size_t cell_idx = 0; cell_idx < this->num_cells(); ++cell_idx) {
      auto cit = this->cells_begin();
      std::advance(cit, cell_idx);

      // Rays are cast from several origins spread through the cell, not
      // only from its centroid (reusex-bi6). A cell is a convex prism, so the
      // centroid and the point halfway from it to each corner all lie inside
      // it. A single centroid lands inside furniture often enough in a
      // furnished room (a screen or lamp at mid-storey height on the NewOffice
      // desk strips): nearly every ray then hits the back of that object
      // within a few cm and the whole cell — 47 m^3 of open office — read as
      // "outside".
      const Eigen::Vector3d centre = (*this)[*cit].pos;
      std::vector<Eigen::Vector3d> origins{centre};
      {
        std::vector<Vertex> corners;
        for (auto [f, f_end] = boost::adjacent_vertices(*cit, *this);
             f != f_end; ++f) {
          if ((*this)[*f].type != CellNodeType::face)
            continue;
          for (auto [v, v_end] = boost::adjacent_vertices(*f, *this);
               v != v_end; ++v)
            if ((*this)[*v].type == CellNodeType::vertex)
              corners.push_back(*v);
        }
        std::sort(corners.begin(), corners.end());
        corners.erase(std::unique(corners.begin(), corners.end()),
                      corners.end());
        for (auto v : corners)
          origins.push_back(centre + 0.5 * ((*this)[v].pos - centre));
      }

      // Each origin votes with its ray fractions, weighted by the share of
      // its rays that travel further than `near_distance`: an origin inside
      // or against an object (all rays stop at once, half of them on its
      // back faces) says little about the room around it. When every
      // origin is boxed in (a thin wall-slab cell between two scanned
      // faces), they count equally.
      const size_t n_labels = c_rp[*cit].size();
      std::vector<double> weighted(n_labels, 0.0);
      std::vector<double> plain(n_labels, 0.0);
      double weight_sum = 0.0;
      std::vector<double> votes(n_labels);

      for (const auto &o : origins) {
        std::fill(votes.begin(), votes.end(), 0.0);
        size_t n_far = 0;
        for (const auto &dir : dirs) {
          RTCRayHit rayhit;
          rayhit.ray.org_x = static_cast<float>(o.x());
          rayhit.ray.org_y = static_cast<float>(o.y());
          rayhit.ray.org_z = static_cast<float>(o.z());
          rayhit.ray.dir_x = static_cast<float>(dir.x());
          rayhit.ray.dir_y = static_cast<float>(dir.y());
          rayhit.ray.dir_z = static_cast<float>(dir.z());

          rayhit.ray.tnear = 0.001f; // Start a bit away from the origin
          rayhit.ray.tfar = std::numeric_limits<float>::infinity();
          rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

          rayhit.ray.mask = -1;
          rayhit.ray.flags = 0;
          rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

          rtcIntersect1(scene_, &rayhit);

          if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) { // No hit
            votes[0] += 1;
            ++n_far;
            continue;
          }
          if (rayhit.ray.tfar >= near_distance)
            ++n_far;

          // A hit on the back of a surface means the origin is behind it.
          const auto normal =
              Eigen::Vector3f(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z)
                  .normalized();
          const Eigen::Vector3f dir_vec(dir.x(), dir.y(), dir.z());
          if (normal.dot(dir_vec) > 0) {
            votes[0] += 1;
            continue;
          }

          auto geometry = rtcGetGeometry(scene_, rayhit.hit.geomID);
          auto *data = static_cast<RTCData *>(rtcGetGeometryUserData(geometry));
          votes[data->label_index + 1] += 1;
        }

        const double n_rays = static_cast<double>(dirs.size());
        const double w = static_cast<double>(n_far) / n_rays;
        for (size_t j = 0; j < n_labels; ++j) {
          plain[j] += votes[j] / n_rays;
          weighted[j] += w * votes[j] / n_rays;
        }
        weight_sum += w;
      }

      std::vector<double> probs(n_labels, 0.0);
      for (size_t j = 0; j < n_labels; ++j)
        probs[j] = weight_sum > 0.0
                       ? weighted[j] / weight_sum
                       : plain[j] / static_cast<double>(origins.size());
      c_rp[*cit] = std::move(probs);

      ++observer;
    }
  }
  rtcReleaseScene(scene_);
  rtcReleaseDevice(device_);

  _mm_setcsr(old_mxcsr); // restore old flags

  // Log sum of probabilities
  auto sum_results = std::vector<double>(labels.size() + 1, 0.0);
  for (auto cit = this->cells_begin(); cit != this->cells_end(); ++cit)
    for (size_t i = 0; i < c_rp[*cit].size(); ++i)
      sum_results[i] += c_rp[*cit][i];
  reusex::trace("Sum probabilities => [{:.3f}]", fmt::join(sum_results, ", "));
}
} // namespace reusex::geometry
