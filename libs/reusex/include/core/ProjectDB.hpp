// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
// Point-cloud aliases (Cloud/CloudN/CloudL and pcl::PointCloud<pcl::PointXYZ>)
// are part of the public signatures below, so the point-type header stays.
// The heavier OpenCV and PCL mesh headers are only needed by the .cpp: they
// are forward-declared here (cv::Mat by value/reference, mesh types via
// std::shared_ptr) to keep DB clients from transitively compiling them.
#include "reusex/core/component_record.hpp"
#include "reusex/core/survey.hpp"
#include "reusex/types/point_types.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Forward declarations for heavy third-party types used only by-value or via
// shared_ptr in the public API (definitions pulled in by ProjectDB.cpp).
namespace cv {
class Mat;
} // namespace cv
namespace pcl {
struct PolygonMesh;
struct TextureMesh;
} // namespace pcl

namespace reusex {

// Forward declarations
namespace core {
struct MaterialPassport;
struct MaterialPassportMetadata;
struct SensorIntrinsics;
} // namespace core

class ProjectDB {
    public:
  /**
   * @brief Opens a ReUseX project database and validates its schema
   *
   * Creates the following tables if they don't exist (write mode only):
   * - projects: Project metadata (building info, survey details)
   * - property_definitions: Leksikon-based property definitions
   * - material_passports: Material passport documents
   * - passport_property_values: Property values for each passport
   * - passport_log: Audit log for tracking changes
   * - schema_version: Schema version tracking
   * - point_clouds / point_cloud_data: Point cloud storage
   * - label_definitions: Semantic label lookup
   * - meshes: Mesh storage
   * - gaussian_splats / gaussian_splat_data: Gaussian splat storage
   * - sensor_frames: Imported sensor frame color images
   * - segmentation_images: Per-frame semantic label images
   * - pipeline_log: Pipeline provenance log
   *
   * @param dbPath Path to the database file
   * @param readOnly If true, opens database in read-only mode. In a
   *        directory the user cannot write (a read-only mount), and with no
   *        live `-wal` beside the file, the open is sqlite-immutable: a
   *        WAL-mode file is unreadable there otherwise (sqlite must create
   *        `-shm` even to read), and nobody can be writing it through that
   *        directory. Elsewhere a read-only open is a plain one and, like any
   *        WAL reader, may create the `-shm` / `-wal` sidecars.
   *        Trade-off: "nobody can be writing" holds for THIS user only. On a
   *        shared drive where a colleague has write access, an immutable
   *        connection never sees their WAL and takes no locks, so it can
   *        read stale data or a page mid-checkpoint (SQLITE_CORRUPT). Reopen
   *        to see their changes. A non-empty `-wal` in such a directory
   *        (an unfinished write) cannot be read at all and fails to open.
   * @throws std::runtime_error if database cannot be opened or schema is
   * invalid
   */
  explicit ProjectDB(std::filesystem::path dbPath, bool readOnly = false);

  /// How a connection is opened. `hardened` is for files from a source the
  /// process does not trust (a server's uploaded or registered cases): the
  /// connection runs with SQLITE_DBCONFIG_DEFENSIVE, trusted_schema=OFF, and
  /// with triggers and views disabled (a ReUseX project has neither), so
  /// nothing stored in the file's schema executes when it is opened or read.
  struct OpenOptions {
    bool read_only = false;
    bool hardened = false;
  };
  ProjectDB(std::filesystem::path dbPath, OpenOptions options);

  /// Process-wide default for `OpenOptions::hardened`, used by the
  /// `(path, readOnly)` constructor — and so by every open in this process
  /// that does not pass options, pipeline stages included. A server sets it
  /// once at startup; the local `rux` CLI leaves it off.
  static void set_hardened_by_default(bool on) noexcept;
  static bool hardened_by_default() noexcept;

  /// Result of check_integrity().
  struct IntegrityReport {
    enum class Kind {
      ok,
      unreadable,        ///< SQLite cannot open the file at all
      corrupt,           ///< `PRAGMA quick_check` reported a problem
      executable_schema, ///< the schema holds a trigger or a view
    };
    Kind kind = Kind::ok;
    std::string detail; ///< SQLite's own words; empty when ok
    bool ok() const noexcept { return kind == Kind::ok; }
  };

  /// Checks a file before a server adopts it, on a hardened read-only
  /// connection that runs no migration: `PRAGMA quick_check`, then that the
  /// schema holds no trigger and no view. Takes time linear in the file size
  /// (minutes for tens of GB). Never throws for a bad file.
  static IntegrityReport check_integrity(const std::filesystem::path &file);

  bool is_read_only() const noexcept;
  /**
   * @brief Destructor closes database connection
   */
  ~ProjectDB();

  // Non-copyable (RAII resource management)
  ProjectDB(const ProjectDB &) = delete;
  ProjectDB &operator=(const ProjectDB &) = delete;

  // Movable
  ProjectDB(ProjectDB &&) noexcept;
  ProjectDB &operator=(ProjectDB &&) noexcept;

  // --- Core Database Operations ---

  bool is_open() const noexcept;
  const std::filesystem::path &path() const noexcept;
  int schema_version() const;
  static int latest_schema_version() noexcept;

  /// What probe() found, without opening a full ProjectDB.
  struct ProbeResult {
    bool is_project = false; ///< sqlite with the ReUseX core tables
    int schema_version = -1; ///< MAX(schema_version.version); -1 if absent
    std::string error;       ///< sqlite / validation message when not a project
  };

  /// Is @p path a ReUseX project, and at which schema? Opens the file
  /// read-only, checks the tables validate_schema() requires and reads the
  /// recorded version. Never writes, creates, migrates or logs — so a GUI can
  /// vet a foreign or empty sqlite file before a read-write open would give
  /// it ReUseX tables.
  ///
  /// The open is sqlite-immutable (`?mode=ro&immutable=1`) unless a non-empty
  /// `-wal` sits beside the file: no locks, no `-shm` / `-wal` created, and it
  /// works in a read-only directory. Trade-off: a writer's in-flight
  /// transaction is not seen, which a table check can live with. With a live
  /// WAL it is a plain read-only open (the WAL's pages count), which may wait
  /// on and report a lock in `error` (is_project false). Under
  /// hardened_by_default() the connection is hardened like any other open;
  /// `rux` leaves that off, so its probe is the plain open described here.
  static ProbeResult probe(const std::filesystem::path &path) noexcept;

  /// One write transaction (BEGIN IMMEDIATE ... COMMIT), rolled back unless
  /// commit() is called. ProjectDB methods that open their own transaction
  /// (add_material_passport, set_survey_part_quantities, add_sample, …)
  /// cannot be called inside one; the resource/template methods can.
  class Transaction {
      public:
    explicit Transaction(ProjectDB &db); ///< throws on a read-only project
    ~Transaction();
    void commit();
    Transaction(const Transaction &) = delete;
    Transaction &operator=(const Transaction &) = delete;

      private:
    ProjectDB *db_;
    bool open_ = true;
  };
  void validate_schema() const;

  // --- Sensor Frame Operations ---

  void save_sensor_frame(int nodeId, const cv::Mat &colorImage);

  void save_sensor_frame(int nodeId, const cv::Mat &color, const cv::Mat &depth,
                         const cv::Mat &confidence,
                         const std::array<double, 16> &worldPose,
                         const core::SensorIntrinsics &intrinsics,
                         double timestamp = -1.0, int scan_id = -1);

  /// Update only the stored world pose (transform) of an existing sensor frame.
  /// Leaves color/depth/confidence/intrinsics blobs untouched. Throws if no
  /// sensor frame with the given node_id exists.
  /// @param worldPose Row-major 4x4 SE(3) world pose (16 doubles).
  void update_sensor_frame_pose(int nodeId,
                                const std::array<double, 16> &worldPose);

  std::vector<int> sensor_frame_ids() const;

  /// Returns {node_id, scan_id} pairs ordered by node_id.
  /// scan_id is 0 when the frame has no recorded scan (schema < v17).
  std::vector<std::pair<int, int>> sensor_frame_ids_with_scan() const;
  cv::Mat sensor_frame_image(int nodeId) const;
  cv::Mat sensor_frame_depth(int nodeId) const;
  cv::Mat sensor_frame_confidence(int nodeId) const;
  /// The stored world pose, **verbatim** — row-major 4x4, identity when the
  /// row is missing, the `transform` is NULL, or the blob is the wrong size.
  ///
  /// This accessor does NOT validate. A well-sized all-zero or NaN transform
  /// is handed back exactly as stored, and the identity fallback is
  /// indistinguishable from a frame legitimately placed at the origin.
  ///
  /// **If a wrong pose would corrupt your output, gate on
  /// `has_sensor_frame_pose()` first** — every pipeline stage does (#336).
  /// Raw use is for read-out surfaces (`rux get`, the GUI API, the Python
  /// bindings) that exist to show what the project actually contains,
  /// including when it is broken.
  ///
  /// Deliberately non-throwing (#336): making it reject degenerate transforms
  /// would break exactly the diagnostic paths you need on a bad project, and
  /// the validity question has a dedicated accessor below.
  std::array<double, 16> sensor_frame_pose(int nodeId) const;

  /// True when this frame carries a usable stored world pose.
  ///
  /// `sensor_frame_pose()` cannot answer this: it returns identity for a frame
  /// with no row, a NULL `transform`, or a wrong-sized blob, so a poseless
  /// frame is indistinguishable from one legitimately at the origin (#330) —
  /// and it returns an all-zero or NaN transform verbatim (#336).
  ///
  /// Checks: the row exists, the blob is exactly 16 doubles, all finite, the
  /// bottom row is `[0,0,0,1]`, and `|det(R)|` is above epsilon. A genuinely
  /// stored identity pose IS valid — a scan may put its first frame at the
  /// origin.
  bool has_sensor_frame_pose(int nodeId) const;

  /// Snapshot of the pose `rux optimize` seeds from (schema v27,
  /// sensor_frames.initial_transform). Written by `rux graph init` and
  /// `rux import rtabmap`; never by optimize.
  /// @throws std::invalid_argument if @p pose is not a rigid transform;
  ///         std::runtime_error if there is no such frame or the project is
  ///         read-only / pre-v27.
  void set_initial_pose(int nodeId, const std::array<double, 16> &pose);

  /// nullopt when NULL, not a rigid transform, the frame is missing, or the
  /// project predates v27 (a read-only open never migrates).
  std::optional<std::array<double, 16>> initial_pose(int nodeId) const;

  bool has_initial_pose(int nodeId) const;

  core::SensorIntrinsics sensor_frame_intrinsics(int nodeId) const;
  bool has_sensor_frame(int nodeId) const;

  /// Get the timestamp (epoch seconds) of a sensor frame. Returns -1.0 if not
  /// set.
  double sensor_frame_timestamp(int nodeId) const;

  /// Find the sensor frame with the closest timestamp to the given value.
  /// Returns -1 if no sensor frames have timestamps.
  int nearest_sensor_frame_by_timestamp(double timestamp) const;

  // --- Panoramic Image Operations ---

  struct PanoramicImage {
    int id;
    std::string filename;
    double timestamp; // -1.0 if unknown
    int node_id;      // -1 if unmatched
    // Refined 6-DoF pose from content-based alignment (`rux align 360`). When
    // has_pose is false the panorama has only its timestamp-matched frame pose.
    bool has_pose = false;
    std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1}; // row-major world
    std::string pose_source = "timestamp"; // "timestamp" | "aligned"
    int align_inliers = -1;                // RANSAC inliers, -1 if not aligned
    // Angular RMS of the inlier bearings after resection, in degrees
    // (PanoramaAlignmentResult::rms_deg). -1 if not aligned.
    double align_rms = -1.0;
  };

  void save_panoramic_image(const std::string &filename,
                            const std::vector<uint8_t> &jpeg_data,
                            double timestamp = -1.0, int nodeId = -1);

  cv::Mat panoramic_image(int id) const;
  cv::Mat panoramic_image(std::string_view filename) const;
  bool has_panoramic_image(std::string_view filename) const;
  void delete_panoramic_image(int id);
  void delete_panoramic_image(std::string_view filename);
  std::vector<PanoramicImage> list_panoramic_images() const;
  int panoramic_image_count() const;

  /// Store a content-aligned pose (row-major 4x4 world) for a panorama.
  void save_panorama_pose(int id, const std::array<double, 16> &pose,
                          int inliers, double rms);

  // --- Panorama Segmentation Operations ---

  bool has_panorama_segmentation(int panoId) const;
  cv::Mat panorama_segmentation(int panoId) const;
  void save_panorama_segmentation(int panoId, const cv::Mat &labels);

  // --- Segmentation Image Operations ---

  bool has_segmentation_image(int nodeId) const;
  cv::Mat segmentation_image(int nodeId) const;
  std::vector<int> segmentation_image_ids() const;
  void save_segmentation_image(int nodeId, const cv::Mat &labels);
  void save_segmentation_images(const std::vector<int> &nodeIds,
                                const std::vector<cv::Mat> &labels);

  // --- Glass Confidence Image Operations ---
  //
  // Per-frame binary depth-suppress map produced by `rux create annotate
  // --glass-filter`.  Stored as CV_8U PNG: 0 = suppress (glass/mirror), 255 =
  // trust depth.  Consumed by reconstruct_point_clouds when glass_filter is
  // enabled.

  bool has_glass_confidence_image(int nodeId) const;
  cv::Mat glass_confidence_image(int nodeId) const;
  std::vector<int> glass_confidence_image_ids() const;
  void save_glass_confidence_image(int nodeId, const cv::Mat &confidence);

  // --- Point Cloud Operations ---

  void save_point_cloud(std::string_view name, const Cloud &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name, const CloudN &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name, const CloudL &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  void save_point_cloud(std::string_view name,
                        const pcl::PointCloud<pcl::PointXYZ> &cloud,
                        std::string_view stage = "",
                        std::string_view paramsJson = "");

  CloudPtr point_cloud_xyzrgb(std::string_view name) const;
  CloudNPtr point_cloud_normal(std::string_view name) const;
  CloudLPtr point_cloud_label(std::string_view name) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr
  point_cloud_xyz(std::string_view name) const;

  bool has_point_cloud(std::string_view name) const;
  void delete_point_cloud(std::string_view name);
  std::vector<std::string> list_point_clouds() const;
  std::string point_cloud_type(std::string_view name) const;

  /// Returns the "storage_order" value from the cloud's parameters JSON, or
  /// an empty string when the key is absent or the JSON is unparseable.
  ///
  /// "morton_10bit" means the cloud was stored in Morton order (10 bits/axis,
  /// 30-bit code) so any prefix is a uniform spatial sample — the LOD read
  /// path can skip its voxel pass and return a prefix directly (#394).
  std::string point_cloud_storage_order(std::string_view name) const;

  /// The `stage` and `parameters` (JSON, may be empty) the cloud was last
  /// saved with — so an in-place edit can save it again without erasing how
  /// it was produced. @throws std::runtime_error when @p name is not a cloud.
  std::pair<std::string, std::string>
  point_cloud_provenance(std::string_view name) const;

  /// Save a serialized tile index blob for the named cloud.
  void save_tile_index(std::string_view name, const std::vector<uint8_t> &blob);

  /// Load the tile index blob, or empty if not set.
  /// @throws std::runtime_error when @p name is not a stored cloud.
  std::vector<uint8_t> tile_index(std::string_view name) const;

  /// A contiguous window of one cloud's stored records, still in storage
  /// layout — no PCL type has been inflated.
  struct CloudPage {
    /// "PointXYZRGB" | "PointXYZ" | "Normal" | "Label".
    std::string point_type;
    uint32_t point_step = 0; ///< Bytes per stored record.
    uint64_t offset = 0;     ///< First point index, clamped to @c total.
    uint64_t count = 0;      ///< Points actually read.
    uint64_t total = 0;      ///< Points in the whole cloud.
    /// @c count * @c point_step bytes, exactly as stored.
    std::vector<uint8_t> data;
  };

  /**
   * @brief Read one page of a point cloud without materializing the cloud.
   *
   * Peak memory is O(page), not O(cloud). The chunk map is built from
   * `length(data)` (which does not load a blob), and only the chunks the
   * requested byte range `[offset*step, (offset+count)*step)` overlaps are
   * touched, via SQLite incremental blob I/O. A 100k-point page of a
   * 10M-point `PointXYZRGB` cloud therefore costs ~1.6 MB, not the ~160 MB
   * the blob occupies (nor the ~320 MB it would occupy once inflated into
   * `pcl::PointXYZRGB`, which is 32 bytes wide because of SSE padding).
   *
   * A record may straddle a chunk boundary, so the read works in raw byte
   * ranges rather than whole records per chunk.
   *
   * @param offset First point index. Past the end yields `count == 0` with
   *               `total` still populated — not an error.
   * @param limit  Maximum points to return. `0` yields an empty page.
   * @throws std::runtime_error when @p name is not a stored cloud, or when
   *         the stored `point_step` contradicts the stored `point_type`.
   */
  CloudPage point_cloud_page(std::string_view name, uint64_t offset,
                             uint64_t limit) const;

  // --- Label Definitions ---

  void save_label_definitions(std::string_view cloudName,
                              const std::map<int, std::string> &labelMap);

  std::map<int, std::string>
  label_definitions(std::string_view cloudName) const;

  // --- Instances (stable identity) ---

  /// One row of the `instances` table: a spatially-distinct object within an
  /// instance-label cloud, carrying a stable GUID that survives regeneration.
  struct InstanceRecord {
    uint32_t instance_id = 0; ///< Label value in the instance cloud (>= 1).
    std::string guid;         ///< Stable identity (UUID-v4-like).
    int semantic_class = -1;  ///< Semantic class this instance belongs to.
    int point_count = 0;      ///< Number of points in the instance.
  };

  /// Replace the full set of instance rows for a cloud (transactional).
  /// Existing rows for the cloud are deleted first; each record's GUID must be
  /// non-empty and unique across the DB. Throws on empty/duplicate GUID.
  void save_instances(const std::string &cloud_name,
                      const std::vector<InstanceRecord> &records);

  /// All instance rows for a cloud, ordered by instance_id.
  std::vector<InstanceRecord> instances(const std::string &cloud_name) const;

  /// Insert one instance row, leaving the cloud's other rows (and their
  /// `instance_materials` links, which a save_instances() delete cascades
  /// away) untouched. Nests inside a Transaction. Throws on an empty or
  /// already-used guid, or an instance_id the cloud already has.
  void add_instance(const std::string &cloud_name,
                    const InstanceRecord &record);

  /// Update one instance row's point_count in place (no delete/re-insert).
  /// @throws std::out_of_range when the cloud has no such instance.
  void set_instance_point_count(const std::string &cloud_name,
                                uint32_t instance_id, int point_count);

  /// Stable GUID of a single instance.
  /// @throws std::runtime_error if the cloud or instance does not exist.
  std::string instance_guid(const std::string &cloud_name,
                            uint32_t instance_id) const;

  // --- Instance ↔ Material Links ---

  /// Link an instance (a row in the `instances` table) to a material passport
  /// by its document guid. Upserts on (cloud, instance_id).
  ///
  /// Validates referential integrity: the instance must exist in the target
  /// cloud's `instances` rows and the passport GUID must exist in
  /// `material_passports`. Throws std::runtime_error otherwise (STANDARDS §5).
  void set_instance_material(std::string_view cloudName, int instanceId,
                             std::string_view materialGuid);

  /// Material passport guid linked to an instance, or nullopt if unlinked.
  std::optional<std::string> instance_material_guid(std::string_view cloudName,
                                                    int instanceId) const;

  /// All instance_id → material_guid links for a cloud.
  std::map<int, std::string>
  instance_materials(std::string_view cloudName) const;

  // --- Material Annotations (VLM-derived) ---

  /// A free-text description plus an arbitrary, prompt-driven set of key/value
  /// attributes for one material passport, as extracted by a vision-language
  /// model (`rux create attributes`, #373). The prompt fully drives which keys
  /// come back; nothing here is a fixed schema and nothing is fabricated — an
  /// empty description with no attributes is never stored.
  struct MaterialAnnotation {
    std::string description; ///< Free-text description of the material.
    std::vector<std::pair<std::string, std::string>>
        attributes; ///< Arbitrary key/value pairs the model returned.
    std::string provider_model; ///< Provenance: "<base_url>|<model>".
    std::string raw_json;       ///< The model's full JSON answer, verbatim.
  };

  /// Upsert the VLM annotation for one material passport (keyed on its
  /// document GUID). Replaces the description/provenance and REPLACES the
  /// key/value rows for that GUID atomically, so no stale key survives a
  /// re-describe.
  /// @throws std::runtime_error if the material passport does not exist
  ///         (STANDARDS §5) — run `rux create materials` first.
  void save_material_annotation(std::string_view materialGuid,
                                const MaterialAnnotation &annotation);

  /// The stored annotation for one material passport, or nullopt when none was
  /// saved. Key/value pairs are returned in key-sorted order (deterministic).
  std::optional<MaterialAnnotation>
  material_annotation(std::string_view materialGuid) const;

  // --- Mesh Operations ---

  void save_mesh(std::string_view name, const pcl::PolygonMesh &mesh,
                 std::string_view stage = "", std::string_view paramsJson = "");

  void save_mesh(std::string_view name, const pcl::TextureMesh &mesh,
                 std::string_view stage = "", std::string_view paramsJson = "");

  std::shared_ptr<pcl::PolygonMesh> mesh(std::string_view name) const;
  std::shared_ptr<pcl::TextureMesh> texture_mesh(std::string_view name) const;
  bool has_mesh(std::string_view name) const;
  std::vector<std::string> list_meshes() const;
  std::string mesh_format(std::string_view name) const;

  struct MeshMetadata {
    std::string name, format, stage, parameters, created_at;
    int vertex_count = 0, face_count = 0;
  };
  MeshMetadata mesh_metadata(std::string_view name) const;

  std::vector<uint8_t> mesh_data_blob(std::string_view name) const;

  struct MeshTextureInfo {
    std::string tex_name, format;
    std::vector<uint8_t> image_data;
    int width, height;
  };
  std::vector<MeshTextureInfo> mesh_texture_blobs(std::string_view name) const;

  struct MeshTextureMetadata {
    std::string tex_name, format;
    int width, height;
  };
  std::vector<MeshTextureMetadata>
  mesh_texture_metadata(std::string_view name) const;

  // --- Gaussian Splat Operations ---
  //
  // A trained splat is stored verbatim, as the INRIA `.ply` bytes, chunked
  // across `gaussian_splat_data` rows the same way point clouds are: a splat
  // is routinely hundreds of MB and SQLite materializes a whole blob column
  // on read, so a single-row layout would cost peak memory proportional to
  // the entire file even to answer "how many Gaussians".
  //
  // The bytes are opaque to core — nothing here inflates a splat into a
  // geometry type. What core *does* own is the guarantee that the bytes are a
  // splat at all: `save_gaussian_splat()` parses the PLY header
  // (core/gaussian_splat.hpp) and refuses anything else, so the point-cloud
  // PLY that `rux export ply` writes is rejected at the door instead of
  // reaching a renderer that would draw nothing and say nothing.

  /// One row of the `gaussian_splats` table, minus the payload.
  struct GaussianSplatMetadata {
    std::string name;
    std::string format; ///< "ply" (INRIA 3DGS layout) — the only one today.
    std::uint64_t gaussian_count = 0;
    int sh_degree = 0;           ///< Derived from the `f_rest_*` count.
    std::uint64_t byte_size = 0; ///< Size of the stored `.ply`, in bytes.
    std::string created_at;
    std::string stage;      ///< Pipeline stage that produced it, may be empty.
    std::string parameters; ///< JSON provenance, may be empty.
  };

  /// Store an INRIA-format 3DGS `.ply` under @p name, replacing any splat
  /// already stored under it (chunks of the previous payload are dropped, so
  /// a re-save never leaves orphaned rows behind).
  ///
  /// The header is parsed for two reasons, not one: it yields the metadata
  /// (`gaussian_count`, `sh_degree`) without a second source of truth, and it
  /// rejects a file that is not a Gaussian splat *before* several hundred MB
  /// are committed to the project.
  ///
  /// @throws std::runtime_error if @p ply is not an INRIA 3DGS PLY.
  void save_gaussian_splat(std::string_view name,
                           const std::vector<uint8_t> &ply,
                           std::string_view stage = "",
                           std::string_view parameters = "");

  /// False — not an error — on a project whose schema predates the splat
  /// tables, which is what a read-only open of an old project looks like
  /// (read-only opens never migrate).
  bool has_gaussian_splat(std::string_view name) const;

  /// Empty on a pre-v12 schema, for the same reason as
  /// `has_gaussian_splat()`.
  std::vector<std::string> list_gaussian_splats() const;

  /// @throws std::runtime_error when there is no splat named @p name.
  GaussianSplatMetadata gaussian_splat_metadata(std::string_view name) const;

  /// The stored bytes, reassembled from their chunks — byte-for-byte what
  /// `save_gaussian_splat()` was given.
  /// @throws std::runtime_error when there is no splat named @p name.
  std::vector<uint8_t> gaussian_splat_blob(std::string_view name) const;

  /// Deleting the metadata row cascades to its data chunks.
  /// @returns false when there was nothing to delete.
  bool delete_gaussian_splat(std::string_view name);

  // --- Pose Graph ---
  //
  // The stored pose graph is the source of truth for `rux optimize` (schema
  // v27, docs/superpowers/specs/2026-10-13-pose-graph-editor-design.md §3).
  // Rows are canonical: from_node_id < to_node_id, UNIQUE(from, to, type).
  // Node positions are read from `sensor_frames` (their current world poses,
  // and the snapshot in `initial_transform`), so there is no nodes table.

  /// One edge stored in `pose_graph_edges`. The first five members keep their
  /// pre-v27 order so `{from, to, type, residual, weight}` still initialises.
  struct PoseGraphEdge {
    int from_node_id = 0;
    int to_node_id = 0;
    /// "odometry" | "loop_closure" | "panorama" (other strings are stored
    /// as given; ruxd may write them).
    std::string edge_type;
    /// 0.5 x whitened squared residual after the last solve (GTSAM
    /// convention). NaN <-> NULL (never solved).
    double residual = std::numeric_limits<double>::quiet_NaN();
    /// 1 / sigma_trans^2. Derived from `information` on write when that is
    /// present. NaN <-> NULL.
    double weight = std::numeric_limits<double>::quiet_NaN();
    /// Measurement T_from^-1 * T_to in the sensor_frames.transform
    /// convention, row-major. nullopt for migrated v15 rows.
    std::optional<std::array<double, 16>> transform;
    /// 6x6 information, GTSAM Pose3 order (rx, ry, rz, tx, ty, tz),
    /// row-major. nullopt: the per-type default sigma of PlaneGraphOptions.
    std::optional<std::array<double, 36>> information;
    /// rtabmap | odometry_init | orb | panorama | file | icp | manual | legacy
    std::string source = "manual";
    /// auto | confirmed | rejected
    std::string status = "auto";
    /// ISO-8601 UTC; "" on write means "now".
    std::string created_at;
    std::optional<double> seed_disagreement_m;
    std::optional<double> seed_disagreement_deg;
    std::optional<double> deviation_m;
    std::optional<double> deviation_deg;
    /// Row id, filled by the readers; ignored on write.
    int64_t id = 0;
  };

  /// All stored edges (every status), ordered by id. Empty when the table
  /// does not exist. On a pre-v27 project opened read-only the rows carry
  /// source "legacy", status "auto" and no measurement.
  std::vector<PoseGraphEdge> list_pose_graph_edges() const;

  /// The edge with key (from, to, type), given in either direction.
  std::optional<PoseGraphEdge>
  pose_graph_edge(int from_node_id, int to_node_id,
                  std::string_view edge_type) const;

  /// True when the pose graph table exists and has at least one row.
  bool has_pose_graph() const;

  /// Per-edge solve metrics, written by `rux optimize`. The key may be given
  /// in either direction. nullopt writes NULL.
  struct PoseGraphEdgeMetrics {
    int from_node_id = 0;
    int to_node_id = 0;
    std::string edge_type;
    std::optional<double> residual;
    std::optional<double> deviation_m;
    std::optional<double> deviation_deg;
    std::optional<double> seed_disagreement_m;
    std::optional<double> seed_disagreement_deg;
  };

  /// Insert @p edge, or replace every column of the row with the same
  /// canonical key (from < to, edge_type). An edge given with from > to is
  /// canonicalised: transform inverted, information mapped through the
  /// adjoint (core::pose_graph::reverse_edge_information); metric magnitudes
  /// are kept as given. `weight` is derived from `information` when present.
  ///
  /// An upsert WITHOUT a measurement (`transform` nullopt) onto an existing
  /// row only changes its status: the stored transform, information, weight,
  /// residual, metrics and source are kept, and status "auto" never
  /// downgrades a "confirmed" row. A new key is inserted as given.
  ///
  /// @returns false — and writes nothing — when the stored row is "rejected"
  ///          and @p overwrite_rejected is false (tombstones survive
  ///          `rux graph discover`).
  /// @throws std::invalid_argument for from == to, an empty edge_type or
  ///         source, an invalid status, a non-rigid transform or an invalid
  ///         information; std::runtime_error on a read-only project.
  bool upsert_pose_graph_edge(const PoseGraphEdge &edge,
                              bool overwrite_rejected = false);

  /// Set "auto" | "confirmed" | "rejected" on the edge (either direction).
  /// @returns false when no such edge exists.
  /// @throws std::invalid_argument for any other status string.
  bool set_pose_graph_edge_status(int from_node_id, int to_node_id,
                                  std::string_view edge_type,
                                  std::string_view status);

  /// Write residual / deviation / seed-disagreement for many edges inside one
  /// SAVEPOINT (nests in a ProjectDB::Transaction). Unknown keys are skipped.
  /// @returns the number of rows updated.
  int update_pose_graph_edge_metrics(
      std::span<const PoseGraphEdgeMetrics> metrics);

  /// Delete every "auto" row with this source; tombstones and user
  /// confirmations stay.
  /// @returns the number of rows deleted.
  int delete_pose_graph_edges_from_source(std::string_view source);

  /// Superseded by upsert_pose_graph_edge (kept for ruxd, which builds against
  /// the installed package). Upserts with overwrite_rejected = true and
  /// source "manual" when the edge has none; returns the row id. Never
  /// downgrades a "confirmed" row to "auto", and (without a measurement)
  /// never touches a stored measurement.
  [[deprecated("use upsert_pose_graph_edge")]]
  int add_pose_graph_edge(const PoseGraphEdge &edge);

  /// Superseded by set_pose_graph_edge_status: marks the matching edges
  /// "rejected" (every type on the pair when @p edge_type is empty) instead
  /// of deleting them. @returns the number of rows marked.
  [[deprecated("use set_pose_graph_edge_status(..., \"rejected\")")]]
  int delete_pose_graph_edges(int from_node_id, int to_node_id,
                              std::string_view edge_type = "");

  // --- Project settings (schema v27) ---

  /// nullopt for an unknown key or a pre-v27 project.
  std::optional<std::string> setting(std::string_view key) const;
  /// Insert or replace. @throws std::runtime_error on a read-only project.
  void set_setting(std::string_view key, std::string_view value);

  // --- Scans (multi-session import, #129) ---

  /// One row of the `scans` table.  Each `rux import` creates one record;
  /// `id_offset` is the node-id watermark at import time so a re-import can be
  /// re-based, and `provenance_json` carries importer-specific metadata.
  struct ScanRecord {
    int id = 0;
    std::string source_path;
    std::string imported_at;
    int id_offset = 0;
    std::string provenance_json;
  };

  /// Creates a new scan record. Computes id_offset = MAX(node_id) from
  /// sensor_frames at call time (0 if empty). Warns if source_path already
  /// appears in the scans table (duplicate import guard).
  ScanRecord create_scan(const std::string &source_path,
                         const std::string &provenance_json = "{}");

  /// Returns all scan records ordered by id.
  std::vector<ScanRecord> scans() const;

  // --- Building Component Operations ---
  //
  // Persistence speaks only core::ComponentRecord (see
  // reusex/core/component_record.hpp), so core stays free of geometry types
  // (#227). To store/load a geometry::BuildingComponent, use the mapping and
  // convenience free functions in reusex/geometry/component_persistence.hpp.

  /// Insert or replace the component row identified by `record.name`.
  /// A guid is generated when `record.guid` is empty; an existing row's guid
  /// is never overwritten.
  void save_component_record(const core::ComponentRecord &record);
  /// Update an existing component's mutable fields (name, type, parent_id,
  /// confidence, metadata, notes), matched by its immutable guid. Geometry is
  /// left untouched. Throws if no component has the given guid.
  void update_component_record_by_guid(const core::ComponentRecord &record);
  /// Load the component row with the given name. Throws if absent.
  core::ComponentRecord component_record(std::string_view name) const;
  bool has_building_component(std::string_view name) const;
  void delete_building_component(std::string_view name);
  std::vector<std::string> list_building_components() const;
  /// Names of the components whose stored `type` discriminator equals `type`.
  std::vector<std::string>
  list_building_components(std::string_view type) const;
  int building_component_count() const;

  // --- Pipeline Log ---

  int log_pipeline_start(std::string_view stage,
                         std::string_view paramsJson = "");

  /// How a pipeline stage run ended. `cancelled` is distinct from `failed`
  /// so a user-requested cancel is never shown as an error.
  enum class PipelineOutcome { success, failed, cancelled };

  void log_pipeline_end(int logId, PipelineOutcome outcome,
                        std::string_view errorMsg = "");

  /// Shim for the two-outcome form: `success` maps to `PipelineOutcome::
  /// success`, `!success` to `PipelineOutcome::failed`. Prefer the
  /// `PipelineOutcome` overload when a cancel is possible.
  void log_pipeline_end(int logId, bool success,
                        std::string_view errorMsg = "");

  struct PipelineLogEntry {
    int id;
    std::string stage;
    std::string started_at;
    std::string finished_at; // Empty if still running
    std::string parameters;  // JSON string
    std::string status;      // "running", "success", "failed", "cancelled"
    std::string error_msg;   // Empty if no error
  };

  std::vector<PipelineLogEntry> pipeline_log(int limit = 0) const;

  // --- Raw table browsing (read-only) ---
  //
  // A generic, read-only window onto every table of the file, for a database
  // viewer (the Qt client's table view, `rux get`-style tooling). Typed
  // accessors above remain the way to *use* the data; these exist to *show*
  // what the file contains, including tables no accessor covers.
  //
  // Blob cells never carry their bytes: only their size and the first few
  // bytes (enough to sniff PNG / JPEG / PLY), read by incremental blob I/O,
  // which touches one page — so paging through `sensor_frames` costs
  // kilobytes, not the images. (A table WITHOUT ROWID has no blob I/O; its
  // blob heads come from substr(), which does read the whole value.) Long text
  // is truncated to `text_limit` bytes with its full size reported.

  /// One user table and its row count. `sqlite_*` internals are excluded.
  struct TableInfo {
    std::string name;
    std::int64_t row_count = 0;
  };

  /// One column of a table, as `PRAGMA table_info` reports it.
  struct TableColumn {
    std::string name;
    std::string declared_type; ///< e.g. "INTEGER", "BLOB"; may be empty
    bool primary_key = false;
    bool not_null = false;
  };

  /// One cell of table_rows().
  struct TableCell {
    enum class Kind { null, integer, real, text, blob };
    Kind kind = Kind::null;
    std::int64_t integer = 0;
    double real = 0.0;
    std::string text;       ///< text (possibly truncated), or blob head bytes
    std::uint64_t size = 0; ///< full byte size of text or blob
    bool truncated = false; ///< text shorter than `size`
  };

  /// Every user table, sorted by name, with its row count.
  std::vector<TableInfo> list_tables() const;

  /// Rows in @p table now (COUNT(*)).
  /// @throws std::invalid_argument when @p table is not a user table.
  std::int64_t table_row_count(std::string_view table) const;

  /// The columns of @p table, in declaration order.
  /// @throws std::invalid_argument when @p table is not a user table.
  std::vector<TableColumn> table_columns(std::string_view table) const;

  /// Rows [@p offset, @p offset + @p limit) of @p table in rowid order (or
  /// primary-key order for a WITHOUT ROWID table), one TableCell per column of
  /// table_columns(). Blob cells carry at most 16 head bytes in `text`.
  /// @throws std::invalid_argument when @p table is not a user table or
  ///         @p offset / @p limit is negative.
  std::vector<std::vector<TableCell>>
  table_rows(std::string_view table, std::int64_t offset, std::int64_t limit,
             std::size_t text_limit = 256) const;

  // --- Project Summary ---

  struct ProjectSummary {
    struct CloudInfo {
      std::string name;
      std::string type; // "PointXYZRGB", "Normal", "Label", "PointXYZ"
      size_t point_count;
      size_t width;
      size_t height;
      bool organized;                    // height > 1
      std::map<int, std::string> labels; // Only for Label clouds

      /// How points are ordered in storage.
      ///
      /// "morton_10bit" means points were sorted by a 30-bit Morton code
      /// (10 bits per axis) over the cloud's bounding box before chunking,
      /// so any stored prefix is a uniform spatial sample — it can be served
      /// directly as an LOD without a per-request voxel pass (#394).
      ///
      /// Empty string means sequential / insertion order, or unspecified
      /// (the "storage_order" key is absent from the parameters JSON).
      std::string storage_order;
    };

    struct MeshInfo {
      std::string name;
      int vertex_count;
      int face_count;
    };

    struct SensorFrameInfo {
      struct ScanInfo {
        int scan_id = 0;
        std::string source_path;
        std::string imported_at;
        int frame_count = 0;
      };

      int total_count;
      int width;                   // 0 if no frames
      int height;                  // 0 if no frames
      int segmented_count;         // frames with segmentation
      std::vector<ScanInfo> scans; // per-scan breakdown; empty if scans table
                                   // absent
    };

    struct PanoramicInfo {
      int total_count = 0;
      int matched_count = 0; // images linked to a sensor frame
    };

    struct ComponentInfo {
      int total_count = 0;
      std::map<std::string, int> count_by_type;
    };

    struct MaterialInfo {
      std::string id;
      std::string guid;
      int property_count = 0;
      std::string created_at;     // ISO 8601 timestamp
      std::string version_number; // Semver (e.g., "0.1.1")
    };

    struct ProjectInfo {
      std::string id;
      std::string name;
      std::string building_address;
      int year_of_construction = 0; // 0 = not set
      std::string survey_date;
      std::string survey_organisation;
      std::string notes;
    };

    std::filesystem::path path;
    int schema_version;
    std::vector<ProjectInfo> projects;
    std::vector<CloudInfo> clouds;
    std::vector<MeshInfo> meshes;
    /// Metadata only — the summary never carries a splat's payload, which is
    /// routinely hundreds of MB. Empty on a pre-v12 schema.
    std::vector<GaussianSplatMetadata> gaussian_splats;
    SensorFrameInfo sensor_frames;
    PanoramicInfo panoramic_images;
    ComponentInfo components;
    std::vector<MaterialInfo> materials;
  };

  ProjectSummary project_summary() const;

  /**
   * @brief A user-defined column in the Notion-like material editor.
   *
   * Backed by the `material_property_definitions` table (schema v18). These
   * describe the editable columns the GUI presents over material passports;
   * they are distinct from the leksikon-based `property_definitions` table.
   */
  struct PropertyDefinition {
    std::string id;
    std::string type; // "text" | "number" | "date" | "boolean" | "select" |
                      // "multiselect"
    std::string name;
    std::vector<std::string> options; // populated for "select"/"multiselect"
    int sort_order = 0;
    int width = 200; // display column width in pixels (schema v19)
  };

  // --- Material Property Definition Operations (schema v18) ---

  /**
   * @brief List all user-defined material column definitions.
   * @return Definitions ordered by sort_order then created_at.
   */
  [[nodiscard]] std::vector<PropertyDefinition>
  list_property_definitions() const;

  /**
   * @brief Add a new material column definition.
   * @param name Column display name
   * @param type One of text/number/date/boolean/select
   * @param options Choices, only meaningful for the "select" type
   * @param sort_order Display order
   * @return The freshly minted definition id (GUID)
   */
  std::string add_property_definition(const std::string &name,
                                      const std::string &type,
                                      const std::vector<std::string> &options,
                                      int sort_order, int width = 200);

  /**
   * @brief Overwrite an existing material column definition.
   * @throws std::runtime_error if the id does not exist
   */
  void update_property_definition(const std::string &id,
                                  const std::string &name,
                                  const std::string &type,
                                  const std::vector<std::string> &options,
                                  int sort_order, int width);

  /**
   * @brief Delete a material column definition by id.
   * @throws std::runtime_error if the id does not exist
   */
  void delete_property_definition(const std::string &id);

  /**
   * @brief Fetch a material's thumbnail blob and its MIME type.
   * @param guid Material passport document GUID
   * @return {blob, mime_type} when a thumbnail exists, else std::nullopt
   */
  [[nodiscard]] std::optional<std::pair<std::vector<std::uint8_t>, std::string>>
  material_thumbnail(const std::string &guid) const;

  /**
   * @brief Insert or replace a material's thumbnail blob.
   * @param guid Material passport document GUID
   * @param blob Raw image bytes
   * @param mime_type Image MIME type (e.g. "image/jpeg")
   */
  void set_material_thumbnail(const std::string &guid,
                              const std::vector<std::uint8_t> &blob,
                              const std::string &mime_type);

  /**
   * @brief Delete a material's thumbnail if present (no-op when absent).
   */
  void delete_material_thumbnail(const std::string &guid);

  // --- Material Passport Operations ---

  core::MaterialPassport material_passport(std::string_view documentGuid) const;
  std::vector<core::MaterialPassport> all_material_passports() const;

  void add_material_passport(const core::MaterialPassport &passport,
                             std::string_view projectId);

  /**
   * @brief Add a material passport with a custom row ID
   *
   * The id parameter overrides the material_passports.id column value
   * (normally set to document_guid). Use this to link a passport to a
   * sensor frame by setting id to the frame's node_id.
   *
   * @param passport Material passport data
   * @param projectId Project identifier (may be empty)
   * @param id Custom row ID for the material_passports.id column
   */
  void add_material_passport(const core::MaterialPassport &passport,
                             std::string_view projectId, std::string_view id);

  /**
   * @brief Delete a material passport by GUID
   * @param documentGuid Document GUID to delete
   * @throws std::runtime_error if passport does not exist
   */
  void delete_material_passport(std::string_view documentGuid);

  /**
   * @brief List document GUIDs ordered by created_at
   * @return Vector of document GUID strings
   */
  std::vector<std::string> list_passport_guids() const;

  /**
   * @brief Get stored property field_name→string pairs for a passport
   *
   * Only includes properties that actually have rows in
   * passport_property_values. Values are returned as human-readable strings
   * (BLOB decoded via as_string()).
   *
   * @param documentGuid Document GUID
   * @return Map of name_en → string value
   * @throws std::runtime_error if passport does not exist
   */
  std::map<std::string, std::string>
  passport_stored_properties(std::string_view documentGuid) const;

  /**
   * @brief Get a single passport property value by field name
   * @param documentGuid Document GUID
   * @param fieldName Property field name (name_en in property_definitions)
   * @return String value of the property
   * @throws std::runtime_error if passport or property not found
   */
  std::string passport_property_value(std::string_view documentGuid,
                                      std::string_view fieldName) const;

  /**
   * @brief Return the sensor frame node_id this passport was linked to
   *
   * When a passport is imported via "rux import photos" and matched to a
   * sensor frame, material_passports.id is set to the frame's node_id
   * (as decimal text). This helper returns that id parsed as an integer,
   * or std::nullopt when the passport was stored without a link (id falls
   * back to the document_guid).
   *
   * @param documentGuid Document GUID
   * @return Node id when the row id is a non-negative integer, else
   *         std::nullopt
   * @throws std::runtime_error if passport not found
   */
  std::optional<int>
  passport_linked_node_id(std::string_view documentGuid) const;

  /**
   * @brief Get passport metadata without loading all properties
   * @param documentGuid Document GUID
   * @return MaterialPassportMetadata struct
   * @throws std::runtime_error if passport not found
   */
  core::MaterialPassportMetadata
  passport_metadata(std::string_view documentGuid) const;

  /**
   * @brief Set a metadata column on a material passport
   *
   * Supported columns: created_at, revised_at, version_number, version_date.
   * document_guid cannot be changed (it is the primary key).
   *
   * @param documentGuid Document GUID (must exist)
   * @param column Metadata column name
   * @param value New value
   * @throws std::runtime_error if passport not found or column not allowed
   */
  void set_passport_metadata_field(std::string_view documentGuid,
                                   std::string_view column,
                                   std::string_view value);

  /**
   * @brief Set a single property value by field name (upsert)
   *
   * Looks up the property_definitions entry by name_en, then upserts
   * into passport_property_values. Creates property_definitions entry if
   * needed.
   *
   * @param documentGuid Document GUID (must exist)
   * @param fieldName Property field name (name_en)
   * @param value String value to store
   * @throws std::runtime_error if passport does not exist
   */
  void set_passport_property(std::string_view documentGuid,
                             std::string_view fieldName,
                             std::string_view value);

  /**
   * @brief Delete a single property value by field name
   * @param documentGuid Document GUID
   * @param fieldName Property field name (name_en)
   * @throws std::runtime_error if passport or property not found
   */
  void delete_passport_property(std::string_view documentGuid,
                                std::string_view fieldName);

  // --- Report PDF Storage (schema v20) ---

  /**
   * @brief Metadata for one stored report PDF version.
   * The PDF blob itself is fetched separately via report_pdf(id).
   */
  struct ReportPdfRecord {
    int64_t id = 0;
    /// UTC, as sqlite's datetime('now') stores it: "YYYY-MM-DD HH:MM:SS"
    /// (no 'T', no zone suffix).
    std::string created_at;
    std::string label; // human-readable tag (e.g. "Ressourcekortlægning")
    std::size_t size_bytes = 0;
    /// 1-based position in generation order (v1 is the first PDF ever made).
    int version = 0;
    /// Survey types that still blocked the report when it was generated;
    /// nullopt for versions made before schema v23.
    std::optional<int> blocking_types;
  };

  /**
   * @brief Store a PDF blob and return its record (id, created_at, size).
   * @param pdf Raw PDF bytes
   * @param label Optional human-readable label stored alongside the blob
   * @param blocking_types Survey types that still blocked the report
   *        (schema v23); nullopt stores NULL
   * @return The new record (does not include the blob payload)
   */
  ReportPdfRecord
  add_report_pdf(const std::vector<std::uint8_t> &pdf,
                 const std::string &label = "",
                 std::optional<int> blocking_types = std::nullopt);

  /**
   * @brief List all stored report PDFs, newest first.
   * Does NOT return the blob payload — call report_pdf(id) for that.
   */
  [[nodiscard]] std::vector<ReportPdfRecord> list_report_pdfs() const;

  /**
   * @brief Fetch one stored PDF blob by id.
   * @return The raw PDF bytes, or std::nullopt if the id does not exist.
   */
  [[nodiscard]] std::optional<std::vector<std::uint8_t>>
  report_pdf(int64_t id) const;

  // --- Resource templates (schema v25) ---
  /// One row of `templates` (resources/templates spec §5.1). members_json
  /// and csv_json are parsed by core/resource_templates.hpp.
  struct ResourceTemplateRecord {
    int64_t id = 0;
    std::string name;
    std::string members_json = "[]";
    std::string csv_json = "{}";
    std::optional<std::string> seed; // "materialepas" | "screening"
    std::string created_at;          // ISO 8601 UTC
    std::string updated_at;
  };
  struct ResourceTemplatePatch {
    std::optional<std::string> name, members_json, csv_json;
  };
  /// By id. Empty on a read-only open of a pre-v25 project.
  [[nodiscard]] std::vector<ResourceTemplateRecord> resource_templates() const;
  [[nodiscard]] std::optional<ResourceTemplateRecord>
  resource_template(int64_t id) const;
  /// id/timestamps are ignored. @throws core::NameConflictError
  ResourceTemplateRecord
  add_resource_template(const ResourceTemplateRecord &rec);
  /// @throws std::out_of_range, core::NameConflictError
  ResourceTemplateRecord
  update_resource_template(int64_t id, const ResourceTemplatePatch &patch);
  bool delete_resource_template(int64_t id); // false when absent

  // --- Survey (Ressourcekortlægning, schema v22) ---
  struct SurveyTypeRecord {
    int64_t id = 0;
    std::string name;
    std::string eak_code;
    std::string bim7aa_code;
    std::string unit = "stk";
    core::Treatment treatment = core::Treatment::genanvendelse;
    core::ReviewStatus review_status = core::ReviewStatus::queue;
    std::optional<double> confidence; // 0..1, AI detection confidence
    std::optional<double> mass_t;     // tonnes
    std::string note;
    bool starred = false;
    int semantic_class = -1; // the class sync_survey seeded it from
    std::string created_at;  // ISO 8601 UTC
    std::string updated_at;
  };
  /// Sparse update; an engaged optional sets the field. For the nullable
  /// numbers the outer optional says "change it", the inner one the new value.
  struct SurveyTypePatch {
    std::optional<std::string> name, eak_code, bim7aa_code, unit, note;
    std::optional<core::Treatment> treatment;
    // Unchecked storage write — set approval through core::set_review_status,
    // which enforces the sample gate.
    std::optional<core::ReviewStatus> review_status;
    std::optional<std::optional<double>> confidence, mass_t;
    std::optional<bool> starred;
  };
  struct SurveyPartRecord {
    std::string code; // "RX-001"
    int64_t type_id = 0;
    std::optional<std::string>
        cloud_name; // read-only, derived from instance_guid
    std::optional<std::uint32_t>
        instance_id; // read-only, derived from instance_guid
    std::optional<std::uint32_t> room_id;
    std::string room_name;
    double quantity = 1.0;
    bool starred = false;
    std::string note;
    std::optional<std::string>
        material_guid; // read-only: the part's passport (survey_parts.
                       // passport_guid, schema v25), else its instance's
                       // instance_materials link — unless another part
                       // owns that passport, then empty
    /// Stable link to `instances.guid` (schema v22); survives `rux create
    /// instances` re-runs because instance identity is reconciled on guid,
    /// not (cloud_id, instance_id). Nullable: a manually added part needs no
    /// instance. Read-only on a patch; set via cloud_name/instance_id on add.
    /// Kept LAST so existing positional aggregate initializers still compile.
    std::optional<std::string> instance_guid{};
  };
  struct SurveyPartPatch {
    std::optional<int64_t> type_id;
    std::optional<double> quantity;
    std::optional<bool> starred;
    std::optional<std::string> note, room_name;
  };

  SurveyTypeRecord
  add_survey_type(const SurveyTypeRecord &rec); // id/timestamps ignored
  [[nodiscard]] std::vector<SurveyTypeRecord> survey_types() const; // by id
  [[nodiscard]] std::optional<SurveyTypeRecord> survey_type(int64_t id) const;
  SurveyTypeRecord update_survey_type(int64_t id, const SurveyTypePatch &patch);
  void add_survey_part(const SurveyPartRecord &rec);
  [[nodiscard]] std::vector<SurveyPartRecord> survey_parts() const; // by code
  [[nodiscard]] std::optional<SurveyPartRecord>
  survey_part(std::string_view code) const;
  SurveyPartRecord update_survey_part(std::string_view code,
                                      const SurveyPartPatch &patch);
  /// Set several parts' quantities in one transaction — all or nothing.
  /// Storage only; the redistribution rule lives in core/survey_service.
  void set_survey_part_quantities(
      const std::vector<std::pair<std::string, double>> &code_quantities);
  [[nodiscard]] bool has_survey_part_for(std::string_view cloud_name,
                                         std::uint32_t instance_id) const;
  [[nodiscard]] int max_survey_part_number() const; // 0 when there are none

  /// Resource storage (schema v25). Call inside a Transaction. Returns the
  /// part's passport guid; on first use creates a bare passport (or adopts
  /// the instance's linked passport when no other part owns it), sets
  /// survey_parts.passport_guid, upserts instance_materials for an
  /// instance-backed part, and makes sure the leksikon property_definitions
  /// exist. @throws std::out_of_range when @p code is unknown.
  std::string ensure_resource_passport(std::string_view code);
  /// @throws std::out_of_range when @p code is unknown.
  void delete_survey_part(std::string_view code);
  /// Deletes a survey type; its parts and sample links cascade. Storage only:
  /// tombstones and passports are core::delete_survey_type's job.
  /// @throws std::out_of_range when @p id is unknown.
  void delete_survey_type(int64_t id);

  /// Tombstones (schema v26, survey_dismissed_instances): instance guids
  /// whose survey part the user deleted, so sync_survey does not re-create
  /// it. Idempotent — dismissing a guid twice keeps the first timestamp.
  void dismiss_instance(std::string_view instance_guid);
  /// False on a read-only open of a pre-v26 project (no table).
  [[nodiscard]] bool
  is_instance_dismissed(std::string_view instance_guid) const;
  /// Every dismissed instance guid, ascending. Empty before v26.
  [[nodiscard]] std::vector<std::string> dismissed_instances() const;
  /// Removes the instance_materials link of (@p cloud_name, @p instance_id).
  /// Returns false when there was none (or the cloud is unknown).
  bool unlink_instance_material(std::string_view cloud_name, int instance_id);
  /// Call inside a Transaction. Moves the values stored under a user
  /// column's field name to @p new_name; a no-op when nothing is stored.
  /// @throws core::NameConflictError when values already exist under
  ///         @p new_name (they would merge).
  void rename_passport_field(std::string_view old_name,
                             std::string_view new_name);
  /// True when any passport stores a value under the field name @p name
  /// (a leksikon name_en or a user column's display name) — including
  /// values a deleted user column left behind.
  [[nodiscard]] bool has_passport_field_values(std::string_view name) const;
  /// Call inside a Transaction. Deletes every value stored under the user
  /// column field name @p name, and its "custom:" definition, so the name
  /// is free again. Leksikon fields are never touched. Returns the number
  /// of values deleted.
  std::size_t delete_passport_field_values(std::string_view name);
  /// True when a survey part or an instance link still references @p guid.
  [[nodiscard]] bool is_passport_linked(std::string_view guid) const;

  struct SampleRecord {
    int64_t id = 0;
    std::string code;  // "P-01"
    std::string title; // "PCB i fugemasse"
    std::string what;  // what was sampled, where
    core::SampleStage stage = core::SampleStage::planlagt;
    core::SampleResult result = core::SampleResult::none;
    std::vector<int64_t> type_ids; // linked survey types, ascending
    std::string created_at;
    std::string updated_at;
  };
  struct SamplePatch {
    std::optional<std::string> title, what;
    std::optional<core::SampleStage> stage;
    std::optional<core::SampleResult> result;
  };
  /// Register a sample, its type links and its initial stage in one
  /// transaction (duplicate type ids collapse).
  /// @throws std::out_of_range when a `type_ids` entry names no survey type.
  /// @throws std::invalid_argument when `stage` is not planlagt/udtaget.
  /// Nothing is written (and no P-## code consumed) when it throws.
  SampleRecord
  add_sample(std::string_view title, std::string_view what,
             const std::vector<int64_t> &type_ids = {},
             core::SampleStage stage = core::SampleStage::planlagt);
  [[nodiscard]] std::vector<SampleRecord> samples() const; // by id
  [[nodiscard]] std::optional<SampleRecord> sample(int64_t id) const;
  SampleRecord
  update_sample(int64_t id,
                const SamplePatch &patch); // storage only; no stage rules
  bool delete_sample(int64_t id);          // false when absent
  void
  set_sample_links(int64_t id,
                   const std::vector<int64_t> &type_ids); // replaces the set
  [[nodiscard]] std::vector<SampleRecord>
  samples_for_type(int64_t type_id) const;

  // --- Project Metadata Operations ---

  struct ProjectMetadata {
    std::string id;
    std::string name;
    std::string building_address;
    int year_of_construction = 0; // 0 = not set
    std::string survey_date;
    std::string survey_organisation;
    std::string notes;
  };

  /**
   * @brief Get project metadata by project ID
   * @param projectId Project identifier
   * @return Project metadata
   * @throws std::runtime_error if project does not exist
   */
  ProjectMetadata get_project_metadata(std::string_view projectId) const;

  /**
   * @brief Update project metadata
   * @param metadata Project metadata to update
   * Creates project if it doesn't exist
   */
  void update_project_metadata(const ProjectMetadata &metadata);

  /**
   * @brief List all project IDs in the database
   * @return Vector of project IDs
   */
  std::vector<std::string> list_project_ids() const;

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace reusex
