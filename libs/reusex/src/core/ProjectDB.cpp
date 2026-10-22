// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/ProjectDB.hpp"
#include "core/MaterialPassport.hpp"
#include "core/SensorIntrinsics.hpp"
#include "core/gaussian_splat.hpp"
#include "core/guid.hpp"
#include "core/label_encoding.hpp"
#include "core/label_semantics.hpp"
#include "core/logging.hpp"
#include "core/materialepas_serialization.hpp"
#include "core/materialepas_traits.hpp"
#include "core/resource_keys.hpp"
#include "core/resource_templates.hpp"
#include "reusex/core/buildups.hpp"
#include "reusex/core/pose_graph.hpp"
#include "reusex/core/space_polygon.hpp"

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <sqlite3.h>

#include <fmt/ranges.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <random>
#include <regex>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <unistd.h>

namespace reusex {

// Generate a UUID-v4-like identifier for building components / instances.
static std::string generate_component_guid() { return core::generate_guid(); }

// ── RAII helper for sqlite3_stmt ────────────────────────────────────────
class StmtGuard {
  sqlite3_stmt *stmt_;

    public:
  explicit StmtGuard(sqlite3_stmt *s) : stmt_(s) {}
  ~StmtGuard() {
    if (stmt_)
      sqlite3_finalize(stmt_);
  }
  StmtGuard(const StmtGuard &) = delete;
  StmtGuard &operator=(const StmtGuard &) = delete;
  sqlite3_stmt *get() const { return stmt_; }
};

// ── RAII helper for sqlite3_blob (incremental blob I/O) ─────────────────
class BlobGuard {
  sqlite3_blob *blob_;

    public:
  explicit BlobGuard(sqlite3_blob *b) : blob_(b) {}
  ~BlobGuard() {
    if (blob_)
      sqlite3_blob_close(blob_);
  }
  BlobGuard(const BlobGuard &) = delete;
  BlobGuard &operator=(const BlobGuard &) = delete;
  sqlite3_blob *get() const { return blob_; }
};

// ── Small sqlite3 column/bind helpers (survey CRUD, schema v22) ─────────
namespace {
std::string column_text(sqlite3_stmt *s, int i) {
  const auto *t = reinterpret_cast<const char *>(sqlite3_column_text(s, i));
  return t ? t : "";
}
std::optional<double> column_opt_double(sqlite3_stmt *s, int i) {
  if (sqlite3_column_type(s, i) == SQLITE_NULL)
    return std::nullopt;
  return sqlite3_column_double(s, i);
}
std::optional<std::int64_t> column_opt_int64(sqlite3_stmt *s, int i) {
  if (sqlite3_column_type(s, i) == SQLITE_NULL)
    return std::nullopt;
  return sqlite3_column_int64(s, i);
}
void bind_opt_double(sqlite3_stmt *s, int i, const std::optional<double> &v) {
  v ? sqlite3_bind_double(s, i, *v) : sqlite3_bind_null(s, i);
}
void bind_text(sqlite3_stmt *s, int i, std::string_view v) {
  sqlite3_bind_text(s, i, v.data(), static_cast<int>(v.size()),
                    SQLITE_TRANSIENT);
}
/// True when @p db has a non-empty `-wal` beside it: committed pages may live
/// only there, so the main file alone is not the whole database.
bool has_live_wal(const std::filesystem::path &db) {
  std::error_code ec;
  const auto size = std::filesystem::file_size(db.string() + "-wal", ec);
  return !ec && size > 0;
}

/// A read-only sqlite URI for @p db. With @p immutable, sqlite treats the file
/// as unchangeable: it takes no locks and never opens or creates the `-shm`
/// / `-wal` sidecars, so the open has no effect on the filesystem and works
/// in a read-only directory. The price: a concurrent writer is not seen and
/// could even be read mid-write, so it is only right when nobody can be
/// writing (an unwritable directory) or for a quick look (probe()), and only
/// without a live WAL (has_live_wal), whose pages immutable would ignore.
std::string read_only_uri(const std::filesystem::path &db, bool immutable) {
  std::string uri = "file:";
  for (char c : std::filesystem::absolute(db).string()) {
    // RFC 3986 reserved characters sqlite's URI parser interprets.
    if (c == '%' || c == '?' || c == '#') {
      static const char hex[] = "0123456789ABCDEF";
      uri += '%';
      uri += hex[(static_cast<unsigned char>(c) >> 4) & 0xF];
      uri += hex[static_cast<unsigned char>(c) & 0xF];
    } else {
      uri += c;
    }
  }
  uri += immutable ? "?mode=ro&immutable=1" : "?mode=ro";
  return uri;
}

/// Whether the current user may create files in @p db's directory (sqlite's
/// WAL needs to, for `-shm`, even to read).
bool directory_writable(const std::filesystem::path &db) {
  const auto dir = std::filesystem::absolute(db).parent_path();
  return ::access(dir.c_str(), W_OK) == 0;
}

/// prepare_v2 that throws with the caller's name and sqlite's message.
sqlite3_stmt *prepare_or_throw(sqlite3 *db, const char *sql, const char *who) {
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error(std::string(who) +
                             ": prepare failed: " + sqlite3_errmsg(db));
  return stmt;
}

/// SAVEPOINT scope: atomic on its own, nests inside a ProjectDB::Transaction.
/// Rolled back to (and released) unless release() is called.
class Savepoint {
  sqlite3 *db_;
  const char *name_;
  bool open_ = true;

  void exec(const std::string &sql) {
    char *err = nullptr;
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err) != SQLITE_OK) {
      std::string msg = err ? err : "unknown error";
      sqlite3_free(err);
      throw std::runtime_error(sql + " failed: " + msg);
    }
  }

    public:
  Savepoint(sqlite3 *db, const char *name) : db_(db), name_(name) {
    exec(std::string("SAVEPOINT ") + name_ + ";");
  }
  ~Savepoint() {
    if (!open_)
      return;
    const std::string sql =
        std::string("ROLLBACK TO ") + name_ + "; RELEASE " + name_ + ";";
    if (sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, nullptr) != SQLITE_OK)
      reusex::warn("{}: rollback to savepoint failed: {}", name_,
                   sqlite3_errmsg(db_));
  }
  void release() {
    exec(std::string("RELEASE ") + name_ + ";");
    open_ = false;
  }
  Savepoint(const Savepoint &) = delete;
  Savepoint &operator=(const Savepoint &) = delete;
};

/// Spec §5.1, verbatim. IF NOT EXISTS because tests that roll a project
/// back past v25 without dropping the table re-run migrateToV25.
constexpr const char *kTemplatesSchema = R"(
  CREATE TABLE IF NOT EXISTS templates (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    name       TEXT NOT NULL UNIQUE,
    members    TEXT NOT NULL DEFAULT '[]',
    csv        TEXT NOT NULL DEFAULT '{}',
    seed       TEXT,
    created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
    updated_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
  );
)";

/// Export-template `columns` (schema v21 config) as key members, via
/// core::legacy_column_member. First position wins: a column that maps to a
/// member already taken is dropped, as are non-string entries; both are
/// counted in one warn naming @p who (STANDARDS §5).
std::vector<core::TemplateMember>
column_members(const nlohmann::json &columns,
               const std::vector<core::ResourceKey> &catalogue,
               std::string_view who) {
  std::vector<core::TemplateMember> out;
  std::size_t non_string = 0, repeated = 0;
  for (const auto &c : columns) {
    if (!c.is_string()) {
      ++non_string;
      continue;
    }
    auto m = core::legacy_column_member(c.get<std::string>(), catalogue);
    if (std::find(out.begin(), out.end(), m) != out.end()) {
      ++repeated;
      continue;
    }
    out.push_back(std::move(m));
  }
  if (non_string + repeated > 0)
    reusex::warn("{}: skipped {} non-string and {} repeated column(s) of {}",
                 who, non_string, repeated, columns.size());
  return out;
}
} // namespace

// ── Compact point serialization helpers ─────────────────────────────────

static constexpr int XYZRGB_STEP = 16; // 3 float + 1 uint32
static constexpr int NORMAL_STEP = 16; // 3 float + curvature
static constexpr int LABEL_STEP = 4;   // 1 uint32
static constexpr int XYZ_STEP = 12;    // 3 float

static std::vector<uint8_t> serializeXYZRGB(const Cloud &cloud) {
  std::vector<uint8_t> buf(cloud.size() * XYZRGB_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.x, sizeof(float));
    std::memcpy(dst + 4, &pt.y, sizeof(float));
    std::memcpy(dst + 8, &pt.z, sizeof(float));
    std::memcpy(dst + 12, &pt.rgba, sizeof(uint32_t));
    dst += XYZRGB_STEP;
  }
  return buf;
}

static CloudPtr deserializeXYZRGB(const void *data, size_t size, uint32_t width,
                                  uint32_t height) {
  auto cloud = std::make_shared<Cloud>();
  size_t count = size / XYZRGB_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.x, src, sizeof(float));
    std::memcpy(&pt.y, src + 4, sizeof(float));
    std::memcpy(&pt.z, src + 8, sizeof(float));
    std::memcpy(&pt.rgba, src + 12, sizeof(uint32_t));
    src += XYZRGB_STEP;
  }
  return cloud;
}

static std::vector<uint8_t> serializeNormal(const CloudN &cloud) {
  std::vector<uint8_t> buf(cloud.size() * NORMAL_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.normal_x, sizeof(float));
    std::memcpy(dst + 4, &pt.normal_y, sizeof(float));
    std::memcpy(dst + 8, &pt.normal_z, sizeof(float));
    std::memcpy(dst + 12, &pt.curvature, sizeof(float));
    dst += NORMAL_STEP;
  }
  return buf;
}

static CloudNPtr deserializeNormal(const void *data, size_t size,
                                   uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<CloudN>();
  size_t count = size / NORMAL_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.normal_x, src, sizeof(float));
    std::memcpy(&pt.normal_y, src + 4, sizeof(float));
    std::memcpy(&pt.normal_z, src + 8, sizeof(float));
    std::memcpy(&pt.curvature, src + 12, sizeof(float));
    src += NORMAL_STEP;
  }
  return cloud;
}

static std::vector<uint8_t> serializeLabel(const CloudL &cloud) {
  std::vector<uint8_t> buf(cloud.size() * LABEL_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.label, sizeof(uint32_t));
    dst += LABEL_STEP;
  }
  return buf;
}

static CloudLPtr deserializeLabel(const void *data, size_t size, uint32_t width,
                                  uint32_t height) {
  auto cloud = std::make_shared<CloudL>();
  size_t count = size / LABEL_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    std::memcpy(&cloud->points[i].label, src, sizeof(uint32_t));
    src += LABEL_STEP;
  }
  return cloud;
}

static std::vector<uint8_t>
serializeXYZ(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  std::vector<uint8_t> buf(cloud.size() * XYZ_STEP);
  auto *dst = buf.data();
  for (const auto &pt : cloud.points) {
    std::memcpy(dst, &pt.x, sizeof(float));
    std::memcpy(dst + 4, &pt.y, sizeof(float));
    std::memcpy(dst + 8, &pt.z, sizeof(float));
    dst += XYZ_STEP;
  }
  return buf;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr
deserializeXYZ(const void *data, size_t size, uint32_t width, uint32_t height) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  size_t count = size / XYZ_STEP;
  cloud->points.resize(count);
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  auto *src = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < count; ++i) {
    auto &pt = cloud->points[i];
    std::memcpy(&pt.x, src, sizeof(float));
    std::memcpy(&pt.y, src + 4, sizeof(float));
    std::memcpy(&pt.z, src + 8, sizeof(float));
    src += XYZ_STEP;
  }
  return cloud;
}

// ── Hardening ───────────────────────────────────────────────────────────

namespace {

std::atomic<bool> g_hardened_by_default{false};

/// Applies ProjectDB::OpenOptions::hardened to a fresh connection, before any
/// statement reads the schema. Each setting is checked: a connection that
/// silently stayed unhardened would be worse than a failed open.
void harden_connection(sqlite3 *db) {
  const struct {
    int op;
    int value;
    const char *name;
  } settings[] = {
      {SQLITE_DBCONFIG_DEFENSIVE, 1, "DEFENSIVE"},
      {SQLITE_DBCONFIG_TRUSTED_SCHEMA, 0, "TRUSTED_SCHEMA"},
      // ReUseX never creates a trigger or a view; one in a file is foreign.
      {SQLITE_DBCONFIG_ENABLE_TRIGGER, 0, "ENABLE_TRIGGER"},
      {SQLITE_DBCONFIG_ENABLE_VIEW, 0, "ENABLE_VIEW"},
  };
  for (const auto &setting : settings) {
    int now = -1;
    if (sqlite3_db_config(db, setting.op, setting.value, &now) != SQLITE_OK ||
        now != setting.value)
      throw std::runtime_error(std::string("Cannot harden the connection: "
                                           "SQLITE_DBCONFIG_") +
                               setting.name + " was not applied");
  }
}

} // namespace

// ── Impl ────────────────────────────────────────────────────────────────

class ProjectDB::Impl {
    public:
  std::filesystem::path dbPath;
  bool readOnly;
  bool hardened = false;
  sqlite3 *db = nullptr;
  /// survey_parts.passport_guid exists (schema v25). Set once at the end of
  /// configure(): a read-write open has migrated, a read-only open of an
  /// older project has not, and neither changes for this connection.
  bool hasPassportColumn = false;
  /// survey_dismissed_instances exists (schema v26); same lifetime rule.
  bool hasDismissedTable = false;
  /// survey_parts.view_* exist (schema v27); same lifetime rule.
  bool hasViewColumns = false;
  /// pose_graph_edges has the v28 columns. Same lifetime rule as above.
  bool hasPoseGraphV28 = false;
  /// sensor_frames.initial_transform exists (v28).
  bool hasInitialTransform = false;
  /// project_settings exists (v28).
  bool hasSettingsTable = false;
  /// storeys exists (v30).
  bool hasStoreysTable = false;
  /// plane_classes exists (v31).
  bool hasPlaneClassesTable = false;
  /// sample_part_links exists (v32).
  bool hasSamplePartLinks = false;
  /// walls and wall_joins exist (v33).
  bool hasWallsTable = false;
  /// openings exists (v34).
  bool hasOpeningsTable = false;
  /// spaces and space_boundaries exist (v35).
  bool hasSpacesTable = false;
  /// survey_parts.dismissed exists (v36).
  bool hasPartDismissedColumn = false;
  /// Survey mass (v37): survey_parts.element_guid/geom_*, survey_types.
  /// buildup_id/element_category, buildups, value_provenance.
  bool hasSurveyMass = false;
  /// report_assets exists (v38).
  bool hasReportAssetsTable = false;

  // cppcheck-suppress unusedStructMember
  static constexpr int LATEST_SCHEMA_VERSION = 38;

  // How long one statement waits for a lock another connection holds before
  // failing with SQLITE_BUSY. sqlite's default is not to wait at all.
  static constexpr int BUSY_TIMEOUT_MS = 5000;

  // Maximum bytes per point_cloud_data row. SQLite's default SQLITE_MAX_LENGTH
  // is 1 GB and the hard compile-time max is 2 GB-1. We chunk large clouds
  // into multiple rows so a single cloud can exceed those per-row limits.
  static constexpr size_t POINT_CLOUD_CHUNK_BYTES = 256ull * 1024 * 1024;

  // Maximum bytes per gaussian_splat_data row. Deliberately far smaller than
  // POINT_CLOUD_CHUNK_BYTES: point clouds are read back through incremental
  // blob I/O (point_cloud_page), whereas a splat is handed out whole, so the
  // chunk size is what bounds the transient copy sqlite makes per row rather
  // than a per-row storage ceiling.
  static constexpr size_t GAUSSIAN_SPLAT_CHUNK_BYTES = 8ull * 1024 * 1024;

  void execOrThrow(const char *sql) {
    char *errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error(std::string("SQL exec failed: ") + error);
    }
  }

  Impl(std::filesystem::path path, bool ro, bool harden)
      : dbPath(std::move(path)), readOnly(ro), hardened(harden) {
    reusex::info("Opening ReUseX database: {}", dbPath);

    // Open sqlite3 connection for project database
    int flags = readOnly ? SQLITE_OPEN_READONLY
                         : (SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE);
    std::string target = dbPath.string();
    // A WAL-mode project in a directory we cannot write is unreadable through
    // a plain read-only open: sqlite must create `-shm` even to SELECT. Nobody
    // writes there through this user either, so open it immutable (no locks,
    // no sidecars) unless a live WAL holds pages the main file lacks.
    if (readOnly && !directory_writable(dbPath) && !has_live_wal(dbPath)) {
      target = read_only_uri(dbPath, /*immutable=*/true);
      flags |= SQLITE_OPEN_URI;
    }
    if (sqlite3_open_v2(target.c_str(), &db, flags, nullptr) != SQLITE_OK) {
      std::string error = sqlite3_errmsg(db);
      sqlite3_close(db);
      throw std::runtime_error("Cannot open database: " + error);
    }

    // ~Impl never runs for a constructor that throws, so anything that fails
    // from here on (a probe that finds the file locked, a migration) must
    // close the handle itself. Leaking it would also leak its WAL read lock,
    // once per failed open — one per 503 in `ruxd --local`.
    try {
      if (hardened)
        harden_connection(db);
      configure();
    } catch (...) {
      sqlite3_close_v2(db);
      db = nullptr;
      throw;
    }

    reusex::info("Project database opened successfully");
  }

  ~Impl() {
    reusex::trace("Closing project database connection");
    if (db) {
      sqlite3_close(db);
    }
  }

  /// Everything the constructor does after the handle is open: pragmas, the
  /// busy timeout, and the schema create / migrate (read-write) or the schema
  /// check (read-only). May throw; the constructor then closes the handle.
  void configure() {
    // Every connection waits out a briefly held lock instead of failing at
    // once. With WAL, readers never block each other, but a reader whose open
    // races another connection's close (which takes the file lock to try a
    // checkpoint) or its WAL-index rebuild got SQLITE_BUSY immediately — the
    // 503s on a full web GUI page load. Writers never block WAL readers, so
    // the wait only ever covers those millisecond windows; a lock held past
    // the timeout is still reported.
    sqlite3_busy_timeout(db, BUSY_TIMEOUT_MS);

    if (!readOnly) {
      // Performance pragmas for write mode
      sqlite3_exec(db, "PRAGMA journal_mode = WAL;", nullptr, nullptr, nullptr);
      sqlite3_exec(db, "PRAGMA synchronous = NORMAL;", nullptr, nullptr,
                   nullptr);
      sqlite3_exec(db, "PRAGMA foreign_keys = ON;", nullptr, nullptr, nullptr);

      // Capture legacy state BEFORE createTables() adds material_passports
      bool isLegacyDb =
          !tableExists("schema_version") && tableExists("material_passports");

      // Create legacy passport tables (always needed)
      createTables();
      // Run schema migrations
      runMigrations(isLegacyDb);
    } else {
      sqlite3_exec(db, "PRAGMA foreign_keys = ON;", nullptr, nullptr, nullptr);

      // Migrations require write access, so a read-only open leaves an older
      // project on its on-disk schema. Readers degrade gracefully (newer
      // columns are treated as absent), but tell the user how to upgrade.
      const int onDisk = getCurrentSchemaVersion();
      if (onDisk >= 0 && onDisk < LATEST_SCHEMA_VERSION &&
          firstStaleNotice(onDisk)) {
        reusex::warn("Project schema is v{} but this build expects v{}. "
                     "Opened read-only, so no migration was applied and "
                     "newer fields are unavailable. Open the project "
                     "read-write once to migrate: {}",
                     onDisk, LATEST_SCHEMA_VERSION, dbPath);
      }
    }
    hasPassportColumn = columnExists("survey_parts", "passport_guid");
    hasDismissedTable = tableExists("survey_dismissed_instances");
    hasViewColumns = columnExists("survey_parts", "view_h");
    hasPoseGraphV28 = columnExists("pose_graph_edges", "status");
    hasInitialTransform = columnExists("sensor_frames", "initial_transform");
    hasSettingsTable = tableExists("project_settings");
    hasStoreysTable = tableExists("storeys");
    hasPlaneClassesTable = tableExists("plane_classes");
    hasSamplePartLinks = tableExists("sample_part_links");
    hasWallsTable = tableExists("walls") && tableExists("wall_joins");
    hasOpeningsTable = tableExists("openings");
    hasSpacesTable = tableExists("spaces") && tableExists("space_boundaries");
    hasPartDismissedColumn = columnExists("survey_parts", "dismissed");
    hasSurveyMass = surveyMassSchemaPresent();
    hasReportAssetsTable = tableExists("report_assets");
  }

  /// True the first time this process opens dbPath read-only at schema
  /// @p onDisk. A server reads a stale case's card, render and summary over
  /// separate read-only connections; the advice to migrate is worth saying
  /// once, not on every one of them (final review #9). Later opens log it
  /// at debug.
  bool firstStaleNotice(int onDisk) const {
    static std::mutex mutex;
    static std::set<std::pair<std::string, int>> noticed;
    std::error_code ec;
    auto key = std::filesystem::weakly_canonical(dbPath, ec).string();
    if (ec)
      key = dbPath.string();
    std::lock_guard<std::mutex> lock(mutex);
    if (noticed.emplace(std::move(key), onDisk).second)
      return true;
    reusex::debug("Project schema is v{} (read-only, not migrated): {}", onDisk,
                  dbPath);
    return false;
  }

  /// `save_instances` (every `rux create instances` re-run) deletes the
  /// cloud's instances rows, and instance_materials cascades with them. The
  /// stage re-links carried-over instances itself, but a link lost any other
  /// way would leave the pipeline (Viewport, create materials, MaterialEPAS
  /// export) blind to a part's passport. Put a missing link back; an existing
  /// one — even to another passport — is left alone (spec §4.1).
  void restoreInstanceLink(std::string_view code, const std::string &guid,
                           const std::string &instance_guid) {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO instance_materials (cloud_id, instance_id, material_guid) "
        "SELECT cloud_id, instance_id, ?1 FROM instances WHERE guid = ?2 "
        "ON CONFLICT(cloud_id, instance_id) DO NOTHING;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    bind_text(s, 2, instance_guid);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) > 0)
      reusex::info("ensure_resource_passport: restored the instance link of "
                   "part '{}' to its passport '{}' (instance {})",
                   code, guid, instance_guid);
  }

  /// True when an instance_materials row links passport @p guid.
  bool passportLinkedByInstance(std::string_view guid) const {
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT 1 FROM instance_materials WHERE material_guid = ? LIMIT 1;",
        "passport linked by instance");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    return sqlite3_step(s) == SQLITE_ROW;
  }

  /// True when a survey part owns passport @p guid (schema v25).
  bool passportOwnedByPart(std::string_view guid) const {
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT 1 FROM survey_parts WHERE passport_guid = ?;",
        "passport owned by part");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    return sqlite3_step(s) == SQLITE_ROW;
  }

  void checkWritable() const {
    if (readOnly)
      throw std::runtime_error("Cannot write to read-only database: " +
                               dbPath.string());
  }

  // ── Schema versioning ──────────────────────────────────────────────

  /// Throws if sqlite cannot answer — in particular when a lock is held past
  /// the busy timeout. Reporting "no such table" then would be a silent lie
  /// (an old schema, version -1) instead of the "database is locked" the
  /// caller can act on (STANDARDS §5).
  bool tableExists(const char *tableName) const {
    const char *query =
        "SELECT name FROM sqlite_master WHERE type='table' AND name=?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error(std::string("Cannot check for table '") +
                               tableName + "': " + sqlite3_errmsg(db));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, tableName, -1, SQLITE_STATIC);
    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW)
      return true;
    if (rc == SQLITE_DONE)
      return false;
    throw std::runtime_error(std::string("Cannot check for table '") +
                             tableName + "': " + sqlite3_errmsg(db));
  }

  /// True if `tableName` exists AND carries a column named `columnName`.
  /// Used to keep read-only opens (which never migrate) working against
  /// projects still on an older schema.
  bool columnExists(const char *tableName, const char *columnName) const {
    const std::string query =
        std::string("PRAGMA table_info(") + tableName + ");";
    // Throws when sqlite cannot answer, like tableExists.
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error(std::string("Cannot list the columns of '") +
                               tableName + "': " + sqlite3_errmsg(db));
    StmtGuard guard(stmt);
    int rc = SQLITE_ROW;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
      const char *name =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      if (name && std::strcmp(name, columnName) == 0)
        return true;
    }
    if (rc != SQLITE_DONE)
      throw std::runtime_error(std::string("Cannot list the columns of '") +
                               tableName + "': " + sqlite3_errmsg(db));
    return false;
  }

  void createSchemaVersionTable() {
    const char *sql = R"(
      CREATE TABLE IF NOT EXISTS schema_version (
        version     INTEGER NOT NULL,
        applied_at  TEXT NOT NULL DEFAULT (datetime('now')),
        description TEXT
      );
    )";
    char *errMsg = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Failed to create schema_version table: " +
                               error);
    }
  }

  int getCurrentSchemaVersion() const {
    if (!tableExists("schema_version"))
      return -1;

    // -1 means "no version recorded", never "could not read it": a busy or
    // otherwise failing read throws (see tableExists).
    const char *query = "SELECT MAX(version) FROM schema_version;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error(std::string("Cannot read schema version: ") +
                               sqlite3_errmsg(db));
    StmtGuard guard(stmt);

    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
      if (sqlite3_column_type(stmt, 0) != SQLITE_NULL)
        return sqlite3_column_int(stmt, 0);
      return -1;
    }
    if (rc == SQLITE_DONE)
      return -1;
    throw std::runtime_error(std::string("Cannot read schema version: ") +
                             sqlite3_errmsg(db));
  }

  void runMigrations(bool isLegacyDb) {
    createSchemaVersionTable();

    int current = getCurrentSchemaVersion();

    if (current < 0) {
      // Fresh DB or legacy DB — use pre-createTables() state to decide
      if (isLegacyDb) {
        // Legacy DB: mark as version 0, then migrate
        insertSchemaVersion(0, "Legacy passport schema");
        current = 0;
      }
      // else: completely fresh DB, will go straight to v1
    }

    if (current >= 0 && current < LATEST_SCHEMA_VERSION) {
      reusex::warn("Migrating project schema from v{} to v{}: {}", current,
                   LATEST_SCHEMA_VERSION, dbPath);
    }

    if (current < 1) {
      migrateToV1();
    }

    if (current < 2) {
      migrateToV2();
    }

    if (current < 3) {
      migrateToV3();
    }

    if (current < 4) {
      migrateToV4();
    }

    if (current < 5) {
      migrateToV5();
    }

    if (current < 6) {
      migrateToV6();
    }

    if (current < 7) {
      migrateToV7();
    }

    if (current < 8) {
      migrateToV8();
    }

    if (current < 9) {
      migrateToV9();
    }

    if (current < 10) {
      migrateToV10();
    }

    if (current < 11) {
      migrateToV11();
    }

    if (current < 12) {
      migrateToV12();
    }

    if (current < 13) {
      migrateToV13();
    }

    if (current < 14) {
      migrateToV14();
    }

    if (current < 15) {
      migrateToV15();
    }

    if (current < 16) {
      migrateToV16();
    }

    if (current < 17) {
      migrateToV17();
    }

    if (current < 18) {
      migrateToV18();
    }

    if (current < 19) {
      migrateToV19();
    }

    if (current < 20) {
      migrateToV20();
    }

    if (current < 21) {
      migrateToV21();
    }

    if (current < 22) {
      migrateToV22();
    }

    if (current < 23) {
      migrateToV23();
    }

    if (current < 24) {
      migrateToV24();
    }

    if (current < 25) {
      migrateToV25();
    }

    if (current < 26) {
      migrateToV26();
    }

    if (current < 27) {
      migrateToV27();
    }

    if (current < 28) {
      migrateToV28();
    }

    if (current < 29) {
      migrateToV29();
    }

    if (current < 30) {
      migrateToV30();
    }

    if (current < 31) {
      migrateToV31();
    }

    if (current < 32) {
      migrateToV32();
    }

    if (current < 33) {
      migrateToV33();
    }

    if (current < 34) {
      migrateToV34();
    }

    if (current < 35) {
      migrateToV35();
    }

    if (current < 36) {
      migrateToV36();
    }

    if (current < 37) {
      migrateToV37();
    }

    if (current < 38) {
      migrateToV38();
    }

    reusex::trace("Schema version: {}", getCurrentSchemaVersion());
  }

  void insertSchemaVersion(int version, const char *description) {
    const char *sql =
        "INSERT INTO schema_version (version, description) VALUES (?, ?);";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to insert schema version: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, version);
    sqlite3_bind_text(stmt, 2, description, -1, SQLITE_STATIC);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to insert schema version row");
  }

  void migrateToV1() {
    reusex::info("Migrating database to schema version 1");

    const char *v1_schema = R"(
      CREATE TABLE IF NOT EXISTS point_clouds (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        name        TEXT NOT NULL UNIQUE,
        point_type  TEXT NOT NULL,
        point_count INTEGER NOT NULL,
        point_step  INTEGER NOT NULL,
        width       INTEGER NOT NULL,
        height      INTEGER NOT NULL DEFAULT 1,
        created_at  TEXT NOT NULL DEFAULT (datetime('now')),
        stage       TEXT,
        parameters  TEXT
      );

      CREATE TABLE IF NOT EXISTS point_cloud_data (
        cloud_id    INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
        chunk_index INTEGER NOT NULL DEFAULT 0,
        data        BLOB NOT NULL,
        PRIMARY KEY (cloud_id, chunk_index)
      );

      CREATE TABLE IF NOT EXISTS label_definitions (
        cloud_id    INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
        label_id    INTEGER NOT NULL,
        name        TEXT NOT NULL,
        color_rgb   INTEGER,
        PRIMARY KEY (cloud_id, label_id)
      );

      CREATE TABLE IF NOT EXISTS meshes (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        name        TEXT NOT NULL UNIQUE,
        format      TEXT NOT NULL,
        data        BLOB NOT NULL,
        vertex_count INTEGER,
        face_count   INTEGER,
        created_at  TEXT NOT NULL DEFAULT (datetime('now')),
        stage       TEXT,
        parameters  TEXT
      );

      CREATE TABLE IF NOT EXISTS sensor_frames (
        node_id     INTEGER PRIMARY KEY,
        color       BLOB,
        depth       BLOB,
        confidence  BLOB,
        transform   BLOB,
        width       INTEGER,
        height      INTEGER
      );

      CREATE TABLE IF NOT EXISTS pipeline_log (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        stage       TEXT NOT NULL,
        started_at  TEXT NOT NULL DEFAULT (datetime('now')),
        finished_at TEXT,
        parameters  TEXT,
        status      TEXT NOT NULL DEFAULT 'running',
        error_msg   TEXT
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v1_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v1 failed: " + error);
    }

    insertSchemaVersion(1, "Add point clouds, meshes, sensor frames, "
                           "pipeline log");
    reusex::info("Migration to schema version 1 complete");
  }

  void migrateToV2() {
    reusex::info("Migrating database to schema version 2");

    const char *v2_schema = R"(
      CREATE TABLE IF NOT EXISTS segmentation_images (
        node_id     INTEGER PRIMARY KEY REFERENCES sensor_frames(node_id) ON DELETE CASCADE,
        label_image BLOB NOT NULL
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v2_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v2 failed: " + error);
    }

    insertSchemaVersion(2, "Add segmentation_images table");
    reusex::info("Migration to schema version 2 complete");
  }

  void migrateToV3() {
    reusex::info("Migrating database to schema version 3");

    // sensor_frames already has depth, confidence, transform columns from v1.
    // Only need to add camera_model TEXT column.
    const char *alter =
        "ALTER TABLE sensor_frames ADD COLUMN camera_model TEXT;";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, alter, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      // Column may already exist if DB was created fresh (v1 schema doesn't
      // include it, but a manual migration might have added it).  Ignore
      // "duplicate column" errors.
      if (error.find("duplicate column") == std::string::npos) {
        throw std::runtime_error("Migration to v3 failed: " + error);
      }
    }

    insertSchemaVersion(3, "Add camera_model column to sensor_frames");
    reusex::info("Migration to schema version 3 complete");
  }

  void migrateToV4() {
    reusex::info("Migrating database to schema version 4");

    const char *v4_schema = R"(
      CREATE TABLE IF NOT EXISTS building_components (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        name          TEXT NOT NULL UNIQUE,
        type          TEXT NOT NULL,
        vertex_data   BLOB NOT NULL,
        vertex_count  INTEGER NOT NULL,
        plane         BLOB NOT NULL,
        parent_id     INTEGER DEFAULT -1,
        confidence    REAL DEFAULT -1.0,
        metadata      TEXT,
        notes         TEXT,
        created_at    TEXT NOT NULL DEFAULT (datetime('now'))
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v4_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v4 failed: " + error);
    }

    insertSchemaVersion(4, "Add building_components table");
    reusex::info("Migration to schema version 4 complete");
  }

  void migrateToV5() {
    reusex::info("Migrating database to schema version 5");

    // Add timestamp column to sensor_frames
    {
      const char *alter =
          "ALTER TABLE sensor_frames ADD COLUMN timestamp REAL;";
      char *errMsg = nullptr;
      if (sqlite3_exec(db, alter, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg;
        sqlite3_free(errMsg);
        if (error.find("duplicate column") == std::string::npos) {
          throw std::runtime_error("Migration to v5 failed: " + error);
        }
      }
    }

    // Create panoramic_images table
    {
      const char *v5_schema = R"(
        CREATE TABLE IF NOT EXISTS panoramic_images (
          id         INTEGER PRIMARY KEY AUTOINCREMENT,
          filename   TEXT NOT NULL,
          image      BLOB NOT NULL,
          timestamp  REAL,
          node_id    INTEGER REFERENCES sensor_frames(node_id) ON DELETE SET NULL,
          created_at TEXT DEFAULT (datetime('now'))
        );
      )";

      char *errMsg = nullptr;
      if (sqlite3_exec(db, v5_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg;
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v5 failed: " + error);
      }
    }

    insertSchemaVersion(5, "Add timestamp to sensor_frames, "
                           "add panoramic_images table");
    reusex::info("Migration to schema version 5 complete");
  }

  void migrateToV6() {
    reusex::info("Migrating database to schema version 6");

    const char *v6_schema = R"(
      CREATE TABLE IF NOT EXISTS mesh_texture_data (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        mesh_id     INTEGER NOT NULL REFERENCES meshes(id) ON DELETE CASCADE,
        tex_name    TEXT NOT NULL,
        image_data  BLOB NOT NULL,
        width       INTEGER NOT NULL,
        height      INTEGER NOT NULL,
        format      TEXT DEFAULT 'jpg',
        UNIQUE(mesh_id, tex_name)
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v6_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v6 failed: " + error);
    }

    insertSchemaVersion(6, "Add mesh_texture_data table");
    reusex::info("Migration to schema version 6 complete");
  }

  void migrateToV7() {
    reusex::info("Migrating database to schema version 7");

    // Rewrite point_cloud_data with a composite primary key
    // (cloud_id, chunk_index) so a single cloud can be split across multiple
    // rows. This lifts the per-row SQLite blob size limit (max 2 GB) for
    // very large point clouds.
    const char *v7_schema = R"(
      CREATE TABLE point_cloud_data_new (
        cloud_id    INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
        chunk_index INTEGER NOT NULL DEFAULT 0,
        data        BLOB NOT NULL,
        PRIMARY KEY (cloud_id, chunk_index)
      );

      INSERT INTO point_cloud_data_new (cloud_id, chunk_index, data)
        SELECT cloud_id, 0, data FROM point_cloud_data;

      DROP TABLE point_cloud_data;
      ALTER TABLE point_cloud_data_new RENAME TO point_cloud_data;
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v7_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v7 failed: " + error);
    }

    insertSchemaVersion(7, "Chunked point_cloud_data for large clouds");
    reusex::info("Migration to schema version 7 complete");
  }

  void migrateToV8() {
    reusex::info("Migrating database to schema version 8");

    // Add a stable, immutable guid to building components so CSV rows can be
    // edited (including renames) and reimported by matching on the guid.
    // SQLite can't add a UNIQUE column via ALTER, so we add the column,
    // backfill unique guids per row, then create a UNIQUE index.
    if (tableExists("building_components")) {
      char *errMsg = nullptr;
      if (sqlite3_exec(db,
                       "ALTER TABLE building_components ADD COLUMN guid TEXT;",
                       nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v8 failed (add column): " +
                                 error);
      }

      // Backfill existing rows with generated guids.
      std::vector<int> ids;
      {
        const char *sel = "SELECT id FROM building_components "
                          "WHERE guid IS NULL OR guid = '';";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("Migration to v8 failed (select ids): " +
                                   std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        while (sqlite3_step(stmt) == SQLITE_ROW)
          ids.push_back(sqlite3_column_int(stmt, 0));
      }
      for (int id : ids) {
        std::string guid = generate_component_guid();
        const char *upd =
            "UPDATE building_components SET guid = ? WHERE id = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, upd, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("Migration to v8 failed (backfill): " +
                                   std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        sqlite3_bind_text(stmt, 1, guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt, 2, id);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("Migration to v8 failed (backfill step): " +
                                   std::string(sqlite3_errmsg(db)));
      }

      if (sqlite3_exec(db,
                       "CREATE UNIQUE INDEX IF NOT EXISTS "
                       "idx_building_components_guid "
                       "ON building_components(guid);",
                       nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v8 failed (index): " + error);
      }
    }

    insertSchemaVersion(8, "Add guid to building_components");
    reusex::info("Migration to schema version 8 complete");
  }

  void migrateToV9() {
    reusex::info("Migrating database to schema version 9");

    // Link table: one material passport guid per instance (a label value in an
    // instance-label cloud). Stores only a reference — points/labels stay in
    // the cloud, material data stays in the passport tables.
    const char *v9_schema = R"(
      CREATE TABLE IF NOT EXISTS instance_materials (
        cloud_id      INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
        instance_id   INTEGER NOT NULL,
        material_guid TEXT NOT NULL,
        PRIMARY KEY (cloud_id, instance_id)
      );
      CREATE INDEX IF NOT EXISTS idx_instance_materials_guid
        ON instance_materials(material_guid);
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v9_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v9 failed: " + error);
    }

    insertSchemaVersion(9, "Add instance_materials link table");
    reusex::info("Migration to schema version 9 complete");
  }

  void migrateToV10() {
    reusex::info("Migrating database to schema version 10");

    // Stable per-instance identity: the new `instances` table gives every
    // spatially-distinct object a GUID that survives regeneration. The
    // `instance_materials` link table is rebuilt with foreign keys onto both
    // `instances` and `material_passports` so dangling links become
    // impossible. SQLite cannot add foreign keys via ALTER, so the link table
    // is rebuilt from scratch; orphaned rows are dropped with a warning.
    execOrThrow("BEGIN TRANSACTION;");
    try {
      // 1. Create the instances table.
      execOrThrow(R"(
        CREATE TABLE IF NOT EXISTS instances (
          cloud_id       INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
          instance_id    INTEGER NOT NULL,
          guid           TEXT NOT NULL UNIQUE,
          semantic_class INTEGER NOT NULL DEFAULT -1,
          point_count    INTEGER NOT NULL DEFAULT 0,
          PRIMARY KEY (cloud_id, instance_id)
        );
      )");

      // 2. Backfill `instances` from existing instance-label clouds so that
      //    v9 instance_materials links can be preserved. We derive rows from
      //    label_definitions of any cloud that currently has material links,
      //    parsing the semantic class from the "SM{class}-{id} (...)" naming
      //    convention (falling back to -1). Each backfilled row gets a fresh
      //    GUID.
      if (tableExists("instance_materials") &&
          tableExists("label_definitions")) {
        // Distinct cloud_ids that have links.
        std::vector<int> linkedClouds;
        {
          const char *sel = "SELECT DISTINCT cloud_id FROM instance_materials;";
          sqlite3_stmt *stmt;
          if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
            throw std::runtime_error(
                "Migration to v10 failed (select linked clouds): " +
                std::string(sqlite3_errmsg(db)));
          StmtGuard guard(stmt);
          while (sqlite3_step(stmt) == SQLITE_ROW)
            linkedClouds.push_back(sqlite3_column_int(stmt, 0));
        }

        for (int cloudId : linkedClouds) {
          // Read label definitions for this cloud: label_id -> name.
          std::vector<std::pair<int, std::string>> defs;
          {
            const char *sel = "SELECT label_id, name FROM label_definitions "
                              "WHERE cloud_id = ?;";
            sqlite3_stmt *stmt;
            if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
              throw std::runtime_error(
                  "Migration to v10 failed (select label defs): " +
                  std::string(sqlite3_errmsg(db)));
            StmtGuard guard(stmt);
            sqlite3_bind_int(stmt, 1, cloudId);
            while (sqlite3_step(stmt) == SQLITE_ROW) {
              int lid = sqlite3_column_int(stmt, 0);
              const char *nm =
                  reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
              defs.emplace_back(lid, nm ? nm : "");
            }
          }

          for (const auto &[labelId, name] : defs) {
            int semClass = parseSemanticClassFromDef(name);
            std::string guid = generate_component_guid();
            const char *ins = R"(
              INSERT OR IGNORE INTO instances
                (cloud_id, instance_id, guid, semantic_class, point_count)
              VALUES (?, ?, ?, ?, 0);
            )";
            sqlite3_stmt *stmt;
            if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK)
              throw std::runtime_error(
                  "Migration to v10 failed (backfill instance): " +
                  std::string(sqlite3_errmsg(db)));
            StmtGuard guard(stmt);
            sqlite3_bind_int(stmt, 1, cloudId);
            sqlite3_bind_int(stmt, 2, labelId);
            sqlite3_bind_text(stmt, 3, guid.c_str(), -1, SQLITE_TRANSIENT);
            sqlite3_bind_int(stmt, 4, semClass);
            if (sqlite3_step(stmt) != SQLITE_DONE)
              throw std::runtime_error(
                  "Migration to v10 failed (backfill step): " +
                  std::string(sqlite3_errmsg(db)));
          }
        }
      }

      // 3. Identify orphaned instance_materials rows (link whose instance was
      //    not backfilled, or whose passport does not exist) and log them
      //    before dropping. Then rebuild the table with FKs and copy only the
      //    valid rows.
      if (tableExists("instance_materials")) {
        std::vector<std::string> orphans;
        {
          const char *sel = R"(
            SELECT im.cloud_id, im.instance_id, im.material_guid
            FROM instance_materials im
            WHERE NOT EXISTS (
              SELECT 1 FROM instances i
              WHERE i.cloud_id = im.cloud_id
                AND i.instance_id = im.instance_id)
               OR NOT EXISTS (
              SELECT 1 FROM material_passports mp
              WHERE mp.document_guid = im.material_guid);
          )";
          sqlite3_stmt *stmt;
          if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
            throw std::runtime_error(
                "Migration to v10 failed (select orphans): " +
                std::string(sqlite3_errmsg(db)));
          StmtGuard guard(stmt);
          while (sqlite3_step(stmt) == SQLITE_ROW) {
            int cid = sqlite3_column_int(stmt, 0);
            int iid = sqlite3_column_int(stmt, 1);
            const char *g =
                reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
            orphans.push_back(fmt::format("cloud_id={} instance_id={} guid={}",
                                          cid, iid, g ? g : "(null)"));
          }
        }
        if (!orphans.empty()) {
          reusex::warn("Migration to v10: dropping {} orphaned "
                       "instance_materials link(s) with no matching instance "
                       "or passport:",
                       orphans.size());
          for (const auto &o : orphans)
            reusex::warn("  - {}", o);
        }
      }

      // Rebuild instance_materials with foreign keys onto instances and
      // material_passports.
      execOrThrow(R"(
        CREATE TABLE instance_materials_new (
          cloud_id      INTEGER NOT NULL REFERENCES point_clouds(id) ON DELETE CASCADE,
          instance_id   INTEGER NOT NULL,
          material_guid TEXT NOT NULL REFERENCES material_passports(document_guid) ON DELETE CASCADE,
          PRIMARY KEY (cloud_id, instance_id),
          FOREIGN KEY (cloud_id, instance_id)
            REFERENCES instances(cloud_id, instance_id) ON DELETE CASCADE
        );
      )");

      if (tableExists("instance_materials")) {
        // Copy only valid rows (instance + passport both exist).
        execOrThrow(R"(
          INSERT INTO instance_materials_new (cloud_id, instance_id, material_guid)
          SELECT im.cloud_id, im.instance_id, im.material_guid
          FROM instance_materials im
          WHERE EXISTS (
            SELECT 1 FROM instances i
            WHERE i.cloud_id = im.cloud_id
              AND i.instance_id = im.instance_id)
            AND EXISTS (
            SELECT 1 FROM material_passports mp
            WHERE mp.document_guid = im.material_guid);
        )");
        execOrThrow("DROP TABLE instance_materials;");
      }
      execOrThrow(
          "ALTER TABLE instance_materials_new RENAME TO instance_materials;");
      execOrThrow("CREATE INDEX IF NOT EXISTS idx_instance_materials_guid "
                  "ON instance_materials(material_guid);");

      insertSchemaVersion(10, "Add instances table; FK-enforced "
                              "instance_materials");
      execOrThrow("COMMIT;");
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
    reusex::info("Migration to schema version 10 complete");
  }

  void migrateToV11() {
    reusex::info("Migrating database to schema version 11");

    // Content-based 360 alignment: a panorama can now carry its OWN refined
    // 6-DoF pose (from PanoramaAlignment / `rux align 360`) instead of only
    // inheriting the timestamp-matched sensor frame's pose. Columns are added
    // to panoramic_images; each is nullable so timestamp-only panoramas stay
    // valid. panorama_segmentation stores an equirect SAM3 label image per
    // panorama (same CV_16U +1-offset PNG convention as segmentation_images).
    if (!tableExists("panoramic_images")) {
      // Nothing to ALTER — create the table already in its v11 shape so the
      // schema stays consistent (a project that never imported panoramas can
      // legitimately be missing it).
      const char *create = R"(
        CREATE TABLE IF NOT EXISTS panoramic_images (
          id            INTEGER PRIMARY KEY AUTOINCREMENT,
          filename      TEXT NOT NULL,
          image         BLOB NOT NULL,
          timestamp     REAL,
          node_id       INTEGER REFERENCES sensor_frames(node_id) ON DELETE SET NULL,
          created_at    TEXT DEFAULT (datetime('now')),
          pose          BLOB,
          pose_source   TEXT,
          align_inliers INTEGER,
          align_rms     REAL
        );
      )";
      char *errMsg = nullptr;
      if (sqlite3_exec(db, create, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v11 failed: " + error);
      }
    } else {
      const char *alters[] = {
          "ALTER TABLE panoramic_images ADD COLUMN pose BLOB;",
          "ALTER TABLE panoramic_images ADD COLUMN pose_source TEXT;",
          "ALTER TABLE panoramic_images ADD COLUMN align_inliers INTEGER;",
          "ALTER TABLE panoramic_images ADD COLUMN align_rms REAL;",
      };
      for (const char *alter : alters) {
        char *errMsg = nullptr;
        if (sqlite3_exec(db, alter, nullptr, nullptr, &errMsg) != SQLITE_OK) {
          std::string error = errMsg ? errMsg : "unknown error";
          sqlite3_free(errMsg);
          if (error.find("duplicate column") == std::string::npos)
            throw std::runtime_error("Migration to v11 failed: " + error);
        }
      }
    }

    const char *v11_schema = R"(
      CREATE TABLE IF NOT EXISTS panorama_segmentation (
        pano_id     INTEGER PRIMARY KEY REFERENCES panoramic_images(id) ON DELETE CASCADE,
        label_image BLOB NOT NULL
      );
    )";
    char *errMsg = nullptr;
    if (sqlite3_exec(db, v11_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v11 failed: " + error);
    }

    insertSchemaVersion(11,
                        "Add panorama pose columns + panorama_segmentation");
    reusex::info("Migration to schema version 11 complete");
  }

  void migrateToV12() {
    reusex::info("Migrating database to schema version 12");

    // Gaussian splat storage (#322). `rux create gsplat` used to write its
    // .ply to a path of the user's choosing, which made the trained splat the
    // one pipeline artefact that did not live in the project — nothing could
    // list it, `rux info` could not see it, and the GUI had to be pointed at
    // a file. The payload is chunked for the same reason point clouds are:
    // sqlite materializes a whole blob column on read, and a splat is
    // routinely hundreds of MB.
    //
    // gaussian_count / sh_degree / byte_size are denormalized from the PLY
    // header at save time so a listing costs no blob reads at all.
    const char *v12_schema = R"(
      CREATE TABLE IF NOT EXISTS gaussian_splats (
        id             INTEGER PRIMARY KEY AUTOINCREMENT,
        name           TEXT NOT NULL UNIQUE,
        format         TEXT NOT NULL,
        gaussian_count INTEGER NOT NULL,
        sh_degree      INTEGER NOT NULL,
        byte_size      INTEGER NOT NULL,
        created_at     TEXT NOT NULL DEFAULT (datetime('now')),
        stage          TEXT,
        parameters     TEXT
      );

      CREATE TABLE IF NOT EXISTS gaussian_splat_data (
        splat_id    INTEGER NOT NULL REFERENCES gaussian_splats(id) ON DELETE CASCADE,
        chunk_index INTEGER NOT NULL DEFAULT 0,
        data        BLOB NOT NULL,
        PRIMARY KEY (splat_id, chunk_index)
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v12_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v12 failed: " + error);
    }

    insertSchemaVersion(12, "Add gaussian_splats + gaussian_splat_data");
    reusex::info("Migration to schema version 12 complete");
  }

  void migrateToV13() {
    reusex::info("Migrating database to schema version 13");

    // Material-keyed VLM-derived annotations (#373). `rux create attributes`
    // iterates the project's material passports; for each material it resolves
    // the instance(s) it is linked to (via `instance_materials`), crops the
    // best sensor view of those points, asks an OpenAI-compatible
    // vision-language endpoint to describe the material, and records the
    // free-text answer + an arbitrary key/value set here so a MaterialPassport
    // can be seeded from it.
    //
    // Keyed on `material_guid` — the same identity that `material_passports`
    // exposes as `document_guid` and that `instance_materials.material_guid`
    // links to. Referential integrity onto the OWNING material is load-bearing:
    // deleting a passport (`delete_material_passport`, which the CASCADE from
    // `material_passports` reaches through `instance_materials`) or otherwise
    // removing the material must not leave a dangling annotation keyed to a
    // GUID nothing owns any more (STANDARDS §3.2). Because `material_passports`
    // has no compound PK we can hang a two-column FK from, we use a plain FK
    // from `material_annotations.material_guid` onto
    // `material_passports(document_guid)` ON DELETE CASCADE (document_guid is
    // UNIQUE, which SQLite accepts as an FK parent). `save_material_annotation`
    // additionally enforces the GUID exists BEFORE inserting, so a nonexistent
    // material is rejected loudly rather than silently written.
    //
    // The arbitrary attributes are stored as queryable rows in a child table
    // `material_annotation_kv`, keyed (material_guid, key), FK-cascading off
    // `material_annotations` so re-describing or deleting a material clears its
    // key/value rows too. `raw_json` preserves the model's full response for
    // provenance; `provider_model` records which endpoint+model produced it.
    const char *v13_schema = R"(
      CREATE TABLE IF NOT EXISTS material_annotations (
        material_guid  TEXT PRIMARY KEY
          REFERENCES material_passports(document_guid) ON DELETE CASCADE,
        description    TEXT,
        provider_model TEXT,
        raw_json       TEXT,
        created_at     TEXT NOT NULL DEFAULT (datetime('now'))
      );

      CREATE TABLE IF NOT EXISTS material_annotation_kv (
        material_guid  TEXT NOT NULL,
        key            TEXT NOT NULL,
        value          TEXT,
        PRIMARY KEY (material_guid, key),
        FOREIGN KEY (material_guid)
          REFERENCES material_annotations(material_guid) ON DELETE CASCADE
      );

      CREATE INDEX IF NOT EXISTS idx_material_annotation_kv_guid
        ON material_annotation_kv(material_guid);
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v13_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v13 failed: " + error);
    }

    insertSchemaVersion(
        13, "Add material_annotations + material_annotation_kv tables");
    reusex::info("Migration to schema version 13 complete");
  }

  void migrateToV14() {
    reusex::info("Migrating database to schema version 14");

    // Per-frame glass/mirror depth-suppress maps (#372). `rux create annotate
    // --glass-filter` runs glass/mirror/transparent prompts through SAM3 and
    // stores a CV_8U binary mask here (255 = trust depth, 0 = suppress).
    // `rux create clouds --glass-filter` reads these to zero glass pixels
    // before back-projection, preventing reflective/transparent artefacts in
    // the fused cloud. Cascade-deletes with the parent sensor frame so removing
    // a frame also removes its glass mask.
    const char *v14_schema = R"(
      CREATE TABLE IF NOT EXISTS glass_confidence_images (
        node_id          INTEGER PRIMARY KEY
          REFERENCES sensor_frames(node_id) ON DELETE CASCADE,
        confidence_image BLOB NOT NULL
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v14_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v14 failed: " + error);
    }

    insertSchemaVersion(14, "Add glass_confidence_images table");
    reusex::info("Migration to schema version 14 complete");
  }

  void migrateToV15() {
    reusex::info("Migrating database to schema version 15");

    // Pose-graph edges written by `rux optimize` after convergence (#265).
    // The table stores the factor-graph topology + per-edge post-solve
    // residuals so the GUI can visualise which constraints are strained.
    // Deliberately no CASCADE to sensor_frames: if a frame is removed the edge
    // is stale, but retaining it until the next `rux optimize` is less
    // surprising than silently emptying the table on a frame delete.
    const char *v15_schema = R"(
      CREATE TABLE IF NOT EXISTS pose_graph_edges (
        id           INTEGER PRIMARY KEY,
        from_node_id INTEGER NOT NULL,
        to_node_id   INTEGER NOT NULL,
        edge_type    TEXT    NOT NULL,
        residual     REAL    NOT NULL,
        weight       REAL
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v15_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v15 failed: " + error);
    }

    insertSchemaVersion(15, "Add pose_graph_edges table");
    reusex::info("Migration to schema version 15 complete");
  }

  // ── Pose-graph helpers ───────────────────────────────────────────────────

  static std::optional<double> optionalDouble(sqlite3_stmt *s, int col) {
    if (sqlite3_column_type(s, col) == SQLITE_NULL)
      return std::nullopt;
    return sqlite3_column_double(s, col);
  }

  template <size_t N>
  static std::optional<std::array<double, N>> optionalBlob(sqlite3_stmt *s,
                                                           int col) {
    const void *blob = sqlite3_column_blob(s, col);
    if (!blob ||
        sqlite3_column_bytes(s, col) != static_cast<int>(N * sizeof(double)))
      return std::nullopt;
    std::array<double, N> out{};
    std::memcpy(out.data(), blob, N * sizeof(double));
    return out;
  }

  static std::string textOr(sqlite3_stmt *s, int col, const char *fallback) {
    const auto *t = reinterpret_cast<const char *>(sqlite3_column_text(s, col));
    return t ? std::string(t) : std::string(fallback);
  }

  /// Column list matching readPoseGraphEdge(); the legacy list serves a
  /// pre-v28 project opened read-only.
  const char *poseGraphColumns() const {
    return hasPoseGraphV28
               ? "id, from_node_id, to_node_id, edge_type, residual, weight, "
                 "transform, information, source, status, created_at, "
                 "seed_disagreement_m, seed_disagreement_deg, deviation_m, "
                 "deviation_deg"
               : "id, from_node_id, to_node_id, edge_type, residual, weight";
  }

  ProjectDB::PoseGraphEdge readPoseGraphEdge(sqlite3_stmt *s) const {
    ProjectDB::PoseGraphEdge e;
    e.id = sqlite3_column_int64(s, 0);
    e.from_node_id = sqlite3_column_int(s, 1);
    e.to_node_id = sqlite3_column_int(s, 2);
    e.edge_type = textOr(s, 3, "");
    e.residual =
        optionalDouble(s, 4).value_or(std::numeric_limits<double>::quiet_NaN());
    e.weight =
        optionalDouble(s, 5).value_or(std::numeric_limits<double>::quiet_NaN());
    if (!hasPoseGraphV28) {
      e.source = std::string(core::pose_graph::kSourceLegacy);
      e.status = std::string(core::pose_graph::kStatusAuto);
      return e;
    }
    e.transform = optionalBlob<16>(s, 6);
    e.information = optionalBlob<36>(s, 7);
    e.source = textOr(s, 8, "legacy");
    e.status = textOr(s, 9, "auto");
    e.created_at = textOr(s, 10, "");
    e.seed_disagreement_m = optionalDouble(s, 11);
    e.seed_disagreement_deg = optionalDouble(s, 12);
    e.deviation_m = optionalDouble(s, 13);
    e.deviation_deg = optionalDouble(s, 14);
    return e;
  }

  std::vector<ProjectDB::PoseGraphEdge> listPoseGraphEdges() const {
    if (!tableExists("pose_graph_edges"))
      return {};
    const std::string sel = std::string("SELECT ") + poseGraphColumns() +
                            " FROM pose_graph_edges ORDER BY id;";
    sqlite3_stmt *stmt =
        prepare_or_throw(db, sel.c_str(), "list_pose_graph_edges");
    StmtGuard guard(stmt);
    std::vector<ProjectDB::PoseGraphEdge> out;
    int rc = SQLITE_ROW;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW)
      out.push_back(readPoseGraphEdge(stmt));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("list_pose_graph_edges: " +
                               std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<ProjectDB::PoseGraphEdge>
  poseGraphEdge(int from, int to, std::string_view type) const {
    if (!tableExists("pose_graph_edges"))
      return std::nullopt;
    // Pre-v28 rows are not canonical: match both orders there.
    const std::string sel =
        std::string("SELECT ") + poseGraphColumns() +
        " FROM pose_graph_edges WHERE edge_type = ?3 AND "
        "((from_node_id = ?1 AND to_node_id = ?2) OR "
        "(from_node_id = ?2 AND to_node_id = ?1)) ORDER BY id LIMIT 1;";
    sqlite3_stmt *stmt = prepare_or_throw(db, sel.c_str(), "pose_graph_edge");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, from);
    sqlite3_bind_int(stmt, 2, to);
    const std::string t(type);
    sqlite3_bind_text(stmt, 3, t.c_str(), -1, SQLITE_TRANSIENT);
    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW)
      return readPoseGraphEdge(stmt);
    if (rc != SQLITE_DONE)
      throw std::runtime_error("pose_graph_edge: " +
                               std::string(sqlite3_errmsg(db)));
    return std::nullopt;
  }

  bool hasPoseGraph() const {
    if (!tableExists("pose_graph_edges"))
      return false;
    const char *sel = "SELECT COUNT(*) FROM pose_graph_edges;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  void requireV28PoseGraph(const char *who) const {
    if (!hasPoseGraphV28)
      throw std::runtime_error(std::string(who) +
                               ": project schema predates v28; open it "
                               "read-write once to migrate");
  }

  static void bindOptionalDouble(sqlite3_stmt *s, int idx,
                                 const std::optional<double> &v) {
    if (v && std::isfinite(*v))
      sqlite3_bind_double(s, idx, *v);
    else
      sqlite3_bind_null(s, idx);
  }

  static void bindNanAsNull(sqlite3_stmt *s, int idx, double v) {
    if (std::isfinite(v))
      sqlite3_bind_double(s, idx, v);
    else
      sqlite3_bind_null(s, idx);
  }

  bool upsertPoseGraphEdge(ProjectDB::PoseGraphEdge e, bool overwriteRejected) {
    namespace pg = core::pose_graph;
    checkWritable();
    requireV28PoseGraph("upsert_pose_graph_edge");
    if (e.from_node_id == e.to_node_id)
      throw std::invalid_argument("upsert_pose_graph_edge: self-loop on node " +
                                  std::to_string(e.from_node_id));
    if (e.edge_type.empty())
      throw std::invalid_argument("upsert_pose_graph_edge: empty edge_type");
    if (e.source.empty())
      throw std::invalid_argument("upsert_pose_graph_edge: empty source");
    if (!pg::is_valid_status(e.status))
      throw std::invalid_argument("upsert_pose_graph_edge: invalid status '" +
                                  e.status + "'");
    if (e.transform && !pg::is_rigid_transform(*e.transform))
      throw std::invalid_argument(
          "upsert_pose_graph_edge: transform is not a rigid 4x4 pose");
    if (e.information && !pg::is_valid_information(*e.information))
      throw std::invalid_argument(
          "upsert_pose_graph_edge: information is not a finite symmetric 6x6");

    if (e.from_node_id > e.to_node_id) {
      std::swap(e.from_node_id, e.to_node_id);
      if (e.transform) {
        const Eigen::Matrix4d T = pg::matrix_from_row_major(*e.transform);
        if (e.information)
          e.information =
              pg::row_major_from_information(pg::reverse_edge_information(
                  pg::information_from_row_major(*e.information), T));
        e.transform = pg::row_major_from_matrix(T.inverse());
      }
    }
    if (e.information) {
      const auto &a = *e.information;
      e.weight = (a[21] + a[28] + a[35]) / 3.0; // mean translational precision
    }

    const char *sql = R"(
      INSERT INTO pose_graph_edges
        (from_node_id, to_node_id, edge_type, residual, weight, transform,
         information, source, status, created_at, seed_disagreement_m,
         seed_disagreement_deg, deviation_m, deviation_deg)
      VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9,
              COALESCE(NULLIF(?10, ''),
                       strftime('%Y-%m-%dT%H:%M:%SZ', 'now')),
              ?11, ?12, ?13, ?14)
      ON CONFLICT (from_node_id, to_node_id, edge_type) DO UPDATE SET
        residual = IIF(?16, excluded.residual, pose_graph_edges.residual),
        weight = IIF(?16, excluded.weight, pose_graph_edges.weight),
        transform = IIF(?16, excluded.transform, pose_graph_edges.transform),
        information =
          IIF(?16, excluded.information, pose_graph_edges.information),
        source = IIF(?16, excluded.source, pose_graph_edges.source),
        status = IIF(NOT ?16 AND excluded.status = 'auto'
                       AND pose_graph_edges.status = 'confirmed',
                     'confirmed', excluded.status),
        created_at = COALESCE(NULLIF(?10, ''), pose_graph_edges.created_at),
        seed_disagreement_m = IIF(?16, excluded.seed_disagreement_m,
                                  pose_graph_edges.seed_disagreement_m),
        seed_disagreement_deg = IIF(?16, excluded.seed_disagreement_deg,
                                    pose_graph_edges.seed_disagreement_deg),
        deviation_m = IIF(?16, excluded.deviation_m,
                          pose_graph_edges.deviation_m),
        deviation_deg = IIF(?16, excluded.deviation_deg,
                            pose_graph_edges.deviation_deg)
      WHERE pose_graph_edges.status <> 'rejected' OR ?15 = 1;
    )";
    sqlite3_stmt *s = prepare_or_throw(db, sql, "upsert_pose_graph_edge");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, e.from_node_id);
    sqlite3_bind_int(s, 2, e.to_node_id);
    sqlite3_bind_text(s, 3, e.edge_type.c_str(), -1, SQLITE_TRANSIENT);
    bindNanAsNull(s, 4, e.residual);
    bindNanAsNull(s, 5, e.weight);
    if (e.transform)
      sqlite3_bind_blob(s, 6, e.transform->data(), 16 * sizeof(double),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(s, 6);
    if (e.information)
      sqlite3_bind_blob(s, 7, e.information->data(), 36 * sizeof(double),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(s, 7);
    sqlite3_bind_text(s, 8, e.source.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 9, e.status.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 10, e.created_at.c_str(), -1, SQLITE_TRANSIENT);
    bindOptionalDouble(s, 11, e.seed_disagreement_m);
    bindOptionalDouble(s, 12, e.seed_disagreement_deg);
    bindOptionalDouble(s, 13, e.deviation_m);
    bindOptionalDouble(s, 14, e.deviation_deg);
    sqlite3_bind_int(s, 15, overwriteRejected ? 1 : 0);
    // ?16: does this upsert carry a measurement? Without one, an existing
    // row is only re-statused (final review I1).
    sqlite3_bind_int(s, 16, e.transform ? 1 : 0);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("upsert_pose_graph_edge: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  bool setPoseGraphEdgeStatus(int from, int to, std::string_view type,
                              std::string_view status) {
    checkWritable();
    requireV28PoseGraph("set_pose_graph_edge_status");
    if (!core::pose_graph::is_valid_status(status))
      throw std::invalid_argument(
          "set_pose_graph_edge_status: invalid status '" + std::string(status) +
          "'");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "UPDATE pose_graph_edges SET status = ?4 WHERE from_node_id = "
        "MIN(?1, ?2) AND to_node_id = MAX(?1, ?2) AND edge_type = ?3;",
        "set_pose_graph_edge_status");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, from);
    sqlite3_bind_int(s, 2, to);
    const std::string t(type), st(status);
    sqlite3_bind_text(s, 3, t.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 4, st.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_pose_graph_edge_status: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  int updatePoseGraphEdgeMetrics(
      std::span<const ProjectDB::PoseGraphEdgeMetrics> metrics) {
    checkWritable();
    requireV28PoseGraph("update_pose_graph_edge_metrics");
    Savepoint sp(db, "pose_graph_metrics");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "UPDATE pose_graph_edges SET residual = ?4, deviation_m = ?5, "
        "deviation_deg = ?6, seed_disagreement_m = ?7, "
        "seed_disagreement_deg = ?8 WHERE from_node_id = MIN(?1, ?2) AND "
        "to_node_id = MAX(?1, ?2) AND edge_type = ?3;",
        "update_pose_graph_edge_metrics");
    StmtGuard guard(s);
    int updated = 0;
    for (const auto &m : metrics) {
      sqlite3_reset(s);
      sqlite3_clear_bindings(s);
      sqlite3_bind_int(s, 1, m.from_node_id);
      sqlite3_bind_int(s, 2, m.to_node_id);
      sqlite3_bind_text(s, 3, m.edge_type.c_str(), -1, SQLITE_TRANSIENT);
      bindOptionalDouble(s, 4, m.residual);
      bindOptionalDouble(s, 5, m.deviation_m);
      bindOptionalDouble(s, 6, m.deviation_deg);
      bindOptionalDouble(s, 7, m.seed_disagreement_m);
      bindOptionalDouble(s, 8, m.seed_disagreement_deg);
      if (sqlite3_step(s) != SQLITE_DONE)
        throw std::runtime_error("update_pose_graph_edge_metrics: " +
                                 std::string(sqlite3_errmsg(db)));
      updated += sqlite3_changes(db);
    }
    sp.release();
    return updated;
  }

  int deletePoseGraphEdgesFromSource(std::string_view source) {
    checkWritable();
    requireV28PoseGraph("delete_pose_graph_edges_from_source");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "DELETE FROM pose_graph_edges WHERE source = ?1 AND status = "
        "'auto';",
        "delete_pose_graph_edges_from_source");
    StmtGuard guard(s);
    const std::string src(source);
    sqlite3_bind_text(s, 1, src.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("delete_pose_graph_edges_from_source: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db);
  }

  void migrateToV16() {
    reusex::info("Migrating database to schema version 16");
    // Spatial tile index for morton_10bit_bitrev clouds (#395).
    // Computed at create clouds time; NULL for older clouds.
    // Guard: the table may not exist (panorama-only DBs) or the column may
    // already exist (downgrade-then-remigrate path).
    if (tableExists("point_clouds") &&
        !columnExists("point_clouds", "tile_index")) {
      const char *sql = "ALTER TABLE point_clouds ADD COLUMN tile_index BLOB;";
      char *errMsg = nullptr;
      if (sqlite3_exec(db, sql, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v16 failed: " + error);
      }
    }
    insertSchemaVersion(16, "Add tile_index column to point_clouds");
    reusex::info("Migration to schema version 16 complete");
  }

  void migrateToV17() {
    reusex::info("Migrating database to schema version 17");

    // Multi-session import support (#129). Each `rux import` records a `scans`
    // row; sensor frames carry a nullable `scan_id` back-reference. Both
    // changes are additive so existing frames stay valid.
    //
    // Guards: the table / column may already exist on a
    // downgrade-then-remigrate path, so use CREATE TABLE IF NOT EXISTS and
    // check columnExists() before the ALTER.
    const char *scans_schema = R"(
      CREATE TABLE IF NOT EXISTS scans (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        source_path TEXT NOT NULL,
        imported_at TEXT NOT NULL DEFAULT (datetime('now')),
        id_offset   INTEGER NOT NULL DEFAULT 0,
        provenance  TEXT
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, scans_schema, nullptr, nullptr, &errMsg) !=
        SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v17 failed: " + error);
    }

    if (tableExists("sensor_frames") &&
        !columnExists("sensor_frames", "scan_id")) {
      const char *alter =
          "ALTER TABLE sensor_frames ADD COLUMN scan_id INTEGER "
          "REFERENCES scans(id);";
      if (sqlite3_exec(db, alter, nullptr, nullptr, &errMsg) != SQLITE_OK) {
        std::string error = errMsg ? errMsg : "unknown error";
        sqlite3_free(errMsg);
        throw std::runtime_error("Migration to v17 failed: " + error);
      }
    }

    // If pre-existing frames are present, back-fill a single "legacy" scan
    // record (id=1) and point every unassigned frame at it.
    if (tableExists("sensor_frames")) {
      bool hasFrames = false;
      {
        const char *cnt = "SELECT COUNT(*) FROM sensor_frames;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, cnt, -1, &stmt, nullptr) == SQLITE_OK) {
          StmtGuard guard(stmt);
          if (sqlite3_step(stmt) == SQLITE_ROW)
            hasFrames = sqlite3_column_int(stmt, 0) > 0;
        }
      }

      if (hasFrames) {
        const char *legacy =
            "INSERT OR IGNORE INTO scans (id, source_path, id_offset, "
            "provenance) VALUES (1, '', 0, '{\"legacy\":true}');";
        if (sqlite3_exec(db, legacy, nullptr, nullptr, &errMsg) != SQLITE_OK) {
          std::string error = errMsg ? errMsg : "unknown error";
          sqlite3_free(errMsg);
          throw std::runtime_error("Migration to v17 failed: " + error);
        }

        const char *backfill =
            "UPDATE sensor_frames SET scan_id = 1 WHERE scan_id IS NULL;";
        if (sqlite3_exec(db, backfill, nullptr, nullptr, &errMsg) !=
            SQLITE_OK) {
          std::string error = errMsg ? errMsg : "unknown error";
          sqlite3_free(errMsg);
          throw std::runtime_error("Migration to v17 failed: " + error);
        }
      }
    }

    insertSchemaVersion(17, "Add scans table and sensor_frames.scan_id (#129)");
    reusex::info("Migration to schema version 17 complete");
  }

  void migrateToV18() {
    reusex::info("Migrating database to schema version 18");

    // Notion-like material editor (#413): user-defined column definitions and
    // per-material thumbnail blobs. Both additive; CREATE TABLE IF NOT EXISTS
    // keeps a downgrade-then-remigrate path safe.
    const char *v18_schema = R"(
      CREATE TABLE IF NOT EXISTS material_property_definitions (
        id          TEXT PRIMARY KEY,
        name        TEXT NOT NULL,
        type        TEXT NOT NULL DEFAULT 'text',
        options     TEXT,
        sort_order  INTEGER NOT NULL DEFAULT 0,
        created_at  TEXT NOT NULL DEFAULT (datetime('now'))
      );

      CREATE TABLE IF NOT EXISTS material_thumbnails (
        material_guid TEXT PRIMARY KEY
          REFERENCES material_passports(document_guid) ON DELETE CASCADE,
        blob          BLOB NOT NULL,
        mime_type     TEXT NOT NULL DEFAULT 'image/jpeg',
        created_at    TEXT NOT NULL DEFAULT (datetime('now'))
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v18_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v18 failed: " + error);
    }

    insertSchemaVersion(
        18, "Add material_property_definitions and material_thumbnails (#413)");
    reusex::info("Migration to schema version 18 complete");
  }

  void migrateToV19() {
    reusex::info("Migrating database to schema version 19");

    // Notion-like material editor: persist per-column display width so the
    // GUI can remember user-resized columns.
    const char *v19_schema = "ALTER TABLE material_property_definitions "
                             "ADD COLUMN width INTEGER NOT NULL DEFAULT 200;";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v19_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      // A fresh DB gets material_property_definitions from createTables() with
      // `width` already present, and then this migration runs over the top of
      // it — so a "duplicate column" here is expected and harmless, exactly as
      // in migrateToV3/V5/V8. Without this guard every fresh ProjectDB open
      // throws.
      if (error.find("duplicate column") == std::string::npos) {
        throw std::runtime_error("Migration to v19 failed: " + error);
      }
    }

    insertSchemaVersion(19, "Add width to material_property_definitions");
    reusex::info("Migration to schema version 19 complete");
  }

  void migrateToV20() {
    reusex::info("Migrating database to schema version 20");

    // Server-side PDF report versioned storage (#456): one row per generated
    // Ressourcekortlægning PDF; the blob is stored inline (passports are
    // typically a few hundred KB). Blobs are fetched one at a time so there is
    // no per-query heap spike from materialising multiple PDFs.
    const char *v20_schema = R"(
      CREATE TABLE IF NOT EXISTS report_pdfs (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        label      TEXT    NOT NULL DEFAULT '',
        created_at TEXT    NOT NULL DEFAULT (datetime('now')),
        pdf_blob   BLOB    NOT NULL
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v20_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v20 failed: " + error);
    }

    insertSchemaVersion(
        20, "Add report_pdfs table for versioned PDF storage (#456)");
    reusex::info("Migration to schema version 20 complete");
  }

  void migrateToV21() {
    reusex::info("Migrating database to schema version 21");

    // Named export templates (#459): stores user-defined column selections and
    // format options for CSV export. Config is stored as JSON so new options
    // (additional filters, sort order, etc.) can be added without schema bumps.
    const char *v21_schema = R"(
      CREATE TABLE IF NOT EXISTS export_templates (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        name       TEXT    NOT NULL,
        config     TEXT    NOT NULL DEFAULT '{}',
        created_at TEXT    NOT NULL DEFAULT (datetime('now')),
        updated_at TEXT    NOT NULL DEFAULT (datetime('now'))
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v21_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v21 failed: " + error);
    }

    insertSchemaVersion(
        21, "Add export_templates table for named CSV export configs (#459)");
    reusex::info("Migration to schema version 21 complete");
  }

  void migrateToV22() {
    reusex::info("Migrating database to schema version 22");

    // Ressourcekortlægning (docs/design/gui-kortlaegning-redesign.md): survey
    // types group instance-parts; samples gate approval. Enum columns are
    // CHECK-constrained to the wire strings in core/survey.hpp.
    const char *v22_schema = R"(
      CREATE TABLE IF NOT EXISTS survey_types (
        id             INTEGER PRIMARY KEY AUTOINCREMENT,
        name           TEXT    NOT NULL,
        eak_code       TEXT    NOT NULL DEFAULT '',
        bim7aa_code    TEXT    NOT NULL DEFAULT '',
        unit           TEXT    NOT NULL DEFAULT 'stk',
        treatment      TEXT    NOT NULL DEFAULT 'genanvendelse'
          CHECK (treatment IN ('bevaring','genbrug','genanvendelse','nyttiggoerelse','bortskaffelse')),
        review_status  TEXT    NOT NULL DEFAULT 'queue'
          CHECK (review_status IN ('queue','approved','rejected')),
        confidence     REAL,
        mass_t         REAL,
        note           TEXT    NOT NULL DEFAULT '',
        starred        INTEGER NOT NULL DEFAULT 0,
        semantic_class INTEGER NOT NULL DEFAULT -1,
        created_at     TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        updated_at     TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS survey_parts (
        code          TEXT    PRIMARY KEY,
        type_id       INTEGER NOT NULL REFERENCES survey_types(id) ON DELETE CASCADE,
        instance_guid TEXT UNIQUE,
        room_id       INTEGER,
        room_name     TEXT    NOT NULL DEFAULT '',
        quantity      REAL    NOT NULL DEFAULT 1,
        starred       INTEGER NOT NULL DEFAULT 0,
        note          TEXT    NOT NULL DEFAULT ''
      );
      CREATE INDEX IF NOT EXISTS idx_survey_parts_type ON survey_parts(type_id);
      CREATE TABLE IF NOT EXISTS samples (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        code       TEXT    NOT NULL UNIQUE,
        title      TEXT    NOT NULL,
        what       TEXT    NOT NULL DEFAULT '',
        stage      TEXT    NOT NULL DEFAULT 'planlagt'
          CHECK (stage IN ('planlagt','udtaget','sendt','svar')),
        result     TEXT    NOT NULL DEFAULT ''
          CHECK (result IN ('','ren','forurenet')),
        created_at TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        updated_at TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS sample_links (
        sample_id INTEGER NOT NULL REFERENCES samples(id) ON DELETE CASCADE,
        type_id   INTEGER NOT NULL REFERENCES survey_types(id) ON DELETE CASCADE,
        PRIMARY KEY (sample_id, type_id)
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v22_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v22 failed: " + error);
    }

    insertSchemaVersion(22,
                        "Add survey types/parts and samples for Kortlægning");
    reusex::info("Migration to schema version 22 complete");
  }

  void migrateToV23() {
    reusex::info("Migrating database to schema version 23");

    // GUI Phase 5 (Rapport): how many survey types still blocked the report
    // when a PDF version was generated, so the GUI can tell a complete
    // version from a draft. NULL for versions generated before v23.
    if (!columnExists("report_pdfs", "blocking_types"))
      execOrThrow("ALTER TABLE report_pdfs ADD COLUMN blocking_types INTEGER;");

    insertSchemaVersion(
        23, "Record blocking survey types per report PDF version (Phase 5)");
    reusex::info("Migration to schema version 23 complete");
  }

  void migrateToV24() {
    reusex::info("Migrating database to schema version 24");

    // GUI Phase 6 (On-site): the bygningsdel a sample was taken at. NULL for
    // a sample registered without one (Miljø & prøver, or before v24). Plain
    // TEXT, not a foreign key: see SampleRecord::part_code.
    // Deliberate exception to STANDARDS §3.2: when a type's parts are
    // cascade-deleted, part_code dangles silently (see the Phase 6 spec).
    if (!columnExists("samples", "part_code"))
      execOrThrow("ALTER TABLE samples ADD COLUMN part_code TEXT;");

    insertSchemaVersion(
        24, "Record the survey part a sample was taken at (Phase 6)");
    reusex::info("Migration to schema version 24 complete");
  }

  std::vector<ProjectDB::PropertyDefinition> listPropertyDefinitions() const {
    const char *query = "SELECT id, name, type, options, sort_order, width "
                        "FROM material_property_definitions "
                        "ORDER BY sort_order, created_at;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error(
          "Failed to prepare list property definitions query: " +
          std::string(sqlite3_errmsg(db)));
    }
    StmtGuard guard(stmt);

    std::vector<ProjectDB::PropertyDefinition> defs;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::PropertyDefinition def;
      const char *id =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      const char *name =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      const char *type =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
      const char *options =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));

      def.id = id ? id : "";
      def.name = name ? name : "";
      def.type = type ? type : "text";
      def.sort_order = sqlite3_column_int(stmt, 4);
      def.width = sqlite3_column_int(stmt, 5);

      if (options && *options) {
        try {
          auto parsed = nlohmann::json::parse(options);
          if (parsed.is_array()) {
            for (const auto &opt : parsed) {
              if (opt.is_string())
                def.options.push_back(opt.get<std::string>());
            }
          }
        } catch (const nlohmann::json::exception &) {
          // Tolerate a malformed options blob rather than failing the whole
          // list.
          reusex::warn("Ignoring malformed options for property definition {}",
                       def.id);
        }
      }
      defs.push_back(std::move(def));
    }
    return defs;
  }

  int64_t queryInt(const char *sql) const {
    sqlite3_stmt *s = prepare_or_throw(db, sql, "queryInt");
    StmtGuard guard(s);
    if (sqlite3_step(s) != SQLITE_ROW)
      throw std::runtime_error(std::string("query failed: ") + sql + ": " +
                               sqlite3_errmsg(db));
    return sqlite3_column_int64(s, 0);
  }

  /// Insert one templates row; unset timestamps default to now.
  /// @throws core::NameConflictError on a taken name.
  int64_t insertTemplate(const std::string &name, const std::string &members,
                         const std::string &csv,
                         const std::optional<std::string> &seed,
                         const std::optional<std::string> &created_at = {},
                         const std::optional<std::string> &updated_at = {}) {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO templates (name, members, csv, seed, created_at, "
        "updated_at) VALUES (?,?,?,?,"
        "COALESCE(?5, strftime('%Y-%m-%dT%H:%M:%SZ','now')),"
        "COALESCE(?6, strftime('%Y-%m-%dT%H:%M:%SZ','now'))) RETURNING id;",
        "insert template");
    StmtGuard guard(s);
    auto bind_opt = [&](int i, const std::optional<std::string> &v) {
      if (v)
        bind_text(s, i, *v);
      else
        sqlite3_bind_null(s, i);
    };
    bind_text(s, 1, name);
    bind_text(s, 2, members);
    bind_text(s, 3, csv);
    bind_opt(4, seed);
    bind_opt(5, created_at);
    bind_opt(6, updated_at);
    if (sqlite3_step(s) != SQLITE_ROW) {
      if (sqlite3_extended_errcode(db) == SQLITE_CONSTRAINT_UNIQUE)
        throw core::NameConflictError("a template named '" + name +
                                      "' already exists");
      throw std::runtime_error("insert template '" + name +
                               "': " + sqlite3_errmsg(db));
    }
    return sqlite3_column_int64(s, 0);
  }

  std::vector<std::string> templateNames() const {
    sqlite3_stmt *s =
        prepare_or_throw(db, "SELECT name FROM templates;", "template names");
    StmtGuard guard(s);
    std::vector<std::string> out;
    while (sqlite3_step(s) == SQLITE_ROW)
      out.push_back(column_text(s, 0));
    return out;
  }

  std::size_t seedTemplates() {
    std::size_t n = 0;
    for (const auto &seed : core::seed_templates()) {
      insertTemplate(seed.name, core::members_json(seed.members).dump(), "{}",
                     seed.tag);
      ++n;
    }
    return n;
  }

  struct MovedTemplates {
    std::size_t moved = 0;
    std::vector<std::string> unmatched; // "<template>: <column>"
  };
  /// Spec §5.4: every export_templates row becomes a templates row; its
  /// columns become key members, the rest of its config the CSV options.
  MovedTemplates moveExportTemplates() {
    MovedTemplates out;
    if (!tableExists("export_templates"))
      return out;
    const auto catalogue = core::key_catalogue(listPropertyDefinitions());
    auto taken = templateNames();
    struct Row {
      std::string name, config;
      std::optional<std::string> created_at, updated_at;
    };
    std::vector<Row> rows;
    {
      // v21 stamped datetime('now') ("YYYY-MM-DD HH:MM:SS", UTC); carry it
      // over in the v25 ISO form. Unparseable stamps read NULL -> now.
      sqlite3_stmt *s = prepare_or_throw(
          db,
          "SELECT name, config, strftime('%Y-%m-%dT%H:%M:%SZ', created_at), "
          "strftime('%Y-%m-%dT%H:%M:%SZ', updated_at) FROM export_templates "
          "ORDER BY id;",
          "migrateToV25");
      StmtGuard guard(s);
      auto opt_text = [&](int i) -> std::optional<std::string> {
        if (sqlite3_column_type(s, i) == SQLITE_NULL)
          return std::nullopt;
        return column_text(s, i);
      };
      while (sqlite3_step(s) == SQLITE_ROW)
        rows.push_back(
            {column_text(s, 0), column_text(s, 1), opt_text(2), opt_text(3)});
    }
    for (const auto &[name, config, created_at, updated_at] : rows) {
      auto cfg = nlohmann::json::parse(config, nullptr, false);
      if (cfg.is_discarded() || !cfg.is_object()) {
        reusex::warn("v25: export template '{}' had unreadable config; moved "
                     "with no columns",
                     name);
        cfg = nlohmann::json::object();
      }
      std::vector<core::TemplateMember> members;
      if (cfg.contains("columns") && cfg["columns"].is_array())
        members = column_members(cfg["columns"], catalogue,
                                 "v25: export template '" + name + "'");
      for (const auto &m : members)
        if (m.ref.rfind(core::kLegacyKeyPrefix, 0) == 0)
          out.unmatched.push_back(name + ": " +
                                  m.ref.substr(core::kLegacyKeyPrefix.size()));
      cfg.erase("columns");
      const bool clash =
          std::find(taken.begin(), taken.end(), name) != taken.end();
      const auto final_name =
          clash ? core::unique_name(name, "eksport", taken) : name;
      insertTemplate(final_name, core::members_json(members).dump(), cfg.dump(),
                     std::nullopt, created_at, updated_at);
      taken.push_back(final_name);
      ++out.moved;
    }
    execOrThrow("DROP TABLE export_templates;");
    if (!out.unmatched.empty()) {
      std::string list;
      for (const auto &u : out.unmatched)
        list += (list.empty() ? "" : "; ") + u;
      reusex::warn("v25: {} export-template column(s) match no resource key "
                   "and are kept as missing members: {}",
                   out.unmatched.size(), list);
    }
    return out;
  }

  /// A copy of passport @p src under a fresh guid: row, property values and
  /// thumbnail (not the log). Spec §4.2 step 2.
  std::string copyPassport(const std::string &src) {
    const std::string dst = core::generate_guid();
    auto run = [&](const char *sql) {
      sqlite3_stmt *s = prepare_or_throw(db, sql, "migrateToV25");
      StmtGuard guard(s);
      bind_text(s, 1, dst);
      bind_text(s, 2, src);
      if (sqlite3_step(s) != SQLITE_DONE)
        throw std::runtime_error("Migration to v25 failed copying passport " +
                                 src + ": " + sqlite3_errmsg(db));
    };
    run("INSERT INTO material_passports (id, project_id, document_guid, "
        "created_at, revised_at, version_number, version_date) "
        "SELECT ?1, project_id, ?1, created_at, revised_at, version_number, "
        "version_date FROM material_passports WHERE document_guid = ?2;");
    run("INSERT INTO passport_property_values (id, passport_id, property_id, "
        "leksikon_guid, sort_order, value) "
        "SELECT ?1 || '_' || v.leksikon_guid, ?1, v.property_id, "
        "v.leksikon_guid, v.sort_order, v.value "
        "FROM passport_property_values v "
        "JOIN material_passports mp ON mp.id = v.passport_id "
        "WHERE mp.document_guid = ?2;");
    run("INSERT INTO material_thumbnails (material_guid, blob, mime_type) "
        "SELECT ?1, blob, mime_type FROM material_thumbnails "
        "WHERE material_guid = ?2;");
    // The VLM description (v13) hangs off document_guid too.
    if (tableExists("material_annotations")) {
      run("INSERT INTO material_annotations (material_guid, description, "
          "provider_model, raw_json, created_at) SELECT ?1, description, "
          "provider_model, raw_json, created_at FROM material_annotations "
          "WHERE material_guid = ?2;");
      run("INSERT INTO material_annotation_kv (material_guid, key, value) "
          "SELECT ?1, key, value FROM material_annotation_kv "
          "WHERE material_guid = ?2;");
    }
    return dst;
  }

  struct LinkCounts {
    std::size_t linked = 0, split = 0, unlinked = 0;
  };
  /// Spec §4.2 steps 2–3: link each instance-backed part to its instance's
  /// passport; when several parts share one, the first by code keeps it and
  /// every other gets a copy (its instance link repointed). Passports no
  /// part references are left alone.
  LinkCounts linkResourcePassports() {
    LinkCounts c;
    // Column checks, not table checks: hand-built legacy fixtures carry an
    // `instances` table that predates instance_id/guid.
    if (columnExists("instances", "guid") &&
        columnExists("instances", "instance_id") &&
        tableExists("instance_materials")) {
      struct Row {
        std::string code;
        int64_t cloud_id = 0, instance_id = 0;
        std::string guid;
      };
      std::vector<Row> rows;
      {
        // Only links to a passport that exists: the new column is a foreign
        // key, and a dangling link has nothing to own.
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT p.code, i.cloud_id, i.instance_id, im.material_guid "
            "FROM survey_parts p "
            "JOIN instances i ON i.guid = p.instance_guid "
            "JOIN instance_materials im ON im.cloud_id = i.cloud_id AND "
            "im.instance_id = i.instance_id "
            "JOIN material_passports mp ON mp.document_guid = "
            "im.material_guid "
            "WHERE p.passport_guid IS NULL "
            "ORDER BY im.material_guid, p.code;",
            "migrateToV25");
        StmtGuard guard(s);
        while (sqlite3_step(s) == SQLITE_ROW)
          rows.push_back({column_text(s, 0), sqlite3_column_int64(s, 1),
                          sqlite3_column_int64(s, 2), column_text(s, 3)});
      }
      std::set<std::string> claimed;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT passport_guid FROM survey_parts WHERE passport_guid IS "
            "NOT NULL;",
            "migrateToV25");
        StmtGuard guard(s);
        while (sqlite3_step(s) == SQLITE_ROW)
          claimed.insert(column_text(s, 0));
      }
      for (const auto &r : rows) {
        std::string guid = r.guid;
        if (claimed.count(guid)) {
          guid = copyPassport(r.guid);
          sqlite3_stmt *s = prepare_or_throw(
              db,
              "UPDATE instance_materials SET material_guid = ? WHERE "
              "cloud_id = ? AND instance_id = ?;",
              "migrateToV25");
          StmtGuard guard(s);
          bind_text(s, 1, guid);
          sqlite3_bind_int64(s, 2, r.cloud_id);
          sqlite3_bind_int64(s, 3, r.instance_id);
          if (sqlite3_step(s) != SQLITE_DONE)
            throw std::runtime_error("Migration to v25 failed repointing " +
                                     r.code + ": " + sqlite3_errmsg(db));
          ++c.split;
        }
        sqlite3_stmt *s = prepare_or_throw(
            db, "UPDATE survey_parts SET passport_guid = ? WHERE code = ?;",
            "migrateToV25");
        StmtGuard guard(s);
        bind_text(s, 1, guid);
        bind_text(s, 2, r.code);
        if (sqlite3_step(s) != SQLITE_DONE)
          throw std::runtime_error("Migration to v25 failed linking " + r.code +
                                   ": " + sqlite3_errmsg(db));
        claimed.insert(guid);
        ++c.linked;
      }
      if (c.split > 0)
        reusex::warn("v25: {} survey part(s) shared a passport with another "
                     "part and now each have their own copy (values, "
                     "thumbnail and VLM annotation copied, log not copied)",
                     c.split);
    }
    c.unlinked = static_cast<std::size_t>(
        queryInt("SELECT COUNT(*) FROM material_passports WHERE document_guid "
                 "NOT IN (SELECT passport_guid FROM survey_parts WHERE "
                 "passport_guid IS NOT NULL);"));
    return c;
  }

  void migrateToV25() {
    reusex::info("Migrating database to schema version 25");
    // Resources and templates (resources/templates design doc §4.2, removed
    // 2026-10-14, see git history): one transaction, so
    // a v24 project upgrades whole or stays v24. Every step is guarded so a
    // test that rolls the version back and reopens re-runs it safely.
    execOrThrow("BEGIN TRANSACTION;");
    try {
      // §4.2 step 1. ADD COLUMN cannot carry UNIQUE; the index does (NULLs
      // stay distinct, so parts without a passport are fine).
      if (!columnExists("survey_parts", "passport_guid"))
        execOrThrow("ALTER TABLE survey_parts ADD COLUMN passport_guid TEXT "
                    "REFERENCES material_passports(document_guid) "
                    "ON DELETE SET NULL;");
      execOrThrow("CREATE UNIQUE INDEX IF NOT EXISTS idx_survey_parts_passport "
                  "ON survey_parts(passport_guid);");
      const auto links = linkResourcePassports(); // steps 2–3
      // Step 4: On-site is gone. Plain TEXT column: no rebuild needed.
      if (columnExists("samples", "part_code"))
        execOrThrow("ALTER TABLE samples DROP COLUMN part_code;");
      execOrThrow(kTemplatesSchema);
      // Seeds first: the table is new, so this is "if templates is empty",
      // and moved export templates can then be suffixed against the seeds.
      std::size_t seeded = 0;
      if (queryInt("SELECT COUNT(*) FROM templates;") == 0)
        seeded = seedTemplates();
      const auto moved = moveExportTemplates();
      insertSchemaVersion(25, "Resources and templates: passport per part, "
                              "templates table, no On-site part_code");
      execOrThrow("COMMIT;");
      reusex::info("Migration to schema version 25 complete: passports "
                   "linked {}, split {}, left unlinked {}; {} seed "
                   "template(s); {} export template(s) moved, {} column(s) "
                   "unmatched",
                   links.linked, links.split, links.unlinked, seeded,
                   moved.moved, moved.unmatched.size());
    } catch (...) {
      if (sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr) != SQLITE_OK)
        reusex::warn("migrateToV25: ROLLBACK failed: {}", sqlite3_errmsg(db));
      throw;
    }
  }

  void migrateToV26() {
    reusex::info("Migrating database to schema version 26");
    // Reject != delete (Kortlægning fixes spec A3): deleting an
    // instance-backed survey part records its instance guid here so
    // sync_survey does not re-create the part. Plain TEXT, not a foreign key:
    // the tombstone must outlive a `rux create instances` re-run that drops
    // and re-inserts the instances rows. Keeping the rows also makes a later
    // "restore deleted scan resources" possible.
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS survey_dismissed_instances (
        instance_guid TEXT PRIMARY KEY,
        dismissed_at  TEXT NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
    )");
    insertSchemaVersion(26, "Survey tombstones: deleted scan-backed parts "
                            "stay deleted (survey_dismissed_instances)");
    reusex::info("Migration to schema version 26 complete");
  }

  void migrateToV27() {
    reusex::info("Migrating database to schema version 27");
    // Resource view record (resource-view-record design doc §2, removed
    // 2026-10-14, see git history): the chosen image of a
    // survey part. ADD COLUMN cannot add a table constraint, so the
    // cross-column invariant is a column CHECK on view_h, the last column
    // added, when every column it names exists. Each step is guarded so a
    // test that rolls the version back re-runs it safely. Existing rows stay
    // NULL, which the CHECK accepts. 1e-9 = core::kViewRegionEpsilon.
    static constexpr const char *kViewH =
        "REAL CHECK ((view_kind IS NULL) = (view_id IS NULL) "
        "AND (view_kind IS NOT NULL OR view_x IS NULL) "
        "AND ((view_x IS NULL AND view_y IS NULL AND view_w IS NULL "
        "AND view_h IS NULL) OR (view_x IS NOT NULL AND view_y IS NOT NULL "
        "AND view_w IS NOT NULL AND view_h IS NOT NULL AND view_w > 0 "
        "AND view_h > 0 AND view_x >= 0 AND view_y >= 0 "
        "AND view_w <= 1.000000001 AND view_y + view_h <= 1.000000001)))";
    const std::pair<const char *, const char *> columns[] = {
        {"view_kind", "TEXT CHECK (view_kind IN ('frame','panorama'))"},
        {"view_id", "INTEGER"},
        {"view_x", "REAL"},
        {"view_y", "REAL"},
        {"view_w", "REAL"},
        {"view_h", kViewH}};
    execOrThrow("BEGIN TRANSACTION;");
    try {
      for (const auto &[name, decl] : columns)
        if (!columnExists("survey_parts", name))
          execOrThrow((std::string("ALTER TABLE survey_parts ADD COLUMN ") +
                       name + " " + decl + ";")
                          .c_str());
      insertSchemaVersion(27, "Resource view record: survey_parts.view_* "
                              "(sys:view, the chosen image of a resource)");
      execOrThrow("COMMIT;");
    } catch (...) {
      if (sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr) != SQLITE_OK)
        reusex::warn("migrateToV27: ROLLBACK failed: {}", sqlite3_errmsg(db));
      throw;
    }
    reusex::info("Migration to schema version 27 complete");
  }

  void migrateToV28() {
    reusex::info("Migrating database to schema version 28");
    // Pose-graph editor (spec §3): the stored graph becomes the source of
    // truth for `rux optimize`. pose_graph_edges is REBUILT rather than
    // ALTERed: v15 declared `residual REAL NOT NULL` (an unsolved edge has no
    // residual) and SQLite cannot add the UNIQUE / CHECK constraints the
    // tombstones match on. Rows are canonicalised to from < to, self-loops
    // dropped, and duplicates collapsed onto the lowest id (SQLite's
    // single-MIN() aggregate returns the bare columns of that row).
    Savepoint sp(db, "migrate_v28");
    if (tableExists("pose_graph_edges") &&
        !columnExists("pose_graph_edges", "status")) {
      execOrThrow(R"(
        CREATE TABLE pose_graph_edges_v28 (
          id                    INTEGER PRIMARY KEY,
          from_node_id          INTEGER NOT NULL,
          to_node_id            INTEGER NOT NULL,
          edge_type             TEXT    NOT NULL,
          residual              REAL,
          weight                REAL,
          transform             BLOB,
          information           BLOB,
          source                TEXT    NOT NULL DEFAULT 'legacy',
          status                TEXT    NOT NULL DEFAULT 'auto'
            CHECK (status IN ('auto', 'confirmed', 'rejected')),
          created_at            TEXT,
          seed_disagreement_m   REAL,
          seed_disagreement_deg REAL,
          deviation_m           REAL,
          deviation_deg         REAL,
          CHECK (from_node_id < to_node_id),
          UNIQUE (from_node_id, to_node_id, edge_type)
        );
        INSERT INTO pose_graph_edges_v28
          (id, from_node_id, to_node_id, edge_type, residual, weight,
           source, status)
        SELECT MIN(id), MIN(from_node_id, to_node_id),
               MAX(from_node_id, to_node_id), edge_type, residual, weight,
               'legacy', 'auto'
        FROM pose_graph_edges
        WHERE from_node_id <> to_node_id
        GROUP BY MIN(from_node_id, to_node_id), MAX(from_node_id, to_node_id),
                 edge_type;
        DROP TABLE pose_graph_edges;
        ALTER TABLE pose_graph_edges_v28 RENAME TO pose_graph_edges;
      )");
    }
    if (tableExists("sensor_frames") &&
        !columnExists("sensor_frames", "initial_transform"))
      execOrThrow("ALTER TABLE sensor_frames ADD COLUMN initial_transform "
                  "BLOB;");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS project_settings (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
    )");
    insertSchemaVersion(28, "Stored pose graph: edge measurements, "
                            "information, provenance, status and metrics; "
                            "sensor_frames.initial_transform; "
                            "project_settings");
    sp.release();
    reusex::info("Migration to schema version 28 complete");
  }

  /// What shiftStoredLabelsV29() did to one cloud.
  struct LabelShiftV29 {
    std::size_t points = 0;
    std::size_t unlabeled = 0;
    std::size_t shifted = 0;
    std::size_t too_large = 0; ///< >= 0x7FFFFFFF: left alone (see below)
  };

  /// Shift every label 1..0x7FFFFFFE in a stored Label cloud's chunks by +1,
  /// in place. The chunks are concatenated first so a record straddling a
  /// chunk boundary is shifted correctly, then written back with each chunk's
  /// original length.
  LabelShiftV29 shiftStoredLabelsV29(int cloudId) {
    struct Chunk {
      sqlite3_int64 rowid;
      size_t size;
    };
    std::vector<Chunk> chunks;
    std::vector<uint8_t> bytes;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db,
          "SELECT rowid, data FROM point_cloud_data WHERE cloud_id = ? "
          "ORDER BY chunk_index;",
          "migrateToV29");
      StmtGuard g(s);
      sqlite3_bind_int(s, 1, cloudId);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
        const auto *src =
            static_cast<const uint8_t *>(sqlite3_column_blob(s, 1));
        const auto n = static_cast<size_t>(sqlite3_column_bytes(s, 1));
        chunks.push_back({sqlite3_column_int64(s, 0), n});
        if (n > 0)
          bytes.insert(bytes.end(), src, src + n);
      }
      if (rc != SQLITE_DONE)
        throw std::runtime_error("migrateToV29: failed to read label chunks: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    if (bytes.size() % LABEL_STEP != 0)
      throw std::runtime_error(
          "migrateToV29: label cloud " + std::to_string(cloudId) + " holds " +
          std::to_string(bytes.size()) + " bytes, not a multiple of " +
          std::to_string(LABEL_STEP));
    LabelShiftV29 stats;
    for (size_t off = 0; off < bytes.size(); off += LABEL_STEP) {
      uint32_t v = 0;
      std::memcpy(&v, bytes.data() + off, sizeof v);
      ++stats.points;
      // Unlabeled stays unlabeled (API class 0 was already lost into it). A
      // value >= 0x7FFFFFFF is a wrapped negative (out of contract) or would
      // become one by the shift: it is no class id, so it is left alone and
      // reported rather than invented into a class.
      if (v == core::kUnlabeled) {
        ++stats.unlabeled;
        continue;
      }
      if (v >= 0x7FFFFFFFu) {
        ++stats.too_large;
        continue;
      }
      ++v;
      ++stats.shifted;
      std::memcpy(bytes.data() + off, &v, sizeof v);
    }
    sqlite3_stmt *u = prepare_or_throw(
        db, "UPDATE point_cloud_data SET data = ? WHERE rowid = ?;",
        "migrateToV29");
    StmtGuard ug(u);
    size_t off = 0;
    for (const auto &c : chunks) {
      sqlite3_reset(u);
      sqlite3_clear_bindings(u);
      static const uint8_t kEmpty = 0;
      sqlite3_bind_blob(u, 1, c.size ? bytes.data() + off : &kEmpty,
                        static_cast<int>(c.size), SQLITE_STATIC);
      sqlite3_bind_int64(u, 2, c.rowid);
      if (sqlite3_step(u) != SQLITE_DONE)
        throw std::runtime_error("migrateToV29: failed to write label chunk: " +
                                 std::string(sqlite3_errmsg(db)));
      off += c.size;
    }
    return stats;
  }

  /// Run one statement with an int binding; @return sqlite3_changes().
  std::size_t execBoundV29(const char *sql, std::optional<int> bound = {}) {
    sqlite3_stmt *s = prepare_or_throw(db, sql, "migrateToV29");
    StmtGuard g(s);
    if (bound)
      sqlite3_bind_int(s, 1, *bound);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error(std::string("migrateToV29: ") + sql +
                               " failed: " + sqlite3_errmsg(db));
    return static_cast<std::size_t>(sqlite3_changes(db));
  }

  void migrateToV29() {
    reusex::info("Migrating database to schema version 29");
    // Decision D8 (reusex-z1v.2.1): point label = API class + 1. Before v29 a
    // semantic label cloud stored the API class verbatim, so API class 0 (the
    // first prompt class) collided with kUnlabeled.
    //  1. Every stored semantic cloud (core::is_semantic_label_cloud(), not an
    //     instance cloud) without parameters.label_encoding is shifted (+1,
    //     0 stays 0), its label_definitions re-keyed and the key stamped.
    //  2. The semantic class numbers stored elsewhere were always the old
    //     (API-equal) numbering, so they move unconditionally:
    //     instances.semantic_class and survey_types.semantic_class (>= 0),
    //     the "SM<cls>-" definitions of every non-semantic label cloud, and
    //     the names generated from a class number ("Klasse N" survey types,
    //     "class N" passport designations). User-edited names are left.
    // Structural label ids (planes, rooms, instances) are not class numbers
    // and keep their values.
    Savepoint sp(db, "migrate_v29");

    struct Cloud {
      int id;
      std::string name;
      std::string stage;
      std::string params;
    };
    std::vector<Cloud> labelClouds;
    // Guarded: legacy fixtures and partial schemas may lack point_clouds.
    if (tableExists("point_clouds")) {
      sqlite3_stmt *s = prepare_or_throw(
          db,
          "SELECT id, name, COALESCE(stage, ''), COALESCE(parameters, '') "
          "FROM point_clouds WHERE point_type = 'Label' ORDER BY id;",
          "migrateToV29");
      StmtGuard g(s);
      while (sqlite3_step(s) == SQLITE_ROW)
        labelClouds.push_back({sqlite3_column_int(s, 0), column_text(s, 1),
                               column_text(s, 2), column_text(s, 3)});
    }
    std::set<int> instanceClouds;
    if (tableExists("instances")) {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT DISTINCT cloud_id FROM instances;", "migrateToV29");
      StmtGuard g(s);
      while (sqlite3_step(s) == SQLITE_ROW)
        instanceClouds.insert(sqlite3_column_int(s, 0));
    }
    const bool haveDefs = tableExists("label_definitions");

    std::vector<std::string> shifted;
    std::set<int> semanticIds;
    for (const auto &c : labelClouds) {
      if (!core::is_semantic_label_cloud(c.name, c.stage) ||
          instanceClouds.count(c.id) != 0)
        continue;
      semanticIds.insert(c.id);
      if (core::has_api_plus_one_encoding(c.params))
        continue;
      nlohmann::json params = nlohmann::json::object();
      if (!c.params.empty()) {
        bool ok = false;
        try {
          auto j = nlohmann::json::parse(c.params);
          if (j.is_object()) {
            params = std::move(j);
            ok = true;
          }
        } catch (const nlohmann::json::exception &) {
        }
        if (!ok) {
          reusex::warn("migrateToV29: cloud '{}' has unparsable parameters; "
                       "keeping them under 'legacy_parameters'",
                       c.name);
          params["legacy_parameters"] = c.params;
        }
      }

      const auto stats = shiftStoredLabelsV29(c.id);

      // Re-keyed by delete + insert: an in-place `label_id + 1` would trip
      // the (cloud_id, label_id) key row by row.
      struct Def {
        int id;
        std::string name;
        bool has_color;
        sqlite3_int64 color;
      };
      std::vector<Def> defs;
      if (haveDefs) {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT label_id, name, color_rgb FROM label_definitions "
            "WHERE cloud_id = ? AND label_id >= 0 ORDER BY label_id;",
            "migrateToV29");
        StmtGuard g(s);
        sqlite3_bind_int(s, 1, c.id);
        while (sqlite3_step(s) == SQLITE_ROW)
          defs.push_back({sqlite3_column_int(s, 0), column_text(s, 1),
                          sqlite3_column_type(s, 2) != SQLITE_NULL,
                          sqlite3_column_int64(s, 2)});
      }
      std::string classZero;
      if (!defs.empty()) {
        execBoundV29("DELETE FROM label_definitions WHERE cloud_id = ? AND "
                     "label_id >= 0;",
                     c.id);
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "INSERT INTO label_definitions (cloud_id, label_id, name, "
            "color_rgb) VALUES (?, ?, ?, ?);",
            "migrateToV29");
        StmtGuard g(s);
        for (const auto &d : defs) {
          if (d.id == 0)
            classZero = d.name;
          sqlite3_reset(s);
          sqlite3_clear_bindings(s);
          sqlite3_bind_int(s, 1, c.id);
          sqlite3_bind_int(s, 2, d.id + 1);
          bind_text(s, 3, d.name);
          if (d.has_color)
            sqlite3_bind_int64(s, 4, d.color);
          else
            sqlite3_bind_null(s, 4);
          if (sqlite3_step(s) != SQLITE_DONE)
            throw std::runtime_error(
                "migrateToV29: failed to re-key label definitions: " +
                std::string(sqlite3_errmsg(db)));
        }
      }

      params[core::kLabelEncodingKey] = core::kLabelEncodingApiPlusOne;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db, "UPDATE point_clouds SET parameters = ? WHERE id = ?;",
            "migrateToV29");
        StmtGuard g(s);
        const auto dumped = params.dump();
        bind_text(s, 1, dumped);
        sqlite3_bind_int(s, 2, c.id);
        if (sqlite3_step(s) != SQLITE_DONE)
          throw std::runtime_error("migrateToV29: failed to stamp encoding: " +
                                   std::string(sqlite3_errmsg(db)));
      }
      shifted.push_back(c.name);

      // Not recoverable here: before v29 API class 0 was stored as 0.
      if (stats.unlabeled > 0)
        reusex::warn(
            "migrateToV29: cloud '{}': {} of {} points are unlabeled. Points "
            "of API class 0{} were stored as unlabeled before v29 and cannot "
            "be told apart from them; re-run `rux create clouds` or `rux "
            "create project` to recover that class",
            c.name, stats.unlabeled, stats.points,
            classZero.empty() ? std::string()
                              : fmt::format(" ('{}')", classZero));
      if (stats.too_large > 0)
        reusex::warn("migrateToV29: cloud '{}': {} of {} points carry a label "
                     ">= 0x7FFFFFFF (a wrapped negative, out of contract); "
                     "left unshifted",
                     c.name, stats.too_large, stats.points);
      reusex::info("migrateToV29: cloud '{}': shifted {} of {} point labels "
                   "and {} label definition(s)",
                   c.name, stats.shifted, stats.points, defs.size());
    }

    // Names generated from a class number, renamed before the numbers move.
    std::size_t passportNames = 0;
    // Every table/column is guarded: legacy fixtures and partial schemas
    // carry only some of them.
    const bool haveInstanceClass =
        tableExists("instances") && columnExists("instances", "semantic_class");
    if (haveInstanceClass && tableExists("instance_materials") &&
        tableExists("material_passports") &&
        columnExists("passport_property_values", "value") &&
        columnExists("passport_property_values", "leksikon_guid")) {
      // "construction item designation" (core/materialepas_traits.hpp),
      // prefilled "class N" by `rux create materials` when the class had no
      // name.
      std::vector<std::pair<std::string, std::string>> rewrites;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT pv.id, i.semantic_class, CAST(pv.value AS TEXT) "
            "FROM passport_property_values pv "
            "JOIN material_passports mp ON mp.id = pv.passport_id "
            "JOIN instance_materials im ON im.material_guid = "
            "mp.document_guid "
            "JOIN instances i ON i.cloud_id = im.cloud_id AND "
            "i.instance_id = im.instance_id "
            "WHERE pv.leksikon_guid = '2_mYRkA$9EcwXAxqBGoMrM' AND "
            "i.semantic_class >= 0;",
            "migrateToV29");
        StmtGuard g(s);
        while (sqlite3_step(s) == SQLITE_ROW) {
          const int cls = sqlite3_column_int(s, 1);
          if (column_text(s, 2) == "class " + std::to_string(cls))
            rewrites.emplace_back(column_text(s, 0),
                                  "class " + std::to_string(cls + 1));
        }
      }
      sqlite3_stmt *u = prepare_or_throw(
          db, "UPDATE passport_property_values SET value = ? WHERE id = ?;",
          "migrateToV29");
      StmtGuard ug(u);
      for (const auto &[id, value] : rewrites) {
        sqlite3_reset(u);
        sqlite3_bind_blob(u, 1, value.data(), static_cast<int>(value.size()),
                          SQLITE_TRANSIENT);
        bind_text(u, 2, id);
        if (sqlite3_step(u) != SQLITE_DONE)
          throw std::runtime_error(
              "migrateToV29: failed to rename a passport designation: " +
              std::string(sqlite3_errmsg(db)));
        ++passportNames;
      }
    }

    std::size_t instanceRows = 0, surveyTypes = 0, surveyNames = 0;
    if (haveInstanceClass)
      instanceRows = execBoundV29("UPDATE instances SET semantic_class = "
                                  "semantic_class + 1 WHERE semantic_class "
                                  ">= 0;");
    if (columnExists("survey_types", "semantic_class")) {
      // sync_survey's fallback name, "Klasse <semantic_class>".
      surveyNames = execBoundV29(
          "UPDATE survey_types SET name = 'Klasse ' || (semantic_class + 1) "
          "WHERE semantic_class >= 0 AND name = 'Klasse ' || semantic_class;");
      surveyTypes = execBoundV29("UPDATE survey_types SET semantic_class = "
                                 "semantic_class + 1 WHERE semantic_class "
                                 ">= 0;");
    }

    // "SM<cls>-<id> (<n>p)" names the semantic class in an instance cloud's
    // definitions. Rewritten on every non-semantic label cloud whose
    // definitions match, not only those with `instances` rows (pre-v10 or
    // sync_downsample'd instance clouds have none).
    std::size_t instanceDefs = 0;
    if (haveDefs) {
      for (const auto &c : labelClouds) {
        if (semanticIds.count(c.id) != 0)
          continue;
        std::vector<std::pair<int, std::string>> rewrites;
        {
          sqlite3_stmt *s = prepare_or_throw(
              db,
              "SELECT label_id, name FROM label_definitions WHERE cloud_id = ? "
              "AND name LIKE 'SM%';",
              "migrateToV29");
          StmtGuard g(s);
          sqlite3_bind_int(s, 1, c.id);
          while (sqlite3_step(s) == SQLITE_ROW) {
            const std::string name = column_text(s, 1);
            std::size_t i = 2;
            while (i < name.size() && i < 12 && name[i] >= '0' &&
                   name[i] <= '9')
              ++i;
            if (i == 2 || i >= name.size() || name[i] != '-')
              continue;
            const long long cls = std::stoll(name.substr(2, i - 2));
            rewrites.emplace_back(sqlite3_column_int(s, 0),
                                  "SM" + std::to_string(cls + 1) +
                                      name.substr(i));
          }
        }
        sqlite3_stmt *u = prepare_or_throw(
            db,
            "UPDATE label_definitions SET name = ? WHERE cloud_id = ? AND "
            "label_id = ?;",
            "migrateToV29");
        StmtGuard ug(u);
        for (const auto &[lid, name] : rewrites) {
          sqlite3_reset(u);
          bind_text(u, 1, name);
          sqlite3_bind_int(u, 2, c.id);
          sqlite3_bind_int(u, 3, lid);
          if (sqlite3_step(u) != SQLITE_DONE)
            throw std::runtime_error(
                "migrateToV29: failed to rewrite instance definition: " +
                std::string(sqlite3_errmsg(db)));
          ++instanceDefs;
        }
      }
    }

    insertSchemaVersion(29, "Point label = API class + 1: semantic label "
                            "clouds shifted and stamped label_encoding");
    sp.release();
    reusex::info("Migration to schema version 29 complete: re-encoded {} "
                 "semantic cloud(s) [{}] (point label = API class + 1); "
                 "re-based {} instance row(s), {} instance definition(s), {} "
                 "survey type(s) ({} renamed) and {} passport designation(s)",
                 shifted.size(), fmt::join(shifted, ", "), instanceRows,
                 instanceDefs, surveyTypes, surveyNames, passportNames);
  }

  void migrateToV32() {
    reusex::info("Migrating database to schema version 32");
    // Samples linked to specific survey parts (rux-frontend frontend-r46).
    // Unlike v24's samples.part_code (dropped in v25), a real foreign key:
    // deleting a part (or its type, which cascades to the parts) drops the
    // link, and a renamed code follows. The link stores no type — a part's
    // samples bear on the type it is filed under now (samples_for_type), so
    // re-filing a part carries its samples with it.
    execOrThrow("BEGIN TRANSACTION;");
    try {
      execOrThrow(R"(
        CREATE TABLE IF NOT EXISTS sample_part_links (
          sample_id INTEGER NOT NULL REFERENCES samples(id) ON DELETE CASCADE,
          part_code TEXT    NOT NULL REFERENCES survey_parts(code)
            ON DELETE CASCADE ON UPDATE CASCADE,
          PRIMARY KEY (sample_id, part_code)
        );
        CREATE INDEX IF NOT EXISTS idx_sample_part_links_part
          ON sample_part_links(part_code);
      )");
      insertSchemaVersion(32, "Link samples to survey parts "
                              "(sample_part_links)");
      execOrThrow("COMMIT;");
    } catch (...) {
      if (sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr) != SQLITE_OK)
        reusex::warn("migrateToV32: ROLLBACK failed: {}", sqlite3_errmsg(db));
      throw;
    }
    reusex::info("Migration to schema version 32 complete");
  }

  /// 1-based position of a report version in generation order.
  int reportVersionOf(int64_t id) const {
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(db,
                           "SELECT COUNT(*) FROM report_pdfs WHERE id <= ?;",
                           -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("report version: prepare failed: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int64(stmt, 1, id);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("report version: query failed: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_column_int(stmt, 0);
  }

  void migrateToV30() {
    reusex::info("Migrating database to schema version 30");
    // Solid-model WP1 (reusex-z1v.3.2): the storeys table, and the project
    // GUID every deterministic row GUID is derived from
    // (core::deterministic_guid). The GUID lives in project_settings, not in
    // `projects`: that is the legacy passport table, keyed by a user-chosen
    // text id and holding zero or several rows, so it cannot name "this
    // project". level_transform = 16 row-major doubles (world -> level frame),
    // like sensor_frames.transform. idx is UNIQUE: it is the key walls,
    // spaces and plane classes refer to a storey by.
    Savepoint sp(db, "migrate_v30");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS storeys (
        id              INTEGER PRIMARY KEY AUTOINCREMENT,
        guid            TEXT    NOT NULL UNIQUE,
        idx             INTEGER NOT NULL UNIQUE,
        name            TEXT    NOT NULL DEFAULT '',
        z_floor         REAL    NOT NULL,
        z_ceiling       REAL    NOT NULL,
        level_transform BLOB    NOT NULL CHECK (length(level_transform) = 128),
        source          TEXT    NOT NULL DEFAULT 'detected'
          CHECK (source IN ('detected', 'manual', 'imported')),
        locked          INTEGER NOT NULL DEFAULT 0 CHECK (locked IN (0, 1)),
        confidence      REAL    NOT NULL DEFAULT -1,
        metadata        TEXT    NOT NULL DEFAULT '',
        run_id          INTEGER,
        created_at      TEXT    NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS project_settings (
        key   TEXT PRIMARY KEY,
        value TEXT NOT NULL
      );
    )");
    {
      sqlite3_stmt *st = prepare_or_throw(
          db,
          "INSERT OR IGNORE INTO project_settings (key, value) "
          "VALUES ('project_guid', ?);",
          "migrateToV30");
      StmtGuard guard(st);
      bind_text(st, 1, core::generate_guid());
      if (sqlite3_step(st) != SQLITE_DONE)
        throw std::runtime_error("migrateToV30: failed to mint project_guid: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    insertSchemaVersion(30, "Solid model: storeys table; project_guid "
                            "setting (namespace of deterministic GUIDs)");
    sp.release();
    reusex::info("Migration to schema version 30 complete");
  }

  // ── Storeys (v30) ────────────────────────────────────────────────────────

  static constexpr const char *kStoreyColumns =
      "id, guid, idx, name, z_floor, z_ceiling, level_transform, source, "
      "locked, confidence, metadata";

  static core::StoreyRecord readStorey(sqlite3_stmt *s) {
    core::StoreyRecord r;
    r.id = sqlite3_column_int64(s, 0);
    r.guid = column_text(s, 1);
    r.idx = sqlite3_column_int(s, 2);
    r.name = column_text(s, 3);
    r.z_floor = sqlite3_column_double(s, 4);
    r.z_ceiling = sqlite3_column_double(s, 5);
    if (auto t = optionalBlob<16>(s, 6))
      r.level_transform = *t;
    r.source = column_text(s, 7);
    r.locked = sqlite3_column_int(s, 8) != 0;
    r.confidence = sqlite3_column_double(s, 9);
    r.metadata = column_text(s, 10);
    return r;
  }

  std::string projectGuid() const {
    auto g = getSetting("project_guid");
    if (!g || g->empty())
      throw std::runtime_error(
          "project_guid: project schema predates v30; open it read-write "
          "once to migrate: " +
          dbPath.string());
    static const std::regex kUuid(
        "^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
        "[0-9a-fA-F]{12}$");
    if (!std::regex_match(*g, kUuid))
      throw std::runtime_error("project_guid: the stored value '" + *g +
                               "' is not a UUID (project_settings key "
                               "'project_guid' was edited by hand?): " +
                               dbPath.string());
    return *g;
  }

  std::vector<core::StoreyRecord> listStoreys() const {
    std::vector<core::StoreyRecord> out;
    if (!hasStoreysTable)
      return out;
    const std::string sql =
        std::string("SELECT ") + kStoreyColumns + " FROM storeys ORDER BY idx;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "storeys");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back(readStorey(s));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("storeys: " + std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<core::StoreyRecord> storeyByGuid(std::string_view guid) const {
    if (!hasStoreysTable)
      return std::nullopt;
    const std::string sql = std::string("SELECT ") + kStoreyColumns +
                            " FROM storeys WHERE guid = ?;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "storey_by_guid");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return readStorey(s);
  }

  size_t saveStoreys(const std::vector<core::StoreyRecord> &storeys,
                     int runId) {
    checkWritable();
    if (!hasStoreysTable)
      throw std::runtime_error("save_storeys: project schema predates v30");
    std::set<std::string> guids;
    std::set<int> idxs;
    for (const auto &r : storeys) {
      if (r.guid.empty())
        throw std::invalid_argument("save_storeys: storey " +
                                    std::to_string(r.idx) + " has no guid");
      if (r.source != "detected" && r.source != "manual" &&
          r.source != "imported")
        throw std::invalid_argument("save_storeys: storey " +
                                    std::to_string(r.idx) +
                                    " has invalid source '" + r.source + "'");
      if (!guids.insert(r.guid).second)
        throw std::invalid_argument("save_storeys: duplicate guid " + r.guid);
      if (!idxs.insert(r.idx).second)
        throw std::invalid_argument("save_storeys: duplicate idx " +
                                    std::to_string(r.idx));
    }

    Savepoint sp(db, "save_storeys");
    std::set<std::string> lockedGuids;
    std::set<int> lockedIdx;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT guid, idx FROM storeys WHERE locked = 1;",
          "save_storeys");
      StmtGuard guard(s);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
        lockedGuids.insert(column_text(s, 0));
        lockedIdx.insert(sqlite3_column_int(s, 1));
      }
      if (rc != SQLITE_DONE)
        throw std::runtime_error("save_storeys: reading locked rows: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    execOrThrow("DELETE FROM storeys WHERE locked = 0;");

    sqlite3_stmt *ins = prepare_or_throw(
        db,
        "INSERT INTO storeys (guid, idx, name, z_floor, z_ceiling, "
        "level_transform, source, locked, confidence, metadata, run_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);",
        "save_storeys");
    StmtGuard guard(ins);
    size_t written = 0;
    for (const auto &r : storeys) {
      if (lockedGuids.count(r.guid)) {
        reusex::info("save_storeys: storey {} ({}) is locked; kept the stored "
                     "row",
                     r.idx, r.guid);
        continue;
      }
      if (lockedIdx.count(r.idx)) {
        reusex::warn("save_storeys: storey idx {} is held by a locked row "
                     "with another guid; the new storey {} was not written",
                     r.idx, r.guid);
        continue;
      }
      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);
      bind_text(ins, 1, r.guid);
      sqlite3_bind_int(ins, 2, r.idx);
      bind_text(ins, 3, r.name);
      sqlite3_bind_double(ins, 4, r.z_floor);
      sqlite3_bind_double(ins, 5, r.z_ceiling);
      sqlite3_bind_blob(ins, 6, r.level_transform.data(),
                        static_cast<int>(sizeof(double) * 16),
                        SQLITE_TRANSIENT);
      bind_text(ins, 7, r.source);
      sqlite3_bind_int(ins, 8, r.locked ? 1 : 0);
      sqlite3_bind_double(ins, 9, r.confidence);
      bind_text(ins, 10, r.metadata);
      if (runId >= 0)
        sqlite3_bind_int(ins, 11, runId);
      else
        sqlite3_bind_null(ins, 11);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("save_storeys: insert of storey " +
                                 std::to_string(r.idx) +
                                 " failed: " + sqlite3_errmsg(db));
      ++written;
    }
    sp.release();
    return written;
  }

  bool hasStoreys() const {
    if (!hasStoreysTable)
      return false;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT EXISTS (SELECT 1 FROM storeys);", "has_storeys");
    StmtGuard guard(s);
    return sqlite3_step(s) == SQLITE_ROW && sqlite3_column_int(s, 0) != 0;
  }

  void deleteStoreys() {
    checkWritable();
    if (hasStoreysTable)
      execOrThrow("DELETE FROM storeys;");
  }

  void migrateToV31() {
    reusex::info("Migrating database to schema version 31");
    // Solid-model WP2 (reusex-z1v.4.2): the role each plane of a label cloud
    // ("planes") plays. Keyed by the cloud's row and the plane label; the
    // rows go with the cloud (ON DELETE CASCADE). `storey` is the storey's
    // idx (the key solid-model records refer to a storey by), not a foreign
    // key to storeys(id): a storeys re-run replaces those rows, which a
    // foreign key would refuse. semantic_class / class_confidence are NULL
    // when no class won the vote.
    Savepoint sp(db, "migrate_v31");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS plane_classes (
        cloud_id         INTEGER NOT NULL
          REFERENCES point_clouds(id) ON DELETE CASCADE,
        plane_label      INTEGER NOT NULL CHECK (plane_label >= 1),
        guid             TEXT    NOT NULL UNIQUE,
        orientation      TEXT    NOT NULL
          CHECK (orientation IN ('horizontal', 'vertical', 'slanted')),
        semantic_class   INTEGER,
        class_confidence REAL,
        role             TEXT    NOT NULL
          CHECK (role IN ('wall', 'floor', 'ceiling', 'slab', 'furniture',
                          'clutter', 'unknown')),
        role_source      TEXT    NOT NULL DEFAULT 'none'
          CHECK (role_source IN ('semantic', 'rules', 'overruled', 'none',
                                 'manual')),
        storey           INTEGER NOT NULL DEFAULT -1,
        point_count      INTEGER NOT NULL DEFAULT 0,
        area             REAL    NOT NULL DEFAULT -1,
        z_min            REAL    NOT NULL DEFAULT 0,
        z_max            REAL    NOT NULL DEFAULT 0,
        source           TEXT    NOT NULL DEFAULT 'detected'
          CHECK (source IN ('detected', 'manual', 'imported')),
        locked           INTEGER NOT NULL DEFAULT 0 CHECK (locked IN (0, 1)),
        metadata         TEXT    NOT NULL DEFAULT '',
        run_id           INTEGER,
        created_at       TEXT    NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        PRIMARY KEY (cloud_id, plane_label)
      );
    )");
    insertSchemaVersion(31, "Solid model: plane_classes table (role of each "
                            "plane of a label cloud)");
    sp.release();
    reusex::info("Migration to schema version 31 complete");
  }

  // ── Plane classes (v31) ──────────────────────────────────────────────────

  static constexpr const char *kPlaneClassColumns =
      "pc.plane_label, pc.guid, pc.orientation, pc.semantic_class, "
      "pc.class_confidence, pc.role, pc.role_source, pc.storey, "
      "pc.point_count, pc.area, pc.z_min, pc.z_max, pc.source, pc.locked, "
      "pc.metadata";

  static core::PlaneClassRecord readPlaneClass(sqlite3_stmt *s) {
    core::PlaneClassRecord r;
    r.plane_label = sqlite3_column_int(s, 0);
    r.guid = column_text(s, 1);
    r.orientation = column_text(s, 2);
    r.semantic_class = sqlite3_column_type(s, 3) == SQLITE_NULL
                           ? -1
                           : sqlite3_column_int(s, 3);
    r.class_confidence = column_opt_double(s, 4).value_or(-1.0);
    r.role = column_text(s, 5);
    r.role_source = column_text(s, 6);
    r.storey = sqlite3_column_int(s, 7);
    r.point_count = sqlite3_column_int(s, 8);
    r.area = sqlite3_column_double(s, 9);
    r.z_min = sqlite3_column_double(s, 10);
    r.z_max = sqlite3_column_double(s, 11);
    r.source = column_text(s, 12);
    r.locked = sqlite3_column_int(s, 13) != 0;
    r.metadata = column_text(s, 14);
    return r;
  }

  std::vector<core::PlaneClassRecord>
  listPlaneClasses(std::string_view cloud) const {
    std::vector<core::PlaneClassRecord> out;
    if (!hasPlaneClassesTable)
      return out;
    const std::string sql =
        std::string("SELECT ") + kPlaneClassColumns +
        " FROM plane_classes pc JOIN point_clouds c ON c.id = pc.cloud_id "
        "WHERE c.name = ? ORDER BY pc.plane_label;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "plane_classes");
    StmtGuard guard(s);
    bind_text(s, 1, cloud);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back(readPlaneClass(s));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("plane_classes: " +
                               std::string(sqlite3_errmsg(db)));
    return out;
  }

  size_t savePlaneClasses(std::string_view cloud,
                          const std::vector<core::PlaneClassRecord> &classes,
                          int runId) {
    checkWritable();
    if (!hasPlaneClassesTable)
      throw std::runtime_error(
          "save_plane_classes: project schema predates v31");
    static const std::set<std::string, std::less<>> kOrientations = {
        "horizontal", "vertical", "slanted"};
    static const std::set<std::string, std::less<>> kRoles = {
        "wall", "floor", "ceiling", "slab", "furniture", "clutter", "unknown"};
    static const std::set<std::string, std::less<>> kRoleSources = {
        "semantic", "rules", "overruled", "none", "manual"};
    std::set<std::string> guids;
    std::set<int> labels;
    for (const auto &r : classes) {
      const auto where =
          "save_plane_classes: plane " + std::to_string(r.plane_label);
      if (r.plane_label < 1)
        throw std::invalid_argument(where + ": plane_label must be >= 1");
      if (r.guid.empty())
        throw std::invalid_argument(where + " has no guid");
      if (r.source != "detected" && r.source != "manual" &&
          r.source != "imported")
        throw std::invalid_argument(where + " has invalid source '" + r.source +
                                    "'");
      if (!kOrientations.count(r.orientation))
        throw std::invalid_argument(where + " has invalid orientation '" +
                                    r.orientation + "'");
      if (!kRoles.count(r.role))
        throw std::invalid_argument(where + " has invalid role '" + r.role +
                                    "'");
      if (!kRoleSources.count(r.role_source))
        throw std::invalid_argument(where + " has invalid role_source '" +
                                    r.role_source + "'");
      if (!guids.insert(r.guid).second)
        throw std::invalid_argument("save_plane_classes: duplicate guid " +
                                    r.guid);
      if (!labels.insert(r.plane_label).second)
        throw std::invalid_argument(
            "save_plane_classes: duplicate plane_label " +
            std::to_string(r.plane_label));
    }

    Savepoint sp(db, "save_plane_classes");
    const int cloudId = getCloudId(cloud); // throws for an unknown cloud
    std::set<std::string> lockedGuids;
    std::set<int> lockedLabels;
    {
      sqlite3_stmt *s =
          prepare_or_throw(db,
                           "SELECT guid, plane_label FROM plane_classes "
                           "WHERE locked = 1 AND cloud_id = ?;",
                           "save_plane_classes");
      StmtGuard guard(s);
      sqlite3_bind_int(s, 1, cloudId);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
        lockedGuids.insert(column_text(s, 0));
        lockedLabels.insert(sqlite3_column_int(s, 1));
      }
      if (rc != SQLITE_DONE)
        throw std::runtime_error("save_plane_classes: reading locked rows: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    {
      sqlite3_stmt *del = prepare_or_throw(
          db, "DELETE FROM plane_classes WHERE locked = 0 AND cloud_id = ?;",
          "save_plane_classes");
      StmtGuard guard(del);
      sqlite3_bind_int(del, 1, cloudId);
      if (sqlite3_step(del) != SQLITE_DONE)
        throw std::runtime_error("save_plane_classes: delete failed: " +
                                 std::string(sqlite3_errmsg(db)));
    }

    sqlite3_stmt *ins = prepare_or_throw(
        db,
        "INSERT INTO plane_classes (cloud_id, plane_label, guid, orientation, "
        "semantic_class, class_confidence, role, role_source, storey, "
        "point_count, area, z_min, z_max, source, locked, metadata, run_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);",
        "save_plane_classes");
    StmtGuard guard(ins);
    size_t written = 0;
    size_t skipped = 0;
    for (const auto &r : classes) {
      if (lockedLabels.count(r.plane_label) || lockedGuids.count(r.guid)) {
        ++skipped;
        continue;
      }
      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);
      sqlite3_bind_int(ins, 1, cloudId);
      sqlite3_bind_int(ins, 2, r.plane_label);
      bind_text(ins, 3, r.guid);
      bind_text(ins, 4, r.orientation);
      if (r.semantic_class >= 0)
        sqlite3_bind_int(ins, 5, r.semantic_class);
      else
        sqlite3_bind_null(ins, 5);
      if (r.class_confidence >= 0.0)
        sqlite3_bind_double(ins, 6, r.class_confidence);
      else
        sqlite3_bind_null(ins, 6);
      bind_text(ins, 7, r.role);
      bind_text(ins, 8, r.role_source);
      sqlite3_bind_int(ins, 9, r.storey);
      sqlite3_bind_int(ins, 10, r.point_count);
      sqlite3_bind_double(ins, 11, r.area);
      sqlite3_bind_double(ins, 12, r.z_min);
      sqlite3_bind_double(ins, 13, r.z_max);
      bind_text(ins, 14, r.source);
      sqlite3_bind_int(ins, 15, r.locked ? 1 : 0);
      bind_text(ins, 16, r.metadata);
      if (runId >= 0)
        sqlite3_bind_int(ins, 17, runId);
      else
        sqlite3_bind_null(ins, 17);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("save_plane_classes: insert of plane " +
                                 std::to_string(r.plane_label) +
                                 " failed: " + sqlite3_errmsg(db));
      ++written;
    }
    sp.release();
    if (skipped > 0)
      reusex::info("save_plane_classes: {} plane(s) of '{}' are locked; kept "
                   "the stored rows",
                   skipped, cloud);
    return written;
  }

  bool hasPlaneClasses(std::string_view cloud) const {
    if (!hasPlaneClassesTable)
      return false;
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "SELECT EXISTS (SELECT 1 FROM plane_classes pc JOIN point_clouds c "
        "ON c.id = pc.cloud_id WHERE c.name = ?);",
        "has_plane_classes");
    StmtGuard guard(s);
    bind_text(s, 1, cloud);
    return sqlite3_step(s) == SQLITE_ROW && sqlite3_column_int(s, 0) != 0;
  }

  void deletePlaneClasses(std::string_view cloud) {
    checkWritable();
    if (!hasPlaneClassesTable)
      return;
    sqlite3_stmt *s =
        prepare_or_throw(db,
                         "DELETE FROM plane_classes WHERE cloud_id IN "
                         "(SELECT id FROM point_clouds WHERE name = ?);",
                         "delete_plane_classes");
    StmtGuard guard(s);
    bind_text(s, 1, cloud);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("delete_plane_classes: " +
                               std::string(sqlite3_errmsg(db)));
  }

  void migrateToV33() {
    reusex::info("Migrating database to schema version 33");
    // Solid-model WP3 (reusex-z1v.5.5): walls with a measured thickness and
    // the joins between them. Coordinates are in the level frame of the
    // wall's storey. `storey` is the storey's idx, not a foreign key to
    // storeys(id): a storeys re-run replaces those rows, which a foreign key
    // would refuse (the plane_classes rule). wall_joins refer to walls by row
    // id and go with them (ON DELETE CASCADE); the API speaks guids.
    Savepoint sp(db, "migrate_v33");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS walls (
        id              INTEGER PRIMARY KEY AUTOINCREMENT,
        guid            TEXT    NOT NULL UNIQUE,
        storey          INTEGER NOT NULL,
        x0              REAL    NOT NULL,
        y0              REAL    NOT NULL,
        x1              REAL    NOT NULL,
        y1              REAL    NOT NULL,
        z_base          REAL    NOT NULL DEFAULT 0,
        height          REAL    NOT NULL DEFAULT 0,
        thickness       REAL    NOT NULL DEFAULT -1,
        thickness_ci_lo REAL    NOT NULL DEFAULT -1,
        thickness_ci_hi REAL    NOT NULL DEFAULT -1,
        lean_deg        REAL    NOT NULL DEFAULT 0,
        is_external     INTEGER NOT NULL DEFAULT 0 CHECK (is_external IN (0, 1)),
        assumed         INTEGER NOT NULL DEFAULT 0 CHECK (assumed IN (0, 1)),
        seen_share      REAL    NOT NULL DEFAULT -1,
        face_a_points   INTEGER NOT NULL DEFAULT 0,
        face_b_points   INTEGER NOT NULL DEFAULT 0,
        co_visibility   REAL    NOT NULL DEFAULT -1,
        axis_family     INTEGER NOT NULL DEFAULT -1,
        source          TEXT    NOT NULL DEFAULT 'detected'
          CHECK (source IN ('detected', 'manual', 'imported')),
        locked          INTEGER NOT NULL DEFAULT 0 CHECK (locked IN (0, 1)),
        confidence      REAL    NOT NULL DEFAULT -1,
        metadata        TEXT    NOT NULL DEFAULT '',
        run_id          INTEGER,
        created_at      TEXT    NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE INDEX IF NOT EXISTS idx_walls_storey ON walls(storey);
      CREATE TABLE IF NOT EXISTS wall_joins (
        wall_a INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
        wall_b INTEGER NOT NULL REFERENCES walls(id) ON DELETE CASCADE,
        kind   TEXT    NOT NULL CHECK (kind IN ('corner', 'tee', 'collinear')),
        at_a   TEXT    NOT NULL CHECK (at_a IN ('start', 'end', 'path')),
        at_b   TEXT    NOT NULL CHECK (at_b IN ('start', 'end', 'path')),
        CHECK (wall_a <> wall_b),
        PRIMARY KEY (wall_a, wall_b)
      );
      CREATE INDEX IF NOT EXISTS idx_wall_joins_b ON wall_joins(wall_b);
    )");
    insertSchemaVersion(33, "Solid model: walls and wall_joins tables");
    sp.release();
    reusex::info("Migration to schema version 33 complete");
  }

  // ── Walls (v33) ──────────────────────────────────────────────────────────

  static constexpr const char *kWallColumns =
      "id, guid, storey, x0, y0, x1, y1, z_base, height, thickness, "
      "thickness_ci_lo, thickness_ci_hi, lean_deg, is_external, assumed, "
      "seen_share, face_a_points, face_b_points, co_visibility, axis_family, "
      "source, locked, confidence, metadata";

  static core::WallRecord readWall(sqlite3_stmt *s) {
    core::WallRecord r;
    r.id = sqlite3_column_int64(s, 0);
    r.guid = column_text(s, 1);
    r.storey = sqlite3_column_int(s, 2);
    r.x0 = sqlite3_column_double(s, 3);
    r.y0 = sqlite3_column_double(s, 4);
    r.x1 = sqlite3_column_double(s, 5);
    r.y1 = sqlite3_column_double(s, 6);
    r.z_base = sqlite3_column_double(s, 7);
    r.height = sqlite3_column_double(s, 8);
    r.thickness = sqlite3_column_double(s, 9);
    r.thickness_ci_lo = sqlite3_column_double(s, 10);
    r.thickness_ci_hi = sqlite3_column_double(s, 11);
    r.lean_deg = sqlite3_column_double(s, 12);
    r.is_external = sqlite3_column_int(s, 13) != 0;
    r.assumed = sqlite3_column_int(s, 14) != 0;
    r.seen_share = sqlite3_column_double(s, 15);
    r.face_a_points = sqlite3_column_int(s, 16);
    r.face_b_points = sqlite3_column_int(s, 17);
    r.co_visibility = sqlite3_column_double(s, 18);
    r.axis_family = sqlite3_column_int(s, 19);
    r.source = column_text(s, 20);
    r.locked = sqlite3_column_int(s, 21) != 0;
    r.confidence = sqlite3_column_double(s, 22);
    r.metadata = column_text(s, 23);
    return r;
  }

  std::vector<core::WallRecord> listWalls() const {
    std::vector<core::WallRecord> out;
    if (!hasWallsTable)
      return out;
    const std::string sql = std::string("SELECT ") + kWallColumns +
                            " FROM walls ORDER BY storey, id;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "walls");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back(readWall(s));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("walls: " + std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<core::WallRecord> wallByGuid(std::string_view guid) const {
    if (!hasWallsTable)
      return std::nullopt;
    const std::string sql =
        std::string("SELECT ") + kWallColumns + " FROM walls WHERE guid = ?;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "wall_by_guid");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return readWall(s);
  }

  std::vector<core::WallJoinRecord> listWallJoins() const {
    std::vector<core::WallJoinRecord> out;
    if (!hasWallsTable)
      return out;
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "SELECT a.guid, b.guid, j.kind, j.at_a, j.at_b FROM wall_joins j "
        "JOIN walls a ON a.id = j.wall_a JOIN walls b ON b.id = j.wall_b "
        "ORDER BY j.wall_a, j.wall_b;",
        "wall_joins");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back({column_text(s, 0), column_text(s, 1), column_text(s, 2),
                     column_text(s, 3), column_text(s, 4)});
    if (rc != SQLITE_DONE)
      throw std::runtime_error("wall_joins: " +
                               std::string(sqlite3_errmsg(db)));
    return out;
  }

  size_t saveWalls(const std::vector<core::WallRecord> &walls,
                   const std::vector<core::WallJoinRecord> &joins, int runId) {
    checkWritable();
    if (!hasWallsTable)
      throw std::runtime_error("save_walls: project schema predates v33");
    std::set<std::string, std::less<>> guids;
    for (const auto &r : walls) {
      if (r.guid.empty())
        throw std::invalid_argument("save_walls: a wall of storey " +
                                    std::to_string(r.storey) + " has no guid");
      if (r.source != "detected" && r.source != "manual" &&
          r.source != "imported")
        throw std::invalid_argument("save_walls: wall " + r.guid +
                                    " has invalid source '" + r.source + "'");
      if (!guids.insert(r.guid).second)
        throw std::invalid_argument("save_walls: duplicate guid " + r.guid);
    }

    Savepoint sp(db, "save_walls");
    std::set<std::string, std::less<>> lockedGuids;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT guid FROM walls WHERE locked = 1;", "save_walls");
      StmtGuard guard(s);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW)
        lockedGuids.insert(column_text(s, 0));
      if (rc != SQLITE_DONE)
        throw std::runtime_error("save_walls: reading locked rows: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    static const std::set<std::string, std::less<>> kKinds = {"corner", "tee",
                                                              "collinear"};
    static const std::set<std::string, std::less<>> kEnds = {"start", "end",
                                                             "path"};
    for (const auto &j : joins) {
      const auto where = "save_walls: join " + j.wall_a + " - " + j.wall_b;
      if (!kKinds.count(j.kind))
        throw std::invalid_argument(where + " has invalid kind '" + j.kind +
                                    "'");
      if (!kEnds.count(j.at_a) || !kEnds.count(j.at_b))
        throw std::invalid_argument(where + " has an invalid end ('" + j.at_a +
                                    "', '" + j.at_b + "')");
      if (j.wall_a == j.wall_b)
        throw std::invalid_argument(where + " joins a wall to itself");
      for (const auto &g : {j.wall_a, j.wall_b})
        if (!guids.count(g) && !lockedGuids.count(g))
          throw std::invalid_argument(where + " names wall " + g +
                                      ", which is neither saved nor locked");
    }
    // Joins of unlocked walls go with them (ON DELETE CASCADE).
    execOrThrow("DELETE FROM walls WHERE locked = 0;");

    sqlite3_stmt *ins = prepare_or_throw(
        db,
        "INSERT INTO walls (guid, storey, x0, y0, x1, y1, z_base, height, "
        "thickness, thickness_ci_lo, thickness_ci_hi, lean_deg, is_external, "
        "assumed, seen_share, face_a_points, face_b_points, co_visibility, "
        "axis_family, source, locked, confidence, metadata, run_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, "
        "?, "
        "?, ?, ?);",
        "save_walls");
    StmtGuard guard(ins);
    size_t written = 0;
    size_t skipped = 0;
    for (const auto &r : walls) {
      if (lockedGuids.count(r.guid)) {
        ++skipped;
        continue;
      }
      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);
      bind_text(ins, 1, r.guid);
      sqlite3_bind_int(ins, 2, r.storey);
      sqlite3_bind_double(ins, 3, r.x0);
      sqlite3_bind_double(ins, 4, r.y0);
      sqlite3_bind_double(ins, 5, r.x1);
      sqlite3_bind_double(ins, 6, r.y1);
      sqlite3_bind_double(ins, 7, r.z_base);
      sqlite3_bind_double(ins, 8, r.height);
      sqlite3_bind_double(ins, 9, r.thickness);
      sqlite3_bind_double(ins, 10, r.thickness_ci_lo);
      sqlite3_bind_double(ins, 11, r.thickness_ci_hi);
      sqlite3_bind_double(ins, 12, r.lean_deg);
      sqlite3_bind_int(ins, 13, r.is_external ? 1 : 0);
      sqlite3_bind_int(ins, 14, r.assumed ? 1 : 0);
      sqlite3_bind_double(ins, 15, r.seen_share);
      sqlite3_bind_int(ins, 16, r.face_a_points);
      sqlite3_bind_int(ins, 17, r.face_b_points);
      sqlite3_bind_double(ins, 18, r.co_visibility);
      sqlite3_bind_int(ins, 19, r.axis_family);
      bind_text(ins, 20, r.source);
      sqlite3_bind_int(ins, 21, r.locked ? 1 : 0);
      sqlite3_bind_double(ins, 22, r.confidence);
      bind_text(ins, 23, r.metadata);
      if (runId >= 0)
        sqlite3_bind_int(ins, 24, runId);
      else
        sqlite3_bind_null(ins, 24);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("save_walls: insert of wall " + r.guid +
                                 " failed: " + sqlite3_errmsg(db));
      ++written;
    }

    // A join already stored between the same two walls (two locked walls
    // kept from an earlier run) stays as it is.
    sqlite3_stmt *join = prepare_or_throw(
        db,
        "INSERT OR IGNORE INTO wall_joins (wall_a, wall_b, kind, at_a, at_b) "
        "SELECT a.id, b.id, ?, ?, ? FROM walls a, walls b "
        "WHERE a.guid = ? AND b.guid = ?;",
        "save_walls");
    StmtGuard joinGuard(join);
    for (const auto &j : joins) {
      sqlite3_reset(join);
      sqlite3_clear_bindings(join);
      bind_text(join, 1, j.kind);
      bind_text(join, 2, j.at_a);
      bind_text(join, 3, j.at_b);
      bind_text(join, 4, j.wall_a);
      bind_text(join, 5, j.wall_b);
      if (sqlite3_step(join) != SQLITE_DONE)
        throw std::runtime_error("save_walls: insert of join " + j.wall_a +
                                 " - " + j.wall_b +
                                 " failed: " + sqlite3_errmsg(db));
    }
    sp.release();
    if (skipped > 0)
      reusex::info("save_walls: {} wall(s) are locked; kept the stored rows",
                   skipped);
    return written;
  }

  bool hasWalls() const {
    if (!hasWallsTable)
      return false;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT EXISTS (SELECT 1 FROM walls);", "has_walls");
    StmtGuard guard(s);
    return sqlite3_step(s) == SQLITE_ROW && sqlite3_column_int(s, 0) != 0;
  }

  void deleteWalls() {
    checkWritable();
    if (hasWallsTable)
      execOrThrow("DELETE FROM wall_joins; DELETE FROM walls;");
  }

  void migrateToV34() {
    reusex::info("Migrating database to schema version 34");
    // Solid-model WP5 (reusex-z1v.7.3): doors, windows and plain openings in
    // the walls' faces. The host wall is named by its guid (wall_guid), not
    // by a foreign key to walls(id): a walls re-run deletes and re-inserts
    // its unlocked rows, and a cascading key would take every opening (a
    // locked, hand-fixed one too) with them, while the wall's GUID survives
    // the re-run (core/wall_identity.hpp). An opening whose host is gone
    // stays until the openings stage runs again; load_solid_model() skips it.
    Savepoint sp(db, "migrate_v34");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS openings (
        id                   INTEGER PRIMARY KEY AUTOINCREMENT,
        guid                 TEXT    NOT NULL UNIQUE,
        wall_guid            TEXT    NOT NULL,
        kind                 TEXT    NOT NULL
          CHECK (kind IN ('door', 'window', 'opening')),
        s_start              REAL    NOT NULL,
        s_end                REAL    NOT NULL,
        z_sill               REAL    NOT NULL,
        z_head               REAL    NOT NULL,
        state                TEXT    NOT NULL DEFAULT 'unknown'
          CHECK (state IN ('open', 'closed', 'unknown')),
        is_external          INTEGER NOT NULL DEFAULT 0
          CHECK (is_external IN (0, 1)),
        source_instance_guid TEXT    NOT NULL DEFAULT '',
        source               TEXT    NOT NULL DEFAULT 'detected'
          CHECK (source IN ('detected', 'manual', 'imported')),
        locked               INTEGER NOT NULL DEFAULT 0 CHECK (locked IN (0, 1)),
        confidence           REAL    NOT NULL DEFAULT -1,
        metadata             TEXT    NOT NULL DEFAULT '',
        run_id               INTEGER,
        created_at           TEXT    NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE INDEX IF NOT EXISTS idx_openings_wall ON openings(wall_guid);
    )");
    insertSchemaVersion(34, "Solid model: openings table");
    sp.release();
    reusex::info("Migration to schema version 34 complete");
  }

  // ── Openings (v34) ───────────────────────────────────────────────────────

  static constexpr const char *kOpeningColumns =
      "id, guid, wall_guid, kind, s_start, s_end, z_sill, z_head, state, "
      "is_external, source_instance_guid, source, locked, confidence, metadata";

  static core::OpeningRecord readOpening(sqlite3_stmt *s) {
    core::OpeningRecord r;
    r.id = sqlite3_column_int64(s, 0);
    r.guid = column_text(s, 1);
    r.wall = column_text(s, 2);
    r.kind = column_text(s, 3);
    r.s_start = sqlite3_column_double(s, 4);
    r.s_end = sqlite3_column_double(s, 5);
    r.z_sill = sqlite3_column_double(s, 6);
    r.z_head = sqlite3_column_double(s, 7);
    r.state = column_text(s, 8);
    r.is_external = sqlite3_column_int(s, 9) != 0;
    r.source_instance_guid = column_text(s, 10);
    r.source = column_text(s, 11);
    r.locked = sqlite3_column_int(s, 12) != 0;
    r.confidence = sqlite3_column_double(s, 13);
    r.metadata = column_text(s, 14);
    return r;
  }

  std::vector<core::OpeningRecord> listOpenings() const {
    std::vector<core::OpeningRecord> out;
    if (!hasOpeningsTable)
      return out;
    const std::string sql = std::string("SELECT ") + kOpeningColumns +
                            " FROM openings ORDER BY wall_guid, s_start, id;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "openings");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back(readOpening(s));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("openings: " + std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<core::OpeningRecord>
  openingByGuid(std::string_view guid) const {
    if (!hasOpeningsTable)
      return std::nullopt;
    const std::string sql = std::string("SELECT ") + kOpeningColumns +
                            " FROM openings WHERE guid = ?;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "opening_by_guid");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return readOpening(s);
  }

  size_t saveOpenings(const std::vector<core::OpeningRecord> &openings,
                      int runId) {
    checkWritable();
    if (!hasOpeningsTable)
      throw std::runtime_error("save_openings: project schema predates v34");
    static const std::set<std::string, std::less<>> kKinds = {"door", "window",
                                                              "opening"};
    static const std::set<std::string, std::less<>> kStates = {"open", "closed",
                                                               "unknown"};
    std::set<std::string, std::less<>> guids;
    for (const auto &r : openings) {
      if (r.guid.empty())
        throw std::invalid_argument("save_openings: an opening in wall " +
                                    r.wall + " has no guid");
      const auto where = "save_openings: opening " + r.guid;
      if (r.wall.empty())
        throw std::invalid_argument(where + " names no host wall");
      if (!kKinds.count(r.kind))
        throw std::invalid_argument(where + " has invalid kind '" + r.kind +
                                    "'");
      if (!kStates.count(r.state))
        throw std::invalid_argument(where + " has invalid state '" + r.state +
                                    "'");
      if (r.source != "detected" && r.source != "manual" &&
          r.source != "imported")
        throw std::invalid_argument(where + " has invalid source '" + r.source +
                                    "'");
      if (!(r.s_end > r.s_start) || !(r.z_head > r.z_sill))
        throw std::invalid_argument(
            where + " is empty (s " + std::to_string(r.s_start) + ".." +
            std::to_string(r.s_end) + ", z " + std::to_string(r.z_sill) + ".." +
            std::to_string(r.z_head) + ")");
      if (!guids.insert(r.guid).second)
        throw std::invalid_argument("save_openings: duplicate guid " + r.guid);
    }

    Savepoint sp(db, "save_openings");
    std::set<std::string, std::less<>> lockedGuids;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT guid FROM openings WHERE locked = 1;", "save_openings");
      StmtGuard guard(s);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW)
        lockedGuids.insert(column_text(s, 0));
      if (rc != SQLITE_DONE)
        throw std::runtime_error("save_openings: reading locked rows: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    execOrThrow("DELETE FROM openings WHERE locked = 0;");

    sqlite3_stmt *ins = prepare_or_throw(
        db,
        "INSERT INTO openings (guid, wall_guid, kind, s_start, s_end, z_sill, "
        "z_head, state, is_external, source_instance_guid, source, locked, "
        "confidence, metadata, run_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);",
        "save_openings");
    StmtGuard guard(ins);
    size_t written = 0, skipped = 0;
    for (const auto &r : openings) {
      if (lockedGuids.count(r.guid)) {
        ++skipped;
        continue;
      }
      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);
      bind_text(ins, 1, r.guid);
      bind_text(ins, 2, r.wall);
      bind_text(ins, 3, r.kind);
      sqlite3_bind_double(ins, 4, r.s_start);
      sqlite3_bind_double(ins, 5, r.s_end);
      sqlite3_bind_double(ins, 6, r.z_sill);
      sqlite3_bind_double(ins, 7, r.z_head);
      bind_text(ins, 8, r.state);
      sqlite3_bind_int(ins, 9, r.is_external ? 1 : 0);
      bind_text(ins, 10, r.source_instance_guid);
      bind_text(ins, 11, r.source);
      sqlite3_bind_int(ins, 12, r.locked ? 1 : 0);
      sqlite3_bind_double(ins, 13, r.confidence);
      bind_text(ins, 14, r.metadata);
      if (runId >= 0)
        sqlite3_bind_int(ins, 15, runId);
      else
        sqlite3_bind_null(ins, 15);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("save_openings: insert of opening " + r.guid +
                                 " failed: " + sqlite3_errmsg(db));
      ++written;
    }
    sp.release();
    if (skipped > 0)
      reusex::info(
          "save_openings: {} opening(s) are locked; kept the stored rows",
          skipped);
    return written;
  }

  bool hasOpenings() const {
    if (!hasOpeningsTable)
      return false;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT EXISTS (SELECT 1 FROM openings);", "has_openings");
    StmtGuard guard(s);
    return sqlite3_step(s) == SQLITE_ROW && sqlite3_column_int(s, 0) != 0;
  }

  void deleteOpenings() {
    checkWritable();
    if (hasOpeningsTable)
      execOrThrow("DELETE FROM openings;");
  }

  void migrateToV35() {
    reusex::info("Migrating database to schema version 35");
    // Solid-model WP4 (reusex-z1v.6.3): spaces (rooms, corridors, open plan)
    // as polygons in their storey's level frame, and the stretches of wall
    // that bound them (IfcRelSpaceBoundary). `storey` is the storey's idx
    // (the walls rule). A boundary goes with its space (ON DELETE CASCADE)
    // but names its wall by guid, not by a foreign key to walls(id): a walls
    // re-run deletes and re-inserts its unlocked rows, and a cascading key
    // would take the boundaries of a locked space with them, while the
    // wall's GUID survives the re-run (the openings rule, v34).
    Savepoint sp(db, "migrate_v35");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS spaces (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        guid        TEXT    NOT NULL UNIQUE,
        storey      INTEGER NOT NULL,
        name        TEXT    NOT NULL DEFAULT '',
        kind        TEXT    NOT NULL DEFAULT 'unknown'
          CHECK (kind IN ('room', 'corridor', 'open_plan', 'unknown')),
        polygon     BLOB    NOT NULL,
        z_floor     REAL    NOT NULL DEFAULT 0,
        z_ceiling   REAL    NOT NULL DEFAULT 0,
        area        REAL    NOT NULL DEFAULT -1,
        perimeter   REAL    NOT NULL DEFAULT -1,
        volume      REAL    NOT NULL DEFAULT -1,
        compactness REAL    NOT NULL DEFAULT -1,
        room_label  INTEGER,
        source      TEXT    NOT NULL DEFAULT 'detected'
          CHECK (source IN ('detected', 'manual', 'imported')),
        locked      INTEGER NOT NULL DEFAULT 0 CHECK (locked IN (0, 1)),
        confidence  REAL    NOT NULL DEFAULT -1,
        metadata    TEXT    NOT NULL DEFAULT '',
        run_id      INTEGER,
        created_at  TEXT    NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE INDEX IF NOT EXISTS idx_spaces_storey ON spaces(storey);
      CREATE TABLE IF NOT EXISTS space_boundaries (
        space_id  INTEGER NOT NULL REFERENCES spaces(id) ON DELETE CASCADE,
        wall_guid TEXT    NOT NULL,
        side      TEXT    NOT NULL CHECK (side IN ('a', 'b')),
        s_start   REAL    NOT NULL,
        s_end     REAL    NOT NULL,
        PRIMARY KEY (space_id, wall_guid, side)
      );
      CREATE INDEX IF NOT EXISTS idx_space_boundaries_wall
        ON space_boundaries(wall_guid);
    )");
    insertSchemaVersion(35, "Solid model: spaces and space_boundaries tables");
    sp.release();
    reusex::info("Migration to schema version 35 complete");
  }

  void migrateToV36() {
    reusex::info("Migrating database to schema version 36");
    // Non-destructive dismissal of one survey part ("Markér som fejl" /
    // "Gendan"): the part stays, keeps its code, passport and links, and is
    // listed, but core/survey_service leaves it out of its type's quantity
    // and of rooms_without_parts. Unlike survey_dismissed_instances (v26, a
    // deleted part's tombstone) it is reversible. Guarded so a test that
    // rolls the version back re-runs it safely.
    Savepoint sp(db, "migrate_v36");
    if (!columnExists("survey_parts", "dismissed"))
      execOrThrow("ALTER TABLE survey_parts ADD COLUMN dismissed INTEGER NOT "
                  "NULL DEFAULT 0 CHECK (dismissed IN (0, 1));");
    insertSchemaVersion(36, "survey_parts.dismissed: non-destructive part "
                            "dismissal");
    sp.release();
    reusex::info("Migration to schema version 36 complete");
  }

  void migrateToV37() {
    reusex::info("Migrating database to schema version 37");
    // Survey quantities and mass grounded in the model (reusex-uprf):
    //  - survey_parts.element_guid / geom_quantity / geom_unit / geom_source:
    //    the wall, opening or slab piece a part is measured on, and what was
    //    measured (core::measure_survey). element_guid is unique (an
    //    element-backed part is keyed on it), plain TEXT: walls / openings
    //    re-runs replace rows but keep their guids.
    //  - survey_types.buildup_id (an id of core::buildup_library) and
    //    element_category (the type core::measure_survey seeded from the
    //    typed model). mass_t stays and now means the user's override.
    //  - buildups: project build-ups overriding / adding library entries.
    //  - value_provenance: where each survey value came from (gap 2). No
    //    foreign key (scope + id name a type or a part); ProjectDB deletes
    //    the rows with their type / part.
    // The frontend kept the chosen build-up as the user column "Opbygning"
    // (one value per part passport); each type takes the name most of its
    // parts carry, when it names a library build-up.
    Savepoint sp(db, "migrate_v37");
    ensureSurveyMassSchemaV37();
    insertSchemaVersion(37, "survey mass: part geometry, build-ups, "
                            "value provenance");
    sp.release();
    reusex::info("Migration to schema version 37 complete");
  }

  /// The survey-mass shape of v37 is all present.
  bool surveyMassSchemaPresent() {
    return columnExists("survey_parts", "element_guid") &&
           columnExists("survey_parts", "geom_quantity") &&
           columnExists("survey_parts", "geom_unit") &&
           columnExists("survey_parts", "geom_source") &&
           columnExists("survey_types", "buildup_id") &&
           columnExists("survey_types", "element_category") &&
           tableExists("buildups") && tableExists("value_provenance");
  }

  /// v37 DDL + the "Opbygning" backfill; every step is guarded, so v38 can
  /// re-run it on a database whose v37 was a different branch's.
  void ensureSurveyMassSchemaV37() {
    if (!columnExists("survey_parts", "element_guid"))
      execOrThrow("ALTER TABLE survey_parts ADD COLUMN element_guid TEXT;");
    if (!columnExists("survey_parts", "geom_quantity"))
      execOrThrow("ALTER TABLE survey_parts ADD COLUMN geom_quantity REAL;");
    if (!columnExists("survey_parts", "geom_unit"))
      execOrThrow("ALTER TABLE survey_parts ADD COLUMN geom_unit TEXT NOT "
                  "NULL DEFAULT '';");
    if (!columnExists("survey_parts", "geom_source"))
      execOrThrow(
          "ALTER TABLE survey_parts ADD COLUMN geom_source TEXT NOT NULL "
          "DEFAULT '' CHECK (geom_source IN ('', 'solid_volume', "
          "'wall_area', 'opening_area', 'slab_area', 'plane_area', "
          "'count'));");
    execOrThrow("CREATE UNIQUE INDEX IF NOT EXISTS idx_survey_parts_element "
                "ON survey_parts(element_guid) WHERE element_guid IS NOT "
                "NULL;");
    if (!columnExists("survey_types", "buildup_id"))
      execOrThrow("ALTER TABLE survey_types ADD COLUMN buildup_id TEXT;");
    if (!columnExists("survey_types", "element_category"))
      execOrThrow("ALTER TABLE survey_types ADD COLUMN element_category TEXT "
                  "CHECK (element_category IS NULL OR element_category IN "
                  "('ydervaeg', 'indervaeg', 'daek', 'tag'));");
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS buildups (
        id         TEXT PRIMARY KEY,
        definition TEXT NOT NULL,
        updated_at TEXT NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS value_provenance (
        scope      TEXT NOT NULL CHECK (scope IN ('type', 'part')),
        id         TEXT NOT NULL,
        field      TEXT NOT NULL,
        kind       TEXT NOT NULL CHECK (kind IN ('scan', 'measured',
                     'calculated', 'entered', 'suggested')),
        source     TEXT NOT NULL DEFAULT '',
        confidence REAL CHECK (confidence IS NULL OR
                     (confidence >= 0 AND confidence <= 1)),
        updated_at TEXT NOT NULL
          DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        PRIMARY KEY (scope, id, field)
      );
    )");
    backfillBuildupsFromUserColumnV37();
  }

  /// v37: the frontend's "Opbygning" user column -> survey_types.buildup_id.
  void backfillBuildupsFromUserColumnV37() {
    // A fresh database runs the migrations before createTables() gives the
    // passport tables their current shape (see the v29 guard above).
    if (!columnExists("survey_parts", "passport_guid") ||
        !columnExists("passport_property_values", "value") ||
        !columnExists("passport_property_values", "property_id") ||
        !columnExists("property_definitions", "name_en") ||
        !columnExists("material_passports", "document_guid"))
      return;
    sqlite3_stmt *s = nullptr;
    if (sqlite3_prepare_v2(
            db,
            "SELECT p.type_id, CAST(v.value AS TEXT) FROM survey_parts p "
            "JOIN material_passports mp ON mp.document_guid = "
            "p.passport_guid JOIN passport_property_values v ON "
            "v.passport_id = mp.id JOIN property_definitions d ON d.id = "
            "v.property_id WHERE lower(trim(d.name_en)) = 'opbygning' AND "
            "v.value IS NOT NULL;",
            -1, &s, nullptr) != SQLITE_OK)
      throw std::runtime_error("migrate_v37: " +
                               std::string(sqlite3_errmsg(db)));
    std::map<int64_t, std::map<std::string, int>> votes;
    while (sqlite3_step(s) == SQLITE_ROW) {
      const auto *txt = sqlite3_column_text(s, 1);
      if (txt)
        ++votes[sqlite3_column_int64(s, 0)]
               [reinterpret_cast<const char *>(txt)];
    }
    sqlite3_finalize(s);
    if (votes.empty())
      return;
    const auto &lib = core::default_buildups();
    std::size_t set = 0, unknown = 0;
    for (const auto &[type_id, names] : votes) {
      // Most parts' name; ties go to the first name in sort order.
      const auto best = std::max_element(
          names.begin(), names.end(),
          [](const auto &a, const auto &b) { return a.second < b.second; });
      const auto b = core::find_buildup_by_name(lib, best->first);
      if (!b) {
        ++unknown;
        continue;
      }
      const std::string id = std::to_string(type_id);
      for (const char *sql :
           {"UPDATE survey_types SET buildup_id = ?1 WHERE id = ?2 AND "
            "buildup_id IS NULL;",
            "INSERT OR IGNORE INTO value_provenance (scope, id, field, kind, "
            "source) VALUES ('type', ?2, 'buildup_id', 'entered', "
            "'migrated:Opbygning');"}) {
        sqlite3_stmt *u = nullptr;
        if (sqlite3_prepare_v2(db, sql, -1, &u, nullptr) != SQLITE_OK)
          throw std::runtime_error("migrate_v37: " +
                                   std::string(sqlite3_errmsg(db)));
        sqlite3_bind_text(u, 1, b->id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(u, 2, id.c_str(), -1, SQLITE_TRANSIENT);
        const int rc = sqlite3_step(u);
        sqlite3_finalize(u);
        if (rc != SQLITE_DONE)
          throw std::runtime_error("migrate_v37: " +
                                   std::string(sqlite3_errmsg(db)));
      }
      ++set;
    }
    reusex::info("migrate_v37: {} survey type(s) took their build-up from the "
                 "'Opbygning' user column",
                 set);
    if (unknown > 0)
      reusex::warn("migrate_v37: {} survey type(s) carry an 'Opbygning' value "
                   "that names no library build-up; left without one",
                   unknown);
  }

  void migrateToV38() {
    reusex::info("Migrating database to schema version 38");
    // Named images the PDF report uses that are not a sensor frame or a
    // panorama: today only the uploaded cover image ("cover",
    // core/report_cover.hpp).
    //
    // Two branches each shipped a v37 before they were merged: survey mass
    // (this file's migrateToV37) and report_assets (now this v38). A project
    // migrated by either one alone is stamped v37 but lacks the other half,
    // so both halves are re-applied here behind existence checks.
    Savepoint sp(db, "migrate_v38");
    if (!surveyMassSchemaPresent()) {
      reusex::warn("migrate_v38: schema v37 lacks the survey-mass columns "
                   "(a report_assets-only v37); adding them");
      ensureSurveyMassSchemaV37();
    }
    execOrThrow(R"(
      CREATE TABLE IF NOT EXISTS report_assets (
        name       TEXT PRIMARY KEY,
        mime_type  TEXT NOT NULL,
        data       BLOB NOT NULL,
        updated_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ', 'now'))
      );
    )");
    insertSchemaVersion(38, "report_assets: uploaded report images (cover)");
    sp.release();
    hasReportAssetsTable = true;
    reusex::info("Migration to schema version 38 complete");
  }

  // ── Spaces (v35) ─────────────────────────────────────────────────────────

  static constexpr const char *kSpaceColumns =
      "id, guid, storey, name, kind, polygon, z_floor, z_ceiling, area, "
      "perimeter, volume, compactness, room_label, source, locked, "
      "confidence, metadata";

  static core::SpaceRecord readSpace(sqlite3_stmt *s) {
    core::SpaceRecord r;
    r.id = sqlite3_column_int64(s, 0);
    r.guid = column_text(s, 1);
    r.storey = sqlite3_column_int(s, 2);
    r.name = column_text(s, 3);
    r.kind = column_text(s, 4);
    {
      const auto *data =
          static_cast<const uint8_t *>(sqlite3_column_blob(s, 5));
      const int bytes = sqlite3_column_bytes(s, 5);
      std::vector<uint8_t> blob;
      if (data && bytes > 0)
        blob.assign(data, data + bytes);
      try {
        r.polygon = core::decode_space_polygon(blob);
      } catch (const std::exception &e) {
        throw std::runtime_error("space " + r.guid + ": " + e.what());
      }
    }
    r.z_floor = sqlite3_column_double(s, 6);
    r.z_ceiling = sqlite3_column_double(s, 7);
    r.area = sqlite3_column_double(s, 8);
    r.perimeter = sqlite3_column_double(s, 9);
    r.volume = sqlite3_column_double(s, 10);
    r.compactness = sqlite3_column_double(s, 11);
    r.room_label = sqlite3_column_type(s, 12) == SQLITE_NULL
                       ? -1
                       : sqlite3_column_int(s, 12);
    r.source = column_text(s, 13);
    r.locked = sqlite3_column_int(s, 14) != 0;
    r.confidence = sqlite3_column_double(s, 15);
    r.metadata = column_text(s, 16);
    return r;
  }

  std::vector<core::SpaceRecord> listSpaces() const {
    std::vector<core::SpaceRecord> out;
    if (!hasSpacesTable)
      return out;
    const std::string sql = std::string("SELECT ") + kSpaceColumns +
                            " FROM spaces ORDER BY storey, id;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "spaces");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back(readSpace(s));
    if (rc != SQLITE_DONE)
      throw std::runtime_error("spaces: " + std::string(sqlite3_errmsg(db)));
    return out;
  }

  std::optional<core::SpaceRecord> spaceByGuid(std::string_view guid) const {
    if (!hasSpacesTable)
      return std::nullopt;
    const std::string sql =
        std::string("SELECT ") + kSpaceColumns + " FROM spaces WHERE guid = ?;";
    sqlite3_stmt *s = prepare_or_throw(db, sql.c_str(), "space_by_guid");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return readSpace(s);
  }

  std::vector<core::SpaceBoundaryRecord> listSpaceBoundaries() const {
    std::vector<core::SpaceBoundaryRecord> out;
    if (!hasSpacesTable)
      return out;
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "SELECT sp.guid, b.wall_guid, b.side, b.s_start, b.s_end "
        "FROM space_boundaries b JOIN spaces sp ON sp.id = b.space_id "
        "ORDER BY sp.storey, sp.id, b.wall_guid, b.side;",
        "space_boundaries");
    StmtGuard guard(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back({column_text(s, 0), column_text(s, 1), column_text(s, 2),
                     sqlite3_column_double(s, 3), sqlite3_column_double(s, 4)});
    if (rc != SQLITE_DONE)
      throw std::runtime_error("space_boundaries: " +
                               std::string(sqlite3_errmsg(db)));
    return out;
  }

  size_t saveSpaces(const std::vector<core::SpaceRecord> &spaces,
                    const std::vector<core::SpaceBoundaryRecord> &boundaries,
                    int runId) {
    checkWritable();
    if (!hasSpacesTable)
      throw std::runtime_error("save_spaces: project schema predates v35");
    static const std::set<std::string, std::less<>> kKinds = {
        "room", "corridor", "open_plan", "unknown"};
    std::set<std::string, std::less<>> guids;
    std::vector<std::vector<uint8_t>> blobs;
    blobs.reserve(spaces.size());
    for (const auto &r : spaces) {
      if (r.guid.empty())
        throw std::invalid_argument("save_spaces: a space of storey " +
                                    std::to_string(r.storey) + " has no guid");
      const auto where = "save_spaces: space " + r.guid;
      if (!kKinds.count(r.kind))
        throw std::invalid_argument(where + " has invalid kind '" + r.kind +
                                    "'");
      if (r.source != "detected" && r.source != "manual" &&
          r.source != "imported")
        throw std::invalid_argument(where + " has invalid source '" + r.source +
                                    "'");
      try {
        blobs.push_back(core::encode_space_polygon(r.polygon));
      } catch (const std::invalid_argument &e) {
        throw std::invalid_argument(where + ": " + e.what());
      }
      if (!guids.insert(r.guid).second)
        throw std::invalid_argument("save_spaces: duplicate guid " + r.guid);
    }
    std::set<std::tuple<std::string, std::string, std::string>> boundaryKeys;
    for (const auto &b : boundaries) {
      const auto where = "save_spaces: boundary of space " + b.space +
                         " on wall " + b.wall + " side " + b.side;
      if (!guids.count(b.space))
        throw std::invalid_argument(where + ": the space is not saved here");
      if (b.wall.empty())
        throw std::invalid_argument(where + " names no wall");
      if (b.side != "a" && b.side != "b")
        throw std::invalid_argument(where + ": side must be a or b");
      if (!(b.s_end >= b.s_start) || !std::isfinite(b.s_start) ||
          !std::isfinite(b.s_end))
        throw std::invalid_argument(where + " has an invalid range (" +
                                    std::to_string(b.s_start) + ".." +
                                    std::to_string(b.s_end) + ")");
      if (!boundaryKeys.emplace(b.space, b.wall, b.side).second)
        throw std::invalid_argument(where + " is given twice");
    }

    Savepoint sp(db, "save_spaces");
    std::set<std::string, std::less<>> lockedGuids;
    {
      sqlite3_stmt *s = prepare_or_throw(
          db, "SELECT guid FROM spaces WHERE locked = 1;", "save_spaces");
      StmtGuard guard(s);
      int rc;
      while ((rc = sqlite3_step(s)) == SQLITE_ROW)
        lockedGuids.insert(column_text(s, 0));
      if (rc != SQLITE_DONE)
        throw std::runtime_error("save_spaces: reading locked rows: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    // Boundaries of unlocked spaces go with them (ON DELETE CASCADE).
    execOrThrow("DELETE FROM spaces WHERE locked = 0;");

    sqlite3_stmt *ins = prepare_or_throw(
        db,
        "INSERT INTO spaces (guid, storey, name, kind, polygon, z_floor, "
        "z_ceiling, area, perimeter, volume, compactness, room_label, source, "
        "locked, confidence, metadata, run_id) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);",
        "save_spaces");
    StmtGuard guard(ins);
    size_t written = 0, skipped = 0;
    for (size_t i = 0; i < spaces.size(); ++i) {
      const auto &r = spaces[i];
      if (lockedGuids.count(r.guid)) {
        ++skipped;
        continue;
      }
      sqlite3_reset(ins);
      sqlite3_clear_bindings(ins);
      bind_text(ins, 1, r.guid);
      sqlite3_bind_int(ins, 2, r.storey);
      bind_text(ins, 3, r.name);
      bind_text(ins, 4, r.kind);
      sqlite3_bind_blob(ins, 5, blobs[i].data(),
                        static_cast<int>(blobs[i].size()), SQLITE_TRANSIENT);
      sqlite3_bind_double(ins, 6, r.z_floor);
      sqlite3_bind_double(ins, 7, r.z_ceiling);
      sqlite3_bind_double(ins, 8, r.area);
      sqlite3_bind_double(ins, 9, r.perimeter);
      sqlite3_bind_double(ins, 10, r.volume);
      sqlite3_bind_double(ins, 11, r.compactness);
      if (r.room_label >= 0)
        sqlite3_bind_int(ins, 12, r.room_label);
      else
        sqlite3_bind_null(ins, 12);
      bind_text(ins, 13, r.source);
      sqlite3_bind_int(ins, 14, r.locked ? 1 : 0);
      sqlite3_bind_double(ins, 15, r.confidence);
      bind_text(ins, 16, r.metadata);
      if (runId >= 0)
        sqlite3_bind_int(ins, 17, runId);
      else
        sqlite3_bind_null(ins, 17);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("save_spaces: insert of space " + r.guid +
                                 " failed: " + sqlite3_errmsg(db));
      ++written;
    }

    sqlite3_stmt *bnd = prepare_or_throw(
        db,
        "INSERT INTO space_boundaries (space_id, wall_guid, side, s_start, "
        "s_end) SELECT id, ?, ?, ?, ? FROM spaces WHERE guid = ?;",
        "save_spaces");
    StmtGuard bndGuard(bnd);
    for (const auto &b : boundaries) {
      if (lockedGuids.count(b.space))
        continue; // the locked space keeps its stored boundaries
      sqlite3_reset(bnd);
      sqlite3_clear_bindings(bnd);
      bind_text(bnd, 1, b.wall);
      bind_text(bnd, 2, b.side);
      sqlite3_bind_double(bnd, 3, b.s_start);
      sqlite3_bind_double(bnd, 4, b.s_end);
      bind_text(bnd, 5, b.space);
      if (sqlite3_step(bnd) != SQLITE_DONE)
        throw std::runtime_error("save_spaces: insert of boundary " + b.space +
                                 " / " + b.wall +
                                 " failed: " + sqlite3_errmsg(db));
    }
    sp.release();
    if (skipped > 0)
      reusex::info("save_spaces: {} space(s) are locked; kept the stored rows",
                   skipped);
    return written;
  }

  bool hasSpaces() const {
    if (!hasSpacesTable)
      return false;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT EXISTS (SELECT 1 FROM spaces);", "has_spaces");
    StmtGuard guard(s);
    return sqlite3_step(s) == SQLITE_ROW && sqlite3_column_int(s, 0) != 0;
  }

  void deleteSpaces() {
    checkWritable();
    if (hasSpacesTable)
      execOrThrow("DELETE FROM space_boundaries; DELETE FROM spaces;");
  }

  // ── Scan helpers (#129) ──────────────────────────────────────────────────

  ProjectDB::ScanRecord createScan(const std::string &source_path,
                                   const std::string &provenance_json) {
    checkWritable();

    // Duplicate-import guard: warn but proceed (a second scan is created by
    // design; the caller can `rux del` the original if that was a mistake).
    {
      const char *dup = "SELECT COUNT(*) FROM scans WHERE source_path = ?;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, dup, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare scan duplicate check: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);
      sqlite3_bind_text(stmt, 1, source_path.c_str(), -1, SQLITE_TRANSIENT);
      int count = 0;
      if (sqlite3_step(stmt) == SQLITE_ROW)
        count = sqlite3_column_int(stmt, 0);
      if (count > 0)
        reusex::warn("importing an already-present source \"{}\" — creating a "
                     "second scan by design; use rux del to remove the "
                     "original if needed",
                     source_path);
    }

    // id_offset = current node-id watermark (0 if no frames yet).
    int id_offset = 0;
    {
      const char *off = "SELECT COALESCE(MAX(node_id), 0) FROM sensor_frames;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, off, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare id_offset query: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);
      if (sqlite3_step(stmt) == SQLITE_ROW)
        id_offset = sqlite3_column_int(stmt, 0);
    }

    {
      const char *ins =
          "INSERT INTO scans (source_path, id_offset, provenance) "
          "VALUES (?, ?, ?);";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare scan insert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);
      sqlite3_bind_text(stmt, 1, source_path.c_str(), -1, SQLITE_TRANSIENT);
      sqlite3_bind_int(stmt, 2, id_offset);
      sqlite3_bind_text(stmt, 3, provenance_json.c_str(), -1, SQLITE_TRANSIENT);
      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to insert scan record: " +
                                 std::string(sqlite3_errmsg(db)));
    }

    ProjectDB::ScanRecord rec;
    rec.id = static_cast<int>(sqlite3_last_insert_rowid(db));
    rec.source_path = source_path;
    rec.id_offset = id_offset;
    rec.provenance_json = provenance_json;

    {
      const char *sel = "SELECT imported_at FROM scans WHERE id = ?;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare imported_at query: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);
      sqlite3_bind_int(stmt, 1, rec.id);
      if (sqlite3_step(stmt) == SQLITE_ROW) {
        if (auto *t =
                reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)))
          rec.imported_at = t;
      }
    }

    return rec;
  }

  std::vector<ProjectDB::ScanRecord> getScans() const {
    if (!tableExists("scans"))
      return {};
    const char *sel = "SELECT id, source_path, imported_at, id_offset, "
                      "provenance FROM scans ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sel, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare scans select: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    std::vector<ProjectDB::ScanRecord> out;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::ScanRecord rec;
      rec.id = sqlite3_column_int(stmt, 0);
      if (auto *t =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
        rec.source_path = t;
      if (auto *t =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2)))
        rec.imported_at = t;
      rec.id_offset = sqlite3_column_int(stmt, 3);
      if (auto *t =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4)))
        rec.provenance_json = t;
      out.push_back(std::move(rec));
    }
    return out;
  }

  // Parse the semantic class id from an instance definition name of the form
  // "SM{class}-{id} (...)". Returns -1 if it doesn't match.
  static int parseSemanticClassFromDef(const std::string &def) {
    // Match a leading "SM" followed by digits and a '-'.
    if (def.size() < 4 || def[0] != 'S' || def[1] != 'M')
      return -1;
    size_t i = 2;
    int value = 0;
    bool any = false;
    while (i < def.size() && def[i] >= '0' && def[i] <= '9') {
      value = value * 10 + (def[i] - '0');
      any = true;
      ++i;
    }
    if (!any || i >= def.size() || def[i] != '-')
      return -1;
    return value;
  }

  // ── MTL parsing helpers ─────────────────────────────────────────────

  /// Parse an MTL file to extract material name -> texture file path mappings
  static std::map<std::string, std::string>
  parseMtlTextures(const std::string &mtlContent) {
    std::map<std::string, std::string> result;
    std::istringstream stream(mtlContent);
    std::string line;
    std::string currentMaterial;

    while (std::getline(stream, line)) {
      // Trim leading whitespace
      size_t start = line.find_first_not_of(" \t");
      if (start == std::string::npos)
        continue;
      line = line.substr(start);

      if (line.rfind("newmtl ", 0) == 0) {
        currentMaterial = line.substr(7);
      } else if (line.rfind("map_Kd ", 0) == 0 && !currentMaterial.empty()) {
        result[currentMaterial] = line.substr(7);
      }
    }
    return result;
  }

  /// Regenerate MTL content with updated texture paths.
  /// Materials whose names are NOT in newPaths have their map_Kd lines
  /// stripped (original paths are from the save-time filesystem and are
  /// no longer valid).
  static std::string
  regenerateMtl(const std::string &originalMtl,
                const std::map<std::string, std::string> &newPaths) {
    std::istringstream stream(originalMtl);
    std::ostringstream output;
    std::string line;
    std::string currentMaterial;

    while (std::getline(stream, line)) {
      // Trim for comparison but write original line
      std::string trimmed = line;
      size_t start = trimmed.find_first_not_of(" \t");
      if (start != std::string::npos)
        trimmed = trimmed.substr(start);

      if (trimmed.rfind("newmtl ", 0) == 0) {
        currentMaterial = trimmed.substr(7);
        output << line << "\n";
      } else if (trimmed.rfind("map_Kd ", 0) == 0 && !currentMaterial.empty()) {
        auto it = newPaths.find(currentMaterial);
        if (it != newPaths.end()) {
          // Replace with path to the blob we extracted to temp dir
          output << "map_Kd " << it->second << "\n";
        }
        // else: strip the map_Kd line — original path is invalid
      } else {
        output << line << "\n";
      }
    }
    return output.str();
  }

  // ── Sensor frame CRUD ─────────────────────────────────────────────

  void saveSensorFrame(int nodeId, const cv::Mat &colorImage) {
    if (colorImage.empty()) {
      throw std::runtime_error("Cannot save empty color image");
    }

    std::vector<unsigned char> jpegBytes;
    if (!cv::imencode(".jpg", colorImage, jpegBytes,
                      {cv::IMWRITE_JPEG_QUALITY, 95})) {
      throw std::runtime_error("Failed to encode color image as JPEG");
    }

    const char *upsert = R"(
      INSERT INTO sensor_frames (node_id, color, width, height)
      VALUES (?, ?, ?, ?)
      ON CONFLICT(node_id) DO UPDATE SET
        color = excluded.color,
        width = excluded.width,
        height = excluded.height;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, jpegBytes.data(),
                      static_cast<int>(jpegBytes.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, colorImage.cols);
    sqlite3_bind_int(stmt, 4, colorImage.rows);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert sensor frame: " +
                               std::string(sqlite3_errmsg(db)));
  }

  std::vector<int> getSensorFrameIds() const {
    const char *sql = "SELECT node_id FROM sensor_frames ORDER BY node_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query sensor frame IDs: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    std::vector<int> ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ids.push_back(sqlite3_column_int(stmt, 0));
    }
    return ids;
  }

  std::vector<std::pair<int, int>> getSensorFrameIdsWithScan() const {
    const char *sql = "SELECT node_id, COALESCE(scan_id, 0) "
                      "FROM sensor_frames ORDER BY node_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query sensor frame ids with scan: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    std::vector<std::pair<int, int>> result;
    while (sqlite3_step(stmt) == SQLITE_ROW)
      result.emplace_back(sqlite3_column_int(stmt, 0),
                          sqlite3_column_int(stmt, 1));
    return result;
  }

  cv::Mat getSensorFrameImage(int nodeId) const {
    const char *sql = "SELECT color FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat(); // Not found

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  }

  void saveSensorFrameFull(int nodeId, const cv::Mat &color,
                           const cv::Mat &depth, const cv::Mat &confidence,
                           const std::array<double, 16> &worldPose,
                           const reusex::core::SensorIntrinsics &intrinsics,
                           double timestamp, int scan_id = -1) {
    if (color.empty())
      throw std::runtime_error("Cannot save empty color image");

    // Encode color as JPEG
    std::vector<unsigned char> jpegBytes;
    if (!cv::imencode(".jpg", color, jpegBytes, {cv::IMWRITE_JPEG_QUALITY, 95}))
      throw std::runtime_error("Failed to encode color image as JPEG");

    // Encode depth as PNG (CV_16UC1 millimeters)
    std::vector<unsigned char> depthBytes;
    if (!depth.empty()) {
      cv::Mat depth16;
      if (depth.type() == CV_32FC1) {
        // Convert meters to millimeters
        cv::Mat mm;
        depth.convertTo(mm, CV_16UC1, 1000.0);
        depth16 = mm;
      } else if (depth.type() == CV_16UC1) {
        depth16 = depth;
      } else {
        depth.convertTo(depth16, CV_16UC1);
      }
      if (!cv::imencode(".png", depth16, depthBytes))
        throw std::runtime_error("Failed to encode depth as PNG");
    }

    // Encode confidence as PNG (CV_8UC1)
    std::vector<unsigned char> confBytes;
    if (!confidence.empty()) {
      cv::Mat conf8;
      if (confidence.type() != CV_8UC1)
        confidence.convertTo(conf8, CV_8UC1);
      else
        conf8 = confidence;
      if (!cv::imencode(".png", conf8, confBytes))
        throw std::runtime_error("Failed to encode confidence as PNG");
    }

    // Camera model as JSON TEXT
    std::string cameraJson = intrinsics.to_json();

    const char *upsert = R"(
      INSERT INTO sensor_frames (node_id, color, depth, confidence, transform, width, height, camera_model, timestamp, scan_id)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
      ON CONFLICT(node_id) DO UPDATE SET
        color = excluded.color,
        depth = excluded.depth,
        confidence = excluded.confidence,
        transform = excluded.transform,
        width = excluded.width,
        height = excluded.height,
        camera_model = excluded.camera_model,
        timestamp = excluded.timestamp,
        scan_id = excluded.scan_id;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare sensor frame upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, jpegBytes.data(),
                      static_cast<int>(jpegBytes.size()), SQLITE_TRANSIENT);
    if (!depthBytes.empty())
      sqlite3_bind_blob(stmt, 3, depthBytes.data(),
                        static_cast<int>(depthBytes.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 3);

    if (!confBytes.empty())
      sqlite3_bind_blob(stmt, 4, confBytes.data(),
                        static_cast<int>(confBytes.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 4);

    // Transform: 16 x float64 = 128 bytes, row-major
    sqlite3_bind_blob(stmt, 5, worldPose.data(),
                      static_cast<int>(worldPose.size() * sizeof(double)),
                      SQLITE_TRANSIENT);

    sqlite3_bind_int(stmt, 6, color.cols);
    sqlite3_bind_int(stmt, 7, color.rows);
    sqlite3_bind_text(stmt, 8, cameraJson.c_str(), -1, SQLITE_TRANSIENT);

    if (timestamp >= 0.0)
      sqlite3_bind_double(stmt, 9, timestamp);
    else
      sqlite3_bind_null(stmt, 9);

    if (scan_id < 0)
      sqlite3_bind_null(stmt, 10);
    else
      sqlite3_bind_int(stmt, 10, scan_id);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert sensor frame: " +
                               std::string(sqlite3_errmsg(db)));
  }

  void updateSensorFramePose(int nodeId,
                             const std::array<double, 16> &worldPose) {
    const char *sql =
        "UPDATE sensor_frames SET transform = ? WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pose update: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    // Transform: 16 x float64 = 128 bytes, row-major (matches
    // saveSensorFrameFull).
    sqlite3_bind_blob(stmt, 1, worldPose.data(),
                      static_cast<int>(worldPose.size() * sizeof(double)),
                      SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, nodeId);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to update sensor frame pose: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) == 0)
      throw std::runtime_error(
          "update_sensor_frame_pose: no sensor frame with node_id " +
          std::to_string(nodeId));
  }

  cv::Mat getSensorFrameDepth(int nodeId) const {
    const char *sql = "SELECT depth FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare depth query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED); // CV_16UC1
  }

  cv::Mat getSensorFrameConfidence(int nodeId) const {
    const char *sql = "SELECT confidence FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare confidence query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED); // CV_8UC1
  }

  /// Read the raw `transform` blob of a sensor frame.
  ///
  /// Shared by getSensorFramePose (which falls back to identity) and
  /// hasSensorFramePose (which reports the failure instead) so the two can
  /// never disagree about what "there is a stored pose" means.
  ///
  /// @return false when the row is missing, the blob is NULL, or the blob is
  ///         not exactly 16 doubles. `out` is untouched in that case.
  bool readSensorFramePoseBlob(int nodeId, std::array<double, 16> &out) const {
    const char *sql = "SELECT transform FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pose query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return false;

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize != static_cast<int>(16 * sizeof(double)))
      return false;

    std::memcpy(out.data(), blob, 16 * sizeof(double));
    return true;
  }

  std::array<double, 16> getSensorFramePose(int nodeId) const {
    // Identity fallback is load-bearing for existing callers; ask
    // hasSensorFramePose() when the difference matters (#330).
    std::array<double, 16> pose = {1, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 1};
    readSensorFramePoseBlob(nodeId, pose);
    return pose;
  }

  /// Is the stored `transform` a pose we can trust as a camera placement?
  ///
  /// A genuinely-stored identity IS a valid pose (a scan may legitimately put
  /// its first frame at the origin), so this rejects only what cannot be a
  /// rigid transform at all.
  bool hasSensorFramePose(int nodeId) const {
    std::array<double, 16> m{};
    // Missing row / NULL / short blob: nothing was ever written here.
    if (!readSensorFramePoseBlob(nodeId, m))
      return false;

    // NaN/Inf survive a round-trip through the BLOB and would poison every
    // downstream matrix product without ever throwing.
    for (double v : m)
      if (!std::isfinite(v))
        return false;

    // Row-major storage, so the bottom row is m[12..15]. An affine 4x4 that
    // is not [0 0 0 1] there is not a pose — it is a garbled or
    // differently-laid-out blob.
    constexpr double kRowTol = 1e-6;
    if (std::abs(m[12]) > kRowTol || std::abs(m[13]) > kRowTol ||
        std::abs(m[14]) > kRowTol || std::abs(m[15] - 1.0) > kRowTol)
      return false;

    // Non-degenerate rotation block. This is the check that catches the
    // all-zeros matrix an importer writes for a frame it has no pose for:
    // that passes the bottom-row test only when m[15] happens to be 1, and
    // fails here because det(R) == 0. A proper rotation has det == +1; we
    // only demand a magnitude away from zero so a scaled or slightly
    // non-orthonormal stored pose is still accepted.
    constexpr double kDetEps = 1e-9;
    const double det = m[0] * (m[5] * m[10] - m[6] * m[9]) -
                       m[1] * (m[4] * m[10] - m[6] * m[8]) +
                       m[2] * (m[4] * m[9] - m[5] * m[8]);
    return std::abs(det) > kDetEps;
  }

  void setInitialPose(int nodeId, const std::array<double, 16> &pose) {
    checkWritable();
    if (!hasInitialTransform)
      throw std::runtime_error("set_initial_pose: project schema predates "
                               "v28; open it read-write once to migrate");
    if (!core::pose_graph::is_rigid_transform(pose))
      throw std::invalid_argument("set_initial_pose: frame " +
                                  std::to_string(nodeId) +
                                  ": not a rigid transform");
    sqlite3_stmt *s = prepare_or_throw(
        db, "UPDATE sensor_frames SET initial_transform = ? WHERE node_id = ?;",
        "set_initial_pose");
    StmtGuard guard(s);
    sqlite3_bind_blob(s, 1, pose.data(), 16 * sizeof(double), SQLITE_TRANSIENT);
    sqlite3_bind_int(s, 2, nodeId);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_initial_pose: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) == 0)
      throw std::runtime_error("set_initial_pose: no sensor frame with "
                               "node_id " +
                               std::to_string(nodeId));
  }

  std::optional<std::array<double, 16>> initialPose(int nodeId) const {
    if (!hasInitialTransform)
      return std::nullopt;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT initial_transform FROM sensor_frames WHERE node_id = ?;",
        "initial_pose");
    StmtGuard guard(s);
    sqlite3_bind_int(s, 1, nodeId);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    auto pose = optionalBlob<16>(s, 0);
    if (!pose || !core::pose_graph::is_rigid_transform(*pose))
      return std::nullopt;
    return pose;
  }

  std::optional<std::string> getSetting(std::string_view key) const {
    if (!hasSettingsTable)
      return std::nullopt;
    sqlite3_stmt *s = prepare_or_throw(
        db, "SELECT value FROM project_settings WHERE key = ?;", "setting");
    StmtGuard guard(s);
    const std::string k(key);
    sqlite3_bind_text(s, 1, k.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return textOr(s, 0, "");
  }

  void putSetting(std::string_view key, std::string_view value) {
    // Write-once: every deterministic row GUID is derived from it, so a
    // change would orphan the identity of every stored solid-model row.
    if (key == "project_guid")
      throw std::invalid_argument(
          "set_setting: 'project_guid' is write-once (minted by the v30 "
          "migration)");
    checkWritable();
    if (!hasSettingsTable)
      throw std::runtime_error("set_setting: project schema predates v28");
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO project_settings (key, value) VALUES (?1, ?2) "
        "ON CONFLICT (key) DO UPDATE SET value = excluded.value;",
        "set_setting");
    StmtGuard guard(s);
    const std::string k(key), v(value);
    sqlite3_bind_text(s, 1, k.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(s, 2, v.c_str(), -1, SQLITE_TRANSIENT);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("set_setting: " +
                               std::string(sqlite3_errmsg(db)));
  }

  reusex::core::SensorIntrinsics getSensorFrameIntrinsics(int nodeId) const {
    const char *sql =
        "SELECT camera_model FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare intrinsics query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return {};

    const char *text =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    if (!text)
      return {};

    return reusex::core::SensorIntrinsics::from_json(text);
  }

  bool hasSensorFrame(int nodeId) const {
    const char *sql = "SELECT 1 FROM sensor_frames WHERE node_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  double getSensorFrameTimestamp(int nodeId) const {
    const char *sql = "SELECT timestamp FROM sensor_frames WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return -1.0;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return -1.0;
    if (sqlite3_column_type(stmt, 0) == SQLITE_NULL)
      return -1.0;
    return sqlite3_column_double(stmt, 0);
  }

  int nearestSensorFrameByTimestamp(double timestamp) const {
    const char *sql = R"(
      SELECT node_id FROM sensor_frames
      WHERE timestamp IS NOT NULL
      ORDER BY ABS(timestamp - ?) LIMIT 1;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return -1;
    StmtGuard guard(stmt);
    sqlite3_bind_double(stmt, 1, timestamp);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return -1;
    return sqlite3_column_int(stmt, 0);
  }

  // ── Panoramic image CRUD ────────────────────────────────────────

  void savePanoramicImage(const std::string &filename,
                          const std::vector<uint8_t> &jpegData,
                          double timestamp, int nodeId) {
    const char *upsert = R"(
      INSERT INTO panoramic_images (filename, image, timestamp, node_id)
      VALUES (?, ?, ?, ?);
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panoramic image insert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, filename.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, jpegData.data(),
                      static_cast<int>(jpegData.size()), SQLITE_TRANSIENT);

    if (timestamp >= 0.0)
      sqlite3_bind_double(stmt, 3, timestamp);
    else
      sqlite3_bind_null(stmt, 3);

    if (nodeId >= 0)
      sqlite3_bind_int(stmt, 4, nodeId);
    else
      sqlite3_bind_null(stmt, 4);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to insert panoramic image: " +
                               std::string(sqlite3_errmsg(db)));
  }

  cv::Mat getPanoramicImageById(int id) const {
    const char *sql = "SELECT image FROM panoramic_images WHERE id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panoramic image query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, id);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  }

  cv::Mat getPanoramicImageByFilename(std::string_view filename) const {
    const char *sql = "SELECT image FROM panoramic_images WHERE filename = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panoramic image query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, filename.data(),
                      static_cast<int>(filename.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    return cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
  }

  bool hasPanoramicImage(std::string_view filename) const {
    const char *sql =
        "SELECT 1 FROM panoramic_images WHERE filename = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, filename.data(),
                      static_cast<int>(filename.size()), SQLITE_TRANSIENT);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  void deletePanoramicImageById(int id) {
    const char *sql = "DELETE FROM panoramic_images WHERE id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panoramic image delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, id);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete panoramic image");
  }

  void deletePanoramicImageByFilename(std::string_view filename) {
    const char *sql = "DELETE FROM panoramic_images WHERE filename = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panoramic image delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, filename.data(),
                      static_cast<int>(filename.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete panoramic image");
  }

  std::vector<ProjectDB::PanoramicImage> listPanoramicImages() const {
    // The v11 pose columns only exist once migrateToV11() has run, and
    // migrations are skipped on read-only opens (see the Impl constructor).
    // Probe the schema instead of unconditionally selecting the new columns:
    // a v10 project opened read-only must still list its panoramas.
    const bool hasV11 = columnExists("panoramic_images", "pose");

    const char *sql = hasV11 ? R"(
      SELECT id, filename, timestamp, node_id,
             pose, pose_source, align_inliers, align_rms
      FROM panoramic_images ORDER BY filename;
    )"
                             : R"(
      SELECT id, filename, timestamp, node_id
      FROM panoramic_images ORDER BY filename;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query panoramic images: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    std::vector<ProjectDB::PanoramicImage> images;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::PanoramicImage img;
      img.id = sqlite3_column_int(stmt, 0);
      const char *fn =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      img.filename = fn ? fn : "";
      img.timestamp = (sqlite3_column_type(stmt, 2) == SQLITE_NULL)
                          ? -1.0
                          : sqlite3_column_double(stmt, 2);
      img.node_id = (sqlite3_column_type(stmt, 3) == SQLITE_NULL)
                        ? -1
                        : sqlite3_column_int(stmt, 3);
      // Refined alignment pose (v11+); absent -> keep identity + "timestamp"
      // defaults from PanoramicImage, i.e. has_pose stays false.
      if (hasV11) {
        const void *poseBlob = sqlite3_column_blob(stmt, 4);
        const int poseBytes = sqlite3_column_bytes(stmt, 4);
        if (poseBlob && poseBytes == static_cast<int>(16 * sizeof(double))) {
          std::memcpy(img.pose.data(), poseBlob, 16 * sizeof(double));
          img.has_pose = true;
        }
        if (sqlite3_column_type(stmt, 5) != SQLITE_NULL) {
          const char *src =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
          if (src)
            img.pose_source = src;
        }
        if (sqlite3_column_type(stmt, 6) != SQLITE_NULL)
          img.align_inliers = sqlite3_column_int(stmt, 6);
        if (sqlite3_column_type(stmt, 7) != SQLITE_NULL)
          img.align_rms = sqlite3_column_double(stmt, 7);
      }
      images.push_back(std::move(img));
    }
    return images;
  }

  void savePanoramaPose(int id, const std::array<double, 16> &pose, int inliers,
                        double rms) {
    const char *sql = R"(
      UPDATE panoramic_images
      SET pose = ?, pose_source = 'aligned', align_inliers = ?, align_rms = ?
      WHERE id = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panorama pose update: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_blob(stmt, 1, pose.data(),
                      static_cast<int>(pose.size() * sizeof(double)),
                      SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 2, inliers);
    sqlite3_bind_double(stmt, 3, rms);
    sqlite3_bind_int(stmt, 4, id);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to update panorama pose: " +
                               std::string(sqlite3_errmsg(db)));
  }

  // ── Panorama segmentation CRUD (equirect label maps) ───────────────

  bool hasPanoramaSegmentation(int panoId) const {
    const char *sql =
        "SELECT 1 FROM panorama_segmentation WHERE pano_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, panoId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  cv::Mat getPanoramaSegmentation(int panoId) const {
    // v11 table; a pre-v11 project opened read-only never gets migrated, so
    // treat a missing table as "no segmentation" rather than an error.
    if (!tableExists("panorama_segmentation"))
      return cv::Mat();
    const char *sql =
        "SELECT label_image FROM panorama_segmentation WHERE pano_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panorama seg query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, panoId);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    if (!blob || blobSize == 0)
      return cv::Mat();
    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    cv::Mat labels16U = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
    if (labels16U.empty())
      return cv::Mat();
    // Storage CV_16U (0 = bg, label+1) -> API CV_32S (-1 = background).
    return reusex::core::storage_mat_to_api(labels16U);
  }

  void savePanoramaSegmentation(int panoId, const cv::Mat &labels) {
    if (labels.empty())
      throw std::runtime_error("Cannot save empty panorama segmentation");
    // API CV_32S (-1 = background) -> storage CV_16U (0 = bg, label+1).
    cv::Mat toSave = reusex::core::api_mat_to_storage(labels);
    std::vector<unsigned char> pngBytes;
    if (!cv::imencode(".png", toSave, pngBytes))
      throw std::runtime_error("Failed to encode panorama segmentation as PNG");

    const char *upsert = R"(
      INSERT INTO panorama_segmentation (pano_id, label_image)
      VALUES (?, ?)
      ON CONFLICT(pano_id) DO UPDATE SET label_image = excluded.label_image;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare panorama seg upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, panoId);
    sqlite3_bind_blob(stmt, 2, pngBytes.data(),
                      static_cast<int>(pngBytes.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert panorama segmentation: " +
                               std::string(sqlite3_errmsg(db)));
  }

  int panoramicImageCount() const {
    const char *sql = "SELECT COUNT(*) FROM panoramic_images;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return 0;
    StmtGuard guard(stmt);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0);
    return 0;
  }

  // ── Segmentation image CRUD ───────────────────────────────────────

  std::vector<int> getSegmentationImageIds() const {
    const char *sql =
        "SELECT node_id FROM segmentation_images ORDER BY node_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query segmentation image IDs: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    std::vector<int> ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ids.push_back(sqlite3_column_int(stmt, 0));
    }
    return ids;
  }

  bool hasSegmentationImage(int nodeId) const {
    const char *sql =
        "SELECT 1 FROM segmentation_images WHERE node_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  cv::Mat getSegmentationImage(int nodeId) const {
    const char *sql =
        "SELECT label_image FROM segmentation_images WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare segmentation query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat(); // Not found

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    cv::Mat labels16U = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);

    if (labels16U.empty())
      return cv::Mat();

    // Storage CV_16U (0 = bg, label+1) -> API CV_32S (-1 = background).
    return reusex::core::storage_mat_to_api(labels16U);
  }

  void saveSegmentationImage(int nodeId, const cv::Mat &labels) {
    if (labels.empty()) {
      throw std::runtime_error("Cannot save empty segmentation labels");
    }

    // API CV_32S (-1 = background) -> storage CV_16U (0 = bg, label+1).
    // Throws std::out_of_range on labels >= 65535 instead of wrapping.
    cv::Mat toSave = reusex::core::api_mat_to_storage(labels);

    std::vector<unsigned char> pngBytes;
    if (!cv::imencode(".png", toSave, pngBytes)) {
      throw std::runtime_error("Failed to encode segmentation labels as PNG");
    }

    const char *upsert = R"(
      INSERT INTO segmentation_images (node_id, label_image)
      VALUES (?, ?)
      ON CONFLICT(node_id) DO UPDATE SET label_image = excluded.label_image;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare segmentation upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, pngBytes.data(),
                      static_cast<int>(pngBytes.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert segmentation image: " +
                               std::string(sqlite3_errmsg(db)));
  }

  void saveSegmentationImages(const std::vector<int> &nodeIds,
                              const std::vector<cv::Mat> &labels) {
    if (nodeIds.size() != labels.size()) {
      throw std::runtime_error(
          "nodeIds and labels vectors must have same size");
    }

    execOrThrow("BEGIN TRANSACTION;");
    try {
      for (size_t i = 0; i < nodeIds.size(); ++i) {
        saveSegmentationImage(nodeIds[i], labels[i]);
      }
      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  // ── Glass confidence image operations ─────────────────────────────

  bool hasGlassConfidenceImage(int nodeId) const {
    const char *sql =
        "SELECT 1 FROM glass_confidence_images WHERE node_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  cv::Mat getGlassConfidenceImage(int nodeId) const {
    const char *sql = "SELECT confidence_image FROM glass_confidence_images "
                      "WHERE node_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare glass confidence query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      return cv::Mat();

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);
    cv::Mat encoded(1, blobSize, CV_8UC1, const_cast<void *>(blob));
    cv::Mat result = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
    return result; // CV_8U: 255=trust, 0=suppress
  }

  std::vector<int> getGlassConfidenceImageIds() const {
    const char *sql =
        "SELECT node_id FROM glass_confidence_images ORDER BY node_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query glass confidence image IDs: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    std::vector<int> ids;
    while (sqlite3_step(stmt) == SQLITE_ROW)
      ids.push_back(sqlite3_column_int(stmt, 0));
    return ids;
  }

  void saveGlassConfidenceImage(int nodeId, const cv::Mat &confidence) {
    if (confidence.empty())
      throw std::runtime_error("Cannot save empty glass confidence image");

    cv::Mat toSave = confidence;
    if (toSave.type() != CV_8U)
      toSave.convertTo(toSave, CV_8U, 255.0);

    std::vector<unsigned char> pngBytes;
    if (!cv::imencode(".png", toSave, pngBytes))
      throw std::runtime_error(
          "Failed to encode glass confidence image as PNG");

    const char *upsert = R"(
      INSERT INTO glass_confidence_images (node_id, confidence_image)
      VALUES (?, ?)
      ON CONFLICT(node_id) DO UPDATE SET confidence_image = excluded.confidence_image;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare glass confidence upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, nodeId);
    sqlite3_bind_blob(stmt, 2, pngBytes.data(),
                      static_cast<int>(pngBytes.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert glass confidence image: " +
                               std::string(sqlite3_errmsg(db)));
  }

  // ── Legacy passport tables ─────────────────────────────────────────

  void createTables() {
    const char *schema = R"(
      CREATE TABLE IF NOT EXISTS projects (
        id                   TEXT PRIMARY KEY,
        name                 TEXT,
        building_address     TEXT,
        year_of_construction INTEGER,
        survey_date          TEXT,
        survey_organisation  TEXT,
        notes                TEXT
      );

      CREATE TABLE IF NOT EXISTS property_definitions (
        id            TEXT PRIMARY KEY,
        leksikon_guid TEXT UNIQUE,
        name_en       TEXT NOT NULL,
        category      TEXT,
        data_type     TEXT,
        unit          TEXT,
        is_array      INTEGER DEFAULT 0
      );

      CREATE TABLE IF NOT EXISTS material_passports (
        id             TEXT PRIMARY KEY,
        project_id     TEXT REFERENCES projects(id),
        document_guid  TEXT UNIQUE,
        created_at     TEXT,
        revised_at     TEXT,
        version_number TEXT,
        version_date   TEXT
      );

      CREATE TABLE IF NOT EXISTS passport_property_values (
        id            TEXT PRIMARY KEY,
        passport_id   TEXT REFERENCES material_passports(id),
        property_id   TEXT REFERENCES property_definitions(id),
        leksikon_guid TEXT,
        sort_order    INTEGER,
        value         BLOB,
        UNIQUE(passport_id, leksikon_guid)
      );

      CREATE TABLE IF NOT EXISTS passport_log (
        id          TEXT PRIMARY KEY,
        passport_id TEXT REFERENCES material_passports(id),
        entry_type  TEXT,
        target_guid TEXT,
        edited_by   TEXT,
        edited_at   TEXT,
        old_value   TEXT,
        new_value   TEXT
      );

      CREATE TABLE IF NOT EXISTS material_property_definitions (
        id          TEXT PRIMARY KEY,
        name        TEXT NOT NULL,
        type        TEXT NOT NULL DEFAULT 'text',
        options     TEXT,
        sort_order  INTEGER NOT NULL DEFAULT 0,
        width       INTEGER NOT NULL DEFAULT 200,
        created_at  TEXT NOT NULL DEFAULT (datetime('now'))
      );

      CREATE TABLE IF NOT EXISTS material_thumbnails (
        material_guid TEXT PRIMARY KEY
          REFERENCES material_passports(document_guid) ON DELETE CASCADE,
        blob          BLOB NOT NULL,
        mime_type     TEXT NOT NULL DEFAULT 'image/jpeg',
        created_at    TEXT NOT NULL DEFAULT (datetime('now'))
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg;
      sqlite3_free(errMsg);
      throw std::runtime_error("Failed to create schema: " + error);
    }

    reusex::trace("Database schema created successfully");
  }

  void validateSchema() const {
    // Check for required project database tables
    const char *tables[] = {"projects", "property_definitions",
                            "material_passports", "passport_property_values",
                            "passport_log"};

    for (const char *table : tables) {
      std::string query =
          "SELECT name FROM sqlite_master WHERE type='table' AND name='" +
          std::string(table) + "';";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Schema validation failed: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      bool found = (sqlite3_step(stmt) == SQLITE_ROW);
      sqlite3_finalize(stmt);

      if (!found) {
        throw std::runtime_error("Required table '" + std::string(table) +
                                 "' not found in database");
      }
    }
    reusex::trace("Database schema validation passed");
  }

  // ── Point cloud CRUD ───────────────────────────────────────────────

  void savePointCloudMeta(std::string_view name, const char *pointType,
                          size_t pointCount, int pointStep, uint32_t width,
                          uint32_t height, const std::vector<uint8_t> &data,
                          std::string_view stage, std::string_view paramsJson) {
    // A savepoint rather than a plain transaction: atomic on its own exactly
    // as before, and it nests inside a ProjectDB::Transaction so a caller can
    // save a cloud together with other writes (core::apply_mask_selection).
    Savepoint sp(db, "save_point_cloud");
    {
      // Upsert point_clouds row
      const char *upsert = R"(
        INSERT INTO point_clouds (name, point_type, point_count, point_step, width, height, stage, parameters)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(name) DO UPDATE SET
          point_type = excluded.point_type,
          point_count = excluded.point_count,
          point_step = excluded.point_step,
          width = excluded.width,
          height = excluded.height,
          created_at = datetime('now'),
          stage = excluded.stage,
          parameters = excluded.parameters,
          tile_index = NULL;
      )";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare point cloud upsert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 2, pointType, -1, SQLITE_STATIC);
      sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(pointCount));
      sqlite3_bind_int(stmt, 4, pointStep);
      sqlite3_bind_int(stmt, 5, static_cast<int>(width));
      sqlite3_bind_int(stmt, 6, static_cast<int>(height));
      if (!stage.empty())
        sqlite3_bind_text(stmt, 7, stage.data(), static_cast<int>(stage.size()),
                          SQLITE_TRANSIENT);
      else
        sqlite3_bind_null(stmt, 7);
      if (!paramsJson.empty())
        sqlite3_bind_text(stmt, 8, paramsJson.data(),
                          static_cast<int>(paramsJson.size()),
                          SQLITE_TRANSIENT);
      else
        sqlite3_bind_null(stmt, 8);

      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("Failed to upsert point cloud metadata: " +
                                 std::string(sqlite3_errmsg(db)));

      // Get the cloud_id (might be new or existing)
      int cloudId = getCloudId(name);

      // Replace any existing chunks for this cloud, then insert the new
      // payload as one or more rows. Chunking keeps each row under SQLite's
      // per-blob size limit (max 2 GB; default 1 GB).
      {
        const char *deleteSql =
            "DELETE FROM point_cloud_data WHERE cloud_id = ?;";
        sqlite3_stmt *delStmt;
        if (sqlite3_prepare_v2(db, deleteSql, -1, &delStmt, nullptr) !=
            SQLITE_OK)
          throw std::runtime_error("Failed to prepare data delete: " +
                                   std::string(sqlite3_errmsg(db)));
        StmtGuard delGuard(delStmt);
        sqlite3_bind_int(delStmt, 1, cloudId);
        if (sqlite3_step(delStmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to clear point cloud data: " +
                                   std::string(sqlite3_errmsg(db)));
      }

      const char *dataInsert =
          "INSERT INTO point_cloud_data (cloud_id, chunk_index, data) "
          "VALUES (?, ?, ?);";

      sqlite3_stmt *dstmt;
      if (sqlite3_prepare_v2(db, dataInsert, -1, &dstmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare data insert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard dguard(dstmt);

      const size_t total = data.size();
      const size_t chunkBytes = POINT_CLOUD_CHUNK_BYTES;
      size_t offset = 0;
      int chunkIndex = 0;
      // Ensure at least one chunk row even for empty payloads (the schema
      // requires NOT NULL, so use a zero-length blob).
      do {
        size_t len = std::min(chunkBytes, total - offset);
        sqlite3_reset(dstmt);
        sqlite3_clear_bindings(dstmt);

        if (sqlite3_bind_int(dstmt, 1, cloudId) != SQLITE_OK)
          throw std::runtime_error("Failed to bind cloud_id: " +
                                   std::string(sqlite3_errmsg(db)));
        if (sqlite3_bind_int(dstmt, 2, chunkIndex) != SQLITE_OK)
          throw std::runtime_error("Failed to bind chunk_index: " +
                                   std::string(sqlite3_errmsg(db)));
        static constexpr uint8_t emptyByte = 0;
        const void *blobPtr =
            len ? static_cast<const void *>(data.data() + offset)
                : static_cast<const void *>(&emptyByte);
        int rc = sqlite3_bind_blob64(
            dstmt, 3, blobPtr, static_cast<sqlite3_uint64>(len), SQLITE_STATIC);
        if (rc != SQLITE_OK)
          throw std::runtime_error(
              "Failed to bind point cloud chunk " + std::to_string(chunkIndex) +
              " (" + std::to_string(len) +
              " bytes): " + std::string(sqlite3_errmsg(db)));

        if (sqlite3_step(dstmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to insert point cloud chunk " +
                                   std::to_string(chunkIndex) + ": " +
                                   std::string(sqlite3_errmsg(db)));

        offset += len;
        ++chunkIndex;
      } while (offset < total);
    }
    sp.release();
  }

  int getCloudId(std::string_view name) const {
    const char *sql = "SELECT id FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query cloud ID");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " + std::string(name));
    return sqlite3_column_int(stmt, 0);
  }

  // Generic point cloud loader: returns raw data, point_type, width, height
  struct CloudMeta {
    std::string point_type;
    uint32_t width;
    uint32_t height;
    std::vector<uint8_t> data;
  };

  CloudMeta loadCloudRaw(std::string_view name) const {
    // Read point cloud metadata.
    const char *metaSql = "SELECT id, point_type, width, height FROM "
                          "point_clouds WHERE name = ?;";
    sqlite3_stmt *mstmt;
    if (sqlite3_prepare_v2(db, metaSql, -1, &mstmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare cloud load: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard mguard(mstmt);
    sqlite3_bind_text(mstmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);

    if (sqlite3_step(mstmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " + std::string(name));

    CloudMeta meta;
    int cloudId = sqlite3_column_int(mstmt, 0);
    meta.point_type =
        reinterpret_cast<const char *>(sqlite3_column_text(mstmt, 1));
    meta.width = static_cast<uint32_t>(sqlite3_column_int(mstmt, 2));
    meta.height = static_cast<uint32_t>(sqlite3_column_int(mstmt, 3));

    // Read all chunks in order and concatenate.
    const char *chunkSql = "SELECT data FROM point_cloud_data "
                           "WHERE cloud_id = ? ORDER BY chunk_index;";
    sqlite3_stmt *cstmt;
    if (sqlite3_prepare_v2(db, chunkSql, -1, &cstmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare cloud chunk load: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard cguard(cstmt);
    sqlite3_bind_int(cstmt, 1, cloudId);

    bool anyChunks = false;
    int rc;
    while ((rc = sqlite3_step(cstmt)) == SQLITE_ROW) {
      anyChunks = true;
      const void *blob = sqlite3_column_blob(cstmt, 0);
      int blobSize = sqlite3_column_bytes(cstmt, 0);
      if (blobSize < 0)
        throw std::runtime_error("Negative chunk size for point cloud: " +
                                 std::string(name));
      if (blobSize > 0 && blob) {
        const auto *src = static_cast<const uint8_t *>(blob);
        meta.data.insert(meta.data.end(), src, src + blobSize);
      }
    }
    if (rc != SQLITE_DONE)
      throw std::runtime_error("Failed to read point cloud chunks: " +
                               std::string(sqlite3_errmsg(db)));

    if (!anyChunks)
      throw std::runtime_error("Point cloud has no data rows: " +
                               std::string(name));

    return meta;
  }

  // Bytes per stored record for a given point_type, or 0 when the type is not
  // one this build knows how to store.
  static uint32_t stepForPointType(std::string_view type) {
    if (type == "PointXYZRGB")
      return XYZRGB_STEP;
    if (type == "Normal")
      return NORMAL_STEP;
    if (type == "Label")
      return LABEL_STEP;
    if (type == "PointXYZ")
      return XYZ_STEP;
    return 0;
  }

  // Partial read: only the chunks the requested byte range overlaps are
  // touched, and only the overlapping sub-range of each. See the doc comment
  // on ProjectDB::point_cloud_page().
  ProjectDB::CloudPage loadCloudPage(std::string_view name, uint64_t offset,
                                     uint64_t limit) const {
    const char *metaSql = "SELECT id, point_type, point_count, point_step FROM "
                          "point_clouds WHERE name = ?;";
    sqlite3_stmt *mstmt;
    if (sqlite3_prepare_v2(db, metaSql, -1, &mstmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare cloud page load: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard mguard(mstmt);
    sqlite3_bind_text(mstmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(mstmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " + std::string(name));

    ProjectDB::CloudPage page;
    const int cloudId = sqlite3_column_int(mstmt, 0);
    page.point_type =
        reinterpret_cast<const char *>(sqlite3_column_text(mstmt, 1));
    const auto storedCount =
        static_cast<uint64_t>(sqlite3_column_int64(mstmt, 2));
    const auto storedStep =
        static_cast<uint32_t>(sqlite3_column_int64(mstmt, 3));

    const uint32_t knownStep = stepForPointType(page.point_type);
    if (knownStep == 0)
      throw std::runtime_error("Point cloud '" + std::string(name) +
                               "' has unsupported point type '" +
                               page.point_type + "'");
    // A stale or zero point_step (pre-v7 rows, a truncated write) must not be
    // trusted silently: the type's step is the authority, and a disagreement
    // is reported rather than papered over (STANDARDS §5).
    if (storedStep != 0 && storedStep != knownStep)
      throw std::runtime_error(
          "Point cloud '" + std::string(name) + "' declares point_step " +
          std::to_string(storedStep) + " but type '" + page.point_type +
          "' stores " + std::to_string(knownStep) + " bytes per record");
    if (storedStep == 0)
      warn("Point cloud '{}' has point_step 0 in its metadata row; falling "
           "back to {} bytes for type '{}'",
           name, knownStep, page.point_type);
    page.point_step = knownStep;

    // Chunk map. length() on a blob column reads the header, not the payload.
    struct ChunkRef {
      sqlite3_int64 rowid;
      uint64_t begin; ///< Global byte offset of this chunk's first byte.
      uint64_t size;
    };
    std::vector<ChunkRef> chunks;
    uint64_t totalBytes = 0;
    {
      const char *chunkSql =
          "SELECT rowid, chunk_index, length(data) FROM point_cloud_data "
          "WHERE cloud_id = ? ORDER BY chunk_index;";
      sqlite3_stmt *cstmt;
      if (sqlite3_prepare_v2(db, chunkSql, -1, &cstmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare cloud chunk map: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard cguard(cstmt);
      sqlite3_bind_int(cstmt, 1, cloudId);

      int rc;
      while ((rc = sqlite3_step(cstmt)) == SQLITE_ROW) {
        const auto len = sqlite3_column_int64(cstmt, 2);
        if (len < 0)
          throw std::runtime_error("Negative chunk size for point cloud: " +
                                   std::string(name));
        chunks.push_back({sqlite3_column_int64(cstmt, 0), totalBytes,
                          static_cast<uint64_t>(len)});
        totalBytes += static_cast<uint64_t>(len);
      }
      if (rc != SQLITE_DONE)
        throw std::runtime_error("Failed to read point cloud chunk map: " +
                                 std::string(sqlite3_errmsg(db)));
    }
    if (chunks.empty())
      throw std::runtime_error("Point cloud has no data rows: " +
                               std::string(name));

    // The bytes on disk are the ground truth; point_count is a cached number.
    const uint64_t byteCount = totalBytes / page.point_step;
    if (byteCount != storedCount)
      warn("Point cloud '{}' metadata says {} points but its {} stored bytes "
           "hold {} records of {} bytes; using the byte count",
           name, storedCount, totalBytes, byteCount, page.point_step);
    page.total = byteCount;

    page.offset = std::min(offset, page.total);
    page.count = std::min(limit, page.total - page.offset);
    if (page.count == 0)
      return page;

    const uint64_t rangeBegin = page.offset * page.point_step;
    const uint64_t rangeEnd = rangeBegin + page.count * page.point_step;
    page.data.resize(static_cast<size_t>(rangeEnd - rangeBegin));

    // A record can straddle a chunk boundary (256 MiB is not a multiple of
    // 12), so copy raw byte ranges rather than whole records per chunk.
    for (const auto &chunk : chunks) {
      if (chunk.size == 0)
        continue;
      const uint64_t chunkEnd = chunk.begin + chunk.size;
      if (chunkEnd <= rangeBegin || chunk.begin >= rangeEnd)
        continue;

      const uint64_t copyBegin = std::max(chunk.begin, rangeBegin);
      const uint64_t copyEnd = std::min(chunkEnd, rangeEnd);

      sqlite3_blob *blob = nullptr;
      if (sqlite3_blob_open(db, "main", "point_cloud_data", "data", chunk.rowid,
                            /*flags=*/0, &blob) != SQLITE_OK)
        throw std::runtime_error("Failed to open point cloud chunk blob: " +
                                 std::string(sqlite3_errmsg(db)));
      BlobGuard bguard(blob);
      if (sqlite3_blob_read(blob, page.data.data() + (copyBegin - rangeBegin),
                            static_cast<int>(copyEnd - copyBegin),
                            static_cast<int>(copyBegin - chunk.begin)) !=
          SQLITE_OK)
        throw std::runtime_error("Failed to read point cloud chunk blob: " +
                                 std::string(sqlite3_errmsg(db)));
    }

    return page;
  }

  bool hasPointCloud(std::string_view name) const {
    const char *sql = "SELECT COUNT(*) FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  void deletePointCloud(std::string_view name) {
    // CASCADE will remove point_cloud_data and label_definitions rows
    const char *sql = "DELETE FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete point cloud: " +
                               std::string(sqlite3_errmsg(db)));
  }

  std::vector<std::string> listPointClouds() const {
    const char *sql = "SELECT name FROM point_clouds ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list point clouds");
    StmtGuard guard(stmt);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  std::string getPointCloudType(std::string_view name) const {
    const char *sql = "SELECT point_type FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query point cloud type");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " + std::string(name));
    return reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
  }

  /// Extract "storage_order" from the cloud's parameters JSON.
  /// Returns "" when the key is absent, the cloud is missing, or JSON is
  /// unparseable — all of which mean "treat as sequential".
  std::string getCloudStorageOrder(std::string_view name) const {
    const char *sql = "SELECT parameters FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return "";
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      return "";
    const char *params_text =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    if (!params_text || !*params_text)
      return "";
    try {
      auto j = nlohmann::json::parse(params_text);
      if (j.contains("storage_order"))
        return j["storage_order"].get<std::string>();
    } catch (...) {
    }
    return "";
  }

  void saveTileIndex(std::string_view name, const std::vector<uint8_t> &blob) {
    const char *sql = "UPDATE point_clouds SET tile_index = ? WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare tile index update: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    if (blob.empty())
      sqlite3_bind_null(stmt, 1);
    else
      sqlite3_bind_blob(stmt, 1, blob.data(), static_cast<int>(blob.size()),
                        SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to save tile index: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) == 0)
      throw std::runtime_error("Point cloud not found: " + std::string(name));
  }

  std::vector<uint8_t> getTileIndex(std::string_view name) const {
    const char *sql = "SELECT tile_index FROM point_clouds WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare tile index query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Point cloud not found: " + std::string(name));
    if (sqlite3_column_type(stmt, 0) == SQLITE_NULL)
      return {};
    const auto *bytes =
        static_cast<const uint8_t *>(sqlite3_column_blob(stmt, 0));
    const int size = sqlite3_column_bytes(stmt, 0);
    return std::vector<uint8_t>(bytes, bytes + size);
  }

  // ── Label definitions ──────────────────────────────────────────────

  void saveLabelDefinitions(std::string_view cloudName,
                            const std::map<int, std::string> &labelMap) {
    int cloudId = getCloudId(cloudName);

    // Savepoint: nests inside a ProjectDB::Transaction (see
    // savePointCloudMeta).
    Savepoint sp(db, "save_label_definitions");
    {
      // Delete existing definitions for this cloud
      {
        const char *del = "DELETE FROM label_definitions WHERE cloud_id = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, del, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("Failed to prepare label delete");
        StmtGuard guard(stmt);
        sqlite3_bind_int(stmt, 1, cloudId);
        sqlite3_step(stmt);
      }

      // Insert new definitions
      const char *ins = R"(
        INSERT INTO label_definitions (cloud_id, label_id, name)
        VALUES (?, ?, ?);
      )";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare label insert");
      StmtGuard guard(stmt);

      for (const auto &[labelId, labelName] : labelMap) {
        sqlite3_bind_int(stmt, 1, cloudId);
        sqlite3_bind_int(stmt, 2, labelId);
        sqlite3_bind_text(stmt, 3, labelName.c_str(), -1, SQLITE_TRANSIENT);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to insert label definition");
        sqlite3_reset(stmt);
      }
    }
    sp.release();
  }

  std::map<int, std::string>
  getLabelDefinitions(std::string_view cloudName) const {
    int cloudId = getCloudId(cloudName);

    const char *sql =
        "SELECT label_id, name FROM label_definitions WHERE cloud_id = ? "
        "ORDER BY label_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query label definitions");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);

    std::map<int, std::string> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int id = sqlite3_column_int(stmt, 0);
      const char *name =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      result[id] = name;
    }
    return result;
  }

  // ── Instances (stable identity) ────────────────────────────────────

  void saveInstances(const std::string &cloudName,
                     const std::vector<ProjectDB::InstanceRecord> &records) {
    int cloudId = getCloudId(cloudName);

    // Validate GUIDs up-front: non-empty, and unique within the batch.
    std::set<std::string> seen;
    for (const auto &r : records) {
      if (r.guid.empty())
        throw std::runtime_error(fmt::format(
            "save_instances: instance {} in cloud '{}' has an empty guid",
            r.instance_id, cloudName));
      if (!seen.insert(r.guid).second)
        throw std::runtime_error(
            fmt::format("save_instances: duplicate guid '{}' within batch for "
                        "cloud '{}'",
                        r.guid, cloudName));
    }

    execOrThrow("BEGIN TRANSACTION;");
    try {
      {
        const char *del = "DELETE FROM instances WHERE cloud_id = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, del, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error("Failed to prepare instances delete: " +
                                   std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        sqlite3_bind_int(stmt, 1, cloudId);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to clear instances: " +
                                   std::string(sqlite3_errmsg(db)));
      }

      const char *ins = R"(
        INSERT INTO instances
          (cloud_id, instance_id, guid, semantic_class, point_count)
        VALUES (?, ?, ?, ?, ?);
      )";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, ins, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare instance insert: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);
      for (const auto &r : records) {
        sqlite3_bind_int(stmt, 1, cloudId);
        sqlite3_bind_int(stmt, 2, static_cast<int>(r.instance_id));
        sqlite3_bind_text(stmt, 3, r.guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt, 4, r.semantic_class);
        sqlite3_bind_int(stmt, 5, r.point_count);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error(
              fmt::format("Failed to insert instance {} (guid {}): {}",
                          r.instance_id, r.guid, sqlite3_errmsg(db)));
        sqlite3_reset(stmt);
      }

      execOrThrow("COMMIT;");
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  std::vector<ProjectDB::InstanceRecord>
  getInstances(const std::string &cloudName) const {
    int cloudId = getCloudId(cloudName);
    const char *sql = "SELECT instance_id, guid, semantic_class, point_count "
                      "FROM instances WHERE cloud_id = ? ORDER BY instance_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query instances: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    std::vector<ProjectDB::InstanceRecord> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::InstanceRecord r;
      r.instance_id = static_cast<uint32_t>(sqlite3_column_int(stmt, 0));
      const char *g =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      r.guid = g ? g : "";
      r.semantic_class = sqlite3_column_int(stmt, 2);
      r.point_count = sqlite3_column_int(stmt, 3);
      result.push_back(std::move(r));
    }
    return result;
  }

  std::string instanceGuid(const std::string &cloudName,
                           uint32_t instanceId) const {
    int cloudId = getCloudId(cloudName);
    const char *sql = "SELECT guid FROM instances "
                      "WHERE cloud_id = ? AND instance_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query instance guid: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    sqlite3_bind_int(stmt, 2, static_cast<int>(instanceId));
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      const char *g =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      if (g)
        return std::string(g);
    }
    throw std::runtime_error(fmt::format("Instance {} not found in cloud '{}'",
                                         instanceId, cloudName));
  }

  bool instanceExists(int cloudId, int instanceId) const {
    const char *sql = "SELECT 1 FROM instances "
                      "WHERE cloud_id = ? AND instance_id = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    sqlite3_bind_int(stmt, 2, instanceId);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  bool passportExists(std::string_view documentGuid) const {
    const char *sql = "SELECT 1 FROM material_passports "
                      "WHERE document_guid = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, documentGuid.data(),
                      static_cast<int>(documentGuid.size()), SQLITE_TRANSIENT);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  // ── Instance ↔ material links ──────────────────────────────────────

  void setInstanceMaterial(std::string_view cloudName, int instanceId,
                           std::string_view materialGuid) {
    int cloudId = getCloudId(cloudName);

    // Referential integrity: the instance and passport must both exist so we
    // can never create a dangling link (STANDARDS §3.2, §5).
    if (!instanceExists(cloudId, instanceId))
      throw std::runtime_error(fmt::format(
          "set_instance_material: instance {} does not exist in cloud '{}' "
          "(run 'rux create instances' first)",
          instanceId, cloudName));
    if (!passportExists(materialGuid))
      throw std::runtime_error(fmt::format(
          "set_instance_material: material passport '{}' does not exist",
          materialGuid));

    const char *sql = R"(
      INSERT INTO instance_materials (cloud_id, instance_id, material_guid)
      VALUES (?, ?, ?)
      ON CONFLICT(cloud_id, instance_id)
        DO UPDATE SET material_guid = excluded.material_guid;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare instance material upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    sqlite3_bind_int(stmt, 2, instanceId);
    sqlite3_bind_text(stmt, 3, materialGuid.data(),
                      static_cast<int>(materialGuid.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to set instance material: " +
                               std::string(sqlite3_errmsg(db)));

    // Schema v25: the part backed by this instance follows the new link,
    // unless another part owns that passport — then it keeps its own (or,
    // having none, reads as passport-less until its first edit) and the two
    // disagree until someone relinks (warn, STANDARDS §5).
    if (hasPassportColumn) {
      // The part's passport before the move, to report one it leaves behind.
      std::optional<std::pair<std::string, std::string>> before;
      {
        sqlite3_stmt *s = prepare_or_throw(
            db,
            "SELECT code, passport_guid FROM survey_parts WHERE instance_guid "
            "= (SELECT guid FROM instances WHERE cloud_id = ? AND "
            "instance_id = ?) AND passport_guid IS NOT NULL;",
            "set_instance_material");
        StmtGuard guard(s);
        sqlite3_bind_int(s, 1, cloudId);
        sqlite3_bind_int(s, 2, instanceId);
        if (sqlite3_step(s) == SQLITE_ROW)
          before = std::make_pair(column_text(s, 0), column_text(s, 1));
      }
      sqlite3_stmt *up = prepare_or_throw(
          db,
          "UPDATE survey_parts SET passport_guid = ?1 "
          "WHERE instance_guid = (SELECT guid FROM instances WHERE "
          "cloud_id = ?2 AND instance_id = ?3) "
          "AND NOT EXISTS (SELECT 1 FROM survey_parts WHERE "
          "passport_guid = ?1);",
          "set_instance_material");
      StmtGuard up_guard(up);
      bind_text(up, 1, materialGuid);
      sqlite3_bind_int(up, 2, cloudId);
      sqlite3_bind_int(up, 3, instanceId);
      if (sqlite3_step(up) != SQLITE_DONE)
        throw std::runtime_error("Failed to sync the part's passport: " +
                                 std::string(sqlite3_errmsg(db)));
      const bool moved =
          before && sqlite3_changes(db) > 0 && before->second != materialGuid;
      if (moved && !passportOwnedByPart(before->second) &&
          !passportLinkedByInstance(before->second)) {
        // The old passport (and any values edited on it) stays in the
        // project but nothing references it any more — say so (§5).
        sqlite3_stmt *n = prepare_or_throw(
            db,
            "SELECT COUNT(*) FROM passport_property_values v JOIN "
            "material_passports mp ON mp.id = v.passport_id WHERE "
            "mp.document_guid = ?;",
            "set_instance_material");
        StmtGuard n_guard(n);
        bind_text(n, 1, before->second);
        sqlite3_step(n);
        const auto values = sqlite3_column_int64(n, 0);
        // Values on it (user edits, possibly) are now out of reach: warn.
        // An empty one loses nothing.
        reusex::log(values > 0 ? reusex::core::LogLevel::warn
                               : reusex::core::LogLevel::info,
                    "set_instance_material: survey part '{}' moved from "
                    "passport '{}' to '{}'; '{}' ({} stored value(s)) is "
                    "now unreferenced",
                    before->first, before->second, materialGuid, before->second,
                    values);
      }
      sqlite3_stmt *chk = prepare_or_throw(
          db,
          "SELECT p.code, p.passport_guid, (SELECT code FROM survey_parts "
          "WHERE passport_guid = ?1) FROM survey_parts p "
          "WHERE p.instance_guid = (SELECT guid FROM instances WHERE "
          "cloud_id = ?2 AND instance_id = ?3) AND (p.passport_guid IS "
          "NULL OR p.passport_guid <> ?1);",
          "set_instance_material");
      StmtGuard chk_guard(chk);
      bind_text(chk, 1, materialGuid);
      sqlite3_bind_int(chk, 2, cloudId);
      sqlite3_bind_int(chk, 3, instanceId);
      if (sqlite3_step(chk) == SQLITE_ROW) {
        if (sqlite3_column_type(chk, 1) == SQLITE_NULL)
          // Not a warn: ensure_resource_passport warns once, on the write
          // that gives this part its own passport (R-P5: one warning).
          reusex::debug("set_instance_material: passport '{}' belongs to "
                        "survey part '{}'; part '{}' has no passport until "
                        "its first edit creates one",
                        materialGuid, column_text(chk, 2), column_text(chk, 0));
        else
          reusex::warn("set_instance_material: passport '{}' belongs to "
                       "survey part '{}'; part '{}' keeps its own passport "
                       "'{}'",
                       materialGuid, column_text(chk, 2), column_text(chk, 0),
                       column_text(chk, 1));
      }
    }
  }

  std::optional<std::string> instanceMaterialGuid(std::string_view cloudName,
                                                  int instanceId) const {
    int cloudId = getCloudId(cloudName);
    const char *sql = "SELECT material_guid FROM instance_materials "
                      "WHERE cloud_id = ? AND instance_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query instance material: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    sqlite3_bind_int(stmt, 2, instanceId);
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      const char *g =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      if (g)
        return std::string(g);
    }
    return std::nullopt;
  }

  std::map<int, std::string>
  getInstanceMaterials(std::string_view cloudName) const {
    int cloudId = getCloudId(cloudName);
    const char *sql =
        "SELECT instance_id, material_guid FROM instance_materials "
        "WHERE cloud_id = ? ORDER BY instance_id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query instance materials: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, cloudId);
    std::map<int, std::string> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      int id = sqlite3_column_int(stmt, 0);
      const char *g =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      if (g)
        result[id] = g;
    }
    return result;
  }

  // ── Material annotation CRUD ────────────────────────────────────────

  void saveMaterialAnnotation(std::string_view materialGuid,
                              const ProjectDB::MaterialAnnotation &annotation) {
    // Referential integrity: the material passport must exist so an annotation
    // row can never dangle (STANDARDS §3.2, §5). The attributes stage runs
    // after `rux create materials`, which writes the `material_passports` and
    // `instance_materials` tables.
    if (!passportExists(materialGuid))
      throw std::runtime_error(fmt::format(
          "save_material_annotation: material passport '{}' does not exist "
          "(run 'rux create materials' first)",
          materialGuid));

    auto bindTextOrNull = [&](sqlite3_stmt *stmt, int col,
                              const std::string &s) {
      if (s.empty())
        sqlite3_bind_null(stmt, col);
      else
        sqlite3_bind_text(stmt, col, s.data(), static_cast<int>(s.size()),
                          SQLITE_TRANSIENT);
    };

    // Upsert the description/provenance row and REPLACE the kv rows atomically:
    // a partial write must never leave stale keys behind (STANDARDS §5, §6).
    execOrThrow("BEGIN TRANSACTION;");
    try {
      {
        const char *sql = R"(
          INSERT INTO material_annotations
            (material_guid, description, provider_model, raw_json)
          VALUES (?, ?, ?, ?)
          ON CONFLICT(material_guid) DO UPDATE SET
            description    = excluded.description,
            provider_model = excluded.provider_model,
            raw_json       = excluded.raw_json,
            created_at     = datetime('now');
        )";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error(
              "Failed to prepare material annotation upsert: " +
              std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        sqlite3_bind_text(stmt, 1, materialGuid.data(),
                          static_cast<int>(materialGuid.size()),
                          SQLITE_TRANSIENT);
        bindTextOrNull(stmt, 2, annotation.description);
        bindTextOrNull(stmt, 3, annotation.provider_model);
        bindTextOrNull(stmt, 4, annotation.raw_json);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to save material annotation: " +
                                   std::string(sqlite3_errmsg(db)));
      }

      // Clear the previous kv rows so no stale key lingers across a
      // re-describe.
      {
        const char *sql =
            "DELETE FROM material_annotation_kv WHERE material_guid = ?;";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error(
              "Failed to prepare material annotation kv delete: " +
              std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        sqlite3_bind_text(stmt, 1, materialGuid.data(),
                          static_cast<int>(materialGuid.size()),
                          SQLITE_TRANSIENT);
        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error(
              "Failed to clear material annotation kv rows: " +
              std::string(sqlite3_errmsg(db)));
      }

      // Insert the new kv rows. A duplicate key within one answer would violate
      // the compound PK; ON CONFLICT REPLACE keeps the last value for a key so
      // a model that repeats a key does not abort the whole save.
      if (!annotation.attributes.empty()) {
        const char *sql = R"(
          INSERT INTO material_annotation_kv (material_guid, key, value)
          VALUES (?, ?, ?)
          ON CONFLICT(material_guid, key) DO UPDATE SET value = excluded.value;
        )";
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
          throw std::runtime_error(
              "Failed to prepare material annotation kv insert: " +
              std::string(sqlite3_errmsg(db)));
        StmtGuard guard(stmt);
        for (const auto &[key, value] : annotation.attributes) {
          if (key.empty())
            continue; // never store an empty key (§5)
          sqlite3_reset(stmt);
          sqlite3_clear_bindings(stmt);
          sqlite3_bind_text(stmt, 1, materialGuid.data(),
                            static_cast<int>(materialGuid.size()),
                            SQLITE_TRANSIENT);
          sqlite3_bind_text(stmt, 2, key.data(), static_cast<int>(key.size()),
                            SQLITE_TRANSIENT);
          bindTextOrNull(stmt, 3, value);
          if (sqlite3_step(stmt) != SQLITE_DONE)
            throw std::runtime_error(
                "Failed to save material annotation kv row: " +
                std::string(sqlite3_errmsg(db)));
        }
      }

      execOrThrow("COMMIT;");
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  std::optional<ProjectDB::MaterialAnnotation>
  materialAnnotation(std::string_view materialGuid) const {
    const char *sql = "SELECT description, provider_model, raw_json "
                      "FROM material_annotations WHERE material_guid = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query material annotation: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, materialGuid.data(),
                      static_cast<int>(materialGuid.size()), SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      return std::nullopt;

    auto colText = [&](sqlite3_stmt *s, int col) -> std::string {
      const char *t =
          reinterpret_cast<const char *>(sqlite3_column_text(s, col));
      return t ? std::string(t) : std::string();
    };
    ProjectDB::MaterialAnnotation annotation;
    annotation.description = colText(stmt, 0);
    annotation.provider_model = colText(stmt, 1);
    annotation.raw_json = colText(stmt, 2);

    // Load the kv rows in a deterministic (key-sorted) order (STANDARDS §6).
    const char *kvSql = "SELECT key, value FROM material_annotation_kv "
                        "WHERE material_guid = ? ORDER BY key;";
    sqlite3_stmt *kvStmt;
    if (sqlite3_prepare_v2(db, kvSql, -1, &kvStmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query material annotation kv: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard kvGuard(kvStmt);
    sqlite3_bind_text(kvStmt, 1, materialGuid.data(),
                      static_cast<int>(materialGuid.size()), SQLITE_TRANSIENT);
    while (sqlite3_step(kvStmt) == SQLITE_ROW)
      annotation.attributes.emplace_back(colText(kvStmt, 0),
                                         colText(kvStmt, 1));
    return annotation;
  }

  // ── Mesh CRUD ──────────────────────────────────────────────────────

  void saveMesh(std::string_view name, const pcl::PolygonMesh &mesh,
                std::string_view stage, std::string_view paramsJson) {
    // Refuse to persist empty meshes: pcl::io::savePLYFileBinary silently
    // writes a header-only or zero-byte file, which then fails to read back
    // with a misleading error. Caller should handle the empty result.
    {
      const size_t v =
          static_cast<size_t>(mesh.cloud.width) * mesh.cloud.height;
      if (v == 0 || mesh.polygons.empty())
        throw std::runtime_error(
            "Refusing to save empty mesh '" + std::string(name) + "' (" +
            std::to_string(v) + " vertices, " +
            std::to_string(mesh.polygons.size()) + " faces)");
    }

    // Serialize mesh to PLY binary via temp file
    auto tmpPath =
        std::filesystem::temp_directory_path() /
        ("reusex_mesh_" + std::to_string(reinterpret_cast<uintptr_t>(&mesh)) +
         ".ply");
    pcl::io::savePLYFileBinary(tmpPath.string(), mesh);

    // Read the file into a buffer
    std::ifstream ifs(tmpPath, std::ios::binary | std::ios::ate);
    if (!ifs)
      throw std::runtime_error("Failed to read temporary PLY file");
    auto fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    std::vector<char> plyData(static_cast<size_t>(fileSize));
    ifs.read(plyData.data(), fileSize);
    ifs.close();
    std::filesystem::remove(tmpPath);

    // Count vertices and faces
    int vertexCount = 0;
    int faceCount = static_cast<int>(mesh.polygons.size());
    if (mesh.cloud.height > 0)
      vertexCount = static_cast<int>(mesh.cloud.width * mesh.cloud.height);

    const char *upsert = R"(
      INSERT INTO meshes (name, format, data, vertex_count, face_count, stage, parameters)
      VALUES (?, 'ply_binary', ?, ?, ?, ?, ?)
      ON CONFLICT(name) DO UPDATE SET
        format = excluded.format,
        data = excluded.data,
        vertex_count = excluded.vertex_count,
        face_count = excluded.face_count,
        created_at = datetime('now'),
        stage = excluded.stage,
        parameters = excluded.parameters;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, plyData.data(), static_cast<int>(plyData.size()),
                      SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, vertexCount);
    sqlite3_bind_int(stmt, 4, faceCount);
    if (!stage.empty())
      sqlite3_bind_text(stmt, 5, stage.data(), static_cast<int>(stage.size()),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 5);
    if (!paramsJson.empty())
      sqlite3_bind_text(stmt, 6, paramsJson.data(),
                        static_cast<int>(paramsJson.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 6);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert mesh: " +
                               std::string(sqlite3_errmsg(db)));
  }

  void saveMesh(std::string_view name, const pcl::TextureMesh &mesh,
                std::string_view stage, std::string_view paramsJson) {
    {
      const size_t v =
          static_cast<size_t>(mesh.cloud.width) * mesh.cloud.height;
      size_t f = 0;
      for (const auto &polys : mesh.tex_polygons)
        f += polys.size();
      if (v == 0 || f == 0)
        throw std::runtime_error("Refusing to save empty texture mesh '" +
                                 std::string(name) + "' (" + std::to_string(v) +
                                 " vertices, " + std::to_string(f) + " faces)");
    }

    // Base for resolving relative texture paths referenced by the MTL.
    // The project's own directory is used rather than the process working
    // directory: current_path() is process-global mutable state, so it is
    // neither thread-safe nor reproducible across invocations (issue #245).
    // Textures produced by reusex::geometry::texture_mesh* are absolute and
    // never hit this fallback; it only matters for meshes assembled elsewhere.
    const std::filesystem::path textureBase =
        std::filesystem::absolute(dbPath).parent_path();

    // Serialize texture mesh to OBJ via temp file (preserves texture data)
    auto tmpPath =
        std::filesystem::temp_directory_path() /
        ("reusex_texmesh_" +
         std::to_string(reinterpret_cast<uintptr_t>(&mesh)) + ".obj");
    pcl::io::saveOBJFile(tmpPath.string(), mesh);

    // Read OBJ file and associated MTL file into buffers
    std::ifstream ifs_obj(tmpPath, std::ios::binary | std::ios::ate);
    if (!ifs_obj)
      throw std::runtime_error("Failed to read temporary OBJ file");
    auto objSize = ifs_obj.tellg();
    ifs_obj.seekg(0, std::ios::beg);
    std::vector<char> objData(static_cast<size_t>(objSize));
    ifs_obj.read(objData.data(), objSize);
    ifs_obj.close();

    // Read MTL file if it exists
    auto mtlPath = tmpPath;
    mtlPath.replace_extension(".mtl");
    std::vector<char> mtlData;
    if (std::filesystem::exists(mtlPath)) {
      std::ifstream ifs_mtl(mtlPath, std::ios::binary | std::ios::ate);
      if (ifs_mtl) {
        auto mtlSize = ifs_mtl.tellg();
        ifs_mtl.seekg(0, std::ios::beg);
        mtlData.resize(static_cast<size_t>(mtlSize));
        ifs_mtl.read(mtlData.data(), mtlSize);
        ifs_mtl.close();
      }
    }

    std::filesystem::remove(tmpPath);
    if (std::filesystem::exists(mtlPath))
      std::filesystem::remove(mtlPath);

    // Combine OBJ and MTL data with a separator
    std::vector<char> combinedData = objData;
    if (!mtlData.empty()) {
      combinedData.push_back('\0'); // Null separator
      combinedData.insert(combinedData.end(), mtlData.begin(), mtlData.end());
    }

    // Count vertices and faces
    int vertexCount = 0;
    int faceCount = 0;
    for (const auto &poly_group : mesh.tex_polygons)
      faceCount += static_cast<int>(poly_group.size());
    if (mesh.cloud.height > 0)
      vertexCount = static_cast<int>(mesh.cloud.width * mesh.cloud.height);

    const char *upsert = R"(
      INSERT INTO meshes (name, format, data, vertex_count, face_count, stage, parameters)
      VALUES (?, 'obj_textured', ?, ?, ?, ?, ?)
      ON CONFLICT(name) DO UPDATE SET
        format = excluded.format,
        data = excluded.data,
        vertex_count = excluded.vertex_count,
        face_count = excluded.face_count,
        created_at = datetime('now'),
        stage = excluded.stage,
        parameters = excluded.parameters;
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare texture mesh upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, combinedData.data(),
                      static_cast<int>(combinedData.size()), SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, vertexCount);
    sqlite3_bind_int(stmt, 4, faceCount);
    if (!stage.empty())
      sqlite3_bind_text(stmt, 5, stage.data(), static_cast<int>(stage.size()),
                        SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 5);
    if (!paramsJson.empty())
      sqlite3_bind_text(stmt, 6, paramsJson.data(),
                        static_cast<int>(paramsJson.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 6);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert texture mesh: " +
                               std::string(sqlite3_errmsg(db)));

    // Store texture image blobs in mesh_texture_data table
    if (!mtlData.empty()) {
      std::string mtlStr(mtlData.data(), mtlData.size());
      auto texPaths = parseMtlTextures(mtlStr);

      // Get the mesh id we just inserted
      const char *idSql = "SELECT id FROM meshes WHERE name = ?;";
      sqlite3_stmt *idStmt;
      if (sqlite3_prepare_v2(db, idSql, -1, &idStmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query mesh id");
      StmtGuard idGuard(idStmt);
      sqlite3_bind_text(idStmt, 1, name.data(), static_cast<int>(name.size()),
                        SQLITE_TRANSIENT);
      if (sqlite3_step(idStmt) != SQLITE_ROW)
        throw std::runtime_error("Mesh not found after insert");
      int meshId = sqlite3_column_int(idStmt, 0);

      // Delete old texture data for this mesh
      const char *delSql = "DELETE FROM mesh_texture_data WHERE mesh_id = ?;";
      sqlite3_stmt *delStmt;
      if (sqlite3_prepare_v2(db, delSql, -1, &delStmt, nullptr) == SQLITE_OK) {
        sqlite3_bind_int(delStmt, 1, meshId);
        sqlite3_step(delStmt);
        sqlite3_finalize(delStmt);
      }

      for (const auto &[matName, texPath] : texPaths) {
        // Resolve relative texture paths against the project directory, not
        // the process working directory (issue #245).
        std::filesystem::path fullPath = texPath;
        if (!fullPath.is_absolute())
          fullPath = textureBase / fullPath;

        if (!std::filesystem::exists(fullPath)) {
          reusex::warn("Texture file not found: {} (resolved '{}' against "
                       "project directory '{}')",
                       fullPath.string(), texPath, textureBase.string());
          continue;
        }

        cv::Mat texImg = cv::imread(fullPath.string(), cv::IMREAD_COLOR);
        if (texImg.empty()) {
          reusex::warn("Failed to read texture: {}", fullPath.string());
          continue;
        }

        // Compress as JPEG
        std::vector<unsigned char> jpegBuf;
        if (!cv::imencode(".jpg", texImg, jpegBuf,
                          {cv::IMWRITE_JPEG_QUALITY, 95})) {
          reusex::warn("Failed to encode texture as JPEG: {}", matName);
          continue;
        }

        const char *texInsert = R"(
          INSERT INTO mesh_texture_data (mesh_id, tex_name, image_data, width, height, format)
          VALUES (?, ?, ?, ?, ?, 'jpg')
          ON CONFLICT(mesh_id, tex_name) DO UPDATE SET
            image_data = excluded.image_data,
            width = excluded.width,
            height = excluded.height;
        )";

        sqlite3_stmt *texStmt;
        if (sqlite3_prepare_v2(db, texInsert, -1, &texStmt, nullptr) !=
            SQLITE_OK) {
          reusex::warn("Failed to prepare texture insert: {}",
                       sqlite3_errmsg(db));
          continue;
        }
        StmtGuard texGuard(texStmt);

        sqlite3_bind_int(texStmt, 1, meshId);
        sqlite3_bind_text(texStmt, 2, matName.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_blob(texStmt, 3, jpegBuf.data(),
                          static_cast<int>(jpegBuf.size()), SQLITE_TRANSIENT);
        sqlite3_bind_int(texStmt, 4, texImg.cols);
        sqlite3_bind_int(texStmt, 5, texImg.rows);

        if (sqlite3_step(texStmt) != SQLITE_DONE) {
          reusex::warn("Failed to store texture {}: {}", matName,
                       sqlite3_errmsg(db));
        } else {
          reusex::debug("Stored texture '{}' ({} x {} px, {:.1f} KB)", matName,
                        texImg.cols, texImg.rows, jpegBuf.size() / 1024.0);
        }
      }
    }
  }

  pcl::PolygonMesh::Ptr getMesh(std::string_view name) const {
    const char *sql = "SELECT data FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh load");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));

    const void *blob = sqlite3_column_blob(stmt, 0);
    int blobSize = sqlite3_column_bytes(stmt, 0);

    // Write to temp file and load via PCL (use hash to avoid path traversal)
    auto tmpPath =
        std::filesystem::temp_directory_path() /
        ("reusex_mesh_load_" +
         std::to_string(std::hash<std::string>{}(std::string(name))) + ".ply");
    {
      std::ofstream ofs(tmpPath, std::ios::binary);
      ofs.write(static_cast<const char *>(blob), blobSize);
    }

    auto result = std::make_shared<pcl::PolygonMesh>();
    if (pcl::io::loadPLYFile(tmpPath.string(), *result) < 0) {
      std::filesystem::remove(tmpPath);
      throw std::runtime_error("Failed to parse PLY mesh data");
    }
    std::filesystem::remove(tmpPath);
    return result;
  }

  pcl::TextureMesh::Ptr getTextureMesh(std::string_view name) const {
    const char *sql = "SELECT id, format, data FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare texture mesh load");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Texture mesh not found: " + std::string(name));

    int meshId = sqlite3_column_int(stmt, 0);
    const char *format =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    if (std::string(format) != "obj_textured")
      throw std::runtime_error(
          "Mesh '" + std::string(name) +
          "' is not a texture mesh (format: " + std::string(format) + ")");

    const void *blob = sqlite3_column_blob(stmt, 2);
    int blobSize = sqlite3_column_bytes(stmt, 2);

    // Find null separator between OBJ and MTL data
    const char *blobData = static_cast<const char *>(blob);
    const char *separator = std::find(blobData, blobData + blobSize, '\0');
    int objSize = static_cast<int>(separator - blobData);

    // Create temp directory for this mesh's textures
    auto texDir = std::filesystem::temp_directory_path() /
                  ("reusex_textures_" + std::to_string(meshId));
    std::filesystem::create_directories(texDir);

    // Query texture blobs from mesh_texture_data
    std::map<std::string, std::string>
        newTexPaths; // material -> temp file path
    {
      const char *texSql =
          "SELECT tex_name, image_data, width, height FROM mesh_texture_data "
          "WHERE mesh_id = ?;";
      sqlite3_stmt *texStmt;
      if (sqlite3_prepare_v2(db, texSql, -1, &texStmt, nullptr) == SQLITE_OK) {
        StmtGuard texGuard(texStmt);
        sqlite3_bind_int(texStmt, 1, meshId);

        while (sqlite3_step(texStmt) == SQLITE_ROW) {
          const char *texName =
              reinterpret_cast<const char *>(sqlite3_column_text(texStmt, 0));
          const void *imgBlob = sqlite3_column_blob(texStmt, 1);
          int imgSize = sqlite3_column_bytes(texStmt, 1);

          if (!texName || !imgBlob || imgSize == 0)
            continue;

          // Decode JPEG blob
          cv::Mat encoded(1, imgSize, CV_8UC1, const_cast<void *>(imgBlob));
          cv::Mat texImg = cv::imdecode(encoded, cv::IMREAD_COLOR);
          if (texImg.empty())
            continue;

          // Write to temp file
          auto texFilePath = texDir / (std::string(texName) + ".jpg");
          cv::imwrite(texFilePath.string(), texImg);
          newTexPaths[texName] = texFilePath.string();
        }
      }
    }

    // Read original MTL content
    std::string mtlStr;
    if (separator != blobData + blobSize &&
        separator + 1 < blobData + blobSize) {
      mtlStr.assign(separator + 1, blobData + blobSize);
    }

    // Check for material name mismatch (e.g. pre-atlas MTL + atlas blob)
    if (!mtlStr.empty() && !newTexPaths.empty()) {
      auto mtlMaterials = parseMtlTextures(mtlStr);
      size_t matched = 0;
      for (const auto &[matName, _] : mtlMaterials) {
        if (newTexPaths.count(matName))
          ++matched;
      }
      if (matched == 0 && !mtlMaterials.empty()) {
        reusex::warn(
            "Textured mesh '{}': stored MTL has {} materials but none match "
            "the {} texture blob(s) in the database. The mesh was likely "
            "saved before atlas support. Re-create with 'rux create texture' "
            "for proper texturing.",
            name, mtlMaterials.size(), newTexPaths.size());
      }
    }

    // Write OBJ and MTL to temp dir
    auto tmpObjPath = texDir / "mesh.obj";
    auto tmpMtlPath = texDir / "mesh.mtl";

    if (!mtlStr.empty() && !newTexPaths.empty()) {
      // Regenerate MTL with updated texture paths
      std::string updatedMtl = regenerateMtl(mtlStr, newTexPaths);
      {
        std::ofstream ofs(tmpMtlPath, std::ios::binary);
        ofs.write(updatedMtl.data(),
                  static_cast<std::streamsize>(updatedMtl.size()));
      }

      // Update OBJ to reference mesh.mtl in same directory
      std::string objStr(blobData, objSize);
      // Replace mtllib line to point to local file
      std::istringstream objStream(objStr);
      std::ostringstream objOut;
      std::string line;
      while (std::getline(objStream, line)) {
        if (line.rfind("mtllib ", 0) == 0) {
          objOut << "mtllib mesh.mtl\n";
        } else {
          objOut << line << "\n";
        }
      }
      std::string updatedObj = objOut.str();
      {
        std::ofstream ofs(tmpObjPath, std::ios::binary);
        ofs.write(updatedObj.data(),
                  static_cast<std::streamsize>(updatedObj.size()));
      }
    } else {
      // No texture blobs available - write OBJ and MTL without texture refs
      {
        std::ofstream ofs(tmpObjPath, std::ios::binary);
        ofs.write(blobData, objSize);
      }
      if (!mtlStr.empty()) {
        // Strip map_Kd lines since texture files aren't in the database
        std::istringstream stream(mtlStr);
        std::ostringstream out;
        std::string line;
        while (std::getline(stream, line)) {
          std::string trimmed = line;
          size_t start = trimmed.find_first_not_of(" \t");
          if (start != std::string::npos)
            trimmed = trimmed.substr(start);
          if (trimmed.rfind("map_Kd ", 0) == 0)
            continue;
          out << line << "\n";
        }
        std::string stripped = out.str();
        std::ofstream ofs(tmpMtlPath, std::ios::binary);
        ofs.write(stripped.data(),
                  static_cast<std::streamsize>(stripped.size()));
      }
    }

    auto result = std::make_shared<pcl::TextureMesh>();
    if (pcl::io::loadOBJFile(tmpObjPath.string(), *result) < 0) {
      std::filesystem::remove_all(texDir);
      throw std::runtime_error("Failed to parse OBJ texture mesh data");
    }

    // Fix texture paths: loadOBJFile may resolve relative paths from the
    // original MTL against the temp OBJ directory, producing invalid paths
    // like /tmp/..././mesh_textures/file.jpg. Remap to actual temp files.
    for (auto &mat : result->tex_materials) {
      if (mat.tex_file.empty())
        continue;
      std::filesystem::path p(mat.tex_file);
      if (!std::filesystem::exists(p)) {
        // Try finding the texture by filename in our temp dir
        auto candidate = texDir / p.filename();
        if (std::filesystem::exists(candidate)) {
          mat.tex_file = candidate.string();
        } else {
          // No texture available — clear to prevent PCL load errors
          mat.tex_file.clear();
        }
      }
    }

    // Leave temp files for viewer access (cleaned on next load)
    return result;
  }

  std::string getMeshFormat(std::string_view name) const {
    const char *sql = "SELECT format FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh format query");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));

    const char *fmt =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    return std::string(fmt ? fmt : "");
  }

  bool hasMesh(std::string_view name) const {
    const char *sql = "SELECT COUNT(*) FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  bool deleteMesh(std::string_view name) {
    // mesh_texture_data rows go with it (ON DELETE CASCADE).
    const char *sql = "DELETE FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete mesh '" + std::string(name) +
                               "': " + std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  std::vector<std::string> listMeshes() const {
    const char *sql = "SELECT name FROM meshes ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list meshes");
    StmtGuard guard(stmt);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  ProjectDB::MeshMetadata getMeshMetadata(std::string_view name) const {
    const char *sql =
        "SELECT name, format, vertex_count, face_count, stage, parameters, "
        "created_at FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh metadata query");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));

    ProjectDB::MeshMetadata meta;
    auto col_text = [&](int col) -> std::string {
      auto *p = reinterpret_cast<const char *>(sqlite3_column_text(stmt, col));
      return p ? std::string(p) : std::string();
    };
    meta.name = col_text(0);
    meta.format = col_text(1);
    meta.vertex_count = sqlite3_column_int(stmt, 2);
    meta.face_count = sqlite3_column_int(stmt, 3);
    meta.stage = col_text(4);
    meta.parameters = col_text(5);
    meta.created_at = col_text(6);
    return meta;
  }

  std::vector<uint8_t> getMeshDataBlob(std::string_view name) const {
    const char *sql = "SELECT data FROM meshes WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh data blob query");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));

    const auto *blob =
        static_cast<const uint8_t *>(sqlite3_column_blob(stmt, 0));
    int blobSize = sqlite3_column_bytes(stmt, 0);
    return std::vector<uint8_t>(blob, blob + blobSize);
  }

  std::vector<ProjectDB::MeshTextureInfo>
  getMeshTextureBlobs(std::string_view name) const {
    // Look up mesh id
    const char *idSql = "SELECT id FROM meshes WHERE name = ?;";
    sqlite3_stmt *idStmt;
    if (sqlite3_prepare_v2(db, idSql, -1, &idStmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh id query");
    StmtGuard idGuard(idStmt);
    sqlite3_bind_text(idStmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(idStmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));
    int meshId = sqlite3_column_int(idStmt, 0);

    const char *sql = "SELECT tex_name, image_data, width, height, format "
                      "FROM mesh_texture_data WHERE mesh_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare texture blob query");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, meshId);

    std::vector<ProjectDB::MeshTextureInfo> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::MeshTextureInfo info;
      auto *tn = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      info.tex_name = tn ? std::string(tn) : "";
      const auto *imgBlob =
          static_cast<const uint8_t *>(sqlite3_column_blob(stmt, 1));
      int imgSize = sqlite3_column_bytes(stmt, 1);
      if (imgBlob && imgSize > 0)
        info.image_data.assign(imgBlob, imgBlob + imgSize);
      info.width = sqlite3_column_int(stmt, 2);
      info.height = sqlite3_column_int(stmt, 3);
      auto *fmt = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
      info.format = fmt ? std::string(fmt) : "jpg";
      result.push_back(std::move(info));
    }
    return result;
  }

  std::vector<ProjectDB::MeshTextureMetadata>
  getMeshTextureMetadata(std::string_view name) const {
    const char *idSql = "SELECT id FROM meshes WHERE name = ?;";
    sqlite3_stmt *idStmt;
    if (sqlite3_prepare_v2(db, idSql, -1, &idStmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare mesh id query");
    StmtGuard idGuard(idStmt);
    sqlite3_bind_text(idStmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(idStmt) != SQLITE_ROW)
      throw std::runtime_error("Mesh not found: " + std::string(name));
    int meshId = sqlite3_column_int(idStmt, 0);

    const char *sql = "SELECT tex_name, width, height, format "
                      "FROM mesh_texture_data WHERE mesh_id = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare texture metadata query");
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, meshId);

    std::vector<ProjectDB::MeshTextureMetadata> result;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::MeshTextureMetadata info;
      auto *tn = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      info.tex_name = tn ? std::string(tn) : "";
      info.width = sqlite3_column_int(stmt, 1);
      info.height = sqlite3_column_int(stmt, 2);
      auto *fmt = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
      info.format = fmt ? std::string(fmt) : "jpg";
      result.push_back(std::move(info));
    }
    return result;
  }

  // ── Gaussian splat CRUD ────────────────────────────────────────────

  /// Every splat read path funnels through this. A read-only open never runs
  /// migrations, so a v11 project opened read-only genuinely has no splat
  /// tables — readers must report "no splats" rather than surface a raw
  /// "no such table: gaussian_splats" from sqlite.
  bool gaussianSplatTablesExist() const {
    return tableExists("gaussian_splats") && tableExists("gaussian_splat_data");
  }

  int getGaussianSplatId(std::string_view name) const {
    const char *sql = "SELECT id FROM gaussian_splats WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query Gaussian splat id: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Gaussian splat not found: " +
                               std::string(name));
    return sqlite3_column_int(stmt, 0);
  }

  void saveGaussianSplat(std::string_view name, const std::vector<uint8_t> &ply,
                         std::string_view stage, std::string_view paramsJson) {
    // Validate BEFORE opening a transaction: a project must never end up
    // holding half a rejected file, and the caller gets the header error with
    // nothing written (STANDARDS §5).
    const std::string_view head(
        reinterpret_cast<const char *>(ply.data()),
        std::min(ply.size(), reusex::core::kGaussianSplatHeaderProbeBytes));
    const auto header = reusex::core::parse_gaussian_splat_ply_header(head);

    if (!gaussianSplatTablesExist())
      throw std::runtime_error(
          "This project has no Gaussian splat tables (schema is older than "
          "v12). Open it read-write once to migrate.");

    execOrThrow("BEGIN TRANSACTION;");
    try {
      const char *upsert = R"(
        INSERT INTO gaussian_splats
          (name, format, gaussian_count, sh_degree, byte_size, stage, parameters)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(name) DO UPDATE SET
          format = excluded.format,
          gaussian_count = excluded.gaussian_count,
          sh_degree = excluded.sh_degree,
          byte_size = excluded.byte_size,
          created_at = datetime('now'),
          stage = excluded.stage,
          parameters = excluded.parameters;
      )";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to prepare Gaussian splat upsert: " +
                                 std::string(sqlite3_errmsg(db)));
      {
        StmtGuard guard(stmt);
        sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                          SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, "ply", -1, SQLITE_STATIC);
        sqlite3_bind_int64(stmt, 3,
                           static_cast<sqlite3_int64>(header.gaussian_count));
        sqlite3_bind_int(stmt, 4, header.sh_degree);
        sqlite3_bind_int64(stmt, 5, static_cast<sqlite3_int64>(ply.size()));
        if (!stage.empty())
          sqlite3_bind_text(stmt, 6, stage.data(),
                            static_cast<int>(stage.size()), SQLITE_TRANSIENT);
        else
          sqlite3_bind_null(stmt, 6);
        if (!paramsJson.empty())
          sqlite3_bind_text(stmt, 7, paramsJson.data(),
                            static_cast<int>(paramsJson.size()),
                            SQLITE_TRANSIENT);
        else
          sqlite3_bind_null(stmt, 7);

        if (sqlite3_step(stmt) != SQLITE_DONE)
          throw std::runtime_error(
              "Failed to upsert Gaussian splat metadata: " +
              std::string(sqlite3_errmsg(db)));
      }

      const int splatId = getGaussianSplatId(name);

      // Replacing a splat keeps the row id, so the old chunks are NOT removed
      // by the ON DELETE CASCADE — drop them explicitly or a shorter payload
      // would leave a tail of the previous one behind it.
      {
        const char *deleteSql =
            "DELETE FROM gaussian_splat_data WHERE splat_id = ?;";
        sqlite3_stmt *delStmt;
        if (sqlite3_prepare_v2(db, deleteSql, -1, &delStmt, nullptr) !=
            SQLITE_OK)
          throw std::runtime_error(
              "Failed to prepare Gaussian splat data delete: " +
              std::string(sqlite3_errmsg(db)));
        StmtGuard delGuard(delStmt);
        sqlite3_bind_int(delStmt, 1, splatId);
        if (sqlite3_step(delStmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to clear Gaussian splat data: " +
                                   std::string(sqlite3_errmsg(db)));
      }

      const char *dataInsert =
          "INSERT INTO gaussian_splat_data (splat_id, chunk_index, data) "
          "VALUES (?, ?, ?);";
      sqlite3_stmt *dstmt;
      if (sqlite3_prepare_v2(db, dataInsert, -1, &dstmt, nullptr) != SQLITE_OK)
        throw std::runtime_error(
            "Failed to prepare Gaussian splat data insert: " +
            std::string(sqlite3_errmsg(db)));
      StmtGuard dguard(dstmt);

      const size_t total = ply.size();
      size_t offset = 0;
      int chunkIndex = 0;
      // Always write at least one row, so an (impossible-in-practice) empty
      // payload still has a data row rather than looking like a missing splat.
      do {
        const size_t len = std::min(GAUSSIAN_SPLAT_CHUNK_BYTES, total - offset);
        sqlite3_reset(dstmt);
        sqlite3_clear_bindings(dstmt);

        if (sqlite3_bind_int(dstmt, 1, splatId) != SQLITE_OK)
          throw std::runtime_error("Failed to bind splat_id: " +
                                   std::string(sqlite3_errmsg(db)));
        if (sqlite3_bind_int(dstmt, 2, chunkIndex) != SQLITE_OK)
          throw std::runtime_error("Failed to bind chunk_index: " +
                                   std::string(sqlite3_errmsg(db)));
        static constexpr uint8_t emptyByte = 0;
        const void *blobPtr =
            len ? static_cast<const void *>(ply.data() + offset)
                : static_cast<const void *>(&emptyByte);
        if (sqlite3_bind_blob64(dstmt, 3, blobPtr,
                                static_cast<sqlite3_uint64>(len),
                                SQLITE_STATIC) != SQLITE_OK)
          throw std::runtime_error(
              "Failed to bind Gaussian splat chunk " +
              std::to_string(chunkIndex) + " (" + std::to_string(len) +
              " bytes): " + std::string(sqlite3_errmsg(db)));

        if (sqlite3_step(dstmt) != SQLITE_DONE)
          throw std::runtime_error("Failed to insert Gaussian splat chunk " +
                                   std::to_string(chunkIndex) + ": " +
                                   std::string(sqlite3_errmsg(db)));

        offset += len;
        ++chunkIndex;
      } while (offset < total);

      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);
    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }

    reusex::info("Stored Gaussian splat '{}': {} Gaussians, SH degree {}, "
                 "{} bytes",
                 name, header.gaussian_count, header.sh_degree, ply.size());
  }

  bool hasGaussianSplat(std::string_view name) const {
    if (!gaussianSplatTablesExist())
      return false;
    const char *sql = "SELECT COUNT(*) FROM gaussian_splats WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0) > 0;
    return false;
  }

  std::vector<std::string> listGaussianSplats() const {
    std::vector<std::string> names;
    if (!gaussianSplatTablesExist())
      return names;

    const char *sql = "SELECT name FROM gaussian_splats ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list Gaussian splats: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const char *n =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      names.emplace_back(n ? n : "");
    }
    return names;
  }

  ProjectDB::GaussianSplatMetadata
  getGaussianSplatMetadata(std::string_view name) const {
    if (!gaussianSplatTablesExist())
      throw std::runtime_error("Gaussian splat not found: " +
                               std::string(name));

    const char *sql = "SELECT name, format, gaussian_count, sh_degree, "
                      "byte_size, created_at, stage, parameters "
                      "FROM gaussian_splats WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error(
          "Failed to prepare Gaussian splat metadata query: " +
          std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Gaussian splat not found: " +
                               std::string(name));

    auto col_text = [&](int col) -> std::string {
      auto *p = reinterpret_cast<const char *>(sqlite3_column_text(stmt, col));
      return p ? std::string(p) : std::string();
    };

    ProjectDB::GaussianSplatMetadata meta;
    meta.name = col_text(0);
    meta.format = col_text(1);
    meta.gaussian_count = static_cast<uint64_t>(sqlite3_column_int64(stmt, 2));
    meta.sh_degree = sqlite3_column_int(stmt, 3);
    meta.byte_size = static_cast<uint64_t>(sqlite3_column_int64(stmt, 4));
    meta.created_at = col_text(5);
    meta.stage = col_text(6);
    meta.parameters = col_text(7);
    return meta;
  }

  std::vector<uint8_t> getGaussianSplatBlob(std::string_view name) const {
    if (!gaussianSplatTablesExist())
      throw std::runtime_error("Gaussian splat not found: " +
                               std::string(name));

    const int splatId = getGaussianSplatId(name);

    const char *sql = "SELECT data FROM gaussian_splat_data "
                      "WHERE splat_id = ? ORDER BY chunk_index;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare Gaussian splat data query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_int(stmt, 1, splatId);

    std::vector<uint8_t> data;
    bool anyChunks = false;
    int rc;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
      anyChunks = true;
      const void *blob = sqlite3_column_blob(stmt, 0);
      const int blobSize = sqlite3_column_bytes(stmt, 0);
      if (blobSize < 0)
        throw std::runtime_error("Negative chunk size for Gaussian splat: " +
                                 std::string(name));
      if (blobSize > 0 && blob) {
        const auto *src = static_cast<const uint8_t *>(blob);
        data.insert(data.end(), src, src + blobSize);
      }
    }
    if (rc != SQLITE_DONE)
      throw std::runtime_error("Failed to read Gaussian splat chunks: " +
                               std::string(sqlite3_errmsg(db)));
    if (!anyChunks)
      throw std::runtime_error("Gaussian splat has no data rows: " +
                               std::string(name));
    return data;
  }

  bool deleteGaussianSplat(std::string_view name) {
    if (!gaussianSplatTablesExist())
      return false;

    const char *sql = "DELETE FROM gaussian_splats WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare Gaussian splat delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete Gaussian splat: " +
                               std::string(sqlite3_errmsg(db)));
    return sqlite3_changes(db) > 0;
  }

  // ── Pipeline log ───────────────────────────────────────────────────

  int logPipelineStart(std::string_view stage, std::string_view paramsJson) {
    const char *sql = R"(
      INSERT INTO pipeline_log (stage, parameters) VALUES (?, ?);
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pipeline log insert");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, stage.data(), static_cast<int>(stage.size()),
                      SQLITE_TRANSIENT);
    if (!paramsJson.empty())
      sqlite3_bind_text(stmt, 2, paramsJson.data(),
                        static_cast<int>(paramsJson.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 2);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to log pipeline start");

    return static_cast<int>(sqlite3_last_insert_rowid(db));
  }

  void logPipelineEnd(int logId, ProjectDB::PipelineOutcome outcome,
                      std::string_view errorMsg) {
    const char *sql = R"(
      UPDATE pipeline_log
      SET finished_at = datetime('now'), status = ?, error_msg = ?
      WHERE id = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare pipeline log update");
    StmtGuard guard(stmt);
    const char *status = [outcome] {
      switch (outcome) {
      case ProjectDB::PipelineOutcome::success:
        return "success";
      case ProjectDB::PipelineOutcome::cancelled:
        return "cancelled";
      case ProjectDB::PipelineOutcome::failed:
      default:
        return "failed";
      }
    }();
    sqlite3_bind_text(stmt, 1, status, -1, SQLITE_STATIC);
    if (!errorMsg.empty())
      sqlite3_bind_text(stmt, 2, errorMsg.data(),
                        static_cast<int>(errorMsg.size()), SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 2);
    sqlite3_bind_int(stmt, 3, logId);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to log pipeline end");
  }

  // ── Material passport operations (unchanged) ──────────────────────

  size_t getNuberOfMaterials() const {
    const char *query = "SELECT COUNT(*) FROM material_passports;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to count materials: " +
                               std::string(sqlite3_errmsg(db)));
    }

    size_t count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      count = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
    }
    sqlite3_finalize(stmt);
    return count;
  }

  class MaterialPassportIterator {
    const Impl &impl;
    std::vector<std::string> document_guids_;
    size_t index = 0;

      public:
    explicit MaterialPassportIterator(const Impl &impl_ref) : impl(impl_ref) {
      // Fetch all document GUIDs (lightweight query)
      const char *query =
          "SELECT document_guid FROM material_passports ORDER BY created_at;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(impl.db, query, -1, &stmt, nullptr) == SQLITE_OK) {
        while (sqlite3_step(stmt) == SQLITE_ROW) {
          const char *guid =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
          if (guid) {
            document_guids_.emplace_back(guid);
          }
        }
        sqlite3_finalize(stmt);
      }
    }

    bool hasNext() const { return index < document_guids_.size(); }

    reusex::core::MaterialPassport next() {
      if (!hasNext()) {
        throw std::out_of_range("No more material passports available");
      }
      return impl.getMaterialPassport(document_guids_[index++]);
    }
  };

  MaterialPassportIterator getMaterialPassports() const {
    return MaterialPassportIterator(*this);
  }

  reusex::core::MaterialPassportMetadata
  fetchPassportMetadata(std::string_view documentGuid) const {
    const char *query = "SELECT document_guid, created_at, revised_at, "
                        "version_number, version_date "
                        "FROM material_passports WHERE document_guid = ?;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare metadata query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    reusex::core::MaterialPassportMetadata metadata;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      // NULL reads as "" (column_text): rows written by hand or before
      // ensure_resource_passport filled every column hold NULLs.
      metadata.document_guid = column_text(stmt, 0);
      metadata.creation_date = column_text(stmt, 1);
      metadata.revision_date = column_text(stmt, 2);
      metadata.version_number = column_text(stmt, 3);
      metadata.version_date = column_text(stmt, 4);
    } else {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Material passport not found: " +
                               std::string(documentGuid));
    }

    sqlite3_finalize(stmt);
    return metadata;
  }

  std::map<std::string, reusex::core::serialization::PropertyValue>
  fetchPropertyValues(std::string_view documentGuid) const {
    const char *query =
        "SELECT pd.leksikon_guid, pd.data_type, ppv.value "
        "FROM material_passports mp "
        "JOIN passport_property_values ppv ON ppv.passport_id = mp.id "
        "JOIN property_definitions pd ON pd.id = ppv.property_id "
        "WHERE mp.document_guid = ? "
        "ORDER BY ppv.sort_order;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare property values query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    std::map<std::string, reusex::core::serialization::PropertyValue>
        property_map;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const char *guid =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
      const char *data_type =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      const void *blob_data = sqlite3_column_blob(stmt, 2);
      int blob_size = sqlite3_column_bytes(stmt, 2);

      if (guid && data_type) {
        auto type = reusex::core::traits::property_type_from_string(data_type);
        if (type) {
          property_map.emplace(guid, reusex::core::serialization::PropertyValue(
                                         blob_data, blob_size, *type));
        }
      }
    }

    sqlite3_finalize(stmt);
    return property_map;
  }

  std::vector<reusex::core::TransactionLogEntry>
  fetchTransactionLog(std::string_view documentGuid) const {
    const char *query = "SELECT entry_type, target_guid, edited_by, edited_at, "
                        "old_value, new_value "
                        "FROM passport_log pl "
                        "JOIN material_passports mp ON pl.passport_id = mp.id "
                        "WHERE mp.document_guid = ? "
                        "ORDER BY pl.edited_at;";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare transaction log query: " +
                               std::string(sqlite3_errmsg(db)));
    }

    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_TRANSIENT);

    std::vector<reusex::core::TransactionLogEntry> entries;

    while (sqlite3_step(stmt) == SQLITE_ROW) {
      reusex::core::TransactionLogEntry entry;

      if (auto type = reusex::core::transaction_type_from_string(
              column_text(stmt, 0))) {
        entry.type = *type;
      }

      entry.guid = column_text(stmt, 1);
      entry.edited_by = column_text(stmt, 2);
      entry.edited_date = column_text(stmt, 3);
      entry.old_value = column_text(stmt, 4);
      entry.new_value = column_text(stmt, 5);

      entries.push_back(std::move(entry));
    }

    sqlite3_finalize(stmt);
    return entries;
  }

  reusex::core::MaterialPassport
  getMaterialPassport(std::string_view documentGuid) const {
    reusex::info("Fetching MaterialPassport: {}", documentGuid);

    // 1. Fetch metadata
    auto metadata = fetchPassportMetadata(documentGuid);

    // 2. Fetch all property values
    auto property_map = fetchPropertyValues(documentGuid);

    // 3. Deserialize into MaterialPassport sections
    reusex::core::MaterialPassport passport;
    passport.metadata = std::move(metadata);

    using namespace reusex::core::serialization;
    Deserializer::deserialize(passport.owner, property_map);
    Deserializer::deserialize(passport.description, property_map);
    Deserializer::deserialize(passport.product, property_map);
    Deserializer::deserialize(passport.certifications, property_map);
    Deserializer::deserialize(passport.dimensions, property_map);
    Deserializer::deserialize(passport.condition, property_map);
    Deserializer::deserialize(passport.pollution, property_map);
    Deserializer::deserialize(passport.environmental, property_map);
    Deserializer::deserialize(passport.fire, property_map);
    Deserializer::deserialize(passport.history, property_map);

    // 4. Fetch transaction log
    passport.transaction_log = fetchTransactionLog(documentGuid);

    reusex::trace("MaterialPassport fetched successfully: {} properties",
                  property_map.size());

    return passport;
  }

  void ensurePropertyDefinitions(
      const reusex::core::traits::PropertyDescriptor *props, size_t count,
      const char *category, sqlite3_stmt *stmt) {
    for (size_t i = 0; i < count; ++i) {
      const auto &prop = props[i];

      if (prop.type == reusex::core::traits::PropertyType::ObjectArray) {
        auto data_type = reusex::core::traits::to_data_type_string(prop.type);

        sqlite3_bind_text(stmt, 1, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, prop.field_name, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 4, category, -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 5, data_type.data(),
                          static_cast<int>(data_type.size()), SQLITE_STATIC);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error("Failed to insert property definition: " +
                                   error);
        }
        sqlite3_reset(stmt);
        continue;
      }

      auto data_type = reusex::core::traits::to_data_type_string(prop.type);

      sqlite3_bind_text(stmt, 1, prop.leksikon_guid, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 2, prop.leksikon_guid, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 3, prop.field_name, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 4, category, -1, SQLITE_STATIC);
      sqlite3_bind_text(stmt, 5, data_type.data(),
                        static_cast<int>(data_type.size()), SQLITE_STATIC);

      if (sqlite3_step(stmt) != SQLITE_DONE) {
        std::string error = sqlite3_errmsg(db);
        sqlite3_finalize(stmt);
        throw std::runtime_error("Failed to insert property definition: " +
                                 error);
      }
      sqlite3_reset(stmt);
    }
  }

  void ensureAllPropertyDefinitions() {
    const char *query = "INSERT OR IGNORE INTO property_definitions "
                        "(id, leksikon_guid, name_en, category, data_type) "
                        "VALUES (?, ?, ?, ?, ?);";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error(
          "Failed to prepare property definition insert: " +
          std::string(sqlite3_errmsg(db)));
    }

    using namespace reusex::core::traits;

    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::Owner>::properties(),
        PropertyTraits<reusex::core::Owner>::property_count(), "Owner", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::ConstructionItemDescription>::properties(),
        PropertyTraits<
            reusex::core::ConstructionItemDescription>::property_count(),
        "Description", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::ProductInformation>::properties(),
        PropertyTraits<reusex::core::ProductInformation>::property_count(),
        "Product", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::Certifications>::properties(),
        PropertyTraits<reusex::core::Certifications>::property_count(),
        "Certifications", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::Dimensions>::properties(),
        PropertyTraits<reusex::core::Dimensions>::property_count(),
        "Dimensions", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::Condition>::properties(),
        PropertyTraits<reusex::core::Condition>::property_count(), "Condition",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::Pollution>::properties(),
        PropertyTraits<reusex::core::Pollution>::property_count(), "Pollution",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::EnvironmentalPotential>::properties(),
        PropertyTraits<reusex::core::EnvironmentalPotential>::property_count(),
        "Environmental", stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::FireProperties>::properties(),
        PropertyTraits<reusex::core::FireProperties>::property_count(), "Fire",
        stmt);
    ensurePropertyDefinitions(
        PropertyTraits<reusex::core::History>::properties(),
        PropertyTraits<reusex::core::History>::property_count(), "History",
        stmt);

    sqlite3_finalize(stmt);
  }

  void addMaterialPassport(const reusex::core::MaterialPassport &passport,
                           std::string_view projectId) {
    addMaterialPassport(passport, projectId, passport.metadata.document_guid);
  }

  void addMaterialPassport(const reusex::core::MaterialPassport &passport,
                           std::string_view projectId, std::string_view id) {
    reusex::info("Adding MaterialPassport: {}",
                 passport.metadata.document_guid);

    // Begin transaction for atomicity
    execOrThrow("BEGIN TRANSACTION;");

    try {
      // 0. Ensure property_definitions are populated
      ensureAllPropertyDefinitions();

      // 0b. Ensure project row exists (skip if projectId is empty → NULL)
      if (!projectId.empty()) {
        const char *upsert_project =
            "INSERT OR IGNORE INTO projects (id) VALUES (?);";
        sqlite3_stmt *proj_stmt;
        if (sqlite3_prepare_v2(db, upsert_project, -1, &proj_stmt, nullptr) !=
            SQLITE_OK) {
          throw std::runtime_error("Failed to prepare project insert: " +
                                   std::string(sqlite3_errmsg(db)));
        }
        sqlite3_bind_text(proj_stmt, 1, projectId.data(), projectId.size(),
                          SQLITE_TRANSIENT);
        if (sqlite3_step(proj_stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(proj_stmt);
          throw std::runtime_error("Failed to insert project: " + error);
        }
        sqlite3_finalize(proj_stmt);
      }

      // 1. Delete existing passport if it exists (for updates)
      std::string passport_id{id};
      const auto &doc_guid = passport.metadata.document_guid;

      // Delete related data (property values and log entries)
      const char *delete_values_query =
          "DELETE FROM passport_property_values WHERE passport_id = "
          "(SELECT id FROM material_passports WHERE document_guid = ?);";
      const char *delete_log_query =
          "DELETE FROM passport_log WHERE passport_id = "
          "(SELECT id FROM material_passports WHERE document_guid = ?);";
      const char *delete_passport_query =
          "DELETE FROM material_passports WHERE document_guid = ?;";

      sqlite3_stmt *del_stmt;
      // Delete property values
      if (sqlite3_prepare_v2(db, delete_values_query, -1, &del_stmt, nullptr) ==
          SQLITE_OK) {
        sqlite3_bind_text(del_stmt, 1, doc_guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(del_stmt);
        sqlite3_finalize(del_stmt);
      }
      // Delete log entries
      if (sqlite3_prepare_v2(db, delete_log_query, -1, &del_stmt, nullptr) ==
          SQLITE_OK) {
        sqlite3_bind_text(del_stmt, 1, doc_guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(del_stmt);
        sqlite3_finalize(del_stmt);
      }
      // Delete passport
      if (sqlite3_prepare_v2(db, delete_passport_query, -1, &del_stmt,
                             nullptr) == SQLITE_OK) {
        sqlite3_bind_text(del_stmt, 1, doc_guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_step(del_stmt);
        sqlite3_finalize(del_stmt);
      }

      // 2. Insert material_passports row
      const char *insert_passport_query =
          "INSERT INTO material_passports (id, project_id, document_guid, "
          "created_at, revised_at, version_number, version_date) "
          "VALUES (?, ?, ?, ?, ?, ?, ?);";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, insert_passport_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error(
            "Failed to prepare insert passport statement: " +
            std::string(sqlite3_errmsg(db)));
      }

      sqlite3_bind_text(stmt, 1, passport_id.c_str(), -1, SQLITE_TRANSIENT);
      if (projectId.empty()) {
        sqlite3_bind_null(stmt, 2);
      } else {
        sqlite3_bind_text(stmt, 2, projectId.data(), projectId.size(),
                          SQLITE_TRANSIENT);
      }
      sqlite3_bind_text(stmt, 3, passport.metadata.document_guid.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 4, passport.metadata.creation_date.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 5, passport.metadata.revision_date.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 6, passport.metadata.version_number.c_str(), -1,
                        SQLITE_TRANSIENT);
      sqlite3_bind_text(stmt, 7, passport.metadata.version_date.c_str(), -1,
                        SQLITE_TRANSIENT);

      if (sqlite3_step(stmt) != SQLITE_DONE) {
        std::string error = sqlite3_errmsg(db);
        sqlite3_finalize(stmt);
        throw std::runtime_error("Failed to insert passport: " + error);
      }
      sqlite3_finalize(stmt);

      // 2. Serialize all sections to property values
      using namespace reusex::core::serialization;
      std::map<std::string, PropertyValue> all_values;

      auto merge_values = [&all_values](auto &&values) {
        all_values.insert(values.begin(), values.end());
      };

      merge_values(Serializer::serialize(passport.owner));
      merge_values(Serializer::serialize(passport.description));
      merge_values(Serializer::serialize(passport.product));
      merge_values(Serializer::serialize(passport.certifications));
      merge_values(Serializer::serialize(passport.dimensions));
      merge_values(Serializer::serialize(passport.condition));
      merge_values(Serializer::serialize(passport.pollution));
      merge_values(Serializer::serialize(passport.environmental));
      merge_values(Serializer::serialize(passport.fire));
      merge_values(Serializer::serialize(passport.history));

      // 3. Insert passport_property_values rows
      const char *insert_value_query =
          "INSERT INTO passport_property_values (id, passport_id, property_id, "
          "leksikon_guid, sort_order, value) VALUES (?, ?, ?, ?, ?, ?);";

      if (sqlite3_prepare_v2(db, insert_value_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Failed to prepare insert value statement: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      int sort_order = 0;
      for (const auto &[guid, value] : all_values) {
        std::string value_id = passport_id + "_" + guid;

        sqlite3_bind_text(stmt, 1, value_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, passport_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 3, guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_int(stmt, 5, sort_order++);
        sqlite3_bind_blob(stmt, 6, value.blob().data(), value.blob().size(),
                          SQLITE_TRANSIENT);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error("Failed to insert property value: " + error);
        }

        sqlite3_reset(stmt);
      }
      sqlite3_finalize(stmt);

      // 4. Insert transaction log entries
      const char *insert_log_query =
          "INSERT INTO passport_log (id, passport_id, entry_type, target_guid, "
          "edited_by, edited_at, old_value, new_value) VALUES (?, ?, ?, ?, ?, "
          "?, ?, ?);";

      if (sqlite3_prepare_v2(db, insert_log_query, -1, &stmt, nullptr) !=
          SQLITE_OK) {
        throw std::runtime_error("Failed to prepare insert log statement: " +
                                 std::string(sqlite3_errmsg(db)));
      }

      int log_idx = 0;
      for (const auto &entry : passport.transaction_log) {
        std::string log_id = passport_id + "_log_" + std::to_string(log_idx++);

        sqlite3_bind_text(stmt, 1, log_id.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 2, passport_id.c_str(), -1, SQLITE_TRANSIENT);

        std::string type_str(reusex::core::to_string(entry.type));
        sqlite3_bind_text(stmt, 3, type_str.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 4, entry.guid.c_str(), -1, SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 5, entry.edited_by.c_str(), -1,
                          SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 6, entry.edited_date.c_str(), -1,
                          SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 7, entry.old_value.c_str(), -1,
                          SQLITE_TRANSIENT);
        sqlite3_bind_text(stmt, 8, entry.new_value.c_str(), -1,
                          SQLITE_TRANSIENT);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
          std::string error = sqlite3_errmsg(db);
          sqlite3_finalize(stmt);
          throw std::runtime_error("Failed to insert log entry: " + error);
        }

        sqlite3_reset(stmt);
      }
      sqlite3_finalize(stmt);

      // Commit transaction
      sqlite3_exec(db, "COMMIT;", nullptr, nullptr, nullptr);

      reusex::info("MaterialPassport added successfully: {} properties",
                   all_values.size());

    } catch (...) {
      sqlite3_exec(db, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw;
    }
  }

  // ── Pipeline Log ───────────────────────────────────────────────────

  std::vector<ProjectDB::PipelineLogEntry> getPipelineLog(int limit) const {
    std::string sql = "SELECT id, stage, started_at, finished_at, parameters, "
                      "status, error_msg FROM pipeline_log ORDER BY id DESC";
    if (limit > 0) {
      sql += " LIMIT " + std::to_string(limit);
    }
    sql += ";";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to query pipeline log: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    std::vector<ProjectDB::PipelineLogEntry> entries;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      ProjectDB::PipelineLogEntry entry;
      entry.id = sqlite3_column_int(stmt, 0);

      const char *stage =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
      entry.stage = stage ? stage : "";

      const char *started =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
      entry.started_at = started ? started : "";

      const char *finished =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
      entry.finished_at = finished ? finished : "";

      const char *params =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
      entry.parameters = params ? params : "";

      const char *status =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
      entry.status = status ? status : "";

      const char *error =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 6));
      entry.error_msg = error ? error : "";

      entries.push_back(std::move(entry));
    }
    return entries;
  }

  // ── Project Summary ────────────────────────────────────────────────

  // ── Building component CRUD ──────────────────────────────────────

  void saveComponentRecord(const core::ComponentRecord &c) {
    // Generate a stable guid on first save; preserved across updates because
    // guid is intentionally omitted from the ON CONFLICT update clause.
    std::string guid = c.guid.empty() ? generate_component_guid() : c.guid;

    const char *upsert = R"(
      INSERT INTO building_components
        (name, guid, type, vertex_data, vertex_count, plane, parent_id, confidence, metadata, notes)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
      ON CONFLICT(name) DO UPDATE SET
        type = excluded.type,
        vertex_data = excluded.vertex_data,
        vertex_count = excluded.vertex_count,
        plane = excluded.plane,
        parent_id = excluded.parent_id,
        confidence = excluded.confidence,
        metadata = excluded.metadata,
        notes = excluded.notes,
        created_at = datetime('now');
    )";

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, upsert, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare building component upsert: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, c.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, guid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 3, c.type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 4, c.vertex_data.data(),
                      static_cast<int>(c.vertex_data.size()), SQLITE_TRANSIENT);
    // vertex_count is derived: the blob holds packed float64 xyz triples.
    sqlite3_bind_int(
        stmt, 5, static_cast<int>(c.vertex_data.size() / (3 * sizeof(double))));

    // Plane: 4 * sizeof(double) = 32 bytes
    sqlite3_bind_blob(stmt, 6, c.plane.data(),
                      static_cast<int>(4 * sizeof(double)), SQLITE_TRANSIENT);

    sqlite3_bind_int(stmt, 7, c.parent_id);
    sqlite3_bind_double(stmt, 8, c.confidence);

    if (!c.metadata.empty())
      sqlite3_bind_text(stmt, 9, c.metadata.c_str(), -1, SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 9);

    if (!c.notes.empty())
      sqlite3_bind_text(stmt, 10, c.notes.c_str(), -1, SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 10);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to upsert building component: " +
                               std::string(sqlite3_errmsg(db)));
  }

  // Update an existing component's mutable fields, matched by its immutable
  // guid (so renames don't create a new row). Geometry is left untouched.
  void updateComponentRecordByGuid(const core::ComponentRecord &c) {
    const char *sql = R"(
      UPDATE building_components SET
        name = ?, type = ?, parent_id = ?, confidence = ?,
        metadata = ?, notes = ?
      WHERE guid = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare component update: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, c.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, c.type.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(stmt, 3, c.parent_id);
    sqlite3_bind_double(stmt, 4, c.confidence);
    if (!c.metadata.empty())
      sqlite3_bind_text(stmt, 5, c.metadata.c_str(), -1, SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 5);
    if (!c.notes.empty())
      sqlite3_bind_text(stmt, 6, c.notes.c_str(), -1, SQLITE_TRANSIENT);
    else
      sqlite3_bind_null(stmt, 6);
    sqlite3_bind_text(stmt, 7, c.guid.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to update building component: " +
                               std::string(sqlite3_errmsg(db)));
    if (sqlite3_changes(db) == 0)
      throw std::runtime_error("No building component with guid: " + c.guid);
  }

  core::ComponentRecord getComponentRecord(std::string_view name) const {
    const char *sql = R"(
      SELECT name, type, vertex_data, vertex_count, plane,
             parent_id, confidence, metadata, notes, guid
      FROM building_components WHERE name = ?;
    )";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare building component query: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("Building component not found: " +
                               std::string(name));

    core::ComponentRecord c;
    c.name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));

    const char *typeStr =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    if (typeStr)
      c.type = typeStr;

    // Vertex data (the vertex_count column is redundant with the blob size)
    const void *vertBlob = sqlite3_column_blob(stmt, 2);
    int vertSize = sqlite3_column_bytes(stmt, 2);
    if (vertBlob && vertSize > 0) {
      const auto *bytes = static_cast<const uint8_t *>(vertBlob);
      c.vertex_data.assign(bytes, bytes + vertSize);
    }

    // Plane
    const void *planeBlob = sqlite3_column_blob(stmt, 4);
    int planeSize = sqlite3_column_bytes(stmt, 4);
    if (planeBlob && planeSize == static_cast<int>(4 * sizeof(double)))
      std::memcpy(c.plane.data(), planeBlob, 4 * sizeof(double));

    c.parent_id = sqlite3_column_int(stmt, 5);
    c.confidence = sqlite3_column_double(stmt, 6);

    // Metadata JSON (opaque to core — interpreted by the geometry layer)
    const char *metaText =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 7));
    if (metaText)
      c.metadata = metaText;

    // Notes
    const char *notesText =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 8));
    if (notesText)
      c.notes = notesText;

    // Guid
    const char *guidText =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 9));
    if (guidText)
      c.guid = guidText;

    return c;
  }

  bool hasBuildingComponent(std::string_view name) const {
    const char *sql =
        "SELECT 1 FROM building_components WHERE name = ? LIMIT 1;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return false;
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    return sqlite3_step(stmt) == SQLITE_ROW;
  }

  void deleteBuildingComponent(std::string_view name) {
    const char *sql = "DELETE FROM building_components WHERE name = ?;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to prepare building component delete: " +
                               std::string(sqlite3_errmsg(db)));
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, name.data(), static_cast<int>(name.size()),
                      SQLITE_TRANSIENT);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("Failed to delete building component: " +
                               std::string(sqlite3_errmsg(db)));
  }

  std::vector<std::string> listBuildingComponents() const {
    const char *sql = "SELECT name FROM building_components ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list building components");
    StmtGuard guard(stmt);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  std::vector<std::string> listBuildingComponents(std::string_view type) const {
    const char *sql =
        "SELECT name FROM building_components WHERE type = ? ORDER BY id;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("Failed to list building components by type");
    StmtGuard guard(stmt);
    sqlite3_bind_text(stmt, 1, type.data(), static_cast<int>(type.size()),
                      SQLITE_TRANSIENT);

    std::vector<std::string> names;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      names.emplace_back(
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
    }
    return names;
  }

  int buildingComponentCount() const {
    const char *sql = "SELECT COUNT(*) FROM building_components;";
    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      return 0;
    StmtGuard guard(stmt);
    if (sqlite3_step(stmt) == SQLITE_ROW)
      return sqlite3_column_int(stmt, 0);
    return 0;
  }

  ProjectDB::ProjectSummary getProjectSummary() const {
    ProjectDB::ProjectSummary summary;
    summary.path = dbPath;
    summary.schema_version = getCurrentSchemaVersion();

    // Query projects
    {
      const char *sql =
          "SELECT id, name, building_address, year_of_construction, "
          "survey_date, survey_organisation, notes "
          "FROM projects ORDER BY id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query projects: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      while (sqlite3_step(stmt) == SQLITE_ROW) {
        ProjectDB::ProjectSummary::ProjectInfo project;

        // ID is required, should never be NULL
        const char *id_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        project.id = id_str ? id_str : "";

        // Optional text fields - check for NULL
        const char *name_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
        project.name = name_str ? name_str : "";

        const char *address_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
        project.building_address = address_str ? address_str : "";

        project.year_of_construction = sqlite3_column_int(stmt, 3);

        const char *survey_date_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
        project.survey_date = survey_date_str ? survey_date_str : "";

        const char *survey_org_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
        project.survey_organisation = survey_org_str ? survey_org_str : "";

        const char *notes_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 6));
        project.notes = notes_str ? notes_str : "";

        summary.projects.push_back(std::move(project));
      }
    }

    // Query point clouds
    {
      const char *sql =
          "SELECT name, point_type, point_count, width, height, parameters "
          "FROM point_clouds ORDER BY id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query point clouds: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      while (sqlite3_step(stmt) == SQLITE_ROW) {
        ProjectDB::ProjectSummary::CloudInfo cloud;
        cloud.name =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        cloud.type =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
        cloud.point_count = static_cast<size_t>(sqlite3_column_int64(stmt, 2));
        cloud.width = static_cast<size_t>(sqlite3_column_int(stmt, 3));
        cloud.height = static_cast<size_t>(sqlite3_column_int(stmt, 4));
        cloud.organized = cloud.height > 1;

        // Parse storage_order from parameters JSON (absent key = sequential).
        const char *params_text =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
        if (params_text && *params_text) {
          try {
            auto j = nlohmann::json::parse(params_text);
            if (j.contains("storage_order"))
              cloud.storage_order = j["storage_order"].get<std::string>();
          } catch (...) {
          }
        }

        // Load label definitions if this is a Label cloud
        if (cloud.type == "Label") {
          cloud.labels = getLabelDefinitions(cloud.name);
        }

        summary.clouds.push_back(std::move(cloud));
      }
    }

    // Query meshes
    {
      const char *sql =
          "SELECT name, vertex_count, face_count FROM meshes ORDER BY id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query meshes: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      while (sqlite3_step(stmt) == SQLITE_ROW) {
        ProjectDB::ProjectSummary::MeshInfo info;
        info.name =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        info.vertex_count = sqlite3_column_int(stmt, 1);
        info.face_count = sqlite3_column_int(stmt, 2);
        summary.meshes.push_back(std::move(info));
      }
    }

    // Query Gaussian splats. Metadata only — the payload is chunked precisely
    // so nobody has to load it to describe it. Guarded on the tables because
    // a read-only open of a pre-v12 project never migrates, and `rux info`
    // must still work there.
    if (tableExists("gaussian_splats")) {
      const char *sql = "SELECT name, format, gaussian_count, sh_degree, "
                        "byte_size, created_at, stage, parameters "
                        "FROM gaussian_splats ORDER BY id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
        StmtGuard guard(stmt);
        auto col_text = [&](int col) -> std::string {
          auto *p =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, col));
          return p ? std::string(p) : std::string();
        };
        while (sqlite3_step(stmt) == SQLITE_ROW) {
          ProjectDB::GaussianSplatMetadata info;
          info.name = col_text(0);
          info.format = col_text(1);
          info.gaussian_count =
              static_cast<uint64_t>(sqlite3_column_int64(stmt, 2));
          info.sh_degree = sqlite3_column_int(stmt, 3);
          info.byte_size = static_cast<uint64_t>(sqlite3_column_int64(stmt, 4));
          info.created_at = col_text(5);
          info.stage = col_text(6);
          info.parameters = col_text(7);
          summary.gaussian_splats.push_back(std::move(info));
        }
      }
    }

    // Query sensor frames
    {
      // Get total count
      const char *count_sql = "SELECT COUNT(*) FROM sensor_frames;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, count_sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to count sensor frames: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard count_guard(stmt);

      if (sqlite3_step(stmt) == SQLITE_ROW) {
        summary.sensor_frames.total_count = sqlite3_column_int(stmt, 0);
      } else {
        summary.sensor_frames.total_count = 0;
      }
    }

    // Get dimensions from first frame if any exist
    if (summary.sensor_frames.total_count > 0) {
      const char *dim_sql = "SELECT camera_model FROM sensor_frames LIMIT 1;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, dim_sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query sensor frame dimensions: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard dim_guard(stmt);

      if (sqlite3_step(stmt) == SQLITE_ROW) {
        const char *json_text =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        if (json_text) {
          try {
            auto intrinsics =
                reusex::core::SensorIntrinsics::from_json(json_text);
            summary.sensor_frames.width = intrinsics.width;
            summary.sensor_frames.height = intrinsics.height;
          } catch (...) {
            summary.sensor_frames.width = 0;
            summary.sensor_frames.height = 0;
          }
        } else {
          summary.sensor_frames.width = 0;
          summary.sensor_frames.height = 0;
        }
      } else {
        summary.sensor_frames.width = 0;
        summary.sensor_frames.height = 0;
      }
    } else {
      summary.sensor_frames.width = 0;
      summary.sensor_frames.height = 0;
    }

    // Get segmented count
    {
      const char *seg_sql = "SELECT COUNT(*) FROM segmentation_images;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, seg_sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to count segmentation images: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard seg_guard(stmt);

      if (sqlite3_step(stmt) == SQLITE_ROW) {
        summary.sensor_frames.segmented_count = sqlite3_column_int(stmt, 0);
      } else {
        summary.sensor_frames.segmented_count = 0;
      }
    }

    // Per-scan breakdown (#129); left empty when the scans table is absent.
    if (tableExists("scans")) {
      const char *scan_sql = "SELECT s.id, s.source_path, s.imported_at, "
                             "COUNT(f.node_id) as frame_count "
                             "FROM scans s "
                             "LEFT JOIN sensor_frames f ON f.scan_id = s.id "
                             "GROUP BY s.id "
                             "ORDER BY s.id;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, scan_sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query scan breakdown: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard scan_guard(stmt);
      while (sqlite3_step(stmt) == SQLITE_ROW) {
        ProjectDB::ProjectSummary::SensorFrameInfo::ScanInfo info;
        info.scan_id = sqlite3_column_int(stmt, 0);
        if (auto *t =
                reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
          info.source_path = t;
        if (auto *t =
                reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2)))
          info.imported_at = t;
        info.frame_count = sqlite3_column_int(stmt, 3);
        summary.sensor_frames.scans.push_back(std::move(info));
      }
    }

    // Query panoramic images
    if (tableExists("panoramic_images")) {
      const char *count_sql = "SELECT COUNT(*) FROM panoramic_images;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, count_sql, -1, &stmt, nullptr) == SQLITE_OK) {
        StmtGuard guard(stmt);
        if (sqlite3_step(stmt) == SQLITE_ROW)
          summary.panoramic_images.total_count = sqlite3_column_int(stmt, 0);
      }

      const char *matched_sql =
          "SELECT COUNT(*) FROM panoramic_images WHERE node_id IS NOT NULL;";
      sqlite3_stmt *mstmt;
      if (sqlite3_prepare_v2(db, matched_sql, -1, &mstmt, nullptr) ==
          SQLITE_OK) {
        StmtGuard guard(mstmt);
        if (sqlite3_step(mstmt) == SQLITE_ROW)
          summary.panoramic_images.matched_count = sqlite3_column_int(mstmt, 0);
      }
    }

    // Query material passports
    {
      const char *sql = "SELECT "
                        "  mp.id, "
                        "  mp.document_guid, "
                        "  mp.created_at, "
                        "  mp.version_number, "
                        "  (SELECT COUNT(*) FROM passport_property_values ppv "
                        "   WHERE ppv.passport_id = mp.id) as property_count "
                        "FROM material_passports mp "
                        "ORDER BY mp.created_at;";

      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
        throw std::runtime_error("Failed to query material passports: " +
                                 std::string(sqlite3_errmsg(db)));
      StmtGuard guard(stmt);

      while (sqlite3_step(stmt) == SQLITE_ROW) {
        ProjectSummary::MaterialInfo info;

        // Extract ID (column 0)
        const char *id_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
        info.id = id_str ? id_str : "";

        // Extract GUID (column 1)
        const char *guid_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
        info.guid = guid_str ? guid_str : "";

        // Extract created_at (column 2)
        const char *created_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
        info.created_at = created_str ? created_str : "";

        // Extract version_number (column 3)
        const char *version_str =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 3));
        info.version_number = version_str ? version_str : "";

        // Extract property_count (column 4)
        info.property_count = sqlite3_column_int(stmt, 4);

        summary.materials.push_back(std::move(info));
      }
    }

    // Query building components
    {
      const char *sql =
          "SELECT type, COUNT(*) FROM building_components GROUP BY type;";
      sqlite3_stmt *stmt;
      if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) == SQLITE_OK) {
        StmtGuard guard(stmt);
        while (sqlite3_step(stmt) == SQLITE_ROW) {
          const char *type_str =
              reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
          int count = sqlite3_column_int(stmt, 1);
          if (type_str) {
            summary.components.count_by_type[type_str] = count;
            summary.components.total_count += count;
          }
        }
      }
    }

    return summary;
  }
};

// ── Public Interface Implementation ─────────────────────────────────────

ProjectDB::ProjectDB(std::filesystem::path dbPath, bool readOnly)
    : ProjectDB(std::move(dbPath),
                OpenOptions{readOnly, hardened_by_default()}) {}

ProjectDB::ProjectDB(std::filesystem::path dbPath, OpenOptions options)
    : impl_(std::make_unique<Impl>(std::move(dbPath), options.read_only,
                                   options.hardened)) {
  impl_->validateSchema();
}

void ProjectDB::set_hardened_by_default(bool on) noexcept {
  g_hardened_by_default.store(on);
}

bool ProjectDB::hardened_by_default() noexcept {
  return g_hardened_by_default.load();
}

ProjectDB::IntegrityReport
ProjectDB::check_integrity(const std::filesystem::path &file) {
  using Kind = IntegrityReport::Kind;
  sqlite3 *db = nullptr;
  const int rc = sqlite3_open_v2(file.string().c_str(), &db,
                                 SQLITE_OPEN_READONLY, nullptr);
  struct Closer {
    sqlite3 *db;
    ~Closer() { sqlite3_close_v2(db); }
  } closer{db}; // after the open: sqlite3_open_v2 sets db even on failure
  if (rc != SQLITE_OK)
    return {Kind::unreadable, db ? sqlite3_errmsg(db) : "cannot open it"};
  try {
    harden_connection(db);
  } catch (const std::exception &e) {
    return {Kind::unreadable, e.what()};
  }
  sqlite3_busy_timeout(db, Impl::BUSY_TIMEOUT_MS);

  // quick_check is O(N) and skips only the index-content cross-check of
  // integrity_check; it catches the malformed b-tree pages that matter here.
  // It answers one row "ok", or up to 100 rows naming problems.
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, "PRAGMA quick_check;", -1, &stmt, nullptr) !=
      SQLITE_OK)
    return {Kind::corrupt, sqlite3_errmsg(db)};
  std::string problems;
  int step = SQLITE_ROW;
  for (int rows = 0; (step = sqlite3_step(stmt)) == SQLITE_ROW && rows < 5;
       ++rows) {
    const auto *text =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    const std::string row = text ? text : "";
    if (row == "ok")
      continue;
    if (!problems.empty())
      problems += "; ";
    problems += row;
  }
  const bool step_failed = step != SQLITE_ROW && step != SQLITE_DONE;
  const std::string step_error = step_failed ? sqlite3_errmsg(db) : "";
  sqlite3_finalize(stmt);
  if (step_failed)
    return {Kind::corrupt, step_error};
  if (!problems.empty())
    return {Kind::corrupt, problems};

  if (sqlite3_prepare_v2(db,
                         "SELECT type, name FROM sqlite_master WHERE type IN "
                         "('trigger', 'view') ORDER BY type, name LIMIT 5;",
                         -1, &stmt, nullptr) != SQLITE_OK)
    return {Kind::corrupt, sqlite3_errmsg(db)};
  std::string found;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const auto *type =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    const auto *name =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    if (!found.empty())
      found += ", ";
    found += std::string(type ? type : "?") + " '" + (name ? name : "") + "'";
  }
  sqlite3_finalize(stmt);
  if (!found.empty())
    return {Kind::executable_schema, found};
  return {};
}

ProjectDB::~ProjectDB() = default;

ProjectDB::ProjectDB(ProjectDB &&) noexcept = default;
ProjectDB &ProjectDB::operator=(ProjectDB &&) noexcept = default;

bool ProjectDB::is_open() const noexcept {
  return impl_ && impl_->db != nullptr;
}

const std::filesystem::path &ProjectDB::path() const noexcept {
  return impl_->dbPath;
}

int ProjectDB::schema_version() const {
  return impl_->getCurrentSchemaVersion();
}

void ProjectDB::validate_schema() const { impl_->validateSchema(); }

bool ProjectDB::is_read_only() const noexcept { return impl_->readOnly; }

// --- Sensor Frame Operations ---

void ProjectDB::save_sensor_frame(int nodeId, const cv::Mat &colorImage) {
  impl_->checkWritable();
  impl_->saveSensorFrame(nodeId, colorImage);
}

void ProjectDB::save_sensor_frame(int nodeId, const cv::Mat &color,
                                  const cv::Mat &depth,
                                  const cv::Mat &confidence,
                                  const std::array<double, 16> &worldPose,
                                  const core::SensorIntrinsics &intrinsics,
                                  double timestamp, int scan_id) {
  impl_->checkWritable();
  impl_->saveSensorFrameFull(nodeId, color, depth, confidence, worldPose,
                             intrinsics, timestamp, scan_id);
}

void ProjectDB::update_sensor_frame_pose(
    int nodeId, const std::array<double, 16> &worldPose) {
  impl_->checkWritable();
  impl_->updateSensorFramePose(nodeId, worldPose);
}

std::vector<int> ProjectDB::sensor_frame_ids() const {
  return impl_->getSensorFrameIds();
}

std::vector<std::pair<int, int>> ProjectDB::sensor_frame_ids_with_scan() const {
  return impl_->getSensorFrameIdsWithScan();
}

cv::Mat ProjectDB::sensor_frame_image(int nodeId) const {
  return impl_->getSensorFrameImage(nodeId);
}

cv::Mat ProjectDB::sensor_frame_depth(int nodeId) const {
  return impl_->getSensorFrameDepth(nodeId);
}

cv::Mat ProjectDB::sensor_frame_confidence(int nodeId) const {
  return impl_->getSensorFrameConfidence(nodeId);
}

std::array<double, 16> ProjectDB::sensor_frame_pose(int nodeId) const {
  return impl_->getSensorFramePose(nodeId);
}

bool ProjectDB::has_sensor_frame_pose(int nodeId) const {
  return impl_->hasSensorFramePose(nodeId);
}

void ProjectDB::set_initial_pose(int nodeId,
                                 const std::array<double, 16> &pose) {
  impl_->setInitialPose(nodeId, pose);
}
std::optional<std::array<double, 16>>
ProjectDB::initial_pose(int nodeId) const {
  return impl_->initialPose(nodeId);
}
bool ProjectDB::has_initial_pose(int nodeId) const {
  return impl_->initialPose(nodeId).has_value();
}
std::optional<std::string> ProjectDB::setting(std::string_view key) const {
  return impl_->getSetting(key);
}
void ProjectDB::set_setting(std::string_view key, std::string_view value) {
  impl_->putSetting(key, value);
}

std::string ProjectDB::project_guid() const { return impl_->projectGuid(); }

size_t ProjectDB::save_storeys(const std::vector<core::StoreyRecord> &storeys,
                               int run_id) {
  return impl_->saveStoreys(storeys, run_id);
}

std::vector<core::StoreyRecord> ProjectDB::storeys() const {
  return impl_->listStoreys();
}

std::optional<core::StoreyRecord>
ProjectDB::storey_by_guid(std::string_view guid) const {
  return impl_->storeyByGuid(guid);
}

bool ProjectDB::has_storeys() const { return impl_->hasStoreys(); }

void ProjectDB::delete_storeys() { impl_->deleteStoreys(); }

size_t ProjectDB::save_plane_classes(
    std::string_view cloud, const std::vector<core::PlaneClassRecord> &classes,
    int run_id) {
  return impl_->savePlaneClasses(cloud, classes, run_id);
}

std::vector<core::PlaneClassRecord>
ProjectDB::plane_classes(std::string_view cloud) const {
  return impl_->listPlaneClasses(cloud);
}

bool ProjectDB::has_plane_classes(std::string_view cloud) const {
  return impl_->hasPlaneClasses(cloud);
}

void ProjectDB::delete_plane_classes(std::string_view cloud) {
  impl_->deletePlaneClasses(cloud);
}

size_t ProjectDB::save_walls(const std::vector<core::WallRecord> &walls,
                             const std::vector<core::WallJoinRecord> &joins,
                             int run_id) {
  return impl_->saveWalls(walls, joins, run_id);
}

std::vector<core::WallRecord> ProjectDB::walls() const {
  return impl_->listWalls();
}

std::optional<core::WallRecord>
ProjectDB::wall_by_guid(std::string_view guid) const {
  return impl_->wallByGuid(guid);
}

bool ProjectDB::has_walls() const { return impl_->hasWalls(); }

std::vector<core::WallJoinRecord> ProjectDB::wall_joins() const {
  return impl_->listWallJoins();
}

void ProjectDB::delete_walls() { impl_->deleteWalls(); }

size_t
ProjectDB::save_openings(const std::vector<core::OpeningRecord> &openings,
                         int run_id) {
  return impl_->saveOpenings(openings, run_id);
}

std::vector<core::OpeningRecord> ProjectDB::openings() const {
  return impl_->listOpenings();
}

std::optional<core::OpeningRecord>
ProjectDB::opening_by_guid(std::string_view guid) const {
  return impl_->openingByGuid(guid);
}

bool ProjectDB::has_openings() const { return impl_->hasOpenings(); }

void ProjectDB::delete_openings() { impl_->deleteOpenings(); }

size_t
ProjectDB::save_spaces(const std::vector<core::SpaceRecord> &spaces,
                       const std::vector<core::SpaceBoundaryRecord> &boundaries,
                       int run_id) {
  return impl_->saveSpaces(spaces, boundaries, run_id);
}

std::vector<core::SpaceRecord> ProjectDB::spaces() const {
  return impl_->listSpaces();
}

std::optional<core::SpaceRecord>
ProjectDB::space_by_guid(std::string_view guid) const {
  return impl_->spaceByGuid(guid);
}

bool ProjectDB::has_spaces() const { return impl_->hasSpaces(); }

std::vector<core::SpaceBoundaryRecord> ProjectDB::space_boundaries() const {
  return impl_->listSpaceBoundaries();
}

void ProjectDB::delete_spaces() { impl_->deleteSpaces(); }

core::SensorIntrinsics ProjectDB::sensor_frame_intrinsics(int nodeId) const {
  return impl_->getSensorFrameIntrinsics(nodeId);
}

bool ProjectDB::has_sensor_frame(int nodeId) const {
  return impl_->hasSensorFrame(nodeId);
}

double ProjectDB::sensor_frame_timestamp(int nodeId) const {
  return impl_->getSensorFrameTimestamp(nodeId);
}

int ProjectDB::nearest_sensor_frame_by_timestamp(double timestamp) const {
  return impl_->nearestSensorFrameByTimestamp(timestamp);
}

// --- Panoramic Image Operations ---

void ProjectDB::save_panoramic_image(const std::string &filename,
                                     const std::vector<uint8_t> &jpeg_data,
                                     double timestamp, int nodeId) {
  impl_->checkWritable();
  impl_->savePanoramicImage(filename, jpeg_data, timestamp, nodeId);
}

cv::Mat ProjectDB::panoramic_image(int id) const {
  return impl_->getPanoramicImageById(id);
}

cv::Mat ProjectDB::panoramic_image(std::string_view filename) const {
  return impl_->getPanoramicImageByFilename(filename);
}

bool ProjectDB::has_panoramic_image(std::string_view filename) const {
  return impl_->hasPanoramicImage(filename);
}

bool ProjectDB::has_panoramic_image(int id) const {
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "SELECT 1 FROM panoramic_images WHERE id = ?;",
      "has_panoramic_image");
  StmtGuard guard(stmt);
  sqlite3_bind_int(stmt, 1, id);
  return sqlite3_step(stmt) == SQLITE_ROW;
}

void ProjectDB::delete_panoramic_image(int id) {
  impl_->checkWritable();
  impl_->deletePanoramicImageById(id);
}

void ProjectDB::delete_panoramic_image(std::string_view filename) {
  impl_->checkWritable();
  impl_->deletePanoramicImageByFilename(filename);
}

std::vector<ProjectDB::PanoramicImage>
ProjectDB::list_panoramic_images() const {
  return impl_->listPanoramicImages();
}

int ProjectDB::panoramic_image_count() const {
  return impl_->panoramicImageCount();
}

void ProjectDB::save_panorama_pose(int id, const std::array<double, 16> &pose,
                                   int inliers, double rms) {
  impl_->checkWritable();
  impl_->savePanoramaPose(id, pose, inliers, rms);
}

bool ProjectDB::has_panorama_segmentation(int panoId) const {
  return impl_->hasPanoramaSegmentation(panoId);
}

cv::Mat ProjectDB::panorama_segmentation(int panoId) const {
  return impl_->getPanoramaSegmentation(panoId);
}

void ProjectDB::save_panorama_segmentation(int panoId, const cv::Mat &labels) {
  impl_->checkWritable();
  impl_->savePanoramaSegmentation(panoId, labels);
}

// --- Segmentation Image Operations ---

bool ProjectDB::has_segmentation_image(int nodeId) const {
  return impl_->hasSegmentationImage(nodeId);
}

cv::Mat ProjectDB::segmentation_image(int nodeId) const {
  return impl_->getSegmentationImage(nodeId);
}

std::vector<int> ProjectDB::segmentation_image_ids() const {
  return impl_->getSegmentationImageIds();
}

void ProjectDB::save_segmentation_image(int nodeId, const cv::Mat &labels) {
  impl_->checkWritable();
  impl_->saveSegmentationImage(nodeId, labels);
}

void ProjectDB::save_segmentation_images(const std::vector<int> &nodeIds,
                                         const std::vector<cv::Mat> &labels) {
  impl_->checkWritable();
  impl_->saveSegmentationImages(nodeIds, labels);
}

// --- Glass Confidence Image Operations ---

bool ProjectDB::has_glass_confidence_image(int nodeId) const {
  return impl_->hasGlassConfidenceImage(nodeId);
}

cv::Mat ProjectDB::glass_confidence_image(int nodeId) const {
  return impl_->getGlassConfidenceImage(nodeId);
}

std::vector<int> ProjectDB::glass_confidence_image_ids() const {
  return impl_->getGlassConfidenceImageIds();
}

void ProjectDB::save_glass_confidence_image(int nodeId,
                                            const cv::Mat &confidence) {
  impl_->checkWritable();
  impl_->saveGlassConfidenceImage(nodeId, confidence);
}

// --- Point Cloud Operations ---

void ProjectDB::save_point_cloud(std::string_view name, const Cloud &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  impl_->checkWritable();
  auto data = serializeXYZRGB(cloud);
  impl_->savePointCloudMeta(name, "PointXYZRGB", cloud.size(), XYZRGB_STEP,
                            cloud.width, cloud.height, data, stage, paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name, const CloudN &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  impl_->checkWritable();
  auto data = serializeNormal(cloud);
  impl_->savePointCloudMeta(name, "Normal", cloud.size(), NORMAL_STEP,
                            cloud.width, cloud.height, data, stage, paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name, const CloudL &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  impl_->checkWritable();
  auto data = serializeLabel(cloud);
  impl_->savePointCloudMeta(name, "Label", cloud.size(), LABEL_STEP,
                            cloud.width, cloud.height, data, stage, paramsJson);
}

void ProjectDB::save_point_cloud(std::string_view name,
                                 const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                 std::string_view stage,
                                 std::string_view paramsJson) {
  impl_->checkWritable();
  auto data = serializeXYZ(cloud);
  impl_->savePointCloudMeta(name, "PointXYZ", cloud.size(), XYZ_STEP,
                            cloud.width, cloud.height, data, stage, paramsJson);
}

CloudPtr ProjectDB::point_cloud_xyzrgb(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "PointXYZRGB")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected PointXYZRGB");
  return deserializeXYZRGB(meta.data.data(), meta.data.size(), meta.width,
                           meta.height);
}

CloudNPtr ProjectDB::point_cloud_normal(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "Normal")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected Normal");
  return deserializeNormal(meta.data.data(), meta.data.size(), meta.width,
                           meta.height);
}

CloudLPtr ProjectDB::point_cloud_label(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "Label")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected Label");
  return deserializeLabel(meta.data.data(), meta.data.size(), meta.width,
                          meta.height);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
ProjectDB::point_cloud_xyz(std::string_view name) const {
  auto meta = impl_->loadCloudRaw(name);
  if (meta.point_type != "PointXYZ")
    throw std::runtime_error("Point cloud '" + std::string(name) +
                             "' is type " + meta.point_type +
                             ", expected PointXYZ");
  return deserializeXYZ(meta.data.data(), meta.data.size(), meta.width,
                        meta.height);
}

bool ProjectDB::has_point_cloud(std::string_view name) const {
  return impl_->hasPointCloud(name);
}

void ProjectDB::delete_point_cloud(std::string_view name) {
  impl_->checkWritable();
  impl_->deletePointCloud(name);
}

std::vector<std::string> ProjectDB::list_point_clouds() const {
  return impl_->listPointClouds();
}

std::string ProjectDB::point_cloud_type(std::string_view name) const {
  return impl_->getPointCloudType(name);
}

std::string ProjectDB::point_cloud_storage_order(std::string_view name) const {
  return impl_->getCloudStorageOrder(name);
}

std::pair<std::string, std::string>
ProjectDB::point_cloud_provenance(std::string_view name) const {
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "SELECT stage, parameters FROM point_clouds WHERE name = ?;",
      "point_cloud_provenance");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, name);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("Point cloud not found: " + std::string(name));
  return {column_text(stmt, 0), column_text(stmt, 1)};
}

void ProjectDB::save_tile_index(std::string_view name,
                                const std::vector<uint8_t> &blob) {
  impl_->checkWritable();
  impl_->saveTileIndex(name, blob);
}

std::vector<uint8_t> ProjectDB::tile_index(std::string_view name) const {
  return impl_->getTileIndex(name);
}

ProjectDB::CloudPage ProjectDB::point_cloud_page(std::string_view name,
                                                 uint64_t offset,
                                                 uint64_t limit) const {
  return impl_->loadCloudPage(name, offset, limit);
}

// --- Label Definitions ---

void ProjectDB::save_label_definitions(
    std::string_view cloudName, const std::map<int, std::string> &labelMap) {
  impl_->checkWritable();
  impl_->saveLabelDefinitions(cloudName, labelMap);
}

std::map<int, std::string>
ProjectDB::label_definitions(std::string_view cloudName) const {
  return impl_->getLabelDefinitions(cloudName);
}

// --- Instances (stable identity) ---

void ProjectDB::save_instances(
    const std::string &cloud_name,
    const std::vector<ProjectDB::InstanceRecord> &records) {
  impl_->checkWritable();
  impl_->saveInstances(cloud_name, records);
}

std::vector<ProjectDB::InstanceRecord>
ProjectDB::instances(const std::string &cloud_name) const {
  return impl_->getInstances(cloud_name);
}

void ProjectDB::add_instance(const std::string &cloud_name,
                             const InstanceRecord &record) {
  impl_->checkWritable();
  if (record.guid.empty())
    throw std::runtime_error(
        fmt::format("add_instance: instance {} in cloud '{}' has an empty guid",
                    record.instance_id, cloud_name));
  const int cloudId = impl_->getCloudId(cloud_name);
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "INSERT INTO instances (cloud_id, instance_id, guid, semantic_class, "
      "point_count) VALUES (?, ?, ?, ?, ?);",
      "add_instance");
  StmtGuard guard(stmt);
  sqlite3_bind_int(stmt, 1, cloudId);
  sqlite3_bind_int(stmt, 2, static_cast<int>(record.instance_id));
  bind_text(stmt, 3, record.guid);
  sqlite3_bind_int(stmt, 4, record.semantic_class);
  sqlite3_bind_int(stmt, 5, record.point_count);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error(fmt::format(
        "add_instance: failed to insert instance {} (guid {}) into '{}': {}",
        record.instance_id, record.guid, cloud_name,
        sqlite3_errmsg(impl_->db)));
}

void ProjectDB::set_instance_point_count(const std::string &cloud_name,
                                         uint32_t instance_id,
                                         int point_count) {
  impl_->checkWritable();
  const int cloudId = impl_->getCloudId(cloud_name);
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "UPDATE instances SET point_count = ? WHERE cloud_id = ? AND "
      "instance_id = ?;",
      "set_instance_point_count");
  StmtGuard guard(stmt);
  sqlite3_bind_int(stmt, 1, point_count);
  sqlite3_bind_int(stmt, 2, cloudId);
  sqlite3_bind_int(stmt, 3, static_cast<int>(instance_id));
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_instance_point_count: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  if (sqlite3_changes(impl_->db) == 0)
    throw std::out_of_range(
        fmt::format("set_instance_point_count: no instance {} in cloud '{}'",
                    instance_id, cloud_name));
}

std::string ProjectDB::instance_guid(const std::string &cloud_name,
                                     uint32_t instance_id) const {
  return impl_->instanceGuid(cloud_name, instance_id);
}

void ProjectDB::set_instance_material(std::string_view cloudName,
                                      int instanceId,
                                      std::string_view materialGuid) {
  impl_->checkWritable();
  impl_->setInstanceMaterial(cloudName, instanceId, materialGuid);
}

std::optional<std::string>
ProjectDB::instance_material_guid(std::string_view cloudName,
                                  int instanceId) const {
  return impl_->instanceMaterialGuid(cloudName, instanceId);
}

std::map<int, std::string>
ProjectDB::instance_materials(std::string_view cloudName) const {
  return impl_->getInstanceMaterials(cloudName);
}

// --- Material Annotations (VLM-derived) ---

void ProjectDB::save_material_annotation(std::string_view materialGuid,
                                         const MaterialAnnotation &annotation) {
  impl_->checkWritable();
  impl_->saveMaterialAnnotation(materialGuid, annotation);
}

std::optional<ProjectDB::MaterialAnnotation>
ProjectDB::material_annotation(std::string_view materialGuid) const {
  return impl_->materialAnnotation(materialGuid);
}

// --- Mesh Operations ---

void ProjectDB::save_mesh(std::string_view name, const pcl::PolygonMesh &mesh,
                          std::string_view stage, std::string_view paramsJson) {
  impl_->checkWritable();
  impl_->saveMesh(name, mesh, stage, paramsJson);
}

void ProjectDB::save_mesh(std::string_view name, const pcl::TextureMesh &mesh,
                          std::string_view stage, std::string_view paramsJson) {
  impl_->checkWritable();
  impl_->saveMesh(name, mesh, stage, paramsJson);
}

pcl::PolygonMesh::Ptr ProjectDB::mesh(std::string_view name) const {
  return impl_->getMesh(name);
}

pcl::TextureMesh::Ptr ProjectDB::texture_mesh(std::string_view name) const {
  return impl_->getTextureMesh(name);
}

bool ProjectDB::has_mesh(std::string_view name) const {
  return impl_->hasMesh(name);
}

bool ProjectDB::delete_mesh(std::string_view name) {
  impl_->checkWritable();
  return impl_->deleteMesh(name);
}

std::vector<std::string> ProjectDB::list_meshes() const {
  return impl_->listMeshes();
}

std::string ProjectDB::mesh_format(std::string_view name) const {
  return impl_->getMeshFormat(name);
}

ProjectDB::MeshMetadata ProjectDB::mesh_metadata(std::string_view name) const {
  return impl_->getMeshMetadata(name);
}

std::vector<uint8_t> ProjectDB::mesh_data_blob(std::string_view name) const {
  return impl_->getMeshDataBlob(name);
}

std::vector<ProjectDB::MeshTextureInfo>
ProjectDB::mesh_texture_blobs(std::string_view name) const {
  return impl_->getMeshTextureBlobs(name);
}

std::vector<ProjectDB::MeshTextureMetadata>
ProjectDB::mesh_texture_metadata(std::string_view name) const {
  return impl_->getMeshTextureMetadata(name);
}

// --- Gaussian Splat Operations ---

void ProjectDB::save_gaussian_splat(std::string_view name,
                                    const std::vector<uint8_t> &ply,
                                    std::string_view stage,
                                    std::string_view parameters) {
  impl_->checkWritable();
  impl_->saveGaussianSplat(name, ply, stage, parameters);
}

bool ProjectDB::has_gaussian_splat(std::string_view name) const {
  return impl_->hasGaussianSplat(name);
}

std::vector<std::string> ProjectDB::list_gaussian_splats() const {
  return impl_->listGaussianSplats();
}

ProjectDB::GaussianSplatMetadata
ProjectDB::gaussian_splat_metadata(std::string_view name) const {
  return impl_->getGaussianSplatMetadata(name);
}

std::vector<uint8_t>
ProjectDB::gaussian_splat_blob(std::string_view name) const {
  return impl_->getGaussianSplatBlob(name);
}

bool ProjectDB::delete_gaussian_splat(std::string_view name) {
  impl_->checkWritable();
  return impl_->deleteGaussianSplat(name);
}

// --- Pipeline Log ---

int ProjectDB::log_pipeline_start(std::string_view stage,
                                  std::string_view paramsJson) {
  impl_->checkWritable();
  return impl_->logPipelineStart(stage, paramsJson);
}

void ProjectDB::log_pipeline_end(int logId, PipelineOutcome outcome,
                                 std::string_view errorMsg) {
  impl_->checkWritable();
  impl_->logPipelineEnd(logId, outcome, errorMsg);
}

void ProjectDB::log_pipeline_end(int logId, bool success,
                                 std::string_view errorMsg) {
  log_pipeline_end(logId,
                   success ? PipelineOutcome::success : PipelineOutcome::failed,
                   errorMsg);
}

// --- Material Passport Operations ---

reusex::core::MaterialPassport
ProjectDB::material_passport(std::string_view documentGuid) const {
  return impl_->getMaterialPassport(documentGuid);
}

std::vector<reusex::core::MaterialPassport>
ProjectDB::all_material_passports() const {
  auto iter = impl_->getMaterialPassports();
  std::vector<reusex::core::MaterialPassport> result;
  while (iter.hasNext()) {
    result.push_back(iter.next());
  }
  return result;
}

void ProjectDB::add_material_passport(
    const reusex::core::MaterialPassport &passport,
    std::string_view projectId) {
  impl_->checkWritable();
  impl_->addMaterialPassport(passport, projectId);
}

void ProjectDB::add_material_passport(
    const reusex::core::MaterialPassport &passport, std::string_view projectId,
    std::string_view id) {
  impl_->checkWritable();
  impl_->addMaterialPassport(passport, projectId, id);
}

void ProjectDB::delete_material_passport(std::string_view documentGuid) {
  impl_->checkWritable();
  // Delete related data (property values, log entries, then passport)
  const char *delete_values_query =
      "DELETE FROM passport_property_values WHERE passport_id = "
      "(SELECT id FROM material_passports WHERE document_guid = ?);";
  const char *delete_log_query =
      "DELETE FROM passport_log WHERE passport_id = "
      "(SELECT id FROM material_passports WHERE document_guid = ?);";
  const char *delete_passport_query =
      "DELETE FROM material_passports WHERE document_guid = ?;";

  sqlite3_stmt *stmt;

  // Delete property values
  if (sqlite3_prepare_v2(impl_->db, delete_values_query, -1, &stmt, nullptr) ==
      SQLITE_OK) {
    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_STATIC);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }

  // Delete log entries
  if (sqlite3_prepare_v2(impl_->db, delete_log_query, -1, &stmt, nullptr) ==
      SQLITE_OK) {
    sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                      SQLITE_STATIC);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }

  // Delete passport
  if (sqlite3_prepare_v2(impl_->db, delete_passport_query, -1, &stmt,
                         nullptr) != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare delete passport statement: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  sqlite3_bind_text(stmt, 1, documentGuid.data(), documentGuid.size(),
                    SQLITE_STATIC);

  int rc = sqlite3_step(stmt);
  int changes = sqlite3_changes(impl_->db);
  sqlite3_finalize(stmt);

  if (rc != SQLITE_DONE) {
    throw std::runtime_error("Failed to delete passport: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  if (changes == 0) {
    throw std::runtime_error("Passport not found: " +
                             std::string(documentGuid));
  }
}

// ── Material property definitions (schema v18, #413) ───────────────────────

std::vector<ProjectDB::PropertyDefinition>
ProjectDB::list_property_definitions() const {
  return impl_->listPropertyDefinitions();
}

std::string ProjectDB::add_property_definition(
    const std::string &name, const std::string &type,
    const std::vector<std::string> &options, int sort_order, int width) {
  impl_->checkWritable();

  const std::string id = core::generate_guid();
  const std::string options_json = nlohmann::json(options).dump();

  const char *query = "INSERT INTO material_property_definitions "
                      "(id, name, type, options, sort_order, width) "
                      "VALUES (?, ?, ?, ?, ?, ?);";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare add property definition statement: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, type.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 4, options_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 5, sort_order);
  sqlite3_bind_int(stmt, 6, width);

  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to add property definition: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  return id;
}

void ProjectDB::update_property_definition(
    const std::string &id, const std::string &name, const std::string &type,
    const std::vector<std::string> &options, int sort_order, int width) {
  impl_->checkWritable();

  const std::string options_json = nlohmann::json(options).dump();

  const char *query = "UPDATE material_property_definitions "
                      "SET name = ?, type = ?, options = ?, sort_order = ?, "
                      "width = ? WHERE id = ?;";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare update property definition statement: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, type.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, options_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int(stmt, 4, sort_order);
  sqlite3_bind_int(stmt, 5, width);
  sqlite3_bind_text(stmt, 6, id.c_str(), -1, SQLITE_TRANSIENT);

  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to update property definition: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  if (sqlite3_changes(impl_->db) == 0) {
    throw std::runtime_error("Property definition not found: " + id);
  }
}

void ProjectDB::delete_property_definition(const std::string &id) {
  impl_->checkWritable();

  const char *query = "DELETE FROM material_property_definitions WHERE id = ?;";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare delete property definition statement: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, id.c_str(), -1, SQLITE_TRANSIENT);

  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to delete property definition: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  if (sqlite3_changes(impl_->db) == 0) {
    throw std::runtime_error("Property definition not found: " + id);
  }
}

// ── Material thumbnails (schema v18, #413) ─────────────────────────────────

std::optional<std::pair<std::vector<std::uint8_t>, std::string>>
ProjectDB::material_thumbnail(const std::string &guid) const {
  const char *query = "SELECT blob, mime_type FROM material_thumbnails WHERE "
                      "material_guid = ?;";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare material thumbnail query: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, guid.c_str(), -1, SQLITE_TRANSIENT);

  if (sqlite3_step(stmt) != SQLITE_ROW) {
    return std::nullopt;
  }

  const void *blob_data = sqlite3_column_blob(stmt, 0);
  const int blob_size = sqlite3_column_bytes(stmt, 0);
  const char *mime =
      reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));

  std::vector<std::uint8_t> blob;
  if (blob_data && blob_size > 0) {
    const auto *bytes = static_cast<const std::uint8_t *>(blob_data);
    blob.assign(bytes, bytes + blob_size);
  }
  std::string mime_type = mime ? mime : "image/jpeg";
  return std::make_pair(std::move(blob), std::move(mime_type));
}

void ProjectDB::set_material_thumbnail(const std::string &guid,
                                       const std::vector<std::uint8_t> &blob,
                                       const std::string &mime_type) {
  impl_->checkWritable();

  const char *query =
      "INSERT INTO material_thumbnails (material_guid, blob, mime_type) "
      "VALUES (?, ?, ?) "
      "ON CONFLICT(material_guid) DO UPDATE SET "
      "  blob = excluded.blob, "
      "  mime_type = excluded.mime_type, "
      "  created_at = datetime('now');";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare set material thumbnail statement: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, guid.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_blob(stmt, 2, blob.empty() ? nullptr : blob.data(),
                    static_cast<int>(blob.size()), SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, mime_type.c_str(), -1, SQLITE_TRANSIENT);

  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to set material thumbnail: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
}

void ProjectDB::delete_material_thumbnail(const std::string &guid) {
  impl_->checkWritable();

  const char *query =
      "DELETE FROM material_thumbnails WHERE material_guid = ?;";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare delete material thumbnail statement: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, guid.c_str(), -1, SQLITE_TRANSIENT);
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to delete material thumbnail: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
}

std::vector<std::string> ProjectDB::list_passport_guids() const {
  const char *query =
      "SELECT document_guid FROM material_passports ORDER BY created_at;";

  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare list passport guids query: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  std::vector<std::string> guids;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const char *guid =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    if (guid) {
      guids.emplace_back(guid);
    }
  }
  return guids;
}

std::map<std::string, std::string>
ProjectDB::passport_stored_properties(std::string_view documentGuid) const {
  const char *query = "SELECT pd.name_en, ppv.value "
                      "FROM passport_property_values ppv "
                      "JOIN property_definitions pd ON pd.id = ppv.property_id "
                      "JOIN material_passports mp ON ppv.passport_id = mp.id "
                      "WHERE mp.document_guid = ? "
                      "ORDER BY ppv.sort_order;";

  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare passport stored properties query: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);

  std::map<std::string, std::string> properties;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    const char *name =
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
    const void *blob_data = sqlite3_column_blob(stmt, 1);
    int blob_size = sqlite3_column_bytes(stmt, 1);

    if (name && blob_data && blob_size > 0) {
      properties.emplace(name, std::string(static_cast<const char *>(blob_data),
                                           static_cast<size_t>(blob_size)));
    } else if (name) {
      properties.emplace(name, std::string{});
    }
  }
  return properties;
}

std::string
ProjectDB::passport_property_value(std::string_view documentGuid,
                                   std::string_view fieldName) const {
  const char *query = "SELECT ppv.value "
                      "FROM passport_property_values ppv "
                      "JOIN property_definitions pd ON pd.id = ppv.property_id "
                      "JOIN material_passports mp ON ppv.passport_id = mp.id "
                      "WHERE mp.document_guid = ? AND pd.name_en = ?;";

  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(
        "Failed to prepare passport property value query: " +
        std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, fieldName.data(),
                    static_cast<int>(fieldName.size()), SQLITE_STATIC);

  if (sqlite3_step(stmt) == SQLITE_ROW) {
    const void *blob_data = sqlite3_column_blob(stmt, 0);
    int blob_size = sqlite3_column_bytes(stmt, 0);
    if (blob_data && blob_size > 0) {
      return std::string(static_cast<const char *>(blob_data),
                         static_cast<size_t>(blob_size));
    }
    return {};
  }

  throw std::runtime_error("Property '" + std::string(fieldName) +
                           "' not found for passport '" +
                           std::string(documentGuid) + "'");
}

reusex::core::MaterialPassportMetadata
ProjectDB::passport_metadata(std::string_view documentGuid) const {
  return impl_->fetchPassportMetadata(documentGuid);
}

void ProjectDB::set_passport_metadata_field(std::string_view documentGuid,
                                            std::string_view column,
                                            std::string_view value) {
  impl_->checkWritable();
  // Only allow known safe columns (prevent SQL injection via column name)
  static const std::map<std::string_view, const char *> allowed = {
      {"created_at",
       "UPDATE material_passports SET created_at = ? WHERE document_guid = ?;"},
      {"revised_at",
       "UPDATE material_passports SET revised_at = ? WHERE document_guid = ?;"},
      {"version_number", "UPDATE material_passports SET version_number = ? "
                         "WHERE document_guid = ?;"},
      {"version_date", "UPDATE material_passports SET version_date = ? WHERE "
                       "document_guid = ?;"},
  };

  auto it = allowed.find(column);
  if (it == allowed.end()) {
    throw std::runtime_error("Cannot set metadata field '" +
                             std::string(column) +
                             "'. Allowed fields: created_at, revised_at, "
                             "version_number, version_date");
  }

  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, it->second, -1, &stmt, nullptr) !=
      SQLITE_OK) {
    throw std::runtime_error("Failed to prepare metadata update: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  if (value.empty()) {
    sqlite3_bind_null(stmt, 1);
  } else {
    sqlite3_bind_text(stmt, 1, value.data(), static_cast<int>(value.size()),
                      SQLITE_STATIC);
  }
  sqlite3_bind_text(stmt, 2, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);

  int rc = sqlite3_step(stmt);
  int changes = sqlite3_changes(impl_->db);

  if (rc != SQLITE_DONE) {
    throw std::runtime_error("Failed to update metadata: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  if (changes == 0) {
    throw std::runtime_error("Passport not found: " +
                             std::string(documentGuid));
  }
}

std::optional<int>
ProjectDB::passport_linked_node_id(std::string_view documentGuid) const {
  const char *query =
      "SELECT id FROM material_passports WHERE document_guid = ?;";
  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare passport id lookup: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);

  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("Passport not found: " +
                             std::string(documentGuid));

  const unsigned char *raw = sqlite3_column_text(stmt, 0);
  if (!raw)
    return std::nullopt;

  std::string id_str(reinterpret_cast<const char *>(raw));
  try {
    size_t pos = 0;
    int value = std::stoi(id_str, &pos);
    if (pos != id_str.size() || value < 0)
      return std::nullopt;
    return value;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

void ProjectDB::set_passport_property(std::string_view documentGuid,
                                      std::string_view fieldName,
                                      std::string_view value) {
  impl_->checkWritable();
  // Verify passport exists
  const char *check_query =
      "SELECT id FROM material_passports WHERE document_guid = ?;";
  sqlite3_stmt *check_stmt;
  if (sqlite3_prepare_v2(impl_->db, check_query, -1, &check_stmt, nullptr) !=
      SQLITE_OK) {
    throw std::runtime_error("Failed to prepare passport check: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard check_guard(check_stmt);

  sqlite3_bind_text(check_stmt, 1, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);

  std::string passport_internal_id;
  if (sqlite3_step(check_stmt) == SQLITE_ROW) {
    passport_internal_id =
        reinterpret_cast<const char *>(sqlite3_column_text(check_stmt, 0));
  } else {
    throw std::runtime_error("Passport not found: " +
                             std::string(documentGuid));
  }

  // Look up property definition by name_en
  const char *prop_query =
      "SELECT id, leksikon_guid FROM property_definitions WHERE name_en = ?;";
  sqlite3_stmt *prop_stmt;
  if (sqlite3_prepare_v2(impl_->db, prop_query, -1, &prop_stmt, nullptr) !=
      SQLITE_OK) {
    throw std::runtime_error("Failed to prepare property lookup: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard prop_guard(prop_stmt);

  sqlite3_bind_text(prop_stmt, 1, fieldName.data(),
                    static_cast<int>(fieldName.size()), SQLITE_STATIC);

  std::string property_id;
  std::string leksikon_guid;
  if (sqlite3_step(prop_stmt) == SQLITE_ROW) {
    property_id =
        reinterpret_cast<const char *>(sqlite3_column_text(prop_stmt, 0));
    leksikon_guid =
        reinterpret_cast<const char *>(sqlite3_column_text(prop_stmt, 1));
  } else {
    // Auto-create property definition for custom/unknown fields
    property_id = "custom:" + std::string(fieldName);
    leksikon_guid = property_id;

    const char *insert_def =
        "INSERT INTO property_definitions "
        "(id, leksikon_guid, name_en, category, data_type) "
        "VALUES (?, ?, ?, 'Custom', 'string');";
    sqlite3_stmt *def_stmt;
    if (sqlite3_prepare_v2(impl_->db, insert_def, -1, &def_stmt, nullptr) !=
        SQLITE_OK) {
      throw std::runtime_error(
          "Failed to prepare custom property definition: " +
          std::string(sqlite3_errmsg(impl_->db)));
    }
    StmtGuard def_guard(def_stmt);
    sqlite3_bind_text(def_stmt, 1, property_id.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(def_stmt, 2, leksikon_guid.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(def_stmt, 3, fieldName.data(),
                      static_cast<int>(fieldName.size()), SQLITE_STATIC);

    if (sqlite3_step(def_stmt) != SQLITE_DONE) {
      throw std::runtime_error("Failed to create custom property definition: " +
                               std::string(sqlite3_errmsg(impl_->db)));
    }
  }

  // Upsert the property value
  const char *upsert_query =
      "INSERT INTO passport_property_values "
      "(id, passport_id, property_id, leksikon_guid, sort_order, value) "
      "VALUES (?, ?, ?, ?, "
      "(SELECT COALESCE(MAX(sort_order), 0) + 1 "
      " FROM passport_property_values WHERE passport_id = ?), ?) "
      "ON CONFLICT(passport_id, leksikon_guid) DO UPDATE SET value = "
      "excluded.value;";

  sqlite3_stmt *upsert_stmt;
  if (sqlite3_prepare_v2(impl_->db, upsert_query, -1, &upsert_stmt, nullptr) !=
      SQLITE_OK) {
    throw std::runtime_error("Failed to prepare property upsert: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard upsert_guard(upsert_stmt);

  std::string value_id = passport_internal_id + "_" + leksikon_guid;

  sqlite3_bind_text(upsert_stmt, 1, value_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(upsert_stmt, 2, passport_internal_id.c_str(), -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_text(upsert_stmt, 3, property_id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(upsert_stmt, 4, leksikon_guid.c_str(), -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_text(upsert_stmt, 5, passport_internal_id.c_str(), -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_blob(upsert_stmt, 6, value.data(),
                    static_cast<int>(value.size()), SQLITE_STATIC);

  if (sqlite3_step(upsert_stmt) != SQLITE_DONE) {
    throw std::runtime_error("Failed to upsert property value: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
}

void ProjectDB::delete_passport_property(std::string_view documentGuid,
                                         std::string_view fieldName) {
  impl_->checkWritable();
  const char *query = "DELETE FROM passport_property_values "
                      "WHERE passport_id = (SELECT id FROM material_passports "
                      "WHERE document_guid = ?) "
                      "AND property_id = (SELECT id FROM property_definitions "
                      "WHERE name_en = ?);";

  sqlite3_stmt *stmt;
  if (sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare delete property: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  StmtGuard guard(stmt);

  sqlite3_bind_text(stmt, 1, documentGuid.data(),
                    static_cast<int>(documentGuid.size()), SQLITE_STATIC);
  sqlite3_bind_text(stmt, 2, fieldName.data(),
                    static_cast<int>(fieldName.size()), SQLITE_STATIC);

  int rc = sqlite3_step(stmt);
  int changes = sqlite3_changes(impl_->db);

  if (rc != SQLITE_DONE) {
    throw std::runtime_error("Failed to delete property: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  if (changes == 0) {
    throw std::runtime_error("Property '" + std::string(fieldName) +
                             "' not found for passport '" +
                             std::string(documentGuid) + "'");
  }
}

// --- Project Metadata ---

ProjectDB::ProjectMetadata
ProjectDB::get_project_metadata(std::string_view projectId) const {
  const char *query =
      "SELECT id, name, building_address, year_of_construction, "
      "survey_date, survey_organisation, notes "
      "FROM projects WHERE id = ?;";

  sqlite3_stmt *stmt;
  int rc = sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare project metadata query: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  sqlite3_bind_text(stmt, 1, projectId.data(), projectId.size(), SQLITE_STATIC);

  ProjectMetadata metadata;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    metadata.id = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));

    if (sqlite3_column_type(stmt, 1) != SQLITE_NULL) {
      metadata.name =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    }

    if (sqlite3_column_type(stmt, 2) != SQLITE_NULL) {
      metadata.building_address =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
    }

    if (sqlite3_column_type(stmt, 3) != SQLITE_NULL) {
      metadata.year_of_construction = sqlite3_column_int(stmt, 3);
    }

    if (sqlite3_column_type(stmt, 4) != SQLITE_NULL) {
      metadata.survey_date =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 4));
    }

    if (sqlite3_column_type(stmt, 5) != SQLITE_NULL) {
      metadata.survey_organisation =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 5));
    }

    if (sqlite3_column_type(stmt, 6) != SQLITE_NULL) {
      metadata.notes =
          reinterpret_cast<const char *>(sqlite3_column_text(stmt, 6));
    }
  } else {
    sqlite3_finalize(stmt);
    throw std::runtime_error("Project not found: " + std::string(projectId));
  }

  sqlite3_finalize(stmt);
  return metadata;
}

void ProjectDB::update_project_metadata(const ProjectMetadata &metadata) {
  impl_->checkWritable();
  const char *query =
      "INSERT INTO projects (id, name, building_address, year_of_construction, "
      "survey_date, survey_organisation, notes) "
      "VALUES (?, ?, ?, ?, ?, ?, ?) "
      "ON CONFLICT(id) DO UPDATE SET "
      "name = excluded.name, "
      "building_address = excluded.building_address, "
      "year_of_construction = excluded.year_of_construction, "
      "survey_date = excluded.survey_date, "
      "survey_organisation = excluded.survey_organisation, "
      "notes = excluded.notes;";

  sqlite3_stmt *stmt;
  int rc = sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare project metadata update: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  sqlite3_bind_text(stmt, 1, metadata.id.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, metadata.name.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 3, metadata.building_address.c_str(), -1,
                    SQLITE_TRANSIENT);

  if (metadata.year_of_construction > 0) {
    sqlite3_bind_int(stmt, 4, metadata.year_of_construction);
  } else {
    sqlite3_bind_null(stmt, 4);
  }

  sqlite3_bind_text(stmt, 5, metadata.survey_date.c_str(), -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 6, metadata.survey_organisation.c_str(), -1,
                    SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 7, metadata.notes.c_str(), -1, SQLITE_TRANSIENT);

  rc = sqlite3_step(stmt);
  sqlite3_finalize(stmt);

  if (rc != SQLITE_DONE) {
    throw std::runtime_error("Failed to update project metadata: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
}

std::vector<std::string> ProjectDB::list_project_ids() const {
  const char *query = "SELECT id FROM projects ORDER BY id;";

  sqlite3_stmt *stmt;
  int rc = sqlite3_prepare_v2(impl_->db, query, -1, &stmt, nullptr);
  if (rc != SQLITE_OK) {
    throw std::runtime_error("Failed to prepare project list query: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }

  std::vector<std::string> project_ids;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    project_ids.emplace_back(
        reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0)));
  }

  sqlite3_finalize(stmt);
  return project_ids;
}

// --- Building Component Operations ---

void ProjectDB::save_component_record(const core::ComponentRecord &record) {
  impl_->checkWritable();
  impl_->saveComponentRecord(record);
}

void ProjectDB::update_component_record_by_guid(
    const core::ComponentRecord &record) {
  impl_->checkWritable();
  impl_->updateComponentRecordByGuid(record);
}

core::ComponentRecord ProjectDB::component_record(std::string_view name) const {
  return impl_->getComponentRecord(name);
}

bool ProjectDB::has_building_component(std::string_view name) const {
  return impl_->hasBuildingComponent(name);
}

void ProjectDB::delete_building_component(std::string_view name) {
  impl_->checkWritable();
  impl_->deleteBuildingComponent(name);
}

std::vector<std::string> ProjectDB::list_building_components() const {
  return impl_->listBuildingComponents();
}

std::vector<std::string>
ProjectDB::list_building_components(std::string_view type) const {
  return impl_->listBuildingComponents(type);
}

int ProjectDB::building_component_count() const {
  return impl_->buildingComponentCount();
}

// --- Project Summary ---

ProjectDB::ProjectSummary ProjectDB::project_summary() const {
  return impl_->getProjectSummary();
}

std::vector<ProjectDB::PipelineLogEntry>
ProjectDB::pipeline_log(int limit) const {
  return impl_->getPipelineLog(limit);
}

// --- Raw table browsing (read-only) ---

namespace {

/// "name" with embedded quotes doubled — an SQL identifier literal.
std::string quote_identifier(std::string_view name) {
  std::string out = "\"";
  for (char c : name) {
    if (c == '"')
      out += '"';
    out += c;
  }
  out += '"';
  return out;
}

} // namespace

std::vector<ProjectDB::TableInfo> ProjectDB::list_tables() const {
  std::vector<TableInfo> out;
  {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "SELECT name FROM sqlite_master WHERE type='table' "
        "AND name NOT LIKE 'sqlite\\_%' ESCAPE '\\' ORDER BY name;",
        "list_tables");
    StmtGuard g(s);
    int rc;
    while ((rc = sqlite3_step(s)) == SQLITE_ROW)
      out.push_back({column_text(s, 0), 0});
    if (rc != SQLITE_DONE)
      throw std::runtime_error(std::string("list_tables: ") +
                               sqlite3_errmsg(impl_->db));
  }
  for (auto &t : out) {
    const std::string sql = "SELECT COUNT(*) FROM " + quote_identifier(t.name);
    sqlite3_stmt *s = prepare_or_throw(impl_->db, sql.c_str(), "list_tables");
    StmtGuard g(s);
    if (sqlite3_step(s) != SQLITE_ROW)
      throw std::runtime_error("list_tables: cannot count " + t.name + ": " +
                               sqlite3_errmsg(impl_->db));
    t.row_count = sqlite3_column_int64(s, 0);
  }
  return out;
}

std::vector<ProjectDB::TableColumn>
ProjectDB::table_columns(std::string_view table) const {
  // Validate against sqlite_master (one indexed lookup, not list_tables()'s
  // COUNT(*) of every table — this runs on every page fetch).
  {
    sqlite3_stmt *v = prepare_or_throw(
        impl_->db,
        "SELECT 1 FROM sqlite_master WHERE type='table' AND name=? "
        "AND name NOT LIKE 'sqlite\\_%' ESCAPE '\\';",
        "table_columns");
    StmtGuard vg(v);
    const std::string name(table);
    sqlite3_bind_text(v, 1, name.c_str(), -1, SQLITE_TRANSIENT);
    const int rc = sqlite3_step(v);
    if (rc == SQLITE_DONE)
      throw std::invalid_argument("table_columns: no table '" + name + "'");
    if (rc != SQLITE_ROW)
      throw std::runtime_error(std::string("table_columns: ") +
                               sqlite3_errmsg(impl_->db));
  }
  const std::string sql = "PRAGMA table_info(" + quote_identifier(table) + ");";
  sqlite3_stmt *s = prepare_or_throw(impl_->db, sql.c_str(), "table_columns");
  StmtGuard g(s);
  std::vector<TableColumn> out;
  int rc;
  // cid | name | type | notnull | dflt_value | pk
  while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
    TableColumn c;
    c.name = column_text(s, 1);
    c.declared_type = column_text(s, 2);
    c.not_null = sqlite3_column_int(s, 3) != 0;
    c.primary_key = sqlite3_column_int(s, 5) != 0;
    out.push_back(std::move(c));
  }
  if (rc != SQLITE_DONE)
    throw std::runtime_error(std::string("table_columns: ") +
                             sqlite3_errmsg(impl_->db));
  return out;
}

std::int64_t ProjectDB::table_row_count(std::string_view table) const {
  (void)table_columns(table); // validates the name
  const std::string sql = "SELECT COUNT(*) FROM " + quote_identifier(table);
  sqlite3_stmt *s = prepare_or_throw(impl_->db, sql.c_str(), "table_row_count");
  StmtGuard g(s);
  if (sqlite3_step(s) != SQLITE_ROW)
    throw std::runtime_error(std::string("table_row_count: ") +
                             sqlite3_errmsg(impl_->db));
  return sqlite3_column_int64(s, 0);
}

std::vector<std::vector<ProjectDB::TableCell>>
ProjectDB::table_rows(std::string_view table, std::int64_t offset,
                      std::int64_t limit, std::size_t text_limit) const {
  if (offset < 0 || limit < 0)
    throw std::invalid_argument("table_rows: offset and limit must be >= 0");
  const auto columns = table_columns(table); // validates the name
  std::vector<std::vector<TableCell>> rows;
  if (limit == 0 || columns.empty())
    return rows;

  const std::string from = quote_identifier(table);
  // Order: rowid when the table has one, else its primary key — a stable
  // order, so consecutive pages neither skip nor repeat rows.
  std::string order;
  bool has_rowid = false;
  {
    sqlite3_stmt *probe = nullptr;
    const std::string sql = "SELECT rowid FROM " + from + " LIMIT 0;";
    if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &probe, nullptr) ==
        SQLITE_OK) {
      order = " ORDER BY rowid";
      has_rowid = true;
    } else {
      std::string keys;
      for (const auto &c : columns)
        if (c.primary_key)
          keys += (keys.empty() ? "" : ", ") + quote_identifier(c.name);
      if (!keys.empty())
        order = " ORDER BY " + keys;
    }
    sqlite3_finalize(probe);
  }

  // Per column: its type, a bounded value and its full size. typeof() and
  // length() of a blob do not read its pages (sqlite's OPFLAG_TYPEOFARG /
  // OPFLAG_LENGTHARG), but any expression ON the blob — substr() included —
  // loads all of it. So a blob's value is not selected at all: its 16 head
  // bytes come from incremental blob I/O by rowid, which reads one page.
  const std::string n = std::to_string(std::max<std::size_t>(text_limit, 1));
  std::string select = has_rowid ? "rowid" : "";
  for (const auto &c : columns) {
    const std::string q = quote_identifier(c.name);
    if (!select.empty())
      select += ", ";
    const std::string blob_value =
        has_rowid ? "NULL"
                  : "substr(" + q + ", 1, 16)"; // no rowid: no blob I/O
    select += "typeof(" + q + "), CASE typeof(" + q + ") WHEN 'blob' THEN " +
              blob_value + " WHEN 'text' THEN substr(" + q + ", 1, " + n +
              ") ELSE " + q + " END, CASE typeof(" + q +
              ") WHEN 'text' THEN length(CAST(" + q + " AS BLOB)) ELSE " +
              "length(" + q + ") END";
  }
  const std::string sql = "SELECT " + select + " FROM " + from + order +
                          " LIMIT " + std::to_string(limit) + " OFFSET " +
                          std::to_string(offset) + ";";
  sqlite3_stmt *s = prepare_or_throw(impl_->db, sql.c_str(), "table_rows");
  StmtGuard g(s);
  const std::string table_name(table);
  const int first = has_rowid ? 1 : 0;
  int rc;
  while ((rc = sqlite3_step(s)) == SQLITE_ROW) {
    const sqlite3_int64 rowid = has_rowid ? sqlite3_column_int64(s, 0) : 0;
    std::vector<TableCell> row;
    row.reserve(columns.size());
    for (int i = 0; i < static_cast<int>(columns.size()); ++i) {
      const int base = first + 3 * i;
      const std::string type = column_text(s, base);
      TableCell cell;
      if (type == "integer") {
        cell.kind = TableCell::Kind::integer;
        cell.integer = sqlite3_column_int64(s, base + 1);
      } else if (type == "real") {
        cell.kind = TableCell::Kind::real;
        cell.real = sqlite3_column_double(s, base + 1);
      } else if (type == "text" || type == "blob") {
        cell.kind =
            type == "text" ? TableCell::Kind::text : TableCell::Kind::blob;
        const auto *p =
            static_cast<const char *>(sqlite3_column_blob(s, base + 1));
        const int bytes = sqlite3_column_bytes(s, base + 1);
        if (p && bytes > 0)
          cell.text.assign(p, static_cast<std::size_t>(bytes));
        cell.size =
            static_cast<std::uint64_t>(sqlite3_column_int64(s, base + 2));
        cell.truncated =
            cell.kind == TableCell::Kind::text && cell.size > cell.text.size();
        if (cell.kind == TableCell::Kind::blob && has_rowid && cell.size > 0) {
          sqlite3_blob *b = nullptr;
          if (sqlite3_blob_open(
                  impl_->db, "main", table_name.c_str(),
                  columns[static_cast<std::size_t>(i)].name.c_str(), rowid, 0,
                  &b) == SQLITE_OK) {
            char head[16];
            const int want = static_cast<int>(
                std::min<std::uint64_t>(sizeof head, cell.size));
            if (sqlite3_blob_read(b, head, want, 0) == SQLITE_OK)
              cell.text.assign(head, static_cast<std::size_t>(want));
          }
          sqlite3_blob_close(b); // a no-op on nullptr
        }
      }
      row.push_back(std::move(cell));
    }
    rows.push_back(std::move(row));
  }
  if (rc != SQLITE_DONE)
    throw std::runtime_error(std::string("table_rows: ") +
                             sqlite3_errmsg(impl_->db));
  return rows;
}

// --- Pose Graph ---

std::vector<ProjectDB::PoseGraphEdge> ProjectDB::list_pose_graph_edges() const {
  return impl_->listPoseGraphEdges();
}

std::optional<ProjectDB::PoseGraphEdge>
ProjectDB::pose_graph_edge(int from_node_id, int to_node_id,
                           std::string_view edge_type) const {
  return impl_->poseGraphEdge(from_node_id, to_node_id, edge_type);
}

bool ProjectDB::upsert_pose_graph_edge(const PoseGraphEdge &edge,
                                       bool overwrite_rejected) {
  return impl_->upsertPoseGraphEdge(edge, overwrite_rejected);
}

bool ProjectDB::set_pose_graph_edge_status(int from_node_id, int to_node_id,
                                           std::string_view edge_type,
                                           std::string_view status) {
  return impl_->setPoseGraphEdgeStatus(from_node_id, to_node_id, edge_type,
                                       status);
}

int ProjectDB::update_pose_graph_edge_metrics(
    std::span<const PoseGraphEdgeMetrics> metrics) {
  return impl_->updatePoseGraphEdgeMetrics(metrics);
}

int ProjectDB::delete_pose_graph_edges_from_source(std::string_view source) {
  return impl_->deletePoseGraphEdgesFromSource(source);
}

int ProjectDB::add_pose_graph_edge(const PoseGraphEdge &edge) {
  PoseGraphEdge e = edge;
  if (e.source.empty())
    e.source = std::string(core::pose_graph::kSourceManual);
  // The old add never touched an existing row, so it must not downgrade a
  // user's confirmation either (a measurement-less add keeps the stored
  // measurement and source: upsertPoseGraphEdge).
  if (e.status == core::pose_graph::kStatusAuto)
    if (const auto row =
            pose_graph_edge(e.from_node_id, e.to_node_id, e.edge_type);
        row && row->status == core::pose_graph::kStatusConfirmed)
      e.status = std::string(core::pose_graph::kStatusConfirmed);
  if (!impl_->upsertPoseGraphEdge(e, /*overwriteRejected=*/true))
    return -1;
  const auto row = pose_graph_edge(e.from_node_id, e.to_node_id, e.edge_type);
  return row ? static_cast<int>(row->id) : -1;
}

int ProjectDB::delete_pose_graph_edges(int from_node_id, int to_node_id,
                                       std::string_view edge_type) {
  if (!edge_type.empty())
    return set_pose_graph_edge_status(from_node_id, to_node_id, edge_type,
                                      core::pose_graph::kStatusRejected)
               ? 1
               : 0;
  int n = 0;
  for (const auto &e : list_pose_graph_edges())
    if (std::min(e.from_node_id, e.to_node_id) ==
            std::min(from_node_id, to_node_id) &&
        std::max(e.from_node_id, e.to_node_id) ==
            std::max(from_node_id, to_node_id))
      n += set_pose_graph_edge_status(e.from_node_id, e.to_node_id, e.edge_type,
                                      core::pose_graph::kStatusRejected)
               ? 1
               : 0;
  return n;
}

// --- Scans (multi-session import, #129) ---

ProjectDB::ScanRecord
ProjectDB::create_scan(const std::string &source_path,
                       const std::string &provenance_json) {
  return impl_->createScan(source_path, provenance_json);
}

std::vector<ProjectDB::ScanRecord> ProjectDB::scans() const {
  return impl_->getScans();
}

bool ProjectDB::has_pose_graph() const { return impl_->hasPoseGraph(); }

// --- Report PDF Storage (schema v20) ---

ProjectDB::ReportPdfRecord
ProjectDB::add_report_pdf(const std::vector<std::uint8_t> &pdf,
                          const std::string &label,
                          std::optional<int> blocking_types) {
  impl_->checkWritable();

  ReportPdfRecord rec;
  {
    const char *sql = R"(
      INSERT INTO report_pdfs (label, pdf_blob, blocking_types)
      VALUES (?, ?, ?)
      RETURNING id, created_at, length(pdf_blob);
    )";
    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr) != SQLITE_OK)
      throw std::runtime_error("add_report_pdf: prepare failed: " +
                               std::string(sqlite3_errmsg(impl_->db)));
    StmtGuard guard(stmt);

    sqlite3_bind_text(stmt, 1, label.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_blob(stmt, 2, pdf.data(), static_cast<int>(pdf.size()),
                      SQLITE_TRANSIENT);
    if (blocking_types)
      sqlite3_bind_int(stmt, 3, *blocking_types);
    else
      sqlite3_bind_null(stmt, 3);

    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::runtime_error("add_report_pdf: insert failed: " +
                               std::string(sqlite3_errmsg(impl_->db)));

    rec.id = sqlite3_column_int64(stmt, 0);
    if (const auto *ts =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
      rec.created_at = ts;
    rec.size_bytes = static_cast<std::size_t>(sqlite3_column_int64(stmt, 2));
  }
  rec.label = label;
  rec.blocking_types = blocking_types;
  rec.version = impl_->reportVersionOf(rec.id);
  return rec;
}

std::vector<ProjectDB::ReportPdfRecord> ProjectDB::list_report_pdfs() const {
  // A read-only open of a pre-v23 project has no blocking_types column.
  const bool has_blocking =
      impl_->columnExists("report_pdfs", "blocking_types");
  const std::string sql =
      std::string("SELECT id, label, created_at, length(pdf_blob), ") +
      (has_blocking ? "blocking_types" : "NULL") +
      ", (SELECT COUNT(*) FROM report_pdfs r2 WHERE r2.id <= r.id) "
      "FROM report_pdfs r ORDER BY id DESC;";

  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql.c_str(), -1, &stmt, nullptr) !=
      SQLITE_OK)
    throw std::runtime_error("list_report_pdfs: prepare failed: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  StmtGuard guard(stmt);

  std::vector<ReportPdfRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    ReportPdfRecord rec;
    rec.id = sqlite3_column_int64(stmt, 0);
    if (const auto *s =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1)))
      rec.label = s;
    if (const auto *s =
            reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2)))
      rec.created_at = s;
    rec.size_bytes = static_cast<std::size_t>(sqlite3_column_int64(stmt, 3));
    if (sqlite3_column_type(stmt, 4) != SQLITE_NULL)
      rec.blocking_types = sqlite3_column_int(stmt, 4);
    rec.version = sqlite3_column_int(stmt, 5);
    out.push_back(std::move(rec));
  }
  return out;
}

// --- Report assets (schema v38) ---

void ProjectDB::set_report_asset(std::string_view name,
                                 const std::vector<std::uint8_t> &data,
                                 std::string_view mime_type) {
  impl_->checkWritable();
  if (!impl_->hasReportAssetsTable)
    throw std::runtime_error("set_report_asset: project schema predates v38 "
                             "(open it read-write once to migrate)");
  if (name.empty())
    throw std::invalid_argument("set_report_asset: empty name");
  if (data.empty())
    throw std::invalid_argument("set_report_asset: empty data");
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "INSERT INTO report_assets (name, mime_type, data) VALUES (?1, ?2, ?3) "
      "ON CONFLICT (name) DO UPDATE SET mime_type = excluded.mime_type, "
      "data = excluded.data, updated_at = strftime('%Y-%m-%dT%H:%M:%SZ', "
      "'now');",
      "set_report_asset");
  StmtGuard guard(s);
  const std::string n(name), m(mime_type);
  sqlite3_bind_text(s, 1, n.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(s, 2, m.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_blob(s, 3, data.data(), static_cast<int>(data.size()),
                    SQLITE_TRANSIENT);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("set_report_asset: " +
                             std::string(sqlite3_errmsg(impl_->db)));
}

std::optional<std::pair<std::vector<std::uint8_t>, std::string>>
ProjectDB::report_asset(std::string_view name) const {
  if (!impl_->hasReportAssetsTable)
    return std::nullopt;
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db, "SELECT data, mime_type FROM report_assets WHERE name = ?;",
      "report_asset");
  StmtGuard guard(s);
  const std::string n(name);
  sqlite3_bind_text(s, 1, n.c_str(), -1, SQLITE_TRANSIENT);
  if (sqlite3_step(s) != SQLITE_ROW)
    return std::nullopt;
  const auto *blob =
      reinterpret_cast<const std::uint8_t *>(sqlite3_column_blob(s, 0));
  const int bytes = sqlite3_column_bytes(s, 0);
  std::vector<std::uint8_t> data;
  if (blob && bytes > 0)
    data.assign(blob, blob + bytes);
  std::string mime;
  if (const auto *t = reinterpret_cast<const char *>(sqlite3_column_text(s, 1)))
    mime = t;
  return std::make_pair(std::move(data), std::move(mime));
}

bool ProjectDB::delete_report_asset(std::string_view name) {
  impl_->checkWritable();
  if (!impl_->hasReportAssetsTable)
    return false;
  sqlite3_stmt *s =
      prepare_or_throw(impl_->db, "DELETE FROM report_assets WHERE name = ?;",
                       "delete_report_asset");
  StmtGuard guard(s);
  const std::string n(name);
  sqlite3_bind_text(s, 1, n.c_str(), -1, SQLITE_TRANSIENT);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("delete_report_asset: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

std::optional<std::vector<std::uint8_t>>
ProjectDB::report_pdf(int64_t id) const {
  const char *sql = "SELECT pdf_blob FROM report_pdfs WHERE id = ?;";

  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(impl_->db, sql, -1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error("report_pdf: prepare failed: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  StmtGuard guard(stmt);

  sqlite3_bind_int64(stmt, 1, id);

  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;

  const auto *blob =
      reinterpret_cast<const std::uint8_t *>(sqlite3_column_blob(stmt, 0));
  const int bytes = sqlite3_column_bytes(stmt, 0);
  if (!blob || bytes <= 0)
    return std::vector<std::uint8_t>{};

  return std::vector<std::uint8_t>(blob, blob + bytes);
}

int ProjectDB::latest_schema_version() noexcept {
  return Impl::LATEST_SCHEMA_VERSION;
}

ProjectDB::ProbeResult
ProjectDB::probe(const std::filesystem::path &path) noexcept {
  ProbeResult r;
  try {
    sqlite3 *db = nullptr;
    // Immutable unless a live WAL holds pages the main file lacks: then no
    // locks are taken and no `-shm` / `-wal` is created, so a probe leaves
    // the directory exactly as it found it and works in a read-only one.
    // A probe only reads the table list and the schema version, so not
    // seeing a concurrent writer's last transaction is harmless.
    const bool immutable = !has_live_wal(path);
    const std::string uri = read_only_uri(path, immutable);
    if (sqlite3_open_v2(uri.c_str(), &db,
                        SQLITE_OPEN_READONLY | SQLITE_OPEN_URI,
                        nullptr) != SQLITE_OK) {
      r.error = std::string("Cannot open database: ") +
                (db ? sqlite3_errmsg(db) : "out of memory");
      sqlite3_close(db);
      return r;
    }
    sqlite3_busy_timeout(db, Impl::BUSY_TIMEOUT_MS);
    struct Close {
      sqlite3 *db;
      ~Close() { sqlite3_close(db); }
    } close{db};
    // A probe vets files it has not opened before, so in a process that
    // hardens its opens (a server) it is hardened too: the schema_version
    // read below must not evaluate anything the file's schema declares. A
    // failure to harden throws, and the catch below reports it as an error.
    if (hardened_by_default())
      harden_connection(db);

    // The same tables validateSchema() requires of every project.
    const char *required[] = {"projects", "property_definitions",
                              "material_passports", "passport_property_values",
                              "passport_log"};
    auto has_table = [&](const char *name) -> int { // 1 yes, 0 no, -1 error
      sqlite3_stmt *st = nullptr;
      if (sqlite3_prepare_v2(
              db, "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?;",
              -1, &st, nullptr) != SQLITE_OK)
        return -1;
      sqlite3_bind_text(st, 1, name, -1, SQLITE_STATIC);
      const int rc = sqlite3_step(st);
      sqlite3_finalize(st);
      return rc == SQLITE_ROW ? 1 : rc == SQLITE_DONE ? 0 : -1;
    };
    for (const char *t : required) {
      const int h = has_table(t);
      if (h < 0) {
        r.error = std::string("Cannot check for table '") + t +
                  "': " + sqlite3_errmsg(db);
        return r;
      }
      if (h == 0) {
        r.error =
            std::string("Required table '") + t + "' not found in database";
        return r;
      }
    }
    if (has_table("schema_version") == 1) {
      sqlite3_stmt *st = nullptr;
      if (sqlite3_prepare_v2(db, "SELECT MAX(version) FROM schema_version;", -1,
                             &st, nullptr) == SQLITE_OK) {
        if (sqlite3_step(st) == SQLITE_ROW &&
            sqlite3_column_type(st, 0) != SQLITE_NULL)
          r.schema_version = sqlite3_column_int(st, 0);
        sqlite3_finalize(st);
      }
    }
    r.is_project = true;
  } catch (const std::exception &e) {
    r.error = e.what();
  } catch (...) {
    r.error = "unknown error";
  }
  return r;
}

// --- Resource templates (schema v25) ---

namespace {
constexpr const char *kTemplateColumns =
    "id, name, members, csv, seed, created_at, updated_at";

ProjectDB::ResourceTemplateRecord read_template(sqlite3_stmt *s) {
  ProjectDB::ResourceTemplateRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.name = column_text(s, 1);
  r.members_json = column_text(s, 2);
  r.csv_json = column_text(s, 3);
  if (sqlite3_column_type(s, 4) != SQLITE_NULL)
    r.seed = column_text(s, 4);
  r.created_at = column_text(s, 5);
  r.updated_at = column_text(s, 6);
  return r;
}
} // namespace

std::vector<ProjectDB::ResourceTemplateRecord>
ProjectDB::resource_templates() const {
  if (!impl_->tableExists("templates"))
    return {};
  const std::string sql = std::string("SELECT ") + kTemplateColumns +
                          " FROM templates ORDER BY id;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "resource_templates");
  StmtGuard guard(stmt);
  std::vector<ResourceTemplateRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_template(stmt));
  return out;
}

std::optional<ProjectDB::ResourceTemplateRecord>
ProjectDB::resource_template(int64_t id) const {
  if (!impl_->tableExists("templates"))
    return std::nullopt;
  const std::string sql = std::string("SELECT ") + kTemplateColumns +
                          " FROM templates WHERE id = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "resource_template");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_template(stmt);
}

ProjectDB::ResourceTemplateRecord
ProjectDB::add_resource_template(const ResourceTemplateRecord &rec) {
  impl_->checkWritable();
  const auto id =
      impl_->insertTemplate(rec.name, rec.members_json, rec.csv_json, rec.seed);
  return *resource_template(id);
}

ProjectDB::ResourceTemplateRecord
ProjectDB::update_resource_template(int64_t id,
                                    const ResourceTemplatePatch &p) {
  impl_->checkWritable();
  // The existence check, the UPDATE and the read-back are one unit.
  Savepoint sp(impl_->db, "update_resource_template");
  if (!resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
  std::vector<std::string> sets;
  if (p.name)
    sets.emplace_back("name = ?");
  if (p.members_json)
    sets.emplace_back("members = ?");
  if (p.csv_json)
    sets.emplace_back("csv = ?");
  if (sets.empty()) {
    auto unchanged = *resource_template(id);
    sp.release();
    return unchanged;
  }
  std::string sql = "UPDATE templates SET ";
  for (const auto &s : sets)
    sql += s + ", ";
  sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "update_resource_template");
  StmtGuard guard(stmt);
  int i = 1;
  if (p.name)
    bind_text(stmt, i++, *p.name);
  if (p.members_json)
    bind_text(stmt, i++, *p.members_json);
  if (p.csv_json)
    bind_text(stmt, i++, *p.csv_json);
  sqlite3_bind_int64(stmt, i, id);
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    if (sqlite3_extended_errcode(impl_->db) == SQLITE_CONSTRAINT_UNIQUE)
      throw core::NameConflictError("a template named '" + p.name.value_or("") +
                                    "' already exists");
    throw std::runtime_error("update_resource_template: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  }
  auto updated = *resource_template(id);
  sp.release();
  return updated;
}

bool ProjectDB::delete_resource_template(int64_t id) {
  impl_->checkWritable();
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "DELETE FROM templates WHERE id = ?;", "delete template");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("delete_resource_template: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

// --- Survey (Ressourcekortlægning, schema v22) ---

namespace {
constexpr const char *kSurveyTypeColumns =
    "id, name, eak_code, bim7aa_code, unit, treatment, review_status, "
    "confidence, mass_t, "
    "note, starred, semantic_class, created_at, updated_at";
/// Columns 14-15, appended when the project is at v37.
constexpr const char *kSurveyTypeMassColumns = ", buildup_id, element_category";

std::string survey_type_select(bool has_mass) {
  return std::string("SELECT ") + kSurveyTypeColumns +
         (has_mass ? kSurveyTypeMassColumns : "") + " FROM survey_types ";
}

ProjectDB::SurveyTypeRecord read_survey_type(sqlite3_stmt *s, bool has_mass) {
  ProjectDB::SurveyTypeRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.name = column_text(s, 1);
  r.eak_code = column_text(s, 2);
  r.bim7aa_code = column_text(s, 3);
  r.unit = column_text(s, 4);
  r.treatment = core::treatment_from_string(column_text(s, 5))
                    .value_or(core::Treatment::genanvendelse);
  r.review_status = core::review_status_from_string(column_text(s, 6))
                        .value_or(core::ReviewStatus::queue);
  r.confidence = column_opt_double(s, 7);
  r.mass_t = column_opt_double(s, 8);
  r.note = column_text(s, 9);
  r.starred = sqlite3_column_int(s, 10) != 0;
  r.semantic_class = sqlite3_column_int(s, 11);
  r.created_at = column_text(s, 12);
  r.updated_at = column_text(s, 13);
  if (has_mass) {
    if (sqlite3_column_type(s, 14) != SQLITE_NULL)
      r.buildup_id = column_text(s, 14);
    if (sqlite3_column_type(s, 15) != SQLITE_NULL)
      r.element_category = column_text(s, 15);
  }
  return r;
}
} // namespace

ProjectDB::SurveyTypeRecord
ProjectDB::add_survey_type(const SurveyTypeRecord &rec) {
  impl_->checkWritable();
  const bool mass = impl_->hasSurveyMass;
  if (!mass && (rec.buildup_id || rec.element_category))
    throw std::runtime_error("add_survey_type: buildup_id / element_category "
                             "need schema v37 (open the project read-write "
                             "once to migrate)");
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      mass ? "INSERT INTO survey_types (name, eak_code, bim7aa_code, "
             "unit, treatment, review_status, "
             "confidence, mass_t, note, starred, semantic_class, buildup_id, "
             "element_category) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?) "
             "RETURNING id;"
           : "INSERT INTO survey_types (name, eak_code, bim7aa_code, "
             "unit, treatment, review_status, "
             "confidence, mass_t, note, starred, semantic_class) "
             "VALUES (?,?,?,?,?,?,?,?,?,?,?) "
             "RETURNING id;",
      "add_survey_type");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, rec.name);
  bind_text(stmt, 2, rec.eak_code);
  bind_text(stmt, 3, rec.bim7aa_code);
  bind_text(stmt, 4, rec.unit);
  bind_text(stmt, 5, core::to_string(rec.treatment));
  bind_text(stmt, 6, core::to_string(rec.review_status));
  bind_opt_double(stmt, 7, rec.confidence);
  bind_opt_double(stmt, 8, rec.mass_t);
  bind_text(stmt, 9, rec.note);
  sqlite3_bind_int(stmt, 10, rec.starred ? 1 : 0);
  sqlite3_bind_int(stmt, 11, rec.semantic_class);
  if (mass) {
    if (rec.buildup_id)
      bind_text(stmt, 12, *rec.buildup_id);
    else
      sqlite3_bind_null(stmt, 12);
    if (rec.element_category)
      bind_text(stmt, 13, *rec.element_category);
    else
      sqlite3_bind_null(stmt, 13);
  }
  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("add_survey_type: insert failed: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  const auto id = sqlite3_column_int64(stmt, 0);
  return *survey_type(id);
}

std::vector<ProjectDB::SurveyTypeRecord> ProjectDB::survey_types() const {
  const std::string sql =
      survey_type_select(impl_->hasSurveyMass) + "ORDER BY id;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_types");
  StmtGuard guard(stmt);
  std::vector<SurveyTypeRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_survey_type(stmt, impl_->hasSurveyMass));
  return out;
}

std::optional<ProjectDB::SurveyTypeRecord>
ProjectDB::survey_type(int64_t id) const {
  const std::string sql =
      survey_type_select(impl_->hasSurveyMass) + "WHERE id = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_type");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_survey_type(stmt, impl_->hasSurveyMass);
}

ProjectDB::SurveyTypeRecord
ProjectDB::update_survey_type(int64_t id, const SurveyTypePatch &p) {
  impl_->checkWritable();
  if (!survey_type(id))
    throw std::out_of_range("no survey type " + std::to_string(id));
  // Built from a fixed column vocabulary, never from input: only the bound
  // values come from the caller.
  std::vector<std::string> sets;
  if (p.name)
    sets.emplace_back("name = ?");
  if (p.eak_code)
    sets.emplace_back("eak_code = ?");
  if (p.bim7aa_code)
    sets.emplace_back("bim7aa_code = ?");
  if (p.unit)
    sets.emplace_back("unit = ?");
  if (p.note)
    sets.emplace_back("note = ?");
  if (p.treatment)
    sets.emplace_back("treatment = ?");
  if (p.review_status)
    sets.emplace_back("review_status = ?");
  if (p.confidence)
    sets.emplace_back("confidence = ?");
  if (p.mass_t)
    sets.emplace_back("mass_t = ?");
  if (p.starred)
    sets.emplace_back("starred = ?");
  if (p.buildup_id) {
    if (!impl_->hasSurveyMass)
      throw std::runtime_error("update_survey_type: buildup_id needs schema "
                               "v37 (open the project read-write once to "
                               "migrate)");
    sets.emplace_back("buildup_id = ?");
  }
  if (sets.empty())
    return *survey_type(id);
  std::string sql = "UPDATE survey_types SET ";
  for (const auto &s : sets)
    sql += s + ", ";
  sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "update_survey_type");
  StmtGuard guard(stmt);
  int i = 1;
  if (p.name)
    bind_text(stmt, i++, *p.name);
  if (p.eak_code)
    bind_text(stmt, i++, *p.eak_code);
  if (p.bim7aa_code)
    bind_text(stmt, i++, *p.bim7aa_code);
  if (p.unit)
    bind_text(stmt, i++, *p.unit);
  if (p.note)
    bind_text(stmt, i++, *p.note);
  if (p.treatment)
    bind_text(stmt, i++, core::to_string(*p.treatment));
  if (p.review_status)
    bind_text(stmt, i++, core::to_string(*p.review_status));
  if (p.confidence)
    bind_opt_double(stmt, i++, *p.confidence);
  if (p.mass_t)
    bind_opt_double(stmt, i++, *p.mass_t);
  if (p.starred)
    sqlite3_bind_int(stmt, i++, *p.starred ? 1 : 0);
  if (p.buildup_id) {
    if (*p.buildup_id)
      bind_text(stmt, i++, **p.buildup_id);
    else
      sqlite3_bind_null(stmt, i++);
  }
  sqlite3_bind_int64(stmt, i, id);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("update_survey_type: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return *survey_type(id);
}

namespace {
/// Parts joined by their stable `instance_guid` (not cloud_id/instance_id:
/// `rux create instances` always deletes+reinserts `instances` rows on
/// re-run, but reconciles guids across the re-run, so joining on guid is
/// what survives it — see the "FOREIGN KEY ... ON DELETE SET NULL nulls the
/// link on every `create instances` re-run" fix). A part whose guid no
/// longer matches any instance row reads with null cloud/instance_id but
/// keeps its own code/quantity/room/note.
/// material_guid is the part's own passport (v25) and falls back to its
/// instance's link — but only when no other part owns that passport, so two
/// parts never write through one passport (a part reads as passport-less
/// instead; ensure_resource_passport gives it its own on first write).
std::string survey_part_select(bool has_passport_column, bool has_view_columns,
                               bool has_dismissed_column,
                               bool has_mass_columns) {
  std::string sql = "SELECT p.code, p.type_id, pc.name, i.instance_id, "
                    "p.room_id, p.room_name, p.quantity, p.starred, p.note, ";
  sql += has_passport_column
             ? "COALESCE(p.passport_guid, CASE WHEN NOT EXISTS (SELECT 1 "
               "FROM survey_parts o WHERE o.passport_guid = "
               "im.material_guid) THEN im.material_guid END)"
             : "im.material_guid";
  sql += ", p.instance_guid";
  // Columns 11-17: the view record, and whether its source still exists
  // (a frame or panorama deleted after the choice reads as no view).
  if (has_view_columns)
    sql += ", p.view_kind, p.view_id, p.view_x, p.view_y, p.view_w, "
           "p.view_h, CASE p.view_kind WHEN 'frame' THEN EXISTS (SELECT 1 "
           "FROM sensor_frames f WHERE f.node_id = p.view_id) "
           "WHEN 'panorama' THEN EXISTS (SELECT 1 FROM panoramic_images pi "
           "WHERE pi.id = p.view_id) ELSE 0 END";
  // Last column (11, or 18 with the view columns): survey_parts.dismissed.
  if (has_dismissed_column)
    sql += ", p.dismissed";
  // Columns 19-22 (v37 implies the view and dismissed columns): the part's
  // measured geometry.
  if (has_mass_columns)
    sql += ", p.element_guid, p.geom_quantity, p.geom_unit, p.geom_source";
  sql += " FROM survey_parts p "
         "LEFT JOIN instances i ON i.guid = p.instance_guid "
         "LEFT JOIN point_clouds pc ON pc.id = i.cloud_id "
         "LEFT JOIN instance_materials im ON im.cloud_id = i.cloud_id AND "
         "im.instance_id = i.instance_id ";
  return sql;
}

ProjectDB::SurveyPartRecord read_survey_part(sqlite3_stmt *s,
                                             bool has_view_columns,
                                             bool has_dismissed_column,
                                             bool has_mass_columns) {
  ProjectDB::SurveyPartRecord r;
  r.code = column_text(s, 0);
  r.type_id = sqlite3_column_int64(s, 1);
  if (sqlite3_column_type(s, 2) != SQLITE_NULL)
    r.cloud_name = column_text(s, 2);
  if (auto v = column_opt_int64(s, 3))
    r.instance_id = static_cast<std::uint32_t>(*v);
  if (auto v = column_opt_int64(s, 4))
    r.room_id = static_cast<std::uint32_t>(*v);
  r.room_name = column_text(s, 5);
  r.quantity = sqlite3_column_double(s, 6);
  r.starred = sqlite3_column_int(s, 7) != 0;
  r.note = column_text(s, 8);
  if (sqlite3_column_type(s, 9) != SQLITE_NULL)
    r.material_guid = column_text(s, 9);
  if (sqlite3_column_type(s, 10) != SQLITE_NULL)
    r.instance_guid = column_text(s, 10);
  if (has_view_columns && sqlite3_column_type(s, 11) != SQLITE_NULL) {
    const std::string kind_text = column_text(s, 11);
    const int id = sqlite3_column_int(s, 12);
    const auto kind = core::view_kind_from_string(kind_text);
    if (!kind || sqlite3_column_int(s, 17) == 0) {
      // STANDARDS §5: a dangling view is not silently dropped.
      reusex::warn("survey part {}: its view ({} #{}) no longer exists; read "
                   "as no image",
                   r.code, kind_text, id);
    } else {
      core::ResourceView v{*kind, id, std::nullopt};
      if (sqlite3_column_type(s, 13) != SQLITE_NULL)
        v.region = core::ViewRegion{
            sqlite3_column_double(s, 13), sqlite3_column_double(s, 14),
            sqlite3_column_double(s, 15), sqlite3_column_double(s, 16)};
      r.view = v;
    }
  }
  if (has_dismissed_column)
    r.dismissed = sqlite3_column_int(s, has_view_columns ? 18 : 11) != 0;
  if (has_mass_columns) {
    if (sqlite3_column_type(s, 19) != SQLITE_NULL)
      r.element_guid = column_text(s, 19);
    r.geom_quantity = column_opt_double(s, 20);
    r.geom_unit = column_text(s, 21);
    r.geom_source = column_text(s, 22);
  }
  return r;
}
} // namespace

void ProjectDB::add_survey_part(const SurveyPartRecord &rec) {
  impl_->checkWritable();
  if (!survey_type(rec.type_id))
    throw std::out_of_range("no survey type " + std::to_string(rec.type_id));
  if (rec.cloud_name.has_value() != rec.instance_id.has_value())
    throw std::invalid_argument(
        "add_survey_part '" + rec.code +
        "': cloud_name and instance_id must both be set or both be empty "
        "(missing " +
        (rec.cloud_name ? "instance_id" : "cloud_name") + ")");
  // Resolve the instance's stable guid now (not its cloud_id/instance_id):
  // `rux create instances` deletes and re-inserts `instances` rows on every
  // re-run, reconciling identity by guid, so the guid is what survives a
  // re-run and (cloud_id, instance_id) is not.
  std::optional<std::string> instance_guid;
  if (rec.cloud_name && rec.instance_id) {
    const int cloudId = impl_->getCloudId(*rec.cloud_name);
    sqlite3_stmt *lookup = prepare_or_throw(
        impl_->db,
        "SELECT guid FROM instances WHERE cloud_id = ? AND instance_id = ?;",
        "add_survey_part");
    StmtGuard lookup_guard(lookup);
    sqlite3_bind_int(lookup, 1, cloudId);
    sqlite3_bind_int64(lookup, 2, *rec.instance_id);
    if (sqlite3_step(lookup) != SQLITE_ROW)
      throw std::out_of_range("no instance " +
                              std::to_string(*rec.instance_id) + " in cloud '" +
                              *rec.cloud_name + "'");
    instance_guid = column_text(lookup, 0);
  }
  if (rec.dismissed && !impl_->hasPartDismissedColumn)
    throw std::runtime_error("add_survey_part '" + rec.code +
                             "': dismissed needs schema v36 (open the project "
                             "read-write once to migrate)");
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      impl_->hasPartDismissedColumn
          ? "INSERT INTO survey_parts (code, type_id, instance_guid, room_id, "
            "room_name, quantity, starred, note, dismissed) "
            "VALUES (?,?,?,?,?,?,?,?,?);"
          : "INSERT INTO survey_parts (code, type_id, instance_guid, room_id, "
            "room_name, quantity, starred, note) VALUES (?,?,?,?,?,?,?,?);",
      "add_survey_part");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, rec.code);
  sqlite3_bind_int64(stmt, 2, rec.type_id);
  if (instance_guid)
    bind_text(stmt, 3, *instance_guid);
  else
    sqlite3_bind_null(stmt, 3);
  rec.room_id ? sqlite3_bind_int64(stmt, 4, *rec.room_id)
              : sqlite3_bind_null(stmt, 4);
  bind_text(stmt, 5, rec.room_name);
  sqlite3_bind_double(stmt, 6, rec.quantity);
  sqlite3_bind_int(stmt, 7, rec.starred ? 1 : 0);
  bind_text(stmt, 8, rec.note);
  if (impl_->hasPartDismissedColumn)
    sqlite3_bind_int(stmt, 9, rec.dismissed ? 1 : 0);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("add_survey_part '" + rec.code +
                             "': " + sqlite3_errmsg(impl_->db));
  if (rec.element_guid) {
    // An element-backed part (schema v37): keyed on its element's guid.
    set_survey_part_geometry(rec.code, rec.element_guid, rec.geom_quantity,
                             rec.geom_unit, rec.geom_source);
  }
}

bool ProjectDB::has_survey_mass_columns() const { return impl_->hasSurveyMass; }

void ProjectDB::set_survey_part_geometry(
    std::string_view code, const std::optional<std::string> &element_guid,
    std::optional<double> geom_quantity, std::string_view geom_unit,
    std::string_view geom_source) {
  impl_->checkWritable();
  if (!impl_->hasSurveyMass)
    throw std::runtime_error("set_survey_part_geometry: needs schema v37 "
                             "(open the project read-write once to migrate)");
  static const std::set<std::string_view> sources = {
      "",          "solid_volume", "wall_area", "opening_area",
      "slab_area", "plane_area",   "count"};
  if (!sources.contains(geom_source))
    throw std::invalid_argument("set_survey_part_geometry: unknown source '" +
                                std::string(geom_source) + "'");
  if (geom_quantity && geom_unit.empty())
    throw std::invalid_argument(
        "set_survey_part_geometry: a quantity needs a unit");
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "UPDATE survey_parts SET element_guid = ?, geom_quantity = ?, "
      "geom_unit = ?, geom_source = ? WHERE code = ?;",
      "set_survey_part_geometry");
  StmtGuard guard(stmt);
  if (element_guid)
    bind_text(stmt, 1, *element_guid);
  else
    sqlite3_bind_null(stmt, 1);
  bind_opt_double(stmt, 2, geom_quantity);
  bind_text(stmt, 3, geom_quantity ? geom_unit : std::string_view(""));
  bind_text(stmt, 4, geom_quantity ? geom_source : std::string_view(""));
  bind_text(stmt, 5, code);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_survey_part_geometry '" + std::string(code) +
                             "': " + sqlite3_errmsg(impl_->db));
  if (sqlite3_changes(impl_->db) == 0)
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
}

std::optional<ProjectDB::SurveyPartRecord>
ProjectDB::survey_part_for_element(std::string_view element_guid) const {
  if (!impl_->hasSurveyMass)
    return std::nullopt;
  const std::string sql =
      survey_part_select(impl_->hasPassportColumn, impl_->hasViewColumns,
                         impl_->hasPartDismissedColumn, true) +
      "WHERE p.element_guid = ?;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "survey_part_for_element");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, element_guid);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_survey_part(stmt, impl_->hasViewColumns,
                          impl_->hasPartDismissedColumn, true);
}

// --- Build-up overrides (schema v37) ---

std::vector<ProjectDB::ProjectBuildupRecord>
ProjectDB::project_buildups() const {
  std::vector<ProjectBuildupRecord> out;
  if (!impl_->hasSurveyMass)
    return out;
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "SELECT id, definition, updated_at FROM buildups ORDER BY id;",
      "project_buildups");
  StmtGuard guard(stmt);
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(
        {column_text(stmt, 0), column_text(stmt, 1), column_text(stmt, 2)});
  return out;
}

void ProjectDB::set_project_buildup(std::string_view id,
                                    std::string_view definition) {
  impl_->checkWritable();
  if (!impl_->hasSurveyMass)
    throw std::runtime_error("set_project_buildup: needs schema v37");
  if (id.empty())
    throw std::invalid_argument("set_project_buildup: empty id");
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "INSERT INTO buildups (id, definition) VALUES (?, ?) ON CONFLICT(id) "
      "DO UPDATE SET definition = excluded.definition, updated_at = "
      "strftime('%Y-%m-%dT%H:%M:%SZ','now');",
      "set_project_buildup");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, id);
  bind_text(stmt, 2, definition);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_project_buildup: " +
                             std::string(sqlite3_errmsg(impl_->db)));
}

bool ProjectDB::delete_project_buildup(std::string_view id) {
  impl_->checkWritable();
  if (!impl_->hasSurveyMass)
    return false;
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, "DELETE FROM buildups WHERE id = ?;",
                       "delete_project_buildup");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("delete_project_buildup: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

// --- Value provenance (schema v37) ---

namespace {
ProjectDB::ValueProvenanceRecord read_provenance(sqlite3_stmt *s) {
  return {column_text(s, 0), column_text(s, 1), column_text(s, 2),
          column_text(s, 3), column_text(s, 4), column_opt_double(s, 5),
          column_text(s, 6)};
}
constexpr const char *kProvenanceSelect =
    "SELECT scope, id, field, kind, source, confidence, updated_at FROM "
    "value_provenance ";
} // namespace

void ProjectDB::set_value_provenance(const ValueProvenanceRecord &rec) {
  impl_->checkWritable();
  if (!impl_->hasSurveyMass)
    throw std::runtime_error("set_value_provenance: needs schema v37");
  if (rec.scope != "type" && rec.scope != "part")
    throw std::invalid_argument("set_value_provenance: scope must be 'type' "
                                "or 'part', not '" +
                                rec.scope + "'");
  static const std::set<std::string_view> kinds = {
      "scan", "measured", "calculated", "entered", "suggested"};
  if (!kinds.contains(rec.kind))
    throw std::invalid_argument("set_value_provenance: unknown kind '" +
                                rec.kind + "'");
  if (rec.confidence && (*rec.confidence < 0.0 || *rec.confidence > 1.0))
    throw std::invalid_argument(
        "set_value_provenance: confidence must be in [0, 1]");
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "INSERT INTO value_provenance (scope, id, field, kind, source, "
      "confidence) VALUES (?,?,?,?,?,?) ON CONFLICT(scope, id, field) DO "
      "UPDATE SET kind = excluded.kind, source = excluded.source, "
      "confidence = excluded.confidence, updated_at = "
      "strftime('%Y-%m-%dT%H:%M:%SZ','now');",
      "set_value_provenance");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, rec.scope);
  bind_text(stmt, 2, rec.id);
  bind_text(stmt, 3, rec.field);
  bind_text(stmt, 4, rec.kind);
  bind_text(stmt, 5, rec.source);
  bind_opt_double(stmt, 6, rec.confidence);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_value_provenance: " +
                             std::string(sqlite3_errmsg(impl_->db)));
}

bool ProjectDB::delete_value_provenance(std::string_view scope,
                                        std::string_view id,
                                        std::string_view field) {
  impl_->checkWritable();
  if (!impl_->hasSurveyMass)
    return false;
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "DELETE FROM value_provenance WHERE scope = ? AND id = ? AND field = ?;",
      "delete_value_provenance");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, scope);
  bind_text(stmt, 2, id);
  bind_text(stmt, 3, field);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("delete_value_provenance: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

std::vector<ProjectDB::ValueProvenanceRecord>
ProjectDB::value_provenance(std::string_view scope, std::string_view id) const {
  std::vector<ValueProvenanceRecord> out;
  if (!impl_->hasSurveyMass)
    return out;
  const std::string sql = std::string(kProvenanceSelect) +
                          "WHERE scope = ? AND id = ? ORDER BY field;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "value_provenance");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, scope);
  bind_text(stmt, 2, id);
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_provenance(stmt));
  return out;
}

std::vector<ProjectDB::ValueProvenanceRecord>
ProjectDB::all_value_provenance() const {
  std::vector<ValueProvenanceRecord> out;
  if (!impl_->hasSurveyMass)
    return out;
  const std::string sql =
      std::string(kProvenanceSelect) + "ORDER BY scope, id, field;";
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db, sql.c_str(), "all_value_provenance");
  StmtGuard guard(stmt);
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_provenance(stmt));
  return out;
}

std::vector<ProjectDB::SurveyPartRecord> ProjectDB::survey_parts() const {
  const std::string sql =
      survey_part_select(impl_->hasPassportColumn, impl_->hasViewColumns,
                         impl_->hasPartDismissedColumn, impl_->hasSurveyMass) +
      "ORDER BY p.code;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_parts");
  StmtGuard guard(stmt);
  std::vector<SurveyPartRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_survey_part(stmt, impl_->hasViewColumns,
                                   impl_->hasPartDismissedColumn,
                                   impl_->hasSurveyMass));
  return out;
}

std::optional<ProjectDB::SurveyPartRecord>
ProjectDB::survey_part(std::string_view code) const {
  const std::string sql =
      survey_part_select(impl_->hasPassportColumn, impl_->hasViewColumns,
                         impl_->hasPartDismissedColumn, impl_->hasSurveyMass) +
      "WHERE p.code = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_part");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, code);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_survey_part(stmt, impl_->hasViewColumns,
                          impl_->hasPartDismissedColumn, impl_->hasSurveyMass);
}

ProjectDB::SurveyPartRecord
ProjectDB::update_survey_part(std::string_view code, const SurveyPartPatch &p) {
  impl_->checkWritable();
  if (!survey_part(code))
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
  if (p.type_id && !survey_type(*p.type_id))
    throw std::out_of_range("no survey type " + std::to_string(*p.type_id));
  std::vector<std::string> sets;
  if (p.type_id)
    sets.emplace_back("type_id = ?");
  if (p.quantity)
    sets.emplace_back("quantity = ?");
  if (p.starred)
    sets.emplace_back("starred = ?");
  if (p.note)
    sets.emplace_back("note = ?");
  if (p.room_name)
    sets.emplace_back("room_name = ?");
  if (p.dismissed) {
    if (!impl_->hasPartDismissedColumn)
      throw std::runtime_error("update_survey_part '" + std::string(code) +
                               "': dismissed needs schema v36 (open the "
                               "project read-write once to migrate)");
    sets.emplace_back("dismissed = ?");
  }
  if (!sets.empty()) {
    std::string sql = "UPDATE survey_parts SET ";
    for (std::size_t k = 0; k < sets.size(); ++k)
      sql += (k ? ", " : "") + sets[k];
    sql += " WHERE code = ?;";
    sqlite3_stmt *stmt =
        prepare_or_throw(impl_->db, sql.c_str(), "update_survey_part");
    StmtGuard guard(stmt);
    int i = 1;
    if (p.type_id)
      sqlite3_bind_int64(stmt, i++, *p.type_id);
    if (p.quantity)
      sqlite3_bind_double(stmt, i++, *p.quantity);
    if (p.starred)
      sqlite3_bind_int(stmt, i++, *p.starred ? 1 : 0);
    if (p.note)
      bind_text(stmt, i++, *p.note);
    if (p.room_name)
      bind_text(stmt, i++, *p.room_name);
    if (p.dismissed)
      sqlite3_bind_int(stmt, i++, *p.dismissed ? 1 : 0);
    bind_text(stmt, i, code);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("update_survey_part: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  return *survey_part(code);
}

void ProjectDB::set_survey_part_quantities(
    const std::vector<std::pair<std::string, double>> &code_quantities) {
  impl_->checkWritable();
  for (const auto &[code, quantity] : code_quantities)
    if (!survey_part(code))
      throw std::out_of_range("no survey part '" + code + "'");
  impl_->execOrThrow("BEGIN TRANSACTION;");
  try {
    sqlite3_stmt *stmt = prepare_or_throw(
        impl_->db, "UPDATE survey_parts SET quantity = ? WHERE code = ?;",
        "set_survey_part_quantities");
    StmtGuard guard(stmt);
    for (const auto &[code, quantity] : code_quantities) {
      sqlite3_reset(stmt);
      sqlite3_bind_double(stmt, 1, quantity);
      bind_text(stmt, 2, code);
      if (sqlite3_step(stmt) != SQLITE_DONE)
        throw std::runtime_error("set_survey_part_quantities: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
    }
    impl_->execOrThrow("COMMIT;");
  } catch (...) {
    // Non-throwing on purpose: a throw here would mask the original
    // exception. Still check the result rather than discard it silently
    // (STANDARDS §5) so a failed rollback — which leaves the transaction
    // open on impl_->db — is at least visible in the log.
    if (sqlite3_exec(impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
        SQLITE_OK)
      reusex::warn("set_survey_part_quantities: ROLLBACK failed: {}",
                   sqlite3_errmsg(impl_->db));
    throw;
  }
}

bool ProjectDB::has_survey_part_for(std::string_view cloud_name,
                                    std::uint32_t instance_id) const {
  sqlite3_stmt *stmt =
      prepare_or_throw(impl_->db,
                       "SELECT 1 FROM survey_parts p "
                       "JOIN instances i ON i.guid = p.instance_guid "
                       "JOIN point_clouds pc ON pc.id = i.cloud_id "
                       "WHERE pc.name = ? AND i.instance_id = ?;",
                       "has_survey_part_for");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, cloud_name);
  sqlite3_bind_int64(stmt, 2, instance_id);
  return sqlite3_step(stmt) == SQLITE_ROW;
}

int ProjectDB::max_survey_part_number() const {
  // Codes are "RX-<n>"; substr from 4 skips the prefix.
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
                                        "SELECT COALESCE(MAX(CAST(substr(code, "
                                        "4) AS INTEGER)), 0) FROM survey_parts "
                                        "WHERE code LIKE 'RX-%';",
                                        "max_survey_part_number");
  StmtGuard guard(stmt);
  sqlite3_step(stmt);
  return sqlite3_column_int(stmt, 0);
}

ProjectDB::Transaction::Transaction(ProjectDB &db) : db_(&db) {
  db_->impl_->checkWritable();
  db_->impl_->execOrThrow("BEGIN IMMEDIATE;");
}

ProjectDB::Transaction::~Transaction() {
  if (!open_)
    return;
  // Non-throwing on purpose; a failed rollback leaves the transaction open
  // on this connection, so at least say so (STANDARDS §5).
  if (sqlite3_exec(db_->impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
      SQLITE_OK)
    reusex::warn("ProjectDB::Transaction: ROLLBACK failed: {}",
                 sqlite3_errmsg(db_->impl_->db));
}

void ProjectDB::Transaction::commit() {
  db_->impl_->execOrThrow("COMMIT;");
  open_ = false;
}

std::string ProjectDB::ensure_resource_passport(std::string_view code) {
  impl_->checkWritable();
  std::optional<std::string> instance_guid, owned;
  {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "SELECT passport_guid, instance_guid FROM survey_parts WHERE code = ?;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, code);
    if (sqlite3_step(s) != SQLITE_ROW)
      throw std::out_of_range("no survey part '" + std::string(code) + "'");
    if (sqlite3_column_type(s, 0) != SQLITE_NULL)
      owned = column_text(s, 0);
    if (sqlite3_column_type(s, 1) != SQLITE_NULL)
      instance_guid = column_text(s, 1);
  }
  if (owned) {
    if (instance_guid)
      impl_->restoreInstanceLink(code, *owned, *instance_guid);
    return *owned;
  }
  // The instance's current link, if any (the part may predate it or have
  // been created after `rux create materials` linked it).
  std::optional<std::string> linked;
  if (instance_guid) {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "SELECT im.material_guid FROM instances i JOIN instance_materials im "
        "ON im.cloud_id = i.cloud_id AND im.instance_id = i.instance_id "
        "WHERE i.guid = ?;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, *instance_guid);
    if (sqlite3_step(s) == SQLITE_ROW)
      linked = column_text(s, 0);
  }
  Savepoint sp(impl_->db, "ensure_resource_passport");
  // set_passport_property files an unknown name_en under an auto-created
  // "custom:" definition; leksikon names must find their real one.
  impl_->ensureAllPropertyDefinitions();
  std::string guid;
  if (linked && impl_->passportExists(*linked) &&
      !impl_->passportOwnedByPart(*linked)) {
    guid = *linked; // adopt the instance's unowned passport
  } else {
    if (linked && impl_->passportOwnedByPart(*linked))
      // Reads already showed this part as passport-less (spec: one passport
      // per part); say once, now that it gets its own, why it is not shared.
      reusex::warn("ensure_resource_passport: the instance of part '{}' is "
                   "linked to passport '{}', which another part owns; '{}' "
                   "gets its own passport and its instance is relinked",
                   code, *linked, code);
    guid = core::generate_guid();
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        // Every metadata column gets a value: the passport readers and the
        // MaterialEPAS export expect text, not NULL, in each one.
        "INSERT INTO material_passports (id, document_guid, created_at, "
        "revised_at, version_number, version_date) VALUES (?1, ?1, "
        "strftime('%Y-%m-%dT%H:%M:%SZ','now'), '', '0.1.0', '');",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  {
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db, "UPDATE survey_parts SET passport_guid = ? WHERE code = ?;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    bind_text(s, 2, code);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  if (instance_guid) {
    // The pipeline's link (Viewport, create materials, MaterialEPAS
    // export) follows in the same transaction (spec §4.1).
    sqlite3_stmt *s = prepare_or_throw(
        impl_->db,
        "INSERT INTO instance_materials (cloud_id, instance_id, material_guid) "
        "SELECT cloud_id, instance_id, ?1 FROM instances WHERE guid = ?2 "
        "ON CONFLICT(cloud_id, instance_id) DO UPDATE SET "
        "material_guid = excluded.material_guid;",
        "ensure_resource_passport");
    StmtGuard guard(s);
    bind_text(s, 1, guid);
    bind_text(s, 2, *instance_guid);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("ensure_resource_passport: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  sp.release();
  return guid;
}

void ProjectDB::delete_survey_part(std::string_view code) {
  impl_->checkWritable();
  if (impl_->hasSurveyMass) {
    sqlite3_stmt *p = prepare_or_throw(
        impl_->db,
        "DELETE FROM value_provenance WHERE scope = 'part' AND id = ?;",
        "delete_survey_part");
    StmtGuard pg(p);
    bind_text(p, 1, code);
    if (sqlite3_step(p) != SQLITE_DONE)
      throw std::runtime_error("delete_survey_part: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  sqlite3_stmt *s =
      prepare_or_throw(impl_->db, "DELETE FROM survey_parts WHERE code = ?;",
                       "delete_survey_part");
  StmtGuard guard(s);
  bind_text(s, 1, code);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("delete_survey_part: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  if (sqlite3_changes(impl_->db) == 0)
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
}

void ProjectDB::set_survey_part_view(
    std::string_view code, const std::optional<core::ResourceView> &view) {
  impl_->checkWritable();
  if (view)
    core::validate_view_shape(*view);
  const auto existing = survey_part(code);
  if (!existing)
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
  // A resource keeps its image once it has one: refuse to clear a set view.
  // Clearing an already-absent view is a no-op success. No internal caller
  // (sync_survey, mask -> resource) ever clears a set view, so the setter
  // can be as strict as patch_resource's higher-level check.
  if (!view && existing->view)
    throw core::KeyValueError(
        std::string(core::kViewKey),
        "'" + std::string(core::kViewKey) +
            "' already has a value and cannot be cleared");
  if (view) {
    const bool frame = view->kind == core::ViewKind::frame;
    if (frame ? !has_sensor_frame(view->id) : !has_panoramic_image(view->id))
      throw std::out_of_range(
          std::string(frame ? "no sensor frame " : "no panorama ") +
          std::to_string(view->id));
  }
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db,
      "UPDATE survey_parts SET view_kind = ?, view_id = ?, view_x = ?, "
      "view_y = ?, view_w = ?, view_h = ? WHERE code = ?;",
      "set_survey_part_view");
  StmtGuard guard(stmt);
  for (int i = 1; i <= 6; ++i)
    sqlite3_bind_null(stmt, i);
  if (view) {
    bind_text(stmt, 1, core::to_string(view->kind));
    sqlite3_bind_int(stmt, 2, view->id);
    if (view->region)
      for (int i = 0; i < 4; ++i)
        sqlite3_bind_double(stmt, 3 + i, (*view->region)[i]);
  }
  bind_text(stmt, 7, code);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("set_survey_part_view '" + std::string(code) +
                             "': " + sqlite3_errmsg(impl_->db));
}

void ProjectDB::delete_survey_type(int64_t id) {
  impl_->checkWritable();
  if (impl_->hasSurveyMass) {
    // The type's own rows and those of the parts that cascade with it.
    for (const char *sql :
         {"DELETE FROM value_provenance WHERE scope = 'type' AND id = "
          "CAST(?1 AS TEXT);",
          "DELETE FROM value_provenance WHERE scope = 'part' AND id IN "
          "(SELECT code FROM survey_parts WHERE type_id = ?1);"}) {
      sqlite3_stmt *p = prepare_or_throw(impl_->db, sql, "delete_survey_type");
      StmtGuard pg(p);
      sqlite3_bind_int64(p, 1, id);
      if (sqlite3_step(p) != SQLITE_DONE)
        throw std::runtime_error("delete_survey_type: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
    }
  }
  sqlite3_stmt *s =
      prepare_or_throw(impl_->db, "DELETE FROM survey_types WHERE id = ?;",
                       "delete_survey_type");
  StmtGuard guard(s);
  sqlite3_bind_int64(s, 1, id);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("delete_survey_type: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  if (sqlite3_changes(impl_->db) == 0)
    throw std::out_of_range("no survey type " + std::to_string(id));
}

void ProjectDB::dismiss_instance(std::string_view instance_guid) {
  impl_->checkWritable();
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "INSERT OR IGNORE INTO survey_dismissed_instances (instance_guid) "
      "VALUES (?);",
      "dismiss_instance");
  StmtGuard guard(s);
  bind_text(s, 1, instance_guid);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("dismiss_instance: " +
                             std::string(sqlite3_errmsg(impl_->db)));
}

bool ProjectDB::is_instance_dismissed(std::string_view instance_guid) const {
  if (!impl_->hasDismissedTable)
    return false;
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "SELECT 1 FROM survey_dismissed_instances WHERE instance_guid = ?;",
      "is_instance_dismissed");
  StmtGuard guard(s);
  bind_text(s, 1, instance_guid);
  return sqlite3_step(s) == SQLITE_ROW;
}

std::vector<std::string> ProjectDB::dismissed_instances() const {
  std::vector<std::string> out;
  if (!impl_->hasDismissedTable)
    return out;
  sqlite3_stmt *s = prepare_or_throw(impl_->db,
                                     "SELECT instance_guid FROM "
                                     "survey_dismissed_instances ORDER BY "
                                     "instance_guid;",
                                     "dismissed_instances");
  StmtGuard guard(s);
  while (sqlite3_step(s) == SQLITE_ROW)
    out.push_back(column_text(s, 0));
  return out;
}

bool ProjectDB::unlink_instance_material(std::string_view cloud_name,
                                         int instance_id) {
  impl_->checkWritable();
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "DELETE FROM instance_materials WHERE instance_id = ?2 AND cloud_id = "
      "(SELECT id FROM point_clouds WHERE name = ?1);",
      "unlink_instance_material");
  StmtGuard guard(s);
  bind_text(s, 1, cloud_name);
  sqlite3_bind_int(s, 2, instance_id);
  if (sqlite3_step(s) != SQLITE_DONE)
    throw std::runtime_error("unlink_instance_material: " +
                             std::string(sqlite3_errmsg(impl_->db)));
  return sqlite3_changes(impl_->db) > 0;
}

bool ProjectDB::has_passport_field_values(std::string_view name) const {
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      "SELECT 1 FROM passport_property_values v JOIN property_definitions d "
      "ON d.id = v.property_id WHERE d.name_en = ? LIMIT 1;",
      "has_passport_field_values");
  StmtGuard guard(s);
  bind_text(s, 1, name);
  return sqlite3_step(s) == SQLITE_ROW;
}

std::size_t ProjectDB::delete_passport_field_values(std::string_view name) {
  impl_->checkWritable();
  sqlite3 *db = impl_->db;
  Savepoint sp(db, "delete_passport_field_values");
  std::optional<std::size_t> deleted; // the values statement's count
  for (const char *sql :
       {"DELETE FROM passport_property_values WHERE property_id IN (SELECT id "
        "FROM property_definitions WHERE name_en = ? AND id LIKE 'custom:%');",
        "DELETE FROM property_definitions WHERE name_en = ? AND id LIKE "
        "'custom:%';"}) {
    sqlite3_stmt *s = prepare_or_throw(db, sql, "delete_passport_field_values");
    StmtGuard guard(s);
    bind_text(s, 1, name);
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("delete_passport_field_values: " +
                               std::string(sqlite3_errmsg(db)));
    if (!deleted)
      deleted = static_cast<std::size_t>(sqlite3_changes(db));
  }
  sp.release();
  return *deleted;
}

void ProjectDB::rename_passport_field(std::string_view old_name,
                                      std::string_view new_name) {
  impl_->checkWritable();
  if (old_name == new_name)
    return;
  sqlite3 *db = impl_->db;
  auto check = [&](sqlite3_stmt *s) {
    if (sqlite3_step(s) != SQLITE_DONE)
      throw std::runtime_error("rename_passport_field: " +
                               std::string(sqlite3_errmsg(db)));
  };
  /// The definition (id, leksikon_guid) whose name_en is @p name.
  auto definition = [&](std::string_view name)
      -> std::optional<std::pair<std::string, std::string>> {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "SELECT id, leksikon_guid FROM property_definitions WHERE name_en = ? "
        "ORDER BY id LIMIT 1;",
        "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, name);
    if (sqlite3_step(s) != SQLITE_ROW)
      return std::nullopt;
    return std::make_pair(column_text(s, 0), column_text(s, 1));
  };
  auto has_values = [&](std::string_view name) {
    return has_passport_field_values(name);
  };
  if (!has_values(old_name))
    return; // nothing stored under the old name
  // Only stored values block the rename: a definition left behind with no
  // values (or a leksikon definition) is reused, not a conflict.
  if (has_values(new_name))
    throw core::NameConflictError(
        "values are already stored under the field name '" +
        std::string(new_name) + "'; renaming onto it would merge them");
  const auto old_def = definition(old_name);
  Savepoint sp(db, "rename_passport_field");
  auto target = definition(new_name);
  if (!target) {
    // New definition first (values reference it), then the values, then the
    // old definition: foreign keys stay satisfied at every statement.
    const std::string new_id = "custom:" + std::string(new_name);
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "INSERT INTO property_definitions (id, leksikon_guid, name_en, "
        "category, data_type) SELECT ?1, ?1, ?2, category, data_type "
        "FROM property_definitions WHERE id = ?3;",
        "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, new_id);
    bind_text(s, 2, new_name);
    bind_text(s, 3, old_def->first);
    check(s);
    target = std::make_pair(new_id, new_id);
  }
  {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "UPDATE passport_property_values SET property_id = ?1, "
        "leksikon_guid = ?2, id = passport_id || '_' || ?2 "
        "WHERE property_id = ?3;",
        "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, target->first);
    bind_text(s, 2, target->second);
    bind_text(s, 3, old_def->first);
    check(s);
  }
  // A user column's definition goes with its name, so the old name is free
  // again; a leksikon definition stays.
  {
    sqlite3_stmt *s = prepare_or_throw(
        db,
        "DELETE FROM property_definitions WHERE id = ? AND id LIKE 'custom:%';",
        "rename_passport_field");
    StmtGuard guard(s);
    bind_text(s, 1, old_def->first);
    check(s);
  }
  sp.release();
}

bool ProjectDB::is_passport_linked(std::string_view guid) const {
  sqlite3_stmt *s = prepare_or_throw(
      impl_->db,
      impl_->hasPassportColumn
          ? "SELECT 1 FROM instance_materials WHERE material_guid = ?1 "
            "UNION ALL SELECT 1 FROM survey_parts WHERE passport_guid = ?1 "
            "LIMIT 1;"
          : "SELECT 1 FROM instance_materials WHERE material_guid = ?1;",
      "is_passport_linked");
  StmtGuard guard(s);
  bind_text(s, 1, guid);
  return sqlite3_step(s) == SQLITE_ROW;
}

namespace {
ProjectDB::SampleRecord read_sample_row(sqlite3_stmt *s) {
  ProjectDB::SampleRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.code = column_text(s, 1);
  r.title = column_text(s, 2);
  r.what = column_text(s, 3);
  r.stage = core::sample_stage_from_string(column_text(s, 4))
                .value_or(core::SampleStage::planlagt);
  r.result = core::sample_result_from_string(column_text(s, 5))
                 .value_or(core::SampleResult::none);
  r.created_at = column_text(s, 6);
  r.updated_at = column_text(s, 7);
  return r;
}
/// The sample columns read_sample_row expects.
constexpr const char *kSampleColumns =
    "id, code, title, what, stage, result, created_at, updated_at";
} // namespace

// Fills type_ids (and, when the v32 table exists, part_codes) for each
// record in place.
static void attach_sample_links(sqlite3 *db,
                                std::vector<ProjectDB::SampleRecord> &records,
                                bool has_part_links) {
  sqlite3_stmt *stmt = prepare_or_throw(
      db,
      "SELECT type_id FROM sample_links WHERE sample_id = ? ORDER BY type_id;",
      "sample_links");
  StmtGuard guard(stmt);
  for (auto &r : records) {
    sqlite3_reset(stmt);
    sqlite3_bind_int64(stmt, 1, r.id);
    while (sqlite3_step(stmt) == SQLITE_ROW)
      r.type_ids.push_back(sqlite3_column_int64(stmt, 0));
  }
  if (!has_part_links)
    return;
  sqlite3_stmt *parts =
      prepare_or_throw(db,
                       "SELECT part_code FROM sample_part_links WHERE "
                       "sample_id = ? ORDER BY part_code;",
                       "sample_part_links");
  StmtGuard parts_guard(parts);
  for (auto &r : records) {
    sqlite3_reset(parts);
    sqlite3_bind_int64(parts, 1, r.id);
    while (sqlite3_step(parts) == SQLITE_ROW)
      r.part_codes.push_back(column_text(parts, 0));
  }
}

// The distinct part codes of @p codes, in first-seen order.
// @throws std::out_of_range ("no survey part '<code>'", as the survey part
//         accessors word it) when one names no survey part.
static std::vector<std::string>
checked_part_codes(sqlite3 *db, const std::vector<std::string> &codes) {
  sqlite3_stmt *stmt = prepare_or_throw(
      db, "SELECT 1 FROM survey_parts WHERE code = ?;", "sample_part_links");
  StmtGuard guard(stmt);
  std::vector<std::string> out;
  for (const auto &c : codes) {
    sqlite3_reset(stmt);
    bind_text(stmt, 1, c);
    if (sqlite3_step(stmt) != SQLITE_ROW)
      throw std::out_of_range("no survey part '" + c + "'");
    if (std::find(out.begin(), out.end(), c) == out.end())
      out.push_back(c);
  }
  return out;
}

// Inserts (sample_id, code) rows; call inside the caller's transaction.
static void insert_sample_part_links(sqlite3 *db, int64_t id,
                                     const std::vector<std::string> &codes,
                                     const char *context) {
  sqlite3_stmt *ins =
      prepare_or_throw(db,
                       "INSERT OR IGNORE INTO sample_part_links "
                       "(sample_id, part_code) VALUES (?, ?);",
                       context);
  StmtGuard guard(ins);
  for (const auto &c : codes) {
    sqlite3_reset(ins);
    sqlite3_bind_int64(ins, 1, id);
    bind_text(ins, 2, c);
    if (sqlite3_step(ins) != SQLITE_DONE)
      throw std::runtime_error(std::string(context) + ": " +
                               sqlite3_errmsg(db));
  }
}

ProjectDB::SampleRecord
ProjectDB::add_sample(std::string_view title, std::string_view what,
                      const std::vector<int64_t> &type_ids,
                      core::SampleStage stage,
                      const std::vector<std::string> &part_codes) {
  impl_->checkWritable();
  // A sample is registered before it reaches a lab; later stages are reached
  // by update_sample (via update_sample_checked).
  if (stage != core::SampleStage::planlagt &&
      stage != core::SampleStage::udtaget)
    throw std::invalid_argument(
        "add_sample: initial stage must be planlagt or udtaget, not " +
        std::string(core::to_string(stage)));
  // Every refusal is checked before the first write, so a bad request
  // consumes no P-## code.
  std::vector<int64_t> links;
  for (auto t : type_ids) {
    if (!survey_type(t))
      throw std::out_of_range("no survey type " + std::to_string(t));
    if (std::find(links.begin(), links.end(), t) == links.end())
      links.push_back(t);
  }
  const auto parts = checked_part_codes(impl_->db, part_codes);

  int64_t id = 0;
  // The row, its links and its stage land together.
  impl_->execOrThrow("BEGIN TRANSACTION;");
  try {
    // Statements live in their own scope: an un-finalised RETURNING
    // statement would still be active at COMMIT and make it fail.
    {
      // Codes are never reused: take the highest ever issued (AUTOINCREMENT's
      // sqlite_sequence survives deletes), not the current count.
      sqlite3_stmt *seq = prepare_or_throw(
          impl_->db,
          "SELECT COALESCE((SELECT seq FROM sqlite_sequence WHERE "
          "name = 'samples'), 0);",
          "add_sample");
      StmtGuard seq_guard(seq);
      if (sqlite3_step(seq) != SQLITE_ROW)
        throw std::runtime_error("add_sample: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
      const int next = sqlite3_column_int(seq, 0) + 1;

      sqlite3_stmt *stmt =
          prepare_or_throw(impl_->db,
                           "INSERT INTO samples (code, title, what, stage) "
                           "VALUES (?,?,?,?) RETURNING id;",
                           "add_sample");
      StmtGuard guard(stmt);
      bind_text(stmt, 1, core::sample_code(next));
      bind_text(stmt, 2, title);
      bind_text(stmt, 3, what);
      bind_text(stmt, 4, core::to_string(stage));
      if (sqlite3_step(stmt) != SQLITE_ROW)
        throw std::runtime_error("add_sample: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
      id = sqlite3_column_int64(stmt, 0);

      sqlite3_stmt *ins =
          prepare_or_throw(impl_->db,
                           "INSERT OR IGNORE INTO sample_links "
                           "(sample_id, type_id) VALUES (?, ?);",
                           "add_sample");
      StmtGuard ins_guard(ins);
      for (auto t : links) {
        sqlite3_reset(ins);
        sqlite3_bind_int64(ins, 1, id);
        sqlite3_bind_int64(ins, 2, t);
        if (sqlite3_step(ins) != SQLITE_DONE)
          throw std::runtime_error("add_sample: " +
                                   std::string(sqlite3_errmsg(impl_->db)));
      }
      insert_sample_part_links(impl_->db, id, parts, "add_sample");
    }
    impl_->execOrThrow("COMMIT;");
  } catch (...) {
    // Non-throwing on purpose (see set_sample_links); a failed rollback is
    // logged rather than discarded (STANDARDS §5).
    if (sqlite3_exec(impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
        SQLITE_OK)
      reusex::warn("add_sample: ROLLBACK failed: {}",
                   sqlite3_errmsg(impl_->db));
    throw;
  }
  return *sample(id);
}

std::vector<ProjectDB::SampleRecord> ProjectDB::samples() const {
  const std::string sql =
      std::string("SELECT ") + kSampleColumns + " FROM samples ORDER BY id;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "samples");
  StmtGuard guard(stmt);
  std::vector<SampleRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_sample_row(stmt));
  attach_sample_links(impl_->db, out, impl_->hasSamplePartLinks);
  return out;
}

std::optional<ProjectDB::SampleRecord> ProjectDB::sample(int64_t id) const {
  const std::string sql =
      std::string("SELECT ") + kSampleColumns + " FROM samples WHERE id = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "sample");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  std::vector<SampleRecord> one{read_sample_row(stmt)};
  attach_sample_links(impl_->db, one, impl_->hasSamplePartLinks);
  return one.front();
}

ProjectDB::SampleRecord ProjectDB::update_sample(int64_t id,
                                                 const SamplePatch &p) {
  impl_->checkWritable();
  if (!sample(id))
    throw std::out_of_range("no sample " + std::to_string(id));
  std::vector<std::string> sets;
  if (p.title)
    sets.emplace_back("title = ?");
  if (p.what)
    sets.emplace_back("what = ?");
  if (p.stage)
    sets.emplace_back("stage = ?");
  if (p.result)
    sets.emplace_back("result = ?");
  if (!sets.empty()) {
    std::string sql = "UPDATE samples SET ";
    for (const auto &s : sets)
      sql += s + ", ";
    sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
    sqlite3_stmt *stmt =
        prepare_or_throw(impl_->db, sql.c_str(), "update_sample");
    StmtGuard guard(stmt);
    int i = 1;
    if (p.title)
      bind_text(stmt, i++, *p.title);
    if (p.what)
      bind_text(stmt, i++, *p.what);
    if (p.stage)
      bind_text(stmt, i++, core::to_string(*p.stage));
    if (p.result)
      bind_text(stmt, i++, core::to_string(*p.result));
    sqlite3_bind_int64(stmt, i, id);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("update_sample: " +
                               std::string(sqlite3_errmsg(impl_->db)));
  }
  return *sample(id);
}

bool ProjectDB::delete_sample(int64_t id) {
  impl_->checkWritable();
  sqlite3_stmt *stmt = prepare_or_throw(
      impl_->db, "DELETE FROM samples WHERE id = ?;", "delete_sample");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  sqlite3_step(stmt);
  return sqlite3_changes(impl_->db) > 0;
}

void ProjectDB::set_sample_links(
    int64_t id, const std::vector<int64_t> &type_ids,
    const std::optional<std::vector<std::string>> &part_codes) {
  impl_->checkWritable();
  if (!sample(id))
    throw std::out_of_range("no sample " + std::to_string(id));
  for (auto t : type_ids)
    if (!survey_type(t))
      throw std::out_of_range("no survey type " + std::to_string(t));
  std::optional<std::vector<std::string>> parts;
  if (part_codes)
    parts = checked_part_codes(impl_->db, *part_codes);
  impl_->execOrThrow("BEGIN TRANSACTION;");
  try {
    sqlite3_stmt *del = prepare_or_throw(
        impl_->db, "DELETE FROM sample_links WHERE sample_id = ?;",
        "set_sample_links");
    StmtGuard del_guard(del);
    sqlite3_bind_int64(del, 1, id);
    sqlite3_step(del);
    sqlite3_stmt *ins = prepare_or_throw(impl_->db,
                                         "INSERT OR IGNORE INTO sample_links "
                                         "(sample_id, type_id) VALUES (?, ?);",
                                         "set_sample_links");
    StmtGuard ins_guard(ins);
    for (auto t : type_ids) {
      sqlite3_reset(ins);
      sqlite3_bind_int64(ins, 1, id);
      sqlite3_bind_int64(ins, 2, t);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("set_sample_links: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
    }
    if (parts) {
      sqlite3_stmt *del_parts = prepare_or_throw(
          impl_->db, "DELETE FROM sample_part_links WHERE sample_id = ?;",
          "set_sample_links");
      StmtGuard del_parts_guard(del_parts);
      sqlite3_bind_int64(del_parts, 1, id);
      if (sqlite3_step(del_parts) != SQLITE_DONE)
        throw std::runtime_error("set_sample_links: " +
                                 std::string(sqlite3_errmsg(impl_->db)));
      insert_sample_part_links(impl_->db, id, *parts, "set_sample_links");
    }
    impl_->execOrThrow("COMMIT;");
  } catch (...) {
    // Non-throwing on purpose: a throw here would mask the original
    // exception. Still check the result rather than discard it silently
    // (STANDARDS §5) so a failed rollback — which leaves the transaction
    // open on impl_->db — is at least visible in the log.
    if (sqlite3_exec(impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr) !=
        SQLITE_OK)
      reusex::warn("set_sample_links: ROLLBACK failed: {}",
                   sqlite3_errmsg(impl_->db));
    throw;
  }
}

void ProjectDB::set_sample_part_links(
    int64_t id, const std::vector<std::string> &part_codes) {
  const auto current = sample(id);
  if (!current)
    throw std::out_of_range("no sample " + std::to_string(id));
  set_sample_links(id, current->type_ids, part_codes);
}

std::vector<ProjectDB::SampleRecord>
ProjectDB::samples_for_type(int64_t type_id) const {
  // A part link bears on the type the part is filed under now, so the join
  // is resolved at read time rather than stored.
  // A dismissed part (v36) is a misdetection and bears on nothing.
  const char *sql =
      !impl_->hasSamplePartLinks
          ? "SELECT sample_id FROM sample_links WHERE type_id = ?1;"
      : impl_->hasPartDismissedColumn
          ? "SELECT sample_id FROM sample_links WHERE type_id = ?1 UNION "
            "SELECT l.sample_id FROM sample_part_links l JOIN survey_parts p "
            "ON p.code = l.part_code WHERE p.type_id = ?1 AND p.dismissed = 0;"
          : "SELECT sample_id FROM sample_links WHERE type_id = ?1 UNION "
            "SELECT l.sample_id FROM sample_part_links l JOIN survey_parts p "
            "ON p.code = l.part_code WHERE p.type_id = ?1;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql, "samples_for_type");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, type_id);
  std::set<int64_t> ids;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    ids.insert(sqlite3_column_int64(stmt, 0));
  std::vector<SampleRecord> out;
  if (ids.empty())
    return out;
  for (auto &s : samples())
    if (ids.count(s.id) != 0)
      out.push_back(std::move(s));
  return out;
}

} // namespace reusex
