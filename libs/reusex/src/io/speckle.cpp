// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle.hpp"
#include "io/export_scene.hpp"

#include "core/logging.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <pcl/common/io.h>

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace reusex::io::speckle {

// ============================================================
// Internal helpers
// ============================================================
namespace {

// ---- MD5 hashing via OpenSSL EVP ----

std::string md5_hash(const std::string &data) {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_len = 0;

  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx)
    throw std::runtime_error("EVP_MD_CTX_new failed");

  if (EVP_DigestInit_ex(ctx, EVP_md5(), nullptr) != 1 ||
      EVP_DigestUpdate(ctx, data.data(), data.size()) != 1 ||
      EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
    EVP_MD_CTX_free(ctx);
    throw std::runtime_error("EVP MD5 digest failed");
  }
  EVP_MD_CTX_free(ctx);

  static constexpr char hex_chars[] = "0123456789abcdef";
  std::string result;
  result.reserve(digest_len * 2);
  for (unsigned int i = 0; i < digest_len; ++i) {
    result.push_back(hex_chars[(digest[i] >> 4) & 0x0F]);
    result.push_back(hex_chars[digest[i] & 0x0F]);
  }
  return result;
}

// Stable UUID-shaped identifier derived from an arbitrary identity string.
// Same input → same output, so re-uploads keep applicationId consistent.
std::string uuid_from_identity(std::string_view identity) {
  std::string h = md5_hash(std::string(identity));
  return fmt::format("{}-{}-{}-{}-{}", h.substr(0, 8), h.substr(8, 4),
                     h.substr(12, 4), h.substr(16, 4), h.substr(20, 12));
}

// ---- Chunking ----

template <typename T>
std::vector<nlohmann::json>
make_chunks(const std::vector<T> &data, std::size_t chunk_size,
            std::vector<nlohmann::json> &all_objects) {
  std::vector<nlohmann::json> chunk_refs;
  for (std::size_t offset = 0; offset < data.size(); offset += chunk_size) {
    std::size_t end = std::min(offset + chunk_size, data.size());
    nlohmann::json chunk_data = nlohmann::json::array();
    for (std::size_t i = offset; i < end; ++i)
      chunk_data.push_back(data[i]);

    nlohmann::json chunk_obj;
    chunk_obj["speckle_type"] = "Speckle.Core.Models.DataChunk";
    chunk_obj["data"] = std::move(chunk_data);

    // Compute hash BEFORE adding metadata
    std::string canonical = chunk_obj.dump(
        -1, ' ', false, nlohmann::json::error_handler_t::replace);
    chunk_obj["id"] = md5_hash(canonical);
    chunk_obj["totalChildrenCount"] = 0;

    chunk_refs.push_back(nlohmann::json({{"referencedId", chunk_obj["id"]},
                                         {"speckle_type", "reference"},
                                         {"__closure", nullptr}}));
    all_objects.push_back(std::move(chunk_obj));
  }
  return chunk_refs;
}

// ---- Serialization ----

// Closure map: detached child id -> depth relative to parent
using ClosureMap = std::map<std::string, int>;

// Forward declare
nlohmann::json serialize_base(const Base &obj,
                              std::vector<nlohmann::json> &all_objects,
                              ClosureMap &closure);

nlohmann::json serialize_point(const Point &pt) {
  nlohmann::json j;
  j["speckle_type"] = pt.speckle_type;
  j["x"] = pt.x;
  j["y"] = pt.y;
  j["z"] = pt.z;
  j["units"] = pt.units;
  return j;
}

/// Extract referenced IDs from chunk refs and add to closure at depth 1.
void track_chunk_refs(const std::vector<nlohmann::json> &refs,
                      ClosureMap &closure) {
  for (const auto &ref : refs)
    closure[ref["referencedId"].get<std::string>()] = 1;
}

nlohmann::json serialize_base(const Base &obj,
                              std::vector<nlohmann::json> &all_objects,
                              ClosureMap &closure) {
  nlohmann::json j;
  j["speckle_type"] = obj.speckle_type;

  if (!obj.name.empty())
    j["name"] = obj.name;

  // applicationId is mandatory in the project convention — fall back to a
  // deterministic UUID derived from the object's identity when not set.
  if (!obj.applicationId.empty()) {
    j["applicationId"] = obj.applicationId;
  } else {
    std::string identity = obj.speckle_type + "|" + obj.name;
    j["applicationId"] = uuid_from_identity(identity);
  }

  // Helper: embed small arrays directly, chunk large ones.
  // chunk_size is per-property, matching official Speckle SDK conventions:
  //   points/vertices/textureCoordinates: 31250 elements (~250 KB for doubles)
  //   colors/faces/sizes:                 62500 elements (~250 KB for ints)
  auto serialize_array = [&](const auto &vec, const std::string &name,
                             std::size_t chunk_size) {
    if (vec.empty())
      return;
    if (vec.size() <= chunk_size) {
      // Embed directly (no chunking needed)
      j[name] = vec;
    } else {
      // Chunk and detach
      auto refs = make_chunks(vec, chunk_size, all_objects);
      track_chunk_refs(refs, closure);
      j[name] = std::move(refs);
    }
  };

  // Official Speckle SDK chunk sizes (keep serialized chunks ~250 KB each)
  constexpr std::size_t kChunkCoords = 31250; // doubles: points, vertices
  constexpr std::size_t kChunkInts = 62500;   // ints: colors, faces, sizes

  // Type-specific fields
  if (auto *pc = dynamic_cast<const Pointcloud *>(&obj)) {
    j["units"] = pc->units;
    serialize_array(pc->points, "points", kChunkCoords);
    serialize_array(pc->colors, "colors", kChunkInts);
    serialize_array(pc->sizes, "sizes", kChunkInts);
  } else if (auto *mesh = dynamic_cast<const Mesh *>(&obj)) {
    j["units"] = mesh->units;
    serialize_array(mesh->vertices, "vertices", kChunkCoords);
    serialize_array(mesh->faces, "faces", kChunkInts);
    serialize_array(mesh->colors, "colors", kChunkInts);
  } else if (auto *line = dynamic_cast<const Line *>(&obj)) {
    j["units"] = line->units;
    j["start"] = serialize_point(line->start);
    j["end"] = serialize_point(line->end);
  } else if (auto *pt = dynamic_cast<const Point *>(&obj)) {
    j["x"] = pt->x;
    j["y"] = pt->y;
    j["z"] = pt->z;
    j["units"] = pt->units;
  } else if (auto *inst = dynamic_cast<const InstanceProxy *>(&obj)) {
    j["units"] = inst->units;
    j["maxDepth"] = inst->maxDepth;
    j["definitionId"] = inst->definitionId;
    j["transform"] = inst->transform;
  } else if (auto *def = dynamic_cast<const InstanceDefinitionProxy *>(&obj)) {
    j["units"] = def->units;
    j["maxDepth"] = def->maxDepth;
    j["objects"] = def->objects;
    // Speckle matches InstanceProxy.definitionId against the proxy's own
    // applicationId. Override the auto-derived UUID with the explicit one
    // so the link is stable across uploads.
    if (!def->definitionAppId.empty())
      j["applicationId"] = def->definitionAppId;
  } else if (auto *col = dynamic_cast<const Collection *>(&obj)) {
    if (!col->collectionType.empty())
      j["collectionType"] = col->collectionType;
    // Emit version as a STRING ("3") — the @link-io/speckle SDK's
    // isRootCollection() does a strict-equality check against the string
    // literals "2"/"3", so a numeric 3 fails to be recognised as a root.
    if (col->version > 0)
      j["version"] = std::to_string(col->version);
    if (!col->instanceDefinitionProxies.empty()) {
      // Embed each definition inline (not detached): Speckle's v3 viewer
      // expects to find them on the parent collection's
      // `instanceDefinitionProxies` field, with their geometry resolved via
      // applicationId against the same collection's elements.
      nlohmann::json defs = nlohmann::json::array();
      for (const auto &def : col->instanceDefinitionProxies) {
        ClosureMap unused;
        nlohmann::json def_json = serialize_base(*def, all_objects, unused);
        std::string canonical = def_json.dump(
            -1, ' ', false, nlohmann::json::error_handler_t::replace);
        def_json["id"] = md5_hash(canonical);
        def_json["totalChildrenCount"] = nullptr;
        defs.push_back(std::move(def_json));
      }
      j["instanceDefinitionProxies"] = std::move(defs);
    }
  }

  // Custom user properties go under a "properties" sub-path (Speckle v3
  // DataObject convention). Standard fields (id, name, speckle_type,
  // applicationId) stay at the top level.
  if (!obj.properties.empty()) {
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[key, value] : obj.properties)
      props[key] = value;
    j["properties"] = std::move(props);
  }

  // Children (`elements`): two encodings —
  //   - Inline (`elements`): full child JSON embedded; required by clients
  //     that don't resolve Speckle's detached `@`-prefixed references.
  //   - Detached (`@elements`): children stored as separate objects with
  //     reference stubs; Speckle's default and more efficient for large
  //     trees.
  // The choice is per-Collection via Collection::embed_elements.
  if (!obj.elements.empty()) {
    const auto *col_for_embed = dynamic_cast<const Collection *>(&obj);
    const bool embed = col_for_embed && col_for_embed->embed_elements;

    nlohmann::json children = nlohmann::json::array();
    for (const auto &child : obj.elements) {
      ClosureMap child_closure;
      nlohmann::json child_json =
          serialize_base(*child, all_objects, child_closure);

      // Compute hash BEFORE adding metadata fields
      std::string canonical = child_json.dump(
          -1, ' ', false, nlohmann::json::error_handler_t::replace);
      std::string child_id = md5_hash(canonical);

      // Add metadata after hashing (Speckle convention)
      child_json["id"] = child_id;
      child_json["totalChildrenCount"] = nullptr;
      if (!child_closure.empty()) {
        nlohmann::json cc;
        for (const auto &[id, d] : child_closure)
          cc[id] = d;
        child_json["__closure"] = std::move(cc);
      }

      // Closure: detached descendants of this child bubble up to the
      // grandparent's closure (incremented depth). The child itself is
      // tracked only when it is detached — inline-embedded children carry
      // no closure entry of their own.
      for (const auto &[id, d] : child_closure)
        closure[id] = d + 1;

      if (embed) {
        children.push_back(std::move(child_json));
      } else {
        closure[child_id] = 1;
        children.push_back(nlohmann::json({{"referencedId", child_id},
                                           {"speckle_type", "reference"},
                                           {"__closure", nullptr}}));
        all_objects.push_back(std::move(child_json));
      }
    }
    j[embed ? "elements" : "@elements"] = std::move(children);
  }

  return j;
}

// ---- HTTP via libcurl ----

size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {
  auto *response = static_cast<std::string *>(userdata);
  response->append(ptr, size * nmemb);
  return size * nmemb;
}

std::string http_post_multipart(const std::string &url,
                                const std::string &token,
                                const std::string &json_payload) {
  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");

  std::string response;
  struct curl_slist *headers = nullptr;
  headers =
      curl_slist_append(headers, ("Authorization: Bearer " + token).c_str());

  curl_mime *mime = curl_mime_init(curl);
  curl_mimepart *part = curl_mime_addpart(mime);
  curl_mime_name(part, "batch-0");
  curl_mime_filename(part, "batch-0");
  curl_mime_data(part, json_payload.c_str(),
                 static_cast<curl_off_t>(json_payload.size()));
  curl_mime_type(part, "application/json");

  // Required for signal-based timeout to work in multithreaded programs.
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_mime_free(mime);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK)
    throw std::runtime_error(std::string("curl error: ") +
                             curl_easy_strerror(res));

  if (http_code < 200 || http_code >= 300)
    throw std::runtime_error("Speckle upload failed (HTTP " +
                             std::to_string(http_code) + "): " + response);

  return response;
}

std::string http_post_json(const std::string &url, const std::string &token,
                           const std::string &json_body) {
  CURL *curl = curl_easy_init();
  if (!curl)
    throw std::runtime_error("curl_easy_init failed");

  std::string response;
  struct curl_slist *headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  headers =
      curl_slist_append(headers, ("Authorization: Bearer " + token).c_str());

  // Required for signal-based timeout to work in multithreaded programs.
  curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

  CURLcode res = curl_easy_perform(curl);
  long http_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK)
    throw std::runtime_error(std::string("curl error: ") +
                             curl_easy_strerror(res));

  if (http_code < 200 || http_code >= 300)
    throw std::runtime_error("GraphQL request failed (HTTP " +
                             std::to_string(http_code) + "): " + response);

  return response;
}

} // anonymous namespace

// ============================================================
// Serialization
// ============================================================

// Collect all objects into a flat list with IDs
std::pair<std::string, std::vector<nlohmann::json>> flatten(const Base &root) {
  std::vector<nlohmann::json> all_objects;
  ClosureMap root_closure;
  nlohmann::json root_json = serialize_base(root, all_objects, root_closure);

  // Compute root hash BEFORE adding metadata fields
  std::string canonical =
      root_json.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
  std::string root_id = md5_hash(canonical);

  // Add metadata after hashing (Speckle convention)
  root_json["id"] = root_id;
  root_json["totalChildrenCount"] = nullptr;
  if (!root_closure.empty()) {
    nlohmann::json cc;
    for (const auto &[id, d] : root_closure)
      cc[id] = d;
    root_json["__closure"] = std::move(cc);
  }

  // Root goes first
  all_objects.insert(all_objects.begin(), std::move(root_json));
  return {root_id, std::move(all_objects)};
}

// ============================================================
// SpeckleClient
// ============================================================

SpeckleClient::SpeckleClient(std::string server_url, std::string project_id,
                             std::string token)
    : server_url_(std::move(server_url)), project_id_(std::move(project_id)),
      token_(std::move(token)) {
  // Remove trailing slash from server URL
  if (!server_url_.empty() && server_url_.back() == '/')
    server_url_.pop_back();

  // Fall back to environment variable
  if (token_.empty()) {
    const char *env_token = std::getenv("SPECKLE_TOKEN");
    if (env_token)
      token_ = env_token;
  }

  if (token_.empty())
    core::warn("SpeckleClient: no auth token provided (set SPECKLE_TOKEN "
               "env or pass token to constructor)");
}

void SpeckleClient::set_max_batch_size(std::size_t bytes) {
  max_batch_bytes_ = bytes;
}

std::string SpeckleClient::send(const Base &root) {
  auto [root_id, objects] = flatten(root);

  core::info("Speckle: uploading {} objects (root: {})", objects.size(),
             root_id);

  std::string url = server_url_ + "/objects/" + project_id_;

  // Split objects into size-limited batches (JSON array format).
  nlohmann::json current_batch = nlohmann::json::array();
  std::size_t current_size = 0;
  std::size_t batch_num = 0;

  auto send_batch = [&]() {
    std::string payload = current_batch.dump();
    core::info("Speckle: sending batch {} ({} bytes, {} objects)", batch_num,
               payload.size(), current_batch.size());
    http_post_multipart(url, token_, payload);
    current_batch = nlohmann::json::array();
    current_size = 0;
    ++batch_num;
  };

  for (auto &obj : objects) {
    std::string obj_str = obj.dump();
    std::size_t obj_size = obj_str.size();

    if (!current_batch.empty() && current_size + obj_size > max_batch_bytes_) {
      send_batch();
    }

    current_batch.push_back(std::move(obj));
    current_size += obj_size;
  }

  if (!current_batch.empty())
    send_batch();

  core::info("Speckle: upload complete ({} batches)", batch_num);
  return root_id;
}

void SpeckleClient::ensure_branch(const std::string &branch) {
  nlohmann::json mutation = {
      {"query", "mutation branchCreate($input: BranchCreateInput!) { "
                "branchCreate(branch: $input) }"},
      {"variables",
       {{"input",
         {{"streamId", project_id_},
          {"name", branch},
          {"description", "Created by ReUseX"}}}}}};

  std::string url = server_url_ + "/graphql";
  std::string response = http_post_json(url, token_, mutation.dump());
  auto resp_json = nlohmann::json::parse(response);

  if (resp_json.contains("errors") && !resp_json["errors"].empty()) {
    // Branch already exists — not an error for our purposes.
    core::debug("Speckle: branch '{}' creation note: {}", branch,
                resp_json["errors"][0]["message"].get<std::string>());
  } else {
    core::info("Speckle: created branch '{}'", branch);
  }
}

std::string SpeckleClient::commit(const std::string &object_id,
                                  const std::string &branch,
                                  const std::string &message) {
  ensure_branch(branch);

  nlohmann::json mutation = {
      {"query", "mutation commitCreate($input: CommitCreateInput!) { "
                "commitCreate(commit: $input) }"},
      {"variables",
       {{"input",
         {{"streamId", project_id_},
          {"objectId", object_id},
          {"branchName", branch},
          {"message", message},
          {"sourceApplication", "ReUseX"}}}}}};

  std::string url = server_url_ + "/graphql";
  std::string response = http_post_json(url, token_, mutation.dump());

  auto resp_json = nlohmann::json::parse(response);
  core::debug("GraphQL commit response: {}", resp_json.dump(2));

  if (resp_json.contains("errors") && !resp_json["errors"].empty()) {
    throw std::runtime_error(
        "Speckle commit failed: " +
        resp_json["errors"][0]["message"].get<std::string>());
  }

  std::string commit_id = resp_json["data"]["commitCreate"].get<std::string>();
  core::info("Speckle: commit created: {}", commit_id);
  return commit_id;
}

std::string SpeckleClient::upload(const Base &root, const std::string &branch,
                                  const std::string &message) {
  std::string root_id = send(root);
  return commit(root_id, branch, message);
}

// ============================================================
// Conversion Helpers
// ============================================================

Pointcloud to_speckle(CloudConstPtr cloud) {
  if (!cloud || cloud->empty())
    throw std::invalid_argument("to_speckle: input cloud is null or empty");

  Pointcloud pc;
  pc.points.reserve(cloud->size() * 3);
  pc.colors.reserve(cloud->size());

  for (const auto &pt : cloud->points) {
    pc.points.push_back(static_cast<double>(pt.x));
    pc.points.push_back(static_cast<double>(pt.y));
    pc.points.push_back(static_cast<double>(pt.z));

    // ARGB packed integer: A=255, R, G, B
    int argb = (255 << 24) | (pt.r << 16) | (pt.g << 8) | pt.b;
    pc.colors.push_back(argb);
  }

  return pc;
}

Mesh to_speckle(const pcl::PolygonMesh &mesh) {
  Mesh speckle_mesh;

  // Check if the mesh cloud has color data
  bool has_color = false;
  for (const auto &field : mesh.cloud.fields) {
    if (field.name == "rgb" || field.name == "rgba") {
      has_color = true;
      break;
    }
  }

  if (has_color) {
    // Extract vertices + colors via PointXYZRGB
    Cloud cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    speckle_mesh.vertices.reserve(cloud.size() * 3);
    speckle_mesh.colors.reserve(cloud.size());
    for (const auto &pt : cloud.points) {
      speckle_mesh.vertices.push_back(static_cast<double>(pt.x));
      speckle_mesh.vertices.push_back(static_cast<double>(pt.y));
      speckle_mesh.vertices.push_back(static_cast<double>(pt.z));

      int argb = (255 << 24) | (pt.r << 16) | (pt.g << 8) | pt.b;
      speckle_mesh.colors.push_back(argb);
    }
  } else {
    // No color data — extract XYZ only
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    speckle_mesh.vertices.reserve(cloud.size() * 3);
    for (const auto &pt : cloud.points) {
      speckle_mesh.vertices.push_back(static_cast<double>(pt.x));
      speckle_mesh.vertices.push_back(static_cast<double>(pt.y));
      speckle_mesh.vertices.push_back(static_cast<double>(pt.z));
    }
  }

  // Convert polygon indices to Speckle face format:
  // [n, i0, i1, ..., n, i0, i1, ...]
  // Speckle uses n=0 for triangles, n=1 for quads
  for (const auto &polygon : mesh.polygons) {
    int n = static_cast<int>(polygon.vertices.size());
    // Speckle convention: 0 = triangle (3 verts), 1 = quad (4 verts),
    // n for n+3 vertices
    speckle_mesh.faces.push_back(n == 3 ? 0 : (n == 4 ? 1 : n - 3));
    for (auto idx : polygon.vertices)
      speckle_mesh.faces.push_back(static_cast<int>(idx));
  }

  return speckle_mesh;
}

Mesh to_speckle(const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &faces) {
  Mesh speckle_mesh;

  // vertices: Nx3 matrix
  speckle_mesh.vertices.reserve(vertices.rows() * 3);
  for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
    speckle_mesh.vertices.push_back(vertices(i, 0));
    speckle_mesh.vertices.push_back(vertices(i, 1));
    speckle_mesh.vertices.push_back(vertices(i, 2));
  }

  // faces: Nx3 matrix (triangles)
  speckle_mesh.faces.reserve(faces.rows() * 4);
  for (Eigen::Index i = 0; i < faces.rows(); ++i) {
    speckle_mesh.faces.push_back(0); // 0 = triangle in Speckle
    speckle_mesh.faces.push_back(static_cast<int>(faces(i, 0)));
    speckle_mesh.faces.push_back(static_cast<int>(faces(i, 1)));
    speckle_mesh.faces.push_back(static_cast<int>(faces(i, 2)));
  }

  return speckle_mesh;
}

// ============================================================
// Sphere mesh generation (shared UV sphere algorithm)
// ============================================================

namespace {

Mesh make_speckle_sphere(double cx, double cy, double cz, double radius,
                         int resolution = 16) {
  Mesh mesh;
  int lat_segments = resolution;
  int lon_segments = resolution * 2;

  // Vertices
  // Top pole
  mesh.vertices.push_back(cx);
  mesh.vertices.push_back(cy);
  mesh.vertices.push_back(cz + radius);

  // Latitude rings
  for (int lat = 1; lat < lat_segments; ++lat) {
    double phi = M_PI * lat / lat_segments;
    double sp = std::sin(phi);
    double cp = std::cos(phi);
    for (int lon = 0; lon < lon_segments; ++lon) {
      double theta = 2.0 * M_PI * lon / lon_segments;
      mesh.vertices.push_back(cx + radius * sp * std::cos(theta));
      mesh.vertices.push_back(cy + radius * sp * std::sin(theta));
      mesh.vertices.push_back(cz + radius * cp);
    }
  }

  // Bottom pole
  mesh.vertices.push_back(cx);
  mesh.vertices.push_back(cy);
  mesh.vertices.push_back(cz - radius);

  int bottom_pole = (lat_segments - 1) * lon_segments + 1;

  // Faces (Speckle format: 0=tri indicator, then 3 indices)
  // Top cap
  for (int lon = 0; lon < lon_segments; ++lon) {
    int next = (lon + 1) % lon_segments;
    mesh.faces.push_back(0);
    mesh.faces.push_back(0);
    mesh.faces.push_back(1 + lon);
    mesh.faces.push_back(1 + next);
  }

  // Middle bands
  for (int lat = 0; lat < lat_segments - 2; ++lat) {
    int ring = 1 + lat * lon_segments;
    int next_ring = ring + lon_segments;
    for (int lon = 0; lon < lon_segments; ++lon) {
      int next = (lon + 1) % lon_segments;
      int a = ring + lon;
      int b = ring + next;
      int c = next_ring + next;
      int d = next_ring + lon;

      mesh.faces.push_back(0);
      mesh.faces.push_back(a);
      mesh.faces.push_back(d);
      mesh.faces.push_back(c);

      mesh.faces.push_back(0);
      mesh.faces.push_back(a);
      mesh.faces.push_back(c);
      mesh.faces.push_back(b);
    }
  }

  // Bottom cap
  int last_ring = 1 + (lat_segments - 2) * lon_segments;
  for (int lon = 0; lon < lon_segments; ++lon) {
    int next = (lon + 1) % lon_segments;
    mesh.faces.push_back(0);
    mesh.faces.push_back(bottom_pole);
    mesh.faces.push_back(last_ring + next);
    mesh.faces.push_back(last_ring + lon);
  }

  // Neutral color for all vertices
  int vertex_count = static_cast<int>(mesh.vertices.size() / 3);
  int argb =
      (200 << 24) | (100 << 16) | (150 << 8) | 200; // Semi-transparent blue
  mesh.colors.assign(vertex_count, argb);

  return mesh;
}

} // namespace

// ============================================================
// Scene Export
// ============================================================

namespace {

// Build the 8 lines that form a small camera frustum, pointing along +Z
// from the local origin (apex at origin, image plane at z = 0.35).
// Coordinates match the reference Grasshopper model so the orientation of
// each InstanceProxy is visually unambiguous (a sphere looks the same from
// every angle, a frustum doesn't).
struct FrustumLine {
  std::string app_id;
  Point start;
  Point end;
};

std::vector<FrustumLine> camera_frustum_lines() {
  constexpr double half_w = 0.158;
  constexpr double half_h = 0.2107;
  constexpr double depth = 0.35;
  const Point apex(0.0, 0.0, 0.0);
  const Point tl(-half_w, half_h, depth);
  const Point tr(half_w, half_h, depth);
  const Point bl(-half_w, -half_h, depth);
  const Point br(half_w, -half_h, depth);
  return {
      {"reusex.material.marker.frustum.L0", apex, bl},
      {"reusex.material.marker.frustum.L1", apex, br},
      {"reusex.material.marker.frustum.L2", apex, tr},
      {"reusex.material.marker.frustum.L3", apex, tl},
      {"reusex.material.marker.frustum.L4", bl, tl},
      {"reusex.material.marker.frustum.L5", tl, tr},
      {"reusex.material.marker.frustum.L6", tr, br},
      {"reusex.material.marker.frustum.L7", br, bl},
  };
}

// Mirror a row-major 4x4 transform into a nested {M00..M33} JSON object,
// matching the project's Location sub-property convention.
nlohmann::json transform_to_location_json(const std::array<double, 16> &t) {
  nlohmann::json loc = nlohmann::json::object();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      loc[fmt::format("M{}{}", row, col)] = t[row * 4 + col];
  return loc;
}

} // namespace

auto export_to_speckle(const ExportScene &scene, const ExportConfig &cfg)
    -> std::vector<SpeckleModel> {
  std::vector<SpeckleModel> models;

  // --- "cloud" model ---
  if (scene.cloud) {
    auto pc = std::make_shared<Pointcloud>(to_speckle(scene.cloud->cloud));
    pc->name = "cloud";
    models.push_back({"cloud", pc});
    core::debug("Speckle: prepared 'cloud' model ({} points)",
                scene.cloud->cloud->size());
  }

  // --- "semantic" model ---
  if (!scene.semantic.empty()) {
    auto root = std::make_shared<Collection>();
    root->name = "semantic";

    for (const auto &cat : scene.semantic) {
      auto cat_col = std::make_shared<Collection>();
      cat_col->name = cat.name;

      // Store label metadata as properties
      std::string hex_color = fmt::format("#{:02x}{:02x}{:02x}", cat.color[0],
                                          cat.color[1], cat.color[2]);
      cat_col->properties["label_id"] = cat.label_id;
      cat_col->properties["color"] = hex_color;

      for (const auto &inst : cat.instances) {
        auto pc = std::make_shared<Pointcloud>(to_speckle(inst.points));
        pc->properties["type"] = cat.name;
        pc->properties["label_id"] = cat.label_id;
        pc->properties["color"] = hex_color;
        if (inst.instance_id > 0)
          pc->properties["instance_id"] = static_cast<int>(inst.instance_id);
        cat_col->elements.push_back(pc);
      }

      root->elements.push_back(cat_col);
    }

    models.push_back({"semantic", root});
    core::debug("Speckle: prepared 'semantic' model ({} categories)",
                scene.semantic.size());
  }

  // --- "mesh" model ---
  if (!scene.meshes.empty()) {
    auto root = std::make_shared<Collection>();
    root->name = "mesh";

    for (const auto &entry : scene.meshes) {
      auto speckle_mesh = std::make_shared<Mesh>(to_speckle(*entry.mesh));
      speckle_mesh->name = entry.name;
      root->elements.push_back(speckle_mesh);
    }

    models.push_back({"mesh", root});
    core::debug("Speckle: prepared 'mesh' model ({} meshes)",
                scene.meshes.size());
  }

  // --- "360" model ---
  if (!scene.panoramas.empty()) {
    auto root = std::make_shared<Collection>();
    root->name = "360";

    for (const auto &entry : scene.panoramas) {
      auto sphere = std::make_shared<Mesh>(
          make_speckle_sphere(entry.x, entry.y, entry.z, 0.15));

      sphere->name = entry.image_name;
      sphere->applicationId =
          uuid_from_identity("panorama:" + entry.image_name);
      sphere->properties["Image Name"] = entry.image_name;
      sphere->properties["imageUrl"] = fmt::format(
          "{}/{}/{}", cfg.image_url_base, cfg.project_id, entry.image_name);

      root->elements.push_back(sphere);
    }

    models.push_back({"360", root});
    core::debug("Speckle: prepared '360' model ({} panoramas)",
                scene.panoramas.size());
  }

  // --- "materials" model ---
  if (!scene.materials.empty()) {
    auto root = std::make_shared<Collection>();
    root->name = "materials";
    root->version = 3; // Speckle v3 root marker — enables proxy resolution.
    // Embed `elements` inline so the reuse-x webapp (which reads
    // speckleRoot.elements directly without resolving "@" references) can
    // walk down to the InstanceProxies.
    root->embed_elements = true;

    // HACK: Wrap material InstanceProxies in a "Cameras" sub-collection and
    // rename each one "Camera N" to match the legacy Grasshopper layout.
    // category=I/O estimate=2h bead=reusex-hbi
    // The reuse-x webapp's loadImages composable hardcodes
    //     speckleRoot.elements.find(c => c.name === 'Cameras')
    //         .elements.filter(e => e.name.startsWith('Camera'))
    // so without this wrapper our material tags never get discovered and
    // no image icons render in the viewer. The naming is misleading
    // (material passports, not cameras) and the wrapper adds a useless
    // level of nesting — keep this only until the webapp is fixed and
    // redeployed.
    //
    // To remove this workaround:
    //   1. In the webapp at
    //      /home/mephisto/repos/MC_Group_IO/apps/reuse-x/src/composables/
    //          images.ts (loadImages, ~line 94), drop both the
    //      `collection.name === 'Cameras'` lookup and the
    //      `name.startsWith('Camera')` filter. Walk speckleRoot.elements
    //      (descending into sub-collections) and pick anything with
    //      speckle_type === "Speckle.Core.Models.Instances.InstanceProxy"
    //      (or, more loosely, anything with properties.Base.imageURL).
    //   2. Redeploy the webapp.
    //   3. Here, undo this block: push marker + InstanceProxies back onto
    //      root->elements directly, set inst->name = entry.name, and drop
    //      the Base["id"] / Base["Index"] forwarding fields below — the
    //      webapp will then read the real material name straight off
    //      inst.name.
    auto cameras = std::make_shared<Collection>();
    cameras->name = "Cameras";
    cameras->embed_elements = true; // same reason as the parent root above

    // Shared definition geometry: a small camera-frustum (8 line
    // segments) stored as siblings of every InstanceProxy. A frustum,
    // unlike a sphere, makes the rotation part of each InstanceProxy
    // transform visible — useful for sanity-checking orientation.
    // The InstanceDefinitionProxy (held on `instanceDefinitionProxies` on
    // the root) references each line by applicationId — Speckle's
    // resolver walks the tree, so the geometry can live inside the
    // "Cameras" sub-collection alongside the instances.
    const std::string definition_app_id = "reusex.material.marker.v1";

    auto definition = std::make_shared<InstanceDefinitionProxy>();
    definition->definitionAppId = definition_app_id;
    definition->name = "Material Marker";
    definition->units = "m";
    definition->maxDepth = 0;

    for (const auto &fl : camera_frustum_lines()) {
      auto line = std::make_shared<Line>();
      line->start = fl.start;
      line->end = fl.end;
      line->applicationId = fl.app_id;
      cameras->elements.push_back(line);
      definition->objects.push_back(fl.app_id);
    }

    root->instanceDefinitionProxies.push_back(definition);

    int camera_index = 0;
    for (const auto &entry : scene.materials) {
      ++camera_index;
      auto inst = std::make_shared<InstanceProxy>();
      // HACK: rename to "Camera N" so the webapp's startsWith('Camera')
      // filter accepts us. Real material name is preserved in Base["id"].
      inst->name = fmt::format("Camera {}", camera_index);
      inst->units = "m";
      inst->maxDepth = 0;
      inst->definitionId = definition_app_id;
      inst->applicationId = uuid_from_identity("material:" + entry.name);
      inst->transform = entry.transform;

      // Base: minimal identification block. Use the original image filename
      // (stored on import via "rux import photos") when available, falling
      // back to the entry name so the URL is still resolvable.
      const std::string &filename =
          entry.image_filename.empty() ? entry.name : entry.image_filename;
      nlohmann::json base = nlohmann::json::object();
      base["id"] = entry.name;      // original material name/guid (HACK)
      base["Index"] = camera_index; // matches "Camera N" (HACK)
      base["fileName"] = filename;
      base["imageURL"] =
          fmt::format("{}/{}/{}", cfg.image_url_base, cfg.project_id, filename);
      inst->properties["Base"] = std::move(base);

      // Location: mirror the transform as a M00..M33 sub-object.
      inst->properties["Location"] =
          transform_to_location_json(inst->transform);

      // Reuse: every stored passport property carried over as-is.
      // Skip "images" — that's the base64 JPEG blob (kept on the passport
      // for archival), already surfaced via Base.imageURL above, and big
      // enough to bloat the inline JSON past Speckle object size limits.
      nlohmann::json reuse = nlohmann::json::object();
      for (const auto &[key, value] : entry.properties) {
        if (key == "images")
          continue;
        reuse[key] = value;
      }

      // HACK: seed the fields the reuse-x webapp's ImageClassificationDialog
      // category=I/O estimate=2h bead=reusex-hbi
      // (apps/reuse-x/src/components/dialogs/ImageClassificationDialog.vue)
      // reads with safe defaults so it doesn't crash on undefined property
      // access. The dialog does e.g.
      //     localReuse.value?.Investigations.replace(/'/g, '"')
      // — the `?.` only short-circuits localReuse.value itself, so a
      // missing `Investigations` makes `.replace` throw. Until the
      // webapp adds proper null-checks (then redeploys), pre-populate the
      // fields its v-model bindings expect. Real values from the
      // passport override these via the loop above; only-defaults-missing
      // fields get the placeholder. `Investigations` must be a string
      // that parses as JSON after `'`→`"` (an empty array literal works).
      //
      // The list below mirrors the dialog's `IReuse` interface 1:1 — every
      // key it binds with v-model, in declaration order. When the webapp
      // adds or renames a field, mirror it here (or drop the whole block
      // once the dialog null-checks properly). `Quantity` is bound to an
      // <Input type="number">, so it seeds as a number, not a string.
      auto default_if_missing = [&](const std::string &key, auto value) {
        if (!reuse.contains(key))
          reuse[key] = value;
      };
      default_if_missing("Risk", "");
      default_if_missing("Unit", "");
      default_if_missing("Level", "");
      default_if_missing("Material", "");
      default_if_missing("Quantity", 0);
      default_if_missing("Condition", "");
      default_if_missing("Circularity", "");
      default_if_missing("Lacation / ID", ""); // sic — webapp typo
      default_if_missing("Investigations", "[]");
      default_if_missing("Remarks / Notes", "");
      default_if_missing("Presumed Material", "");
      default_if_missing("Type Code Level 1", "");
      default_if_missing("Type Code Level 2", "");
      default_if_missing("Type Code Level 3", "");
      default_if_missing("Building Component", "");

      inst->properties["Reuse"] = std::move(reuse);

      cameras->elements.push_back(inst);
    }

    root->elements.push_back(cameras);

    models.push_back({"materials", root});
    core::debug("Speckle: prepared 'materials' model ({} passports)",
                scene.materials.size());
  }

  // --- "components" model ---
  if (!scene.components.empty()) {
    auto root = std::make_shared<Collection>();
    root->name = "components";

    // Group by type
    std::map<std::string, std::shared_ptr<Collection>> type_collections;
    for (const auto &entry : scene.components) {
      std::string type_name = entry.properties.count("type")
                                  ? entry.properties.at("type")
                                  : "unknown";
      auto &col = type_collections[type_name];
      if (!col) {
        col = std::make_shared<Collection>();
        col->name = type_name;
      }

      auto mesh = std::make_shared<Mesh>();
      const auto &verts = entry.boundary.vertices;
      if (verts.size() >= 3) {
        for (const auto &v : verts) {
          mesh->vertices.push_back(v.x());
          mesh->vertices.push_back(v.y());
          mesh->vertices.push_back(v.z());
        }
        // Triangle fan from vertex 0
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
          mesh->faces.push_back(0); // triangle indicator
          mesh->faces.push_back(0);
          mesh->faces.push_back(static_cast<int>(i));
          mesh->faces.push_back(static_cast<int>(i + 1));
        }
      }
      mesh->name = entry.name;
      for (const auto &[k, v] : entry.properties)
        mesh->properties[k] = v;

      col->elements.push_back(mesh);
    }

    for (auto &[type_name, col] : type_collections)
      root->elements.push_back(col);

    models.push_back({"components", root});
    core::debug("Speckle: prepared 'components' model ({} components)",
                scene.components.size());
  }

  return models;
}

} // namespace reusex::io::speckle
