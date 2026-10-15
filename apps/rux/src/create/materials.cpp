// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/materials.hpp"
#include "exit_status.hpp"
#include "global-params.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/label_encoding.hpp>
#include <reusex/core/materialepas_json_export.hpp>
#include <reusex/core/materialepas_json_import.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <regex>
#include <set>
#include <string>

using nlohmann::json;

namespace {

/// Parse the semantic class id from an instance definition name of the form
/// "SM{class}-{id} ({n}p)". Returns -1 if it doesn't match.
int parse_semantic_class(const std::string &def) {
  static const std::regex re(R"(SM(\d+)-\d+.*)");
  std::smatch m;
  if (std::regex_match(def, m, re))
    return std::stoi(m[1].str());
  return -1;
}

/// Set the "construction item designation" property value in a passport
/// template JSON (produced by generate_blank_template / to_json).
void set_designation(json &tpl, const std::string &value) {
  if (!tpl.contains("sections"))
    return;
  for (auto &section : tpl["sections"]) {
    if (!section.contains("properties"))
      continue;
    for (auto &prop : section["properties"]) {
      if (prop.value("name", std::string{}) ==
          "construction item designation") {
        prop["value"] = value;
        return;
      }
    }
  }
}

} // namespace

void setup_subcommand_create_materials(CLI::App &parent,
                                       std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandCreateMaterialsOptions>();
  auto *sub = parent.add_subcommand("materials",
                                    "Create a material passport per instance");

  sub->footer(R"(
DESCRIPTION:
  Generates one material passport (Danish "Materialepas") per instance in an
  instance-label cloud, and links each passport's guid back onto the instance
  (stored in the instance_materials table). Each passport's designation is
  prefilled from the instance's semantic class so it is identifiable, and can
  then be enriched via the CSV round-trip ('rux export csv' / 'rux import csv').

EXAMPLES:
  rux create materials                    # One passport per instance
  rux create materials --clear            # Regenerate for all instances
  rux -p scan.rux create materials        # Custom project path

WORKFLOW:
  1. rux create annotate --net model      # Semantic segmentation
  2. rux create clouds                     # Reconstruct labeled cloud
  3. rux create instances                  # Instance segmentation
  4. rux create materials                  # One passport per instance
  5. rux export csv                        # Edit / enrich passports

NOTES:
  - Requires an instance-label cloud (default: 'instances')
  - Skips instances already linked to a passport unless --clear is given
  - Semantic class names come from the semantic cloud's label definitions
)");

  sub->add_option("-i,--instances", opt->instances_cloud_name,
                  "Instance-label cloud name")
      ->default_val(opt->instances_cloud_name);
  sub->add_option("-s,--semantic", opt->semantic_cloud_name,
                  "Semantic-label cloud name (for class naming)")
      ->default_val(opt->semantic_cloud_name);
  sub->add_option("--project-id", opt->project_id,
                  "Project id to associate created passports with (optional)");
  sub->add_flag("--clear", opt->clear,
                "Regenerate passports for instances already linked");

  sub->callback([opt, global_opt]() {
    spdlog::trace("calling create materials subcommand");
    rux::finish(run_subcommand_create_materials(*opt, *global_opt));
  });
}

int run_subcommand_create_materials(SubcommandCreateMaterialsOptions const &opt,
                                    const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    spdlog::info("Opening project database: {}", project_path.string());
    reusex::ProjectDB db(project_path, /*readOnly=*/false);

    // Instance definitions: instance_id -> "SM{class}-{id} ({n}p)".
    auto instance_defs = db.label_definitions(opt.instances_cloud_name);
    if (instance_defs.empty()) {
      spdlog::warn("No instances found in cloud '{}' (run 'rux create "
                   "instances' first)",
                   opt.instances_cloud_name);
      return RuxError::SUCCESS;
    }

    // Semantic class id -> human name (optional; falls back to "class N").
    std::map<int, std::string> semantic_names;
    try {
      semantic_names = db.label_definitions(opt.semantic_cloud_name);
      reusex::core::warn_if_unstamped_semantic_cloud(db,
                                                     opt.semantic_cloud_name);
    } catch (const std::exception &) {
      // Semantic cloud may not carry label definitions; that's fine.
    }

    // Instances whose survey part the user deleted (schema v26 tombstones):
    // minting a passport for one would bring the deleted part's material
    // back, so they are skipped, --clear included.
    std::set<int> dismissed;
    for (const auto &rec : db.instances(opt.instances_cloud_name))
      if (db.is_instance_dismissed(rec.guid))
        dismissed.insert(static_cast<int>(rec.instance_id));

    size_t created = 0, skipped = 0, skipped_dismissed = 0;
    for (const auto &[instance_id, def] : instance_defs) {
      if (dismissed.count(instance_id) != 0) {
        ++skipped_dismissed;
        continue;
      }
      if (auto existing = db.instance_material_guid(opt.instances_cloud_name,
                                                    instance_id)) {
        if (!opt.clear) {
          ++skipped;
          continue;
        }
        // --clear: drop the previously-linked passport so it isn't orphaned.
        try {
          db.delete_material_passport(*existing);
        } catch (const std::exception &) {
          // Already gone; ignore.
        }
      }

      // Resolve a human-readable designation from the semantic class.
      int sem_class = parse_semantic_class(def);
      std::string designation;
      auto nit = semantic_names.find(sem_class);
      if (nit != semantic_names.end())
        designation = nit->second;
      else if (sem_class >= 0)
        designation = fmt::format("class {}", sem_class);
      else
        designation = def; // fall back to the raw instance definition

      // Fresh passport with a new guid + dates; prefill the designation.
      json tpl = reusex::core::json_export::generate_blank_template();
      set_designation(tpl, designation);
      std::string guid = tpl["metadata"]["document guid"].get<std::string>();

      auto passport = reusex::core::json_import::from_json(tpl);
      db.add_material_passport(passport, opt.project_id);
      db.set_instance_material(opt.instances_cloud_name, instance_id, guid);
      ++created;
    }

    spdlog::info("Created {} passport(s), skipped {} already-linked and {} "
                 "dismissed instance(s)",
                 created, skipped, skipped_dismissed);
    return RuxError::SUCCESS;
  } catch (const std::exception &e) {
    spdlog::error("create materials failed: {}", e.what());
    return RuxError::GENERIC;
  }
}
