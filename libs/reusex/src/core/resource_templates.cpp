// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/resource_templates.hpp"

#include "reusex/core/logging.hpp"

#include <algorithm>
#include <initializer_list>
#include <set>

namespace reusex::core {

std::vector<TemplateMember> parse_members(const nlohmann::json &members) {
  if (!members.is_array())
    throw std::invalid_argument("'members' must be an array");
  std::vector<TemplateMember> out;
  for (std::size_t i = 0; i < members.size(); ++i) {
    const auto &m = members[i];
    const std::string bad = "members[" + std::to_string(i) +
                            "] must be {\"category\": name} or {\"key\": id}";
    if (!m.is_object() || m.size() != 1)
      throw std::invalid_argument(bad);
    const auto it = m.begin();
    if ((it.key() != "category" && it.key() != "key") || !it->is_string() ||
        it->get<std::string>().empty())
      throw std::invalid_argument(bad);
    out.push_back(
        {it.key() == "category" ? MemberKind::category : MemberKind::key,
         it->get<std::string>()});
  }
  return out;
}

std::vector<TemplateMember> read_members(std::string_view stored,
                                         std::string_view template_name) {
  const auto j = nlohmann::json::parse(stored, nullptr, false);
  try {
    if (!j.is_discarded())
      return parse_members(j);
  } catch (const std::invalid_argument &e) {
    reusex::warn("template '{}': stored members unreadable ({}); treated as "
                 "empty",
                 template_name, e.what());
    return {};
  }
  reusex::warn("template '{}': stored members are not JSON; treated as empty",
               template_name);
  return {};
}

nlohmann::json members_json(const std::vector<TemplateMember> &members) {
  auto out = nlohmann::json::array();
  for (const auto &m : members) {
    nlohmann::json obj = nlohmann::json::object();
    obj[m.kind == MemberKind::category ? "category" : "key"] = m.ref;
    out.push_back(std::move(obj));
  }
  return out;
}

ResolvedTemplate resolve_template(const std::vector<TemplateMember> &members,
                                  const std::vector<ResourceKey> &catalogue) {
  ResolvedTemplate out;
  std::set<std::string> seen;
  auto add = [&](const std::string &id) {
    if (seen.insert(id).second)
      out.keys.push_back(id);
  };
  for (const auto &m : members) {
    if (m.kind == MemberKind::key) {
      if (find_key(catalogue, m.ref))
        add(m.ref);
      else
        out.missing.push_back(m);
      continue;
    }
    bool any = false;
    for (const auto &k : catalogue)
      if (k.category == m.ref) {
        add(k.id);
        any = true;
      }
    if (!any)
      out.missing.push_back(m);
  }
  return out;
}

std::vector<SeedTemplate> seed_templates() {
  SeedTemplate full{std::string(kSeedMaterialepas), "Materialepas (fuld)", {}};
  for (auto &c : leksikon_categories())
    full.members.push_back({MemberKind::category, std::move(c)});
  SeedTemplate screening{
      std::string(kSeedScreening), "Hurtig genbrugsscreening", {}};
  // The screening seed lists the built-in keys a reader scans; the view
  // record is an image, shown by the report itself (sub-project C).
  for (const auto &k : builtin_keys())
    if (k.id != kViewKey)
      screening.members.push_back({MemberKind::key, k.id});
  return {full, screening};
}

namespace {
void pick(const nlohmann::json &csv, const char *field, std::string &target,
          std::initializer_list<const char *> allowed) {
  const auto it = csv.find(field);
  if (it == csv.end())
    return;
  if (it->is_string())
    for (const char *a : allowed)
      if (it->get<std::string>() == a) {
        target = a;
        return;
      }
  std::string list;
  for (const char *a : allowed)
    list += (list.empty() ? "" : ", ") + nlohmann::json(a).dump();
  throw std::invalid_argument(std::string("'csv.") + field +
                              "' must be one of " + list);
}
} // namespace

CsvOptions parse_csv_options(const nlohmann::json &csv) {
  if (!csv.is_object())
    throw std::invalid_argument("'csv' must be an object");
  CsvOptions o;
  pick(csv, "delimiter", o.delimiter, {";", ",", "\t"});
  pick(csv, "encoding", o.encoding, {"utf-8", "utf-8-bom"});
  pick(csv, "header", o.header, {"label", "key"});
  for (auto it = csv.begin(); it != csv.end(); ++it)
    if (it.key() != "delimiter" && it.key() != "encoding" &&
        it.key() != "header")
      o.extra[it.key()] = *it;
  return o;
}

CsvOptions read_csv_options(std::string_view stored,
                            std::string_view template_name) {
  auto j = nlohmann::json::parse(stored, nullptr, false);
  if (j.is_discarded() || !j.is_object()) {
    reusex::warn("template '{}': stored CSV options unreadable; using "
                 "defaults",
                 template_name);
    return {};
  }
  for (const char *field : {"delimiter", "encoding", "header"}) {
    if (!j.contains(field))
      continue;
    nlohmann::json probe = nlohmann::json::object();
    probe[field] = j[field];
    try {
      (void)parse_csv_options(probe);
    } catch (const std::invalid_argument &e) {
      reusex::warn("template '{}': stored CSV option ignored ({}); using the "
                   "default",
                   template_name, e.what());
      j.erase(field);
    }
  }
  return parse_csv_options(j);
}

nlohmann::json csv_options_json(const CsvOptions &o) {
  nlohmann::json out = o.extra.is_object() ? o.extra : nlohmann::json::object();
  out["delimiter"] = o.delimiter;
  out["encoding"] = o.encoding;
  out["header"] = o.header;
  return out;
}

TemplateMember legacy_column_member(std::string_view column,
                                    const std::vector<ResourceKey> &catalogue) {
  for (const auto &k : catalogue)
    if (k.source == KeySource::column && k.label == column)
      return {MemberKind::key, k.id};
  for (const auto &k : catalogue)
    if (k.label == column)
      return {MemberKind::key, k.id};
  for (const auto &k : catalogue)
    if (k.source == KeySource::leksikon && k.field == column)
      return {MemberKind::key, k.id};
  return {MemberKind::key, std::string(kLegacyKeyPrefix) + std::string(column)};
}

std::string unique_name(std::string_view base, std::string_view tag,
                        const std::vector<std::string> &taken) {
  auto is_taken = [&](const std::string &n) {
    return std::find(taken.begin(), taken.end(), n) != taken.end();
  };
  const std::string stem = std::string(base) + " (" + std::string(tag);
  std::string candidate = stem + ")";
  for (int n = 2; is_taken(candidate); ++n)
    candidate = stem + " " + std::to_string(n) + ")";
  return candidate;
}

namespace {
std::vector<std::string> template_names(const ProjectDB &db) {
  std::vector<std::string> out;
  for (const auto &t : db.resource_templates())
    out.push_back(t.name);
  return out;
}

TemplateView make_view(const ProjectDB::ResourceTemplateRecord &rec,
                       const std::vector<ResourceKey> &catalogue) {
  TemplateView v{rec,
                 read_members(rec.members_json, rec.name),
                 read_csv_options(rec.csv_json, rec.name),
                 {}};
  v.resolved = resolve_template(v.members, catalogue);
  if (!v.resolved.missing.empty()) {
    std::string refs;
    for (const auto &m : v.resolved.missing)
      refs += (refs.empty() ? "" : ", ") + m.ref;
    reusex::warn("template '{}' (id {}): {} member(s) no longer resolve and "
                 "are skipped: {}",
                 rec.name, rec.id, v.resolved.missing.size(), refs);
  }
  return v;
}
} // namespace

std::vector<TemplateView> template_views(const ProjectDB &db) {
  const auto catalogue = key_catalogue(db);
  std::vector<TemplateView> out;
  for (const auto &rec : db.resource_templates())
    out.push_back(make_view(rec, catalogue));
  return out;
}

TemplateView template_view(const ProjectDB &db, int64_t id) {
  const auto rec = db.resource_template(id);
  if (!rec)
    throw std::out_of_range("no template " + std::to_string(id));
  return make_view(*rec, key_catalogue(db));
}

TemplateView create_template(ProjectDB &db, const TemplateInput &in) {
  if (!in.name || in.name->empty())
    throw std::invalid_argument("'name' is required and must be non-empty");
  ProjectDB::ResourceTemplateRecord rec;
  rec.name = *in.name;
  if (in.members)
    rec.members_json = members_json(parse_members(*in.members)).dump();
  if (in.csv)
    rec.csv_json = csv_options_json(parse_csv_options(*in.csv)).dump();
  return template_view(db, db.add_resource_template(rec).id);
}

TemplateView update_template(ProjectDB &db, int64_t id,
                             const TemplateInput &in) {
  if (!db.resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
  ProjectDB::ResourceTemplatePatch p;
  if (in.name) {
    if (in.name->empty())
      throw std::invalid_argument("'name' must be non-empty");
    p.name = *in.name;
  }
  if (in.members)
    p.members_json = members_json(parse_members(*in.members)).dump();
  if (in.csv)
    p.csv_json = csv_options_json(parse_csv_options(*in.csv)).dump();
  db.update_resource_template(id, p);
  return template_view(db, id);
}

void delete_template(ProjectDB &db, int64_t id) {
  if (!db.delete_resource_template(id))
    throw std::out_of_range("no template " + std::to_string(id));
}

TemplateView duplicate_template(ProjectDB &db, int64_t id) {
  const auto src = db.resource_template(id);
  if (!src)
    throw std::out_of_range("no template " + std::to_string(id));
  ProjectDB::ResourceTemplateRecord copy;
  copy.name = unique_name(src->name, "kopi", template_names(db));
  copy.members_json = src->members_json;
  copy.csv_json = src->csv_json;
  return template_view(db, db.add_resource_template(copy).id);
}

std::vector<TemplateView> restore_seed_templates(ProjectDB &db) {
  std::vector<TemplateView> out;
  // One transaction: every missing seed lands, or none does.
  ProjectDB::Transaction tx(db);
  const auto existing = db.resource_templates();
  auto names = template_names(db);
  for (const auto &seed : seed_templates()) {
    const bool present =
        std::any_of(existing.begin(), existing.end(),
                    [&](const auto &r) { return r.seed == seed.tag; });
    if (present)
      continue;
    ProjectDB::ResourceTemplateRecord rec;
    rec.name = std::find(names.begin(), names.end(), seed.name) == names.end()
                   ? seed.name
                   : unique_name(seed.name, "standard", names);
    rec.members_json = members_json(seed.members).dump();
    rec.seed = seed.tag;
    names.push_back(rec.name);
    out.push_back(template_view(db, db.add_resource_template(rec).id));
  }
  tx.commit();
  return out;
}

} // namespace reusex::core
