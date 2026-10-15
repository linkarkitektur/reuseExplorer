// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/pipeline_ui.hpp>

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>

#include <cmath>
#include <map>

namespace rux::qt {

namespace pl = reusex::pipeline;

QString stage_name_da(pl::JobStage stage) {
  switch (stage) {
  case pl::JobStage::clouds:
    return "Punktskyer";
  case pl::JobStage::planes:
    return "Planer";
  case pl::JobStage::rooms:
    return "Rum";
  case pl::JobStage::instances:
    return "Instanser";
  case pl::JobStage::mesh:
    return "Mesh";
  case pl::JobStage::optimize:
    return "Optimér poser";
  case pl::JobStage::graph_discover:
    return "Find kanter";
  case pl::JobStage::storeys:
    return "Etager";
  }
  return {};
}

QString stage_blurb_da(pl::JobStage stage) {
  switch (stage) {
  case pl::JobStage::clouds:
    return "Projicér dybdebillederne til én samlet punktsky med normaler.";
  case pl::JobStage::planes:
    return "Find plane flader — vægge, gulve og lofter — i punktskyen.";
  case pl::JobStage::rooms:
    return "Del planerne op i rum med Leiden-klyngedannelse.";
  case pl::JobStage::instances:
    return "Del de semantiske mærkater op i enkelte genstande i rummet.";
  case pl::JobStage::mesh:
    return "Løs cellekomplekset til et vandtæt mesh (MIP).";
  case pl::JobStage::optimize:
    return "Forfin billedernes poser med plan-landemærker i en posegraf.";
  case pl::JobStage::graph_discover:
    return "Find nye løkkekanter (ORB, 360°-panoramaer, kantfil) og gem dem i "
           "posegrafen.";
  case pl::JobStage::storeys:
    return "Find etagerne: gulv- og loftshøjder og den målte lodrette "
           "retning (niveauramme).";
  }
  return {};
}

QString parameter_label_da(pl::JobStage stage,
                           const pl::ParameterDescriptor &d) {
  static const std::map<std::string, const char *> common = {
      {"filter", "Punktfilter"},
  };
  static const std::map<std::pair<int, std::string>, const char *> labels = {
      {{0, "resolution"}, "Voxelstørrelse [m]"},
      {{0, "min_distance"}, "Mindste dybde [m]"},
      {{0, "max_distance"}, "Største dybde [m]"},
      {{0, "sampling_factor"}, "Pixeludtynding"},
      {{0, "confidence_threshold"}, "Mindste konfidens"},
      {{1, "angle_threshold"}, "Vinkeltærskel [°]"},
      {{1, "plane_dist_threshold"}, "Afstandstærskel [m]"},
      {{1, "min_inliers"}, "Mindste plan [punkter]"},
      {{1, "radius"}, "Vækstradius [m]"},
      {{1, "interval_0"}, "Første genberegning"},
      {{1, "interval_factor"}, "Genberegningsfaktor"},
      {{1, "adaptive"}, "Tilpas tærskler til støjen"},
      {{1, "noise_seed"}, "Frø til støjestimat"},
      {{2, "grid_size"}, "Gitterstørrelse [m]"},
      {{2, "resolution"}, "Leiden-opløsning"},
      {{2, "beta"}, "Leiden-beta"},
      {{2, "max_iter"}, "Højst iterationer"},
      {{2, "propagate_k"}, "Naboer ved udbredelse"},
      {{2, "propagate_max_radius"}, "Udbredelsesradius [m]"},
      {{3, "semantic_cloud"}, "Semantisk sky"},
      {{3, "output_cloud"}, "Resultatsky"},
      {{3, "cluster_tolerance"}, "Klyngeafstand [m]"},
      {{3, "min_cluster_size"}, "Mindste instans [punkter]"},
      {{3, "max_cluster_size"}, "Største instans [punkter]"},
      {{3, "labels"}, "Kun disse mærkater"},
      {{4, "solver"}, "MIP-løser"},
      {{4, "time_limit_seconds"}, "Tidsgrænse [s]"},
      {{4, "output_name"}, "Meshets navn"},
      {{4, "search_threshold"}, "Søgeafstand [m]"},
      {{4, "new_plane_offset"}, "Afstand til ny plan [m]"},
      {{4, "alpha"}, "Vægt på vægkompleksitet"},
      {{4, "max_cells"}, "Højst celler"},
      {{4, "sectioned"}, "Løs etage for etage"},
      {{4, "sectioned_threshold"}, "Etagevis fra [celler]"},
      {{5, "min_observations"}, "Mindst set af [billeder]"},
      {{5, "assoc_rounds"}, "Tilknytningsrunder"},
      {{5, "no_gnc"}, "Uden GNC (hurtigere, mindre robust)"},
      {{5, "dry_run"}, "Prøvekørsel — skriv ikke poserne"},
      {{7, "loop_closure"}, "ORB-løkkelukning"},
      {{7, "use_panoramas"}, "Brug 360°-panoramaer"},
      {{7, "loop_edges_file"}, "Kantfil (JSON)"},
      {{7, "loop_proposal"}, "Kandidatforslag"},
      {{7, "loop_min_frame_gap"}, "Mindste billedafstand"},
      {{7, "loop_min_inliers"}, "Mindste inliers"},
      {{7, "pcm"}, "Konsistensfilter (PCM)"},
      {{7, "dry_run"}, "Prøvekørsel — skriv ikke kanterne"},
      {{8, "horizontal_cos"}, "Vandret når |n·z| over"},
      {{8, "inlier_m"}, "Inlierbånd ved tilpasning [m]"},
      {{8, "max_fit_iterations"}, "Højst gentilpasninger"},
      {{8, "hist_bin_m"}, "Histograminterval [m]"},
      {{8, "peak_min_share"}, "Mindste andel for et niveau"},
      {{8, "slab_merge_m"}, "Dæk: slå niveauer sammen under [m]"},
      {{8, "min_storey_height_m"}, "Mindste etagehøjde [m]"},
      {{8, "default_height_m"}, "Etagehøjde uden loft [m]"},
  };
  if (auto it = labels.find({static_cast<int>(stage), d.key});
      it != labels.end())
    return it->second;
  if (auto it = common.find(d.key); it != common.end())
    return it->second;
  return QString::fromStdString(d.label);
}

QString local_time_da(const std::string &iso_utc) {
  const QString text = QString::fromStdString(iso_utc);
  QDateTime dt = QDateTime::fromString(text, Qt::ISODate);
  if (!dt.isValid())
    return text;
  return QLocale(QLocale::Danish, QLocale::Denmark)
      .toString(dt.toLocalTime(), "d. MMM yyyy HH:mm");
}

reusex::core::PipelineStage contract_stage(pl::JobStage stage) {
  using P = reusex::core::PipelineStage;
  switch (stage) {
  case pl::JobStage::clouds:
    return P::clouds;
  case pl::JobStage::planes:
    return P::planes;
  case pl::JobStage::rooms:
    return P::rooms;
  case pl::JobStage::instances:
    return P::instances;
  case pl::JobStage::mesh:
    return P::mesh;
  case pl::JobStage::optimize:
    return P::optimize;
  case pl::JobStage::graph_discover:
    return P::graph_discover;
  case pl::JobStage::storeys:
    return P::storeys;
  }
  return P::clouds;
}

std::optional<pl::JobStage> stage_from_log_name(std::string_view log_name) {
  for (const auto &name : pl::job_stage_names()) {
    const auto s = pl::parse_job_stage(name);
    if (s && pl::pipeline_log_name(*s) == log_name)
      return s;
  }
  return std::nullopt;
}

bool is_default(const pl::ParameterDescriptor &d, const QJsonValue &v) {
  const auto &def = d.default_value;
  if (std::holds_alternative<std::monostate>(def))
    return v.isNull() || v.isUndefined() ||
           (v.isString() && v.toString().isEmpty()) ||
           (v.isArray() && v.toArray().isEmpty());
  if (const auto *x = std::get_if<double>(&def))
    return v.isDouble() &&
           static_cast<float>(v.toDouble()) == static_cast<float>(*x);
  if (const auto *x = std::get_if<long long>(&def))
    return v.isDouble() && std::llround(v.toDouble()) == *x;
  if (const auto *x = std::get_if<bool>(&def))
    return v.isBool() && v.toBool() == *x;
  if (const auto *x = std::get_if<std::string>(&def))
    return v.isString() && v.toString().toStdString() == *x;
  return false;
}

std::optional<CliParam> cli_param(const pl::ParameterDescriptor &d,
                                  const QJsonValue &v) {
  CliParam p;
  p.key = d.key;
  switch (d.type) {
  case pl::ParameterType::number:
    if (!v.isDouble())
      return std::nullopt;
    p.kind = CliValueKind::number;
    // The option structs hold floats: print the float's shortest form.
    p.value = format_number(static_cast<float>(v.toDouble()));
    return p;
  case pl::ParameterType::integer:
    if (!v.isDouble())
      return std::nullopt;
    p.kind = CliValueKind::integer;
    p.value = std::to_string(std::llround(v.toDouble()));
    return p;
  case pl::ParameterType::boolean:
    if (!v.isBool())
      return std::nullopt;
    p.kind = CliValueKind::boolean;
    p.value = v.toBool() ? "true" : "false";
    return p;
  case pl::ParameterType::string:
    if (!v.isString())
      return std::nullopt;
    p.kind = CliValueKind::string;
    p.value = v.toString().toStdString();
    return p;
  case pl::ParameterType::integer_list: {
    if (!v.isArray())
      return std::nullopt;
    p.kind = CliValueKind::integer_list;
    for (const auto &e : v.toArray()) {
      if (!p.value.empty())
        p.value += ',';
      p.value += std::to_string(std::llround(e.toDouble()));
    }
    return p;
  }
  }
  return std::nullopt;
}

CliCommand cli_for_parameters(pl::JobStage stage, const QString &json,
                              const QString &project) {
  const QJsonObject obj = QJsonDocument::fromJson(json.toUtf8()).object();
  const std::string stage_name(pl::to_string(stage));
  std::vector<CliParam> params;
  for (const auto &d : pl::stage_parameters(stage)) {
    const QString key = QString::fromStdString(d.key);
    if (!obj.contains(key))
      continue;
    const QJsonValue v = obj.value(key);
    if (!d.presence_sensitive && is_default(d, v))
      continue;
    if (auto p = cli_param(d, v))
      params.push_back(*p);
  }
  // Keys the CLI writes that no descriptor lists (clouds' glass filter).
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    const std::string key = it.key().toStdString();
    bool known = key == "job_id";
    for (const auto &d : pl::stage_parameters(stage))
      known = known || d.key == key;
    if (known || cli_flag_for(stage_name, key).empty())
      continue;
    const QJsonValue v = it.value();
    if (v.isBool()) {
      if (v.toBool()) // false is the CLI default of every such flag
        params.push_back({key, CliValueKind::boolean, "true"});
    } else if (v.isDouble()) {
      params.push_back({key, CliValueKind::number,
                        format_number(static_cast<float>(v.toDouble()))});
    } else if (v.isString()) {
      params.push_back({key, CliValueKind::string, v.toString().toStdString()});
    }
  }
  return build_cli_command(stage_name, project.toStdString(), params);
}

} // namespace rux::qt
