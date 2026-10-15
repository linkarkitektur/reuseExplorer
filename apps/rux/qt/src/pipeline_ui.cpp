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
  // Keyed by the stage's wire name (pl::to_string), not its enum value.
  static const std::map<std::pair<std::string, std::string>, const char *>
      labels = {
          {{"clouds", "resolution"}, "Voxelstørrelse [m]"},
          {{"clouds", "min_distance"}, "Mindste dybde [m]"},
          {{"clouds", "max_distance"}, "Største dybde [m]"},
          {{"clouds", "sampling_factor"}, "Pixeludtynding"},
          {{"clouds", "confidence_threshold"}, "Mindste konfidens"},
          {{"planes", "angle_threshold"}, "Vinkeltærskel [°]"},
          {{"planes", "plane_dist_threshold"}, "Afstandstærskel [m]"},
          {{"planes", "min_inliers"}, "Mindste plan [punkter]"},
          {{"planes", "radius"}, "Vækstradius [m]"},
          {{"planes", "interval_0"}, "Første genberegning"},
          {{"planes", "interval_factor"}, "Genberegningsfaktor"},
          {{"planes", "adaptive"}, "Tilpas tærskler til støjen"},
          {{"planes", "noise_seed"}, "Frø til støjestimat"},
          {{"rooms", "grid_size"}, "Gitterstørrelse [m]"},
          {{"rooms", "resolution"}, "Leiden-opløsning"},
          {{"rooms", "beta"}, "Leiden-beta"},
          {{"rooms", "max_iter"}, "Højst iterationer"},
          {{"rooms", "propagate_k"}, "Naboer ved udbredelse"},
          {{"rooms", "propagate_max_radius"}, "Udbredelsesradius [m]"},
          {{"rooms", "min_room_area"}, "Mindste rum [m²]"},
          {{"instances", "semantic_cloud"}, "Semantisk sky"},
          {{"instances", "output_cloud"}, "Resultatsky"},
          {{"instances", "cluster_tolerance"}, "Klyngeafstand [m]"},
          {{"instances", "min_cluster_size"}, "Mindste instans [punkter]"},
          {{"instances", "max_cluster_size"}, "Største instans [punkter]"},
          {{"instances", "labels"}, "Kun disse mærkater"},
          {{"mesh", "solver"}, "MIP-løser"},
          {{"mesh", "time_limit_seconds"}, "Tidsgrænse [s]"},
          {{"mesh", "output_name"}, "Meshets navn"},
          {{"mesh", "search_threshold"}, "Søgeafstand [m]"},
          {{"mesh", "new_plane_offset"}, "Afstand til ny plan [m]"},
          {{"mesh", "alpha"}, "Vægt på vægkompleksitet"},
          {{"mesh", "max_cells"}, "Højst celler"},
          {{"mesh", "sectioned"}, "Løs etage for etage"},
          {{"mesh", "sectioned_threshold"}, "Etagevis fra [celler]"},
          {{"optimize", "min_observations"}, "Mindst set af [billeder]"},
          {{"optimize", "assoc_rounds"}, "Tilknytningsrunder"},
          {{"optimize", "no_gnc"}, "Uden GNC (hurtigere, mindre robust)"},
          {{"optimize", "dry_run"}, "Prøvekørsel — skriv ikke poserne"},
          {{"graph_discover", "loop_closure"}, "ORB-løkkelukning"},
          {{"graph_discover", "use_panoramas"}, "Brug 360°-panoramaer"},
          {{"graph_discover", "loop_edges_file"}, "Kantfil (JSON)"},
          {{"graph_discover", "loop_proposal"}, "Kandidatforslag"},
          {{"graph_discover", "loop_min_frame_gap"}, "Mindste billedafstand"},
          {{"graph_discover", "loop_min_inliers"}, "Mindste inliers"},
          {{"graph_discover", "pcm"}, "Konsistensfilter (PCM)"},
          {{"graph_discover", "dry_run"}, "Prøvekørsel — skriv ikke kanterne"},
          {{"storeys", "horizontal_cos"}, "Vandret når |n·z| over"},
          {{"storeys", "surface_max_tilt_deg"}, "Gulv/loft højst hældning [°]"},
          {{"storeys", "inlier_m"}, "Inlierbånd ved tilpasning [m]"},
          {{"storeys", "max_fit_iterations"}, "Højst gentilpasninger"},
          {{"storeys", "hist_bin_m"}, "Højdeinterval [m]"},
          {{"storeys", "peak_min_share"}, "Mindste andel for et niveau"},
          {{"storeys", "slab_merge_m"}, "Dæk: slå niveauer sammen under [m]"},
          {{"storeys", "min_storey_height_m"}, "Mindste etagehøjde [m]"},
          {{"storeys", "default_height_m"}, "Etagehøjde uden loft [m]"},
          {{"storeys", "use_normal_sign"},
           "Gulv/loft ud fra normalens retning"},
          {{"storeys", "per_storey_frame"}, "Egen lodret for hver etage"},
          {{"storeys", "storey_frame_tol_deg"},
           "Egen lodret over afvigelse [°]"},
          {{"storeys", "guid_match_m"}, "Samme etage inden for [m]"},
  };
  if (auto it = labels.find({std::string(pl::to_string(stage)), d.key});
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
