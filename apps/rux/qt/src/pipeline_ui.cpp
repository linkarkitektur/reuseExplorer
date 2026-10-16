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
  case pl::JobStage::classify:
    return "Klassificér planer";
  case pl::JobStage::walls:
    return "Vægge";
  case pl::JobStage::openings:
    return "Åbninger";
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
  case pl::JobStage::classify:
    return "Giv hver plan en rolle — væg, gulv, loft, møbel — ud fra "
           "semantik og geometri.";
  case pl::JobStage::walls:
    return "Find væggene: akse, målt tykkelse og hvordan de mødes (hjørner, "
           "T-samlinger).";
  case pl::JobStage::openings:
    return "Find døre, vinduer og åbninger i væggene: hvor væggen mangler og "
           "intet står foran den.";
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
          {{"rooms", "merge_affinity"}, "Rumsammenlægning"},
          {{"instances", "semantic_cloud"}, "Semantisk sky"},
          {{"instances", "output_cloud"}, "Resultatsky"},
          {{"instances", "cluster_tolerance"}, "Klyngeafstand [m]"},
          {{"instances", "min_cluster_size"}, "Mindste instans [punkter]"},
          {{"instances", "max_cluster_size"}, "Største instans [punkter]"},
          {{"instances", "labels"}, "Kun disse mærkater"},
          {{"mesh", "solver"}, "MIP-løser"},
          {{"mesh", "cuopt_max_variables"}, "cuOpt-grænse [variable]"},
          {{"mesh", "time_limit_seconds"}, "Tidsgrænse [s]"},
          {{"mesh", "output_name"}, "Meshets navn"},
          {{"mesh", "search_threshold"}, "Søgeafstand [m]"},
          {{"mesh", "new_plane_offset"}, "Afstand til ny plan [m]"},
          {{"mesh", "alpha"}, "Vægt på vægkompleksitet"},
          {{"mesh", "max_cells"}, "Højst celler"},
          {{"mesh", "sectioned"}, "Løs etage for etage"},
          {{"mesh", "sectioned_threshold"}, "Etagevis fra [celler]"},
          {{"mesh", "structural_filter"}, "Kun bærende planer (vægge, etager)"},
          {{"mesh", "structural_min_wall_area"}, "Mindste vægflade [m²]"},
          {{"mesh", "structural_collinear_offset"},
           "Saml vægstykker inden for [m]"},
          {{"mesh", "structural_storey_clearance"},
           "Møbelhøjde under etage [m]"},
          {{"mesh", "structural_footprint_overlap"}, "Møbel over gulv, andel"},
          {{"mesh", "plane_classes"}, "Udelad møbel- og rodplaner"},
          {{"optimize", "min_observations"}, "Mindst set af [billeder]"},
          {{"optimize", "assoc_rounds"}, "Tilknytningsrunder"},
          {{"optimize", "no_gnc"}, "Uden GNC (hurtigere, mindre robust)"},
          {{"optimize", "manhattan"}, "Retvinklet bygning (Manhattan)"},
          {{"optimize", "manhattan_sigma_deg"}, "Manhattan-usikkerhed [°]"},
          {{"optimize", "manhattan_max_angle_deg"}, "Manhattan-grænse [°]"},
          {{"optimize", "manhattan_level_max_angle_deg"},
           "Manhattan-grænse, vandret [°]"},
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
          {{"classify", "horizontal_cos"}, "Vandret når |n·op| over"},
          {{"classify", "vertical_cos"}, "Lodret når |n·op| under"},
          {{"classify", "min_large_area_m2"}, "Mindste gulv/loft [m²]"},
          {{"classify", "band_m"}, "Bånd om gulv og loft [m]"},
          {{"classify", "min_wall_height_m"}, "Mindste væghøjde [m]"},
          {{"classify", "min_wall_height_share"}, "… eller andel af etagen"},
          {{"classify", "max_furniture_area_m2"}, "Største møbelflade [m²]"},
          {{"classify", "table_min_m"}, "Laveste bordplade [m]"},
          {{"classify", "table_max_m"}, "Højeste bordplade [m]"},
          {{"classify", "min_storey_height_m"}, "Mindste etagehøjde [m]"},
          {{"classify", "area_cell_m"}, "Arealgitter [m]"},
          {{"classify", "extent_quantile"}, "Højdekvantil"},
          {{"classify", "min_labelled_share"}, "Mindste mærkede andel"},
          {{"classify", "min_class_share"}, "Mindste klasseandel"},
          {{"classify", "role_map"}, "Klasse → rolle (JSON-fil)"},
          {{"walls", "max_families"}, "Ekstra vægretninger"},
          {{"walls", "family_angle_deg"}, "Retningsvindue [°]"},
          {{"walls", "min_family_share"}, "Mindste andel for en retning"},
          {{"walls", "wall_role_only"}, "Kun vægplaner giver retninger"},
          {{"walls", "family_refine_iterations"}, "Forfiningsrunder"},
          {{"walls", "family_refine_deg"}, "Smalleste forfiningsvindue [°]"},
          {{"walls", "assign_angle_deg"}, "Tilknyt punkter inden for [°]"},
          {{"walls", "square_deg"}, "Ret vægplaner op inden for [°]"},
          {{"walls", "bin_m"}, "Profilinterval [m]"},
          {{"walls", "min_peak_points"}, "Mindste top [punkter]"},
          {{"walls", "min_peak_prominence_share"}, "Mindste topfremtræden"},
          {{"walls", "min_peak_separation_m"}, "Mindste topafstand [m]"},
          {{"walls", "face_window_m"}, "Fladevindue [m]"},
          {{"walls", "min_column_share"}, "Mindste søjleandel"},
          {{"walls", "run_gap_m"}, "Del ved hul over [m]"},
          {{"walls", "min_run_m"}, "Korteste vægstykke [m]"},
          {{"walls", "min_run_points"}, "Færrest punkter pr. stykke"},
          {{"walls", "cell_m"}, "Celle [m]"},
          {{"walls", "storey_margin_m"}, "Margin om etagen [m]"},
          {{"walls", "min_gap_m"}, "Tyndeste væg [m]"},
          {{"walls", "max_gap_m"}, "Tykkeste væg [m]"},
          {{"walls", "min_shared_m"}, "Mindste fælles længde [m]"},
          {{"walls", "max_valley_share"}, "Største dalandel"},
          {{"walls", "min_co_visibility"}, "Mindste fælles synlighed"},
          {{"walls", "max_crossed_gap_m"}, "Krydset par under [m]"},
          {{"walls", "min_cell_points"}, "Mindste punkter pr. celle"},
          {{"walls", "outlier_sigma_k"}, "Udligger ved sigma"},
          {{"walls", "min_robust_sigma_m"}, "Mindste robuste sigma [m]"},
          {{"walls", "bootstrap_samples"}, "Bootstrap-gentagelser"},
          {{"walls", "seed"}, "Frø (bootstrap)"},
          {{"walls", "ci_level"}, "Konfidensniveau"},
          {{"walls", "default_thickness_m"}, "Antaget tykkelse [m]"},
          {{"walls", "external_strip_near_m"}, "Ydervægsbånd fra [m]"},
          {{"walls", "external_strip_far_m"}, "Ydervægsbånd til [m]"},
          {{"walls", "external_max_z_m"}, "Ydervægsbånd højde [m]"},
          {{"walls", "min_room_density"}, "Mindste rumtæthed [pkt/m²]"},
          {{"walls", "snap_wide_m"}, "Fang (bred) [m]"},
          {{"walls", "snap_narrow_m"}, "Fang (smal) [m]"},
          {{"walls", "bridge_gap_m"}, "Bro over hul [m]"},
          {{"walls", "bridge_offset_m"}, "Bro: forskydning [m]"},
          {{"walls", "bridge_thickness_m"}, "Bro: tykkelsesforskel [m]"},
          {{"walls", "recess_max_length_m"}, "Længste niche [m]"},
          {{"walls", "corner_m"}, "Hjørne inden for [m]"},
          {{"walls", "tee_m"}, "T-samling inden for [m]"},
          {{"walls", "tee_on_wall_m"}, "T-samling tolerance [m]"},
          {{"walls", "min_seen_share"}, "Mindste sete andel"},
          {{"walls", "guid_match_m"}, "Samme væg inden for [m]"},
          {{"walls", "guid_match_angle_deg"}, "Samme væg vinkel [°]"},
          {{"walls", "guid_min_overlap"}, "Samme væg overlap"},
          {{"openings", "cell_m"}, "Rastercelle [m]"},
          {{"openings", "slab_margin_m"}, "Vægmargin [m]"},
          {{"openings", "unknown_half_thickness_m"},
           "Ukendt halv tykkelse [m]"},
          {{"openings", "front_m"}, "Skjult af ting inden for [m]"},
          {{"openings", "floor_clear_m"}, "Fri af gulv [m]"},
          {{"openings", "ceiling_clear_m"}, "Fri af loft [m]"},
          {{"openings", "local_levels"}, "Mål gulv og loft lokalt"},
          {{"openings", "level_search_m"}, "Søg gulv/loft inden for [m]"},
          {{"openings", "min_level_points"}, "Mindste punkter i gulvlag"},
          {{"openings", "sill_snap_m"}, "Bundkarm til gulv under [m]"},
          {{"openings", "min_wall_points"}, "Mindste vægpunkter"},
          {{"openings", "min_occluder_points"}, "Mindste skjulende punkter"},
          {{"openings", "morph_radius"}, "Morfologiradius [celler]"},
          {{"openings", "min_width_m"}, "Smalleste åbning [m]"},
          {{"openings", "min_height_m"}, "Laveste åbning [m]"},
          {{"openings", "min_fill"}, "Mindste fyldning"},
          {{"openings", "min_open_share"}, "Mindste gennemsete andel"},
          {{"openings", "min_jamb_share"}, "Mindste karmandel"},
          {{"openings", "drop_top_bands"}, "Drop usete topbånd"},
          {{"openings", "top_band_max_m"}, "Topbånd under [m]"},
          {{"openings", "min_floor_height_m"}, "Laveste åbning fra gulv [m]"},
          {{"openings", "door_max_sill_m"}, "Højeste dørtrin [m]"},
          {{"openings", "door_min_height_m"}, "Laveste dør [m]"},
          {{"openings", "door_max_height_m"}, "Højeste dør [m]"},
          {{"openings", "full_height_tol_m"}, "Gulv-til-loft tolerance [m]"},
          {{"openings", "mullion_max_gap_m"}, "Bredeste sprosse [m]"},
          {{"openings", "mullion_max_dz_m"}, "Sprosse: højdeforskel [m]"},
          {{"openings", "evidence_band_m"}, "Evidensbånd [m]"},
          {{"openings", "min_class_share"}, "Mindste klasseandel"},
          {{"openings", "min_instance_points"}, "Mindste instanspunkter"},
          {{"openings", "min_glass_share"}, "Mindste glasandel"},
          {{"openings", "crossing_max_step_m"}, "Længste kameraskridt [m]"},
          {{"openings", "crossing_tol_m"}, "Krydsning: tolerance [m]"},
          {{"openings", "crossing_door_max_sill_m"},
           "Krydset: højeste brystning [m]"},
          {{"openings", "crossing_storey_below_m"}, "Kamera under gulv [m]"},
          {{"openings", "crossing_storey_above_m"}, "Kamera over loft [m]"},
          {{"openings", "leaf_margin_m"}, "Dørblad: margin [m]"},
          {{"openings", "min_leaf_points"}, "Mindste dørbladspunkter"},
          {{"openings", "leaf_bin_deg"}, "Dørblad: vinkelinterval [°]"},
          {{"openings", "leaf_min_radius_share"}, "Dørblad: indre radius"},
          {{"openings", "guid_match_m"}, "Samme åbning inden for [m]"},
          {{"openings", "guid_min_overlap"}, "Samme åbning overlap"},
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
  case pl::JobStage::classify:
    return P::classify;
  case pl::JobStage::walls:
    return P::walls;
  case pl::JobStage::openings:
    return P::openings;
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
