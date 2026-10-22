// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The 3D workspace (Stream Q, Q3): RTABMap's 3D view, calmer. A layer panel
// on the left (points coloured by stored RGB or by any label cloud, with a
// --label-* legend; the building elements -- walls, slabs, roof, spaces,
// doors, windows -- each category and storey shown or hidden on its own
// ("Bygningsdele", rux_qt/element_filter.hpp), drawn as typed solids or as
// wall boxes, opening frames and space fills; mesh; camera frustums;
// panoramas; a cut plane), the canvas in the centre with view presets over it,
// and a picked point described to the inspector.
//
// The scene comes from visualize::populate_scene() — the builder `rux render`
// uses — one layer at a time, built off the GUI thread with its own
// read-only ProjectDB and handed over when done. Nothing loads until the
// page is first shown (activate()): a 1.2-million-point cloud is not read
// just because a project was opened.

#include <rux_qt/element_filter.hpp>
#include <rux_qt/selection.hpp>

#include <reusex/visualize/render_view.hpp>

#include <QMap>
#include <QString>
#include <QWidget>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QVBoxLayout;

namespace rux::qt {

class ProjectSession;
class SceneView;
class ViewSettings;

/// The swatch token of point label @p label on a scale of @p nslots:
/// `--label-N` by the web's slot rule, `--label-unlabeled` for 0 and
/// `--label-invalid` for an out-of-contract (wrapped -1) value.
QString label_swatch_token(std::uint32_t label, int nslots);

class Viewer3DWorkspace : public QWidget {
  Q_OBJECT
    public:
  Viewer3DWorkspace(ProjectSession &session, ViewSettings &view_settings,
                    bool interactive, QWidget *parent = nullptr);
  ~Viewer3DWorkspace() override;

  /// The page is on screen: load what is not loaded yet.
  void activate();
  const Selection &selection() const { return selection_; }

  /// Colour the points by stored RGB ("cloud") or a label cloud
  /// ("planes", "rooms", "instances", "labels").
  void set_colour_by(const QString &layer);
  void set_layer_visible(reusex::visualize::Layer layer, bool on);
  void set_view(reusex::visualize::ViewPreset view);
  void set_cut(bool on, std::optional<double> height = std::nullopt);
  /// Show or hide one building-element category ("wall", "door", ...:
  /// visualize::kElementCategories), every category, or exactly @p only.
  void set_element_shown(const std::string &category, bool on);
  void set_all_elements_shown(bool on);
  void show_only_elements(const std::vector<std::string> &only);
  /// Show the building elements of one storey (StoreyRecord::idx), or all.
  void set_storey_filter(std::optional<int> storey);
  const ElementFilter &element_filter() const { return elements_; }
  /// Pick the point under @p pos of the canvas (as a click there does).
  bool pick_at(const QPoint &pos);
  SceneView *view() const { return view_; }

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    private:
  struct LayerState;
  struct BuildResult;
  void reset();
  void rebuild_panel();
  void ensure_layer(reusex::visualize::Layer layer);
  void layer_built(std::shared_ptr<BuildResult> result);
  void apply_visibility();
  void apply_cut();
  void frame_camera();
  void update_legend();
  void update_status();
  void pick(const QPoint &pos);
  bool has_data(reusex::visualize::Layer layer) const;
  /// Whether the actors of @p layer draw at all (its box, the cloud's colour
  /// choice, the opening-frames rule), before the element filter.
  bool layer_shown(int layer) const;
  void add_elements_group();
  void sync_element_boxes();
  void elements_changed();
  QString colour_cloud() const;

  ProjectSession &session_;
  /// The project's view yaw, applied by every preset camera.
  ViewSettings &view_settings_;
  bool interactive_ = false;
  bool active_ = false;
  unsigned generation_ = 0;
  SceneView *view_ = nullptr;

  QWidget *panel_ = nullptr;
  QVBoxLayout *panel_layout_ = nullptr;
  QComboBox *colour_ = nullptr;
  QCheckBox *cut_on_ = nullptr;
  QDoubleSpinBox *cut_height_ = nullptr;
  QWidget *legend_ = nullptr;
  QMap<int, QCheckBox *> layer_boxes_;
  ElementFilter elements_;
  QCheckBox *elements_all_ = nullptr;
  std::map<std::string, QCheckBox *> element_boxes_;
  std::map<std::string, QLabel *> element_counts_;
  QComboBox *storey_ = nullptr;
  QWidget *presets_ = nullptr;
  QLabel *status_ = nullptr;
  QLabel *mode_ = nullptr;

  std::map<int, std::unique_ptr<LayerState>> layers_;
  QMap<int, bool> visible_;
  reusex::visualize::ViewPreset preset_ = reusex::visualize::ViewPreset::orbit;
  bool camera_placed_ = false;
  /// The user moved the camera: a resize no longer re-frames it.
  bool camera_moved_ = false;
  bool cut_ = true;
  std::optional<double> cut_height_m_;
  double cut_z_ = 0.0;
  bool cut_from_planes_ = false;
  Selection selection_;
  void *marker_ = nullptr; // vtkActor of the picked point
};

} // namespace rux::qt
