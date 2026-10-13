// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The frame browser of the Database workspace — RTABMap's DatabaseViewer A/B
// compare, made calm: two sides (A, B) each with a slider, the image in one
// of three layers (colour, depth, confidence) with the label overlay and its
// legend, and the frame's pose, intrinsics and time; and a filmstrip
// (click = A, Shift+click = B).
//
// Keys: ←/→ move A, Shift+←/→ move B (Ctrl: 10 frames), while the focus is
// in the browser and not in a text field (rux_qt/database_logic.hpp).

#include <rux_qt/database_logic.hpp>
#include <rux_qt/selection.hpp>

#include <QAbstractListModel>
#include <QFrame>
#include <QHash>
#include <QSet>
#include <QStyledItemDelegate>
#include <QVector>
#include <QWidget>

#include <array>
#include <map>
#include <memory>
#include <utility>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QLabel;
class QListView;
class QPushButton;
class QSlider;
class QSpinBox;
class QVBoxLayout;

namespace rux::qt {

struct DecodedFrame;
class EdgeEditor;
class FrameImageLoader;
class Pill;
class ProjectSession;
class PropertyList;

/// Which plane an image pane shows under the label overlay.
enum class FrameLayer { color = 0, depth, confidence };

/// One image: the chosen plane fitted on the near-black canvas, the label
/// overlay on top, and a caption chip (layer, range).
class ImagePane : public QWidget {
  Q_OBJECT
    public:
  explicit ImagePane(QWidget *parent = nullptr);
  void set_frame(std::shared_ptr<const DecodedFrame> frame);
  void set_layer(FrameLayer layer);
  void set_overlay(bool on, int opacity_percent);
  /// Show "Henter …" over the last image until the next set_frame().
  void set_loading(bool loading);
  /// A text instead of an image (no frames, an error).
  void set_message(const QString &text);
  QSize sizeHint() const override;

    protected:
  void paintEvent(QPaintEvent *) override;

    private:
  void rebuild_tables();
  std::shared_ptr<const DecodedFrame> frame_;
  FrameLayer layer_ = FrameLayer::color;
  bool overlay_ = true;
  int opacity_ = 35;
  bool loading_ = false;
  QString message_;
  QVector<QRgb> depth_table_, conf_levels_table_, label_table_;
};

/// One side of the compare: A or B.
class FrameSide : public QFrame {
  Q_OBJECT
    public:
  FrameSide(const QString &letter, QWidget *parent = nullptr);
  /// The frames the slider runs over (sorted ids).
  void set_ids(const std::vector<int> &ids);
  /// Show frame @p index (into the ids); -1 = none.
  void set_index(int index);
  void set_frame(std::shared_ptr<const DecodedFrame> frame);
  void set_loading(bool loading);
  void set_layer(FrameLayer layer);
  void set_overlay(bool on, int opacity_percent);
  /// Pose, intrinsics and time of the shown frame (read on the GUI thread).
  void set_metadata(const QVector<std::pair<QString, QString>> &rows);
  void set_message(const QString &text);
  void set_label_names(const std::map<int, std::string> *names) {
    names_ = names;
  }

    signals:
  void index_requested(int index);
  void activated(); ///< clicked: becomes the inspector's selection

    protected:
  void mousePressEvent(QMouseEvent *e) override;

    private:
  void refresh_legend();
  std::vector<int> ids_;
  int index_ = -1;
  bool overlay_ = true;
  std::shared_ptr<const DecodedFrame> frame_;
  const std::map<int, std::string> *names_ = nullptr;
  QSpinBox *id_ = nullptr;
  QLabel *position_ = nullptr;
  QSlider *slider_ = nullptr;
  QPushButton *prev_ = nullptr;
  QPushButton *next_ = nullptr;
  ImagePane *image_ = nullptr;
  QLabel *legend_ = nullptr;
  PropertyList *meta_ = nullptr;
  QVBoxLayout *meta_box_ = nullptr;
};

/// The filmstrip: one thumbnail per frame, virtualised by QListView.
class FilmstripModel : public QAbstractListModel {
  Q_OBJECT
    public:
  enum Role {
    IdRole = Qt::UserRole + 1,
    ThumbRole,
    MarkRole,      ///< "A", "B", "AB" or empty
    SegmentedRole, ///< has a segmentation image
    DegreeRole,    ///< pose-graph edges (pending included)
  };
  FilmstripModel(FrameImageLoader &loader, QObject *parent = nullptr);
  void set_frames(std::vector<int> ids, QSet<int> segmented);
  void set_marks(int a_id, int b_id);
  void set_degrees(QHash<int, int> degrees);
  int rowCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;

    private:
  void thumbnail_ready(int id);
  FrameImageLoader &loader_;
  std::vector<int> ids_;
  QSet<int> segmented_;
  QHash<int, int> degrees_;
  int a_ = -1, b_ = -1;
};

class FilmstripDelegate : public QStyledItemDelegate {
  Q_OBJECT
    public:
  using QStyledItemDelegate::QStyledItemDelegate;
  QSize sizeHint(const QStyleOptionViewItem &,
                 const QModelIndex &) const override;
  void paint(QPainter *p, const QStyleOptionViewItem &option,
             const QModelIndex &index) const override;
  /// Thumbnail height in logical pixels.
  static int thumb_height();
};

class FrameBrowser : public QWidget {
  Q_OBJECT
    public:
  FrameBrowser(ProjectSession &session, EdgeEditor &editor,
               FrameImageLoader &loader, QWidget *parent = nullptr);
  ~FrameBrowser() override;

  /// Re-read the frames of the open project (or clear).
  void reload();
  const FramePair &pair() const { return pair_; }
  /// Jump A to the frame nearest @p id (a scan's first frame).
  void show_a_near(int id);
  void set_a(int id);
  void set_b(int id);
  /// The inspector content of frame A (or B).
  Selection frame_selection(bool side_b) const;

    signals:
  /// A or B moved.
  void pair_changed(int a_id, int b_id);
  void selection_changed(const rux::qt::Selection &selection);

    protected:
  bool eventFilter(QObject *watched, QEvent *event) override;
  void showEvent(QShowEvent *e) override;

    private:
  void sync_sides();
  void sync_side(bool side_b);
  void frame_ready(int id);
  void apply_view();
  void refresh_strip_marks();
  QVector<std::pair<QString, QString>> metadata(int id) const;

  ProjectSession &session_;
  EdgeEditor &editor_;
  FrameImageLoader &loader_;
  FramePair pair_;
  std::map<int, std::string> label_names_;
  QSet<int> segmented_;
  FrameLayer layer_ = FrameLayer::color;
  bool overlay_ = true;
  int opacity_ = 35;
  bool selection_is_b_ = false;

  QLabel *subtitle_ = nullptr;
  QButtonGroup *layers_ = nullptr;
  QCheckBox *overlay_box_ = nullptr;
  QSlider *opacity_slider_ = nullptr;
  FrameSide *side_a_ = nullptr;
  FrameSide *side_b_ = nullptr;
  QListView *film_ = nullptr;
  FilmstripModel *film_model_ = nullptr;
  QLabel *empty_ = nullptr;
  QWidget *body_ = nullptr;
};

} // namespace rux::qt
