// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A flat equirectangular panorama in Oversigt (spec §5.3): decoded off the
// GUI thread (own read-only ProjectDB), scaled to at most kPanoramaMaxSide,
// shown in a pan/zoom view — wheel zooms about the cursor (clamped), a drag
// pans, F fits. No spherical look-around.

#include <QGraphicsView>
#include <QImage>
#include <QSize>
#include <QString>
#include <QWidget>

class QGraphicsPixmapItem;
class QLabel;
class QStackedLayout;

namespace rux::qt {

/// The pan/zoom canvas: fits the image until the user zooms.
class PanoramaView : public QGraphicsView {
  Q_OBJECT
    public:
  explicit PanoramaView(QWidget *parent = nullptr);
  /// Fit the whole image (and follow resizes again).
  void fit();

    protected:
  void wheelEvent(QWheelEvent *e) override;
  void keyPressEvent(QKeyEvent *e) override;
  void resizeEvent(QResizeEvent *e) override;

    private:
  bool user_view_ = false;
};

class PanoramaViewer : public QWidget {
  Q_OBJECT
    public:
  explicit PanoramaViewer(QWidget *parent = nullptr);
  /// Decode panorama @p id of the project at @p project off the GUI thread
  /// and show it; a newer call (or clear()) drops an older result.
  void show_panorama(const QString &project, int id, const QString &caption);
  /// Show nothing; drops any decode still running.
  void clear();
  /// A decode is running whose result will be shown.
  bool loading() const { return loading_; }

    private:
  /// @p stored is the panorama's size in the project (before scaling).
  void loaded(unsigned generation, const QImage &image, QSize stored,
              const QString &error);
  void show_message(const QString &text);

  PanoramaView *view_ = nullptr;
  QGraphicsPixmapItem *item_ = nullptr;
  QStackedLayout *stack_ = nullptr;
  QLabel *caption_ = nullptr;
  QLabel *size_ = nullptr;
  QLabel *message_ = nullptr;
  unsigned generation_ = 0;
  bool loading_ = false;
};

} // namespace rux::qt
