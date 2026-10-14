// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Rediger's pair images (spec §5.2): A | B side by side, colour or depth,
// decoded off the GUI thread by its own FrameImageLoader (Oversigt keeps
// its own cache; the two pages never wait on each other). A side whose
// slot is empty says which click fills it; a decode that lands after A or
// B moved on is dropped.

#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/graph_view.hpp>

#include <QWidget>

class QButtonGroup;
class QLabel;

namespace rux::qt {

class FrameImageLoader;
class ProjectSession;

class PairImages : public QWidget {
  Q_OBJECT
    public:
  explicit PairImages(ProjectSession &session, QWidget *parent = nullptr);
  /// A and B (-1 = empty) and the slot the next plain click fills, which
  /// an empty side's hint names.
  void set_pair(int a, int b, Slot next);
  void set_layer(FrameLayer layer);
  FrameLayer layer() const { return layer_; }

    private:
  void reload();
  void sync();
  void frame_ready(int id);
  ProjectSession &session_;
  FrameImageLoader *loader_ = nullptr;
  ImagePane *pane_a_ = nullptr;
  ImagePane *pane_b_ = nullptr;
  QLabel *caption_a_ = nullptr;
  QLabel *caption_b_ = nullptr;
  QButtonGroup *layers_ = nullptr;
  FrameLayer layer_ = FrameLayer::color;
  int a_ = -1, b_ = -1;
  Slot next_ = Slot::a;
};

} // namespace rux::qt
