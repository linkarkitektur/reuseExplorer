// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph edits banner (the web's WriteBanner): how many edits wait,
// what kind, why a save failed or is held back, and Gem / Kassér. Lives at
// the top of Rediger; it watches the EdgeEditor and needs nothing else.

#include <QFrame>

class QLabel;
class QPushButton;

namespace rux::qt {

class EdgeEditor;

class PendingBanner : public QFrame {
  Q_OBJECT
    public:
  explicit PendingBanner(EdgeEditor &editor, QWidget *parent = nullptr);
  void sync();

    private:
  EdgeEditor &editor_;
  QLabel *text_ = nullptr;
  QLabel *error_ = nullptr;
  QPushButton *save_ = nullptr;
  QPushButton *discard_ = nullptr;
};

} // namespace rux::qt
