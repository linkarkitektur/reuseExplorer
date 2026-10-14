// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What the inspector shows for the current selection in a workspace — a
// frame, a pose-graph edge, a table row, a cloud, a log entry. Workspaces
// describe the selection as plain rows; the Inspector lays them out, so every
// selection looks the same whichever workspace it came from.

#include <QImage>
#include <QString>
#include <QVector>
#include <utility>

namespace rux::qt {

struct SelectionRow {
  enum class Style {
    value, ///< prose value (Archivo)
    mono,  ///< a figure, id or code (JetBrains Mono)
    name,  ///< the key is a raw name (a column, a cloud): mono, not caps
  };
  SelectionRow() = default;
  /// Brace-initialisable as {key, value[, style[, swatch_token]]} without
  /// -Wmissing-field-initializers for the members left out.
  SelectionRow(QString key_, QString value_, Style style_ = Style::mono,
               QString swatch_token_ = {})
      : key(std::move(key_)), value(std::move(value_)), style(style_),
        swatch_token(std::move(swatch_token_)) {}

  QString key;
  QString value;
  Style style = Style::mono;
  /// A colour token for a swatch before the key (a legend row), or empty.
  QString swatch_token;
};

struct SelectionSection {
  QString title; ///< caps section label; empty = no label
  QVector<SelectionRow> rows;
  /// Free text under the rows (a JSON blob, an error), shown in mono.
  QString block;
  /// @ref block is a shell command: wrap it between words with `\`
  /// continuations (CommandBlock), never inside a flag.
  bool command = false;
};

struct Selection {
  QString kind;     ///< caps eyebrow over the title: "Billede A", "Kant", …
  QString title;    ///< headline, e.g. "Billede 1234"
  QString subtitle; ///< one muted line under it
  QString pill;     ///< optional status pill text
  QString pill_tone = "outline";
  QImage preview; ///< optional thumbnail shown under the title
  QVector<SelectionSection> sections;

  bool empty() const { return title.isEmpty() && sections.isEmpty(); }
};

} // namespace rux::qt
