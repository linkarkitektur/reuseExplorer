// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What the command palette lists — the Qt-free table. AppShell maps each
// entry's `id` to the code that runs it; the ranking tests rank this exact
// table, so a change to a title or a keyword is caught by them.
//
// Ids: "page:<n>" (nav order, 0 = Start), "open", "reload", "close",
// "copy-path", "copy-cli", "inspector", "theme", "quit", "pane:<key>"
// (a Rediger pane, see EditPane), "recent:<path>".

#include <rux_qt/fuzzy.hpp>
#include <rux_qt/recent.hpp>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

/// One nav page as the palette and the rail name it.
struct PageEntry {
  std::string name;     ///< "Database"
  std::string keywords; ///< aliases, Danish and English
};

/// The shell's pages in rail order (Start, Oversigt, 3D, Rediger,
/// Pipeline, Log). Alt+1 … Alt+6.
const std::vector<PageEntry> &page_entries();

/// The Rediger workspace's panes (spec §5.2), in their QSettings order.
enum class EditPane { graph = 0, edges, images, overlay };
inline constexpr int kEditPaneCount = 4;
std::string_view edit_pane_key(EditPane pane); ///< "graph" | "edges" | …
std::string edit_pane_name_da(EditPane pane);  ///< "Graf", "Kantliste", …
std::optional<EditPane> parse_edit_pane(std::string_view key);

/// RUX_QT_PAGE value -> rail index, or -1. "database" is an alias of
/// "oversigt" (scripts written before the rename keep working).
int smoke_page_index(std::string_view name);

struct PaletteEntry {
  std::string id;
  std::string group;    ///< "Sider" | "Handlinger" | "Seneste projekter"
  std::string title;    ///< ranked and highlighted
  std::string keywords; ///< ranked as one contiguous run, never shown
  std::string subtitle; ///< shown (a path, a command); NOT searchable
  std::string shortcut; ///< "Ctrl+O"; also the QAction's key sequence
  std::string badge;    ///< "Her" | "Åben" | "Mangler" | ""
  bool enabled = true;
};

struct PaletteState {
  bool project_open = false;
  bool project_failed = false;
  std::string project_path; ///< absolute; empty when none
  int current_page = 0;
  bool inspector_visible = true;
  bool dark_theme = true;
  std::vector<RecentEntry> recent;
  /// The `rux` line of what is on screen (the Pipeline form, a selected log
  /// run); empty = `rux -p <project> info`.
  std::string cli_command;
};

/// Every page, the actions that apply in @p s, and the recent projects.
std::vector<PaletteEntry> build_palette(const PaletteState &s);

/// The ranking input for @p entries: title + keywords. Subtitles are not
/// searchable — full paths would make "home" or "tmp" match every row.
std::vector<PaletteCandidate>
palette_candidates(const std::vector<PaletteEntry> &entries);

/// The title the palette and the start page give a recent project: the
/// directory name for a generic "project.rux", else the file's base name.
std::string recent_project_title(const std::string &path);

/// The shortcut of an action id, as build_palette lists it ("" if none).
std::string action_shortcut(const std::string &id);

} // namespace rux::qt
