// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/cli_command.hpp>
#include <rux_qt/palette_table.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>

namespace rux::qt {
namespace {

struct ActionDef {
  const char *id;
  const char *shortcut;
};

constexpr ActionDef kShortcuts[] = {
    {"open", "Ctrl+O"},      {"reload", "F5"},          {"close", "Ctrl+W"},
    {"inspector", "Ctrl+I"}, {"theme", "Ctrl+Shift+L"}, {"quit", "Ctrl+Q"},
};

std::string lower_ascii(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

} // namespace

const std::vector<PageEntry> &page_entries() {
  static const std::vector<PageEntry> pages = {
      {"Start", "forside velkomst start"},
      {"Oversigt", "database db tabeller billeder frames databaseviewer data "
                   "oversigt panoramaer"},
      {"3D", "punktsky viewer vtk mesh scene"},
      {"Rediger", "posegraf pose graph kanter loop closure graf rediger edit "
                  "icp løkkelukning"},
      {"Pipeline", "kør trin stage job parametre"},
      {"Log", "kørselslog pipeline-log historik"},
  };
  return pages;
}

std::string_view edit_pane_key(EditPane p) {
  switch (p) {
  case EditPane::graph:
    return "graph";
  case EditPane::edges:
    return "edges";
  case EditPane::images:
    return "images";
  case EditPane::overlay:
    return "overlay";
  }
  return "graph";
}

std::string edit_pane_name_da(EditPane p) {
  switch (p) {
  case EditPane::graph:
    return "Graf";
  case EditPane::edges:
    return "Kantliste";
  case EditPane::images:
    return "Billeder";
  case EditPane::overlay:
    return "3D-overlay";
  }
  return {};
}

std::optional<EditPane> parse_edit_pane(std::string_view key) {
  for (int i = 0; i < kEditPaneCount; ++i)
    if (edit_pane_key(static_cast<EditPane>(i)) == key)
      return static_cast<EditPane>(i);
  return std::nullopt;
}

int smoke_page_index(std::string_view name) {
  static constexpr std::string_view kNames[] = {"start",   "oversigt", "3d",
                                                "rediger", "pipeline", "log"};
  if (name == "database")
    return 1;
  for (int i = 0; i < 6; ++i)
    if (kNames[i] == name)
      return i;
  return -1;
}

std::string action_shortcut(const std::string &id) {
  for (const auto &a : kShortcuts)
    if (id == a.id)
      return a.shortcut;
  return {};
}

std::string recent_project_title(const std::string &path) {
  const std::filesystem::path p(path);
  const std::string stem = p.stem().string();
  const std::string l = lower_ascii(stem);
  if ((l == "project" || l == "projekt") && p.has_parent_path() &&
      !p.parent_path().filename().empty())
    return p.parent_path().filename().string();
  return stem;
}

std::vector<PaletteEntry> build_palette(const PaletteState &s) {
  std::vector<PaletteEntry> out;
  const auto &pages = page_entries();
  for (std::size_t i = 0; i < pages.size(); ++i) {
    PaletteEntry e;
    e.id = "page:" + std::to_string(i);
    e.group = "Sider";
    e.title = pages[i].name;
    e.keywords = "side " + pages[i].keywords;
    e.shortcut = "Alt+" + std::to_string(i + 1);
    e.badge = static_cast<int>(i) == s.current_page ? "Her" : "";
    out.push_back(std::move(e));
  }
  auto action = [&](const char *id, std::string title, const char *kw,
                    std::string subtitle = {}) {
    PaletteEntry e;
    e.id = id;
    e.group = "Handlinger";
    e.title = std::move(title);
    e.keywords = kw;
    e.subtitle = std::move(subtitle);
    e.shortcut = action_shortcut(id);
    out.push_back(std::move(e));
  };
  action("open", "Åbn projekt…", "open fil rux");
  if (s.project_open || s.project_failed)
    action("reload", "Genindlæs projekt", "reload igen");
  if (s.project_open) {
    action("close", "Luk projekt", "close");
    action("copy-path", "Kopiér projektets sti", "copy path udklipsholder",
           s.project_path);
    action("copy-cli", "Kopiér som rux-kommando", "cli terminal info",
           !s.cli_command.empty()
               ? s.cli_command
               : "rux -p " + shell_quote(s.project_path) + " info");
  }
  action("inspector", s.inspector_visible ? "Skjul inspektør" : "Vis inspektør",
         "panel højre inspector");
  action("theme", s.dark_theme ? "Skift til lyst tema" : "Skift til mørkt tema",
         "theme lys mørk dark light");
  action("quit", "Afslut", "quit exit");
  if (s.project_open) {
    // Titles in the definite form ("grafen"), one per EditPane.
    static constexpr const char *kPaneTitles[kEditPaneCount] = {
        "grafen", "kantlisten", "billederne", "3D-overlay"};
    for (int i = 0; i < kEditPaneCount; ++i) {
      const auto p = static_cast<EditPane>(i);
      PaletteEntry e;
      e.id = "pane:" + std::string(edit_pane_key(p));
      e.group = "Handlinger";
      e.title = std::string("Vis eller skjul ") + kPaneTitles[i];
      e.keywords = "rediger panel vis skjul " + edit_pane_name_da(p);
      out.push_back(std::move(e));
    }
  }

  for (const auto &r : s.recent) {
    PaletteEntry e;
    e.id = "recent:" + r.path;
    e.group = "Seneste projekter";
    e.title = recent_project_title(r.path);
    const std::filesystem::path p(r.path);
    // The folder and file name, not the whole path (see palette_candidates).
    e.keywords = "seneste recent " + p.parent_path().filename().string() + " " +
                 p.filename().string();
    e.subtitle = r.path;
    e.enabled = !r.missing;
    e.badge = r.missing
                  ? "Mangler"
                  : (s.project_open && r.path == s.project_path ? "Åben" : "");
    out.push_back(std::move(e));
  }
  return out;
}

std::vector<PaletteCandidate>
palette_candidates(const std::vector<PaletteEntry> &entries) {
  std::vector<PaletteCandidate> c;
  c.reserve(entries.size());
  for (const auto &e : entries)
    c.push_back({e.title, e.keywords});
  return c;
}

} // namespace rux::qt
