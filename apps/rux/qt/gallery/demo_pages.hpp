// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The gallery's demo pages (Stream Q, phase Q0). They prove the design loop
// — tokens.css -> QSS -> screenshot — before any real workspace exists, and
// are the reference for how a page composes the shared widgets. The real
// app shell (Q1) replaces demo_frame().

#include <rux_qt/pages.hpp>

class QWidget;

namespace rux::qt::gallery {

/// Title bar + nav rail + content + optional inspector, as the app will
/// frame every workspace. @p active is the nav row to highlight.
QWidget *demo_frame(const PageContext &ctx, int active, QWidget *content,
                    QWidget *inspector);

QWidget *make_components_page(const PageContext &ctx);
QWidget *make_viewport_page(const PageContext &ctx);
/// The real shell on its 3D page (shell_pages.cpp).
QWidget *make_shell_3d(const PageContext &ctx);

/// Register every demo page with the page registry.
void register_demo_pages();
/// The real app shell's pages (shell_pages.cpp).
void register_shell_pages();

/// A page's own check failed (logged as "ERROR …" by the page): the
/// gallery still writes the PNG, then exits 6 like a layout guard.
void report_page_failure();
bool page_failed();

} // namespace rux::qt::gallery
