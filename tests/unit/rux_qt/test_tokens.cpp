// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the Qt client's design-token reader (Stream Q, phase Q0).
//
// The native client takes every colour and size from
// apps/rux/qt/theme/tokens.css at run time — a vendored copy of the
// rux-frontend repo's src/tokens.css (scripts/sync-tokens.sh). These
// functions are the Qt-free half of that: parse the CSS custom properties
// for a theme, normalise each value into something QSS accepts, and
// substitute var(--x) in a stylesheet template. They live in `rux_qt_core`
// (no Qt), so they run in the light test binary.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <rux_qt/gallery_args.hpp>
#include <rux_qt/tokens.hpp>

#include <clocale>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#include "../../support/source_dir.hpp"

using namespace rux::qt;
using Catch::Matchers::WithinAbs;

namespace {

constexpr const char *kCss = R"css(
/* header comment with a fake --nope: #fff; inside */
:root {
  color-scheme: light;
  --color-surface: #e9e9e7; /* the workbench */
  --color-text: #1c2530;
  --space-3: 0.75rem;
  --radius-md: 4px;
  --font-sans: 'Archivo', 'Helvetica Neue', Arial, sans-serif;
  --font-mono:
    ui-monospace, 'JetBrains Mono',
    monospace;
  --shadow-sm: 0 1px 2px rgb(29 45 61 / 10%);
  --tracking-caps: 0.08em;
  --alias: var(--color-text);
}

[data-theme='dark'] {
  color-scheme: dark;
  --color-surface: #141c25;
}

@media (prefers-reduced-motion: reduce) {
  :root {
    --space-3: 0px;
  }
}
)css";

} // namespace

TEST_CASE("ParseTokensCss_LightTheme_ReadsRootBlockOnly", "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::light);
  CHECK(t.at("--color-surface") == "#e9e9e7");
  CHECK(t.at("--color-text") == "#1c2530");
  // A comment in the header never becomes a token.
  CHECK(t.count("--nope") == 0);
  // The reduced-motion override is a media query, not a theme: ignored.
  CHECK(t.at("--space-3") == "12px");
}

TEST_CASE("ParseTokensCss_DarkTheme_OverlaysDarkBlockOnRoot",
          "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::dark);
  CHECK(t.at("--color-surface") == "#141c25");
  // Not re-pointed by the dark block: inherited from :root.
  CHECK(t.at("--color-text") == "#1c2530");
}

TEST_CASE("ParseTokensCss_Values_AreNormalisedForQss", "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::light);
  // QSS has no rem: 0.75rem at a 16px root is 12px.
  CHECK(t.at("--space-3") == "12px");
  CHECK(t.at("--radius-md") == "4px");
  // A value split over lines collapses to one line.
  CHECK(t.at("--font-mono") == "ui-monospace, 'JetBrains Mono', monospace");
  // QSS does not parse the CSS4 `rgb(r g b / a%)` form.
  CHECK(t.at("--shadow-sm") == "0 1px 2px rgba(29, 45, 61, 10%)");
  // A token defined in terms of another resolves to the target's value.
  CHECK(t.at("--alias") == "#1c2530");
}

TEST_CASE("NormaliseValue_RemRounding_UsesNearestWholePixel",
          "[rux_qt][tokens]") {
  CHECK(normalise_value("0.84375rem") == "14px"); // 13.5px body text
  CHECK(normalise_value("1.45rem") == "23px");
  CHECK(normalise_value("-0.25rem") == "-4px");
  CHECK(normalise_value("0") == "0");
  CHECK(normalise_value("0.08em") == "0.08em"); // em is relative: untouched
}

TEST_CASE("ResolveVars_KnownTokens_AreSubstituted", "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::dark);
  const auto r = resolve_vars(
      "QWidget { background: var(--color-surface); padding: var(--space-3); }",
      t);
  CHECK(r.text == "QWidget { background: #141c25; padding: 12px; }");
  CHECK(r.missing.empty());
}

TEST_CASE("ResolveVars_UnknownToken_BecomesMagentaAndIsReported",
          "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::light);
  const auto r = resolve_vars("a { color: var(--color-nope); b: var(--nope-2); "
                              "c: var(--color-nope); }",
                              t);
  CHECK(r.text == "a { color: magenta; b: magenta; c: magenta; }");
  // Each missing name is reported once, in first-seen order.
  REQUIRE(r.missing.size() == 2);
  CHECK(r.missing[0] == "--color-nope");
  CHECK(r.missing[1] == "--nope-2");
}

TEST_CASE("ResolveVars_QssComments_AreKeptButNotResolved", "[rux_qt][tokens]") {
  const TokenMap t = parse_tokens_css(kCss, ThemeMode::light);
  // A var() named only in a comment must not count as missing.
  const auto r =
      resolve_vars("/* var(--gone) */ a { color: var(--color-text); }", t);
  CHECK(r.missing.empty());
  CHECK(r.text.find("#1c2530") != std::string::npos);
}

TEST_CASE("ParseLengthPx_AcceptsPxRemAndZero", "[rux_qt][tokens]") {
  CHECK_THAT(*length_px("12px"), WithinAbs(12.0, 1e-9));
  CHECK_THAT(*length_px("0.75rem"), WithinAbs(12.0, 1e-9));
  CHECK_THAT(*length_px("0"), WithinAbs(0.0, 1e-9));
  CHECK_FALSE(length_px("0.08em").has_value());
  CHECK_FALSE(length_px("#fff").has_value());
}

TEST_CASE("ParseLengthEm_AcceptsEmOnly", "[rux_qt][tokens]") {
  CHECK_THAT(*length_em("0.18em"), WithinAbs(0.18, 1e-9));
  CHECK_FALSE(length_em("4px").has_value());
}

TEST_CASE("ParseColor_HexAndRgbaForms", "[rux_qt][tokens]") {
  const auto a = parse_color("#5980a6");
  REQUIRE(a);
  CHECK(a->r == 0x59);
  CHECK(a->g == 0x80);
  CHECK(a->b == 0xa6);
  CHECK(a->a == 255);

  const auto b = parse_color("#fff");
  REQUIRE(b);
  CHECK(b->r == 255);
  CHECK(b->g == 255);

  const auto c = parse_color("rgba(29, 45, 61, 10%)");
  REQUIRE(c);
  CHECK(c->r == 29);
  CHECK(c->a == 26); // 10% of 255, rounded

  const auto d = parse_color("rgb(0 0 0 / 40%)");
  REQUIRE(d);
  CHECK(d->a == 102);

  CHECK_FALSE(parse_color("12px").has_value());
  CHECK_FALSE(parse_color("#12").has_value());
}

TEST_CASE("ParseTokensCss_RealTokensFile_HasEveryRoleTheQtClientUses",
          "[rux_qt][tokens]") {
  // The real file, so a rename on the design side shows up here rather than
  // as a magenta widget in a screenshot nobody looked at.
  std::ifstream in(reusex::test_support::source_dir() /
                   "apps/rux/qt/theme/tokens.css");
  REQUIRE(in);
  std::stringstream ss;
  ss << in.rdbuf();
  for (ThemeMode mode : {ThemeMode::light, ThemeMode::dark}) {
    const TokenMap t = parse_tokens_css(ss.str(), mode);
    for (const char *name :
         {"--color-canvas", "--color-surface", "--color-surface-raised",
          "--color-chrome", "--color-on-chrome", "--color-text",
          "--color-accent", "--label-0", "--label-7", "--label-unlabeled",
          "--font-sans", "--font-display", "--font-mono", "--font-size-md",
          "--space-4", "--radius-lg", "--layout-nav-width",
          "--layout-panel-width", "--tracking-caps"}) {
      INFO(name);
      CHECK(t.count(name) == 1);
    }
    // Every colour role parses as a colour.
    CHECK(parse_color(t.at("--color-canvas")).has_value());
  }
}

TEST_CASE("ParseGalleryArgs_FullFlagSet", "[rux_qt][gallery]") {
  const char *argv[] = {
      "rux-qt-gallery", "--page",       "components", "--theme", "dark",
      "--size",         "1440x900",     "--scale",    "2",       "--project",
      "/tmp/x.rux",     "--screenshot", "out.png",    "--dev",   "--gl"};
  const GalleryArgs a = parse_gallery_args(15, argv);
  CHECK(a.error.empty());
  CHECK(a.page == "components");
  CHECK(a.theme == ThemeMode::dark);
  CHECK(a.width == 1440);
  CHECK(a.height == 900);
  CHECK(a.scale == 2.0);
  CHECK(a.project == "/tmp/x.rux");
  CHECK(a.screenshot == "out.png");
  CHECK(a.dev);
  CHECK(a.gl);
}

TEST_CASE("ParseGalleryArgs_Defaults", "[rux_qt][gallery]") {
  const char *argv[] = {"rux-qt-gallery"};
  const GalleryArgs a = parse_gallery_args(1, argv);
  CHECK(a.error.empty());
  CHECK(a.page == "components");
  CHECK(a.theme == ThemeMode::dark);
  CHECK(a.width == 1440);
  CHECK(a.height == 900);
  CHECK(a.scale == 1.0);
  CHECK_FALSE(a.dev);
}

TEST_CASE("ParseGalleryArgs_BadValues_ReportAnError", "[rux_qt][gallery]") {
  {
    const char *argv[] = {"g", "--theme", "sepia"};
    CHECK_FALSE(parse_gallery_args(3, argv).error.empty());
  }
  {
    const char *argv[] = {"g", "--size", "1440"};
    CHECK_FALSE(parse_gallery_args(3, argv).error.empty());
  }
  {
    const char *argv[] = {"g", "--page"};
    CHECK_FALSE(parse_gallery_args(2, argv).error.empty());
  }
  {
    const char *argv[] = {"g", "--bogus"};
    CHECK_FALSE(parse_gallery_args(2, argv).error.empty());
  }
  {
    const char *argv[] = {"g", "--scale", "0"};
    CHECK_FALSE(parse_gallery_args(3, argv).error.empty());
  }
}

TEST_CASE("NormaliseValue_CommaDecimalLocale_StillParsesDots",
          "[rux_qt][tokens]") {
  // A QApplication calls setlocale(LC_ALL, ""); under da_DK the C library
  // reads "0.75" as 0. Found as every font size resolving to 0 in the first
  // gallery screenshot.
  const char *old = std::setlocale(LC_NUMERIC, nullptr);
  const std::string saved = old ? old : "C";
  if (!std::setlocale(LC_NUMERIC, "da_DK.UTF-8") &&
      !std::setlocale(LC_NUMERIC, "de_DE.UTF-8"))
    SKIP("no comma-decimal locale installed");
  CHECK(normalise_value("0.84375rem") == "14px");
  CHECK(parse_color("rgba(29, 45, 61, 0.5)")->a == 128);
  std::setlocale(LC_NUMERIC, saved.c_str());
}

namespace {

std::string slurp(const std::filesystem::path &p) {
  std::ifstream in(p);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

const std::filesystem::path kSource = reusex::test_support::source_dir();

} // namespace

TEST_CASE("AppQss_EveryVarReference_ResolvesAgainstRealTokens",
          "[rux_qt][tokens]") {
  // CI does not run qt_shot.sh or token_lint, so this is the guard against a
  // design-side rename leaving app.qss with a token that renders magenta.
  const std::string css = slurp(kSource / "apps/rux/qt/theme/tokens.css");
  const std::string qss = slurp(kSource / "apps/rux/qt/styles/app.qss");
  REQUIRE_FALSE(css.empty());
  REQUIRE_FALSE(qss.empty());
  for (ThemeMode mode : {ThemeMode::light, ThemeMode::dark}) {
    TokenMap t = parse_tokens_css(css, mode);
    // --qt-* tokens are generated by rux::qt::Theme at run time.
    t["--qt-icon-check"] = "/generated/check.png";
    t["--qt-icon-chevron"] = "/generated/chevron.png";
    const auto r = resolve_vars(qss, t);
    for (const auto &m : r.missing)
      FAIL_CHECK("app.qss names unknown token " << m);
    // Comments are copied through (they mention var(--token) in prose).
    const std::string code = std::regex_replace(
        r.text, std::regex(R"(/\*[\s\S]*?\*/)"), std::string());
    CHECK(code.find("var(") == std::string::npos);
  }
}

TEST_CASE("QtSources_EveryTokenStringLiteral_ExistsInRealTokens",
          "[rux_qt][tokens]") {
  // theme().color("--x") and friends: a C++ lookup of an unknown token is
  // magenta at run time just like the QSS. Only strings in a token family
  // (--color-, --space-, …) are checked: "--page" is a CLI flag.
  const TokenMap t = parse_tokens_css(
      slurp(kSource / "apps/rux/qt/theme/tokens.css"), ThemeMode::dark);
  std::set<std::string> families;
  for (const auto &[name, value] : t)
    families.insert(name.substr(0, name.find('-', 2)));
  const std::regex lit(R"re("(--[a-z0-9][a-z0-9-]*)")re");
  int checked = 0;
  for (const auto &entry :
       std::filesystem::recursive_directory_iterator(kSource / "apps/rux/qt")) {
    const auto ext = entry.path().extension();
    if (ext != ".cpp" && ext != ".hpp")
      continue;
    // A line marked `token-lint: allow` (the rux flag table's "--radius",
    // which is `create planes -r`) is not a token lookup — same rule as
    // token_lint.py. Every other line of every file is scanned.
    std::string src;
    {
      std::istringstream lines(slurp(entry.path()));
      for (std::string line; std::getline(lines, line);)
        if (line.find("token-lint: allow") == std::string::npos)
          src += line + "\n";
    }
    for (auto m = std::sregex_iterator(src.begin(), src.end(), lit);
         m != std::sregex_iterator(); ++m) {
      const std::string name = (*m)[1].str();
      if (name.rfind("--qt-", 0) == 0 ||
          !families.count(name.substr(0, name.find('-', 2))))
        continue;
      ++checked;
      if (!t.count(name))
        FAIL_CHECK(entry.path().filename().string()
                   << " names unknown token " << name);
    }
  }
  CHECK(checked > 20); // the scan actually saw the code's lookups
}
