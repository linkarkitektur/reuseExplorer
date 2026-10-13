# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# The native Qt client of rux (Stream Q). Three targets, split so the design
# loop rebuilds in seconds and the pure logic is testable without Qt:
#
#   rux_qt_core     token parsing, the gallery's command line, palette
#                   ranking, the recent list, the launch decision. Standard
#                   library only and ALWAYS built (even BUILD_QT_CLIENT=OFF):
#                   rux_lib uses it to decide what plain `rux` does, and the
#                   LIGHT unit-test binary tests it (tests/unit/rux_qt/).
#   rux_qt_lib      the Qt layer (linked into the `rux` executable only,
#                   never rux_lib — see apps/rux/CMakeLists.txt): theme loader (tokens.css -> QSS + QPalette,
#                   hot reload), bundled fonts, shared widgets, the page
#                   registry. Qt6 Widgets/OpenGLWidgets + VTK GUISupportQt +
#                   reusex_visualize (core + the headless VTK renderer); never
#                   the `reusex` umbrella, so no libtorch/TensorRT.
#   rux-qt-gallery  renders any registered page to PNG headless — the
#                   design-studio skill's `qt_shot.sh` drives it.
#
# AUTOMOC/AUTORCC are set per target: they are OFF globally (only
# GUI_ENABLED turns them on for the whole tree).

# ---------------------------------------------------------------------------
add_library(rux_qt_core STATIC
    src/core/tokens.cpp
    src/core/gallery_args.cpp
    src/core/fuzzy.cpp
    src/core/recent.cpp
    src/core/launch.cpp
    src/core/palette_table.cpp
    src/core/background.cpp
    src/core/database_logic.cpp
    src/core/edge_colouring.cpp
    src/core/edge_table.cpp
    src/core/cli_command.cpp
    src/core/workspace_logic.cpp
    src/core/graph_view.cpp)
target_include_directories(rux_qt_core PUBLIC include)
target_compile_features(rux_qt_core PUBLIC cxx_std_20)

# Everything below is the Qt half. Without the client, `rux` still links
# rux_qt_core to decide what plain `rux` does (help instead of the app).
if(NOT BUILD_QT_CLIENT)
    return()
endif()

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    RenderingCore
    RenderingOpenGL2
    InteractionStyle
    GUISupportQt)

# ---------------------------------------------------------------------------
# Headers are listed as sources so AUTOMOC finds the Q_OBJECT classes: it
# only looks next to a .cpp, and the public headers live in include/.
file(GLOB rux_qt_lib_SOURCES CONFIGURE_DEPENDS "src/*.cpp" "include/rux_qt/*.hpp")
add_library(rux_qt_lib STATIC
    ${rux_qt_lib_SOURCES}
    resources/rux_qt.qrc)
set_target_properties(rux_qt_lib PROPERTIES AUTOMOC ON AUTORCC ON)
target_include_directories(rux_qt_lib PUBLIC include)
# Where `--dev` reads the live tokens.css and QSS template from. Release
# binaries embed a snapshot of both through rux_qt.qrc instead. tokens.css is
# vendored here (scripts/sync-tokens.sh) rather than read from
# apps/rux/frontend, which moved to the standalone rux-frontend repo.
target_compile_definitions(rux_qt_lib PRIVATE
    RUX_QT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    RUX_QT_TOKENS_CSS="${CMAKE_CURRENT_SOURCE_DIR}/theme/tokens.css")
target_link_libraries(rux_qt_lib
    PUBLIC
        rux_qt_core
        Qt6::Widgets
        Qt6::OpenGLWidgets
        reusex_core
        # the Pipeline workspace runs stages in-process (pipeline::JobRunner)
        reusex_pipeline
    PRIVATE
        reusex_visualize
        # slam: frame-pair ICP for the pair strip (slam/frame_pair_icp.hpp)
        reusex_slam
        opencv_core
        opencv_imgproc
        ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS rux_qt_lib MODULES ${VTK_LIBRARIES})

# ---------------------------------------------------------------------------
file(GLOB rux_qt_gallery_SOURCES CONFIGURE_DEPENDS "gallery/*.cpp")
add_executable(rux-qt-gallery ${rux_qt_gallery_SOURCES})
set_target_properties(rux-qt-gallery PROPERTIES AUTOMOC ON)
target_link_libraries(rux-qt-gallery PRIVATE rux_qt_lib ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS rux-qt-gallery MODULES ${VTK_LIBRARIES})
