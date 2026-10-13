# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Library Target Configuration
# ===============================================
#
# The library is split into one static library per pipeline-stage module, with
# explicit `target_link_libraries` encoding the layer diagram in
# docs/STANDARDS.md §1. Illegal dependencies surface as link errors instead of
# being silently allowed (#222).
#
# Layer diagram (a module may only depend on lower layers):
#
#   Layer 4:  visualize  pipeline
#   Layer 3:  segmentation  reconstruction  slam  io  vision            (peers)
#             gsplat_common  gsplat
#   Layer 2:  core
#   Layer 1:  utils, types.hpp
#
# `gsplat` is two targets, not one: `reusex_gsplat_common` is the torch-free CPU
# half and is always built, while `reusex_gsplat` adds the CUDA trainer and only
# exists under WITH_CUDA + the vendored rasterizer. See the gsplat source lists
# below for why (#332).
#
# `reusex_geometry_common` holds the CGAL/PCL geometry helpers (`geometry/utils`,
# `cgal_utils`, `CoplanarPolygon`, `BuildingComponent`) shared by several
# Layer-3 peers; it sits just above utils so those consumers can share it
# without linking each other.
#
# The `reusex` target remains a backward-compatible INTERFACE umbrella that
# links every module, so apps/rux, tests and bindings build unchanged —
# `ruxd`, in its own repo, links the exported `ReUseX::reusex` target
# instead.

# -----------------------------------------------
# Per-module source discovery
# -----------------------------------------------
# Phase B (#222) relocated segmentation/reconstruction/slam into their own
# src/<module>/ dirs. The one-line include/geometry/ forwarding shims left
# behind for consumers were removed in #248 once every call site was migrated
# to the new <reusex/{segmentation,reconstruction,slam}/...> paths. The shared
# CGAL/PCL helpers (reusex_geometry_common) still live under src/geometry/ and
# are listed explicitly below.
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# CMAKE_INSTALL_INCLUDEDIR is referenced by the $<INSTALL_INTERFACE> include
# paths below; cmake/Installation.cmake installs the tree at the same place.
include(GNUInstallDirs)

# Layer 1 — utils (no internal dependencies)
file(GLOB_RECURSE REUSEX_UTILS_SOURCES CONFIGURE_DEPENDS "${SRC}/utils/*.cpp")

# Layer 2 — core (ProjectDB, logging, materials, stages, validate)
file(GLOB_RECURSE REUSEX_CORE_SOURCES CONFIGURE_DEPENDS "${SRC}/core/*.cpp")

# Layer 1.5 — shared geometry helpers (CGAL/PCL primitives). Kept separate so
# the Layer-3 peers can depend on it without depending on each other.
set(REUSEX_GEOMETRY_COMMON_SOURCES
    ${SRC}/geometry/utils.cpp
    ${SRC}/geometry/cgal_utils.cpp
    ${SRC}/geometry/CoplanarPolygon.cpp
    ${SRC}/geometry/BuildingComponent.cpp
    # Equirectangular<->perspective reprojection: a low-level OpenCV/Eigen
    # primitive shared by slam (panorama alignment) and vision (SAM3-on-360),
    # so it lives in the common geometry layer both reach via core.
    ${SRC}/geometry/EquirectProjection.cpp)

# Layer 3 — io (format conversion only)
file(GLOB_RECURSE REUSEX_IO_SOURCES CONFIGURE_DEPENDS "${SRC}/io/*.cpp")

# Layer 3 — vision (ML models, backends, datasets) — includes CUDA kernels
file(GLOB_RECURSE REUSEX_VISION_SOURCES CONFIGURE_DEPENDS "${SRC}/vision/*.cpp")
file(GLOB_RECURSE REUSEX_VISION_CUDA_SOURCES CONFIGURE_DEPENDS "${SRC}/vision/*.cu")
list(APPEND REUSEX_VISION_SOURCES ${REUSEX_VISION_CUDA_SOURCES})

# Layer 3 — segmentation (planes, rooms, instances, cloud creation/refinement)
# Relocated to src/segmentation/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_SEGMENTATION_SOURCES CONFIGURE_DEPENDS
     "${SRC}/segmentation/*.cpp")

# Layer 3 — reconstruction (cell complex, solidifier, mesh, texture, windows)
# Relocated to src/reconstruction/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_RECONSTRUCTION_SOURCES CONFIGURE_DEPENDS
     "${SRC}/reconstruction/*.cpp")

# Layer 3 — slam (registration / pose-graph optimization)
# Relocated to src/slam/ in #222 (Phase B).
file(GLOB_RECURSE REUSEX_SLAM_SOURCES CONFIGURE_DEPENDS
     "${SRC}/slam/*.cpp")

# Layer 3 — gsplat (3D Gaussian Splatting, #240), split into a CPU half and a
# CUDA half (#332).
#
# WHY THE SPLIT EXISTS: CI builds the CPU-only nix variant
# (`nix build .#checks.x86_64-linux.tests`, WITH_CUDA=OFF). While the whole
# module was one CUDA-gated target, the module — and with it every one of its
# tests — simply vanished from the only build a pull request actually gets to
# prove anything with. Yet roughly half of gsplat never touches torch or the
# rasterizer: the `GaussianCloud` value type (seeding, SH round trip, `.ply`
# I/O, `validate()`), the `ProjectDB` -> `TrainingView` loader (frame
# selection, downscale + the matching intrinsics rescale, pose composition,
# panorama tiling), and the view-sampling helpers (train/held-out split, stride
# sampler, scene extent). That is where the CPU-testable logic lives, so it is
# built unconditionally as `reusex_gsplat_common` and its tests run everywhere.
#
# Both halves list their sources EXPLICITLY rather than globbing, so a new file
# has to be assigned to one side deliberately. A torch include drifting into
# the CPU half would otherwise re-break the CPU build silently, which is the
# exact failure this split exists to prevent.

# CPU half — no torch, no CUDA, always built.
set(REUSEX_GSPLAT_COMMON_SOURCES
    ${SRC}/gsplat/GaussianCloud.cpp
    ${SRC}/gsplat/TrainingViews.cpp
    ${SRC}/gsplat/view_sampling.cpp)

# CUDA half — the training loop and everything torch-typed. Optional: needs
# WITH_CUDA plus the vendored Apache-2.0 gsplat rasterizer (pkgs/gsplat-cuda).
set(REUSEX_GSPLAT_SOURCES
    ${SRC}/gsplat/train.cpp
    ${SRC}/gsplat/mcmc.cpp
    ${SRC}/gsplat/optimizer.cpp
    ${SRC}/gsplat/rasterize.cpp
    ${SRC}/gsplat/ssim.cpp)

# Layer 4 — visualize (optional PCL/Qt/VTK)
file(GLOB_RECURSE REUSEX_VISUALIZE_SOURCES CONFIGURE_DEPENDS
     "${SRC}/visualize/*.cpp")

# Layer 4 — pipeline (DB-level stage execution + the in-process job runner).
# Added in #265 (GUI Phase 1). Unlike a Layer-3 peer it is ALLOWED to link
# several peers at once, because "run stage X against a project" inherently
# spans core (ProjectDB) plus whichever peers implement that stage. Nothing may
# depend on it except apps/tests, which keeps the peer layer clean.
# See docs/STANDARDS.md section 1.
file(GLOB_RECURSE REUSEX_PIPELINE_SOURCES CONFIGURE_DEPENDS
     "${SRC}/pipeline/*.cpp")

# Headers (attached to targets for IDE/install visibility; one flat tree).
file(GLOB_RECURSE REUSEX_HEADERS CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.cuh")

# -----------------------------------------------
# ML Backend Configuration (filters vision sources for disabled backends)
# -----------------------------------------------
# MLBackendConfig filters the variable named REUSEX_SOURCES, so hand it the
# vision source list under that name, then read it back.
set(REUSEX_SOURCES ${REUSEX_VISION_SOURCES})
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MLBackendConfig.cmake)
set(REUSEX_VISION_SOURCES ${REUSEX_SOURCES})

# -----------------------------------------------
# Create symlink for prefixed includes
# -----------------------------------------------
# build/include/reusex -> libs/reusex/include so consumers can use
# #include <reusex/core/...> while internal code uses "core/..." directly.
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include)
file(CREATE_LINK
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/include/reusex
    SYMBOLIC
)

# -----------------------------------------------
# Generate version header
# -----------------------------------------------
file(READ ${CMAKE_SOURCE_DIR}/LICENSE.md LICENSE_TEXT)
string(REPLACE "\"" "\\\"" LICENSE_TEXT "${LICENSE_TEXT}")
set(PROJECT_VERSION ${CMAKE_PROJECT_VERSION})
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/version.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/core/version.hpp
    @ONLY
)

# -----------------------------------------------
# Embed the canonical SAM 3.1 engine-build recipe
# -----------------------------------------------
# python/reusex_sam3/engine-build.json is the single source of truth for how
# each SAM 3.1 ONNX graph becomes a TensorRT engine. Embedding it lets the
# native EngineBuilder build from a bare `make export` ONNX dir (which does not
# carry engine-build.json) without a second, hand-maintained copy in C++.
set(REUSEX_SAM3_ENGINE_BUILD_JSON
    ${CMAKE_SOURCE_DIR}/python/reusex_sam3/engine-build.json)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${REUSEX_SAM3_ENGINE_BUILD_JSON})
file(READ ${REUSEX_SAM3_ENGINE_BUILD_JSON} SAM3_ENGINE_BUILD_JSON)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision/sam3/engine_build_default.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/vision/sam3/engine_build_default.hpp
    @ONLY
)

# -----------------------------------------------
# Embed the PDF report's Typst template
# -----------------------------------------------
# apps/rux/resources/report.typ is the only copy of the Ressourcekortlægning
# template (report-template-workflow spec). Listing it in
# CMAKE_CONFIGURE_DEPENDS re-runs configure when it is edited, so the
# generated header — and report_generator.cpp — rebuild with it.
set(REUSEX_REPORT_TEMPLATE_TYP
    ${CMAKE_SOURCE_DIR}/apps/rux/resources/report.typ)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${REUSEX_REPORT_TEMPLATE_TYP})
file(READ ${REUSEX_REPORT_TEMPLATE_TYP} REUSEX_REPORT_TEMPLATE_TEXT)
string(FIND "${REUSEX_REPORT_TEMPLATE_TEXT}" ")reusextypst\"" _report_delim)
if(NOT _report_delim EQUAL -1)
    message(FATAL_ERROR
        "${REUSEX_REPORT_TEMPLATE_TYP} contains the raw-string delimiter "
        "')reusextypst\"' used to embed it; rename the delimiter in "
        "report_template_default.hpp.in")
endif()
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core/report_template_default.hpp.in
    ${CMAKE_BINARY_DIR}/generated/reusex/core/report_template_default.hpp
    @ONLY
)

# ===============================================
# Shared usage requirements (include dirs, flags, external deps)
# ===============================================
# Every module links this INTERFACE target, so include dirs, compile options
# and third-party libraries are declared once and propagated uniformly. It also
# propagates to consumers of the `reusex` umbrella (rux, ruxd, tests, bindings).
add_library(reusex_common INTERFACE)

target_include_directories(reusex_common INTERFACE
    # Prefixed access for consumers: reusex/core/...
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/include>
    # Flat access for internal library code: core/..., geometry/...
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # Installed package (cmake/Installation.cmake): public headers live under
    # include/reusex/ (so <reusex/core/...> resolves from include/), and the
    # vendored extern headers under include/reusex/extern/ — public headers
    # include them as <pcl/planar_region_growing.hpp>, and keeping them out of
    # a shared include/pcl/ avoids colliding with the real PCL in /usr/local.
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/reusex/extern>
)

target_link_libraries(reusex_common INTERFACE
    # Geometry & CGAL
    CGAL::CGAL
    CGAL::Eigen3_support
    CGAL::TBB_support
    # MIP Solver
    highs::highs
    # 3D Formats
    OpenNURBS
    E57Format
    # Point Cloud Library (excluding visualization)
    pcl_common
    pcl_io
    pcl_filters
    pcl_segmentation
    pcl_registration
    pcl_surface
    # Ray tracing
    embree
    # Linear Algebra
    Eigen3::Eigen
    # Formatting used by ReUseX logging facade
    fmt::fmt
    # JSON serialization
    nlohmann_json::nlohmann_json
    # Ranges — header-only, and included by public headers
    # (geometry/utils.hpp, reconstruction/*.hpp), so it must be PUBLIC: as a
    # private dep it would only reach consumers as $<LINK_ONLY:...>, without
    # its include directories.
    range-v3::range-v3
    # Graph algorithms
    igraph::igraph
    # Computer Vision
    opencv_core
    opencv_imgproc
    opencv_highgui
    # SLAM database access
    rtabmap::rtabmap
    # Common linker flags
    ${COMMON_LINKER_FLAGS}
)

# BUILD_INTERFACE only: these are this build's warning/OpenMP/sanitizer flags
# (-Wall -Wextra [-Werror] -fopenmp ...). An installed-package consumer must not
# inherit our -Werror or warning set; the OpenMP runtime still reaches it
# through COMMON_LINKER_FLAGS (-lgomp) above.
target_compile_options(reusex_common INTERFACE
    $<BUILD_INTERFACE:$<$<COMPILE_LANGUAGE:CXX>:${COMMON_COMPILER_FLAGS}>>
    $<BUILD_INTERFACE:$<$<COMPILE_LANGUAGE:C>:${COMMON_COMPILER_FLAGS}>>
)

# MIP solver compile definition (USE_HIGHS / USE_CUOPT) attaches to the shared
# config so every module and the umbrella inherit it. MIPSolverConfig computes
# USE_MIP_SOLVER; its configure_mip_solver() helper uses PUBLIC keywords that an
# INTERFACE target rejects, so we apply the result with INTERFACE keywords here.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MIPSolverConfig.cmake)
if(USE_MIP_SOLVER STREQUAL "CUOPT")
    target_compile_definitions(reusex_common INTERFACE USE_CUOPT)
    target_link_libraries(reusex_common INTERFACE cuopt::cuopt)
    message(STATUS "MIP Solver: cuOpt (GPU-accelerated)")
elseif(USE_MIP_SOLVER STREQUAL "HIGHS")
    target_compile_definitions(reusex_common INTERFACE USE_HIGHS)
    # highs::highs already linked above.
    message(STATUS "MIP Solver: HiGHS (CPU)")
else()
    message(FATAL_ERROR "Unknown MIP solver: ${USE_MIP_SOLVER}")
endif()

# Some third-party PRIVATE-only dependencies of individual translation units
# (not part of the public interface). Declared on a dedicated INTERFACE target
# so we can attach them to the specific owning module(s) below.
add_library(reusex_private_deps INTERFACE)
target_link_libraries(reusex_private_deps INTERFACE
    # Speckle upload support (io/speckle.cpp)
    CURL::libcurl
    OpenSSL::Crypto
    # EXIF metadata reading (io/exif.cpp)
    Exiv2::exiv2lib
    # OpenMVS — Multi-View Stereo (AGPL-3.0-or-later), kept PRIVATE
    OpenMVS::MVS
)

# ===============================================
# Module targets
# ===============================================
# Helper: declare a module static library with the shared config attached.
function(reusex_add_module NAME)
    add_library(${NAME} STATIC ${ARGN})
    target_link_libraries(${NAME} PUBLIC reusex_common)
    target_link_libraries(${NAME} PRIVATE reusex_private_deps)
    set_target_properties(${NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endfunction()

# --- Layer 1 ---------------------------------------------------------------
reusex_add_module(reusex_utils ${REUSEX_UTILS_SOURCES})

# --- Layer 1.5 — shared geometry helpers -----------------------------------
# geometry_common uses core/logging only; it must NOT depend on core, so it
# stays usable from every higher layer. It links utils + the shared config.
reusex_add_module(reusex_geometry_common ${REUSEX_GEOMETRY_COMMON_SOURCES})
target_link_libraries(reusex_geometry_common PUBLIC reusex_utils)

# --- Layer 2 ---------------------------------------------------------------
reusex_add_module(reusex_core ${REUSEX_CORE_SOURCES})
target_link_libraries(reusex_core PUBLIC reusex_utils)
# sqlite3 backs ProjectDB. Previously pulled in transitively via the monolithic
# shared lib; per-module static libs need it named explicitly so the symbols are
# on the final link line (DSO-missing-from-command-line otherwise).
find_package(SQLite3 REQUIRED)
# CMake 4.3 renamed FindSQLite3's target to SQLite3::SQLite3 and deprecated
# SQLite::SQLite3 (an author warning per link). Use the new name, aliasing it on
# older CMake as FindSQLite3's own documentation recommends.
if(NOT TARGET SQLite3::SQLite3)
    add_library(SQLite3::SQLite3 ALIAS SQLite::SQLite3)
endif()
target_link_libraries(reusex_core PUBLIC SQLite3::SQLite3)
# NOTE (#227): core deliberately does NOT link reusex_geometry_common.
# ProjectDB persists the core-owned POD `core::ComponentRecord`; the
# BuildingComponent <-> record mapping lives in the header-only adapter
# geometry/component_persistence.hpp, compiled into its consumers (io, rux,
# tests) rather than into either module. See docs/STANDARDS.md §1.

# --- Layer 3 (peers — MUST NOT link each other) ----------------------------
reusex_add_module(reusex_io ${REUSEX_IO_SOURCES})
target_link_libraries(reusex_io PUBLIC reusex_core)
# LAYERING EXCEPTION (#222): io/export_scene consumes geometry::BuildingComponent
# / cgal_utils / unweld to serialize a reconstructed scene. Peer io -> geometry
# dependency, documented in docs/STANDARDS.md §1 ([target]) and export_scene.hpp.
target_link_libraries(reusex_io PUBLIC reusex_geometry_common reusex_reconstruction)
# opencv_imgcodecs: cv::imread / cv::imencode for .insp dual-fisheye stitching (#461).
# opencv_stitching: MultiBandBlender + ExposureCompensator for seam blend (#468).
# PRIVATE: not part of any public header; opencv_core/imgproc arrive via reusex_common.
target_link_libraries(reusex_io PRIVATE opencv_imgcodecs opencv_stitching)

reusex_add_module(reusex_vision ${REUSEX_VISION_SOURCES})
target_link_libraries(reusex_vision PUBLIC reusex_core)
# opencv_imgcodecs backs the VLM client's cv::imencode(.jpg) of instance crops
# (OpenAiCompatibleVlmClient.cpp). PRIVATE: not part of any public header;
# opencv_core / opencv_imgproc already arrive via reusex_common.
target_link_libraries(reusex_vision PRIVATE opencv_imgcodecs)
# ML backend libraries + REUSEX_USE_* defines attach to the vision module only.
configure_ml_backends(reusex_vision)

reusex_add_module(reusex_segmentation ${REUSEX_SEGMENTATION_SOURCES})
target_link_libraries(reusex_segmentation PUBLIC reusex_core reusex_geometry_common)

reusex_add_module(reusex_reconstruction ${REUSEX_RECONSTRUCTION_SOURCES})
target_link_libraries(reusex_reconstruction PUBLIC reusex_core reusex_geometry_common)

reusex_add_module(reusex_slam ${REUSEX_SLAM_SOURCES})
target_link_libraries(reusex_slam PUBLIC reusex_core)
# GTSAM — factor-graph pose-graph optimization, used only inside
# PlaneGraphOptimizer.cpp. PRIVATE: not part of any public header. As a static
# lib this still propagates for final linking.
target_link_libraries(reusex_slam PRIVATE gtsam)
# OpenCV feature matching (ORB + BFMatcher) backs the P2 loop-closure front-end
# (LoopClosure.cpp). PRIVATE: not part of any public header. opencv_core /
# opencv_imgproc already arrive via reusex_common.
target_link_libraries(reusex_slam PRIVATE opencv_features2d)
# solvePnPRansac (panorama alignment resection) lives in opencv_calib3d;
# opencv_imgcodecs backs the optional ORB-correspondence debug figure.
target_link_libraries(reusex_slam PRIVATE opencv_calib3d opencv_imgcodecs)
# NARROW EXCEPTION (#222): slam registration reuses segmentation's Surfel /
# surfel_extraction (PlaneGraphOptimizer, JointPairwiseRegistration operate on
# extracted surfels). Explicit peer edge slam -> segmentation, kept narrow.
target_link_libraries(reusex_slam PUBLIC reusex_segmentation)

# --- Layer 3 — gsplat_common (CPU half, always built) ----------------------
# The torch-free part of the Gaussian-splatting module: the GaussianCloud value
# type, the ProjectDB -> TrainingView loader, and the view-sampling helpers.
# An ordinary Layer-3 peer in every respect — it links core + geometry_common
# (for EquirectProjection, which the panorama tiling uses) and links no peer.
# Built unconditionally so the CPU-only CI variant compiles and runs its tests
# (#332); see the source-list comment above for why that matters.
reusex_add_module(reusex_gsplat_common ${REUSEX_GSPLAT_COMMON_SOURCES})
target_link_libraries(reusex_gsplat_common
    PUBLIC reusex_core reusex_geometry_common)

# --- Layer 3 — gsplat (optional: WITH_CUDA + vendored gsplat) --------------
# 3D Gaussian Splatting trainer (#240). The differentiable rasterizer is
# gsplat's Apache-2.0 CUDA backend, vendored by pkgs/gsplat-cuda as a LibTorch-
# linked static library; gsplat::gsplat propagates ${TORCH_LIBRARIES} PUBLIC, so
# linking it is what brings LibTorch into this module. Skipped entirely (with a
# STATUS line, never silently) when CUDA is off or the package is absent — the
# `rux create gsplat` subcommand then reports that the build lacks support
# rather than failing to link.
set(REUSEX_HAVE_GSPLAT OFF)
if(WITH_CUDA AND REUSEX_GSPLAT_SOURCES)
    find_package(gsplat-cuda CONFIG QUIET)
    if(gsplat-cuda_FOUND)
        set(REUSEX_HAVE_GSPLAT ON)
    else()
        message(STATUS "gsplat-cuda not found — reusex_gsplat module disabled "
                       "(`rux create gsplat` will report unavailable)")
    endif()
else()
    message(STATUS "WITH_CUDA=OFF — reusex_gsplat module disabled")
endif()

if(REUSEX_HAVE_GSPLAT)
    reusex_add_module(reusex_gsplat ${REUSEX_GSPLAT_SOURCES})
    # The CUDA half is built on the CPU half, not beside it: train.cpp consumes
    # GaussianCloud, TrainingView and the view-sampling helpers directly.
    # PUBLIC, so a consumer that links only reusex_gsplat still resolves them
    # (and reaches core / geometry_common transitively).
    target_link_libraries(reusex_gsplat PUBLIC reusex_gsplat_common)
    # gsplat + LibTorch, kept PRIVATE. gsplat::gsplat propagates
    # ${TORCH_LIBRARIES} — including Torch's _GLIBCXX_USE_CXX11_ABI define and
    # its 1.8 GB CUDA closure — so letting it go PUBLIC would push both onto
    # every consumer of the `reusex` umbrella. The module's public headers
    # therefore expose no torch type (pimpl, like ProjectDB hides sqlite3;
    # STANDARDS §2). As a static library the link still propagates to the final
    # executable, which is all that is needed.
    target_link_libraries(reusex_gsplat PRIVATE gsplat::gsplat)
    # Signals to consumers (apps/rux, tests) that the trainer is compiled in.
    target_compile_definitions(reusex_gsplat PUBLIC REUSEX_HAVE_GSPLAT)

    # Which compute capabilities the vendored kernels were actually built for.
    # pkgs/gsplat-cuda builds ONE architecture by default (sm_89) and emits no
    # PTX, so there is no JIT fallback — on any other GPU the first kernel
    # launch dies with "no kernel image is available", minutes into a run.
    # require_cuda() checks this list up front and names the override; keep the
    # two in step when changing `cudaCapabilities` in the derivation.
    set(REUSEX_GSPLAT_CUDA_ARCHS "89" CACHE STRING
        "Compute capabilities (major*10+minor, comma-separated) the vendored \
gsplat kernels were compiled for. Mirror pkgs/gsplat-cuda/package.nix's \
cudaCapabilities.")
    target_compile_definitions(reusex_gsplat
        PRIVATE REUSEX_GSPLAT_CUDA_ARCHS="${REUSEX_GSPLAT_CUDA_ARCHS}")
endif()

# --- Layer 4 — visualize (optional) ----------------------------------------
if(REUSEX_VISUALIZE_SOURCES)
    reusex_add_module(reusex_visualize ${REUSEX_VISUALIZE_SOURCES})
    target_link_libraries(reusex_visualize PUBLIC reusex_core)
    # Layer 4 -> Layer 1.5 (legal downward edge): render_view draws building
    # components, so it needs BuildingComponent / CoplanarPolygon and the
    # header-only pose helpers in geometry/transform_utils.hpp.
    target_link_libraries(reusex_visualize PUBLIC reusex_geometry_common)
    # VTK backs the headless render_view() (#294). PRIVATE: no VTK type appears
    # in visualize/render_view.hpp, so consumers never compile VTK headers.
    target_link_libraries(reusex_visualize PRIVATE ${VTK_LIBRARIES})
    # Registers the RenderingOpenGL2 object factory in this module's TUs.
    # Without it vtkRenderWindow::New() returns a base object with no OpenGL
    # implementation and rendering silently produces nothing.
    vtk_module_autoinit(TARGETS reusex_visualize MODULES ${VTK_LIBRARIES})
endif()

# --- Layer 4 — pipeline (stage execution + job runner) ---------------------
reusex_add_module(reusex_pipeline ${REUSEX_PIPELINE_SOURCES})
target_link_libraries(reusex_pipeline PUBLIC
    reusex_core
    reusex_io
    reusex_reconstruction
    reusex_segmentation)

# ===============================================
# Umbrella target — backward compatible `reusex`
# ===============================================
# INTERFACE library linking every module, so existing consumers that link
# `reusex` and include <reusex/...> keep working unchanged.
add_library(reusex INTERFACE)
target_link_libraries(reusex INTERFACE
    reusex_common
    reusex_utils
    reusex_geometry_common
    reusex_core
    reusex_io
    reusex_vision
    reusex_segmentation
    reusex_reconstruction
    reusex_slam
    reusex_gsplat_common
    reusex_pipeline
)
if(TARGET reusex_visualize)
    target_link_libraries(reusex INTERFACE reusex_visualize)
endif()
if(TARGET reusex_gsplat)
    target_link_libraries(reusex INTERFACE reusex_gsplat)
endif()

# -----------------------------------------------
# Package names: ReUseX::<module>
# -----------------------------------------------
# The installed package (cmake/Installation.cmake) exports every target under
# the ReUseX:: namespace with the `reusex_` prefix dropped — reusex_core is
# ReUseX::core, the umbrella stays ReUseX::reusex. The same names are aliased
# here so in-tree code can already be written the way an external consumer
# (`find_package(ReUseX CONFIG REQUIRED)`) writes it.
set(REUSEX_PACKAGE_TARGETS
    reusex reusex_common reusex_private_deps
    reusex_utils reusex_geometry_common reusex_core reusex_io reusex_vision
    reusex_segmentation reusex_reconstruction reusex_slam
    reusex_gsplat_common reusex_pipeline)
foreach(_opt reusex_visualize reusex_gsplat)
    if(TARGET ${_opt})
        list(APPEND REUSEX_PACKAGE_TARGETS ${_opt})
    endif()
endforeach()
foreach(_tgt IN LISTS REUSEX_PACKAGE_TARGETS)
    if(_tgt STREQUAL "reusex")
        set(_export_name reusex)
    else()
        string(REGEX REPLACE "^reusex_" "" _export_name "${_tgt}")
    endif()
    set_target_properties(${_tgt} PROPERTIES EXPORT_NAME ${_export_name})
    add_library(ReUseX::${_export_name} ALIAS ${_tgt})
endforeach()
unset(_export_name)

# -----------------------------------------------
# Diagnostics
# -----------------------------------------------
foreach(mod utils geometry_common core io vision segmentation reconstruction slam pipeline visualize gsplat_common gsplat)
    if(TARGET reusex_${mod})
        get_target_property(_srcs reusex_${mod} SOURCES)
        list(LENGTH _srcs _n)
        message(STATUS "reusex module '${mod}': ${_n} source(s)")
    endif()
endforeach()

# -----------------------------------------------
# Export library information
# -----------------------------------------------
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REUSEX_LIBRARY_TARGET reusex PARENT_SCOPE)
    set(USE_MIP_SOLVER ${USE_MIP_SOLVER} PARENT_SCOPE)
    set(ENABLED_ML_BACKENDS ${ENABLED_ML_BACKENDS} PARENT_SCOPE)
    set(REUSEX_HAVE_GSPLAT ${REUSEX_HAVE_GSPLAT} PARENT_SCOPE)
    set(REUSEX_PACKAGE_TARGETS ${REUSEX_PACKAGE_TARGETS} PARENT_SCOPE)
endif()
