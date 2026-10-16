# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# ReUseX Unit Tests with Catch2 v3
# ===============================================

message(STATUS "Configuring unit tests...")

# -----------------------------------------------
# Find dependencies
# -----------------------------------------------
find_package(Catch2 3 REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)
# CMake 4.3 renamed FindSQLite3's target to SQLite3::SQLite3 and deprecated
# SQLite::SQLite3 (an author warning per link). Use the new name, aliasing it on
# older CMake as FindSQLite3's own documentation recommends.
if(NOT TARGET SQLite3::SQLite3)
    add_library(SQLite3::SQLite3 ALIAS SQLite::SQLite3)
endif()

# Note: ENABLE_COVERAGE option is now set in main CMakeLists.txt
# Coverage flags are automatically inherited from parent scope

# -----------------------------------------------
# Collect test source files from unit/ directory
# -----------------------------------------------
#
# The unit tests are split across TWO executables, purely for run-time speed
# (#268). `catch_discover_tests` registers one ctest test per TEST_CASE, and
# each one pays a full process start. When every test case linked the umbrella
# `reusex` target the binary pulled in 505 shared objects — libtorch_cuda
# (1.8 GB), TensorRT, cuOpt, NCCL, the OpenCV CUDA modules — and the dynamic
# loader spent ~1.9 s per process before main() even ran. Multiplied by ~360
# test cases that was ~11 minutes of pure relocation work.
#
# Only tests/unit/vision needs the ML stack, tests/unit/rux_app needs rux_lib
# (the `rux` CLI minus main(), which links the `reusex` umbrella and the PCL
# viewer), and tests/unit/visualize needs the PCL/Qt viewer module. Those
# directories get their own "heavy" binary; everything else links just the
# modules it actually uses and starts an order of magnitude faster.
#
# The app layer splits the same way the library does (#249): `tests/unit/rux/`
# is LIGHT and covers `rux_core_lib` — the app logic that stops at reusex_core
# (path parsing, filter validation, stage prerequisites, stdin/format
# handling). Only a test that needs a subcommand, a router or the viewer
# belongs in `tests/unit/rux_app/`.
#
# gsplat splits *within* its directory rather than by directory (#332).
# `tests/unit/gsplat/cuda/` is heavy — those tests link torch, and the ones
# that also need a device are [gpu]-tagged and SKIP without one. Everything
# directly under `tests/unit/gsplat/` is light: it exercises
# `reusex_gsplat_common`, the torch-free half of the module (GaussianCloud,
# the TrainingViews loader, the view-sampling helpers).
#
# That distinction is the entire point of #332. CI builds the CPU-only nix
# variant, where `REUSEX_HAVE_GSPLAT` is false; while the whole directory was
# heavy-and-gated, a CPU-only build compiled ZERO gsplat tests, so none of the
# module's logic — SH round trip, `.ply` I/O, the held-out split rule, the
# intrinsics rescale — was ever exercised by a pull request. Putting the
# CPU-testable surface in the binary CI actually builds is what changes that.
#
# Test *names* are unchanged either way: ctest still sees one test per
# TEST_CASE, so `ctest -R <name>` keeps working exactly as before.
file(GLOB_RECURSE UNIT_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/unit/*.cpp")

set(HEAVY_TEST_SOURCES ${UNIT_TEST_SOURCES})
list(FILTER HEAVY_TEST_SOURCES INCLUDE REGEX
     "/unit/(vision|rux_app|visualize)/|/unit/gsplat/cuda/")

# tests/unit/gsplat/cuda exercises reusex_gsplat, which only exists when CUDA
# and the vendored gsplat rasterizer were both found. Drop those sources
# otherwise rather than fail to link a CPU-only build. The CPU-only gsplat
# tests one directory up are unaffected and keep running.
if(NOT REUSEX_HAVE_GSPLAT)
    list(FILTER HEAVY_TEST_SOURCES EXCLUDE REGEX "/unit/gsplat/cuda/")
    message(STATUS "Excluding tests/unit/gsplat/cuda (gsplat backend disabled)")
endif()

# reusex/vision/nms.hpp is a torch::Tensor API, and MLBackendConfig.cmake drops
# nms.cpp/.hpp from the library entirely when the LibTorch backend is off. Its
# test has to follow the same rule or a -DML_BACKENDS=ONNX build stops
# compiling.
if(NOT "LibTorch" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER HEAVY_TEST_SOURCES EXCLUDE REGEX "/unit/vision/test_nms\\.cpp$")
    message(STATUS "Excluding test_nms.cpp (LibTorch backend disabled)")
endif()

# Same rule for the TensorRT backend: MLBackendConfig.cmake drops
# src/vision/tensor_rt/** (the .cpp *and* the .cu kernels) from reusex_vision
# when TensorRT is off, so any test that names a symbol from there stops
# linking. That is what breaks the CPU build (WITH_CUDA=OFF), which CI runs via
# `nix build .#checks.x86_64-linux.tests` (#202).
#
#   * test_transpose_kernel.cpp calls chw_to_hwc_device_roundtrip(), defined in
#     kernels/transpose.cu, and needs a CUDA device at run time anyway.
#   * test_annotate_resume.cpp instantiates TensorRTDataset purely as a concrete
#     IDataset to exercise the base-class resume logic; without the backend even
#     its vtable is gone.
#
# The condition is deliberately the same ENABLED_ML_BACKENDS check the library
# filter uses, so the two cannot drift apart.
if(NOT "TensorRT" IN_LIST ENABLED_ML_BACKENDS)
    list(FILTER HEAVY_TEST_SOURCES EXCLUDE REGEX
         "/unit/vision/test_(transpose_kernel|annotate_resume)\\.cpp$")
    message(STATUS "Excluding TensorRT-only vision tests (backend disabled)")
endif()

set(LIGHT_TEST_SOURCES ${UNIT_TEST_SOURCES})
list(FILTER LIGHT_TEST_SOURCES EXCLUDE REGEX
     "/unit/(vision|rux_app|visualize)/|/unit/gsplat/cuda/")

# -----------------------------------------------
# Create unit test executables
# -----------------------------------------------
add_executable(reusex_unit_tests ${LIGHT_TEST_SOURCES})

# Deliberately NOT the umbrella `reusex` target: linking the individual
# modules keeps reusex_vision (and with it libtorch/TensorRT) out of this
# binary. Adding a test that needs vision means moving it to
# tests/unit/vision/, not widening this list.
#
# rux_core_lib (#249) is here for the same reason: it stops at reusex_core, so
# tests/unit/rux gets the app layer's pure logic -- path_parser, filter_utils --
# under test at light-binary start-up cost.
#
# ruxd and its API/CLI/Postgres libraries moved to the standalone ruxd repo
# (repo split, P4) along with tests/unit/ruxd{,_api,_cli,_pg}.
target_link_libraries(reusex_unit_tests
    PRIVATE
        reusex_core
        reusex_io
        reusex_utils
        reusex_geometry_common
        reusex_segmentation
        reusex_reconstruction
        reusex_slam
        reusex_pipeline
        # The torch-free half of the gsplat module (#332). It is a plain
        # core+geometry_common consumer -- linking it adds no libtorch, no
        # TensorRT and no CUDA closure, which is exactly why the CPU-testable
        # gsplat tests can live in this binary.
        reusex_gsplat_common
        rux_core_lib
        Catch2::Catch2WithMain
        fmt::fmt
        OpenSSL::Crypto
        SQLite3::SQLite3
)

# The Qt client's Qt-free half: tokens.css parsing, the gallery's command
# line, palette ranking, the recent list, what plain `rux` launches.
# Standard library only and built even with BUILD_QT_CLIENT=OFF
# (apps/rux/qt/CMakeLists.txt), so tests/unit/rux_qt always runs and costs
# this binary no Qt start-up.
target_link_libraries(reusex_unit_tests PRIVATE rux_qt_core)

target_include_directories(reusex_unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/reusex/include
    ${CMAKE_BINARY_DIR}/generated
    # tests/unit/gsplat/test_view_sampling.cpp covers split_views /
    # stride_sample / scene_extent directly, and those live behind
    # libs/reusex/src/gsplat/view_sampling.hpp, which is module-private on
    # purpose (STANDARDS §2: it is an implementation detail of the training
    # loop). Same arrangement the heavy binary uses for optimizer.hpp below.
    ${CMAKE_SOURCE_DIR}/libs/reusex/src
)

# Tests that read checked-in files (docs/CONTRACTS.md for the stage contract
# #246, the report template, tokens.css, engine-build.json) locate the source
# tree through reusex::test_support::source_dir() (tests/support/source_dir.hpp).
# The absolute path is a definition on that one TU only, never on the targets:
# a target-wide -DREUSEX_SOURCE_DIR="/abs/path" sits in every test TU's
# command line, which ccache hashes verbatim, so no test object could be
# shared between two worktrees (reusex-37v). Both executables compile it.
set(TEST_SUPPORT_SOURCE_DIR_TU "${CMAKE_CURRENT_SOURCE_DIR}/support/source_dir.cpp")
set_source_files_properties("${TEST_SUPPORT_SOURCE_DIR_TU}" PROPERTIES
    COMPILE_DEFINITIONS "REUSEX_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
target_sources(reusex_unit_tests PRIVATE "${TEST_SUPPORT_SOURCE_DIR_TU}")

# Vision tests: these genuinely need the ML backends, so they carry the full
# dependency closure.
if(HEAVY_TEST_SOURCES)
    add_executable(reusex_unit_tests_vision ${HEAVY_TEST_SOURCES})

    # rux_lib carries the `rux` CLI (#249): every subcommand, every
    # database/*_router.cpp and rux::run() itself, minus the `main` symbol,
    # reachable from tests/unit/rux_app.
    target_link_libraries(reusex_unit_tests_vision
        PRIVATE
            reusex
            rux_lib
            Catch2::Catch2WithMain
            fmt::fmt
            OpenSSL::Crypto
            SQLite3::SQLite3
    )

    target_include_directories(reusex_unit_tests_vision PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )

    # Lets tests locate checked-in reference assets (e.g. the canonical
    # python/reusex_sam3/engine-build.json parsed by test_engine_build_profiles);
    # see TEST_SUPPORT_SOURCE_DIR_TU above.
    target_sources(reusex_unit_tests_vision PRIVATE "${TEST_SUPPORT_SOURCE_DIR_TU}")

    # configure_ml_backends() links torch PRIVATE to reusex_vision, so neither
    # the torch headers nor its ABI flags reach a consumer of the umbrella
    # `reusex` target. tests/unit/vision/test_nms.cpp includes
    # reusex/vision/nms.hpp, which is torch::Tensor-typed, so it needs its own
    # link.
    #
    # find_package(Torch) has to be repeated here: Dependencies.cmake runs in
    # the libs/reusex directory scope, and imported targets are visible only in
    # the directory that created them (and below). Without this, `torch`
    # degrades to a bare -ltorch with no include directories and the test fails
    # to compile on <torch/torch.h>. Kept behind the same guard as the source
    # filter above.
    if("LibTorch" IN_LIST ENABLED_ML_BACKENDS)
        find_package(Torch REQUIRED)
        target_link_libraries(reusex_unit_tests_vision PRIVATE torch)
    endif()

    # tests/unit/gsplat/cuda/test_optimizer.cpp covers the density-control row
    # remap directly, and test_ssim.cpp the SSIM loss. Both live behind
    # module-private headers (libs/reusex/src/gsplat/optimizer.hpp, ssim.hpp)
    # which are private on purpose — they expose torch types, STANDARDS §2 — so
    # the tests need the src root on their include path, and torch's headers
    # whether or not the LibTorch *ML backend* happens to be enabled. Testing
    # the remap through the public API is not an option: carrying Adam's moments
    # across a remap has no observable effect other than the moments themselves.
    #
    # Neither test needs a CUDA *device*; they run on CPU tensors. The ones that
    # do are [gpu]-tagged in test_train.cpp and SKIP when has_cuda_device() is
    # false.
    if(TARGET reusex_gsplat)
        find_package(Torch REQUIRED)
        target_link_libraries(reusex_unit_tests_vision PRIVATE torch)
        target_include_directories(reusex_unit_tests_vision PRIVATE
            ${CMAKE_SOURCE_DIR}/libs/reusex/src)
    endif()
endif()

# -----------------------------------------------
# Integration tests (future)
# -----------------------------------------------
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/integration/*.cpp")

if(INTEGRATION_TEST_SOURCES)
    add_executable(reusex_integration_tests ${INTEGRATION_TEST_SOURCES})
    target_link_libraries(reusex_integration_tests
        PRIVATE
            reusex
            Catch2::Catch2WithMain
            fmt::fmt
            # cv::imwrite, for the opt-in render artifacts written when
            # REUSEX_TEST_RENDER_DIR is set (#294).
            opencv_imgcodecs
    )
    target_include_directories(reusex_integration_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )

    # test_render_view.cpp / test_real_scan_fixture.cpp locate
    # tests/fixtures/scans/office_corridor.rux through
    # reusex::test_support::source_dir(); see TEST_SUPPORT_SOURCE_DIR_TU above.
    target_sources(reusex_integration_tests PRIVATE "${TEST_SUPPORT_SOURCE_DIR_TU}")

    # test_cli_exit_codes.cpp asserts the process exit code of the real binary
    # (#299) — the one thing no in-process test can observe — so it needs the
    # path to `rux` and an explicit build-order dependency on it. `tests` is
    # added after `apps/rux`, so the target exists here.
    if(TARGET rux)
        target_compile_definitions(reusex_integration_tests PRIVATE
            REUSEX_RUX_EXECUTABLE="$<TARGET_FILE:rux>")
        add_dependencies(reusex_integration_tests rux)
    endif()
endif()

# -----------------------------------------------
# Benchmarks (Catch2 BENCHMARK; run via scripts/bench.sh, not ctest)
# -----------------------------------------------
file(GLOB_RECURSE BENCHMARK_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/benchmarks/*.cpp")

if(BENCHMARK_SOURCES)
    add_executable(reusex_benchmarks ${BENCHMARK_SOURCES})
    target_link_libraries(reusex_benchmarks
        PRIVATE
            reusex
            Catch2::Catch2WithMain
            fmt::fmt
    )
    target_include_directories(reusex_benchmarks PRIVATE
        ${CMAKE_SOURCE_DIR}/libs/reusex/include
        ${CMAKE_BINARY_DIR}/generated
    )
endif()

# -----------------------------------------------
# Enable CTest and discover tests
# -----------------------------------------------
include(CTest)
include(Catch)

# Discover each test binary twice, splitting on the Catch2 `[gpu]` tag.
#
# There is exactly one GPU on a dev box, and the cuOpt / cuDSS tests fall over
# when several of them initialise the solver at the same time ("CUDSS call
# ended unsuccessfully ... cudssSetThreadingLayer" at `ctest -j48`). That is a
# device-contention problem, not the temp-path collision fixed in #262, and it
# is the one thing that stopped the suite scaling past -j8.
#
# RESOURCE_LOCK makes ctest run every [gpu]-tagged test one at a time while
# leaving the other ~350 free to run fully parallel. The locked set is small
# (~11 tests, ~30 s total), well under the critical path set by the slowest
# single test, so serializing them costs no wall time.
#
# Two calls on one target are safe: catch_discover_tests derives its generated
# file names from a hash of TEST_SPEC, so the two sets do not overwrite each
# other. Total discovered test count is unchanged.
function(reusex_discover_tests target)
    catch_discover_tests(${target} TEST_SPEC "~[gpu]")
    catch_discover_tests(${target} TEST_SPEC "[gpu]"
                         PROPERTIES RESOURCE_LOCK reusex_gpu)
endfunction()

reusex_discover_tests(reusex_unit_tests)

if(HEAVY_TEST_SOURCES)
    reusex_discover_tests(reusex_unit_tests_vision)
endif()

if(INTEGRATION_TEST_SOURCES)
    catch_discover_tests(reusex_integration_tests)
endif()

# -----------------------------------------------
# Installed-package consumer (repo split, P1)
# -----------------------------------------------
# `cmake --install`s the ReUseX_Development component into a temp prefix, then
# configures, builds and runs tests/package/consumer against it with
# find_package(ReUseX CONFIG REQUIRED). This is what keeps the exported CMake
# package (cmake/Installation.cmake, cmake/ReUseXConfig.cmake.in) from rotting
# now that ruxd consumes the library from outside this tree. It needs a
# complete build (it installs the built archives) and re-runs every
# find_package of the dependency closure, so it takes on the order of a minute;
# `ctest -L package` runs it alone, `-LE package` skips it.
option(REUSEX_PACKAGE_TEST
       "Register the installed-package consumer ctest (tests/package)" ON)
if(REUSEX_PACKAGE_TEST)
    add_test(NAME reusex_package_consumer
             COMMAND ${CMAKE_COMMAND}
                     -DREUSEX_BUILD_DIR=${CMAKE_BINARY_DIR}
                     -DCONSUMER_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/package/consumer
                     -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/package_test
                     -DBUILD_CONFIG=$<CONFIG>
                     -DGENERATOR=${CMAKE_GENERATOR}
                     -DCXX_COMPILER=${CMAKE_CXX_COMPILER}
                     "-DOUTER_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/package/run_package_test.cmake)
    set_tests_properties(reusex_package_consumer PROPERTIES
        LABELS package
        TIMEOUT 1800
        # Its own work dir, but a long multi-process configure+link: keep it
        # from competing with itself if ctest is re-run concurrently.
        RESOURCE_LOCK reusex_package_test)
endif()

# -----------------------------------------------
# Bench script self-tests (stdlib-only python)
# -----------------------------------------------
# scripts/bench-solid-compare.py gates the solid-model benchmark
# (reusex-z1v.2.5); its self-test needs only python3, so it is skipped
# quietly where no interpreter is around (e.g. a minimal nix check build).
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    add_test(NAME bench_solid_compare_selftest
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_SOURCE_DIR}/scripts/test_bench_solid_compare.py)
    set_tests_properties(bench_solid_compare_selftest PROPERTIES LABELS bench)

    # Regression guard for reusex-4hy: a __FILE__-derived fixture path breaks
    # under the dev shell's shared ccache (CCACHE_BASEDIR, #55) but not in the
    # nix sandbox build, so it slips past CI unless something greps for it.
    add_test(NAME check_no_file_macro_paths
             COMMAND ${Python3_EXECUTABLE}
                     ${CMAKE_CURRENT_SOURCE_DIR}/support/check_no_file_macro_paths.py)
    set_tests_properties(check_no_file_macro_paths PROPERTIES LABELS lint)
endif()

message(STATUS "Unit tests configured successfully")
