# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  cmake,
  stdenv,
  config,
  cudaSupport ? config.cudaSupport,
  # Opt-in torch-free build (`.#cuda-notorch` / `.#cpu-notorch`). When false,
  # libtorch and gsplat-cuda (itself LibTorch-linked) are left out of
  # buildInputs and ML_BACKENDS is pinned to the non-LibTorch backends, so:
  #   - the LibTorch ML backend (YOLO .pt), vision/nms and the vendored
  #     torchvision kernel are excluded by MLBackendConfig.cmake;
  #   - reusex_gsplat is skipped (REUSEX_HAVE_GSPLAT off) and
  #     `rux create gsplat` reports that the build lacks support.
  # The default `true` keeps the derivation byte-identical to before.
  withLibtorch ? true,
  # Also install what ctest needs to run the suite later, in another
  # derivation, into a second output `testsuite`, and expose that run as
  # passthru.check (reusex-uk8). flake.nix turns it on for `.#cpu` only:
  # `checks.tests` is `cpu.check`. Off, the derivation has the single `out`.
  withTestsOutput ? false,
  cudaPackages,
  qt6,
  pkg-config,
  opennurbs,
  highs,
  boost,
  # fmt,
  spdlog,
  #spdmon,
  range-v3,
  pcl,
  embree,
  eigen,
  cgal,
  gtsam,
  # rux NEEDs libmetis.so directly (via GTSAM's ordering API). Without metis in
  # our own buildInputs its lib dir is absent from rux's RUNPATH, so the binary
  # fails at startup with "libmetis.so: cannot open shared object file".
  metis,
  rtabmap,
  librealsense,
  octomap,
  igraph,
  mpfr,
  opencv,
  # glfw,
  blender,
  # python,
  # imgui,
  # glm,
  # libGLU,
  catch2_3,
  libe57format,
  cli11,
  curl,
  nlohmann_json,
  yaml-cpp,
  openssl,
  # Prebuilt libtorch (pkgs/libtorch: 2.9.0+cu128, CUDA 12.x), replacing the
  # from-source build whose pinned pytorch submodules became unfetchable.
  libtorch,
  oneDNN,
  protobuf,
  libpng,
  trtsam3,
  tokenizers-cpp,
  onnxruntime,
  exiv2,
  openmvs,
  nanoflann,
  libjxl,
  cuOpt,
  gsplat-cuda,
  addDriverRunpath,
}: let
  # Individual CUDA redist libraries linked by the build. This is the set
  # LibTorch's Caffe2 CMake config, trtsam3, cuOpt and OpenCV's CUDA modules
  # resolve through FindCUDAToolkit / CUDA::* targets, plus the headers they
  # include. Shared with the dev shell through passthru (see shell.nix).
  cudaLibraries = lib.optionals cudaSupport (with cudaPackages; [
    cuda_cudart
    cccl # <thrust/*>, <cub/*> (CUDA::cccl / torch headers)
    cuda_nvrtc # Torch imported target references CUDA_nvrtc_LIBRARY
    cuda_nvtx
    cuda_profiler_api
    libcublas
    libcufft
    libcurand
    libcusolver
    libcusparse
    cudnn
  ]);

  effectiveStdenv =
    if cudaSupport
    then cudaPackages.backendStdenv
    else stdenv;
  # What the compile reads. Run-time-only test inputs are deliberately left
  # out (testFileset below), for two reasons (reusex-uk8):
  #   - an edit to one of them reruns the tests but does not recompile, and
  #   - tests/fixtures holds the one LFS file in this set. A `github:` flake
  #     ref (how ruxd and the main-branch CI step fetch `.#cpu`) is a GitHub
  #     tarball with LFS *pointers*, a local checkout has the content, so with
  #     fixtures in the build source the two never hashed alike and CI rebuilt
  #     the package nobody had pushed.
  buildFileset = lib.fileset.unions [
    # Build system
    ./CMakeLists.txt
    ./cmake
    # Embedded into the generated version header by reusexLibrary.cmake
    # (file(READ ...LICENSE.md)); the build fails to configure without it.
    ./LICENSE.md
    # C++ library source, headers, and CMake config
    ./libs
    # Applications: the rux CLI (incl. the native Qt client, apps/rux/qt).
    # apps/blender has no CMakeLists.txt and is not part of the C++ build.
    # ruxd (the HTTP service worker) moved to its own repo (repo split, P4).
    ./apps/rux
    # Python bindings (pybind11, BUILD_PYTHON_BINDINGS)
    ./bindings
    # Tests: unit, integration, benchmarks, support (sources only; the
    # fixtures are run-time data, see testFileset)
    (lib.fileset.difference ./tests ./tests/fixtures)
    # Embedded into a generated header by reusexLibrary.cmake (the native
    # TensorRT EngineBuilder's default SAM 3.1 recipe)
    ./python/reusex_sam3/engine-build.json
  ];

  # Everything the test binaries and ctest read at run time, on top of the
  # build source: reusex::test_support::source_dir() points here when
  # passthru.check runs the suite.
  testFileset = lib.fileset.unions [
    buildFileset
    # Binary fixtures (office_corridor.rux is LFS: CI checks out with
    # `lfs: true`, and a local git+file flake reads the smudged work tree)
    ./tests/fixtures
    # Accessed at test runtime via source_dir() (test_stage_contract.cpp)
    ./docs/CONTRACTS.md
    # Read by test_pose_edits.cpp (every doc example must parse + match the schema)
    ./docs/design/pose-edits.md
    ./docs/design/pose-edits.schema.json
    # Run by the bench_solid_compare_selftest ctest (tests/CMakeLists.txt).
    # Missing from this set it failed every nix check with "can't open
    # file" — unnoticed, because ci.yml piped `nix build` into `tee`
    # without pipefail (reusex-37v).
    ./scripts/bench-solid-compare.py
    ./scripts/test_bench_solid_compare.py
    # Run by the bench_poses_compare_selftest ctest (reusex-rcv.1.6),
    # same reason as the bench-solid pair above.
    ./scripts/bench-poses-compare.py
    ./scripts/test_bench_poses_compare.py
  ];
in
  effectiveStdenv.mkDerivation (finalAttrs: {
    pname = "ReUseX";
    version = "0.0.5";

    src = lib.fileset.toSource {
      root = ./.;
      fileset = buildFileset;
    };

    outputs = ["out"] ++ lib.optionals withTestsOutput ["testsuite"];

    # Native dependencies
    # programs and libraries used at build-time
    nativeBuildInputs =
      [
        cmake
        pkg-config
        qt6.qtbase
        # wrapQtAppsNoGuiHook was deprecated in nixpkgs and now just warns and
        # forwards here; wrapQtAppsHook is the supported name.
        qt6.wrapQtAppsHook
        blender.pythonPackages.python # Pin Python version to Blender's (3.11)
      ]
      # CUDA-only build tools:
      # - cuda_nvcc is the compiler, required to compile the .cu sources and for
      #   CMake's `enable_language(CUDA)`. It is deliberately the bare redist
      #   package, not `cudatoolkit`: the merged toolkit's lib/ ends up in
      #   rux's RUNPATH and drags the whole toolkit (nvcc included, ~4 GiB)
      #   into the runtime closure. The libraries actually linked are listed
      #   individually in buildInputs below.
      # - addDriverRunpath ships a helper that patches a binary's RUNPATH to
      #   include /run/opengl-driver/lib. On NixOS the real libcuda.so lives
      #   there, not in any Nix store path — without this any CUDA-using binary
      #   fails at runtime with cudaErrorStubLibrary.
      ++ lib.optionals cudaSupport [
        cudaPackages.cuda_nvcc
        addDriverRunpath
      ];

    buildInputs =
      [
        opennurbs
        highs
        boost

        # fmt
        spdlog
        #spdmon
        range-v3
        libpng

        pcl
        embree
        eigen
        cgal

        # GTSAM — factor-graph optimization for the `rux optimize` pose-graph
        # back-end (plane-landmark factors + GNC). MIT.
        gtsam
        # METIS — pulled in transitively via GTSAM; listed so libmetis.so lands
        # in rux's RUNPATH (see the arg comment above).
        metis

        rtabmap
        librealsense
        octomap

        libe57format

        igraph

        mpfr

        opencv
        cli11
      ]
      ++ lib.optionals withLibtorch [libtorch]
      ++ [
        oneDNN
        protobuf # should be in libtorch?
        onnxruntime

        catch2_3
        tokenizers-cpp
        blender.pythonPackages.pybind11

        curl
        nlohmann_json
        yaml-cpp
        openssl
        exiv2

        # OpenMVS — library-linked Multi-View Stereo for `rux create dense`.
        # AGPL-3.0-or-later; combined work is therefore AGPL.
        openmvs
        # nanoflann is exposed in OpenMVS::Common's link interface; CMake
        # validates it at consumer configure time, so it must be findable.
        nanoflann
        # OpenMVS::IO link interface contains a bare `jxl` library name
        # (rather than an absolute path or imported target), so libjxl
        # must be on the linker search path at consumer link time.
        libjxl
      ]
      # CUDA-only runtime dependencies. trtsam3 is the TensorRT-based SAM
      # backend and cuOpt is the GPU MIP/LP solver — both are inherently CUDA;
      # the vision/solver code paths that use them are gated out by CMake when
      # the backends aren't found.
      ++ lib.optionals cudaSupport (
        [
          trtsam3
          cuOpt
        ]
        ++ lib.optionals withLibtorch [
          # gsplat's CUDA rasterization backend (Apache-2.0), vendored as a
          # standalone LibTorch-linked static library. Backs the `reusex_gsplat`
          # module and `rux create gsplat` (#240). CUDA-only by construction;
          # reusexLibrary.cmake skips the module when it is not found.
          gsplat-cuda
        ]
        # Individual CUDA redist libraries instead of the merged `cudatoolkit`
        # (see nativeBuildInputs and `cudaLibraries` above).
        ++ cudaLibraries
      );

    # Drive the project's WITH_CUDA option from cudaSupport. For CPU and ROCm
    # builds (cudaSupport = false) this keeps CMake from enabling the CUDA
    # language (no nvcc required) and auto-excludes the CUDA-only code paths
    # (TensorRT backend + .cu kernels, cuOpt solver). ROCm-ness itself comes
    # from libtorch/onnxruntime being built under a rocmSupport nixpkgs.
    cmakeFlags =
      [
        (lib.cmakeBool "WITH_CUDA" cudaSupport)
        # The `reusex_package_consumer` ctest installs the build tree into a
        # temp prefix with `cmake --install --prefix`, which the absolute
        # CMAKE_INSTALL_*DIR nixpkgs' cmake hook passes would ignore. Here the
        # same consumer runs against the real $out instead (installCheckPhase).
        (lib.cmakeBool "REUSEX_PACKAGE_TEST" false)
      ]
      # Without libtorch, pin the backend list instead of relying on AUTO: an
      # explicit list makes a missing TensorRT/ONNX dependency a configure
      # error rather than a silently smaller build. This is what AUTO resolves
      # to in the default build, minus LibTorch (OpenVINO is not packaged).
      ++ lib.optionals (!withLibtorch) [
        (lib.cmakeFeature "ML_BACKENDS" (
          if cudaSupport
          then "TensorRT;ONNX"
          else "ONNX"
        ))
      ];

    # Wrapped by hand in postFixup: only `rux` (the Qt client) needs Qt's
    # plugin path; wrapping everything would add a shell hop to every tool.
    dontWrapQtApps = true;

    # The CUDA packages this build compiles against, for shell.nix to merge
    # into one toolkit root. Inside this derivation nixpkgs'
    # setupCUDAToolkitCompilers hook passes -DCUDAToolkit_INCLUDE_DIR/_ROOT
    # lists to CMake, which is what lets the split redist packages satisfy
    # LibTorch's bundled FindCUDAToolkit. A dev shell never runs that hook, so
    # it needs a single directory instead. passthru is not part of the
    # derivation, so this does not touch the package or its closure.
    passthru =
      {
        cudaToolkitPackages =
          lib.optionals cudaSupport ([cudaPackages.cuda_nvcc] ++ cudaLibraries);
      }
      // lib.optionalAttrs withTestsOutput {
        # Runs the compiled suite of this derivation: ctest over the
        # `testsuite` output, then the installed-package consumer against
        # `out` (find_package(ReUseX CONFIG REQUIRED); the CMake package is
        # only proven by consuming it). Compiles nothing of ReUseX — only the
        # few-line consumer. Its output is a pass stamp with no references
        # (allowedReferences), so substituting it fetches a few KiB. This is
        # `checks.tests` in flake.nix (reusex-uk8).
        check = effectiveStdenv.mkDerivation {
          pname = "ReUseX-tests";
          inherit (finalAttrs) version;
          src = lib.fileset.toSource {
            root = ./.;
            fileset = testFileset;
          };
          # The consumer re-runs every find_dependency of ReUseXConfig.cmake,
          # so it needs the same dependency closure on CMAKE_PREFIX_PATH.
          inherit (finalAttrs) nativeBuildInputs buildInputs;
          dontConfigure = true;
          dontBuild = true;
          dontFixup = true;
          dontWrapQtApps = true;
          doCheck = true;
          checkPhase = ''
            runHook preCheck
            # The test binaries were compiled in another sandbox. These two
            # replace the paths baked into them (tests/support/source_dir.cpp,
            # tests/support/rux_executable.hpp).
            export REUSEX_TEST_SOURCE_DIR="$PWD"
            export REUSEX_TEST_RUX_EXECUTABLE="${finalAttrs.finalPackage.out}/bin/rux"
            # A writable copy: ctest writes Testing/ next to the build tree
            # and the tests run in build/tests. The ctest files carry the
            # placeholders postInstall put in place of the build's paths.
            build="$TMPDIR/build"
            cp -r "${finalAttrs.finalPackage.testsuite}/build" "$build"
            chmod -R u+w "$build"
            find "$build" -name '*.cmake' -exec sed -i \
              -e "s|@REUSEX_TEST_BUILD_DIR@|$build|g" \
              -e "s|@REUSEX_TEST_SOURCE_DIR@|$REUSEX_TEST_SOURCE_DIR|g" {} +
            # --parallel: the suite is dominated by per-process loader
            # overhead, so this is a near-linear speedup (#268). Safe since
            # #262 gave temp files pid-unique names.
            ctest --test-dir "$build" --output-on-failure \
              --parallel "''${NIX_BUILD_CORES:-1}" \
              --output-log "$TMPDIR/ctest.log"
            ctest_count=$(ctest --test-dir "$build" -N | sed -n 's/^Total Tests: //p')
            echo "ctest: $ctest_count tests registered"
            test "''${ctest_count:-0}" -gt 0 \
              || { echo "ctest found no tests in the testsuite output" >&2; exit 1; }
            cmake -DINSTALLED_PREFIX=${finalAttrs.finalPackage.out} \
              -DCONSUMER_SOURCE_DIR="$PWD/tests/package/consumer" \
              -DWORK_DIR="$TMPDIR/reusex-package-test" \
              "-DGENERATOR=Unix Makefiles" \
              -DBUILD_CONFIG=Release \
              -P "$PWD/tests/package/run_package_test.cmake"
            runHook postCheck
          '';
          # Store paths are blanked so the stamp references nothing: CI
          # substitutes just this, never the package's closure.
          installPhase = ''
            runHook preInstall
            mkdir -p "$out"
            sed -E 's|/nix/store/[0-9a-z]{32}-|/nix/store/<hash>-|g' \
              "$TMPDIR/ctest.log" > "$out/ctest.log"
            echo "ReUseX-${finalAttrs.version}: ctest ($ctest_count tests) and install check passed" \
              > "$out/passed"
            runHook postInstall
          '';
          allowedReferences = [];
        };
      };

    # The library installs as a CMake package (cmake/Installation.cmake):
    # headers under include/reusex/, the per-module static libraries in lib/,
    # and lib/cmake/ReUseX/ReUseXConfig.cmake, so another derivation (ruxd,
    # after the repo split) can `find_package(ReUseX CONFIG REQUIRED)` against
    # this one. Fail here, not in a downstream build, if that ever stops being
    # installed.
    postInstall =
      ''
        test -f $out/lib/cmake/ReUseX/ReUseXConfig.cmake \
          || { echo "ReUseX CMake package config was not installed" >&2; exit 1; }
      ''
      # The `testsuite` output: what ctest needs to run the suite without
      # this build tree: every CTestTestfile.cmake, the catch_discover_tests
      # *_include/*_tests.cmake lists, and the test executables (static
      # ReUseX, so no build-tree libraries). The ctest files hold absolute
      # paths into this sandbox; they become placeholders here and
      # passthru.check substitutes its own. A leftover path would make
      # ctest look for files that do not exist, so that fails here.
      + lib.optionalString withTestsOutput ''
        build_dir=$(pwd -P)
        source_dir=$(cd "$NIX_BUILD_TOP/$sourceRoot" && pwd -P)
        find . -name CMakeFiles -prune -o -type f \( -name CTestTestfile.cmake \
            -o -name '*_include.cmake' -o -name '*_tests.cmake' \) -print \
          | while read -r f; do install -Dm644 "$f" "$testsuite/build/$f"; done
        for exe in tests/reusex_unit_tests tests/reusex_unit_tests_vision \
                   tests/reusex_integration_tests; do
          if [ -e "$exe" ]; then install -Dm755 "$exe" "$testsuite/build/$exe"; fi
        done
        find "$testsuite/build" -name '*.cmake' -exec sed -i \
          -e "s|$build_dir|@REUSEX_TEST_BUILD_DIR@|g" \
          -e "s|$source_dir|@REUSEX_TEST_SOURCE_DIR@|g" {} +
        if grep -rlE "(^|[\"' ;=(])$NIX_BUILD_TOP/" "$testsuite/build" --include='*.cmake'; then
          echo "testsuite: build paths left in the ctest files above" >&2
          exit 1
        fi
      '';

    # Patch the installed binaries and shared libraries so the dynamic loader
    # finds the host NVIDIA driver (libcuda.so) at /run/opengl-driver/lib.
    # Without this, `nix run .#default` would die with cudaErrorStubLibrary.
    postFixup =
      lib.optionalString cudaSupport ''
        for f in $(find $out/bin $out/lib -type f \
                        \( -executable -o -name '*.so' -o -name '*.so.*' \) \
                        2>/dev/null); do
          if isELF "$f"; then
            addDriverRunpath "$f"
          fi
        done
      ''
      # Plain `rux` is the native Qt client (apps/rux/qt), so the installed
      # binary must find Qt's platform plugins (xcb, wayland) outside the dev
      # shell. dontWrapQtApps above keeps the hook from wrapping every binary
      # (the other tools need no Qt plugins); only rux is wrapped, after the
      # runpath loop so patchelf sees the real ELF, not the wrapper.
      + ''
        wrapQtApp $out/bin/rux
      '';

    meta = with lib; {
      description = "ReUseX: A tool for processing lidar scans with the aim to facilitate reuse in the construction industry";
      license = licenses.gpl3Plus;
      mainProgram = "rux";
      maintainers = with maintainers; [];
    };
  })
