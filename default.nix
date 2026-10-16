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
in
  effectiveStdenv.mkDerivation rec {
    pname = "ReUseX";
    version = "0.0.5";

    src = lib.fileset.toSource {
      root = ./.;
      fileset = lib.fileset.unions [
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
        # Tests: unit, integration, benchmarks, support, and binary fixtures
        ./tests
        # Accessed at test runtime via REUSEX_SOURCE_DIR (test_stage_contract.cpp)
        ./docs/CONTRACTS.md
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
        # Embedded into a generated header by reusexLibrary.cmake (the native
        # TensorRT EngineBuilder's default SAM 3.1 recipe)
        ./python/reusex_sam3/engine-build.json
      ];
    };

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
    passthru.cudaToolkitPackages =
      lib.optionals cudaSupport ([cudaPackages.cuda_nvcc] ++ cudaLibraries);

    # The library installs as a CMake package (cmake/Installation.cmake):
    # headers under include/reusex/, the per-module static libraries in lib/,
    # and lib/cmake/ReUseX/ReUseXConfig.cmake, so another derivation (ruxd,
    # after the repo split) can `find_package(ReUseX CONFIG REQUIRED)` against
    # this one. Fail here, not in a downstream build, if that ever stops being
    # installed.
    postInstall = ''
      test -f $out/lib/cmake/ReUseX/ReUseXConfig.cmake \
        || { echo "ReUseX CMake package config was not installed" >&2; exit 1; }
    '';

    # Build and run tests/package/consumer against the installed $out with
    # find_package(ReUseX CONFIG REQUIRED): the installed CMake package is only
    # proven by consuming it. Off by default (it re-runs the whole dependency
    # lookup, minutes on the CUDA variant); `checks.tests` in flake.nix turns
    # it on for the CPU variant CI builds.
    doInstallCheck = false;
    installCheckPhase = ''
      runHook preInstallCheck
      cmake -DINSTALLED_PREFIX=$out \
        -DCONSUMER_SOURCE_DIR=$NIX_BUILD_TOP/$sourceRoot/tests/package/consumer \
        -DWORK_DIR=$TMPDIR/reusex-package-test \
        "-DGENERATOR=Unix Makefiles" \
        -DBUILD_CONFIG=Release \
        -P $NIX_BUILD_TOP/$sourceRoot/tests/package/run_package_test.cmake
      runHook postInstallCheck
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
  }
