# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}: let
  # One merged CUDA root for the dev shell (cudaSupport only).
  #
  # The package build compiles against the individual CUDA redist packages
  # (cf6d7fd9, which keeps nvcc and the merged toolkit out of rux's runtime
  # closure). That works inside the derivation only because nixpkgs'
  # setupCUDAToolkitCompilers hook runs in configurePhase and hands CMake
  # -DCUDAToolkit_INCLUDE_DIR / -DCUDAToolkit_ROOT lists spanning every
  # package. A dev shell never runs configurePhase, so a plain cmake (or the
  # `configure` helper below) sees only the bare cuda_nvcc. CMake's own
  # FindCUDAToolkit copes, but LibTorch's bundled copy
  # (Caffe2/FindCUDAToolkit.cmake) takes the toolkit root from the CUDA
  # compiler and needs <root>/include/cuda_runtime.h and cublas_v2.h, so it
  # fails with "Could NOT find CUDAToolkit (missing: CUDAToolkit_INCLUDE_DIR)".
  #
  # Merging the package's own CUDA inputs (passthru.cudaToolkitPackages, same
  # store paths, every output) into one directory and pointing the compiler
  # at it gives that finder the layout it expects. This exists only in the
  # shell: the package derivation and its runtime closure are unchanged.
  reusex = self.packages.${system}.default;
  cudaDevToolkit = pkgs.symlinkJoin {
    name = "reusex-devshell-cuda-${pkgs.cudaPackages.cudaMajorMinorVersion}";
    paths = pkgs.lib.concatMap (p: map (o: p.${o}) p.outputs) reusex.cudaToolkitPackages;
  };
  cudaEnv = pkgs.lib.optionalAttrs (reusex.cudaToolkitPackages != []) {
    # CUDACXX seeds CMAKE_CUDA_COMPILER on a fresh configure; the toolkit root
    # CMake derives from it is what Caffe2's FindCUDAToolkit uses.
    CUDACXX = "${cudaDevToolkit}/bin/nvcc";
    # Read by CMake's FindCUDAToolkit (CMP0074) and by FindCUDA, which
    # LibTorch also calls; they would otherwise resolve to the nvcc on PATH.
    CUDAToolkit_ROOT = "${cudaDevToolkit}";
    CUDA_PATH = "${cudaDevToolkit}";
  };

  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │         ReUseX  development shell           │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  configure [Release|Debug]   cmake + compile_commands.json"
    echo "  build                       cmake --build --parallel"
    echo "  run-tests                   build, then ctest"
    echo "  clean                       remove build/"
    echo "  format                      clang-format all C++ sources"
    echo "  lint                        cppcheck static analysis"
    echo "  docs                        build Doxygen docs"
    echo "  menu                        show this menu"
    echo ""
  '';

  # Convenience scripts available in any shell (bash, fish, zsh).
  scripts = pkgs.lib.attrValues {
    menu = pkgs.writeShellScriptBin "menu" motd;
    configure = pkgs.writeShellScriptBin "configure" ''
      cmake -B "$PWD/build" -GNinja \
        -DCMAKE_BUILD_TYPE=''${1:-Release} \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        "''${@:2}"
      ln -sf "$PWD/build/compile_commands.json" "$PWD/compile_commands.json"
    '';
    build = pkgs.writeShellScriptBin "build" ''
      cmake --build "$PWD/build" --parallel "$@"
    '';
    run-tests = pkgs.writeShellScriptBin "run-tests" ''
      cmake --build "$PWD/build" --parallel \
        && ctest --test-dir "$PWD/build" --output-on-failure --parallel "$@"
    '';
    clean = pkgs.writeShellScriptBin "clean" ''
      rm -rf "$PWD/build" && echo "Build directory removed."
    '';
    format = pkgs.writeShellScriptBin "format" ''
      find "$PWD/libs" "$PWD/apps" \( -name '*.cpp' -o -name '*.hpp' \) \
        | xargs clang-format -i && echo "Done."
    '';
    lint = pkgs.writeShellScriptBin "lint" ''
      cppcheck --enable=all --suppress=missingIncludeSystem \
        --suppress=unmatchedSuppression --inline-suppr \
        --check-level=exhaustive \
        -I "$PWD/libs/reusex/include" \
        -I "$PWD/build/include" \
        -I "$PWD/build/generated" \
        -I "$PWD/apps/rux/include" \
        "$PWD/libs/" "$PWD/apps/"
    '';
    docs = pkgs.writeShellScriptBin "docs" ''
      cmake --build "$PWD/build" --target doc "$@"
    '';
  };
in
  pkgs.mkShell {
    inputsFrom = [reusex];
    env = cudaEnv;
    buildInputs = self.checks.${system}.pre-commit-check.enabledPackages;

    packages =
      scripts
      ++ (with pkgs; [
        # Python 3.11 (matches Blender) - must come first to take precedence
        blender.pythonPackages.python

        # Documentation tools
        help2man # For generating man pages from --help output
        pandoc
        sphinx
        graphviz # For Doxygen diagrams and visualization

        # Debugging and analysis tools
        gdb
        valgrind
        kdePackages.kcachegrind
        heaptrack # Memory profiler with GUI

        # Build tools
        cmake-format # Format CMakeLists.txt files
        ccache # Cache C++ compilation to speed up rebuilds
        ninja # Faster build system alternative to Make
        bear # Generate compile_commands.json for LSP/clangd

        # C++ development tools
        clang-tools # Includes clang-format, clang-tidy, clang-rename
        cppcheck # Static analysis for C++
        include-what-you-use # Check #include dependencies

        # Performance profiling
        perf # Performance profiling
        hotspot # GUI for perf data visualization

        # Development tools
        libnotify # Send notification when build finishes
        sqlite
        ffmpeg
        openusd
        jq # JSON processor for scripts
        ripgrep # Fast code search (faster than grep)
        fd # Fast file finder (faster than find)
        hyperfine # Command-line benchmarking

        # Version control tools
        tig # Text-mode interface for git
        git-filter-repo # Advanced git history rewriting
        gitui # Terminal UI for git (alternative)
        beads

        #qt6.full
        #qtcreator

        # PDF generation (core::report_generator, Ressourcekortlægning #456;
        # `ruxd`, in its own repo, calls it at runtime too) — the
        # ReportPdf_*_Compiles tests SKIP when typst is not on PATH.
        typst

        # DevOps tools
        nix-update
        sqlitebrowser
        hugin
        github-copilot-cli
        claude-code
        gh
        doxygen
        tree
        git-lfs

        # Code coverage
        lcov
        gcovr # Alternative coverage report generator

        # Python development
        python3Packages.black # Python code formatter
        python3Packages.pytest # Testing framework
        python3Packages.mypy # Type checking
      ]);

    shellHook =
      ''
        export VIRTUAL_ENV_PROMPT="ReUseX"

        # Run ctest in parallel by default, even for a bare `ctest` typed in
        # build/ (#268). Almost all of this suite's wall time is per-process
        # dynamic-loader overhead rather than test work — one process per
        # TEST_CASE, ~360 of them — so it scales nearly linearly with cores:
        # ~12 min serial vs ~75 s at -j8. An explicit `--parallel N` on the
        # command line still wins over this.
        export CTEST_PARALLEL_LEVEL="''${CTEST_PARALLEL_LEVEL:-$(nproc)}"

        # NixOS keeps the real libcuda.so outside Nix, at /run/opengl-driver/lib.
        # Binaries built in this shell (./build/apps/rux/rux, ctest binaries) are
        # not wrapped with addDriverRunpath, so without this they load the stub
        # libcuda.so from cuda_cudart and crash with cudaErrorStubLibrary.
        if [ -d /run/opengl-driver/lib ]; then
          export LD_LIBRARY_PATH="/run/opengl-driver/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        fi

        ${motd}
      ''
      + self.checks.${system}.pre-commit-check.shellHook
      + ''

        # Force core.hooksPath to an ABSOLUTE path (#317). pre-commit-hooks.nix's
        # installationScript (above) stores a path relative to the toplevel
        # working directory, which is correct for the main checkout but wrong
        # for a linked worktree: `.git` there is a FILE (gitdir pointer), not
        # a directory, so the relative path resolves to nothing and git
        # silently runs no hooks. Re-pin it to the absolute git-common-dir on
        # every shell entry; idempotent, silent on success.
        if command -v git >/dev/null 2>&1 && git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
          git config --local core.hooksPath "$(git rev-parse --path-format=absolute --git-common-dir)/hooks"
        fi

        # beads: inject bd's hook sections into the hooks git-hooks.nix just
        # (re)installed, above pre-commit's final `exec`, so both run. The pin
        # above already undoes a core.hooksPath=.beads/hooks left by `bd init`.
        # Idempotent and quiet (CLAUDE.md "Beads: git hooks").
        if command -v bd >/dev/null 2>&1 \
          && _bd_wt=$(git rev-parse --show-toplevel 2>/dev/null) \
          && [ -d "$_bd_wt/.beads" ]; then
          bd hooks install >/dev/null 2>&1 \
            || echo "beads: 'bd hooks install' failed; run it by hand" >&2
        fi
        unset _bd_wt
      '';
  }
