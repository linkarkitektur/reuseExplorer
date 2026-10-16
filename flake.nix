# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  description = "ReUseX";

  # Our overlays rebuild OpenCV/RTABMap/GTSAM/HiGHS/OpenNURBS and we vendor
  # libtorch + tokenizers-cpp, none of which cache.nixos.org can serve. Those
  # paths live in reusex.cachix.org (public, read-only without a token) so
  # neither CI nor a fresh dev machine has to build them from source.
  # See docs/guides/ci-cache.md. Nix asks before honouring these the first
  # time; CI opts in up front via `accept-flake-config = true`.
  nixConfig = {
    extra-substituters = ["https://reusex.cachix.org"];
    extra-trusted-public-keys = [
      "reusex.cachix.org-1:0+y68O2+rBxXytqIepgqOSRsWa+5Ccpst9weS0sUBak="
    ];
  };

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    pyproject-nix = {
      url = "github:nix-community/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";

    pre-commit-hooks = {
      url = "github:cachix/git-hooks.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  }; # end of inputs

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    pre-commit-hooks,
    ...
  }: (flake-utils.lib.eachSystem ["x86_64-linux"] (
    system:
    # flake-utils.lib.eachDefaultSystem (system:
    let
      inherit (nixpkgs) lib;

      # Import nixpkgs with custom configurations and overlays
      pkgs = import nixpkgs {
        inherit system;

        # Set systm comfigurations such as CUDA support and unfree packages
        config = {
          cudaSupport = true;
          hardware.nvidia.open = false;
          allowUnfree = true;
          # Build CUDA code only for the workstation GPU (RTX 6000 Ada = sm_89)
          # instead of the default multi-arch list — much faster builds, smaller
          # closure. This is the only GPU this stack targets.
          cudaCapabilities = ["8.9"];
          # tensorrt (cuda12.9-tensorrt-10.16.1.11, updated by the nixpkgs bump)
          # carries no known vulnerabilities, so no permittedInsecurePackages
          # entry is needed anymore.
        };

        # Set overlays and custom fixes for broken packages
        overlays = import ./overlays {inherit lib;};
      };
    in {
      formatter = pkgs.alejandra;

      checks.pre-commit-check = pre-commit-hooks.lib.${system}.run {
        src = ./.;
        default_stages = ["pre-commit"];
        hooks = {
          check-added-large-files.enable = true;
          check-case-conflicts.enable = true;
          check-executables-have-shebangs.enable = true;
          check-shebang-scripts-are-executable.enable = true;
          check-merge-conflicts.enable = true;
          alejandra.enable = true;
          # Nix static analysis (anti-patterns + dead code). statix runs
          # repo-wide, so skip .direnv (cached flake-input sources).
          statix = {
            enable = true;
            settings.ignore = [".direnv"];
          };
          deadnix.enable = true;
          # C++/CUDA formatting per .clang-format (docs/STANDARDS.md §9).
          clang-format = {
            enable = true;
            types_or = ["c++" "c" "cuda"];
          };
          reuse = {
            enable = true;
          };
          git-lfs-pre-push = {
            enable = true;
            name = "git-lfs pre-push";
            entry = "${pkgs.writeShellScript "git-lfs-pre-push" ''
              exec ${pkgs.git-lfs}/bin/git-lfs pre-push "$PRE_COMMIT_REMOTE_NAME" "$PRE_COMMIT_REMOTE_URL"
            ''}";
            stages = ["pre-push"];
            pass_filenames = false;
            always_run = true;
          };

          git-lfs-post-checkout = {
            enable = true;
            name = "git-lfs post-checkout";
            entry = "${pkgs.writeShellScript "git-lfs-post-checkout" ''
              exec ${pkgs.git-lfs}/bin/git-lfs post-checkout "$PRE_COMMIT_FROM_REF" "$PRE_COMMIT_TO_REF" "$PRE_COMMIT_CHECKOUT_TYPE"
            ''}";
            stages = ["post-checkout"];
            pass_filenames = false;
            always_run = true;
          };

          git-lfs-post-merge = {
            enable = true;
            name = "git-lfs post-merge";
            entry = "${pkgs.writeShellScript "git-lfs-post-merge" ''
              exec ${pkgs.git-lfs}/bin/git-lfs post-merge "$PRE_COMMIT_IS_SQUASH_MERGE"
            ''}";
            stages = ["post-merge"];
            pass_filenames = false;
            always_run = true;
          };

          git-lfs-post-commit = {
            enable = true;
            name = "git-lfs post-commit";
            entry = "${pkgs.git-lfs}/bin/git-lfs post-commit";
            stages = ["post-commit"];
            pass_filenames = false;
            always_run = true;
          };
        };
      };

      # Hermetic build-and-test gate: builds the CPU variant with the unit
      # tests enabled and runs ctest inside the sandbox. The CPU variant is
      # used so the check needs no GPU (works in CI and on any machine).
      # Run with: nix build .#checks.x86_64-linux.tests  (or nix flake check)
      # For the fast incremental dev loop use scripts/check.sh instead.
      #
      # The output is a pass stamp, not the package (reusex-37v): once the
      # build, ctest and the install check have passed, $out is replaced by
      # the ctest log. Its existence in a store means "this exact derivation
      # passed". scripts/ci-local.sh builds it locally and pushes it to
      # reusex.cachix.org; CI on the same commit then substitutes the stamp
      # (a few KiB, no runtime closure) and compiles nothing. Keeping the
      # installed package out of the output is what makes pushing one per PR
      # affordable on the 5 GB Cachix tier. The package itself is .#cpu.
      checks.tests = self.packages.${system}.cpu.overrideAttrs (old: {
        pname = old.pname + "-tests";
        doCheck = true;
        # Builds tests/package/consumer against the installed package
        # (find_package(ReUseX)); see installCheckPhase in default.nix.
        doInstallCheck = true;
        checkPhase = ''
          runHook preCheck
          # --parallel: the suite is dominated by per-process loader
          # overhead, so this is a near-linear speedup (#268). Safe since
          # #262 gave temp files pid-unique names.
          ctest --output-on-failure --parallel "''${NIX_BUILD_CORES:-1}" \
            --output-log "$TMPDIR/ctest.log"
          runHook postCheck
        '';
        # Last hook of the last phase that needs the installed package.
        postInstallCheck = ''
          rm -rf "$out"
          mkdir -p "$out"
          cp "$TMPDIR/ctest.log" "$out/ctest.log"
          echo "${old.pname}-${old.version}: build, ctest and install check passed" \
            > "$out/passed"
        '';
      });

      packages = let
        # Get all custom packages
        allPackages = pkgs.lib.packagesFromDirectoryRecursive {
          callPackage = pkgs.lib.callPackageWith pkgs;
          directory = ./pkgs;
        };
        # Filter out broken packages from exports
        nonBrokenPackages = lib.filterAttrs (_name: pkg: !(pkg.meta.broken or false)) allPackages;

        # Heavy custom packages that compile through the plain stdenv (i.e. are
        # NOT covered by the ccache'd cudaPackages.backendStdenv from
        # overlays/ccache.nix) get their compiler wrapped in ccache here.
        # Prebuilt (libtorch) and Rust (tokenizers-cpp) packages are excluded:
        # ccache does not help them.
        ccachedPackageNames = ["gtsam" "trtsam3" "opennurbs"];
        ccachedPackages =
          nonBrokenPackages
          // lib.genAttrs
          (lib.filter (n: nonBrokenPackages ? ${n}) ccachedPackageNames)
          (n: nonBrokenPackages.${n}.override {stdenv = pkgs.ccacheStdenv;});

        # ReUseX build variants. The default/CUDA build reuses the top-level
        # (CUDA-configured) nixpkgs; the cpu and rocm builds use a fresh
        # nixpkgs with the matching GPU config. cudaSupport drives the
        # WITH_CUDA CMake option, which gates the CUDA language, the TensorRT
        # backend (+ its .cu kernels) and the cuOpt solver.
        reusex = pkgs.callPackage ./default.nix {}; # CUDA (default)

        # Opt-in torch-free CUDA build (see withLibtorch in default.nix).
        reusexCudaNoTorch = pkgs.callPackage ./default.nix {withLibtorch = false;};

        mkReusex = {
          cudaSupport ? false,
          rocmSupport ? false,
          withLibtorch ? true,
        }: let
          variantPkgs = import nixpkgs {
            inherit system;
            config = {
              inherit cudaSupport rocmSupport;
              allowUnfree = true;
            };
            overlays = import ./overlays {inherit lib;};
          };
        in
          variantPkgs.callPackage ./default.nix {
            inherit cudaSupport withLibtorch;
            # The CUDA path already gets ccache via cudaPackages.backendStdenv
            # (overlays/ccache.nix); wrap the plain stdenv so the CPU/ROCm
            # ReUseX compiles are cached too.
            stdenv = variantPkgs.ccacheStdenv;
          };

        reusexCpu = mkReusex {};
        reusexRocm = mkReusex {rocmSupport = true;};
        reusexCpuNoTorch = mkReusex {withLibtorch = false;};
      in
        {
          # ReUseX build variants (each provides bin/rux, plus the
          # ReUseX CMake package other repos — e.g. ruxd — build against).
          default = reusex; # CUDA / NVIDIA GPU
          cuda = reusex;
          cpu = reusexCpu;
          rocm = reusexRocm;
          # Opt-in variants without libtorch: no LibTorch ML backend (YOLO
          # .pt) and no `rux create gsplat`. TensorRT/ONNX (CUDA) or ONNX
          # (CPU) inference is unaffected.
          cuda-notorch = reusexCudaNoTorch;
          cpu-notorch = reusexCpuNoTorch;

          inherit (pkgs) rtabmap;
        }
        # All custom packages (excluding broken ones), heavy ones ccache-wrapped
        // ccachedPackages; # end of packages

      devShells =
        {
          default = import ./shell.nix {
            inherit pkgs self system;
          };
        }
        // (
          let
            devshellFiles = builtins.readDir ./devshells;
            validShell = name: devshellFiles.${name} == "regular" && lib.hasSuffix ".nix" name;
            shellNames = builtins.filter validShell (builtins.attrNames devshellFiles);
            toShellAttr = name: {
              name = lib.removeSuffix ".nix" name;
              value = import (./devshells + "/${name}") {
                inherit pkgs self system;
              };
            };
          in
            builtins.listToAttrs (builtins.map toShellAttr shellNames)
        ); # end of devShells
    }
  )); # end of outputs
}
