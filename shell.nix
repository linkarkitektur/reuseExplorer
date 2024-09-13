let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-24.05";

  # nixpkgs overlay for OpenCV contrib
  overlay-for-opencv-contrib = self: super: {
    opencv = super.opencv.override {
      enableContrib = true;
      enableFfmpeg = true;
      enableGtk3 = true;
      enableIpp = true;
      enableUnfree = true;
      enableCuda = true;
      enableCudnn = true;
    };
  };

  # inxpkgs overlay for PCL with cuda
  overlay-for-pcl-cuda = self: super: {
    pcl = super.pcl.override {
      cudaSupport = true;
    };
  };



  # Fetch the specific version of nixpkgs you need
  oldNixpkgs = import (fetchTarball {
    url = "https://github.com/NixOS/nixpkgs/archive/27ba408bba01511329810cf02697c26f86bf0342.zip";
  }) {};
  
  embree3 = oldNixpkgs.embree;

  pkgs = import nixpkgs { config = { allowUnfree = true; }; overlays = [ overlay-for-opencv-contrib overlay-for-pcl-cuda]; };


in
  # Define the environment using stdenv.mkDerivation
  pkgs.stdenv.mkDerivation {
    name = "reuseExplorer-env";

    # Add dependencies here
    buildInputs = [
      pkgs.gcc              # C++ compiler
      pkgs.cmake            # CMake build tool (if using CMake)
      pkgs.boost            # Boost library (example dependency)
      pkgs.pkg-config       # Pkg-config for library detection

      # Project dependencies
      pkgs.pcl
      pkgs.boost
      pkgs.flann
      pkgs.vtk
      pkgs.qhull

      pkgs.opencv
      pkgs.cgal
      embree3
      pkgs.eigen
      pkgs.gurobi
      pkgs.scip
      pkgs.tbb
      pkgs.python312Packages.pybind11
      pkgs.cudaPackages.cudnn_8_9
      pkgs.cudaPackages.cudatoolkit
      pkgs.mpi
      pkgs.mpfr
      pkgs.libusb1
    ];

    shellHook = ''
     echo "This is the reuseExplorer dev shell!"
    '';
  }

## Expose the derivation to nix-shell
#pkgs.mkShell {
#  buildInputs = [ reuseExplorerEnv ];
#
#  # Set up environment variables if necessary
#  shellHook = ''
#    echo "This is the reuseExplorer dev shell!"
#  '';
#}

