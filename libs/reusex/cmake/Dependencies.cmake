# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Find all project dependencies
# ===============================================

message(STATUS "Finding project dependencies...")

# -----------------------------------------------
# Core dependencies
# -----------------------------------------------
find_package(Qt6 COMPONENTS Core Widgets Gui OpenGL REQUIRED)
find_package(Boost CONFIG REQUIRED)
find_package(TBB REQUIRED)

# Logging and formatting
find_package(fmt CONFIG REQUIRED)

find_package(range-v3 CONFIG REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

# -----------------------------------------------
# Protobuf dependencies (for ML backends)
# -----------------------------------------------
# Both TensorRT (via tokenizers_cpp) and LibTorch depend on protobuf.
# Protobuf requires abseil and utf8_range to load successfully.
# Finding all three explicitly here prevents target redefinition errors when
# multiple backends try to find protobuf independently.
#
# Root cause: tokenizers_cpp calls find_package(Protobuf QUIET), creating
# partial targets. When LibTorch calls find_package(Protobuf CONFIG QUIET),
# CMake detects some targets exist but not all, triggering an error.
#
# Solution: Find protobuf once with all dependencies satisfied.
find_package(absl CONFIG REQUIRED)
find_package(utf8_range CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
# tokenizers_cpp's config then calls find_package(Protobuf) in MODULE mode, and
# FindProtobuf's find_path(Protobuf_INCLUDE_DIR) hits the prebuilt libtorch's
# include/ first -- it ships protobuf 3.13.0's headers for its own internal use.
# The headers then disagreed with the one libprotobuf/protoc in the closure
# (the protobuf package found above) and FindProtobuf warned "Protobuf compiler
# version X doesn't match library version 3.13.0". Pin the module-mode lookup
# to the headers of the protobuf found above, so both modes describe the same
# package. FORCE so an existing build dir drops the libtorch path it cached.
get_target_property(_reusex_protobuf_includes protobuf::libprotobuf
                    INTERFACE_INCLUDE_DIRECTORIES)
list(GET _reusex_protobuf_includes 0 _reusex_protobuf_include)
set(Protobuf_INCLUDE_DIR "${_reusex_protobuf_include}" CACHE PATH
    "Protobuf headers, pinned to the CONFIG-mode protobuf package" FORCE)
unset(_reusex_protobuf_includes)
unset(_reusex_protobuf_include)

# -----------------------------------------------
# ML Backends (Optional)
# -----------------------------------------------
# User can specify which backends to enable via:
#   -DML_BACKENDS="AUTO" (default - find all available)
#   -DML_BACKENDS="TensorRT;LibTorch" (explicit list)
#   -DML_BACKENDS="NONE" (disable all)

set(ML_BACKENDS "AUTO" CACHE STRING "ML backends to enable (AUTO, NONE, or list: TensorRT;LibTorch;ONNX;OpenVINO)")

# Define available backends and their dependencies
set(ML_BACKEND_TensorRT_PACKAGES "trtsam3;tokenizers_cpp")
set(ML_BACKEND_LibTorch_PACKAGES "Torch 2.9.0")
set(ML_BACKEND_ONNX_PACKAGES "onnxruntime;tokenizers_cpp")
set(ML_BACKEND_OpenVINO_PACKAGES "openvino")  # Future

# -----------------------------------------------
# Workaround: OpenCV 4.13.0 + LibTorch legacy FindCUDA cache pollution
# -----------------------------------------------
# LibTorch's Caffe2Config.cmake calls a bundled legacy find_package(CUDA) which
# caches CUDA_FOUND and CUDA_VERSION. On reconfiguration, when trtsam3 triggers
# find_dependency(OpenCV) before LibTorch is processed, OpenCV sees the cached
# CUDA_FOUND=TRUE and calls the legacy find_cuda_helper_libs() macro -- which
# does not exist yet (macros are not cached). Two mitigations:
#
# 1. Unset legacy FindCUDA cache variables so OpenCV uses modern CUDAToolkit.
# 2. Define a find_cuda_helper_libs shim as a safety net.

# Mitigation 1: Clear stale legacy FindCUDA cache entries
unset(CUDA_FOUND CACHE)
unset(CUDA_VERSION CACHE)
unset(CUDA_VERSION_STRING CACHE)

# Mitigation 2: Provide find_cuda_helper_libs if not already defined
if(NOT COMMAND find_cuda_helper_libs)
  macro(find_cuda_helper_libs _name)
    find_library(CUDA_${_name}_LIBRARY
      NAMES ${_name}
      PATHS "${CUDAToolkit_LIBRARY_DIR}" "${CUDA_TOOLKIT_ROOT_DIR}"
      PATH_SUFFIXES lib64 lib/x64 lib
      NO_DEFAULT_PATH
    )
    mark_as_advanced(CUDA_${_name}_LIBRARY)
  endmacro()
endif()

# Determine which backends to search for
if(ML_BACKENDS STREQUAL "AUTO")
    set(BACKENDS_TO_FIND TensorRT LibTorch ONNX OpenVINO)
    # TensorRT is NVIDIA/CUDA-only; don't search for it in a non-CUDA build.
    if(NOT WITH_CUDA)
        list(REMOVE_ITEM BACKENDS_TO_FIND TensorRT)
    endif()
elseif(ML_BACKENDS STREQUAL "NONE")
    set(BACKENDS_TO_FIND "")
    message(STATUS "All ML backends disabled")
else()
    set(BACKENDS_TO_FIND ${ML_BACKENDS})
endif()

# Find each backend
set(ENABLED_ML_BACKENDS "")
foreach(backend IN LISTS BACKENDS_TO_FIND)
    message(STATUS "Searching for ${backend} backend...")
    set(all_found TRUE)
    foreach(pkg IN LISTS ML_BACKEND_${backend}_PACKAGES)
        # Parse package name and version (e.g., "Torch 2.9.0" -> "Torch" + "2.9.0")
        string(REPLACE " " ";" pkg_parts ${pkg})
        list(GET pkg_parts 0 pkg_name)
        list(LENGTH pkg_parts pkg_parts_len)
        # Caffe2's public/cuda.cmake (inside TorchConfig) warns whenever
        # CMAKE_CUDA_ARCHITECTURES is defined, then overwrites it with OFF and
        # appends -gencode flags for TORCH_CUDA_ARCH_LIST to CMAKE_CUDA_FLAGS.
        # CUDAOptions.cmake sets both to the same arch set, so hide ours while
        # Torch loads and then do what Caffe2 would have done: OFF if Torch
        # supplied the -gencode flags, our list back if Torch was not found.
        if(pkg_name STREQUAL "Torch" AND DEFINED CMAKE_CUDA_ARCHITECTURES)
            set(_reusex_cuda_archs "${CMAKE_CUDA_ARCHITECTURES}")
            unset(CMAKE_CUDA_ARCHITECTURES)
        endif()
        if(pkg_parts_len GREATER 1)
            list(GET pkg_parts 1 pkg_version)
            find_package(${pkg_name} ${pkg_version} CONFIG QUIET)
        else()
            find_package(${pkg_name} CONFIG QUIET)
        endif()
        if(DEFINED _reusex_cuda_archs)
            if(${pkg_name}_FOUND)
                set(CMAKE_CUDA_ARCHITECTURES OFF)
            else()
                set(CMAKE_CUDA_ARCHITECTURES "${_reusex_cuda_archs}")
            endif()
            unset(_reusex_cuda_archs)
        endif()

        if(NOT ${pkg_name}_FOUND)
            set(all_found FALSE)
            if(NOT ML_BACKENDS STREQUAL "AUTO")
                message(FATAL_ERROR "${backend} backend requested but ${pkg_name} not found")
            endif()
            break()
        endif()
    endforeach()

    if(all_found)
        list(APPEND ENABLED_ML_BACKENDS ${backend})
        message(STATUS "ML Backend ${backend}: ENABLED")
    else()
        message(STATUS "ML Backend ${backend}: DISABLED (dependencies not found)")
    endif()
endforeach()

# At least one backend must be enabled (unless explicitly NONE)
if(NOT ML_BACKENDS STREQUAL "NONE" AND NOT ENABLED_ML_BACKENDS)
    message(FATAL_ERROR "No ML backends available. Install at least one: LibTorch, TensorRT, ONNX Runtime, or OpenVINO")
endif()

# The native EngineBuilder (libs/reusex/src/vision/tensor_rt/common/EngineBuilder.cpp)
# builds SAM3 engines from ONNX on-device, so it needs the TensorRT ONNX parser
# (libnvonnxparser + NvOnnxParser.h). trtsam3 propagates cudaPackages.tensorrt,
# which puts these in the closure, but its CMake target only links libnvinfer —
# so locate the parser lib/header explicitly here and hand them to
# configure_ml_backends() via cache variables.
if("TensorRT" IN_LIST ENABLED_ML_BACKENDS)
    find_library(REUSEX_TRT_ONNXPARSER_LIB nvonnxparser)
    find_path(REUSEX_TRT_ONNXPARSER_INCLUDE NvOnnxParser.h)
    if(NOT REUSEX_TRT_ONNXPARSER_LIB OR NOT REUSEX_TRT_ONNXPARSER_INCLUDE)
        message(FATAL_ERROR
            "TensorRT backend enabled but libnvonnxparser / NvOnnxParser.h were "
            "not found. They ship with TensorRT (cudaPackages.tensorrt, "
            "propagated by trtsam3). lib=${REUSEX_TRT_ONNXPARSER_LIB} "
            "include=${REUSEX_TRT_ONNXPARSER_INCLUDE}")
    endif()
    message(STATUS "TensorRT ONNX parser: ${REUSEX_TRT_ONNXPARSER_LIB}")
endif()

# Export for use in library CMakeLists and root summary
set(ENABLED_ML_BACKENDS ${ENABLED_ML_BACKENDS} PARENT_SCOPE)

# -----------------------------------------------
# Computer Vision & Point Cloud Processing
# -----------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui stitching)
find_package(RTABMap REQUIRED)
find_package(PCL REQUIRED)

# VTK — the rendering backend behind reusex_visualize's headless render_view()
# (#294). Already in the dependency closure via pcl_visualization; named here
# explicitly because the visualize module uses the VTK API directly and needs
# vtk_module_autoinit() to register the OpenGL2 object factory.
#
# Headless rendering relies on this VTK being built with VTK_OPENGL_HAS_EGL
# (it is, in the flake): when no X/Wayland session is reachable VTK falls back
# from vtkXOpenGLRenderWindow to vtkEGLRenderWindow and renders on the GPU
# anyway, which is what makes `env -u DISPLAY rux render` work.
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    CommonExecutionModel
    RenderingCore
    RenderingOpenGL2
    # vtkVectorText: stroke-font geometry for the plan overlay's labels.
    RenderingFreeType)

# -----------------------------------------------
# Geometry & Optimization
# -----------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(highs REQUIRED)
if(WITH_CUDA)
    find_package(cuOpt)  # Optional - requires NVIDIA GPU; else HiGHS CPU solver
endif()
find_package(embree REQUIRED)

# GTSAM — factor-graph optimization for the `rux optimize` pose-graph back-end
# (OrientedPlane3 plane-landmark factors + GncOptimizer). MIT.
find_package(GTSAM REQUIRED)

# CGAL with support modules
find_package(CGAL REQUIRED Core)
include(CGAL_Eigen3_support)
include(CGAL_TBB_support)

# OpenMVS — library-linked Multi-View Stereo. Exposes OpenMVS::MVS plus
# transitive OpenMVS::{Common,Math,IO}. AGPL-3.0-or-later — combined work
# is therefore AGPL.
#
# The OpenMVSTargets.cmake config eagerly validates every imported target's
# INTERFACE_LINK_LIBRARIES, so we must materialise the named targets it
# references before find_package(OpenMVS): Boost::program_options (used by
# OpenMVS::Common's CLI helpers) and JsonCpp::JsonCpp (pulled in via
# VTK::jsoncpp from the opencv_viz dependency).
find_package(Boost REQUIRED COMPONENTS iostreams program_options serialization)
find_package(jsoncpp REQUIRED)
find_package(nanoflann REQUIRED)
find_package(OpenMVS REQUIRED)

# -----------------------------------------------
# Graph Algorithms
# -----------------------------------------------
find_package(igraph REQUIRED)

# -----------------------------------------------
# I/O Formats
# -----------------------------------------------
find_package(E57Format REQUIRED)
find_package(opennurbs REQUIRED)

# EXIF metadata reading (for 360 panoramic image import)
find_package(exiv2 REQUIRED CONFIG)

# -----------------------------------------------
# Speckle upload support (HTTP, JSON, hashing)
# -----------------------------------------------
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# -----------------------------------------------
# Optional GUI support
# -----------------------------------------------
option(GUI_ENABLED "if true, enables GUI visualization features" OFF)
if (GUI_ENABLED)
    message(STATUS "GUI support enabled")
    message(STATUS "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
    message(STATUS "CGAL_GRAPHICSVIEW_PACKAGE_DIR: ${CGAL_GRAPHICSVIEW_PACKAGE_DIR}")
    message(STATUS "CGAL_MODULES_DIR: ${CGAL_MODULES_DIR}")

    find_package(CGAL COMPONENTS Qt6)

    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)
    add_definitions(-DCGAL_USE_BASIC_VIEWER)
else()
    message(STATUS "GUI support disabled")
endif()

message(STATUS "All dependencies found successfully")
