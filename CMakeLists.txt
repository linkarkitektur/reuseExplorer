cmake_minimum_required(VERSION 3.8)
project(linkml)

set(LIB_PROJECT_NAME "${PROJECT_NAME}_lib")
set(PYBIND_PROJECT_NAME "${PROJECT_NAME}_py")

# ===============================================
# global settings


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# ===============================================
# options

option(LIN_ENABLE_ASAN "if true, enables clang/MSVC address sanitizer" OFF)
option(LIN_ENABLE_MSAN "if true, enables clang/MSVC memory sanitizer" OFF)
option(LIN_ENABLE_UBSAN "if true, enables clang/MSVC undefined behaviour sanitizer" OFF)
option(LIN_ENABLE_TSAN "if true, enables clang/MSVC thread sanitizer" OFF)
option(OPENMP, "if true, enables ")

if (LIN_ENABLE_ASAN AND LIN_ENABLE_TSAN)
    message(FATAL_ERROR "Can only enable one of TSan or ASan at a time")
endif()
if (LIN_ENABLE_ASAN AND LIN_ENABLE_MSAN)
    message(FATAL_ERROR "Can only enable one of ASan or MSan at a time")
endif()

option(LIN_ENABLE_WERROR "if true, enables -Werror, /WX" OFF)


# ===============================================
# compiler and linker flags

set(COMMON_COMPILER_FLAGS "")
set(COMMON_LINKER_FLAGS "")

if (MSVC)
    list(APPEND COMMON_COMPILER_FLAGS /MP)
    list(APPEND COMMON_COMPILER_FLAGS /openmp)

    if (LIN_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS /WX)
    endif()
else()
    list(APPEND COMMON_COMPILER_FLAGS -Wall -Wextra)

    # Add support for OpenMP
    list(APPEND COMMON_COMPILER_FLAGS -fopenmp -lgomp)

    if (LIN_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS -Werror)
    endif()

    if (LIN_ENABLE_ASAN OR LIN_ENABLE_TSAN OR LIN_ENABLE_MSAN OR LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fno-omit-frame-pointer -g)
        list(APPEND COMMON_LINKER_FLAGS -fno-omit-frame-pointer -g)
    endif()

    if (LIN_ENABLE_ASAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=address)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=address)
    endif()

    if (LIN_ENABLE_TSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=thread)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=thread)
    endif()

    if (LIN_ENABLE_MSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=memory)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=memory)
    endif()

    if (LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
        list(APPEND COMMON_LINKER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
    endif()
endif()

# ===============================================
# Bin dir
if(MSVC)
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin)
elseif(CMAKE_BUILD_TYPE STREQUAL "")
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/Default)
else()
    set(BIN_DIR ${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE})
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${BIN_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BIN_DIR})
#set(GLOW_BIN_DIR ${CMAKE_SOURCE_DIR}/bin)

## ===============================================
## disable glfw additionals
#option(GLFW_BUILD_EXAMPLES  OFF)
#option(GLFW_BUILD_TESTS  OFF)
#option(GLFW_BUILD_DOCS  OFF)
#option(GLFW_INSTALL  OFF)



# set(ENV{CUDACXX} "/usr/local/cuda/bin/nvcc")

# ===============================================s
# add submodules
add_subdirectory(extern/clean-core)
add_subdirectory(extern/typed-geometry)
add_subdirectory(extern/ctracer)
add_subdirectory(extern/reflector)
add_subdirectory(extern/task-dispatcher)
add_subdirectory(extern/rich-log)
add_subdirectory(extern/polymesh)
add_subdirectory(extern/nanoflann)
add_subdirectory(extern/pybind11)
add_subdirectory(extern/polyscope)
add_subdirectory(extern/glpk)
add_subdirectory(extern/lpsolve)
# add_subdirectory(extern/scip)
add_subdirectory(extern/soplex)
# add_subdirectory(extern/cgal)
# add_subdirectory(extern/pcl)





#add_subdirectory(extern/glfw)
#add_subdirectory(extern/glow)
#add_subdirectory(extern/imgui)
#add_subdirectory(extern/glow-extras)

find_package( OpenCV REQUIRED )

# CGAL and its components
find_package(CGAL REQUIRED)

# Boost and its components
find_package(Boost REQUIRED)
if(NOT Boost_FOUND)

  message(
    STATUS
      "NOTICE: This project requires the Boost library, and will not be compiled."
  )

  return()

endif()
find_package(Eigen3 3.1.0) #(requires 3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
  message(
    STATUS
      "NOTICE: This project requires Eigen 3.1 (or greater) and will not be compiled."
  )
  return()
endif()

find_package(SCIP REQUIRED)
include(CGAL_SCIP_support)



# ===============================================
# configure executable
file(GLOB_RECURSE LIB_SOURCES
    "src/*.cc"
    "src/*.cpp"
)

file(GLOB_RECURSE LIB_HEADERS
    "src/*.hh"
    "src/*.h"
)

list(FILTER LIB_SOURCES EXCLUDE REGEX ".*py.cpp")

# group sources according to folder structure
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_SOURCES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${LIB_HEADERS})



## ===============================================
## add executable
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(${LIB_PROJECT_NAME} STATIC ${LIB_SOURCES} ${LIB_HEADERS})
set_target_properties(${LIB_PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(${LIB_PROJECT_NAME} PRIVATE pybind11::embed)
target_link_libraries(${LIB_PROJECT_NAME} PUBLIC
    clean-core
    typed-geometry
    ctracer
    reflector
    task-dispatcher
    rich-log
    polymesh
    nanoflann
    polyscope
    glpk
    lpsolve
    # scip
    soplex
    CGAL::Eigen3_support
    CGAL::SCIP_support
    # CGAL::GLPK_support
    # pcl
#    glfw
#    glow
#    imgui
#    glow-extras

    ${OpenCV_LIBS}

    ${COMMON_LINKER_FLAGS}
)



target_include_directories(${LIB_PROJECT_NAME} PUBLIC "src")
target_compile_options(${LIB_PROJECT_NAME} PUBLIC ${COMMON_COMPILER_FLAGS})


# dependency grouping in the IDE
foreach(TARGET_NAME
    clean-core
    typed-geometry
    ctracer
    reflector
    task-dispatcher
    rich-log
    polymesh
    nanoflann
    polyscope
    glpk
    lpsolve
    # scip
    soplex
#    glfw
#    glow
#    imgui
#    glow-extras

)
set_property(TARGET ${TARGET_NAME} PROPERTY FOLDER "Extern")
endforeach()


# ===============================================
# pyind11
pybind11_add_module("${PYBIND_PROJECT_NAME}" src/linkmlpy.cpp)
target_link_libraries("${PYBIND_PROJECT_NAME}" PRIVATE ${LIB_PROJECT_NAME})

