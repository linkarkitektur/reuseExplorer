# HDF5
list(APPEND CMAKE_MODULE_PATH "./cmake/FindHDF5.cmake") # Replaces the bundled FindHDF5.cmake module
find_package(HDF5 1.10 COMPONENTS C HL)
if (HDF5_FOUND)
    message(NOTICE "HDF5 found")
    message(STATUS "HDF5 INCLUDE_DIR: ${HDF5_INCLUDE_DIRS}")
else()
    message(WARNING "HDF5 not found")
endif()

