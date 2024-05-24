# Eigen and its components
find_package(Eigen3 3.3 REQUIRED) #(requires 3.1.0 or greater)
if (EIGEN3_FOUND)
  message(NOTICE "Eigen3 found")
  message(STATUS "Eigen3 Version: ${EIGEN3_VERSION}")
  message(STATUS "Eigen3 INCLUDE: ${EIGEN3_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "Eigen3 not found")
endif()