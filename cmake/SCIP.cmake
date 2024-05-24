# SCIP
find_package(SCIP REQUIRED)
if (SCIP_FOUND)
    message(NOTICE "SCIP found")
    message(STATUS "SCIP Version: ${SCIP_VERSION}: ${SCIP_DIR}")
else()
    message( FATAL_ERROR "SCIP not found")
endif()