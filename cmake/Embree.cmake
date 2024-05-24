# Embree
find_package(embree 3.12.2 REQUIRED)
IF (EMBREE_FOUND)
  message(NOTICE "Embree found")
  message(STATUS "Embree INCLUDE: ${EMBREE_INCLUDE_DIRS}")
  message(STATUS "Embree LIB: ${EMBREE_LIBRARY})")

  IF (EMBREE_ISPC_SUPPORT)
    message(STATUS "Embree ISPC support enabled")
    OPTION(ENABLE_ISPC_SUPPORT "Build ISPC version of find_embree tutorial." OFF)

    # this configures the ADD_EMBREE_ISPC_EXECUTABLE from Embree
    IF (ENABLE_ISPC_SUPPORT)
      SET(ISPC_TARGETS "sse2;sse4;avx;avx2")
      SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake" ${CMAKE_MODULE_PATH})
      INCLUDE(ispc)

      get_target_property(embree_include_dir embree INTERFACE_INCLUDE_DIRECTORIES)
      INCLUDE_DIRECTORIES_ISPC(${embree_include_dir})
      ADD_EMBREE_ISPC_EXECUTABLE(find_embree_ispc find_embree_ispc.cpp find_embree_ispc.ispc)
      TARGET_LINK_LIBRARIES(find_embree_ispc embree)
    ENDIF()
  ENDIF()
ELSE()
ENDIF()



