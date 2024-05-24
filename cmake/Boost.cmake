# Boost and its components
find_package(Boost REQUIRED)
if(Boost_FOUND)
  message(NOTICE "Boost found")
  message(STATUS "Boost_VERSION: ${Boost_VERSION}")
  message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
  message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
else()
  message(FATAL_ERROR "Boost not found")
  message(
    STATUS
      "NOTICE: This project requires the Boost library, and will not be compiled."
  )
  return()
endif()
