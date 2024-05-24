find_package( OpenCV REQUIRED )
if (OpenCV_FOUND)
    message(NOTICE "OpenCV found")
    message(STATUS "OpenCV Version: ${OpenCV_VERSION} - ${OpenCV_INCLUDE_DIRS}")

    set("OPENCV_LOG_LEVEL" "WARNING")

    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS})
    add_definitions(${OpenCV_DEFINITIONS})
else()
    message(FATAL_ERROR "OpenCV not found")
endif()