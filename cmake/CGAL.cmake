# CGAL and its components

find_package(MPI REQUIRED)
find_package(MPFR REQUIRED)
find_package(CGAL REQUIRED)
if(CGAL_FOUND)
    message(NOTICE "CGAL found")
    message(STATUS "CGAL Version: ${CGAL_VERSION}")


    if (SCIP_FOUND)
        message(STATUS "CGAL adding Support for SCIP")
        include(CGAL_SCIP_support)
    else()
        message(FATAL_ERROR "CGAL SCIP not found")
    endif()


    if (EIGEN3_FOUND)
        message(STATUS "CGAL adding Support for Eigen")
        include(CGAL_Eigen3_support)

        if(NOT TARGET CGAL::Eigen3_support)
          message(
            STATUS
              "NOTICE: This project requires Eigen 3.1 (or greater) and will not be compiled."
          )
          return()
        endif()   
    else()
        message(FATAL_ERROR "CGAL Eigen not found")
    endif()

    if (TBB_FOUND)
        message(STATUS "CGAL adding Support for TBB")
        include(CGAL_TBB_support)
    else()
        message(FATAL_ERROR "CGAL TBB not found")
    endif()

    if (OPENCV_FOUND)
        message(STATUS "CGAL adding Support for OpenCV")
        include(CGAL_OpenCV_support)
    else()
        message(FATAL_ERROR "CGAL OpneCV not found")
    endif()



else()
  message(FATAL_ERROR "CGAL not found")
endif()
  

