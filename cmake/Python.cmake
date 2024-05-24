# Python
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
if(Python_FOUND)
    message(NOTICE "Python found")
    message(STATUS "Python Version: ${Python_VERSION}")
else()
    message(FATAL_ERROR "Python not found")
endif()