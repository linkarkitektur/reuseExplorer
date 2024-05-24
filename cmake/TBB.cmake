# TBB
find_package( TBB REQUIRED )
if (TBB_FOUND)
    message(NOTICE "TBB found")
    message(STATUS "TBB in ${TBB_DIR}")
else()
    message(WARNING "TBB not found")
endif()