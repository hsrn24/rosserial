project(ros-lib)

add_library(ros-lib INTERFACE)

#
# Create list of subdirectories
#
function(list_subdirectories retval curdir return_relative include_root)
   file(GLOB sub-dir RELATIVE ${curdir} *)
   if(${include_root})
     set(list_of_dirs .)
   else()
     set(list_of_dirs "")
   endif()
   foreach(dir IN LISTS sub-dir)
     if(IS_DIRECTORY ${curdir}/${dir})
       if (${return_relative})
         list(APPEND list_of_dirs ${dir})
         message(DEBUG "ros-lib: found ${dir}")
       else()
         list(APPEND list_of_dirs ${curdir}/${dir})
         message(DEBUG "ros-lib: found ${dir}")
       endif()
     endif()
   endforeach()
   set(${retval} ${list_of_dirs} PARENT_SCOPE)
endfunction()

list_subdirectories(
    MBED_ROSSERIAL_INCLUDES 
    ${CMAKE_CURRENT_LIST_DIR}
    ON
    ON
)

list(APPEND MBED_ROSSERIAL_SOURCES
    duration.cpp
    time.cpp
    MbedHardware.cpp
)

target_include_directories(ros-lib 
    INTERFACE
    ${MBED_ROSSERIAL_INCLUDES}
)

target_sources(ros-lib 
    INTERFACE
    ${MBED_ROSSERIAL_SOURCES}
)