cmake_minimum_required(VERSION 2.6)
project( orchomp )

set( ROS TRUE )
if ( ROS )
    find_package( catkin REQUIRED )
    catkin_package()
endif( ROS )

set( MOTION_OPTIMIZER_DIRECTORY ../chomp )

set( verbose FALSE )
if ( verbose )
    set( CMAKE_VERBOSE_MAKEFILE on )
endif( verbose )

set( CMAKE_BUILD_TYPE Release)

include(settings.cmake)

find_package(OpenRAVE REQUIRED)
include_directories(${OpenRAVE_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS})

add_subdirectory( ${MOTION_OPTIMIZER_DIRECTORY} chomp)
include_directories( ${MOTION_OPTIMIZER_DIRECTORY} )

#the libraries necessary for orchomp.
set( ORCHOMPLIBS   
    src/orchomp.cpp

    src/orchomp_mod_utils.cpp
    src/orchomp_mod_parse.cpp
    src/orchomp_mod.cpp
    
    src/orchomp_kdata.cpp
    src/orchomp_distancefield.cpp
    src/orchomp_collision.cpp
    src/orchomp_constraint.cpp
    src/orchomp_collision_pruner.cpp

    src/utils/os.c
    src/utils/util_shparse.c
    )

#This is all of the ros stuff
add_library(orchomp SHARED ${ORCHOMPLIBS})

set_target_properties(orchomp PROPERTIES COMPILE_FLAGS
                      "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS 
                      "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(orchomp motionoptimizer mzcommon nlopt ${OpenRAVE_LIBRARIES})


ADD_CUSTOM_TARGET(debug
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Debug"
  )

ADD_CUSTOM_TARGET(release
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Release"
  )
