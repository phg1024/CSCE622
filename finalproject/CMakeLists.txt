cmake_minimum_required(VERSION 2.8)
project(halfedge_data_structure)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g")

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()
include( ${CGAL_USE_FILE} )

set(BOOST_LIBRARYDIR /usr/lib64)

find_package(Boost REQUIRED COMPONENTS graph timer system)
include_directories(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
link_directories(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})
link_libraries(${Boost_LIBRARIES} -lboost_timer -lboost_system)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${OPENGL_LIBRARY_DIRS})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${GLEW_LIBRARY_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${GLUT_LIBRARY_DIRS})

add_library(meshloader meshloader.cpp)
link_libraries(meshloader)

include( CGAL_CreateSingleSourceCGALProgram )
create_single_source_cgal_program( "build_mesh_cgal.cpp" )

add_subdirectory(tests)
add_subdirectory(third_party/tinyobjloader)
