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

find_package(Boost REQUIRED COMPONENTS graph)
include_directories(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
link_directories(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${OPENGL_LIBRARY_DIRS})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${GLEW_LIBRARY_DIRS})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
link_directories(${LINK_DIRECTORIES} ${GLUT_LIBRARY_DIRS})

add_executable(test_hds test_hds.cpp)
target_link_libraries(test_hds meshloader ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${GLUT_LIBRARIES})

include_directories(..)
link_directories(..)
