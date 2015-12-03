cmake_minimum_required(VERSION 2.6)
project(path_count CXX)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g")

set(BOOST_LIBRARYDIR /usr/lib64)

find_package(Boost REQUIRED COMPONENTS graph)
include_directories(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
link_directories(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

set(CMAKE_PREFIX_PATH /usr/local/lib/cmake/glfw)
find_package(GLM REQUIRED)
include_directories(${INCLUDE_DIRECTORIES} ${GLM_INCLUDE_DIRS})
find_package(glfw3 REQUIRED)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

add_executable(mesh_simplification mesh_simplification.cpp)
target_link_libraries(mesh_simplification stringutils meshloader glfw3 ${OPENGL_LIBRARIES})
add_library(stringutils stringutils.cpp)
add_library(meshloader meshloader.cpp)
