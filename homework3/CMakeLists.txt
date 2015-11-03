cmake_minimum_required(VERSION 3.1)
project(path_count CXX)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")

find_package(Boost REQUIRED COMPONENTS graph)
include_directories(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
link_directories(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

add_executable(path_count main.cpp)
target_link_libraries(path_count)
