cmake_minimum_required(VERSION 2.8)
project(pcd_viewer)
include(cmake/FindGFlags.cmake)

find_package(PCL 1.7 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(PREFIX_DIR ${CMAKE_BINARY_DIR}/output/)

include_directories(./include)
add_subdirectory(src)
