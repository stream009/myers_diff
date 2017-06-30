cmake_minimum_required(VERSION 3.8)

project(myers_diff CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(Boost 1.64.0 REQUIRED)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories("include")
include_directories("3rd_party/range-v3/include")

add_library(myers_diff src/myers_diff.cpp)
