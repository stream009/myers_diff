cmake_minimum_required(VERSION 3.0)

project(myers_diff CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost 1.64.0 REQUIRED)

option(BUILD_TEST "build test suite" OFF)

add_library(myers_diff src/myers_diff.cpp)

target_compile_options(myers_diff PRIVATE
    -Wall -Wextra -pedantic -Wconversion -Wsign-conversion
)

target_include_directories(myers_diff
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
    PRIVATE
        ${Boost_INCLUDE_DIRS}
        3rd_party/range-v3/include
)

# don't build test when it is included as sub project
if(NOT CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    set(BUILD_TEST OFF)
endif()

if (BUILD_TEST)
    add_subdirectory(test)
endif()
