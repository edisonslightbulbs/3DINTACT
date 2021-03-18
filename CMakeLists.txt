cmake_minimum_required(VERSION 3.11)

project(dbscan)

# optimization for dev environments
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# project version
set(MAJOR 0)
set(MINOR 1)
set(REMAP_VERSION ${MAJOR}.${MINOR})

# CXX and C versions
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_FLAGS "-pthread")

# cmake modules path spec
list(APPEND CMAKE_MODULE_PATH "$ENV{CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# cmake binary path spec
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# clang-tidy rules
# if(UNIX AND NOT APPLE)
#     set(CMAKE_CXX_CLANG_TIDY
#         clang-tidy -p pwd
#         --checks=-*,readability-*,clang-analyzer-*,performance-*)
# endif()

# build shared library (on)
option(BUILD_SHARED_LIBS "Build Shared Library" ON)

find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# main project include paths
set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SRC_DIR ${PROJECT_DIR}/src)
set(EXT_DIR ${PROJECT_DIR}/external)
set(LIBS_DIR ${PROJECT_DIR}/libs)

# for all headers files in project ...
file(GLOB_RECURSE HEADERS
    ${PROJECT_DIR}/*.h
    )

# ... find respective directories
set (INCLUDE_DIRS "")
foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# external src paths
file(GLOB_RECURSE EXT_SRC ${EXT_DIR}/*.cpp)

# libs src paths
file(GLOB_RECURSE LIBS_SRC ${LIBS_DIR}/*.cpp)

# define project target
add_executable(dbscan
    ${EXT_SRC}
    ${LIBS_SRC}
    ${SRC_DIR}/main.cpp
    )

target_include_directories(dbscan PRIVATE
    ${INCLUDE_DIRS}
    )

target_link_libraries(dbscan
    glog
    gflags
    Eigen3::Eigen
    )

# graph and colorize data post clustering
option(GRAPH "view graphs" ON)

if(GRAPH)
    add_custom_command(TARGET dbscan
        POST_BUILD
        COMMAND ./build/bin/dbscan --logtostderr=1
        COMMAND cloudcompare.CloudCompare ./build/bin/context.ply >/dev/null 2>&1 &
        #COMMAND cloudcompare.CloudCompare ./build/bin/context2d.ply >/dev/null 2>&1 &
        #COMMAND /bin/python3 ./scripts/graph.py
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()
