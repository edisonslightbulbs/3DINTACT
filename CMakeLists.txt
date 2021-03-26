cmake_minimum_required(VERSION 3.11)
project(INTACToolkit)

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

# optimization
option(OPTIMIZE_CODEBASE "Clang-tidy checks during build" OFF)
if(OPTIMIZE_CODEBASE)

    if(UNIX AND NOT APPLE)
        set(CMAKE_CXX_CLANG_TIDY
            clang-tidy -p pwd
            --checks=-*,readability-*,clang-analyzer-*,performance-*)
    endif()

    set(CMAKE_CXX_FLAGS "-Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()


# build shared library (on)
option(BUILD_SHARED_LIBS "Build Shared Library" ON)

# dependencies
find_package(Git REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# main project include paths
set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SRC_DIR ${PROJECT_DIR}/src)
set(EXT_DIR ${PROJECT_DIR}/external)
set(LIBS_DIR ${PROJECT_DIR}/libs)
set(KINECT_SDK ${EXT_DIR}/Azure-Kinect-Sensor-SDK)


# submodule management
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")

    option(GIT_SUBMODULE "Check submodules during build" OFF) # (default ON)
    if(GIT_SUBMODULE)
        message(STATUS "Updating submodule dependencies")
        execute_process(
            # update submodules correctly
            COMMAND ${GIT_EXECUTABLE} pull --recurse-submodules -j 8
            COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -j 8
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMODULE_RESULT)

        if(NOT GIT_SUBMODULE_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMODULE_RESULT}, please checkout submodules")
        endif()
    endif()

    option(K4A_SDK_BUILD "Build the kinectSDK submodule" OFF) # (default ON)
    if(K4A_SDK_BUILD)
        if(NOT EXISTS "${KINECT_SDK}/CMakeLists.txt")
            message(FATAL_ERROR "The submodules were not downloaded!  GIT_SUBMODULE was turned off or failed. Please clone kinect SDK submodule and try again.")
        endif()

        execute_process(
            COMMAND /bin/bash ./scripts/setup-azure-kinect/build-project.sh -j 12
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

        if(NOT EXISTS "${KINECT_SDK}/build/src/sdk/include/k4a/k4aversion.h")
            message(FATAL_ERROR "The kinectSDK submodule project did not build successfully. Please build the submodule project first.")
        endif()
    endif()
endif()

set(K4A_LIBRARY ${KINECT_SDK}/build/bin)
set(K4A_SRC_INCLUDE ${KINECT_SDK}/build/src/sdk/include)
set(K4A_INCLUDE ${KINECT_SDK}/include)

# find header directories
file(GLOB_RECURSE HEADERS
    ${LIBS_DIR}/*.h
    )
set (INCLUDE_DIRS "")

foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

# src paths
file(GLOB_RECURSE LIBS_SRC ${LIBS_DIR}/*.cpp)


# INTACT project target, includes, and linkages
add_executable(INTACToolkit
    ${EXT_SRC}
    ${LIBS_SRC}
    ${SRC_DIR}/main.cpp
    )
target_include_directories(INTACToolkit PRIVATE
    ${K4A_INCLUDE}
    ${INCLUDE_DIRS}
    ${K4A_SRC_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    )

target_link_libraries(INTACToolkit
    glog
    gflags
    Eigen3::Eigen
    ${OpenCV_LIBS}
    )

if(UNIX AND NOT APPLE)
    target_link_libraries(INTACToolkit
        ${K4A_LIBRARY}/libk4a.so
        )
endif()

option(RUN "execute target" ON) # (default ON)
if(RUN)
    add_custom_command(TARGET INTACToolkit
        POST_BUILD
        COMMAND mkdir -p ./output
        COMMAND ./build/bin/INTACToolkit --logtostderr=1
        COMMAND cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()

