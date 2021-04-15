cmake_minimum_required(VERSION 3.11)
project(INTACToolkit)

# optimization for dev environments
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# project version
set(MAJOR 0)
set(MINOR 1)
set(REMAP_VERSION ${MAJOR}.${MINOR})

# CXX and C versions
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_FLAGS "-pthread")

# cmake modules path spec
list(APPEND CMAKE_MODULE_PATH "$ENV{CMAKE_MODULE_PATH}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# cmake binary path spec
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

# build options
option(EXECUTE_TARGET "Execute Project Target" ON)              # default OFF
option(BUILD_SHARED_LIBS "Build Shared Library" ON)             # default ON
option(K4A_SDK_BUILD  "Build the kinect SDK project" ON)        # default ON
option(UPDATE_GIT_SUBMODULES "Update all git submodules " ON)   # default ON
option(OPTIMIZE_PROJECT "Tidy up and optimize source code" OFF) # default OFF


if(OPTIMIZE_PROJECT)
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


# dependencies
find_package(Git REQUIRED)
find_package(glog REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(Pangolin REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


# main project include paths
set(PROJECT_DIR ${CMAKE_CURRENT_LIST_DIR})
set(SRC_DIR ${PROJECT_DIR}/src)
set(EXT_DIR ${PROJECT_DIR}/external)
set(LIBS_DIR ${PROJECT_DIR}/libs)


# make sure submodule paths exist
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    if(UPDATE_GIT_SUBMODULES)
        message(STATUS "Updating submodule dependencies")
        execute_process(
            COMMAND ${GIT_EXECUTABLE} pull --recurse-submodules -j 12
            COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive -j 12
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE GIT_SUBMODULE_RESULT)

        if(NOT GIT_SUBMODULE_RESULT EQUAL "0")
            message(FATAL_ERROR "Failed to update git submodules. ${GIT_SUBMODULE_RESULT}")
        endif()
    endif()

    if(K4A_SDK_BUILD)
        if(NOT EXISTS "${EXT_DIR}/Azure-Kinect-Sensor-SDK/CMakeLists.txt")
            message(FATAL_ERROR "Please make sure the kinect SDK project is setup correctly.")
        endif()

        execute_process(
            COMMAND /bin/bash ./scripts/build_kinect_sdk.sh -j 12
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

        if(NOT EXISTS "${EXT_DIR}/Azure-Kinect-Sensor-SDK/build/src/sdk/include/k4a/k4aversion.h")
            message(FATAL_ERROR "Please build kinect SDK project manually.")
        endif()
    endif()
endif()


# after making sure SDK directories are setup, set Kinect paths
set(KINECT_SDK ${EXT_DIR}/Azure-Kinect-Sensor-SDK)
set(K4A_LIBRARY ${KINECT_SDK}/build)
set(K4A_VERSION ${KINECT_SDK}/build/src/sdk/include)
set(K4A_INCLUDE ${KINECT_SDK}/include)


# find headers
set (INCLUDE_DIRS "")
file(GLOB_RECURSE HEADERS
    ${LIBS_DIR}/*.h
    )
foreach (HEADER ${HEADERS})
    get_filename_component(DIR ${HEADER} PATH)
    list (APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)


# find src
file(GLOB_RECURSE LIBS_SRC ${LIBS_DIR}/*.cpp)


# set-up target
add_executable(INTACToolkit
    ${EXT_SRC}
    ${LIBS_SRC}
    ${SRC_DIR}/main.cpp
    )


# set-up target include directories
target_include_directories(INTACToolkit PRIVATE
    ${K4A_VERSION}
        ${K4A_INCLUDE}
    ${INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )


# link libraries
target_link_libraries(INTACToolkit
    glog
    gflags
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${K4A_LIBRARY}/bin/libk4a.so
    )


# link libraries
if(EXECUTE_TARGET)
    add_custom_command(TARGET INTACToolkit
        POST_BUILD
        COMMAND mkdir -p ./output
        COMMAND rm -rf ./output/*.*
        COMMAND ./build/bin/INTACToolkit --logtostderr=1
        #COMMAND cloudcompare.CloudCompare ./output/rgb2depth.ply >/dev/null 2>&1 &
        #COMMAND cloudcompare.CloudCompare ./output/depth2rgb.ply >/dev/null 2>&1 &
        #COMMAND cloudcompare.CloudCompare ./output/context.ply >/dev/null 2>&1 &
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()
