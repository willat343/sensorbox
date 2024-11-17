cmake_minimum_required(VERSION 3.5)
project(sensorbox_catkin
    VERSION 0.2.0
    DESCRIPTION "sensorbox catkin"
    LANGUAGES CXX
)

# Find catkin (which adds to CMAKE_PREFIX_PATH)
find_package(catkin REQUIRED)

# Setup
include(${CMAKE_CURRENT_SOURCE_DIR}/../sensorbox/cmake/setup.cmake)

# Install sensorbox
include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
string(REPLACE ";" "|" CMAKE_PREFIX_PATH_EXT_PROJ "${CMAKE_PREFIX_PATH}")
ExternalProject_Add(sensorbox
    PREFIX ${CMAKE_BINARY_DIR}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sensorbox
    INSTALL_DIR ${CATKIN_DEVEL_PREFIX}
    BUILD_ALWAYS TRUE
    LIST_SEPARATOR | # Use alternate list separator to pass lists to CMAKE_ARGS
    CMAKE_ARGS
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_EXT_PROJ}
        -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DBUILD_DOCUMENTATION=${BUILD_DOCUMENTATION}
        -DBUILD_TESTS=${BUILD_TESTS}
)

# Catkin package
catkin_package(
    INCLUDE_DIRS
        ${CATKIN_DEVEL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}
        ${sensorbox_SYSTEM_INCLUDE_DIRS}
    LIBRARIES
        ${CATKIN_DEVEL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libsensorbox.so
        ${sensorbox_SYSTEM_LIBRARIES}
)
