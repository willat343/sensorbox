cmake_minimum_required(VERSION 3.10)
project(sensorbox_catkin
    VERSION 0.2.0
    DESCRIPTION "sensorbox catkin"
    LANGUAGES CXX
)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
    mathbox_catkin
)

# Install sensorbox
include(ExternalProject)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
ExternalProject_Add(sensorbox
    PREFIX ${CMAKE_BINARY_DIR}
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sensorbox
    INSTALL_DIR ${CATKIN_DEVEL_PREFIX}
    BUILD_ALWAYS TRUE
    CMAKE_ARGS
        -Dmathbox_DIR=${CATKIN_DEVEL_PREFIX}
        -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)

# Catkin package
catkin_package(
    INCLUDE_DIRS
        ${CATKIN_DEVEL_PREFIX}/include
    LIBRARIES
        ${CATKIN_DEVEL_PREFIX}/lib/libsensorbox.so
)
