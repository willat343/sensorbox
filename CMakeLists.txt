cmake_minimum_required(VERSION 3.21)
project(sensorbox
    VERSION 0.6.0
    DESCRIPTION "Sensor Toolbox"
    LANGUAGES CXX
)

## Include Standard CMake Packages
# GNUInstallDirs for Install directories
include(GNUInstallDirs)
# FetchContent for dependencies
include(FetchContent)
# CTest for BUILD_TESTING option (default ON)
include(CTest)
# Package version and config file generation
include(CMakePackageConfigHelpers)

# General Options
option(BUILD_DOCUMENTATION "Build Doxygen Documentation" OFF)
option(BUILD_SENSORBOX_ROS "Build sensorbox ROS 1 conversion functions" OFF)
option(DEVELOPER_MODE "Use local versions of cmakebox, cppbox, mathbox, and convert" OFF)

# Configure FetchContent source directories for DEVELOPER_MODE
if (DEVELOPER_MODE)
    set(FETCHCONTENT_SOURCE_DIR_CMAKEBOX "${CMAKE_SOURCE_DIR}/dependencies/cmakebox")
    set(FETCHCONTENT_SOURCE_DIR_CPPBOX "${CMAKE_SOURCE_DIR}/dependencies/cppbox")
    set(FETCHCONTENT_SOURCE_DIR_MATHBOX "${CMAKE_SOURCE_DIR}/dependencies/mathbox")
    set(FETCHCONTENT_SOURCE_DIR_CONVERT "${CMAKE_SOURCE_DIR}/dependencies/convert")
endif()

# Dependency: cmakebox
set(CMAKEBOX_VERSION "0.0.3")
FetchContent_Declare(
    cmakebox
    GIT_REPOSITORY git@github.com:willat343/cmakebox.git
    GIT_TAG v${CMAKEBOX_VERSION}
)
FetchContent_MakeAvailable(cmakebox)
list(APPEND CMAKE_MODULE_PATH "${cmakebox_SOURCE_DIR}/cmake")
include(CMakeBox)

# CMake Build Type
setup_build_type()

# Get CXX Compiler Information
get_cxx_compiler_info()

# Dependency: Eigen3 (Target: Eigen3::Eigen)
set(EIGEN3_VERSION "3.4.0")
import_Eigen3(
    VERSION ${EIGEN3_VERSION}
)

# Dependency: nlohmann_json (Target: nlohmann_json::nlohmann_json)
set(NLOHMANN_JSON_VERSION "3.7.3")
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # Note: gcc/g++
    if (CMAKE_CXX_COMPILER_VERSION_MAJOR GREATER_EQUAL 10)
        # g++-10 with c++17 requires nlohmann_json >=3.8.0. For further details see issues:
        #  - https://github.com/nlohmann/json/issues/1912
        set(NLOHMANN_JSON_VERSION "3.8.0")
    endif()
endif()
import_nlohmann_json(
    VERSION ${NLOHMANN_JSON_VERSION}
)

# Dependency: cppbox (Target: cppbox::cppbox)
set(CPPBOX_VERSION "0.1.0")
import_dependency(
    cppbox
    TARGET cppbox::cppbox
    VERSION ${CPPBOX_VERSION}
    USE_FIND_PACKAGE_REQUIRED_VERSION ${CPPBOX_VERSION}
    GIT_REPOSITORY git@github.com:willat343/cppbox.git
    GIT_TAG v${CPPBOX_VERSION}
)

# Dependency: mathbox (Target: mathbox::mathbox)
set(MATHBOX_VERSION "0.6.0")
import_dependency(
    mathbox
    TARGET mathbox::mathbox
    VERSION ${MATHBOX_VERSION}
    USE_FIND_PACKAGE_REQUIRED_VERSION ${MATHBOX_VERSION}
    GIT_REPOSITORY git@github.com:willat343/mathbox.git
    GIT_TAG v${MATHBOX_VERSION}
)

# Dependency: convert (Target: convert::convert) and sensorbox_ros
set(BUILD_EIGEN_COMPONENTS ON CACHE BOOL "" FORCE)
set(SENSORBOX_ROS_SRC_FILES)
set(SENSORBOX_ROS_SYSTEM_INCLUDE_DIRECTORIES)
set(SENSORBOX_ROS_LINK_LIBRARIES)
if (BUILD_SENSORBOX_ROS)
    set(BUILD_ROS_COMPONENTS ON CACHE BOOL "" FORCE)
    find_package(sensor_msgs REQUIRED)
    list(APPEND SENSORBOX_ROS_SRC_FILES
        src/convert/sensorbox_ros/imu.cpp
        src/convert/sensorbox_ros/pose_twist.cpp
        src/convert/sensorbox_ros/pose.cpp
    )
    list(APPEND SENSORBOX_ROS_INCLUDE_DIRECTORIES
        ${sensor_msgs_INCLUDE_DIRS}
    )
    list(APPEND SENSORBOX_ROS_LINK_LIBRARIES
        ${sensor_msgs_LIBRARIES}
    )
endif()
set(CONVERT_VERSION "0.6.0")
import_dependency(
    convert
    TARGET convert::convert
    VERSION ${CONVERT_VERSION}
    USE_FIND_PACKAGE_REQUIRED_VERSION ${CONVERT_VERSION}
    GIT_REPOSITORY git@github.com:willat343/convert.git
    GIT_TAG v${CONVERT_VERSION}
)

# Build library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED
    ${SENSORBOX_ROS_SRC_FILES}
    src/ros1_decoder.cpp
    src/sensor.cpp
    src/unary.cpp
)
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})
# Include directories
target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
# System include directories
if (SENSORBOX_ROS_SYSTEM_INCLUDE_DIRECTORIES)
    target_include_directories(${LIBRARY_NAME} PUBLIC SYSTEM
        ${SENSORBOX_ROS_SYSTEM_INCLUDE_DIRECTORIES}
    )
endif()
# Properties
set_target_properties(${LIBRARY_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)
# Compile options
target_compile_options(${LIBRARY_NAME} PRIVATE
    -Wall -Wextra -Wpedantic
)
# Link libraries
target_link_libraries(${LIBRARY_NAME} PUBLIC
    ${SENSORBOX_ROS_LINK_LIBRARIES}
    convert::convert    # May depend on Eigen3::Eigen
    mathbox::mathbox    # Depends on Eigen3::Eigen
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    cppbox::cppbox
)

# Configure package version and config files
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Installation
install(
    TARGETS ${LIBRARY_NAME}
    EXPORT ${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    EXPORT ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}-export.cmake
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
        PATTERN "*.hpp"
        PATTERN "*.h"
)

# Build-Tree Export
export(
    EXPORT ${PROJECT_NAME}
    NAMESPACE ${PROJECT_NAME}::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-export.cmake"
)
export(PACKAGE ${PROJECT_NAME})

# Uninstallation
create_uninstall_target()

# Build Tests
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# Build Documentation
if (BUILD_DOCUMENTATION)
    find_package(Doxygen REQUIRED)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    configure_file(Doxyfile.in ${DOXYGEN_OUT})
    add_custom_target(documentation ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating documentation with doxygen."
        VERBATIM
    )
endif()
