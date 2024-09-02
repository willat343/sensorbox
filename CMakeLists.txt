cmake_minimum_required(VERSION 3.5)
project(sensorbox
    VERSION 0.1.0
    DESCRIPTION "Sensor Toolbox"
    LANGUAGES CXX
)

# CMake Build Type
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to Release" )
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel" )
endif()

# General Options
option(BUILD_DOCUMENTATION "Build Doxygen Documentation" OFF)
option(BUILD_TESTS "Build Tests" OFF)

# Variables
set(SENSORBOX_SYSTEM_INCLUDE_DIRS "")
set(SENSORBOX_SYSTEM_LIBRARIES "")

# Eigen3 Package
find_package(Eigen3 3.3 REQUIRED)
list(APPEND SENSORBOX_SYSTEM_LIBRARIES
    Eigen3::Eigen
)

# Installation directories
include(GNUInstallDirs)

# Build library
add_library(${PROJECT_NAME} SHARED
    src/imu.cpp
)

# Properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
)

# Compile options
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall -Wextra -Wpedantic
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# System include directories
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${SENSORBOX_SYSTEM_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${SENSORBOX_SYSTEM_LIBRARIES}
)

# Configure package version and config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
    EXPORT ${PROJECT_NAME}
    DESTINATION ${PROJECT_NAME}
    FILE ${PROJECT_NAME}-export.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
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

# Uninstallation
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        @ONLY
    )

    # Add uninstall target
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

# Build Tests
if(BUILD_TESTS)
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
