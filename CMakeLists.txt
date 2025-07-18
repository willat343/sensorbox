cmake_minimum_required(VERSION 3.10)
project(sensorbox
    VERSION 0.5.0
    DESCRIPTION "Sensor Toolbox"
    LANGUAGES CXX
)

# Installation directories
include(GNUInstallDirs)

# CMake Build Type
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel" )
endif()
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

# General Options
option(BUILD_DOCUMENTATION "Build Doxygen Documentation" OFF)
option(BUILD_TESTS "Build Tests" OFF)
option(DEVELOPER_MODE "Use local versions of cppbox, mathbox" OFF)
option(USE_SYSTEM_Eigen3 "Use system install of Eigen3" ON)
option(USE_SYSTEM_nlohmann_json "Use system install of nlohmann_json" OFF)

# Configure FetchContent source directories for DEVELOPER_MODE
if (DEVELOPER_MODE)
    set(FETCHCONTENT_SOURCE_DIR_CPPBOX "${CMAKE_SOURCE_DIR}/dependencies/cppbox")
    set(FETCHCONTENT_SOURCE_DIR_MATHBOX "${CMAKE_SOURCE_DIR}/dependencies/mathbox")
endif()

# Dependency: Eigen3 (Target: Eigen3::Eigen, Note: Eigen3::Eigen sets INTERFACE_INCLUDE_DIRECTORIES)
if (NOT TARGET Eigen3::Eigen)
    set(EIGEN3_VERSION "3.3.7")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # Note: gcc/g++
        if (CMAKE_CXX_COMPILER_VERSION_MAJOR GREATER_EQUAL 10)
            # g++-10 with c++20 requires Eigen3 >=3.3.9. For further details see issues:
            #  - https://gitlab.com/libeigen/eigen/-/issues/2012
            #  - https://gitlab.com/libeigen/eigen/-/issues/1872
            set(EIGEN3_VERSION "3.3.9")
        endif()
    endif()
    if (${USE_SYSTEM_Eigen3})
        find_package(Eigen3 ${EIGEN3_VERSION} REQUIRED)
        message(STATUS "Found Eigen3 at ${Eigen3_DIR} with version ${Eigen3_VERSION}")
    else()
        FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG ${EIGEN3_VERSION}
        )
        FetchContent_MakeAvailable(eigen)
        message(STATUS "Fetched eigen to ${eigen_SOURCE_DIR} with version ${EIGEN3_VERSION}")
    endif()
endif()

# Dependency: nlohmann_json (Target: nlohmann_json::nlohmann_json)
if (NOT TARGET nlohmann_json::nlohmann_json)
    set(NLOHMANN_JSON_VERSION "3.7.3")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU") # Note: gcc/g++
        if (CMAKE_CXX_COMPILER_VERSION_MAJOR GREATER_EQUAL 10)
            # g++-10 with c++17 requires nlohmann_json >=3.8.0. For further details see issues:
            #  - https://github.com/nlohmann/json/issues/1912
            set(NLOHMANN_JSON_VERSION "3.8.0")
        endif()
    endif()
    if (${USE_SYSTEM_nlohmann_json})
        find_package(nlohmann_json ${NLOHMANN_JSON_VERSION} REQUIRED)
        message(STATUS "Found nlohmann_json at ${nlohmann_json_SOURCE_DIR} with version ${nlohmann_json_VERSION}")
    else()
        set(JSON_BuildTests OFF CACHE BOOL "")
        FetchContent_Declare(
            nlohmann_json
            GIT_REPOSITORY https://github.com/nlohmann/json.git
            GIT_TAG v${NLOHMANN_JSON_VERSION}
        )
        FetchContent_MakeAvailable(nlohmann_json)
        message(STATUS "Fetched nlohmann_json to ${nlohmann_json_SOURCE_DIR} with version ${NLOHMANN_JSON_VERSION}")
    endif()
endif()

# Dependency: cppbox (Target: cppbox::cppbox)
if (NOT TARGET cppbox::cppbox)
    set(CPPBOX_VERSION "0.0.4")
    FetchContent_Declare(
        cppbox
        GIT_REPOSITORY git@github.com:willat343/cppbox.git
        GIT_TAG v${CPPBOX_VERSION}
    )
    FetchContent_MakeAvailable(cppbox)
    if (${DEVELOPER_MODE})
        message(STATUS "Found cppbox at ${cppbox_SOURCE_DIR} (DEVELOPER_MODE = ON)")
    else()
        message(STATUS "Fetched cppbox to ${cppbox_SOURCE_DIR} with version ${CPPBOX_VERSION}")
    endif()
endif()

# Dependency: mathbox (Target: mathbox::mathbox)
if (NOT TARGET mathbox::mathbox)
    set(MATHBOX_VERSION "0.5.0")
    FetchContent_Declare(
        mathbox
        GIT_REPOSITORY git@github.com:willat343/mathbox.git
        GIT_TAG v${MATHBOX_VERSION}
    )
    FetchContent_MakeAvailable(mathbox)
    if (${DEVELOPER_MODE})
        message(STATUS "Found mathbox at ${mathbox_SOURCE_DIR} (DEVELOPER_MODE = ON)")
    else()
        message(STATUS "Fetched mathbox to ${mathbox_SOURCE_DIR} with version ${MATHBOX_VERSION}")
    endif()
endif()

# Build library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED
    src/unary.cpp
)
add_library(${PROJECT_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})

# Include directories
target_include_directories(${LIBRARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

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
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    mathbox::mathbox
    cppbox::cppbox
)

# Configure package version and config files
include(CMakePackageConfigHelpers)
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
