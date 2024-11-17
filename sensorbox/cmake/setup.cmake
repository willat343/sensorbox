# CMake Build Type
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel" )
endif()
message(STATUS "Build type set to ${CMAKE_BUILD_TYPE}")

# General Options
option(BUILD_DOCUMENTATION "Build Doxygen Documentation" OFF)
option(BUILD_TESTS "Build Tests" OFF)

# Installation directories
include(GNUInstallDirs)

# Variables
set(sensorbox_SYSTEM_INCLUDE_DIRS "")
set(sensorbox_SYSTEM_LIBRARIES "")

# Find packages
find_package(Eigen3 3.3 REQUIRED)
list(APPEND sensorbox_SYSTEM_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
)
list(APPEND sensorbox_SYSTEM_LIBRARIES
    Eigen3::Eigen
)

# Find mathbox
find_package(mathbox REQUIRED)
list(APPEND sensorbox_SYSTEM_INCLUDE_DIRS
    ${mathbox_INCLUDE_DIRS}
)
list(APPEND sensorbox_SYSTEM_LIBRARIES
    ${mathbox_LIBRARIES}
)
