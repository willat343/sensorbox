# Enable ctest
enable_testing()

# Build Example
add_executable(${PROJECT_NAME}_tests
    src/imu.cpp
    src/test/test_instances.cpp
)

# Properties
set_target_properties(${PROJECT_NAME}_tests PROPERTIES
    CXX_STANDARD 20
)

# Ensure the tests link to the built library, not installed library (this works by setting BUILD_RPATH, and then
# stopping the linker from setting RUNPATH instead of RPATH (RPATH is checked before LD_LIBRARY_PATH, while
# RUNPATH is checked after))
target_link_options(${PROJECT_NAME}_tests PUBLIC
    -Wl,--disable-new-dtags
)

# Compile Options
target_compile_options(${PROJECT_NAME}_tests PRIVATE
    -Wall -Wextra -Wpedantic
)

# Include Directories
target_include_directories(${PROJECT_NAME}_tests PUBLIC
    ../include
    include
)

# System Include Directories
target_include_directories(${PROJECT_NAME}_tests SYSTEM PUBLIC
    ${SENSORBOX_SYSTEM_INCLUDE_DIRS}
)

# Link Libraries
target_link_libraries(${PROJECT_NAME}_tests PUBLIC
    sensorbox
    ${SENSORBOX_SYSTEM_LIBRARIES}
    gtest_main
    gtest
    pthread
)

# Add test
add_test(NAME ${PROJECT_NAME}_tests COMMAND ${PROJECT_NAME}_tests)
