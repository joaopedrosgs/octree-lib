cmake_minimum_required(VERSION 3.14)
project(octree-lib VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(OCTREE_BUILD_TESTS "Build tests" ON)
option(OCTREE_BUILD_BENCHMARKS "Build benchmarks" ON)
option(OCTREE_BUILD_EXAMPLES "Build examples" ON)

# Header-only library
add_library(octree INTERFACE)
target_include_directories(octree INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enable warnings
if(MSVC)
    target_compile_options(octree INTERFACE /W4 /WX)
else()
    target_compile_options(octree INTERFACE -Wall -Wextra -Wpedantic -Werror)
endif()

# Tests
if(OCTREE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Benchmarks
if(OCTREE_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# Examples
if(OCTREE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation
include(GNUInstallDirs)
install(TARGETS octree
    EXPORT octree-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(EXPORT octree-targets
    FILE octree-targets.cmake
    NAMESPACE octree::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/octree
)

# Generate package config
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/octree-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/octree-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/octree-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/octree
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/octree-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/octree-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/octree
)
