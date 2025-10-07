include(FetchContent)

# Fetch Google Test
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()
include(GoogleTest)

# Test executable
add_executable(octree_tests
    test_octree.cpp
    test_point.cpp
    test_aabb.cpp
    test_raycast.cpp
)

target_link_libraries(octree_tests
    PRIVATE
    octree
    GTest::gtest_main
)

# Discover tests
gtest_discover_tests(octree_tests)
