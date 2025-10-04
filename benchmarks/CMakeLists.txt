include(FetchContent)

# Fetch Google Benchmark
FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.8.3
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(benchmark)

# Benchmark executable
add_executable(octree_benchmarks
    benchmark_octree.cpp
    benchmark_memory.cpp
    benchmark_voxel.cpp
)

target_link_libraries(octree_benchmarks
    PRIVATE
    octree
    benchmark::benchmark
    benchmark::benchmark_main
)

# Optimization flags
if(MSVC)
    target_compile_options(octree_benchmarks PRIVATE
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:Debug>:/Od>
    )
else()
    target_compile_options(octree_benchmarks PRIVATE
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:Debug>:-O0>
    )
endif()
