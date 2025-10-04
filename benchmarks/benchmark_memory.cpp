#include <octree/octree.hpp>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>

using namespace octree;

// Random number generator for reproducible benchmarks
static std::mt19937 rng(123);

// Generate random points within a boundary
template<typename T>
std::vector<Point3D<T>> generateRandomPoints(size_t count, T min, T max) {
    std::uniform_real_distribution<T> dist(min, max);
    std::vector<Point3D<T>> points;
    points.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        points.emplace_back(dist(rng), dist(rng), dist(rng));
    }

    return points;
}

// Benchmark: Memory usage with varying point counts
static void BM_MemoryUsage(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    // Report memory statistics
    state.counters["NodeCount"] = benchmark::Counter(
        static_cast<double>(stats.nodeCount));
    state.counters["PointCount"] = benchmark::Counter(
        static_cast<double>(stats.pointCount));
    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_MemoryUsage)
    ->Range(100, 100000)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory usage with different node capacities
static void BM_MemoryUsageDifferentCapacity(benchmark::State& state) {
    const size_t numPoints = 10000;
    const size_t maxPointsPerNode = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary, maxPointsPerNode);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    // Report memory statistics
    state.counters["NodeCount"] = benchmark::Counter(
        static_cast<double>(stats.nodeCount));
    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_MemoryUsageDifferentCapacity)
    ->RangeMultiplier(2)
    ->Range(4, 64)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory usage with different data types
static void BM_MemoryUsageFloat(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<float> boundary(Point3D<float>(0.0f, 0.0f, 0.0f),
                        Point3D<float>(1000.0f, 1000.0f, 1000.0f));

    Octree<float, int> tree(boundary);
    std::uniform_real_distribution<float> dist(0.0f, 1000.0f);

    std::vector<Point3D<float>> points;
    points.reserve(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        points.emplace_back(dist(rng), dist(rng), dist(rng));
    }

    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_MemoryUsageFloat)
    ->Range(100, 100000)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory usage with large data types
static void BM_MemoryUsageLargeData(benchmark::State& state) {
    struct LargeData {
        double values[10];
        int ids[5];
    };

    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, LargeData> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    LargeData data{};
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], data);
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_MemoryUsageLargeData)
    ->Range(100, 10000)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory overhead of subdivision
static void BM_MemorySubdivisionOverhead(benchmark::State& state) {
    const size_t maxPointsPerNode = 1; // Force maximum subdivision
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary, maxPointsPerNode);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    state.counters["NodeCount"] = benchmark::Counter(
        static_cast<double>(stats.nodeCount));
    state.counters["PointCount"] = benchmark::Counter(
        static_cast<double>(stats.pointCount));
    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
    state.counters["NodesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.nodeCount) / stats.pointCount);
}
BENCHMARK(BM_MemorySubdivisionOverhead)
    ->Range(10, 1000)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory efficiency comparison - shallow vs deep trees
static void BM_MemoryShallowVsDeep(benchmark::State& state) {
    const size_t numPoints = 10000;
    const size_t maxDepth = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary, 8, maxDepth);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    auto stats = tree.getMemoryStats();

    for (auto _ : state) {
        auto currentStats = tree.getMemoryStats();
        benchmark::DoNotOptimize(currentStats);
    }

    state.counters["NodeCount"] = benchmark::Counter(
        static_cast<double>(stats.nodeCount));
    state.counters["TotalBytes"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes));
    state.counters["BytesPerPoint"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_MemoryShallowVsDeep)
    ->DenseRange(4, 12, 2)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Peak memory during insertion
static void BM_MemoryInsertionGrowth(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    size_t maxBytes = 0;
    for (auto _ : state) {
        Octree<double, int> tree(boundary);

        for (size_t i = 0; i < numPoints; ++i) {
            tree.insert(points[i], static_cast<int>(i));

            if (i % 100 == 0) {
                auto stats = tree.getMemoryStats();
                maxBytes = std::max(maxBytes, stats.totalBytes);
            }
        }

        benchmark::DoNotOptimize(tree);
    }

    state.counters["PeakBytes"] = benchmark::Counter(
        static_cast<double>(maxBytes));
}
BENCHMARK(BM_MemoryInsertionGrowth)
    ->Range(100, 10000)
    ->Unit(benchmark::kMillisecond);
