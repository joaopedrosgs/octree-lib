#include <octree/octree.hpp>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>

using namespace octree;

// Random number generator for reproducible benchmarks
static std::mt19937 rng(42);

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

// Benchmark: Insert points
static void BM_Insert(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (auto _ : state) {
        Octree<double, int> tree(boundary);
        for (size_t i = 0; i < numPoints; ++i) {
            tree.insert(points[i], static_cast<int>(i));
        }
        benchmark::DoNotOptimize(tree);
    }

    state.SetItemsProcessed(state.iterations() * numPoints);
}
BENCHMARK(BM_Insert)->Range(100, 100000);

// Benchmark: Insert with different max points per node
static void BM_InsertDifferentCapacity(benchmark::State& state) {
    const size_t numPoints = 10000;
    const size_t maxPointsPerNode = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (auto _ : state) {
        Octree<double, int> tree(boundary, maxPointsPerNode);
        for (size_t i = 0; i < numPoints; ++i) {
            tree.insert(points[i], static_cast<int>(i));
        }
        benchmark::DoNotOptimize(tree);
    }

    state.SetItemsProcessed(state.iterations() * numPoints);
}
BENCHMARK(BM_InsertDifferentCapacity)->RangeMultiplier(2)->Range(4, 64);

// Benchmark: Range query
static void BM_QueryRange(benchmark::State& state) {
    const size_t numPoints = 10000;
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    // Query range covers state.range(0) percent of the space
    double rangePct = state.range(0) / 100.0;
    double rangeSize = 1000.0 * rangePct;
    AABB<double> queryRange(Point3D<double>(0.0, 0.0, 0.0),
                           Point3D<double>(rangeSize, rangeSize, rangeSize));

    std::vector<std::pair<Point3D<double>, int>> results;
    for (auto _ : state) {
        results.clear();
        tree.queryRange(queryRange, results);
        benchmark::DoNotOptimize(results);
    }
}
BENCHMARK(BM_QueryRange)->Arg(10)->Arg(25)->Arg(50)->Arg(75)->Arg(100);

// Benchmark: K-nearest neighbor query
static void BM_QueryKNearest(benchmark::State& state) {
    const size_t numPoints = 10000;
    const size_t k = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    Point3D<double> queryPoint(500.0, 500.0, 500.0);
    std::vector<std::pair<Point3D<double>, int>> results;

    for (auto _ : state) {
        results.clear();
        tree.queryKNearest(queryPoint, k, results);
        benchmark::DoNotOptimize(results);
    }
}
BENCHMARK(BM_QueryKNearest)->RangeMultiplier(10)->Range(1, 1000);

// Benchmark: Radius query
static void BM_QueryRadius(benchmark::State& state) {
    const size_t numPoints = 10000;
    const double radius = static_cast<double>(state.range(0));
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    Point3D<double> queryPoint(500.0, 500.0, 500.0);
    std::vector<std::pair<Point3D<double>, int>> results;

    for (auto _ : state) {
        results.clear();
        tree.queryRadius(queryPoint, radius, results);
        benchmark::DoNotOptimize(results);
    }
}
BENCHMARK(BM_QueryRadius)->Arg(10)->Arg(50)->Arg(100)->Arg(200)->Arg(500);

// Benchmark: Contains check
static void BM_Contains(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    for (auto _ : state) {
        bool result = tree.contains(points[numPoints / 2]);
        benchmark::DoNotOptimize(result);
    }

    state.SetItemsProcessed(state.iterations());
}
BENCHMARK(BM_Contains)->Range(100, 100000);

// Benchmark: Traverse all points
static void BM_Traverse(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    for (auto _ : state) {
        int sum = 0;
        tree.traverse([&sum](const Point3D<double>& p, const int& data) {
            (void)p;
            sum += data;
        });
        benchmark::DoNotOptimize(sum);
    }

    state.SetItemsProcessed(state.iterations() * numPoints);
}
BENCHMARK(BM_Traverse)->Range(100, 100000);

// Benchmark: Clear operation
static void BM_Clear(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);

    for (auto _ : state) {
        state.PauseTiming();
        Octree<double, int> tree(boundary);
        for (size_t i = 0; i < numPoints; ++i) {
            tree.insert(points[i], static_cast<int>(i));
        }
        state.ResumeTiming();

        tree.clear();
        benchmark::DoNotOptimize(tree);
    }
}
BENCHMARK(BM_Clear)->Range(100, 100000);

// Benchmark: Clustered vs uniformly distributed data
static void BM_InsertClustered(benchmark::State& state) {
    const size_t numPoints = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    // Generate clustered points
    std::normal_distribution<double> dist(500.0, 100.0);
    std::vector<Point3D<double>> points;
    points.reserve(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        points.emplace_back(
            std::clamp(dist(rng), 0.0, 1000.0),
            std::clamp(dist(rng), 0.0, 1000.0),
            std::clamp(dist(rng), 0.0, 1000.0)
        );
    }

    for (auto _ : state) {
        Octree<double, int> tree(boundary);
        for (size_t i = 0; i < numPoints; ++i) {
            tree.insert(points[i], static_cast<int>(i));
        }
        benchmark::DoNotOptimize(tree);
    }

    state.SetItemsProcessed(state.iterations() * numPoints);
}
BENCHMARK(BM_InsertClustered)->Range(100, 100000);

// Benchmark: Query performance with different tree depths
static void BM_QueryDifferentDepths(benchmark::State& state) {
    const size_t numPoints = 10000;
    const size_t maxDepth = state.range(0);
    AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                         Point3D<double>(1000.0, 1000.0, 1000.0));

    Octree<double, int> tree(boundary, 8, maxDepth);
    auto points = generateRandomPoints<double>(numPoints, 0.0, 1000.0);
    for (size_t i = 0; i < numPoints; ++i) {
        tree.insert(points[i], static_cast<int>(i));
    }

    Point3D<double> queryPoint(500.0, 500.0, 500.0);
    std::vector<std::pair<Point3D<double>, int>> results;

    for (auto _ : state) {
        results.clear();
        tree.queryKNearest(queryPoint, 10, results);
        benchmark::DoNotOptimize(results);
    }
}
BENCHMARK(BM_QueryDifferentDepths)->DenseRange(4, 12, 2);
