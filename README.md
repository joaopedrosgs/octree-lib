# Octree Library

[![CI](https://github.com/joaopedrosgs/octree-lib/actions/workflows/ci.yml/badge.svg)](https://github.com/joaopedrosgs/octree-lib/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.14%2B-blue.svg)](https://cmake.org/)

A high-performance, header-only C++17 octree library for efficient spatial indexing and querying of 3D data.

## Features

- **Header-only**: Easy integration, just include and use
- **Template-based**: Flexible coordinate types (float, double, etc.) and custom data storage
- **Comprehensive queries**:
  - Range queries (AABB intersection)
  - K-nearest neighbor search
  - Radius-based queries
  - Point containment checks
  - **Raycasting** (first hit & all hits along ray)
- **Performance optimized**: Efficient spatial partitioning with configurable parameters
- **Memory efficient**: Track memory usage with built-in statistics
- **Well-tested**: Comprehensive unit tests with Google Test
- **Benchmarked**: Performance and memory benchmarks included
- **Modern C++**: C++17 standard with best practices

## Quick Start

### Installation

#### Option 1: Header-only (simplest)

Simply copy `include/octree/octree.hpp` to your project:

```cpp
#include "octree.hpp"  // That's it!
```

#### Option 2: CMake FetchContent (recommended)

Add to your `CMakeLists.txt`:

```cmake
include(FetchContent)

FetchContent_Declare(
    octree
    GIT_REPOSITORY https://github.com/joaopedrosgs/octree-lib.git
    GIT_TAG main  # or specific version tag
)
FetchContent_MakeAvailable(octree)

# Link to your target
target_link_libraries(your_target PRIVATE octree)
```

#### Option 3: System-wide installation

```bash
mkdir build && cd build
cmake ..
cmake --build .
sudo cmake --install .
```

Then in your CMakeLists.txt:
```cmake
find_package(octree REQUIRED)
target_link_libraries(your_target PRIVATE octree::octree)
```

### Basic Usage

```cpp
#include <octree/octree.hpp>

using namespace octree;

// Create a bounding box for the octree
AABB<double> boundary(
    Point3D<double>(0.0, 0.0, 0.0),
    Point3D<double>(100.0, 100.0, 100.0)
);

// Create an octree (max 8 points per node, max depth 8)
Octree<double, int> tree(boundary, 8, 8);

// Insert points with associated data
tree.insert(Point3D<double>(10.0, 20.0, 30.0), 1);
tree.insert(Point3D<double>(50.0, 50.0, 50.0), 2);

// Query points in a region
AABB<double> queryRegion(
    Point3D<double>(0.0, 0.0, 0.0),
    Point3D<double>(60.0, 60.0, 60.0)
);
std::vector<std::pair<Point3D<double>, int>> results;
tree.queryRange(queryRegion, results);

// Find k-nearest neighbors
Point3D<double> queryPoint(25.0, 25.0, 25.0);
tree.queryKNearest(queryPoint, 5, results);

// Find points within a radius
tree.queryRadius(queryPoint, 30.0, results);

// Raycast for collision detection
Ray3D<double> ray(Point3D<double>(0, 10, 10), Point3D<double>(1, 0, 0));
std::pair<Point3D<double>, int> hit;
if (tree.raycastFirst(ray, 100.0, hit)) {
    // Ray hit a point!
}
```

## Building

### Requirements

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.14 or higher
- Google Test (automatically fetched for tests)
- Google Benchmark (automatically fetched for benchmarks)

### Build Options

```bash
mkdir build && cd build

# Basic build
cmake ..
cmake --build .

# Disable tests
cmake -DOCTREE_BUILD_TESTS=OFF ..

# Disable benchmarks
cmake -DOCTREE_BUILD_BENCHMARKS=OFF ..

# Disable examples
cmake -DOCTREE_BUILD_EXAMPLES=OFF ..

# Release build for performance
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

### Running Tests

```bash
cd build
ctest --output-on-failure
# or
./tests/octree_tests
```

### Running Benchmarks

```bash
cd build
./benchmarks/octree_benchmarks

# Run specific benchmark suites
./benchmarks/octree_benchmarks --benchmark_filter=BM_Insert
./benchmarks/octree_benchmarks --benchmark_filter=BM_VoxelWorld
./benchmarks/octree_benchmarks --benchmark_filter=BM_Memory
```

See [VOXEL_BENCHMARKS.md](VOXEL_BENCHMARKS.md) for detailed voxel game performance analysis.

### Running Examples

```bash
cd build
./examples/basic_usage
./examples/spatial_queries
./examples/custom_data
```

## API Reference

### Core Classes

#### `Point3D<T>`

Represents a 3D point with coordinates of type `T`.

**Methods:**
- `distance(const Point3D& other)` - Euclidean distance
- `distanceSquared(const Point3D& other)` - Squared distance (faster)

#### `AABB<T>`

Axis-Aligned Bounding Box.

**Methods:**
- `contains(const Point3D<T>& point)` - Check if point is inside
- `intersects(const AABB& other)` - Check if boxes intersect
- `center()` - Get center point
- `distanceSquared(const Point3D<T>& point)` - Distance from point to box

#### `Octree<T, DataType>`

Main octree data structure.

**Template Parameters:**
- `T` - Coordinate type (float, double, etc.)
- `DataType` - Type of data associated with each point

**Constructor:**
```cpp
Octree(const AABB<T>& boundary,
       size_t maxPointsPerNode = 8,
       size_t maxDepth = 8)
```

**Methods:**

- `bool insert(const Point3D<T>& point, const DataType& data)`
  - Insert a point with associated data

- `void queryRange(const AABB<T>& range, std::vector<std::pair<Point3D<T>, DataType>>& results)`
  - Find all points within a bounding box

- `void queryKNearest(const Point3D<T>& point, size_t k, std::vector<std::pair<Point3D<T>, DataType>>& results)`
  - Find k nearest neighbors to a point

- `void queryRadius(const Point3D<T>& point, T radius, std::vector<std::pair<Point3D<T>, DataType>>& results)`
  - Find all points within a radius

- `bool contains(const Point3D<T>& point)`
  - Check if a point exists in the tree

- `void clear()`
  - Remove all points

- `size_t size()`
  - Get number of points

- `bool empty()`
  - Check if tree is empty

- `void traverse(std::function<void(const Point3D<T>&, const DataType&)> visitor)`
  - Visit all points with a callback

- `MemoryStats getMemoryStats()`
  - Get memory usage statistics

#### `Ray3D<T>`

3D ray for raycasting operations.

**Constructor:**
```cpp
Ray3D(const Point3D<T>& origin, const Point3D<T>& direction)
```

**Raycast Methods:**

- `bool raycastFirst(const Ray3D<T>& ray, T maxDistance, std::pair<Point3D<T>, DataType>& result)`
  - Find the first point hit by a ray
  - Returns true if a point was hit

- `void raycastAll(const Ray3D<T>& ray, T maxDistance, std::vector<std::pair<Point3D<T>, DataType>>& results)`
  - Find all points hit by a ray within maxDistance

**Example:**
```cpp
// Cast a ray from player position
Point3D<double> playerPos(0, 10, 0);
Point3D<double> lookDir(1, 0, 0);  // Looking along X axis
Ray3D<double> ray(playerPos, lookDir);

// Find first hit
std::pair<Point3D<double>, int> hit;
if (tree.raycastFirst(ray, 100.0, hit)) {
    std::cout << "Hit at distance: "
              << playerPos.distance(hit.first) << "\n";
}

// Find all hits
std::vector<std::pair<Point3D<double>, int>> allHits;
tree.raycastAll(ray, 100.0, allHits);
```

## Advanced Usage

### Custom Data Types

The octree supports any data type:

```cpp
struct GameObject {
    std::string name;
    int health;
    double rotation;
};

Octree<double, GameObject> gameTree(boundary);
gameTree.insert(Point3D<double>(10, 20, 30),
               GameObject{"Player", 100, 0.0});
```

### Performance Tuning

Adjust `maxPointsPerNode` and `maxDepth` based on your data distribution:

```cpp
// More subdivisions, better for clustered data
Octree<double, int> tree(boundary, 4, 10);

// Fewer subdivisions, better for uniform data
Octree<double, int> tree(boundary, 16, 6);

// Voxel games: Higher capacity for sparse worlds
Octree<double, VoxelBlock> voxelWorld(boundary, 128, 8);
```

**Voxel game benchmarks** show that 128 blocks/node provides best performance. See [VOXEL_BENCHMARKS.md](VOXEL_BENCHMARKS.md).

### Memory Monitoring

```cpp
auto stats = tree.getMemoryStats();
std::cout << "Nodes: " << stats.nodeCount << "\n";
std::cout << "Points: " << stats.pointCount << "\n";
std::cout << "Memory: " << stats.totalBytes << " bytes\n";
std::cout << "Bytes/point: " << (stats.totalBytes / stats.pointCount) << "\n";
```

## Performance

Typical performance characteristics (10,000 points):

- **Insertion**: O(log n) average
- **Range query**: O(log n + k) where k is result count
- **K-NN query**: O(log n + k)
- **Radius query**: O(log n + k)
- **Raycast (first hit)**: ~1-10μs depending on scene complexity
- **Raycast (all hits)**: ~8-50μs depending on ray length and density
- **Memory**: ~100-200 bytes per point (depends on data type and tree depth)

See `benchmarks/` for detailed performance analysis.

## Examples

See the `examples/` directory for complete examples:

- `basic_usage.cpp` - Basic operations and queries
- `spatial_queries.cpp` - Advanced spatial query examples
- `custom_data.cpp` - Using custom data types
- `raycast_example.cpp` - Raycasting for collision detection and line-of-sight
- `voxel_pathfinding.cpp` - A* pathfinding implementation for voxel games
- `frustum_culling.cpp` - Frustum and occlusion culling for efficient rendering

## Voxel Game Applications

The octree is particularly well-suited for voxel games (Minecraft-like). Here's how to use it:

### A* Pathfinding

Implement pathfinding using the octree for neighbor queries:

```cpp
// Find walkable neighbors using queryRange
AABB<double> neighborBox(
    Point3D<double>(pos.x - gridSize, pos.y - gridSize, pos.z - gridSize),
    Point3D<double>(pos.x + gridSize, pos.y + gridSize, pos.z + gridSize)
);
std::vector<std::pair<Point3D<double>, VoxelType>> neighbors;
world.queryRange(neighborBox, neighbors);

// Use A* algorithm with octree lookups
// See examples/voxel_pathfinding.cpp for complete implementation
```

**Performance**: 6-connectivity pathfinding in 10x10 grid completes in <1ms

### Frustum & Occlusion Culling

Optimize rendering by culling invisible chunks:

```cpp
// 1. Frustum culling - only render what camera can see
AABB<double> frustum = createFrustumAABB(camera);
std::vector<std::pair<Point3D<double>, Chunk>> visible;
world.queryRange(frustum, visible);

// 2. Distance culling - render distance limit
world.queryRadius(camera.position, renderDistance, visible);

// 3. Occlusion culling - skip chunks behind walls
for (auto& chunk : visible) {
    if (!isOccluded(world, camera.position, chunk.position)) {
        render(chunk);
    }
}
```

**Performance**: Culling pipeline reduces render load by 90%+ in typical scenes

**See**: `examples/frustum_culling.cpp` for complete implementation

## Testing

The library includes comprehensive tests covering:

- Point and AABB operations
- Insertion and containment
- All query types
- Edge cases and boundary conditions
- Large datasets
- Different data types

Run tests with:
```bash
cd build
ctest --output-on-failure
```

## License

MIT License - See LICENSE file for details

## Contributing

Contributions are welcome! See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

Quick start:
1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

All contributions must pass CI checks (builds on Linux/macOS/Windows, all tests pass).

## References

- [Octree - Wikipedia](https://en.wikipedia.org/wiki/Octree)
- [Spatial Partitioning](https://en.wikipedia.org/wiki/Space_partitioning)

## Support

For issues, questions, or contributions, please use the GitHub issue tracker.
