#include <octree/octree.hpp>
#include <iostream>
#include <random>
#include <iomanip>

using namespace octree;

int main() {
    std::cout << "=== Octree Spatial Queries Example ===\n\n";

    // Create a large octree
    AABB<double> boundary(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(1000.0, 1000.0, 1000.0)
    );

    Octree<double, int> tree(boundary);

    // Generate random points
    std::cout << "Generating random points...\n";
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(0.0, 1000.0);

    const int numPoints = 10000;
    for (int i = 0; i < numPoints; ++i) {
        Point3D<double> p(dist(rng), dist(rng), dist(rng));
        tree.insert(p, i);
    }

    std::cout << "Inserted " << tree.size() << " points\n\n";

    // Query 1: Find points in a specific region
    std::cout << "=== Query 1: Region Search ===\n";
    AABB<double> region(
        Point3D<double>(400.0, 400.0, 400.0),
        Point3D<double>(600.0, 600.0, 600.0)
    );

    std::vector<std::pair<Point3D<double>, int>> results;
    tree.queryRange(region, results);

    std::cout << "Found " << results.size() << " points in region [400-600, 400-600, 400-600]\n";
    std::cout << "First 5 points:\n";
    for (size_t i = 0; i < std::min(size_t(5), results.size()); ++i) {
        const auto& [point, data] = results[i];
        std::cout << "  Point " << data << ": ("
                  << std::setprecision(2) << std::fixed
                  << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    // Query 2: Find nearest neighbors to the center
    std::cout << "\n=== Query 2: Nearest Neighbors ===\n";
    Point3D<double> center(500.0, 500.0, 500.0);
    results.clear();

    tree.queryKNearest(center, 10, results);

    std::cout << "10 nearest neighbors to center (500, 500, 500):\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& [point, data] = results[i];
        double distance = center.distance(point);
        std::cout << "  " << (i + 1) << ". Point " << data << " at distance "
                  << std::setprecision(2) << std::fixed << distance
                  << " (" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    // Query 3: Find all points within a radius
    std::cout << "\n=== Query 3: Radius Search ===\n";
    Point3D<double> queryPoint(100.0, 100.0, 100.0);
    double radius = 50.0;
    results.clear();

    tree.queryRadius(queryPoint, radius, results);

    std::cout << "Found " << results.size() << " points within radius " << radius
              << " of (100, 100, 100)\n";

    if (!results.empty()) {
        std::cout << "Sample points:\n";
        for (size_t i = 0; i < std::min(size_t(5), results.size()); ++i) {
            const auto& [point, data] = results[i];
            double distance = queryPoint.distance(point);
            std::cout << "  Point " << data << " at distance "
                      << std::setprecision(2) << std::fixed << distance << "\n";
        }
    }

    // Query 4: Multiple radius queries to compare
    std::cout << "\n=== Query 4: Radius Comparison ===\n";
    Point3D<double> origin(500.0, 500.0, 500.0);

    for (double r : {25.0, 50.0, 100.0, 200.0}) {
        results.clear();
        tree.queryRadius(origin, r, results);
        std::cout << "Radius " << std::setw(6) << r << ": "
                  << std::setw(5) << results.size() << " points\n";
    }

    // Query 5: Octant-based queries
    std::cout << "\n=== Query 5: Octant Queries ===\n";
    Point3D<double> treeCenter = boundary.center();

    // Query each octant
    const char* octantNames[] = {
        "Lower-Left-Front", "Lower-Right-Front",
        "Lower-Left-Back", "Lower-Right-Back",
        "Upper-Left-Front", "Upper-Right-Front",
        "Upper-Left-Back", "Upper-Right-Back"
    };

    std::array<AABB<double>, 8> octants = {{
        AABB<double>(boundary.min, treeCenter),
        AABB<double>(Point3D<double>(treeCenter.x, boundary.min.y, boundary.min.z),
                     Point3D<double>(boundary.max.x, treeCenter.y, treeCenter.z)),
        AABB<double>(Point3D<double>(boundary.min.x, treeCenter.y, boundary.min.z),
                     Point3D<double>(treeCenter.x, boundary.max.y, treeCenter.z)),
        AABB<double>(Point3D<double>(treeCenter.x, treeCenter.y, boundary.min.z),
                     Point3D<double>(boundary.max.x, boundary.max.y, treeCenter.z)),
        AABB<double>(Point3D<double>(boundary.min.x, boundary.min.y, treeCenter.z),
                     Point3D<double>(treeCenter.x, treeCenter.y, boundary.max.z)),
        AABB<double>(Point3D<double>(treeCenter.x, boundary.min.y, treeCenter.z),
                     Point3D<double>(boundary.max.x, treeCenter.y, boundary.max.z)),
        AABB<double>(Point3D<double>(boundary.min.x, treeCenter.y, treeCenter.z),
                     Point3D<double>(treeCenter.x, boundary.max.y, boundary.max.z)),
        AABB<double>(treeCenter, boundary.max)
    }};

    for (size_t i = 0; i < 8; ++i) {
        results.clear();
        tree.queryRange(octants[i], results);
        std::cout << std::setw(20) << std::left << octantNames[i]
                  << ": " << std::setw(5) << std::right << results.size() << " points\n";
    }

    // Memory statistics
    std::cout << "\n=== Memory Statistics ===\n";
    auto stats = tree.getMemoryStats();
    std::cout << "Total nodes: " << stats.nodeCount << "\n";
    std::cout << "Total points: " << stats.pointCount << "\n";
    std::cout << "Memory used: " << stats.totalBytes << " bytes ("
              << (stats.totalBytes / 1024.0) << " KB)\n";
    std::cout << "Bytes per point: " << (stats.totalBytes / stats.pointCount) << "\n";

    return 0;
}
