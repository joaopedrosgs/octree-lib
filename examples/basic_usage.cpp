#include <octree/octree.hpp>
#include <iostream>

using namespace octree;

int main() {
    std::cout << "=== Octree Basic Usage Example ===\n\n";

    // 1. Create an octree with a bounding box
    AABB<double> boundary(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(100.0, 100.0, 100.0)
    );

    // Parameters: boundary, max points per node, max depth
    Octree<double, int> tree(boundary, 8, 8);

    std::cout << "Created octree with boundary: "
              << "[(" << boundary.min.x << "," << boundary.min.y << "," << boundary.min.z << "), "
              << "(" << boundary.max.x << "," << boundary.max.y << "," << boundary.max.z << ")]\n\n";

    // 2. Insert points
    std::cout << "Inserting points...\n";
    tree.insert(Point3D<double>(10.0, 10.0, 10.0), 1);
    tree.insert(Point3D<double>(20.0, 20.0, 20.0), 2);
    tree.insert(Point3D<double>(30.0, 30.0, 30.0), 3);
    tree.insert(Point3D<double>(80.0, 80.0, 80.0), 4);
    tree.insert(Point3D<double>(90.0, 90.0, 90.0), 5);

    std::cout << "Inserted " << tree.size() << " points\n\n";

    // 3. Check if a point exists
    Point3D<double> searchPoint(20.0, 20.0, 20.0);
    if (tree.contains(searchPoint)) {
        std::cout << "Point (" << searchPoint.x << "," << searchPoint.y << "," << searchPoint.z
                  << ") exists in the tree\n";
    }

    // 4. Range query
    std::cout << "\n=== Range Query ===\n";
    AABB<double> queryRange(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(40.0, 40.0, 40.0)
    );

    std::vector<std::pair<Point3D<double>, int>> results;
    tree.queryRange(queryRange, results);

    std::cout << "Found " << results.size() << " points in range:\n";
    for (const auto& [point, data] : results) {
        std::cout << "  Point (" << point.x << "," << point.y << "," << point.z
                  << ") with data: " << data << "\n";
    }

    // 5. K-nearest neighbors
    std::cout << "\n=== K-Nearest Neighbors ===\n";
    Point3D<double> queryPoint(50.0, 50.0, 50.0);
    results.clear();
    tree.queryKNearest(queryPoint, 3, results);

    std::cout << "3 nearest neighbors to (" << queryPoint.x << "," << queryPoint.y << "," << queryPoint.z << "):\n";
    for (const auto& [point, data] : results) {
        double dist = queryPoint.distance(point);
        std::cout << "  Point (" << point.x << "," << point.y << "," << point.z
                  << ") with data: " << data << ", distance: " << dist << "\n";
    }

    // 6. Radius query
    std::cout << "\n=== Radius Query ===\n";
    double radius = 30.0;
    results.clear();
    tree.queryRadius(queryPoint, radius, results);

    std::cout << "Points within radius " << radius << " of (" << queryPoint.x << "," << queryPoint.y << "," << queryPoint.z << "):\n";
    for (const auto& [point, data] : results) {
        double dist = queryPoint.distance(point);
        std::cout << "  Point (" << point.x << "," << point.y << "," << point.z
                  << ") with data: " << data << ", distance: " << dist << "\n";
    }

    // 7. Traverse all points
    std::cout << "\n=== Traverse All Points ===\n";
    int count = 0;
    tree.traverse([&count](const Point3D<double>& point, const int& data) {
        std::cout << "  Point " << ++count << ": (" << point.x << "," << point.y << "," << point.z
                  << ") with data: " << data << "\n";
    });

    // 8. Memory statistics
    std::cout << "\n=== Memory Statistics ===\n";
    auto stats = tree.getMemoryStats();
    std::cout << "Node count: " << stats.nodeCount << "\n";
    std::cout << "Point count: " << stats.pointCount << "\n";
    std::cout << "Total bytes: " << stats.totalBytes << "\n";
    std::cout << "Bytes per point: " << (stats.totalBytes / stats.pointCount) << "\n";

    // 9. Clear the tree
    std::cout << "\n=== Clear Tree ===\n";
    tree.clear();
    std::cout << "Tree cleared. Size: " << tree.size() << "\n";

    return 0;
}
