#include <octree/octree.hpp>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <chrono>

using namespace octree;

// Voxel types
enum VoxelType {
    AIR = 0,
    SOLID = 1,
    WALKABLE = 2
};

// Hash function for Point3D to use in unordered containers
struct Point3DHash {
    size_t operator()(const Point3D<double>& p) const {
        size_t h1 = std::hash<double>{}(p.x);
        size_t h2 = std::hash<double>{}(p.y);
        size_t h3 = std::hash<double>{}(p.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct Point3DEqual {
    bool operator()(const Point3D<double>& a, const Point3D<double>& b) const {
        return a == b;
    }
};

// A* Node
struct AStarNode {
    Point3D<double> position;
    double gCost;  // Cost from start
    double hCost;  // Heuristic cost to end
    double fCost;  // gCost + hCost
    Point3D<double> parent;

    AStarNode() : position(0, 0, 0), gCost(0), hCost(0), fCost(0), parent(-1, -1, -1) {}

    AStarNode(const Point3D<double>& pos, double g, double h, const Point3D<double>& p)
        : position(pos), gCost(g), hCost(h), fCost(g + h), parent(p) {}

    bool operator>(const AStarNode& other) const {
        return fCost > other.fCost;
    }
};

// Manhattan distance heuristic for voxel grids
double manhattanDistance(const Point3D<double>& a, const Point3D<double>& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
}

// Euclidean distance heuristic
double euclideanDistance(const Point3D<double>& a, const Point3D<double>& b) {
    return a.distance(b);
}

// Get 6-connected neighbors (cardinal directions)
std::vector<Point3D<double>> getNeighbors6(const Point3D<double>& pos, double gridSize) {
    return {
        Point3D<double>(pos.x + gridSize, pos.y, pos.z),
        Point3D<double>(pos.x - gridSize, pos.y, pos.z),
        Point3D<double>(pos.x, pos.y + gridSize, pos.z),
        Point3D<double>(pos.x, pos.y - gridSize, pos.z),
        Point3D<double>(pos.x, pos.y, pos.z + gridSize),
        Point3D<double>(pos.x, pos.y, pos.z - gridSize)
    };
}

// Get 26-connected neighbors (including diagonals)
std::vector<Point3D<double>> getNeighbors26(const Point3D<double>& pos, double gridSize) {
    std::vector<Point3D<double>> neighbors;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                neighbors.push_back(Point3D<double>(
                    pos.x + dx * gridSize,
                    pos.y + dy * gridSize,
                    pos.z + dz * gridSize
                ));
            }
        }
    }

    return neighbors;
}

// Check if a voxel position is walkable
bool isWalkable(const Octree<double, VoxelType>& world, const Point3D<double>& pos) {
    std::pair<Point3D<double>, VoxelType> result;

    // Check if there's a voxel at this position
    Ray3D<double> ray(pos, Point3D<double>(0, 1, 0));  // Cast upward
    if (world.raycastFirst(ray, 0.1, result)) {
        if (result.first == pos) {
            return result.second == WALKABLE || result.second == AIR;
        }
    }

    // If no voxel found, check with contains
    std::vector<std::pair<Point3D<double>, VoxelType>> nearby;
    AABB<double> checkBox(
        Point3D<double>(pos.x - 0.1, pos.y - 0.1, pos.z - 0.1),
        Point3D<double>(pos.x + 0.1, pos.y + 0.1, pos.z + 0.1)
    );
    world.queryRange(checkBox, nearby);

    for (const auto& [p, type] : nearby) {
        if (p == pos) {
            return type == WALKABLE || type == AIR;
        }
    }

    return true;  // Empty space is walkable
}

// A* pathfinding algorithm
std::vector<Point3D<double>> findPath(
    const Octree<double, VoxelType>& world,
    const Point3D<double>& start,
    const Point3D<double>& goal,
    double gridSize,
    bool use26Connectivity = false)
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
    std::unordered_set<Point3D<double>, Point3DHash, Point3DEqual> closedSet;
    std::unordered_map<Point3D<double>, AStarNode, Point3DHash, Point3DEqual> allNodes;

    // Initialize start node
    AStarNode startNode(start, 0, manhattanDistance(start, goal), Point3D<double>(-1, -1, -1));
    openSet.push(startNode);
    allNodes[start] = startNode;

    int nodesExplored = 0;

    while (!openSet.empty()) {
        AStarNode current = openSet.top();
        openSet.pop();

        // Goal reached
        if (current.position == goal) {
            std::cout << "Path found! Nodes explored: " << nodesExplored << "\n";

            // Reconstruct path
            std::vector<Point3D<double>> path;
            Point3D<double> pos = current.position;

            while (!(pos.x == -1 && pos.y == -1 && pos.z == -1)) {
                path.push_back(pos);
                pos = allNodes[pos].parent;
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        // Skip if already processed
        if (closedSet.count(current.position)) continue;
        closedSet.insert(current.position);
        nodesExplored++;

        // Get neighbors
        auto neighbors = use26Connectivity ?
            getNeighbors26(current.position, gridSize) :
            getNeighbors6(current.position, gridSize);

        for (const auto& neighborPos : neighbors) {
            // Skip if out of bounds
            if (!world.boundary().contains(neighborPos)) continue;

            // Skip if already processed
            if (closedSet.count(neighborPos)) continue;

            // Skip if not walkable
            if (!isWalkable(world, neighborPos)) continue;

            // Calculate costs
            double moveCost = (use26Connectivity &&
                              (neighborPos.x != current.position.x) +
                              (neighborPos.y != current.position.y) +
                              (neighborPos.z != current.position.z) > 1) ?
                              1.414 : 1.0;  // Diagonal cost

            double newGCost = current.gCost + moveCost;
            double hCost = manhattanDistance(neighborPos, goal);

            // Check if this is a better path
            if (allNodes.find(neighborPos) == allNodes.end() ||
                newGCost < allNodes[neighborPos].gCost) {

                AStarNode neighborNode(neighborPos, newGCost, hCost, current.position);
                allNodes[neighborPos] = neighborNode;
                openSet.push(neighborNode);
            }
        }
    }

    std::cout << "No path found! Nodes explored: " << nodesExplored << "\n";
    return {};  // No path found
}

int main() {
    std::cout << "=== Voxel Pathfinding Example with A* ===\n\n";

    // Create voxel world
    AABB<double> boundary(
        Point3D<double>(0, 0, 0),
        Point3D<double>(100, 100, 100)
    );

    Octree<double, VoxelType> world(boundary);
    double gridSize = 10.0;

    // Create a simple maze/obstacle course
    std::cout << "Building voxel world...\n";

    // Floor (walkable)
    for (int x = 0; x < 10; ++x) {
        for (int z = 0; z < 10; ++z) {
            world.insert(Point3D<double>(x * gridSize, 0, z * gridSize), WALKABLE);
        }
    }

    // Walls (obstacles)
    for (int z = 2; z < 8; ++z) {
        world.insert(Point3D<double>(5 * gridSize, 0, z * gridSize), SOLID);
    }

    // Another wall
    for (int x = 2; x < 5; ++x) {
        world.insert(Point3D<double>(x * gridSize, 0, 3 * gridSize), SOLID);
    }

    std::cout << "World created with " << world.size() << " voxels\n\n";

    // Example 1: Simple pathfinding
    std::cout << "=== Example 1: Basic A* Pathfinding (6-connectivity) ===\n";
    Point3D<double> start(0, 0, 0);
    Point3D<double> goal(90, 0, 90);

    auto path = findPath(world, start, goal, gridSize, false);

    if (!path.empty()) {
        std::cout << "Path length: " << path.size() << " steps\n";
        std::cout << "First 5 waypoints:\n";
        for (size_t i = 0; i < std::min(size_t(5), path.size()); ++i) {
            std::cout << "  " << i << ". (" << path[i].x << ", "
                      << path[i].y << ", " << path[i].z << ")\n";
        }
    }

    // Example 2: Pathfinding with obstacles
    std::cout << "\n=== Example 2: Navigating Around Obstacles ===\n";
    start = Point3D<double>(0, 0, 50);
    goal = Point3D<double>(90, 0, 50);

    path = findPath(world, start, goal, gridSize, false);

    if (!path.empty()) {
        std::cout << "Path length: " << path.size() << " steps\n";
        std::cout << "Path found navigating around wall\n";
    }

    // Example 3: 26-connectivity (diagonal movement)
    std::cout << "\n=== Example 3: Diagonal Movement (26-connectivity) ===\n";
    start = Point3D<double>(0, 0, 0);
    goal = Point3D<double>(90, 0, 90);

    auto path6 = findPath(world, start, goal, gridSize, false);
    auto path26 = findPath(world, start, goal, gridSize, true);

    std::cout << "6-connectivity path length: " << path6.size() << " steps\n";
    std::cout << "26-connectivity path length: " << path26.size() << " steps\n";
    std::cout << "Diagonal movement saves " << (path6.size() - path26.size()) << " steps\n";

    // Example 4: No path scenario
    std::cout << "\n=== Example 4: No Path Available ===\n";

    // Block the path completely
    for (int z = 0; z < 10; ++z) {
        world.insert(Point3D<double>(5 * gridSize, 0, z * gridSize), SOLID);
    }

    start = Point3D<double>(0, 0, 50);
    goal = Point3D<double>(90, 0, 50);

    path = findPath(world, start, goal, gridSize, false);

    if (path.empty()) {
        std::cout << "Correctly identified no path exists\n";
    }

    // Example 5: Performance test
    std::cout << "\n=== Example 5: Performance Test ===\n";

    // Clear obstacles
    world.clear();
    for (int x = 0; x < 10; ++x) {
        for (int z = 0; z < 10; ++z) {
            world.insert(Point3D<double>(x * gridSize, 0, z * gridSize), WALKABLE);
        }
    }

    start = Point3D<double>(0, 0, 0);
    goal = Point3D<double>(90, 0, 90);

    auto startTime = std::chrono::high_resolution_clock::now();
    path = findPath(world, start, goal, gridSize, false);
    auto endTime = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    std::cout << "Pathfinding time: " << duration.count() << " microseconds\n";
    std::cout << "Path length: " << path.size() << " steps\n";

    // Example 6: Multi-level pathfinding
    std::cout << "\n=== Example 6: 3D Pathfinding (Multiple Levels) ===\n";

    // Add platforms at different heights
    for (int y = 1; y <= 3; ++y) {
        for (int x = 3; x < 7; ++x) {
            world.insert(Point3D<double>(x * gridSize, y * gridSize, 50), WALKABLE);
        }
    }

    start = Point3D<double>(30, 0, 50);
    goal = Point3D<double>(60, 30, 50);

    path = findPath(world, start, goal, gridSize, true);

    if (!path.empty()) {
        std::cout << "3D path found with " << path.size() << " steps\n";
        std::cout << "Path climbs from y=0 to y=" << path.back().y << "\n";
    }

    std::cout << "\n=== Summary ===\n";
    std::cout << "A* pathfinding can be efficiently implemented using octree's:\n";
    std::cout << "- queryRange() for neighbor finding\n";
    std::cout << "- raycastFirst() for walkability checks\n";
    std::cout << "- Spatial partitioning for fast lookups\n";

    return 0;
}
