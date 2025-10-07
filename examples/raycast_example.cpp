#include <octree/octree.hpp>
#include <iostream>
#include <iomanip>

using namespace octree;

int main() {
    std::cout << "=== Octree Raycast Example ===\n\n";

    // Create octree with voxel world
    AABB<double> boundary(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(100.0, 100.0, 100.0)
    );

    Octree<double, std::string> world(boundary);

    // Place some blocks in the world
    world.insert(Point3D<double>(50.0, 10.0, 10.0), "Wall");
    world.insert(Point3D<double>(30.0, 10.0, 10.0), "Door");
    world.insert(Point3D<double>(70.0, 10.0, 10.0), "Window");
    world.insert(Point3D<double>(10.0, 50.0, 10.0), "Ceiling");
    world.insert(Point3D<double>(10.0, 10.0, 50.0), "Floor");

    std::cout << "Created world with " << world.size() << " objects\n\n";

    // Example 1: Simple raycast - looking straight ahead
    std::cout << "=== Example 1: Looking Straight Ahead ===\n";
    Point3D<double> playerPos(0.0, 10.0, 10.0);
    Point3D<double> lookDirection(1.0, 0.0, 0.0);  // Looking along X axis
    Ray3D<double> ray(playerPos, lookDirection);

    std::pair<Point3D<double>, std::string> hit;
    if (world.raycastFirst(ray, 100.0, hit)) {
        double distance = playerPos.distance(hit.first);
        std::cout << "Hit: " << hit.second << " at ("
                  << hit.first.x << ", " << hit.first.y << ", " << hit.first.z << ")\n";
        std::cout << "Distance: " << std::fixed << std::setprecision(2) << distance << " units\n";
    } else {
        std::cout << "No hit\n";
    }

    // Example 2: Raycast in different direction
    std::cout << "\n=== Example 2: Looking Up ===\n";
    playerPos = Point3D<double>(10.0, 0.0, 10.0);
    lookDirection = Point3D<double>(0.0, 1.0, 0.0);  // Looking up (Y axis)
    ray = Ray3D<double>(playerPos, lookDirection);

    if (world.raycastFirst(ray, 100.0, hit)) {
        double distance = playerPos.distance(hit.first);
        std::cout << "Hit: " << hit.second << " at ("
                  << hit.first.x << ", " << hit.first.y << ", " << hit.first.z << ")\n";
        std::cout << "Distance: " << std::fixed << std::setprecision(2) << distance << " units\n";
    } else {
        std::cout << "No hit\n";
    }

    // Example 3: Find all objects along a ray
    std::cout << "\n=== Example 3: Find All Objects Along Ray ===\n";
    playerPos = Point3D<double>(0.0, 10.0, 10.0);
    lookDirection = Point3D<double>(1.0, 0.0, 0.0);
    ray = Ray3D<double>(playerPos, lookDirection);

    std::vector<std::pair<Point3D<double>, std::string>> allHits;
    world.raycastAll(ray, 100.0, allHits);

    std::cout << "Found " << allHits.size() << " objects along the ray:\n";
    for (const auto& [pos, name] : allHits) {
        double distance = playerPos.distance(pos);
        std::cout << "  - " << name << " at distance " << std::fixed
                  << std::setprecision(2) << distance << " units\n";
    }

    // Example 4: Limited distance raycast
    std::cout << "\n=== Example 4: Limited Distance (max 40 units) ===\n";
    playerPos = Point3D<double>(0.0, 10.0, 10.0);
    lookDirection = Point3D<double>(1.0, 0.0, 0.0);
    ray = Ray3D<double>(playerPos, lookDirection);

    if (world.raycastFirst(ray, 40.0, hit)) {  // Only look 40 units ahead
        double distance = playerPos.distance(hit.first);
        std::cout << "Hit: " << hit.second << " at distance "
                  << std::fixed << std::setprecision(2) << distance << " units\n";
    } else {
        std::cout << "No hit within 40 units\n";
    }

    // Example 5: Diagonal raycast
    std::cout << "\n=== Example 5: Diagonal Raycast ===\n";
    playerPos = Point3D<double>(0.0, 0.0, 0.0);
    lookDirection = Point3D<double>(1.0, 1.0, 1.0);  // 45-degree angle
    ray = Ray3D<double>(playerPos, lookDirection);

    if (world.raycastFirst(ray, 100.0, hit)) {
        double distance = playerPos.distance(hit.first);
        std::cout << "Hit: " << hit.second << " at ("
                  << hit.first.x << ", " << hit.first.y << ", " << hit.first.z << ")\n";
        std::cout << "Distance: " << std::fixed << std::setprecision(2) << distance << " units\n";
    } else {
        std::cout << "No hit\n";
    }

    // Example 6: Game scenario - Player mining
    std::cout << "\n=== Example 6: Mining Simulation ===\n";

    // Create a voxel grid
    Octree<double, int> voxelWorld(boundary);

    // Fill with voxel blocks
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            for (int z = 0; z < 10; ++z) {
                int blockType = (y == 0) ? 1 : 0;  // 1 = stone, 0 = air
                if (blockType > 0) {
                    voxelWorld.insert(
                        Point3D<double>(x * 10.0, y * 10.0, z * 10.0),
                        blockType
                    );
                }
            }
        }
    }

    std::cout << "Voxel world created with " << voxelWorld.size() << " blocks\n";

    // Player trying to mine a block
    Point3D<double> playerPosition(45.0, 20.0, 45.0);
    Point3D<double> aimDirection(0.0, -1.0, 0.0);  // Looking down
    Ray3D<double> miningRay(playerPosition, aimDirection);

    std::pair<Point3D<double>, int> targetBlock;
    if (voxelWorld.raycastFirst(miningRay, 50.0, targetBlock)) {
        std::cout << "Player can mine block of type " << targetBlock.second
                  << " at position (" << targetBlock.first.x << ", "
                  << targetBlock.first.y << ", " << targetBlock.first.z << ")\n";
        std::cout << "Mining distance: "
                  << std::fixed << std::setprecision(2)
                  << playerPosition.distance(targetBlock.first) << " units\n";
    } else {
        std::cout << "No block in mining range\n";
    }

    // Example 7: 360-degree scan
    std::cout << "\n=== Example 7: 360-Degree Scan ===\n";
    Point3D<double> scanPos(50.0, 10.0, 50.0);
    int numRays = 8;
    int hitsFound = 0;

    std::cout << "Scanning around position (" << scanPos.x << ", "
              << scanPos.y << ", " << scanPos.z << ")...\n";

    for (int i = 0; i < numRays; ++i) {
        double angle = (2.0 * 3.14159 * i) / numRays;
        Point3D<double> direction(std::cos(angle), 0.0, std::sin(angle));
        Ray3D<double> scanRay(scanPos, direction);

        if (world.raycastFirst(scanRay, 50.0, hit)) {
            ++hitsFound;
            std::cout << "  Direction " << (i * 360 / numRays) << "°: "
                      << hit.second << " at distance "
                      << std::fixed << std::setprecision(2)
                      << scanPos.distance(hit.first) << " units\n";
        }
    }

    std::cout << "Total hits: " << hitsFound << "/" << numRays << "\n";

    return 0;
}
