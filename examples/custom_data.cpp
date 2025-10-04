#include <octree/octree.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace octree;

// Example 1: Simple struct
struct Particle {
    int id;
    double mass;
    double velocity[3];
};

// Example 2: More complex struct
struct GameObject {
    std::string name;
    int health;
    double rotation;

    GameObject() : name(""), health(0), rotation(0.0) {}
    GameObject(const std::string& n, int h, double r)
        : name(n), health(h), rotation(r) {}
};

// Example 3: Class with methods
class Entity {
public:
    Entity(int id, const std::string& type, bool active)
        : id_(id), type_(type), active_(active) {}

    int getId() const { return id_; }
    std::string getType() const { return type_; }
    bool isActive() const { return active_; }
    void setActive(bool active) { active_ = active; }

private:
    int id_;
    std::string type_;
    bool active_;
};

int main() {
    std::cout << "=== Octree Custom Data Types Example ===\n\n";

    // Setup common boundary
    AABB<double> boundary(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(100.0, 100.0, 100.0)
    );

    // Example 1: Using simple structs
    std::cout << "=== Example 1: Particle System ===\n";
    Octree<double, Particle> particleTree(boundary);

    // Insert particles
    for (int i = 0; i < 5; ++i) {
        Particle p;
        p.id = i;
        p.mass = 1.0 + i * 0.5;
        p.velocity[0] = i * 0.1;
        p.velocity[1] = i * 0.2;
        p.velocity[2] = i * 0.3;

        Point3D<double> position(i * 20.0, i * 15.0, i * 10.0);
        particleTree.insert(position, p);
    }

    std::cout << "Inserted " << particleTree.size() << " particles\n";

    // Query nearby particles
    Point3D<double> queryPos(40.0, 40.0, 40.0);
    std::vector<std::pair<Point3D<double>, Particle>> nearbyParticles;
    particleTree.queryKNearest(queryPos, 3, nearbyParticles);

    std::cout << "\n3 nearest particles to (40, 40, 40):\n";
    for (const auto& [pos, particle] : nearbyParticles) {
        std::cout << "  Particle " << particle.id
                  << " at (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n"
                  << "    Mass: " << particle.mass
                  << ", Velocity: (" << particle.velocity[0] << ", "
                  << particle.velocity[1] << ", " << particle.velocity[2] << ")\n";
    }

    // Example 2: Game objects
    std::cout << "\n=== Example 2: Game Objects ===\n";
    Octree<double, GameObject> gameTree(boundary);

    // Insert game objects
    gameTree.insert(Point3D<double>(10.0, 10.0, 10.0),
                   GameObject("Player", 100, 0.0));
    gameTree.insert(Point3D<double>(30.0, 30.0, 30.0),
                   GameObject("Enemy1", 50, 45.0));
    gameTree.insert(Point3D<double>(50.0, 50.0, 50.0),
                   GameObject("Enemy2", 50, 90.0));
    gameTree.insert(Point3D<double>(70.0, 70.0, 70.0),
                   GameObject("Powerup", 0, 0.0));

    std::cout << "Inserted " << gameTree.size() << " game objects\n";

    // Find objects in range
    AABB<double> visibleRegion(
        Point3D<double>(20.0, 20.0, 20.0),
        Point3D<double>(60.0, 60.0, 60.0)
    );

    std::vector<std::pair<Point3D<double>, GameObject>> visibleObjects;
    gameTree.queryRange(visibleRegion, visibleObjects);

    std::cout << "\nGame objects in visible region:\n";
    for (const auto& [pos, obj] : visibleObjects) {
        std::cout << "  " << obj.name
                  << " at (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n"
                  << "    Health: " << obj.health
                  << ", Rotation: " << obj.rotation << " degrees\n";
    }

    // Example 3: Using classes
    std::cout << "\n=== Example 3: Entity System ===\n";
    Octree<double, Entity> entityTree(boundary);

    // Insert entities
    entityTree.insert(Point3D<double>(15.0, 15.0, 15.0),
                     Entity(1, "NPC", true));
    entityTree.insert(Point3D<double>(45.0, 45.0, 45.0),
                     Entity(2, "Monster", true));
    entityTree.insert(Point3D<double>(75.0, 75.0, 75.0),
                     Entity(3, "Item", false));
    entityTree.insert(Point3D<double>(85.0, 85.0, 85.0),
                     Entity(4, "Monster", true));

    std::cout << "Inserted " << entityTree.size() << " entities\n";

    // Find active entities near a point
    Point3D<double> playerPos(50.0, 50.0, 50.0);
    double searchRadius = 50.0;

    std::vector<std::pair<Point3D<double>, Entity>> nearbyEntities;
    entityTree.queryRadius(playerPos, searchRadius, nearbyEntities);

    std::cout << "\nActive entities within " << searchRadius << " units of player:\n";
    for (const auto& [pos, entity] : nearbyEntities) {
        if (entity.isActive()) {
            double distance = playerPos.distance(pos);
            std::cout << "  Entity #" << entity.getId()
                      << " (" << entity.getType() << ")\n"
                      << "    Distance: " << distance << " units\n";
        }
    }

    // Example 4: Using with primitive types
    std::cout << "\n=== Example 4: Float coordinates with string data ===\n";
    AABB<float> floatBoundary(
        Point3D<float>(0.0f, 0.0f, 0.0f),
        Point3D<float>(10.0f, 10.0f, 10.0f)
    );

    Octree<float, std::string> labelTree(floatBoundary);

    labelTree.insert(Point3D<float>(1.5f, 2.3f, 3.7f), "Point A");
    labelTree.insert(Point3D<float>(4.2f, 5.8f, 6.1f), "Point B");
    labelTree.insert(Point3D<float>(7.9f, 8.3f, 9.2f), "Point C");

    std::cout << "Inserted " << labelTree.size() << " labeled points\n";

    std::vector<std::pair<Point3D<float>, std::string>> allLabels;
    labelTree.queryRange(floatBoundary, allLabels);

    std::cout << "\nAll labeled points:\n";
    for (const auto& [pos, label] : allLabels) {
        std::cout << "  \"" << label << "\" at ("
                  << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
    }

    // Example 5: Traverse with custom operation
    std::cout << "\n=== Example 5: Custom Operations ===\n";

    int totalHealth = 0;
    gameTree.traverse([&totalHealth](const Point3D<double>& pos, const GameObject& obj) {
        (void)pos;
        totalHealth += obj.health;
    });

    std::cout << "Total health of all game objects: " << totalHealth << "\n";

    // Count active entities
    int activeCount = 0;
    entityTree.traverse([&activeCount](const Point3D<double>& pos, const Entity& entity) {
        (void)pos;
        if (entity.isActive()) {
            ++activeCount;
        }
    });

    std::cout << "Active entities: " << activeCount << " / " << entityTree.size() << "\n";

    // Memory comparison
    std::cout << "\n=== Memory Usage Comparison ===\n";
    auto particleStats = particleTree.getMemoryStats();
    auto gameStats = gameTree.getMemoryStats();
    auto entityStats = entityTree.getMemoryStats();

    std::cout << "Particle tree (simple struct): "
              << particleStats.totalBytes << " bytes, "
              << (particleStats.totalBytes / particleStats.pointCount) << " bytes/point\n";
    std::cout << "Game object tree (with strings): "
              << gameStats.totalBytes << " bytes, "
              << (gameStats.totalBytes / gameStats.pointCount) << " bytes/point\n";
    std::cout << "Entity tree (class): "
              << entityStats.totalBytes << " bytes, "
              << (entityStats.totalBytes / entityStats.pointCount) << " bytes/point\n";

    return 0;
}
