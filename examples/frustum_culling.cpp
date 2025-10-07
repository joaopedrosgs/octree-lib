#include <octree/octree.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <chrono>
#include <iomanip>

using namespace octree;

// Camera structure
struct Camera {
    Point3D<double> position;
    Point3D<double> forward;
    Point3D<double> up;
    double fov;        // Field of view in degrees
    double nearPlane;
    double farPlane;

    Camera(const Point3D<double>& pos, const Point3D<double>& fwd, double fovDeg = 90.0)
        : position(pos), forward(fwd.normalized()), fov(fovDeg), nearPlane(1.0), farPlane(100.0)
    {
        // Calculate up vector (assuming Y is up)
        Point3D<double> worldUp(0, 1, 0);
        Point3D<double> right(
            forward.z * worldUp.y - forward.y * worldUp.z,
            forward.x * worldUp.z - forward.z * worldUp.x,
            forward.y * worldUp.x - forward.x * worldUp.y
        );
        up = Point3D<double>(
            right.y * forward.z - right.z * forward.y,
            right.z * forward.x - right.x * forward.z,
            right.x * forward.y - right.y * forward.x
        ).normalized();
    }
};

// Simple frustum representation as an AABB (conservative, axis-aligned)
AABB<double> createFrustumAABB(const Camera& camera) {
    double halfFarWidth = camera.farPlane * std::tan(camera.fov * 3.14159 / 360.0);

    // Approximate frustum as AABB
    Point3D<double> farCenter = camera.position + camera.forward * camera.farPlane;

    Point3D<double> min(
        std::min(camera.position.x, farCenter.x) - halfFarWidth,
        std::min(camera.position.y, farCenter.y) - halfFarWidth,
        std::min(camera.position.z, farCenter.z) - halfFarWidth
    );

    Point3D<double> max(
        std::max(camera.position.x, farCenter.x) + halfFarWidth,
        std::max(camera.position.y, farCenter.y) + halfFarWidth,
        std::max(camera.position.z, farCenter.z) + halfFarWidth
    );

    return AABB<double>(min, max);
}

// Check if point is in frustum (simple dot product test)
bool isInFrustum(const Camera& camera, const Point3D<double>& point) {
    Point3D<double> toPoint = point - camera.position;
    double distance = toPoint.length();

    // Check distance
    if (distance < camera.nearPlane || distance > camera.farPlane) {
        return false;
    }

    // Check field of view
    double dotProduct = toPoint.normalized().dot(camera.forward);
    double halfFovRad = camera.fov * 3.14159 / 360.0;
    double minDot = std::cos(halfFovRad);

    return dotProduct >= minDot;
}

// Voxel chunk data
struct VoxelChunk {
    Point3D<double> position;
    int blockCount;
    bool visible;

    VoxelChunk() : position(0, 0, 0), blockCount(0), visible(true) {}

    VoxelChunk(const Point3D<double>& pos, int blocks)
        : position(pos), blockCount(blocks), visible(true) {}
};

// Occlusion test using raycast
bool isOccluded(const Octree<double, VoxelChunk>& world,
                const Point3D<double>& cameraPos,
                const Point3D<double>& targetPos) {
    Point3D<double> direction = (targetPos - cameraPos).normalized();
    double distance = cameraPos.distance(targetPos);
    Ray3D<double> ray(cameraPos, direction);

    std::pair<Point3D<double>, VoxelChunk> hit;
    if (world.raycastFirst(ray, distance - 1.0, hit)) {
        // Something is between camera and target
        return hit.first.distance(cameraPos) < targetPos.distance(cameraPos);
    }

    return false;
}

int main() {
    std::cout << "=== Frustum and Occlusion Culling Example ===\n\n";

    // Create voxel world
    AABB<double> worldBounds(
        Point3D<double>(0, 0, 0),
        Point3D<double>(1000, 100, 1000)
    );

    Octree<double, VoxelChunk> world(worldBounds);

    // Populate world with chunks
    std::cout << "Creating voxel chunks...\n";
    int chunkSize = 16;
    int totalChunks = 0;

    for (int x = 0; x < 1000; x += chunkSize) {
        for (int y = 0; y < 100; y += chunkSize) {
            for (int z = 0; z < 1000; z += chunkSize) {
                Point3D<double> chunkPos(x + chunkSize/2.0, y + chunkSize/2.0, z + chunkSize/2.0);
                world.insert(chunkPos, VoxelChunk(chunkPos, 4096));
                totalChunks++;
            }
        }
    }

    std::cout << "World created with " << totalChunks << " chunks\n\n";

    // Example 1: Basic frustum culling
    std::cout << "=== Example 1: Basic Frustum Culling ===\n";

    Camera camera(
        Point3D<double>(500, 50, 500),  // Center of world
        Point3D<double>(1, 0, 0),       // Looking along X axis
        90.0                            // 90 degree FOV
    );

    // Method 1: Naive - check all chunks
    auto start = std::chrono::high_resolution_clock::now();
    int visibleNaive = 0;

    world.traverse([&](const Point3D<double>& pos, const VoxelChunk& chunk) {
        (void)chunk;
        if (isInFrustum(camera, pos)) {
            visibleNaive++;
        }
    });

    auto end = std::chrono::high_resolution_clock::now();
    auto durationNaive = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Naive approach (check all " << totalChunks << " chunks):\n";
    std::cout << "  Visible chunks: " << visibleNaive << "\n";
    std::cout << "  Time: " << durationNaive.count() << " μs\n\n";

    // Method 2: Octree frustum culling with AABB
    start = std::chrono::high_resolution_clock::now();
    AABB<double> frustumAABB = createFrustumAABB(camera);
    std::vector<std::pair<Point3D<double>, VoxelChunk>> potentiallyVisible;
    world.queryRange(frustumAABB, potentiallyVisible);

    // Refine with actual frustum test
    int visibleOctree = 0;
    for (const auto& [pos, chunk] : potentiallyVisible) {
        if (isInFrustum(camera, pos)) {
            visibleOctree++;
        }
    }

    end = std::chrono::high_resolution_clock::now();
    auto durationOctree = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Octree-accelerated approach:\n";
    std::cout << "  Potentially visible (AABB): " << potentiallyVisible.size() << "\n";
    std::cout << "  Actually visible (refined): " << visibleOctree << "\n";
    std::cout << "  Time: " << durationOctree.count() << " μs\n";
    std::cout << "  Speedup: " << std::fixed << std::setprecision(1)
              << (double)durationNaive.count() / durationOctree.count() << "x\n\n";

    // Example 2: Render distance culling
    std::cout << "=== Example 2: Render Distance Culling ===\n";

    double renderDistance = 100.0;
    std::vector<std::pair<Point3D<double>, VoxelChunk>> inRange;

    start = std::chrono::high_resolution_clock::now();
    world.queryRadius(camera.position, renderDistance, inRange);
    end = std::chrono::high_resolution_clock::now();
    auto durationRadius = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Chunks within " << renderDistance << " units:\n";
    std::cout << "  Count: " << inRange.size() << "\n";
    std::cout << "  Query time: " << durationRadius.count() << " μs\n\n";

    // Example 3: Occlusion culling with raycasts
    std::cout << "=== Example 3: Occlusion Culling ===\n";

    // Create a simple occluder (wall of chunks)
    for (int y = 0; y < 100; y += chunkSize) {
        for (int z = 400; z < 600; z += chunkSize) {
            Point3D<double> occluderPos(550, y + chunkSize/2.0, z + chunkSize/2.0);
            world.insert(occluderPos, VoxelChunk(occluderPos, 4096));
        }
    }

    // Camera looking at the wall
    Camera occlusionCamera(
        Point3D<double>(500, 50, 500),
        Point3D<double>(1, 0, 0),
        90.0
    );

    // Get potentially visible chunks
    frustumAABB = createFrustumAABB(occlusionCamera);
    potentiallyVisible.clear();
    world.queryRange(frustumAABB, potentiallyVisible);

    start = std::chrono::high_resolution_clock::now();

    int visibleAfterOcclusion = 0;
    int occludedChunks = 0;

    for (const auto& [pos, chunk] : potentiallyVisible) {
        if (isInFrustum(occlusionCamera, pos)) {
            if (!isOccluded(world, occlusionCamera.position, pos)) {
                visibleAfterOcclusion++;
            } else {
                occludedChunks++;
            }
        }
    }

    end = std::chrono::high_resolution_clock::now();
    auto durationOcclusion = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "With occlusion culling:\n";
    std::cout << "  Potentially visible: " << potentiallyVisible.size() << "\n";
    std::cout << "  Actually visible: " << visibleAfterOcclusion << "\n";
    std::cout << "  Occluded: " << occludedChunks << "\n";
    std::cout << "  Culling ratio: " << std::fixed << std::setprecision(1)
              << (100.0 * occludedChunks / potentiallyVisible.size()) << "%\n";
    std::cout << "  Time: " << durationOcclusion.count() << " μs\n\n";

    // Example 4: Level-of-detail culling
    std::cout << "=== Example 4: Distance-Based LOD ===\n";

    struct LODLevel {
        double maxDistance;
        int blockCount;
    };

    std::vector<LODLevel> lodLevels = {
        {50.0, 4096},   // Full detail
        {100.0, 1024},  // Medium detail
        {200.0, 256},   // Low detail
        {300.0, 64}     // Very low detail
    };

    std::vector<std::pair<Point3D<double>, VoxelChunk>> chunks;
    world.queryRange(frustumAABB, chunks);

    std::map<int, int> lodCounts;

    for (const auto& [pos, chunk] : chunks) {
        double distance = camera.position.distance(pos);

        for (size_t i = 0; i < lodLevels.size(); ++i) {
            if (distance < lodLevels[i].maxDistance) {
                lodCounts[lodLevels[i].blockCount]++;
                break;
            }
        }
    }

    std::cout << "LOD distribution:\n";
    for (const auto& [blocks, count] : lodCounts) {
        std::cout << "  " << blocks << " blocks: " << count << " chunks\n";
    }

    // Example 5: Portal-based culling simulation
    std::cout << "\n=== Example 5: Portal Culling ===\n";

    // Define a portal (doorway)
    Point3D<double> portalCenter(550, 50, 500);
    double portalRadius = 20.0;

    // Camera on one side of portal
    Camera portalCamera(
        Point3D<double>(500, 50, 500),
        Point3D<double>(1, 0, 0),
        60.0
    );

    // Only render chunks visible through the portal
    std::vector<std::pair<Point3D<double>, VoxelChunk>> throughPortal;

    // First, check if chunks are visible through portal
    for (const auto& [pos, chunk] : potentiallyVisible) {
        // Check if ray from camera through portal hits the chunk
        Point3D<double> toPortal = (portalCenter - portalCamera.position).normalized();
        Point3D<double> toChunk = (pos - portalCamera.position).normalized();

        double angle = std::acos(toPortal.dot(toChunk));
        double portalAngle = std::atan(portalRadius / portalCamera.position.distance(portalCenter));

        if (angle < portalAngle) {
            throughPortal.push_back({pos, chunk});
        }
    }

    std::cout << "Portal culling results:\n";
    std::cout << "  Total potentially visible: " << potentiallyVisible.size() << "\n";
    std::cout << "  Visible through portal: " << throughPortal.size() << "\n";
    std::cout << "  Culled: " << (potentiallyVisible.size() - throughPortal.size()) << "\n";
    std::cout << "  Culling ratio: " << std::fixed << std::setprecision(1)
              << (100.0 * (potentiallyVisible.size() - throughPortal.size()) / potentiallyVisible.size())
              << "%\n\n";

    // Example 6: Combined culling pipeline
    std::cout << "=== Example 6: Full Culling Pipeline ===\n";

    Camera finalCamera(
        Point3D<double>(500, 50, 500),
        Point3D<double>(1, 0, 0),
        90.0
    );

    auto pipelineStart = std::chrono::high_resolution_clock::now();

    // Step 1: Frustum culling (AABB approximation)
    AABB<double> frustum = createFrustumAABB(finalCamera);
    std::vector<std::pair<Point3D<double>, VoxelChunk>> step1;
    world.queryRange(frustum, step1);

    // Step 2: Precise frustum test
    std::vector<std::pair<Point3D<double>, VoxelChunk>> step2;
    for (const auto& [pos, chunk] : step1) {
        if (isInFrustum(finalCamera, pos)) {
            step2.push_back({pos, chunk});
        }
    }

    // Step 3: Distance culling
    std::vector<std::pair<Point3D<double>, VoxelChunk>> step3;
    double maxRenderDist = 150.0;
    for (const auto& [pos, chunk] : step2) {
        if (finalCamera.position.distance(pos) < maxRenderDist) {
            step3.push_back({pos, chunk});
        }
    }

    // Step 4: Occlusion culling (sample - not all chunks)
    std::vector<std::pair<Point3D<double>, VoxelChunk>> step4;
    for (const auto& [pos, chunk] : step3) {
        if (!isOccluded(world, finalCamera.position, pos)) {
            step4.push_back({pos, chunk});
        }
    }

    auto pipelineEnd = std::chrono::high_resolution_clock::now();
    auto pipelineDuration = std::chrono::duration_cast<std::chrono::microseconds>(
        pipelineEnd - pipelineStart);

    std::cout << "Culling pipeline results:\n";
    std::cout << "  Total chunks: " << totalChunks << "\n";
    std::cout << "  After frustum AABB: " << step1.size() << " ("
              << std::fixed << std::setprecision(1)
              << (100.0 * step1.size() / totalChunks) << "%)\n";
    std::cout << "  After precise frustum: " << step2.size() << " ("
              << (100.0 * step2.size() / totalChunks) << "%)\n";
    std::cout << "  After distance culling: " << step3.size() << " ("
              << (100.0 * step3.size() / totalChunks) << "%)\n";
    std::cout << "  After occlusion culling: " << step4.size() << " ("
              << (100.0 * step4.size() / totalChunks) << "%)\n";
    std::cout << "  Final culling ratio: "
              << (100.0 * (totalChunks - step4.size()) / totalChunks) << "%\n";
    std::cout << "  Pipeline time: " << pipelineDuration.count() << " μs\n\n";

    std::cout << "=== Summary ===\n";
    std::cout << "Octree enables efficient culling through:\n";
    std::cout << "1. queryRange() - Fast frustum AABB culling\n";
    std::cout << "2. queryRadius() - Distance-based culling\n";
    std::cout << "3. raycastFirst() - Occlusion testing\n";
    std::cout << "4. Spatial partitioning - Early rejection of large regions\n\n";

    std::cout << "Typical culling pipeline for voxel games:\n";
    std::cout << "  Frustum → Distance → Occlusion → Render\n";
    std::cout << "  Can reduce render load by 90%+ in typical scenes\n";

    return 0;
}
