#include <octree/octree.hpp>
#include <benchmark/benchmark.h>
#include <random>
#include <vector>
#include <unordered_map>

using namespace octree;

// Voxel block data structure
struct VoxelBlock {
    uint8_t blockType;      // 0 = air, 1 = stone, 2 = dirt, etc.
    uint8_t metadata;       // Light level, orientation, etc.
    uint16_t durability;    // Block health/durability

    VoxelBlock() : blockType(0), metadata(0), durability(100) {}
    VoxelBlock(uint8_t type, uint8_t meta = 0, uint16_t dur = 100)
        : blockType(type), metadata(meta), durability(dur) {}
};

// Chunk configuration
struct ChunkConfig {
    int chunkSizeX = 16;
    int chunkSizeY = 16;
    int chunkSizeZ = 16;
    int worldChunksX = 16;  // Number of chunks in X
    int worldChunksY = 4;   // Number of chunks in Y (height)
    int worldChunksZ = 16;  // Number of chunks in Z
};

// Generate a chunk of voxel data
std::vector<std::pair<Point3D<double>, VoxelBlock>> generateChunk(
    int chunkX, int chunkY, int chunkZ,
    const ChunkConfig& config,
    std::mt19937& rng)
{
    std::vector<std::pair<Point3D<double>, VoxelBlock>> blocks;
    std::uniform_real_distribution<double> blockDist(0.0, 1.0);

    int baseX = chunkX * config.chunkSizeX;
    int baseY = chunkY * config.chunkSizeY;
    int baseZ = chunkZ * config.chunkSizeZ;

    for (int x = 0; x < config.chunkSizeX; ++x) {
        for (int y = 0; y < config.chunkSizeY; ++y) {
            for (int z = 0; z < config.chunkSizeZ; ++z) {
                // Skip air blocks (sparse voxel world)
                double density = blockDist(rng);

                // Simulate terrain: more solid at bottom, more air at top
                double heightFactor = 1.0 - (static_cast<double>(baseY + y) /
                                             (config.worldChunksY * config.chunkSizeY));

                if (density < heightFactor * 0.7) {  // 70% solid at bottom, less at top
                    Point3D<double> pos(
                        static_cast<double>(baseX + x),
                        static_cast<double>(baseY + y),
                        static_cast<double>(baseZ + z)
                    );

                    // Determine block type based on height
                    uint8_t blockType = 1;  // stone
                    if (baseY + y > config.worldChunksY * config.chunkSizeY * 0.8) {
                        blockType = 3;  // grass
                    } else if (baseY + y > config.worldChunksY * config.chunkSizeY * 0.6) {
                        blockType = 2;  // dirt
                    }

                    blocks.emplace_back(pos, VoxelBlock(blockType));
                }
            }
        }
    }

    return blocks;
}

// Benchmark: Load chunks progressively and track memory
static void BM_VoxelWorld_LoadChunks(benchmark::State& state) {
    const int numChunksToLoad = static_cast<int>(state.range(0));
    ChunkConfig config;
    config.chunkSizeX = 16;
    config.chunkSizeY = 16;
    config.chunkSizeZ = 16;

    std::mt19937 rng(42);

    // Create world boundary
    int worldSizeX = config.chunkSizeX * config.worldChunksX;
    int worldSizeY = config.chunkSizeY * config.worldChunksY;
    int worldSizeZ = config.chunkSizeZ * config.worldChunksZ;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    for (auto _ : state) {
        state.PauseTiming();
        Octree<double, VoxelBlock> voxelWorld(worldBounds, 64, 10);  // 64 blocks per node
        state.ResumeTiming();

        // Load chunks in spiral pattern (like a player walking)
        int chunksLoaded = 0;
        for (int radius = 0; radius < 10 && chunksLoaded < numChunksToLoad; ++radius) {
            for (int dx = -radius; dx <= radius && chunksLoaded < numChunksToLoad; ++dx) {
                for (int dz = -radius; dz <= radius && chunksLoaded < numChunksToLoad; ++dz) {
                    if (std::abs(dx) == radius || std::abs(dz) == radius) {
                        int cx = config.worldChunksX / 2 + dx;
                        int cz = config.worldChunksZ / 2 + dz;

                        if (cx >= 0 && cx < config.worldChunksX &&
                            cz >= 0 && cz < config.worldChunksZ) {

                            for (int cy = 0; cy < config.worldChunksY; ++cy) {
                                auto blocks = generateChunk(cx, cy, cz, config, rng);
                                for (const auto& [pos, block] : blocks) {
                                    voxelWorld.insert(pos, block);
                                }
                            }
                            ++chunksLoaded;
                        }
                    }
                }
            }
        }

        auto stats = voxelWorld.getMemoryStats();
        benchmark::DoNotOptimize(stats);
    }

    // Calculate and report final stats
    Octree<double, VoxelBlock> finalWorld(worldBounds, 64, 10);
    int chunksLoaded = 0;
    for (int radius = 0; radius < 10 && chunksLoaded < numChunksToLoad; ++radius) {
        for (int dx = -radius; dx <= radius && chunksLoaded < numChunksToLoad; ++dx) {
            for (int dz = -radius; dz <= radius && chunksLoaded < numChunksToLoad; ++dz) {
                if (std::abs(dx) == radius || std::abs(dz) == radius) {
                    int cx = config.worldChunksX / 2 + dx;
                    int cz = config.worldChunksZ / 2 + dz;

                    if (cx >= 0 && cx < config.worldChunksX &&
                        cz >= 0 && cz < config.worldChunksZ) {

                        for (int cy = 0; cy < config.worldChunksY; ++cy) {
                            auto blocks = generateChunk(cx, cy, cz, config, rng);
                            for (const auto& [pos, block] : blocks) {
                                finalWorld.insert(pos, block);
                            }
                        }
                        ++chunksLoaded;
                    }
                }
            }
        }
    }

    auto stats = finalWorld.getMemoryStats();
    state.counters["Chunks"] = benchmark::Counter(static_cast<double>(chunksLoaded));
    state.counters["Blocks"] = benchmark::Counter(static_cast<double>(stats.pointCount));
    state.counters["Nodes"] = benchmark::Counter(static_cast<double>(stats.nodeCount));
    state.counters["MemoryMB"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / (1024.0 * 1024.0));
    state.counters["BytesPerBlock"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_VoxelWorld_LoadChunks)
    ->Arg(1)->Arg(4)->Arg(16)->Arg(64)->Arg(256)
    ->Unit(benchmark::kMillisecond);

// Benchmark: Query blocks around player (view frustum)
static void BM_VoxelWorld_PlayerView(benchmark::State& state) {
    const int renderDistance = static_cast<int>(state.range(0));  // chunks
    ChunkConfig config;
    std::mt19937 rng(42);

    // Setup world
    int worldSizeX = config.chunkSizeX * config.worldChunksX;
    int worldSizeY = config.chunkSizeY * config.worldChunksY;
    int worldSizeZ = config.chunkSizeZ * config.worldChunksZ;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    Octree<double, VoxelBlock> voxelWorld(worldBounds, 64, 10);

    // Load chunks around spawn
    for (int cx = 0; cx < config.worldChunksX; ++cx) {
        for (int cy = 0; cy < config.worldChunksY; ++cy) {
            for (int cz = 0; cz < config.worldChunksZ; ++cz) {
                auto blocks = generateChunk(cx, cy, cz, config, rng);
                for (const auto& [pos, block] : blocks) {
                    voxelWorld.insert(pos, block);
                }
            }
        }
    }

    // Player position at center
    Point3D<double> playerPos(
        static_cast<double>(worldSizeX / 2),
        static_cast<double>(worldSizeY / 2),
        static_cast<double>(worldSizeZ / 2)
    );

    // Query range based on render distance
    double queryRadius = renderDistance * config.chunkSizeX;
    AABB<double> viewFrustum(
        Point3D<double>(
            playerPos.x - queryRadius,
            playerPos.y - queryRadius,
            playerPos.z - queryRadius
        ),
        Point3D<double>(
            playerPos.x + queryRadius,
            playerPos.y + queryRadius,
            playerPos.z + queryRadius
        )
    );

    std::vector<std::pair<Point3D<double>, VoxelBlock>> visibleBlocks;

    for (auto _ : state) {
        visibleBlocks.clear();
        voxelWorld.queryRange(viewFrustum, visibleBlocks);
        benchmark::DoNotOptimize(visibleBlocks);
    }

    state.counters["VisibleBlocks"] = benchmark::Counter(
        static_cast<double>(visibleBlocks.size()));
}
BENCHMARK(BM_VoxelWorld_PlayerView)
    ->Arg(2)->Arg(4)->Arg(8)->Arg(16)
    ->Unit(benchmark::kMicrosecond);

// Benchmark: Ray casting for block interaction
static void BM_VoxelWorld_RayCast(benchmark::State& state) {
    ChunkConfig config;
    std::mt19937 rng(42);

    int worldSizeX = config.chunkSizeX * config.worldChunksX;
    int worldSizeY = config.chunkSizeY * config.worldChunksY;
    int worldSizeZ = config.chunkSizeZ * config.worldChunksZ;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    Octree<double, VoxelBlock> voxelWorld(worldBounds, 64, 10);

    // Load some chunks
    for (int cx = 6; cx < 10; ++cx) {
        for (int cy = 0; cy < config.worldChunksY; ++cy) {
            for (int cz = 6; cz < 10; ++cz) {
                auto blocks = generateChunk(cx, cy, cz, config, rng);
                for (const auto& [pos, block] : blocks) {
                    voxelWorld.insert(pos, block);
                }
            }
        }
    }

    // Player looking at blocks
    Point3D<double> playerPos(100.0, 30.0, 100.0);

    std::vector<std::pair<Point3D<double>, VoxelBlock>> nearbyBlocks;

    for (auto _ : state) {
        nearbyBlocks.clear();
        // Query blocks in reach (5 blocks)
        voxelWorld.queryRadius(playerPos, 5.0, nearbyBlocks);
        benchmark::DoNotOptimize(nearbyBlocks);
    }

    state.counters["BlocksInReach"] = benchmark::Counter(
        static_cast<double>(nearbyBlocks.size()));
}
BENCHMARK(BM_VoxelWorld_RayCast)->Unit(benchmark::kMicrosecond);

// Benchmark: Neighbor finding for lighting propagation
static void BM_VoxelWorld_LightingPropagation(benchmark::State& state) {
    ChunkConfig config;
    std::mt19937 rng(42);

    int worldSizeX = config.chunkSizeX * config.worldChunksX;
    int worldSizeY = config.chunkSizeY * config.worldChunksY;
    int worldSizeZ = config.chunkSizeZ * config.worldChunksZ;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    Octree<double, VoxelBlock> voxelWorld(worldBounds, 64, 10);

    // Load central chunks
    for (int cx = 7; cx < 9; ++cx) {
        for (int cy = 1; cy < 3; ++cy) {
            for (int cz = 7; cz < 9; ++cz) {
                auto blocks = generateChunk(cx, cy, cz, config, rng);
                for (const auto& [pos, block] : blocks) {
                    voxelWorld.insert(pos, block);
                }
            }
        }
    }

    // Light sources to propagate from
    std::vector<Point3D<double>> lightSources = {
        {120.0, 25.0, 120.0},
        {125.0, 28.0, 125.0},
        {115.0, 30.0, 118.0}
    };

    std::vector<std::pair<Point3D<double>, VoxelBlock>> affectedBlocks;

    for (auto _ : state) {
        affectedBlocks.clear();

        // For each light source, find blocks within light radius
        for (const auto& lightPos : lightSources) {
            std::vector<std::pair<Point3D<double>, VoxelBlock>> lit;
            voxelWorld.queryRadius(lightPos, 15.0, lit);  // 15 block light radius
            affectedBlocks.insert(affectedBlocks.end(), lit.begin(), lit.end());
        }

        benchmark::DoNotOptimize(affectedBlocks);
    }

    state.counters["LitBlocks"] = benchmark::Counter(
        static_cast<double>(affectedBlocks.size()));
}
BENCHMARK(BM_VoxelWorld_LightingPropagation)->Unit(benchmark::kMicrosecond);

// Benchmark: Different octree configurations for voxel data
static void BM_VoxelWorld_ConfigComparison(benchmark::State& state) {
    const int maxBlocksPerNode = static_cast<int>(state.range(0));
    const int maxDepth = static_cast<int>(state.range(1));

    ChunkConfig config;
    config.chunkSizeX = 16;
    config.chunkSizeY = 16;
    config.chunkSizeZ = 16;

    std::mt19937 rng(42);

    int worldSizeX = config.chunkSizeX * 8;  // Smaller world for comparison
    int worldSizeY = config.chunkSizeY * 4;
    int worldSizeZ = config.chunkSizeZ * 8;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    for (auto _ : state) {
        state.PauseTiming();
        Octree<double, VoxelBlock> voxelWorld(worldBounds, maxBlocksPerNode, maxDepth);
        state.ResumeTiming();

        // Load all chunks
        for (int cx = 0; cx < 8; ++cx) {
            for (int cy = 0; cy < 4; ++cy) {
                for (int cz = 0; cz < 8; ++cz) {
                    auto blocks = generateChunk(cx, cy, cz, config, rng);
                    for (const auto& [pos, block] : blocks) {
                        voxelWorld.insert(pos, block);
                    }
                }
            }
        }

        benchmark::DoNotOptimize(voxelWorld);
    }

    // Report memory stats
    Octree<double, VoxelBlock> finalWorld(worldBounds, maxBlocksPerNode, maxDepth);
    for (int cx = 0; cx < 8; ++cx) {
        for (int cy = 0; cy < 4; ++cy) {
            for (int cz = 0; cz < 8; ++cz) {
                auto blocks = generateChunk(cx, cy, cz, config, rng);
                for (const auto& [pos, block] : blocks) {
                    finalWorld.insert(pos, block);
                }
            }
        }
    }

    auto stats = finalWorld.getMemoryStats();
    state.counters["MemoryKB"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / 1024.0);
    state.counters["Nodes"] = benchmark::Counter(static_cast<double>(stats.nodeCount));
    state.counters["BytesPerBlock"] = benchmark::Counter(
        static_cast<double>(stats.totalBytes) / stats.pointCount);
}
BENCHMARK(BM_VoxelWorld_ConfigComparison)
    ->Args({32, 8})
    ->Args({64, 8})
    ->Args({128, 8})
    ->Args({64, 6})
    ->Args({64, 10})
    ->Args({64, 12})
    ->Unit(benchmark::kMillisecond);

// Benchmark: Memory comparison with naive storage
static void BM_VoxelWorld_MemoryComparison(benchmark::State& state) {
    ChunkConfig config;
    std::mt19937 rng(42);

    int worldSizeX = config.chunkSizeX * 8;
    int worldSizeY = config.chunkSizeY * 4;
    int worldSizeZ = config.chunkSizeZ * 8;

    AABB<double> worldBounds(
        Point3D<double>(0.0, 0.0, 0.0),
        Point3D<double>(
            static_cast<double>(worldSizeX),
            static_cast<double>(worldSizeY),
            static_cast<double>(worldSizeZ)
        )
    );

    // Octree storage
    Octree<double, VoxelBlock> octreeWorld(worldBounds, 64, 10);

    // Naive hash map storage
    std::unordered_map<uint64_t, VoxelBlock> hashMapWorld;

    auto posToKey = [](const Point3D<double>& pos) -> uint64_t {
        uint64_t x = static_cast<uint64_t>(pos.x);
        uint64_t y = static_cast<uint64_t>(pos.y);
        uint64_t z = static_cast<uint64_t>(pos.z);
        return (x << 32) | (y << 16) | z;
    };

    // Load chunks
    size_t blockCount = 0;
    for (int cx = 0; cx < 8; ++cx) {
        for (int cy = 0; cy < 4; ++cy) {
            for (int cz = 0; cz < 8; ++cz) {
                auto blocks = generateChunk(cx, cy, cz, config, rng);
                for (const auto& [pos, block] : blocks) {
                    octreeWorld.insert(pos, block);
                    hashMapWorld[posToKey(pos)] = block;
                    ++blockCount;
                }
            }
        }
    }

    for (auto _ : state) {
        auto octreeStats = octreeWorld.getMemoryStats();
        benchmark::DoNotOptimize(octreeStats);
    }

    auto octreeStats = octreeWorld.getMemoryStats();
    size_t hashMapBytes = hashMapWorld.size() * (sizeof(uint64_t) + sizeof(VoxelBlock)) +
                          hashMapWorld.bucket_count() * sizeof(void*);

    state.counters["Blocks"] = benchmark::Counter(static_cast<double>(blockCount));
    state.counters["Octree_KB"] = benchmark::Counter(
        static_cast<double>(octreeStats.totalBytes) / 1024.0);
    state.counters["HashMap_KB"] = benchmark::Counter(
        static_cast<double>(hashMapBytes) / 1024.0);
    state.counters["Octree_BytesPerBlock"] = benchmark::Counter(
        static_cast<double>(octreeStats.totalBytes) / blockCount);
    state.counters["HashMap_BytesPerBlock"] = benchmark::Counter(
        static_cast<double>(hashMapBytes) / blockCount);
}
BENCHMARK(BM_VoxelWorld_MemoryComparison)->Unit(benchmark::kMillisecond);
