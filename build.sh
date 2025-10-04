#!/bin/bash
# Build script for octree-lib

set -e

BUILD_TYPE=${1:-Release}
BUILD_DIR="build"

echo "=== Building octree-lib in $BUILD_TYPE mode ==="

# Create and enter build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo "Configuring..."
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ..

# Build
echo "Building..."
cmake --build . -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# Run tests
echo ""
echo "=== Running tests ==="
ctest --output-on-failure

echo ""
echo "=== Build complete ==="
echo "Run examples:"
echo "  ./examples/basic_usage"
echo "  ./examples/spatial_queries"
echo "  ./examples/custom_data"
echo ""
echo "Run benchmarks:"
echo "  ./benchmarks/octree_benchmarks"
