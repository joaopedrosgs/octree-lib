#include <octree/octree.hpp>
#include <gtest/gtest.h>
#include <vector>
#include <algorithm>

using namespace octree;

class OctreeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a standard octree for testing
        AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                             Point3D<double>(100.0, 100.0, 100.0));
        tree = std::make_unique<Octree<double, int>>(boundary, 4, 8);
    }

    std::unique_ptr<Octree<double, int>> tree;
};

TEST_F(OctreeTest, ConstructorAndEmpty) {
    EXPECT_TRUE(tree->empty());
    EXPECT_EQ(tree->size(), 0);
}

TEST_F(OctreeTest, InsertSinglePoint) {
    Point3D<double> p(50.0, 50.0, 50.0);
    EXPECT_TRUE(tree->insert(p, 1));
    EXPECT_EQ(tree->size(), 1);
    EXPECT_FALSE(tree->empty());
}

TEST_F(OctreeTest, InsertMultiplePoints) {
    std::vector<Point3D<double>> points = {
        {10.0, 10.0, 10.0},
        {20.0, 20.0, 20.0},
        {30.0, 30.0, 30.0},
        {40.0, 40.0, 40.0}
    };

    for (size_t i = 0; i < points.size(); ++i) {
        EXPECT_TRUE(tree->insert(points[i], static_cast<int>(i)));
    }

    EXPECT_EQ(tree->size(), points.size());
}

TEST_F(OctreeTest, InsertOutsideBoundary) {
    Point3D<double> p(150.0, 150.0, 150.0); // Outside [0, 100]
    EXPECT_FALSE(tree->insert(p, 1));
    EXPECT_EQ(tree->size(), 0);
}

TEST_F(OctreeTest, InsertOnBoundary) {
    // Points on the boundary should be included
    EXPECT_TRUE(tree->insert(Point3D<double>(0.0, 0.0, 0.0), 1));
    EXPECT_TRUE(tree->insert(Point3D<double>(100.0, 100.0, 100.0), 2));
    EXPECT_EQ(tree->size(), 2);
}

TEST_F(OctreeTest, Contains) {
    Point3D<double> p(50.0, 50.0, 50.0);
    tree->insert(p, 1);

    EXPECT_TRUE(tree->contains(p));
    EXPECT_FALSE(tree->contains(Point3D<double>(51.0, 51.0, 51.0)));
}

TEST_F(OctreeTest, Clear) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);
    tree->insert(Point3D<double>(60.0, 60.0, 60.0), 2);
    EXPECT_EQ(tree->size(), 2);

    tree->clear();
    EXPECT_TRUE(tree->empty());
    EXPECT_EQ(tree->size(), 0);
}

TEST_F(OctreeTest, QueryRangeAll) {
    // Insert points
    std::vector<Point3D<double>> points = {
        {10.0, 10.0, 10.0},
        {20.0, 20.0, 20.0},
        {30.0, 30.0, 30.0}
    };

    for (size_t i = 0; i < points.size(); ++i) {
        tree->insert(points[i], static_cast<int>(i));
    }

    // Query entire boundary
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRange(tree->boundary(), results);

    EXPECT_EQ(results.size(), points.size());
}

TEST_F(OctreeTest, QueryRangePartial) {
    // Insert points across the space
    tree->insert(Point3D<double>(10.0, 10.0, 10.0), 1);
    tree->insert(Point3D<double>(90.0, 90.0, 90.0), 2);
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 3);

    // Query only the lower corner
    AABB<double> range(Point3D<double>(0.0, 0.0, 0.0),
                       Point3D<double>(30.0, 30.0, 30.0));
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRange(range, results);

    EXPECT_EQ(results.size(), 1);
    EXPECT_EQ(results[0].second, 1);
}

TEST_F(OctreeTest, QueryRangeEmpty) {
    tree->insert(Point3D<double>(10.0, 10.0, 10.0), 1);

    // Query range with no points
    AABB<double> range(Point3D<double>(80.0, 80.0, 80.0),
                       Point3D<double>(90.0, 90.0, 90.0));
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRange(range, results);

    EXPECT_EQ(results.size(), 0);
}

TEST_F(OctreeTest, QueryKNearestSingle) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);
    tree->insert(Point3D<double>(60.0, 60.0, 60.0), 2);
    tree->insert(Point3D<double>(40.0, 40.0, 40.0), 3);

    Point3D<double> query(50.0, 50.0, 50.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryKNearest(query, 1, results);

    ASSERT_EQ(results.size(), 1);
    EXPECT_EQ(results[0].second, 1); // Exact match
}

TEST_F(OctreeTest, QueryKNearestMultiple) {
    // Insert points at known distances
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1); // distance 0
    tree->insert(Point3D<double>(51.0, 50.0, 50.0), 2); // distance 1
    tree->insert(Point3D<double>(52.0, 50.0, 50.0), 3); // distance 2
    tree->insert(Point3D<double>(53.0, 50.0, 50.0), 4); // distance 3

    Point3D<double> query(50.0, 50.0, 50.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryKNearest(query, 2, results);

    ASSERT_EQ(results.size(), 2);
    // Results should be the two closest points
    EXPECT_EQ(results[0].second, 1);
    EXPECT_EQ(results[1].second, 2);
}

TEST_F(OctreeTest, QueryKNearestMoreThanAvailable) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);
    tree->insert(Point3D<double>(60.0, 60.0, 60.0), 2);

    Point3D<double> query(50.0, 50.0, 50.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryKNearest(query, 10, results);

    EXPECT_EQ(results.size(), 2); // Only 2 points available
}

TEST_F(OctreeTest, QueryKNearestZero) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);

    Point3D<double> query(50.0, 50.0, 50.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryKNearest(query, 0, results);

    EXPECT_EQ(results.size(), 0);
}

TEST_F(OctreeTest, QueryRadius) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);
    tree->insert(Point3D<double>(55.0, 50.0, 50.0), 2); // distance 5
    tree->insert(Point3D<double>(60.0, 50.0, 50.0), 3); // distance 10

    Point3D<double> query(50.0, 50.0, 50.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRadius(query, 7.0, results);

    EXPECT_EQ(results.size(), 2); // Points at distance 0 and 5
}

TEST_F(OctreeTest, QueryRadiusNone) {
    tree->insert(Point3D<double>(50.0, 50.0, 50.0), 1);
    tree->insert(Point3D<double>(60.0, 60.0, 60.0), 2);

    Point3D<double> query(10.0, 10.0, 10.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRadius(query, 5.0, results);

    EXPECT_EQ(results.size(), 0);
}

TEST_F(OctreeTest, Subdivision) {
    // Insert enough points to force subdivision (max 4 per node)
    for (int i = 0; i < 10; ++i) {
        tree->insert(Point3D<double>(50.0 + i, 50.0, 50.0), i);
    }

    EXPECT_EQ(tree->size(), 10);

    // All points should still be queryable
    AABB<double> range(Point3D<double>(0.0, 0.0, 0.0),
                       Point3D<double>(100.0, 100.0, 100.0));
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryRange(range, results);

    EXPECT_EQ(results.size(), 10);
}

TEST_F(OctreeTest, Traverse) {
    tree->insert(Point3D<double>(10.0, 10.0, 10.0), 1);
    tree->insert(Point3D<double>(20.0, 20.0, 20.0), 2);
    tree->insert(Point3D<double>(30.0, 30.0, 30.0), 3);

    int count = 0;
    tree->traverse([&count](const Point3D<double>& p, const int& data) {
        (void)p;
        count++;
        EXPECT_GT(data, 0);
        EXPECT_LE(data, 3);
    });

    EXPECT_EQ(count, 3);
}

TEST_F(OctreeTest, MemoryStats) {
    // Empty tree
    auto stats = tree->getMemoryStats();
    EXPECT_GT(stats.nodeCount, 0);
    EXPECT_EQ(stats.pointCount, 0);
    EXPECT_GT(stats.totalBytes, 0);

    // Add points
    for (int i = 0; i < 100; ++i) {
        tree->insert(Point3D<double>(i % 10 * 10.0, i % 10 * 10.0, i % 10 * 10.0), i);
    }

    stats = tree->getMemoryStats();
    EXPECT_GT(stats.nodeCount, 1); // Should have subdivided
    EXPECT_EQ(stats.pointCount, 100);
    EXPECT_GT(stats.totalBytes, 0);
}

TEST_F(OctreeTest, DifferentDataTypes) {
    AABB<float> boundary(Point3D<float>(0.0f, 0.0f, 0.0f),
                         Point3D<float>(100.0f, 100.0f, 100.0f));

    // Test with string data
    Octree<float, std::string> stringTree(boundary);
    stringTree.insert(Point3D<float>(50.0f, 50.0f, 50.0f), "test");
    EXPECT_EQ(stringTree.size(), 1);

    // Test with struct data
    struct CustomData {
        int id;
        double value;
    };

    Octree<double, CustomData> structTree(tree->boundary());
    structTree.insert(Point3D<double>(50.0, 50.0, 50.0), CustomData{1, 3.14});
    EXPECT_EQ(structTree.size(), 1);
}

TEST_F(OctreeTest, LargeDataset) {
    // Insert a large number of points
    const int numPoints = 10000;
    for (int i = 0; i < numPoints; ++i) {
        double x = (i % 100);
        double y = ((i / 100) % 100);
        double z = (i / 10000);
        tree->insert(Point3D<double>(x, y, z), i);
    }

    EXPECT_EQ(tree->size(), numPoints);

    // Verify we can still query efficiently
    Point3D<double> query(50.0, 50.0, 0.0);
    std::vector<std::pair<Point3D<double>, int>> results;
    tree->queryKNearest(query, 10, results);

    EXPECT_EQ(results.size(), 10);
}

TEST_F(OctreeTest, BoundaryEdgeCases) {
    // Test points exactly on octant boundaries
    Point3D<double> center = tree->boundary().center();

    EXPECT_TRUE(tree->insert(center, 1));
    EXPECT_TRUE(tree->insert(Point3D<double>(center.x + 0.1, center.y, center.z), 2));
    EXPECT_TRUE(tree->insert(Point3D<double>(center.x - 0.1, center.y, center.z), 3));
    EXPECT_TRUE(tree->insert(Point3D<double>(center.x, center.y + 0.1, center.z), 4));

    EXPECT_EQ(tree->size(), 4);
}
