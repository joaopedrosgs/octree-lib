#include <octree/octree.hpp>
#include <gtest/gtest.h>

using namespace octree;

class RaycastTest : public ::testing::Test {
protected:
    void SetUp() override {
        AABB<double> boundary(Point3D<double>(0.0, 0.0, 0.0),
                             Point3D<double>(100.0, 100.0, 100.0));
        tree = std::make_unique<Octree<double, int>>(boundary, 4, 8);
    }

    std::unique_ptr<Octree<double, int>> tree;
};

// Ray3D Tests
TEST(Ray3DTest, Construction) {
    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    EXPECT_EQ(ray.origin.x, 0.0);
    EXPECT_EQ(ray.origin.y, 0.0);
    EXPECT_EQ(ray.origin.z, 0.0);
}

TEST(Ray3DTest, DirectionNormalization) {
    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(2, 0, 0));
    // Direction should be normalized
    EXPECT_DOUBLE_EQ(ray.direction.length(), 1.0);
}

TEST(Ray3DTest, AtFunction) {
    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    Point3D<double> p = ray.at(5.0);
    EXPECT_DOUBLE_EQ(p.x, 5.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
    EXPECT_DOUBLE_EQ(p.z, 0.0);
}

// Point3D operator tests
TEST(Point3DTest, Addition) {
    Point3D<double> p1(1, 2, 3);
    Point3D<double> p2(4, 5, 6);
    Point3D<double> result = p1 + p2;
    EXPECT_EQ(result.x, 5.0);
    EXPECT_EQ(result.y, 7.0);
    EXPECT_EQ(result.z, 9.0);
}

TEST(Point3DTest, Subtraction) {
    Point3D<double> p1(4, 5, 6);
    Point3D<double> p2(1, 2, 3);
    Point3D<double> result = p1 - p2;
    EXPECT_EQ(result.x, 3.0);
    EXPECT_EQ(result.y, 3.0);
    EXPECT_EQ(result.z, 3.0);
}

TEST(Point3DTest, ScalarMultiplication) {
    Point3D<double> p(1, 2, 3);
    Point3D<double> result = p * 2.0;
    EXPECT_EQ(result.x, 2.0);
    EXPECT_EQ(result.y, 4.0);
    EXPECT_EQ(result.z, 6.0);
}

TEST(Point3DTest, DotProduct) {
    Point3D<double> p1(1, 0, 0);
    Point3D<double> p2(1, 0, 0);
    EXPECT_DOUBLE_EQ(p1.dot(p2), 1.0);

    Point3D<double> p3(0, 1, 0);
    EXPECT_DOUBLE_EQ(p1.dot(p3), 0.0);
}

TEST(Point3DTest, Length) {
    Point3D<double> p(3, 4, 0);
    EXPECT_DOUBLE_EQ(p.length(), 5.0);
}

TEST(Point3DTest, Normalized) {
    Point3D<double> p(3, 4, 0);
    Point3D<double> norm = p.normalized();
    EXPECT_DOUBLE_EQ(norm.length(), 1.0);
}

// AABB-Ray intersection tests
TEST(AABBRayTest, RayIntersectsBox) {
    AABB<double> box(Point3D<double>(0, 0, 0), Point3D<double>(10, 10, 10));
    Ray3D<double> ray(Point3D<double>(-5, 5, 5), Point3D<double>(1, 0, 0));

    double tMin, tMax;
    EXPECT_TRUE(box.intersectsRay(ray, tMin, tMax));
    EXPECT_GT(tMax, tMin);
}

TEST(AABBRayTest, RayMissesBox) {
    AABB<double> box(Point3D<double>(0, 0, 0), Point3D<double>(10, 10, 10));
    Ray3D<double> ray(Point3D<double>(-5, 20, 5), Point3D<double>(1, 0, 0));

    double tMin, tMax;
    EXPECT_FALSE(box.intersectsRay(ray, tMin, tMax));
}

TEST(AABBRayTest, RayOriginInsideBox) {
    AABB<double> box(Point3D<double>(0, 0, 0), Point3D<double>(10, 10, 10));
    Ray3D<double> ray(Point3D<double>(5, 5, 5), Point3D<double>(1, 0, 0));

    double tMin, tMax;
    EXPECT_TRUE(box.intersectsRay(ray, tMin, tMax));
}

TEST(AABBRayTest, RayBehindBox) {
    AABB<double> box(Point3D<double>(0, 0, 0), Point3D<double>(10, 10, 10));
    Ray3D<double> ray(Point3D<double>(20, 5, 5), Point3D<double>(1, 0, 0));

    double tMin, tMax;
    bool intersects = box.intersectsRay(ray, tMin, tMax);
    // Ray pointing away from box
    if (intersects) {
        EXPECT_LT(tMax, 0);  // Intersection is behind ray origin
    }
}

// Raycast tests
TEST_F(RaycastTest, RaycastFirstNoHit) {
    tree->insert(Point3D<double>(50, 50, 50), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(0, 1, 0));
    std::pair<Point3D<double>, int> result;

    EXPECT_FALSE(tree->raycastFirst(ray, 100.0, result));
}

TEST_F(RaycastTest, RaycastFirstHit) {
    tree->insert(Point3D<double>(50, 0, 0), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::pair<Point3D<double>, int> result;

    EXPECT_TRUE(tree->raycastFirst(ray, 100.0, result));
    EXPECT_EQ(result.second, 1);
    EXPECT_DOUBLE_EQ(result.first.x, 50.0);
}

TEST_F(RaycastTest, RaycastFirstClosest) {
    tree->insert(Point3D<double>(30, 0, 0), 1);
    tree->insert(Point3D<double>(50, 0, 0), 2);
    tree->insert(Point3D<double>(70, 0, 0), 3);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::pair<Point3D<double>, int> result;

    EXPECT_TRUE(tree->raycastFirst(ray, 100.0, result));
    EXPECT_EQ(result.second, 1);  // Should hit the closest point
    EXPECT_DOUBLE_EQ(result.first.x, 30.0);
}

TEST_F(RaycastTest, RaycastFirstMaxDistance) {
    tree->insert(Point3D<double>(50, 0, 0), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::pair<Point3D<double>, int> result;

    // Max distance too short
    EXPECT_FALSE(tree->raycastFirst(ray, 40.0, result));

    // Max distance sufficient
    EXPECT_TRUE(tree->raycastFirst(ray, 60.0, result));
}

TEST_F(RaycastTest, RaycastFirstBehindRay) {
    tree->insert(Point3D<double>(-50, 0, 0), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::pair<Point3D<double>, int> result;

    EXPECT_FALSE(tree->raycastFirst(ray, 100.0, result));
}

TEST_F(RaycastTest, RaycastAllNoHits) {
    tree->insert(Point3D<double>(50, 50, 50), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(0, 1, 0));
    std::vector<std::pair<Point3D<double>, int>> results;

    tree->raycastAll(ray, 100.0, results);
    EXPECT_EQ(results.size(), 0);
}

TEST_F(RaycastTest, RaycastAllMultipleHits) {
    tree->insert(Point3D<double>(30, 0, 0), 1);
    tree->insert(Point3D<double>(50, 0, 0), 2);
    tree->insert(Point3D<double>(70, 0, 0), 3);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::vector<std::pair<Point3D<double>, int>> results;

    tree->raycastAll(ray, 100.0, results);
    EXPECT_EQ(results.size(), 3);
}

TEST_F(RaycastTest, RaycastAllMaxDistance) {
    tree->insert(Point3D<double>(30, 0, 0), 1);
    tree->insert(Point3D<double>(50, 0, 0), 2);
    tree->insert(Point3D<double>(70, 0, 0), 3);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::vector<std::pair<Point3D<double>, int>> results;

    tree->raycastAll(ray, 55.0, results);
    EXPECT_EQ(results.size(), 2);  // Only first two within distance
}

TEST_F(RaycastTest, RaycastDiagonal) {
    tree->insert(Point3D<double>(10, 10, 10), 1);

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 1, 1));
    std::pair<Point3D<double>, int> result;

    EXPECT_TRUE(tree->raycastFirst(ray, 50.0, result));
    EXPECT_EQ(result.second, 1);
}

TEST_F(RaycastTest, RaycastWithDifferentDataTypes) {
    AABB<float> boundary(Point3D<float>(0, 0, 0), Point3D<float>(100, 100, 100));
    Octree<float, std::string> stringTree(boundary);

    stringTree.insert(Point3D<float>(50, 0, 0), "target");

    Ray3D<float> ray(Point3D<float>(0, 0, 0), Point3D<float>(1, 0, 0));
    std::pair<Point3D<float>, std::string> result;

    EXPECT_TRUE(stringTree.raycastFirst(ray, 100.0f, result));
    EXPECT_EQ(result.second, "target");
}

TEST_F(RaycastTest, RaycastManyPoints) {
    // Insert many points
    for (int i = 0; i < 100; ++i) {
        tree->insert(Point3D<double>(i, 0, 0), i);
    }

    Ray3D<double> ray(Point3D<double>(0, 0, 0), Point3D<double>(1, 0, 0));
    std::vector<std::pair<Point3D<double>, int>> results;

    tree->raycastAll(ray, 100.0, results);
    EXPECT_GT(results.size(), 0);
}
