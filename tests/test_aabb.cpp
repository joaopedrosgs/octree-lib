#include <octree/octree.hpp>
#include <gtest/gtest.h>

using namespace octree;

TEST(AABBTest, DefaultConstructor) {
    AABB<double> box;
    // Default constructed points should be (0,0,0)
    EXPECT_EQ(box.min.x, 0.0);
    EXPECT_EQ(box.max.x, 0.0);
}

TEST(AABBTest, ParameterizedConstructor) {
    Point3D<double> min(0.0, 0.0, 0.0);
    Point3D<double> max(10.0, 10.0, 10.0);
    AABB<double> box(min, max);

    EXPECT_EQ(box.min.x, 0.0);
    EXPECT_EQ(box.max.x, 10.0);
}

TEST(AABBTest, Contains) {
    AABB<double> box(Point3D<double>(0.0, 0.0, 0.0),
                     Point3D<double>(10.0, 10.0, 10.0));

    EXPECT_TRUE(box.contains(Point3D<double>(5.0, 5.0, 5.0)));
    EXPECT_TRUE(box.contains(Point3D<double>(0.0, 0.0, 0.0)));  // boundary
    EXPECT_TRUE(box.contains(Point3D<double>(10.0, 10.0, 10.0))); // boundary
    EXPECT_FALSE(box.contains(Point3D<double>(-1.0, 5.0, 5.0)));
    EXPECT_FALSE(box.contains(Point3D<double>(11.0, 5.0, 5.0)));
}

TEST(AABBTest, Center) {
    AABB<double> box(Point3D<double>(0.0, 0.0, 0.0),
                     Point3D<double>(10.0, 20.0, 30.0));
    Point3D<double> center = box.center();

    EXPECT_DOUBLE_EQ(center.x, 5.0);
    EXPECT_DOUBLE_EQ(center.y, 10.0);
    EXPECT_DOUBLE_EQ(center.z, 15.0);
}

TEST(AABBTest, Intersects) {
    AABB<double> box1(Point3D<double>(0.0, 0.0, 0.0),
                      Point3D<double>(10.0, 10.0, 10.0));
    AABB<double> box2(Point3D<double>(5.0, 5.0, 5.0),
                      Point3D<double>(15.0, 15.0, 15.0));
    AABB<double> box3(Point3D<double>(11.0, 11.0, 11.0),
                      Point3D<double>(20.0, 20.0, 20.0));

    EXPECT_TRUE(box1.intersects(box2));
    EXPECT_TRUE(box2.intersects(box1));
    EXPECT_FALSE(box1.intersects(box3));
    EXPECT_FALSE(box3.intersects(box1));
}

TEST(AABBTest, IntersectsSelf) {
    AABB<double> box(Point3D<double>(0.0, 0.0, 0.0),
                     Point3D<double>(10.0, 10.0, 10.0));
    EXPECT_TRUE(box.intersects(box));
}

TEST(AABBTest, DistanceSquared) {
    AABB<double> box(Point3D<double>(0.0, 0.0, 0.0),
                     Point3D<double>(10.0, 10.0, 10.0));

    // Point inside box
    EXPECT_DOUBLE_EQ(box.distanceSquared(Point3D<double>(5.0, 5.0, 5.0)), 0.0);

    // Point outside on X axis
    EXPECT_DOUBLE_EQ(box.distanceSquared(Point3D<double>(15.0, 5.0, 5.0)), 25.0);

    // Point outside on multiple axes
    Point3D<double> p(15.0, 15.0, 15.0);
    EXPECT_DOUBLE_EQ(box.distanceSquared(p), 75.0); // 5^2 + 5^2 + 5^2
}
