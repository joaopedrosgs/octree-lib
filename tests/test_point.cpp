#include <octree/octree.hpp>
#include <gtest/gtest.h>

using namespace octree;

TEST(Point3DTest, DefaultConstructor) {
    Point3D<double> p;
    EXPECT_EQ(p.x, 0.0);
    EXPECT_EQ(p.y, 0.0);
    EXPECT_EQ(p.z, 0.0);
}

TEST(Point3DTest, ParameterizedConstructor) {
    Point3D<double> p(1.0, 2.0, 3.0);
    EXPECT_EQ(p.x, 1.0);
    EXPECT_EQ(p.y, 2.0);
    EXPECT_EQ(p.z, 3.0);
}

TEST(Point3DTest, Distance) {
    Point3D<double> p1(0.0, 0.0, 0.0);
    Point3D<double> p2(3.0, 4.0, 0.0);
    EXPECT_DOUBLE_EQ(p1.distance(p2), 5.0);
    EXPECT_DOUBLE_EQ(p1.distanceSquared(p2), 25.0);
}

TEST(Point3DTest, DistanceSymmetric) {
    Point3D<double> p1(1.0, 2.0, 3.0);
    Point3D<double> p2(4.0, 5.0, 6.0);
    EXPECT_DOUBLE_EQ(p1.distance(p2), p2.distance(p1));
}

TEST(Point3DTest, Equality) {
    Point3D<int> p1(1, 2, 3);
    Point3D<int> p2(1, 2, 3);
    Point3D<int> p3(1, 2, 4);

    EXPECT_TRUE(p1 == p2);
    EXPECT_FALSE(p1 == p3);
    EXPECT_TRUE(p1 != p3);
    EXPECT_FALSE(p1 != p2);
}

TEST(Point3DTest, FloatingPointTypes) {
    Point3D<float> pf(1.5f, 2.5f, 3.5f);
    EXPECT_FLOAT_EQ(pf.x, 1.5f);

    Point3D<double> pd(1.5, 2.5, 3.5);
    EXPECT_DOUBLE_EQ(pd.x, 1.5);
}
