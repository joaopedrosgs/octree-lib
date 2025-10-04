#pragma once

#include <array>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

namespace octree {

/**
 * 3D Point representation
 */
template<typename T>
struct Point3D {
    T x, y, z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

    T distanceSquared(const Point3D& other) const {
        T dx = x - other.x;
        T dy = y - other.y;
        T dz = z - other.z;
        return dx * dx + dy * dy + dz * dz;
    }

    T distance(const Point3D& other) const {
        return std::sqrt(distanceSquared(other));
    }

    bool operator==(const Point3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const Point3D& other) const {
        return !(*this == other);
    }
};

/**
 * Axis-Aligned Bounding Box
 */
template<typename T>
struct AABB {
    Point3D<T> min;
    Point3D<T> max;

    AABB() = default;
    AABB(const Point3D<T>& min_, const Point3D<T>& max_) : min(min_), max(max_) {}

    bool contains(const Point3D<T>& point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    bool intersects(const AABB& other) const {
        return !(other.min.x > max.x || other.max.x < min.x ||
                 other.min.y > max.y || other.max.y < min.y ||
                 other.min.z > max.z || other.max.z < min.z);
    }

    Point3D<T> center() const {
        return Point3D<T>(
            (min.x + max.x) / 2,
            (min.y + max.y) / 2,
            (min.z + max.z) / 2
        );
    }

    T distanceSquared(const Point3D<T>& point) const {
        T dx = std::max({min.x - point.x, T(0), point.x - max.x});
        T dy = std::max({min.y - point.y, T(0), point.y - max.y});
        T dz = std::max({min.z - point.z, T(0), point.z - max.z});
        return dx * dx + dy * dy + dz * dz;
    }
};

/**
 * Octree node data structure
 */
template<typename T, typename DataType>
struct OctreeNode {
    AABB<T> boundary;
    std::vector<std::pair<Point3D<T>, DataType>> points;
    std::array<std::unique_ptr<OctreeNode>, 8> children;
    bool subdivided = false;

    explicit OctreeNode(const AABB<T>& boundary_) : boundary(boundary_) {}
};

/**
 * Octree spatial data structure
 *
 * Template parameters:
 * - T: Coordinate type (float, double, etc.)
 * - DataType: Associated data type stored with each point
 */
template<typename T, typename DataType = int>
class Octree {
public:
    /**
     * Constructor
     * @param boundary The bounding box for the octree
     * @param maxPointsPerNode Maximum points before subdivision (default: 8)
     * @param maxDepth Maximum tree depth (default: 8)
     */
    explicit Octree(const AABB<T>& boundary,
                    size_t maxPointsPerNode = 8,
                    size_t maxDepth = 8)
        : root_(std::make_unique<OctreeNode<T, DataType>>(boundary))
        , maxPointsPerNode_(maxPointsPerNode)
        , maxDepth_(maxDepth)
        , size_(0) {}

    /**
     * Insert a point with associated data
     * @param point The 3D point to insert
     * @param data The associated data
     * @return true if inserted successfully
     */
    bool insert(const Point3D<T>& point, const DataType& data) {
        if (insertImpl(root_.get(), point, data, 0)) {
            ++size_;
            return true;
        }
        return false;
    }

    /**
     * Query points within a bounding box
     * @param range The query bounding box
     * @param results Vector to store found points and data
     */
    void queryRange(const AABB<T>& range,
                    std::vector<std::pair<Point3D<T>, DataType>>& results) const {
        queryRangeImpl(root_.get(), range, results);
    }

    /**
     * Find k nearest neighbors to a query point
     * @param point The query point
     * @param k Number of neighbors to find
     * @param results Vector to store found points and data
     */
    void queryKNearest(const Point3D<T>& point,
                       size_t k,
                       std::vector<std::pair<Point3D<T>, DataType>>& results) const {
        if (k == 0) return;

        std::vector<std::pair<T, std::pair<Point3D<T>, DataType>>> candidates;
        queryKNearestImpl(root_.get(), point, k, candidates);

        results.clear();
        results.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            results.push_back(candidate.second);
        }
    }

    /**
     * Find all points within a radius of a query point
     * @param point The query point
     * @param radius The search radius
     * @param results Vector to store found points and data
     */
    void queryRadius(const Point3D<T>& point,
                     T radius,
                     std::vector<std::pair<Point3D<T>, DataType>>& results) const {
        T radiusSquared = radius * radius;
        queryRadiusImpl(root_.get(), point, radiusSquared, results);
    }

    /**
     * Check if a point exists in the octree
     * @param point The point to search for
     * @return true if found
     */
    bool contains(const Point3D<T>& point) const {
        return containsImpl(root_.get(), point);
    }

    /**
     * Clear all points from the octree
     */
    void clear() {
        AABB<T> boundary = root_->boundary;
        root_ = std::make_unique<OctreeNode<T, DataType>>(boundary);
        size_ = 0;
    }

    /**
     * Get the number of points in the octree
     */
    size_t size() const { return size_; }

    /**
     * Check if the octree is empty
     */
    bool empty() const { return size_ == 0; }

    /**
     * Get the bounding box of the octree
     */
    const AABB<T>& boundary() const { return root_->boundary; }

    /**
     * Traverse all points in the octree
     * @param visitor Function to call for each point and data
     */
    void traverse(std::function<void(const Point3D<T>&, const DataType&)> visitor) const {
        traverseImpl(root_.get(), visitor);
    }

    /**
     * Get memory usage statistics
     */
    struct MemoryStats {
        size_t nodeCount = 0;
        size_t pointCount = 0;
        size_t totalBytes = 0;
    };

    MemoryStats getMemoryStats() const {
        MemoryStats stats;
        getMemoryStatsImpl(root_.get(), stats);
        return stats;
    }

private:
    std::unique_ptr<OctreeNode<T, DataType>> root_;
    size_t maxPointsPerNode_;
    size_t maxDepth_;
    size_t size_;

    bool insertImpl(OctreeNode<T, DataType>* node,
                    const Point3D<T>& point,
                    const DataType& data,
                    size_t depth) {
        if (!node->boundary.contains(point)) {
            return false;
        }

        // If node hasn't been subdivided and has room, add point here
        if (!node->subdivided) {
            if (node->points.size() < maxPointsPerNode_ || depth >= maxDepth_) {
                node->points.emplace_back(point, data);
                return true;
            }

            // Subdivide if we've reached capacity
            subdivide(node);
        }

        // Insert into appropriate child
        for (auto& child : node->children) {
            if (child->boundary.contains(point)) {
                return insertImpl(child.get(), point, data, depth + 1);
            }
        }

        return false;
    }

    void subdivide(OctreeNode<T, DataType>* node) {
        Point3D<T> center = node->boundary.center();
        const Point3D<T>& min = node->boundary.min;
        const Point3D<T>& max = node->boundary.max;

        // Create 8 octants
        node->children[0] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(min, center));
        node->children[1] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(center.x, min.y, min.z), Point3D<T>(max.x, center.y, center.z)));
        node->children[2] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(min.x, center.y, min.z), Point3D<T>(center.x, max.y, center.z)));
        node->children[3] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(center.x, center.y, min.z), Point3D<T>(max.x, max.y, center.z)));
        node->children[4] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(min.x, min.y, center.z), Point3D<T>(center.x, center.y, max.z)));
        node->children[5] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(center.x, min.y, center.z), Point3D<T>(max.x, center.y, max.z)));
        node->children[6] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(Point3D<T>(min.x, center.y, center.z), Point3D<T>(center.x, max.y, max.z)));
        node->children[7] = std::make_unique<OctreeNode<T, DataType>>(
            AABB<T>(center, max));

        node->subdivided = true;
    }

    void queryRangeImpl(const OctreeNode<T, DataType>* node,
                        const AABB<T>& range,
                        std::vector<std::pair<Point3D<T>, DataType>>& results) const {
        if (!node || !node->boundary.intersects(range)) {
            return;
        }

        // Check points in this node
        for (const auto& [point, data] : node->points) {
            if (range.contains(point)) {
                results.emplace_back(point, data);
            }
        }

        // Recursively check children
        if (node->subdivided) {
            for (const auto& child : node->children) {
                queryRangeImpl(child.get(), range, results);
            }
        }
    }

    void queryKNearestImpl(const OctreeNode<T, DataType>* node,
                          const Point3D<T>& point,
                          size_t k,
                          std::vector<std::pair<T, std::pair<Point3D<T>, DataType>>>& candidates) const {
        if (!node) return;

        // Check if we should prune this node
        if (!candidates.empty() && candidates.size() >= k) {
            T maxDist = candidates.front().first;
            T nodeDist = node->boundary.distanceSquared(point);
            if (nodeDist > maxDist) {
                return;
            }
        }

        // Add points from this node to candidates
        for (const auto& [p, data] : node->points) {
            T distSq = point.distanceSquared(p);
            candidates.emplace_back(distSq, std::make_pair(p, data));
        }

        // Sort and keep only k nearest
        if (candidates.size() > k) {
            std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(),
                            [](const auto& a, const auto& b) { return a.first < b.first; });
            candidates.resize(k);
        }

        // Recursively search children, prioritizing closer octants
        if (node->subdivided) {
            std::array<std::pair<T, const OctreeNode<T, DataType>*>, 8> childDistances;
            for (size_t i = 0; i < 8; ++i) {
                if (node->children[i]) {
                    childDistances[i] = {
                        node->children[i]->boundary.distanceSquared(point),
                        node->children[i].get()
                    };
                } else {
                    childDistances[i] = {std::numeric_limits<T>::max(), nullptr};
                }
            }

            std::sort(childDistances.begin(), childDistances.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });

            for (const auto& [dist, child] : childDistances) {
                if (child) {
                    queryKNearestImpl(child, point, k, candidates);
                }
            }
        }

        // Final sort to ensure k nearest
        if (candidates.size() > k) {
            std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(),
                            [](const auto& a, const auto& b) { return a.first < b.first; });
            candidates.resize(k);
        }
    }

    void queryRadiusImpl(const OctreeNode<T, DataType>* node,
                        const Point3D<T>& point,
                        T radiusSquared,
                        std::vector<std::pair<Point3D<T>, DataType>>& results) const {
        if (!node) return;

        // Check if node's AABB is too far
        if (node->boundary.distanceSquared(point) > radiusSquared) {
            return;
        }

        // Check points in this node
        for (const auto& [p, data] : node->points) {
            if (point.distanceSquared(p) <= radiusSquared) {
                results.emplace_back(p, data);
            }
        }

        // Recursively check children
        if (node->subdivided) {
            for (const auto& child : node->children) {
                queryRadiusImpl(child.get(), point, radiusSquared, results);
            }
        }
    }

    bool containsImpl(const OctreeNode<T, DataType>* node,
                      const Point3D<T>& point) const {
        if (!node || !node->boundary.contains(point)) {
            return false;
        }

        // Check points in this node
        for (const auto& [p, data] : node->points) {
            if (p == point) {
                return true;
            }
        }

        // Recursively check children
        if (node->subdivided) {
            for (const auto& child : node->children) {
                if (containsImpl(child.get(), point)) {
                    return true;
                }
            }
        }

        return false;
    }

    void traverseImpl(const OctreeNode<T, DataType>* node,
                     const std::function<void(const Point3D<T>&, const DataType&)>& visitor) const {
        if (!node) return;

        for (const auto& [point, data] : node->points) {
            visitor(point, data);
        }

        if (node->subdivided) {
            for (const auto& child : node->children) {
                traverseImpl(child.get(), visitor);
            }
        }
    }

    void getMemoryStatsImpl(const OctreeNode<T, DataType>* node, MemoryStats& stats) const {
        if (!node) return;

        ++stats.nodeCount;
        stats.pointCount += node->points.size();
        stats.totalBytes += sizeof(OctreeNode<T, DataType>);
        stats.totalBytes += node->points.capacity() * sizeof(std::pair<Point3D<T>, DataType>);

        if (node->subdivided) {
            for (const auto& child : node->children) {
                getMemoryStatsImpl(child.get(), stats);
            }
        }
    }
};

} // namespace octree
