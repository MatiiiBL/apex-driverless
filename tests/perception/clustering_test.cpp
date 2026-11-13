#include "apex/perception/clustering.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"

namespace apex::perception {
namespace {

ObstaclePoint at(double x, double y, std::int32_t id = -1) {
    return {static_cast<float>(x), static_cast<float>(y), 0.1F, id};
}

// The cluster index of every point, -1 for points in no cluster.
std::vector<int> labels(const Clusters& clusters, std::size_t points) {
    std::vector<int> label(points, -1);
    for (std::size_t k = 0; k < clusters.size(); ++k) {
        for (const std::uint32_t i : clusters[k]) label[i] = static_cast<int>(k);
    }
    return label;
}

TEST(GridClusterer, EmptyInputGivesNoClusters) {
    GridClusterer clusterer(ClusterConfig{}, LidarGeometry{});
    Clusters clusters;
    clusterer.cluster({}, clusters);
    EXPECT_EQ(clusters.size(), 0U);
    EXPECT_TRUE(clusters.members.empty());
}

TEST(GridClusterer, PointsCloserThanACellAlwaysJoin) {
    const ClusterConfig config;
    GridClusterer clusterer(config, LidarGeometry{});
    Rng rng(11);
    for (int trial = 0; trial < 500; ++trial) {
        const double x = rng.uniform(1.0, 39.0);
        const double y = rng.uniform(-20.0, 20.0);
        const double angle = rng.uniform(0.0, 6.283);
        const double d = rng.uniform(0.0, 0.999 * config.cell_size);
        const std::vector<ObstaclePoint> points = {
            at(x, y), at(x + d * std::cos(angle), y + d * std::sin(angle))};
        Clusters clusters;
        clusterer.cluster(points, clusters);
        ASSERT_EQ(clusters.size(), 1U) << "trial " << trial;
    }
}

TEST(GridClusterer, PointsFartherThanTheLinkDistanceStayApart) {
    const ClusterConfig config;
    GridClusterer clusterer(config, LidarGeometry{});
    const double link = 2.0 * std::sqrt(2.0) * config.cell_size;
    Rng rng(12);
    for (int trial = 0; trial < 500; ++trial) {
        const double x = rng.uniform(1.0, 30.0);
        const double y = rng.uniform(-20.0, 20.0);
        const double angle = rng.uniform(0.0, 6.283);
        const double d = rng.uniform(1.001 * link, 1.0);
        const std::vector<ObstaclePoint> points = {
            at(x, y), at(x + d * std::cos(angle), y + d * std::sin(angle))};
        Clusters clusters;
        clusterer.cluster(points, clusters);
        ASSERT_EQ(clusters.size(), 2U) << "trial " << trial;
    }
}

TEST(GridClusterer, ChainsAndSeparatesBlobs) {
    GridClusterer clusterer(ClusterConfig{}, LidarGeometry{});
    std::vector<ObstaclePoint> points;
    // An L-shaped chain of points 0.1 m apart: one cluster however it bends.
    for (int i = 0; i < 20; ++i) points.push_back(at(5.0 + 0.1 * i, 0.0, 1));
    for (int i = 1; i < 20; ++i) points.push_back(at(6.9, 0.1 * i, 1));
    // Two cone-sized blobs 0.6 m apart, surface to surface.
    for (int i = 0; i < 8; ++i) points.push_back(at(10.0, -2.0 + 0.03 * i, 2));
    for (int i = 0; i < 8; ++i) points.push_back(at(10.0, -2.0 + 0.21 + 0.6 + 0.03 * i, 3));
    Clusters clusters;
    clusterer.cluster(points, clusters);
    ASSERT_EQ(clusters.size(), 3U);
    const std::vector<int> label = labels(clusters, points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = 0; j < points.size(); ++j) {
            EXPECT_EQ(label[i] == label[j], points[i].truth_id == points[j].truth_id);
        }
    }
}

TEST(GridClusterer, EveryPointInsideTheGridLandsInExactlyOneCluster) {
    GridClusterer clusterer(ClusterConfig{}, LidarGeometry{});
    Rng rng(4);
    std::vector<ObstaclePoint> points;
    for (int i = 0; i < 3000; ++i) {
        const double x = rng.uniform(0.0, 39.0);
        const double y = rng.uniform(-39.0, 39.0);
        points.push_back(at(x, y));
    }
    // Behind a LiDAR that only looks forward: outside the grid, so in no cluster.
    points.push_back(at(-5.0, 0.0));
    points.push_back(at(-0.5, 60.0));
    Clusters clusters;
    clusterer.cluster(points, clusters);
    std::vector<int> seen(points.size(), 0);
    for (const std::uint32_t i : clusters.members) ++seen[i];
    for (std::size_t i = 0; i + 2 < points.size(); ++i) EXPECT_EQ(seen[i], 1);
    EXPECT_EQ(seen[points.size() - 2], 0);
    EXPECT_EQ(seen[points.size() - 1], 0);
}

TEST(GridClusterer, IsDeterministicAndKeepsNoStateBetweenScans) {
    GridClusterer clusterer(ClusterConfig{}, LidarGeometry{});
    Rng rng(5);
    std::vector<ObstaclePoint> first;
    std::vector<ObstaclePoint> second;
    for (int i = 0; i < 400; ++i) {
        const double x = rng.uniform(1.0, 20.0);
        const double y = rng.uniform(-5.0, 5.0);
        first.push_back(at(x, y));
    }
    for (int i = 0; i < 400; ++i) {
        const double x = rng.uniform(1.0, 20.0);
        const double y = rng.uniform(-5.0, 5.0);
        second.push_back(at(x, y));
    }
    Clusters a;
    Clusters b;
    Clusters c;
    clusterer.cluster(first, a);
    clusterer.cluster(second, b);
    clusterer.cluster(first, c);
    EXPECT_EQ(a.members, c.members);
    EXPECT_EQ(a.offsets, c.offsets);
    GridClusterer fresh(ClusterConfig{}, LidarGeometry{});
    Clusters d;
    fresh.cluster(second, d);
    EXPECT_EQ(b.members, d.members);
    EXPECT_EQ(b.offsets, d.offsets);
}

TEST(GridClusterer, RejectsBadConfig) {
    ClusterConfig config;
    config.cell_size = 0.0;
    EXPECT_THROW(GridClusterer(config, LidarGeometry{}), std::invalid_argument);
    LidarGeometry geometry;
    geometry.azimuth_max = geometry.azimuth_min;
    EXPECT_THROW(GridClusterer(ClusterConfig{}, geometry), std::invalid_argument);
}

}  // namespace
}  // namespace apex::perception
