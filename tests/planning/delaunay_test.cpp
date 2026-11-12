#include "apex/planning/delaunay.hpp"

#include <set>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"
#include "apex/planning/predicates.hpp"

namespace apex::planning {
namespace {

std::size_t distinct_count(const std::vector<Vec2>& points) {
    std::set<std::pair<double, double>> unique;
    for (const Vec2& p : points) unique.insert({p.x(), p.y()});
    return unique.size();
}

// Checks every property of a Delaunay triangulation of `points` against brute force:
//  - triangles are counter-clockwise and non-degenerate, with consistent neighbors;
//  - no point lies strictly inside any triangle's circumcircle (the oracle);
//  - the hull edges have every point on their inner side or on them, every distinct point is a
//    vertex, and Euler's formula T = 2 n - 2 - h holds, so the triangles tile the convex hull.
void expect_delaunay(const std::vector<Vec2>& points, const Triangulation& mesh) {
    ASSERT_EQ(mesh.triangles.size(), mesh.neighbors.size());
    const auto at = [&points](std::int32_t i) { return points[static_cast<std::size_t>(i)]; };
    std::set<std::pair<double, double>> used;
    std::size_t hull_edges = 0;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const auto& tri = mesh.triangles[t];
        ASSERT_EQ(orientation(at(tri[0]), at(tri[1]), at(tri[2])), 1) << "triangle " << t;
        for (std::size_t i = 0; i < 3; ++i) {
            used.insert({at(tri[i]).x(), at(tri[i]).y()});
            const std::int32_t a = tri[(i + 1) % 3];
            const std::int32_t b = tri[(i + 2) % 3];
            const std::int32_t n = mesh.neighbors[t][i];
            if (n < 0) {
                ++hull_edges;
                for (const Vec2& p : points) EXPECT_GE(orientation(at(a), at(b), p), 0);
                continue;
            }
            // The neighbor has the same edge, reversed, and points back.
            const auto& other = mesh.triangles[static_cast<std::size_t>(n)];
            bool found = false;
            for (std::size_t j = 0; j < 3; ++j) {
                if (other[(j + 1) % 3] == b && other[(j + 2) % 3] == a) {
                    found = true;
                    EXPECT_EQ(mesh.neighbors[static_cast<std::size_t>(n)][j],
                              static_cast<std::int32_t>(t));
                }
            }
            EXPECT_TRUE(found) << "triangle " << t << " edge " << i;
        }
        for (const Vec2& p : points) {
            EXPECT_LE(in_circle(at(tri[0]), at(tri[1]), at(tri[2]), p), 0)
                << "a point inside the circumcircle of triangle " << t;
        }
    }
    const std::size_t n = distinct_count(points);
    EXPECT_EQ(used.size(), n);
    EXPECT_EQ(mesh.triangles.size() + 2 + hull_edges, 2 * n);
}

std::vector<Vec2> uniform_points(std::size_t n, std::uint64_t seed) {
    Rng rng(seed);
    std::vector<Vec2> points(n);
    for (Vec2& p : points) p = {rng.uniform(0.0, 100.0), rng.uniform(0.0, 100.0)};
    return points;
}

TEST(Delaunay, RandomPointSets) {
    for (const std::size_t n : {3U, 4U, 10U, 100U, 400U}) {
        for (std::uint64_t seed = 1; seed <= 3; ++seed) {
            const std::vector<Vec2> points = uniform_points(n, seed);
            expect_delaunay(points, delaunay_triangulation(points));
        }
    }
}

// Two noisy lines of cones, as the planner sees a straight: long, thin, nearly degenerate.
TEST(Delaunay, ConeLikeStrips) {
    Rng rng(3);
    std::vector<Vec2> points;
    for (int i = 0; i < 40; ++i) {
        points.push_back({2.5 * i + rng.normal(0, 0.05), 1.75 + rng.normal(0, 0.05)});
        points.push_back({2.5 * i + rng.normal(0, 0.05), -1.75 + rng.normal(0, 0.05)});
    }
    expect_delaunay(points, delaunay_triangulation(points));
}

TEST(Delaunay, TooFewOrCollinearPointsGiveNoTriangles) {
    EXPECT_TRUE(delaunay_triangulation(std::vector<Vec2>{}).triangles.empty());
    EXPECT_TRUE(delaunay_triangulation(std::vector<Vec2>{{1, 1}}).triangles.empty());
    EXPECT_TRUE(
        delaunay_triangulation(std::vector<Vec2>{{1, 1}, {1, 1}, {1, 1}}).triangles.empty());
    // Exactly collinear: {0.1 i, 0.3 i} would not be, after rounding, and the exact predicates
    // would rightly triangulate the slivers.
    std::vector<Vec2> line;
    for (int i = 0; i < 20; ++i) line.push_back({0.25 * i, 0.75 * i});
    EXPECT_TRUE(delaunay_triangulation(line).triangles.empty());
}

TEST(Delaunay, CollinearPointsAndOneApex) {
    std::vector<Vec2> points;
    for (int i = 0; i < 20; ++i) points.push_back({static_cast<double>(i), 0.0});
    points.push_back({7.5, 3.0});
    const Triangulation mesh = delaunay_triangulation(points);
    expect_delaunay(points, mesh);
    EXPECT_EQ(mesh.triangles.size(), 19U);  // a fan from the apex
}

TEST(Delaunay, CollinearPointsInsertedLast) {
    // A triangle first, then points along its edges and their extensions.
    std::vector<Vec2> points{{0, 0}, {10, 0}, {0, 10}};
    for (int i = -5; i <= 15; ++i) {
        points.push_back({static_cast<double>(i), 0.0});
        points.push_back({0.0, static_cast<double>(i)});
        points.push_back({static_cast<double>(i), static_cast<double>(10 - i)});
    }
    expect_delaunay(points, delaunay_triangulation(points));
}

TEST(Delaunay, DuplicatesAreLeftOut) {
    std::vector<Vec2> points = uniform_points(60, 11);
    const Triangulation unique = delaunay_triangulation(points);
    // Every point twice more, interleaved and at the front.
    std::vector<Vec2> repeated = points;
    for (std::size_t i = 0; i < points.size(); ++i) {
        repeated.insert(repeated.begin() + static_cast<std::ptrdiff_t>(2 * i), points[i]);
        repeated.push_back(points[points.size() - 1 - i]);
    }
    const Triangulation mesh = delaunay_triangulation(repeated);
    expect_delaunay(repeated, mesh);
    EXPECT_EQ(mesh.triangles.size(), unique.triangles.size());
}

// All 48 integer points on x^2 + y^2 = 5525 (5^2 13 17), so every quadruple is cocircular,
// with and without the center, and shifted far from the origin.
TEST(Delaunay, CocircularPoints) {
    std::vector<Vec2> circle;
    for (int x = -74; x <= 74; ++x) {
        for (int y = -74; y <= 74; ++y) {
            if (x * x + y * y == 5525)
                circle.push_back({static_cast<double>(x), static_cast<double>(y)});
        }
    }
    ASSERT_EQ(circle.size(), 48U);
    for (const double offset : {0.0, 1e6}) {
        std::vector<Vec2> points = circle;
        for (Vec2& p : points) p += Vec2{offset, offset};
        const Triangulation ring = delaunay_triangulation(points);
        expect_delaunay(points, ring);
        EXPECT_EQ(ring.triangles.size(), 46U);
        points.push_back(Vec2{offset, offset});
        expect_delaunay(points, delaunay_triangulation(points));
    }
}

TEST(Delaunay, GridsExactAndPerturbed) {
    std::vector<Vec2> grid;
    for (int i = 0; i < 15; ++i) {
        for (int j = 0; j < 15; ++j) grid.push_back({0.5 * i, 0.5 * j});
    }
    const Triangulation exact = delaunay_triangulation(grid);
    expect_delaunay(grid, exact);
    EXPECT_EQ(exact.triangles.size(), 2U * 14U * 14U);

    // Perturbations of a few ulps keep the predicates on their exact fallback.
    Rng rng(5);
    std::vector<Vec2> perturbed = grid;
    for (Vec2& p : perturbed) {
        p.x() += 1e-15 * static_cast<double>(rng.index(5)) * (rng.bernoulli(0.5) ? 1 : -1);
        p.y() += 1e-15 * static_cast<double>(rng.index(5)) * (rng.bernoulli(0.5) ? 1 : -1);
    }
    expect_delaunay(perturbed, delaunay_triangulation(perturbed));
}

TEST(Delaunay, DeterministicForTheSameInput) {
    const std::vector<Vec2> points = uniform_points(200, 4);
    const Triangulation a = delaunay_triangulation(points);
    const Triangulation b = delaunay_triangulation(points);
    EXPECT_EQ(a.triangles, b.triangles);
    EXPECT_EQ(a.neighbors, b.neighbors);
}

}  // namespace
}  // namespace apex::planning
