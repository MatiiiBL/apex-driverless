#include "apex/core/geometry.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace apex {
namespace {

TEST(Geometry, ProjectOnSegmentClampsToEnds) {
    const Vec2 a{0.0, 0.0};
    const Vec2 b{4.0, 0.0};
    const SegmentProjection mid = project_on_segment({1.0, 2.0}, a, b);
    EXPECT_DOUBLE_EQ(mid.t, 0.25);
    EXPECT_DOUBLE_EQ(mid.distance, 2.0);
    const SegmentProjection before = project_on_segment({-3.0, 4.0}, a, b);
    EXPECT_DOUBLE_EQ(before.t, 0.0);
    EXPECT_DOUBLE_EQ(before.distance, 5.0);
    EXPECT_DOUBLE_EQ(project_on_segment({1.0, 1.0}, a, a).distance, std::sqrt(2.0));
}

TEST(Geometry, SegmentIntersection) {
    const auto hit = segment_intersection({0, 0}, {2, 2}, {0, 2}, {2, 0});
    ASSERT_TRUE(hit.has_value());
    EXPECT_TRUE(hit->isApprox(Vec2{1, 1}));
    EXPECT_FALSE(segment_intersection({0, 0}, {1, 0}, {0, 1}, {1, 1}).has_value());  // parallel
    EXPECT_FALSE(segment_intersection({0, 0}, {1, 1}, {2, 0}, {3, -1}).has_value());
    EXPECT_TRUE(segment_intersection({0, 0}, {2, 0}, {1, 0}, {3, 0}).has_value());  // overlap
    EXPECT_TRUE(segment_intersection({0, 0}, {1, 0}, {1, 0}, {1, 1}).has_value());  // touching
}

TEST(Geometry, PolygonAreaAndContainment) {
    const std::vector<Vec2> square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    EXPECT_DOUBLE_EQ(polygon_signed_area(square), 4.0);
    const std::vector<Vec2> clockwise(square.rbegin(), square.rend());
    EXPECT_DOUBLE_EQ(polygon_signed_area(clockwise), -4.0);
    EXPECT_TRUE(point_in_polygon({1.0, 1.0}, square));
    EXPECT_FALSE(point_in_polygon({3.0, 1.0}, square));
}

TEST(Geometry, SelfIntersection) {
    const std::vector<Vec2> square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    EXPECT_FALSE(polyline_self_intersects(square, true));
    const std::vector<Vec2> bowtie{{0, 0}, {2, 2}, {2, 0}, {0, 2}};
    EXPECT_TRUE(polyline_self_intersects(bowtie, true));
    EXPECT_FALSE(polyline_self_intersects(bowtie, false) &&
                 !polyline_self_intersects(bowtie, true));
    const std::vector<Vec2> other{{1, -1}, {1, 3}};
    EXPECT_TRUE(polylines_intersect(square, true, other, false));
}

TEST(Geometry, CumulativeLengthAndDistance) {
    const std::vector<Vec2> square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
    const std::vector<double> open = cumulative_length(square, false);
    ASSERT_EQ(open.size(), 4U);
    EXPECT_DOUBLE_EQ(open.back(), 6.0);
    const std::vector<double> closed = cumulative_length(square, true);
    ASSERT_EQ(closed.size(), 5U);
    EXPECT_DOUBLE_EQ(closed.back(), 8.0);
    EXPECT_DOUBLE_EQ(distance_to_polyline({1.0, -3.0}, square, true), 3.0);
    // The open square has no left side: the nearest points are its two ends.
    EXPECT_DOUBLE_EQ(distance_to_polyline({-1.0, 1.0}, square, false), std::sqrt(2.0));
    EXPECT_DOUBLE_EQ(distance_to_polyline({-1.0, 1.0}, square, true), 1.0);
}

TEST(Geometry, MengerCurvatureOfACircle) {
    const double r = 5.0;
    const Vec2 a = r * heading_vector(0.0);
    const Vec2 b = r * heading_vector(0.4);
    const Vec2 c = r * heading_vector(0.8);
    EXPECT_NEAR(menger_curvature(a, b, c), 1.0 / r, 1e-12);  // counter-clockwise: left turn
    EXPECT_NEAR(menger_curvature(c, b, a), -1.0 / r, 1e-12);
    EXPECT_DOUBLE_EQ(menger_curvature({0, 0}, {1, 0}, {2, 0}), 0.0);
}

}  // namespace
}  // namespace apex
