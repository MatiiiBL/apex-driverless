#include "apex/planning/delaunay.hpp"

#include <algorithm>
#include <optional>

#include "apex/planning/predicates.hpp"

namespace apex::planning {

namespace {

using Index = std::int32_t;

constexpr Index kInfinity = -1;  // the vertex at infinity that closes the hull

// A triangle of the mesh under construction. Ghost triangles (u, v, infinity) keep the vertex
// at infinity in slot 2; the open half-plane left of u -> v is their "circumcircle".
struct Triangle {
    std::array<Index, 3> vertex{};
    std::array<Index, 3> neighbor{};  // across the edge opposite vertex i; never missing
    std::uint32_t visit = 0;          // the insertion that last classified this triangle
    bool in_cavity = false;           // and its verdict
    bool alive = true;
};

constexpr std::size_t next(std::size_t i) {
    return (i + 1) % 3;
}
constexpr std::size_t prev(std::size_t i) {
    return (i + 2) % 3;
}

// True when b lies strictly between a and c.
bool between(double a, double b, double c) {
    return (a < b && b < c) || (c < b && b < a);
}

class Builder {
public:
    explicit Builder(std::span<const Vec2> points) : points_(points) {}

    Triangulation run() {
        const std::optional<std::array<std::size_t, 3>> seed = seed_triangle();
        if (!seed) return {};
        start((*seed)[0], (*seed)[1], (*seed)[2]);
        for (std::size_t i = 0; i < points_.size(); ++i) {
            if (i != (*seed)[0] && i != (*seed)[1] && i != (*seed)[2]) insert(i);
        }
        return result();
    }

private:
    // A cavity edge u -> v (counter-clockwise as seen from inside the cavity) and the triangle
    // beyond it, which survives.
    struct BoundaryEdge {
        Index u;
        Index v;
        Index outside;
        Index created;  // the new triangle (u, v, p)
    };

    [[nodiscard]] const Vec2& point(Index i) const { return points_[static_cast<std::size_t>(i)]; }
    Triangle& triangle(Index t) { return triangles_[static_cast<std::size_t>(t)]; }
    [[nodiscard]] static bool ghost(const Triangle& t) { return t.vertex[2] == kInfinity; }

    // The first point, the first one distinct from it, and the first one not collinear with both.
    [[nodiscard]] std::optional<std::array<std::size_t, 3>> seed_triangle() const {
        const std::size_t n = points_.size();
        std::size_t second = 1;
        while (second < n && points_[second] == points_[0]) ++second;
        if (second >= n) return std::nullopt;
        for (std::size_t third = 1; third < n; ++third) {
            const int side = orientation(points_[0], points_[second], points_[third]);
            if (side > 0) return std::array<std::size_t, 3>{0, second, third};
            if (side < 0) return std::array<std::size_t, 3>{0, third, second};
        }
        return std::nullopt;
    }

    // One solid triangle (counter-clockwise) and the three ghosts around it.
    void start(std::size_t a, std::size_t b, std::size_t c) {
        const auto ia = static_cast<Index>(a);
        const auto ib = static_cast<Index>(b);
        const auto ic = static_cast<Index>(c);
        add({ia, ib, ic}, {1, 2, 3});
        // Ghost across the edge opposite each vertex; a ghost's neighbor 2 is the solid one,
        // its neighbors 0 and 1 the ghosts across its edges to infinity.
        add({ic, ib, kInfinity}, {3, 2, 0});
        add({ia, ic, kInfinity}, {1, 3, 0});
        add({ib, ia, kInfinity}, {2, 1, 0});
        last_ = 0;
    }

    void add(const std::array<Index, 3>& vertex, const std::array<Index, 3>& neighbor) {
        Triangle t;
        t.vertex = vertex;
        t.neighbor = neighbor;
        triangles_.push_back(t);
    }

    // p lies in the open circumcircle of t. For a ghost that is the open half-plane beyond its
    // hull edge, plus the open edge itself (so a point on the hull splits the edge).
    [[nodiscard]] bool encloses(const Triangle& t, const Vec2& p) const {
        if (!ghost(t)) {
            return in_circle(point(t.vertex[0]), point(t.vertex[1]), point(t.vertex[2]), p) > 0;
        }
        const Vec2& u = point(t.vertex[0]);
        const Vec2& v = point(t.vertex[1]);
        const int side = orientation(u, v, p);
        if (side != 0) return side > 0;
        return u.x() != v.x() ? between(u.x(), p.x(), v.x()) : between(u.y(), p.y(), v.y());
    }

    // A triangle whose open circumcircle contains p, or that has p as a vertex: the solid
    // triangle containing p, or the ghost beyond the hull edge p is outside of. Visibility walk
    // from the last triangle created: step across any edge p is strictly beyond. It visits no
    // triangle twice in a Delaunay triangulation (Edelsbrunner, "An acyclicity theorem for
    // cell complexes in d dimensions", Combinatorica 10(3), 1990).
    Index locate(const Vec2& p) {
        Index t = last_;
        for (std::size_t steps = 0; steps <= triangles_.size(); ++steps) {
            const Triangle& current = triangle(t);
            if (ghost(current)) return t;
            Index step = t;
            for (std::size_t i = 0; i < 3 && step == t; ++i) {
                if (orientation(point(current.vertex[next(i)]), point(current.vertex[prev(i)]), p) <
                    0) {
                    step = current.neighbor[i];
                }
            }
            if (step == t) return t;
            t = step;
        }
        // The walk cannot cycle in a Delaunay triangulation; scan rather than loop if it did.
        for (std::size_t i = 0; i < triangles_.size(); ++i) {
            if (triangles_[i].alive && encloses(triangles_[i], p)) return static_cast<Index>(i);
        }
        return last_;
    }

    void insert(std::size_t index) {
        const Vec2& p = points_[index];
        const Index start_triangle = locate(p);
        for (const Index v : triangle(start_triangle).vertex) {
            if (v != kInfinity && point(v) == p) return;  // a duplicate
        }

        // The cavity: every triangle whose circumcircle contains p. It is connected and
        // star-shaped from p, so a flood fill from the located triangle finds it.
        ++visit_;
        cavity_.clear();
        boundary_.clear();
        triangle(start_triangle).visit = visit_;
        triangle(start_triangle).in_cavity = true;
        stack_.assign(1, start_triangle);
        while (!stack_.empty()) {
            const Index t = stack_.back();
            stack_.pop_back();
            cavity_.push_back(t);
            for (std::size_t i = 0; i < 3; ++i) {
                const Index n = triangle(t).neighbor[i];
                Triangle& beyond = triangle(n);
                if (beyond.visit != visit_) {
                    beyond.visit = visit_;
                    beyond.in_cavity = encloses(beyond, p);
                    if (beyond.in_cavity) stack_.push_back(n);
                }
                if (!beyond.in_cavity) {
                    boundary_.push_back(
                        {triangle(t).vertex[next(i)], triangle(t).vertex[prev(i)], n, -1});
                }
            }
        }

        // Replace the cavity by a fan of triangles (u, v, p), one per boundary edge.
        for (const Index t : cavity_) {
            triangle(t).alive = false;
            free_.push_back(t);
        }
        const auto ip = static_cast<Index>(index);
        for (BoundaryEdge& edge : boundary_) {
            edge.created = allocate({edge.u, edge.v, ip});
            Triangle& outside = triangle(edge.outside);
            for (std::size_t j = 0; j < 3; ++j) {
                if (outside.vertex[next(j)] == edge.v && outside.vertex[prev(j)] == edge.u) {
                    outside.neighbor[j] = edge.created;
                }
            }
        }
        for (const BoundaryEdge& edge : boundary_) {
            Triangle& created = triangle(edge.created);
            created.neighbor[2] = edge.outside;
            for (const BoundaryEdge& other : boundary_) {
                if (other.u == edge.v) created.neighbor[0] = other.created;  // across (v, p)
                if (other.v == edge.u) created.neighbor[1] = other.created;  // across (p, u)
            }
        }
        for (const BoundaryEdge& edge : boundary_) {
            Triangle& created = triangle(edge.created);
            // Ghosts keep infinity in slot 2: rotate (inf, v, p) to (v, p, inf) and (u, inf, p)
            // to (p, u, inf), with their neighbors.
            const std::size_t shift = created.vertex[0] == kInfinity   ? 1
                                      : created.vertex[1] == kInfinity ? 2
                                                                       : 0;
            if (shift == 0) {
                last_ = edge.created;
                continue;
            }
            std::rotate(created.vertex.begin(), created.vertex.begin() + shift,
                        created.vertex.end());
            std::rotate(created.neighbor.begin(), created.neighbor.begin() + shift,
                        created.neighbor.end());
        }
    }

    Index allocate(const std::array<Index, 3>& vertex) {
        Triangle fresh;
        fresh.vertex = vertex;
        if (free_.empty()) {
            triangles_.push_back(fresh);
            return static_cast<Index>(triangles_.size() - 1);
        }
        const Index t = free_.back();
        free_.pop_back();
        triangle(t) = fresh;
        return t;
    }

    // The solid triangles, renumbered; ghost neighbors become -1.
    Triangulation result() const {
        std::vector<Index> id(triangles_.size(), -1);
        Triangulation out;
        for (std::size_t t = 0; t < triangles_.size(); ++t) {
            if (triangles_[t].alive && !ghost(triangles_[t])) {
                id[t] = static_cast<Index>(out.triangles.size());
                out.triangles.push_back(triangles_[t].vertex);
            }
        }
        for (const Triangle& t : triangles_) {
            if (!t.alive || ghost(t)) continue;
            std::array<Index, 3> neighbors{};
            for (std::size_t i = 0; i < 3; ++i) {
                neighbors[i] = id[static_cast<std::size_t>(t.neighbor[i])];
            }
            out.neighbors.push_back(neighbors);
        }
        return out;
    }

    std::span<const Vec2> points_;
    std::vector<Triangle> triangles_;
    std::vector<Index> free_;
    std::uint32_t visit_ = 0;
    Index last_ = 0;  // a solid triangle near the last insertion, where the next walk starts
    std::vector<Index> stack_;
    std::vector<Index> cavity_;
    std::vector<BoundaryEdge> boundary_;
};

}  // namespace

Triangulation delaunay_triangulation(std::span<const Vec2> points) {
    return Builder(points).run();
}

}  // namespace apex::planning
