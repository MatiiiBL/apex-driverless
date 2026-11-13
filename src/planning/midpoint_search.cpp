#include "apex/planning/midpoint_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/planning/delaunay.hpp"
#include "apex/planning/predicates.hpp"

namespace apex::planning {

namespace {

using Index = std::int32_t;

// Steps between midpoints shorter than this (m) keep the previous direction: their own
// direction is mostly cone noise.
constexpr double kMinStep = 0.5;

// When no edge is within start_radius of the car, first edges are taken this far (m) beyond
// the nearest one, enough for the two or three edges of the first pair of cones.
constexpr double kStartWindow = 2.0;

double square(double x) {
    return x * x;
}

// An edge to cross and what it says about the track there.
struct Crossing {
    Index left;  // the cone that ends up on the path's left
    Index right;
    double width;    // of the track at this edge (m)
    double spacing;  // length of the cone-line segment beside the triangle crossed (m), 0 first
};

// A candidate path, as its last crossed edge and a link to the rest.
struct Node {
    Index triangle = -1;  // entered by crossing the edge; -1 beyond the convex hull
    Index entry = -1;     // that edge's index in `triangle` (the vertex opposite it)
    Index parent = -1;    // index into the search's nodes, -1 for the first edge
    Index left = -1;
    Index right = -1;
    Vec2 point = Vec2::Zero();      // the edge's midpoint
    Vec2 direction = Vec2::Zero();  // of travel, unit
    double step = 0.0;              // from the previous point (m)
    double length = 0.0;
    double cost = 0.0;
};

class Search {
public:
    Search(std::span<const SearchCone> cones, const Pose2& car, const ExplorationConfig& config)
        : cones_(cones), config_(config), car_(car) {
        std::vector<Vec2> points;
        points.reserve(cones.size());
        for (const SearchCone& cone : cones) points.push_back(cone.position);
        mesh_ = delaunay_triangulation(points);
        best_entry_.assign(3 * mesh_.triangles.size(), std::numeric_limits<double>::infinity());
        max_turn_ = deg_to_rad(config.max_turn_deg);
        heading_scale_ = deg_to_rad(config.heading_scale_deg);
    }

    std::optional<MidpointPath> run() {
        std::vector<Index> frontier = start();
        for (std::size_t depth = 1; depth < config_.max_edges && !frontier.empty(); ++depth) {
            std::vector<Index> candidates;
            for (const Index n : frontier) expand(n, candidates);
            keep_best(candidates);
            frontier = std::move(candidates);
        }
        if (nodes_.empty()) return std::nullopt;
        Index best = 0;
        for (Index n = 1; n < static_cast<Index>(nodes_.size()); ++n) {
            if (node(n).cost < node(best).cost) best = n;
        }
        return backtrack(best);
    }

private:
    [[nodiscard]] const Node& node(Index n) const { return nodes_[static_cast<std::size_t>(n)]; }
    [[nodiscard]] const Vec2& cone(Index i) const {
        return cones_[static_cast<std::size_t>(i)].position;
    }
    [[nodiscard]] const std::array<Index, 3>& vertices(Index t) const {
        return mesh_.triangles[static_cast<std::size_t>(t)];
    }

    // The car as the path's origin: a node with no edge, heading where the car heads.
    [[nodiscard]] Node origin() const {
        Node n;
        n.point = car_.position();
        n.direction = heading_vector(car_.yaw);
        return n;
    }

    // First edges: every edge with its midpoint ahead of the car, crossed away from the car. A
    // triangle's edge u -> v (counter-clockwise, so the triangle is on its left) with the car
    // strictly on its right is crossed into the triangle, with u on the path's left; a hull edge
    // with the car on its inner side is crossed out of the hull. Edges count within
    // start_radius, or, when no edge that close passes the hard limits (the cones beside the car
    // were never mapped, as when noisy perception loses the big orange cones at the staging
    // position), within kStartWindow of the nearest edge that does, up to cone_range: the path
    // then starts with a straight run from the car to its first edge.
    std::vector<Index> start() {
        struct Candidate {
            Index triangle;
            Index edge;
            bool into_triangle;  // false: out of the hull
            double distance;
        };
        std::vector<Candidate> candidates;
        for (std::size_t t = 0; t < mesh_.triangles.size(); ++t) {
            const auto& v = mesh_.triangles[t];
            for (std::size_t i = 0; i < 3; ++i) {
                const Index u = v[(i + 1) % 3];
                const Index w = v[(i + 2) % 3];
                const Vec2 local = car_.to_local(0.5 * (cone(u) + cone(w)));
                if (local.x() <= 0.0 || local.norm() > config_.cone_range) continue;
                const int side = orientation(cone(u), cone(w), car_.position());
                const bool into = side < 0;
                if (!into && !(side > 0 && mesh_.neighbors[t][i] < 0)) continue;
                candidates.push_back(
                    {static_cast<Index>(t), static_cast<Index>(i), into, local.norm()});
            }
        }
        std::stable_sort(
            candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });

        const Node from = origin();
        std::vector<Index> created;
        const auto try_edge = [&](const Candidate& c) {
            const auto& v = vertices(c.triangle);
            const Index u = v[static_cast<std::size_t>((c.edge + 1) % 3)];
            const Index w = v[static_cast<std::size_t>((c.edge + 2) % 3)];
            // No cone line yet: the width is the edge's extent across the car's heading.
            const double width = std::abs(cross(from.direction, cone(u) - cone(w)));
            if (c.into_triangle) {
                add(from, -1, {u, w, width, 0.0}, c.triangle, c.edge, created);
            } else {
                add(from, -1, {w, u, width, 0.0}, -1, -1, created);
            }
        };
        std::size_t next = 0;
        for (; next < candidates.size() && candidates[next].distance <= config_.start_radius;
             ++next) {
            try_edge(candidates[next]);
        }
        if (created.empty()) {
            double reach = std::numeric_limits<double>::infinity();
            for (; next < candidates.size() && candidates[next].distance <= reach; ++next) {
                try_edge(candidates[next]);
                if (!created.empty() && std::isinf(reach)) {
                    reach = candidates[next].distance + kStartWindow;
                }
            }
        }
        keep_best(created);
        return created;
    }

    // Children of node n: out of its triangle through either other edge.
    void expand(Index n, std::vector<Index>& created) {
        const Node from = node(n);
        if (from.triangle < 0 || from.length >= config_.max_path_length) return;
        const auto& v = vertices(from.triangle);
        const auto& neighbors = mesh_.neighbors[static_cast<std::size_t>(from.triangle)];
        const auto entry = static_cast<std::size_t>(from.entry);
        for (const std::size_t exit : {(entry + 1) % 3, (entry + 2) % 3}) {
            const Index beyond = neighbors[exit];
            if (beyond >= 0 && visited(n, beyond)) continue;
            // Leaving through the edge opposite vertex `exit`: v[exit + 2] is on the left. The
            // edge not crossed, from the vertex opposite the entry to the one opposite the exit,
            // is a segment of one cone line; the third vertex, shared by both crossed edges, is
            // on the other line, and its distance to that segment is the track's width there,
            // whatever the direction of travel and however long the crossed diagonals. (It
            // overestimates in the fan of triangles on the outside of a tight hairpin, where the
            // inner cone faces several outer ones; the edge's extent across the direction of
            // travel overestimates on long diagonals, which are far more common.)
            const Vec2& line_from = cone(v[entry]);
            const Vec2& line_to = cone(v[exit]);
            const Vec2& opposite = cone(v[3 - entry - exit]);
            const Crossing crossing{v[(exit + 2) % 3], v[(exit + 1) % 3],
                                    project_on_segment(opposite, line_from, line_to).distance,
                                    (line_to - line_from).norm()};
            Index beyond_entry = -1;
            if (beyond >= 0) {
                const auto& back = mesh_.neighbors[static_cast<std::size_t>(beyond)];
                beyond_entry = static_cast<Index>(std::distance(
                    back.begin(), std::find(back.begin(), back.end(), from.triangle)));
            }
            add(from, n, crossing, beyond, beyond_entry, created);
        }
    }

    // Every cone at least min_cone_clearance from the segment a-b. Without colors, a path
    // across the infield of a hairpin can look like a straight corridor between the two
    // inner cone lines; the cones of the inner line it cuts through stand in that corridor.
    [[nodiscard]] bool clear_of_cones(const Vec2& a, const Vec2& b) const {
        return std::all_of(cones_.begin(), cones_.end(), [&](const SearchCone& c) {
            return project_on_segment(c.position, a, b).distance >= config_.min_cone_clearance;
        });
    }

    [[nodiscard]] bool visited(Index n, Index triangle) const {
        for (Index k = n; k >= 0; k = node(k).parent) {
            if (node(k).triangle == triangle) return true;
        }
        return false;
    }

    // Scores crossing an edge after `from`, and records it unless a hard limit or a cheaper
    // path into the same triangle edge rules it out.
    void add(const Node& from, Index parent, const Crossing& crossing, Index triangle, Index entry,
             std::vector<Index>& created) {
        Node child;
        child.triangle = triangle;
        child.entry = entry;
        child.parent = parent;
        child.left = crossing.left;
        child.right = crossing.right;
        child.point = 0.5 * (cone(crossing.left) + cone(crossing.right));
        const Vec2 delta = child.point - from.point;
        child.step = delta.norm();
        child.direction = child.step >= kMinStep ? Vec2(delta / child.step) : from.direction;
        child.length = from.length + child.step;

        const double width = crossing.width;
        if ((cone(crossing.left) - cone(crossing.right)).norm() > config_.max_edge_length ||
            width < config_.min_width || width > config_.max_width) {
            return;
        }
        const bool first = parent < 0;
        if (!first && crossing.spacing > config_.max_cone_spacing) return;
        // Only the midpoint for the first edge: the car itself may stand close to cones, and the
        // straight run out to a first edge beyond start_radius is no path in a curve (the
        // smoothed path turns).
        if (!clear_of_cones(first ? child.point : from.point, child.point)) return;
        double cost = from.cost - config_.length_reward * child.step;
        cost += config_.width_weight * square((width - config_.track_width) / config_.track_width);
        const double wrong_left =
            std::max(0.0, -cones_[static_cast<std::size_t>(crossing.left)].side);
        const double wrong_right =
            std::max(0.0, cones_[static_cast<std::size_t>(crossing.right)].side);
        cost += config_.color_weight * (wrong_left + wrong_right);
        if (!first) {
            cost += config_.spacing_weight *
                    square(std::max(0.0, crossing.spacing - config_.cone_spacing) /
                           config_.cone_spacing);
        }
        if (child.step >= kMinStep) {
            const double turn = std::abs(std::atan2(cross(from.direction, child.direction),
                                                    from.direction.dot(child.direction)));
            if (turn > max_turn_) return;
            if (first) {
                cost += config_.heading_weight * square(turn / heading_scale_);
            } else {
                const double curvature = turn / std::max(kMinStep, 0.5 * (child.step + from.step));
                cost += config_.curvature_weight * square(curvature / config_.curvature_scale);
            }
        }
        child.cost = cost;

        if (triangle >= 0) {
            double& best = best_entry_[static_cast<std::size_t>(3 * triangle + entry)];
            if (cost >= best) return;
            best = cost;
        }
        created.push_back(static_cast<Index>(nodes_.size()));
        nodes_.push_back(child);
    }

    // The beam: the cheapest beam_width candidates, ties in creation order.
    void keep_best(std::vector<Index>& candidates) const {
        std::stable_sort(candidates.begin(), candidates.end(),
                         [this](Index a, Index b) { return node(a).cost < node(b).cost; });
        if (candidates.size() > config_.beam_width) candidates.resize(config_.beam_width);
    }

    [[nodiscard]] MidpointPath backtrack(Index best) const {
        MidpointPath path;
        path.length = node(best).length;
        path.cost = node(best).cost;
        for (Index k = best; k >= 0; k = node(k).parent) {
            path.midpoints.push_back(node(k).point);
            path.left.push_back(node(k).left);
            path.right.push_back(node(k).right);
        }
        std::reverse(path.midpoints.begin(), path.midpoints.end());
        std::reverse(path.left.begin(), path.left.end());
        std::reverse(path.right.begin(), path.right.end());
        return path;
    }

    std::span<const SearchCone> cones_;
    const ExplorationConfig& config_;
    Pose2 car_;
    Triangulation mesh_;
    std::vector<Node> nodes_;
    std::vector<double> best_entry_;  // cheapest cost into each (triangle, edge)
    double max_turn_ = 0.0;
    double heading_scale_ = 1.0;
};

}  // namespace

std::optional<MidpointPath> search_midpoint_path(std::span<const SearchCone> cones,
                                                 const Pose2& car,
                                                 const ExplorationConfig& config) {
    if (cones.size() < 3) return std::nullopt;
    return Search(cones, car, config).run();
}

}  // namespace apex::planning
