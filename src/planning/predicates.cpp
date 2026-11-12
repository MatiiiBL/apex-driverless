// Built with -ffp-contract=off (src/planning/CMakeLists.txt): expansion arithmetic relies on
// every product and sum being rounded on its own, which a fused multiply-add would break.
#include "apex/planning/predicates.hpp"

#include <cmath>
#include <vector>

namespace apex::planning {

namespace {

// Relative rounding error of one double operation (round to nearest): half an ulp of 1.
constexpr double kEpsilon = 0x1p-53;
// Shewchuk's forward error bounds for the plain floating-point determinants, relative to the
// sum of the magnitudes of their terms ("ccwerrboundA" and "iccerrboundA").
constexpr double kOrientationBound = (3.0 + 16.0 * kEpsilon) * kEpsilon;
constexpr double kInCircleBound = (10.0 + 96.0 * kEpsilon) * kEpsilon;

// x = fl(a + b) and y its rounding error, so x + y == a + b exactly (Knuth's Two-Sum).
void two_sum(double a, double b, double& x, double& y) {
    x = a + b;
    const double b_virtual = x - a;
    const double a_virtual = x - b_virtual;
    y = (a - a_virtual) + (b - b_virtual);
}

// x = fl(a b) and y its rounding error, so x + y == a b exactly. std::fma rounds once.
void two_product(double a, double b, double& x, double& y) {
    x = a * b;
    y = std::fma(a, b, -x);
}

int sign_of(double value) {
    return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0);
}

// An exact real number as a sum of doubles that do not overlap bitwise, kept in increasing
// order of magnitude with zeros removed, so its sign is the sign of its largest term.
class Expansion {
public:
    Expansion() = default;

    // a - b, exactly.
    static Expansion difference(double a, double b) {
        double x = 0.0;
        double y = 0.0;
        two_sum(a, -b, x, y);
        Expansion e;
        e.add(y);
        e.add(x);
        return e;
    }

    // Shewchuk's GROW-EXPANSION with zero elimination: adds b exactly and keeps the
    // invariants (his Theorem 10).
    void add(double b) {
        double carry = b;
        std::size_t kept = 0;
        for (const double term : terms_) {
            double sum = 0.0;
            double error = 0.0;
            two_sum(carry, term, sum, error);
            carry = sum;
            if (error != 0.0) terms_[kept++] = error;
        }
        terms_.resize(kept);
        if (carry != 0.0) terms_.push_back(carry);
    }

    void add(const Expansion& other) {
        for (const double term : other.terms_) add(term);
    }

    void subtract(const Expansion& other) {
        for (const double term : other.terms_) add(-term);
    }

    // a b, exactly, added to this expansion.
    void add_product(double a, double b) {
        double x = 0.0;
        double y = 0.0;
        two_product(a, b, x, y);
        add(y);
        add(x);
    }

    [[nodiscard]] Expansion times(const Expansion& other) const {
        Expansion product;
        for (const double a : terms_) {
            for (const double b : other.terms_) product.add_product(a, b);
        }
        return product;
    }

    [[nodiscard]] int sign() const { return terms_.empty() ? 0 : sign_of(terms_.back()); }

private:
    std::vector<double> terms_;
};

// (a - c) x (b - c) expanded into six products of input coordinates, each exact as two terms.
int orientation_exact(const Vec2& a, const Vec2& b, const Vec2& c) {
    Expansion det;
    det.add_product(a.x(), b.y());
    det.add_product(-a.x(), c.y());
    det.add_product(-c.x(), b.y());
    det.add_product(-a.y(), b.x());
    det.add_product(a.y(), c.x());
    det.add_product(c.y(), b.x());
    return det.sign();
}

int in_circle_exact(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const Expansion adx = Expansion::difference(a.x(), d.x());
    const Expansion ady = Expansion::difference(a.y(), d.y());
    const Expansion bdx = Expansion::difference(b.x(), d.x());
    const Expansion bdy = Expansion::difference(b.y(), d.y());
    const Expansion cdx = Expansion::difference(c.x(), d.x());
    const Expansion cdy = Expansion::difference(c.y(), d.y());

    auto cross = [](const Expansion& ux, const Expansion& uy, const Expansion& vx,
                    const Expansion& vy) {
        Expansion result = ux.times(vy);
        result.subtract(uy.times(vx));
        return result;
    };
    auto lift = [](const Expansion& x, const Expansion& y) {
        Expansion result = x.times(x);
        result.add(y.times(y));
        return result;
    };
    Expansion det = lift(adx, ady).times(cross(bdx, bdy, cdx, cdy));
    det.add(lift(bdx, bdy).times(cross(cdx, cdy, adx, ady)));
    det.add(lift(cdx, cdy).times(cross(adx, ady, bdx, bdy)));
    return det.sign();
}

}  // namespace

int orientation(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double left = (a.x() - c.x()) * (b.y() - c.y());
    const double right = (a.y() - c.y()) * (b.x() - c.x());
    const double det = left - right;
    // Terms of opposite signs cannot cancel: the computed sign is right.
    double magnitude = 0.0;
    if (left > 0.0) {
        if (right <= 0.0) return sign_of(det);
        magnitude = left + right;
    } else if (left < 0.0) {
        if (right >= 0.0) return sign_of(det);
        magnitude = -left - right;
    } else {
        return sign_of(det);
    }
    const double bound = kOrientationBound * magnitude;
    if (det > bound || -det > bound) return sign_of(det);
    return orientation_exact(a, b, c);
}

// The 3x3 determinant of the points relative to d, each row (dx, dy, dx^2 + dy^2).
int in_circle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const double adx = a.x() - d.x();
    const double ady = a.y() - d.y();
    const double bdx = b.x() - d.x();
    const double bdy = b.y() - d.y();
    const double cdx = c.x() - d.x();
    const double cdy = c.y() - d.y();

    const double bdxcdy = bdx * cdy;
    const double cdxbdy = cdx * bdy;
    const double alift = adx * adx + ady * ady;
    const double cdxady = cdx * ady;
    const double adxcdy = adx * cdy;
    const double blift = bdx * bdx + bdy * bdy;
    const double adxbdy = adx * bdy;
    const double bdxady = bdx * ady;
    const double clift = cdx * cdx + cdy * cdy;

    const double det =
        alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);
    const double permanent = (std::abs(bdxcdy) + std::abs(cdxbdy)) * alift +
                             (std::abs(cdxady) + std::abs(adxcdy)) * blift +
                             (std::abs(adxbdy) + std::abs(bdxady)) * clift;
    const double bound = kInCircleBound * permanent;
    if (det > bound || -det > bound) return sign_of(det);
    return in_circle_exact(a, b, c, d);
}

}  // namespace apex::planning
