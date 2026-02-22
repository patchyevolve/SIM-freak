#pragma once
// =============================================================================
//  math/Vec2.h  — Immutable-style 2D vector for simSUS physics engine
//  All operations are in metres (SI units).
// =============================================================================

#include <cmath>
#include <stdexcept>
#include <string>

struct Vec2
{
    double x = 0.0;
    double y = 0.0;

    // ── Construction ──────────────────────────────────────────────────────────
    constexpr Vec2() = default;
    constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}

    // ── Arithmetic operators ──────────────────────────────────────────────────
    constexpr Vec2 operator+(const Vec2& o) const { return { x + o.x, y + o.y }; }
    constexpr Vec2 operator-(const Vec2& o) const { return { x - o.x, y - o.y }; }
    constexpr Vec2 operator*(double s)      const { return { x * s,   y * s   }; }
    constexpr Vec2 operator/(double s)      const { return { x / s,   y / s   }; }
    constexpr Vec2 operator-()              const { return { -x, -y }; }

    constexpr Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    constexpr Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    constexpr Vec2& operator*=(double s)      { x *= s;   y *= s;   return *this; }
    constexpr Vec2& operator/=(double s)      { x /= s;   y /= s;   return *this; }

    // ── Comparison ────────────────────────────────────────────────────────────
    constexpr bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
    constexpr bool operator!=(const Vec2& o) const { return !(*this == o); }

    // ── Core math ─────────────────────────────────────────────────────────────
    /// Squared magnitude (avoids sqrt — use for comparisons)
    constexpr double norm_sq() const { return x * x + y * y; }

    /// Euclidean magnitude
    double norm() const { return std::sqrt(norm_sq()); }

    /// Dot product
    constexpr double dot(const Vec2& o) const { return x * o.x + y * o.y; }

    /// Unit vector; throws if the vector is zero-length
    Vec2 normalized() const
    {
        double n = norm();
        if (n < 1e-300)
            throw std::domain_error("Vec2::normalized() called on zero vector");
        return *this / n;
    }

    /// Distance to another point
    double dist_to(const Vec2& o) const { return (*this - o).norm(); }

    /// Squared distance (avoids sqrt)
    constexpr double dist_sq_to(const Vec2& o) const { return (*this - o).norm_sq(); }

    /// True if all components are finite (no NaN / Inf)
    bool is_finite() const { return std::isfinite(x) && std::isfinite(y); }

    // ── Debug ─────────────────────────────────────────────────────────────────
    std::string to_string() const
    {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

    // ── Tests (call from main with --test) ────────────────────────────────────
    static bool RunTests();
};

// Free scalar–vector multiply  (s * v)
inline constexpr Vec2 operator*(double s, const Vec2& v) { return v * s; }
