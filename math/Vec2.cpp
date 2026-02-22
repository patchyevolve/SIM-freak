// =============================================================================
//  math/Vec2.cpp  — RunTests() for Vec2
//  (The struct itself is header-only; this file just holds the test suite.)
// =============================================================================

#include "Vec2.h"
#include <iostream>
#include <cassert>
#include <cmath>

bool Vec2::RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Vec2::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Vec2::" << name << "\n";
    };

    // Arithmetic
    Vec2 a{3.0, 4.0}, b{1.0, 2.0};
    check("add",   (a + b) == Vec2{4.0, 6.0});
    check("sub",   (a - b) == Vec2{2.0, 2.0});
    check("scale", (a * 2.0) == Vec2{6.0, 8.0});
    check("div",   (a / 2.0) == Vec2{1.5, 2.0});
    check("neg",   (-a)      == Vec2{-3.0, -4.0});
    check("scalar_mul", (2.0 * a) == Vec2{6.0, 8.0});

    // In-place
    Vec2 c{1.0, 1.0};
    c += Vec2{2.0, 3.0};
    check("iadd", c == Vec2{3.0, 4.0});
    c -= Vec2{1.0, 1.0};
    check("isub", c == Vec2{2.0, 3.0});
    c *= 2.0;
    check("imul", c == Vec2{4.0, 6.0});
    c /= 2.0;
    check("idiv", c == Vec2{2.0, 3.0});

    // Magnitude
    check("norm_sq",   std::abs(a.norm_sq() - 25.0) < 1e-12);
    check("norm",      std::abs(a.norm()    -  5.0) < 1e-12);

    // Dot product: (3,4)·(1,2) = 11
    check("dot", std::abs(a.dot(b) - 11.0) < 1e-12);

    // Normalize: {3,4} → {0.6, 0.8}
    Vec2 n = a.normalized();
    check("normalized_x", std::abs(n.x - 0.6) < 1e-12);
    check("normalized_y", std::abs(n.y - 0.8) < 1e-12);
    check("normalized_unit", std::abs(n.norm() - 1.0) < 1e-12);

    // Distance
    check("dist_to",    std::abs(Vec2{0,0}.dist_to({3,4}) - 5.0) < 1e-12);
    check("dist_sq_to", std::abs(Vec2{0,0}.dist_sq_to({3,4}) - 25.0) < 1e-12);

    // is_finite
    check("is_finite_true",  Vec2{1.0, 2.0}.is_finite());
    check("is_finite_nan",  !Vec2{std::numeric_limits<double>::quiet_NaN(), 0}.is_finite());
    check("is_finite_inf",  !Vec2{std::numeric_limits<double>::infinity(), 0}.is_finite());

    // Zero-vector normalize should throw
    bool threw = false;
    try { Vec2{}.normalized(); } catch (const std::domain_error&) { threw = true; }
    check("normalize_zero_throws", threw);

    return ok;
}
