#include "physics/world.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } \
} while (0)
#define CHECK_NEAR(a, b, tol) do { \
    double _a = (a), _b = (b); \
    if (std::fabs(_a - _b) > (tol)) { \
        std::printf("FAIL %s:%d: %s=%.9g vs %s=%.9g\n", __FILE__, __LINE__, #a, _a, #b, _b); \
        ++g_failures; \
    } \
} while (0)

using gs::Body;
using gs::World;

static std::vector<double> snapshot(const World& w) {
    std::vector<double> s;
    for (const auto& b : w.bodies) {
        s.insert(s.end(), {b.position.x, b.position.y, b.position.z,
                           b.velocity.x, b.velocity.y, b.velocity.z,
                           b.mass, b.density});
    }
    return s;
}

static World makeOrbitWorld() {
    // Two bodies, softened-circular orbit, one period ≈ 10000 s.
    World w;
    const double m1 = 5.9e24, m2 = 1.0e20, r = 1.0e7;
    const double mtot = m1 + m2;
    Body a; a.mass = m1; a.position = {-r * m2 / mtot, 0, 0};
    Body b; b.mass = m2; b.position = { r * m1 / mtot, 0, 0};
    const double eps = std::max(0.1 * (a.radius() + b.radius()), 5.0e4);
    const double vrel = std::sqrt(gs::G * mtot * (r * r) /
                                  std::pow(r * r + eps * eps, 1.5));
    a.velocity = {0, 0, -vrel * m2 / mtot};
    b.velocity = {0, 0,  vrel * m1 / mtot};
    w.spawn(a); w.spawn(b);
    return w;
}

static void test_fps_independence() {
    World a = makeOrbitWorld(), b = makeOrbitWorld();
    int stepsA = 0, stepsB = 0;
    for (int i = 0; i < 60; ++i)  stepsA += a.advance(1.0 / 60.0);
    for (int i = 0; i < 480; ++i) stepsB += b.advance(1.0 / 480.0);
    CHECK(std::abs(stepsA - stepsB) <= 2);
    CHECK(std::abs(stepsA - 480) <= 2);
    CHECK_NEAR(a.simTime, b.simTime, 4 * World::SIM_DT);
}

static void test_step_cap_and_clamp() {
    World w;
    w.spawn(Body{}); w.spawn(Body{});
    int n = w.advance(10.0);           // huge frame: clamped to 0.25 s → capped at 8 steps
    CHECK(n == 8);
    World w2;
    w2.spawn(Body{}); w2.spawn(Body{});
    CHECK(w2.advance(5.0) == 8);
}

static void test_paused_bit_identical() {
    World w = makeOrbitWorld();
    w.paused = true;
    const auto before = snapshot(w);
    int total = 0;
    for (int i = 0; i < 100; ++i) total += w.advance(1.0 / 60.0);
    CHECK(total == 0);
    const auto after = snapshot(w);
    CHECK(before.size() == after.size());
    CHECK(before == after);
    CHECK(w.simTime == 0.0);
}

int main() {
    test_fps_independence();
    test_step_cap_and_clamp();
    test_paused_bit_identical();
    if (g_failures) { std::printf("%d failure(s)\n", g_failures); return 1; }
    std::printf("all physics tests passed\n");
    return 0;
}
