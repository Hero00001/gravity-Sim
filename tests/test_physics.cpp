#include "physics/world.hpp"
#include "physics/constants.hpp"
#include <algorithm>
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

static void test_softening_no_nan_at_contact() {
    World w;
    Body a; a.mass = 1e24; a.position = {0, 0, 0};
    Body b; b.mass = 1e24; b.position = {1e-6, 0, 0};   // essentially touching
    w.spawn(a); w.spawn(b);
    const auto acc = w.computeAccelerations();
    CHECK(std::isfinite(acc[0].x) && std::isfinite(acc[0].y) && std::isfinite(acc[0].z));
    CHECK(std::isfinite(acc[1].x) && std::isfinite(acc[1].y) && std::isfinite(acc[1].z));
    CHECK(glm::length(acc[0]) < 1.0e7);                  // bounded: G*m/eps^2 ≈ 2.7e4
}

static void test_pair_softening_formula() {
    gs::Body a; a.mass = 1e24;                    // rocky radius ≈ 3.7e5 m
    gs::Body b; b.mass = 1e24;
    gs::WorldConfig cfg;
    const double eps = gs::pairSoftening(a, b, cfg);
    CHECK_NEAR(eps, std::max(0.1 * (a.radius() + b.radius()), 5.0e4), 1.0);
    gs::Body tiny; tiny.mass = 1.0; tiny.density = 5515.0;
    gs::Body huge; huge.mass = 1e20; huge.density = 1408.0;
    CHECK_NEAR(gs::pairSoftening(tiny, huge, cfg), 5.0e4, 1.0);   // floor wins
}

static void test_orbit_closes() {
    World w = makeOrbitWorld();
    const glm::dvec3 r0 = w.bodies[1].position - w.bodies[0].position;
    const double len0 = glm::length(r0);
    // period T = 2*pi*sqrt(r^3/(G*M)) ≈ 10035 s (softened) → 1000 steps of 10 s ≈ 1 period
    for (int i = 0; i < 1000; ++i) w.step(10.0);
    const glm::dvec3 r1 = w.bodies[1].position - w.bodies[0].position;
    CHECK_NEAR(glm::length(r1), len0, 0.001 * len0);          // radius within 0.1%
    const double a0 = std::atan2(r0.z, r0.x);
    const double a1 = std::atan2(r1.z, r1.x);
    double sweep = a1 - a0;
    while (sweep > 3.14159265358979) sweep -= 2 * 3.14159265358979;
    while (sweep < -3.14159265358979) sweep += 2 * 3.14159265358979;
    CHECK(std::fabs(sweep) < 0.063);                            // within 1% of full revolution
}

static void test_momentum_conserved_in_flight() {
    World w = makeOrbitWorld();
    auto p = [&] {
        glm::dvec3 t(0.0);
        for (const auto& b : w.bodies) t += b.mass * b.velocity;
        return t;
    };
    const glm::dvec3 before = p();
    double scale = 0.0;
    for (const auto& b : w.bodies) scale += glm::length(b.mass * b.velocity);
    for (int i = 0; i < 500; ++i) w.step(10.0);
    const glm::dvec3 after = p();
    // tolerance vs Σ|m·v| scale (brief defect B3): net momentum ≡ 0, so a value-referenced tol degenerates to 0
    CHECK(glm::length(after - before) <= 1e-12 * scale);
}

static void test_energy_bounded() {
    World w = makeOrbitWorld();
    auto energy = [&] {
        double ke = 0.0, pe = 0.0;
        for (const auto& b : w.bodies) ke += 0.5 * b.mass * glm::dot(b.velocity, b.velocity);
        const glm::dvec3 d = w.bodies[1].position - w.bodies[0].position;
        const double r = glm::length(d);
        const double eps = std::max(0.1 * (w.bodies[0].radius() + w.bodies[1].radius()), 5.0e4);
        pe -= gs::G * w.bodies[0].mass * w.bodies[1].mass / std::sqrt(r * r + eps * eps);
        return ke + pe;
    };
    const double e0 = energy();
    for (int i = 0; i < 1000; ++i) w.step(10.0);
    const double e1 = energy();
    CHECK(std::fabs(e1 - e0) <= 0.01 * std::fabs(e0));
}

int main() {
    test_fps_independence();
    test_step_cap_and_clamp();
    test_paused_bit_identical();
    test_softening_no_nan_at_contact();
    test_pair_softening_formula();
    test_orbit_closes();
    test_momentum_conserved_in_flight();
    test_energy_bounded();
    if (g_failures) { std::printf("%d failure(s)\n", g_failures); return 1; }
    std::printf("all physics tests passed\n");
    return 0;
}
