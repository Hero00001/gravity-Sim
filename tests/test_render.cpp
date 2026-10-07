#include "render/mesh.hpp"
#include "render/grid.hpp"
#include "render/camera.hpp"
#include "physics/body.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>

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

// --- mesh (bug 11): exact bounds, no extra ring past the south pole ---
static void test_sphere_mesh_bounds() {
    const int stacks = 16, sectors = 24;
    const double r = 3.5;
    const auto v = gs::render::sphereVertices(r, stacks, sectors);

    CHECK(v.size() == std::size_t(stacks) * sectors * 6 * 3);   // exact count, no extra ring
    for (float x : v) CHECK(x >= -float(r) - 1e-5f && x <= float(r) + 1e-5f);

    float minY = 1e30f, maxY = -1e30f;
    for (std::size_t i = 1; i < v.size(); i += 3) {
        minY = std::min(minY, v[i]);
        maxY = std::max(maxY, v[i]);
    }
    CHECK(minY <= -float(r) + 1e-4f && minY >= -float(r) - 1e-4f);
    CHECK(maxY <= float(r) + 1e-4f && maxY >= float(r) - 1e-4f);
}

// --- grid (bug 6): immutable base, pure displacement, no rest-level drift ---
static void test_grid_base_stable_and_pure() {
    gs::render::GridConfig cfg;                       // size 20000, div 25, y=-600
    const auto base = gs::render::buildGridBase(cfg);
    CHECK(!base.empty());
    CHECK(base.size() % 6 == 0);                      // line segments: pairs of xyz
    for (std::size_t i = 0; i < base.size(); i += 3)
        CHECK_NEAR(base[i + 1], float(cfg.planeYFactor * cfg.sizeUnits), 1e-3f);
    for (int i = 0; i < 100; ++i) {
        const auto again = gs::render::buildGridBase(cfg);
        CHECK(again == base);                          // deterministic, never mutated
    }
}

static void test_grid_bodyless_displacement_identity() {
    gs::render::GridConfig cfg;
    const auto base = gs::render::buildGridBase(cfg);
    std::vector<gs::Body> none;
    for (int i = 0; i < 100; ++i) {
        const auto out = gs::render::displaceGrid(base, cfg, none);
        CHECK(out == base);                            // bug 6 regression: rest level never drifts
    }
}

static void test_grid_flat_mode_identity_and_bend_dips_down() {
    gs::render::GridConfig cfg;
    cfg.mode = gs::render::GridMode::Flat;
    const auto base = gs::render::buildGridBase(cfg);
    gs::Body heavy; heavy.mass = 1.989e30; heavy.density = 1408.0;
    heavy.position = {0, 0, 0};
    CHECK(gs::render::displaceGrid(base, cfg, {heavy}) == base);

    cfg.mode = gs::render::GridMode::Bend;
    const auto bent = gs::render::displaceGrid(base, cfg, {heavy});
    CHECK(bent.size() == base.size());
    double maxDip = 0.0;
    for (std::size_t i = 0; i < bent.size(); i += 3) {
        const double dy = double(base[i + 1]) - double(bent[i + 1]);
        CHECK(dy >= -1e-6);                            // never displaced upward
        maxDip = std::max(maxDip, dy);
    }
    CHECK(maxDip > 0.0);
    CHECK(maxDip <= 0.25 * cfg.sizeUnits + 1e-6);      // clamp respected
}

static void test_camera_pick_math() {
    gs::render::Camera cam;
    cam.pos = glm::vec3(0.0f, 0.0f, 5000.0f);
    cam.yaw = -90.0f; cam.pitch = 0.0f;                 // looks down -Z toward the origin
    const float aspect = 1.0f;
    const glm::vec3 target(0.0f, 0.0f, 0.0f);
    const glm::vec3 ndc = cam.projectToNDC(target, aspect);
    CHECK(std::fabs(ndc.x) < 1e-3f);
    CHECK(std::fabs(ndc.y) < 1e-3f);
    CHECK(ndc.z > -1.0f && ndc.z < 1.0f);
    const glm::vec3 dir = cam.unprojectDir(ndc.x, ndc.y, aspect);
    const glm::vec3 hit = gs::render::Camera::rayPlaneIntersect(cam.pos, dir, target, cam.front());
    CHECK(std::fabs(hit.x - target.x) < 1e-2f);
    CHECK(std::fabs(hit.y - target.y) < 1e-2f);
    CHECK(std::fabs(hit.z - target.z) < 1e-2f);
}

int main() {
    test_camera_pick_math();
    test_sphere_mesh_bounds();
    test_grid_base_stable_and_pure();
    test_grid_bodyless_displacement_identity();
    test_grid_flat_mode_identity_and_bend_dips_down();
    if (g_failures) { std::printf("%d failure(s)\n", g_failures); return 1; }
    std::printf("all render tests passed\n");
    return 0;
}
