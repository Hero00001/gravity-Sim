#pragma once
#include "render/camera.hpp"
#include "render/grid.hpp"
#include "physics/world.hpp"
#include <vector>
#include <cstdint>
#include <unordered_map>

namespace gs::render {

class Renderer {
public:
    bool init();
    void shutdown();
    // Full draw path — viewport follows fbw/fbh every frame (resize-safe), then the
    // starfield, grid, trails, glow halos and bodies are drawn from the (const) world
    // state. `selectedId` gets a bright ring; `hoverId` (when different) gets a dim one.
    void draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
              const GridConfig& grid, std::uint64_t selectedId = 0,
              std::uint64_t hoverId = 0);

private:
    struct GpuBody { unsigned vao = 0, vbo = 0; double radius = -1.0; int count = 0; };
    unsigned progBody_ = 0, progTrail_ = 0, progStar_ = 0, progGlow_ = 0;
    unsigned gridVao_ = 0, gridVbo_ = 0;
    unsigned trailVao_ = 0, trailVbo_ = 0;
    unsigned starVao_ = 0, starVbo_ = 0;
    unsigned glowVao_ = 0, glowVbo_ = 0;
    int starCount_ = 0;
    std::unordered_map<std::uint64_t, GpuBody> gpu_;
    std::vector<float> scratch_;
    std::vector<float> ring_;              // reused selection/hover ring vertices

    // The grid template only changes when the scene's grid config changes, so it is
    // rebuilt on demand instead of every frame (spec §4.9: build once per scene config).
    GridConfig gridCfg_;
    std::vector<float> gridBase_;
    bool gridBaseValid_ = false;

    // Uniform locations cached once at init() (spec #10) instead of re-querying per frame.
    struct BodyUniforms { int model = -1, view = -1, proj = -1, objectColor = -1, isGrid = -1, glow = -1; };
    struct TrailUniforms { int view = -1, proj = -1, tintColor = -1; };
    struct StarUniforms { int view = -1, proj = -1, camPos = -1, radius = -1; };
    struct GlowUniforms { int view = -1, proj = -1, center = -1, right = -1, up = -1,
                          scale = -1, color = -1, strength = -1; };
    BodyUniforms uBody_;
    TrailUniforms uTrail_;
    StarUniforms uStar_;
    GlowUniforms uGlow_;

    void syncBodies(const gs::World& world);
    void drawStarfield(const Camera& cam, float aspect);
    void drawBodies(const gs::World& world, const Camera& cam, float aspect);
    void drawGlow(const gs::World& world, const Camera& cam, float aspect);
    void drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam, float aspect);
    void drawTrails(const gs::World& world, const Camera& cam, float aspect);
    void drawRing(const gs::World& world, const Camera& cam, float aspect, std::uint64_t id,
                  const glm::vec4& color, float ringScale);
};

} // namespace gs::render
