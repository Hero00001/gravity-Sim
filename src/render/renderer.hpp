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
    // Full draw path — viewport follows fbw/fbh every frame (resize-safe), then grid,
    // trails, and bodies are drawn from the (const) world state.
    void draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
              const GridConfig& grid, std::uint64_t selectedId = 0);

private:
    struct GpuBody { unsigned vao = 0, vbo = 0; double radius = -1.0; int count = 0; };
    unsigned progBody_ = 0, progTrail_ = 0;
    unsigned gridVao_ = 0, gridVbo_ = 0;
    unsigned trailVao_ = 0, trailVbo_ = 0;
    std::unordered_map<std::uint64_t, GpuBody> gpu_;
    std::vector<float> scratch_;

    // Uniform locations cached once at init() (spec #10) instead of re-querying per frame.
    struct BodyUniforms { int model = -1, view = -1, proj = -1, objectColor = -1, isGrid = -1, glow = -1; };
    struct TrailUniforms { int view = -1, proj = -1, tintColor = -1; };
    BodyUniforms uBody_;
    TrailUniforms uTrail_;

    void syncBodies(const gs::World& world);
    void drawBodies(const gs::World& world, const Camera& cam, float aspect);
    void drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam, float aspect);
    void drawTrails(const gs::World& world, const Camera& cam, float aspect);
    void drawSelection(const gs::World& world, const Camera& cam, float aspect, std::uint64_t id);
};

} // namespace gs::render
