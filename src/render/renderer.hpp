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
    // Full draw path — implemented in Task 10/12; for Task 1 it draws grid + bodies.
    void draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
              const GridConfig& grid);
    // Simple path used by the Task-1 scaffold.
    void drawOne(const std::vector<float>& verts, const glm::vec3& posUnits,
                 const glm::vec4& color);

private:
    struct GpuBody { unsigned vao = 0, vbo = 0; double radius = -1.0; int count = 0; };
    unsigned progBody_ = 0, progTrail_ = 0;
    unsigned gridVao_ = 0, gridVbo_ = 0;
    unsigned trailVao_ = 0, trailVbo_ = 0;
    std::unordered_map<std::uint64_t, GpuBody> gpu_;
    std::vector<float> scratch_;

    void syncBodies(const gs::World& world);
    void drawBodies(const gs::World& world, const Camera& cam, float aspect);
    void drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam, float aspect);
    void drawTrails(const gs::World& world, const Camera& cam, float aspect);
};

} // namespace gs::render
