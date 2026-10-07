#pragma once
#include "physics/body.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>

namespace gs {

struct WorldConfig {
    double softeningFrac  = 0.1;
    double softeningFloor = 5.0e4;   // m
    double trailSampleDt  = 0.033;   // wall-seconds between trail samples
    std::size_t trailCap  = 512;     // ring-buffer cap (samples)
    int    maxStepsPerFrame = 8;
    double maxFrameDelta   = 0.25;   // s
};

double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg);

class World {
public:
    static constexpr double SIM_DT = 1.0 / 480.0;

    std::vector<Body> bodies;
    WorldConfig config;
    bool   paused   = false;
    double timeScale = 1.0;
    double simTime  = 0.0;
    std::uint64_t nextId = 1;

    // Visual floor for body radii (metres). Real bodies are often far too small to
    // see at a scene's framed scale (e.g. Earth is 0.64 units across a 450,000-unit
    // solar system), so every body is drawn at least this big. It is raised per scene
    // by applySceneToWorld() to a fraction of the scene radius. Purely visual — physics
    // always uses Body::radius().
    double minVisualRadiusMeters = 3.0e6;   // 0.3 world-units (spec MIN_VISUAL)
    double displayRadius(const Body& b) const {
        return std::max(b.radius(), minVisualRadiusMeters);
    }

    std::uint64_t spawn(const Body& b);
    bool removeById(std::uint64_t id);
    void reset();                                 // clear bodies + reset sim clock/accumulators
    void step(double dt);                       // one leapfrog KDK step
    std::vector<glm::dvec3> computeAccelerations() const;
    int advance(double frameDeltaSeconds);      // timestep + trail sampling; 0 when paused
    void stepOnce();                            // one step regardless of pause (single-step key)
    void setTrailCap(std::size_t cap);          // applies to all bodies + future spawns

private:
    double accum_ = 0.0;
    double trailAccum_ = 0.0;
    void mergeOverlaps();
};

} // namespace gs
