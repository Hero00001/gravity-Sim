#pragma once
#include "physics/body.hpp"
#include <vector>
#include <cstdint>
#include <cmath>
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
    //
    // Below the floor a compressive map is applied instead of a hard clamp, so bodies
    // stay visible AND a heavier body still reads as bigger:
    //     display = floor · (1 + span · (r/floor)^pow)      with 0 < r < floor
    // i.e. display ∈ [floor, (1+span)·floor], strictly increasing in the physical
    // radius. A hard clamp would have drawn the Sun and Mercury the same size.
    double minVisualRadiusMeters = 3.0e6;   // 0.3 world-units (spec MIN_VISUAL)
    static constexpr double kVisualSpan = 3.0;   // max extra size above the floor
    static constexpr double kVisualPow  = 0.25;  // compression strength
    double displayRadius(const Body& b) const {
        const double r = b.radius();
        if (r >= minVisualRadiusMeters) return r;
        const double x = std::max(r, 0.0) / minVisualRadiusMeters;   // in [0, 1)
        return minVisualRadiusMeters * (1.0 + kVisualSpan * std::pow(x, kVisualPow));
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
