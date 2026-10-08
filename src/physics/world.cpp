#include "physics/world.hpp"
#include "physics/constants.hpp"
#include "physics/barnes_hut.hpp"
#include <algorithm>
#include <cmath>

namespace gs {

constexpr std::size_t kBarnesHutThreshold = 64;   // spec §4.7: direct below/at 64, BH above

double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg) {
    return pairSoftening(a, b, cfg.softeningFrac, cfg.softeningFloor);
}

std::uint64_t World::spawn(const Body& b) {
    Body copy = b;
    copy.id = nextId++;
    bodies.push_back(copy);
    bodies.back().trail.setCap(config.trailCap);
    return copy.id;
}

bool World::removeById(std::uint64_t id) {
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
        if (it->id == id) { bodies.erase(it); return true; }
    }
    return false;
}

void World::reset() {
    bodies.clear();
    nextId = 1;
    simTime = 0.0;
    accum_ = 0.0;
    trailAccum_ = 0.0;
    paused = false;
}

std::vector<glm::dvec3> World::computeAccelerations() const {
    const std::size_t n = bodies.size();
    if (n < 2) return std::vector<glm::dvec3>(n, glm::dvec3(0.0));

    // One canonical pair-softening (spec §4.4) for both paths, so the forces do not
    // change when the solver swaps at the 64-body threshold. Barnes-Hut above it,
    // exact O(n²) at or below.
    if (n > kBarnesHutThreshold)
        return barnesHutAccelerations(bodies, 0.5, config.softeningFrac, config.softeningFloor);
    return directAccelerations(bodies, config.softeningFrac, config.softeningFloor);
}

void World::step(double dt) {
    const std::size_t n = bodies.size();
    if (n < 2) { simTime += dt; return; }

    auto acc = computeAccelerations();
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost && !bodies[i].grabbed) bodies[i].velocity += 0.5 * dt * acc[i];
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost && !bodies[i].grabbed) bodies[i].position += dt * bodies[i].velocity;

    acc = computeAccelerations();
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost && !bodies[i].grabbed) bodies[i].velocity += 0.5 * dt * acc[i];

    mergeOverlaps();
    simTime += dt;
}

int World::advance(double frameDeltaSeconds) {
    const double rawFd = frameDeltaSeconds;
    double fd = frameDeltaSeconds;
    if (fd > config.maxFrameDelta) fd = config.maxFrameDelta;
    if (fd < 0.0) fd = 0.0;

    if (paused) return 0;                    // no stepping, merging, or trail sampling

    // Advance exactly `fd * timeScale` sim-seconds, never more and never less, at any
    // frame rate and any time scale:
    //   * normally every sub-step is the preferred fixed size dtBase = SIM_DT * timeScale,
    //     which keeps ts = 1 bit-identical to a pure fixed-step integrator;
    //   * if the frame's debt exceeds what the step budget can pay at dtBase (a slow
    //     frame, or a big time scale), the sub-step size grows so the debt is still paid
    //     in full — the sim keeps the requested speed instead of silently running slow.
    // The old code always stepped a fixed SIM_DT and then *discarded* the backlog, so at
    // timeScale 1e4..1e6 the 8-step cap was hit every frame and the presets looked frozen.
    const double dtBase = SIM_DT * timeScale;
    const double owed = accum_ + fd * timeScale;

    // +1e-9 absorbs float error so an exact whole number of sub-steps (e.g. 8.0 at 60 fps)
    // is never truncated to 7 and leaked into the next frame.
    int steps = (dtBase > 0.0) ? int(owed / dtBase + 1e-9) : 0;   // whole sub-steps we can afford
    if (steps > config.maxStepsPerFrame) steps = config.maxStepsPerFrame;

    int n = 0;
    double dt = dtBase;
    if (steps > 0) {
        // Pay the whole debt with a larger sub-step only when the step budget binds.
        if (steps == config.maxStepsPerFrame && owed > steps * dtBase) dt = owed / double(steps);
        for (int i = 0; i < steps; ++i) step(dt);
        n = steps;
    }
    accum_ = owed - double(n) * dt;          // residue (< dtBase) carries into the next frame
    if (accum_ < 0.0) accum_ = 0.0;          // (capped case pays the debt exactly)

    if (n > 0) {
        trailAccum_ += rawFd;   // raw wall delta (pre-clamp) — wall-clock based, timeScale-independent
        if (trailAccum_ >= config.trailSampleDt) {
            for (auto& b : bodies)
                if (!b.ghost && !b.grabbed) b.trail.push(b.position);
            trailAccum_ = 0.0;
        }
    }
    return n;
}

void World::stepOnce() {
    step(SIM_DT);
    for (auto& b : bodies)
        if (!b.ghost && !b.grabbed) b.trail.push(b.position);
}

void World::setTrailCap(std::size_t cap) {
    config.trailCap = cap;
    for (auto& b : bodies) b.trail.setCap(cap);
}

void World::mergeOverlaps() {
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t i = 0; i < bodies.size() && !changed; ++i) {
            if (bodies[i].ghost || bodies[i].grabbed) continue;
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                if (bodies[j].ghost || bodies[j].grabbed) continue;
                Body& A = bodies[i];
                Body& B = bodies[j];
                if (glm::length(B.position - A.position) >= A.radius() + B.radius())
                    continue;
                const bool aHeavier = A.mass >= B.mass;
                Body& heavy = aHeavier ? A : B;
                Body& light = aHeavier ? B : A;
                const double m1 = heavy.mass, m2 = light.mass, m = m1 + m2;
                heavy.velocity = (m1 * heavy.velocity + m2 * light.velocity) / m;
                heavy.position = (m1 * heavy.position + m2 * light.position) / m;
                heavy.density  = (m1 * heavy.density  + m2 * light.density)  / m;
                heavy.mass = m;
                const std::size_t eraseIdx = aHeavier ? j : i;
                bodies.erase(bodies.begin() + std::ptrdiff_t(eraseIdx));
                changed = true;
                break;
            }
        }
    }
}

} // namespace gs
