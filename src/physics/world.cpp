#include "physics/world.hpp"
#include "physics/constants.hpp"
#include "physics/barnes_hut.hpp"
#include <algorithm>
#include <cmath>

namespace gs {

constexpr std::size_t kBarnesHutThreshold = 64;   // spec §4.7: direct below/at 64, BH above

double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg) {
    return std::max(cfg.softeningFrac * (a.radius() + b.radius()), cfg.softeningFloor);
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

    // Barnes-Hut above the threshold (spec §4.7, θ = 0.5). Below it, exact O(n²).
    if (n > kBarnesHutThreshold)
        return barnesHutAccelerations(bodies, 0.5, config.softeningFloor);

    std::vector<glm::dvec3> acc(n, glm::dvec3(0.0));
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].ghost) continue;                       // grabbed bodies still attract
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j || bodies[j].ghost) continue;
            const glm::dvec3 d = bodies[j].position - bodies[i].position;
            const double r2 = glm::dot(d, d);
            const double eps = pairSoftening(bodies[i], bodies[j], config);
            const double u2 = r2 + eps * eps;
            const double denom = u2 * std::sqrt(u2);          // (r^2+eps^2)^1.5
            if (denom > 0.0) acc[i] += G * bodies[j].mass * d / denom;
        }
    }
    return acc;
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

    const double ts = paused ? 0.0 : timeScale;
    accum_ += fd * ts;

    int n = 0;
    while (accum_ >= SIM_DT && n < config.maxStepsPerFrame) {
        step(SIM_DT);
        accum_ -= SIM_DT;
        ++n;
    }
    if (n == config.maxStepsPerFrame && accum_ >= SIM_DT) accum_ = 0.0;  // discard backlog

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
