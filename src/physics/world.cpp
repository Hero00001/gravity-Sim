#include "physics/world.hpp"
#include "physics/constants.hpp"
#include <algorithm>
#include <cmath>

namespace gs {

double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg) {
    return std::max(cfg.softeningFrac * (a.radius() + b.radius()), cfg.softeningFloor);
}

std::uint64_t World::spawn(const Body& b) {
    Body copy = b;
    copy.id = nextId++;
    bodies.push_back(copy);
    return copy.id;
}

bool World::removeById(std::uint64_t id) {
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
        if (it->id == id) { bodies.erase(it); return true; }
    }
    return false;
}

std::vector<glm::dvec3> World::computeAccelerations() const {
    const std::size_t n = bodies.size();
    std::vector<glm::dvec3> acc(n, glm::dvec3(0.0));
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].ghost) continue;
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
        if (!bodies[i].ghost) bodies[i].velocity += 0.5 * dt * acc[i];
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost) bodies[i].position += dt * bodies[i].velocity;

    acc = computeAccelerations();
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost) bodies[i].velocity += 0.5 * dt * acc[i];

    simTime += dt;
}

int World::advance(double frameDeltaSeconds) {
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
    return n;
}

void World::stepOnce() {
    step(SIM_DT);
}

} // namespace gs
