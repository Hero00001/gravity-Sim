#pragma once
#include <glm/glm.hpp>
#include <deque>
#include <cstdint>
#include <cmath>

namespace gs {

struct TrailRing {
    std::deque<glm::dvec3> pts;
    std::size_t cap = 512;
    void push(const glm::dvec3& p) {
        pts.push_back(p);
        while (pts.size() > cap) pts.pop_front();
    }
    void setCap(std::size_t c) {
        cap = c;
        while (pts.size() > cap) pts.pop_front();
    }
    void clear() { pts.clear(); }
};

struct Body {
    glm::dvec3 position{0.0};   // m
    glm::dvec3 velocity{0.0};   // m/s
    double mass   = 0.0;        // kg
    double density = 3344.0;    // kg/m^3
    glm::vec4 color{1.0f, 0.0f, 0.0f, 1.0f};
    std::uint64_t id = 0;
    bool ghost = false;         // true while being placed: no forces, no merges, no trail
    bool glow  = false;         // rendered as blown-out point source
    TrailRing trail;

    double radius() const {
        return std::cbrt(3.0 * mass / (4.0 * 3.14159265358979323846 * density));
    }
};

} // namespace gs
