#include "render/grid.hpp"
#include "physics/body.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <algorithm>

namespace gs::render {

std::vector<float> buildGridBase(const GridConfig& cfg) {
    std::vector<float> v;
    if (cfg.mode == GridMode::Off) return v;
    const float step = float(cfg.sizeUnits / cfg.divisions);
    const float half = float(cfg.sizeUnits / 2.0);
    const float y = float(cfg.planeYFactor * cfg.sizeUnits);
    for (int zs = 0; zs <= cfg.divisions; ++zs) {           // lines running along X
        const float z = -half + zs * step;
        for (int xs = 0; xs < cfg.divisions; ++xs) {
            const float x0 = -half + xs * step;
            v.insert(v.end(), {x0, y, z, x0 + step, y, z});
        }
    }
    for (int xs = 0; xs <= cfg.divisions; ++xs) {           // lines running along Z
        const float x = -half + xs * step;
        for (int zs = 0; zs < cfg.divisions; ++zs) {
            const float z0 = -half + zs * step;
            v.insert(v.end(), {x, y, z0, x, y, z0 + step});
        }
    }
    return v;
}

std::vector<float> displaceGrid(const std::vector<float>& base, const GridConfig& cfg,
                                const std::vector<gs::Body>& bodies) {
    if (cfg.mode != GridMode::Bend || bodies.empty()) return base;   // Flat/Off → identity
    std::vector<float> out = base;                                    // never mutate the base
    const double maxDipUnits = 0.25 * cfg.sizeUnits;                 // clamp (spec §4.9)
    for (std::size_t i = 0; i + 2 < out.size(); i += 3) {
        const double x = out[i], y = out[i + 1], z = out[i + 2];
        const glm::dvec3 vm(x, y, z);
        const glm::dvec3 vmM = vm * UNIT;                            // world-units → meters
        double dipM = 0.0;
        for (const auto& b : bodies) {
            if (b.ghost) continue;
            const double d = glm::length(b.position - vmM);
            const double rs = 2.0 * gs::G * b.mass / (gs::C * gs::C); // Schwarzschild radius (m)
            if (d > rs) dipM += 2.0 * std::sqrt(rs * (d - rs));
        }
        const double dipUnits = std::min(dipM / UNIT, maxDipUnits);
        out[i + 1] = float(double(base[i + 1]) - dipUnits);
    }
    return out;
}

} // namespace gs::render
