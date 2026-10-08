#include "render/grid.hpp"
#include "physics/body.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <algorithm>

namespace gs::render {

namespace {
// Visual "spacetime well" shape. The true potential depth is ∝ rs/d (rs = 2GM/c²), which
// at astronomical scales is a needle: for the Sun at Earth's orbit it is ~1e-8 of the
// grid size, so the sheet renders dead flat (and the spec's √(rs·d) embedding actually
// grows with distance, i.e. dips *away* from the mass). We therefore keep the physical
// ingredients — every body's well depth is proportional to its Schwarzschild radius, so
// mass ordering is preserved — but give the well a scene-relative width so it is visible.
inline double schwarzschild(double massKg) {
    return 2.0 * gs::G * massKg / (gs::C * gs::C);
}
} // namespace

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

    // Well geometry, derived from the grid size so every scene gets a visible sheet.
    const double wellDepthUnits = 0.20 * cfg.sizeUnits;              // deepest well (units)
    const double wellWidthM     = 0.30 * cfg.sizeUnits * UNIT;       // half-width (metres)
    const double maxDipUnits    = 0.25 * cfg.sizeUnits;              // total clamp (spec §4.9)
    const double width2         = wellWidthM * wellWidthM;

    // Heaviest body = reference depth; every well is scaled by its own rs relative to it.
    double rsMax = 0.0;
    for (const auto& b : bodies) {
        if (b.ghost) continue;
        rsMax = std::max(rsMax, schwarzschild(b.mass));
    }
    if (rsMax <= 0.0) return base;

    std::vector<float> out = base;                                   // never mutate the base
    for (std::size_t i = 0; i + 2 < out.size(); i += 3) {
        const double x = out[i], y = out[i + 1], z = out[i + 2];
        const glm::dvec3 vmM = glm::dvec3(x, y, z) * UNIT;           // world-units → metres
        double dipUnits = 0.0;
        for (const auto& b : bodies) {
            if (b.ghost) continue;
            const double rel = schwarzschild(b.mass) / rsMax;        // in (0, 1]
            const double d2 = glm::dot(b.position - vmM, b.position - vmM);
            dipUnits += wellDepthUnits * rel / (1.0 + d2 / width2);  // smooth potential well
        }
        out[i + 1] = float(double(base[i + 1]) - std::min(dipUnits, maxDipUnits));
    }
    return out;
}

} // namespace gs::render
