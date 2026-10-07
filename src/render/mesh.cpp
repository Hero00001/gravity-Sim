#include "render/mesh.hpp"
#include <cmath>

namespace gs::render {
namespace {
void sph(double r, double theta, double phi, float out[3]) {
    out[0] = float(r * std::sin(theta) * std::cos(phi));
    out[1] = float(r * std::cos(theta));
    out[2] = float(r * std::sin(theta) * std::sin(phi));
}
}

std::vector<float> sphereVertices(double radius, int stacks, int sectors) {
    std::vector<float> v;
    v.reserve(std::size_t(stacks) * sectors * 18);
    const double PI = 3.14159265358979323846;
    for (int i = 0; i < stacks; ++i) {
        const double t1 = double(i) / stacks * PI;
        const double t2 = double(i + 1) / stacks * PI;
        for (int j = 0; j < sectors; ++j) {
            const double p1 = double(j) / sectors * 2.0 * PI;
            const double p2 = double(j + 1) / sectors * 2.0 * PI;
            float a[3], b[3], c[3], d[3];
            sph(radius, t1, p1, a); sph(radius, t1, p2, b);
            sph(radius, t2, p1, c); sph(radius, t2, p2, d);
            for (float* p : {a, b, c}) v.insert(v.end(), p, p + 3);
            for (float* p : {b, d, c}) v.insert(v.end(), p, p + 3);
        }
    }
    return v;
}
} // namespace gs::render
