#include "physics/barnes_hut.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <array>
#include <memory>
#include <algorithm>

namespace gs {
namespace {

constexpr int kMaxDepth = 24;
constexpr double kClusterHalf = 1e-3;   // below this half-size, stop subdividing

struct Node {
    glm::dvec3 center{0.0, 0.0, 0.0};
    double half = 0.0;
    double mass = 0.0;               // total mass in this node
    glm::dvec3 com{0.0, 0.0, 0.0};   // mass-weighted position (divide by mass for COM)
    double radiusMax = 0.0;          // largest physical radius inside (for softening)
    int bodyIndex = -1;              // >=0 only for a single-body leaf (index into pos[])
    bool leaf = false;
    std::array<std::unique_ptr<Node>, 8> children;
};

int octantOf(const glm::dvec3& d) {
    return (d.x > 0.0 ? 1 : 0) | (d.y > 0.0 ? 2 : 0) | (d.z > 0.0 ? 4 : 0);
}

std::unique_ptr<Node> build(const std::vector<glm::dvec3>& pos,
                            const std::vector<double>& mass,
                            const std::vector<double>& radius,
                            const std::vector<int>& idx,
                            const glm::dvec3& center, double half, int depth) {
    auto node = std::make_unique<Node>();
    node->center = center;
    node->half = half;
    if (idx.empty()) return node;
    if (idx.size() == 1) {
        const int i = idx[0];
        node->leaf = true;
        node->bodyIndex = i;
        node->mass = mass[i];
        node->com = pos[i] * mass[i];
        node->radiusMax = radius[i];
        return node;
    }
    double m = 0.0, rmax = 0.0;
    glm::dvec3 c(0.0);
    for (int i : idx) { m += mass[i]; c += pos[i] * mass[i]; rmax = std::max(rmax, radius[i]); }
    node->mass = m;
    node->com = c;
    node->radiusMax = rmax;
    if (depth >= kMaxDepth || half <= kClusterHalf) {
        // Too deep / too small: treat the cluster as a single pseudo-mass.
        node->leaf = true;
        node->bodyIndex = -1;
        return node;
    }
    std::array<std::vector<int>, 8> buckets;
    for (int i : idx) {
        const glm::dvec3 d = pos[i] - center;
        buckets[octantOf(d)].push_back(i);
    }
    for (int o = 0; o < 8; ++o) {
        if (buckets[o].empty()) continue;
        const double q = half / 2.0;
        const glm::dvec3 off((o & 1) ? q : -q, (o & 2) ? q : -q, (o & 4) ? q : -q);
        node->children[o] = build(pos, mass, radius, buckets[o], center + off, q, depth + 1);
    }
    return node;
}

// Softening for a monopole interaction: a single-body leaf uses the exact per-pair
// value (identical to the direct solver); an aggregated node uses its largest member
// radius, which is conservative (never softer than the true pair).
double nodeSoftening(const Node* node, double selfRadius, double frac, double floorM) {
    const double otherR = node->radiusMax;
    return std::max(frac * (selfRadius + otherR), floorM);
}

glm::dvec3 accel(const Node* node, const glm::dvec3& pos, int self,
                 const std::vector<double>& radius, double selfRadius,
                 double theta, double frac, double floorM) {
    if (!node || node->mass == 0.0) return glm::dvec3(0.0);
    if (node->leaf) {
        if (node->bodyIndex == self) return glm::dvec3(0.0);   // exclude self
        const glm::dvec3 d = node->com / node->mass - pos;
        const double r2 = glm::dot(d, d);
        const double eps = nodeSoftening(node, selfRadius, frac, floorM);
        const double u2 = r2 + eps * eps;
        const double denom = u2 * std::sqrt(u2);
        return (denom > 0.0) ? G * node->mass * d / denom : glm::dvec3(0.0);
    }
    const glm::dvec3 com = node->com / node->mass;   // mass genuinely sits at the COM
    // Opening criterion (classic Barnes-Hut): approximate this node as a single mass at its
    // COM when its size s is small relative to its distance r from the body: s/r < theta.
    // Crucially, r is the distance to the node's CENTRE OF MASS (not its geometric centre):
    // a node whose COM is offset toward the body must be opened more aggressively, otherwise
    // the monopole approximation inherits the offset and the error blows past theta^2. Using
    // the geometric centre here had been the source of a ~1.2% floor that the theta=0.5
    // solver could not get under 1%.
    const glm::dvec3 dC = com - pos;                  // distance to the centre of mass
    const double rC = std::sqrt(glm::dot(dC, dC));
    const double s = 2.0 * node->half;          // node side length
    if (rC > 0.0 && s / rC < theta) {
        const double r2 = glm::dot(dC, dC);
        const double eps = nodeSoftening(node, selfRadius, frac, floorM);
        const double u2 = r2 + eps * eps;
        const double denom = u2 * std::sqrt(u2);
        return (denom > 0.0) ? G * node->mass * dC / denom : glm::dvec3(0.0);
    }
    glm::dvec3 a(0.0);
    for (int o = 0; o < 8; ++o)
        if (node->children[o])
            a += accel(node->children[o].get(), pos, self, radius, selfRadius,
                       theta, frac, floorM);
    return a;
}

} // namespace

std::vector<glm::dvec3> directAccelerations(const std::vector<Body>& bodies,
                                            double softeningFrac, double softeningFloor) {
    const std::size_t n = bodies.size();
    std::vector<glm::dvec3> acc(n, glm::dvec3(0.0));
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].ghost) continue;
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j || bodies[j].ghost) continue;
            const glm::dvec3 d = bodies[j].position - bodies[i].position;
            const double r2 = glm::dot(d, d);
            const double eps = pairSoftening(bodies[i], bodies[j], softeningFrac, softeningFloor);
            const double u2 = r2 + eps * eps;
            const double denom = u2 * std::sqrt(u2);
            if (denom > 0.0) acc[i] += G * bodies[j].mass * d / denom;
        }
    }
    return acc;
}

std::vector<glm::dvec3> barnesHutAccelerations(const std::vector<Body>& bodies,
                                               double theta,
                                               double softeningFrac, double softeningFloor) {
    const std::size_t n = bodies.size();
    std::vector<glm::dvec3> acc(n, glm::dvec3(0.0));
    std::vector<glm::dvec3> pos;
    std::vector<double> mass;
    std::vector<double> radius;
    std::vector<int> toBody;          // pos-array index -> original body index
    std::vector<int> idx;            // pos-array indices [0, m)
    pos.reserve(n); mass.reserve(n); radius.reserve(n); toBody.reserve(n); idx.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].ghost) continue;
        pos.push_back(bodies[i].position);
        mass.push_back(bodies[i].mass);
        radius.push_back(bodies[i].radius());
        toBody.push_back(static_cast<int>(i));
        idx.push_back(static_cast<int>(pos.size()) - 1);
    }
    if (idx.empty()) return acc;

    glm::dvec3 mn = pos[0], mx = pos[0];
    for (const auto& p : pos) { mn = glm::min(mn, p); mx = glm::max(mx, p); }
    const glm::dvec3 center = (mn + mx) * 0.5;
    double half = 0.0;
    for (const auto& p : pos) {
        const double dx = std::fabs(p.x - center.x);
        const double dy = std::fabs(p.y - center.y);
        const double dz = std::fabs(p.z - center.z);
        half = std::max(half, std::max(dx, std::max(dy, dz)));
    }
    half = std::max(half * 1.0000001, 1.0);

    auto root = build(pos, mass, radius, idx, center, half, 0);
    for (std::size_t k = 0; k < pos.size(); ++k) {
        const int self = idx[k];                 // equals k
        acc[toBody[k]] = accel(root.get(), pos[k], self, radius, radius[k],
                               theta, softeningFrac, softeningFloor);
    }
    return acc;
}

} // namespace gs
