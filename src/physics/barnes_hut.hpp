#pragma once
#include "physics/body.hpp"
#include <vector>

namespace gs {

// O(n^2) reference acceleration with a single global softening `eps` (metres).
// Used as the cross-test reference for the Barnes-Hut approximation and as the
// below-64-body path inside World::step.
std::vector<glm::dvec3> directAccelerations(const std::vector<Body>& bodies, double eps);

// Barnes-Hut tree-code acceleration. `theta` is the opening angle (spec: 0.5),
// `eps` the softening length in metres. Ghost bodies are excluded as sources and
// receive zero acceleration.
std::vector<glm::dvec3> barnesHutAccelerations(const std::vector<Body>& bodies,
                                               double theta, double eps);

} // namespace gs
