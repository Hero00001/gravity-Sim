#pragma once
#include "physics/body.hpp"
#include <vector>

namespace gs {

// O(n^2) reference acceleration. Softening is the canonical per-pair Plummer form
// pairSoftening(a, b, frac, floor) — identical to the Barnes-Hut path, so switching
// solvers at the 64-body threshold never changes the forces.
std::vector<glm::dvec3> directAccelerations(const std::vector<Body>& bodies,
                                            double softeningFrac, double softeningFloor);

// Barnes-Hut tree-code acceleration. `theta` is the opening angle (spec: 0.5).
// Softening matches directAccelerations exactly: single-body leaves use the true
// per-pair value, aggregated nodes use their largest member radius (conservative).
// Ghost bodies are excluded as sources and receive zero acceleration.
std::vector<glm::dvec3> barnesHutAccelerations(const std::vector<Body>& bodies,
                                               double theta,
                                               double softeningFrac,
                                               double softeningFloor);

} // namespace gs
