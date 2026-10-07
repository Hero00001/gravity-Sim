#pragma once
#include <vector>
namespace gs::render {
// Triangle soup centered at the origin; loops i in [0, stacks) — no off-by-one ring.
std::vector<float> sphereVertices(double radius, int stacks = 16, int sectors = 24);
}
