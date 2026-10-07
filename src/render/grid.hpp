#pragma once
#include "physics/body.hpp"
#include <vector>
namespace gs::render {
enum class GridMode { Bend, Flat, Off };
struct GridConfig {
    GridMode mode = GridMode::Bend;
    double sizeUnits = 20000.0;
    int divisions = 25;
    double planeYFactor = -0.03;
};
std::vector<float> buildGridBase(const GridConfig& cfg);
std::vector<float> displaceGrid(const std::vector<float>& base, const GridConfig& cfg,
                                const std::vector<gs::Body>& bodies);
}
