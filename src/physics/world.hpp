#pragma once
#include "physics/body.hpp"
#include <vector>
namespace gs {
class World {
public:
    std::vector<Body> bodies;
};
}
