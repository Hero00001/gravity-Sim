#pragma once
#include "physics/body.hpp"
#include <string>
#include <vector>

namespace gs {

class World;   // forward declaration; only references are used in this header

// Per-scene grid configuration. Kept in the physics layer (GL-free) as a mirror
// of render::GridConfig so Scene serialization never depends on render types.
enum class SceneGridMode { Bend, Flat, Off };
struct SceneGrid {
    SceneGridMode mode = SceneGridMode::Bend;
    double sizeUnits = 20000.0;
    int divisions = 25;
};

// A complete, serializable scene: bodies + simulation/grid metadata.
// Trails, ghost/grabbed flags and glow are intentionally NOT part of a Scene.
struct Scene {
    std::string name;
    std::vector<Body> bodies;
    double timeScale = 1.0;     // seconds of sim per second of real time
    SceneGrid grid;
    double refRadiusUnits = 5000.0;   // camera framing reference (world-units)
};

// Built-in presets (spec #3). n in 1..4; returns an empty Scene otherwise.
Scene makeSolarSystem();
Scene makeBinaryStars();
Scene makeSlingshot();
Scene makeChaos();
Scene preset(int n);

// Save / load (spec #9). Plain-text "gsim 1" format.
// saveScene returns false on IO failure; loadScene throws std::runtime_error on parse error.
bool saveScene(const Scene& s, const std::string& path);
Scene loadScene(const std::string& path);

// World <-> Scene bridge. Used to load a scene into the live sim and to snapshot
// the current state for the F10 reset key (reset == re-apply the last loaded scene).
Scene snapshotFromWorld(const World& w, const std::string& name,
                        const SceneGrid& grid, double refRadiusUnits);
void applySceneToWorld(World& w, const Scene& s);

} // namespace gs
