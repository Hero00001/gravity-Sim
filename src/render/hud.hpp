#pragma once
#include <string>
#include <cstdint>
#include <vector>

namespace gs::render {

// Snapshot of everything the HUD needs for one frame.
struct HudData {
    int fbw = 800, fbh = 600;
    float fps = 0.0f;
    double simTime = 0.0;      // seconds
    double timeScale = 1.0;
    bool paused = false;
    int bodyCount = 0;
    std::string sceneName = "Sandbox";
    bool hudVisible = true;
    // selection panel (right side)
    bool hasSelection = false;
    std::uint64_t selId = 0;
    double selMass = 0.0;      // kg
    double selSpeed = 0.0;     // km/s
    double selDist = 0.0;      // km to heaviest body
    // placing hints (bottom)
    bool placing = false;
    double placeMass = 0.0;    // kg
};

// Custom-rendered text overlay (no ImGui). Uses stb_easy_font, whose output is a
// list of QUADS (4 vertices each). We expand those to triangles with a static
// index buffer (6 indices/quad) and draw with glDrawElements(GL_TRIANGLES,...) —
// drawing the raw 4-vertex quads as GL_TRIANGLES garbles the glyphs. Drawn last
// with depth test off and blending on.
class Hud {
public:
    bool init();
    void shutdown();
    void draw(const HudData& d);
private:
    unsigned prog_ = 0;
    unsigned vao_ = 0, vbo_ = 0, ebo_ = 0;
    int uResLoc_ = -1, uColorLoc_ = -1, uScaleLoc_ = -1;
    std::vector<float> buf_;            // stb_easy_font vertex output (x,y,z,color)
    std::vector<unsigned> indices_;     // quad -> two triangles
};

} // namespace gs::render
