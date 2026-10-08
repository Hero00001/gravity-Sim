#pragma once
#include "physics/world.hpp"
#include "physics/scene.hpp"
#include "render/camera.hpp"
#include "render/grid.hpp"
#include <GLFW/glfw3.h>
#include <cstdint>
#include <cstddef>
#include <string>
#include <glm/glm.hpp>

namespace gs::app {

// Per-frame context handed to Input::update().
struct InputContext {
    gs::World* world = nullptr;
    gs::render::Camera* cam = nullptr;
    double frameDelta = 0.0;    // raw wall seconds this frame
    int fbw = 800, fbh = 600;
};

// Owns the input state machine: Idle <-> Placing, plus selection / follow / grab,
// and scene-level state (current grid config, name, reset snapshot). All world
// mutation flows through here.
class Input {
public:
    void onKey(int key, int action, int mods);
    void onMouseButton(int button, int action);
    void onCursor(double x, double y);
    void onScroll(double yoffset);
    void update(GLFWwindow* win, InputContext& ctx);   // polled every frame
    bool quitRequested = false;
    std::uint64_t selectedId() const { return selectedId_; }
    std::uint64_t hoverId() const { return hoverId_; }
    bool hudVisible() const { return hudVisible_; }
    bool helpVisible() const { return helpVisible_; }
    bool isFollowing() const { return followId_ != 0; }
    int trailCap() const { return int(kTrailCaps[trailCapIdx_]); }
    bool isPlacing() const { return mode_ == Mode::Placing; }
    std::uint64_t placingId() const { return placingId_; }

    // Scene-level accessors (spec #3 / #9). main.cpp reads these each frame.
    void bind(gs::World* w, gs::render::Camera* c) { world_ = w; cam_ = c; }
    void loadPreset(int n);                             // 1..4 (spec #3)
    const gs::render::GridConfig& gridConfig() const { return gridCfg_; }
    const std::string& sceneName() const { return sceneName_; }

private:
    enum class Mode { Idle, Placing };
    Mode mode_ = Mode::Idle;
    std::uint64_t placingId_ = 0;
    bool rmbHeld_ = false;
    bool lookHeld_ = false;                             // RMB/MMB drag = look around
    float lastX_ = 400.f, lastY_ = 300.f;
    bool firstMouse_ = true;
    int trailCapIdx_ = 4;                               // 512
    bool hudVisible_ = true;
    bool helpVisible_ = true;                           // F1; auto-hides after a while
    bool helpAutoHidden_ = false;
    double appTime_ = 0.0;                              // wall seconds since start
    gs::World* world_ = nullptr;
    gs::render::Camera* cam_ = nullptr;
    int fbw_ = 800, fbh_ = 600;
    static constexpr std::size_t kTrailCaps[6] = {64, 128, 256, 512, 1024, 2048};

    // selection / follow / grab
    std::uint64_t selectedId_ = 0;
    std::uint64_t hoverId_ = 0;
    std::uint64_t followId_ = 0;
    bool grabbing_ = false;
    std::uint64_t grabbedId_ = 0;
    bool pendingPress_ = false;
    std::uint64_t pressBodyId_ = 0;
    float pressX_ = 0, pressY_ = 0;
    glm::vec3 grabPlaneCenter_{0, 0, 0};               // fixed plane through body at grab start
    glm::vec3 grabPrevUnits_{0, 0, 0};
    glm::vec3 grabVelUnits_{0, 0, 0};                  // units/sec, smoothed

    // scene-level state (spec #3 / #9)
    gs::render::GridConfig gridCfg_;                   // current grid, mirrored for the renderer
    std::string sceneName_ = "Sandbox";
    gs::Scene snapshot_;                               // reset snapshot (taken at preset load / quickload)
    double refRadiusUnits_ = 5000.0;                  // camera framing reference

    void startPlacing();
    void finishPlacing(bool cancel);
    std::uint64_t pickBody(float px, float py);
    glm::vec3 grabPlanePointUnits(float px, float py);

    void quicksave();                                  // F5
    void quickload();                                  // F9
    void resetToSnapshot();                            // F10
    static gs::render::GridConfig toGridConfig(const gs::SceneGrid& g);
    static gs::SceneGrid toSceneGrid(const gs::render::GridConfig& c);
    std::string quicksavePath() const;
};

} // namespace gs::app
