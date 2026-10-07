#include "app/input.hpp"
#include "physics/constants.hpp"
#include <GLFW/glfw3.h>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace fs = std::filesystem;
namespace gs::app {

void Input::update(GLFWwindow* win, InputContext& ctx) {
    world_ = ctx.world;
    cam_ = ctx.cam;
    fbw_ = ctx.fbw; fbh_ = ctx.fbh;

    // Camera flight is polled every frame (fixes stuttery WASD, bug 7).
    const float speed = 5000.0f * float(ctx.frameDelta);
    if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) cam_->pos += speed * cam_->front();
    if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) cam_->pos -= speed * cam_->front();
    if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) cam_->pos -= speed * cam_->right();
    if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) cam_->pos += speed * cam_->right();
    if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS) cam_->pos += speed * cam_->up();
    if (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) cam_->pos -= speed * cam_->up();

    // RMB mass growth while placing (single growth path).
    if (mode_ == Mode::Placing && rmbHeld_) {
        for (auto& b : world_->bodies)
            if (b.id == placingId_) b.mass *= std::exp(2.0 * ctx.frameDelta);
    }

    // Follow: lock camera to the followed body each frame (after the WASD poll, so it wins).
    if (followId_ != 0) {
        for (const auto& b : world_->bodies) {
            if (b.id == followId_) { cam_->updateFollow(glm::vec3(b.position) / float(gs::UNIT)); break; }
        }
    }

    // Grab: track pointer velocity (units/sec) for the fling on release.
    if (grabbing_) {
        const glm::vec3 cur = grabPlanePointUnits(lastX_, lastY_);
        const double dt = std::max(ctx.frameDelta, 1e-3);
        const glm::vec3 vel = (cur - grabPrevUnits_) / float(dt);
        grabVelUnits_ = grabVelUnits_ * 0.5f + vel * 0.5f;
        grabPrevUnits_ = cur;
    }
}

void Input::onCursor(double x, double y) {
    if (firstMouse_) { lastX_ = float(x); lastY_ = float(y); firstMouse_ = false; }
    if (grabbing_) {
        const glm::vec3 p = grabPlanePointUnits(float(x), float(y));
        for (auto& b : world_->bodies)
            if (b.id == grabbedId_) { b.position = glm::dvec3(p) * double(gs::UNIT); break; }
        lastX_ = float(x); lastY_ = float(y);
        return;
    }
    cam_->rotate(float(x - lastX_) * 0.1f, float(lastY_ - y) * 0.1f);
    lastX_ = float(x); lastY_ = float(y);

    // Promote a press into a grab once the pointer moves > 5 px (bug 12 discipline).
    if (pendingPress_) {
        const float dx = float(x) - pressX_, dy = float(y) - pressY_;
        if (std::sqrt(dx * dx + dy * dy) > 5.0f) {
            grabbing_ = true;
            grabbedId_ = pressBodyId_;
            pendingPress_ = false;
            for (auto& b : world_->bodies) {
                if (b.id == grabbedId_) {
                    b.grabbed = true;
                    grabPlaneCenter_ = glm::vec3(b.position) / float(gs::UNIT);
                    grabPrevUnits_ = grabPlaneCenter_;
                    grabVelUnits_ = glm::vec3(0.0f);
                    break;
                }
            }
        }
    }
}

void Input::startPlacing() {
    gs::Body b;
    b.mass = 1e22;
    b.density = 3344.0;
    b.color = {0.9f, 0.4f, 0.2f, 1.0f};
    b.ghost = true;
    b.trail.setCap(kTrailCaps[trailCapIdx_]);
    // Spawn ~20% of the way from the camera toward its focus, IN FRONT of the camera.
    // Work in world-units, then convert to metres once. (The old code added the camera
    // position in *units* to an offset in *metres*, so the body landed ~d0 units from
    // the ORIGIN — nowhere near the camera — and appeared as "nothing happened".)
    const float viewDist = std::max(glm::length(cam_->pos), 100.0f);   // units
    const float d0 = viewDist * 0.2f;                                  // units
    const glm::vec3 spawnUnits = cam_->pos + cam_->front() * d0;
    b.position = glm::dvec3(spawnUnits) * double(gs::UNIT);
    placingId_ = world_->spawn(b);
    mode_ = Mode::Placing;
}

void Input::finishPlacing(bool cancel) {
    if (mode_ != Mode::Placing) return;
    if (cancel) world_->removeById(placingId_);
    else
        for (auto& b : world_->bodies)
            if (b.id == placingId_) { b.ghost = false; break; }
    mode_ = Mode::Idle;
    placingId_ = 0;
}

// Screen-space pick: project each body to NDC, measure pixel distance to the cursor,
// accept the nearest within max(20px, projectedRadius+8px). Returns 0 if none.
std::uint64_t Input::pickBody(float px, float py) {
    if (!world_ || !cam_) return 0;
    const float aspect = (fbh_ > 0) ? float(fbw_) / float(fbh_) : 1.0f;
    float bestD = 1e30f; std::uint64_t best = 0;
    for (const auto& b : world_->bodies) {
        if (b.ghost) continue;
        const glm::vec3 u = glm::vec3(b.position) / float(gs::UNIT);
        const glm::vec3 ndc = cam_->projectToNDC(u, aspect);
        if (ndc.z < -1.0f || ndc.z > 1.0f) continue;
        const float sx = (ndc.x * 0.5f + 0.5f) * float(fbw_);
        const float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * float(fbh_);
        const float dx = sx - px, dy = sy - py;
        const float d = std::sqrt(dx * dx + dy * dy);
        const float dispU = float(world_->displayRadius(b) / gs::UNIT);
        const glm::vec3 edge = cam_->projectToNDC(u + cam_->right() * dispU, aspect);
        float thr = 20.0f;
        if (edge.z > -1.0f && edge.z < 1.0f) {
            const float ex = (edge.x * 0.5f + 0.5f) * float(fbw_);
            const float ey = (1.0f - (edge.y * 0.5f + 0.5f)) * float(fbh_);
            thr = std::max(20.0f, std::sqrt((ex - sx) * (ex - sx) + (ey - sy) * (ey - sy)) + 8.0f);
        }
        if (d <= thr && d < bestD) { bestD = d; best = b.id; }
    }
    return best;
}

// Intersect the cursor ray with the fixed grab plane (through the body at grab start, normal = camera front).
glm::vec3 Input::grabPlanePointUnits(float px, float py) {
    const float aspect = (fbh_ > 0) ? float(fbw_) / float(fbh_) : 1.0f;
    const float ndcX = (px / float(fbw_)) * 2.0f - 1.0f;
    const float ndcY = 1.0f - (py / float(fbh_)) * 2.0f;
    const glm::vec3 dir = cam_->unprojectDir(ndcX, ndcY, aspect);
    return gs::render::Camera::rayPlaneIntersect(cam_->pos, dir, grabPlaneCenter_, cam_->front());
}

void Input::onMouseButton(int button, int action) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            if (mode_ == Mode::Placing) return;            // ignore extra press while placing
            const float px = lastX_, py = lastY_;
            std::uint64_t hit = pickBody(px, py);
            if (hit != 0) {
                pendingPress_ = true;
                pressBodyId_ = hit;
                pressX_ = px; pressY_ = py;
            } else {
                startPlacing();                            // empty space → spawn
            }
        } else if (action == GLFW_RELEASE) {
            if (mode_ == Mode::Placing) {
                finishPlacing(false);
            } else if (grabbing_) {
                for (auto& b : world_->bodies) {
                    if (b.id == grabbedId_) {
                        glm::dvec3 v = glm::dvec3(grabVelUnits_) * gs::UNIT;   // units/s → m/s
                        const double sp = glm::length(v);
                        if (sp > 1e8) v *= 1e8 / sp;                          // clamp fling speed
                        b.velocity = v;
                        b.grabbed = false;
                        break;
                    }
                }
                grabbing_ = false; grabbedId_ = 0;
            } else if (pendingPress_) {
                selectedId_ = pressBodyId_;                // click without drag → select
                pendingPress_ = false;
            }
        }
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
        rmbHeld_ = (action == GLFW_PRESS);
}

void Input::onScroll(double yoffset) {
    if (mode_ == Mode::Placing) {
        for (auto& b : world_->bodies) {
            if (b.id != placingId_) continue;
            const glm::vec3 units = glm::vec3(b.position) / float(gs::UNIT);
            const float d = glm::length(units - cam_->pos);
            const glm::vec3 moved = units + cam_->front() * float(yoffset) * 0.1f * d;
            b.position = glm::dvec3(moved) * double(gs::UNIT);
        }
    } else {
        const float d = std::max(glm::length(cam_->pos), 500.0f);
        const float dir = (yoffset > 0.0) ? 1.0f : -1.0f;
        cam_->pos += cam_->front() * dir * 0.2f * d;
    }
}

void Input::onKey(int key, int action, int) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    if (key == GLFW_KEY_Q) quitRequested = true;
    if (key == GLFW_KEY_P && action == GLFW_PRESS) world_->paused = !world_->paused;
    if (key == GLFW_KEY_PERIOD && world_->paused && action == GLFW_PRESS)
        world_->stepOnce();                                   // single-step while paused

    if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS && trailCapIdx_ > 0) {
        --trailCapIdx_;
        world_->setTrailCap(kTrailCaps[trailCapIdx_]);
    }
    if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS && trailCapIdx_ < 5) {
        ++trailCapIdx_;
        world_->setTrailCap(kTrailCaps[trailCapIdx_]);
    }

    // Time controls (spec #6): = / - step timeScale by ~x√10 (range 1e-3..1e7); 0 resets to realtime.
    if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS)
        world_->timeScale = std::min(world_->timeScale * std::sqrt(10.0), 1e7);
    if (key == GLFW_KEY_MINUS && action == GLFW_PRESS)
        world_->timeScale = std::max(world_->timeScale / std::sqrt(10.0), 1e-3);
    if (key == GLFW_KEY_0 && action == GLFW_PRESS)
        world_->timeScale = 1.0;
    // H toggles the HUD.
    if (key == GLFW_KEY_H && action == GLFW_PRESS) hudVisible_ = !hudVisible_;

    // Preset scenes 1-4 (spec #3).
    if (action == GLFW_PRESS && key >= GLFW_KEY_1 && key <= GLFW_KEY_4)
        loadPreset(key - GLFW_KEY_1 + 1);
    // Save / load / reset (spec #9).
    if (action == GLFW_PRESS && key == GLFW_KEY_F5) quicksave();
    if (action == GLFW_PRESS && key == GLFW_KEY_F9) quickload();
    if (action == GLFW_PRESS && key == GLFW_KEY_F10) resetToSnapshot();

    // Follow toggle on the selected body (F).
    if (key == GLFW_KEY_F && action == GLFW_PRESS && selectedId_ != 0) {
        if (followId_ == selectedId_) { cam_->stopFollow(); followId_ = 0; }
        else {
            for (const auto& b : world_->bodies) {
                if (b.id == selectedId_) {
                    cam_->startFollow(glm::vec3(b.position) / float(gs::UNIT));
                    followId_ = selectedId_;
                    break;
                }
            }
        }
    }
    // Delete removes the selected body.
    if (key == GLFW_KEY_DELETE && action == GLFW_PRESS && selectedId_ != 0) {
        if (followId_ == selectedId_) { cam_->stopFollow(); followId_ = 0; }
        world_->removeById(selectedId_);
        selectedId_ = 0;
    }
    // Esc cancels a placement, otherwise deselects.
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        if (mode_ == Mode::Placing) { finishPlacing(true); return; }
        selectedId_ = 0;
        pendingPress_ = false;
    }

    // Arrow-key nudge while placing (5% of view distance, bug 12).
    if (mode_ == Mode::Placing) {
        for (auto& b : world_->bodies) {
            if (b.id != placingId_) continue;
            const glm::vec3 units = glm::vec3(b.position) / float(gs::UNIT);
            const float d = glm::length(units - cam_->pos);
            const float stepUnits = 0.05f * d;
            glm::vec3 delta(0.0f);
            if (key == GLFW_KEY_UP)    delta += cam_->up();
            if (key == GLFW_KEY_DOWN)  delta -= cam_->up();
            if (key == GLFW_KEY_RIGHT) delta += cam_->right();
            if (key == GLFW_KEY_LEFT)  delta -= cam_->right();
            b.position = glm::dvec3(units + delta * stepUnits) * double(gs::UNIT);
            break;
        }
    }
}

gs::render::GridConfig Input::toGridConfig(const gs::SceneGrid& g) {
    gs::render::GridConfig c;
    c.mode = g.mode == gs::SceneGridMode::Flat ? gs::render::GridMode::Flat
            : g.mode == gs::SceneGridMode::Off  ? gs::render::GridMode::Off
                                               : gs::render::GridMode::Bend;
    c.sizeUnits = g.sizeUnits;
    c.divisions = g.divisions;
    return c;
}

gs::SceneGrid Input::toSceneGrid(const gs::render::GridConfig& c) {
    gs::SceneGrid g;
    g.mode = c.mode == gs::render::GridMode::Flat ? gs::SceneGridMode::Flat
            : c.mode == gs::render::GridMode::Off  ? gs::SceneGridMode::Off
                                                  : gs::SceneGridMode::Bend;
    g.sizeUnits = c.sizeUnits;
    g.divisions = c.divisions;
    return g;
}

std::string Input::quicksavePath() const {
    return (fs::path("scenes") / "quicksave.gsim").string();
}

void Input::loadPreset(int n) {
    if (!world_) return;
    gs::Scene s = gs::preset(n);
    if (s.bodies.empty()) return;
    gs::applySceneToWorld(*world_, s);
    snapshot_ = s;
    sceneName_ = s.name;
    gridCfg_ = toGridConfig(s.grid);
    refRadiusUnits_ = s.refRadiusUnits;
    selectedId_ = 0; followId_ = 0; grabbing_ = false; grabbedId_ = 0; pendingPress_ = false;
    if (cam_) cam_->stopFollow();
    if (cam_) cam_->frameScene(s.refRadiusUnits);
}

void Input::quicksave() {
    if (!world_) return;
    gs::Scene s = gs::snapshotFromWorld(*world_, sceneName_, toSceneGrid(gridCfg_), refRadiusUnits_);
    std::error_code ec;
    fs::create_directories(fs::path("scenes"), ec);
    gs::saveScene(s, quicksavePath());
}

void Input::quickload() {
    if (!world_) return;
    try {
        gs::Scene s = gs::loadScene(quicksavePath());
        gs::applySceneToWorld(*world_, s);
        snapshot_ = s;
        sceneName_ = s.name;
        gridCfg_ = toGridConfig(s.grid);
        refRadiusUnits_ = s.refRadiusUnits;
        selectedId_ = 0; followId_ = 0; grabbing_ = false; grabbedId_ = 0; pendingPress_ = false;
        if (cam_) cam_->stopFollow();
        if (cam_) cam_->frameScene(s.refRadiusUnits);
    } catch (const std::exception&) {
        // No quicksave yet (or unreadable) — ignore.
    }
}

void Input::resetToSnapshot() {
    if (!world_ || snapshot_.bodies.empty()) return;
    gs::applySceneToWorld(*world_, snapshot_);
    sceneName_ = snapshot_.name;
    gridCfg_ = toGridConfig(snapshot_.grid);
    refRadiusUnits_ = snapshot_.refRadiusUnits;
    selectedId_ = 0; followId_ = 0; grabbing_ = false; grabbedId_ = 0; pendingPress_ = false;
    if (cam_) cam_->stopFollow();
    if (cam_) cam_->frameScene(snapshot_.refRadiusUnits);
}

} // namespace gs::app
