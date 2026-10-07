#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "render/renderer.hpp"
#include "render/mesh.hpp"
#include "render/camera.hpp"
#include "render/grid.hpp"
#include "physics/constants.hpp"
#include "app/input.hpp"
#include "render/hud.hpp"
#include <string>
#include <iostream>
#include <cmath>

namespace {
// Aggregates the per-run app state so GLFW callbacks can reach it via the global pointer.
struct App {
    gs::World world;
    gs::render::Camera cam;
    gs::app::Input input;
    gs::render::Hud hud;
};
App* g_app = nullptr;

void keyCb(GLFWwindow*, int key, int, int action, int mods) {
    g_app->input.onKey(key, action, mods);
}
void mouseBtnCb(GLFWwindow*, int button, int action, int) {
    g_app->input.onMouseButton(button, action);
}
void cursorCb(GLFWwindow*, double x, double y) { g_app->input.onCursor(x, y); }
void scrollCb(GLFWwindow*, double, double y) { g_app->input.onScroll(y); }
} // namespace

int main() {
    if (!glfwInit()) { std::cerr << "glfwInit failed\n"; return 1; }
    GLFWwindow* win = glfwCreateWindow(800, 600, "Gravity Sim", nullptr, nullptr);
    if (!win) { std::cerr << "window failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::cerr << "glewInit failed\n"; return 1; }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    App app;
    g_app = &app;

    // App starts with a preset loaded (spec #3 / bug 9): wire input to the app state
    // and load preset 1 (Solar System), which also auto-frames the camera.
    app.input.bind(&app.world, &app.cam);
    app.input.loadPreset(1);

    glfwSetWindowUserPointer(win, &app);
    glfwSetKeyCallback(win, keyCb);
    glfwSetMouseButtonCallback(win, mouseBtnCb);
    glfwSetCursorPosCallback(win, cursorCb);
    glfwSetScrollCallback(win, scrollCb);
    glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    gs::render::Renderer renderer;
    if (!renderer.init()) return 1;
    if (!app.hud.init()) return 1;

    double last = glfwGetTime();                          // first-frame guard (bug 13)
    while (!glfwWindowShouldClose(win)) {
        const double now = glfwGetTime();
        const double fd = now - last;
        last = now;

        int fbw, fbh;
        glfwGetFramebufferSize(win, &fbw, &fbh);

        gs::app::InputContext ctx{&app.world, &app.cam, fd, fbw, fbh};
        app.input.update(win, ctx);
        if (app.input.quitRequested) glfwSetWindowShouldClose(win, GLFW_TRUE);

        app.world.advance(fd);

        gs::render::HudData hd;
        hd.fbw = fbw; hd.fbh = fbh;
        hd.fps = float(1.0 / std::max(fd, 1e-3));
        hd.simTime = app.world.simTime;
        hd.timeScale = app.world.timeScale;
        hd.paused = app.world.paused;
        hd.bodyCount = int(app.world.bodies.size());
        hd.sceneName = app.input.sceneName();
        hd.hudVisible = app.input.hudVisible();
        {
            const auto sid = app.input.selectedId();
            if (sid != 0) {
                for (const auto& b : app.world.bodies) {
                    if (b.id != sid) continue;
                    hd.hasSelection = true; hd.selId = b.id;
                    hd.selMass = b.mass;
                    hd.selSpeed = glm::length(b.velocity) / 1000.0;     // km/s
                    double maxm = -1.0; const gs::Body* heavy = nullptr;
                    for (const auto& o : app.world.bodies)
                        if (!o.ghost && o.mass > maxm) { maxm = o.mass; heavy = &o; }
                    if (heavy && heavy->id != b.id)
                        hd.selDist = glm::length(b.position - heavy->position) / 1000.0;  // km
                    break;
                }
            }
            if (app.input.isPlacing()) {
                hd.placing = true;
                for (const auto& b : app.world.bodies)
                    if (b.id == app.input.placingId()) { hd.placeMass = b.mass; break; }
            }
        }

        renderer.draw(app.world, app.cam, fbw, fbh, app.input.gridConfig(), app.input.selectedId());
        app.hud.draw(hd);
        glfwSwapBuffers(win);
        glfwPollEvents();
    }
    app.hud.shutdown();
    renderer.shutdown();
    glfwTerminate();
    return 0;
}
