#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "render/renderer.hpp"
#include "render/mesh.hpp"
#include "render/camera.hpp"
#include "physics/constants.hpp"
#include <iostream>

namespace {
gs::render::Camera* g_cam = nullptr;
double g_lastX = 400.0, g_lastY = 300.0;
bool g_firstMouse = true;

void cursorCb(GLFWwindow*, double x, double y) {
    if (g_firstMouse) { g_lastX = x; g_lastY = y; g_firstMouse = false; }
    g_cam->rotate(float(x - g_lastX) * 0.1f, float(g_lastY - y) * 0.1f);
    g_lastX = x; g_lastY = y;
}
} // namespace

int main() {
    if (!glfwInit()) { std::cerr << "glfwInit failed\n"; return 1; }
    GLFWwindow* win = glfwCreateWindow(800, 600, "Gravity Sim v2", nullptr, nullptr);
    if (!win) { std::cerr << "window failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::cerr << "glewInit failed\n"; return 1; }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    gs::render::Camera cam;
    g_cam = &cam;
    glfwSetCursorPosCallback(win, cursorCb);
    glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    gs::render::Renderer renderer;
    if (!renderer.init()) return 1;

    double last = glfwGetTime();
    while (!glfwWindowShouldClose(win)) {
        const double now = glfwGetTime();
        const float fd = float(now - last);
        last = now;

        const float speed = 5000.0f * fd;
        if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) cam.pos += speed * cam.front();
        if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) cam.pos -= speed * cam.front();
        if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) cam.pos -= speed * cam.right();
        if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) cam.pos += speed * cam.right();
        if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS) cam.pos += speed * cam.up();
        if (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) cam.pos -= speed * cam.up();
        if (glfwGetKey(win, GLFW_KEY_Q) == GLFW_PRESS) glfwSetWindowShouldClose(win, GLFW_TRUE);

        int fbw, fbh;
        glfwGetFramebufferSize(win, &fbw, &fbh);
        renderer.draw(gs::World{}, cam, fbw, fbh, gs::render::GridConfig{});
        glfwSwapBuffers(win);
        glfwPollEvents();
    }
    renderer.shutdown();
    glfwTerminate();
    return 0;
}
