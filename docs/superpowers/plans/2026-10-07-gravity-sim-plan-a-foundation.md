# Gravity Sim — Plan A (Foundation) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rebuild the gravity sandbox on a GL-free, SI-unit physics core with leapfrog integration, merging collisions, fixed timestep, pure grid displacement, trails, and redesigned input — delivering a single testable app (`gravity_sim_v2`) with bugs 1–13 and 15 fixed.

**Architecture:** `physics/` is pure C++ (glm math only, no GL/GLFW) and owns Body/World/trails/timestep; `render/` holds GL-free pure functions (mesh, grid) plus the GL renderer/camera; `app/` owns the input state machine and main loop. Tests are assert-based executables registered in CTest.

**Tech Stack:** C++20, CMake 3.16+, vendored GLFW 3.4 / GLEW / GLM (existing `third_party/`), OpenGL 3.3. No new dependencies in this plan.

**Spec:** `docs/superpowers/specs/2026-10-07-gravity-sim-overhaul-design.md` — the plan argues from the spec; executors read both. This plan covers spec **delivery Phases 1–4** (Phases 5–9: separate follow-up plans).

## Global Constraints

- Spec values (verbatim): `dt = 1/480 s` · max 8 steps/frame · frameDelta clamped ≤ **0.25 s** · `UNIT = 1e7` m per world-unit · `MIN_VISUAL = 3e6` m · softening `ε = max(0.1 × (R₁+R₂), 5e4)` · leapfrog KDK · trail cap default **512**, sample ≥ **0.033** wall-seconds · grid plane `y = −0.03 × size`, dip clamp `−0.25 × size`, dip `= 2√(rs(d − rs))`, `rs = 2Gm/c²` skipped when `d ≤ rs`.
- `physics/` sources must not include GL, GLFW, or OpenGL headers (glm allowed). Tests link only `physics/` + GL-free `render/` files.
- Physics state uses **double**; floats only at the render boundary.
- Existing targets `gravity_sim`, `gravity_sim_3Dgrid`, `3D_test` must keep building unchanged until the (later) consolidation plan.
- C++20, Windows x64, MSVC or MinGW via the existing vendored deps. No new dependencies.
- **No git commits without explicit user approval.** At each "Commit" step, show the command and ask the user first.
- Controls (spec §7, binding): `P` pause toggle · `Space`/`Left Shift` camera up/down · `Q` quit · `[` `]` trail length · placement: wheel = depth, arrows = 5%-of-view-distance steps, RMB hold = grow, Esc = cancel, release = finalize.
- World/scene terminology: bodies live in **meters**; camera, grid, and rendering live in **world-units** (×`UNIT` to convert).

---

### Task 1: CMake scaffold + minimal windowed app

**Files:**
- Modify: `CMakeLists.txt`
- Create: `src/physics/constants.hpp`, `src/physics/body.hpp`, `src/render/mesh.hpp`, `src/render/mesh.cpp`, `src/render/camera.hpp`, `src/render/renderer.hpp`, `src/render/renderer.cpp`, `src/app/main.cpp`

**Interfaces:**
- Produces (used by all later tasks):
  - `gs::G = 6.67430e-11`, `gs::C = 299792458.0`, `gs::UNIT = 1e7` (`physics/constants.hpp`)
  - `gs::Body` with `radius() const → double`, fields `position/velocity (glm::dvec3)`, `mass/density (double)`, `color (glm::vec4)`, `id`, `ghost`, `glow`, `trail`
  - `gs::render::sphereVertices(double radius, int stacks=16, int sectors=24) → std::vector<float>` (in `mesh.hpp`, namespace `gs::render`)
  - `gs::render::Camera` with `front()/right()/up()/view()/projection(aspect)/rotate(dx,dy)`, fields `pos (glm::vec3, world-units)`, `yaw`, `pitch`
  - `gs::render::Renderer::{init(), shutdown(), draw(world, cam, fbw, fbh, gridCfg)}` — for this task, an overload `drawOne(const std::vector<float>& verts, const glm::vec3& posUnits, const glm::vec4& color)`
  - `gravity_target(name sources...)` CMake function now variadic

- [ ] **Step 1: Rewrite the CMake target function and add test wiring**

Replace the body of `gravity_target` in `CMakeLists.txt` (the three existing call sites stay valid — same first argument, sources passed through), and append at the end of the file:

```cmake
function(gravity_target name)
  add_executable(${name} ${ARGN})
  target_include_directories(${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${GLM_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR})
  target_link_libraries(${name} PRIVATE
    ${GLFW_IMPLIB}
    ${GLEW_IMPLIB}
    ${PLATFORM_LIBS})
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${GLFW_DLL}" "${GLEW_DLL}"
      "$<TARGET_FILE_DIR:${name}>"
    COMMENT "Copying runtime DLLs for ${name}")
endfunction()

gravity_target(gravity_sim        gravity_sim.cpp)
gravity_target(gravity_sim_3Dgrid gravity_sim_3Dgrid.cpp)
gravity_target(3D_test            3D_test.cpp)

gravity_target(gravity_sim_v2
  src/app/main.cpp
  src/render/renderer.cpp
  src/render/mesh.cpp)

enable_testing()
```

(The `gravity_target` calls currently in the file are replaced by the block above — net effect: same three legacy targets, plus `gravity_sim_v2`, plus `enable_testing()`.)

- [ ] **Step 2: Create `src/physics/constants.hpp`**

```cpp
#pragma once
namespace gs {
inline constexpr double G   = 6.67430e-11;   // m^3 kg^-1 s^-2
inline constexpr double C   = 299792458.0;    // m/s
inline constexpr double UNIT = 1e7;           // meters per world-unit
}
```

- [ ] **Step 3: Create `src/physics/body.hpp`**

```cpp
#pragma once
#include <glm/glm.hpp>
#include <deque>
#include <cstdint>
#include <cmath>

namespace gs {

struct TrailRing {
    std::deque<glm::dvec3> pts;
    std::size_t cap = 512;
    void push(const glm::dvec3& p) {
        pts.push_back(p);
        while (pts.size() > cap) pts.pop_front();
    }
    void setCap(std::size_t c) {
        cap = c;
        while (pts.size() > cap) pts.pop_front();
    }
    void clear() { pts.clear(); }
};

struct Body {
    glm::dvec3 position{0.0};   // m
    glm::dvec3 velocity{0.0};   // m/s
    double mass   = 0.0;        // kg
    double density = 3344.0;    // kg/m^3
    glm::vec4 color{1.0f, 0.0f, 0.0f, 1.0f};
    std::uint64_t id = 0;
    bool ghost = false;         // true while being placed: no forces, no merges, no trail
    bool glow  = false;         // rendered as blown-out point source
    TrailRing trail;

    double radius() const {
        return std::cbrt(3.0 * mass / (4.0 * 3.14159265358979323846 * density));
    }
};

} // namespace gs
```

- [ ] **Step 4: Create `src/render/mesh.hpp` and `mesh.cpp`**

```cpp
#pragma once
#include <vector>
namespace gs::render {
// Triangle soup centered at the origin; loops i in [0, stacks) — no off-by-one ring.
std::vector<float> sphereVertices(double radius, int stacks = 16, int sectors = 24);
}
```

```cpp
#include "render/mesh.hpp"
#include <cmath>

namespace gs::render {
namespace {
void sph(double r, double theta, double phi, float out[3]) {
    out[0] = float(r * std::sin(theta) * std::cos(phi));
    out[1] = float(r * std::cos(theta));
    out[2] = float(r * std::sin(theta) * std::sin(phi));
}
}

std::vector<float> sphereVertices(double radius, int stacks, int sectors) {
    std::vector<float> v;
    v.reserve(std::size_t(stacks) * sectors * 18);
    const double PI = 3.14159265358979323846;
    for (int i = 0; i < stacks; ++i) {
        const double t1 = double(i) / stacks * PI;
        const double t2 = double(i + 1) / stacks * PI;
        for (int j = 0; j < sectors; ++j) {
            const double p1 = double(j) / sectors * 2.0 * PI;
            const double p2 = double(j + 1) / sectors * 2.0 * PI;
            float a[3], b[3], c[3], d[3];
            sph(radius, t1, p1, a); sph(radius, t1, p2, b);
            sph(radius, t2, p1, c); sph(radius, t2, p2, d);
            for (float* p : {a, b, c}) v.insert(v.end(), p, p + 3);
            for (float* p : {b, d, c}) v.insert(v.end(), p, p + 3);
        }
    }
    return v;
}
} // namespace gs::render
```

- [ ] **Step 5: Create `src/render/camera.hpp` (header-only)**

```cpp
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace gs::render {

struct Camera {
    glm::vec3 pos{0.0f, 1000.0f, 5000.0f};   // world-units
    float yaw = -90.0f;
    float pitch = 0.0f;
    float fov = 45.0f;
    float nearPlane = 0.1f;
    float farPlane = 500000.0f;

    glm::vec3 front() const {
        const float yr = glm::radians(yaw), pr = glm::radians(pitch);
        return glm::normalize(glm::vec3(
            std::cos(yr) * std::cos(pr), std::sin(pr), std::sin(yr) * std::cos(pr)));
    }
    glm::vec3 right() const { return glm::normalize(glm::cross(front(), glm::vec3(0, 1, 0))); }
    glm::vec3 up()    const { return glm::normalize(glm::cross(right(), front())); }

    void rotate(float dx, float dy) {
        yaw += dx;
        pitch = std::clamp(pitch + dy, -89.0f, 89.0f);
    }
    glm::mat4 view() const { return glm::lookAt(pos, pos + front(), glm::vec3(0, 1, 0)); }
    glm::mat4 projection(float aspect) const {
        return glm::perspective(glm::radians(fov), aspect, nearPlane, farPlane);
    }
};

} // namespace gs::render
```

- [ ] **Step 6: Create `src/render/renderer.hpp` and `renderer.cpp` (minimal: body shader + one-sphere draw)**

```cpp
#pragma once
#include "render/camera.hpp"
#include "render/grid.hpp"
#include "physics/world.hpp"
#include <vector>
#include <cstdint>
#include <unordered_map>

namespace gs::render {

class Renderer {
public:
    bool init();
    void shutdown();
    // Full draw path — implemented in Task 10/12; for Task 1 it draws grid + bodies.
    void draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
              const GridConfig& grid);
    // Simple path used by the Task-1 scaffold.
    void drawOne(const std::vector<float>& verts, const glm::vec3& posUnits,
                 const glm::vec4& color);

private:
    struct GpuBody { unsigned vao = 0, vbo = 0; double radius = -1.0; int count = 0; };
    unsigned progBody_ = 0, progTrail_ = 0;
    unsigned gridVao_ = 0, gridVbo_ = 0;
    unsigned trailVao_ = 0, trailVbo_ = 0;
    std::unordered_map<std::uint64_t, GpuBody> gpu_;
    std::vector<float> scratch_;

    void syncBodies(const gs::World& world);
    void drawBodies(const gs::World& world, const Camera& cam, float aspect);
    void drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam, float aspect);
    void drawTrails(const gs::World& world, const Camera& cam, float aspect);
};

} // namespace gs::render
```

```cpp
#include "render/renderer.hpp"
#include "render/grid.hpp"
#include "physics/constants.hpp"
#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

namespace gs::render {
namespace {

const char* kBodyVS = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 model, view, projection;
out float lightIntensity;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    vec3 worldPos = (model * vec4(aPos, 1.0)).xyz;
    vec3 normal = normalize(aPos);
    lightIntensity = max(dot(normal, normalize(-worldPos)), 0.15);
})glsl";

const char* kBodyFS = R"glsl(
#version 330 core
in float lightIntensity;
out vec4 FragColor;
uniform vec4 objectColor;
uniform bool isGrid;
uniform bool GLOW;
void main() {
    if (isGrid)      FragColor = objectColor;
    else if (GLOW)    FragColor = vec4(objectColor.rgb * 100000.0, objectColor.a);
    else {
        float fade = smoothstep(0.0, 1.0, lightIntensity);
        FragColor = vec4(objectColor.rgb * fade, objectColor.a);
    }
})glsl";

const char* kTrailVS = R"glsl(
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in float aAlpha;
uniform mat4 view, projection;
out float vAlpha;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    vAlpha = aAlpha;
})glsl";

const char* kTrailFS = R"glsl(
#version 330 core
in float vAlpha;
out vec4 FragColor;
uniform vec4 tintColor;
void main() { FragColor = vec4(tintColor.rgb, tintColor.a * vAlpha); })glsl";

unsigned compile(unsigned type, const char* src) {
    unsigned s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "shader compile failed: " << log << "\n";
    }
    return s;
}

unsigned link(const char* vs, const char* fs) {
    unsigned p = glCreateProgram();
    unsigned v = compile(GL_VERTEX_SHADER, vs);
    unsigned f = compile(GL_FRAGMENT_SHADER, fs);
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

void setMats(unsigned prog, const Camera& cam, float aspect, const glm::mat4& model) {
    glUniformMatrix4fv(glGetUniformLocation(prog, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(prog, "view"), 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(glGetUniformLocation(prog, "projection"), 1, GL_FALSE,
                       glm::value_ptr(cam.projection(aspect)));
}

} // namespace

bool Renderer::init() {
    progBody_ = link(kBodyVS, kBodyFS);
    progTrail_ = link(kTrailVS, kTrailFS);
    if (!progBody_ || !progTrail_) return false;

    glGenVertexArrays(1, &gridVao_);
    glGenBuffers(1, &gridVbo_);
    glGenVertexArrays(1, &trailVao_);
    glGenBuffers(1, &trailVbo_);
    glBindVertexArray(trailVao_);
    glBindBuffer(GL_ARRAY_BUFFER, trailVbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
    return true;
}

void Renderer::shutdown() {
    for (auto& [id, g] : gpu_) {
        glDeleteVertexArrays(1, &g.vao);
        glDeleteBuffers(1, &g.vbo);
    }
    gpu_.clear();
    glDeleteVertexArrays(1, &gridVao_);
    glDeleteBuffers(1, &gridVbo_);
    glDeleteVertexArrays(1, &trailVao_);
    glDeleteBuffers(1, &trailVbo_);
    glDeleteProgram(progBody_);
    glDeleteProgram(progTrail_);
}

void Renderer::drawOne(const std::vector<float>& verts, const glm::vec3& posUnits,
                       const glm::vec4& color) {
    unsigned vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glUseProgram(progBody_);
    glUniform1i(glGetUniformLocation(progBody_, "isGrid"), 0);
    glUniform1i(glGetUniformLocation(progBody_, "GLOW"), 0);
    glUniform4f(glGetUniformLocation(progBody_, "objectColor"),
                color.r, color.g, color.b, color.a);
    setMats(progBody_, Camera{}, 800.0f / 600.0f, glm::translate(glm::mat4(1.0f), posUnits));
    glDrawArrays(GL_TRIANGLES, 0, GLint(verts.size() / 3));

    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
}

// Tasks 10/11 implement syncBodies/drawBodies/drawGrid (Task 10) and drawTrails (Task 11);
// the Task-1 build keeps them declared but only uses a bodyless grid draw.
void Renderer::draw(const gs::World&, const Camera& cam, int fbw, int fbh,
                    const GridConfig&) {
    glViewport(0, 0, fbw, fbh);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    (void)cam;
}

void Renderer::syncBodies(const gs::World&) {}
void Renderer::drawBodies(const gs::World&, const Camera&, float) {}
void Renderer::drawGrid(const GridConfig&, const gs::World&, const Camera&, float) {}
void Renderer::drawTrails(const gs::World&, const Camera&, float) {}

} // namespace gs::render
```

Note: `renderer.hpp` includes `render/grid.hpp` and `physics/world.hpp` — both created in this step as **stubs** so the scaffold compiles (full versions come in Tasks 4 and 10):

`src/physics/world.hpp` (stub):

```cpp
#pragma once
#include "physics/body.hpp"
#include <vector>
namespace gs {
class World {
public:
    std::vector<Body> bodies;
};
}
```

`src/render/grid.hpp` (stub):

```cpp
#pragma once
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
```

(Stub `grid.hpp` needs `physics/body.hpp` for its declaration — add `#include "physics/body.hpp"` to it.)

- [ ] **Step 7: Create `src/app/main.cpp` (scaffold: one sphere, fly camera)**

```cpp
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
```

- [ ] **Step 8: Configure and build**

Run: `cmake -S . -B build -G "Visual Studio 17 2022"` then `cmake --build build --config Debug`
Expected: all four targets build (`gravity_sim`, `gravity_sim_3Dgrid`, `3D_test`, `gravity_sim_v2`) with no errors.

- [ ] **Step 9: Run `build\Debug\gravity_sim_v2.exe`**

Expected: 800×600 window titled "Gravity Sim v2" opens; mouse-look, WASD/Space/Shift flight, `Q` quits. Legacy three targets still run as before.

- [ ] **Step 10: Ask user to approve commit, then commit**

```bash
git add CMakeLists.txt src/
git commit -m "scaffold: gravity_sim_v2 app skeleton, CMake variadic targets, CTest wiring"
```

---

### Task 2: Fixed-timestep + pause semantics (`World::advance`) with tests

**Files:**
- Modify: `src/physics/world.hpp` (replace stub)
- Create: `src/physics/world.cpp`, `tests/test_physics.cpp`
- Modify: `CMakeLists.txt` (test target)

**Interfaces:**
- Consumes: `gs::Body`, `gs::World` stub fields.
- Produces (all later tasks rely on these exact signatures):
  - `gs::WorldConfig { double softeningFrac=0.1; double softeningFloor=5.0e4; double trailSampleDt=0.033; int maxStepsPerFrame=8; double maxFrameDelta=0.25; }`
  - `static constexpr double gs::World::SIM_DT` = 1/480
  - `std::uint64_t gs::World::spawn(const Body&)` → id
  - `bool gs::World::removeById(std::uint64_t)` → removed?
  - `void gs::World::step(double dt)` — one leapfrog KDK step (forces, integrate); merges come in Task 5, trails in Task 6, softening in Task 4.
  - `int gs::World::advance(double frameDeltaSeconds)` — clamps fd ≤ 0.25, accumulates `fd × timeScale` (0 while paused), runs ≤ 8 steps of `SIM_DT`, returns steps run; 0 when paused.
  - Fields: `std::vector<Body> bodies`, `bool paused=false`, `double timeScale=1.0`, `double simTime=0.0`, `std::uint64_t nextId=1`, `WorldConfig config`

- [ ] **Step 1: Write the failing test file `tests/test_physics.cpp`**

```cpp
#include "physics/world.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } \
} while (0)
#define CHECK_NEAR(a, b, tol) do { \
    double _a = (a), _b = (b); \
    if (std::fabs(_a - _b) > (tol)) { \
        std::printf("FAIL %s:%d: %s=%.9g vs %s=%.9g\n", __FILE__, __LINE__, #a, _a, #b, _b); \
        ++g_failures; \
    } \
} while (0)

using gs::Body;
using gs::World;

static std::vector<double> snapshot(const World& w) {
    std::vector<double> s;
    for (const auto& b : w.bodies) {
        s.insert(s.end(), {b.position.x, b.position.y, b.position.z,
                           b.velocity.x, b.velocity.y, b.velocity.z,
                           b.mass, b.density});
    }
    return s;
}

static World makeOrbitWorld() {
    // Two bodies, softened-circular orbit, one period ≈ 10000 s.
    World w;
    const double m1 = 5.9e24, m2 = 1.0e20, r = 1.0e7;
    const double mtot = m1 + m2;
    Body a; a.mass = m1; a.position = {-r * m2 / mtot, 0, 0};
    Body b; b.mass = m2; b.position = { r * m1 / mtot, 0, 0};
    const double eps = std::max(0.1 * (a.radius() + b.radius()), 5.0e4);
    const double vrel = std::sqrt(gs::G * mtot * (r * r) /
                                  std::pow(r * r + eps * eps, 1.5));
    a.velocity = {0, 0, -vrel * m2 / mtot};
    b.velocity = {0, 0,  vrel * m1 / mtot};
    w.spawn(a); w.spawn(b);
    return w;
}

static void test_fps_independence() {
    World a = makeOrbitWorld(), b = makeOrbitWorld();
    int stepsA = 0, stepsB = 0;
    for (int i = 0; i < 60; ++i)  stepsA += a.advance(1.0 / 60.0);
    for (int i = 0; i < 480; ++i) stepsB += b.advance(1.0 / 480.0);
    CHECK(std::abs(stepsA - stepsB) <= 2);
    CHECK(std::abs(stepsA - 480) <= 2);
    CHECK_NEAR(a.simTime, b.simTime, 4 * World::SIM_DT);
}

static void test_step_cap_and_clamp() {
    World w;
    w.spawn(Body{}); w.spawn(Body{});
    int n = w.advance(10.0);           // huge frame: clamped to 0.25 s → capped at 8 steps
    CHECK(n == 8);
    World w2;
    w2.spawn(Body{}); w2.spawn(Body{});
    CHECK(w2.advance(5.0) == 8);
}

static void test_paused_bit_identical() {
    World w = makeOrbitWorld();
    w.paused = true;
    const auto before = snapshot(w);
    int total = 0;
    for (int i = 0; i < 100; ++i) total += w.advance(1.0 / 60.0);
    CHECK(total == 0);
    const auto after = snapshot(w);
    CHECK(before.size() == after.size());
    CHECK(before == after);
    CHECK(w.simTime == 0.0);
}

int main() {
    test_fps_independence();
    test_step_cap_and_clamp();
    test_paused_bit_identical();
    if (g_failures) { std::printf("%d failure(s)\n", g_failures); return 1; }
    std::printf("all physics tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake --build build --config Debug --target test_physics` (target does not exist yet)
Expected: FAIL — CMake error "unknown target test_physics" (no test target defined).

- [ ] **Step 3: Register the test target in `CMakeLists.txt`**

Append after the existing test wiring:

```cmake
add_executable(test_physics tests/test_physics.cpp src/physics/world.cpp)
target_include_directories(test_physics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src ${GLM_INCLUDE_DIR})
add_test(NAME physics COMMAND test_physics)
```

- [ ] **Step 4: Write the implementation — `src/physics/world.hpp`**

```cpp
#pragma once
#include "physics/body.hpp"
#include <vector>
#include <cstdint>

namespace gs {

struct WorldConfig {
    double softeningFrac  = 0.1;
    double softeningFloor = 5.0e4;   // m
    double trailSampleDt  = 0.033;   // wall-seconds between trail samples
    int    maxStepsPerFrame = 8;
    double maxFrameDelta   = 0.25;   // s
};

class World {
public:
    static constexpr double SIM_DT = 1.0 / 480.0;

    std::vector<Body> bodies;
    WorldConfig config;
    bool   paused   = false;
    double timeScale = 1.0;
    double simTime  = 0.0;
    std::uint64_t nextId = 1;

    std::uint64_t spawn(const Body& b);
    bool removeById(std::uint64_t id);
    void step(double dt);                       // one leapfrog KDK step
    std::vector<glm::dvec3> computeAccelerations() const;
    int advance(double frameDeltaSeconds);      // timestep + trail sampling; 0 when paused
    void stepOnce();                            // one step regardless of pause (single-step key)

private:
    double accum_ = 0.0;
    double trailAccum_ = 0.0;
};

} // namespace gs
```

- [ ] **Step 5: Write `src/physics/world.cpp` (this task's slice: spawn/remove/step/advance, no softening, no merges, no trail push yet)**

```cpp
#include "physics/world.hpp"
#include "physics/constants.hpp"
#include <cmath>

namespace gs {

std::uint64_t World::spawn(const Body& b) {
    Body copy = b;
    copy.id = nextId++;
    bodies.push_back(copy);
    return copy.id;
}

bool World::removeById(std::uint64_t id) {
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
        if (it->id == id) { bodies.erase(it); return true; }
    }
    return false;
}

std::vector<glm::dvec3> World::computeAccelerations() const {
    const std::size_t n = bodies.size();
    std::vector<glm::dvec3> acc(n, glm::dvec3(0.0));
    for (std::size_t i = 0; i < n; ++i) {
        if (bodies[i].ghost) continue;
        for (std::size_t j = 0; j < n; ++j) {
            if (i == j || bodies[j].ghost) continue;
            const glm::dvec3 d = bodies[j].position - bodies[i].position;
            const double r2 = glm::dot(d, d);
            const double denom = r2 * std::sqrt(r2);   // r^3 — softened in Task 4
            if (denom > 0.0) acc[i] += G * bodies[j].mass * d / denom;
        }
    }
    return acc;
}

void World::step(double dt) {
    const std::size_t n = bodies.size();
    if (n < 2) { simTime += dt; return; }

    auto acc = computeAccelerations();
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost) bodies[i].velocity += 0.5 * dt * acc[i];
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost) bodies[i].position += dt * bodies[i].velocity;

    acc = computeAccelerations();
    for (std::size_t i = 0; i < n; ++i)
        if (!bodies[i].ghost) bodies[i].velocity += 0.5 * dt * acc[i];

    simTime += dt;
}

int World::advance(double frameDeltaSeconds) {
    double fd = frameDeltaSeconds;
    if (fd > config.maxFrameDelta) fd = config.maxFrameDelta;
    if (fd < 0.0) fd = 0.0;

    const double ts = paused ? 0.0 : timeScale;
    accum_ += fd * ts;

    int n = 0;
    while (accum_ >= SIM_DT && n < config.maxStepsPerFrame) {
        step(SIM_DT);
        accum_ -= SIM_DT;
        ++n;
    }
    if (n == config.maxStepsPerFrame && accum_ >= SIM_DT) accum_ = 0.0;  // discard backlog
    return n;
}

void World::stepOnce() {
    step(SIM_DT);
}

} // namespace gs
```

- [ ] **Step 6: Build and run the test to verify it passes**

Run: `cmake --build build --config Debug --target test_physics && ctest --test-dir build -C Debug -R physics --output-on-failure`
Expected: PASS — "all physics tests passed", ctest `Passed`.

- [ ] **Step 7: Ask user to approve commit, then commit**

```bash
git add CMakeLists.txt src/physics/ tests/
git commit -m "feat: fixed-timestep World::advance with pause semantics + tests"
```

---

### Task 3: Leapfrog orbit quality (energy/momentum/period tests)

**Files:**
- Modify: `src/physics/world.cpp` (`step` — nothing structural; tests validate Task 2 math under 1000-step integration), `tests/test_physics.cpp`

**Interfaces:**
- Consumes: `World::step(double)`, `World::spawn`, `Body::radius()`.
- Produces: no API changes — this task locks in integrator correctness before softening lands.

- [ ] **Step 1: Add the failing tests to `tests/test_physics.cpp` (before `main`)**

```cpp
static void test_orbit_closes() {
    World w = makeOrbitWorld();
    const glm::dvec3 r0 = w.bodies[1].position - w.bodies[0].position;
    const double len0 = glm::length(r0);
    // period T = 2*pi*sqrt(r^3/(G*M)) ≈ 10035 s (softened) → 1000 steps of 10 s ≈ 1 period
    for (int i = 0; i < 1000; ++i) w.step(10.0);
    const glm::dvec3 r1 = w.bodies[1].position - w.bodies[0].position;
    CHECK_NEAR(glm::length(r1), len0, 0.001 * len0);          // radius within 0.1%
    const double a0 = std::atan2(r0.z, r0.x);
    const double a1 = std::atan2(r1.z, r1.x);
    double sweep = a1 - a0;
    while (sweep > 3.14159265358979) sweep -= 2 * 3.14159265358979;
    while (sweep < -3.14159265358979) sweep += 2 * 3.14159265358979;
    CHECK(std::fabs(sweep) < 0.063);                            // within 1% of full revolution
}

static void test_momentum_conserved_in_flight() {
    World w = makeOrbitWorld();
    auto p = [&] {
        glm::dvec3 t(0.0);
        for (const auto& b : w.bodies) t += b.mass * b.velocity;
        return t;
    };
    const glm::dvec3 before = p();
    for (int i = 0; i < 500; ++i) w.step(10.0);
    const glm::dvec3 after = p();
    CHECK(glm::length(after - before) <= 1e-9 * glm::length(before));
}

static void test_energy_bounded() {
    World w = makeOrbitWorld();
    auto energy = [&] {
        double ke = 0.0, pe = 0.0;
        for (const auto& b : w.bodies) ke += 0.5 * b.mass * glm::dot(b.velocity, b.velocity);
        const glm::dvec3 d = w.bodies[1].position - w.bodies[0].position;
        const double r = glm::length(d);
        const double eps = std::max(0.1 * (w.bodies[0].radius() + w.bodies[1].radius()), 5.0e4);
        pe -= gs::G * w.bodies[0].mass * w.bodies[1].mass / std::sqrt(r * r + eps * eps);
        return ke + pe;
    };
    const double e0 = energy();
    for (int i = 0; i < 1000; ++i) w.step(10.0);
    const double e1 = energy();
    CHECK(std::fabs(e1 - e0) <= 0.01 * std::fabs(e0));
}
```

Register in `main()`:

```cpp
    test_orbit_closes();
    test_momentum_conserved_in_flight();
    test_energy_bounded();
```

- [ ] **Step 2: Run the test to verify current state**

Run: `cmake --build build --config Debug --target test_physics && build\Debug\test_physics.exe`
Expected: all three new checks pass already (Task 2's leapfrog is correct) — this task is a **characterization gate**. If any fails: do not "adjust the tolerance"; re-read `World::step` against spec §4.3 (kick-drift-kick, both half-kicks, force recomputed after drift) and fix the integrator, not the test.

- [ ] **Step 3: Ask user to approve commit, then commit**

```bash
git add tests/test_physics.cpp src/physics/world.cpp
git commit -m "test: lock leapfrog orbit quality — closes, momentum, energy bounds"
```

---

### Task 4: Plummer softening (bug 4)

**Files:**
- Modify: `src/physics/world.hpp`, `src/physics/world.cpp`, `tests/test_physics.cpp`

**Interfaces:**
- Produces: `double gs::pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg)` — free function in `gs` namespace (used by tests and later by energy assertions).

- [ ] **Step 1: Write the failing test**

```cpp
static void test_softening_no_nan_at_contact() {
    World w;
    Body a; a.mass = 1e24; a.position = {0, 0, 0};
    Body b; b.mass = 1e24; b.position = {1e-6, 0, 0};   // essentially touching
    w.spawn(a); w.spawn(b);
    const auto acc = w.computeAccelerations();
    CHECK(std::isfinite(acc[0].x) && std::isfinite(acc[0].y) && std::isfinite(acc[0].z));
    CHECK(std::isfinite(acc[1].x) && std::isfinite(acc[1].y) && std::isfinite(acc[1].z));
    CHECK(glm::length(acc[0]) < 1.0e7);                  // bounded: G*m/eps^2 ≈ 2.7e4
}

static void test_pair_softening_formula() {
    gs::Body a; a.mass = 1e24;                    // rocky radius ≈ 3.7e5 m
    gs::Body b; b.mass = 1e24;
    gs::WorldConfig cfg;
    const double eps = gs::pairSoftening(a, b, cfg);
    CHECK_NEAR(eps, std::max(0.1 * (a.radius() + b.radius()), 5.0e4), 1.0);
    gs::Body tiny; tiny.mass = 1.0; tiny.density = 5515.0;
    gs::Body huge; huge.mass = 1e30; huge.density = 1408.0;
    CHECK_NEAR(gs::pairSoftening(tiny, huge, cfg), 5.0e4, 1.0);   // floor wins
}
```

Register both in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --config Debug --target test_physics && build\Debug\test_physics.exe`
Expected: FAIL — `pairSoftening` not declared; and the NaN test fails too (raw 1/r³ at r=1e-6 → Inf).

- [ ] **Step 3: Implement softening**

Add to `world.hpp` (after `WorldConfig`, before `class World`):

```cpp
double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg);
```

In `world.cpp`:

```cpp
double pairSoftening(const Body& a, const Body& b, const WorldConfig& cfg) {
    return std::max(cfg.softeningFrac * (a.radius() + b.radius()), cfg.softeningFloor);
}
```

Replace the force denominator in `computeAccelerations`:

```cpp
            const double eps = pairSoftening(bodies[i], bodies[j], config);
            const double u2 = r2 + eps * eps;
            const double denom = u2 * std::sqrt(u2);          // (r^2+eps^2)^1.5
            if (denom > 0.0) acc[i] += G * bodies[j].mass * d / denom;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --config Debug --target test_physics && ctest --test-dir build -C Debug -R physics --output-on-failure`
Expected: PASS — all physics tests green (orbit tests still pass: ε ≪ r for the orbit fixture).

- [ ] **Step 5: Ask user to approve commit, then commit**

```bash
git add src/physics/ tests/test_physics.cpp
git commit -m "fix: Plummer softening eliminates force singularity (bug 4)"
```

---

### Task 5: Momentum-conserving merges (bugs 2, 3)

**Files:**
- Modify: `src/physics/world.cpp` (`step` calls `mergeOverlaps()` after the final kick), `tests/test_physics.cpp`

**Interfaces:**
- Produces: private `void World::mergeOverlaps()` — chains until no overlaps; skips `ghost` bodies; survivor = heavier body (keeps color/glow/trail); density mass-weighted; momentum exact.

- [ ] **Step 1: Write the failing tests**

```cpp
static void test_merge_conserves_momentum() {
    World w;
    Body a; a.mass = 1e22; a.density = 3344; a.position = {0, 0, 0};
    a.velocity = {600, 0, 0}; a.color = {1, 0, 0, 1};
    Body b; b.mass = 2e22; b.density = 5515; b.position = {1e5, 0, 0};
    b.velocity = {-300, 400, 0}; b.color = {0, 1, 0, 1};
    w.spawn(a); w.spawn(b);
    const glm::dvec3 pBefore = a.mass * a.velocity + b.mass * b.velocity;
    w.step(1.0 / 480.0);
    CHECK(w.bodies.size() == 1);
    const Body& s = w.bodies[0];
    CHECK_NEAR(s.mass, 3e22, 1.0);
    const glm::dvec3 pAfter = s.mass * s.velocity;
    CHECK(glm::length(pAfter - pBefore) <= 1e-9 * glm::length(pBefore));
    CHECK(s.color.g > 0.5f);                       // heavier body's color survives
    CHECK_NEAR(s.density, (1e22 * 3344.0 + 2e22 * 5515.0) / 3e22, 1.0);
}

static void test_merge_chain_three() {
    World w;
    const double m = 1e22;
    Body a; a.mass = m; a.position = {0, 0, 0};      a.velocity = {100, 0, 0};
    Body b; b.mass = m; b.position = {1e5, 0, 0};    b.velocity = {0, 100, 0};
    Body c; c.mass = m; c.position = {0, 1e5, 0};    c.velocity = {0, 0, 100};
    w.spawn(a); w.spawn(b); w.spawn(c);
    w.step(1.0 / 480.0);
    CHECK(w.bodies.size() == 1);
    CHECK_NEAR(w.bodies[0].mass, 3.0 * m, 1.0);
    const glm::dvec3 p(100.0 * m, 100.0 * m, 100.0 * m);
    CHECK(glm::length(w.bodies[0].mass * w.bodies[0].velocity - p)
          <= 1e-9 * glm::length(p));
}

static void test_ghost_never_merges_or_attracts() {
    World w;
    Body star; star.mass = 1e30; star.position = {0, 0, 0};
    Body ghost; ghost.mass = 1e24; ghost.position = {0, 0, 0}; ghost.ghost = true;
    Body planet; planet.mass = 1e24; planet.position = {1e9, 0, 0};
    w.spawn(star); w.spawn(ghost); w.spawn(planet);
    auto acc = w.computeAccelerations();
    CHECK(glm::length(acc[2]) > 0.0);              // planet feels the star
    const double withGhost = glm::length(acc[2]);
    w.removeById(ghost.id = w.bodies[1].id);
    acc = w.computeAccelerations();
    CHECK_NEAR(glm::length(acc[1]), withGhost, 1e-6 * withGhost);  // ghost contributed nothing
}
```

Register all three in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --config Debug --target test_physics && build\Debug\test_physics.exe`
Expected: FAIL — `test_merge_conserves_momentum`: `w.bodies.size() == 1` fails (bodies never merge).

- [ ] **Step 3: Implement merging**

Add to `world.hpp` under `private:`:

```cpp
    void mergeOverlaps();
```

At the end of `World::step` (before `simTime += dt`), insert:

```cpp
    mergeOverlaps();
```

Append to `world.cpp`:

```cpp
void World::mergeOverlaps() {
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t i = 0; i < bodies.size() && !changed; ++i) {
            if (bodies[i].ghost) continue;
            for (std::size_t j = i + 1; j < bodies.size(); ++j) {
                if (bodies[j].ghost) continue;
                Body& A = bodies[i];
                Body& B = bodies[j];
                if (glm::length(B.position - A.position) >= A.radius() + B.radius())
                    continue;
                const bool aHeavier = A.mass >= B.mass;
                Body& heavy = aHeavier ? A : B;
                Body& light = aHeavier ? B : A;
                const double m1 = heavy.mass, m2 = light.mass, m = m1 + m2;
                heavy.velocity = (m1 * heavy.velocity + m2 * light.velocity) / m;
                heavy.position = (m1 * heavy.position + m2 * light.position) / m;
                heavy.density  = (m1 * heavy.density  + m2 * light.density)  / m;
                heavy.mass = m;
                const std::size_t eraseIdx = aHeavier ? j : i;
                bodies.erase(bodies.begin() + std::ptrdiff_t(eraseIdx));
                changed = true;
                break;
            }
        }
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --config Debug --target test_physics && ctest --test-dir build -C Debug -R physics --output-on-failure`
Expected: PASS — all tests green.

- [ ] **Step 5: Ask user to approve commit, then commit**

```bash
git add src/physics/ tests/test_physics.cpp
git commit -m "feat: momentum-conserving chained merges replace velocity flips (bugs 2,3)"
```

---

### Task 6: Trail ring buffer + wall-clock sampling (feature #5, data side)

**Files:**
- Modify: `src/physics/world.cpp` (`advance` and `stepOnce` push trails), `tests/test_physics.cpp`

**Interfaces:**
- Consumes: `gs::TrailRing` (already in `body.hpp`), `config.trailSampleDt`, `config.trailCap` — add `std::size_t trailCap = 512;` to `WorldConfig`.
- Produces: after `advance()` with steps > 0 and ≥ 0.033 wall-seconds elapsed, every non-ghost body has a trail sample; `stepOnce()` always samples once. New: `void World::setTrailCap(std::size_t cap)` applies to all bodies and future spawns (field `trailCap`).

- [ ] **Step 1: Write the failing tests**

```cpp
static void test_trail_ring_eviction() {
    gs::TrailRing t;
    t.cap = 3;
    for (int i = 0; i < 6; ++i) t.push({double(i), 0, 0});
    CHECK(t.pts.size() == 3);
    CHECK(t.pts.front().x == 3.0);                 // oldest survived values: 3,4,5
    CHECK(t.pts.back().x == 5.0);
    t.setCap(1);
    CHECK(t.pts.size() == 1);
    CHECK(t.pts.front().x == 5.0);
}

static void test_trail_sampling_requires_steps_and_wall_time() {
    World w;
    Body a; a.mass = 1e24; a.position = {0, 0, 0};
    Body b; b.mass = 1e24; b.position = {1e9, 0, 0};
    w.spawn(a); w.spawn(b);

    for (int i = 0; i < 5; ++i) w.advance(0.01);   // steps run, but < 0.033 wall-s total
    CHECK(w.bodies[0].trail.pts.empty());

    double wall = 0.0;
    while (w.bodies[0].trail.pts.empty()) { wall += 0.01; CHECK(wall < 1.0); w.advance(0.01); }
    CHECK(w.bodies[1].trail.pts.size() == w.bodies[0].trail.pts.size());
    CHECK(w.bodies[0].trail.pts.back() == w.bodies[0].position);

    w.paused = true;
    const std::size_t n = w.bodies[0].trail.pts.size();
    for (int i = 0; i < 50; ++i) w.advance(0.02);
    CHECK(w.bodies[0].trail.pts.size() == n);       // paused → no samples
}

static void test_step_once_samples_trail() {
    World w;
    Body a; a.mass = 1e24; Body b; b.mass = 1e24; b.position = {1e9, 0, 0};
    w.spawn(a); w.spawn(b);
    w.paused = true;
    w.stepOnce();
    CHECK(w.bodies[0].trail.pts.size() == 1);
}
```

Register all three in `main()`.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --config Debug --target test_physics && build\Debug\test_physics.exe`
Expected: FAIL — `trail.pts` never fills (no sampling implemented); `trailCap` not a member.

- [ ] **Step 3: Implement sampling**

`world.hpp`: add to `WorldConfig`: `std::size_t trailCap = 512;` and to `World` public section:

```cpp
    void setTrailCap(std::size_t cap);
```

`world.cpp`:

```cpp
void World::setTrailCap(std::size_t cap) {
    config.trailCap = cap;
    for (auto& b : bodies) b.trail.setCap(cap);
}
```

In `spawn`, after pushing the body: `bodies.back().trail.setCap(config.trailCap);`

At the end of `advance`, after the step loop:

```cpp
    if (n > 0) {
        trailAccum_ += frameDeltaSeconds;   // raw wall delta, before clamping — pass it through
        if (trailAccum_ >= config.trailSampleDt) {
            for (auto& b : bodies)
                if (!b.ghost) b.trail.push(b.position);
            trailAccum_ = 0.0;
        }
    }
```

To have the raw value available, rename the parameter at the top of `advance` usage: keep `const double rawFd = frameDeltaSeconds;` before clamping and use `rawFd` in the trail block.

In `stepOnce`, after `step(SIM_DT)`:

```cpp
    for (auto& b : bodies)
        if (!b.ghost) b.trail.push(b.position);
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cmake --build build --config Debug --target test_physics && ctest --test-dir build -C Debug -R physics --output-on-failure`
Expected: PASS.

- [ ] **Step 5: Ask user to approve commit, then commit**

```bash
git add src/physics/ tests/test_physics.cpp
git commit -m "feat: trail ring buffer with wall-clock sampling (feature #5 data side)"
```

---

### Task 7: Radius lifecycle test + fixed spawn scene (bug 5)

**Files:**
- Modify: `tests/test_physics.cpp`, `src/app/main.cpp` (real initial scene replaces placeholder)

**Interfaces:**
- Consumes: everything from Tasks 2–6.
- Produces: initial scene convention — star at origin (`mass 1.989e30`, `density 1408`, `glow=true`, yellow) + two planets on the x-axis at `±5e10 m` with circular velocity `v = √(G·M/r)` along ∓z... precisely: planet at `+x` has velocity `(0,0,+v)`, planet at `−x` has `(0,0,−v)` (same rotational sense).

- [ ] **Step 1: Write the failing radius test**

```cpp
static void test_radius_constant_through_lifecycle() {
    World w;
    Body a; a.mass = 1e22; a.density = 3344; a.position = {0, 0, 0};
    Body b; b.mass = 1e22; b.density = 3344; b.position = {1e12, 0, 0};
    const double r0 = a.radius();
    w.spawn(a);
    w.bodies[0].mass *= 8.0;                       // RMB growth
    CHECK_NEAR(w.bodies[0].radius(), r0 * 2.0, 1e-6 * r0);   // cube-root law
    w.bodies[0].mass /= 8.0;
    CHECK_NEAR(w.bodies[0].radius(), r0, 1e-9 * r0);         // unchanged by history
    for (int i = 0; i < 100; ++i) w.step(World::SIM_DT);     // motion never alters radius
    CHECK_NEAR(w.bodies[0].radius(), r0, 1e-9 * r0);
    (void)b;
}
```

Register in `main()`.

- [ ] **Step 2: Run test to verify it passes immediately**

Run: `cmake --build build --config Debug --target test_physics && build\Debug\test_physics.exe`
Expected: PASS — `radius()` is a pure function of `(mass, density)`; this is the **regression guard for bug 5** (the old code's `/1e6` vs `/30000` split cannot reappear inside physics). If it fails, `radius()` has acquired external state — fix `body.hpp`, not the test.

- [ ] **Step 3: Replace the scaffold scene in `src/app/main.cpp`**

Replace the empty loop body's scene setup with a `gs::World` built before the loop:

```cpp
    gs::World world;
    {
        gs::Body star;
        star.mass = 1.989e30; star.density = 1408.0;
        star.color = {1.0f, 0.929f, 0.176f, 1.0f}; star.glow = true;
        world.spawn(star);

        const double r = 5.0e10;                      // 5000 world-units
        const double v = std::sqrt(gs::G * star.mass / r);
        gs::Body p1; p1.mass = 5.97e24; p1.density = 5515.0;
        p1.color = {0.0f, 1.0f, 1.0f, 1.0f};
        p1.position = {r, 0, 0}; p1.velocity = {0, 0, v};
        world.spawn(p1);
        gs::Body p2 = p1;
        p2.position = {-r, 0, 0}; p2.velocity = {0, 0, -v};
        world.spawn(p2);
    }
    double last = glfwGetTime();                      // first-frame guard (bug 13)
```

In the loop, advance the world and draw it (replacing the empty `renderer.draw(gs::World{}, ...)`):

```cpp
        world.advance(float(now - last));
        renderer.draw(world, cam, fbw, fbh, gridCfg);
```

with `gs::render::GridConfig gridCfg;` declared before the loop, and add `#include <cmath>`.

- [ ] **Step 4: Build, run, and manually verify orbital motion**

Run: `cmake --build build --config Debug --target gravity_sim_v2` then `build\Debug\gravity_sim_v2.exe`
Expected: two cyan planets circle the glowing yellow star on stable paths for ≥ 60 s of watching; no drift into the star, no flinging off screen. **This is the headline demo of the physics core.**

- [ ] **Step 5: Ask user to approve commit, then commit**

```bash
git add tests/test_physics.cpp src/app/main.cpp
git commit -m "feat: SI-unit initial scene with stable orbits; radius lifecycle guard (bug 5)"
```

---

### Task 8: Pure sphere mesh tests (bug 11) — test_render target

**Files:**
- Create: `tests/test_render.cpp`
- Modify: `CMakeLists.txt`
- Modify: `src/render/mesh.cpp` only if a test fails (expected pass from Task 1 design)

**Interfaces:**
- Consumes: `gs::render::sphereVertices`.
- Produces: CTest target `render`.

- [ ] **Step 1: Write `tests/test_render.cpp`**

```cpp
#include "render/mesh.hpp"
#include <cmath>
#include <cstdio>

static int g_failures = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++g_failures; } \
} while (0)

int main() {
    const int stacks = 16, sectors = 24;
    const double r = 3.5;
    const auto v = gs::render::sphereVertices(r, stacks, sectors);

    CHECK(v.size() == std::size_t(stacks) * sectors * 6 * 3);   // exact count, no extra ring
    for (float x : v) CHECK(x >= -float(r) - 1e-5f && x <= float(r) + 1e-5f);

    // no degenerate "past the pole" ring: min y must reach -r (south pole covered exactly once)
    float minY = 1e30f, maxY = -1e30f;
    for (std::size_t i = 1; i < v.size(); i += 3) {
        minY = std::min(minY, v[i]);
        maxY = std::max(maxY, v[i]);
    }
    CHECK(minY <= -float(r) + 1e-4f && minY >= -float(r) - 1e-4f);
    CHECK(maxY <= float(r) + 1e-4f && maxY >= float(r) - 1e-4f);

    if (g_failures) { std::printf("%d failure(s)\n", g_failures); return 1; }
    std::printf("all render tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Register the target — append to `CMakeLists.txt`**

```cmake
add_executable(test_render tests/test_render.cpp src/render/mesh.cpp)
target_include_directories(test_render PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src ${GLM_INCLUDE_DIR})
add_test(NAME render COMMAND test_render)
```

- [ ] **Step 3: Run test to verify it passes (regression gate for bug 11)**

Run: `cmake --build build --config Debug --target test_render && ctest --test-dir build -C Debug -R render --output-on-failure`
Expected: PASS. If it fails: the loop bound regressed to `i <= stacks` — fix `mesh.cpp` (root cause: off-by-one produces an extra inverted ring past the south pole).

- [ ] **Step 4: Ask user to approve commit, then commit**

```bash
git add tests/test_render.cpp CMakeLists.txt src/render/mesh.cpp
git commit -m "test: sphere mesh exact bounds — guards off-by-one ring (bug 11)"
```

---

### Task 9: Pure grid displacement (bug 6) — base template + dip math

**Files:**
- Modify: `src/render/grid.hpp` (replace stub), Create: `src/render/grid.cpp`, `tests/test_render.cpp`, `CMakeLists.txt` (grid.cpp into test_render target)

**Interfaces:**
- Produces:
  - `std::vector<float> gs::render::buildGridBase(const GridConfig&)` — wireframe lines on the plane `y = planeYFactor × sizeUnits`, world-units, **never mutated afterwards**
  - `std::vector<float> gs::render::displaceGrid(const std::vector<float>& base, const GridConfig&, const std::vector<gs::Body>& bodies)` — pure; `Flat`/`Off` ⇒ returns `base` unchanged; `Bend` ⇒ per-vertex `dip_m = Σ 2√(rs(d−rs))` over bodies with `d > rs`, clamped to `0.25 × sizeUnits × UNIT`, output `y = base.y − dip_m / UNIT`

- [ ] **Step 1: Write the failing tests (append to `tests/test_render.cpp` before `main`, call them from `main`)**

```cpp
#include "physics/body.hpp"
#include "physics/constants.hpp"
#include "render/grid.hpp"
#include <vector>

static void test_grid_base_stable_and_pure() {
    gs::render::GridConfig cfg;                       // size 20000, div 25, y=-600
    const auto base = gs::render::buildGridBase(cfg);
    CHECK(!base.empty());
    CHECK(base.size() % 6 == 0);                      // line segments: pairs of xyz
    for (std::size_t i = 0; i < base.size(); i += 3)
        CHECK_NEAR(base[i + 1], float(cfg.planeYFactor * cfg.sizeUnits), 1e-3f);
    for (int i = 0; i < 100; ++i) {
        const auto again = gs::render::buildGridBase(cfg);
        CHECK(again == base);                          // deterministic, never mutated
    }
}

static void test_grid_bodyless_displacement_identity() {
    gs::render::GridConfig cfg;
    const auto base = gs::render::buildGridBase(cfg);
    std::vector<gs::Body> none;
    for (int i = 0; i < 100; ++i) {
        const auto out = gs::render::displaceGrid(base, cfg, none);
        CHECK(out == base);                            // bug 6 regression: rest level never drifts
    }
}

static void test_grid_flat_mode_identity_and_bend_dips_down() {
    gs::render::GridConfig cfg;
    cfg.mode = gs::render::GridMode::Flat;
    const auto base = gs::render::buildGridBase(cfg);
    gs::Body heavy; heavy.mass = 1.989e30; heavy.density = 1408.0;
    heavy.position = {0, 0, 0};
    CHECK(gs::render::displaceGrid(base, cfg, {heavy}) == base);

    cfg.mode = gs::render::GridMode::Bend;
    const auto bent = gs::render::displaceGrid(base, cfg, {heavy});
    CHECK(bent.size() == base.size());
    double maxDip = 0.0;
    for (std::size_t i = 0; i < bent.size(); i += 3) {
        const double dy = double(base[i + 1]) - double(bent[i + 1]);
        CHECK(dy >= -1e-6);                            // never displaced upward
        maxDip = std::max(maxDip, dy);
    }
    CHECK(maxDip > 0.0);
    CHECK(maxDip <= 0.25 * cfg.sizeUnits + 1e-6);      // clamp respected
}
```

(`test_render.cpp` needs `#include <algorithm>` and the `CHECK_NEAR` macro — copy the macro block from `test_physics.cpp`.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build --config Debug --target test_render && build\Debug\test_render.exe`
Expected: FAIL — link error: `buildGridBase`/`displaceGrid` undefined (stub header only).

- [ ] **Step 3: Implement `src/render/grid.cpp`**

```cpp
#include "render/grid.hpp"
#include "physics/body.hpp"
#include "physics/constants.hpp"
#include <cmath>
#include <algorithm>

namespace gs::render {

std::vector<float> buildGridBase(const GridConfig& cfg) {
    std::vector<float> v;
    if (cfg.mode == GridMode::Off) return v;
    const float step = float(cfg.sizeUnits / cfg.divisions);
    const float half = float(cfg.sizeUnits / 2.0);
    const float y = float(cfg.planeYFactor * cfg.sizeUnits);
    for (int zs = 0; zs <= cfg.divisions; ++zs) {           // lines running along X
        const float z = -half + zs * step;
        for (int xs = 0; xs < cfg.divisions; ++xs) {
            const float x0 = -half + xs * step;
            v.insert(v.end(), {x0, y, z, x0 + step, y, z});
        }
    }
    for (int xs = 0; xs <= cfg.divisions; ++xs) {           // lines running along Z
        const float x = -half + xs * step;
        for (int zs = 0; zs < cfg.divisions; ++zs) {
            const float z0 = -half + zs * step;
            v.insert(v.end(), {x, y, z0, x, y, z0 + step});
        }
    }
    return v;
}

std::vector<float> displaceGrid(const std::vector<float>& base, const GridConfig& cfg,
                                const std::vector<gs::Body>& bodies) {
    if (cfg.mode != GridMode::Bend || bodies.empty()) return base;
    std::vector<float> out = base;
    const double maxDipUnits = 0.25 * cfg.sizeUnits;
    for (std::size_t i = 0; i + 2 < out.size(); i += 3) {
        const glm::dvec3 vm(double(out[i]), double(out[i + 1]), double(out[i + 2]));
        const glm::dvec3 vmM = vm * UNIT;                    // world-units → meters
        double dipM = 0.0;
        for (const auto& b : bodies) {
            if (b.ghost) continue;
            const double d = glm::length(b.position - vmM);
            const double rs = 2.0 * gs::G * b.mass / (gs::C * gs::C);
            if (d > rs) dipM += 2.0 * std::sqrt(rs * (d - rs));
        }
        const double dipUnits = std::min(dipM / UNIT, maxDipUnits);
        out[i + 1] = float(double(base[i + 1]) - dipUnits);
    }
    return out;
}

} // namespace gs::render
```

Add `#include "physics/body.hpp"` to `grid.hpp` if not already present (the declaration uses `gs::Body`).

- [ ] **Step 4: Register grid.cpp in the render test — update `CMakeLists.txt`**

```cmake
add_executable(test_render tests/test_render.cpp src/render/mesh.cpp src/render/grid.cpp)
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build --config Debug --target test_render && ctest --test-dir build -C Debug --output-on-failure`
Expected: PASS — both `physics` and `render` suites green.

- [ ] **Step 6: Ask user to approve commit, then commit**

```bash
git add src/render/ tests/test_render.cpp CMakeLists.txt
git commit -m "fix: immutable base grid + pure displacement — kills grid drift (bug 6)"
```

---

### Task 10: Full renderer — bodies, bending grid, resize (bugs 10, part of 15)

**Files:**
- Modify: `src/render/renderer.cpp` (replace placeholder `draw`/`syncBodies`/`drawBodies`/`drawGrid`), `src/render/renderer.hpp` (drop `drawOne`), `src/app/main.cpp` (use real grid config + remove scaffold leftovers)

**Interfaces:**
- Consumes: `buildGridBase`/`displaceGrid` (Task 9), `sphereVertices` (Task 1), `Body::radius()/color/glow/ghost/id`, `UNIT`.
- Produces: `Renderer::draw` contract — viewport follows `fbw/fbh` every frame (resize-safe), grid drawn first (with `glPolygonOffset`), then trails placeholder (Task 12), then bodies with `displayRadius = max(radius(), 3e6) / UNIT`.

- [ ] **Step 1: Replace the placeholder implementations in `renderer.cpp`**

Add includes at top: `"physics/constants.hpp"` (already present).

```cpp
void Renderer::draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
                    const GridConfig& grid) {
    glViewport(0, 0, fbw, fbh);                          // every frame → resize-safe (bug 10)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const float aspect = fbh > 0 ? float(fbw) / float(fbh) : 1.0f;

    syncBodies(world);
    drawGrid(grid, world, cam, aspect);
    drawTrails(world, cam, aspect);
    drawBodies(world, cam, aspect);
}

void Renderer::syncBodies(const gs::World& world) {
    // delete GPU bodies that no longer exist
    for (auto it = gpu_.begin(); it != gpu_.end();) {
        bool found = false;
        for (const auto& b : world.bodies) if (b.id == it->first) { found = true; break; }
        if (found) ++it;
        else { glDeleteVertexArrays(1, &it->second.vao);
               glDeleteBuffers(1, &it->second.vbo); it = gpu_.erase(it); }
    }
    // create / refresh
    for (const auto& b : world.bodies) {
        auto it = gpu_.find(b.id);
        if (it == gpu_.end()) {
            const double disp = std::max(b.radius(), 3.0e6);      // MIN_VISUAL (spec §4.5)
            const auto verts = sphereVertices(disp / UNIT);
            GpuBody g;
            glGenVertexArrays(1, &g.vao);
            glGenBuffers(1, &g.vbo);
            glBindVertexArray(g.vao);
            glBindBuffer(GL_ARRAY_BUFFER, g.vbo);
            glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(),
                         GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
            g.radius = b.radius();
            g.count = int(verts.size() / 3);
            gpu_.emplace(b.id, g);
        } else if (std::fabs(b.radius() - it->second.radius) >
                   0.005 * std::max(it->second.radius, 1.0)) {
            const double disp = std::max(b.radius(), 3.0e6);
            const auto verts = sphereVertices(disp / UNIT);
            glBindBuffer(GL_ARRAY_BUFFER, it->second.vbo);
            glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(),
                         GL_STATIC_DRAW);
            it->second.radius = b.radius();
            it->second.count = int(verts.size() / 3);
        }
    }
}

void Renderer::drawBodies(const gs::World& world, const Camera& cam, float aspect) {
    glUseProgram(progBody_);
    glUniform1i(glGetUniformLocation(progBody_, "isGrid"), 0);
    for (const auto& b : world.bodies) {
        auto it = gpu_.find(b.id);
        if (it == gpu_.end()) continue;
        glUniform1i(glGetUniformLocation(progBody_, "GLOW"), b.glow ? 1 : 0);
        glUniform4f(glGetUniformLocation(progBody_, "objectColor"),
                    b.color.r, b.color.g, b.color.b, b.color.a);
        const glm::vec3 units = glm::vec3(b.position) / float(UNIT);
        setMats(progBody_, cam, aspect, glm::translate(glm::mat4(1.0f), units));
        glBindVertexArray(it->second.vao);
        glDrawArrays(GL_TRIANGLES, 0, it->second.count);
    }
}

void Renderer::drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam,
                        float aspect) {
    if (grid.mode == GridMode::Off) return;
    const auto base = buildGridBase(grid);
    const auto displaced = displaceGrid(base, grid, world.bodies);

    glUseProgram(progBody_);
    glUniform1i(glGetUniformLocation(progBody_, "isGrid"), 1);
    glUniform1i(glGetUniformLocation(progBody_, "GLOW"), 0);
    glUniform4f(glGetUniformLocation(progBody_, "objectColor"), 1.f, 1.f, 1.f, 0.25f);
    setMats(progBody_, cam, aspect, glm::mat4(1.0f));

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);                        // keeps grid under bodies
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, displaced.size() * sizeof(float), displaced.data(),
                 GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, GLsizei(displaced.size() / 3));
    glBindVertexArray(0);
    glDisable(GL_POLYGON_OFFSET_FILL);
}
```

`drawTrails` stays the empty stub for now (Task 11 fills it).

- [ ] **Step 2: Delete `drawOne` from `renderer.hpp`/`renderer.cpp` and its uses**

- [ ] **Step 3: Build and run — verify grid, resize, lighting**

Run: `cmake --build build --config Debug && build\Debug\gravity_sim_v2.exe`
Expected: bending wireframe grid below the scene, visibly dipping under the star; planets orbit above it; **resize the window** — rendering fills the window with correct aspect at all sizes (bug 10); grid stays put over 60+ s of watching (bug 6 fixed at render level).

- [ ] **Step 4: Run the full test suite**

Run: `ctest --test-dir build -C Debug --output-on-failure`
Expected: all tests PASS.

- [ ] **Step 5: Ask user to approve commit, then commit**

```bash
git add src/render/ src/app/main.cpp
git commit -m "feat: full renderer — bodies, bending grid with polygon offset, resize-safe viewport"
```

---

### Task 11: Trail rendering (feature #5, render side)

**Files:**
- Modify: `src/render/renderer.cpp` (`drawTrails`), `src/app/main.cpp` (`[` `]` handled in Task 13 — nothing here)

**Interfaces:**
- Consumes: `Body::trail.pts` (deque of meters), `progTrail_`, `trailVao_/trailVbo_` (already created in `init`).
- Produces: none new.

- [ ] **Step 1: Implement `drawTrails` in `renderer.cpp`**

```cpp
void Renderer::drawTrails(const gs::World& world, const Camera& cam, float aspect) {
    glUseProgram(progTrail_);
    glUniformMatrix4fv(glGetUniformLocation(progTrail_, "view"), 1, GL_FALSE,
                       glm::value_ptr(cam.view()));
    glUniformMatrix4fv(glGetUniformLocation(progTrail_, "projection"), 1, GL_FALSE,
                       glm::value_ptr(cam.projection(aspect)));
    glBindVertexArray(trailVao_);
    glBindBuffer(GL_ARRAY_BUFFER, trailVbo_);

    for (const auto& b : world.bodies) {
        const auto& pts = b.trail.pts;
        if (pts.size() < 2) continue;
        scratch_.clear();                                 // reused → no per-frame allocation
        scratch_.reserve(pts.size() * 4);
        const float n = float(pts.size() - 1);
        std::size_t i = 0;
        for (const auto& p : pts) {
            const glm::vec3 u = glm::vec3(p) / float(UNIT);
            const float a = 0.65f * (float(i) / n);       // faint tail → brighter head
            scratch_.insert(scratch_.end(), {u.x, u.y, u.z, a});
            ++i;
        }
        glUniform4f(glGetUniformLocation(progTrail_, "tintColor"),
                    b.color.r, b.color.g, b.color.b, b.color.a);
        glBufferData(GL_ARRAY_BUFFER, scratch_.size() * sizeof(float), nullptr,
                     GL_STREAM_DRAW);                     // orphan
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        scratch_.size() * sizeof(float), scratch_.data());
        glDrawArrays(GL_LINE_STRIP, 0, GLsizei(pts.size()));
    }
    glBindVertexArray(0);
}
```

- [ ] **Step 2: Build and run — verify trails**

Run: `cmake --build build --config Debug --target gravity_sim_v2 && build\Debug\gravity_sim_v2.exe`
Expected: fading colored tails follow both planets; tails lengthen over ~17 s and then hold steady (512-sample cap); the glowing star's trail stays tiny (it barely moves).

- [ ] **Step 3: Ask user to approve commit, then commit**

```bash
git add src/render/renderer.cpp
git commit -m "feat: fading trail rendering with orphaned stream VBOs (feature #5)"
```

---

### Task 12: Input state machine — placement, camera polling, pause toggle (bugs 7, 8, 9, 12; controls per spec §7)

**Files:**
- Create: `src/app/input.hpp`, `src/app/input.cpp`
- Modify: `src/app/main.cpp` (callbacks forward to `Input`, polled update each frame), `CMakeLists.txt` (add `src/app/input.cpp` to `gravity_sim_v2`)

**Interfaces:**
- Consumes: `World::{spawn,removeById,paused,timeScale,advance,stepOnce,setTrailCap}`, `Camera`, `UNIT`, `Body`.
- Produces:

```cpp
namespace gs::app {
struct InputContext {
    gs::World* world = nullptr;
    gs::render::Camera* cam = nullptr;
    double frameDelta = 0.0;    // raw wall seconds this frame
    int fbw = 800, fbh = 600;
};
class Input {
public:
    void onKey(int key, int action, int mods);
    void onMouseButton(int button, int action);
    void onCursor(double x, double y);
    void onScroll(double yoffset);
    void update(GLFWwindow* win, InputContext& ctx);   // polled every frame
    bool quitRequested = false;
private:
    enum class Mode { Idle, Placing };
    Mode mode_ = Mode::Idle;
    std::uint64_t placingId_ = 0;
    bool rmbHeld_ = false;
    float lastX_ = 400.f, lastY_ = 300.f;
    bool firstMouse_ = true;
    int trailCapIdx_ = 4;                               // 512
    gs::World* world_ = nullptr;
    gs::render::Camera* cam_ = nullptr;
    static constexpr std::size_t kTrailCaps[6] = {64, 128, 256, 512, 1024, 2048};
    void startPlacing();
    void finishPlacing(bool cancel);
};
}
```

- [ ] **Step 1: Create `src/app/input.hpp` exactly as the Interfaces block above**

- [ ] **Step 2: Create `src/app/input.cpp` — mouse, placement, scroll**

```cpp
#include "app/input.hpp"
#include "physics/constants.hpp"
#include <GLFW/glfw3.h>
#include <cmath>
#include <algorithm>

namespace gs::app {

void Input::update(GLFWwindow* win, InputContext& ctx) {
    world_ = ctx.world;
    cam_ = ctx.cam;

    // mouse-look (polling cursor callback happens via onCursor; this polls keys only)
    const float speed = 5000.0f * float(ctx.frameDelta);
    if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) cam_->pos += speed * cam_->front();
    if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) cam_->pos -= speed * cam_->front();
    if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) cam_->pos -= speed * cam_->right();
    if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) cam_->pos += speed * cam_->right();
    if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS) cam_->pos += speed * cam_->up();
    if (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) cam_->pos -= speed * cam_->up();

    // RMB mass growth while placing: *e^2 per second held (single growth path)
    if (mode_ == Mode::Placing && rmbHeld_) {
        for (auto& b : world_->bodies)
            if (b.id == placingId_) b.mass *= std::exp(2.0 * ctx.frameDelta);
    }
    // note: frameDelta here is raw wall time; growth is timeScale-independent
}

void Input::onCursor(double x, double y) {
    if (firstMouse_) { lastX_ = float(x); lastY_ = float(y); firstMouse_ = false; }
    cam_->rotate(float(x - lastX_) * 0.1f, float(lastY_ - y) * 0.1f);
    lastX_ = float(x); lastY_ = float(y);
}

void Input::startPlacing() {
    gs::Body b;
    b.mass = 1e22;
    b.density = 3344.0;
    b.color = {0.9f, 0.4f, 0.2f, 1.0f};
    b.ghost = true;
    b.trail.setCap(kTrailCaps[trailCapIdx_]);
    const float d0 = std::clamp(glm::length(cam_->pos), 100.0f, 50000.0f);
    b.position = glm::dvec3((cam_->pos + cam_->front() * d0) * double(gs::UNIT));
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

void Input::onMouseButton(int button, int action) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS && mode_ == Mode::Idle) startPlacing();
        else if (action == GLFW_RELEASE && mode_ == Mode::Placing) finishPlacing(false);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
        rmbHeld_ = (action == GLFW_PRESS);
}

void Input::onScroll(double yoffset) {
    if (mode_ == Mode::Placing) {
        // wheel moves the body along the view ray (bug 12)
        for (auto& b : world_->bodies) {
            if (b.id != placingId_) continue;
            const glm::vec3 units = glm::vec3(b.position) / float(gs::UNIT);
            const float d = glm::length(units - cam_->pos);
            const glm::vec3 moved = units + cam_->front() * float(yoffset) * 0.1f * d;
            b.position = glm::dvec3(moved) * double(gs::UNIT);
        }
    } else {
        // dolly proportional to distance from origin
        const float d = std::max(glm::length(cam_->pos), 500.0f);
        cam_->pos += cam_->front() * float(yoffset > 0 ? 1.0 : -1.0) * 0.2f * d;
    }
}

void Input::onKey(int key, int action, int) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    if (key == GLFW_KEY_Q) quitRequested = true;
    if (key == GLFW_KEY_P && action == GLFW_PRESS) world_->paused = !world_->paused;
    if (key == GLFW_KEY_PERIOD && world_->paused && action == GLFW_PRESS)
        world_->stepOnce();                                   // single-step

    if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_PRESS && trailCapIdx_ > 0) {
        --trailCapIdx_;
        world_->setTrailCap(kTrailCaps[trailCapIdx_]);
    }
    if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_PRESS && trailCapIdx_ < 5) {
        ++trailCapIdx_;
        world_->setTrailCap(kTrailCaps[trailCapIdx_]);
    }

    if (mode_ == Mode::Placing) {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) { finishPlacing(true); return; }
        for (auto& b : world_->bodies) {
            if (b.id != placingId_) continue;
            const glm::vec3 units = glm::vec3(b.position) / float(gs::UNIT);
            const float d = glm::length(units - cam_->pos);
            const float stepUnits = 0.05f * d;                 // 5% of view distance (bug 12)
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

} // namespace gs::app
```

Add `#include <algorithm>` for `std::clamp`.

- [ ] **Step 3: Rewire `main.cpp`**

Replace the raw callbacks/keys with: GLFW user pointer holding a `struct App { gs::World world; gs::render::Camera cam; gs::app::Input input; }*`:

```cpp
namespace {
struct App {
    gs::World world;
    gs::render::Camera cam;
    gs::app::Input input;
};
App* g_app = nullptr;

void keyCb(GLFWwindow* w, int key, int, int action, int mods) {
    g_app->input.onKey(key, action, mods);
    (void)w;
}
void mouseBtnCb(GLFWwindow*, int button, int action, int) {
    g_app->input.onMouseButton(button, action);
}
void cursorCb(GLFWwindow*, double x, double y) { g_app->input.onCursor(x, y); }
void scrollCb(GLFWwindow*, double, double y) { g_app->input.onScroll(y); }
} // namespace
```

Wire after `Renderer::init`:

```cpp
    App app; g_app = &app;
    glfwSetWindowUserPointer(win, &app);
    glfwSetKeyCallback(win, keyCb);
    glfwSetMouseButtonCallback(win, mouseBtnCb);
    glfwSetCursorPosCallback(win, cursorCb);
    glfwSetScrollCallback(win, scrollCb);
    glfwSetInputMode(win, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
```

Loop body:

```cpp
        const double now = glfwGetTime();
        const double fd = now - last;
        last = now;

        int fbw, fbh;
        glfwGetFramebufferSize(win, &fbw, &fbh);

        gs::app::InputContext ctx{&app.world, &app.cam, fd, fbw, fbh};
        app.input.update(win, ctx);
        if (app.input.quitRequested) glfwSetWindowShouldClose(win, GLFW_TRUE);

        app.world.advance(fd);
        renderer.draw(app.world, app.cam, fbw, fbh, gridCfg);
        glfwSwapBuffers(win);
        glfwPollEvents();
```

Move the Task-7 scene construction into `app.world`. Delete the old `g_cam`/`cursorCb` scaffold globals. Update `CMakeLists.txt`: add `src/app/input.cpp` to the `gravity_sim_v2` source list.

- [ ] **Step 4: Build and run the full manual checklist (bugs 7, 8, 9, 12)**

Run: `cmake --build build --config Debug && build\Debug\gravity_sim_v2.exe`

Manual checklist (all must pass):
- [ ] WASD flight is smooth and continuous while keys are held (no stutter/pause) — bug 7
- [ ] Left Shift only moves the camera down; never displaces a placing body — bug 8
- [ ] `P` toggles pause; app starts unpaused; paused universe is completely frozen (orbits stop, no velocity weirdness) — bug 9
- [ ] LMB press spawns a body; wheel visibly moves it nearer/farther; arrows move it clearly (~5% view distance per press); RMB-hold visibly grows it (watch HUD-less: it gets bigger); Esc cancels; release freezes it in place — bugs 12, placement redesign
- [ ] `Space`/`Shift` still camera up/down; `[`/`]` change trail length; `Q` quits

- [ ] **Step 5: Run full test suite**

Run: `ctest --test-dir build -C Debug --output-on-failure`
Expected: all PASS.

- [ ] **Step 6: Ask user to approve commit, then commit**

```bash
git add src/app/ CMakeLists.txt
git commit -m "feat: input state machine — polled camera, pause toggle, wheel/arrow placement (bugs 7,8,9,12)"
```

---

### Task 13: Final verification pass (bugs 13, 15 + Plan A acceptance)

**Files:**
- Modify: only if a checklist item fails (expected: no changes)
- Review: `src/` for bug-15 leftovers

**Interfaces:**
- Consumes: all tasks.
- Produces: Plan A acceptance — the demo milestone for the user.

- [ ] **Step 1: Verify bug 13 (first-frame spike)**

In `main.cpp`, confirm `double last = glfwGetTime();` executes **after** scene setup and **before** the loop, and that `world.advance(fd)` receives a first `fd ≈ 0`. Run the app: on the very first frame the camera must not jump and the sim must not skip steps.

- [ ] **Step 2: Bug-15 sweep (code review checklist)**

- [ ] No `glGetUniformLocation` inside per-body/per-frame loops except where unavoidable (cache into locals at `init` if trivially possible; `setMats` may look up — acceptable for Plan A, noted for Plan B optimization)
- [ ] No `std::vector<float>` temporary per body-pair in physics (`computeAccelerations` uses `glm::dvec3` only) — grep `std::vector` in `world.cpp` → only `computeAccelerations`'s return accumulator
- [ ] No dead members: `grep -n "Initalizing\|Launched\|LastPos\|target" src/` → zero hits
- [ ] `physics/` has no GL includes: `grep -rn "GL/" src/physics/` → zero hits

- [ ] **Step 3: Full build from clean + full test suite**

Run:
```bat
cmake --build build --config Debug --target clean
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```
Expected: 4 app targets + 2 test targets build with no new warnings; ctest reports `physics` + `render` PASS.

- [ ] **Step 4: Full manual acceptance run (10 minutes with the app)**

- [ ] Watch the two-planet scene: orbits stable for 10+ minutes (no inward spiral — leapfrog), no FPS-dependent speed change (drag window between monitors / resize — sim rate identical)
- [ ] Spawn a body, grow it big with RMB, release: **no size jump** (bug 5), it begins falling toward the star immediately
- [ ] Two spawned bodies touching → merge into one, conserving motion (fly one into another)
- [ ] Pause with `P`, wait 30 s, unpause: everything resumes exactly where it stopped
- [ ] Trail + grid + resize + camera smoothness all good
- [ ] No NaN explosions after 5 minutes of casual play

- [ ] **Step 5: Ask user to accept Plan A (demo) — then ask approval to commit**

```bash
git add -A
git commit -m "plan A complete: SI physics core, leapfrog, merges, trails, input overhaul — bugs 1-13,15 fixed"
```

---

## Plan self-review

**Spec coverage (Phases 1–4):** Phase 1 scaffold → Task 1. Phase 2 physics (§4.1–4.7 minus BH) → Tasks 2–7 (units §4.1: constants + SI everywhere + UNIT conversions at boundaries; §4.2: Task 2; §4.3: Tasks 2–3; §4.4: Task 4; §4.5: Task 7; §4.6: Task 5; §4.8: Task 6). Phase 3 render → Tasks 8–11 (mesh Task 8; grid Task 9/10; resize Task 10; trails Tasks 6+11; §4.9 grid spec → Task 9). Phase 4 input → Task 12 (all of spec §7's Plan-A keys). Bug map: 1,2,3,4,5,6,7,8,9,10,11,12,13,15 → Tasks 2,5,5,4,7,9,12,12,12,10,8,12,7,13. Bug 14 (consolidation) and features #3,#4,#6,#7,#8,#9,#10 → Plans B/C, out of scope here.

**Placeholder scan:** Task 7 Step 3 `gridCfg` — declared in the step text ("`gs::render::GridConfig gridCfg;` declared before the loop") ✓. Task 10 explicitly names the line to delete rather than leaving it ✓. No TBDs.

**Type consistency check:** `spawn(const Body&) → uint64_t` used consistently (Tasks 2, 5, 7, 12); `advance(double) → int` (Tasks 2, 6, 12); `TrailRing::pts/cap/push/setCap` (Tasks 1, 6, 11); `GridConfig{mode,sizeUnits,divisions,planeYFactor}` (Tasks 1, 9, 10); `sphereVertices(double,int,int)` (Tasks 1, 10); `InputContext{world,cam,frameDelta,fbw,fbh}` (Task 12) matches main's aggregate init in Task 12 Step 3. Self-review fixes applied inline: scaffold placeholder sphere removed, stray `projection` uniform line removed from `drawBodies`, `ggs::UNIT` typo corrected, unused `viewDistance` helper dropped.
