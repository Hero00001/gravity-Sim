#include "render/renderer.hpp"
#include "render/grid.hpp"
#include "render/mesh.hpp"
#include "physics/constants.hpp"
#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cmath>
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
        glDeleteShader(s);
        return 0;
    }
    return s;
}

unsigned link(const char* vs, const char* fs) {
    unsigned p = glCreateProgram();
    unsigned v = compile(GL_VERTEX_SHADER, vs);
    unsigned f = compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) {
        glDeleteShader(v);
        glDeleteShader(f);
        glDeleteProgram(p);
        return 0;
    }
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(p, 512, nullptr, log);
        std::cerr << "shader link failed: " << log << "\n";
        glDeleteProgram(p);
        return 0;
    }
    return p;
}

void setMats(unsigned prog, int locModel, int locView, int locProj,
             const Camera& cam, float aspect, const glm::mat4& model) {
    glUniformMatrix4fv(locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(locView, 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(locProj, 1, GL_FALSE, glm::value_ptr(cam.projection(aspect)));
}

} // namespace

bool Renderer::init() {
    progBody_ = link(kBodyVS, kBodyFS);
    progTrail_ = link(kTrailVS, kTrailFS);
    if (!progBody_ || !progTrail_) {
        if (progBody_) glDeleteProgram(progBody_);
        if (progTrail_) glDeleteProgram(progTrail_);
        return false;
    }

    // Cache uniform locations once (spec #10): the body shader and trail shader only ever
    // reference a fixed set of uniforms, so querying them per frame is pure waste.
    uBody_.model       = glGetUniformLocation(progBody_, "model");
    uBody_.view        = glGetUniformLocation(progBody_, "view");
    uBody_.proj        = glGetUniformLocation(progBody_, "projection");
    uBody_.objectColor = glGetUniformLocation(progBody_, "objectColor");
    uBody_.isGrid      = glGetUniformLocation(progBody_, "isGrid");
    uBody_.glow        = glGetUniformLocation(progBody_, "GLOW");
    uTrail_.view       = glGetUniformLocation(progTrail_, "view");
    uTrail_.proj       = glGetUniformLocation(progTrail_, "projection");
    uTrail_.tintColor  = glGetUniformLocation(progTrail_, "tintColor");

    glGenVertexArrays(1, &gridVao_);
    glGenBuffers(1, &gridVbo_);
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
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

void Renderer::draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
                    const GridConfig& grid, std::uint64_t selectedId) {
    glViewport(0, 0, fbw, fbh);                          // every frame → resize-safe (bug 10)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const float aspect = fbh > 0 ? float(fbw) / float(fbh) : 1.0f;

    syncBodies(world);
    drawGrid(grid, world, cam, aspect);
    drawTrails(world, cam, aspect);
    drawBodies(world, cam, aspect);
    drawSelection(world, cam, aspect, selectedId);
}

void Renderer::syncBodies(const gs::World& world) {
    // delete GPU bodies that no longer exist (removed/merged)
    for (auto it = gpu_.begin(); it != gpu_.end();) {
        bool found = false;
        for (const auto& b : world.bodies) if (b.id == it->first) { found = true; break; }
        if (found) ++it;
        else { glDeleteVertexArrays(1, &it->second.vao);
               glDeleteBuffers(1, &it->second.vbo); it = gpu_.erase(it); }
    }
    // create / refresh (display radius = max(physical, world visual floor))
    for (const auto& b : world.bodies) {
        auto it = gpu_.find(b.id);
        const double disp = world.displayRadius(b);
        if (it == gpu_.end()) {
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
            g.radius = disp;
            g.count = int(verts.size() / 3);
            gpu_.emplace(b.id, g);
        } else if (std::fabs(disp - it->second.radius) >
                   0.005 * std::max(it->second.radius, 1.0)) {
            const auto verts = sphereVertices(disp / UNIT);
            glBindBuffer(GL_ARRAY_BUFFER, it->second.vbo);
            glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(),
                         GL_STATIC_DRAW);
            it->second.radius = disp;
            it->second.count = int(verts.size() / 3);
        }
    }
}

void Renderer::drawBodies(const gs::World& world, const Camera& cam, float aspect) {
    glUseProgram(progBody_);
    glUniform1i(uBody_.isGrid, 0);
    for (const auto& b : world.bodies) {
        auto it = gpu_.find(b.id);
        if (it == gpu_.end()) continue;
        glUniform1i(uBody_.glow, b.glow ? 1 : 0);
        glUniform4f(uBody_.objectColor, b.color.r, b.color.g, b.color.b, b.color.a);
        const glm::vec3 units = glm::vec3(b.position) / float(UNIT);
        setMats(progBody_, uBody_.model, uBody_.view, uBody_.proj, cam, aspect,
                glm::translate(glm::mat4(1.0f), units));
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
    glUniform1i(uBody_.isGrid, 1);
    glUniform1i(uBody_.glow, 0);
    glUniform4f(uBody_.objectColor, 1.f, 1.f, 1.f, 0.25f);
    setMats(progBody_, uBody_.model, uBody_.view, uBody_.proj, cam, aspect, glm::mat4(1.0f));

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -1.0f);                        // keeps grid under bodies (z-fighting)
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, displaced.size() * sizeof(float), displaced.data(),
                 GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, GLsizei(displaced.size() / 3));
    glBindVertexArray(0);
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void Renderer::drawTrails(const gs::World& world, const Camera& cam, float aspect) {
    glUseProgram(progTrail_);
    glUniformMatrix4fv(uTrail_.view, 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(uTrail_.proj, 1, GL_FALSE, glm::value_ptr(cam.projection(aspect)));
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
        glUniform4f(uTrail_.tintColor, b.color.r, b.color.g, b.color.b, b.color.a);
        glBufferData(GL_ARRAY_BUFFER, scratch_.size() * sizeof(float), nullptr,
                     GL_STREAM_DRAW);                     // orphan → reused buffer
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        scratch_.size() * sizeof(float), scratch_.data());
        glDrawArrays(GL_LINE_STRIP, 0, GLsizei(pts.size()));
    }
    glBindVertexArray(0);
}

void Renderer::drawSelection(const gs::World& world, const Camera& cam, float aspect, std::uint64_t id) {
    if (id == 0) return;
    const gs::Body* sel = nullptr;
    for (const auto& b : world.bodies) if (b.id == id) { sel = &b; break; }
    if (!sel) return;
    const glm::vec3 c = glm::vec3(sel->position) / float(gs::UNIT);
    const float disp = float(world.displayRadius(*sel) / gs::UNIT);
    const float ringR = disp * 1.6f + 2.0f;
    const glm::vec3 r = cam.right();
    const glm::vec3 u = cam.up();
    std::vector<float> verts;
    const int N = 48;
    static const float PI = 3.14159265358979323846f;
    for (int i = 0; i < N; ++i) {
        const float a0 = 2.0f * PI * float(i) / float(N);
        const float a1 = 2.0f * PI * float(i + 1) / float(N);
        const glm::vec3 p0 = c + (r * std::cos(a0) + u * std::sin(a0)) * ringR;
        const glm::vec3 p1 = c + (r * std::cos(a1) + u * std::sin(a1)) * ringR;
        verts.insert(verts.end(), {p0.x, p0.y, p0.z, p1.x, p1.y, p1.z});
    }
    glUseProgram(progBody_);
    glUniform1i(uBody_.isGrid, 1);
    glUniform1i(uBody_.glow, 0);
    glUniform4f(uBody_.objectColor, 1.0f, 1.0f, 0.0f, 1.0f);
    setMats(progBody_, uBody_.model, uBody_.view, uBody_.proj, cam, aspect, glm::mat4(1.0f));
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, GLsizei(verts.size() / 3));
    glBindVertexArray(0);
}

} // namespace gs::render
