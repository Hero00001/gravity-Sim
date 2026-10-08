#include "render/renderer.hpp"
#include "render/grid.hpp"
#include "render/mesh.hpp"
#include "physics/constants.hpp"
#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <unordered_set>

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
    // Headlight from the origin (where the dominant mass normally sits).
    lightIntensity = max(dot(normal, normalize(-worldPos)), 0.0);
})glsl";

const char* kBodyFS = R"glsl(
#version 330 core
in float lightIntensity;
out vec4 FragColor;
uniform vec4 objectColor;
uniform bool isGrid;
uniform bool GLOW;
void main() {
    if (isGrid)       FragColor = objectColor;
    else if (GLOW)    FragColor = vec4(objectColor.rgb * 6.0, objectColor.a);
    else {
        // Keep a lit/dark gradient but never crush the dark side to pure black.
        float fade = 0.35 + 0.65 * smoothstep(0.0, 1.0, lightIntensity);
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

// Background stars: a point cloud on a sphere that rides with the camera, so it reads as
// "infinitely far away" and never parallaxes with the scene.
const char* kStarVS = R"glsl(
#version 330 core
layout(location=0) in vec3 aDir;
layout(location=1) in float aBright;
uniform mat4 view, projection;
uniform vec3 uCamPos;
uniform float uRadius;
out float vBright;
void main() {
    gl_Position = projection * view * vec4(uCamPos + aDir * uRadius, 1.0);
    gl_PointSize = 1.0 + aBright * 2.0;
    vBright = aBright;
})glsl";

const char* kStarFS = R"glsl(
#version 330 core
in float vBright;
out vec4 FragColor;
void main() { FragColor = vec4(vec3(0.78, 0.84, 1.0), 0.20 + 0.70 * vBright); })glsl";

// Camera-facing additive halo for self-luminous bodies (stars).
const char* kGlowVS = R"glsl(
#version 330 core
layout(location=0) in vec2 aXY;
uniform mat4 view, projection;
uniform vec3 uCenter, uRight, uUp;
uniform float uScale;
out vec2 vUV;
void main() {
    vUV = aXY;
    vec3 world = uCenter + (uRight * aXY.x + uUp * aXY.y) * uScale;
    gl_Position = projection * view * vec4(world, 1.0);
})glsl";

const char* kGlowFS = R"glsl(
#version 330 core
in vec2 vUV;
out vec4 FragColor;
uniform vec4 uColor;
uniform float uStrength;
void main() {
    float r = length(vUV);
    if (r >= 1.0) discard;
    float a = pow(1.0 - r, 2.5);
    FragColor = vec4(uColor.rgb, a * uStrength);
})glsl";

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

void setMats(int locModel, int locView, int locProj,
             const Camera& cam, float aspect, const glm::mat4& model) {
    glUniformMatrix4fv(locModel, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(locView, 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(locProj, 1, GL_FALSE, glm::value_ptr(cam.projection(aspect)));
}

bool sameGrid(const GridConfig& a, const GridConfig& b) {
    return a.mode == b.mode && a.sizeUnits == b.sizeUnits &&
           a.divisions == b.divisions && a.planeYFactor == b.planeYFactor;
}

} // namespace

bool Renderer::init() {
    progBody_ = link(kBodyVS, kBodyFS);
    progTrail_ = link(kTrailVS, kTrailFS);
    progStar_ = link(kStarVS, kStarFS);
    progGlow_ = link(kGlowVS, kGlowFS);
    if (!progBody_ || !progTrail_ || !progStar_ || !progGlow_) {
        if (progBody_) glDeleteProgram(progBody_);
        if (progTrail_) glDeleteProgram(progTrail_);
        if (progStar_) glDeleteProgram(progStar_);
        if (progGlow_) glDeleteProgram(progGlow_);
        return false;
    }

    // Cache uniform locations once (spec #10): each program only ever references a fixed
    // set of uniforms, so querying them per frame is pure waste.
    uBody_.model       = glGetUniformLocation(progBody_, "model");
    uBody_.view        = glGetUniformLocation(progBody_, "view");
    uBody_.proj        = glGetUniformLocation(progBody_, "projection");
    uBody_.objectColor = glGetUniformLocation(progBody_, "objectColor");
    uBody_.isGrid      = glGetUniformLocation(progBody_, "isGrid");
    uBody_.glow        = glGetUniformLocation(progBody_, "GLOW");
    uTrail_.view       = glGetUniformLocation(progTrail_, "view");
    uTrail_.proj       = glGetUniformLocation(progTrail_, "projection");
    uTrail_.tintColor  = glGetUniformLocation(progTrail_, "tintColor");
    uStar_.view        = glGetUniformLocation(progStar_, "view");
    uStar_.proj        = glGetUniformLocation(progStar_, "projection");
    uStar_.camPos      = glGetUniformLocation(progStar_, "uCamPos");
    uStar_.radius      = glGetUniformLocation(progStar_, "uRadius");
    uGlow_.view        = glGetUniformLocation(progGlow_, "view");
    uGlow_.proj        = glGetUniformLocation(progGlow_, "projection");
    uGlow_.center      = glGetUniformLocation(progGlow_, "uCenter");
    uGlow_.right       = glGetUniformLocation(progGlow_, "uRight");
    uGlow_.up          = glGetUniformLocation(progGlow_, "uUp");
    uGlow_.scale       = glGetUniformLocation(progGlow_, "uScale");
    uGlow_.color       = glGetUniformLocation(progGlow_, "uColor");
    uGlow_.strength    = glGetUniformLocation(progGlow_, "uStrength");

    // --- grid line buffer ---
    glGenVertexArrays(1, &gridVao_);
    glGenBuffers(1, &gridVbo_);
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    // --- trail buffer (pos.xyz + alpha) ---
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

    // --- starfield (direction.xyz + brightness), generated once, deterministic ---
    {
        const int kStars = 1600;
        std::mt19937 rng(0xC0FFEE);
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        std::vector<float> verts;
        verts.reserve(std::size_t(kStars) * 4);
        for (int i = 0; i < kStars; ++i) {
            // Uniform on the sphere (z uniform, azimuth uniform).
            const double z = 1.0 - 2.0 * u01(rng);
            const double a = 6.283185307179586 * u01(rng);
            const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
            const float bright = float(std::pow(u01(rng), 3.0));   // few bright, many dim
            verts.insert(verts.end(), {float(std::cos(a) * r), float(z), float(std::sin(a) * r),
                                       bright});
        }
        starCount_ = kStars;
        glGenVertexArrays(1, &starVao_);
        glGenBuffers(1, &starVbo_);
        glBindVertexArray(starVao_);
        glBindBuffer(GL_ARRAY_BUFFER, starVbo_);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(float)),
                     verts.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
    }

    // --- glow quad (unit billboard) ---
    {
        const float quad[12] = {-1.f, -1.f, 1.f, -1.f, 1.f, 1.f,
                                -1.f, -1.f, 1.f,  1.f, -1.f, 1.f};
        glGenVertexArrays(1, &glowVao_);
        glGenBuffers(1, &glowVbo_);
        glBindVertexArray(glowVao_);
        glBindBuffer(GL_ARRAY_BUFFER, glowVbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glBindVertexArray(0);
    }
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
    glDeleteVertexArrays(1, &starVao_);
    glDeleteBuffers(1, &starVbo_);
    glDeleteVertexArrays(1, &glowVao_);
    glDeleteBuffers(1, &glowVbo_);
    glDeleteProgram(progBody_);
    glDeleteProgram(progTrail_);
    glDeleteProgram(progStar_);
    glDeleteProgram(progGlow_);
}

void Renderer::draw(const gs::World& world, const Camera& cam, int fbw, int fbh,
                    const GridConfig& grid, std::uint64_t selectedId, std::uint64_t hoverId) {
    glViewport(0, 0, fbw, fbh);                          // every frame → resize-safe (bug 10)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const float aspect = fbh > 0 ? float(fbw) / float(fbh) : 1.0f;

    syncBodies(world);

    // Backdrop first, with depth off so it only shows where nothing else draws.
    glDisable(GL_DEPTH_TEST);
    drawStarfield(cam, aspect);
    glEnable(GL_DEPTH_TEST);

    drawGrid(grid, world, cam, aspect);
    drawTrails(world, cam, aspect);
    drawGlow(world, cam, aspect);                        // halo behind the body itself
    drawBodies(world, cam, aspect);
    if (hoverId != 0 && hoverId != selectedId)
        drawRing(world, cam, aspect, hoverId, glm::vec4(0.55f, 0.85f, 1.0f, 0.55f), 1.35f);
    drawRing(world, cam, aspect, selectedId, glm::vec4(1.0f, 0.85f, 0.2f, 1.0f), 1.6f);
}

void Renderer::drawStarfield(const Camera& cam, float aspect) {
    if (starCount_ == 0) return;
    glEnable(GL_PROGRAM_POINT_SIZE);
    glUseProgram(progStar_);
    glUniformMatrix4fv(uStar_.view, 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(uStar_.proj, 1, GL_FALSE, glm::value_ptr(cam.projection(aspect)));
    glUniform3f(uStar_.camPos, cam.pos.x, cam.pos.y, cam.pos.z);
    glUniform1f(uStar_.radius, cam.farPlane * 0.9f);
    glBindVertexArray(starVao_);
    glDrawArrays(GL_POINTS, 0, starCount_);
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);
}

void Renderer::syncBodies(const gs::World& world) {
    // delete GPU bodies that no longer exist (removed/merged)
    std::unordered_set<std::uint64_t> live;
    live.reserve(world.bodies.size() * 2);
    for (const auto& b : world.bodies) live.insert(b.id);
    for (auto it = gpu_.begin(); it != gpu_.end();) {
        if (live.count(it->first)) ++it;
        else { glDeleteVertexArrays(1, &it->second.vao);
               glDeleteBuffers(1, &it->second.vbo); it = gpu_.erase(it); }
    }
    // create / refresh (display radius = compressive map of the physical radius)
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
        setMats(uBody_.model, uBody_.view, uBody_.proj, cam, aspect,
                glm::translate(glm::mat4(1.0f), units));
        glBindVertexArray(it->second.vao);
        glDrawArrays(GL_TRIANGLES, 0, it->second.count);
    }
}

void Renderer::drawGlow(const gs::World& world, const Camera& cam, float aspect) {
    bool any = false;
    for (const auto& b : world.bodies) if (b.glow) { any = true; break; }
    if (!any) return;

    glUseProgram(progGlow_);
    glUniformMatrix4fv(uGlow_.view, 1, GL_FALSE, glm::value_ptr(cam.view()));
    glUniformMatrix4fv(uGlow_.proj, 1, GL_FALSE, glm::value_ptr(cam.projection(aspect)));
    const glm::vec3 r = cam.right(), u = cam.up();
    glUniform3f(uGlow_.right, r.x, r.y, r.z);
    glUniform3f(uGlow_.up, u.x, u.y, u.z);

    glDisable(GL_DEPTH_TEST);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);                    // additive halo
    glBindVertexArray(glowVao_);
    for (const auto& b : world.bodies) {
        if (!b.glow) continue;
        const float disp = float(world.displayRadius(b) / gs::UNIT);
        const glm::vec3 c = glm::vec3(b.position) / float(gs::UNIT);
        glUniform3f(uGlow_.center, c.x, c.y, c.z);
        glUniform1f(uGlow_.scale, disp * 3.2f);
        glUniform4f(uGlow_.color, b.color.r, b.color.g, b.color.b, 1.0f);
        glUniform1f(uGlow_.strength, 0.85f);
        glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    glBindVertexArray(0);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);     // restore normal blending
    glEnable(GL_DEPTH_TEST);
}

void Renderer::drawGrid(const GridConfig& grid, const gs::World& world, const Camera& cam,
                        float aspect) {
    if (grid.mode == GridMode::Off) return;

    // Rebuild the immutable template only when the scene's grid config actually changes.
    if (!gridBaseValid_ || !sameGrid(gridCfg_, grid)) {
        gridCfg_ = grid;
        gridBase_ = buildGridBase(grid);
        gridBaseValid_ = true;
    }
    const auto displaced = displaceGrid(gridBase_, grid, world.bodies);

    glUseProgram(progBody_);
    glUniform1i(uBody_.isGrid, 1);
    glUniform1i(uBody_.glow, 0);
    glUniform4f(uBody_.objectColor, 0.42f, 0.62f, 0.85f, 0.30f);   // cool blueprint grid
    setMats(uBody_.model, uBody_.view, uBody_.proj, cam, aspect, glm::mat4(1.0f));

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

void Renderer::drawRing(const gs::World& world, const Camera& cam, float aspect,
                        std::uint64_t id, const glm::vec4& color, float ringScale) {
    if (id == 0) return;
    const gs::Body* sel = nullptr;
    for (const auto& b : world.bodies) if (b.id == id) { sel = &b; break; }
    if (!sel) return;
    const glm::vec3 c = glm::vec3(sel->position) / float(gs::UNIT);
    const float disp = float(world.displayRadius(*sel) / gs::UNIT);
    const float ringR = disp * ringScale;      // proportional so it scales with the scene
    const glm::vec3 r = cam.right();
    const glm::vec3 u = cam.up();
    ring_.clear();
    const int N = 48;
    static const float PI = 3.14159265358979323846f;
    for (int i = 0; i < N; ++i) {
        const float a0 = 2.0f * PI * float(i) / float(N);
        const float a1 = 2.0f * PI * float(i + 1) / float(N);
        const glm::vec3 p0 = c + (r * std::cos(a0) + u * std::sin(a0)) * ringR;
        const glm::vec3 p1 = c + (r * std::cos(a1) + u * std::sin(a1)) * ringR;
        ring_.insert(ring_.end(), {p0.x, p0.y, p0.z, p1.x, p1.y, p1.z});
    }
    glUseProgram(progBody_);
    glUniform1i(uBody_.isGrid, 1);
    glUniform1i(uBody_.glow, 0);
    glUniform4f(uBody_.objectColor, color.r, color.g, color.b, color.a);
    setMats(uBody_.model, uBody_.view, uBody_.proj, cam, aspect, glm::mat4(1.0f));
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, ring_.size() * sizeof(float), ring_.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_LINES, 0, GLsizei(ring_.size() / 3));
    glBindVertexArray(0);
}

} // namespace gs::render
