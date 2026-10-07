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
    if (!progBody_ || !progTrail_) {
        if (progBody_) glDeleteProgram(progBody_);
        if (progTrail_) glDeleteProgram(progTrail_);
        return false;
    }

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
