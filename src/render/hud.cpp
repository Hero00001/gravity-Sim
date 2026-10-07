#include "render/hud.hpp"
#include <GL/glew.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>

#define STB_EASY_FONT_IMPLEMENTATION
#include "stb_easy_font.h"

namespace gs::render {

namespace {
unsigned compile(unsigned type, const char* src) {
    unsigned s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { glDeleteShader(s); return 0; }
    return s;
}
unsigned link(const char* vs, const char* fs) {
    unsigned p = glCreateProgram();
    unsigned v = compile(GL_VERTEX_SHADER, vs);
    unsigned f = compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) { glDeleteProgram(p); return 0; }
    glAttachShader(p, v); glAttachShader(p, f); glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    GLint ok = 0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { glDeleteProgram(p); return 0; }
    return p;
}
const char* kHudVS = R"glsl(
#version 330 core
layout(location=0) in vec2 aPos;
uniform vec2 uRes;
uniform float uScale;      // font pixels -> screen pixels (legibility / HiDPI)
void main(){
  vec2 p = aPos * uScale;
  vec2 ndc = vec2(p.x / uRes.x * 2.0 - 1.0, 1.0 - p.y / uRes.y * 2.0);
  gl_Position = vec4(ndc, 0.0, 1.0);
})glsl";
const char* kHudFS = R"glsl(
#version 330 core
uniform vec4 uColor;
out vec4 frag;
void main(){ frag = uColor; })glsl";
} // namespace

bool Hud::init() {
    prog_ = link(kHudVS, kHudFS);
    if (!prog_) return false;
    uResLoc_   = glGetUniformLocation(prog_, "uRes");
    uColorLoc_ = glGetUniformLocation(prog_, "uColor");
    uScaleLoc_ = glGetUniformLocation(prog_, "uScale");

    buf_.resize(40000);                          // floats; 4 per vertex -> 10k vertices
    const int maxQuads = int(buf_.size() / 4 / 4);   // floats / (floats/vertex) / (verts/quad)
    indices_.resize(std::size_t(maxQuads) * 6);
    for (int q = 0; q < maxQuads; ++q) {
        const unsigned b = unsigned(q) * 4;
        indices_[std::size_t(q) * 6 + 0] = b + 0;
        indices_[std::size_t(q) * 6 + 1] = b + 1;
        indices_[std::size_t(q) * 6 + 2] = b + 2;
        indices_[std::size_t(q) * 6 + 3] = b + 0;
        indices_[std::size_t(q) * 6 + 4] = b + 2;
        indices_[std::size_t(q) * 6 + 5] = b + 3;
    }

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(buf_.size() * sizeof(float)), nullptr, GL_DYNAMIC_DRAW);
    // stb_easy_font emits 4 floats per vertex (x, y, z=0, color); we use x,y only.
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(indices_.size() * sizeof(unsigned)),
                 indices_.data(), GL_STATIC_DRAW);
    glBindVertexArray(0);
    return true;
}

void Hud::shutdown() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ebo_);
    if (prog_) glDeleteProgram(prog_);
    prog_ = 0;
}

void Hud::draw(const HudData& d) {
    if (!d.hudVisible) return;

    // Keep text legible on small windows and on HiDPI framebuffers (where the
    // framebuffer is bigger than the window in pixels). 2x minimum.
    const float scale = std::max(2.0f, float(d.fbh) / 600.0f);
    const float W = float(d.fbw) / scale;    // usable width, in font pixels
    const float H = float(d.fbh) / scale;    // usable height, in font pixels
    const float LH = 16.0f;                  // line height (glyphs are ~12 tall)

    glUseProgram(prog_);
    glUniform2f(uResLoc_, float(d.fbw), float(d.fbh));
    glUniform1f(uScaleLoc_, scale);
    glUniform4f(uColorLoc_, 0.75f, 1.0f, 0.9f, 1.0f);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int vcount = 0;  // vertex count (each vertex = 4 floats: x,y,z,packed-color)
    auto text = [&](float x, float y, const char* s) {
        const std::size_t used_floats = (std::size_t)vcount * 4;
        if (used_floats >= buf_.size()) return;
        const int avail_bytes = (int)((buf_.size() - used_floats) * sizeof(float));
        // stb_easy_font_print returns the number of QUADS; each quad = 4 vertices.
        const int quads = stb_easy_font_print(x, y, (char*)s, nullptr,
                                              buf_.data() + used_floats, avail_bytes);
        vcount += quads * 4;
    };

    char line[160];
    float y = 8.0f;
    std::snprintf(line, sizeof(line), "FPS %.0f", d.fps);
    text(8.0f, y, line); y += LH;

    char tbuf[40];
    const double t = d.simTime;
    if (t < 3600.0)            std::snprintf(tbuf, sizeof(tbuf), "T+%.0f s", t);
    else if (t < 86400.0)      std::snprintf(tbuf, sizeof(tbuf), "T+%.2f h", t / 3600.0);
    else if (t < 31557600.0)   std::snprintf(tbuf, sizeof(tbuf), "T+%.2f d", t / 86400.0);
    else                       std::snprintf(tbuf, sizeof(tbuf), "T+%.2f y", t / 31557600.0);
    std::snprintf(line, sizeof(line), "%s   x%.2g", tbuf, d.timeScale);
    text(8.0f, y, line); y += LH;

    std::snprintf(line, sizeof(line), "bodies %d   scene %s", d.bodyCount, d.sceneName.c_str());
    text(8.0f, y, line); y += LH;

    if (d.paused) { text(8.0f, y, "[PAUSED]"); y += LH; }

    if (d.hasSelection) {
        const float x = W - 190.0f;
        float sy = 8.0f;
        std::snprintf(line, sizeof(line), "body #%llu", (unsigned long long)d.selId);
        text(x, sy, line); sy += LH;
        std::snprintf(line, sizeof(line), "mass %.3g kg", d.selMass);
        text(x, sy, line); sy += LH;
        std::snprintf(line, sizeof(line), "speed %.3g km/s", d.selSpeed);
        text(x, sy, line); sy += LH;
        std::snprintf(line, sizeof(line), "dist %.3g km", d.selDist);
        text(x, sy, line); sy += LH;
    }

    if (d.placing) {
        std::snprintf(line, sizeof(line), "placing mass %.3g kg", d.placeMass);
        text(8.0f, H - 2.0f * LH, line);
        text(8.0f, H - LH, "wheel=depth  RMB=grow  release=drop  arrows=move");
    }

    const int quads = std::min(vcount / 4, int(indices_.size() / 6));
    if (quads > 0) {
        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(std::size_t(vcount) * 4 * sizeof(float)),
                     buf_.data(), GL_DYNAMIC_DRAW);
        glDrawElements(GL_TRIANGLES, quads * 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    glEnable(GL_DEPTH_TEST);
}

} // namespace gs::render
