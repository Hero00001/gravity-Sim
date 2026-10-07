#include "render/hud.hpp"
#include <GL/glew.h>
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
void main(){
  vec2 ndc = vec2(aPos.x / uRes.x * 2.0 - 1.0, 1.0 - aPos.y / uRes.y * 2.0);
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
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, 40000 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    // stb_easy_font emits 4 floats per vertex (x, y, z=0, color); we use x,y only.
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glBindVertexArray(0);
    buf_.resize(40000);
    return true;
}

void Hud::shutdown() {
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    if (prog_) glDeleteProgram(prog_);
    prog_ = 0;
}

void Hud::draw(const HudData& d) {
    if (!d.hudVisible) return;

    glUseProgram(prog_);
    glUniform2f(glGetUniformLocation(prog_, "uRes"), float(d.fbw), float(d.fbh));
    glUniform4f(glGetUniformLocation(prog_, "uColor"), 0.75f, 1.0f, 0.9f, 1.0f);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    int vcount = 0;  // vertex count (each vertex = 4 floats: x,y,z,packed-color)
    auto text = [&](float x, float y, const char* s) {
        const std::size_t used_floats = (std::size_t)vcount * 4;
        const int avail_bytes = (int)((buf_.size() - used_floats) * sizeof(float));
        if (avail_bytes <= 0) return;
        // stb_easy_font_print returns the number of quads; each quad = 4 vertices.
        const int quads = stb_easy_font_print(x, y, (char*)s, nullptr,
                                              buf_.data() + used_floats, avail_bytes);
        vcount += quads * 4;
    };

    char line[160];
    std::snprintf(line, sizeof(line), "FPS %.0f", d.fps);
    text(10.0f, 10.0f, line);

    char tbuf[40];
    const double t = d.simTime;
    if (t < 3600.0)            std::snprintf(tbuf, sizeof(tbuf), "T+%.0f s", t);
    else if (t < 86400.0)      std::snprintf(tbuf, sizeof(tbuf), "T+%.2f h", t / 3600.0);
    else if (t < 31557600.0)   std::snprintf(tbuf, sizeof(tbuf), "T+%.2f d", t / 86400.0);
    else                       std::snprintf(tbuf, sizeof(tbuf), "T+%.2f y", t / 31557600.0);
    std::snprintf(line, sizeof(line), "%s   x%.2g", tbuf, d.timeScale);
    text(10.0f, 30.0f, line);

    if (d.paused) text(10.0f, 50.0f, "[PAUSED]");

    std::snprintf(line, sizeof(line), "bodies %d   scene %s", d.bodyCount, d.sceneName.c_str());
    text(10.0f, d.paused ? 70.0f : 50.0f, line);

    if (d.hasSelection) {
        const float x = float(d.fbw) - 280.0f;
        std::snprintf(line, sizeof(line), "body #%llu", (unsigned long long)d.selId);
        text(x, 10.0f, line);
        std::snprintf(line, sizeof(line), "mass %.3g kg", d.selMass);
        text(x, 30.0f, line);
        std::snprintf(line, sizeof(line), "speed %.3g km/s", d.selSpeed);
        text(x, 50.0f, line);
        std::snprintf(line, sizeof(line), "dist %.3g km", d.selDist);
        text(x, 70.0f, line);
    }

    if (d.placing) {
        std::snprintf(line, sizeof(line), "placing mass %.3g kg", d.placeMass);
        text(10.0f, float(d.fbh) - 50.0f, line);
        text(10.0f, float(d.fbh) - 30.0f, "wheel=depth  RMB=grow  release=drop  arrows=move");
    }

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vcount * 4 * sizeof(float), buf_.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, vcount);
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
}

} // namespace gs::render
