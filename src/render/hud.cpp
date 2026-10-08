#include "render/hud.hpp"
#include <GL/glew.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>

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

constexpr float kLineH = 16.0f;   // line height in font pixels (glyphs are 12 tall)
constexpr float kPad   = 6.0f;    // panel padding in font pixels

// One HUD block: a set of text lines plus the translucent panel behind them.
struct Block {
    float x = 0.0f, y = 0.0f;     // text origin (font pixels)
    bool rightAligned = false;
    std::vector<std::string> lines;
    float w = 0.0f;               // measured text width
    float panelX = 0.0f, panelY = 0.0f, panelW = 0.0f, panelH = 0.0f;
};
} // namespace

bool Hud::init() {
    prog_ = link(kHudVS, kHudFS);
    if (!prog_) return false;
    uResLoc_   = glGetUniformLocation(prog_, "uRes");
    uColorLoc_ = glGetUniformLocation(prog_, "uColor");
    uScaleLoc_ = glGetUniformLocation(prog_, "uScale");

    buf_.resize(64000);                              // floats; 4 per vertex -> 16k vertices
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

    // Keep text legible on small windows and on HiDPI framebuffers. Integer scale only,
    // so glyph edges stay pixel-aligned (a fractional scale looks blurry).
    const float scale = std::max(2.0f, std::round(float(d.fbh) / 700.0f));
    const float W = float(d.fbw) / scale;    // usable width, in font pixels
    const float H = float(d.fbh) / scale;    // usable height, in font pixels

    int vcount = 0;  // vertex count (each vertex = 4 floats: x,y,z,packed-color)

    // Panel quad: perimeter order must match stb_easy_font's (x0,y0),(x1,y0),(x1,y1),(x0,y1)
    // so the shared index buffer triangulates it correctly.
    auto rect = [&](float x, float y, float w, float h) {
        const std::size_t used = std::size_t(vcount) * 4;
        if (used + 16 > buf_.size()) return;
        float* p = buf_.data() + used;
        p[0] = x;     p[1] = y;     p[2] = 0.f; p[3] = 0.f;
        p[4] = x + w; p[5] = y;     p[6] = 0.f; p[7] = 0.f;
        p[8] = x + w; p[9] = y + h; p[10] = 0.f; p[11] = 0.f;
        p[12] = x;    p[13] = y + h; p[14] = 0.f; p[15] = 0.f;
        vcount += 4;
    };
    auto text = [&](float x, float y, const char* s) {
        const std::size_t used_floats = (std::size_t)vcount * 4;
        if (used_floats >= buf_.size()) return;
        const int avail_bytes = (int)((buf_.size() - used_floats) * sizeof(float));
        const int quads = stb_easy_font_print(x, y, const_cast<char*>(s), nullptr,
                                              buf_.data() + used_floats, avail_bytes);
        vcount += quads * 4;
    };

    // ---------------- collect blocks ----------------
    std::vector<Block> blocks;

    // --- status (top-left) ---
    {
        char line[192], tbuf[48];
        const double t = d.simTime;
        if (t < 3600.0)          std::snprintf(tbuf, sizeof(tbuf), "T+%.0f s", t);
        else if (t < 86400.0)    std::snprintf(tbuf, sizeof(tbuf), "T+%.2f h", t / 3600.0);
        else if (t < 31557600.0) std::snprintf(tbuf, sizeof(tbuf), "T+%.2f d", t / 86400.0);
        else                     std::snprintf(tbuf, sizeof(tbuf), "T+%.2f y", t / 31557600.0);

        Block b; b.x = 8.0f; b.y = 8.0f;
        std::snprintf(line, sizeof(line), "FPS %-4.0f  %s  x%.2g", d.fps, tbuf, d.timeScale);
        b.lines.emplace_back(line);
        std::snprintf(line, sizeof(line), "%s   bodies %d   trails %d%s%s",
                      d.sceneName.c_str(), d.bodyCount, d.trailCap,
                      d.paused ? "   [PAUSED]" : "", d.following ? "   [FOLLOWING]" : "");
        b.lines.emplace_back(line);
        blocks.push_back(std::move(b));
    }

    // --- selection (top-right) ---
    if (d.hasSelection) {
        char line[160];
        Block b; b.x = 0.0f; b.y = 8.0f; b.rightAligned = true;
        std::snprintf(line, sizeof(line), "body #%llu", (unsigned long long)d.selId);
        b.lines.emplace_back(line);
        std::snprintf(line, sizeof(line), "mass  %.3g kg", d.selMass);
        b.lines.emplace_back(line);
        std::snprintf(line, sizeof(line), "speed %.3g km/s", d.selSpeed);
        b.lines.emplace_back(line);
        std::snprintf(line, sizeof(line), "dist  %.3g km", d.selDist);
        b.lines.emplace_back(line);
        b.lines.emplace_back("F follow   Del remove");
        blocks.push_back(std::move(b));
    }

    // --- bottom stack (placing hints + controls), built upward from the bottom edge ---
    float bottomY = H - 8.0f;
    if (d.helpVisible) {
        Block b; b.x = 8.0f; b.y = 0.0f;   // y filled in below
        b.lines = {
            "CONTROLS                              F1 hide",
            "WASD fly        Space / Shift  up / down",
            "RMB drag look   Wheel zoom",
            "LMB click select   LMB drag grab & fling",
            "LMB empty space spawn   wheel depth   RMB grow   Esc cancel",
            "1-4 presets   P pause   . step   = / - speed   0 realtime",
            "F follow   Del remove   [ ] trail length   F5 / F9 / F10 save / load / reset",
        };
        b.y = bottomY - float(b.lines.size()) * kLineH;
        bottomY = b.y - 10.0f;
        blocks.push_back(std::move(b));
    }
    if (d.placing) {
        char line[160];
        Block b; b.x = 8.0f; b.y = 0.0f;
        std::snprintf(line, sizeof(line), "placing mass %.3g kg", d.placeMass);
        b.lines.emplace_back(line);
        b.lines.emplace_back("wheel depth   RMB grow   arrows nudge   release drop   Esc cancel");
        b.y = bottomY - float(b.lines.size()) * kLineH;
        blocks.push_back(std::move(b));
    }

    // ---------------- measure ----------------
    for (auto& b : blocks) {
        float w = 0.0f;
        for (const auto& l : b.lines)
            w = std::max(w, float(stb_easy_font_width(const_cast<char*>(l.c_str()))));
        b.w = w;
        if (b.rightAligned) b.x = W - w - kPad;
        b.panelX = b.x - kPad;
        b.panelY = b.y - kPad;
        b.panelW = w + 2.0f * kPad;
        b.panelH = float(b.lines.size()) * kLineH + 2.0f * kPad;
    }

    // ---------------- emit: all panels first, then all text ----------------
    for (const auto& b : blocks) rect(b.panelX, b.panelY, b.panelW, b.panelH);
    const int panelQuads = vcount / 4;
    for (const auto& b : blocks) {
        for (std::size_t i = 0; i < b.lines.size(); ++i)
            text(b.x, b.y + float(i) * kLineH, b.lines[i].c_str());
    }
    const int textQuads = vcount / 4 - panelQuads;

    // ---------------- draw ----------------
    if (vcount <= 0) return;
    glUseProgram(prog_);
    glUniform2f(uResLoc_, float(d.fbw), float(d.fbh));
    glUniform1f(uScaleLoc_, scale);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(std::size_t(vcount) * 4 * sizeof(float)),
                 buf_.data(), GL_DYNAMIC_DRAW);

    const int maxQuads = int(indices_.size() / 6);
    if (panelQuads > 0) {
        glUniform4f(uColorLoc_, 0.03f, 0.05f, 0.09f, 0.62f);
        glDrawElements(GL_TRIANGLES, std::min(panelQuads, maxQuads) * 6,
                       GL_UNSIGNED_INT, nullptr);
    }
    if (textQuads > 0 && panelQuads < maxQuads) {
        const int drawable = std::min(textQuads, maxQuads - panelQuads);
        glUniform4f(uColorLoc_, 0.82f, 0.90f, 1.00f, 1.0f);
        glDrawElements(GL_TRIANGLES, drawable * 6, GL_UNSIGNED_INT,
                       (void*)(std::size_t(panelQuads) * 6 * sizeof(unsigned)));
    }
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
}

} // namespace gs::render
