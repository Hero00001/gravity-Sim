#include "physics/scene.hpp"
#include "physics/world.hpp"
#include "physics/constants.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <random>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

namespace gs {

namespace {
constexpr double kTwoPi = 6.2831853071795864769;

// Place a body on a circular orbit about a central mass M at the origin (orbital
// plane = y = 0). `pos` lies in that plane; tangential velocity is chosen so the
// orbit is circular: v = sqrt(G*M/r).
Body circularOrbit(const glm::dvec3& pos, double M, double m, double density,
                   const glm::vec4& color, bool glow = false) {
    Body b;
    b.mass = m; b.density = density; b.position = pos; b.color = color; b.glow = glow;
    const double r = glm::length(pos);
    const double v = std::sqrt(G * M / r);
    const glm::dvec3 radial = glm::normalize(pos);
    const glm::dvec3 tangent = glm::normalize(glm::cross(glm::dvec3(0, 1, 0), radial));
    b.velocity = tangent * v;
    return b;
}

// HSL (h in [0,1]) -> RGB vec4, for pleasant chaos colors without a dependency.
glm::vec4 hsl(float h, float s, float l) {
    float r, g, b;
    if (s == 0.0f) { r = g = b = l; }
    else {
        auto hue2rgb = [l, s](float p, float q, float t) -> float {
            if (t < 0.0f) t += 1.0f;
            if (t > 1.0f) t -= 1.0f;
            if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
            if (t < 1.0f / 2.0f) return q;
            if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
            return p;
        };
        const float q = (l < 0.5f) ? l * (1.0f + s) : l + s - l * s;
        const float p = 2.0f * l - q;
        r = hue2rgb(p, q, h + 1.0f / 3.0f);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1.0f / 3.0f);
    }
    return glm::vec4(r, g, b, 1.0f);
}
} // namespace

Scene makeSolarSystem() {
    Scene s;
    s.name = "Solar System";
    s.timeScale = 1e6;
    s.grid = {SceneGridMode::Bend, 500000.0, 40};
    s.refRadiusUnits = 450000.0;          // ~ Neptune orbit

    const double M = 1.989e30;
    Body sun; sun.mass = M; sun.density = 1408.0;
    sun.color = {1.0f, 0.929f, 0.176f, 1.0f}; sun.glow = true;
    s.bodies.push_back(sun);

    struct P { double r; double m; double d; glm::vec4 c; };
    const P planets[] = {
        { 5.79e10, 3.30e23, 5429, {0.70f, 0.60f, 0.50f, 1.0f}},  // Mercury
        { 1.082e11, 4.87e24, 5243, {0.90f, 0.80f, 0.50f, 1.0f}}, // Venus
        { 1.496e11, 5.97e24, 5515, {0.20f, 0.50f, 1.00f, 1.0f}}, // Earth
        { 2.279e11, 6.42e23, 3933, {0.80f, 0.30f, 0.20f, 1.0f}}, // Mars
        { 7.785e11, 1.898e27, 1326, {0.80f, 0.70f, 0.50f, 1.0f}}, // Jupiter
        { 1.434e12, 5.68e26, 687,  {0.90f, 0.85f, 0.60f, 1.0f}}, // Saturn
        { 2.871e12, 8.68e25, 1271, {0.60f, 0.80f, 0.90f, 1.0f}}, // Uranus
        { 4.495e12, 1.02e26, 1638, {0.40f, 0.50f, 0.90f, 1.0f}}, // Neptune
    };
    const int n = int(sizeof(planets) / sizeof(planets[0]));
    for (int i = 0; i < n; ++i) {
        const double ang = i * 2.39996323;   // golden-angle spread in the x-z plane
        const glm::dvec3 pos{std::cos(ang) * planets[i].r, 0.0, std::sin(ang) * planets[i].r};
        s.bodies.push_back(circularOrbit(pos, M, planets[i].m, planets[i].d, planets[i].c));
    }
    return s;
}

Scene makeBinaryStars() {
    Scene s;
    s.name = "Binary Stars";
    s.timeScale = 1e5;
    s.grid = {SceneGridMode::Flat, 200000.0, 30};
    s.refRadiusUnits = 80000.0;

    const double m = 0.5 * 1.989e30;        // 0.5 solar masses each
    const double d = 2e11;                  // separation
    const double v = std::sqrt(G * m / (2.0 * d));   // each on circular orbit about COM
    Body a; a.mass = m; a.density = 1408.0; a.color = {0.6f, 0.7f, 1.0f, 1.0f}; a.glow = true;
    a.position = {-d / 2, 0, 0}; a.velocity = {0, 0,  v};
    Body b = a; b.position = {d / 2, 0, 0}; b.velocity = {0, 0, -v};
    s.bodies.push_back(a); s.bodies.push_back(b);

    const double R = 6e11;
    const double vp = std::sqrt(G * (2.0 * m) / R);
    Body p; p.mass = 5.97e24; p.density = 5515.0; p.color = {0.4f, 1.0f, 0.4f, 1.0f};
    p.position = {R, 0, 0}; p.velocity = {0, 0, vp};
    s.bodies.push_back(p);
    return s;
}

Scene makeSlingshot() {
    Scene s;
    s.name = "Slingshot";
    s.timeScale = 1e5;
    s.grid = {SceneGridMode::Bend, 500000.0, 40};
    s.refRadiusUnits = 250000.0;

    const double M = 1.989e30;
    Body star; star.mass = M; star.density = 1408.0;
    star.color = {1.0f, 0.929f, 0.176f, 1.0f}; star.glow = true;
    s.bodies.push_back(star);

    const double rj = 3e11;
    const double vj = std::sqrt(G * M / rj);
    Body j; j.mass = 1.898e27; j.density = 1326.0; j.color = {0.8f, 0.7f, 0.5f, 1.0f};
    j.position = {rj, 0, 0}; j.velocity = {0, 0, vj};
    s.bodies.push_back(j);

    Body probe; probe.mass = 1e3; probe.density = 1000.0; probe.color = {0.9f, 0.9f, 1.0f, 1.0f};
    probe.position = {-2e12, 0, 3e11};
    probe.velocity = {25000.0, 0.0, -8000.0};
    s.bodies.push_back(probe);
    return s;
}

Scene makeChaos() {
    Scene s;
    s.name = "Chaos";
    s.timeScale = 1e4;
    s.grid = {SceneGridMode::Bend, 20000.0, 25};
    s.refRadiusUnits = 8000.0;

    const double diskR = 5e10;
    std::mt19937 rng(12345);                 // fixed seed -> reproducible
    std::uniform_real_distribution<double> ang(0.0, kTwoPi);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_real_distribution<double> lm(20.0, 24.0);   // log10(mass)
    const int N = 40;
    for (int i = 0; i < N; ++i) {
        const double rr = diskR * std::sqrt(u01(rng));
        const double a = ang(rng);
        const glm::dvec3 pos{std::cos(a) * rr, 0.0, std::sin(a) * rr};
        const double sp = 2000.0 * u01(rng);
        const glm::dvec3 vel = glm::normalize(glm::cross(glm::dvec3(0, 1, 0), pos)) * sp;
        const double mass = std::pow(10.0, lm(rng));
        const double dens = 1000.0 + 4000.0 * u01(rng);
        Body b; b.mass = mass; b.density = dens; b.position = pos; b.velocity = vel;
        b.color = hsl(float(i) / float(N), 0.7f, 0.6f);
        s.bodies.push_back(b);
    }
    return s;
}

Scene preset(int n) {
    switch (n) {
        case 1: return makeSolarSystem();
        case 2: return makeBinaryStars();
        case 3: return makeSlingshot();
        case 4: return makeChaos();
        default: return Scene{};
    }
}

bool saveScene(const Scene& s, const std::string& path) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::ofstream out(path);
    if (!out) return false;
    out << std::setprecision(17);
    out << "gsim 1\n";
    out << "timescale " << s.timeScale << "\n";
    const char* mode = s.grid.mode == SceneGridMode::Flat ? "flat"
                     : s.grid.mode == SceneGridMode::Off  ? "off" : "bend";
    out << "grid " << mode << " " << s.grid.sizeUnits << " " << s.grid.divisions << "\n";
    out << "refradius " << s.refRadiusUnits << "\n";
    for (const auto& b : s.bodies) {
        out << "body "
            << b.position.x << " " << b.position.y << " " << b.position.z << " "
            << b.velocity.x << " " << b.velocity.y << " " << b.velocity.z << " "
            << b.mass << " " << b.density << " " << b.radius() << " ";
        out << std::setprecision(9);
        out << b.color.r << " " << b.color.g << " " << b.color.b << " " << b.color.a << "\n";
        out << std::setprecision(17);
    }
    return static_cast<bool>(out);
}

Scene loadScene(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open scene file: " + path);
    Scene s;
    std::string line;
    bool haveGrid = false;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string tok;
        if (!(ss >> tok)) continue;
        if (tok == "gsim") {
            int v = 0; ss >> v; (void)v;
        } else if (tok == "timescale") {
            if (!(ss >> s.timeScale)) throw std::runtime_error("bad timescale line");
        } else if (tok == "grid") {
            std::string m; double sz = 0; int dv = 0;
            if (!(ss >> m >> sz >> dv)) throw std::runtime_error("bad grid line");
            s.grid.mode = m == "flat" ? SceneGridMode::Flat
                        : m == "off"  ? SceneGridMode::Off : SceneGridMode::Bend;
            s.grid.sizeUnits = sz; s.grid.divisions = dv;
            haveGrid = true;
        } else if (tok == "refradius") {
            if (!(ss >> s.refRadiusUnits)) throw std::runtime_error("bad refradius line");
        } else if (tok == "body") {
            Body b; double r = 0;
            if (!(ss >> b.position.x >> b.position.y >> b.position.z
                     >> b.velocity.x >> b.velocity.y >> b.velocity.z
                     >> b.mass >> b.density >> r
                     >> b.color.r >> b.color.g >> b.color.b >> b.color.a))
                throw std::runtime_error("bad body line");
            s.bodies.push_back(b);
        } else {
            throw std::runtime_error("unknown scene token: " + tok);
        }
    }
    if (!haveGrid) throw std::runtime_error("scene missing grid line");
    if (s.bodies.empty()) throw std::runtime_error("scene has no bodies");
    return s;
}

Scene snapshotFromWorld(const World& w, const std::string& name,
                        const SceneGrid& grid, double refRadiusUnits) {
    Scene s;
    s.name = name;
    s.bodies = w.bodies;
    s.timeScale = w.timeScale;
    s.grid = grid;
    s.refRadiusUnits = refRadiusUnits;
    for (auto& b : s.bodies) { b.ghost = false; b.grabbed = false; }
    return s;
}

void applySceneToWorld(World& w, const Scene& s) {
    w.reset();
    w.timeScale = s.timeScale;
    // Real bodies are microscopic next to an astronomical scene (Earth is ~0.6 units
    // across a 450,000-unit solar system), so raise the visual floor to a fraction of
    // the scene's framed radius — otherwise every preset renders as an empty grid.
    w.minVisualRadiusMeters = std::max(3.0e6, s.refRadiusUnits * UNIT * 0.015);
    for (const auto& b : s.bodies) {
        Body c = b;
        c.ghost = false; c.grabbed = false; c.trail.clear();
        w.spawn(c);
    }
}

} // namespace gs
