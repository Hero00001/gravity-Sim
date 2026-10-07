# Gravity Sim

A 3D gravity sandbox written in C++20 and OpenGL 3.3. You drop bodies into space, give them a nudge, and watch real Newtonian gravity pull them around. A live wireframe grid bends around the heavy bodies — a rough, visual take on how mass warps spacetime — and you can pause, rewind, load preset systems, or fling a planet past a star with the mouse.

## Why I built this

I made this as an experiment combining two things I genuinely care about: physics and C++. I wanted to see whether a physical idea — gravity — could be turned into something you can poke at. Not a textbook animation, but a world where you can spawn a star, fling a moon past it, hold a button and grow a body until the whole grid sinks toward it.

It started with the embarrassingly basic question of whether I could even get a triangle on screen. The very first thing I rendered was an orange triangle (`3D_test.cpp`, kept in the repo as a historical hello-world even though it is no longer part of the build) — and that little triangle is the whole reason I kept going. The project has a deeper personal meaning to me than most things I've coded. It's the first thing I built where I could *feel* the physics working under my own hands, and that feeling is why I kept coming back to experiment with it instead of moving on to something else.

## Features

- **Real Newtonian gravity in SI units** — every body attracts every other with F = G·m₁·m₂/r², using the actual gravitational constant and double-precision metres/kilograms. Orbits are physically correct, not hand-tuned.
- **Leapfrog (kick-drift-kick) integration** — a symplectic integrator, so orbits stay closed and energy doesn't drift over long runs.
- **Plummer softening** — a smooth ε softening (scaled to each pair's radii, with a 50 km floor) keeps the force finite at contact and stops singularities when bodies pass through each other.
- **Momentum-conserving merges** — when two bodies overlap they coalesce into one: mass and momentum are conserved, density is mass-weighted, and the larger body's colour survives.
- **Barnes-Hut O(n log n) gravity** — for more than 64 bodies an octree approximation replaces the O(n²) all-pairs sum, with θ = 0.5 and a cross-checked, sub-1% error against the exact solver (CTest #11).
- **3D free-fly camera** — WASD to move, mouse to look, scroll to zoom.
- **Spawn, select, follow, grab & fling** — left-click empty space drops a new body; left-click a body selects it (a yellow ring marks it); drag a body to grab and reposition it, and release to fling it with the pointer's velocity.
- **Space-time grid** — a wireframe grid whose height is displaced around each body (using its Schwarzschild radius with a `sqrt` falloff), so heavy objects visibly "sink" the grid. Modes: **bend**, **flat**, or **off**.
- **In-window HUD** — rendered with `stb_easy_font`: FPS, mission time, time-scale, body count, current scene, and selected-body readouts (mass, speed, distance). Toggle with **H**.
- **Time controls** — pause/step (**P** / **.**), and a geometric time-scale dial (**=** speeds up by ×√10, **−** slows by /√10, **0** returns to real time). Useful for watching a solar system tick in seconds or a galaxy drift for millennia.
- **Preset scenes** — four one-key systems: **1** Solar System, **2** Binary Stars, **3** Slingshot, **4** Chaos disk.
- **Save / load / reset** — **F5** quicksaves the current world to `scenes/quicksave.gsim`, **F9** reloads it, **F10** snaps back to the preset/loaded state you started from. Scenes use a plain-text `.gsim` format.

## Requirements

- Windows x64 (developed and tested on Windows 10/11)
- CMake 3.16 or newer
- A C++20 compiler — Visual Studio 2022 (verified) or MinGW-w64 (MSYS2)
- An OpenGL 3.3 capable GPU / driver

GLFW, GLEW and GLM are **vendored** in `third_party/`, so there is nothing to install before building — the project uses those copies directly.

## Building

From the repo root, with Visual Studio:

```bat
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Debug
```

With MinGW instead:

```bat
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build
```

The build copies `glfw3.dll` and `glew32.dll` next to the executable automatically. To run the sim:

```bat
build\Debug\gravity_sim.exe
```

(With the MinGW generator the executable is at `build\gravity_sim.exe`.)

The app launches straight into the **Solar System** preset and is already running — press **P** to pause.

## Controls

| Input | Action |
|---|---|
| W / A / S / D | Move the camera |
| Space / Left Shift | Move the camera up / down |
| Mouse | Look around |
| Scroll | Zoom (or push/pull a body you're placing) |
| **1** / **2** / **3** / **4** | Load preset: Solar System / Binary Stars / Slingshot / Chaos |
| Left click (empty space) | Spawn a body in front of the camera; hold and it stays "placing" |
| Arrow keys (while placing) | Nudge the new body in the camera X/Y plane |
| Right-click hold (while placing) | Grow the new body's mass |
| Left release (while placing) | Drop the body into the world |
| Esc (while placing) | Cancel the placement |
| Left click (on a body) | Select it (yellow ring) |
| Drag (on a body) | Grab and move it; release to fling with pointer velocity |
| **F** | Toggle camera follow on the selected body |
| Delete | Remove the selected body |
| **P** | Pause / resume the simulation |
| **.** (period) | Single-step the physics while paused |
| **=** / **−** | Increase / decrease the time scale (×√10 steps) |
| **0** | Reset the time scale to real time (×1) |
| **H** | Toggle the HUD |
| **[** / **]** | Decrease / increase trail length |
| **F5** / **F9** / **F10** | Quicksave / quickload / reset to snapshot |
| **Q** | Quit |

## How the physics works

Every body has a mass, a density, and a derived radius from r = (3m / 4πρ)^(1/3), so its on-screen size scales the way a real sphere of rock or gas would. State is stored in SI units as `double`: positions in metres, masses in kilograms, with `UNIT = 1e7` m/unit for the render mapping.

The simulation advances with a fixed internal timestep (`SIM_DT = 1/480 s`) and a leapfrog kick-drift-kick (KDK) integrator, so it is **frame-rate independent** — 60 fps and 480 fps stepping produce the same trajectory (CTest #1). Each frame's real elapsed time is clamped to 0.25 s and split into at most 8 fixed sub-steps, with any remainder carried into the next frame.

Accelerations use Plummer softening: `a = G·m·d / (r² + ε²)^(3/2)`, where `ε = max(0.1·(R₁+R₂), 5e4 m)`. For ≤ 64 bodies the exact O(n²) all-pairs sum runs; above that an octree (Barnes-Hut, θ = 0.5) approximates distant clusters by their centre of mass. Both paths are cross-checked to agree to better than 1% (CTest #11).

When two bodies overlap they merge: the survivor keeps the combined mass and momentum, its density is the mass-weighted mean, and the heavier original body's colour is retained. Ghost bodies (the body currently being placed, or one you're dragging) exert no force and receive none.

The grid bending is a **visualization, not real general relativity**. For each body it computes a Schwarzschild radius and displaces the grid vertically with a `sqrt`-based falloff, then the whole grid follows the system's centre of mass. The result is a "rubber sheet" effect that reacts live as bodies move and grow.

## Scene file format (`.gsim`)

Scenes are plain text, one body per line, easy to read and diff:

```
gsim 1
timescale 1e+06
grid bend 500000 200
body px py pz vx vy vz mass density r g b a
...
```

- `gsim 1` — format version.
- `timescale` — the time multiplier the scene was saved with.
- `grid` — `bend|flat|off`, grid half-size (units), divisions.
- `body` — position (m), velocity (m/s), mass (kg), density (kg/m³), and RGBA colour (0–1).

Save → load → save is byte-identical (CTest #10).

## Project structure

```
gravity_sim/
├── CMakeLists.txt            # Build script (vendored deps, MSVC + MinGW)
├── 3D_test.cpp               # Historical hello-world (not built; see "Why I built this")
├── src/
│   ├── app/
│   │   ├── main.cpp          # GLFW window, callback wiring, main loop
│   │   ├── input.cpp/.hpp    # camera, spawn/select/follow/grab, presets, save/load
│   ├── render/
│   │   ├── renderer.cpp/.hpp  # draw path: bodies, grid, trails, HUD, selection
│   │   ├── mesh.cpp/.hpp      # sphere mesh generation
│   │   ├── grid.cpp/.hpp      # grid geometry + spacetime displacement
│   │   ├── hud.cpp/.hpp       # stb_easy_font overlay
│   │   ├── camera.hpp         # fly/follow camera
│   ├── physics/
│   │   ├── world.cpp/.hpp     # body store, leapfrog step, merges, accelerations
│   │   ├── scene.cpp/.hpp     # presets, save/load (.gsim), snapshot apply
│   │   ├── barnes_hut.cpp/.hpp# octree + O(n log n) acceleration
│   │   ├── body.hpp           # Body struct (mass, density, colour, trail)
│   │   ├── constants.hpp      # G, UNIT, etc.
├── tests/
│   ├── test_physics.cpp       # leapfrog, softness, merges, scenes, Barnes-Hut cross-check
│   ├── test_render.cpp        # mesh / grid geometry checks
├── third_party/
│   ├── glm/                   # GLM — header-only math library
│   ├── glew/                  # GLEW — headers + x64 DLL / import lib
│   └── glfw/                  # GLFW 3.4 — headers + x64 DLL / import lib
└── .vscode/                  # Build / run tasks for VS Code
```

## Tests

The project ships two CTest suites:

```bat
ctest --test-dir build --build-config Debug
```

- **physics** — frame-rate independence, sub-step clamping, pause determinism, softening safety, orbit closure, momentum conservation, momentum-conserving merges, ghost-body isolation, energy boundedness, trail ring behaviour, preset sanity, scene round-trip byte-equality, and the Barnes-Hut vs direct-solver cross-check (< 1% max relative error, θ = 0.5).
- **render** — mesh and grid geometry sanity.

## Known limitations & possible improvements

Limitations:

- Single-threaded and CPU-only; Barnes-Hut keeps hundreds of bodies smooth but a million-body galaxy would need a GPU port.
- The grid "bending" is an approximation drawn for effect, not actual general relativity.
- The build scripts and prebuilt libraries target x64 Windows.
- Quicksave is a single slot (`scenes/quicksave.gsim`); there is no named-slot save browser yet.

Things I'd love to try next:

- A proper relativistic correction toggle for the grid (e.g. a real Schwarzschild embedding) rather than the current heuristic.
- Named save slots and a simple scene browser.
- A GPU (compute-shader) Barnes-Hut pass for true large-N scales.
- Custom orbit-insertion tools (e.g. "drop a body on a circular orbit around the selected body").

## Credits

- Code by [Hero00001](https://github.com/Hero00001)
- [GLFW](https://www.glfw.org/) — zlib license (see `third_party/glfw/LICENSE.md`)
- [GLEW](https://glew.sourceforge.net/) — BSD-style license (see `third_party/glew/LICENSE.txt`)
- [GLM](https://github.com/g-truc/glm) — MIT license (see `third_party/glm/copying.txt`)
- [stb_easy_font](https://github.com/nothings/stb) — public domain (Sean Barrett), used for the HUD

The project code itself currently has no license file — if you'd like to reuse or build on it, feel free to reach out.
