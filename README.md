# Gravity Sim

A small 3D gravity sandbox written in C++ and OpenGL. You drop bodies into space, give them a nudge, and watch them pull on each other. In the main scene, a wireframe grid bends around the bodies — a rough attempt at showing how mass warps spacetime.

## Why I built this

I made this as an experiment combining two things I genuinely care about: physics and C++. I wanted to see whether a physical idea — gravity — could be turned into something you can poke at. Not a textbook animation, but a world where you can spawn a star, fling a moon past it, hold a button and grow a body until the whole grid sinks toward it.

It started with the embarrassingly basic question of whether I could even get a triangle on screen. `3D_test.cpp` is still in the repo as the answer to that question — and it's the whole reason I kept going. The project has a deeper personal meaning to me than most things I've coded. It's the first thing I built where I could *feel* the physics working under my own hands, and that feeling is why I kept coming back to experiment with it instead of moving on to something else.

## What's in the repo

Three programs, in roughly the order I wrote them:

| Program | Source | What it is |
|---|---|---|
| `3D_test` | `3D_test.cpp` | The very first thing that rendered: an orange triangle. A minimal GLFW + GLEW hello-world. |
| `gravity_sim` | `gravity_sim.cpp` | The main sandbox. Two moons and a glowing central star, with the spacetime-bending grid. |
| `gravity_sim_3Dgrid` | `gravity_sim_3Dgrid.cpp` | A simpler Earth + Moon setup on a flat grid. |

## Features

- **Real Newtonian gravity** — every body attracts every other body with F = G·m₁·m₂/r², using the actual gravitational constant.
- **3D free-fly camera** — WASD to move, mouse to look, scroll to zoom.
- **Spawn your own bodies** — left-click creates one in front of the camera; while holding the button, arrow keys place it and right-click grows its mass.
- **Space-time grid** — a wireframe plane whose height is displaced around each body (using its Schwarzschild radius), so heavy objects visibly "sink" the grid around them.
- **Glowing star** — the central body is rendered blown-out, like a bright point source.
- **Pause time** — hold K to freeze the sim and line up a view.

## Requirements

- Windows x64 (developed and tested on Windows 10)
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

The build copies `glfw3.dll` and `glew32.dll` next to each executable automatically. To run the main sim:

```bat
build\Debug\gravity_sim.exe
```

(With the MinGW generator the executable is at `build\gravity_sim.exe`.)

Note: `gravity_sim` starts paused — give K a quick tap to unpause.

## Controls

| Input | Action |
|---|---|
| W / A / S / D | Move the camera |
| Mouse | Look around |
| Scroll | Zoom |
| Space / Left Shift | Move the camera up / down |
| Left click (hold) | Spawn a body in front of the camera and enter "placing" mode |
| Arrow keys (while placing) | Move the new body in the X/Y plane |
| Shift + Arrow keys (while placing) | Move the new body along Z |
| Right click (while placing) | Grow the new body's mass |
| K (hold) | Pause the simulation |
| Q | Quit |

The `gravity_sim_3Dgrid` variant has the same camera and spawn controls, including right-click mass growth while placing a body.

## How the physics works

Every body has a mass, a density, and a derived radius from r = (3m / 4πρ)^(1/3), so its on-screen size scales the way a real sphere of rock or gas would.

Positions live in an abstract "unit" space rather than meters. Distances are multiplied by 1000 when fed into Newton's law, purely so the real gravitational constant produces forces on a workable scale — the exact scaling doesn't matter, as long as the starting velocities are tuned to match. Each frame, every pair of bodies exchanges a gravitational acceleration. Collisions are handled very crudely: velocity is damped and flipped. This is not a carefully integrated N-body simulation; it's a sandbox where the constants were tuned by feel until things moved the way I wanted them to.

The grid bending in `gravity_sim.cpp` is a visualization, not real general relativity. For each body it computes a Schwarzschild radius and displaces the grid vertically with a `sqrt`-based falloff, then the whole grid follows the system's centre of mass. The result is a nice "rubber sheet" effect that reacts live as bodies move and grow.

## Project structure

```
gravity_sim/
├── CMakeLists.txt            # Build script (vendored deps, MSVC + MinGW)
├── gravity_sim.cpp           # Main sandbox with the spacetime grid
├── gravity_sim_3Dgrid.cpp    # Earth + Moon on a flat grid
├── 3D_test.cpp               # First triangle — where it all started
├── third_party/
│   ├── glm/                  # GLM — header-only math library
│   ├── glew/                 # GLEW — headers + x64 DLL / import lib
│   └── glfw/                 # GLFW 3.4 — headers + x64 DLL / import lib
└── .vscode/                  # Build / run tasks for VS Code
```

## Known limitations & possible improvements

Limitations:

- The units are ad hoc: positions are in an abstract space, and velocities are scaled by hand-tuned constants (`/94`, `/96`). Getting a stable Keplerian orbit means hand-picking the starting velocity.
- Collisions flip and damp velocity instead of conserving momentum.
- The grid "bending" is an approximation drawn for effect, not actual general relativity.
- Single-threaded and O(n²) — fine for a handful of bodies, not for a galaxy.
- The build scripts and prebuilt libraries target x64 Windows.

Things I'd love to try next:

- A proper integrator (leapfrog / velocity Verlet) so orbits actually stay closed.
- Preset scenes — a real solar system, binary stars, slingshot maneuvers.
- A small in-window HUD instead of printing mass to the console.
- Spatial partitioning so hundreds of bodies don't cost O(n²).
- Grabbing bodies with the mouse and flinging them.

## Credits

- Code by [Hero00001](https://github.com/Hero00001)
- [GLFW](https://www.glfw.org/) — zlib license (see `third_party/glfw/LICENSE.md`)
- [GLEW](https://glew.sourceforge.net/) — BSD-style license (see `third_party/glew/LICENSE.txt`)
- [GLM](https://github.com/g-truc/glm) — MIT license (see `third_party/glm/copying.txt`)

The project code itself currently has no license file — if you'd like to reuse or build on it, feel free to reach out.
