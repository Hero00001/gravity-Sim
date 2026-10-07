# Gravity Sim Overhaul — Design Spec

**Date:** 2026-10-07
**Status:** Approved design, pending user spec review
**Scope:** Full rework of the gravity sandbox: physics correctness, 15 bug fixes, 10 improvements, consolidation to a single testable application.

---

## 1. Background & current state

The repo is a C++20/OpenGL 3.0 gravity sandbox with three executables built from `CMakeLists.txt`:

| Target | Source | Role |
|---|---|---|
| `gravity_sim` | `gravity_sim.cpp` (616 lines) | Main sandbox: 2 moons + glowing star, spacetime-bending wireframe grid |
| `gravity_sim_3Dgrid` | `gravity_sim_3Dgrid.cpp` (532 lines) | Earth + Moon on a flat grid — ~90% duplicated code |
| `3D_test` | `3D_test.cpp` | First-ever triangle; historical keepsake |

Physics is O(n²) Newtonian gravity interleaved into the render loop, using a hand-tuned mixed unit system (1 unit = 1000 m, magic divisors `/94` and `/96`, `distance *= 1000`). The README documents this honestly: "constants were tuned by feel."

**Approved direction:** balanced sandbox — coherent physics so orbits are stable on their own, plus playful tools (presets, grab & fling, trails). Consolidate to one sim. Full automated physics tests. All 9 user-selected features + the 10-item improvement list + all 15 bugs fixed.

## 2. Goals / Non-goals

**Goals**

- Coherent, frame-rate-independent physics in pure SI units with a testable GL-free core.
- Stable closed orbits (leapfrog integrator, softening).
- Momentum-conserving collisions (merging).
- All 15 analyzed bugs fixed at root cause, test-first where testable.
- 10 improvements: physics core, merging, presets, HUD, trails, time controls, select/follow, grab & fling, save/load/reset, Barnes-Hut performance.
- One application; automated test suite (`ctest`) guarding physics forever.

**Non-goals**

- No ImGui/GUI toolkit (HUD is custom-rendered text).
- No audio, networking, or relativistic/GR physics.
- No new platforms — Windows x64 + MSVC/MinGW remain the target.
- `3D_test.cpp` stays in the repo but is no longer built.

## 3. Architecture

```
src/
├── physics/              ← GL-free, pure math, double precision
│   ├── body.hpp          Body struct: pos, vel, mass, density, color, trail ring-buffer
│   ├── world.hpp/.cpp    World::step(dt): leapfrog gravity, softening, merges, bounds
│   ├── barnes_hut.hpp/.cpp   octree accelerator (auto-swaps with direct O(n²))
│   └── scene.hpp/.cpp    Scene struct: presets, load/save (plain text), snapshots
├── render/
│   ├── renderer.hpp/.cpp shader + draw bodies / bending grid / trails (reads const World&)
│   ├── camera.hpp/.cpp   free-fly + orbit-follow modes
│   ├── hud.hpp/.cpp      text overlay (stb_truetype font atlas)
│   └── mesh.hpp          sphere grid generation (pure, testable)
└── app/
    ├── main.cpp          single executable: init, main loop, wiring
    └── input.cpp         input state machine: idle / placing / grabbing / selecting

tests/test_physics.cpp    assert-based test exe, registered with CTest
docs/superpowers/specs/   this document
```

**Boundaries**

- `physics/` never includes GL/GLFW/glm-GL headers — that is what makes bugs 1–6 unit-testable.
- `render/` receives `const World&` and draws; owns no simulation state.
- `app/` owns input, camera, and the mutation of the World (spawn/grab/select/place).

**Consolidation:** the new `app/main.cpp` replaces both old sims. `gravity_sim_3Dgrid`'s Earth–Moon scene becomes preset #2; its flat grid becomes a per-scene grid mode. Old monoliths remain built until Phase 9, then are deleted (git history preserves them).

**Dependencies:** existing vendored GLFW/GLEW/GLM unchanged. One addition: `third_party/stb/stb_truetype.h` (single public-domain header) for HUD text. No other new dependencies.

## 4. Physics core

### 4.1 Units

- The World stores **SI only**: meters, kilograms, seconds — as **double**.
- The renderer converts meters → world-units through one constant: `UNIT = 1e7` m per world unit.
  - 1 AU = 14,960 units; Sun radius ≈ 69.6 units; Earth radius ≈ 0.64 units; default grid 20,000 units ≈ 1.3 AU.
- Floats are used only at the render boundary. No physics quantity is stored as float.
- All legacy scale constants (`×1000`, `/94`, `/96`, `sizeRatio`, `/1000000`) are deleted.

### 4.2 Timestep

- Fixed simulation step: `dt = 1/480 s`.
- Accumulator pattern: each render frame adds `frameDelta × timeScale` (frameDelta clamped to ≤ 0.25 s), then runs as many fixed steps as accumulated, capped at **8 steps/frame**. If the cap is hit, leftover time is discarded — the sim slows down under load instead of exploding (spiral-of-death guard).
- `pause == true` ⇒ `World::step` is never called (no integration, no merging, no trail sampling).
- Single-step (while paused) advances exactly one `dt`.

### 4.3 Integrator

Leapfrog kick–drift–kick:

1. `v += a·dt/2`
2. `x += v·dt`
3. recompute `a` from all pairs
4. `v += a·dt/2`

Symplectic ⇒ orbits close instead of secularly decaying.

### 4.4 Gravity & softening

- Newtonian pairwise: `F = G·m₁·m₂ / (r² + ε²)` in magnitude, direction along separation; Plummer form: `a = G·m·r̂ / (r² + ε²)^{3/2}`.
- Per-pair softening: `ε = max(0.1 × (R₁ + R₂), 5e4 m)` (50 km floor), where `R` are **physical** radii.
- Acceleration is accumulated for every body from every other body (Newton's third law holds exactly).

### 4.5 Radii

- `physicalRadius = (3m / (4πρ))^{1/3}` — the single radius formula, in meters, recomputed only when mass changes (spawn growth, merge).
- Physics (collision, softening) uses **physical radius only**.
- Rendering uses `displayRadius = max(physicalRadius, MIN_VISUAL)` with `MIN_VISUAL = 3e6 m` (0.3 world units) so small bodies stay visible. Visual size never feeds back into physics — this kills bug 5 by construction.

### 4.6 Collisions → merging

- After each step: find all pairs with `dist < R₁ + R₂`; merge them:
  - `m = m₁ + m₂`
  - `v = (m₁v₁ + m₂v₂) / m` (momentum conserved exactly)
  - `x = (m₁x₁ + m₂x₂) / m`
  - density = mass-weighted average; survivor's color = color of the heavier body; trail = heavier body's trail.
- Repeat merge passes until no overlaps remain (chain merges).
- Merging happens only inside `World::step` — never while paused.

### 4.7 Barnes-Hut

- Octree rebuilt each step when `bodyCount > 64`; θ = 0.5 opening angle.
- At or below 64 bodies: direct O(n²).
- Both paths cross-tested: max relative acceleration error < 1% on a random 100-body configuration.

### 4.8 Trail sampling

- Ring buffer per body, sampled once per frame **in which the world stepped**, on wall-clock ≥ 33 ms between samples → ≈17 s of visible history at the default 512-point cap, independent of timeScale.
- Cap adjustable with `[` / `]` (stops: 64, 128, 256, 512, 1024, 2048).

### 4.9 Spacetime grid (signature visual, kept)

- Grid configuration is per-scene: `mode ∈ {bend, flat, off}`, `size` (units), `divisions`.
- Base vertices are built **once** per scene config and never mutated — an immutable template. Each frame a pure function `displaceGrid(baseVertices, bodies)` returns a fresh displaced copy (this is the structural fix for bug 6).
- Base plane sits at `y = −0.03 × size` (just below the action, which lives near y = 0). In `bend` mode each vertex dips **downward** (−y): per body, `dip = 2·√(rs·(d − rs))` using the body's Schwarzschild radius `rs = 2Gm/c²` and distance `d` (meters), guarded to skip `d ≤ rs`; total dip = sum over bodies, clamped to `−0.25 × size`. No centre-of-mass following — the sheet's rest level never moves.
- `flat` mode skips displacement; `off` skips drawing. Rendered with `glPolygonOffset` to prevent z-fighting against bodies.

## 5. Bug-fix map

Each testable bug gets a failing test first (TDD), then the root-cause fix. Physics bugs are fixed *while building* the physics module — the module replaces the buggy paths outright.

| # | Bug | Root cause | Fix | Verified by |
|---|---|---|---|---|
| 1 | Sim speed varies with FPS | `/94`, `/96` magic divisors, no dt | Fixed-dt accumulator (§4.2) | CTest: identical world state after runs driven with different frame-delta patterns |
| 2 | Physics runs while paused | collision call outside `if(!pause)` | All stepping lives in `World::step()`; paused ⇒ never called | CTest: paused world bit-identical over 100 step attempts |
| 3 | Collision flips whole velocity, per frame | wrong response model | Momentum-conserving merge (§4.6) | CTest: momentum exact to 1e-9 relative |
| 4 | Slingshot explosions / NaN | 1/r² singularity | Plummer softening (§4.4) | CTest: finite acceleration at r → 0 |
| 5 | Body jumps 33× on release | `/1e6` vs `/sizeRatio` (30000) | One SI radius formula (§4.5); visuals decoupled | CTest: physical radius constant through spawn→place→release→merge lifecycle |
| 6 | Grid slowly sinks/jitters | `UpdateGridVertices` reads last frame's mutated output as the original grid | Immutable base-grid vertices; pure `displaceGrid(base, bodies)` produces fresh output each frame; no COM-following shift | CTest: bodyless grid output identical after 100 invocations |
| 7 | Stuttery WASD | movement handled in event callback | Per-frame `glfwGetKey` polling in `app/input.cpp` | Manual checklist |
| 8 | Shift drops camera while placing Z | Shift = both camera-down and placement Z-modifier | Controls redesign (§7): wheel = placement depth; Shift reserved solely for camera-down | Manual checklist |
| 9 | Pause is a confusing hold-state, starts paused | press/release asymmetry | `P` = toggle (Space stays camera-up, matching existing muscle memory); app starts unpaused with preset loaded | Manual checklist |
| 10 | Window resize breaks viewport/aspect | fixed `glViewport` 800×600, fixed projection aspect | Framebuffer-size callback; viewport + projection aspect recomputed on resize | Manual checklist |
| 11 | Stray ring past south pole | `for (i = 0.0f; i <= stacks; …)` off-by-one | Regenerated sphere mesh in `mesh.hpp` | CTest: exact vertex count; all vertices satisfy `−r ≤ coord ≤ r` |
| 12 | Arrow-key placement nearly invisible | step = 0.2 × the `/1e6` radius (~0.18 units) | Placement redesign (§7): wheel moves along view ray, arrows step 5% of view distance | Manual checklist |
| 13 | First-frame deltaTime spike | `lastFrame = 0.0` vs. elapsed `glfwGetTime()` | Clamp frameDelta ≤ 0.25 s + zero-init on first frame | Code review |
| 14 | ~500 lines duplicated across two monoliths | two independent copies of everything | Consolidation (§3): single app, shared modules | One build target |
| 15 | Dead members, per-frame `glGetUniformLocation`, heap allocs per body-pair | leftover scaffolding | Cleaned during restructure (uniforms cached at init; `glm::vec3` math; buffers reused) | Code review |

## 6. Feature specifications

The 10 improvements. #1 (physics core), #2 (merging), #10 (Barnes-Hut) are specified in §4; #3–#9 below.

### #3 Preset scenes

- Keys `1`–`4` load built-in presets. Loading replaces the World and takes a reset snapshot.
- **1 — Solar System:** Sun + 8 planets (real SI masses; circular-orbit velocities `v = √(GM/r)`; planets on the y = 0 plane). Default `timeScale = 1e6` (~31 s per Earth year). Grid: bending, 500,000 units, 40 divisions. Camera auto-frames to Neptune's orbit.
- **2 — Binary Stars:** two 0.5 M☉ stars in mutual circular orbit + one circumbinary planet. Flat grid. Default `timeScale = 1e5`.
- **3 — Slingshot:** star + Jupiter-mass planet + small probe on a hyperbolic flyby trajectory (velocities pre-computed for a visible gravity assist). Bending grid. `timeScale = 1e5`.
- **4 — Chaos:** N = 40 random bodies in a disk (fixed RNG seed for reproducibility), masses 1e20–1e24 kg, disk radius 5e10 m. Bending grid. `timeScale = 1e4`.
- On load: camera frames the scene's reference radius; HUD shows the scene name.

### #4 HUD overlay

- Top-left block: FPS · sim elapsed time (formatted s / h / d / y) · timeScale (`×1.0e6`) · PAUSED badge · body count · current scene name.
- Right panel when a body is selected: body id, mass (scientific, kg), speed (km/s), distance to the heaviest body (km).
- While placing: live mass readout + control hints (wheel = depth, RMB = grow, release = drop).
- `H` toggles visibility. Rendered with a baked stb_truetype font atlas (one glyph texture, batched colored quads, own shader; drawn last with blending, depth test off).

### #5 Trails / orbit paths

- Per-body ring buffer (§4.8), drawn as `GL_LINE_STRIP` with per-vertex alpha fading toward the tail; drawn before bodies.
- `[` / `]` change cap (§4.8 stops). Trails live in `physics/Body` (testable), rendered by `render/`.

### #6 Time controls

| Key | Action |
|---|---|
| `P` | Pause toggle (starts unpaused) |
| `=` / `-` | timeScale up / down one stop |
| `0` | Reset timeScale to 1× (realtime) |
| `.` | Single-step one `dt` (only while paused) |

- timeScale range 10⁻³ … 10⁷, stops every ×√10 (~20 stops).
- Presets set their own default timeScale; HUD always displays the current value.

### #7 Select / follow / delete

- Picking is **screen-space**: cursor ray unprojected; each body projected to NDC; pick the body whose projected center is nearest the cursor if within `max(20 px, projectedRadiusPx + 8 px)`.
- Click (press + release, pointer moved < 5 px) on a body → select: highlight ring (billboarded circle outline) + right-panel info.
- `F` → follow: camera position locks to the body (keeps its offset/orientation, mouse orbits around it); `F` again releases. `Delete` removes the selected body. Click empty space or `Esc` → deselect.

### #8 Grab & fling

- Press on a body + drag → grab: body moves on the plane through its center, normal = camera forward (ray–plane intersection each move). While held: the body receives no forces (its position is the pointer's), trail sampling is skipped for it — but it **still attracts other bodies** from its current position (dragging a star through space scatters the planets, which is the whole point).
- Release → velocity = smoothed pointer velocity over the last ~100 ms of movement, clamped to 1e8 m/s.
- Works while paused: reposition, aim, release, unpause — the precision tool.
- Press-on-body + drag > 5 px = grab; press-on-body + release < 5 px = select (§7); press on empty space = spawn (§7 placement).

### #9 Save / load / reset

- Plain-text format, `scenes/*.gsim`:

  ```
  gsim 1
  timescale 1000000
  grid bend 500000 40
  body px py pz vx vy vz mass density r g b a
  …
  ```

  Positions/velocities are SI doubles (m), mass kg, density kg/m³, color floats 0–1. Trails are not saved.
- `F5` = quicksave (`scenes/quicksave.gsim`), `F9` = quickload, `F10` = reset to snapshot (taken at preset load / quickload).
- Presets serialize through the identical writer (one code path, round-trip tested).

### #10 Performance

- Barnes-Hut (§4.7) + micro-cleanups: uniform locations cached at init; no per-frame heap allocation (trail buffers, math via `glm::vec3`); dynamic VBOs use buffer orphaning (`glBufferData` with NULL + `glBufferSubData`) for trails.

## 7. Controls (final)

| Input | Action |
|---|---|
| W A S D | Fly camera (polled every frame) |
| Space / Left Shift | Camera up / down |
| Mouse move | Look around |
| Scroll | Dolly forward/back |
| LMB press on empty space | Spawn body in front of camera → placing mode |
| · wheel (while placing) | Move body along view ray |
| · arrow keys (while placing) | Move in view plane, step = 5% of view distance |
| · RMB hold (while placing) | Grow mass (`×e²` per second held) |
| · Esc (while placing) | Cancel spawn |
| · release LMB | Finalize body (zero velocity) |
| LMB press on body + drag | Grab; release = fling |
| LMB click on body (< 5 px) | Select |
| F / Delete / Esc | Follow-toggle / delete selected / deselect |
| P / = / - / 0 / . | Pause toggle / faster / slower / realtime / single-step |
| 1 2 3 4 | Load preset |
| F5 / F9 / F10 | Quicksave / quickload / reset |
| H | HUD toggle |
| [ / ] | Trail length shorter / longer |
| Q | Quit |

Modifier discipline: **Shift means camera-down only.** No input has two meanings; all state transitions (idle → placing → …) live in `app/input.cpp`.

Spawn defaults: mass `1e22` kg, color `(0.9, 0.4, 0.2)`, zero velocity, density `3344 kg/m³` (rock). RMB-hold growth doubles mass every ≈0.35 s (single growth path — the old per-click ×1.2 path is removed); the HUD shows the live mass while placing.

## 8. Testing & verification

- **Framework:** minimal assert macros in `tests/test_physics.cpp` (no external test framework), registered via CMake `add_test`; run with `ctest --test-dir build`.
- **CTest coverage (physics + pure functions):**
  1. Circular orbit closes within 0.1% radial error after 1,000 steps (leapfrog + units).
  2. Total momentum conserved across a merge to 1e-9 relative.
  3. Energy drift bounded (< 1% over 1,000 steps; leapfrog has no secular drift).
  4. No NaN/Inf acceleration at r → 0 (softening).
  5. Paused world is bit-identical after 100 step attempts.
  6. Same final state from two different frame-delta patterns (FPS independence).
  7. Physical radius constant through spawn → grow → release → merge.
  8. Grid displacement output identical across 100 calls on a bodyless world; deterministic given inputs.
  9. Sphere mesh: exact vertex count; every coordinate within [−r, +r].
  10. Scene save → load → save round-trip byte-identical.
  11. Barnes-Hut vs direct solver: max relative error < 1% on 100 random bodies.
  12. Merge chain: three overlapping bodies in one step end as one body with conserved momentum.
- **Manual checklist** (bugs 7–13 + UX): camera smoothness, resize, pause toggle, placement (wheel/arrows/RMB/Esc), select, follow, grab-fling, HUD, trails, time controls, all 4 presets, save/load/reset, quit.
- **Phase gate:** clean build (no new warnings) + all ctests green + phase checklist + demo to the user before a phase is called done.

## 9. Delivery phases

Each phase is independently buildable and demo-able. Commits happen per green phase (asked of the user first).

| Phase | Content | Delivers |
|---|---|---|
| 1 | Scaffold `src/` layout, CMake test wiring, minimal `gravity_sim_v2` main (fly-cam, draw spheres, spawn) | Structure + `ctest` runs (even if trivially) |
| 2 | Physics core §4 (SI units, accumulator, leapfrog, softening, merging) + tests | Bugs 1–5 fixed; improvements #1, #2 |
| 3 | Render rework: mesh, pure grid displacement, resize handling, trails | Bugs 6, 10, 11, 13; improvement #5 |
| 4 | Input/camera overhaul: polling, pause toggle, placement redesign, modifier cleanup | Bugs 7–9, 12; controls per §7 |
| 5 | Select / follow / grab & fling | Features #7, #8 |
| 6 | HUD + time controls | Features #4, #6 |
| 7 | Presets + save/load/reset | Features #3, #9 |
| 8 | Barnes-Hut + performance pass | Feature #10 |
| 9 | Consolidation: delete old monoliths, rename `gravity_sim_v2` → `gravity_sim`, drop `3D_test` from build, README rewrite, final full checklist + full test run | Ship state |

Old targets (`gravity_sim`, `gravity_sim_3Dgrid`, `3D_test`) build untouched through Phases 1–8 and are removed in Phase 9.

## 10. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Depth-buffer z-fighting at solar-system scale | Per-scene near/far planes; `glPolygonOffset` on the grid; MIN_VISUAL radius floor |
| Renderer accidentally feeds visuals back into physics | Boundary rule: physics reads only SI fields; `displayRadius` exists only in `render/` (reviewed in code review, structurally separate) |
| timeScale × 10⁷ overruns the 8-step cap constantly | Cap exists by design: sim slows under load; HUD shows effective behavior; dt is small (1/480) so 8 steps ≈ 60 ms of sim per frame at worst |
| stb_truetype vendoring | Single public-domain header, same vendored pattern as existing deps; verified file header on add |
| Scope creep during phases | Non-goals list (§2) is binding; new ideas go into a follow-up spec |
