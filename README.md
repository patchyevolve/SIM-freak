# simSUS — Solar System & N-Body Simulation Engine (Language-Agnostic Blueprint)

A real-time gravitational simulation with rendering, camera controls, object creation, collisions, presets, save/load, and HUD diagnostics.

This README is intentionally **implementation-focused** so you can rebuild a cleaner, production-quality version of this project in **any language** (Python, Rust, C++, TypeScript, Java, Go, etc.) while keeping the same behavior and architecture.

---

## 1) What This Project Does

simSUS simulates celestial bodies under Newtonian gravity and displays them in an interactive 2D scene.

Core capabilities:
- N-body gravity (all bodies attract all other bodies).
- RK4 numerical integration with sub-stepping for stability.
- Time warp and pause.
- Collision detection + inelastic merge.
- Multiple presets (Solar System, Binary Star, Figure-8).
- Selection panel with body stats.
- Add-object workflow for runtime body creation.
- Save/load complete simulation state as JSON.
- Optional energy/momentum diagnostics.

---

## 2) Product Goals for a “Polished” Rewrite

When rebuilding this in another language, optimize for:

1. **Correctness**: predictable numerics, deterministic stepping, unit consistency (SI).
2. **Performance**: smooth rendering at 60 FPS for moderate body counts.
3. **Extensibility**: easy to add body types, integrators, UI features.
4. **User Experience**: responsive camera controls, clear HUD, recoverable save/load.
5. **Testability**: decouple simulation core from rendering/input.

---

## 3) Domain Model (Portable Across Languages)

## 3.1 Body abstraction
Every simulated object should expose:
- `name`
- `mass` (kg)
- `radius` (m)
- `position` (m, 2D vector)
- `velocity` (m/s, 2D vector)
- `color` or render style
- optional metadata (`body_type`, temperature, atmosphere, fuel, etc.)

Derived values:
- speed, distance, kinetic energy, orbital period estimate, surface gravity.

Suggested portable schema:

```text
Body {
  id: UUID
  name: string
  kind: enum { Star, Planet, Moon, Asteroid, Spacecraft, Custom }
  mass_kg: float64
  radius_m: float64
  pos: Vec2
  vel: Vec2
  render: RenderProps
  flags: BodyFlags
  extra: map<string, scalar>
}
```

## 3.2 Simulation state
```text
SimulationState {
  bodies: List<Body>
  sim_time_s: float64
  real_time_s: float64
  paused: bool
  time_warp: float64
  config: PhysicsConfig
}
```

## 3.3 Config model
```text
PhysicsConfig {
  G: float64                 // usually 6.67430e-11
  softening_m: float64       // avoids singularities in close passes
  base_dt_s: float64         // simulated seconds per real second at 1x
  sub_steps: int
  trail_interval: int
  collision_policy: enum
}
```

---

## 4) Physics Engine Design

## 4.1 Force law
Use Newtonian gravity in SI units:

- Force magnitude: `F = G * m1 * m2 / r^2`
- Acceleration on body i from j: `a_i += G * m_j * r_ij / (|r_ij|^2 + eps^2)^(3/2)`

Where:
- `r_ij = (pos_j - pos_i)`
- `eps = softening_m`

Softening prevents unstable spikes when two bodies are nearly coincident.

## 4.2 Integration
Current project uses RK4. Keep that by default in the polished rewrite.

Recommended integrator interface:

```text
integrate(state, dt, acceleration_fn) -> new_state
```

Support pluggable methods:
- RK4 (default)
- Symplectic Euler (fast)
- Velocity Verlet (good energy behavior)

## 4.3 Time stepping strategy
Per frame:
1. Convert real frame delta to simulated delta using `base_dt * time_warp`.
2. Split into `sub_steps` chunks.
3. Integrate each chunk.
4. Resolve collisions.
5. Emit events and update diagnostics.

This keeps user-facing warp independent from rendering FPS.

## 4.4 Collision model
Simple inelastic merge policy:
- Trigger when center distance <= sum of radii.
- Merge into one body:
  - `mass = m1 + m2`
  - `velocity = (m1*v1 + m2*v2) / (m1 + m2)` (momentum conservation)
  - radius by volume add (assuming same density):
    - `r_new = (r1^3 + r2^3)^(1/3)`
- Choose style/name by heavier body or deterministic rule.

---

## 5) Architecture for a Professional Rewrite

Use a layered design:

```text
[Input + UI Layer]
        |
[Application Layer / Use Cases]
        |
[Simulation Engine Core]
        |
[Math + Physics Utilities]
```

### 5.1 Core modules
- `math/vec2`: immutable vectors, dot, norm, normalize.
- `physics/gravity`: acceleration, energy, momentum.
- `physics/integrators`: RK4 + alternatives.
- `domain/body`: body types and serialization.
- `sim/simulation`: stepping, collision, events, time warp.
- `sim/presets`: known initial conditions.
- `io/state`: save/load JSON schema + versioning.
- `render/*`: camera + body renderer + trails.
- `ui/*`: HUD, menus, selection.

### 5.2 Event system
Expose typed events from simulation core:
- `BodyAdded`
- `BodyRemoved`
- `CollisionMerged`
- `EnergyDriftExceeded`
- `PresetLoaded`

UI subscribes; core remains UI-agnostic.

### 5.3 Determinism guidelines
- Use fixed simulation tick path (even if render FPS varies).
- Avoid unordered map iteration in physics loops.
- Seed randomness for preset-generated objects.
- Include floating-point tolerance in tests.

---

## 6) Rendering & Camera Blueprint

## 6.1 World-space vs screen-space
- Physics uses meters (world coordinates).
- Camera maps world <-> screen with `meters_per_pixel`.
- Zoom and pan should preserve cursor focus when zooming.

## 6.2 Visual polish checklist
- Anti-aliased circles/orbits where available.
- Consistent color palette and typography.
- Trail fade effect (alpha by age).
- Selection halo + label + velocity vector.
- Graceful clipping/culling for off-screen objects.
- Frame-time and sim-time overlays.

## 6.3 Input behavior
- Scroll: zoom.
- Middle drag: pan.
- Left click: select.
- Follow mode toggles selected body lock.
- Keyboard for pause/warp/preset/save/load/add object.

---

## 7) Save/Load Format (Versioned)

Use a versioned JSON schema from day one:

```json
{
  "schema_version": 1,
  "sim": {
    "sim_time_s": 0,
    "paused": false,
    "time_warp": 1.0,
    "config": {
      "G": 6.6743e-11,
      "softening_m": 1000000.0,
      "base_dt_s": 3600.0,
      "sub_steps": 8
    }
  },
  "bodies": []
}
```

Migration strategy:
- Add converters `v1 -> v2 -> v3`.
- Never silently drop unknown fields.
- Validate before load, return actionable errors.

---

## 8) Presets & Data Strategy

Recommended preset types:
1. **Solar system (approximate)** for familiarity.
2. **Binary star + planet** for emergent dynamics.
3. **Figure-8 3-body** as a stability showcase.

Store preset definitions in data files (`json/yaml/toml`) where possible, not only code.

---

## 9) Testing Strategy (Language-Agnostic)

Minimum automated suite:

1. **Vector math tests**
   - add/subtract/scale/norm/normalize.
2. **Gravity tests**
   - two-body equal/opposite forces.
   - acceleration magnitude sanity checks.
3. **Integrator regression tests**
   - one step against known expected values.
4. **Conservation trend tests**
   - total momentum near-constant without collisions.
   - energy drift bounded over short run.
5. **Collision tests**
   - overlap detection.
   - merge mass/momentum/radius rules.
6. **Serialization tests**
   - save then load equals original (within tolerance).
7. **Preset smoke tests**
   - each preset steps for N ticks without NaNs/Infs.

For UI-enabled stacks, add screenshot tests for HUD regressions.

---

## 10) Performance Roadmap

If body count grows, use staged optimizations:

1. Baseline O(n²) pairwise interactions.
2. SIMD/vectorization for force loops.
3. Spatial partitioning / Barnes-Hut for large N.
4. Parallel force computation (thread pool / GPU).
5. Separate physics tick from render tick.

Benchmarks to track:
- ms per simulation step vs body count.
- max stable body count at 60 FPS.
- memory usage per body + trail segment.

---

## 11) Porting Guide by Language Family

### Rust / C++
- Strongly type units or wrap scalar types to avoid meter/km confusion.
- Prefer ECS or data-oriented arrays for performance.
- Use serde/json or nlohmann/json for snapshots.

### TypeScript / Web
- Canvas2D for simplicity, WebGL for scale.
- Keep physics in a Worker to avoid UI jank.
- Persist saves in IndexedDB/localStorage.

### Java / Kotlin / C#
- Keep core physics in pure library module.
- Use JavaFX/Swing/MonoGame/Unity front-end adapters.

### Go
- Keep simulation engine deterministic and single-threaded first.
- Add goroutine parallelism only after profiling.

### Python rewrite improvement
- Numpy arrays for vectorized force calculation.
- Optional Numba/Cython acceleration.

---

## 12) Suggested Implementation Phases

Phase 1 — **Core physics library**
- Vec2, Body, gravity, RK4, stepping.
- CLI demo (no UI) that prints orbital stats.

Phase 2 — **Rendering & camera**
- Draw bodies, zoom/pan, selection.

Phase 3 — **Gameplay UX**
- HUD panels, notifications, controls, add-object flow.

Phase 4 — **Persistence & presets**
- Save/load, schema validation, built-in presets.

Phase 5 — **Hardening**
- Tests, benchmarks, profiling, docs, CI.

Phase 6 — **Polish**
- Better visuals, orbits, accessibility settings, localization.

---

## 13) Definition of Done for “Purely Polished”

A rewrite is considered polished when all are true:
- Stable 60 FPS at target body count.
- No crashes from invalid input/save files.
- Deterministic stepping from same seed.
- Save/load compatibility guaranteed by schema versioning.
- Test suite + CI passing.
- Clear user controls/help panel.
- Codebase modular with API docs for core engine.

---

## 14) Quick Start for This Existing Python Project

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

Run tests:

```bash
pytest -q
```

---

## 15) Recommended Next Upgrades (if continuing this repo)

1. Introduce strict JSON schema validation for saves.
2. Add configurable integrator selection in UI.
3. Add orbital path predictor overlays.
4. Add replay/record feature.
5. Add CI workflow with lint + tests + artifact screenshots.

---

If you use this README as a blueprint, you can recreate simSUS in virtually any language with cleaner architecture, better reliability, and production-ready maintainability.
