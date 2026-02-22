# simSUS — Solar System & N-Body Simulation Engine (C++)

A high-performance, real-time gravitational simulation engine built with C++17 and SFML. `simSUS` simulates celestial bodies under Newtonian gravity with high numerical stability, advanced physics solvers, and high-fidelity interactive visualization.

![simSUS Logo](https://raw.githubusercontent.com/patchyevolve/SIM-freak/main/logo.png) <!-- Note: Replace with actual logo link if available -->

## 🚀 Physics & Solvers

- **Dual Gravity Kernels**:
  - **O(n²) Direct Sum**: High-accuracy pairwise interactions for small to medium counts.
  - **O(n log n) Barnes-Hut**: Optimized quadtree-based gravity solver for massive body counts (O(n log n) complexity).
- **Pluggable Integrators**:
  - **RK4 (Runge-Kutta 4th Order)**: Default high-precision integrator.
  - **Symplectic Euler**: Energy-conserving first-order integrator for fast simulations.
  - **Velocity Verlet**: Balanced stability and speed with good long-term energy behavior.
- **Sub-Stepping**: Performance-tuned temporal sub-stepping for stable simulation at high time-warps.
- **Inelastic Collisions**: Momentum-conserving merge policy with volume-additive mass integration.

## 🪐 Simulation Presets

Equipped with 7 built-in initial condition generators:
1. **Solar System**: Accurate SI-unit approximation of the inner and outer planets.
2. **Binary Star**: Stable orbital dance of two high-mass bodies.
3. **Figure-8**: Classic restricted 3-body solution showing gravitational choreography.
4. **Black Hole**: High-mass singularity with relativistic visual distortion.
5. **Collision**: High-speed impact scenario designed to test merge logic.
6. **Nebula**: Large-scale chaotic cloud collapse.
7. **Small Galaxy**: Spiral-arm formation with thousands of star-mass particles.

## 🎨 Advanced Rendering & UI

- **High-Fidelity Visuals**:
  - **Gravitational Lensing**: Real-time GLSL fragment shaders for relativistic light bending around black holes.
  - **Atmosphere & Rings**: Specialized rendering routines for gas giants and terrestrial planets.
  - **Sphere of Influence (SOI)**: Visualized radius of gravitational dominance for massive bodies.
  - **Trail System**: Alpha-faded temporal trails tracking historical orbital paths.
- **Adaptive Visualization**:
  - **LOD (Level of Detail) Batching**: Efficiently render thousands of bodies using vertex batching for distant objects.
  - **Orbit Prediction**: Real-time orbital path projection based on current state.
  - **Coordinate Grid**: Deformable spatial grid for orientation and diagnostics.
- **Interactive Controls**:
  - **Camera**: Smooth zoom-to-cursor and middle-drag panning.
  - **Follow Mode**: Lock camera to any selected body (ideal for planetary tracking).
  - **Selection HUD**: Real-time diagnostics (mass, velocity, surface gravity, periapsis estimate).

## 🛠 Tech Stack

- **Language**: C++17
- **Graphics & Windowing**: [SFML 2.5+](https://www.sfml-dev.org/)
- **GPU Acceleration**: GLSL (Shaders for Lensing, Grids, and Compute)
- **Serialization**: [nlohmann/json](https://github.com/nlohmann/json)
- **Build**: Visual Studio 2019/2022 (NuGet optimized)

## 🏗 Build & Run

### Build
1. Open the solution in **Visual Studio**.
2. Install **SFML** via NuGet (`SFML_VS2019` or `SFML_VS2022`).
3. Ensure **C++17 Standrard** is enabled in Project Properties.
4. Build in **Release** mode for performance.

### CLI Usage
```bash
simSUS.exe                   # GUI Mode (Default: Solar System)
simSUS.exe --test            # Run all internal unit tests
simSUS.exe --preset <type>   # Load specific preset (binary, figure8, blackhole, nebula, etc.)
simSUS.exe --save <path>     # Snapshot current state to JSON
simSUS.exe --load <path>     # Resume simulation from JSON
```

### Controls
- **Scroll**: Zoom (preserves focus)
- **Middle Click**: Pan
- **Left Click**: Select body
- **F**: Toggle Follow Mode
- **P**: Pause
- **Numbers (1-7)**: Switch Presets

## 🧪 Modules
- `math/`: Fast `Vec2` primitives and vector algebra.
- `physics/`: Integrators, BH Tree, and Gravity laws.
- `domain/`: Unified `Body` and `SimulationState` models.
- `sim/`: Collision resolution, event dispatcher, and presets.
- `render/`: Quadtree batching, GLSL shaders, camera, and HUD systems.

---
Developed by [patchyevolve](https://github.com/patchyevolve)
