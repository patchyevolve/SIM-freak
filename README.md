# simSUS — Solar System & N-Body Simulation Engine (C++)

A high-performance, real-time gravitational simulation engine built with C++17 and SFML. `simSUS` is a multi-layered physics laboratory featuring relativistic rendering, stellar evolution, and scalable N-body solvers.

- [GitHub Repository](https://github.com/patchyevolve/SIM-freak)

## 🚀 Physics & Solvers

- **Dual Gravity Kernels**:
  - **O(n²) Direct Sum**: High-accuracy pairwise interactions for small to medium counts.
  - **O(n log n) Barnes-Hut**: Quadtree-based solver for massive simulations (10,000+ bodies).
- **Stellar Evolution Suite**:
  - **Composition-Based Aging**: Bodies track Hydrogen, Helium, Silicates, and Iron.
  - **Relativistic Lifecycle**: Automated transitions from Main Sequence → Red Giant → Neutron Star/Black Hole based on core density and mass limits (Chandrasekhar-ish).
  - **Thermodynamics**: Surface temperature and luminosity modeling.
- **Advanced Dynamics**:
  - **Pluggable Integrators**: RK4, Symplectic Euler, and Velocity Verlet.
  - **Collision & Fragmentation**: Inelastic merging OR high-energy fragmentation/shattering based on impact velocity.
  - **Passive Body Optimization**: `is_passive` flag for massive particles (e.g., dust clouds, galactic arms) that respond to gravity without exerting it.

## 🪐 Simulation Presets

1. **Solar System**: Accurate SI-unit approximation of the inner and outer planets.
2. **Binary Star**: Stable orbital dance of two high-mass bodies.
3. **Figure-8**: Classic restricted 3-body solution showing gravitational choreography.
4. **Black Hole**: Gargantua-class singularity with relativistic light bending.
5. **Collision**: High-speed impact scenario designed to test merge and fragmentation logic.
6. **Nebula**: Large-scale chaotic cloud collapse with 2000+ particles.
7. **Small Galaxy**: Spiral-arm formation with 10,000+ particles utilizing Barnes-Hut.

## 🎨 High-Fidelity Rendering

- **Relativistic Visuals**:
  - **Gravitational Lensing**: Real-time GLSL light bending and event horizon distortion.
  - **Accretion Effects**: Accretion disk glows and intensity halos for singularities.
  - **Atmosphere & Rings**: Multi-layered atmospheric scattering and orbital ring rendering.
  - **Sphere of Influence (SOI)**: Visualized dominance zones for gravitational capture.
- **Adaptive HUD & Analysis**:
  - **Live Diagnostics**: Real-time Energy (J), Momentum (kg·m/s), and Sim-time tracking.
  - **Orbit Predictor**: Forward-projected path lines for all active bodies.
  - **Body Editor**: Runtime adjustment of mass, composition, and visual style.
  - **LOD Batching**: Vertex-batched rendering for massive particle counts.

## 🛠 Tech Stack

- **C++17 Core**: Strict SI-unit physics and data-oriented design.
- **SFML 2.5+**: Windowing, input, and 2D graphics.
- **OpenGL/GLSL**: Fragment shaders for lensing, grids, and future compute acceleration.
- **nlohmann/json**: Versioned snapshot system for state persistence.

## 🏗 Build & Usage

```bash
# GUI Mode (Default)
simSUS.exe

# Management & Performance
simSUS.exe --test            # Run core math/physics unit tests
simSUS.exe --preset <name>   # load binary, figure8, blackhole, nebula, galaxy
simSUS.exe --save <path>     # Snapshot to JSON
simSUS.exe --load <path>     # Resume from JSON
```

- **Scroll**: Zoom focus | **Middle Drag**: Pan | **Left Click**: Select
- **F**: Follow Mode | **P**: Pause | **1-7**: Switch Presets

---
Developed by [patchyevolve](https://github.com/patchyevolve)
