# simSUS Technical Handbook

This handbook serves as the definitive reference for the `simSUS` simulation engine, detailing its multi-layered architecture, physics solvers, and high-fidelity rendering pipeline.

## 🏗 System Architecture

The project is structured into distinct modules to separate concerns between data, simulation, and visualization:

- **`domain/`**: Core entity models. `Body.h` contains the physical state and composition.
- **`physics/`**: The "Heart" of the engine. Contains numerical integrators and gravitational solvers.
- **`sim/`**: High-level simulation logic, presets, and the `StellarEvolution` rules engine.
- **`render/`**: Visualisation layer using SFML and modern GLSL.
- **`app/`**: Application lifecycle, GUI components, and input orchestration.
- **`io/`**: YAML/JSON persistence for simulation snapshots.

---

## 🚀 Physics & Gravitational Solvers

`simSUS` uses a hybrid solver system to balance extreme performance with precision:

### 1. The Titan Solver (GPU)
For $N \ge 512$, the engine dispatches a **tiled O(N²)** gravity kernel to the GPU.
- **Performance**: Optimized for 10,000+ bodies at 60 FPS.
- **Precision**: Uses shared memory tiling to minimize memory bandwidth bottlenecks.

### 2. Barnes-Hut (CPU)
An O(N log N) tree-based solver used as a fallback or for medium-scale simulations.
- **Theta ($\theta$)**: Configured to 0.7 for a balance of speed and accuracy.

### 3. Numerical Integrators
Selectable via `Simulation.h`:
- **RK4 (Runge-Kutta 4th Order)**: The default high-precision solver.
- **Verlet**: Symplectic integrator for energy conservation over long periods.
- **Symplectic Euler**: High-speed, low-precision fallback.

---

## 🌌 Advanced Mechanics

### Relativistic Time Dilation
The engine implements **Schwarzschild & Special Relativity** proximity effects.
- Time passes slower for bodies near high-mass singularities (Black Holes).
- `Simulation.cpp` calculates the `local_time_scale` for every body each frame.

### Stellar Evolution
Stars are not static. `StellarEvolution.pp` simulates:
- **Fusion**: Hydrogen → Helium conversion over millions of simulated years.
- **Lifecycle Phases**: Automatic transitions (Main Sequence → Red Giant → White Dwarf/Neutron Star).
- **Collapse**: High-mass bodies collapsing into Black Holes based on density thresholds.

---

## 🎨 High-Fidelity Rendering

The rendering pipeline uses a multi-pass approach to achieve a cinematic look:

1.  **Scene Pass**: Renders bodies, trails, and background nebula to a texture.
2.  **Relativistic Pass**: Applies Gravitational Lensing and light warping via `lensing.frag`.
3.  **Bloom Pass**: Extracts high-intensity highlights and applies a Gaussian blur.
4.  **UI Overlay**: Draws the HUD and Editor panels on top.

### Specialized Shaders
- **`star_surface.frag`**: High-contrast boiling plasma with volumetric corona.
- **`accretion_disk.frag`**: Simulates Doppler beaming and brightness asymmetry for black holes.
- **`nebula.frag`**: Procedural fractal noise for a dynamic deep-space background.

---

## 🛠 Developer Utilities

- **Unit Testing**: Run `simSUS.exe --test` to verify all math and physics modules.
- **Spatial Hashing**: Efficient collision detection even with 10k+ particles.
- **Diagnostic HUD**: Real-time monitoring of Energy Drift ($E_{drift}$), GPU utilization, and zoom scales.
