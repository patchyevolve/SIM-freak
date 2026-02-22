# simSUS — Advanced N-Body Gravitational Engine (C++)

![simSUS Logo](https://img.shields.io/badge/Physics-Relativistic-blueviolet?style=for-the-badge)
![SFML](https://img.shields.io/badge/Graphics-SFML_2.5-green?style=for-the-badge)
![Compute Shaders](https://img.shields.io/badge/Accelerator-Titan_GPU-orange?style=for-the-badge)

A high-performance, precision-focused gravitational simulation engine built with **C++17** and **Modern OpenGL**. `simSUS` (Simulated Solar Universal System) is designed to handle everything from accurate SI-unit solar systems to chaotic 10,000-body galactic collisions using a hybrid CPU/GPU architecture.

---

## 🚀 The Titan Core (Physics Engine)

`simSUS` utilizes a sophisticated multi-stage physics pipeline to ensure both performance and numerical stability:

### 1. Hybrid Solve Architecture
The engine dynamically selects the most efficient solver based on simulation scale:
- **GPU Titan Kernel (Compute Shader)**: A tiled O(N²) parallel solver that leverages OpenGL Compute Shaders. It optimizes memory bandwidth using shared memory tiling to simulate **10,000+ bodies** at a locked 60 FPS.
- **Barnes-Hut Quadtree**: An O(N log N) fallback for high-particle counts on systems without compute shader support. Uses a recursive quadtree with a $\theta = 0.7$ opening criterion.
- **Direct Sum (O(N²))**: High-precision CPU solver for low body counts, ensuring zero-approximation accuracy.

### 2. Advanced Integration
- **Relativistic Time Dilation**: Real-time implementation of **Schwarzschild & Special Relativity**. Bodies experience time dilation ($local\_time\_scale$) based on their velocity and proximity to high-mass singularities (Black Holes).
- **Sub-stepping & Variable Time-Steps**: Automatically adapts simulation steps (up to 128x sub-steps) to prevent "tunneling" during high-velocity close encounters.
- **Collision & Fragmentation**: High-speed impacts can trigger **fragmentation**, where a body splinters into a debris cloud of passive particles, conserving both mass and momentum.

---

## 🎨 High-Fidelity Cinematic Rendering

Visuals in `simSUS` are driven by a multi-pass GLSL pipeline designed for "cinematic physics":

- **Gravitational Lensing**: Full Schwarzschild light-bending simulation. Black holes dynamically warp the background stars and accretion disks into Einstein rings.
- **Accretion Disk Dynamics**: Procedural rendering of black hole disks with **Doppler Beaming** (asymmetric brightness reflecting relativistic motion).
- **Boiling Stellar Surfaces**: High-contrast FBM (Fractal Brownian Motion) shaders for stars, featuring animated solar flares, granulation, and volumetric coronas.
- **Multi-Pass Bloom**: A sophisticated post-processing stack (Threshold -> Gaussian Blur H/V -> Additive Composite) that makes stellar objects feel high-energy and luminous.
- **Dynamic Nebula Background**: A deep-space fractal noise background that responds to camera position for a parallax-rich experience.

---

## 🧬 Stellar Evolution & Composition

Unlike standard N-body simulations, `simSUS` tracks the life and death of stars:

- **Elemental Tracking**: Every body has a composition (Hydrogen, Helium, Silicates, Iron, etc.) that dictates its physical behavior.
- **Lifecycle Suite**:
  - **Protostar Phase**: Gravitational collapse of gas clouds (passive bodies).
  - **Main Sequence**: Hydrogen-to-Helium fusion cycles affecting surface temperature and luminosity.
  - **Red Giant Transition**: Dramatic expansion and cooling as core fuel depletes.
  - **Death Remnants**: Depending on mass, stars collapse into **White Dwarfs**, **Neutron Stars**, or **Black Holes** when density thresholds are breached.

---

## 🪐 Simulation Presets

1.  **Solar System**: Exact SI-unit distances and masses for all major planets and the Sun.
2.  **Binary Star Dance**: A stable, high-mass orbital solution demonstrating RK4 precision.
3.  **Black Hole Interstellar**: A Gargantua-class singularity with 1000 surrounding particles.
4.  **Megascale Galaxy**: Clean 10,001 particle spiral formation spanning thousands of AUs.
5.  **Chaotic Nebula**: 2,000+ particles in a collapsing gas cloud.

---

## 🛠 Developer & CLI Interface

`simSUS` provides a robust CLI for technical diagnostics:

```bash
# Standard Launch
simSUS.exe

# Technical Diagnostics
simSUS.exe --test            # Run massive internal unit test suite (Physics/Integrators/Math)
simSUS.exe --preset galaxy   # Boot directly into the megascale galaxy simulation
simSUS.exe --load state.json # Resume a previously saved orbital snapshot
```

### Controls Handbook
- **Scroll**: Adaptive Zoom | **Middle-Drag**: Pan focus
- **Left-Click**: Select Body | **F**: Follow/Target Mode
- **Space**: Pause | **[ / ]**: Warp Time (0.5x to 80x+)
- **1-7**: Instant Preset Switching
- **A**: Add Body Mode | **S/L**: Save/Load State
- **H**: Toggle HUD Help

---

## 🏗 Dependencies & Tech
- **Core**: C++17, SFML 2.5.1
- **Physics**: OpenGL 4.3+ (Compute Shaders), nlohmann/json
- **Math**: Custom precision `Vec2` library with O(1) distance caching.

Developed by [patchyevolve](https://github.com/patchyevolve)
