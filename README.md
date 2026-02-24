# simSUS — Advanced N-Body Gravitational Simulation Engine

![simSUS Logo](https://img.shields.io/badge/Physics-Relativistic-blueviolet?style=for-the-badge)
![SFML](https://img.shields.io/badge/Graphics-SFML_2.5-green?style=for-the-badge)
![C++](https://img.shields.io/badge/C%2B%2B-20-blue?style=for-the-badge)
![OpenGL](https://img.shields.io/badge/OpenGL-4.3-orange?style=for-the-badge)

A high-performance, precision-focused gravitational simulation engine built with **C++20** and **Modern OpenGL**. `simSUS` (Simulated Universal System) is a comprehensive physics laboratory designed to simulate everything from sub-kilometer asteroids to supermassive black holes across millions of simulated years, featuring real-time visualization, stellar evolution, and cinematic rendering effects.

---

## ✨ Key Features

- **Real-time N-Body Physics**: Simulate thousands of gravitationally interacting bodies with multiple solver algorithms
- **Stellar Evolution**: Watch stars evolve from protostars through main sequence, red giant, and final collapse phases
- **Relativistic Effects**: Gravitational lensing, time dilation, and light bending near massive objects
- **Cinematic Rendering**: Multi-pass shader pipeline with bloom, atmospheric effects, accretion disks, and procedural surfaces
- **Interactive UI**: Real-time body editing, preset scenarios, camera controls, and diagnostic HUD
- **Multiple Integrators**: Choose between RK4, Velocity Verlet, and Symplectic Euler for different accuracy/speed tradeoffs
- **Save/Load System**: Persist and restore simulation states via JSON
- **Comprehensive Presets**: Solar system, binary stars, figure-8 orbits, black holes, nebulae, and more

---

## 🏗 System Architecture

The project follows a clean, modular architecture separating physics, simulation logic, and visualization:

```
simSUS/
├── domain/          # Core data structures (Body, composition, properties)
├── math/            # Vector math utilities (Vec2)
├── physics/         # Gravity solvers and numerical integrators
│   ├── Gravity      # O(N²) direct sum and Barnes-Hut tree
│   ├── Integrators  # RK4, Verlet, Symplectic Euler
│   └── BarnesHut    # Spatial tree optimization
├── sim/             # High-level simulation orchestration
│   ├── Simulation   # Main simulation controller
│   ├── Presets      # Built-in scenarios
│   ├── StellarEvolution  # Stellar lifecycle rules
│   └── EventBus     # Collision and event system
├── render/          # Visualization layer (SFML + OpenGL)
│   ├── BodyRenderer # Multi-pass body rendering
│   ├── Camera       # World ↔ Screen transforms
│   ├── TrailSystem  # Orbital path visualization
│   ├── GridRenderer # Reference grid
│   ├── HUD          # Diagnostic overlay
│   └── Shaders      # GLSL effects (lensing, bloom, atmosphere)
├── app/             # Application lifecycle and UI
│   ├── AppLoop      # Main window and event loop
│   ├── InputHandler # Keyboard/mouse controls
│   ├── AddBodyDialog    # Body creation UI
│   └── BodyEditorPanel  # Property editing
└── io/              # State persistence (JSON serialization)
```

### Architecture Principles:
- **Data-Oriented Design**: `Body` struct is the single source of truth for all physical state
- **Stateless Physics**: Gravity and integrator functions are pure, operating on body vectors
- **Decoupled Rendering**: Visualization is completely separate from physics simulation
- **Event-Driven**: Collision and lifecycle events propagate through EventBus

---

## 🚀 Physics Engine

### Gravity Solvers

simSUS implements multiple gravitational force calculation strategies:

**1. Direct Sum (O(N²))**
- Pairwise force calculation between all bodies
- Exact Newtonian gravity with softening parameter
- Optimal for systems with < 1000 bodies
- Used as default solver

**2. Barnes-Hut Tree (O(N log N))**
- Spatial octree partitioning for distant force approximation
- Configurable theta parameter (default: 0.7) balances speed vs accuracy
- Ideal for large, sparse systems (galaxies, star clusters)
- Reduces computational complexity for 10,000+ body simulations

**3. GPU Compute Shader (Future)**
- Parallel O(N²) implementation using OpenGL compute shaders
- Shared memory tiling for cache optimization
- Target: 10,000+ bodies at 60 FPS

### Numerical Integration

Three integrator options provide different accuracy/performance tradeoffs:

**RK4 (Runge-Kutta 4th Order)** - Default
- Fourth-order accuracy with four force evaluations per step
- Excellent precision for general-purpose simulations
- Best for scenarios requiring high accuracy over moderate timescales

**Velocity Verlet**
- Symplectic integrator preserving energy in periodic systems
- Second-order accuracy with perfect energy conservation
- Ideal for long-term orbital stability (planetary systems)
- Minimal energy drift even over millions of years

**Symplectic Euler**
- First-order symplectic method
- Fast but less accurate
- Suitable for large particle systems where individual precision is less critical

### Physics Configuration

```cpp
PhysicsConfig cfg;
cfg.G = 6.6743e-11;           // Gravitational constant
cfg.softening_m = 1.0e6;      // Softening length (prevents singularities)
cfg.base_dt_s = 3600.0;       // Base timestep (1 hour)
cfg.sub_steps = 8;            // Substeps per frame
cfg.integrator = IntegratorType::RK4;
```

---

## 🌌 Advanced Features

### Stellar Evolution System

Bodies evolve dynamically based on composition, mass, and density:

**Lifecycle Stages:**
- **Protostar**: Collapsing gas cloud, not yet fusing
- **Main Sequence**: Stable hydrogen fusion (like our Sun)
- **Red Giant**: Hydrogen depleted, helium shell burning, massive radius expansion
- **White Dwarf**: Cooling remnant, no active fusion
- **Neutron Star**: Ultra-dense collapsed core (density > 10¹⁷ kg/m³)
- **Black Hole**: Gravitational collapse beyond neutron degeneracy (density > 5×10¹⁷ kg/m³)

**Physical Processes:**
- Hydrogen → Helium fusion (rate: ~2×10⁻¹⁸ per second)
- Composition tracking (H, He, C, O, Fe, Si, ice, rock)
- Temperature-dependent blackbody radiation
- Mass-dependent lifecycle transitions
- Chandrasekhar limit enforcement (~1.4 solar masses)

### Relativistic Effects

**Gravitational Lensing** (`lensing.frag`)
- Schwarzschild metric approximation: α = 4GM/(c²r)
- Light path warping near massive objects
- Einstein ring formation around black holes

**Time Dilation**
- Proper time calculation: dt' = dt√(1 - Rs/r)
- Schwarzschild radius: Rs = 2GM/c²
- Bodies near singularities experience slower time passage

### Collision System

- Automatic collision detection via spatial overlap
- Inelastic merging: momentum and mass conservation
- Composition blending based on mass ratios
- Fragmentation for high-velocity impacts
- Event notifications via EventBus

---

## 🎨 Rendering Pipeline

Multi-pass shader-based rendering system for cinematic visuals:

### Rendering Passes

**1. Scene Pass**
- Body geometry with LOD (Level of Detail) batching
- Procedural stellar surfaces using Fractal Brownian Motion
- Atmospheric halos with density falloff
- Orbital trails and velocity vectors
- Reference grid with adaptive scaling

**2. Specialized Shaders**
- `star_surface.frag`: Boiling plasma with limb darkening and corona
- `accretion_disk.frag`: Relativistic beaming for black hole disks
- `atmosphere.frag`: Atmospheric scattering and glow
- `nebula.frag`: Procedural deep-space background
- `lensing.frag`: Gravitational light bending

**3. Post-Processing**
- Bloom extraction (threshold bright sources)
- Dual-pass Gaussian blur (horizontal + vertical)
- Additive blending for HDR glow effect
- Final composition with UI overlay

### Visual Features

- Dynamic camera with smooth follow mode
- Zoom range: 10⁵ to 10¹² meters per pixel
- Parallax starfield background
- Temperature-based body coloring
- Magnetosphere and aurora visualization
- Planetary rings and atmosphere rendering

---

## 🪐 Built-in Presets

**Solar System**
- Mercury through Neptune with accurate orbital parameters
- Realistic mass ratios and distances
- Stable long-term evolution

**Binary Star**
- Two stars in mutual orbit
- Demonstrates stable binary dynamics

**Figure-8**
- Three-body choreographic orbit
- Famous periodic solution to three-body problem

**Black Hole**
- Massive singularity with accretion disk
- Demonstrates gravitational lensing effects

**Collision**
- High-velocity impact scenario
- Tests collision and merging physics

**Nebula**
- Dense gas cloud with hundreds of particles
- Demonstrates gravitational collapse

**Galaxy (Small)**
- Rotating disk of 1000+ bodies
- Spiral arm formation

**Stellar Death**
- Star undergoing supernova collapse
- Lifecycle transition demonstration

---

## 🛠 Building from Scratch

### Prerequisites

**Required Software:**
- Visual Studio 2019 or 2022 (with C++ desktop development workload)
- Windows 10/11 (64-bit)
- Git (for cloning repository)

**Required Libraries:**
- SFML 2.5+ (installed via NuGet)
- OpenGL 4.3+ capable GPU

### Build Steps

**1. Clone the Repository**
```bash
git clone <repository-url>
cd simPUS
```

**2. Open in Visual Studio**
- Open `simPUS/simPUS.vcxproj` or `simPUS/simPUS.slnx`
- Visual Studio will automatically restore NuGet packages (SFML)

**3. Configure Build**
- Select configuration: `Debug` or `Release`
- Select platform: `x64` (recommended) or `Win32`
- Ensure C++ language standard is set to C++20

**4. Build**
- Build → Build Solution (Ctrl+Shift+B)
- Shaders will automatically copy to output directory

**5. Run**
- Debug → Start Debugging (F5) or Start Without Debugging (Ctrl+F5)

### Command-Line Usage

```bash
# Launch GUI (default)
simSUS.exe

# Run unit tests
simSUS.exe --test

# CLI mode with preset
simSUS.exe --preset solar
simSUS.exe --preset binary
simSUS.exe --preset figure8

# Save/load simulation state
simSUS.exe --save mysim.json
simSUS.exe --load mysim.json
```

---

## 🎮 Controls

### Camera
- **Mouse Drag**: Pan camera
- **Mouse Wheel**: Zoom in/out
- **F**: Follow selected body
- **Escape**: Release camera follow

### Simulation
- **Space**: Pause/Resume
- **+/-**: Increase/decrease time warp
- **R**: Reset to initial state
- **C**: Clear all bodies

### Body Selection
- **Left Click**: Select body
- **Right Click**: Deselect
- **E**: Edit selected body properties
- **Delete**: Remove selected body

### UI
- **Tab**: Toggle HUD
- **G**: Toggle grid
- **T**: Toggle trails
- **H**: Show help

---

## 📊 Performance Characteristics

### Computational Complexity

| Bodies | Direct Sum | Barnes-Hut | Frame Time (est.) |
|--------|-----------|------------|-------------------|
| 100    | O(10⁴)    | O(10²)     | < 1 ms            |
| 1,000  | O(10⁶)    | O(10⁴)     | ~10 ms            |
| 10,000 | O(10⁸)    | O(10⁵)     | ~1000 ms / ~100 ms|

### Optimization Strategies

- Sub-stepping: Multiple physics steps per frame for stability
- LOD rendering: Batch small bodies, detailed shaders for large ones
- Spatial culling: Skip off-screen bodies
- Energy calculation caching: Skip for N > 500 bodies
- Adaptive timestep: Configurable sub-steps and base dt

---

## 🧪 Testing

The project includes comprehensive unit tests for all major components:

```bash
simSUS.exe --test
```

**Test Coverage:**
- `Vec2`: Vector math operations
- `Body`: Physical properties and derived stats
- `Gravity`: Force calculations and energy conservation
- `Integrators`: Numerical accuracy and stability
- `Simulation`: Step logic and collision handling
- `Presets`: Initial condition generation
- `IO`: JSON serialization/deserialization

---

## 🐛 Troubleshooting

**Shaders not loading**
- Ensure shader files are in `render/` directory relative to executable
- Check Visual Studio copies shaders to output directory (see .vcxproj)

**SFML not found**
- Restore NuGet packages in Visual Studio
- Check `packages/` directory contains SFML_VS2019

**Performance issues**
- Reduce number of bodies
- Switch to Barnes-Hut solver for large systems
- Decrease sub-steps
- Disable trails and post-processing effects

**Simulation instability**
- Increase sub-steps
- Reduce time warp
- Switch to RK4 or Verlet integrator
- Check for extremely close bodies (increase softening)

---

## 📚 Further Reading

- [N-Body Problem](https://en.wikipedia.org/wiki/N-body_problem)
- [Barnes-Hut Algorithm](https://en.wikipedia.org/wiki/Barnes%E2%80%93Hut_simulation)
- [Runge-Kutta Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods)
- [Symplectic Integrators](https://en.wikipedia.org/wiki/Symplectic_integrator)
- [Stellar Evolution](https://en.wikipedia.org/wiki/Stellar_evolution)
- [Gravitational Lensing](https://en.wikipedia.org/wiki/Gravitational_lens)
- [SFML Documentation](https://www.sfml-dev.org/documentation/)

---

## 📄 License

This project is open source. See repository for license details.

---

## 👤 Author & Contact

**Author:** patchyevolve  
**GitHub:** https://github.com/patchyevolve  
**Email:** patchyevolve765@gmail.com  
**Project Repository:** https://github.com/patchyevolve/simPUS

**Contributions:** Issues and pull requests are welcome!

---

## 📚 Documentation

For detailed implementation guides:
- **[ProjectHandbook.md](ProjectHandbook.md)** - Complete 300+ page technical guide (36,000+ words)
- **[QUICKSTART.md](QUICKSTART.md)** - Get running in 30 minutes
- **[DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)** - Navigation guide

---

**Built with ❤️ and physics**
