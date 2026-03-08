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
- **Thermodynamics & Tidal Heating**: Realistic temperature transitions and internal friction heating from gravitational stress
- **Relativistic Effects**: Gravitational lensing, time dilation, and light bending near massive objects
- **Cinematic Rendering**: Robust multi-pass shader pipeline with bloom, atmospheric effects, accretion disks, and procedural surfaces
- **Interactive UI**: Real-time body editing, preset scenarios, camera controls, and an enhanced diagnostic HUD with bold headers and a full preset list
- **Slingshot Tool**: Precision orbital insertion with visual prediction and calibrated fixed velocity multipliers for physical stability
- **Tiered LOD Batching**: High-performance rendering system that batches distant stars into single vertex arrays while maintaining high-fidelity shaders for close-up objects
- **Robust Distribution**: Ready-to-use Windows installer with full dependency management and automated setup
- **Multiple Integrators**: Choose between RK4, Velocity Verlet, and Symplectic Euler for different accuracy/speed tradeoffs
- **Save/Load System**: Persist and restore simulation states via JSON
- **Comprehensive Presets**: 8 built-in scenarios covering solar systems, binary stars, figure-8 orbits, black holes, nebulae, galaxies, and stellar evolution events

---

## 🛠 Getting Started

### Installation (Windows)
1. Open the local [releases/](file:///d:/codeWorks/simPUS/releases) folder in the project root.
2. Run `simPUS_Setup.exe` to install the simulation (includes SFML DLLs and shaders).
3. Once installed, launch `simPUS` from your Desktop or Start Menu.

### Building from Source (Developers)
- **Compiler**: C++20 compatible (MSVC 2022+ recommended)
- **Dependencies**: SFML 2.5.1+, OpenGL 4.3+
- **Build System**: Visual Studio Solution included (`simPUS.slnx`)
- **Distribution**: Use [Inno Setup](https://jrsoftware.org/) with the included [simPUS_installer.iss](file:///d:/codeWorks/simPUS/simPUS_installer.iss). The compiled installer is automatically placed in the `releases/` folder.

---

## 🎮 Controls & Interface

The simulation features an enhanced **Help Menu (H)** with high-contrast headers and a full preset guide.

| Key | Action |
|:---:|:---|
| **H** | **Toggle Enhanced Help Menu** (Blue headers, bold text) |
| **Scroll** | Precision Zoom (500 km/px to 100 AU/px) |
| **Mid-drag** | Smooth Camera Pan |
| **Left Click** | Select Body |
| **Space** | Pause/Resume Simulation |
| **[ / ]** | Adjust Time Warp (x0.25 to x1,000,000x multiplier) |
| **1 - 8** | **Load Presets**: (Solar, Binary, Fig-8, BH, Collision, Nebula, Galaxy, Death) |
| **F** | Focus/Follow selected body |
| **C** | Clear all trails and orbit predictions |
| **A** | **Add Body Mode**: Click to place a body; click again to edit properties |
| **S / L** | Quick Save/Load simulation state |

---

## 🏗 System Architecture

The project follows a clean, modular architecture separating physics, simulation logic, and visualization:

```
simSUS/
├── releases/        # Local installer distribution folder
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
- **Performance Optimized**: 
    - **Tiered LOD Batching**: Distant celestial bodies are batched into a single `VertexArray` for O(1) draw calls.
    - **Squared-Distance Math**: Proximity checks eliminate costly `sqrt` operations across the physics engine.
    - **Hoisted Lookups**: Simulation loops minimize redundant lookups by caching state references.
    - **Robust Uniforms**: Every GLSL uniform update is validated via `getNativeHandle()` to prevent driver overhead.

---

## 🌌 Physics & Thermodynamics

### Stable Thermodynamics Model
`simSUS` utilizes an **Exponential Decay Model** for surface temperature transitions:
$T_{new} = T_{target} + (T_{old} - T_{target}) \cdot e^{-dt/\tau}$

This ensures perfectly stable temperature behavior even at extreme simulation speeds ($1,000,000x$), eliminating the oscillations common in linear transition models.

### Tidal Stress & Heating
Bodies passing within the **Roche Limit** of a more massive object experience tidal stress. This stress contributes to:
- **Tidal Heating**: Internal friction converted to thermal energy.
- **Disruption**: Bodies may fragment into smaller asteroids if stress exceeds the structural limit.

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
- **Thermodynamics**: Surface temperature calculation based on distance to stars and albedo
- **Tidal Heating**: Internal friction heat generated by gravitational stress (`tidal_stress`)
- **Exponential Decay**: Smooth temperature transitions that remain stable even at high time warps
- Composition tracking (H, He, C, O, Fe, Si, ice, rock)
- Temperature-dependent blackbody radiation
- Mass-dependent lifecycle transitions
- Chandrasekhar limit enforcement (~1.4 solar masses)

### Relativistic Effects

**Gravitational Lensing** (`lensing.frag`)
- Schwarzschild metric approximation: α = 4GM/(c²r)
- Light path warping near massive objects
- Einstein ring formation around black holes

**Relativistic Color Shifts**
- **Gravitational Redshift**: Light losing energy when escaping deep gravity wells (Potential-based).
- **Doppler Shift**: Color shifts based on radial velocity relative to the camera center.
- **Combined Z-Shift**: Integrated calculation for realistic celestial body coloring.

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

**4. Robustness & Validation**
- **Shader Handle Verification**: Every uniform update (`setUniform`) is protected by `getNativeHandle()` checks to prevent driver crashes.
- **Graceful Fallbacks**: If a specialized shader fails to compile, the engine automatically reverts to high-performance legacy CPU rendering.
- **LOD Management**: Dynamic Level-of-Detail system batches thousands of distant stars while using advanced shaders for high-interest bodies.

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
