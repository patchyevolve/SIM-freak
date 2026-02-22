# simSUS — Solar System & N-Body Simulation Engine (C++)

A high-performance, real-time gravitational simulation engine built with C++17 and SFML. `simSUS` simulates celestial bodies under Newtonian gravity with high numerical stability and interactive visualization.

![simSUS Logo](https://raw.githubusercontent.com/patchyevolve/SIM-freak/main/logo.png) <!-- Note: Replace with actual logo link if available -->

## 🚀 Features

- **N-Body Gravity**: Full O(n²) pairwise gravitational interaction.
- **RK4 Integrator**: 4th-order Runge-Kutta numerical integration for high precision.
- **Sub-Stepping**: Performance-tuned sub-stepping for stable simulation at high time warps.
- **Collision Detection**: Inelastic merge policy based on momentum conservation and volume addition.
- **Simulation Presets**: Immediate access to Solar System, Binary Star, and Figure-8 stability showcases.
- **Save/Load System**: Snapshot support using a versioned JSON schema.
- **Interactive UI**: Real-time HUD diagnostics, camera zoom/pan, and orbital path prediction.
- **Hybrid Rendering**: SFML-based window with OpenGL compute shaders for accelerated physics (Beta).

## 🛠 Tech Stack

- **Language**: C++17
- **Graphics**: [SFML](https://www.sfml-dev.org/) (Simple and Fast Multimedia Library)
- **JSON**: [nlohmann/json](https://github.com/nlohmann/json)
- **Physics**: RK4 Numerical Integration
- **Build System**: Visual Studio 2019 / 2022 (NuGet Support)

## 🏗 Build Instructions

### Phase 1: CLI Only
1. Add all `.cpp` files to your C++17 console project.
2. Define `SIMSUS_WINDOW_DISABLED` (if not using SFML).
3. Build and run to see console diagnostics.

### Phase 2: GUI Mode (Full)
1. Ensure SFML is installed. Recommendation: Use **NuGet Package Manager** in Visual Studio and search for `SFML_VS2019`.
2. Ensure `SIMSUS_WINDOW_ENABLED` is defined in `main.cpp`.
3. Include the `render/` and `app/` modules.
4. Build and Run.

## 📖 Usage

### CLI Flags
```bash
simSUS.exe                   # Opens SFML window (Default)
simSUS.exe --test            # Runs internal unit tests
simSUS.exe --cli             # Runs a CLI-only performance demo
simSUS.exe --preset solar    # Load Solar System preset in CLI
simSUS.exe --save <path>     # Export current state to JSON
simSUS.exe --load <path>     # Import state from JSON
```

### Controls (Window Mode)
- **Scroll**: Zoom in/out
- **Middle Click + Drag**: Pan camera
- **Left Click**: Select body for tracking
- **F**: Toggle follow mode on selected body
- **P**: Toggle pause
- **Numbers (1-3)**: Load presets

## 🧪 Testing

The project includes a built-in unit test suite for core modules:
- `Vec2` math verification
- `Body` physics properties
- `Gravity` force calculation
- `Integrator` step accuracy
- `IO` serialization sanity

Run via: `simSUS.exe --test`

## 📂 Project Structure

- `math/`: Vector math utilities (`Vec2`).
- `physics/`: RK4 integrators and gravity laws.
- `domain/`: Object models and body abstractions.
- `sim/`: Simulation management and collision logic.
- `io/`: Save/load state handlers.
- `render/`: SFML & OpenGL rendering systems, trails, and HUD.
- `app/`: Application loop and input handling.

---
Developed by [patchyevolve](https://github.com/patchyevolve)
