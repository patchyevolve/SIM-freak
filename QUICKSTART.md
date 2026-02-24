# simSUS Quick Start Guide
## Get Your Simulation Running in 30 Minutes

This is the express version of the ProjectHandbook.md. Follow these steps to get a working simulation FAST.

---

## Prerequisites (5 minutes)

1. **Install Visual Studio 2022 Community** (free)
   - Download: https://visualstudio.microsoft.com/downloads/
   - Select: "Desktop development with C++"
   - Install

2. **Create Project**
   ```
   Visual Studio → New Project → Console App (C++)
   Name: simSUS
   ```

3. **Install SFML**
   ```
   Tools → NuGet Package Manager → Manage NuGet Packages
   Browse → Search "SFML_VS2019" → Install
   ```

4. **Configure Project**
   ```
   Right-click project → Properties
   C/C++ → General → Additional Include Directories: $(ProjectDir)..
   C/C++ → Language → C++ Language Standard: ISO C++20
   ```

---

## Build Order (25 minutes)

### Step 1: Vec2 (5 min)

Create `math/Vec2.h` and `math/Vec2.cpp` from handbook Chapter 2.

Test in main.cpp:
```cpp
#include "math/Vec2.h"
int main() {
    return Vec2::RunTests() ? 0 : 1;
}
```

### Step 2: Body (5 min)

Create `domain/Body.h` and `domain/Body.cpp` from handbook Chapter 3.

Add to main.cpp:
```cpp
#include "domain/Body.h"
// Add Body::RunTests() call
```

### Step 3: Gravity (5 min)

Create `physics/Gravity.h` and `physics/Gravity.cpp` from handbook Chapter 4.

### Step 4: Integrators (5 min)

Create `physics/Integrators.h` and `physics/Integrators.cpp` from handbook Chapter 5.

### Step 5: Visualization (5 min)

Create:
- `sim/Presets.h` and `sim/Presets.cpp`
- `sim/Simulation.h` and `sim/Simulation.cpp`
- `render/Camera.h` and `render/Camera.cpp`
- `render/BodyRenderer.h` and `render/BodyRenderer.cpp`

Replace main.cpp with the visual version from handbook Chapter 6.

---

## Run It!

Press **Ctrl+F5**

You should see:
- Yellow Sun in center
- Blue Earth orbiting

**Controls:**
- Space: Pause/Resume
- Mouse Wheel: Zoom
- Left Drag: Pan

---

## Troubleshooting

**Black screen?**
- Check bodies are added: `std::cout << sim.bodies().size();`
- Try zooming out (scroll wheel)

**Build errors?**
- Check Additional Include Directories: `$(ProjectDir)..`
- Ensure all .cpp files are included in project

**SFML errors?**
- Verify NuGet package installed
- Check build configuration (Debug/Release)

---

## Next Steps

See **ProjectHandbook.md** for:
- Detailed explanations of every concept
- Advanced features (stellar evolution, Barnes-Hut)
- Optimization techniques
- Debugging strategies

See **README.md** for:
- Project overview
- Feature list
- Architecture details

---

## Author & Contact

**Author:** patchyevolve  
**GitHub:** https://github.com/patchyevolve  
**Email:** patchyevolve765@gmail.com

**Questions?** Open an issue at https://github.com/patchyevolve/simPUS/issues or check the full documentation!

---

**Congratulations! You built a working N-body simulation!**

Now read the handbook to understand WHY it works and HOW to extend it.
