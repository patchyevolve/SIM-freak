# simSUS Documentation Index

## Overview

This project includes comprehensive documentation at three levels:

| Document | Purpose | Size | Best For |
|----------|---------|------|----------|
| **QUICKSTART.md** | Get running fast | 4 KB | First-time builders |
| **README.md** | Project overview | 16 KB | Understanding features |
| **ProjectHandbook.md** | Complete guide | 235+ KB | Deep learning |

---

## QUICKSTART.md (4 KB, 95 lines)

**Purpose:** Get a working simulation in 30 minutes

**Contents:**
- Prerequisites checklist
- 5-step build order
- Minimal code samples
- Quick troubleshooting
- **New**: Controls for Slingshot and HUD diagnostics

**Use when:**
- You want to see results immediately
- You're following along with the handbook
- You need a quick reference

---

## README.md (16 KB, 330 lines)

**Purpose:** Professional project overview and reference

**Contents:**
1. **Introduction** - What is simSUS?
2. **Key Features** - **Updated** with Thermodynamics, Tidal Heating, and Shader Robustness
3. **Architecture** - How is it organized?
4. **Physics Engine** - Gravity solvers and integrators
5. **Advanced Features** - Stellar evolution, relativistic effects (Z-shift), and tidal stress
6. **Rendering Pipeline** - Visualization system with GPU handle validation
7. **Built-in Presets** - Ready-to-run scenarios
8. **Building from Scratch** - Setup instructions
9. **Controls** - **Updated** with Slingshot mechanics
10. **Performance** - Optimization strategies (LOD, Culling)
11. **Testing** - Quality assurance
12. **Troubleshooting** - Common issues
13. **Further Reading** - External resources

**Use when:**
- You want to understand what simSUS can do
- You need to explain the project to others
- You're looking for specific features

---

## ProjectHandbook.md (235+ KB, 7,000+ lines, 33,000+ words)

**Purpose:** Complete educational guide from zero to expert

### PART I: CONCEPTUAL FOUNDATION

**Chapter: Conceptual Foundation**
- What are we really building?
- The N-body problem explained
- Why numerical approximation?

**Chapter: The Big Picture**
- Complete data flow diagram
- How components connect
- Architecture principles

### PART II: DEEP DIVES

**Phase 1: Mathematical Foundation - Vec2**
- Why we need vectors (with examples)
- Why double precision matters (astronomical scales)
- Complete implementation with explanations

**Phase 2: The Body - Domain Model**
- Design philosophy: state vs derived
- The "alive" flag pattern
- Single source of truth principle

**Phase 3: Gravity - Physics Engine**
- Newton's law conceptually explained
- The softening parameter (why and how)
- The O(N²) problem visualized

**Phase 6.5: Robust Shader Management (NEW)**
- Native handle validation (`getNativeHandle`)
- Graceful fallbacks to CPU rendering
- Preventing console spam and driver hangs

**Phase 8.1: Relativistic Visuals (NEW)**
- Gravitational Redshift math
- Doppler Shift (Radial velocity)
- Combined Z-shift calculation

**Phase 8.5: Thermodynamics & Tidal Heating (NEW)**
- The physics of heat and tidal stress
- Exponential decay logic for stable transitions
- Roche limit and tidal disruption mechanics

**Phase 8.7: Orbital Mechanics (NEW)**
- Sphere of Influence (SOI) math ($r_{soi} = a(m/M)^{0.4}$)
- Orbit Prediction via Shadow Simulation

**Phase 9: Performance & Optimization (UPDATED)**
- Barnes-Hut Tree (O(N log N))
- Section 9.2: Advanced Rendering Optimizations (Viewport Culling, Tiered LOD Diamond Batching)
- Visual Polish: Cubic trail fading math
