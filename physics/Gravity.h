#pragma once
// =============================================================================
//  physics/Gravity.h  — Newtonian gravity calculations
// =============================================================================

#include "../domain/Body.h"
#include <vector>

namespace Gravity
{
    // ─────────────────────────────────────────────────────────────────────────
    /// O(n²) pairwise force accumulation.
    /// Writes computed `accel` directly into each body in the vector.
    /// `softening_m`  — epsilon to avoid singularities (see README §4.1)
    void compute_accelerations(std::vector<Body>& bodies,
                               double G,
                               double softening_m);

    // ─────────────────────────────────────────────────────────────────────────
    /// Acceleration on body `i` from body `j` (single pair).
    Vec2 acceleration_from(const Body& i, const Body& j,
                           double G, double softening_m);

    // ─────────────────────────────────────────────────────────────────────────
    /// Total linear momentum of all alive bodies (kg·m/s).
    Vec2   total_momentum(const std::vector<Body>& bodies);

    /// Total kinetic energy (J).
    double total_kinetic_energy(const std::vector<Body>& bodies);

    /// Total gravitational potential energy (J).
    double total_potential_energy(const std::vector<Body>& bodies,
                                  double G, double softening_m);

    /// Total mechanical energy = KE + PE (J).
    double total_energy(const std::vector<Body>& bodies,
                        double G, double softening_m);

    // ─────────────────────────────────────────────────────────────────────────
    /// GPU Management
    bool InitGPU();
    void Cleanup();
    bool IsGpuReady();

    bool RunTests();
}
