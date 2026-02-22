#pragma once
// =============================================================================
//  physics/BarnesHut.h  — O(n log n) quadtree gravity solver
//
//  Build a BH tree for N bodies: O(n log n).
//  Query gravitational acceleration: O(log n) per body.
//  Opening-angle theta (default 0.5) trades accuracy for speed.
//  For N <= BH_DIRECT_THRESHOLD the O(n²) direct sum is used instead
//  (more accurate and faster for small N).
// =============================================================================

#include "../domain/Body.h"
#include <vector>

// ── Opening angle threshold ───────────────────────────────────────────────────
static constexpr double BH_THETA = 0.5;        // Lower = more accurate, slower
static constexpr int    BH_DIRECT_THRESHOLD = 64; // Below this N, use O(n²)

// ── Internal quadtree node ────────────────────────────────────────────────────
struct BHNode
{
    double cx = 0.0, cy = 0.0;   // Centre-of-mass position
    double mass = 0.0;            // Total mass in this cell
    double x0, y0, x1, y1;       // Bounding box [x0,x1) × [y0,y1)
    double size = 0.0;            // Max(width, height) — for theta test
    int    children[4] = {-1,-1,-1,-1}; // Quadrant children: NW, NE, SW, SE
    int    body_idx = -1;         // Index into bodies array if leaf; -1 otherwise
    bool   is_leaf = true;
};

// ── Barnes-Hut tree ───────────────────────────────────────────────────────────
class BHTree
{
public:
    /// Build the quadtree from alive bodies.
    void build(const std::vector<Body>& bodies);

    /// Compute gravitational acceleration on body at index `i`
    Vec2 acceleration_on(int i, const std::vector<Body>& bodies,
                         double G, double softening_m) const;

    /// Reset — called automatically by build()
    void clear() { m_nodes.clear(); }

    int  node_count() const { return static_cast<int>(m_nodes.size()); }

private:
    std::vector<BHNode> m_nodes;

    int  new_node();
    void insert(int node_idx, int body_idx, const std::vector<Body>& bodies);
    void compute_mass(int node_idx, const std::vector<Body>& bodies);
    Vec2 accel_recursive(int node_idx, const Body& b,
                         double G, double softening_m) const;
};
