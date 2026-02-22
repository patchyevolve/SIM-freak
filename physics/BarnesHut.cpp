// =============================================================================
//  physics/BarnesHut.cpp  — O(n log n) quadtree gravity solver implementation
// =============================================================================

#include "BarnesHut.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cassert>

static constexpr int MAX_BH_NODES = 1 << 22; // 4M nodes — sufficient for 1M bodies

// ── Allocate a new node index ─────────────────────────────────────────────────
int BHTree::new_node()
{
    m_nodes.emplace_back();
    return static_cast<int>(m_nodes.size()) - 1;
}

// ── Build the tree ─────────────────────────────────────────────────────────────
void BHTree::build(const std::vector<Body>& bodies)
{
    m_nodes.clear();
    m_nodes.reserve(bodies.size() * 4);

    // Find bounding box of all alive bodies
    double x0 =  std::numeric_limits<double>::max();
    double y0 =  std::numeric_limits<double>::max();
    double x1 = -std::numeric_limits<double>::max();
    double y1 = -std::numeric_limits<double>::max();

    bool any = false;
    for (const auto& b : bodies) {
        if (!b.alive) continue;
        x0 = std::min(x0, b.pos.x); y0 = std::min(y0, b.pos.y);
        x1 = std::max(x1, b.pos.x); y1 = std::max(y1, b.pos.y);
        any = true;
    }

    if (!any) return;

    // Pad by 10% to avoid edge collisions
    double pad = std::max((x1 - x0), (y1 - y0)) * 0.05 + 1.0;
    x0 -= pad; y0 -= pad; x1 += pad; y1 += pad;

    // Create root
    int root = new_node();
    m_nodes[root].x0 = x0; m_nodes[root].y0 = y0;
    m_nodes[root].x1 = x1; m_nodes[root].y1 = y1;
    m_nodes[root].size = std::max(x1 - x0, y1 - y0);

    // Insert all alive, non-passive bodies that have mass
    for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
        if (bodies[i].alive && !bodies[i].flags.is_passive && bodies[i].mass_kg > 0.0)
            insert(root, i, bodies);
    }

    // Compute masses / centre-of-mass up the tree
    compute_mass(root, bodies);
}

// ── Insert body into node ─────────────────────────────────────────────────────
void BHTree::insert(int ni, int bi, const std::vector<Body>& bodies)
{
    BHNode& node = m_nodes[ni];
    const Body& b = bodies[bi];

    if (node.is_leaf && node.body_idx == -1)
    {
        // Empty leaf — just store this body
        node.body_idx = bi;
        return;
    }

    if (node.is_leaf)
    {
        // Occupied leaf — subdivide
        int old_bi = node.body_idx;
        node.body_idx = -1;
        node.is_leaf  = false;

        double mx = (node.x0 + node.x1) * 0.5;
        double my = (node.y0 + node.y1) * 0.5;

        // Create 4 children: NW(0), NE(1), SW(2), SE(3)
        for (int q = 0; q < 4; ++q) {
            int child = new_node();
            BHNode& cn = m_nodes[child];
            bool right = (q & 1) != 0;
            bool bottom= (q & 2) != 0;
            cn.x0 = right  ? mx : node.x0;
            cn.x1 = right  ? node.x1 : mx;
            cn.y0 = bottom ? my : node.y0;
            cn.y1 = bottom ? node.y1 : my;
            cn.size = std::max(cn.x1 - cn.x0, cn.y1 - cn.y0);
            m_nodes[ni].children[q] = child;
        }

        // Re-insert the old body
        insert(ni, old_bi, bodies);
    }

    // Recurse into correct quadrant for new body
    BHNode& cur = m_nodes[ni]; // Re-fetch: vector may have reallocated
    double mx = (cur.x0 + cur.x1) * 0.5;
    double my = (cur.y0 + cur.y1) * 0.5;
    int q = 0;
    if (b.pos.x >= mx) q |= 1;
    if (b.pos.y >= my) q |= 2;
    insert(cur.children[q], bi, bodies);
}

// ── Compute centre-of-mass recursively ────────────────────────────────────────
void BHTree::compute_mass(int ni, const std::vector<Body>& bodies)
{
    BHNode& node = m_nodes[ni];

    if (node.is_leaf)
    {
        if (node.body_idx >= 0) {
            const Body& b = bodies[node.body_idx];
            node.mass = b.mass_kg;
            node.cx   = b.pos.x;
            node.cy   = b.pos.y;
        }
        return;
    }

    double total_m = 0.0, cx = 0.0, cy = 0.0;
    for (int q = 0; q < 4; ++q) {
        int child = m_nodes[ni].children[q]; // re-read: node ref may be stale
        if (child == -1) continue;
        compute_mass(child, bodies);
        double cm = m_nodes[child].mass;
        total_m += cm;
        cx += m_nodes[child].cx * cm;
        cy += m_nodes[child].cy * cm;
    }
    m_nodes[ni].mass = total_m;
    if (total_m > 0.0) {
        m_nodes[ni].cx = cx / total_m;
        m_nodes[ni].cy = cy / total_m;
    }
}

// ── Acceleration from one node on body b ─────────────────────────────────────
Vec2 BHTree::accel_recursive(int ni, const Body& b,
                              double G, double softening_m) const
{
    const BHNode& node = m_nodes[ni];
    if (node.mass == 0.0) return {};

    double dx = node.cx - b.pos.x;
    double dy = node.cy - b.pos.y;
    double dist2 = dx * dx + dy * dy + softening_m * softening_m;
    double dist  = std::sqrt(dist2);

    // Barnes-Hut opening criterion: if s/d < theta, treat as point mass
    if (node.is_leaf || (node.size / dist < BH_THETA))
    {
        // Skip self-interaction
        if (node.is_leaf && node.body_idx >= 0) {
            const Body* self_test = &b; (void)self_test;
            // We only skip if distance is ~0 (self)
            if (dist < 1.0) return {};
        }
        double mag = G * node.mass / dist2;
        return { mag * dx / dist, mag * dy / dist };
    }

    // Otherwise recurse into children
    Vec2 acc{};
    for (int q = 0; q < 4; ++q) {
        int child = node.children[q];
        if (child != -1)
            acc = acc + accel_recursive(child, b, G, softening_m);
    }
    return acc;
}

// ── Public query ──────────────────────────────────────────────────────────────
Vec2 BHTree::acceleration_on(int i, const std::vector<Body>& bodies,
                              double G, double softening_m) const
{
    if (m_nodes.empty() || !bodies[i].alive) return {};
    return accel_recursive(0, bodies[i], G, softening_m);
}
