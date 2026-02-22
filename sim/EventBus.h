#pragma once
// =============================================================================
//  sim/EventBus.h  — Lightweight typed pub/sub for simulation events
//  UI layers subscribe; physics core only emits.
// =============================================================================

#include "../domain/Body.h"
#include <functional>
#include <vector>
#include <string>

// ── Event types ───────────────────────────────────────────────────────────────
struct EvBodyAdded    { Body body; };
struct EvBodyRemoved  { std::string id; std::string name; };
struct EvCollision    { Body survivor; Body absorbed; };
struct EvPresetLoaded { std::string preset_name; size_t body_count; };
struct EvBhAbsorption { std::string absorbed_id; std::string bh_id; }; // Phase 25D

// ── Simple typed event channel ────────────────────────────────────────────────
template<typename T>
class EventChannel
{
public:
    using Handler = std::function<void(const T&)>;

    void subscribe(Handler h) { m_handlers.push_back(std::move(h)); }

    void emit(const T& evt) const
    {
        for (const auto& h : m_handlers) h(evt);
    }

    void clear() { m_handlers.clear(); }

private:
    std::vector<Handler> m_handlers;
};

// ── Aggregate bus ─────────────────────────────────────────────────────────────
struct EventBus
{
    EventChannel<EvBodyAdded>    on_body_added;
    EventChannel<EvBodyRemoved>  on_body_removed;
    EventChannel<EvCollision>    on_collision;
    EventChannel<EvPresetLoaded> on_preset_loaded;
    EventChannel<EvBhAbsorption> on_bh_absorption;  // Phase 25D

    void clear_all()
    {
        on_body_added.clear();
        on_body_removed.clear();
        on_collision.clear();
        on_preset_loaded.clear();
        on_bh_absorption.clear();
    }
};
