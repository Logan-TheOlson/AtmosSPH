#pragma once
#include "engine/Scene.h"
#include <imgui.h>
#include <vector>

// Mouse editing of SceneEvents directly in the sim view: drag on empty space to create an area,
// drag an area to move it, drag its handles to resize it or to set the push direction/strength.
// Everything is drawn on ImGui's background draw list, so it shows live but can never end up in a
// recorded video (recording captures the scene pass, before ImGui).
namespace EventGizmo {
    // Screen <-> world mapping for the sim view. World units are simulation pixels with +y up;
    // screen units are ImGui points (y down).
    struct View {
        ImVec2 origin;               // screen position of world (0, 0)
        float  scale = 1.f;          // ImGui points per world px (1 / framebuffer scale)
        float  cameraOffsetX = 0.f;  // display-only x shift (periodic "follow flow" camera)
        float  wrapHalfWidth = 0.f;  // > 0: display x wraps into (-hw, hw] like the particles do
        ImVec2 clipMin, clipMax;     // the sim view's screen rect: drawing and new-area drags stay in it
    };

    struct Context {
        View                     view;
        std::vector<SceneEvent>* events   = nullptr;
        int*                     selected = nullptr;  // index into *events, -1 = none
        const SceneEvent*        defaults = nullptr;  // shape/mode/accel/duration used for newly dragged areas
        const std::vector<char>* fired    = nullptr;  // live: per-event "Once already fired" flags
        double                   simTime  = 0.0;
        bool                     live     = false;    // live run (new events start now) vs. pre-run preview
        bool                     interactive = true;  // false: draw only (e.g. while batch rendering)
        // Reuse for plain areas (SceneColorRegion edited through SceneEvent proxies): no push
        // arrow/handle, and an optional per-area tint replacing the mode colors.
        bool                     showArrows = true;
        const std::vector<ImVec4>* colors   = nullptr;
        const char*              label      = "Persistent color area";  // tooltip name when showArrows is false
        float                    fillScale  = 1.f;      // multiplies the area fill opacity (< 1 lets what's underneath show through)
    };

    struct Result {
        bool created = false;   // an event was appended (and selected)
        int  deleted = -1;      // index that was removed this frame, or -1
        bool changed = false;   // any existing event's geometry/acceleration was edited
    };

    // Draws every event and handles the mouse/keyboard. Call once per frame after the panels have
    // been submitted (so io.WantCaptureMouse already reflects them).
    Result Update(const Context& ctx);

    // World px of arrow length per unit of acceleration/kick (fixed so arrow length reads as strength).
    inline constexpr float ArrowPxPerUnit = 0.25f;
}
