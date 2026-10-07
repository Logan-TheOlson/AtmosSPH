#pragma once
#include <imgui.h>

// The docked-panel shell around the sim view: a top bar plus a dockspace whose free central area
// shows the simulation. The side panels ("###LeftPanel" / "###RightPanel", stable ids so their
// titles can change) dock left and right by default; the layout persists in imgui.ini.
namespace Workspace {
    inline constexpr const char* LeftPanelId  = "###LeftPanel";
    inline constexpr const char* RightPanelId = "###RightPanel";

    struct Frame {
        // The free central area, in ImGui points.
        ImVec2 centerMin, centerMax;
    };

    // Rebuilds the default left/right layout on the next Begin().
    void RequestLayoutReset();

    // The top bar is a window the caller fills: `if (BeginTopBar()) { ... } EndTopBar();`
    // (EndTopBar is always required). Call both before DockSpace().
    bool BeginTopBar();
    void EndTopBar();

    // Submits the dockspace (building the default layout if needed) and returns the free central
    // area. Call once per frame, before the panel windows.
    Frame DockSpace();
}
