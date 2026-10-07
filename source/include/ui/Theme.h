#pragma once
#include <imgui.h>

// Shared UI palette: ApplyModernStyle (VulkanContext.cpp) builds ImGui's style from these, and
// panels/gizmos use the same values for their accents.
namespace Theme {
    inline const ImVec4 Canvas   (0.114f, 0.133f, 0.173f, 1.f); // #1d222c — sim view clear color
    inline const ImVec4 Panel    (0.149f, 0.165f, 0.208f, 1.f); // #262a35 — window background
    inline const ImVec4 PanelAlt (0.180f, 0.200f, 0.251f, 1.f); // #2e3340 — raised / popups / tabs
    inline const ImVec4 Field    (0.224f, 0.251f, 0.314f, 1.f); // #394050 — inputs, buttons
    inline const ImVec4 FieldHov (0.267f, 0.298f, 0.376f, 1.f); // #444c60
    inline const ImVec4 Accent   (0.486f, 0.549f, 1.000f, 1.f); // #7c8cff — periwinkle
    inline const ImVec4 AccentHov(0.600f, 0.655f, 1.000f, 1.f);
    inline const ImVec4 AccentAct(0.380f, 0.439f, 0.882f, 1.f);
    inline const ImVec4 Teal     (0.310f, 0.820f, 0.710f, 1.f); // #4fd1b5 — running / ok
    inline const ImVec4 Amber    (1.000f, 0.706f, 0.329f, 1.f); // #ffb454 — one-shot events
    inline const ImVec4 Violet   (0.706f, 0.549f, 1.000f, 1.f); // #b48cff — continuous events
    inline const ImVec4 Coral    (1.000f, 0.420f, 0.478f, 1.f); // #ff6b7a — recording / destructive
    inline const ImVec4 Text     (0.890f, 0.902f, 0.937f, 1.f); // #e3e6ef
    inline const ImVec4 TextDim  (0.545f, 0.573f, 0.659f, 1.f); // #8b92a8

    inline ImVec4 WithAlpha(ImVec4 c, float a) { c.w = a; return c; }
    inline ImU32  U32(const ImVec4& c)         { return ImGui::ColorConvertFloat4ToU32(c); }
}
