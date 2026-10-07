#pragma once
#include "ui/Theme.h"
#include <imgui.h>
#include <algorithm>

// Small reusable widgets shared by the setup panel, the live panel and the top bar.
namespace Widgets {
    // A rounded, tinted pill with text, drawn inline (advances the layout cursor like a button).
    inline void Chip(const char* text, const ImVec4& color) {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 pad(10.f, 3.f);
        const ImVec2 ts  = ImGui::CalcTextSize(text);
        const ImVec2 size(ts.x + pad.x * 2.f, ts.y + pad.y * 2.f);
        // Occupies a full frame-height cell so it lines up with neighboring buttons.
        const float  cell = std::max(size.y, ImGui::GetFrameHeight());
        ImVec2 pos = ImGui::GetCursorScreenPos();
        pos.y += (cell - size.y) * 0.5f;
        dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), Theme::U32(Theme::WithAlpha(color, 0.22f)), size.y * 0.5f);
        dl->AddText(ImVec2(pos.x + pad.x, pos.y + pad.y), Theme::U32(color), text);
        ImGui::Dummy(ImVec2(size.x, cell));
    }

    // Small dim caption above a group of fields.
    inline void Caption(const char* text) {
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, Theme::TextDim);
        ImGui::TextUnformatted(text);
        ImGui::PopStyleColor();
    }

    // Filled accent button (primary action).
    inline bool PrimaryButton(const char* label, const ImVec2& size) {
        ImGui::PushStyleColor(ImGuiCol_Button,        Theme::Accent);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Theme::AccentHov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  Theme::AccentAct);
        ImGui::PushStyleColor(ImGuiCol_Text,          Theme::Panel);
        const bool pressed = ImGui::Button(label, size);
        ImGui::PopStyleColor(4);
        return pressed;
    }

    // Tinted button for destructive actions.
    inline bool DangerButton(const char* label, const ImVec2& size = ImVec2(0.f, 0.f)) {
        ImGui::PushStyleColor(ImGuiCol_Button,        Theme::WithAlpha(Theme::Coral, 0.30f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, Theme::WithAlpha(Theme::Coral, 0.50f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  Theme::WithAlpha(Theme::Coral, 0.70f));
        const bool pressed = ImGui::Button(label, size);
        ImGui::PopStyleColor(3);
        return pressed;
    }
}
